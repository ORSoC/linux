/*
 *  linux/arch/or32/kernel/signal.c
 *
 *  or32 version
 *    author(s): Matjaz Breskvar (phoenix@bsemi.com)
 *
 *  For more information about OpenRISC processors, licensing and
 *  design services you may contact Beyond Semiconductor at
 *  sales@bsemi.com or visit website http://www.bsemi.com.
 *
 *  derived from cris, i386, m68k, ppc, sh ports.
 *
 *  changes:
 *  18. 11. 2003: Matjaz Breskvar (phoenix@bsemi.com)
 *    initial port to or32 architecture
 *
 *  Based on linux/arch/cris/kernel/signal.c
 *    Copyright (C) 2000, 2001, 2002 Axis Communications AB
 *    Authors:  Bjorn Wesen (bjornw@axis.com)
 *
 */

#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/ptrace.h>
#include <linux/unistd.h>
#include <linux/stddef.h>
#include <linux/tracehook.h>

#include <asm/processor.h>
#include <asm/ucontext.h>
#include <asm/uaccess.h>
#include <asm/or32-hf.h>

#define DEBUG_SIG 0

#define _BLOCKABLE (~(sigmask(SIGKILL) | sigmask(SIGSTOP)))

asmlinkage long
_sys_sigaltstack(const stack_t *uss, stack_t *uoss, struct pt_regs *regs)
{
	return do_sigaltstack(uss, uoss, regs->sp);
}

struct rt_sigframe {
	struct siginfo *pinfo;
	void *puc;
	struct siginfo info;
	struct ucontext uc;
	unsigned char retcode[16];  /* trampoline code */
};


static int restore_sigcontext(struct pt_regs *regs, struct sigcontext *sc)
{
	unsigned int err = 0;
	unsigned long old_usp;

	phx_signal("regs %p, sc %p",
		   regs, sc);

	/* Alwys make any pending restarted system call return -EINTR */
	current_thread_info()->restart_block.fn = do_no_restart_syscall;

	/* restore the regs from &sc->regs (same as sc, since regs is first)
	 * (sc is already checked for VERIFY_READ since the sigframe was
	 *  checked in sys_sigreturn previously)
	 */

	if (__copy_from_user(regs, sc, sizeof(struct pt_regs)))
                goto badframe;

	/* make sure the U-flag is set so user-mode cannot fool us */

	regs->sr &= ~SPR_SR_SM;
//      __PHX__ FIXME	
//	regs->dccr |= 1 << 8;

	/* restore the old USP as it was before we stacked the sc etc.
	 * (we cannot just pop the sigcontext since we aligned the sp and
	 *  stuff after pushing it)
	 */

	err |= __get_user(old_usp, &sc->usp);
	phx_signal("old_usp 0x%lx", old_usp);

//      __PHX__ REALLY ???
//	wrusp(old_usp);
	regs->sp = old_usp;

	/* TODO: the other ports use regs->orig_XX to disable syscall checks
	 * after this completes, but we don't use that mechanism. maybe we can
	 * use it now ? 
	 */

	return err;

badframe:
	return 1;
}

asmlinkage long
_sys_rt_sigreturn(struct pt_regs *regs)
{
	struct rt_sigframe *frame = (struct rt_sigframe __user *)regs->sp;
	sigset_t set;
	stack_t st;

        /*
         * Since we stacked the signal on a dword boundary,
         * then frame should be dword aligned here.  If it's
         * not, then the user is trying to mess with us.
         */
        if (((long)frame) & 3)
                goto badframe;

	if (verify_area(VERIFY_READ, frame, sizeof(*frame)))
		goto badframe;
	if (__copy_from_user(&set, &frame->uc.uc_sigmask, sizeof(set)))
		goto badframe;

	sigdelsetmask(&set, ~_BLOCKABLE);
	spin_lock_irq(&current->sighand->siglock);
	current->blocked = set;
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);
	
	if (restore_sigcontext(regs, &frame->uc.uc_mcontext))
		goto badframe;

	if (__copy_from_user(&st, &frame->uc.uc_stack, sizeof(st)))
		goto badframe;
	/* It is more difficult to avoid calling this function than to
	   call it and ignore errors.  */
	do_sigaltstack(&st, NULL, regs->sp);

//	return regs->gprs[1];
	return regs->gpr[11];

badframe:
	force_sig(SIGSEGV, current);
	return 0;
}	

/*
 * Set up a signal frame.
 */

static int setup_sigcontext(struct sigcontext *sc, struct pt_regs *regs,
			    unsigned long mask)
{
	int err = 0;
	unsigned long usp = regs->sp;

	phx_signal("sc %p, regs %p, mask 0x%lx",
		   sc, regs, mask);

	/* copy the regs. they are first in sc so we can use sc directly */

	err |= __copy_to_user(sc, regs, sizeof(struct pt_regs));

        /* Set the frametype to CRIS_FRAME_NORMAL for the execution of
           the signal handler. The frametype will be restored to its previous
           value in restore_sigcontext. */
	//        regs->frametype = CRIS_FRAME_NORMAL;

	/* then some other stuff */

	err |= __put_user(mask, &sc->oldmask);

	err |= __put_user(usp, &sc->usp);

	return err;
}


static inline unsigned long
align_sigframe(unsigned long sp)
{
	return (sp & ~3UL);
}

/*
 * Work out where the signal frame should go.  It's either on the user stack
 * or the alternate stack.
 */

static inline void __user *
get_sigframe(struct k_sigaction *ka, struct pt_regs * regs, size_t frame_size)
{
	unsigned long sp = regs->sp;
	int onsigstack = on_sig_stack(sp);

	/* redzone */
	sp -= STACK_FRAME_OVERHEAD;

	/* This is the X/Open sanctioned signal stack switching.  */
	if ((ka->sa.sa_flags & SA_ONSTACK) && !onsigstack) {
		if (current->sas_ss_size)
			sp = current->sas_ss_sp + current->sas_ss_size;
	}

	sp = align_sigframe(sp - frame_size);

	/*
	 * If we are on the alternate signal stack and would overflow it, don't.
	 * Return an always-bogus address instead so we will die with SIGSEGV.
	 */
	if (onsigstack && !likely(on_sig_stack(sp)))
		return (void __user *)-1L;

	return (void __user *)sp;
}

/* grab and setup a signal frame.
 * 
 * basically we stack a lot of state info, and arrange for the
 * user-mode program to return to the kernel using either a
 * trampoline which performs the syscall sigreturn, or a provided
 * user-mode trampoline.
 */
static void setup_rt_frame(int sig, struct k_sigaction *ka, siginfo_t *info,
			   sigset_t *set, struct pt_regs * regs)
{
	struct rt_sigframe *frame;
	unsigned long return_ip;
	int err = 0;

	phx_signal("sig %d, ka %p, info %p, set %p, regs %p",
		   sig, ka, info, set, regs);

	frame = get_sigframe(ka, regs, sizeof(*frame));

	if (!access_ok(VERIFY_WRITE, frame, sizeof(*frame)))
		goto give_sigsegv;

	err |= __put_user(&frame->info, &frame->pinfo);
	err |= __put_user(&frame->uc, &frame->puc);

	if (ka->sa.sa_flags & SA_SIGINFO)
		err |= copy_siginfo_to_user(&frame->info, info);
	if (err)
		goto give_sigsegv;

	/* Clear all the bits of the ucontext we don't use.  */
        err |= __clear_user(&frame->uc, offsetof(struct ucontext, uc_mcontext));
	/* Added by jonas */
        err |= __put_user(0, &frame->uc.uc_flags);
        err |= __put_user(NULL, &frame->uc.uc_link);
        err |= __put_user((void *)current->sas_ss_sp,
                        &frame->uc.uc_stack.ss_sp);
        err |= __put_user(sas_ss_flags(regs->sp),
                        &frame->uc.uc_stack.ss_flags);
        err |= __put_user(current->sas_ss_size, &frame->uc.uc_stack.ss_size);
	/* End added by jonas */
	err |= setup_sigcontext(&frame->uc.uc_mcontext, regs, set->sig[0]);

	err |= __copy_to_user(&frame->uc.uc_sigmask, set, sizeof(*set));

	if (err)
		goto give_sigsegv;

	/* trampoline - the desired return ip is the retcode itself */
	return_ip = (unsigned long)&frame->retcode;
	phx_signal("ktrampoline: return_ip 0x%lx", return_ip);
	/* This is l.ori r11,r0,__NR_sigreturn, l.sys 1 */
	err |= __put_user(0xa960        , (short *)(frame->retcode+0));
	err |= __put_user(__NR_rt_sigreturn, (short *)(frame->retcode+2));
	err |= __put_user(0x20000001, (unsigned long *)(frame->retcode+4));
	err |= __put_user(0x15000000, (unsigned long *)(frame->retcode+8));

	if (err)
		goto give_sigsegv;

	/* TODO what is the current->exec_domain stuff and invmap ? */

	/* Set up registers for signal handler */
	regs->pc = (unsigned long) ka->sa.sa_handler; /* what we enter NOW   */
	regs->gpr[9] = (unsigned long) return_ip;    /* what we enter LATER */
	regs->gpr[3] = (unsigned long) sig;          /* arg 1: signo */
        regs->gpr[4] = (unsigned long) &frame->info; /* arg 2: (siginfo_t*) */
        regs->gpr[5] = (unsigned long) &frame->uc;   /* arg 3: ucontext */

	/* actually move the usp to reflect the stacked frame */

	//	wrusp((unsigned long)frame);
	regs->sp = (unsigned long)frame;

	return;

give_sigsegv:
	if (sig == SIGSEGV)
		ka->sa.sa_handler = SIG_DFL;
	force_sig(SIGSEGV, current);
}

/*
 * OK, we're invoking a handler
 */	


static inline void
handle_signal(unsigned long sig,
	      siginfo_t *info, struct k_sigaction *ka,
	      sigset_t *oldset, struct pt_regs * regs)
{
	setup_rt_frame(sig, ka, info, oldset, regs);

	if (ka->sa.sa_flags & SA_ONESHOT)
		ka->sa.sa_handler = SIG_DFL;

	spin_lock_irq(&current->sighand->siglock);
	sigorsets(&current->blocked, &current->blocked, &ka->sa.sa_mask);
	if (!(ka->sa.sa_flags & SA_NODEFER))
		sigaddset(&current->blocked, sig);
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);
}

/*
 * Note that 'init' is a special process: it doesn't get signals it doesn't
 * want to handle. Thus you cannot kill init even with a SIGKILL even by
 * mistake.
 *
 * Also note that the regs structure given here as an argument, is the latest
 * pushed pt_regs. It may or may not be the same as the first pushed registers
 * when the initial usermode->kernelmode transition took place. Therefore
 * we can use user_mode(regs) to see if we came directly from kernel or user
 * mode below.
 */

void do_signal(struct pt_regs *regs)
{
	siginfo_t info;
	int signr;
	struct k_sigaction ka;
#if 0
	check_stack(NULL, __FILE__, __FUNCTION__, __LINE__);
#endif

	/*
	 * We want the common case to go fast, which
	 * is why we may in certain cases get here from
	 * kernel mode. Just return without doing anything
	 * if so.
	 */
	if (!user_mode(regs))
		return;

	signr = get_signal_to_deliver(&info, &ka, regs, NULL);


	/* If we are coming out of a syscall then we need
	 * to check if the syscall was interrupted and wants to be
	 * restarted after handling the signal.  If so, the original
	 * syscall number is put back into r11 and the PC rewound to
	 * point at the l.sys instruction that resulted in the 
	 * original syscall.  Syscall results other than the four
	 * below mean that the syscall executed to completion and no
	 * restart is necessary.
	 */
	if (regs->syscallno) {
		int restart = 0;

		switch(regs->gpr[11]) {
		case -ERESTART_RESTARTBLOCK:
		case -ERESTARTNOHAND:
			/* Restart if there is no signal handler */
			restart = (signr <= 0);
			break;
		case -ERESTARTSYS:
			/* Restart if there no signal handler or
			 * SA_RESTART flag is set */
			restart = (signr <= 0 || (ka.sa.sa_flags & SA_RESTART));
			break;
		case -ERESTARTNOINTR:
			/* Always restart */
			restart = 1;
			break;
		}

		if (restart) {
			if (regs->gpr[11] == -ERESTART_RESTARTBLOCK)
				regs->gpr[11] = __NR_restart_syscall;
			else
				regs->gpr[11] = regs->orig_gpr11;
			regs->pc -= 4;
		} else {
			regs->gpr[11] = -EINTR;
		}
	}


	if (signr <= 0) {
		/* no signal to deliver so we just put the saved sigmask
		 * back */
		if (test_thread_flag(TIF_RESTORE_SIGMASK)) {
			clear_thread_flag(TIF_RESTORE_SIGMASK);
			sigprocmask(SIG_SETMASK, &current->saved_sigmask, NULL);
		}
	} else { /* signr > 0 */
		sigset_t *oldset;

		if (current_thread_info()->flags & _TIF_RESTORE_SIGMASK)
			oldset = &current->saved_sigmask; 
		else
			oldset = &current->blocked;

		/* Whee!  Actually deliver the signal.  */
		handle_signal(signr, &info, &ka, oldset, regs);
		/* a signal was successfully delivered; the saved
		 * sigmask will have been stored in the signal frame,
		 * and will be restored by sigreturn, so we can simply
		 * clear the TIF_RESTORE_SIGMASK flag */
		if (test_thread_flag(TIF_RESTORE_SIGMASK))
			clear_thread_flag(TIF_RESTORE_SIGMASK);

		tracehook_signal_handler(signr, &info, &ka, regs,
					 test_thread_flag(TIF_SINGLESTEP));
	}

	return;
}

asmlinkage void
do_notify_resume(struct pt_regs *regs)
{
	if (current_thread_info()->flags & _TIF_SIGPENDING)
		do_signal(regs);

	if (current_thread_info()->flags & _TIF_NOTIFY_RESUME) {
		clear_thread_flag(TIF_NOTIFY_RESUME);
		tracehook_notify_resume(regs);
		if (current->replacement_session_keyring)
			key_replace_session_keyring();
	}
}

