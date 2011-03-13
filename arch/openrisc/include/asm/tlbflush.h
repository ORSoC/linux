#ifndef __ASM_OPENRISC_TLBFLUSH_H
#define __ASM_OPENRISC_TLBFLUSH_H

#include <linux/mm.h>
#include <asm/processor.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>
#include <asm/current.h>
/*#include <asm/spr_defs.h>*/
#include <linux/sched.h>

/*
 * TLB flushing (implemented in arch/or32/mm/tlb.c):
 *
 *  - flush_tlb() flushes the current mm struct TLBs
 *  - flush_tlb_all() flushes all processes TLBs
 *  - flush_tlb_mm(mm) flushes the specified mm context TLB's
 *  - flush_tlb_page(vma, vmaddr) flushes one page
 *  - flush_tlb_range(mm, start, end) flushes a range of pages
 *
 */

void flush_tlb_all(void);
void flush_tlb_mm(struct mm_struct* mm);
void flush_tlb_page(struct vm_area_struct* vma, unsigned long addr);
void flush_tlb_range(struct vm_area_struct* vma,
		     unsigned long start,
		     unsigned long end);


#if 0
static inline void flush_tlb_pgtables(struct mm_struct *mm,
                                      unsigned long start, unsigned long end)
{
        /* OR32 does not keep any page table caches in TLB */
}

#endif

static inline void flush_tlb(void) 
{
	flush_tlb_mm(current->mm);
}

static inline void flush_tlb_kernel_range(unsigned long start,
					  unsigned long end)
{
	flush_tlb_range(NULL, start, end);
}




#endif /* __ASM_OPENRISC_TLBFLUSH_H */
