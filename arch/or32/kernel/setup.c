/*
 *  linux/arch/or32/kernel/setup.c
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
 */
 
/*
 * This file handles the architecture-dependent parts of initialization
 */
 
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/user.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <linux/seq_file.h>
#include <linux/serial.h>
#include <linux/initrd.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
 
#include <asm/board.h>
#include <asm/segment.h>
#include <asm/system.h>
#include <asm/smp.h>
#include <asm/pgtable.h>
#include <asm/types.h>
#include <asm/setup.h>
#include <asm/io.h>
#include <asm/cpuinfo.h>

/*
 * Debugging stuff
 */

int __phx_mmu__ = 0;
int __phx_warn__ = 0;
int __phx_debug__ = 0;
int __phx_signal__ = 0;
 
/*
 * Setup options
 */
 
extern int root_mountflags;
extern char _stext, _etext, _edata, _end;
extern int __init setup_early_serial8250_console(char *cmdline);
#ifdef CONFIG_BLK_DEV_INITRD
//extern unsigned long initrd_start, initrd_end;
extern char __initrd_start, __initrd_end;
extern char __initramfs_start, __initramfs_end;
#endif

extern u32 _fdt_start;
 
unsigned long or32_mem_size;
unsigned long fb_mem_start;

#ifdef CONFIG_CMDLINE
char __initdata cmd_line[COMMAND_LINE_SIZE] = CONFIG_CMDLINE;
#else
char __initdata cmd_line[COMMAND_LINE_SIZE] = "console=uart,mmio,0x90000000,115200";
//char __initdata cmd_line[COMMAND_LINE_SIZE] = "console=uart,mmio,0x90000000,115200 root=/dev/nfs rw nfsroot=172.30.0.1:/home/jonas/local/opencores/linux-2.6/arch/or32/support/rootfs,rw,nolock ip=172.30.0.2::::::";
#endif

extern const unsigned long text_start, edata; /* set by the linker script */ 
       
//char cmd_line[COMMAND_LINE_SIZE];

static unsigned long __init setup_memory(void)
{
	unsigned long bootmap_size, start_pfn, max_low_pfn;


#ifndef CONFIG_FB_OC_SHMEM_SIZE
#define CONFIG_FB_OC_SHMEM_SIZE 0
#endif /* CONFIG_FB_OC_SHMEM_SIZE */


	/* min_low_pfn points to the start of DRAM, start_pfn points
	 * to the first DRAM pages after the kernel, and max_low_pfn
	 * to the end of DRAM. Partial pages are not useful, so round it 
	 * down.
	 */ 
	start_pfn   = PFN_UP(__pa(&_end));
	max_low_pfn = PFN_DOWN(CONFIG_OR32_MEMORY_SIZE
			       -CONFIG_FB_OC_SHMEM_SIZE);
//	min_low_pfn = PAGE_OFFSET >> PAGE_SHIFT;

#undef CONFIG_FB_OC_SHMEM_SIZE

	/*
	 * set the beginning of frame buffer
	 */
	fb_mem_start = PFN_PHYS(max_low_pfn);

	/* 
	 * initialize the boot-time allocator (with low memory only)
	 */ 
	bootmap_size = init_bootmem(start_pfn, max_low_pfn);
	free_bootmem(PFN_PHYS(start_pfn), PFN_PHYS(max_low_pfn - start_pfn));
	reserve_bootmem(PFN_PHYS(start_pfn), bootmap_size, BOOTMEM_DEFAULT);//rgd

	return(max_low_pfn);
}


struct cpuinfo cpuinfo;

static void print_cpuinfo(void) {
	unsigned long upr = mfspr(SPR_UPR);
	unsigned long vr = mfspr(SPR_VR);
	unsigned int version;
	unsigned int revision;       

	version = (vr & SPR_VR_VER) >> 24;
	revision = (vr & SPR_VR_REV);

	printk(KERN_INFO "CPU: OpenRISC-%x (revision %x) @%d MHz\n", 
		version, revision, cpuinfo.clock_frequency / 1000000);

	if (!(upr & SPR_UPR_UP)) {
		printk(KERN_INFO "-- no UPR register... unable to detect configuration\n");
		return;
	}

	if (upr & SPR_UPR_DCP) 
		printk(KERN_INFO "-- dcache: %4d bytes total, %2d bytes/line, %d way(s)\n",
			cpuinfo.dcache_size, cpuinfo.dcache_block_size, 1);
	else 
		printk(KERN_INFO "-- dcache disabled\n");
	if (upr & SPR_UPR_ICP) 
		printk(KERN_INFO "-- icache: %4d bytes total, %2d bytes/line, %d way(s)\n",
			cpuinfo.icache_size, cpuinfo.icache_block_size, 1);
	else 
		printk(KERN_INFO "-- icache disabled\n");
	
	if (upr & SPR_UPR_DMP)
		printk(KERN_INFO "-- dmmu: (assumed) %4d entries, %d way(s)\n",
			CONFIG_OR32_DTLB_ENTRIES, 1);
	if (upr & SPR_UPR_IMP)
		printk(KERN_INFO "-- immu: (assumed) %4d entries, %d way(s)\n",
			CONFIG_OR32_ITLB_ENTRIES, 1);

	printk(KERN_INFO "-- additional features:\n");
	if (upr & SPR_UPR_DUP)
		printk(KERN_INFO "-- debug unit\n");
	if (upr & SPR_UPR_PCUP)
		printk(KERN_INFO "-- performance counters\n");
	if (upr & SPR_UPR_PMP)
		printk(KERN_INFO "-- power management\n");
	if (upr & SPR_UPR_PICP)
		printk(KERN_INFO "-- PIC\n");
	if (upr & SPR_UPR_TTP)
		printk(KERN_INFO "-- timer\n");
	if (upr & SPR_UPR_CUP)
		printk(KERN_INFO "-- custom unit(s)\n");
}

static inline unsigned int fcpu(struct device_node *cpu, char *n)
{
        int *val;
        return (val = (int *) of_get_property(cpu, n, NULL)) ? *val : 0;
}

extern void __ic_enable(u32 icache_size, u32 icache_block_size);
extern void __dc_enable(u32 dcache_size, u32 dcache_block_size);

void __init setup_cpuinfo(void)
{
        struct device_node *cpu = NULL;
	unsigned long iccfgr,dccfgr;
	unsigned long cache_set_size, cache_ways;;

        cpu = (struct device_node *) of_find_node_by_type(NULL, "cpu");
        if (!cpu) {
                printk(KERN_ERR "Device tree missing CPU node\n");
		return;
	}

	iccfgr = mfspr(SPR_ICCFGR);
	cache_ways = 1 << (iccfgr & SPR_ICCFGR_NCW);
	cache_set_size = 1 << ((iccfgr & SPR_ICCFGR_NCS) >> 3);
	cpuinfo.icache_block_size = 16 << ((iccfgr & SPR_ICCFGR_CBS) >> 7);
	cpuinfo.icache_size = cache_set_size * cache_ways * cpuinfo.icache_block_size;

	dccfgr = mfspr(SPR_DCCFGR);
	cache_ways = 1 << (dccfgr & SPR_DCCFGR_NCW);
	cache_set_size = 1 << ((dccfgr & SPR_DCCFGR_NCS) >> 3);
	cpuinfo.dcache_block_size = 16 << ((dccfgr & SPR_DCCFGR_CBS) >> 7);
	cpuinfo.dcache_size = cache_set_size * cache_ways * cpuinfo.dcache_block_size;



	cpuinfo.clock_frequency =  fcpu(cpu, "clock-frequency");
/*	cpuinfo.icache_size = fcpu(cpu, "i-cache-size");
	cpuinfo.icache_block_size = fcpu(cpu, "i-cache-block-size");
	cpuinfo.dcache_size = fcpu(cpu, "d-cache-size");
	cpuinfo.dcache_block_size = fcpu(cpu, "d-cache-block-size");
*/
	of_node_put(cpu);

	print_cpuinfo();

//	printk("IC ENABLE........................\n");
//	__ic_enable(cpuinfo.icache_size, cpuinfo.icache_block_size);
//	__dc_enable(cpuinfo.dcache_size, cpuinfo.dcache_block_size);
}

void __init or32_early_setup(/*unsigned long fdt*/) {

	early_init_devtree((void *) &_fdt_start);


/*	if (fdt)
		printk("FDT at 0x%08x\n", fdt);
	else*/
		printk("Compiled-in FDT at 0x%08x\n",
		       (unsigned int)&_fdt_start);


}



static inline unsigned long extract_value_bits(unsigned long reg, 
					       short bit_nr, short width)
{
	return((reg >> bit_nr) & (0 << width));
}

static inline unsigned long extract_value(unsigned long reg, 
					  unsigned long mask)
{
	while (!(mask & 0x1)) {
		reg  = reg  >> 1;
		mask = mask >> 1;
	}
	return(mask & reg);
}

void __init detect_timer(void) 
{
	unsigned long cfg;

	cfg = mfspr(SPR_TTMR);
	printk("period 0x%x\n", SPR_TTMR & SPR_TTMR_PERIOD);
}

void __init detect_unit_config(unsigned long upr, unsigned long mask,
			       char *text, void (*func)(void))
{
        if (text != NULL)
		printk("%s", text);

	if ( upr & mask ) {
		if (func != NULL)
			func();
		else
			printk("present\n");
	}
	else
		printk("not present\n");
}

void __init setup_arch(char **cmdline_p)
{
	unsigned long max_low_pfn;

	unflatten_device_tree();

	setup_cpuinfo();

	/* process 1's initial memory region is the kernel code/data */
	init_mm.start_code = (unsigned long) &_stext;
	init_mm.end_code =   (unsigned long) &_etext;
	init_mm.end_data =   (unsigned long) &_edata;
	init_mm.brk =        (unsigned long) &_end;

#ifdef CONFIG_BLK_DEV_INITRD
	initrd_start = (unsigned long)&__initrd_start;
	initrd_end = (unsigned long)&__initrd_end;
	initrd_below_start_ok = 1;
#endif

        /* setup bootmem allocator */
	max_low_pfn = setup_memory();
		
	/* paging_init() sets up the MMU and marks all pages as reserved */
	paging_init();
	
#ifdef CONFIG_SERIAL_8250_CONSOLE
	//	early_serial_console_init(command_line); RGD
	//setup_early_serial8250_console(cmd_line);
#endif

#if defined(CONFIG_VT) && defined(CONFIG_DUMMY_CONSOLE)
	if(!conswitchp)
        	conswitchp = &dummy_con;
#endif

	*cmdline_p = cmd_line;
	
	/* Save command line copy for /proc/cmdline RGD removed 2.6.21*/
	//memcpy(saved_command_line, command_line, COMMAND_LINE_SIZE);
	//saved_command_line[COMMAND_LINE_SIZE-1] = '\0';

	/* fire up 8051 */
//	printk("Starting 8051...\n");
//	oc8051_init();

#ifndef CONFIG_OR32_ANONYMOUS
	printk("Linux/or32 port 2003-2005 OpenCores <phoenix@bsemi.com>\n");
#endif /* CONFIG_OR32_ANONYMOUS */
}

static int show_cpuinfo(struct seq_file *m, void *v)
{
	unsigned long vr;
	int version, revision;

	vr = mfspr(SPR_VR);
	version = (vr & SPR_VR_VER) >> 24;
	revision = vr & SPR_VR_REV;

	return seq_printf(m,
		"cpu\t\t: OpenRISC-%d\n"
		"revision\t: %lu\n"
		"dcache size\t: %d kB\n"
		"dcache block size\t: %d bytes\n"
		"icache size\t: %d kB\n"
		"icache block size\t: %d bytes\n"
		"immu\t\t: %s\n"
		"dmmu\t\t: %s\n"
		"bogomips\t: %lu.%02lu\n",

		version,
		revision,
		cpuinfo.dcache_size,
		cpuinfo.dcache_block_size,
		cpuinfo.icache_size,
		cpuinfo.icache_block_size,
		"(assumed) 64 entries, 1 way",
		"(assumed) 64 entries, 1 way",
		(loops_per_jiffy * HZ) / 500000,
		((loops_per_jiffy * HZ) / 5000) % 100);
}

static void *c_start(struct seq_file *m, loff_t *pos)
{
	/* We only have one CPU... */
	return *pos < 1 ? (void *)1 : NULL;
}
 
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
}

struct seq_operations cpuinfo_op = {
	start:  c_start,
	next:   c_next,
	stop:   c_stop,
	show:   show_cpuinfo,
};

/*void arch_gettod(int *year, int *mon, int *day, int *hour,
		                  int *min, int *sec)
{
   
	if (mach_gettod)
		mach_gettod(year, mon, day, hour, min, sec);
	else
		*year = *mon = *day = *hour = *min = *sec = 0;
}
*/
/*RGD this awful hack is because our compiler does
 *support the "weak" attribute correctly at this time
 *once we do (support weak) this should be removed!!
 */
void __start_notes(void){}
void __stop_notes(void){}
