/*
 * OpenRISC reset.c
 *
 * Copyright (C) 2013 Henrik Nordstrom <henrik.nordstrom@orsoc.se>
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 * This implements reset & restart functionality
 */

#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/pm.h>

u32 *ordb2_reset = NULL;

void machine_restart(char *cmd)
{
	local_irq_disable();
	if (ordb2_reset) {
		printk(KERN_INFO "Rebooting..\n");
		*ordb2_reset = 1;
	}
	wmb();
	printk(KERN_INFO "*** MACHINE RESTART ***\n");
	__asm__("l.nop 1");
	while(1);
}

/*
 * Similar to machine_power_off, but don't shut off power.  Add code
 * here to freeze the system for e.g. post-mortem debug purpose when
 * possible.  This halt has nothing to do with the idle halt.
 */
void machine_halt(void)
{
	local_irq_disable();
	printk(KERN_INFO "*** MACHINE HALT ***\n");
	__asm__("l.nop 1");
	while(1);
}

/* If or when software power-off is implemented, add code here.  */
void machine_power_off(void)
{
	local_irq_disable();
	printk(KERN_INFO "*** MACHINE POWER OFF ***\n");
	__asm__("l.nop 1");
	if (ordb2_reset) {
		printk(KERN_INFO "Resetting the FPGA..\n");
		*ordb2_reset = 2;
	}
	while(1);
}

void (*pm_power_off) (void) = machine_power_off;
