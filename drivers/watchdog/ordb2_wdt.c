/*
 *	ORDB2 Watchdog 1.00: Watchdog driver for ORBD2 board
 *
 *	(c) Copyright 2013 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/watchdog.h>

extern u32 *ordb2_reset;

/*
 *	Watchdog operations
 */

static int watchdog_ping(struct watchdog_device *dev)
{
	*ordb2_reset = 1 << 2;
	return 0;
}

static int watchdog_stop(struct watchdog_device *dev)
{
	pr_warn("ordb2 watchdog cannot be stopped\n");
	return -EBUSY;
}

/*
 *	Watchdog registration
 */

static struct watchdog_info wdt_info = {
	.options = WDIOC_GETBOOTSTATUS | WDIOC_KEEPALIVE,
	.identity = "ordb2",
};
	
static struct watchdog_ops wdt_ops = {
	.owner = THIS_MODULE,
	.start = &watchdog_ping,
	.stop = &watchdog_stop,
};

static struct watchdog_device wdt_device = {
	.info = &wdt_info,
	.ops = &wdt_ops,
	.bootstatus = 1,
	.status = 1 << WDOG_ACTIVE | 1 << WDOG_NO_WAY_OUT,
};

static int __init watchdog_init(void)
{
	int ret;
	if (!ordb2_reset) {
		pr_info("ordb2 reset register not found\n");
		return -ENODEV;
	}
	ret = watchdog_register_device(&wdt_device);
	if (ret) {
		pr_err("failed to register ordb2 watchdog driver\n");
		return ret;
	}
	return 0;
}

static void __exit watchdog_exit(void)
{
	watchdog_unregister_device(&wdt_device);
}

module_init(watchdog_init);
module_exit(watchdog_exit);

MODULE_AUTHOR("Henrik Nordstrom <henrik@henriknordstrom.net>");
MODULE_DESCRIPTION("ORDB2 WatchDog Timer Driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");

