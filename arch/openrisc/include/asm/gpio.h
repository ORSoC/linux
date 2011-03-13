/*
 * Generic GPIO API implementation for OpenRISC
 *
 * Copyright (c) 2010 Jonas Bonn
 *
 * Based on arch/microblaze/include/asm/gpio.h
 * Copyright (c) 2007-2008  MontaVista Software, Inc.
 *
 * Author: Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_OPENRISC_GPIO_H
#define __ASM_OPENRISC_GPIO_H

#include <linux/errno.h>
#include <asm-generic/gpio.h>

#ifdef CONFIG_GPIOLIB

/*
 * OpenRISC (or1k) does not have on-chip GPIO's so there is not really
 * any standardized implementation that makes sense here.  If passing
 * through gpiolib becomes a bottleneck then it may make sense, on a 
 * case-by-case basis, to implement these inlined/rapid versions.
 *
 * Just call gpiolib.
 */
static inline int gpio_get_value(unsigned int gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned int gpio, int value)
{
	__gpio_set_value(gpio, value);
}

static inline int gpio_cansleep(unsigned int gpio)
{
	return __gpio_cansleep(gpio);
}

/*
 * Not implemented, yet.
 */
static inline int gpio_to_irq(unsigned int gpio)
{
	return -ENOSYS;
}

static inline int irq_to_gpio(unsigned int irq)
{
	return -EINVAL;
}

#endif /* CONFIG_GPIOLIB */

#endif /* __ASM_OPENRISC_GPIO_H */
