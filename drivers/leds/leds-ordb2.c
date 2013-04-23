/*
 * ORDB2 Quad MAC Led driver
 * Copyright (C) Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/leds.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <asm/io.h>

static struct platform_device *pdev;

static u32 led_value = 0x101;
static u32 *led_reg;

struct ordb2_led {
	struct led_classdev cdev;
	int onoff;
	int blink;
};

static int ordb2_led_blink(struct led_classdev *led_cdev,
				unsigned long *delay_on,
                                unsigned long *delay_off)
{
	struct ordb2_led    *data =
                container_of(led_cdev, struct ordb2_led, cdev);
	if (*delay_on == 0 || delay_off == 0) {
		*delay_on = 500;
		*delay_off = 500;
	}
	led_value |= data->blink;
	if (*delay_on <= *delay_off)
		led_value |= data->onoff;
	else
		led_value &= ~data->onoff;
	*led_reg = led_value;
	return 0;
}

static void ordb2_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct ordb2_led    *data =
                container_of(led_cdev, struct ordb2_led, cdev);
	led_value &= ~(data->onoff | data->blink);
	if (value)
		led_value |= data->onoff;
	*led_reg = led_value;
}

/* This really should be rewritten to feed from devicetree */

static struct ordb2_led system_led = {
	.cdev = {
		.name			= "ordb2:green:system",
		.max_brightness		= 1,
		.brightness_set		= ordb2_led_set,
		.blink_set		= ordb2_led_blink,
		.flags			= LED_CORE_SUSPENDRESUME,
	},
	.onoff = 1<<0,
	.blink = 1<<(8+0),
};

static struct ordb2_led error_led = {
	.cdev = {
		.name		= "ordb2:red:error",
		.brightness_set	= ordb2_led_set,
		.flags			= LED_CORE_SUSPENDRESUME,
	},
	.onoff = 1<<1,
	.blink = 1<<(8+1),
};

static int __devinit ordb2_led_probe(struct platform_device *pdev)
{
	int ret;

	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EBUSY;

	led_reg = ioremap(res->start, resource_size(res));
	if (!led_reg)
		return -ENOMEM;
	
	ret = led_classdev_register(&pdev->dev, &system_led.cdev);
	if (ret < 0)
		goto err2;

	ret = led_classdev_register(&pdev->dev, &error_led.cdev);
	if (ret < 0)
		goto err1;

	*led_reg = led_value;

	return 0;

err1:
	led_classdev_unregister(&system_led.cdev);
err2:
	iounmap(led_reg);
	led_reg = NULL;

	return ret;
}

static int __devexit ordb2_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&system_led.cdev);
	led_classdev_unregister(&error_led.cdev);
	if (led_reg) {
		iounmap(led_reg);
		led_reg = NULL;
	}
	return 0;
}

static struct of_device_id ordb2_leds_match[] = {
	{.compatible = "opencores,ordb2-leds" },
	{},
}

MODULE_DEVICE_TABLE(of, ordb2_led_match);

static struct platform_driver ordb2_led_driver = {
	.probe		= ordb2_led_probe,
	.remove		= __devexit_p(ordb2_led_remove),
	.driver		= {
		.name		= "ordb2-leds",
		.owner		= THIS_MODULE,
		.of_match_table = ordb2_leds_match,
	},
};

static int __init ordb2_led_init(void)
{
	int ret;

	ret = platform_driver_register(&ordb2_led_driver);
	if (ret < 0)
		goto out;

out:
	return ret;
}

static void __exit ordb2_led_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&ordb2_led_driver);
}

module_init(ordb2_led_init);
module_exit(ordb2_led_exit);

MODULE_AUTHOR("Henrik Nordstrom <henrik@henriknordstrom.net>");
MODULE_DESCRIPTION("ORSoC quadmac led driver");
MODULE_LICENSE("GPL");

