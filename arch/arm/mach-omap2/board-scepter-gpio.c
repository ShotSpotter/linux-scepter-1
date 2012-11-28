/*
 * linux/arch/arm/mach-omap2/board-scepter-gpio.c
 *
 * Copyright (C) 2012 ShotSpotter Inc.
 * Author: Sarah Newman <snewman@shotspotter.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/davinci_emac.h>
#include <linux/i2c/at24.h>
#include <linux/regulator/machine.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "hsmmc.h"
#include "mux.h"

struct gpio_export_t {
	const char* name;
	int num;
	int is_input;
	int initial_value;
};


static int __init scepter_gpio_init(struct gpio_export_t* gpio)
{
	int r;
	omap_mux_init_gpio(gpio->num,
		gpio->is_input ? OMAP_PIN_INPUT : OMAP_PIN_OUTPUT);
	r = gpio_request(gpio->num,gpio->name);
	if (r < 0) {
		printk(KERN_ERR "failed to request GPIO#%d\n", gpio->num);
		return -EBUSY;
	}
	if(gpio->is_input)
		gpio_direction_input(gpio->num);
	else
		gpio_direction_output(gpio->num,gpio->initial_value);
	gpio_export(gpio->num,0);
	return 0;
}

struct gpio_export_t __initdata scepter_gpio_revb[] =
	{
		{ .name =  "tps65910-sleep",
			.num = 1,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gps-pwron",
			.num = 34,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "gps-rst",
			.num = 35,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "eth-relay",
			.num = 36,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "power-good-3810",
			.num = 38,
			.is_input = 1,
		},
		{ .name =  "low-voltage-4356",
			.num = 39,
			.is_input = 1,
		},
		{ .name =  "fault-4356",
			.num = 40,
			.is_input = 1,
		},
		{ .name =  "gsm-wdis",
			.num = 53,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "gsm-1v8-pwron",
			.num = 56,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-pwron",
			.num = 55,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-hwr",
			.num = 64,
			.is_input = 1,
		},
		{ .name =  "gsm-3304-on",
			.num = 54,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "usb-speed",
			.num = 74,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "usb-softcon",
			.num = 75,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-txon",
			.num = 63,
			.is_input = 1,
		},
		{ .name =  "eth-rst-n",
			.num = 65,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "force-shutdown",
			.num = 66,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "cpld-rev0",
			.num = 67,
			.is_input = 1,
		},
		{ .name =  "cpld-rev1",
			.num = 68,
			.is_input = 1,
		},
		{ .name =  "cpld-rev2",
			.num = 69,
			.is_input = 1,
		},
		{ .name =  "cpld-heartbeat",
			.num = 76,
			.is_input = 1,
		},
		{ .name =  "eeprom-wp",
			.num = 77,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "shutdown-flag",
			.num = 79,
			.is_input = 1,
		},
		{ .name =  "good-4356",
			.num = 100,
			.is_input = 1,
		},
		{ .name =  "board-rev0",
			.num = 126,
			.is_input = 1,
		},
		{ .name =  "board-rev1",
			.num = 127,
			.is_input = 1,
		},
		{ .name =  "board-rev2",
			.num = 136,
			.is_input = 1,
		},
		{ .name =  "board-rev3",
			.num = 137,
			.is_input = 1,
		},
		{
				.name = NULL
		}
};

struct gpio_export_t __initdata scepter_gpio_401_0061_02[] =
	{
		{ .name =  "tps65910-sleep",
			.num = 1,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gps-pwron",
			.num = 34,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "gps-rst",
			.num = 35,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "power-good-3810",
			.num = 38,
			.is_input = 1,
		},
		{ .name =  "low-voltage-4356",
			.num = 39,
			.is_input = 1,
		},
		{ .name =  "fault-4356",
			.num = 40,
			.is_input = 1,
		},
		{ .name =  "gsm-wdis",
			.num = 53,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "gsm-1v8-pwron",
			.num = 56,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-pwron",
			.num = 55,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-hwr",
			.num = 64,
			.is_input = 1,
		},
		{ .name =  "gsm-3304-on",
			.num = 54,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "usb-speed",
			.num = 74,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "usb-softcon",
			.num = 75,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-txon",
			.num = 63,
			.is_input = 1,
		},
		{ .name =  "force-shutdown",
			.num = 66,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "cpld-rev0",
			.num = 67,
			.is_input = 1,
		},
		{ .name =  "cpld-rev1",
			.num = 68,
			.is_input = 1,
		},
		{ .name =  "cpld-rev2",
			.num = 69,
			.is_input = 1,
		},
		{ .name =  "cpld-heartbeat",
			.num = 76,
			.is_input = 1,
		},
		{ .name =  "eeprom-wp",
			.num = 77,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "shutdown-flag",
			.num = 79,
			.is_input = 1,
		},
		{ .name =  "good-4356",
			.num = 100,
			.is_input = 1,
		},
		{ .name =  "board-rev0",
			.num = 126,
			.is_input = 1,
		},
		{ .name =  "board-rev1",
			.num = 127,
			.is_input = 1,
		},
		{ .name =  "board-rev2",
			.num = 88,
			.is_input = 1,
		},
		{ .name =  "board-rev3",
			.num = 89,
			.is_input = 1,
		},
		{
				.name = NULL
		}
};

struct gpio_export_t __initdata scepter_gpio_400_0100_01[] =
	{
		{ .name =  "tps65910-sleep",
			.num = 1,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gps-pwron",
			.num = 34,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "gps-rst",
			.num = 35,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "power-good-3810",
			.num = 38,
			.is_input = 1,
		},
		{ .name =  "low-voltage-4356",
			.num = 39,
			.is_input = 1,
		},
		{ .name =  "fault-4356",
			.num = 40,
			.is_input = 1,
		},
		{ .name =  "force-shutdown",
			.num = 66,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "cpld-rev0",
			.num = 67,
			.is_input = 1,
		},
		{ .name =  "cpld-rev1",
			.num = 68,
			.is_input = 1,
		},
		{ .name =  "cpld-rev2",
			.num = 69,
			.is_input = 1,
		},
		{ .name =  "cpld-heartbeat",
			.num = 76,
			.is_input = 1,
		},
		{ .name =  "eeprom-wp",
			.num = 77,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "shutdown-flag",
			.num = 79,
			.is_input = 1,
		},
		{ .name =  "good-4356",
			.num = 100,
			.is_input = 1,
		},
		{ .name =  "board-rev0",
			.num = 126,
			.is_input = 1,
		},
		{ .name =  "board-rev1",
			.num = 127,
			.is_input = 1,
		},
		{ .name =  "board-rev2",
			.num = 88,
			.is_input = 1,
		},
		{ .name =  "board-rev3",
			.num = 89,
			.is_input = 1,
		},
		{ .name =  "eth-relay",
			.num = 36,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "eth-rst-n",
			.num = 65,
			.is_input = 0,
			.initial_value = 1,
		},
		{
				.name = NULL
		}
};

struct gpio_export_t __initdata scepter_gpio_400_0104_01[] =
	{
		{ .name =  "tps65910-sleep",
			.num = 1,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gps-pwron",
			.num = 34,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "gps-rst",
			.num = 35,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "power-good-3810",
			.num = 38,
			.is_input = 1,
		},
		{ .name =  "low-voltage-4356",
			.num = 39,
			.is_input = 1,
		},
		{ .name =  "fault-4356",
			.num = 40,
			.is_input = 1,
		},
		{ .name =  "force-shutdown",
			.num = 66,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "cpld-rev0",
			.num = 67,
			.is_input = 1,
		},
		{ .name =  "cpld-rev1",
			.num = 68,
			.is_input = 1,
		},
		{ .name =  "cpld-rev2",
			.num = 69,
			.is_input = 1,
		},
		{ .name =  "cpld-heartbeat",
			.num = 76,
			.is_input = 1,
		},
		{ .name =  "eeprom-wp",
			.num = 77,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "shutdown-flag",
			.num = 79,
			.is_input = 1,
		},
		{ .name =  "good-4356",
			.num = 100,
			.is_input = 1,
		},
		{ .name =  "board-rev0",
			.num = 126,
			.is_input = 1,
		},
		{ .name =  "board-rev1",
			.num = 127,
			.is_input = 1,
		},
		{ .name =  "board-rev2",
			.num = 88,
			.is_input = 1,
		},
		{ .name =  "board-rev3",
			.num = 89,
			.is_input = 1,
		},
		{ .name =  "eth-relay",
			.num = 36,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "eth-rst-n",
			.num = 65,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-wdis",
			.num = 53,
			.is_input = 0,
			.initial_value = 0,
		},
		{ .name =  "gsm-1v8-pwron",
			.num = 56,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-pwron",
			.num = 55,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-hwr",
			.num = 64,
			.is_input = 1,
		},
		{ .name =  "gsm-3304-on",
			.num = 54,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "usb-speed",
			.num = 74,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "usb-softcon",
			.num = 75,
			.is_input = 0,
			.initial_value = 1,
		},
		{ .name =  "gsm-txon",
			.num = 63,
			.is_input = 1,
		},
		{
				.name = NULL
		}
};

static struct gpio_export_t __initdata if_brd_gpio =
{
		.name = "if_brd_det",
		.num = -1,
		.is_input = 1,
};

void __init scepter_gpio_revb_init(void)
{
	struct gpio_export_t*	gpio;
	for(gpio = scepter_gpio_revb; gpio->name; gpio++) {
		scepter_gpio_init(gpio);
	}
}

void scepter_gpio_init_400_0104_01(void)
{
	struct gpio_export_t*	gpio;
	for(gpio = scepter_gpio_400_0104_01; gpio->name; gpio++) {
		scepter_gpio_init(gpio);
	}
	if_brd_gpio.num = 129;
	scepter_gpio_init(&if_brd_gpio);
}

void __init scepter_gpio_init_401_0061_02(void)
{
	struct gpio_export_t*	gpio;
	for(gpio = scepter_gpio_401_0061_02; gpio->name; gpio++) {
		scepter_gpio_init(gpio);
	}
	if_brd_gpio.num = 129;
	scepter_gpio_init(&if_brd_gpio);
}

void __init scepter_gpio_init_400_0100_01(void)
{
	struct gpio_export_t*	gpio;
	for(gpio = scepter_gpio_400_0100_01; gpio->name; gpio++) {
		scepter_gpio_init(gpio);
	}
	if_brd_gpio.num = 129;
	scepter_gpio_init(&if_brd_gpio);
}

int __init scepter_detect_if_brd(void)
{
	if(if_brd_gpio.num < 0)
		return -1;
	return gpio_get_value(if_brd_gpio.num);
}
