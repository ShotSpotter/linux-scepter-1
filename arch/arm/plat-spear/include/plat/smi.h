/*
 * arch/arm/plat-spear/include/plat/smi.h
 *
 * Copyright (C) 2010 ST Microelectronics
 * Shiraz Hashim <shiraz.hashim@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef __PLAT_SMI_H
#define __PLAT_SMI_H

#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>

/* macro to define partitions for flash devices */
#define DEFINE_PARTS(n, of, s)		\
{					\
	.name = n,			\
	.offset = of,			\
	.size = s,			\
}

/**
 * struct spear_smi_flash_info - platform structure for passing flash
 * information
 *
 * name: name of the serial nor flash for identification
 * mem_base: the memory base on which the flash is mapped
 * size: size of the flash in bytes
 * num_parts: number of partitions
 * parts: parition details
 * fast_mode: whether flash supports fast mode
 */

struct spear_smi_flash_info {
	char *name;
	unsigned long mem_base;
	unsigned long size;
	int num_parts;
	struct mtd_partition *parts;
	u8 fast_mode;
};

/**
 * struct spear_smi_plat_data - platform structure for configuring smi
 *
 * clk_rate: clk rate at which SMI must operate
 * num_flashes: number of flashes present on board
 * board_flash_info: specific details of each flash present on board
 */
struct spear_smi_plat_data {
	unsigned long clk_rate;
	int num_flashes;
	struct spear_smi_flash_info *board_flash_info;
};

static inline void smi_set_plat_data(struct platform_device *pdev,
		struct spear_smi_plat_data *pdata)
{
	pdev->dev.platform_data = pdata;
}

#endif /* __PLAT_SMI_H */
