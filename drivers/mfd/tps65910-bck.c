/*
 * tps65910-bck.c  --  TI TPS6591x
 *
 * Copyright 2013 ShotSpotter Inc.
 *
 * Author: Sarah Newman <snewman@shotspotter.com>
 *
 * Roughly based on tps65910-bck.c and mcp3001.c
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mfd/tps65910.h>

#define DRIVER_NAME "tps65910-bck"

static int tps65910_bck_enabled = 0;

static ssize_t tps65910_bck_show(struct device *dev, struct device_attribute *attr,
			   char *buf);

static ssize_t tps65910_bck_store(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count);

#define bck(offset) \
static DEVICE_ATTR(bck##offset##_reg, 0660, tps65910_bck_show, tps65910_bck_store)

bck(1);
bck(2);
bck(3);
bck(4);
bck(5);


#define bck_arr(offset) &dev_attr_bck##offset##_reg.attr

static struct attribute *tps65910_bck_attrs[] = {
	bck_arr(1),
	bck_arr(2),
	bck_arr(3),
	bck_arr(4),
	bck_arr(5),
	NULL,
};

static const struct attribute_group tps65910_bck_attr_group = {
	.attrs = (struct attribute **) tps65910_bck_attrs,
};

static int tps65910_bck_attr_index(struct attribute *attr)
{
	int i;
	const struct attribute *p;

	for (i = 0; (p = tps65910_bck_attrs[i]) != NULL; i++) {
		if (p == attr) {
			return i;
		}
	}
	return -1;
}

static ssize_t tps65910_bck_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct tps65910 *tps65910 = dev_get_drvdata(dev);
	u8 reg;
	int len = 0;
	int index;

  index = tps65910_bck_attr_index(&attr->attr);

  if (index < 0) {
  	dev_dbg(dev, DRIVER_NAME ": invalid attribute index.\n");
  	return 0;
  }

  tps65910->read(tps65910,TPS65910_BCK1+index,1,&reg);
 	len += sprintf(buf+len,"%d\n", (int)reg);
	return len;
}

static ssize_t tps65910_bck_store(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t count)
{
	struct tps65910 *tps65910 = dev_get_drvdata(dev);
	int tmpreg;
	int index;
	int c;

  index = tps65910_bck_attr_index(&attr->attr);

  if (index < 0 ) {
  	dev_dbg(dev, DRIVER_NAME ": invalid attribute index.\n");
  	return 0;
  }

 	c = sscanf(buf, "%du", &tmpreg);
 	if((c > 0) && (tmpreg <= 255)) {
 		u8 reg = (u8) tmpreg;
 		tps65910->write(tps65910,TPS65910_BCK1+index,1,&reg);
 	}
	return count;
}

int tps65910_bck_init(struct tps65910 *tps65910, struct bck_reg_mode_data *bck_reg_modes)
{
	int status, i;

	if(!bck_reg_modes)
		return 0;

	for(i = 0; i < TPS65910_BCK_COUNT; i++) {
		tps65910_bck_attrs[i]->mode = bck_reg_modes->mode[i];
	}

	status = sysfs_create_group(&(tps65910->dev->kobj),&tps65910_bck_attr_group);
	if (status != 0) {
		dev_dbg(tps65910->dev, DRIVER_NAME ": sysfs create group failed.\n");
	}
	tps65910_bck_enabled = 1;

	return status;
}

int tps65910_bck_exit(struct tps65910 *tps65910)
{
	if(tps65910_bck_enabled)
		sysfs_remove_group(&(tps65910->dev->kobj),&tps65910_bck_attr_group);
	return 0;
}

