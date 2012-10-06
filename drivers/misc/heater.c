/*
 * linux/drivers/misc/heater.c
 *
 * Copyright (C) 2012 HY Research LLC
 * Author: Matthew R. Laue
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/kdev_t.h>
#include <linux/err.h>
#include <linux/gpio.h>

MODULE_AUTHOR("Matthew R. Laue");
MODULE_LICENSE("GPL");

struct heater {
	struct device *dev;
	int gpio;
	const char *name;
};

#define HEAT_ADC0_GPIO 70
#define HEAT_ADC1_GPIO 71
#define HEAT_3304_GPIO 37

static struct heater heaters[] = {
	{ .dev = NULL, .gpio = HEAT_ADC0_GPIO, .name = "heater-adc0" },
	{ .dev = NULL, .gpio = HEAT_ADC1_GPIO, .name = "heater-adc1" },
	{	.dev = NULL, .gpio = HEAT_3304_GPIO, .name = "heater-gsm" },
};

#define HEATER_CNT (sizeof(heaters)/sizeof(heaters[0]))

static ssize_t
heater_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct heater *h = dev_get_drvdata(dev);
	int value = gpio_get_value(h->gpio) ? 1 : 0;
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t
heater_store(struct device *dev, struct device_attribute *attr,
             const char *buf, size_t count)
{
	struct heater *h = dev_get_drvdata(dev);
	if (count > 2) {
		return -EINVAL;
	} else if (count == 2 && buf[1] != '\n') {
		return -EINVAL;
	} else if (count == 0) {
		return 0;
	} else if (buf[0] == '0') {
		gpio_set_value(h->gpio, 0);
		return count;
	} else if (buf[0] == '1') {
		gpio_set_value(h->gpio, 1);
		return count;
	}
	return -EINVAL;
}

static DEVICE_ATTR(value, S_IWUSR | S_IRUGO, heater_show, heater_store);

static const struct attribute *heater_attrs[] = {
	&dev_attr_value.attr,
	NULL,
};

static const struct attribute_group heater_attr_group = {
	.attrs = (struct attribute **) heater_attrs,
};

static struct class_attribute heater_class_attrs[] = {
	__ATTR_NULL,
};

static struct class heater_class = {
	.name = "heater",
	.owner = THIS_MODULE,
	.class_attrs = heater_class_attrs,
};

static int __init
heater_dev_init(struct heater *h)
{
	int retval;
	struct device *dev;

	dev = device_create(&heater_class, NULL, MKDEV(0, 0),
			    h, h->name);
	if (IS_ERR(dev)) {
		retval = PTR_ERR(dev);
		printk(KERN_ERR "device_create failed for %s: %d\n",
		       h->name, retval);
		return retval;
	}

	retval = device_create_file(dev, &dev_attr_value);
	if (retval != 0) {
		printk(KERN_ERR "device_create_file failed for %s: %d\n",
		       h->name, retval);
		device_unregister(dev);
		return retval;
	}

	retval = gpio_request(h->gpio, h->name);
	if (retval != 0) {
		printk(KERN_ERR "gpio_request for gpio%d failed: %d\n",
		       h->gpio, retval);
		device_unregister(dev);
		return retval;
	}

	gpio_direction_output(h->gpio, 0);
	h->dev = dev;

	return 0;
}

static int __init
heater_init(void)
{
	int retval, i;

	retval = class_register(&heater_class);
	if (retval < 0) {
		printk(KERN_ERR "heater class register failed: %d\n", retval);
		return retval;
	}

	for (i = 0; i < HEATER_CNT; i++) {
		retval = heater_dev_init(&heaters[i]);
		if (retval != 0) {
			goto error;
		}
	}

	printk(KERN_INFO "heater: initialized\n");
	return 0;

error:
	for (i = 0; i < 2; i++) {
		if (heaters[i].dev != NULL) {
			device_unregister(heaters[i].dev);
			gpio_free(heaters[i].gpio);
		}
	}
	return retval;
}

static void __exit
heater_exit(void)
{
	int i;
	for (i = 0; i < HEATER_CNT; i++) {
		device_unregister(heaters[i].dev);
	}
	class_unregister(&heater_class);
	printk(KERN_INFO "heater: exit\n");
}

module_init(heater_init);
module_exit(heater_exit);
