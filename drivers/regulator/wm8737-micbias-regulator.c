/*
 * wm8737-micbias-regulator.c
 *
 * Copyright 2012 ShotSpotter Inc.
 *
 * Author: Sarah Newman <snewman@shotspotter.com>
 *
 * Based off of fixed.c by Mark Brown
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This is useful for systems with mixed controllable and
 * non-controllable regulators, as well as for allowing testing on
 * systems with no controllable regulators.
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <sound/soc.h>
/*TODO DO NOT WANT to reimplement soc i2c functions like other people
 */
#include <linux/regulator/wm8737-micbias-regulator.h>
#include "../../sound/soc/codecs/wm8737.h"
#include <linux/regulator/machine.h>

#define NUM_MICBIAS_LVL 4

struct wm8737_micbias_data {
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	int micbias_cached;
	int enabled;
	struct snd_soc_dai *dai;
	/*TODO pull avdd from supply regulator*/
	int avdd_mV; //Only support one value for now
	int mvdd_mV;
	struct mutex mutex;
	int micbias_level_uV[NUM_MICBIAS_LVL];
};


const static int AVDD_UNITY_MULTIPLIER = 1000;
const static int MVDD_OFFSET_MV = 170;
const static int AVDD_MULTIPLIER[NUM_MICBIAS_LVL] =
{
		0,
		750,
		900,
		1200
};

static int wm8737_micbias_is_enabled(struct regulator_dev *rdev)
{
	struct wm8737_micbias_data *data = rdev_get_drvdata(rdev);

	return data->enabled;
}

static int wm8737_micbias_enable(struct regulator_dev *rdev)
{
	struct wm8737_micbias_data *data = rdev_get_drvdata(rdev);
	struct snd_soc_dai *dai = data->dai;
	struct snd_soc_codec *codec	= dai->codec;
	int ret = 0;

	mutex_lock(&data->mutex);
	if(!codec && data->micbias_cached != 0) {
		ret = -ENXIO;
		goto out;
	}
	data->enabled = 1;

	if(!data->micbias_cached)
		goto out;

	snd_soc_update_bits(codec,WM8737_POWER_MANAGEMENT,
			WM8737_MICBIAS_MASK, data->micbias_cached);

out:
	mutex_unlock(&data->mutex);
	return ret;
}

static int wm8737_micbias_disable(struct regulator_dev *rdev)
{
	struct wm8737_micbias_data *data = rdev_get_drvdata(rdev);
	struct snd_soc_dai *dai = data->dai;
	struct snd_soc_codec *codec	= dai->codec;
	int ret = 0;

	mutex_lock(&data->mutex);
	if(!codec && data->micbias_cached != 0) {
		ret = -ENXIO;
		goto out;
	}

	data->enabled = 0;

	if(!data->micbias_cached)
		goto out;

	snd_soc_update_bits(codec,WM8737_POWER_MANAGEMENT,
			WM8737_MICBIAS_MASK, 0);

out:
	mutex_unlock(&data->mutex);
	return ret;
}


static int wm8737_micbias_get_voltage(struct regulator_dev *rdev)
{
	struct wm8737_micbias_data *data = rdev_get_drvdata(rdev);
	int ret;
	mutex_lock(&data->mutex);
	ret = data->micbias_level_uV[data->micbias_cached];
	mutex_unlock(&data->mutex);
	return ret;
}

static int
wm8737_micbias_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV)
{
	struct wm8737_micbias_data *data = rdev_get_drvdata(rdev);
	struct snd_soc_dai *dai = data->dai;
	struct snd_soc_codec *codec	= dai->codec;
	int vsel;

	if(!codec)
		return -ENXIO;

	for(vsel = 0; vsel < NUM_MICBIAS_LVL; vsel++) {
		int val_uV = data->micbias_level_uV[vsel];

		if(val_uV == min_uV)
			break;

		if(val_uV == max_uV)
			break;
	}

	if(vsel == NUM_MICBIAS_LVL)
		return -EDOM;

	mutex_lock(&data->mutex);

	data->micbias_cached = vsel;

	if(data->enabled)
		snd_soc_update_bits(codec,WM8737_POWER_MANAGEMENT,
			WM8737_MICBIAS_MASK, data->micbias_cached);

	mutex_unlock(&data->mutex);

	return 0;
}

static int wm8737_micbias_list_voltage(struct regulator_dev *rdev,
				      unsigned selector)
{
	struct wm8737_micbias_data *data = rdev_get_drvdata(rdev);

	if(selector >= NUM_MICBIAS_LVL)
		return -EINVAL;

	return data->micbias_level_uV[selector];
}

static struct regulator_ops wm8737_micbias_ops = {
	.is_enabled = wm8737_micbias_is_enabled,
	.enable = wm8737_micbias_enable,
	.disable = wm8737_micbias_disable,
	.get_voltage = wm8737_micbias_get_voltage,
	.set_voltage = wm8737_micbias_set_voltage,
	.list_voltage = wm8737_micbias_list_voltage,
};

static ssize_t micbias_voltage_available(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	int value = 0, len = 0;
	int i;

	for(i=0; value >= 0; i++) {
		value = wm8737_micbias_list_voltage(rdev,i);
		if(value >= 0)
			len += sprintf(buf+len,"%d\n",value);
	}

	return len;
}


static ssize_t micbias_voltage_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	long value;
	ssize_t			status;

	status = strict_strtol(buf, 0, &value);

	if (status == 0)
		status = wm8737_micbias_set_voltage(rdev,value,value);

	if(status == 0)
		status = size;

	return status;
}

static const DEVICE_ATTR(microvolts_set, 0200,
		NULL, micbias_voltage_store);

static const DEVICE_ATTR(microvolts_available, 0444,
		micbias_voltage_available, NULL);


static int __devinit wm8737_micbias_probe(struct platform_device *pdev)
{
	struct wm8737_micbias_config *config = pdev->dev.platform_data;
	struct wm8737_micbias_data *drvdata;
	const int NAME_SZ = 32;
	char *name;
	int ret;
	int id;
	int i;

	if(pdev->id > MAX_WM8737_ID) {
		dev_err(&pdev->dev, "Invalid wm8737 ID of %d\n",pdev->id);
		return -EINVAL;
	}

	if(pdev->id < 0)
		id = 0;
	else
		id = pdev->id;

	drvdata = kzalloc(sizeof(struct wm8737_micbias_data), GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&pdev->dev, "Failed to allocate device data\n");
		ret = -ENOMEM;
		return ret;
	}

	name = kzalloc(NAME_SZ, GFP_KERNEL);
	if (name == NULL) {
		dev_err(&pdev->dev, "Failed to allocate supply name\n");
		ret = -ENOMEM;
		goto err_drvdata;
	}

	snprintf(name,NAME_SZ,"wm8737-micbias-reg-%d",id);
	name[NAME_SZ-1] = '\0';
	drvdata->desc.name = name;

	drvdata->desc.type = REGULATOR_VOLTAGE;
	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.ops = &wm8737_micbias_ops;
	drvdata->desc.n_voltages = NUM_MICBIAS_LVL;

	drvdata->micbias_cached = 0;
	drvdata->dai = &wm8737_dai[id];

	drvdata->enabled = 0;
	drvdata->avdd_mV = config->avdd_mV;
	drvdata->mvdd_mV = config->mvdd_mV;
	for(i = 0; i < NUM_MICBIAS_LVL; i++) {
		int tmp = config->avdd_mV * AVDD_MULTIPLIER[i];
		if(tmp > (config->mvdd_mV-MVDD_OFFSET_MV)*AVDD_UNITY_MULTIPLIER)
			tmp  = (config->mvdd_mV-MVDD_OFFSET_MV)*AVDD_UNITY_MULTIPLIER;
		drvdata->micbias_level_uV[i] = tmp;
	}

	config->init_data->constraints.min_uV = 1;
	config->init_data->constraints.max_uV = drvdata->micbias_level_uV[NUM_MICBIAS_LVL-1];

	mutex_init(&drvdata->mutex);

	drvdata->rdev = regulator_register(&drvdata->desc, &pdev->dev,
					  config->init_data, drvdata);

	if (IS_ERR(drvdata->rdev)) {
		ret = PTR_ERR(drvdata->rdev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		goto err_name;
	}

	ret = device_create_file(&drvdata->rdev->dev, &dev_attr_microvolts_set);
	if (ret < 0)
		goto err_regulator;

	ret = device_create_file(&drvdata->rdev->dev, &dev_attr_microvolts_available);
	if (ret < 0)
		goto err_regulator;

	platform_set_drvdata(pdev, drvdata);

	return 0;

err_regulator:
	regulator_unregister(drvdata->rdev);
err_name:
	kfree(drvdata->desc.name);
err_drvdata:
	kfree(drvdata);

	return ret;
}

static int __devexit wm8737_micbias_remove(struct platform_device *pdev)
{
	struct wm8737_micbias_data *drvdata = platform_get_drvdata(pdev);

	regulator_unregister(drvdata->rdev);
	kfree(drvdata->desc.name);
	kfree(drvdata);

	return 0;
}

static struct platform_driver wm8737_micbias_regulator_voltage_driver = {
	.probe		= wm8737_micbias_probe,
	.remove		= __devexit_p(wm8737_micbias_remove),
	.driver		= {
		.name		= "wm8737-micbias-reg",
		.owner		= THIS_MODULE,
	},
};

static int __init wm8737_micbias_regulator_voltage_init(void)
{
	return platform_driver_register(&wm8737_micbias_regulator_voltage_driver);
}
subsys_initcall(wm8737_micbias_regulator_voltage_init);

static void __exit wm8737_micbias_regulator_voltage_exit(void)
{
	platform_driver_unregister(&wm8737_micbias_regulator_voltage_driver);
}
module_exit(wm8737_micbias_regulator_voltage_exit);

MODULE_AUTHOR("Sarah Newman <snewman@shotspotter.com>");
MODULE_DESCRIPTION("wm8737 micbias voltage driver");
MODULE_LICENSE("GPL");
