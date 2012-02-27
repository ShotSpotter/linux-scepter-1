/*
 * generic-slave.c  --  Driver for chips with no controls
 *
 * Copyright 2012 ShotSpotter Inc.
 * Author: Sarah Newman <snewman@shotspotter.com>
 *
 * Based off of ads117x.c by Graeme Gregory <gg@slimlogic.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "generic-slave.h"

#define GENERIC_SLAVE_RATES (SNDRV_PCM_RATE_8000_192000)

#define GENERIC_SLAVE_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_dai generic_slave_dai = {
	.name = "Generic slave",
	.id = 1,
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 32,
		.rates = GENERIC_SLAVE_RATES,
		.formats = GENERIC_SLAVE_FORMATS,},
};
EXPORT_SYMBOL_GPL(generic_slave_dai);

static int generic_slave_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->name = "generic_slave";
	codec->owner = THIS_MODULE;
	codec->dai = &generic_slave_dai;
	codec->num_dai = 1;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "generic_slave: failed to create pcms\n");
		kfree(codec);
		return ret;
	}

	return 0;
}

static int generic_slave_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	snd_soc_free_pcms(socdev);
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_generic_slave = {
	.probe =	generic_slave_probe,
	.remove =	generic_slave_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_generic_slave);

static __devinit int generic_slave_platform_probe(struct platform_device *pdev)
{
	generic_slave_dai.dev = &pdev->dev;
	return snd_soc_register_dai(&generic_slave_dai);
}

static int __devexit generic_slave_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&generic_slave_dai);
	return 0;
}

static struct platform_driver generic_slave_codec_driver = {
	.driver = {
			.name = "generic_slave",
			.owner = THIS_MODULE,
	},

	.probe = generic_slave_platform_probe,
	.remove = __devexit_p(generic_slave_platform_remove),
};

static int __init generic_slave_init(void)
{
	return platform_driver_register(&generic_slave_codec_driver);
}
module_init(generic_slave_init);

static void __exit generic_slave_exit(void)
{
	platform_driver_unregister(&generic_slave_codec_driver);
}
module_exit(generic_slave_exit);

MODULE_DESCRIPTION("ASoC generic_slave driver");
MODULE_AUTHOR("Sarah Newman");
MODULE_LICENSE("GPL");
