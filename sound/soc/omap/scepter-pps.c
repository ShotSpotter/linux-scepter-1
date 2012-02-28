/*
 * scepter-pps.c
 *
 * Author: Sarah Newman <snewman@shotspotter.com
 *
 *
 * Based off of omap3beagle.c by Steve Sakoman
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by t he Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>
#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/generic-slave.h"

static int scepter_pps_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 12228000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops scepter_pps_ops = {
	.hw_params = scepter_pps_hw_params,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link scepter_pps_dai = {
	.name = "PPS",
	.stream_name = "PPS",
	.cpu_dai = &omap_mcbsp_dai[2],
	.codec_dai = &generic_slave_dai,
	.ops = &scepter_pps_ops,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_scepter_pps = {
	.name = "scepter-pps",
	.platform = &omap_soc_platform,
	.dai_link = &scepter_pps_dai,
	.num_links = 1,
};

/* Audio subsystem */
static struct snd_soc_device scepter_pps_snd_devdata = {
	.card = &snd_soc_scepter_pps,
	.codec_dev = &soc_codec_dev_generic_slave,
};

static struct platform_device *scepter_pps_snd_device;

static int __init scepter_pps_soc_init(void)
{
	int ret;


	scepter_pps_snd_device = platform_device_alloc("soc-audio", 2);
	if (!scepter_pps_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(scepter_pps_snd_device, &scepter_pps_snd_devdata);
	scepter_pps_snd_devdata.dev = &scepter_pps_snd_device->dev;
	*(unsigned int *)scepter_pps_dai.cpu_dai->private_data = 2; /* McBSP3 */

	ret = platform_device_add(scepter_pps_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(scepter_pps_snd_device);

	return ret;
}

static void __exit scepter_pps_soc_exit(void)
{
	platform_device_unregister(scepter_pps_snd_device);
}

module_init(scepter_pps_soc_init);
module_exit(scepter_pps_soc_exit);

MODULE_AUTHOR("Sarah Newman <snewman@shotspotter.com>");
MODULE_DESCRIPTION("ALSA SoC Scepter PPS");
MODULE_LICENSE("GPL");
