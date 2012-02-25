/*
 * wm8737-sync.c  --  audio for synchronized wm8737's
 *
 * Copyright (C) 2012 ShotSpotter Inc.
 *
 * Author: Sarah Newman <snewman@shotspotter.com>
 *
 * Initially based off of sound/soc/zoom2.c
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
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
#include <mach/scepter.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/wm8737.h"

static int wm8737_omap_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops wm8737_omap_ops = {
	.hw_params = wm8737_omap_hw_params,
};

/* Wm8737_omap machine DAPM */
static const struct snd_soc_dapm_widget wm8737_omap_twl4030_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
	SND_SOC_DAPM_LINE("Aux In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* External Mics: MAINMIC, SUBMIC with bias*/
	{"MAINMIC", NULL, "Mic Bias 1"},
	{"SUBMIC", NULL, "Mic Bias 2"},
	{"Mic Bias 1", NULL, "Ext Mic"},
	{"Mic Bias 2", NULL, "Ext Mic"},

	/* External Speakers: HFL, HFR */
	{"Ext Spk", NULL, "HFL"},
	{"Ext Spk", NULL, "HFR"},

	/* Headset Stereophone:  HSOL, HSOR */
	{"Headset Stereophone", NULL, "HSOL"},
	{"Headset Stereophone", NULL, "HSOR"},

	/* Headset Mic: HSMIC with bias */
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	/* Aux In: AUXL, AUXR */
	{"Aux In", NULL, "AUXL"},
	{"Aux In", NULL, "AUXR"},
};

static int wm8737_omap_twl4030_init(struct snd_soc_codec *codec)
{
	int ret;

	/* Add Wm8737_omap specific widgets */
	ret = snd_soc_dapm_new_controls(codec, wm8737_omap_twl4030_dapm_widgets,
				ARRAY_SIZE(wm8737_omap_twl4030_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Wm8737_omap specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* Wm8737_omap connected pins */
	snd_soc_dapm_enable_pin(codec, "Ext Mic");
	snd_soc_dapm_enable_pin(codec, "Ext Spk");
	snd_soc_dapm_enable_pin(codec, "Headset Mic");
	snd_soc_dapm_enable_pin(codec, "Headset Stereophone");
	snd_soc_dapm_enable_pin(codec, "Aux In");

	/* TWL4030 not connected pins */
	snd_soc_dapm_nc_pin(codec, "CARKITMIC");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC0");
	snd_soc_dapm_nc_pin(codec, "DIGIMIC1");

	snd_soc_dapm_nc_pin(codec, "OUTL");
	snd_soc_dapm_nc_pin(codec, "OUTR");
	snd_soc_dapm_nc_pin(codec, "EARPIECE");
	snd_soc_dapm_nc_pin(codec, "PREDRIVEL");
	snd_soc_dapm_nc_pin(codec, "PREDRIVER");
	snd_soc_dapm_nc_pin(codec, "CARKITL");
	snd_soc_dapm_nc_pin(codec, "CARKITR");

	ret = snd_soc_dapm_omap(codec);

	return ret;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link wm8737_omap_dai[] = {
	{
		.name = "TWL4030 I2S",
		.stream_name = "TWL4030 Audio",
		.cpu_dai = &omap_mcbsp_dai[0],
		.codec_dai = &twl4030_dai[TWL4030_DAI_HIFI],
		.init = wm8737_omap_twl4030_init,
		.ops = &wm8737_omap_ops,
	}
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_wm8737_omap = {
	.name = "Wm8737_omap",
	.platform = &omap_soc_platform,
	.dai_link = wm8737_omap_dai,
	.num_links = ARRAY_SIZE(wm8737_omap_dai),
};

/* EXTMUTE callback function */
void wm8737_omap_set_hs_extmute(int mute)
{
	gpio_set_value(Wm8737_omap_HEADSET_EXTMUTE_GPIO, mute);
}

/* Audio subsystem */
static struct snd_soc_device wm8737_omap_snd_devdata = {
	.card = &snd_soc_wm8737_omap,
	.codec_dev = &soc_codec_dev_twl4030,
	.codec_data = &twl4030_setup,
};

static struct platform_device *wm8737_omap_snd_device;

static int __init wm8737_omap_soc_init(void)
{
	int ret;

	printk(KERN_INFO "wm8737_omap SoC init\n");

	wm8737_omap_snd_device = platform_device_alloc("soc-audio", -1);
	if (!wm8737_omap_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(wm8737_omap_snd_device, &wm8737_omap_snd_devdata);
	wm8737_omap_snd_devdata.dev = &wm8737_omap_snd_device->dev;
	*(unsigned int *)wm8737_omap_dai[0].cpu_dai->private_data = 0; /* McBSP1 */
	*(unsigned int *)wm8737_omap_dai[1].cpu_dai->private_data = 1; /* McBSP2 */
	*(unsigned int *)wm8737_omap_dai[2].cpu_dai->private_data = 2; /* McBSP3 */

	ret = platform_device_add(wm8737_omap_snd_device);
	if (ret)
		goto err1;

	BUG_ON(gpio_request(Wm8737_omap_HEADSET_MUX_GPIO, "hs_mux") < 0);
	gpio_direction_output(Wm8737_omap_HEADSET_MUX_GPIO, 0);

	BUG_ON(gpio_request(Wm8737_omap_HEADSET_EXTMUTE_GPIO, "ext_mute") < 0);
	gpio_direction_output(Wm8737_omap_HEADSET_EXTMUTE_GPIO, 0);

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(wm8737_omap_snd_device);

	return ret;
}
module_init(wm8737_omap_soc_init);

static void __exit wm8737_omap_soc_exit(void)
{
	gpio_free();
	gpio_free();

	platform_device_unregister(wm8737_sync_snd_device);
}
module_exit(wm8737_sync_soc_exit);

/*TODO consider if wm8737 omap specific stuff and sync-specific stuff can be broken into 2 modules */

struct wm8737_sync_soc {
	struct snd_soc_card sound_soc_wm8737;
	struct snd_soc_device wm8737_snd_devdata;
	struct wm8737_setup_data wm8737_setup;
	struct platform_device *wm8737_snd_device;
	struct snd_soc_dai_link wm8737_dai;
};

struct wm8737_sync_data {
	int cntr_rst_gpio;
	int mclk_gate_gpio;
	int audio_mclk;
	struct wm8737_sync_card master;
	struct wm8737_sync_card slave[MAX_WM8737_SLAVES];
	int num_slaves;
};

static struct platform_driver gpio_led_driver = {
	.probe		= wm8737_sync_probe,
	.remove		= __devexit_p(wm8737_sync_remove),
	.driver		= {
		.name	= "wm8737-sync",
		.owner	= THIS_MODULE,
	},
};

static int __devinit wm8737_sync_probe(struct platform_device *pdev) {
	struct wm8737_sync_platform_data *pdata = pdev->dev.platform_data;
	struct wm8737_sync_data	*sync_data;
	int ret = 0;

	if(!pdata)
		return -EBUSY;

	sync_data = kzalloc(sizeof(wm8737_sync_data),GFP_KERNEL);
	if(!sync_data)
		return -ENOMEM;

	if(pdata->sample_cnt_rst_gpio >= 0){
		if(gpio_request(pdata->sample_cnt_rst_gpio, "sample_cnt_rst") < 0) {
			ret = -EINVAL;
			goto free_mem;
		}
		gpio_direction_output(pdata->sample_cnt_rst_gpio, 1);
	}

	if(pdata->mclk_en_gpio >= 0){
		if(gpio_request(pdata->mclk_en_gpio, "mclk_en") < 0) {
			ret = -EINVAL;
			goto free_cnt_gpio;
		}

		gpio_direction_output(pdata->mclk_en_gpio, 0);
	}

	/*do stuff*/

	platform_set_drvdata(pdev, sync_data);
	return ret;

free_cnt_gpio:

free_mem:
	kfree(sync_data);
}

static int __devexit wm8737_sync_remove(struct platform_device *pdev)
{
	struct wm8737_sync_platform_data *pdata = pdev->dev.platform_data;
	struct wm8737_sync_data	*sync_data;

	sync_data = platform_get_drvdata(pdev);
	/* Do stuff */
	kfree(sync_data);
}

MODULE_AUTHOR("Sarah Newman <snewman@shotspotter.com>");
MODULE_DESCRIPTION("ALSA SoC wm8737-sync");
MODULE_LICENSE("GPL");

