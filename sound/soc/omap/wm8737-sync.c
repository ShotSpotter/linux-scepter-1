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
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <mach/gpio.h>
#include <plat/wm8737-sync.h>
#include <sound/wm8737.h>


#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/wm8737.h"



/*TODO consider if wm8737 omap specific stuff and sync-specific stuff can be broken into 2 modules */

struct wm8737_sync_soc {
	struct snd_soc_card sound_soc;
	struct snd_soc_device snd_devdata;
	struct wm8737_setup_data wm8737_setup;
	struct platform_device *snd_device;
	struct snd_soc_dai_link dai;
	unsigned int codec_daifmt;
	unsigned int cpu_daifmt;
	struct wm8737_sync_data *sync_data;
};

struct wm8737_sync_data {
	int cnt_rst_gpio;
	int mclk_en_gpio;
	unsigned int mclk;
	struct wm8737_sync_soc master;
	struct wm8737_sync_soc slaves[MAX_WM8737_SLAVES];
	struct work_struct trigger_start_work;
	struct work_struct trigger_stop_work;

};


static int wm8737_sync_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct wm8737_sync_soc *wm8737_soc = container_of(rtd->dai,struct wm8737_sync_soc,dai);
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,wm8737_soc->codec_daifmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,wm8737_soc->cpu_daifmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, wm8737_soc->sync_data->mclk,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static int wm8737_sync_prepare(struct snd_pcm_substream *substream) {
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct wm8737_sync_soc *wm8737_soc = container_of(rtd->dai,struct wm8737_sync_soc,dai);
	struct wm8737_sync_data *sync_data = wm8737_soc->sync_data;

	runtime->hw.info &= ~(SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_PAUSE);

	if(gpio_is_valid(sync_data->mclk_en_gpio) && (0 != gpio_get_value(sync_data->mclk_en_gpio)))
		return -EPERM;

	if(gpio_is_valid(sync_data->cnt_rst_gpio)) {
		gpio_set_value(sync_data->cnt_rst_gpio,0);
		udelay(1);
	}

	return 0;
}

static int wm8737_master_sync_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct wm8737_sync_soc *wm8737_soc = container_of(rtd->dai,struct wm8737_sync_soc,dai);
	struct wm8737_sync_data *sync_data = wm8737_soc->sync_data;
	int err = 0;
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		schedule_work(&sync_data->trigger_start_work);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		schedule_work(&sync_data->trigger_stop_work);
		break;

	default:
		err = -EINVAL;
	}
	return err;
}
		
static void wm8737_sync_trigger_start_work(struct work_struct *work)
{
	struct wm8737_sync_data *sync_data = container_of(work, struct wm8737_sync_data, trigger_start_work);

	if(gpio_is_valid(sync_data->mclk_en_gpio)) {
		gpio_set_value(sync_data->mclk_en_gpio,1);
	}
}

static void wm8737_sync_trigger_stop_work(struct work_struct *work)
{
	struct wm8737_sync_data *sync_data = container_of(work, struct wm8737_sync_data, trigger_stop_work);

	if(gpio_is_valid(sync_data->cnt_rst_gpio)) {
		gpio_set_value(sync_data->cnt_rst_gpio,1);
	}

	if(gpio_is_valid(sync_data->mclk_en_gpio)) {
		gpio_set_value(sync_data->mclk_en_gpio,0);
	}
}

static struct snd_soc_ops wm8737_sync_master_ops = {
	.hw_params = wm8737_sync_hw_params,
	.prepare = wm8737_sync_prepare,
	.trigger = wm8737_master_sync_trigger,
};

static struct snd_soc_ops wm8737_sync_slave_ops = {
	.hw_params = wm8737_sync_hw_params,
	.prepare = wm8737_sync_prepare,
};


static int wm8737_omap_soc_register(struct wm8737_sync_soc *wm8737_soc,
		struct wm8737_omap_data *wm8737_omap, struct wm8737_sync_data *sync_data, int is_master)
{
	int ret;
	char name[32];

	if(wm8737_omap->wm8737_id > MAX_WM8737_ID) {
		printk(KERN_ERR "Invalid WM8737 ID %d\n",wm8737_omap->wm8737_id);
		return -EINVAL;		
	}
	
	if(wm8737_omap->mcbsp_id >= NUM_LINKS) {
		printk(KERN_ERR "Invalid McBSP number %d\n",wm8737_omap->mcbsp_id);
		return -EINVAL;
	}
	
	wm8737_soc->sync_data = sync_data;
	
	if((wm8737_omap->cpu_dai_audio_fmt & ~7) || (wm8737_omap->codec_dai_audio_fmt & ~7)) {
		printk(KERN_ERR "Too extensive wm8737 daifmt\n");
		return -EINVAL;
	}

	wm8737_soc->cpu_daifmt = wm8737_omap->cpu_dai_audio_fmt | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM;

	wm8737_soc->codec_daifmt = wm8737_omap->codec_dai_audio_fmt | SND_SOC_DAIFMT_NB_NF;
	wm8737_soc->codec_daifmt |= is_master ? SND_SOC_DAIFMT_CBM_CFM : SND_SOC_DAIFMT_CBS_CFS;
	
	wm8737_soc->snd_device = platform_device_alloc("soc-audio", wm8737_omap->wm8737_id);
	if (!wm8737_soc->snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}
	

	wm8737_soc->dai.name = "wm8737";
	wm8737_soc->dai.stream_name = "wm8737";
	snprintf(name,sizeof(name),"wm8737-%d",wm8737_omap->wm8737_id);
	name[sizeof(name)-1] = '\0';


	wm8737_soc->dai.name = name;
	wm8737_soc->dai.stream_name = name;

	/* cpu_dai does not have to point to the matching McBSP number but it seems like the easiest
	 * way to do it
	 */
	wm8737_soc->dai.cpu_dai = &omap_mcbsp_dai[wm8737_omap->mcbsp_id];
	wm8737_soc->dai.codec_dai = &wm8737_dai[wm8737_omap->wm8737_id];
	wm8737_soc->dai.ops = is_master ? &wm8737_sync_master_ops : &wm8737_sync_slave_ops;
	
	if(is_master)
		snprintf(name,sizeof(name),"wm8737-smaster-%d",wm8737_omap->wm8737_id);
	else
		snprintf(name,sizeof(name),"wm8737-sslave-%d",wm8737_omap->wm8737_id);


	name[sizeof(name)-1] = '\0';
	wm8737_soc->sound_soc.name = name;
	wm8737_soc->sound_soc.platform = &omap_soc_platform;
	wm8737_soc->sound_soc.dai_link = &wm8737_soc->dai;
	wm8737_soc->sound_soc.num_links = 1;
	
	wm8737_soc->wm8737_setup.id = wm8737_omap->wm8737_id;
	
	wm8737_soc->snd_devdata.card = &wm8737_soc->sound_soc;
	wm8737_soc->snd_devdata.codec_dev = &soc_codec_dev_wm8737;
	wm8737_soc->snd_devdata.codec_data = &wm8737_soc->wm8737_setup;
	
	platform_set_drvdata(wm8737_soc->snd_device, &wm8737_soc->snd_devdata);
	wm8737_soc->snd_devdata.dev = &wm8737_soc->snd_device->dev;
	*(unsigned int *)wm8737_soc->dai.cpu_dai->private_data =  wm8737_omap->mcbsp_id;

	ret = platform_device_add(wm8737_soc->snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(wm8737_soc->snd_device);

	return ret;
}

static void wm8737_omap_soc_unregister(struct wm8737_sync_soc *wm8737_soc)
{
	platform_device_unregister(wm8737_soc->snd_device);
}

static int __devinit wm8737_sync_probe(struct platform_device *pdev) {
	struct wm8737_sync_platform_data *pdata = pdev->dev.platform_data;
	struct wm8737_sync_data	*sync_data;
	int i, ret = 0;
	
	if(!pdata)
		return -EBUSY;

	sync_data = kzalloc(sizeof(struct wm8737_sync_data),GFP_KERNEL);
	if(!sync_data)
		return -ENOMEM;

	sync_data->cnt_rst_gpio = pdata->sample_cnt_rst_gpio;
	sync_data->mclk_en_gpio = pdata->mclk_en_gpio;
	INIT_WORK(&sync_data->trigger_start_work,	wm8737_sync_trigger_start_work);
	INIT_WORK(&sync_data->trigger_stop_work,	wm8737_sync_trigger_stop_work);
			
	if(gpio_is_valid(sync_data->cnt_rst_gpio)){
		if(gpio_request(sync_data->cnt_rst_gpio, "sample_cnt_rst") < 0) {
			ret = -EINVAL;
			goto free_mem;
		}
		gpio_direction_output(sync_data->cnt_rst_gpio, 1);
	}

	if(gpio_is_valid(sync_data->mclk_en_gpio)){
		if(gpio_request(sync_data->mclk_en_gpio, "mclk_en") < 0) {
			ret = -EINVAL;
			goto free_cnt_gpio;
		}

		gpio_direction_output(sync_data->mclk_en_gpio, 0);
	}
	
	ret = wm8737_omap_soc_register(&sync_data->master,pdata->master, sync_data, 1);
	if(ret)
		goto free_mclk_en_gpio;
		
	for(i = 0; i < pdata->num_slaves; i++) {
		ret = wm8737_omap_soc_register(&sync_data->slaves[i],&pdata->slaves[i], sync_data, 0);
		if(ret)
			goto free_slaves;
	}

	platform_set_drvdata(pdev, sync_data);
	return ret;
	
free_slaves:
	for(i = i-1; i >= 0; i--) {
		wm8737_omap_soc_unregister(&sync_data->slaves[i]);
	}

	wm8737_omap_soc_unregister(&sync_data->master);
	
free_mclk_en_gpio:
	gpio_free(sync_data->mclk_en_gpio);

free_cnt_gpio:
	gpio_free(sync_data->cnt_rst_gpio);

free_mem:
	kfree(sync_data);
	
	return ret;
}

static int __devexit wm8737_sync_remove(struct platform_device *pdev)
{
	struct wm8737_sync_platform_data *pdata = pdev->dev.platform_data;
	struct wm8737_sync_data	*sync_data;
	int i;
	
	sync_data = platform_get_drvdata(pdev);

	for(i = pdata->num_slaves - 1; i >= 0; i--) {
		wm8737_omap_soc_unregister(&sync_data->slaves[i]);
	}

	wm8737_omap_soc_unregister(&sync_data->master);
	
	if(gpio_is_valid(sync_data->mclk_en_gpio))
		gpio_free(sync_data->mclk_en_gpio);

	if(gpio_is_valid(sync_data->cnt_rst_gpio))
		gpio_free(sync_data->cnt_rst_gpio);
	
	kfree(sync_data);
	
	return 0;
}

static struct platform_driver wm8737_sync_driver = {
	.probe		= wm8737_sync_probe,
	.remove		= __devexit_p(wm8737_sync_remove),
	.driver		= {
		.name	= "wm8737-sync",
		.owner	= THIS_MODULE,
	},
};

static int __init wm8737_sync_init(void)
{
	int ret;

	ret = platform_driver_register(&wm8737_sync_driver);

	return ret;
}

static void __exit wm8737_sync_exit(void)
{
	platform_driver_unregister(&wm8737_sync_driver);
}
module_init(wm8737_sync_init);
module_exit(wm8737_sync_exit);


MODULE_AUTHOR("Sarah Newman <snewman@shotspotter.com>");
MODULE_DESCRIPTION("ALSA SoC wm8737-sync");
MODULE_LICENSE("GPL");

