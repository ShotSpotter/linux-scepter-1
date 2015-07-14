/*
 * linux/arch/arm/mach-omap2/board-scepter-pps.c
 *
 * Copyright (C) 2015 shotspotter inc
 * Author: Rob Calhoun <rcalhoun@shotspotter.com>
 *
 * Based on mach-omap2/board-scepter.c and sample clients in drivers/pps/clients
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

#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/gpmc.h>
#include <plat/mmc.h>
#include "mux.h"


/* kernelpps includes */

#include <linux/module.h>
#include <linux/time.h>
#include <linux/pps_kernel.h>


/*
 * kernel pps integration, see https://www.kernel.org/doc/Documentation/pps/pps.txt
 * BUT keep in mind this is 2.6.34 so look at drivers/kernelpps/clients.
 * Also see https://www.ietf.org/rfc/rfc2783.txt which is modestly helpful.
 */

static int pps_source = -1;  // holds our source id, int >=0.

static void pps_gpio_scepter_echo(int source, int event, void *data)
{
  printk(KERN_INFO "echo pps %s%s for source %d",
          event & PPS_CAPTUREASSERT ? "assert" : "",
          event & PPS_CAPTURECLEAR ? "clear" : "",
          source);
}


static struct pps_source_info pps_gpio_scepter_info = {
  .name         = "scepterpps",
  .path         = "/dev/ttyS3",
  .mode         = PPS_CAPTUREASSERT | PPS_CAPTURECLEAR | PPS_OFFSETASSERT | PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC,
  .echo         = pps_gpio_scepter_echo,
  .owner        = THIS_MODULE,
};

static int init_kernel_pps(void)
{
  int ret;
  ret = pps_register_source(&pps_gpio_scepter_info, PPS_CAPTUREASSERT | PPS_CAPTURECLEAR | PPS_CANWAIT | PPS_TSFMT_TSPEC);
  if (ret < 0) {
    printk(KERN_ERR "Cannot register kernel pps source! (err %d)", ret);
    return ret;
  } else {
    pps_source = ret; // module scope!
    printk(KERN_NOTICE "Registered kernel pps source %d",pps_source);
  }
  return 0;
}


static void pps_gpio_scepter_event(int source, int rising_edge)
{
  struct timespec __ts;
  struct pps_ktime ts;

  if (pps_source < 0) {
    printk(KERN_ERR "pps_gpio_scepter_event called without valid source");
  }

  /* First of all we get the time stamp... */
  getnstimeofday(&__ts);

  // jiffies and HZ are linux globals
  //printk(KERN_DEBUG "PPS event at %lu, value %d", jiffies, rising_edge);

  /* ... and translate it to PPS time data struct */
  ts.sec = __ts.tv_sec;
  ts.nsec = __ts.tv_nsec;

  if (rising_edge) {
    pps_event(source, &ts, PPS_CAPTUREASSERT, NULL);
  } else {
    pps_event(source, &ts, PPS_CAPTURECLEAR, NULL);
  }
}


/*
 * SCP1 has the gps pps line wired into two places:
 * a) MCSPI2_SOMI/GPT10_PWM_EVT/HSUSB2_DATA5/GPIO_180 (H22)
 * b) McBSP3_DR/UART2_RTS/GPIO_141 (B4)
 *
 * (a) is used to drive the kernel pps feature, intialized below.
 * (b) is used to send pps signal into McBSP3 as data, which is then
 *     sampled synchronously with the audio data and used by SensApp
 *     to timestamp the audio. Since this is bonkers, we are switching
 *     to kernel pps + sysclock + (if necssary) a call to the mcbsp
 *     driver to get current audio sample during interrupt time.
 *
 * Some of the code below is a little weird; that is because I modified
 * Hunyue's board test code to implement this. rcalhoun 2015-07-010
 */

struct gpio_name_t {
	int gpio;
	int hitjustonce;
	const char* name;
};

struct gpio_name_t gpios[] = {
		{180,0,"gps-pps"},
};


static irqreturn_t gps_pps_interrupt(int irq, void *dev_id)
{

  struct gpio_name_t *gpn = (struct gpio_name_t *)dev_id;
  int rising_edge;
  rising_edge = gpio_get_value(gpn->gpio);
  pps_gpio_scepter_event(pps_source,rising_edge);
  return IRQ_HANDLED;
}


static int init_gps_pps_gpios(void)
{
  int irq;
  int ret = 0;
  struct gpio_name_t* gpn = &(gpios[0]);

  printk(KERN_NOTICE "Initializing gps-pps");
  ret = omap_mux_init_gpio(gpn->gpio,OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE4);
  if (ret) {
    printk(KERN_ERR "could not change pin mux for %d",gpn->gpio);
    goto bad_gpio;
  }
  ret = gpio_request(gpn->gpio,gpn->name);
  if (ret) {
    printk(KERN_ERR "could not get gpio %d",gpn->gpio);
    goto bad_gpio;
  }
  gpio_direction_input(gpn->gpio);
  irq = gpio_to_irq(gpn->gpio);
  printk(KERN_ERR "Assigning gpio %d irq %d",gpn->gpio,irq);
  ret = request_irq(irq,gps_pps_interrupt,IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,gpn->name, gpn);
  if(ret) {
    printk(KERN_ERR "could not request irq for %d",gpn->gpio);
	goto bad_irq;
  } else {
    printk(KERN_NOTICE "gpio %d irq %d",gpn->gpio,irq);
  }

  return ret;

bad_irq:
  free_irq(gpio_to_irq(gpios[0].gpio),NULL);

bad_gpio:
  gpio_free(gpios[0].gpio);
  return ret;
}


static void __exit scepter_pps_exit(void)
{
  pps_unregister_source(pps_source);
}

static void __init scepter_pps_init(void)
{
    printk(KERN_NOTICE "scepter: initialize kernel pps functions");
    init_kernel_pps();
    printk(KERN_NOTICE "scepter: assign irq to kernelpps");
    init_gps_pps_gpios();
}

module_init(scepter_pps_init);

module_exit(scepter_pps_exit);

MODULE_AUTHOR("Rob Calhoun <rcalhoun@shotspotter.com>");
MODULE_DESCRIPTION("SCP1 kernel pps support");
MODULE_LICENSE("GPL");
