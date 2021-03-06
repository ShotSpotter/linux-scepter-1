/*******************************************************************************
 *
 *  drivers/watchdog/cns3xxx_wdt.c
 *
 *  Watchdog timer driver for the CNS3XXX SOCs
 *
 *  Author: Scott Shu
 *
 *  Copyright (c) 2008 Cavium Networks
 *
 *  This file is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, Version 2, as
 *  published by the Free Software Foundation.
 *
 *  This file is distributed in the hope that it will be useful,
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or
 *  NONINFRINGEMENT.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this file; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or
 *  visit http://www.gnu.org/licenses/.
 *
 *  This file may also be available under a different license from Cavium.
 *  Contact Cavium Networks for more information
 *
 ******************************************************************************/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <linux/slab.h>

#include <mach/cns3xxx.h>

#define TWD_TIMER_LOAD			0x00
#define TWD_TIMER_COUNTER		0x04
#define TWD_TIMER_CONTROL		0x08
#define TWD_TIMER_INTSTAT		0x0C

#define TWD_WDOG_LOAD			0x20
#define TWD_WDOG_COUNTER		0x24
#define TWD_WDOG_CONTROL		0x28
#define TWD_WDOG_INTSTAT		0x2C
#define TWD_WDOG_RESETSTAT		0x30
#define TWD_WDOG_DISABLE		0x34

#define TWD_TIMER_CONTROL_ENABLE	(1 << 0)
#define TWD_TIMER_CONTROL_ONESHOT	(0 << 1)
#define TWD_TIMER_CONTROL_PERIODIC	(1 << 1)
#define TWD_TIMER_CONTROL_IT_ENABLE	(1 << 2)

struct cns3xxx_wdt {
	unsigned long	timer_alive;
	struct device	*dev;
	void __iomem	*base;
	int		irq;
	unsigned int	perturb;
	char		expect_close;
};

static struct platform_device *cns3xxx_wdt_dev;

static spinlock_t wdt_lock;

#define TIMER_MARGIN	60
static int cns3xxx_margin = TIMER_MARGIN;
module_param(cns3xxx_margin, int, 0);
MODULE_PARM_DESC(cns3xxx_margin,
	"CNS3XXX timer margin in seconds. (0 < cns3xxx_margin < 65536, default="
				__MODULE_STRING(TIMER_MARGIN) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

#define ONLY_TESTING	0
static int cns3xxx_noboot = ONLY_TESTING;
module_param(cns3xxx_noboot, int, 0);
MODULE_PARM_DESC(cns3xxx_noboot, "CNS3XXX watchdog action, "
	"set to 1 to ignore reboots, 0 to reboot (default="
					__MODULE_STRING(ONLY_TESTING) ")");

/*
 *	This is the interrupt handler.  Note that we only use this
 *	in testing mode, so don't actually do a reboot here.
 */
static irqreturn_t cns3xxx_wdt_fire(int irq, void *arg)
{
	struct cns3xxx_wdt *wdt = arg;

	/* Check it really was our interrupt */
	if (readl(wdt->base + TWD_WDOG_INTSTAT)) {
		dev_printk(KERN_CRIT, wdt->dev,
					"Triggered - Reboot ignored.\n");
		/* Clear the interrupt on the watchdog */
		writel(1, wdt->base + TWD_WDOG_INTSTAT);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/*
 *	cns3xxx_wdt_keepalive - reload the timer
 *
 *	Note that the spec says a DIFFERENT value must be written to the reload
 *	register each time.  The "perturb" variable deals with this by adding 1
 *	to the count every other time the function is called.
 */
static void cns3xxx_wdt_keepalive(struct cns3xxx_wdt *wdt)
{
	unsigned int count;

	/* Assume prescale is set to 256
	 * (CPU_Clock *1024*1024/prescale(256)/2 -1
	 */
	count = ((((cns3xxx_cpu_clock()*1000*1000)>>8) *
		cns3xxx_margin)>>1) - 1;

	/* Reload the counter */
	spin_lock(&wdt_lock);
	writel(count + wdt->perturb, wdt->base + TWD_WDOG_LOAD);
	wdt->perturb = wdt->perturb ? 0 : 1;
	spin_unlock(&wdt_lock);
}

static void cns3xxx_wdt_stop(struct cns3xxx_wdt *wdt)
{
	spin_lock(&wdt_lock);
	writel(0x12345678, wdt->base + TWD_WDOG_DISABLE);
	writel(0x87654321, wdt->base + TWD_WDOG_DISABLE);
	writel(0x0, wdt->base + TWD_WDOG_CONTROL);
	spin_unlock(&wdt_lock);
}

static void cns3xxx_wdt_start(struct cns3xxx_wdt *wdt)
{
	dev_printk(KERN_INFO, wdt->dev, "enabling watchdog.\n");

	/* This loads the count register but does NOT start the count yet */
	cns3xxx_wdt_keepalive(wdt);
	spin_lock(&wdt_lock);

	if (cns3xxx_noboot) {
		/* Enable watchdog - prescale=256, watchdog mode=0, enable=1 */
		writel(0x0000FF01, wdt->base + TWD_WDOG_CONTROL);
	} else {
		/* Enable watchdog - prescale=256, watchdog mode=1, enable=1 */
		writel(0x0000FF09, wdt->base + TWD_WDOG_CONTROL);
	}
	spin_unlock(&wdt_lock);
}

static int cns3xxx_wdt_set_heartbeat(int t)
{
	if (t < 0x0001 || t > 0xFFFF)
		return -EINVAL;

	cns3xxx_margin = t;
	return 0;
}

/*
 *	/dev/watchdog handling
 */
static int cns3xxx_wdt_open(struct inode *inode, struct file *file)
{
	struct cns3xxx_wdt *wdt = platform_get_drvdata(cns3xxx_wdt_dev);

	if (test_and_set_bit(0, &wdt->timer_alive))
		return -EBUSY;

	if (nowayout)
		__module_get(THIS_MODULE);

	file->private_data = wdt;

	/*
	 *	Activate timer
	 */
	cns3xxx_wdt_start(wdt);

	return nonseekable_open(inode, file);
}

static int cns3xxx_wdt_release(struct inode *inode, struct file *file)
{
	struct cns3xxx_wdt *wdt = file->private_data;

	/*
	 * Shut off the timer.
	 * Lock it in if it's a module and we set nowayout
	 */
	if (wdt->expect_close == 42)
		cns3xxx_wdt_stop(wdt);
	else {
		dev_printk(KERN_CRIT, wdt->dev,
				"unexpected close, not stopping watchdog!\n");
		cns3xxx_wdt_keepalive(wdt);
	}
	clear_bit(0, &wdt->timer_alive);
	wdt->expect_close = 0;
	return 0;
}

static ssize_t cns3xxx_wdt_write(struct file *file, const char *data,
						size_t len, loff_t *ppos)
{
	struct cns3xxx_wdt *wdt = file->private_data;

	/*
	 *	Refresh the timer.
	 */
	if (len) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			wdt->expect_close = 0;

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					wdt->expect_close = 42;
			}
		}
		cns3xxx_wdt_keepalive(wdt);
	}
	return len;
}

static struct watchdog_info ident = {
	.options		= WDIOF_SETTIMEOUT |
				  WDIOF_KEEPALIVEPING |
				  WDIOF_MAGICCLOSE,
	.identity		= "CNS3XXX Watchdog",
};

static long cns3xxx_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct cns3xxx_wdt *wdt = file->private_data;
	int ret;
	union {
		struct watchdog_info ident;
		int i;
	} uarg;

	if (_IOC_DIR(cmd) && _IOC_SIZE(cmd) > sizeof(uarg))
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = copy_from_user(&uarg, (void __user *)arg, _IOC_SIZE(cmd));
		if (ret)
			return -EFAULT;
	}

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		uarg.ident = ident;
		ret = 0;
		break;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		uarg.i = 0;
		ret = 0;
		break;

	case WDIOC_SETOPTIONS:
		ret = -EINVAL;
		if (uarg.i & WDIOS_DISABLECARD) {
			cns3xxx_wdt_stop(wdt);
			ret = 0;
		}
		if (uarg.i & WDIOS_ENABLECARD) {
			cns3xxx_wdt_start(wdt);
			ret = 0;
		}
		break;

	case WDIOC_KEEPALIVE:
		cns3xxx_wdt_keepalive(wdt);
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = cns3xxx_wdt_set_heartbeat(uarg.i);
		if (ret)
			break;

		cns3xxx_wdt_keepalive(wdt);
		/* Fall */
	case WDIOC_GETTIMEOUT:
		uarg.i = cns3xxx_margin;
		ret = 0;
		break;

	default:
		return -ENOTTY;
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = copy_to_user((void __user *)arg, &uarg, _IOC_SIZE(cmd));
		if (ret)
			ret = -EFAULT;
	}
	return ret;
}

/*
 *	System shutdown handler.  Turn off the watchdog if we're
 *	restarting or halting the system.
 */
static void cns3xxx_wdt_shutdown(struct platform_device *dev)
{
	struct cns3xxx_wdt *wdt = platform_get_drvdata(dev);

	if (system_state == SYSTEM_RESTART || system_state == SYSTEM_HALT)
		cns3xxx_wdt_stop(wdt);
}

/*
 *	Kernel Interfaces
 */
static const struct file_operations cns3xxx_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= cns3xxx_wdt_write,
	.unlocked_ioctl	= cns3xxx_wdt_ioctl,
	.open		= cns3xxx_wdt_open,
	.release	= cns3xxx_wdt_release,
};

static struct miscdevice cns3xxx_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &cns3xxx_wdt_fops,
};

static int __devinit cns3xxx_wdt_probe(struct platform_device *dev)
{
	struct cns3xxx_wdt *wdt;
	struct resource *res;
	int ret;

	printk(KERN_INFO "watchdog: cns3xxx_wdt_probe\n");

	/* We only accept one device, and it must have an id of -1 */
	if (dev->id != -1)
		return -ENODEV;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		goto err_out;
	}

	wdt = kzalloc(sizeof(struct cns3xxx_wdt), GFP_KERNEL);
	if (!wdt) {
		ret = -ENOMEM;
		goto err_out;
	}

	wdt->dev = &dev->dev;
	wdt->irq = platform_get_irq(dev, 0);
	printk(KERN_INFO "watchdog irq = %d\n", wdt->irq);
	if (wdt->irq < 0) {
		ret = -ENXIO;
		goto err_free;
	}
	wdt->base = ioremap(res->start, res->end - res->start + 1);
	printk(KERN_INFO "watchdog start_regs  = 0x%8X\n",
		(unsigned int)wdt->base);
	if (!wdt->base) {
		ret = -ENOMEM;
		goto err_free;
	}

	cns3xxx_wdt_miscdev.parent = &dev->dev;
	ret = misc_register(&cns3xxx_wdt_miscdev);
	if (ret) {
		dev_printk(KERN_ERR, wdt->dev,
			"cannot register miscdev on minor=%d (err=%d)\n",
							WATCHDOG_MINOR, ret);
		goto err_misc;
	}

	ret = request_irq(wdt->irq, cns3xxx_wdt_fire, IRQF_DISABLED,
			dev->name, wdt);
	if (ret) {
		dev_printk(KERN_ERR, wdt->dev,
			"cannot register IRQ%d for watchdog\n", wdt->irq);
		goto err_irq;
	}

	cns3xxx_wdt_stop(wdt);
	platform_set_drvdata(dev, wdt);
	cns3xxx_wdt_dev = dev;

	return 0;

err_irq:
	misc_deregister(&cns3xxx_wdt_miscdev);
err_misc:
	platform_set_drvdata(dev, NULL);
	iounmap(wdt->base);
err_free:
	kfree(wdt);
err_out:
	return ret;
}

static int __devexit cns3xxx_wdt_remove(struct platform_device *dev)
{
	struct cns3xxx_wdt *wdt = platform_get_drvdata(dev);

	printk(KERN_INFO "watchdog: cns3xxx_wdt_remove\n");

	platform_set_drvdata(dev, NULL);

	misc_deregister(&cns3xxx_wdt_miscdev);

	cns3xxx_wdt_dev = NULL;

	free_irq(wdt->irq, wdt);
	iounmap(wdt->base);
	kfree(wdt);
	return 0;
}


static struct platform_driver cns3xxx_wdt_driver = {
	.probe		= cns3xxx_wdt_probe,
	.remove		= __devexit_p(cns3xxx_wdt_remove),
	.shutdown	= cns3xxx_wdt_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "cns3xxx-wdt",
	},
};

static char banner[] __initdata = KERN_INFO
	"CNS3XXX Watchdog Timer, noboot=%d margin=%d sec (nowayout= %d)\n";

static int __init cns3xxx_wdt_init(void)
{
	/*
	 * Check that the margin value is within it's range;
	 * if not reset to the default
	 */
	if (cns3xxx_wdt_set_heartbeat(cns3xxx_margin)) {
		cns3xxx_wdt_set_heartbeat(TIMER_MARGIN);
		printk(KERN_INFO "cns3xxx_margin value must be 0"
			"< cns3xxx_margin < 65536, using %d\n",
			TIMER_MARGIN);
	}

	printk(banner, cns3xxx_noboot, cns3xxx_margin, nowayout);

	spin_lock_init(&wdt_lock);

	return platform_driver_register(&cns3xxx_wdt_driver);
}

static void __exit cns3xxx_wdt_exit(void)
{
	platform_driver_unregister(&cns3xxx_wdt_driver);
}

module_init(cns3xxx_wdt_init);
module_exit(cns3xxx_wdt_exit);

MODULE_AUTHOR("Scott Shu");
MODULE_DESCRIPTION("CNS3XXX Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:cns3xxx-wdt");
