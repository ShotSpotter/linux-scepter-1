/*
 * Freescale LBC and UPM routines.
 *
 * Copyright (c) 2007-2008  MontaVista Software, Inc.
 * Copyright (c) 2010 Freescale Semiconductor
 *
 * Author: Anton Vorontsov <avorontsov@ru.mvista.com>
 * Author: Jack Lan <Jack.Lan@freescale.com>
 * Author: Roy Zang <tie-fei.zang@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <asm/prom.h>
#include <asm/fsl_lbc.h>

static spinlock_t fsl_lbc_lock = __SPIN_LOCK_UNLOCKED(fsl_lbc_lock);
struct fsl_lbc_ctrl *fsl_lbc_ctrl_dev;
EXPORT_SYMBOL(fsl_lbc_ctrl_dev);

/**
 * fsl_lbc_addr - convert the base address
 * @addr_base:	base address of the memory bank
 *
 * This function converts a base address of lbc into the right format for the
 * BR register. If the SOC has eLBC then it returns 32bit physical address
 * else it convers a 34bit local bus physical address to correct format of
 * 32bit address for BR register (Example: MPC8641).
 */
u32 fsl_lbc_addr(phys_addr_t addr_base)
{
	void *dev;
	u32 addr = addr_base & 0xffff8000;

	dev = of_find_node_by_name(NULL, "localbus");
	if (!dev) {
		printk(KERN_INFO "fsl-lbc: can't find localbus node\n");
		of_node_put(dev);
		return 0;
	}

	if (of_device_is_compatible(dev, "fsl,elbc") ||
			of_device_is_compatible(dev, "fsl,p3041-rev1.0-elbc")) {
		of_node_put(dev);
		return addr;
	} else {
		of_node_put(dev);
		return addr | ((addr_base & 0x300000000ull) >> 19);
	}
}
EXPORT_SYMBOL(fsl_lbc_addr);

/**
 * fsl_lbc_find - find Localbus bank
 * @addr_base:	base address of the memory bank
 *
 * This function walks LBC banks comparing "Base address" field of the BR
 * registers with the supplied addr_base argument. When bases match this
 * function returns bank number (starting with 0), otherwise it returns
 * appropriate errno value.
 */
int fsl_lbc_find(phys_addr_t addr_base)
{
	int i;
	struct fsl_lbc_regs __iomem *lbc;

	if (!fsl_lbc_ctrl_dev || !fsl_lbc_ctrl_dev->regs)
		return -ENODEV;

	lbc = fsl_lbc_ctrl_dev->regs;
	for (i = 0; i < ARRAY_SIZE(lbc->bank); i++) {
		__be32 br = in_be32(&lbc->bank[i].br);
		__be32 or = in_be32(&lbc->bank[i].or);

		if (br & BR_V && (br & or & BR_BA) == fsl_lbc_addr(addr_base))
			return i;
	}

	return -ENOENT;
}
EXPORT_SYMBOL(fsl_lbc_find);

/**
 * fsl_upm_find - find pre-programmed UPM via base address
 * @addr_base:	base address of the memory bank controlled by the UPM
 * @upm:	pointer to the allocated fsl_upm structure
 *
 * This function fills fsl_upm structure so you can use it with the rest of
 * UPM API. On success this function returns 0, otherwise it returns
 * appropriate errno value.
 */
int fsl_upm_find(phys_addr_t addr_base, struct fsl_upm *upm)
{
	int bank;
	__be32 br;
	struct fsl_lbc_regs __iomem *lbc;

	bank = fsl_lbc_find(addr_base);
	if (bank < 0)
		return bank;

	if (!fsl_lbc_ctrl_dev || !fsl_lbc_ctrl_dev->regs)
		return -ENODEV;

	lbc = fsl_lbc_ctrl_dev->regs;
	br = in_be32(&lbc->bank[bank].br);

	switch (br & BR_MSEL) {
	case BR_MS_UPMA:
		upm->mxmr = &lbc->mamr;
		break;
	case BR_MS_UPMB:
		upm->mxmr = &lbc->mbmr;
		break;
	case BR_MS_UPMC:
		upm->mxmr = &lbc->mcmr;
		break;
	default:
		return -EINVAL;
	}

	switch (br & BR_PS) {
	case BR_PS_8:
		upm->width = 8;
		break;
	case BR_PS_16:
		upm->width = 16;
		break;
	case BR_PS_32:
		upm->width = 32;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(fsl_upm_find);

/**
 * fsl_upm_run_pattern - actually run an UPM pattern
 * @upm:	pointer to the fsl_upm structure obtained via fsl_upm_find
 * @io_base:	remapped pointer to where memory access should happen
 * @mar:	MAR register content during pattern execution
 *
 * This function triggers dummy write to the memory specified by the io_base,
 * thus UPM pattern actually executed. Note that mar usage depends on the
 * pre-programmed AMX bits in the UPM RAM.
 */
int fsl_upm_run_pattern(struct fsl_upm *upm, void __iomem *io_base, u32 mar)
{
	int ret = 0;
	unsigned long flags;

	if (!fsl_lbc_ctrl_dev || !fsl_lbc_ctrl_dev->regs)
		return -ENODEV;

	spin_lock_irqsave(&fsl_lbc_lock, flags);

	out_be32(&fsl_lbc_ctrl_dev->regs->mar, mar);

	switch (upm->width) {
	case 8:
		out_8(io_base, 0x0);
		break;
	case 16:
		out_be16(io_base, 0x0);
		break;
	case 32:
		out_be32(io_base, 0x0);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&fsl_lbc_lock, flags);

	return ret;
}
EXPORT_SYMBOL(fsl_upm_run_pattern);

static int __devinit fsl_lbc_ctrl_init(struct fsl_lbc_ctrl *ctrl,
				       struct device_node *node)
{
	struct fsl_lbc_regs __iomem *lbc = ctrl->regs;

	/*
	 * NAND transactions can tie up the bus for a long time, so set the
	 * bus timeout to max by clearing LBCR[BMT] (highest base counter
	 * value) and setting LBCR[BMTPS] to the highest prescaler value.
	 */
	clrsetbits_be32(&lbc->lbcr, LBCR_BMT, 15);

	/* clear event registers */
	setbits32(&lbc->ltesr, LTESR_CLEAR);
	out_be32(&lbc->lteatr, 0);
	out_be32(&lbc->ltear, 0);
	out_be32(&lbc->lteccr, LTECCR_CLEAR);
	out_be32(&lbc->ltedr, LTEDR_ENABLE);

	/* Set the monitor timeout value to the maximum for erratum A001 */
	if (of_device_is_compatible(node, "fsl,elbc") ||
			of_device_is_compatible(node, "fsl,p3041-rev1.0-elbc"))
		clrsetbits_be32(&lbc->lbcr, LBCR_BMT, LBCR_BMTPS);


	return 0;
}

/*
 * NOTE: This interrupt is used to report localbus events of various kinds,
 * such as transaction errors on the chipselects.
 */

static irqreturn_t fsl_lbc_ctrl_irq(int irqno, void *data)
{
	struct fsl_lbc_ctrl *ctrl = data;
	struct fsl_lbc_regs __iomem *lbc = ctrl->regs;
	u32 status;
	unsigned long flags;
 
	spin_lock_irqsave(&fsl_lbc_lock, flags);

	status = in_be32(&lbc->ltesr);

	if (status) {
		out_be32(&lbc->ltesr, LTESR_CLEAR);
		out_be32(&lbc->lteatr, 0);
		out_be32(&lbc->ltear, 0);
		ctrl->irq_status = status;

		if (status & LTESR_BM)
			dev_err(ctrl->dev, "Local bus monitor time-out: "
				"LTESR 0x%08X\n", status);
		if (status & LTESR_WP)
			dev_err(ctrl->dev, "Write protect error: "
				"LTESR 0x%08X\n", status);
		if (status & LTESR_ATMW)
			dev_err(ctrl->dev, "Atomic write error: "
				"LTESR 0x%08X\n", status);
		if (status & LTESR_ATMR)
			dev_err(ctrl->dev, "Atomic read error: "
				"LTESR 0x%08X\n", status);
		if (status & LTESR_CS)
			dev_err(ctrl->dev, "Chip select error: "
				"LTESR 0x%08X\n", status);
		if (status & LTESR_UPM)
				;
		if (status & LTESR_FCT) {
			dev_err(ctrl->dev, "FCM command time-out: "
				"LTESR 0x%08X\n", status);
			smp_wmb();
			wake_up(&ctrl->irq_wait);
		}
		if (status & LTESR_PAR) {
			dev_err(ctrl->dev, "Parity or Uncorrectable ECC error: "
			"LTESR 0x%08X\n", status);
			smp_wmb();
			wake_up(&ctrl->irq_wait);
		}
		if (status & LTESR_CC) {
			smp_wmb();
			wake_up(&ctrl->irq_wait);
		}
		if (status & ~LTESR_MASK)
			dev_err(ctrl->dev, "Unknown error: "
				"LTESR 0x%08X\n", status);
		spin_unlock_irqrestore(&fsl_lbc_lock, flags);
		return IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&fsl_lbc_lock, flags);
	return IRQ_NONE;
}

/* fsl_lbc_ctrl_probe
 *
 * called by device layer when it finds a device matching
 * one our driver can handled. This code allocates all of
 * the resources needed for the controller only.  The
 * resources for the NAND banks themselves are allocated
 * in the chip probe function.
*/

static int __devinit fsl_lbc_ctrl_probe(struct of_device *ofdev,
		const struct of_device_id *match)
{
	int ret;

	if (!ofdev->node) {
		dev_err(&ofdev->dev, "Device OF-Node is NULL");
		return -EFAULT;
	}

	fsl_lbc_ctrl_dev = kzalloc(sizeof(*fsl_lbc_ctrl_dev), GFP_KERNEL);
	if (!fsl_lbc_ctrl_dev)
		return -ENOMEM;

	dev_set_drvdata(&ofdev->dev, fsl_lbc_ctrl_dev);

	spin_lock_init(&fsl_lbc_ctrl_dev->lock);
	init_waitqueue_head(&fsl_lbc_ctrl_dev->irq_wait);

	fsl_lbc_ctrl_dev->regs = of_iomap(ofdev->node, 0);
	if (!fsl_lbc_ctrl_dev->regs) {
		dev_err(&ofdev->dev, "failed to get memory region\n");
		ret = -ENODEV;
		goto err;
	}

	fsl_lbc_ctrl_dev->irq[0] = of_irq_to_resource(ofdev->node, 0, NULL);
	if (fsl_lbc_ctrl_dev->irq[0] == NO_IRQ) {
		dev_err(&ofdev->dev, "failed to get irq resource\n");
		ret = -ENODEV;
		goto err;
	}

	fsl_lbc_ctrl_dev->dev = &ofdev->dev;

	ret = fsl_lbc_ctrl_init(fsl_lbc_ctrl_dev, ofdev->node);
	if (ret < 0)
		goto err;

	ret = request_irq(fsl_lbc_ctrl_dev->irq[0], fsl_lbc_ctrl_irq, 0,
				"fsl-lbc", fsl_lbc_ctrl_dev);
	if (ret != 0) {
		dev_err(&ofdev->dev, "failed to install irq (%d)\n",
			fsl_lbc_ctrl_dev->irq[0]);
		ret = fsl_lbc_ctrl_dev->irq[0];
		goto err;
	}

	fsl_lbc_ctrl_dev->irq[1] = of_irq_to_resource(ofdev->node, 1, NULL);
	if (fsl_lbc_ctrl_dev->irq[1] != NO_IRQ) {
		ret = request_irq(fsl_lbc_ctrl_dev->irq[1], fsl_lbc_ctrl_irq,
				IRQF_SHARED, "fsl-lbc-err", fsl_lbc_ctrl_dev);
		if (ret != 0) {
			dev_err(&ofdev->dev, "failed to install irq (%d)\n",
					fsl_lbc_ctrl_dev->irq[1]);
			ret = fsl_lbc_ctrl_dev->irq[1];
			goto err;
		}
	}

	/* Enable interrupts for any detected events */
	out_be32(&fsl_lbc_ctrl_dev->regs->lteir, LTEIR_ENABLE);

	return 0;

err:
	if (fsl_lbc_ctrl_dev->regs)
		iounmap(fsl_lbc_ctrl_dev->regs);
	kfree(fsl_lbc_ctrl_dev);
	return ret;
}

static int __devexit fsl_lbc_ctrl_remove(struct of_device *ofdev)
{
	struct fsl_lbc_ctrl *ctrl = dev_get_drvdata(&ofdev->dev);

	if (ctrl->irq[0])
		free_irq(ctrl->irq[0], ctrl);

	if (ctrl->irq[1])
		free_irq(ctrl->irq[1], ctrl);

	if (ctrl->regs)
		iounmap(ctrl->regs);

	dev_set_drvdata(&ofdev->dev, NULL);
	kfree(ctrl);
	return 0;
}

#ifdef CONFIG_SUSPEND
#ifdef CONFIG_P1022_DS
#define COUNT_OF_BANK_P1022	(8)
#define COUNT_OF_BANKS COUNT_OF_BANK_P1022
#else
#define COUNT_OF_BANKS (8)
#endif


/* save lbc registers */
static int fsl_lbc_suspend(struct of_device *ofdev, pm_message_t state)
{
	struct fsl_lbc_ctrl *ctrl = dev_get_drvdata(&ofdev->dev);
	struct fsl_lbc_regs __iomem *lbc = ctrl->regs;
	struct fsl_lbc_regs *saved_lbc = &ctrl->saved_regs;
	int i;

	for (i = 0; i < COUNT_OF_BANKS; i++) {
		saved_lbc->bank[i].br =
			in_be32(&lbc->bank[i].br);
		saved_lbc->bank[i].or =
			in_be32(&lbc->bank[i].or);
	}
	saved_lbc->mar = in_be32(&lbc->mar);
	saved_lbc->mamr = in_be32(&lbc->mamr);
	saved_lbc->mbmr = in_be32(&lbc->mbmr);
	saved_lbc->mcmr = in_be32(&lbc->mcmr);
	saved_lbc->mrtpr = in_be32(&lbc->mrtpr);
	saved_lbc->mdr = in_be32(&lbc->mdr);
	saved_lbc->lsor = in_be32(&lbc->lsor);
	saved_lbc->lsdmr = in_be32(&lbc->lsdmr);
	saved_lbc->lurt = in_be32(&lbc->lurt);
	saved_lbc->lsrt = in_be32(&lbc->lsrt);
	saved_lbc->ltedr = in_be32(&lbc->ltedr);
	saved_lbc->lteir = in_be32(&lbc->lteir);
	saved_lbc->lteatr = in_be32(&lbc->lteatr);
	saved_lbc->ltear = in_be32(&lbc->ltear);
	saved_lbc->lbcr = in_be32(&lbc->lbcr);
	saved_lbc->lcrr = in_be32(&lbc->lcrr);
	saved_lbc->fmr = in_be32(&lbc->fmr);
	saved_lbc->fir = in_be32(&lbc->fir);
	saved_lbc->fcr = in_be32(&lbc->fcr);
	saved_lbc->fbar = in_be32(&lbc->fbar);
	saved_lbc->fpar = in_be32(&lbc->fpar);
	saved_lbc->fbcr = in_be32(&lbc->fbcr);

	return 0;
}

/* restore lbc registers */
static int fsl_lbc_resume(struct of_device *ofdev)
{
	struct fsl_lbc_ctrl *ctrl = dev_get_drvdata(&ofdev->dev);
	struct fsl_lbc_regs __iomem *lbc = ctrl->regs;
	struct fsl_lbc_regs *saved_lbc = &ctrl->saved_regs;
	int i;

	for (i = 0; i < COUNT_OF_BANKS; i++) {
		out_be32(&lbc->bank[i].br,
				saved_lbc->bank[i].br);
		out_be32(&lbc->bank[i].or,
				saved_lbc->bank[i].or);
	}

	out_be32(&lbc->mar, saved_lbc->mar);
	out_be32(&lbc->mamr, saved_lbc->mamr);
	out_be32(&lbc->mbmr, saved_lbc->mbmr);
	out_be32(&lbc->mcmr, saved_lbc->mcmr);
	out_be32(&lbc->mrtpr, saved_lbc->mrtpr);
	out_be32(&lbc->mdr, saved_lbc->mdr);
	out_be32(&lbc->lsor, saved_lbc->lsor);
	out_be32(&lbc->lsdmr, saved_lbc->lsdmr);
	out_be32(&lbc->lurt, saved_lbc->lurt);
	out_be32(&lbc->lsrt, saved_lbc->lsrt);
	out_be32(&lbc->ltedr, saved_lbc->ltedr);
	out_be32(&lbc->lteir, saved_lbc->lteir);
	out_be32(&lbc->lteatr, saved_lbc->lteatr);
	out_be32(&lbc->ltear, saved_lbc->ltear);
	out_be32(&lbc->lbcr, saved_lbc->lbcr);
	out_be32(&lbc->lcrr, saved_lbc->lcrr);
	out_be32(&lbc->fmr, saved_lbc->fmr);
	out_be32(&lbc->fir, saved_lbc->fir);
	out_be32(&lbc->fcr, saved_lbc->fcr);
	out_be32(&lbc->fbar, saved_lbc->fbar);
	out_be32(&lbc->fpar, saved_lbc->fpar);
	out_be32(&lbc->fbcr, saved_lbc->fbcr);

	return 0;
}
#endif /* CONFIG_SUSPEND */

static const struct of_device_id fsl_lbc_match[] = {
	{ .compatible = "fsl,elbc", },
	{ .compatible = "fsl,p3041-rev1.0-elbc", },
	{ .compatible = "fsl,pq3-localbus", },
	{ .compatible = "fsl,pq2-localbus", },
	{ .compatible = "fsl,pq2pro-localbus", },
	{},
};

static struct of_platform_driver fsl_lbc_ctrl_driver = {
	.driver = {
		.name	= "fsl-elbc",
	},
	.match_table = fsl_lbc_match,
	.probe = fsl_lbc_ctrl_probe,
	.remove = __devexit_p(fsl_lbc_ctrl_remove),
#ifdef CONFIG_SUSPEND
	.suspend     = fsl_lbc_suspend,
	.resume      = fsl_lbc_resume,
#endif
};

static int __init fsl_lbc_init(void)
{
	return of_register_platform_driver(&fsl_lbc_ctrl_driver);
}

static void __exit fsl_lbc_exit(void)
{
	of_unregister_platform_driver(&fsl_lbc_ctrl_driver);
}
module_init(fsl_lbc_init);
module_exit(fsl_lbc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale Enhanced Local Bus Controller driver");
