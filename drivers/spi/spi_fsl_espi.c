/*
 * Freescale eSPI controller driver.
 *
 * Copyright 2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_spi.h>
#include <sysdev/fsl_soc.h>
#include <linux/interrupt.h>

#include "spi_fsl_lib.h"

/* eSPI Controller mode register definitions */
#define SPMODE_ENABLE		(1 << 31)
#define SPMODE_LOOP		(1 << 30)
#define SPMODE_TXTHR(x)		((x) << 8)
#define SPMODE_RXTHR(x)		((x) << 0)

/* eSPI Controller CS mode register definitions */
#define CSMODE_CI_INACTIVEHIGH	(1 << 31)
#define CSMODE_CP_BEGIN_EDGECLK	(1 << 30)
#define CSMODE_REV		(1 << 29)
#define CSMODE_DIV16		(1 << 28)
#define CSMODE_PM(x)		((x) << 24)
#define CSMODE_POL_1		(1 << 20)
#define CSMODE_LEN(x)		((x) << 16)
#define CSMODE_BEF(x)		((x) << 12)
#define CSMODE_AFT(x)		((x) << 8)
#define CSMODE_CG(x)		((x) << 3)

/* Default mode/csmode for eSPI controller */
#define SPMODE_INIT_VAL (SPMODE_TXTHR(4) | SPMODE_RXTHR(3))
#define CSMODE_INIT_VAL (CSMODE_POL_1 | CSMODE_BEF(0) \
		| CSMODE_AFT(0) | CSMODE_CG(1))

/* SPIE register values */
#define	SPIE_NE		0x00000200	/* Not empty */
#define	SPIE_NF		0x00000100	/* Not full */

/* SPIM register values */
#define	SPIM_NE		0x00000200	/* Not empty */
#define	SPIM_NF		0x00000100	/* Not full */
#define SPIE_RXCNT(reg)     ((reg >> 24) & 0x3F)
#define SPIE_TXCNT(reg)     ((reg >> 16) & 0x3F)

/* SPCOM register values */
#define SPCOM_CS(x)		((x) << 30)
#define SPCOM_TRANLEN(x)	((x) << 0)
#define	SPCOM_TRANLEN_MAX	0xFFFF	/* Max transaction length */

static void fsl_espi_change_mode(struct spi_device *spi)
{
	struct mpc8xxx_spi *mspi = spi_master_get_devdata(spi->master);
	struct spi_mpc8xxx_cs *cs = spi->controller_state;
	__be32 __iomem *mode;
	__be32 __iomem *espi_mode = NULL;
	u32 tmp;
	unsigned long flags;

	espi_mode = &mspi->espi_base->mode;
	mode = &mspi->espi_base->csmode[spi->chip_select];

	/* Turn off IRQs locally to minimize time that SPI is disabled. */
	local_irq_save(flags);

	/* Turn off SPI unit prior changing mode */
	tmp = mpc8xxx_spi_read_reg(espi_mode);
	mpc8xxx_spi_write_reg(espi_mode, tmp & ~SPMODE_ENABLE);
	mpc8xxx_spi_write_reg(mode, cs->hw_mode);
	mpc8xxx_spi_write_reg(espi_mode, tmp);

	local_irq_restore(flags);
}

static u32 fsl_espi_tx_buf_lsb(struct mpc8xxx_spi *mpc8xxx_spi)
{
	u32 data;
	u16 data_h, data_l;

	const u32 *tx = mpc8xxx_spi->tx;
	if (!tx)
		return 0;

	data = *tx++ << mpc8xxx_spi->tx_shift;
	data_l = data & 0xffff;
	data_h = (data >> 16) & 0xffff;
	swab16s(&data_l);
	swab16s(&data_h);
	data = data_h | data_l;

	mpc8xxx_spi->tx = tx;
	return data;
}

static int fsl_espi_setup_transfer(struct spi_device *spi,
					struct spi_transfer *t)
{
	struct mpc8xxx_spi *mpc8xxx_spi;
	int bits_per_word = 0;
	u8 pm;
	u32 hz = 0;
	struct spi_mpc8xxx_cs	*cs = spi->controller_state;

	mpc8xxx_spi = spi_master_get_devdata(spi->master);

	if (t) {
		bits_per_word = t->bits_per_word;
		hz = t->speed_hz;
	}

	/* spi_transfer level calls that work per-word */
	if (!bits_per_word)
		bits_per_word = spi->bits_per_word;

	/* Make sure its a bit width we support [4..16] */
	if ((bits_per_word < 4) || (bits_per_word > 16))
		return -EINVAL;

	if (!hz)
		hz = spi->max_speed_hz;

	cs->rx_shift = 0;
	cs->tx_shift = 0;
	cs->get_rx = mpc8xxx_spi_rx_buf_u32;
	cs->get_tx = mpc8xxx_spi_tx_buf_u32;
	if (bits_per_word <= 8) {
		cs->rx_shift = 8 - bits_per_word;
	} else if (bits_per_word <= 16) {
		cs->rx_shift = 16 - bits_per_word;
		if (spi->mode & SPI_LSB_FIRST)
			cs->get_tx = fsl_espi_tx_buf_lsb;
	} else
		return -EINVAL;

	mpc8xxx_spi->rx_shift = cs->rx_shift;
	mpc8xxx_spi->tx_shift = cs->tx_shift;
	mpc8xxx_spi->get_rx = cs->get_rx;
	mpc8xxx_spi->get_tx = cs->get_tx;

	bits_per_word = bits_per_word - 1;

	/* mask out bits we are going to set */
	cs->hw_mode &= ~(CSMODE_LEN(0xF) | CSMODE_DIV16
				  | CSMODE_PM(0xF));

	cs->hw_mode |= CSMODE_LEN(bits_per_word);

	if ((mpc8xxx_spi->spibrg / hz) > 64) {
		cs->hw_mode |= CSMODE_DIV16;
		pm = (mpc8xxx_spi->spibrg - 1) / (hz * 64) + 1;

		WARN_ONCE(pm > 16, "%s: Requested speed is too low: %d Hz. "
			  "Will use %d Hz instead.\n", dev_name(&spi->dev),
			  hz, mpc8xxx_spi->spibrg / 1024);
		if (pm > 16)
			pm = 16;
	} else {
		pm = (mpc8xxx_spi->spibrg - 1) / (hz * 4) + 1;
	}
	if (pm)
		pm--;

	cs->hw_mode |= CSMODE_PM(pm);

	fsl_espi_change_mode(spi);
	return 0;
}

int fsl_espi_cpu_bufs(struct mpc8xxx_spi *mspi, struct spi_transfer *t,
		unsigned int len)
{
	u32 word;

	mspi->count = len;

	/* enable rx ints */
	mpc8xxx_spi_write_reg(&mspi->espi_base->mask, SPIM_NE);

	/* transmit word */
	word = mspi->get_tx(mspi);
	mpc8xxx_spi_write_reg(&mspi->espi_base->transmit, word);

	return 0;
}

static int fsl_espi_bufs(struct spi_device *spi, struct spi_transfer *t,
			    bool is_dma_mapped)
{
	struct mpc8xxx_spi *mpc8xxx_spi = spi_master_get_devdata(spi->master);
	unsigned int len = t->len;
	u8 bits_per_word;
	int ret;

	bits_per_word = spi->bits_per_word;
	if (t->bits_per_word)
		bits_per_word = t->bits_per_word;

	mpc8xxx_spi->len = t->len;
	len = roundup(len, 4) / 4;

	mpc8xxx_spi->tx = t->tx_buf;
	mpc8xxx_spi->rx = t->rx_buf;

	INIT_COMPLETION(mpc8xxx_spi->done);

	/* Set SPCOM[CS] and SPCOM[TRANLEN] field */
	if ((t->len - 1) > SPCOM_TRANLEN_MAX) {
		dev_err(mpc8xxx_spi->dev, "Transaction length (%d)"
				" beyond the SPCOM[TRANLEN] field\n", t->len);
		return -EINVAL;
	}
	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->command,
		(SPCOM_CS(spi->chip_select) | SPCOM_TRANLEN(t->len - 1)));

	ret = fsl_espi_cpu_bufs(mpc8xxx_spi, t, len);
	if (ret)
		return ret;

	wait_for_completion(&mpc8xxx_spi->done);

	/* disable rx ints */
	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->mask, 0);

	return mpc8xxx_spi->count;
}

static void fsl_espi_do_one_msg(struct spi_message *m)
{
	struct spi_device *spi = m->spi;
	struct mpc8xxx_spi *mspi = spi_master_get_devdata(spi->master);
	struct spi_message message;
	struct spi_transfer *t, *first, trans;
	u8 *local_buf, *rx_buf = NULL;
	unsigned int n_tx = 0;
	unsigned int n_rx = 0;
	int status = 0;
	int i = 0;

	spi_message_init(&message);
	memset(&trans, 0, sizeof(trans));

	first = list_first_entry(&m->transfers, struct spi_transfer,
			transfer_list);
	list_for_each_entry(t, &m->transfers, transfer_list) {
		if ((first->bits_per_word != t->bits_per_word) ||
			(first->speed_hz != t->speed_hz)) {
			status = -EINVAL;
			dev_err(mspi->dev, "bits_per_word/speed_hz should be"
					" same for the same SPI transfer\n");
			return;
		}

		trans.speed_hz = t->speed_hz;
		trans.bits_per_word = t->bits_per_word;
		trans.delay_usecs = max(first->delay_usecs, t->delay_usecs);

		if (t->tx_buf)
			n_tx += t->len;

		if (t->rx_buf) {
			n_rx += t->len;
			rx_buf = t->rx_buf;
		}
	}

	local_buf = kzalloc(n_tx * 2 + roundup(n_rx + n_tx, 4), GFP_KERNEL);
	if (!local_buf) {
		status = -ENOMEM;
		return;
	}

	list_for_each_entry(t, &m->transfers, transfer_list) {
		if (t->tx_buf) {
			memcpy(local_buf + i, t->tx_buf, t->len);
			i += t->len;
		}
	}

	trans.len = n_tx + n_rx;
	trans.tx_buf = local_buf;
	trans.rx_buf = local_buf + n_tx;
	spi_message_add_tail(&trans, &message);

	list_for_each_entry(t, &message.transfers, transfer_list) {
		if (t->bits_per_word || t->speed_hz) {
			status = -EINVAL;

			status = fsl_espi_setup_transfer(spi, t);
			if (status < 0)
				break;
		}

		if (t->len)
			status = fsl_espi_bufs(spi, t, 0);
		if (status) {
			status = -EMSGSIZE;
			break;
		}
		m->actual_length += t->len;

		if (rx_buf)
			memcpy(rx_buf, t->rx_buf + n_tx, n_rx);

		if (t->delay_usecs)
			udelay(t->delay_usecs);
	}

	m->status = status;
	m->complete(m->context);

	fsl_espi_setup_transfer(spi, NULL);
	kfree(local_buf);
}

static int fsl_espi_setup(struct spi_device *spi)
{
	struct mpc8xxx_spi *mpc8xxx_spi;
	int retval;
	u32 hw_mode;
	u32 loop_mode;
	struct spi_mpc8xxx_cs	*cs = spi->controller_state;

	if (!spi->max_speed_hz)
		return -EINVAL;

	if (!cs) {
		cs = kzalloc(sizeof *cs, GFP_KERNEL);
		if (!cs)
			return -ENOMEM;
		spi->controller_state = cs;
	}

	mpc8xxx_spi = spi_master_get_devdata(spi->master);

	hw_mode = cs->hw_mode; /* Save orginal settings */
	cs->hw_mode = mpc8xxx_spi_read_reg(
			&mpc8xxx_spi->espi_base->csmode[spi->chip_select]);
	/* mask out bits we are going to set */
	cs->hw_mode &= ~(CSMODE_CP_BEGIN_EDGECLK | CSMODE_CI_INACTIVEHIGH
			 | CSMODE_REV);

	if (spi->mode & SPI_CPHA)
		cs->hw_mode |= CSMODE_CP_BEGIN_EDGECLK;
	if (spi->mode & SPI_CPOL)
		cs->hw_mode |= CSMODE_CI_INACTIVEHIGH;
	if (!(spi->mode & SPI_LSB_FIRST))
		cs->hw_mode |= CSMODE_REV;

	/* Handle the loop mode */
	loop_mode = mpc8xxx_spi_read_reg(&mpc8xxx_spi->espi_base->mode);
	loop_mode &= ~SPMODE_LOOP;
	if (spi->mode & SPI_LOOP)
		loop_mode |= SPMODE_LOOP;
	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->mode, loop_mode);

	retval = fsl_espi_setup_transfer(spi, NULL);
	if (retval < 0) {
		cs->hw_mode = hw_mode; /* Restore settings */
		return retval;
	}
	return 0;
}

static void fsl_espi_cpu_irq(struct mpc8xxx_spi *mspi, u32 events)
{
	/* We need handle RX first */
	if (events & SPIE_NE) {
		u32 rx_data, tmp;
		u8 rx_data_8;

		/* Spin until RX is done */
		while (SPIE_RXCNT(events) < min(4, mspi->len)) {
			cpu_relax();
			events = mpc8xxx_spi_read_reg(&mspi->espi_base->event);
		}

		if (mspi->len >= 4) {
			rx_data = mpc8xxx_spi_read_reg(
					&mspi->espi_base->receive);
		} else {
			tmp = mspi->len;
			rx_data = 0;
			while (tmp--) {
				rx_data_8 = in_8((u8 *)
						&mspi->espi_base->receive);
				rx_data |= (rx_data_8 << (tmp * 8));
			}

			rx_data <<= (4 - mspi->len) * 8;
		}

		mspi->len -= 4;

		if (mspi->rx)
			mspi->get_rx(rx_data, mspi);
	}

	if ((events & SPIE_NF) == 0)
		/* spin until TX is done */
		while (((events =
			mpc8xxx_spi_read_reg(&mspi->espi_base->event)) &
						SPIE_NF) == 0)
			cpu_relax();

	/* Clear the events */
	mpc8xxx_spi_write_reg(&mspi->espi_base->event, events);

	mspi->count -= 1;
	if (mspi->count) {
		u32 word = mspi->get_tx(mspi);

		mpc8xxx_spi_write_reg(&mspi->espi_base->transmit, word);
	} else {
		complete(&mspi->done);
	}
}

irqreturn_t fsl_espi_irq(s32 irq, void *context_data)
{
	struct mpc8xxx_spi *mspi = context_data;
	irqreturn_t ret = IRQ_NONE;
	u32 events;

	/* Get interrupt events(tx/rx) */
	events = mpc8xxx_spi_read_reg(&mspi->espi_base->event);
	if (events)
		ret = IRQ_HANDLED;

	dev_dbg(mspi->dev, "%s: events %x\n", __func__, events);

	fsl_espi_cpu_irq(mspi, events);

	return ret;
}

static void fsl_espi_remove(struct mpc8xxx_spi *mspi)
{
	iounmap(mspi->espi_base);
}

static struct spi_master * __devinit fsl_espi_probe(struct device *dev,
		struct resource *mem, unsigned int irq)
{
	struct fsl_spi_platform_data *pdata = dev->platform_data;
	struct spi_master *master;
	struct mpc8xxx_spi *mpc8xxx_spi;
	u32 regval;
	int i, ret = 0;

	master = spi_alloc_master(dev, sizeof(struct mpc8xxx_spi));
	if (master == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	dev_set_drvdata(dev, master);

	ret = mpc8xxx_spi_probe(dev, mem, irq);
	if (ret)
		goto err_probe;

	master->setup = fsl_espi_setup;
	master->flags = SPI_MASTER_TRANS_LIMIT;

	mpc8xxx_spi = spi_master_get_devdata(master);
	mpc8xxx_spi->spi_do_one_msg = fsl_espi_do_one_msg;
	mpc8xxx_spi->spi_remove = fsl_espi_remove;

	mpc8xxx_spi->espi_base = ioremap(mem->start, resource_size(mem));
	if (mpc8xxx_spi->espi_base == NULL) {
		ret = -ENOMEM;
		goto err_probe;
	}

	/* Register for SPI Interrupt */
	ret = request_irq(mpc8xxx_spi->irq, fsl_espi_irq,
			  0, "fsl_espi", mpc8xxx_spi);

	if (ret != 0)
		goto err_free_irq;

	if (mpc8xxx_spi->flags & SPI_QE_CPU_MODE) {
		mpc8xxx_spi->rx_shift = 16;
		mpc8xxx_spi->tx_shift = 24;
	}

	/* SPI controller initializations */
	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->mode, 0);
	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->mask, 0);
	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->command, 0);
	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->event, 0xffffffff);

	/* Init eSPI CS mode register */
	for (i = 0; i < pdata->max_chipselect; i++)
		mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->csmode[i],
				CSMODE_INIT_VAL);

	/* Enable SPI interface */
	regval = pdata->initial_spmode | SPMODE_INIT_VAL | SPMODE_ENABLE;

	mpc8xxx_spi_write_reg(&mpc8xxx_spi->espi_base->mode, regval);

	ret = spi_register_master(master);
	if (ret < 0)
		goto unreg_master;

	dev_info(dev, "at 0x%p (irq = %d)\n", mpc8xxx_spi->espi_base,
			mpc8xxx_spi->irq);

	return master;

unreg_master:
	free_irq(mpc8xxx_spi->irq, mpc8xxx_spi);
err_free_irq:
	iounmap(mpc8xxx_spi->espi_base);
err_probe:
	spi_master_put(master);
err:
	return ERR_PTR(ret);
}

static int of_fsl_espi_get_chipselects(struct device *dev)
{
	struct device_node *np = dev_archdata_get_node(&dev->archdata);
	struct fsl_spi_platform_data *pdata = dev->platform_data;
	const u32 *prop;
	int len;

	prop = of_get_property(np, "fsl,espi-num-chipselects", &len);
	if (!prop || len < sizeof(*prop)) {
		dev_err(dev, "No 'fsl,espi-num-chipselects' property\n");
		return -EINVAL;
	}

	pdata->max_chipselect = *prop;
	pdata->cs_control = NULL;

	return 0;
}

static int __devinit of_fsl_espi_probe(struct of_device *ofdev,
					  const struct of_device_id *ofid)
{
	struct device *dev = &ofdev->dev;
	struct device_node *np = ofdev->node;
	struct spi_master *master;
	struct resource mem;
	struct resource irq;
	int ret = -ENOMEM;

	ret = of_mpc8xxx_spi_probe(ofdev, ofid);
	if (ret)
		return ret;

	ret = of_fsl_espi_get_chipselects(dev);
	if (ret)
		goto err;

	ret = of_address_to_resource(np, 0, &mem);
	if (ret)
		goto err;

	ret = of_irq_to_resource(np, 0, &irq);
	if (!ret) {
		ret = -EINVAL;
		goto err;
	}

	master = fsl_espi_probe(dev, &mem, irq.start);
	if (IS_ERR(master)) {
		ret = PTR_ERR(master);
		goto err;
	}

	of_register_spi_devices(master, np);

	return 0;

err:
	return ret;
}

static int __devexit of_fsl_espi_remove(struct of_device *ofdev)
{
	int ret;

	ret = mpc8xxx_spi_remove(&ofdev->dev);
	if (ret)
		return ret;

	return 0;
}

#ifdef CONFIG_SUSPEND
/* save espi registers */
static int fsl_espi_suspend(struct of_device *ofdev, pm_message_t state)
{
	struct mpc8xxx_spi *fsl_espi;
	struct spi_master *master;
	int i;
	master = platform_get_drvdata(ofdev);
	fsl_espi = spi_master_get_devdata(master);

	fsl_espi->espi_save_regs.mode = in_be32(&fsl_espi->espi_base->mode);
	fsl_espi->espi_save_regs.event = in_be32(&fsl_espi->espi_base->event);
	fsl_espi->espi_save_regs.mask = in_be32(&fsl_espi->espi_base->mask);
	for (i = 0; i < 4; i++)
		fsl_espi->espi_save_regs.csmode[i] =
			in_be32(&fsl_espi->espi_base->csmode[i]);

	return 0;
}

/* restore espi registers */
static int fsl_espi_resume(struct of_device *ofdev)
{
	struct mpc8xxx_spi *fsl_espi;
	struct spi_master *master;
	int i;
	master = platform_get_drvdata(ofdev);
	fsl_espi = spi_master_get_devdata(master);

	out_be32(&fsl_espi->espi_base->mode, fsl_espi->espi_save_regs.mode);
	out_be32(&fsl_espi->espi_base->event, fsl_espi->espi_save_regs.event);
	out_be32(&fsl_espi->espi_base->mask, fsl_espi->espi_save_regs.mask);
	for (i = 0; i < 4; i++)
		out_be32(&fsl_espi->espi_base->csmode[i],
			fsl_espi->espi_save_regs.csmode[i]);

	return 0;
}
#endif

static const struct of_device_id of_fsl_espi_match[] = {
	{ .compatible = "fsl,mpc8536-espi" },
	{}
};
MODULE_DEVICE_TABLE(of, of_fsl_espi_match);

static struct of_platform_driver of_fsl_espi_driver = {
	.name		= "fsl_espi",
	.match_table	= of_fsl_espi_match,
	.probe		= of_fsl_espi_probe,
	.remove		= __devexit_p(of_fsl_espi_remove),
#ifdef CONFIG_SUSPEND
	.suspend     = fsl_espi_suspend,
	.resume      = fsl_espi_resume,
#endif
};

static int __init fsl_espi_init(void)
{
	return of_register_platform_driver(&of_fsl_espi_driver);
}
module_init(fsl_espi_init);

static void __exit fsl_espi_exit(void)
{
	of_unregister_platform_driver(&of_fsl_espi_driver);
}
module_exit(fsl_espi_exit);

MODULE_AUTHOR("Mingkai Hu");
MODULE_DESCRIPTION("Enhanced Freescale SPI Driver");
MODULE_LICENSE("GPL");
