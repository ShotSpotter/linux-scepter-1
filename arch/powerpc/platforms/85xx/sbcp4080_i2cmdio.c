/*
 *  Copyright (C) 2011 Wind River Systems, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  The SBCP4080 board has four SFP Module Ports connected to P4080 SGMII
 *  interface, and the SFP ports communicate with the host MAC using I2C
 *  bus. So we need implement an I2C MDIO driver in order to make PHYs
 *  accessible to MAC.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

/* GPIO 0 4 9 26 18 19 need to be set as output */
#define GPIO_GPDIR_I2CMDIO	0x88403020
#define GPIO_GPDAT_OFFSET	2

struct p4080ds_i2cmdio {
	struct i2c_client *client;
	u32 *gpio_reg;
};

/* GPIO 0 4 9 and 26 are used to enable SFP port1-4 */
static const int sfp_gpio_enable[] = {
	0, 4, 9, 26
};

/* GPIO 18-19 are used to select I2C mux routing:
 * 0b00 = SFP port1
 * 0b01 = SFP port3
 * 0b10 = SFP port2
 * 0b11 = SFP port4
*/
static const int sfp_gpio_select[] = {
	0, 2, 1, 3
};

/* SFP PHY is compatible with marvell 88e1111 */
static const struct i2c_device_id p4080ds_i2cmdio_ids[] = {
	{ "88e1111", 0 },
	{ }
};

int p4080ds_i2cmdio_write(struct mii_bus *bus, int port_addr, int dev_addr,
			int regnum, u16 value)
{
	int tmp;
	struct p4080ds_i2cmdio *priv = (struct p4080ds_i2cmdio *)bus->priv;

	/* big-endian to little-endian */
	tmp =  ((value >> 8) & 0x00ff) | ((value << 8) & 0xff00);

	/* set GPIO 18-19 to select I2C mux routing */
	clrsetbits_be32(priv->gpio_reg, (3 << (31 - 19)),
				(sfp_gpio_select[port_addr] << (31 - 19)));

	return i2c_smbus_write_word_data(priv->client, regnum, tmp);
}

int p4080ds_i2cmdio_read(struct mii_bus *bus, int port_addr, int dev_addr,
			int regnum)
{
	int ret, val;
	struct p4080ds_i2cmdio *priv = (struct p4080ds_i2cmdio *)bus->priv;

	/* Most 1000baseT copper SFP modules seem to use a Marvell 88E1111 PHY,
	 * so here we fake a 88E1111 phy id(0x01410e30) to match with 88E1111
	 * phy driver.
	 */
	if (regnum == 2)
		return 0x0141;

	if (regnum == 3)
		return 0x0c90;

	/* set GPIO 18-19 to select I2C mux routing */
	clrsetbits_be32(priv->gpio_reg, (3 << (31 - 19)),
			(sfp_gpio_select[port_addr] << (31 - 19)));

	val = i2c_smbus_read_word_data(priv->client, regnum);

	if (val < 0)
		return val;

	/* little-endian to big-endian */
	ret = ((val >> 8) & 0x00ff) | ((val << 8) & 0xff00);
	return ret;
}

static int p4080ds_i2cmdio_reset(struct mii_bus *bus)
{
	struct device_node *np, *child;
	struct p4080ds_i2cmdio *priv = (struct p4080ds_i2cmdio *)bus->priv;
	struct i2c_client *client = priv->client;
	int phy_enable = 0;

	np = dev_archdata_get_node(&client->dev.archdata);
	for_each_child_of_node(np, child) {
		const __be32 *addr;
		int len;

		addr = of_get_property(child, "reg", &len);
		if (!addr || len < sizeof(*addr) || *addr >= 4 || *addr < 0) {
			dev_err(&client->dev, "%s has invalid PHY address\n",
				child->full_name);
			continue;
		}

		phy_enable |= (0x1 << (0x1f - sfp_gpio_enable[*addr]));
	}

	/* enable related SFP module */
	clrbits32(priv->gpio_reg, phy_enable);
	/* Delay some time for the PHY to initialize itself */
	udelay(120000);

	return 0;

}

static int p4080ds_i2cmdio_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct mii_bus *new_bus;
	struct device_node *np, *gpio;
	struct p4080ds_i2cmdio *priv;
	u64 reg;
	int i;
	const u32 *addr;
	int err = 0;


	np = dev_archdata_get_node(&client->dev.archdata);
	new_bus = mdiobus_alloc();
	if (!new_bus)
		return -ENOMEM;

	new_bus->name = "Freescale P4080DS I2C MDIO Bus";
	new_bus->read = &p4080ds_i2cmdio_read;
	new_bus->write = &p4080ds_i2cmdio_write;
	new_bus->reset = &p4080ds_i2cmdio_reset;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto err_priv_alloc;
	}

	priv->client = client;
	new_bus->priv = priv;

	new_bus->irq = kcalloc(PHY_MAX_ADDR, sizeof(int), GFP_KERNEL);

	if (!new_bus->irq) {
		err = -ENOMEM;
		goto err_irq_alloc;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		new_bus->irq[i] = PHY_POLL;

	/* Find the GPIO register pointer */
	gpio = of_find_compatible_node(NULL, NULL, "fsl,qoriq-gpio");

	if (!gpio) {
		err = -ENODEV;
		goto err_no_gpio;
	}

	addr = of_get_address(gpio, 0, NULL, NULL);
	if (!addr) {
		err = -ENODEV;
		goto err_no_gpio_addr;
	}

	reg = of_translate_address(gpio, addr);

	priv->gpio_reg = ioremap(reg, sizeof(*priv->gpio_reg));

	if (!priv->gpio_reg) {
		err = -ENOMEM;
		goto err_ioremap;
	}

	/* Set the direction of the GPIO ports used by MDIO bus */
	setbits32(priv->gpio_reg, GPIO_GPDIR_I2CMDIO);

	priv->gpio_reg += GPIO_GPDAT_OFFSET;

	dev_set_drvdata(&client->dev, new_bus);
	new_bus->parent = &client->dev;
	sprintf(new_bus->id, "%s", np->name);

	err = of_mdiobus_register(new_bus, np);

	if (err) {
		printk(KERN_ERR "%s: Cannot register as MDIO bus\n",
				new_bus->name);
		goto err_registration;
	}

	return 0;

err_registration:
err_ioremap:
err_no_gpio_addr:
err_no_gpio:
	kfree(new_bus->irq);
err_irq_alloc:
err_priv_alloc:
	kfree(new_bus);

	return err;
}

static int __devexit p4080ds_i2cmdio_remove(struct i2c_client *client)
{
	struct device *device = &client->dev;
	struct mii_bus *bus = dev_get_drvdata(device);

	mdiobus_unregister(bus);
	dev_set_drvdata(device, NULL);
	bus->priv = NULL;
	mdiobus_free(bus);

	return 0;

}

static struct i2c_driver p4080ds_i2cmdio_driver = {
	.driver = {
		.name = "marvell,88e1111",
		.owner = THIS_MODULE,
	},
	.probe = p4080ds_i2cmdio_probe,
	.remove = __devexit_p(p4080ds_i2cmdio_remove),
	.id_table = p4080ds_i2cmdio_ids,
};

int __init p4080ds_i2cmdio_init(void)
{
	return i2c_add_driver(&p4080ds_i2cmdio_driver);
}

void p4080ds_i2cmdio_exit(void)
{
	i2c_del_driver(&p4080ds_i2cmdio_driver);
}
subsys_initcall_sync(p4080ds_i2cmdio_init);
module_exit(p4080ds_i2cmdio_exit);
