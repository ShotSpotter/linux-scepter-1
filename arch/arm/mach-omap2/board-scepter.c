/*
 * linux/arch/arm/mach-omap2/board-scepter.c
 *
 * Copyright (C) 2012 HY Research LLC
 * Author: Hunyue Yau <hy-git@hy-research.com>
 *
 * Based on mach-omap2/board-am3517evm.c
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
#include <linux/davinci_emac.h>
#include <linux/i2c/at24.h>
#include <linux/regulator/machine.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/usb.h>
#include <plat/nand.h>
#include <plat/gpmc.h>
#include <plat/mmc.h>

#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include <linux/spi/mcp3001.h>

#include "hsmmc.h"
#include "mux.h"

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K

static struct mtd_partition scepter_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "xloader",
		.offset         = 0,
		.size           = 4 * (SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 14 * (SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot-params",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2 * (SZ_128K)
	},
	{
		.name           = "linux-kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40 * (SZ_128K)
	},
	{
		.name           = "rootfs",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,
	},
};

static struct omap_nand_platform_data scepter_nand_data = {
	.parts          = scepter_nand_partitions,
	.nr_parts       = ARRAY_SIZE(scepter_nand_partitions),
	.nand_setup     = NULL,
	.dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
	.devsize        = 1,
	.dev_ready      = NULL,
};

void __init scepter_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}
	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				" in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		scepter_nand_data.cs   = nandcs;
		scepter_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
				GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
		scepter_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);
		gpmc_nand_init(&scepter_nand_data);
	}
}


static struct gpio_led gpio_leds[] = {
	{
		.name			= "CPU_LED_2",
		.gpio			= 43,
	},
	{
		.name			= "CPU_LED_1",
		.gpio			= 42,
	},
	{
		.name			= "CPU_STAT_LED0",
		.gpio			= 85,
	},
	{
		.name			= "CPU_STAT_LED1",
		.gpio			= 86,
	},
	{
		.name			= "CPU_STAT_LED2",
		.gpio			= 87,
	},
	{
		.name			= "CPU_STAT_LED3",
		.gpio			= 88,
	},
	{
		.name			= "CPU_STAT_LED4",
		.gpio			= 89,
	},
	{
		.name			= "CPU_STAT_LED5",
		.gpio			= 90,
	},
};
static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

#define AM35XX_EVM_PHY_MASK          (0xF)
#define AM35XX_EVM_MDIO_FREQUENCY    (1000000) /*PHY bus frequency */

static struct emac_platform_data scepter_emac_pdata = {
	.phy_mask       = AM35XX_EVM_PHY_MASK,
	.mdio_max_freq  = AM35XX_EVM_MDIO_FREQUENCY,
	.rmii_en        = 1,
};

static int __init eth_addr_setup(char *str)
{
	int i;
	long res;

	if (str == NULL)
		return 0;
	for (i = 0; i <  ETH_ALEN; i++) {
		if (!strict_strtol(&str[i * 3], 16, &res))
			scepter_emac_pdata.mac_addr[i] = res;
		else
			return -EINVAL;
	}
	return 1;
}

/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name           = "davinci_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(am3517_emac_resources),
	.resource       = am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR |
			AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
			AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

void scepter_ethernet_init(struct emac_platform_data *pdata)
{
	u32 regval;

	pdata->ctrl_reg_offset          = AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset      = AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset          = AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->mdio_reg_offset          = AM35XX_EMAC_MDIO_OFFSET;
	pdata->ctrl_ram_size            = AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version                  = EMAC_VERSION_2;
	pdata->hw_ram_addr              = AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable	        = am3517_enable_ethernet_int;
	pdata->interrupt_disable        = am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data     = pdata;
	platform_device_register(&am3517_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
 }

/* TPS65023 specific initialization */

#define TPS65910_I2C_ADDRESS 0x2D
static struct i2c_board_info __initdata scepter_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ADDRESS),
	},
};

static struct i2c_board_info __initdata scepter_i2c3_boardinfo[] = {
	{
		I2C_BOARD_INFO("24c64", 0x50),
	},
};

static int __init scepter_i2c_init(void)
{
	omap_register_i2c_bus(1, 200, NULL, 0);
	omap_register_i2c_bus(3, 400, scepter_i2c3_boardinfo,
			ARRAY_SIZE(scepter_i2c3_boardinfo));

	return 0;
}

/*
 * Board initialization
 */
static struct omap_board_config_kernel scepter_config[] __initdata = {
};

static struct platform_device *scepter_devices[] __initdata = {
	&leds_gpio,
};

static void __init scepter_init_irq(void)
{
	omap_board_config = scepter_config;
	omap_board_config_size = ARRAY_SIZE(scepter_config);

	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static const struct ohci_hcd_omap_platform_data ohci_pdata __initconst = {
	.port_mode[0] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.port_mode[2] = OMAP_OHCI_PORT_MODE_UNUSED,
	.es2_compatibility = 1, /*p1, p2, and p3 bypass do not exist*/
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.wires          = 4,
		.gpio_cd        = 99,
		.ocr_mask       = MMC_VDD_165_195 |
				MMC_VDD_26_27 | MMC_VDD_27_28 |
				MMC_VDD_29_30 |
				MMC_VDD_30_31 | MMC_VDD_31_32,
	},
	{
		.mmc            = 2,
		.wires          = 4,
		.gpio_cd        = 138,
		.ocr_mask       = MMC_VDD_165_195 |
				MMC_VDD_26_27 | MMC_VDD_27_28 |
				MMC_VDD_29_30 |
				MMC_VDD_30_31 | MMC_VDD_31_32,
	},
	{}      /* Terminator */
};

static struct omap2_mcspi_device_config mcp3001_mcspi_config = {
	.turbo_mode = 0,
	.single_channel = 1,
};

static struct mcp3001_platform_data mcp3001_config __initdata = {
	.nvals = 32,
};

static struct spi_board_info scepter_spi_board_info[] __initdata = {
	{
		.modalias		= "mcp3k1",
		.bus_num		= 1,
		.chip_select		= 0,
		// 2.80MHz Vdd == 5V
		// 1.05MHz Vdd == 2.7V
		.max_speed_hz		= 1050000,
		.controller_data	= &mcp3001_mcspi_config,
		.mode			= SPI_MODE_3,
		.irq			= -1,
		.platform_data		= &mcp3001_config,
	}
};

static void __init scepter_spi_init(void)
{
	spi_register_board_info(scepter_spi_board_info,
			  	ARRAY_SIZE(scepter_spi_board_info));
}

#define GSM_3304_ON	54
#define GSM_PWRON	55
#define GSM_HWR		64

static void __init scepter_gsm_init(void)
{
	int r, i;

	omap_mux_init_gpio(GSM_PWRON, OMAP_PIN_OUTPUT);
	r = gpio_request(GSM_PWRON, "gsm-pwron");
	if (r < 0) {
		printk(KERN_ERR "[GSM] failed to get GPIO#%d\n", GSM_PWRON);
		return;
	}
	gpio_export(GSM_PWRON,0);
	gpio_direction_output(GSM_PWRON, 0);

        omap_mux_init_gpio(GSM_HWR, OMAP_PIN_INPUT);
	r = gpio_request(GSM_HWR, "gsm-hwr");
	if (r < 0) {
		printk(KERN_ERR "[GSM] failed to get GPIO#%d\n", GSM_HWR);
		return;
	}
	gpio_export(GSM_HWR,0);
	gpio_direction_input(GSM_HWR);

	omap_mux_init_gpio(GSM_3304_ON, OMAP_PIN_OUTPUT);
	r = gpio_request(GSM_3304_ON, "gsm-3304-on");
	if (r < 0) {
		printk(KERN_ERR "[GSM] failed to get GPIO#%d\n", GSM_3304_ON);
		return;
	}
	gpio_export(GSM_3304_ON,0);
	gpio_direction_output(GSM_3304_ON, 0);

#if 0
        msleep(20000);
        printk("[GSM] modem HW state: %d\n", gpio_get_value(GSM_HWR));
        gpio_set_value(GSM_3304_ON, 1);
        msleep(1);
        gpio_set_value(GSM_3304_ON, 0);
#endif

	for (i = 0; i < 2000; i++) {
		msleep(1);
		r = gpio_get_value(GSM_HWR);
		if (r  == 0) break;
	}
	/*
	 * Note - ericsson integrators guide '2/1553-KRD 131 24 Uen Rev A'
	 * indicates that POWER_ON (GSM_3304_ON_N) should be negated within
	 * 10 seconds of HW_READY (GSM_HWR_N) being asserted or else
	 * the module will enter an uncontrolled shutdown.  However,
	 * testing suggests that negating POWER_ON will actually turn off
	 * the module.
	 */

	/* TODO the board comes up with POWER_ON assertedd.
	 * A period of time with POWER_ON negated may be necessary
	 * but this has not been tested.
	 */
	if (r != 0) {
		printk(KERN_ERR "[GSM] initialization failed.\n");
		return;
	}

	printk(KERN_INFO "GSM module successfully initialized [%d].\n", i);
}

#define USB_SPEED_GPIO		74
#define USB_SOFTCON_GPIO	75

static void __init scepter_cpu_usb_init(void)
{
	int r;
	int gpio = USB_SPEED_GPIO;

	printk("Setting USB speed ...\n");
	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);
	r = gpio_request(gpio, "usb-speed");
	if (r < 0) {
		printk(KERN_ERR "Failed to request GPIO#%d\n", gpio);
		return;
	}
	gpio_direction_output(gpio, 1);

	gpio = USB_SOFTCON_GPIO;

	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);
	r = gpio_request(gpio, "usb-softcon");
	if (r < 0) {
		printk(KERN_ERR "Failed to request GPIO#%d\n", gpio);
		return;
	}
	gpio_direction_output(gpio, 1);
	gpio_set_value(gpio, 1);
	gpio_export(gpio,0);
}

#define TPS65910_DEVCTRL_REG	0x3F
#define TPS65910_I2C_ADDRESS	0x2D

static int tps65910_read_reg(struct i2c_adapter *adapter, u8 addr)
{
	struct i2c_msg msg[2];
	u8 data[2];
	int ret;

	msg[0].addr = TPS65910_I2C_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = 1;
	data[0] = addr;
	msg[0].buf = data;
	msg[1].addr = TPS65910_I2C_ADDRESS;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data;

	ret = i2c_transfer(adapter, msg, 2);
	if (ret < 0)
		return ret;
	else
		return data[0];
}

static int tps65910_write_reg(struct i2c_adapter *adapter, u8 addr, u8 val)
{
	struct i2c_msg msg[1];
	u8 data[2];
	int ret;

	msg[0].addr = TPS65910_I2C_ADDRESS;
	msg[0].flags = 0;
	msg[0].len = 2;
	data[0] = addr;
	data[1] = val;
	msg[0].buf = data;

	ret = i2c_transfer(adapter, msg, 1);
	if (ret < 0)
		return ret;
	else
		return data[0];
}


static void __init scepter_pmic_init(void)
{
	struct i2c_adapter *adapter;
	int reg;

	adapter = i2c_get_adapter(1);
	if (adapter == NULL) {
		printk(KERN_ERR "I2C adapter[1] is not available ?!\n");
		return -1;
	}

	reg = tps65910_read_reg(adapter, TPS65910_DEVCTRL_REG);
	if (reg < 0) {
		printk(KERN_ERR "I2C failed to read DEVCTRL_REG ?!\n");
	} else {
		printk("[tps65910] DEVCTRL_REG=0x%x\n", reg);
	}

	tps65910_write_reg(adapter, TPS65910_DEVCTRL_REG, 0x40);
	if (reg < 0) {
		printk(KERN_ERR "I2C failed to write DEVCTRL_REG ?!\n");
	} else {
		printk("[tps65910] DEVCTRL_REG=0x%x\n", reg);
	}

	reg = tps65910_read_reg(adapter, TPS65910_DEVCTRL_REG);
	if (reg < 0) {
		printk(KERN_ERR "I2C failed to read DEVCTRL_REG ?!\n");
	} else {
		printk("[tps65910] Post DEVCTRL_REG=0x%x\n", reg);
	}

	i2c_put_adapter(adapter);
}

static void __init scepter_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

	scepter_cpu_usb_init();
	scepter_gsm_init();

	scepter_i2c_init();
	platform_add_devices(scepter_devices,
				ARRAY_SIZE(scepter_devices));

	scepter_spi_init();
	omap_serial_init();
	scepter_flash_init();

	usb_ohci_init(&ohci_pdata);

	scepter_ethernet_init(&scepter_emac_pdata);

	i2c_register_board_info(1, scepter_i2c1_boardinfo,
				ARRAY_SIZE(scepter_i2c1_boardinfo));

	/* MMC init function */
	omap2_hsmmc_init(mmc);

        /* CPLD - Set the version gpio pins to MODE 4 and IN */
        omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_INPUT, 67);
        omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_INPUT, 68);
        omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_INPUT, 69);
	scepter_pmic_init();
}

static void __init scepter_map_io(void)
{
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}

MACHINE_START(SCEPTER, "Scepter Board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= scepter_map_io,
	.init_irq	= scepter_init_irq,
	.init_machine	= scepter_init,
	.timer		= &omap_timer,
MACHINE_END
