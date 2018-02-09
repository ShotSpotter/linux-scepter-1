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

#include <sound/wm8737.h>
#if 0
#include <plat/wm8737-sync.h>
#include <sound/soc-dai.h>
#endif
#include <linux/regulator/machine.h>
#include <linux/regulator/wm8737-micbias-regulator.h>
#include <linux/regulator/fixed.h>

#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include <linux/spi/mcp3001.h>

#include "hsmmc.h"
#include "mux.h"


static char* part_num = "";
module_param(part_num, charp, 0444);

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K

#define FULL_PRODUCTION 0

static struct mtd_partition scepter_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "xloader",
		.offset         = 0,
		.size           = 4 * (SZ_128K),
#ifndef CONFIG_MACH_SCEPTER_BOARD_TEST
		.mask_flags     = MTD_WRITEABLE
#endif
	},
	{
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 14 * (SZ_128K),
#ifndef CONFIG_MACH_SCEPTER_BOARD_TEST
		.mask_flags     = MTD_WRITEABLE
#endif
	},
	{
		.name           = "uboot-params",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 1 * (SZ_128K)
#if FULL_PRODUCTION
		.mask_flags     = MTD_WRITEABLE
#endif
	},
	{
		.name           = "uboot-params-copy",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 1 * (SZ_128K)
#if FULL_PRODUCTION
		.mask_flags     = MTD_WRITEABLE
#endif
	},
	{
		.name           = "linux-kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40 * (SZ_128K),
	},
	{
		.name           = "linux-kernel-failsafe",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40 * (SZ_128K),
#if FULL_PRODUCTION
		.mask_flags     = MTD_WRITEABLE
#endif
	},
	{
		.name           = "rootfs-failsafe",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 256 * (SZ_128K),
#if FULL_PRODUCTION
		.mask_flags     = MTD_WRITEABLE
#endif
	},
	{
		.name           = "mfg",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40 * (SZ_128K),
#if FULL_PRODUCTION
		.mask_flags     = MTD_WRITEABLE
#endif
	},
	{
		.name           = "config",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 80 * (SZ_128K),
	},
	{
		.name           = "cache",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 240 * (SZ_128K),
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

static void __init scepter_flash_init(void)
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

static void scepter_ethernet_init(struct emac_platform_data *pdata)
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

static struct regulator_consumer_supply scepter_wm8737_dvdd_supplies[] = {
		REGULATOR_SUPPLY("DCVDD","2-001a"),
		REGULATOR_SUPPLY("DCVDD","2-001b"),
		REGULATOR_SUPPLY("DBVDD","2-001a"),
		REGULATOR_SUPPLY("DBVDD","2-001b"),
};


static struct regulator_init_data wm8737_dvdd_initdata = {
	.consumer_supplies = scepter_wm8737_dvdd_supplies,
	.num_consumer_supplies = ARRAY_SIZE(scepter_wm8737_dvdd_supplies),
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config scepter_wm8737_dvdd_config = {
	.supply_name		= "wm8737-dvdd",
	.microvolts		= 3300000,
	.gpio			= 128,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.startup_delay = 50000,
	.init_data		= &wm8737_dvdd_initdata,
};

static struct platform_device scepter_wm8737_dvdd_device = {
	.name = "reg-fixed-voltage",
	.id = 0,
	.dev	= {
		.platform_data = &scepter_wm8737_dvdd_config,
	},
};

static struct regulator_consumer_supply scepter_wm8737_avdd_mvdd_supplies[] = {
		REGULATOR_SUPPLY("AVDD","2-001a"),
		REGULATOR_SUPPLY("AVDD","2-001b"),
		REGULATOR_SUPPLY("MVDD","2-001a"),
		REGULATOR_SUPPLY("MVDD","2-001b"),
};

static struct regulator_init_data wm8737_avdd_mvdd_initdata = {
#if 0
	/*supply_regulator_dev is the correct relationship but it is broken.*/
	.supply_regulator_dev = &scepter_wm8737_dvdd_device.dev,
#endif
	.consumer_supplies = scepter_wm8737_avdd_mvdd_supplies,
	.num_consumer_supplies = ARRAY_SIZE(scepter_wm8737_avdd_mvdd_supplies),
	.constraints = {
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config scepter_wm8737_avdd_mvdd_config = {
	.supply_name		= "wm8737-avdd-mvdd",
	.microvolts		= 2800000,
	.gpio			= -1,
	.init_data		= &wm8737_avdd_mvdd_initdata,
};

static struct platform_device scepter_wm8737_avdd_mvdd_device = {
	.name = "reg-fixed-voltage",
	.id = 1,
	.dev	= {
		.platform_data = &scepter_wm8737_avdd_mvdd_config,
	},
};

static struct regulator_consumer_supply scepter_wm8737_micbias_supply[] = {
		REGULATOR_SUPPLY("MICBIAS","2-001a"),
		REGULATOR_SUPPLY("MICBIAS","2-001b"),
};

static struct regulator_init_data wm8737_micbias_initdata[] = {
		{
#if 0
				/*supply_regulator_dev is the correct relationship but it is broken.*/
				.supply_regulator_dev = &scepter_wm8737_avdd_mvdd_device.dev,
#endif
				.consumer_supplies = &scepter_wm8737_micbias_supply[0],
				.num_consumer_supplies = 1,
				.constraints = {
						.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
				},
		},
		{
#if 0
				.supply_regulator_dev = &scepter_wm8737_avdd_mvdd_device.dev,
#endif
				.consumer_supplies = &scepter_wm8737_micbias_supply[1],
				.num_consumer_supplies = 1,
				.constraints = {
						.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
				},
		},
};

static struct wm8737_micbias_config scepter_wm8737_micbias_config[] =
{
		{
				.avdd_mV = 2800,
				.mvdd_mV = 2800,
				.init_data		= &wm8737_micbias_initdata[0],
		},
		{
				.avdd_mV = 2800,
				.mvdd_mV = 2800,
				.init_data		= &wm8737_micbias_initdata[1],
		}
};

static struct platform_device scepter_wm8737_micbias_device[] = {
		{
				.name = "wm8737-micbias-reg",
				.id = 0,
				.dev	= {
						.platform_data = &scepter_wm8737_micbias_config[0],
				},
		},
		{
				.name = "wm8737-micbias-reg",
				.id = 1,
				.dev	= {
						.platform_data = &scepter_wm8737_micbias_config[1],
				},
		},
};

#if 0
static struct wm8737_platform_data scepter_wm8737_data[] = {
		{
				.id = 0,
		},
		{
				.id = 1,
		}
};
static struct wm8737_omap_data scepter_wm8737_master =
{
		.wm8737_id = 0,
		.mcbsp_id = 0,
		.codec_dai_audio_fmt = SND_SOC_DAIFMT_I2S,
		.cpu_dai_audio_fmt = SND_SOC_DAIFMT_I2S,
};

static struct wm8737_omap_data scepter_wm8737_slaves[] =
{
		{
				.wm8737_id = 1,
				.mcbsp_id = 1,
				.codec_dai_audio_fmt = SND_SOC_DAIFMT_I2S,
				.cpu_dai_audio_fmt = SND_SOC_DAIFMT_I2S,
		}
};

static struct wm8737_sync_platform_data scepter_wm8737_sync_data = {
		.sample_cnt_rst_gpio = 84,
		.mclk_en_gpio = 106,
		.mclk = 12288000,
		.master = &scepter_wm8737_master,
		.slaves = scepter_wm8737_slaves,
		.num_slaves = ARRAY_SIZE(scepter_wm8737_slaves),
};

static struct platform_device scepter_wm8737_sync = {
		.name = "wm8737-sync",
		.id = -2,
		.dev = {
			.platform_data = &scepter_wm8737_sync_data,
		},
};
#endif
static struct platform_device generic_soc_slave = {
		.name = "asoc-slave-codec",
		.id = -1,
};

#include <linux/mfd/tps65910.h>

static struct bck_reg_mode_data scepter_tps65910_bck_mode = {
		.mode = {0440,0440,0660,0660,0660},
};

struct tps65910_board scepter_tps65910 = {
	.gpio_base = OMAP_MAX_GPIO_LINES,
#ifdef CONFIG_MACH_SCEPTER_BOARD_TEST
	.en_dev_slp = 1,
#else
	.en_dev_slp = 0,
#endif
	.irq_base = TPS65910_IRQ_BASE,
	.irq = INT_34XX_SYS_NIRQ,
	.tps65910_pmic_init_data = NULL,
	.slp_keepon = NULL,
	.bck_reg_modes = &scepter_tps65910_bck_mode,
};

#define TPS65910_I2C_ADDRESS 0x2D
static struct i2c_board_info __initdata scepter_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ADDRESS),
		.platform_data = &scepter_tps65910,
	},
};

static struct i2c_board_info __initdata scepter_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("wm8737",0x1A),
#if 0
		.platform_data = &(scepter_wm8737_data[0]),
#endif
	},
	{
		I2C_BOARD_INFO("wm8737",0x1B),
#if 0
		.platform_data = &(scepter_wm8737_data[1]),
#endif
	}
};

static struct i2c_board_info __initdata scepter_i2c3_boardinfo[] = {
	{
	},
};

static int __init scepter_i2c_init(void)
{
	omap_register_i2c_bus(1, 100, scepter_i2c1_boardinfo,
				ARRAY_SIZE(scepter_i2c1_boardinfo));
	omap_register_i2c_bus(2, 100, scepter_i2c2_boardinfo,
			ARRAY_SIZE(scepter_i2c2_boardinfo));
	omap_register_i2c_bus(3, 100, scepter_i2c3_boardinfo,
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
#if 0
	&scepter_wm8737_sync,
#endif
	&generic_soc_slave,
	&scepter_wm8737_dvdd_device,
	&scepter_wm8737_avdd_mvdd_device,
	&scepter_wm8737_micbias_device[0],
	&scepter_wm8737_micbias_device[1],
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

static struct omap2_hsmmc_info mmc_if_brd[] = {
	{
		.mmc            = 1,
		.wires          = 4,
		.gpio_cd        = -1,
		.gpio_wp        = -1,
		.ocr_mask       = MMC_VDD_32_33	| MMC_VDD_33_34,
	},
	{
		.mmc            = 2,
		.wires          = 4,
		.gpio_cd        = -1,
		.gpio_wp        = -1,
		.ext_clock      = 1,
		.max_freq       = 26000000,
		.ocr_mask       = MMC_VDD_32_33	| MMC_VDD_33_34,
	},
	{}      /* Terminator */
};

static struct omap2_hsmmc_info mmc_no_if_brd[] = {
	{
		.mmc            = 1,
		.wires          = 4,
		.gpio_cd        = -1,
		.gpio_wp        = -1,
		.ocr_mask       = MMC_VDD_32_33	| MMC_VDD_33_34,
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


static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_PERIPHERAL,
	.power                  = 500,
};

static __init void scepter_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}

static void __init ericsson_cellular_init(void)
{
	usb_ohci_init(&ohci_pdata);
}

struct scepter_part_t {
	const char* part;
	int has_ethernet;
	void (*misc_init)(void);
	void (*gpio_init)(void);
	void (*cell_init)(void);
};

extern void scepter_gpio_revb_init(void);
extern void scepter_gpio_init_400_0100(void);
extern void scepter_gpio_init_400_0103(void);
extern void scepter_gpio_init_400_0105(void);

const struct scepter_part_t scepter_part_list[] =
{
		{ .part = "400-0100-01RevB2",
			.has_ethernet = 1,
			.gpio_init = scepter_gpio_revb_init,
			.cell_init = ericsson_cellular_init,
		},
		{ .part = "400-0100",
			.has_ethernet = 0,
			.gpio_init = scepter_gpio_init_400_0100,
			.cell_init = ericsson_cellular_init,
		},
		{ .part = "400-0106",
			.has_ethernet = 0,
			.gpio_init = scepter_gpio_init_400_0100,
			.cell_init = ericsson_cellular_init,
		},
		{ .part = "400-0105",
			.has_ethernet = 1,
			.gpio_init = scepter_gpio_init_400_0105,
			.cell_init = ericsson_cellular_init,
		},
		{ .part = "400-0103",
			.has_ethernet = 1,
			.gpio_init = scepter_gpio_init_400_0103,
			.cell_init = NULL,
		},
		{ .part = NULL},
};


static void __init scepter_part_init(void)
{
	const struct scepter_part_t* spart = scepter_part_list;
	while(spart->part && (strstr(part_num,spart->part) != part_num)) {
		spart++;
	}

	if(spart->part == NULL) {
		printk(KERN_CRIT "Cannot find scepter part number.\n");

		return;
	}

	if(spart->gpio_init)
		spart->gpio_init();

	if(spart->has_ethernet)
		scepter_ethernet_init(&scepter_emac_pdata);

	if(spart->cell_init)
		spart->cell_init();
}

extern int scepter_detect_if_brd(void);

static void __init scepter_init(void)
{
    printk(KERN_NOTICE "init mux");
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
    printk(KERN_NOTICE "scepter part_init");
	scepter_part_init();
    printk(KERN_NOTICE "scepter i2c_init");
	scepter_i2c_init();
    printk(KERN_NOTICE "scpeter_devices");

	platform_add_devices(scepter_devices,
				ARRAY_SIZE(scepter_devices));
    printk(KERN_NOTICE "scepter spi_init");
	scepter_spi_init();
    printk(KERN_NOTICE "scepter omap_serial_init");
	omap_serial_init();
    printk(KERN_NOTICE "scepter flash_init");
	scepter_flash_init();

	/* MMC init function */
	if (scepter_detect_if_brd() != 0) {
      printk(KERN_NOTICE "mmc detected, initializing...");
	  scepter_musb_init();
      omap2_hsmmc_init(mmc_if_brd);
    } else {
      printk(KERN_NOTICE "No mmc detected.");
      omap2_hsmmc_init(mmc_no_if_brd);
    }
    printk(KERN_NOTICE "scepter init complete.");
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
