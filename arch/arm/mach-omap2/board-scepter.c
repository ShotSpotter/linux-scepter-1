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
#include <linux/i2c/pca953x.h>
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
/* VDCDC1 -> VDD_CORE */
static struct regulator_consumer_supply am3517_evm_vdcdc1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

/* VDCDC2 -> VDDSHV */
static struct regulator_consumer_supply am3517_evm_vdcdc2_supplies[] = {
	{
		.supply = "vddshv",
	},
};

/* VDCDC2 |-> VDDS
	   |-> VDDS_SRAM_CORE_BG
	   |-> VDDS_SRAM_MPU */
static struct regulator_consumer_supply am3517_evm_vdcdc3_supplies[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
};

/* LDO1 |-> VDDA1P8V_USBPHY
	 |-> VDDA_DAC */
static struct regulator_consumer_supply am3517_evm_ldo1_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
	{
		.supply = "vdda_dac",
	},
};

/* LDO2 -> VDDA3P3V_USBPHY */
static struct regulator_consumer_supply am3517_evm_ldo2_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};

static struct regulator_init_data am3517_evm_regulator_data[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc1_supplies),
		.consumer_supplies = am3517_evm_vdcdc1_supplies,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc2_supplies),
		.consumer_supplies = am3517_evm_vdcdc2_supplies,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc3_supplies),
		.consumer_supplies = am3517_evm_vdcdc3_supplies,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_ldo1_supplies),
		.consumer_supplies = am3517_evm_ldo1_supplies,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_ldo2_supplies),
		.consumer_supplies = am3517_evm_ldo2_supplies,
	},
};

static struct i2c_board_info __initdata am3517evm_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("s35390a", 0x30),
		.type		= "s35390a",
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &am3517_evm_regulator_data[0],
	},
};

/*
 * I2C GPIO Expander - TCA6416
 */

/* Mounted on Base-Board */
static struct pca953x_platform_data am3517evm_gpio_expander_info_0 = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
};
static struct i2c_board_info __initdata am3517evm_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic23", 0x1A),
	},
	{
		I2C_BOARD_INFO("tca6416", 0x21),
		.platform_data = &am3517evm_gpio_expander_info_0,
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
	omap_register_i2c_bus(2, 400, am3517evm_i2c2_boardinfo,
			ARRAY_SIZE(am3517evm_i2c2_boardinfo));
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

enum usbhs_omap_port_mode {
	OMAP_USBHS_PORT_MODE_UNUSED,
	OMAP_EHCI_PORT_MODE_PHY,
	OMAP_EHCI_PORT_MODE_TLL,
	OMAP_EHCI_PORT_MODE_HSIC,
	OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0,
	OMAP_OHCI_PORT_MODE_PHY_6PIN_DPDM,
	OMAP_OHCI_PORT_MODE_PHY_3PIN_DATSE0,
	OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	OMAP_OHCI_PORT_MODE_TLL_6PIN_DATSE0,
	OMAP_OHCI_PORT_MODE_TLL_6PIN_DPDM,
	OMAP_OHCI_PORT_MODE_TLL_3PIN_DATSE0,
	OMAP_OHCI_PORT_MODE_TLL_4PIN_DPDM,
	OMAP_OHCI_PORT_MODE_TLL_2PIN_DATSE0,
	OMAP_OHCI_PORT_MODE_TLL_2PIN_DPDM
};

static void __init scepter_init_irq(void)
{
	omap_board_config = scepter_config;
	omap_board_config_size = ARRAY_SIZE(scepter_config);

	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#define ohci_hcd_omap_platform_data ehci_hcd_omap_platform_data
static const struct ohci_hcd_omap_platform_data ohci_pdata __initconst = {
	.port_mode[0] = OMAP_OHCI_PORT_MODE_TLL_4PIN_DPDM,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_TLL_4PIN_DPDM,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
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
		.gpio_cd        = 127,
		.gpio_wp        = 126,
		.ocr_mask       = MMC_VDD_165_195 |
				MMC_VDD_26_27 | MMC_VDD_27_28 |
				MMC_VDD_29_30 |
				MMC_VDD_30_31 | MMC_VDD_31_32,
	},
	{
		.mmc            = 2,
		.wires          = 4,
		.gpio_cd        = 138,
		.gpio_wp        = 176,
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

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static struct resource ohci_resources[] = {
	{
		.start	= OMAP34XX_OHCI_BASE,
		.end	= OMAP34XX_OHCI_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_34XX_OHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ohci_device = {
	.name			= "ohci-omap3",
	.id			= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(ohci_resources),
	.resource	= ohci_resources,
};

static void setup_ohci_io_mux(const enum usbhs_omap_port_mode *port_mode)
{
	switch (port_mode[0]) {
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DPDM:
		omap_mux_init_signal("mm1_rxdp",
				     OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mm1_rxdm",
				     OMAP_PIN_INPUT_PULLDOWN);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_4PIN_DPDM:
		omap_mux_init_signal("mm1_rxrcv",
				     OMAP_PIN_INPUT_PULLDOWN);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_PHY_3PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_3PIN_DATSE0:
		omap_mux_init_signal("mm1_txen_n", OMAP_PIN_OUTPUT);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DPDM:
		omap_mux_init_signal("mm1_txse0",
				     OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mm1_txdat",
				     OMAP_PIN_INPUT_PULLDOWN);
		break;
	case OMAP_USBHS_PORT_MODE_UNUSED:
		/* FALLTHROUGH */
	default:
		break;
	}
	switch (port_mode[1]) {
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DPDM:
		omap_mux_init_signal("mm2_rxdp",
				     OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mm2_rxdm",
				     OMAP_PIN_INPUT_PULLDOWN);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_4PIN_DPDM:
		omap_mux_init_signal("mm2_rxrcv",
				     OMAP_PIN_INPUT_PULLDOWN);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_PHY_3PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_3PIN_DATSE0:
		omap_mux_init_signal("mm2_txen_n", OMAP_PIN_OUTPUT);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DPDM:
		omap_mux_init_signal("mm2_txse0",
				     OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mm2_txdat",
				     OMAP_PIN_INPUT_PULLDOWN);
		break;
	case OMAP_USBHS_PORT_MODE_UNUSED:
		/* FALLTHROUGH */
	default:
		break;
	}
	switch (port_mode[2]) {
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_PHY_6PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_6PIN_DPDM:
		omap_mux_init_signal("mm3_rxdp",
				     OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mm3_rxdm",
				     OMAP_PIN_INPUT_PULLDOWN);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM:
	case OMAP_OHCI_PORT_MODE_TLL_4PIN_DPDM:
		omap_mux_init_signal("mm3_rxrcv",
				     OMAP_PIN_INPUT_PULLDOWN);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_PHY_3PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_3PIN_DATSE0:
		omap_mux_init_signal("mm3_txen_n", OMAP_PIN_OUTPUT);
		/* FALLTHROUGH */
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DATSE0:
	case OMAP_OHCI_PORT_MODE_TLL_2PIN_DPDM:
		omap_mux_init_signal("mm3_txse0",
				     OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mm3_txdat",
				     OMAP_PIN_INPUT_PULLDOWN);
		break;
	case OMAP_USBHS_PORT_MODE_UNUSED:
		/* FALLTHROUGH */
	default:
		break;
	}
}

void __init usb_ohci_init(const struct ohci_hcd_omap_platform_data *pdata)
{
	printk(KERN_INFO "Initializing USB OHCI driver...\n");
	platform_device_add_data(&ohci_device, pdata, sizeof(*pdata));

	/* Setup Pin IO MUX for OHCI */
	if (cpu_is_omap34xx())
		setup_ohci_io_mux((enum usbhs_omap_port_mode *)pdata->port_mode);

	if (platform_device_register(&ohci_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (OHCI) device\n");
		return;
	}
}

static void __init scepter_spi_init(void)
{
	spi_register_board_info(scepter_spi_board_info,
			  	ARRAY_SIZE(scepter_spi_board_info));
}

#define GSM_3304_ON 54
#define GSM_PWRON 55
#define GSM_HWR 64

static void __init scepter_gsm_init(void)
{
	int r, i;

	omap_mux_init_gpio(GSM_PWRON, OMAP_PIN_OUTPUT);
	r = gpio_request(GSM_PWRON, "gsm-pwron");
	if (r < 0) {
		printk(KERN_ERR "[GSM] failed to get GPIO#%d\n", GSM_PWRON);
		return;
	}
	gpio_direction_output(GSM_PWRON, 0);

        omap_mux_init_gpio(GSM_HWR, OMAP_PIN_INPUT);
	r = gpio_request(GSM_HWR, "gsm-hwr");
	if (r < 0) {
		printk(KERN_ERR "[GSM] failed to get GPIO#%d\n", GSM_HWR);
		return;
	}
	gpio_direction_input(GSM_HWR);

	omap_mux_init_gpio(GSM_3304_ON, OMAP_PIN_OUTPUT);
	r = gpio_request(GSM_3304_ON, "gsm-3304-on");
	if (r < 0) {
		printk(KERN_ERR "[GSM] failed to get GPIO#%d\n", GSM_3304_ON);
		return;
	}
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
	gpio_set_value(GSM_3304_ON, 1);

	if (r != 0) {
		printk(KERN_ERR "[GSM] initialization failed.\n");
		return;
	}

	printk(KERN_INFO "GSM module successfully initialized [%d].\n", i);
}

static void __init scepter_cpu_usb_speed_init(void)
{
	int r;
	int gpio = 74;

        printk("Setting USB speed ...\n");
	omap_mux_init_gpio(gpio, OMAP_PIN_OUTPUT);
	r = gpio_request(gpio, "cpu-usb-speed");
	if (r < 0) {
		printk(KERN_ERR "failed to request GPIO#%d\n", gpio);
		return;
	}
	gpio_direction_output(gpio, 1);
}

static void __init scepter_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

	scepter_cpu_usb_speed_init();
	scepter_gsm_init();

	scepter_i2c_init();
	platform_add_devices(scepter_devices,
				ARRAY_SIZE(scepter_devices));

	scepter_spi_init();
	omap_serial_init();
	scepter_flash_init();

	/* Configure GPIO for EHCI port */
	omap_mux_init_gpio(57, OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);
	usb_ohci_init(&ohci_pdata);

	scepter_ethernet_init(&scepter_emac_pdata);

	i2c_register_board_info(1, am3517evm_i2c1_boardinfo,
				ARRAY_SIZE(am3517evm_i2c1_boardinfo));
	/* MMC init function */
	omap2_hsmmc_init(mmc);

        /* CPLD - Set the version gpio pins to MODE 4 and IN */
        omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_INPUT, 67);
        omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_INPUT, 68);
        omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_INPUT, 69);
}

static void __init scepter_map_io(void)
{
	omap2_set_globals_343x();
	omap34xx_map_common_io();
}
#if 0
MACHINE_START(SCEPTER, "Scepter Board")
#else
MACHINE_START(OMAP3517EVM, "Scepter Board")
#endif
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= scepter_map_io,
	.init_irq	= scepter_init_irq,
	.init_machine	= scepter_init,
	.timer		= &omap_timer,
MACHINE_END
