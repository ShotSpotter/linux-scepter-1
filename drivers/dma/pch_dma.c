/**
 * @file pch_dma.c
 *
 * @brief
 *		This file defines the methods of PCH_DMA_CONTROLLER driver.
 *
 * @version 0.90
 * @section
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307, USA.
 *
 * <hr>
 */

/*
 * History:
 * Copyright (C) 2008 OKI SEMICONDUCTOR Co., LTD.
 *
 *
 * created:
 *	OKISEMI 04/14/2010
 *
 */

/* inclusion of system specific header files. */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/interrupt.h>

/* inclusion of module specific header files. */
#include "pch_dma.h"

/* Device specific limitations and properties. */
/* The maximum number of channels allowed in any of the PCH device. */
#define PCH_DMA_CHANNELS_MAX		12
/* The no. of DMA devices allowable. */
#define PCH_DMA_MAX_DEVS			2

/* The maximum number of transfer size in bytes for a channel */
#define PCH_DMA_8BIT_SIZE_MAX		2047
#define PCH_DMA_16BIT_SIZE_MAX		4094
#define PCH_DMA_32BIT_SIZE_MAX		4096

/* The Device ID of the DMA device */
#define PCI_DEVICE_ID_INTEL_PCH1_DMA4_0	0x8815
#define PCI_DEVICE_ID_INTEL_PCH1_DMA8_0	0x8810

/* The Device ID of DMA requesting devices. */
#define PCI_DEVICE_ID_PCH_UART0		0x8811
#define PCI_DEVICE_ID_PCH_UART1		0x8812
#define PCI_DEVICE_ID_PCH_UART2		0x8813
#define PCI_DEVICE_ID_PCH_UART3		0x8814
#define PCI_DEVICE_ID_PCH_SPI		0x8816

/* Internal device IDs used for identifing the DMAC. */
#define PCH_DMA_4CH0		0x40
#define PCH_DMA_4CH1		0x41
#define PCH_DMA_4CH2		0x42
#define PCH_DMA_4CH3		0x43
#define PCH_DMA_4CH4		0x44
#define PCH_DMA_8CH0		0x80
#define PCH_DMA_8CH1		0x81
#define PCH_DMA_8CH2		0x82
#define PCH_DMA_8CH3		0x83
#define PCH_DMA_12CH0	0xC0

/* Status denoting macros. */
#define DMA_STATUS_IDLE		0
#define DMA_STATUS_DESC_READ	1
#define DMA_STATUS_WAIT		2
#define DMA_STATUS_ACCESS		3

/* Constant used to denote disable interrupt. */
#define PCH_DMA_INTERRUPT_DISABLE	0
/* Constant used to denote enable interrupt. */
#define PCH_DMA_INTERRUPT_ENABLE	1

/* The counter limit. */
#define COUNTER_LIMIT		0xFFFF

/* Value used for masking the 3 LSB bits. */
#define MSK_ALL_THREE		0x7

/* macro to set a specified bit(mask) at the specified address */
#define PCH_DMA_BIT_SET(reg, bitmask) \
		 iowrite32(((ioread32((reg)) | (bitmask))), (reg))

/* macro to clear a specified bit(mask) at the specified address */
#define PCH_DMA_BIT_CLEAR(reg, bitMask) \
		iowrite32((ioread32((reg)) & (~(bitMask))), (reg))

/* Macro for setting selected bits of control register. */
#define DEFAULT_CONTROL_REGISTER_VALUE	0x33333333


MODULE_LICENSE("GPL");

/* The module name variable */
#define MODULE_NAME "pch_dma"
/* The return value of @ref get_dev_type for invalid device type */
#define PCH_INVALID_DEVICE			0xFFFF


#ifdef DEBUG
#define PCH_DMA_DEBUG(fmt, args...)\
	printk(KERN_DEBUG MODULE_NAME ": " fmt, ##args)
#else
#define PCH_DMA_DEBUG(fmt, args...)	do { } while (0)
#endif

/**
 * struct pch_dma_ch_regs - The register.
 * @in_ad	DMA Inside address register
 * @out_ad	DMA Outside address register
 * @sz		DMA Mode & Size register
 * @nx_ad	DMA Next Descriptor address register
 */
struct pch_dma_ch_regs {
	u32	in_ad;
	u32	out_ad;
	u32	sz;
	u32	nx_ad;
};

/**
 * struct pch_dma_regs - The register.
 * @dma_ctl0		DMA Control register 0
 * @dma_ctl1		DMA Control register 1
 * @dma_ctl2		DMA Control register 2
 * @reserved1		DMA Control register 3
 * @dma_sts0		DMA Status register 0
 * @dma_sts1		DMA Status register 1
 * @reserved2		DMA Status register 2
 * @reserved3		Reserved register
 * @dmach		DMA ch register
 */
struct pch_dma_regs {
	u32			dma_ctl0;
	u32			dma_ctl1;
	u32			dma_ctl2;
	u32			dma_ctl3;
/* Mask for mode bits.*/
#define DMA_MASK_MODE_BITS		0x00000003UL
/* DMA shift mode bits.*/
#define DMA_SHIFT_MODE_BITS		4
/* Mask for priority bits.*/
#define DMA_MASK_PRIORITY_BITS	0x3UL
/* Shift value for DMA priority bits.*/
#define DMA_SHIFT_PRIORITY_BITS	4
/* Direct Start Bit Setting values.*/
#define DMA_DIR_START		0x00000100UL
/* Interrupt Enable Bit setting values.*/
#define DMA_INTERRUPT_BIT		0x00000001UL
	u32			dma_sts0;
	u32			dma_sts1;
	u32			dma_sts2;
/* Abort notify Bit Setting Values */
#define DMA_ABORT_OCCUR		0x00000100UL
/* Interrupt notify Bit Setting Values */
#define DMA_INTERRUPT_OCCUR		0x00000001UL
/* Mask for status bits. */
#define DMA_MASK_STATUS_BITS	0x3UL
/* The DMA size status bits. */
#define DMA_SIZE_STATUS_BITS	2
/* The shift value for DMA status bits. */
#define DMA_SHIFT_STATUS_BITS	16
	u32			reserved3;
	struct pch_dma_ch_regs	dma[8];
};

/**
 * struct pch_dma_devices - Format for maintaining the device information.
 * @base_addr		The remapped base address for register access
 * @dev_typ		The device type indicating number of DMA channels
 * @dev		The void pointer for storing any references if required
 */
struct pch_dma_devices {
	u32 base_addr;
	u32 dev_typ;
};

/**
 * struct pch_dma_controller_info - Format for storing the details
 *						of the DMA channels.
 * @dma_trans_mode		DMA Transfer Mode
 * @ch_enable			To know if channel is enabled or not
 * @head_of_list		Pointer to start descriptor
 * @tail_of_list		Pointer to last descriptor
 * @call_back_func_ptr	Address of the call back function that is to be
 *				called when an interrupt occurs
 * @callback_data		The data to passed to the callback
 *				function during invocation
 * @dma_access_size		To store the access size (8bit, 16bit or 32bit)
 * @dma_trans_size		To store the value of Transfer Size
 * @dma_trans_direction	To store the Direction of Transfer
 *				(IN to OUT or OUT to IN)
 * @in_addr			The in_address
 * @out_addr			The out_address
 */
struct pch_dma_controller_info {
	u16 dma_trans_mode;
	u16 ch_enable;
	struct pch_dma_desc *head_of_list;
	struct pch_dma_desc *tail_of_list;
	void (*call_back_func_ptr) (int, unsigned long);
	u32 callback_data;
	u16 dma_access_size;
	u16 dma_trans_size;
	u16 dma_trans_direction;
	dma_addr32_t in_addr;
	dma_addr32_t out_addr;
};

/**
 * struct pch_dma_channel_alloc_table - Format for storing the details of
 *				the allocation details of the DMA channels.
 * @dma_dev_id	The DMA device ID
 * @request_signal	The request type
 * @req_device_id	The device ID of the requested device
 * @channel		The channel number
 * @ch_found:1	The flag variable for channel in use
 * @ch_alloced:1	The flag variable for channel allocate
 * @reg		The regster of the DMA device
 */
struct pch_dma_channel_alloc_table {
	u32 dma_dev_id;
	enum pch_channel_request_id request_signal;
	u32 req_device_id;
	u16 channel;
	u16 ch_found:1;
	u16 ch_alloced:1;
	struct pch_dma_regs *reg;
};


/* Device suspend flag. */
static u8 pch_dma_suspended;

/* Device lock variable. */
spinlock_t pch_dma_lock;

/* Stores the details of the DMA devices. */
struct pch_dma_devices pch_dma_devices[PCH_DMA_MAX_DEVS];

/* The structure for specifying the supported
				device IDs to the PCI Kernel subsystem. */
static const struct pci_device_id pch_dma_pcidev_id[] __devinitdata = {
	/* 4 Channel DMA device ID */
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_PCH1_DMA4_0)},
	/* 8 Channel DMA device ID */
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_PCH1_DMA8_0)},
};


/* Retains the specific channel information. */
struct pch_dma_controller_info pch_dma_channel_info[PCH_DMA_CHANNELS_MAX];

/* Channel Allocation Table for DMA */
/* Retains the specific channel allocation information. */
struct
pch_dma_channel_alloc_table pch_dma_channel_table[PCH_DMA_CHANNELS_MAX]
= {
	/* 4 channel DMA device0 (Reserved for GE.) */
	{PCH_DMA_4CH0, PCH_DMA_TX_DATA_REQ0, PCI_DEVICE_ID_PCH_SPI, 0, 0, 0, 0},
	{PCH_DMA_4CH0, PCH_DMA_RX_DATA_REQ0, PCI_DEVICE_ID_PCH_SPI, 1, 0, 0, 0},
	{PCH_DMA_4CH0, 0, 0, 2, 0, 0, 0},
	{PCH_DMA_4CH0, 0, 0, 3, 0, 0, 0},

	/* 8 channel DMA device0 (Reserved for GE.) */
	{PCH_DMA_8CH0, PCH_DMA_TX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART0, 0, 0, 0,
	 0},
	{PCH_DMA_8CH0, PCH_DMA_RX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART0, 1, 0, 0,
	 0},
	{PCH_DMA_8CH0, PCH_DMA_TX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART1, 2, 0, 0,
	 0},
	{PCH_DMA_8CH0, PCH_DMA_RX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART1, 3, 0, 0,
	 0},
	{PCH_DMA_8CH0, PCH_DMA_TX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART2, 4, 0, 0,
	 0},
	{PCH_DMA_8CH0, PCH_DMA_RX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART2, 5, 0, 0,
	 0},
	{PCH_DMA_8CH0, PCH_DMA_TX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART3, 6, 0, 0,
	 0},
	{PCH_DMA_8CH0, PCH_DMA_RX_DATA_REQ0, PCI_DEVICE_ID_PCH_UART3, 7, 0, 0,
	 0}
};

inline s32 dma_get_interrupt_status(u16 ch, u32 stat0, u32 stat2)
{
	if (ch < 8)
		return (((stat0) & (DMA_INTERRUPT_OCCUR << ch)) != 0);
	else
		return (((stat2) & (DMA_INTERRUPT_OCCUR << (ch - 8))) != 0);
}

inline s32 dma_get_abort_status(u16 ch, u32 stat0, u32 stat2)
{
	if (ch < 8)
		return (((stat0) & (DMA_ABORT_OCCUR << ch)) != 0);
	else
		return (((stat2) & (DMA_ABORT_OCCUR << (ch - 8))) != 0);
}

/**
 * dma_enable_disable_interrupt - Enables or Disables Interrupts .
 * @channel	Channel number
 * @enable	Flag to indicate whether to enable or disable interrupt.
 *
 * Writes the corresponding register to either enable or disable interrupts.
 * The main tasks performed by this function are:
 *	- If enable is DMA_INTERRUPT_ENABLE (1), sets the DMAn Interrupt
 *	  Enable bit in control register2.
 *	- If enable is DMA_INTERRUPT_DISABLE (0), clears the DMAn Interrupt
 *	  Enable bit in control register2.
 */
static void dma_enable_disable_interrupt(u32 channel, s32 enable)
{
	struct pch_dma_regs *reg;
	u16 ch;

	ch = pch_dma_channel_table[channel].channel;
	reg = pch_dma_channel_table[channel].reg;

	if (ch < 8) {
		if (PCH_DMA_INTERRUPT_ENABLE == enable) {
			PCH_DMA_BIT_SET(&reg->dma_ctl2,
					(DMA_INTERRUPT_BIT << ch));
		} else {
			PCH_DMA_BIT_CLEAR(&reg->dma_ctl2,
					  (DMA_INTERRUPT_BIT << ch));
		}

	} else {
		if (PCH_DMA_INTERRUPT_ENABLE == enable) {
			PCH_DMA_BIT_SET(&reg->dma_ctl2,
					(DMA_INTERRUPT_BIT << (ch + 8)));
		} else {
			PCH_DMA_BIT_CLEAR(&reg->dma_ctl2,
					  (DMA_INTERRUPT_BIT << (ch + 8)));
		}
	}
	PCH_DMA_DEBUG("%s  CTL2 Register Value = %x  returns = %d\n",
			__func__, ioread32(&reg->dma_ctl2), 0);
}

/**
 * get_dma_status - Gets the Status of DMA.
 * @channel		Channel number.
 * @dma_status	Address of variable to which status information
 *			is copied.
 *
 * Gets the status of the specified DMA Channel.
 * The main task performed by this function is:
 *	- Reads the data in the DMAn (for channel .n.) Status bit of Status
 *	  register0 (4ch or 8ch) or Status register2 (12ch) and copy the
 *	  value into dma_status.
 */
static void get_dma_status(u32 channel, u16 *dma_status)
{
	u32 status_val;
	struct pch_dma_regs *reg;
	u16 ch;

	ch = pch_dma_channel_table[channel].channel;
	reg = pch_dma_channel_table[channel].reg;

	if (ch < 8) {
		status_val = ioread32(&reg->dma_sts0);
		*dma_status = (u16) ((status_val >>
			(DMA_SHIFT_STATUS_BITS + (ch * DMA_SIZE_STATUS_BITS)))
			& (DMA_MASK_STATUS_BITS));
	} else {
		status_val = ioread32(&reg->dma_sts2);
		*dma_status = (u16) ((status_val >>
		(DMA_SHIFT_STATUS_BITS + ((ch - 8) * DMA_SIZE_STATUS_BITS)))
			& (DMA_MASK_STATUS_BITS));
	}

	PCH_DMA_DEBUG("%s invoked successfully.\n", __func__);
}

/**
 * dma_set_mode - Sets the Mode of transfer for DMA.
 * @channel		The channel for which mode is to be set.
 * @mode_param	Structure which contains the parameters for
 *			the setting of Mode.
 *
 * Does the setting of direction of transfer, access size type and transfer
 * mode. This function does not perform any register write.
 * The main tasks performed by this function are:
 *	- Set the dma_trans_direction field of pch_dma_channel_info with
 *	  the direction of transfer specified.
 *	- Set the dma_access_size field of pch_dma_channel_info with the
 *	  Access Size Type specified.
 *	- Set the dma_trans_mode field of pch_dma_channel_info structure
 *	  with the DMA mode specified.
 */
static void dma_set_mode(u32 channel, struct pch_dma_mode_param mode_param)
{
	pch_dma_channel_info[channel].dma_access_size = mode_param.dma_size_typ;
	pch_dma_channel_info[channel].dma_trans_mode =
	    mode_param.dma_trans_mode;
	pch_dma_channel_info[channel].dma_trans_direction =
	    mode_param.trans_direction;

	PCH_DMA_DEBUG(
		"Function dma_set_mode invoked successfully.\n");
}

/**
 * dma_set_addr - Sets the Inside and Outside address in the case of
 *			ONE SHOT MODE
 * @channel	Channel for which addresses is to be set.
 * @iaddr	Inside address to be set
 * @oaddr	Outside address to be set
 *
 * This function updates Inside address and outside address to be set in
 * ONE SHOT mode. The main tasks performed by this function are:
 *	- Set the field in_addr of the pch_dma_channel_info structure of
 *	  the corresponding channel to the value of the argument iaddr.
 *	- Set the field out_addr of the pch_dma_channle_info structure of
 *	  the corresponding channel to the value of the argument oaddr.
 */
static void dma_set_addr(u32 channel, dma_addr32_t iaddr, dma_addr32_t oaddr)
{
	pch_dma_channel_info[channel].in_addr = iaddr;
	pch_dma_channel_info[channel].out_addr = oaddr;

	PCH_DMA_DEBUG("%s invoked successfully.\n", __func__);
}

/**
 * dma_enable_ch - Enables the DMA channel specified.
 * @channel	Channel number that is to be enabled.
 *
 * This function sets the entire DMA settings such as the transfer direction,
 * transfer mode and enables the channel.
 * The main tasks performed by this function are:
 *	- Sets the transfer direction.
 *	- Sets the transfer mode.
 *	- Enabling the channel.
 */
static void dma_enable_ch(u32 channel)
{
	struct pch_dma_regs *reg;
	u16 transfer_mode;
	u32 ctl0;
	u32 ctrl_val = DEFAULT_CONTROL_REGISTER_VALUE;
	s32 ch;

	/* Marking the channel as enabled. */
	pch_dma_channel_info[channel].ch_enable = 1;

	ch = pch_dma_channel_table[channel].channel;
	reg = pch_dma_channel_table[channel].reg;

	ctl0 = 0;

	/* Setting of transfer direction. */
	if (pch_dma_channel_info[channel].dma_trans_direction ==
	    PCH_DMA_DIR_OUT_TO_IN) {
		ctl0 |= PCH_DMA_DIR_OUT_TO_IN;
	}

	/* Setting the transfer mode features. */
	transfer_mode = pch_dma_channel_info[channel].dma_trans_mode;

	/* If scatter gather mode. */
	if (transfer_mode == DMA_SCATTER_GATHER_MODE) {
		u32 next_desc;

		next_desc = ((u32) pch_dma_channel_info[channel].head_of_list);
		iowrite32(next_desc, &reg->dma[ch].nx_ad);

		ctl0 |= DMA_SCATTER_GATHER_MODE;
	}
	/* If one shot mode. */
	else {
		u32 in_address = pch_dma_channel_info[channel].in_addr;
		u32 out_address = pch_dma_channel_info[channel].out_addr;
		u32 access_size = pch_dma_channel_info[channel].dma_access_size;
		u32 count = pch_dma_channel_info[channel].dma_trans_size;

		ctl0 |= DMA_ONE_SHOT_MODE;

		count |= access_size;
		iowrite32(in_address, &reg->dma[ch].in_ad);
		iowrite32(out_address, &reg->dma[ch].out_ad);
		iowrite32(count, &reg->dma[ch].sz);
	}

	/* Enabling the interrupts. */
	dma_enable_disable_interrupt(channel, PCH_DMA_INTERRUPT_ENABLE);

	/* Updating Control register. */
	if (ch < 8) {
		/* Clearing the three bits corresponding
		   to the mode and transfer direction of
		   specific channel.
		 */
		ctrl_val &= ~((MSK_ALL_THREE) << (ch * DMA_SHIFT_MODE_BITS));

		/* Setting the transfer mode and direction. */
		ctrl_val |= (ctl0 << (ch * DMA_SHIFT_MODE_BITS));

		/* Updating to the register. */
		iowrite32(ctrl_val, &reg->dma_ctl0);

		PCH_DMA_DEBUG(
			"%s -> Control register(0) value: %x.\n",
			 __func__, ioread32(&reg->dma_ctl0));
	} else {
		/* Clearing the three bits corresponding
		   to the mode and transfer direction of
		   specific channel.
		 */
		ctrl_val &=
		    ~((MSK_ALL_THREE) << ((ch - 8) * DMA_SHIFT_MODE_BITS));

		/* Setting the transfer mode and direction. */
		ctrl_val |= (ctl0 << ((ch - 8) * DMA_SHIFT_MODE_BITS));

		/* Updating to the register. */
		iowrite32(ctrl_val, &reg->dma_ctl3);

		PCH_DMA_DEBUG(
			"%s -> Control register(3) value: %x.\n",
			__func__, ioread32(&reg->dma_ctl3));
	}
	PCH_DMA_DEBUG("%s invoked successfully\n", __func__);
}

/**
 * dma_disable_ch - Disables the DMA channel specified.
 * @channel	Channel to be disabled.
 *
 * This function performs the necessary register updation in-order to
 * disable the DMA channel.
 */
static void dma_disable_ch(u32 channel)
{
	struct pch_dma_regs *reg;
	u16 ch;

	ch = pch_dma_channel_table[channel].channel;
	reg = pch_dma_channel_table[channel].reg;

	if (channel < 8) {
		/* Clearing the mode bits of the channel */
		PCH_DMA_BIT_CLEAR(&reg->dma_ctl0,
			(DMA_MASK_MODE_BITS << (ch * DMA_SHIFT_MODE_BITS)));
	} else {
		/* Clearing the mode bits of the channel */
		PCH_DMA_BIT_CLEAR(&reg->dma_ctl3,
			(DMA_MASK_MODE_BITS << ((ch - 8)
				* DMA_SHIFT_MODE_BITS)));
	}

	/* Updating the enable variable. */
	pch_dma_channel_info[channel].ch_enable = (u16) 0;

	PCH_DMA_DEBUG("%s invoked successfully\n", __func__);
}

/**
 * dma_set_count - Sets the count value .
 * @channel	Channel number for which value is to be set
 * @count	Transfer Size value.
 *
 * Updates the transfer size for ONE_SHOT_MODE of DMA Transfer.
 * The main tasks performed by	this function are:
 *	- Set the dma_trans_size field of the pch_dma_channel_info
 *	  structure to the value of the argument count.
 */
static void dma_set_count(u32 channel, u32 count)
{
	pch_dma_channel_info[channel].dma_trans_size = count;
	PCH_DMA_DEBUG("%s invoked successfully\n", __func__);
}

/**
 * dma_add_desc - Adds descriptors to the existing list of descriptors.
 * @channel	Channel number.
 * @start	Reference to first descriptor of list.
 * @end	Reference to last descriptor of list.
 *
 * This function accepts the descriptor list and appends it to the existing
 * list of descriptors. The main tasks performed by this function are:
 *	- Obtains the virtual address of the end of the currently set
 *	  descriptor list. If it is not successful returns with an error.
 *	- Appends the value of the argument start to the next_desc field of
 *	  the descriptor pointed by the tail_of_list field of the
 *	  pch_dma_channel_info structure with the value of the argument
 *	  start after appropriately setting the last two bits to denote
 *	  Follow_Next_Descriptor_Without_Interrupt.
 *	- Updates the value of the argument end to the tail_of_list field of
 *	  the pch_dma_channel_info structure for the corresponding channel.
 */
static void dma_add_desc(u32 channel, struct pch_dma_desc *start,
		 struct pch_dma_desc *end)
{
	struct pch_dma_desc *desc_addr;

	desc_addr = pch_dma_channel_info[channel].tail_of_list;

	/* Obtaining the virtual address. */
	desc_addr = (struct pch_dma_desc *) phys_to_virt((u32) desc_addr);

	/* If virtual address calculation successful. */
	desc_addr->next_desc = (u32) start;
	pch_dma_channel_info[channel].tail_of_list = end;

	PCH_DMA_DEBUG("%s invoked successfully\n", __func__);
}

/**
 * dma_set_callback  - To set callback function.
 * @channel		Channel number.
 * @pch_dma_cbr	Function pointer to call back function.
 * @data		The data to be passed to the callback function
 *			during invoking.
 *
 * Sets the callback function to be called for a channel.
 * The main task performed by this function is:
 *	- Updates the callback pointer for the channel in the structure
 *	  pch_dma_channel_info with the parameter passed.
 */
static void dma_set_callback(u32 channel,
		      void (*pch_dma_cbr) (int value, unsigned long data1),
		      u32 data)
{
	pch_dma_channel_info[channel].call_back_func_ptr = pch_dma_cbr;
	pch_dma_channel_info[channel].callback_data = data;

	PCH_DMA_DEBUG("%s invoked successfully.\n", __func__);
}

/**
 * dma_interrupt - Interrupt handler.
 * @irq	Interrupt Request number
 * @dev_id	dev_id of device for which interrupt is raised .
 *
 * Return codes
 *	IRQ_HANDLED:	If interrupt has been processed.
 *	IRQ_NONE:	If no interrupt has been processed.
 *
 * Handles the interrupt for the DMA. The main tasks performed by this
 * function are:
 *	- Checks each DMA channels whether a DMA transmission end or DMA
 *	  status interrupt has occurred.
 *	- If a transmission end interrupt has occurred, then invoke the
 *	  callback function with PCH_DMA_END, denoting that the DMA
 *	  transmission has end.
 *	- If a DMA abort interrupt has occurred, then invoke the callback
 *	  function with PCH_DMA_ABORT, denoting that a DMA abort has
 *	  occurred.
 */
static irqreturn_t dma_interrupt(s32 irq, void *dev_id)
{
	irqreturn_t retval = IRQ_NONE;
	u32 status_reg0;
	u32 status_reg2;
	struct pch_dma_regs *reg;
	u32 dev_type;
	u32 i;
	u16 status;

	reg = (struct pch_dma_regs __iomem *)
		(((struct pch_dma_devices *) dev_id)->base_addr);
	dev_type = ((struct pch_dma_devices *) dev_id)->dev_typ;

	/* Reading the status registers. */
	status_reg0 = ioread32(&reg->dma_sts0);
	status_reg2 = ioread32(&reg->dma_sts2);
	PCH_DMA_DEBUG(
		"%s -> Status register STS0: %x STS2: "
		  "%x.\n", __func__, status_reg0, status_reg2);

	/* Clearing the interrupts. */
	iowrite32(status_reg0, &reg->dma_sts0);
	iowrite32(status_reg2, &reg->dma_sts2);

	/* Handling the interrupts. */
	for (i = 0; i < PCH_DMA_CHANNELS_MAX; i++) {
		if ((pch_dma_channel_table[i].dma_dev_id == dev_type) &&
		    (pch_dma_channel_table[i].ch_alloced == 1) &&
		    (pch_dma_channel_info[i].ch_enable == 1)
		    ) {
			status =
			    dma_get_interrupt_status(pch_dma_channel_table
						     [i].channel, status_reg0,
						     status_reg2);
			PCH_DMA_DEBUG(
			    "%s -> Interrupt status for ch: %d is "
			    "%x.\n", __func__, i, status);

			if (status == 1) {
				s32 value = PCH_DMA_END;

				status =
				    dma_get_abort_status(pch_dma_channel_table
							 [i].channel,
							 status_reg0,
							 status_reg2);

				if (status == 1) {
					value = PCH_DMA_ABORT;

					PCH_DMA_DEBUG(
					    "%s -> DMA Abort "
					    "interrupt from channel%d.\n",
						__func__, i);
				}
#ifdef DEBUG
				else {
					PCH_DMA_DEBUG(
					    "%s -> DMA Completion "
					    "interrupt from channel%d.\n",
						__func__, i);
				}
#endif
				if (pch_dma_channel_info[i].
				    call_back_func_ptr) {
					u32 data =
					    pch_dma_channel_info
					    [i].callback_data;
					(pch_dma_channel_info
					 [i].call_back_func_ptr) (value, data);
				}

				/* Determining whether the channel has been
				disabled. */
				{
					u32 ctrl_val;
					s32 ch =
					    pch_dma_channel_table[i].channel;
					if (ch < 8) {
						ctrl_val =
						ioread32(&reg->dma_ctl0);

						ctrl_val &=
						    ((0x3) <<
						    (ch * DMA_SHIFT_MODE_BITS));
					} else {
						ctrl_val =
						ioread32(&reg->dma_ctl3);
						ctrl_val &=
						    ((0x3) <<
						     ((ch - 8) *
						      DMA_SHIFT_MODE_BITS));
					}

					pch_dma_channel_info[i].ch_enable =
					    (ctrl_val != 0) ? 1 : 0;

				}	/* End */

				retval = IRQ_HANDLED;
			}
		}
	}

	PCH_DMA_DEBUG(
		"%s returns %d.\n", __func__, retval);
	return retval;
}

/**
 * dma_direct_start - This function is used to initiate the DMA transfer process.
 * @channel	Channel number for which DMA transfer is to be started.
 */
static void dma_direct_start(u32 channel)
{
	s32 ch;
	struct pch_dma_regs *reg;

	ch = pch_dma_channel_table[channel].channel;
	reg = pch_dma_channel_table[channel].reg;

	if (ch < 8) {
		PCH_DMA_BIT_SET(&reg->dma_ctl2,
				(DMA_DIR_START << ch));
	} else {
		PCH_DMA_BIT_SET(&reg->dma_ctl2,
				(DMA_DIR_START << (ch + 6)));
	}

	PCH_DMA_DEBUG(
		"Function dma_direct_start  Direct2 RegValue = %x\n",
		ioread32(&reg->dma_ctl2));
}

/**
 * dma_set_priority - Set the priority.
 * @channel	DMA channel number.
 * @priority	Priority to be set for the DMA channel.
 *
 * Sets the priority for a channel.
 * The main task performed by this function is:
 *	- Set the value of DMAn Priority Level bits for the channel in
 *	  the Control register1.
 */
static void dma_set_priority(u32 channel, s32 priority)
{
	s32 ch;
	struct pch_dma_regs *reg;
	u32 reg_val;

	ch = pch_dma_channel_table[channel].channel;
	reg = pch_dma_channel_table[channel].reg;

	reg_val = ioread32(&reg->dma_ctl1);

	if (ch < 8) {
		reg_val &=
		    ~(DMA_MASK_PRIORITY_BITS << (ch * DMA_SHIFT_PRIORITY_BITS));
		reg_val |= (((u32) priority) << (ch * DMA_SHIFT_PRIORITY_BITS));
	} else {
		reg_val &=
		    ~(DMA_MASK_PRIORITY_BITS <<
		      (((ch - 8) * DMA_SHIFT_PRIORITY_BITS) + 2));
		reg_val |=
		    (((u32) priority) <<
		     (((ch - 8) * DMA_SHIFT_PRIORITY_BITS) + 2));
	}
	iowrite32(reg_val, &reg->dma_ctl1);

	PCH_DMA_DEBUG(
		"%s invoked successfully.\n", __func__);
}

/**
 * dma_set_desc - Sets descriptors .
 * @channel	Channel number.
 * @start	Reference to first descriptor of list.
 * @end	Reference to last descriptor of list.
 *
 * This functions sets the descriptor settings for SCATTER GATHER mode.
 * It does not perform any register settings, instead retains the data for
 * further use. The main tasks performed by this function are:
 *	- Sets the head_of_list field of the pch_dma_channel_info structure
 *	  to the value of the argument start.
 *	- Set the tail_of_list field of the pch_dma_channel_info structure
 *	  to the value of the argument end.
 */

static void dma_set_desc(u32 channel, struct pch_dma_desc *start,
		 struct pch_dma_desc *end)
{
	pch_dma_channel_info[channel].head_of_list = start;
	pch_dma_channel_info[channel].tail_of_list = end;

	PCH_DMA_DEBUG(
		"%s invoked successfully.\n", __func__);
}

/**
 * get_free_ch - Get a free channel info entry and populate the entry.
 * @index	Index in the pch_dma_channel_table
 *
 * Reset all the entries within the array pch_dma_channel_info[index]
 */
static void get_free_ch(u32 index)
{
	memset((void *)&pch_dma_channel_info[index], 0,
	       sizeof(struct pch_dma_controller_info));
	PCH_DMA_DEBUG(
		"%s invoked successfully.\n", __func__);
}

/**
 * dma_request_ch - Reserves a channel based on request.
 * @req_dev_id	Device id of the device that requests DMA  .
 * @dreq	DMA request signal number.
 *
 * Return codes
 *	DMA channel number (>=0):	Success.
 *	-EBUSY:			DMA channel cannot be allocated.
 *
 * This function is invoked when a kernel module requests to reserve a DMA
 * channel. The main tasks performed by this function are:
 *	- Checks the @ref pch_dma_channel_table for a matching entry
 *	  corresponding to the dev_id of the requesting device and dreq
 *	  signal.
 *	- If there is a matching entry, checks if this channel is already
 *	  allocated.
 *	- If no invoke get_free_ch to reset the entries for the
 *	  corresponding channel and return the entry index.
 *	- If no matching entry is found return -EBUSY.
 */
static s32 dma_request_ch(u32 req_dev_id, s32 dreq)
{
	s32 retval;
	u32 i;

	for (i = 0; i < PCH_DMA_CHANNELS_MAX; i++) {
		if ((pch_dma_channel_table[i].req_device_id == req_dev_id) &&
		    (pch_dma_channel_table[i].request_signal == dreq)) {
			if ((1 == pch_dma_channel_table[i].ch_found) &&
			    (0 == pch_dma_channel_table[i].ch_alloced)) {
				get_free_ch(i);
				PCH_DMA_DEBUG(
				   "%s -> Function get_free_ch.\n", __func__);
				pch_dma_channel_table[i].ch_alloced = 1;
				retval = i;

				break;
			}
		}
	}

	if (PCH_DMA_CHANNELS_MAX == i) {
		retval = -EBUSY;
		printk(KERN_ERR MODULE_NAME ": "
			"%s ->  Not able to allocate "
			"channel.\n", __func__);
	}

	PCH_DMA_DEBUG(
		"%s returns %d.\n", __func__, retval);
	return retval;
}

/**
 * dma_free_ch - Frees the requested channel.
 * @channel	DMA channel number to be freed.
 *
 * This function is invoked when a kernel module requests to free a DMA
 * channel. The main tasks performed by this function are:
 *	- If the channel is already free return "0".
 *	- Else disable the channel by invoking dma_disable_ch API.
 *	- Disable the channel interrupt by invoking
 *	  dma_enable_disable_interrupt
 *	- Mark the channel as free in the structures
 *	  pch_dma_channel_info and pch_dma_channel_table and return "0".
 */
static void dma_free_ch(u32 channel)
{
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		PCH_DMA_DEBUG(
			"%s -> Channel is already free\n", __func__);
	} else {
		/* To stop any active transfer on DMA, disable DMA */
		dma_disable_ch(channel);
		PCH_DMA_DEBUG(
			"%s -> Function dma_disable_ch.\n", __func__);

		dma_enable_disable_interrupt(channel,
						   PCH_DMA_INTERRUPT_DISABLE);
		PCH_DMA_DEBUG(
		    "%s -> Function dma_enable_disable_interrupt "
		     "invoked successfully.\n", __func__);

		pch_dma_channel_table[channel].ch_alloced = 0;
	}
	PCH_DMA_DEBUG("%s invoked successfully.\n", __func__);
}

/**
 * dma_init - Initializes local data structures for the DMAC device.
 * @base_addr	The base address for register access.
 * @dev_type	The type of the device.
 *
 * This function is called when a DMA device is detected.
 * It initializes the data structures associated with the obtained device.
 * The main tasks performed by this function are:
 *	- Waits until the status of a DMA channel becomes idle and then
 *	  disables it.
 *	- Initializes the data structures that can be used further.
 */
static void __init dma_init(u32 base_addr, u32 dev_type)
{
	u32 i;
	u32 counter;
	u16 DMAStatus;

	for (i = 0; i < PCH_DMA_CHANNELS_MAX; i++) {
		if (pch_dma_channel_table[i].dma_dev_id == dev_type) {
			counter = COUNTER_LIMIT;

			pch_dma_channel_table[i].ch_found = 1;
			pch_dma_channel_table[i].ch_alloced = 0;
			pch_dma_channel_table[i].reg =
				(struct pch_dma_regs __iomem *)base_addr;

			do {
				get_dma_status(i, &DMAStatus);
			} while ((counter--) && (DMAStatus != DMA_STATUS_IDLE));

			dma_disable_ch(i);
			PCH_DMA_DEBUG(
			"%s -> Channel %d disabled.\n", __func__, i);

			dma_enable_disable_interrupt
			    (i, PCH_DMA_INTERRUPT_DISABLE);
			PCH_DMA_DEBUG(
			    "%s -> Interrupt disabled for channel %d.\n",
				__func__, i);
		}
	}

	PCH_DMA_DEBUG(
		"%s invoked successfully.\n", __func__);
}

/**
 * dma_exit - De-initializes the DMA device.
 * @dev_type	The type of the device.
 *
 * The main tasks performed by this function are:
 *	- Waits for a small interval for each channel if the channel is not
 *	  idle so that it can complete its transfer.
 *	- Disables the channel.
 *	- Disables the concerned interrupt.
 */
static void dma_exit(u32 dev_type)
{
	u32 i;
	u32 counter;
	u16 DMAStatus;

	for (i = 0; i < PCH_DMA_CHANNELS_MAX; i++) {
		if (pch_dma_channel_table[i].dma_dev_id == dev_type &&
		    pch_dma_channel_table[i].ch_found == 1) {
			counter = COUNTER_LIMIT;
			get_dma_status(i, &DMAStatus);

			while ((counter > 0) &&
			       (DMAStatus != DMA_STATUS_IDLE)) {
				counter--;
				get_dma_status(i, &DMAStatus);
			}

			dma_disable_ch(i);
			PCH_DMA_DEBUG(
			"%s -> Channel %d disabled.\n", __func__, i);

			dma_enable_disable_interrupt
			    (i, PCH_DMA_INTERRUPT_DISABLE);
			PCH_DMA_DEBUG(
			"%s -> Interrupt disabled for channel "
				  "%d.\n", __func__, i);
		}
	}

	PCH_DMA_DEBUG(
		"%s invoked successfully.\n", __func__);
}

/**
 * pch_request_dma - Used to request a DMA channel.
 * @dev	PCI device that requires the DMA channel.
 * @dreq	DMA request signal number.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	Device is in suspend mode.
 *	-EINVAL:	pdev does not have a DMA request type or number
 *			'dreq' or 'pdev' is NULL.
 *
 * Requests to reserve a DMA channel that connects to number 'dreq'
 * (DMA request signal) of PCI device 'pdev' to the appropriate DMA channel
 * allocated for it within the DMA Controller. This	function is called by
 * functions from other	kernel modules.
 * The tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API and returns the status code
 *	  returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_request_dma(struct pci_dev *pdev, s32 dreq)
{
	s32 retval;

	/* Attaining the lock.  */
	spin_lock(&pch_dma_lock);

	/* If device suspended. */
	if (1 == pch_dma_suspended) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Device is in suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_request_dma;
	}
	/* Invalid device structure. */
	if (NULL == pdev) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Obtained device structure "
			"is NULL.\n", __func__);
		retval = -EINVAL;
		goto err_request_dma;
	}
	/* Invalid request signal. */
	if ((dreq < PCH_DMA_TX_DATA_REQ0) ||
		 (dreq > PCH_DMA_RX_DATA_REQ5)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid request signal.\n", __func__);
		retval = -EINVAL;
		goto err_request_dma;
	}
	/* Requesting for reserving a DMA channel. */
	retval = dma_request_ch((u32) (pdev->device), dreq);
	PCH_DMA_DEBUG("%s -> Function dma_request_ch returned "
			"%d.\n", __func__, retval);

err_request_dma:
	/* Releasing the lock. */
	spin_unlock(&pch_dma_lock);

	PCH_DMA_DEBUG(
	"%s returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_request_dma);

/**
 * pch_free_dma - Used to free a DMA channel.
 * @channel	DMA channel number.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	Device is in suspend mode.
 *	-ENODEV:	Specified DMA channel does not exist.
 *
 * Frees the allocated DMA channel that is provided	as the argument to the
 * function. This function is called by the functions from other kernel
 * modules. The main tasks performed by this	function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API for freeing the channel and
 *	  returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_free_dma(s32 channel)
{
	s32 retval;

	if (1 == pch_dma_suspended) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Device is in suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_free_dma;
	}
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel number: "
			"%d.\n", __func__, channel);
		retval = -ENODEV;
		goto err_free_dma;
	}
	dma_free_ch((u32)channel);
	PCH_DMA_DEBUG("%s -> Function dma_free_ch.\n", __func__);
	retval = 0;
err_free_dma:
	PCH_DMA_DEBUG("%s returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_free_dma);

/**
 * pch_set_dma_mode - Used to set the mode of the DMA.
 * @channel		DMA channel number
 * @mode_param	Contains info about direction of DMA transfer,
 *			mode and Size type
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	The device is in suspend mode.
 *	-ENODEV:	Specified DMA channel does not exist.
 *	-EINVAL:	Parameter passed is invalid.
 *	-EBUSY:	DMA channel is already enabled.
 *
 * Sets the mode of DMA transfer - One shot mode or Scatter/gather mode.
 * In addition to this,	the function also sets the direction of DMA transfer
 * and DMA Size type. This function is called by functions from other kernel
 * modules. The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to set the required settings
 *	  and returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_set_dma_mode(s32 channel, struct pch_dma_mode_param mode_param)
{
	s32 retval;

	/* Checking if device suspended.                                */
	if (1 == pch_dma_suspended) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Device is in suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_set_dma_mode;
	}
	/* Checking for validity of channel number.     */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel number : " "%d.\n",
			__func__, channel);
		retval = -ENODEV;
		goto err_set_dma_mode;
	}
	/* Checking whether channel not allocated.              */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not allocated.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_mode;
	}
	/* Checking if channel already enabled.                 */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel already enabled.\n", __func__);
		retval = -EBUSY;
		goto err_set_dma_mode;
	}
	/* Checking for validity of DMA Transfer MODE.  */
	if ((mode_param.dma_trans_mode != (u16) DMA_ONE_SHOT_MODE) &&
		 (mode_param.dma_trans_mode !=
		  (u16) DMA_SCATTER_GATHER_MODE)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid DMA Transfer mode.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_mode;
	}
	/* Checking for validity of Transfer Direction. */
	if ((mode_param.trans_direction != (u16) PCH_DMA_DIR_OUT_TO_IN)
		 && (mode_param.trans_direction !=
		     (u16) PCH_DMA_DIR_IN_TO_OUT)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid DMA Transfer Direction." \
			"\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_mode;
	}
	/* Checking for validity of Transfer Size Type. */
	if ((mode_param.dma_size_typ != (u16) PCH_DMA_SIZE_TYPE_8BIT) &&
		 (mode_param.dma_size_typ != (u16) PCH_DMA_SIZE_TYPE_16BIT) &&
		 (mode_param.dma_size_typ != (u16) PCH_DMA_SIZE_TYPE_32BIT)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid DMA Size Type.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_mode;
	}
	/* Setting the required DMA mode. */
	dma_set_mode((u32)channel, mode_param);
	PCH_DMA_DEBUG(
		"%s -> Function dma_set_mode.\n", __func__);
	retval = 0;

err_set_dma_mode:
	PCH_DMA_DEBUG(
		"F%s returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_set_dma_mode);

/**
 * pch_set_dma_addr - Used to set the in and out address of the DMA channel.
 * @channel	DMA channel number .
 * @iaddr	Address of inside bridge.
 * @oaddr	Address of outside bridge.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	The device is in suspend mode.
 *	-ENODEV:	Specified DMA channel does not exist.
 *	-EINVAL:	Parameter passed is invalid.
 *	-EBUSY:	DMA transfer in progress or channel is already enabled.
 *
 * Sets the address of the inside bridge and the outside bridge for the
 * 'One Shot Mode' of DMA Transfer. This function is invoked by functions
 * from other modules. The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to set the inside and outside
 *	  address and returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules. The following points
 * has to be noted while passing the in-address and out-address paramter.
 *	- The address passed should be valid physical address within the
 *	  memory space.
 *	- It should not be a configuration space or IO space address.
 *	- If the transfer is for large data, the address should point to
 *	  contagious alligned memory space.
 */
s32 pch_set_dma_addr(s32 channel, dma_addr32_t iaddr, dma_addr32_t oaddr)
{
	s32 retval;

	/* If the device is in suspend mode. */
	if (1 == pch_dma_suspended) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Device is in suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_set_dma_addr;
	}
	/* Checking for validity of channel number  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Invalid Channel number: %d.\n", __func__, channel);
		retval = -ENODEV;
		goto err_set_dma_addr;
	}
	/* Checking whether channel is not allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Channel not allocated.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_addr;
	}
	/* Checking whether the channel is already enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Channel already enabled.\n", __func__);
		retval = -EBUSY;
		goto err_set_dma_addr;
	}
	/*Checking if addresses specified are NULL or not */
	if ((iaddr == 0) || (oaddr == 0)) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Invalid address.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_addr;
	}
	/* Checking if the mode of transfer is  other than ONE_SHOT. */
	if (pch_dma_channel_info[channel].dma_trans_mode !=
		 (u16) DMA_ONE_SHOT_MODE) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Current Mode is not DMA_ONE_SHOT_MODE.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_addr;
	}
	/* setting the in and out address. */
	dma_set_addr((u32)channel, iaddr, oaddr);
	PCH_DMA_DEBUG(
		"%s -> Function dma_set_addr.\n", __func__);
	retval = 0;

err_set_dma_addr:
	PCH_DMA_DEBUG(
	"%s returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_set_dma_addr);

/**
 * pch_set_dma_count - Used to set the DMA transfer count for a DMA channel.
 * @channel	DMA channel number.
 * @count	The number of bytes to transfer.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	The device is in suspend mode.
 *	-ENODEV:	Specified DMA channel does not exist.
 *	-EBUSY:	DMA transfer in progress or channel is already enabled.
 *	-EINVAL:	Parameter passed is invalid.
 *
 * Sets the value of DMA transfer count. This function sets the count value
 * only for the 'One Shot Mode' of DMA Transfer. This function is invoked by
 * functions from other modules.
 * The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to set the access count
 *	  settings and returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_set_dma_count(s32 channel, u32 count)
{
	s32 retval;
	u32 max_count;

	/* Checking if the device is in suspend mode. */
	if (1 == pch_dma_suspended) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> The device is in suspend mode.", __func__);
		retval = -EAGAIN;
		goto err_set_dma_count;
	}
	/* Checking for validity of channel number.  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Invalid Channel number : %d.\n", __func__, channel);
		retval = -ENODEV;
		goto err_set_dma_count;
	}
	/* Checking whether channel is not allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Channel is not allocated.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_count;
	}
	/* Checking whether the channel is enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
		"%s -> Channel already enabled.\n", __func__);
		retval = -EBUSY;
		goto err_set_dma_count;
	}
	/* Checking if the mode of transfer is other than ONE_SHOT. */
	if (pch_dma_channel_info[channel].dma_trans_mode !=
		 (u16) DMA_ONE_SHOT_MODE) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Current Mode is "
			"not DMA_ONE_SHOT_MODE.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_count;
	}
	/* Checking the limits of count value. */
	switch (pch_dma_channel_info[channel].dma_access_size) {
	case PCH_DMA_SIZE_TYPE_8BIT:
		max_count = PCH_DMA_8BIT_COUNT_MAX;
		break;
	case PCH_DMA_SIZE_TYPE_16BIT:
		max_count = PCH_DMA_16BIT_COUNT_MAX;
		break;
	case PCH_DMA_SIZE_TYPE_32BIT:
		max_count = PCH_DMA_32BIT_COUNT_MAX;
		break;
	default:
		printk(KERN_ERR MODULE_NAME ": "
		 "%s -> Invalid Access Size.\n", __func__);
		max_count = 0;
		retval = -EINVAL;
		goto err_set_dma_count;
	}

	if (max_count < count) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Count (%d) exceeds "
			"limit the maximum expected count (%d).\n",
			__func__, count, max_count);
		retval = -EINVAL;
		goto err_set_dma_count;
	}
	/* Setting the count. */
	dma_set_count((u32)channel, count);
	PCH_DMA_DEBUG(
		    "%s -> Function dma_set_count.\n", __func__);
	retval = 0;

err_set_dma_count:
	PCH_DMA_DEBUG(
		"%s returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_set_dma_count);

/**
 * pch_set_dma_desc - Used to set the DMA channel descriptors.
 * @channel	DMA channel number
 * @start	A pointer to the first descriptor.
 * @end	A pointer to the last descriptor.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	The device is in suspend.
 *	-EINVAL:	For invalid parameters.
 *	-ENODEV:	Specified DMA channel is not exist.
 *	-EBUSY:	If DMA transfer is in progress or channel is already
 *			enabled.
 * Sets the DMA descriptor for the 'Scatter/Gather mode' of DMA transfer.
 * This function is invoked by	functions from other kernel modules.
 * The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not suitable
 *	  error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to set the descriptor settings and
 *	  returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules. The following points
 * have to be noted while passing the "start" and "end" pointer of the
 * descriptor.
 *	- The address pointed by them should be physical address with valid
 *	  virtual address.
 *	- The space should be alligned and accessible by the DMA hardware.
 *	- An easy way to perform this is to allocate the descriptor memory
 *	  using kmalloc.
 *	- The last two bits of the physical address should be suitably set
 *	  so as to perform suitable action after completion of each
 *	  descriptor action.
 *	- The in-address and out-address within each descriptor should be
 *	  a valid memory space physical address.
 */
s32 pch_set_dma_desc(s32 channel, struct pch_dma_desc *start,
		     struct pch_dma_desc *end)
{
	s32 retval;

	/* Checking if the device is in suspend mode. */
	if (1 == pch_dma_suspended) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> The device is in "
			"suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_set_dma_desc;
	}
	/* Checking for validity of channel number  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel number "
			": %d.\n", __func__, channel);
		retval = -ENODEV;
		goto err_set_dma_desc;
	}
	/* Checking whether channel is not allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not allocated.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_desc;
	}
	/* Checking whether the channel is enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel already enabled.\n", __func__);
		retval = -EBUSY;
		goto err_set_dma_desc;
	}
	/* Checking if the mode is other than SCATTER_GATHER. */
	if (pch_dma_channel_info[channel].dma_trans_mode !=
		 (u16) DMA_SCATTER_GATHER_MODE) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Current mode id is not "
			"SCATTER GATHER.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_desc;
	}
	/* Checking whether start and end pointers are NULL or not */
	if ((start == NULL) || (end == NULL)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> NULL pointer parameter.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_desc;
	}
	/* Setting the descriptors. */
	dma_set_desc((u32)channel, start, end);
	PCH_DMA_DEBUG(
		"%s -> Function dma_set_desc.\n", __func__);
	retval = 0;

err_set_dma_desc:
	PCH_DMA_DEBUG(
		"%s returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_set_dma_desc);

/**
 * pch_add_dma_desc - Used to append the DMA descriptors for a channel.
 * @channel	DMA channel number
 * @start	A pointer to the first descriptor.
 * @end	A pointer to the last descriptor.
 *
 * Return codes
 *	0:		success.
 *	-EAGAIN:	The device is in suspend mode.
 *	-ENODEV:	Specified DMA channel does not exist.
 *	-EINVAL:	Invalid parameters passed.
 *	-EBUSY:	If DMA Transfer in progress or channel is already enabled.
 *
 * Used when a new chain of descriptors is to be appended to the existing
 * chain of descriptors. This function is invoked by functions from other
 * modules. The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to append the descriptor
 *	  settings and returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 * The following points have to be noted while passing the "start" and "end"
 * pointer of the descriptor.
 *	- The address pointer by them should be physical address with valid
 *	  virtual address.
 *	- The space should be alligned and accessible by the DMA hardware.
 *	- An easy way to perform this is to allocate the descriptor memory
 *	  using kmalloc.
 *	- The last two bits of the physical address should be suitably set
 *	  so as to perform suitable	action after completion of each
 *	  descriptor action.
 *	- The in-address and out-address within each descriptor should be
 *	  a valid memory space physical address.
 */
s32 pch_add_dma_desc(s32 channel, struct pch_dma_desc *start,
		     struct pch_dma_desc *end)
{
	s32 retval;

	/* Checking whether the device is in suspend mode. */
	if (1 == pch_dma_suspended) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> The device is in suspend "
			"mode.\n", __func__);
		retval = -EAGAIN;
		goto err_add_dma_desc;
	}
	/* Checking for validity of channel number  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel "
			"number : %d", __func__, channel);
		retval = -ENODEV;
		goto err_add_dma_desc;
	}
	/* Checking whether channel is not allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not alloctaed.\n", __func__);
		retval = -EINVAL;
		goto err_add_dma_desc;
	}
	/* Checking whether the channel is enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel already enabled.\n", __func__);
		retval = -EBUSY;
		goto err_add_dma_desc;
	}
	/* Checking whether the mode is other than SCATTER_GATHER. */
	if (pch_dma_channel_info[channel].dma_trans_mode !=
		 (u16) DMA_SCATTER_GATHER_MODE) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Current mode id is not "
			"SCATTER_GATHER.\n", __func__);
		retval = -EINVAL;
		goto err_add_dma_desc;
	}
	/* Checking if descriptor field of the channel is set earlier. */
	if ((pch_dma_channel_info[channel].head_of_list == NULL) ||
		 (pch_dma_channel_info[channel].tail_of_list == NULL)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Descriptor list not "
			"set earlier.\n", __func__);
		retval = -EINVAL;
		goto err_add_dma_desc;
	}
	/* Checking whether start and end pointers are NULL or not */
	if ((start == NULL) || (end == NULL)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> NULL pointer parameter.\n", __func__);
		retval = -EINVAL;
		goto err_add_dma_desc;
	}
	/* Appending the descriptors to the available list. */
	dma_add_desc((u32)channel, start, end);
	PCH_DMA_DEBUG(
	    "%s -> Function dma_add_desc.\n", __func__);
	retval = 0;

err_add_dma_desc:
	PCH_DMA_DEBUG(
		"%s returns %d.\n", __func__, retval);
	return retval;
}

/**
 * pch_enable_dma - Used to enable a DMA channel.
 * @channel	DMA channel number .
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	Device is in suspend mode.
 *	-ENODEV:	Specified DMA channel does not exist.
 *	-EINVAL:	Specified channel is not allocated.
 *	-EBUSY:	DMA Transfer already in progress or channel is
 *			already enabled.
 *
 * Used when a DMA channel has to be enabled.
 * This function is invoked by functions from other kernel modules.
 * The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to enable the channel and
 *	  returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_enable_dma(s32 channel)
{
	s32 retval;

	/* Checking whether the device is in suspend mode. */
	if (pch_dma_suspended == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Device is in suspend "
			"mode.\n", __func__);
		retval = -EAGAIN;
		goto err_enable_dma;
	}
	/* Checking for validity of channel number  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s ->Invalid Channel number "
			": %d.\n", __func__, channel);
		retval = -ENODEV;
		goto err_enable_dma;
	}
	/* Checking whether channel is allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not allocated.\n", __func__);
		retval = -EINVAL;
		goto err_enable_dma;
	}
	/* Checking whether the channel is already enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel already enabled.\n", __func__);
		retval = -EBUSY;
		goto err_enable_dma;
	}
	/* Enabling the channel. */
	dma_enable_ch((u32)channel);
	PCH_DMA_DEBUG(
		"%s -> Function dma_enable_ch.\n", __func__);
	retval = 0;

err_enable_dma:
	PCH_DMA_DEBUG(
		"%s returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_enable_dma);

/**
 * pch_disable_dma - Used to disable a DMA channel.
 * @channel	DMA channel number .
 *
 * Return codes
 *	0:		Success
 *	-ENODEV:	Specified DMA channel does not exist.
 *	-EINVAL:	Specified channel is not allocated.
 *
 * Used when a DMA channel has to be disabled.
 * This function is invoked by functions from other kernel modules.
 * The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to disable the channel and
 *	  returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_disable_dma(s32 channel)
{
	s32 retval;
	u16 statusInfo;

	/* Checking whether the device is in suspend mode. */
	if (pch_dma_suspended == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Device is in "
			"suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_disable_dma;
	}
	/* Checking for validity of channel number.  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel "
			"number : %d", __func__, channel);
		retval = -ENODEV;
		goto err_disable_dma;
	}
	/* Checking whether channel is allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not "
			"allocated.\n", __func__);
		retval = -EINVAL;
		goto err_disable_dma;
	}
	/* Check whether channel is already disabled. */
	if (pch_dma_channel_info[channel].ch_enable == (u16) 0) {
		retval = 0;
	} else {
		u32 counter = COUNTER_LIMIT;

		/* Wait for any DMA for certain interval transfer to end
		   before disabling the channel */
		do {
			get_dma_status((u32)channel, &statusInfo);
		} while ((counter--) && (statusInfo != (u16) DMA_STATUS_IDLE));

		/* Disabling the channel. */
		dma_disable_ch((u32)channel);
		PCH_DMA_DEBUG(
			"%s -> Function dma_disable_ch.\n", __func__);
		retval = 0;
	}

err_disable_dma:
	PCH_DMA_DEBUG(
		"%s returns " "%d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_disable_dma);

/**
 * pch_dma_set_callback
 *	- Used to set the callback function for particular DMA channel.
 * @channel	DMA channel number .
 * @pch_dma_cbr	Pointer to the call-back function.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN	Device is in suspend mode.
 *	-EINVAL	Parameter passed is invalid.
 *	-ENODEV	Specified DMA channel does not exist.
 *	-EBUSY		If the channel is already enabled.
 *
 * Sets the callback function to be called when an interrupt occurs.
 * This function is invoked by	functions from other kernel modules.
 * The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to set the callback function
 *	  settings and returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_dma_set_callback(s32 channel,
			 void (*pch_dma_cbr) (int value, unsigned long data1),
			 u32 data)
{
	s32 retval;

	/* Checking whether the device is in suspend mode. */
	if (pch_dma_suspended == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> The device is "
			"in suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_dma_set_callback;
	}
	/* Checking for validity of channel number  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel "
			"number : %d.\n", __func__, channel);
		retval = -ENODEV;
		goto err_dma_set_callback;
	}
	/* Checking whether channel is not allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not allocated.\n", __func__);
		retval = -EINVAL;
		goto err_dma_set_callback;
	}
	/* Checking whether the channel is already enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel already enabled.\n", __func__);
		retval = -EBUSY;
		goto err_dma_set_callback;
	}
	/* Checking whether function pointer is NULL or not */
	if (pch_dma_cbr == NULL) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> NULL pointer parameter.\n", __func__);
		retval = -EINVAL;
		goto err_dma_set_callback;
	}
	/* Setting the callback. */
	dma_set_callback((u32)channel, pch_dma_cbr, data);
	PCH_DMA_DEBUG(
		"%s -> Function dma_set_callback.\n", __func__);
	retval = 0;

err_dma_set_callback:
	PCH_DMA_DEBUG(
		"%s " "returns %d.\n", __func__, retval);
	return retval;
}
EXPORT_SYMBOL(pch_dma_set_callback);

/**
 * pch_set_dma_priority - Sets the priority of the DMA channel.
 * @channel	DMA channel number.
 * @priority	Priority to be set for the DMA channel.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN:	Device is in suspend mode.
 *	-EINVAL:	Parameter passed is invalid.
 *	-EBUSY:	If channel is in use.
 *	-ENODEV:	Specified DMA channel does not exist.
 *
 * Sets the priority that has to be assigned for a particular channel.
 * This function is invoked by functions from other kernel modules.
 * The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not,
 *	  suitable error status codes are returned to the called function.
 *	- If valid, interacts with the HAL API to set the DMA channel
 *	  priority settings and returns the status code returned
 *	  by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_set_dma_priority(s32 channel, s32 priority)
{
	s32 retval;

	/* Checking whether the device is in suspend mode. */
	if (pch_dma_suspended == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> The device is "
			"in suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_set_dma_priority;
	}
	/* Checking for validity of channel number  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel "
			"number : %d", __func__, channel);
		retval = -ENODEV;
		goto err_set_dma_priority;
	}
	/* Checking whether channel is not allocated. */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not "
			"allocated.\n", __func__);
		retval = -EINVAL;
		goto err_set_dma_priority;
	}
	/* Checking whether the device is enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel already "
			"enabled.\n", __func__);
		retval = -EBUSY;
		goto err_set_dma_priority;
	}
	/* Check for validity of priority value */
	if ((priority > 3) || (priority < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid value "
			"priority (%d)", __func__, priority);
		retval = -EINVAL;
		goto err_set_dma_priority;
	}
	dma_set_priority((u32)channel, priority);
	PCH_DMA_DEBUG(
		"%s -> Function dma_set_priority.\n", __func__);
	retval = 0;

err_set_dma_priority:
	PCH_DMA_DEBUG(
	"%s returns " "%d.\n", __func__, retval);
	return retval;
}

/**
 * pch_dma_direct_start	Used to initiate a DMA transfer.
 * @channel	DMA channel number.
 *
 * Return codes
 *	0:		Success.
 *	-EAGAIN	Device is in suspend mode.
 *	-EBUSY		Specified DMA channel is not idle.
 *	-ENODEV	Specified DMA channel does not exist.
 *	-EINVAL	Specified channel is not allocated.
 *
 * Generates the DMA request to begin DMA transfer on a particular channel.
 * This function is invoked by functions from other kernel modules.
 * The main tasks performed by this function are:
 *	- Verifies whether the obtained parameters are valid, if not
 *	  suitable error status codes are returned to the called function.
 *	- If valid interacts with the HAL API to initiate the DMA process
 *	  and returns the status code returned by the HAL API.
 * This function is accessible by other kernel modules.
 */
s32 pch_dma_direct_start(s32 channel)
{
	s32 retval = 0;

	/* Checking whether the device is in suspend mode. */
	if (pch_dma_suspended == 1) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> The device is in "
			"suspend mode.\n", __func__);
		retval = -EAGAIN;
		goto err_dma_direct_start;
	}
	/* Checking for validity of channel number  */
	if ((channel >= PCH_DMA_CHANNELS_MAX) || (channel < 0)) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Invalid Channel "
			"number : %d.\n", __func__, channel);
		retval = -ENODEV;
		goto err_dma_direct_start;
	}
	/* Checking whether channel is reserved or not */
	if (pch_dma_channel_table[channel].ch_alloced == (u16) 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not "
			"allocated.\n", __func__);
		retval = -EINVAL;
		goto err_dma_direct_start;
	}
	/* Checking whether the device is not enabled. */
	if (pch_dma_channel_info[channel].ch_enable == 0) {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Channel not "
			"enabled.\n", __func__);
		retval = -EBUSY;
		goto err_dma_direct_start;
	}
	/* Initiating the DMA transfer */
	dma_direct_start((u32)channel);
	PCH_DMA_DEBUG(
		"%s -> Function dma_direct_start.\n", __func__);
	retval = 0;

err_dma_direct_start:
	PCH_DMA_DEBUG(
	"%s returns " "%d.\n", __func__, retval);
	return retval;
}

/**
 * get_dev_type - Returns the PCH device type for given PCI device id.
 * @devid	The device ID to be verified.
 *
 * Return codes
 *	PCH_INVALID_DEVICE:				Invalid device detected.
 *	Values other than PCH_INVALID_DEVICE:	Detected device is valid
 *							and supported.
 *
 * This function returns the type of the detected DMA device.
 * The type specifies the number of DMA channels contained within the
 * detected device. The tasks performed by this function include:
 *	- Matches the PCI device ID passed to it with a set of known device
 *	  IDs.
 *	- If a match is found it returns a constant which indicates the
 *	  device type (number of DMA channels) within the device.
 *	- If no match is found it returns @ref PCH_INVALID_DEVICE.
 */
static inline u32 get_dev_type(u32 devid)
{
	u32 dev_type;

	switch (devid) {
	case PCI_DEVICE_ID_INTEL_PCH1_DMA4_0:
		dev_type = PCH_DMA_4CH0;
		break;

	case PCI_DEVICE_ID_INTEL_PCH1_DMA8_0:
		dev_type = PCH_DMA_8CH0;
		break;

	default:
		PCH_DMA_DEBUG(
			"%s -> Unknown PCI device 0x%x\n", __func__, devid);
		dev_type = PCH_INVALID_DEVICE;
		break;

	}
		PCH_DMA_DEBUG(
			"%s returns %x.\n", __func__, dev_type);
	return dev_type;
}

/**
 * pch_dma_probe - Implements the probe function for the PCI driver.
 * @pdev	Reference to the pci_device structure.
 * @id		Reference to the pci_device_id for which this device matches.
 *
 * Return codes
 *	0:		On success.
 *	-EIO:		pci_enable_device error status code.
 *	-EBUSY:	pci_request_regions/request_irq error status code.
 *	-EINVAL:	pci_enable_device/request_irq error status code
 *			/invalid device ID.
 *	-ENOMEM:	request_irq/pci_iomap error status code.
 *	-ENOSYS:	request_irq error status code.
 *
 * This function acts as the probe function for the PCI driver.
 * The PCI core will be invoking this function once it determines that this
 * driver is suitable for handling a particular hardware.
 * The main tasks performed by this function are:
 *	- Confirms whether the detected device is supported by the driver.
 *	- Enables the PCi device.
 *	- Attains the device specific resources and store it for further use.
 *	- Enables the device and registers the handler for handling the
 *	  device interrupts.
 *	- Initializes the device specific data structures.
 */
static s32 __devinit pch_dma_probe(struct pci_dev *pdev,
				   const struct pci_device_id *id)
{
	static u8 pch_dma_dcount;
	s32 retval;
	u32 dev_type;
	u32 base_addr;

	/* Getting the internally used device ID of the detected device. */
	dev_type = get_dev_type(id->device);
	/* If invalid device. */
	if ((PCH_INVALID_DEVICE == dev_type)) {
		dev_err(&pdev->dev, "%s -> Invalid device ID "
			"%x.\n", __func__, id->device);
		retval = -EINVAL;
		goto err_probe1;
	}
	dev_dbg(&pdev->dev, "%s -> Valid device ID detected %x.\n",
		  __func__, id->device);

	/* Enabling the detected device */
	retval = pci_enable_device(pdev);
	if (0 != retval) {
		dev_err(&pdev->dev,
			"%s -> Function pci_enable_device "
			"failed, returned %d.\n", __func__, retval);
		goto err_probe1;
	}
	dev_dbg(&pdev->dev,
		"%s -> Function pci_enable_device invoked "
		"successfully returned %d.\n", __func__, retval);

	pci_set_master(pdev);
	dev_dbg(&pdev->dev, "%s -> Function pci_set_master invoked "
		  "successfully.\n", __func__);

	/* Requesting the PCI device regions. */
	retval = pci_request_regions(pdev, MODULE_NAME);
	if (0 != retval) {
		dev_err(&pdev->dev,
			"%s -> Function pci_request_regions "
			"failed, returned %d.\n", __func__, retval);
		goto err_probe2;
	}
	dev_dbg(&pdev->dev,
		"%s -> Function pci_request_regions invoked "
		"successfully returned %d.\n", __func__, retval);

	/* Remapping the device space to kernel space. */
	/* Wipro 1/13/2010 Use Mem BAR */
	base_addr = (u32) pci_iomap(pdev, 1, 0);
	if (0 == base_addr) {
		dev_err(&pdev->dev,
			"%s -> Function pci_iomap failed "
			"returned %x.\n", __func__, base_addr);
		retval = -ENOMEM;
		goto err_probe3;
	}
	dev_dbg(&pdev->dev,
		"%s -> Function pci_iomap invoked successfully.\n", __func__);

	/* Filling in the details within the device structure. */
	pch_dma_devices[pch_dma_dcount].dev_typ = dev_type;
	pch_dma_devices[pch_dma_dcount].base_addr = base_addr;

	/* Registering the interrupt handler. */
	retval =
	    request_irq(pdev->irq, dma_interrupt, IRQF_SHARED,
			MODULE_NAME, &pch_dma_devices[pch_dma_dcount]);
	if (0 != retval) {
		dev_err(&pdev->dev,
			"%s -> Function request_irq failed, "
			"returned %d.\n", __func__, retval);

		goto err_probe4;
	}
	dev_dbg(&pdev->dev, "%s -> Function request_irq invoked "
	     "successfully returned %d.\n", __func__, retval);

	/* Initializing the DMA device. */
	dma_init(base_addr, dev_type);
	dev_dbg(&pdev->dev,
		"%s -> Function dma_init invoked successfully.\n", __func__);

	/* Stroing the device structure reference for further use. */
	pci_set_drvdata(pdev, &pch_dma_devices[pch_dma_dcount]);

	/* Initializing the suspend flag and lock variable. */
	if (0 == pch_dma_dcount) {	/* Initialize only once. */
		pch_dma_suspended = 0;
		spin_lock_init(&pch_dma_lock);
	}

	/* Incrementing the device structure index. */
	pch_dma_dcount++;

	/* Probe successful. */
	dev_dbg(&pdev->dev, "%s -> Probe successful.\n", __func__);
	return 0;

err_probe4:
	/* Unmapping the remapped region. */
	pci_iounmap(pdev, (void *)base_addr);
	dev_dbg(&pdev->dev, "%s -> Function pci_iounmap invoked "
		"successfully.\n", __func__);
err_probe3:
	/* Releasing the requested regions. */
	pci_release_regions(pdev);
	dev_dbg(&pdev->dev, "%s -> Function pci_release_regions "
		"invoked successfully.\n", __func__);
err_probe2:
	/* Disabling the device. */
	pci_disable_device(pdev);
	dev_dbg(&pdev->dev, "%s -> Function pci_disable_device "
		"invoked successfully.\n", __func__);
err_probe1:
	dev_dbg(&pdev->dev,
		"%s -> Probe failed. returns %d.\n", __func__, retval);
	return retval;
}

/**
 * pch_dma_remove - Implements the remove function for the PCi driver.
 * @pdev	Reference to the pci_device structure.
 *
 * This function is invoked by the PCI subsystem of the	Kernel when the DMA
 * device is removed or the module is unloaded. It de-initializes and
 * releases all the resources attained during device detection.
 * The main tasks performed by this function are:
 *	- De-initializes the DMA device.
 *	- De-initializes the device specific data structures.
 *	- Releases all the resources attained during the
 *	  device detection phase.
 */
static void __devexit pch_dma_remove(struct pci_dev *pdev)
{
	struct pch_dma_devices *dev;

	/* Getting the driver data. */
	dev = pci_get_drvdata(pdev);
	/* Re-setting the driver data. */
	pci_set_drvdata(pdev, NULL);

	/* De-initializing the device. */
	dma_exit(dev->dev_typ);
	dev_dbg(&pdev->dev, "%s -> Function dma_exit invoked "
		  "successfully.\n", __func__);

	/* Un-registering the interrupt handler. */
	free_irq(pdev->irq, dev);
	dev_dbg(&pdev->dev, "%s -> Function free_irq invoked "
		  "successfully.\n", __func__);

	/* Un-mapping the remapped memory address. */
	pci_iounmap(pdev, (void *)dev->base_addr);
	dev->base_addr = 0;
	dev_dbg(&pdev->dev, "%s -> Function pci_iounmap invoked "
		  "successfully.\n", __func__);

	/* Releasing the requested regions. */
	pci_release_regions(pdev);
	dev_dbg(&pdev->dev, "%s -> Function pci_release_regions "
		  "invoked successfully.\n", __func__);

	/* Disabling the device. */
	pci_disable_device(pdev);
	dev_dbg(&pdev->dev, "%s -> Function pci_disable_device "
		  "invoked successfully.\n", __func__);

	dev_dbg(&pdev->dev, "%s invoked  "
		  "successfully for device %x.\n", __func__, pdev->device);
}

#ifdef CONFIG_PM
/**
 * pch_dma_suspend - Implements the suspend function for the pci_driver.
 * @pdev	Reference to the pci_device structure.
 * @state	The state of the device.
 *
 * Return codes
 *	0:		Operation successful.
 *	-ENOMEM:	pci_save_state error status code.
 *
 * This function is used as the suspend function of the PCI Driver.
 * The PCI core will be invoking this function once it receives a suspend
 * event from the PM layer.
 * The main tasks performed by this functions are:
 *	- Prepares the device so that it can enter the suspend state
 *		by saving the current state.
 *	- Disables all the DMA channels and the associated interrupts.
 *	- Changes the power state of the device to low power state.
 */
static s32 pch_dma_suspend(struct pci_dev *pdev, pm_message_t state)
{
	s32 retval;
	struct pch_dma_devices *dev;

	/* Setting flag for denoting Suspension. */
	pch_dma_suspended = 1;

	/* Getting the driver data. */
	dev = pci_get_drvdata(pdev);

	/* Saving the current state of the device. */
	retval = pci_save_state(pdev);
	if (retval == 0) {
		dev_dbg(&pdev->dev,
			"%s -> Function pci_save_state invoked "
			  "successfully (returned %d).\n", __func__, retval);

		/* De-initializing the device for suspension. */
		dma_exit(dev->dev_typ);
		dev_dbg(&pdev->dev,
			"%s -> Function dma_exit invoked "
			  "successfully.\n", __func__);

		/* Disabling the wake-up feature. */
		pci_enable_wake(pdev, PCI_D3hot, 0);
		dev_dbg(&pdev->dev,
			"%s -> Function pci_enable_wake "
			  "invoked successfully.\n", __func__);

		/* Setting the device to new state. */
		pci_set_power_state(pdev, pci_choose_state(pdev, state));
		dev_dbg(&pdev->dev,
			"%s -> Function pci_set_power_state "
			  "invoked successfully.\n", __func__);

		/* Disabling the device. */
		pci_disable_device(pdev);
		dev_dbg(&pdev->dev,
			"%s -> Function pci_disable_device "
			  "invoked successfully.\n", __func__);

		retval = 0;
		dev_dbg(&pdev->dev,
			"%s -> Suspension successful for "
			  "the device %x.\n", __func__, pdev->device);
	} else {
		dev_err(&pdev->dev,
			"%s -> Function pci_save_state failed"
			"returned %d.\n", __func__, retval);

		/* De-setting the flag on Suspend failure. */
		pch_dma_suspended = 0;

		dev_dbg(&pdev->dev,
			"%s -> Suspension un-successful for "
			  "the device %x.\n", __func__, pdev->device);
	}

	dev_dbg(&pdev->dev, "%s returns %d.\n", __func__, retval);
	return retval;
}

/**
 * pch_dma_resume - Implements the resume function for the pci_driver
 * @pdev	Pointer to the pci_device structure.
 *
 * Return codes
 *	0:	  Operation successful.
 *	-EIO:	  pci_enable_device error status code.
 *	-EINVAL: pci_enable_device error status code.
 *
 * This function is used as the resume function of the PCI driver.
 * The PCI core will be invoking this function once it receives a resume
 * event from	the PM layer. The main tasks performed by this function are:
 *	- Restores the power state of the device to normal state.
 *	- Enables the device so that it returns to its normal state.
 */
static s32 pch_dma_resume(struct pci_dev *pdev)
{
	s32 retval;

	/* Setting the device to normal power state. */
	(void)pci_set_power_state(pdev, PCI_D0);
	/* Restoring the device state. */
	(void)pci_restore_state(pdev);
	/* Enabling the device. */
	retval = pci_enable_device(pdev);

	if (0 == retval) {
		pci_set_master(pdev);
		(void)pci_enable_wake(pdev, PCI_D3hot, 0);
		retval = 0;
		/* De-setting the suspend flag to denote resumption
		successful. */
		pch_dma_suspended = 0;
	} else {
		dev_err(&pdev->dev,
			"%s -> Function pci_enable_device failed "
			"returned = %d. device = %x.\n",
			__func__, retval, pdev->device);
	}
	dev_dbg(&pdev->dev, "%s returns %d.\n", __func__, retval);
	return retval;
}
#else
#define pch_dma_suspend	NULL
#define pch_dma_resume	NULL
#endif

/* struct pch_dma_controller_driver
 *	- Used for registering the PCI driver functionalities */
static struct pci_driver pch_dma_controller_driver = {
	.name = MODULE_NAME,
	.id_table = pch_dma_pcidev_id,
	.probe = pch_dma_probe,
	.remove = __devexit_p(pch_dma_remove),
	.suspend = pch_dma_suspend,
	.resume = pch_dma_resume
};
MODULE_DEVICE_TABLE(pci, pch_dma_pcidev_id);

/**
 * pch_dma_pci_init - Module initialization routine.
 *
 * Return codes:
 *	0:		Loading successful.
 *	-EEXIST:	pci_register_driver error status code.
 *	-EINVAL:	pci_register_driver error status code.
 *	-ENOMEM:	pci_register_driver error status code.
 *
 * This function is invoked when the module is loaded.
 * The main tasks performed by this function are:
 *	- Initializes the module.
 *	- Initializes the local structures and registers the module
 *		as PCI Driver with the kernel subsystem.
 */
static __init s32 pch_dma_pci_init(void)
{
	s32 retval;

	/* Registering the module as PCI Driver. */
	retval = pci_register_driver(&pch_dma_controller_driver);

	if (retval == 0) {
		printk(KERN_INFO MODULE_NAME ": "
			"%s invoked successfully.\n", __func__);
	} else {
		printk(KERN_ERR MODULE_NAME ": "
			"%s -> Function pci_register_driver "
			"failed returned %d.\n", __func__, retval);
	}
	return retval;
}
module_init(pch_dma_pci_init);

/**
 * pch_dma_pci_exit - Module exit handler.
 *
 * Kernel subsystem will be invoking this routine once the module gets
 * unloaded. The main tasks performed by this function are:
 *	- Un-registers the PCI driver.
 *	- Unloads the module.
 */
static __exit void pch_dma_pci_exit(void)
{
	/* Un-registering the module as PCI Driver. */
	pci_unregister_driver(&pch_dma_controller_driver);
	printk(KERN_INFO MODULE_NAME ": "
			"%s invoked successfully.\n", __func__);
}
module_exit(pch_dma_pci_exit);

