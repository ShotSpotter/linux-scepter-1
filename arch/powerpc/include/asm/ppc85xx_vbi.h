/*
 * ppc85xx.h - PowerPC specific header
 *
 * Copyright 2007 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 */

#ifndef __INCppc85xxh
#define __INCppc85xxh

#if (CPU == PPCE500MC)
#include <sys/ppc/ppcE500mc.h>
#endif

#if (CPU==PPC85XX)
#define PPC_e500v2
#endif


/* Maximum cache line size for the cpu */
#undef	_CPU_CACHE_ALIGN_SIZE
#if (CPU == PPCE500MC)
#define	_CPU_CACHE_ALIGN_SHIFT	6	/* cache align size = 32 */
#define	_CPU_CACHE_ALIGN_SIZE	64	/* cannot use (1<<5) because compiler*/
#else
#define	_CPU_CACHE_ALIGN_SHIFT	5	/* cache align size = 32 */
#define	_CPU_CACHE_ALIGN_SIZE	32	/* cannot use (1<<5) because compiler
                                         * directive __attribute__ cant hdl */
#endif
/* MMU supports software TLB miss handler */

#undef  _WRS_TLB_MISS_CLASS_HW
#define _WRS_TLB_MISS_CLASS_SW  1

/* MMU uses PID register to extend virtual address tag */

#undef	_WRS_MMU_CLASS_NONE
#undef	_WRS_MMU_CLASS_SR_REGS
#define _WRS_MMU_CLASS_PID_REG	1

/* MMU ASID context register name for _WRS_MMU_CLASS_PID_REG */

#define	_PPC_PID_REG		PID0

/* MMU cannot be disabled */

#define	PPC_NO_REAL_MODE

/*
 * MMU supports the M attribute bit, allowing the coherency-required property
 * to be configured on a per-page basis.  Therefore, even when snooping is
 * enabled, cacheDmaMalloc needs to call arch support to set the M bit.
 */
#define	_WRS_SNOOP_NEEDS_DMA_MALLOC_SUPPORT

#if  (CPU != PPCE500MC) || (CPU != PPCE200)
#define _WRS_SPE_SUPPORT	1
#endif /* (CPU != PPCE500MC) */

#define _WRS_E500_FIXED_POINT_SUPPORT 1

#ifdef _WRS_SPE_SUPPORT
#undef _PPC_MSR_FP
#define _PPC_MSR_SPE_U		0x0200
#define _PPC_MSR_SPE		0x02000000
#define MSR_SPE_BIT_SHIFT	25
#endif /* _WRS_SPE_SUPPORT */

/* #define IMMR	638	* bogus hack */

/* Special Purpose Register (SPR) numbers */

#define CSRR0   58      /* Critical SRR0 */
#define CSRR1   59      /* Critical SRR1 */
#define DEAR    61      /* Data Exception Address Register */
#define ESR     62      /* Exception Syndrome Register */
#define IVPR    63      /* Interrupt Vector Prefix Register */
#define IVOR0   400     /* IVOR Critical Input */
#define IVOR1   401     /* IVOR Machine Check */
#define IVOR2   402     /* IVOR Data Storage */
#define IVOR3   403     /* IVOR Instruction Storage */
#define IVOR4   404     /* IVOR External Input */
#define IVOR5   405     /* IVOR Alignment */
#define IVOR6   406     /* IVOR Program */
#define IVOR7   407     /* IVOR Floating Point Unavailable */
#define IVOR8   408     /* IVOR System Call */
#define IVOR9   409     /* IVOR Auxiliary Processor Unavailable */
#define IVOR10  410     /* IVOR Decrementer */
#define IVOR11  411     /* IVOR Fixed Interval Timer */
#define IVOR12  412     /* IVOR Watchdog Timer */
#define IVOR13  413     /* IVOR Data TLB Error */
#define IVOR14  414     /* IVOR Instruction TLB Error */
#define IVOR15  415     /* IVOR Debug */
#if (CPU==PPC85XX) || (CPU==PPCE200)
#define IVOR32  528     /* IVOR SPE */
#define IVOR33  529     /* IVOR Vector FP Data */
#define IVOR34  530     /* IVOR Vector FP Round */
#endif
#define IVOR35  531     /* IVOR Performance Monitor */

#define MCSRR0  570     /* Machine Check SRR0 */
#define MCSRR1  571     /* Machine Check SRR1 */
#define MCAR    573     /* Machine Check Address Register */

#define MCSR    572     /* Machine Check Syndrome Register */

#if (CPU==PPC85XX) || (CPU==PPCE200)
#define SPEFSCR 512     /* SPE Floating-pt Status and Control Register */
#endif

/* SPRG0-SPRG3 are defined correctly in asmPpc.h */
#define SPRG4_R 260     /* Special Purpose Register General 4, read */
#define SPRG4_W 276     /* Special Purpose Register General 4, write */
#define SPRG5_R 261     /* Special Purpose Register General 5, read */
#define SPRG5_W 277     /* Special Purpose Register General 5, write */
#define SPRG6_R 262     /* Special Purpose Register General 6, read */
#define SPRG6_W 278     /* Special Purpose Register General 6, write */
#define SPRG7_R 263     /* Special Purpose Register General 7, read */
#define SPRG7_W 279     /* Special Purpose Register General 7, write */
#define TBL_R   268     /* Time Base Lower, read */
#define TBL_W   284     /* Time Base Lower, write */
#define TBU_R   269     /* Time Base Upper, read */
#define TBU_W   285     /* Time Base Upper, write */
#define TCR     340     /* Timer Control Register */
#define TSR     336     /* Timer Status Register */
#define USPRG0  256     /* User Special Purpose Register General 0 */
#define DBCR0   308     /* Debug Control Register 0 */
#define DBCR1   309     /* Debug Control Register 1 */
#define DBCR2   310     /* Debug Control Register 2 */
#if (CPU==PPCE500MC)
#define DBCR4   563     /* Debug Control Register 4 */
#define DBSRWR  306     /* Debug Status Read Write  */
#endif
#define DBSR    304     /* Debug Status Register */
#define IAC1    312     /* Instr Address Compare Register 1 */
#define IAC2    313     /* Instr Address Compare Register 2 */
#define DAC1    316     /* Data Address Compare Register 1 */
#define DAC2    317     /* Data Address Compare Register 2 */
#define L1CFG0  515     /* L1 Config Register 0 */
#define L1CFG1  516     /* L1 Config Register 1 */
#define L1CSR0  1010    /* L1 Control Status Register 0 */
#define L1CSR1  1011    /* L1 Control Status Register 1 */
#ifdef PPC_e200
#define L1FINV0 1016
#endif

#define MAS0    624     /* MMU Assist Register 0 */
#define MAS1    625     /* MMU Assist Register 1 */
#define MAS2    626     /* MMU Assist Register 2 */
#define MAS3    627     /* MMU Assist Register 3 */
#define MAS4    628     /* MMU Assist Register 4 */

#define MAS6    630     /* MMU Assist Register 6 */
#if defined(PPC_e500v2) || (CPU == PPCE500MC)
#define MAS7    944     /* MMU Assist Register 7 */
#endif /* PPC_e500v2 */

#define TLB0CFG 688     /* TLB 0 Config Register */
#define TLB1CFG 689     /* TLB 1 Config Register */
#define MMUCSR0 1012    /* MMU Control Status Register 0 */
#define MMUCFG  1015    /* MMU Config Register */


/*
 * Machine check exception class is new to PPC in E500.  Although
 * bit position is same as classic MSR[ME] and is named the same,
 * the define of _PPC_MSR_MCE signifies the present of this
 * class of exception.  If present, both _PPC_MSR_MCE and
 * _PPC_MSR_ME should be defined to the mask of 0x1000.
 * In addition, Critical Exception Class is also a requirement.
 * The critical exception code stub does not mask exceptions
 * and is used for machine check exception class as well.
 * Therefore, _EXC_OFF_CRTL should be defined.
 */

#if (CPU!=PPCE200)
#define _PPC_MSR_MCE    _PPC_MSR_ME	/* machine check enable */
#else
#undef _PPC_MSR_MCE
#endif

#define	_PPC_MSR_BIT_WE		13
#define	_PPC_MSR_BIT_CE		14
#define	_PPC_MSR_BIT_DE		22
#define _PPC_MSR_BIT_IS		26
#define _PPC_MSR_BIT_DS		27

/* Only allow the following bits to be modified by the guest
 * Some of these are also forced on by the hypervisor */
#if (CPU==PPC85XX) || (CPU==PPCE200)
#define CONTROL_MSR_MASK  (_PPC_MSR_CE | _PPC_MSR_SPE | _PPC_MSR_EE |\
			   _PPC_MSR_PR | _PPC_MSR_ME | _PPC_MSR_DE |\
			   _PPC_MSR_IS | _PPC_MSR_DS | _PPC_MSR_RI)
#endif

#define _PPC_MSR_MMU_EXTRACT(src, dst) \
	rlwinm	dst, src, 0, _PPC_MSR_BIT_IS, _PPC_MSR_BIT_DS

/* INT_MASK definition (mask EE & CE bits) : overwrite the one in asmPpc.h */

#undef  INT_MASK
#define INT_MASK(src, des) \
	rlwinm  des, src, 0, _PPC_MSR_BIT_EE+1, _PPC_MSR_BIT_EE-1; \
	rlwinm  des, des, 0, _PPC_MSR_BIT_CE+1, _PPC_MSR_BIT_CE-1


#define HID0  1008
#if (CPU != PPCE500MC)
#define HID1  1009
#endif

#define DECAR 54
#define BUCSR 1013

#define _PPC_BUCSR_FI 0x200            /* Invalidate branch cache */
#define _PPC_BUCSR_E 0x1               /* Enable branch prediction */

/* E500core other than e500mc has no FPU */

#if (CPU==PPCE500MC)
#define _WRS_HARDWARE_FP
#endif

#ifndef _WRS_HARDWARE_FP
#undef  _PPC_MSR_FP             /* floating point not available */
#undef  _PPC_MSR_FE1            /* floating point not available */
#undef  _PPC_MSR_FE0            /* floating point not available */
#undef  _PPC_MSR_BIT_FP         /* MSR Floating Point Aval. bit - FP */
#undef  _PPC_MSR_BIT_FE0        /* MSR FP exception mode 0 bit - FE0 */
#undef  _PPC_MSR_BIT_FE1        /* MSR FP exception mode 1 bit - FE1 */
#undef  _EXC_PROG_SRR1_FPU      /* floating point not available */
#else	/* _WRS_HARDWARE_FP */
#define _WRS_FP_CLASS_HW	1
#define  _PPC_MSR_BIT_FP         18
#define  _PPC_MSR_BIT_FE0        20
#define  _PPC_MSR_BIT_FE1        23
#endif	/* _WRS_HARDWARE_FP */



/*
 * Exception syndrome register mask bits:
 * 0 - error not occured 1 - error occured
 */

#define _PPC_ESR_PIL_U 0x0800      /* Pgm Interrupt -- Illegal Insn */
#define _PPC_ESR_PPR_U 0x0400      /* Pgm Interrupt -- Previleged Insn */
#define _PPC_ESR_PTR_U 0x0200      /* Pgm Interrupt -- Trap */
#if (CPU == PPCE500MC)
#define _PPC_ESR_FP_U  0x0100      /* Floating Point Operation */
#endif /* (CPU == PPCE500MC) */
#define _PPC_ESR_ST_U  0x0080      /* Store Operation */
#define _PPC_ESR_DLK_U 0x0020      /* Data Locked -- DSI occured */
#define _PPC_ESR_ILK_U 0x0010      /* Inst Locked -- DSI occured */
#define _PPC_ESR_AP_U  0x0008      /* AP Operation */
#define _PPC_ESR_BO_U  0x0002      /* Byte Ordering Exception */
#define _PPC_ESR_PIL   0x08000000  /* Pgm Interrupt -- Illegal Insn */
#define _PPC_ESR_PPR   0x04000000  /* Pgm Interrupt -- Previleged Insn */
#define _PPC_ESR_PTR   0x02000000  /* Pgm Interrupt -- Trap */
#if (CPU == PPCE500MC)
#define _PPC_ESR_FP    0x01000000  /* Floating Point Operation */
#endif /* (CPU == PPCE500MC) */
#define _PPC_ESR_ST    0x00800000  /* Store Operation */
#define _PPC_ESR_DLK   0x00200000  /* Data Storage Interrupt -- Locking */
#define _PPC_ESR_ILK   0x00100000  /* Inst Locked -- DSI occured */
#define _PPC_ESR_AP    0x00080000  /* AP Operation */
#define _PPC_ESR_BO    0x00020000  /* Byte Ordering Exception */
#define _PPC_ESR_SPE   0x00000080  /* SPE exception */
#define _PPC_ESR_EPID  0x00000040 /* External PID Access */
#define _PPC_ESR_VLEMI 0x00000020 /* VLE instruction */
#define _PPC_ESR_MIF   0x00000002 /* Misaligned Insn Fetch (ITLB, ISI) */
#define _PPC_ESR_XTE   0x00000001 /* External Transaction Err (ISI, DSI) */


#define   _PPC_EPCR_EXTGS      0x80000000 /* Guest gets external ints */
#define   _PPC_EPCR_DTLBGS     0x40000000 /* Guest gets DTLB errors */
#define   _PPC_EPCR_ITLBGS     0x20000000 /* Guest gets ITLB errors */
#define   _PPC_EPCR_DSIGS      0x10000000 /* Guest gets DSIs */
#define   _PPC_EPCR_ISIGS      0x08000000 /* Guest gets ISIs */
#define   _PPC_EPCR_DUVD       0x04000000 /* Disable Embedded HV Debug */
#define   _PPC_EPCR_DGTMI      0x00800000 /* Disable guest TLB management insns*/
#define   _PPC_EPCR_DMIUH      0x00400000 /* Disable MAS int updates for hypervisor*/

/* Bits in the upper half of TCR */

#define _PPC_TCR_WP_U     0xc000  /* Watchdog Timer Period */
#define _PPC_TCR_WRC_U    0x3000  /* Watchdog Timer Reset Control */
#define _PPC_TCR_WIE_U    0x0800  /* Watchdog Timer Interrupt Enable */
#define _PPC_TCR_DIE_U    0x0400  /* Decrementer Interrupt Enable */
#define _PPC_TCR_FP_U     0x0300  /* Fixed Interval Timer Period */
#define _PPC_TCR_FIE_U    0x0080  /* Fixed Interval Timer Interrupt Enable */
#define _PPC_TCR_ARE_U    0x0040  /* Decrementer Auto-Reload Enable */
#define _PPC_TCR_WPEXT_U  0x0040  /* Decrementer Auto-Reload Enable */
#define _PPC_TCR_FPEXT_U  0x0040  /* Decrementer Auto-Reload Enable */

/* Bits in the upper half of TSR */

#define _PPC_TSR_ENW_U  0x8000  /* Enable Next Watchdog Timer Exception */
#define _PPC_TSR_WIS_U  0x4000  /* Watchdog Timer Interrupt Status */
#define _PPC_TSR_WRS_U  0x3000  /* Watchdog Timer Reset Status */
#define _PPC_TSR_DIS_U  0x0800  /* Decrementer Interrupt Status */
#define _PPC_TSR_FIS_U  0x0400  /* Fixed Interval Timer Interrupt Status */

/* versions of the aligned for 32-bit TCR/TSR register access */

#define _PPC_TCR_DIE (_PPC_TCR_DIE_U << 16)
#define _PPC_TSR_DIS (_PPC_TSR_DIS_U << 16)

/* hardware dependent register 0 */

#define _PPC_HID0_DOZE  0x00800000      /* DOZE power management mode */
#define _PPC_HID0_NAP   0x00400000      /* NAP power management mode */
#define _PPC_HID0_SLEEP 0x00200000      /* SLEEP power management mode */
#define _PPC_HID0_TBEN	0x00004000	/* time base enable */
#define _PPC_HID0_MAS7EN  0x00000080    /* Enable use of MAS7 for tlbre */
#define _PPC_HID0_DCFA    0x00000040    /* Use this bit to flush only valid entries same as 74XX */
#define _PPC_HID0_BIT_MAS7EN   24
#define _PPC_HID0_BIT_DCFA     25
#define _PPC_85XX_USE_DCFA

/* hardware dependent register 1 */

#define _PPC_HID1_ABE   0x00001000      /* Address broadcast enable */

/* Cache Defines */

/* Instruction and Data Cache bit fields are the same */

#ifndef PPC_e200
#define _PPC_L1CSR_E   0x00000001 /* Enable */
#define _PPC_L1CSR_FI  0x00000002 /* Flash Invalidate */
#define _PPC_L1CSR_FLR 0x00000100 /* Lock Bits Flash */
#define _PPC_L1CSR_LO  0x00000200 /* Lock Overflow */
#define _PPC_L1CSR_UL  0x00000400 /* Unable to lock   - status bit */
#define _PPC_L1CSR_UL_V(x)  (x >> 10)
#define _PPC_L1CSR_SLC 0x00000800 /* Snoop lock clear  - status bit */
#define _PPC_L1CSR_SLC_V(x) (x >> 11)
#define _PPC_L1CSR_PIE 0x00008000 /* Parity Injection Enable */
#define _PPC_L1CSR_CPE 0x00010000 /* Parity Enable */

/* Instruction and Data Cache bit fields are the same */

#define _PPC_L1CFG_SIZE_MASK   0x00000FFF
#define _PPC_L1CFG_NWAY_MASK   0x000FF000
#define _PPC_L1CFG_NWAY_V(x)   (x >> 12)
#define _PPC_L1CFG_PA_MASK     0x00100000
#define _PPC_L1CFG_PA_V(x)     (x >> 16)
#define _PPC_L1CFG_LA_MASK     0x00200000
#define _PPC_L1CFG_LA_V(x)     (x >> 17)
#define _PPC_L1CFG_REPL_MASK   0x00400000
#define _PPC_L1CFG_REPL_V(x)   (x >> 18)
#define _PPC_L1CFG_BSIZE_MASK  0x01800000
#define _PPC_L1CFG_BSIZE_V(x)  (x >> 19)
#define _PPC_L1CFG_CARCH_MASK  0xC0000000    /* L1CFG0 only */
#define _PPC_L1CFG_CARCH_V(x)  (x >> 30)

#else /* PPC_e200 */

#define _PPC_L1CSR_WID_MSK 0xf0000000
#define _PPC_L1CSR_WID_SHFT 28
#define _PPC_L1CSR_WDD_MSK 0x0f000000
#define _PPC_L1CSR_WDD_SHFT 24
#define _PPC_L1CSR_WID_SET(x)  ((x<<_PPC_L1CSR_WID_SHFT) & _PPC_L1CSR_WID_MSK)
#define _PPC_L1CSR_WDD_SET(x)  ((x<<_PPC_L1CSR_WDD_SHFT) & _PPC_L1CSR_WDD_MSK)
#define _PPC_L1CSR_WID_GET(x)  ((x & _PPC_L1CSR_WID_MSK)>>_PPC_L1CSR_WID_SHFT)
#define _PPC_L1CSR_WDD_GET(x)  ((x & _PPC_L1CSR_WDD_MSK)>>_PPC_L1CSR_WDD_SHFT)

#define _PPC_L1CSR_AWID   0x00800000
#define _PPC_L1CSR_AWDD   0x00400000
#define _PPC_L1CSR_CWM    0x00100000
#define _PPC_L1CSR_DPB    0x00080000
#define _PPC_L1CSR_DSB    0x00040000
#define _PPC_L1CSR_DSTRM  0x00020000
#define _PPC_L1CSR_CPE    0x00010000
#define _PPC_L1CSR_CUL    0x00000800
#define _PPC_L1CSR_CLO    0x00000400
#define _PPC_L1CSR_CLFC   0x00000200
#define _PPC_L1CSR_CABT   0x00000004
#define _PPC_L1CSR_CINV   0x00000002
#define _PPC_L1CSR_CE     0x00000001

#define _PPC_L1CFG_SIZE_MASK   0x000007FF
#define _PPC_L1CFG_NWAY_MASK   0x007FF100
#define _PPC_L1CFG_NWAY_V(x)   (x >> 11)
#define _PPC_L1CFG_PA_MASK     0x00080000
#define _PPC_L1CFG_PA_V(x)     (x >> 19)
#define _PPC_L1CFG_LA_MASK     0x00100000
#define _PPC_L1CFG_LA_V(x)     (x >> 20)
#define _PPC_L1CFG_REPL_MASK   0x00600000
#define _PPC_L1CFG_REPL_V(x)   (x >> 21)
#define _PPC_L1CFG_BSIZE_MASK  0x01800000
#define _PPC_L1CFG_BSIZE_V(x)  (x >> 23)
#define _PPC_L1CFG_CFISWA_MASK 0x08000000
#define _PPC_L1CFG_CFAHA_MASK  0x10000000
#define _PPC_L1CFG_CWPA_MASK   0x20000000
#define _PPC_L1CFG_CARCH_MASK  0xC0000000
#define _PPC_L1CFG_CARCH_V(x)  (x >> 30)

#define _PPC_L1FINV0_CWAY_MSK  0x07000000
#define _PPC_L1FINV0_CWAY_SET(x) ((x << 24) & _PPC_L1FINV0_CWAY_MSK)
#define _PPC_L1FINV0_CSET_MSK  0x00000FC0
#define _PPC_L1FINV0_CSET_SET(x) ((x << 6) & _PPC_L1FINV0_CSET_MSK)
#define _PPC_L1FINV0_CCMD_MSK  0x00000003
#define _PPC_L1FINV0_CCMD_SET(x) (x & _PPC_L1FINV0_CCMD_MSK)

#ifndef _ASMLANGUAGE
typedef union e200_l1cfg0
    {
    UINT32 l1Cfg0;
    struct
	{
	uint32_t carch:2;
	uint32_t cwpa:1;
	uint32_t cfaha:1;
	uint32_t cfiswa:1;
	uint32_t :2;
	uint32_t cbsize:2;
	uint32_t crepl:2;
	uint32_t cla:1;
	uint32_t cpa:1;
	uint32_t cnway:8;
	uint32_t csize:11;
	} field;
    } E200_L1CFG0;
#endif /* _ASMLANGUAGE */

#endif /* PPC_e200 */

/* MMU register defines */

#define PID     48
#define PID_MASK 0x0FF
#define PID0    48
#if (CPU != PPCE500MC)
#define PID1    633
#define PID2    634
#endif /* (CPU != PPCE500MC) */



#define _PPC_MMUCSR0_L2TLB1_FI  0x00000002
#define _PPC_MMUCSR0_L2TLB1_FI_V(x) (x >> 1)
#define _PPC_MMUCSR0_L2TLB0_FI  0x00000004
#define _PPC_MMUCSR0_L2TLB0_FI_V(x) (x >> 2)
#define _PPC_MMUCSR0_DL1MMU_FI  0x00000008
#define _PPC_MMUCSR0_DL1MMU_FI_V(x) (x >> 3)
#define _PPC_MMUCSR0_IL1MMU_FI  0x00000010
#define _PPC_MMUCSR0_IL1MMU_FI_V(x) (x >> 4)

#if (CPU==PPCE500MC)
#define _PPC_MMUCSR0_DYN_INVAL  (_PPC_MMUCSR0_L2TLB0_FI)
#else
#define _PPC_MMUCSR0_DYN_INVAL  (_PPC_MMUCSR0_DL1MMU_FI | \
			       _PPC_MMUCSR0_IL1MMU_FI | \
			       _PPC_MMUCSR0_L2TLB0_FI)
#endif

/* Standard PTE registers */
#define _PPC_MAS0_NV           0x00000000
#define _PPC_MAS0_ESEL_MASK    0x03ff0000
#define _PPC_MAS0_ESEL_BIT     16
#define _PPC_MAS0_ESEL_V(x)    (x >> _PPC_MAS0_ESEL_BIT)
#define _PPC_MAS0_TLBSEL1      0x10000000
#define _PPC_MAS0_TLBSEL_MASK  0x30000000

/* Standard PTE registers */
#define _PPC_MAS1_V            0x80000000
#define _PPC_MAS1_IPROT        0x40000000
#define _PPC_MAS1_TID_MASK     0x00ff0000
#define _PPC_MAS1_TS           0x00001000
#define _PPC_MAS1_TSIZE_MASK   0x00000f00

/* Standard PTE registers */
#define _PPC_MAS2_EPN_MASK     0xFFFFF000
#define _PPC_MAS2_X0           0x00000040
#define _PPC_MAS2_X1           0x00000020
#define _PPC_MAS2_W            0x00000010
#define _PPC_MAS2_I            0x00000008
#define _PPC_MAS2_M            0x00000004
#define _PPC_MAS2_G            0x00000002
#define _PPC_MAS2_E            0x00000001

/* Standard PTE registers */
#define _PPC_MAS3_RPN_MASK     0xFFFFF000
#define _PPC_MAS3_U0_U3_MASK   0x000003C0
#define _PPC_MAS3_UX           0x00000020
#define _PPC_MAS3_SX           0x00000010
#define _PPC_MAS3_UW           0x00000008
#define _PPC_MAS3_SW           0x00000004
#define _PPC_MAS3_UR           0x00000002
#define _PPC_MAS3_SR           0x00000001

/* Default value register for MAS0-3 */
#define _PPC_MAS4_TLBSELD      0x10000000
#define _PPC_MAS4_TSIZED       0x00000F00
#define _PPC_MAS4_X0D          0x00000040
#define _PPC_MAS4_X1D          0x00000020
#define _PPC_MAS4_WD           0x00000010
#define _PPC_MAS4_ID           0x00000008
#define _PPC_MAS4_MD           0x00000004
#define _PPC_MAS4_GD           0x00000002
#define _PPC_MAS4_ED           0x00000001

/* used in tlbsx */
#define _PPC_MAS5_SGS          0x80000000
#define _PPC_MAS5_SLPID_MASK   0x000000FF

/* used in tlbsx */
#define _PPC_MAS6_SPID_MASK    0x00FF0000
#define _PPC_MAS6_SAS          0x00000001

/* e500v2/e500mc */
#define _PPC_MAS7_ERPN_MASK    0x0000000F

/* Possible Extension to PTE - probably just program based LPI info */
#define _PPC_MAS8_TGS          0x80000000
#define _PPC_MAS8_VF           0x40000000
#define _PPC_MAS8_TLPID_MASK   0x000000FF

/* Range of hardware context numbers (PID register & TLB TID field) */

#define MMU_ASID_MIN            1
#define MMU_ASID_MAX            255
#define MMU_ASID_GLOBAL         MMU_ASID_MIN

/* debug control register 0 */

#define _DBCR0_IDM_U    0x4000          /* internal debug mode */
#define _DBCR0_RST_U    0x3000          /* reset */
#define _DBCR0_ICMP_U   0x0800          /* instruction completion debug event */
#define _DBCR0_BRT_U    0x0400          /* branch taken debug event */
#define _DBCR0_IRPT_U   0x0200          /* interrupt debug event */
#define _DBCR0_TRAP_U   0x0100          /* trap debug event */
#define _DBCR0_IAC1_U   0x0080          /* instruction address compare 1 */
#define _DBCR0_IAC2_U   0x0040          /* instruction address compare 2 */
#define _DBCR0_DAC1R_U  0x0008          /* data address compare 1 Read */
#define _DBCR0_DAC1W_U  0x0004          /* data address compare 1 Write */
#define _DBCR0_DAC2R_U  0x0002          /* data address compare 2 Read */
#define _DBCR0_DAC2W_U  0x0001          /* data address compare 2 Write */
#define _DBCR0_IDM      0x40000000      /* internal debug mode */
#define _DBCR0_RST      0x30000000      /* reset */
#define _DBCR0_ICMP     0x08000000      /* instruction completion debug event */
#define _DBCR0_BRT      0x04000000      /* branch taken */
#define _DBCR0_IRPT     0x02000000      /* exception debug event */
#define _DBCR0_TRAP     0x01000000      /* trap debug event */
#define _DBCR0_IAC1     0x00800000      /* instruction address compare 1 */
#define _DBCR0_IAC2     0x00400000      /* instruction address compare 2 */
#define _DBCR0_DAC1R    0x00080000      /* data address compare 1 Read */
#define _DBCR0_DAC1W    0x00040000      /* data address compare 1 Write */
#define _DBCR0_DAC2R    0x00020000      /* data address compare 2 Read */
#define _DBCR0_DAC2W    0x00010000      /* data address compare 2 Write */
#define _DBCR0_RET      0x00008000      /* return debug event */
#define _DBCR0_FT       0x00000001      /* freeze timers on debug */

/* debug control register 1 */

#define _DBCR1_IAC1US_U   0xc000        /* IAC 1 User/Supervisor */
#define _DBCR1_IAC1ER_U   0x3000        /* IAC 1 Effective/Real */
#define _DBCR1_IAC2US_U   0x0c00        /* IAC 2 User/Supervisor */
#define _DBCR1_IAC2ER_U   0x0300        /* IAC 2 Effective/Real */
#define _DBCR1_IAC12M_U   0x00c0        /* IAC 1/2 Mode */
#define _DBCR1_IAC12AT_U  0x0001        /* IAC 1/2 Auto-Toggle Enable */
#define _DBCR1_IAC1US     0xc0000000    /* IAC 1 User/Supervisor */
#define _DBCR1_IAC1ER     0x30000000    /* IAC 1 Effective/Real */
#define _DBCR1_IAC2US     0x0c000000    /* IAC 2 User/Supervisor */
#define _DBCR1_IAC2ER     0x03000000    /* IAC 2 Effective/Real */
#define _DBCR1_IAC12M     0x00c00000    /* IAC 1/2 Mode */

/* debug control register 2 */

#define _DBCR2_DAC1US_U   0xc000        /* DAC 1 User/Supervisor */
#define _DBCR2_DAC1ER_U   0x3000        /* DAC 1 Effective/Real */
#define _DBCR2_DAC2US_U   0x0c00        /* DAC 2 User/Supervisor */
#define _DBCR2_DAC2ER_U   0x0300        /* DAC 2 Effective/Real */
#define _DBCR2_DAC12M_U   0x00c0        /* DAC 1/2 Mode */
#define _DBCR2_DAC1US     0xc0000000    /* DAC 1 User/Supervisor */
#define _DBCR2_DAC1ER     0x30000000    /* DAC 1 Effective/Real */
#define _DBCR2_DAC2US     0x0c000000    /* DAC 2 User/Supervisor */
#define _DBCR2_DAC2ER     0x03000000    /* DAC 2 Effective/Real */
#define _DBCR2_DAC12M     0x00c00000    /* DAC 1/2 Mode */

/* debug status register */

#define _DBSR_IDE_U     0x8000          /* Imprecise Debug Event */
#define _DBSR_UDE_U     0x4000          /* Unconditional Debug Event */
#define _DBSR_MRR_U     0x3000          /* Most Recent Reset */
#define _DBSR_ICMP_U    0x0800          /* Instruction Completion Debug Event */
#define _DBSR_BRT_U     0x0400          /* Branch Taken Debug Event */
#define _DBSR_IRPT_U    0x0200          /* Interrupt Debug Event */
#define _DBSR_TRAP_U    0x0100          /* Trap Debug Event */
#define _DBSR_IAC1_U    0x0080          /* IAC 1 Debug Event */
#define _DBSR_IAC2_U    0x0040          /* IAC 2 Debug Event */
#define _DBSR_DAC1R_U   0x0008          /* DAC/DVC 1 Read Debug Event */
#define _DBSR_DAC1W_U   0x0004          /* DAC/DVC 1 Write Debug Event */
#define _DBSR_DAC2R_U   0x0002          /* DAC/DVC 2 Read Debug Event */
#define _DBSR_DAC2W_U   0x0001          /* DAC/DVC 2 Write Debug Event */
#define _DBSR_IDE       0x80000000      /* Imprecise Debug Event */
#define _DBSR_UDE       0x40000000      /* Unconditional Debug Event */
#define _DBSR_MRR       0x30000000      /* Most Recent Reset */
#define _DBSR_ICMP      0x08000000      /* Instruction Completion Debug Event */
#define _DBSR_BRT       0x04000000      /* Branch Taken Debug Event */
#define _DBSR_IRPT      0x02000000      /* Interrupt Debug Event */
#define _DBSR_TRAP      0x01000000      /* Trap Debug Event */
#define _DBSR_IAC1      0x00800000      /* IAC 1 Debug Event */
#define _DBSR_IAC2      0x00400000      /* IAC 2 Debug Event */
#define _DBSR_DAC1R     0x00080000      /* DAC/DVC 1 Read Debug Event */
#define _DBSR_DAC1W     0x00040000      /* DAC/DVC 1 Write Debug Event */
#define _DBSR_DAC2R     0x00020000      /* DAC/DVC 2 Read Debug Event */
#define _DBSR_DAC2W     0x00010000      /* DAC/DVC 2 Write Debug Event */
#define _DBSR_RET       0x00008000      /* Return Debug Event */

/* mask for hardware breakpoints */

#define _DBSR_HWBP_MSK  ( _DBSR_IAC1 | _DBSR_IAC2 | \
                          _DBSR_DAC1R | _DBSR_DAC1W | \
                          _DBSR_DAC2R | _DBSR_DAC2W )

#endif /* __INCppc85xxh */
