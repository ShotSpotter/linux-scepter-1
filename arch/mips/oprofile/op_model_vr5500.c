/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2009 Wind River Systems, Inc.
 */
#include <linux/cpumask.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <asm/irq_regs.h>

#include "op_impl.h"

#define M_PERFCTL_EXL			(1UL      <<  0)
#define M_PERFCTL_KERNEL		(1UL      <<  1)
#define M_PERFCTL_SUPERVISOR		(1UL      <<  2)
#define M_PERFCTL_USER			(1UL      <<  3)
#define M_PERFCTL_INTERRUPT_ENABLE	(1UL      <<  4)
#define M_PERFCTL_INTERRUPT		(1UL      <<  5)
#define M_PERFCTL_EVENT(event)		(((event) & 0xf)  << 6)
#define M_PERFCTL_COUNT_ENABLE		(1UL      <<  10)

#define NUM_COUNTERS                    2

static int (*save_perf_irq) (void);

#define __define_perf_accessors(r, n)    				\
									\
	static inline unsigned int r_c0_ ## r ## n(void)		\
	{								\
		return read_c0_ ## r ## n();				\
	}								\
									\
	static inline void w_c0_ ## r ## n(unsigned int value)		\
	{								\
		write_c0_ ## r ## n(value);				\
	}								\

__define_perf_accessors(perfcntr, 0)
__define_perf_accessors(perfcntr, 1)

__define_perf_accessors(perfctrl, 0)
__define_perf_accessors(perfctrl, 1)

struct op_mips_model op_model_vr5500_ops;

static struct vr5500_register_config {
	unsigned int control[NUM_COUNTERS];
	unsigned int counter[NUM_COUNTERS];
} reg;

/* Compute all of the registers in preparation for enabling profiling.  */
static void vr5500_reg_setup(struct op_counter_config *ctr)
{
	int i;
	unsigned int counters = NUM_COUNTERS;

	/* Compute the performance counter control word.  */
	for (i = 0; i < counters; i++) {
		reg.control[i] = 0;
		reg.counter[i] = 0;

		if (!ctr[i].enabled)
			continue;

		reg.control[i] = M_PERFCTL_EVENT(ctr[i].event) |
		    M_PERFCTL_INTERRUPT_ENABLE | M_PERFCTL_COUNT_ENABLE;
		if (ctr[i].kernel)
			reg.control[i] |= M_PERFCTL_KERNEL;
		if (ctr[i].user)
			reg.control[i] |= M_PERFCTL_USER;
		if (ctr[i].exl)
			reg.control[i] |= M_PERFCTL_EXL;

		reg.counter[i] = 0xffffffff - ctr[i].count + 1;
	}
}

/* Program all of the registers in preparation for enabling profiling.  */
static void vr5500_cpu_setup(void *args)
{
	w_c0_perfctrl1(0);
	w_c0_perfcntr1(reg.counter[1]);

	w_c0_perfctrl0(0);
	w_c0_perfcntr0(reg.counter[0]);
}

/* Start all counters on current CPU */
static void vr5500_cpu_start(void *args)
{
	w_c0_perfctrl1(reg.control[1]);
	w_c0_perfctrl0(reg.control[0]);
}

/* Stop all counters on current CPU */
static void vr5500_cpu_stop(void *args)
{
	w_c0_perfctrl1(0);
	w_c0_perfctrl0(0);
}

static int vr5500_perfcount_handler(void)
{
	unsigned int control;
	unsigned int counter;
	int handled = IRQ_NONE;
	unsigned int counters = NUM_COUNTERS;

	if (cpu_has_mips_r2 && !(read_c0_cause() & (1 << 26)))
		return handled;

	switch (counters) {
	#define HANDLE_COUNTER(n) 					\
	case n + 1:							\
		control = r_c0_perfctrl ## n();				\
		counter = r_c0_perfcntr ## n();				\
		if ((control & M_PERFCTL_INTERRUPT_ENABLE) &&		\
			(control & M_PERFCTL_INTERRUPT)) {		\
			oprofile_add_sample(get_irq_regs(), n);		\
			w_c0_perfcntr ## n(reg.counter[n]);		\
			w_c0_perfctrl ## n(control & ~M_PERFCTL_INTERRUPT); \
			handled = IRQ_HANDLED;				\
		}
	HANDLE_COUNTER(1)
	HANDLE_COUNTER(0)
	}

	return handled;
}

static void reset_counters(void *arg)
{
	w_c0_perfctrl1(0);
	w_c0_perfcntr1(0);

	w_c0_perfctrl0(0);
	w_c0_perfcntr0(0);
}

static int __init vr5500_init(void)
{
	on_each_cpu(reset_counters, NULL, 1);

	switch (current_cpu_type()) {
	case CPU_R5500:
		op_model_vr5500_ops.cpu_type = "mips/vr5500";
		break;

	default:
		printk(KERN_ERR "Profiling unsupported for this CPU\n");

		return -ENODEV;
	}

	save_perf_irq = perf_irq;
	perf_irq = vr5500_perfcount_handler;

	return 0;
}

static void vr5500_exit(void)
{
	on_each_cpu(reset_counters, NULL, 1);

	perf_irq = save_perf_irq;
}

struct op_mips_model op_model_vr5500_ops = {
	.reg_setup = vr5500_reg_setup,
	.cpu_setup = vr5500_cpu_setup,
	.init = vr5500_init,
	.exit = vr5500_exit,
	.cpu_start = vr5500_cpu_start,
	.cpu_stop = vr5500_cpu_stop,
	.num_counters = NUM_COUNTERS,
};
