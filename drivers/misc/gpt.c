#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <plat/dmtimer.h>
#include <plat/clock.h>

#include <mach/irqs.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthew R. Laue");

/*
 * Transition Capture Mode
 *   0x1 -> rising edge
 *   0x2 -> falling edge
 *   0x3 -> both
 */
int tcm = 0x1;
module_param(tcm, int, 0644);

long counter[3];
module_param_array(counter, long, NULL, 0444);

#define GPT_CLK_ID 10
#define GPT_FCK "gpt10_fck"

struct omap_dm_timer *gptimer;

static irqreturn_t gpt10_interrupt(int irq, void *dev_id)
{
    unsigned int l;
    struct omap_dm_timer *gpt = (struct omap_dm_timer *)dev_id;

    l = omap_dm_timer_read_counter(gpt);

    counter[2] = counter[1];
    counter[1] = counter[0];
    counter[0] = l;

    //omap_dm_timer_write_status(gpt, OMAP_TIMER_INT_OVERFLOW);
    return IRQ_HANDLED;
}

static int __init
gpt_init(void)
{
    u32 rate;
    int id = GPT_CLK_ID, irq, rc;
    struct clk *clk;
    int flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL;

    if (tcm < 1 || tcm > 3) {
        printk(KERN_ERR "[gpt] invalid transition capture mode: %d\n", tcm);
        return -1;
    }

    gptimer = omap_dm_timer_request_specific(id);
    if (gptimer == NULL) {
        printk(KERN_ERR "[gpt] failed to request timer: %d\n", id);
        return -1;
    }

    irq = omap_dm_timer_get_irq(gptimer);
    rc = request_irq(irq, gpt10_interrupt, flags, "gpt10", (void *)gptimer);
    if (rc != 0) {
        printk(KERN_ERR "[gpt] failed to get IRQ%d: %d\n", irq, rc);
        return -1;
    }

    clk = omap_dm_timer_get_fclk(gptimer);
    rate = clk_get_rate(clk);

#if 0 // enable this to see 1 per second interrupts (test only)
    omap_dm_timer_set_int_enable(gptimer, OMAP_TIMER_INT_OVERFLOW);
    omap_dm_timer_set_load_start(gptimer, 1, ~rate);
#else
    omap_dm_timer_set_tcm(gptimer, tcm);
    omap_dm_timer_set_int_enable(gptimer, OMAP_TIMER_INT_CAPTURE);
    omap_dm_timer_start(gptimer);
#endif

    printk("[gpt] GPTIMER%d at %u HZ : IRQ%d\n", id, rate, irq);
    return 0;
}

static void __exit
gpt_exit(void) {
    omap_dm_timer_stop(gptimer);
    omap_dm_timer_set_int_enable(gptimer, 0);
    free_irq(omap_dm_timer_get_irq(gptimer), (void *)gptimer);
    omap_dm_timer_free(gptimer);
    printk("[gpt] exit.\n");
}

module_init(gpt_init);
module_exit(gpt_exit);
