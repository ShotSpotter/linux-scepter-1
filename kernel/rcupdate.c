/*
 * Read-Copy Update mechanism for mutual exclusion
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright IBM Corporation, 2001
 *
 * Authors: Dipankar Sarma <dipankar@in.ibm.com>
 *	    Manfred Spraul <manfred@colorfullife.com>
 *
 * Based on the original work by Paul McKenney <paulmck@us.ibm.com>
 * and inputs from Rusty Russell, Andrea Arcangeli and Andi Kleen.
 * Papers:
 * http://www.rdrop.com/users/paulmck/paper/rclockpdcsproof.pdf
 * http://lse.sourceforge.net/locking/rclock_OLS.2001.05.01c.sc.pdf (OLS2001)
 *
 * For detailed explanation of Read-Copy Update mechanism see -
 *		http://lse.sourceforge.net/locking/rcupdate.html
 *
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/atomic.h>
#include <linux/bitops.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/hardirq.h>

#ifdef CONFIG_DEBUG_LOCK_ALLOC
static struct lock_class_key rcu_lock_key;
struct lockdep_map rcu_lock_map =
	STATIC_LOCKDEP_MAP_INIT("rcu_read_lock", &rcu_lock_key);
EXPORT_SYMBOL_GPL(rcu_lock_map);

static struct lock_class_key rcu_bh_lock_key;
struct lockdep_map rcu_bh_lock_map =
	STATIC_LOCKDEP_MAP_INIT("rcu_read_lock_bh", &rcu_bh_lock_key);
EXPORT_SYMBOL_GPL(rcu_bh_lock_map);

static struct lock_class_key rcu_sched_lock_key;
struct lockdep_map rcu_sched_lock_map =
	STATIC_LOCKDEP_MAP_INIT("rcu_read_lock_sched", &rcu_sched_lock_key);
EXPORT_SYMBOL_GPL(rcu_sched_lock_map);
#endif

int rcu_scheduler_active __read_mostly;
EXPORT_SYMBOL_GPL(rcu_scheduler_active);

#ifdef CONFIG_DEBUG_LOCK_ALLOC

int debug_lockdep_rcu_enabled(void)
{
	return rcu_scheduler_active && debug_locks &&
	       current->lockdep_recursion == 0;
}
EXPORT_SYMBOL_GPL(debug_lockdep_rcu_enabled);

/**
 * rcu_read_lock_bh_held - might we be in RCU-bh read-side critical section?
 *
 * Check for bottom half being disabled, which covers both the
 * CONFIG_PROVE_RCU and not cases.  Note that if someone uses
 * rcu_read_lock_bh(), but then later enables BH, lockdep (if enabled)
 * will show the situation.
 *
 * Check debug_lockdep_rcu_enabled() to prevent false positives during boot.
 */
int rcu_read_lock_bh_held(void)
{
	if (!debug_lockdep_rcu_enabled())
		return 1;
	return in_softirq();
}
EXPORT_SYMBOL_GPL(rcu_read_lock_bh_held);

#endif /* #ifdef CONFIG_DEBUG_LOCK_ALLOC */

/*
 * This function is invoked towards the end of the scheduler's initialization
 * process.  Before this is called, the idle task might contain
 * RCU read-side critical sections (during which time, this idle
 * task is booting the system).  After this function is called, the
 * idle tasks are prohibited from containing RCU read-side critical
 * sections.
 */
void rcu_scheduler_starting(void)
{
	WARN_ON(num_online_cpus() != 1);
	WARN_ON(nr_context_switches() > 0);
	rcu_scheduler_active = 1;
}

/*
 * Awaken the corresponding synchronize_rcu() instance now that a
 * grace period has elapsed.
 */
void wakeme_after_rcu(struct rcu_head  *head)
{
	struct rcu_synchronize *rcu;

	rcu = container_of(head, struct rcu_synchronize, head);
	complete(&rcu->completion);
}

#ifdef CONFIG_PROVE_RCU
/*
 * wrapper function to avoid #include problems.
 */
int rcu_my_thread_group_empty(void)
{
	return thread_group_empty(current);
}
EXPORT_SYMBOL_GPL(rcu_my_thread_group_empty);
#endif /* #ifdef CONFIG_PROVE_RCU */

#ifdef CONFIG_DEBUG_OBJECTS_RCU_HEAD
static inline void debug_rcu_head_init(struct rcu_head *head)
{
	debug_object_init(head, &rcuhead_debug_descr);
}

static inline void debug_rcu_head_free(struct rcu_head *head)
{
	debug_object_free(head, &rcuhead_debug_descr);
}

/*
 * fixup_init is called when:
 * - an active object is initialized
 */
static int rcuhead_fixup_init(void *addr, enum debug_obj_state state)
{
	struct rcu_head *head = addr;

	switch (state) {
	case ODEBUG_STATE_ACTIVE:
		/*
		 * Ensure that queued callbacks are all executed.
		 * If we detect that we are nested in a RCU read-side critical
		 * section, we should simply fail, otherwise we would deadlock.
		 */
#ifndef CONFIG_PREEMPT
		WARN_ON(1);
		return 0;
#else
		if (rcu_preempt_depth() != 0 || preempt_count() != 0 ||
		    irqs_disabled()) {
			WARN_ON(1);
			return 0;
		}
		rcu_barrier();
		rcu_barrier_sched();
		rcu_barrier_bh();
		debug_object_init(head, &rcuhead_debug_descr);
		return 1;
#endif
	default:
		return 0;
	}
}

/*
 * fixup_activate is called when:
 * - an active object is activated
 * - an unknown object is activated (might be a statically initialized object)
 * Activation is performed internally by call_rcu().
 * Let's make it valid to activate a static object.
 */
static int rcuhead_fixup_activate(void *addr, enum debug_obj_state state)
{
	struct rcu_head *head = addr;

	switch (state) {

	case ODEBUG_STATE_NOTAVAILABLE:
		/*
		 * This is not really a fixup. The work struct was
		 * statically initialized. We just make sure that it
		 * is tracked in the object tracker.
		 */
		debug_object_init(head, &rcuhead_debug_descr);
		debug_object_activate(head, &rcuhead_debug_descr);
		return 0;

	case ODEBUG_STATE_ACTIVE:
		/*
		 * Ensure that queued callbacks are all executed.
		 * If we detect that we are nested in a RCU read-side critical
		 * section, we should simply fail, otherwise we would deadlock.
		 */
#ifndef CONFIG_PREEMPT
		WARN_ON(1);
		return 0;
#else
		if (rcu_preempt_depth() != 0 || preempt_count() != 0 ||
		    irqs_disabled()) {
			WARN_ON(1);
			return 0;
		}
		rcu_barrier();
		rcu_barrier_sched();
		rcu_barrier_bh();
		debug_object_activate(head, &rcuhead_debug_descr);
		return 1;
#endif
	default:
		return 0;
	}
}

/*
 * fixup_free is called when:
 * - an active object is freed
 */
static int rcuhead_fixup_free(void *addr, enum debug_obj_state state)
{
	struct rcu_head *head = addr;

	switch (state) {
	case ODEBUG_STATE_ACTIVE:
		/*
		 * Ensure that queued callbacks are all executed.
		 * If we detect that we are nested in a RCU read-side critical
		 * section, we should simply fail, otherwise we would deadlock.
		 */
#ifndef CONFIG_PREEMPT
		WARN_ON(1);
		return 0;
#else
		if (rcu_preempt_depth() != 0 || preempt_count() != 0 ||
		    irqs_disabled()) {
			WARN_ON(1);
			return 0;
		}
		rcu_barrier();
		rcu_barrier_sched();
		rcu_barrier_bh();
		debug_object_free(head, &rcuhead_debug_descr);
		return 1;
#endif
	default:
		return 0;
	}
}

void rcu_head_init_on_stack(struct rcu_head *head)
{
	debug_object_init_on_stack(head, &rcuhead_debug_descr);
	rcu_head_init(head);
}
EXPORT_SYMBOL_GPL(rcu_head_init_on_stack);

void destroy_rcu_head_on_stack(struct rcu_head *head)
{
	debug_object_free(head, &rcuhead_debug_descr);
}
EXPORT_SYMBOL_GPL(destroy_rcu_head_on_stack);

struct debug_obj_descr rcuhead_debug_descr = {
	.name = "rcu_head",
	.fixup_init = rcuhead_fixup_init,
	.fixup_activate = rcuhead_fixup_activate,
	.fixup_free = rcuhead_fixup_free,
};
EXPORT_SYMBOL_GPL(rcuhead_debug_descr);
#else	/* !CONFIG_DEBUG_OBJECTS_RCU_HEAD */
static inline void debug_rcu_head_init(struct rcu_head *head)
{
}
#endif /* #else !CONFIG_DEBUG_OBJECTS_RCU_HEAD */

/**
 * rcu_head_init - initialize a RCU head
 * @head:     the rcu head to be initialized
 */
void rcu_head_init(struct rcu_head *head)
{
	debug_rcu_head_init(head);
	head->next = NULL;
	head->func = NULL;
}
EXPORT_SYMBOL_GPL(rcu_head_init);
