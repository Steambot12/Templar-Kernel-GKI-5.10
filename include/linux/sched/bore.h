/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/linux/sched/bore.h - BORE (Burst-Oriented Response Enhancer)
 *
 * BORE CPU Scheduler modification 7.0.0 (upstream firelzrd/bore-scheduler,
 * stable 1e2f57ad) adapted to this tree's GKI layout: the per-task state
 * lives in KABI slots (see include/linux/sched.h), not in new struct
 * fields. When CONFIG_SCHED_BORE is off nothing in here compiles and the
 * KABI slots stay reserved, so the KMI is untouched.
 */
#ifndef _LINUX_SCHED_BORE_H
#define _LINUX_SCHED_BORE_H

#include <linux/sched.h>

#define SCHED_BORE_AUTHOR   "Masahito Suzuki"
#define SCHED_BORE_PROGNAME "BORE CPU Scheduler modification"
#define SCHED_BORE_VERSION  "7.0.0"

/* Sysctls (defined in kernel/sched/fair.c). */
extern u8   __read_mostly sched_bore;
extern u8   __read_mostly sched_burst_inherit_type;
extern u8   __read_mostly sched_burst_protect_slice_lv;
extern u8   __read_mostly sched_burst_smoothness;
extern u8   __read_mostly sched_burst_penalty_offset;
extern uint __read_mostly sched_burst_penalty_scale;
extern uint __read_mostly sched_burst_cache_lifetime;
extern uint __read_mostly sched_credit_cap_us;

/*
 * Static keys (defined in kernel/sched/fair.c) gate the hot paths so a
 * booted kernel can disable BORE entirely at runtime without recompiling.
 */
DECLARE_STATIC_KEY_TRUE(sched_bore_key);
DECLARE_STATIC_KEY_TRUE(sched_burst_inherit_key);
DECLARE_STATIC_KEY_TRUE(sched_burst_ancestor_key);
DECLARE_STATIC_KEY_TRUE (sched_burst_protect_slice_cond_key);
DECLARE_STATIC_KEY_FALSE(sched_burst_protect_slice_prefer_key);
DECLARE_STATIC_KEY_FALSE(sched_credit_key);

/* Effective nice priority under BORE: static prio + demotion score. */
static inline u8 effective_prio_bore(struct task_struct *p)
{
	int prio = p->static_prio - MAX_RT_PRIO;

	if (static_branch_likely(&sched_bore_key))
		prio += p->se.burst_score;
	return min(39, prio);
}

/*
 * 7.0.0 sleep credit: stamp the sleep time here, on the next wakeup
 * place_entity() shortens the deadline by the capped sleep duration.
 */
extern void bore_note_sleep(struct task_struct *p, u64 now);
extern u64  bore_credit_ns(struct task_struct *p);

extern void task_fork_bore(struct task_struct *p,
			   struct task_struct *parent,
			   u64 clone_flags, u64 now);
extern void reset_task_bore(struct task_struct *p);
extern void sched_init_bore(void);

extern int  sched_bore_update_handler(struct ctl_table *table,
		int write, void __user *buffer, size_t *lenp, loff_t *ppos);
extern int  sched_burst_inherit_type_update_handler(struct ctl_table *table,
		int write, void __user *buffer, size_t *lenp, loff_t *ppos);
extern int  sched_burst_protect_slice_lv_update_handler(struct ctl_table *table,
		int write, void __user *buffer, size_t *lenp, loff_t *ppos);
extern int  sched_credit_cap_us_update_handler(struct ctl_table *table,
		int write, void __user *buffer, size_t *lenp, loff_t *ppos);

#endif /* _LINUX_SCHED_BORE_H */
