// SPDX-License-Identifier: GPL-2.0
/*
 * aegis.c - AEGIS: Adaptive Energy-Gated Idle Subsystem (v0.1)
 *
 * Author: Steambot12 <wilfridus.it@gmail.com>
 *
 * An energy-ledger cpuidle governor that gates deep idle states on paid
 * residency rather than statistical runqueue heuristics. Every state carries
 * a dynamic debt seeded at its target residency:
 *
 *     pay(i) = debt[i] * (1 + h * (1 - C))
 *
 * where C is the per-CPU calmness in [0, 1] and h is the headroom tunable.
 * Calm CPUs pay near face value and descend aggressively to deep states,
 * stopping the scheduler tick to allow hardware power collapse down to
 * low-power standby (10mA - 50mA).
 *
 * Transient background service wakeups decay over a 1-second quiet window,
 * while wakeups from sustained deep sleep retain standby affinity to return
 * to deep sleep within 5 seconds of event completion.
 *
 * Integer math, Q12.20 fixed-point; select & reflect run IRQ-disabled.
 */

#define pr_fmt(fmt) "aegis: " fmt

#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/ktime.h>
#include <linux/percpu.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/tick.h>
#include <linux/timekeeping.h>
#include <linux/types.h>

#define AEGIS_VERSION			"0.1"

#define AEGIS_SHIFT			20
#define AEGIS_ONE			(1 << AEGIS_SHIFT)

/* Debt ratchet: 1/16 down on a win, 1/4 up on a miss. */
#define AEGIS_DEBT_SHRINK_SHIFT		4
#define AEGIS_DEBT_GROW_SHIFT		2

/* Catastrophic miss threshold: residency < debt/4 */
#define AEGIS_DEAD_SHIFT		2
#define AEGIS_CATASTROPHE_MULT		2

/* Calmness recovery: 1/8 of the remaining gap per paid debt */
#define AEGIS_CALM_UP_SHIFT		3

/* Headroom h in thousandths: 1.0 default, max 4.0 */
#define AEGIS_DEFAULT_HEADROOM_MILLTHS	1000
#define AEGIS_MAX_HEADROOM_MILLTHS	4000

/* Cold-start calmness: 7/8 (~87.5% calm) for immediate deep-sleep entry */
#define AEGIS_COLD_CALM			(AEGIS_ONE * 7 / 8)

/* Temporal recovery: 1 second of quiet auto-restores calm and decays debt */
#define AEGIS_RECOVERY_TIME_NS		(1000000000ULL)

/* Deep sleep standby affinity: 5 seconds after a long sleep (>= 50ms) */
#define AEGIS_DEEP_AFFINITY_NS		(5000000000ULL)
#define AEGIS_LONG_SLEEP_NS		(50ULL * NSEC_PER_MSEC)

struct aegis_stats {
	u64	selects;
	u64	wfi_settle;	/* windows no state could pay: settled at WFI */
	u64	misses;		/* early wakes */
	u64	deep_entries;	/* entries into TIMER_STOP / deep C-states */
	u64	deep_affinity;	/* fast deep entries under standby affinity */
	u64	residency_ns;
};

struct aegis_cpu_data {
	s64	debt[CPUIDLE_STATE_MAX];
	u32	calm;		/* Q12.20, [0, AEGIS_ONE] */
	u64	last_wake_ns;	/* monotonic timestamp of last wakeup */
	u64	last_miss_ns;	/* monotonic timestamp of last miss */
	bool	deep_affinity;	/* recently woke from deep sleep (standby mode) */
	unsigned int headroom_millths;
	bool	initialized;
	bool	reset_pending;	/* set by sysfs, consumed in select */
	struct aegis_stats stats;
};

static DEFINE_PER_CPU(struct aegis_cpu_data, aegis_data);

static inline s64 aegis_min_s64(s64 a, s64 b)
{
	return a < b ? a : b;
}

static inline s64 aegis_max_s64(s64 a, s64 b)
{
	return a > b ? a : b;
}

static void aegis_reset_ledger(struct aegis_cpu_data *d,
			       struct cpuidle_driver *drv)
{
	int i;

	for (i = 1; i < drv->state_count && i < CPUIDLE_STATE_MAX; i++)
		d->debt[i] = drv->states[i].target_residency_ns;

	d->debt[0] = 0;
	d->calm = AEGIS_COLD_CALM;
	d->last_wake_ns = 0;
	d->last_miss_ns = 0;
	d->deep_affinity = false;
}

/*
 * aegis_decay_penalties - Auto-decay miss penalties over time.
 * If more than AEGIS_RECOVERY_TIME_NS (1s) has passed since the last miss,
 * background service bursts have settled. Rapidly restore calmness towards
 * AEGIS_COLD_CALM and decay any inflated debt back toward baseline DT targets.
 */
static void aegis_decay_penalties(struct aegis_cpu_data *d,
				  struct cpuidle_driver *drv,
				  u64 now_ns)
{
	u64 elapsed;
	int i;

	if (!d->last_miss_ns)
		return;

	elapsed = now_ns - d->last_miss_ns;
	if (elapsed < AEGIS_RECOVERY_TIME_NS)
		return;

	if (d->calm < AEGIS_COLD_CALM)
		d->calm = min_t(u32, d->calm + ((AEGIS_COLD_CALM - d->calm) >> 1),
				AEGIS_COLD_CALM);

	for (i = 1; i < drv->state_count && i < CPUIDLE_STATE_MAX; i++) {
		s64 base = drv->states[i].target_residency_ns;
		if (d->debt[i] > base) {
			s64 excess = d->debt[i] - base;
			d->debt[i] -= (excess >> 1);
		}
	}

	if (d->calm >= AEGIS_COLD_CALM)
		d->last_miss_ns = 0;
	else
		d->last_miss_ns = now_ns;
}

static int aegis_select(struct cpuidle_driver *drv, struct cpuidle_device *dev,
			bool *stop_tick)
{
	struct aegis_cpu_data *d = this_cpu_ptr(&aegis_data);
	ktime_t delta_tick;
	s64 headroom, charge_scale, pay;
	s64 window_s64;
	u64 window_ns, now_ns;
	int i, idx = 0;
	s64 latency_req;
	bool in_deep_affinity;

	if (unlikely(!d->initialized || drv->state_count <= 1)) {
		*stop_tick = false;
		return 0;
	}

	if (unlikely(d->reset_pending)) {
		aegis_reset_ledger(d, drv);
		d->reset_pending = false;
	}

	now_ns = ktime_get_mono_fast_ns();

	/* Auto-recover calm and decay penalty if quiet (< 5s target) */
	aegis_decay_penalties(d, drv, now_ns);

	latency_req = cpuidle_governor_latency_req(dev->cpu);
	if (latency_req < 0)
		latency_req = PM_QOS_LATENCY_ANY_NS;

	/* Next event timer clamp: never negative */
	window_s64 = ktime_to_ns(tick_nohz_get_sleep_length(&delta_tick));
	window_ns = window_s64 < 0 ? 0 : (u64)window_s64;

	/*
	 * Check Standby Deep Affinity:
	 * If the CPU recently finished a true deep sleep (residency >= 50ms)
	 * within the last 5 seconds, it is handling a brief event (e.g. notification).
	 * Bypass headroom charge to plunge immediately back into deep state.
	 */
	in_deep_affinity = d->deep_affinity &&
			   (now_ns - d->last_wake_ns < AEGIS_DEEP_AFFINITY_NS);
	if (!in_deep_affinity && d->deep_affinity &&
	    (now_ns - d->last_wake_ns >= AEGIS_DEEP_AFFINITY_NS)) {
		d->deep_affinity = false;
	}

	/* charge(i) = debt[i] * (1 + h * (1 - calm)), h in Q12.20 */
	if (in_deep_affinity) {
		charge_scale = AEGIS_ONE;
		d->stats.deep_affinity++;
	} else {
		headroom = ((s64)d->headroom_millths << AEGIS_SHIFT) / 1000;
		charge_scale = AEGIS_ONE +
			      (((s64)headroom * (AEGIS_ONE - (s64)d->calm)) >>
			       AEGIS_SHIFT);
	}

	/*
	 * Aggressive Energy-Gated Descent:
	 * Search backwards from deepest state (e.g. C4 / Cluster Power Down)
	 * to state 1. Heavy background services with brief sleep intervals
	 * are accommodated by granting deep state whenever the sleep window
	 * covers the energy gate.
	 */
	for (i = min_t(int, drv->state_count - 1, CPUIDLE_STATE_MAX - 1); i >= 1; i--) {
		const struct cpuidle_state *st = &drv->states[i];

		if (dev->states_usage[i].disable)
			continue;
		if (st->flags & CPUIDLE_FLAG_POLLING)
			continue;
		if (st->flags & CPUIDLE_FLAG_UNUSABLE)
			continue;
		if (st->exit_latency_ns > (u64)latency_req)
			continue;

		pay = ((s64)d->debt[i] * charge_scale) >> AEGIS_SHIFT;
		if ((s64)window_ns < pay)
			continue;

		idx = i;
		break;
	}

	if (!idx) {
		/*
		 * Settle at WFI (state 0).
		 * If expected sleep covers a tick period, stop the tick
		 * so the core is not periodically awakened by the scheduler.
		 */
		d->stats.wfi_settle++;
		d->stats.selects++;
		if (window_ns >= TICK_NSEC)
			*stop_tick = true;
		else
			*stop_tick = tick_nohz_tick_stopped();
		return 0;
	}

	/*
	 * For deep state (idx > 0):
	 * In order to reach 10mA - 50mA low discharging current, the local
	 * scheduler tick MUST be stopped whenever entering deep states.
	 * Only keep the tick running if the expected window is shorter than
	 * TICK_NSEC AND the state does not stop the timer AND tick is not
	 * already stopped.
	 */
	if ((drv->states[idx].flags & CPUIDLE_FLAG_TIMER_STOP) ||
	    window_ns >= TICK_NSEC || tick_nohz_tick_stopped()) {
		*stop_tick = true;
	} else {
		/*
		 * Window is short and tick runs: demote if target residency
		 * exceeds delta_tick to ensure safe wakeup.
		 */
		*stop_tick = false;
		while (idx > 0 &&
		       drv->states[idx].target_residency_ns > (u64)delta_tick) {
			idx--;
			while (idx > 0 && (dev->states_usage[idx].disable ||
			       (drv->states[idx].flags & CPUIDLE_FLAG_POLLING) ||
			       (drv->states[idx].flags & CPUIDLE_FLAG_UNUSABLE)))
				idx--;
		}
	}

	if (drv->states[idx].flags & CPUIDLE_FLAG_TIMER_STOP)
		d->stats.deep_entries++;

	d->stats.selects++;
	return idx;
}

static void aegis_reflect(struct cpuidle_device *dev, int index)
{
	struct aegis_cpu_data *d = this_cpu_ptr(&aegis_data);
	struct cpuidle_driver *drv = cpuidle_get_cpu_driver(dev);
	s64 debt, lo, hi, next;
	u64 measured = dev->last_residency_ns;
	u64 now_ns;

	dev->last_state_idx = index;

	if (unlikely(!drv || !d->initialized))
		return;

	now_ns = ktime_get_mono_fast_ns();
	d->last_wake_ns = now_ns;
	d->stats.residency_ns += measured;

	/*
	 * Standby Deep Sleep Detection:
	 * If the CPU was in deep sleep for >= 50ms, mark deep_affinity = true
	 * and restore full calmness (AEGIS_ONE).
	 * Subsequent wake bursts (e.g. push notifications) will return to
	 * deep sleep within < 5s.
	 */
	if (index >= 1 && measured >= AEGIS_LONG_SLEEP_NS) {
		d->deep_affinity = true;
		d->calm = AEGIS_ONE;
	}

	/*
	 * Anti-Trap for State 0 (WFI):
	 * If the CPU was settled in WFI but actually slept longer than
	 * state 1's target residency, the energy gate was overly strict.
	 * Lower state 1's debt and gently boost calmness so the CPU is not
	 * stuck in WFI loops.
	 */
	if (index == 0) {
		if (drv->state_count > 1 &&
		    measured >= drv->states[1].target_residency_ns) {
			s64 d1 = d->debt[1];
			s64 d1_lo = aegis_max_s64((s64)drv->states[1].target_residency_ns / 2, 1);
			d->debt[1] = aegis_max_s64(d1 - (d1 >> AEGIS_DEBT_SHRINK_SHIFT), d1_lo);
			d->calm = min_t(u32, d->calm + ((AEGIS_ONE - d->calm) >> 4), AEGIS_ONE);
		}
		return;
	}

	if (index >= drv->state_count || index >= CPUIDLE_STATE_MAX)
		return;

	debt = d->debt[index];
	/*
	 * Dynamic debt range:
	 * Floor at target_residency / 2 allows deep states to be unlocked easily
	 * during calm periods. Ceiling at 2x target residency protects against
	 * severe thrashing.
	 */
	lo = aegis_max_s64((s64)drv->states[index].target_residency_ns / 2, 1);
	hi = 2 * (s64)drv->states[index].target_residency_ns;

	if ((s64)measured >= debt) {
		/*
		 * Paid: Shrink the debt and creep the calmness up.
		 */
		next = debt - (debt >> AEGIS_DEBT_SHRINK_SHIFT);
		d->debt[index] = aegis_max_s64(next, lo);
		d->calm = min_t(u32, d->calm + ((AEGIS_ONE - d->calm) >>
						 AEGIS_CALM_UP_SHIFT),
				AEGIS_ONE);
	} else if ((s64)measured < (debt >> AEGIS_DEAD_SHIFT)) {
		/*
		 * Catastrophic Miss: Early wake (< debt / 4).
		 * Double the debt up to ceiling 'hi'.
		 * Dampen calmness, but if under deep affinity, protect calm from
		 * complete collapse so notification bursts (< 5s) do not
		 * permanently ruin deep sleep.
		 */
		d->last_miss_ns = now_ns;
		d->stats.misses++;
		d->debt[index] = aegis_min_s64(debt * AEGIS_CATASTROPHE_MULT, hi);

		if (d->deep_affinity)
			d->calm >>= 1;
		else
			d->calm >>= 2;
	} else {
		/*
		 * Ordinary Miss: Normal IRQ or timer woke the CPU.
		 * Grow the debt only; DO NOT penalize calmness.
		 * Normal IRQ activity must never lock the CPU out of deep sleep!
		 */
		next = debt + (debt >> AEGIS_DEBT_GROW_SHIFT);
		d->debt[index] = aegis_min_s64(next, hi);
		d->stats.misses++;
	}
}

static int aegis_enable_device(struct cpuidle_driver *drv,
			       struct cpuidle_device *dev)
{
	struct aegis_cpu_data *d = per_cpu_ptr(&aegis_data, dev->cpu);
	struct aegis_stats stats = d->stats;
	unsigned int headroom = d->headroom_millths;

	memset(d, 0, sizeof(*d));
	d->stats = stats;
	d->headroom_millths = headroom ? headroom : AEGIS_DEFAULT_HEADROOM_MILLTHS;

	aegis_reset_ledger(d, drv);
	d->initialized = true;

	return 0;
}

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%s\n", AEGIS_VERSION);
}

static ssize_t stats_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	struct aegis_stats s = { };
	int cpu, len = 0;

	for_each_possible_cpu(cpu) {
		const struct aegis_stats *c =
			&per_cpu_ptr(&aegis_data, cpu)->stats;

		s.selects += c->selects;
		s.wfi_settle += c->wfi_settle;
		s.misses += c->misses;
		s.deep_entries += c->deep_entries;
		s.deep_affinity += c->deep_affinity;
		s.residency_ns += c->residency_ns;
	}

	len += sysfs_emit_at(buf, len, "selects: %llu\n", s.selects);
	len += sysfs_emit_at(buf, len, "wfi_settle: %llu\n", s.wfi_settle);
	len += sysfs_emit_at(buf, len, "misses: %llu\n", s.misses);
	len += sysfs_emit_at(buf, len, "miss_rate_permil: %llu\n",
			     s.selects ? s.misses * 1000 / s.selects : 0);
	len += sysfs_emit_at(buf, len, "deep_entries: %llu\n",
			     s.deep_entries);
	len += sysfs_emit_at(buf, len, "deep_affinity_hits: %llu\n",
			     s.deep_affinity);
	len += sysfs_emit_at(buf, len, "total_residency_ms: %llu\n",
			     s.residency_ns / NSEC_PER_MSEC);
	return len;
}

/* Calmness of the first online CPU, permille */
static ssize_t calm_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	int cpu = cpumask_first(cpu_online_mask);
	struct aegis_cpu_data *d;

	if (cpu >= nr_cpu_ids)
		return sysfs_emit(buf, "0\n");

	d = per_cpu_ptr(&aegis_data, cpu);
	return sysfs_emit(buf, "%u\n",
			  (unsigned int)((u64)d->calm * 1000 >> AEGIS_SHIFT));
}

/* Per-state debt of the first online CPU, microseconds */
static ssize_t debt_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	int cpu = cpumask_first(cpu_online_mask);
	struct aegis_cpu_data *d;
	struct cpuidle_driver *drv;
	struct cpuidle_device *dev;
	int i, len = 0;

	if (cpu >= nr_cpu_ids)
		return sysfs_emit(buf, "0\n");

	dev = per_cpu(cpuidle_devices, cpu);
	if (!dev)
		return sysfs_emit(buf, "0\n");

	drv = cpuidle_get_cpu_driver(dev);
	if (!drv)
		return sysfs_emit(buf, "0\n");

	d = per_cpu_ptr(&aegis_data, cpu);

	for (i = 1; i < drv->state_count && i < CPUIDLE_STATE_MAX; i++)
		len += sysfs_emit_at(buf, len, "state%u: %llu%s",
				     i, (u64)d->debt[i] / 1000,
				     (i + 1 < drv->state_count && i + 1 < CPUIDLE_STATE_MAX) ? " " : "\n");
	return len;
}

static ssize_t headroom_show(struct kobject *kobj,
			     struct kobj_attribute *attr, char *buf)
{
	int cpu = cpumask_first(cpu_online_mask);

	if (cpu >= nr_cpu_ids)
		return sysfs_emit(buf, "0\n");
	return sysfs_emit(buf, "%u\n",
			  per_cpu_ptr(&aegis_data, cpu)->headroom_millths);
}

/*
 * Headroom h in thousandths: 0 -> pay the debt at face value (maximum
 * energy, deep states unlocked as soon as window covers debt);
 * 4000 -> charge up to 5x debt when disturbed. Applies to all CPUs.
 */
static ssize_t headroom_store(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned int val;
	int cpu;

	if (kstrtouint(buf, 10, &val) || val > AEGIS_MAX_HEADROOM_MILLTHS)
		return -EINVAL;

	for_each_possible_cpu(cpu)
		per_cpu_ptr(&aegis_data, cpu)->headroom_millths = val;

	return count;
}

/*
 * "echo all > reset": each CPU resets its own ledger from inside
 * aegis_select(), ensuring thread-safe per-CPU updates.
 */
static ssize_t reset_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t count)
{
	int cpu;

	if (!sysfs_streq(buf, "all"))
		return -EINVAL;

	for_each_possible_cpu(cpu)
		per_cpu_ptr(&aegis_data, cpu)->reset_pending = true;

	return count;
}

static struct kobj_attribute version_attr	= __ATTR_RO(version);
static struct kobj_attribute stats_attr		= __ATTR_RO(stats);
static struct kobj_attribute calm_attr		= __ATTR_RO(calm);
static struct kobj_attribute debt_attr		= __ATTR_RO(debt);
static struct kobj_attribute headroom_attr	= __ATTR_RW(headroom);
static struct kobj_attribute reset_attr		= __ATTR_WO(reset);

static struct attribute *aegis_attrs[] = {
	&version_attr.attr,
	&stats_attr.attr,
	&calm_attr.attr,
	&debt_attr.attr,
	&headroom_attr.attr,
	&reset_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(aegis);

static struct kobject *aegis_kobj;

static struct cpuidle_governor aegis_governor = {
	.name	= "aegis",
	.rating	= 25,
	.enable	= aegis_enable_device,
	.select	= aegis_select,
	.reflect = aegis_reflect,
};

static int __init aegis_init(void)
{
	int ret;

	ret = cpuidle_register_governor(&aegis_governor);
	if (ret)
		return ret;

	/* cpu_dev_init() runs in driver_init(), before any initcall. */
	aegis_kobj = kobject_create_and_add("aegis",
					     &cpu_subsys.dev_root->kobj);
	if (aegis_kobj && sysfs_create_groups(aegis_kobj, aegis_groups)) {
		kobject_put(aegis_kobj);
		aegis_kobj = NULL;
	}
	if (!aegis_kobj)
		pr_warn("sysfs init failed, continuing without tunables\n");

	return 0;
}
postcore_initcall(aegis_init);
