// SPDX-License-Identifier: GPL-2.0
/*
 * aegis.c - AEGIS: Adaptive Energy-Gated Idle Subsystem (v0.2)
 *
 * Author: Steambot12 <wilfridus.it@gmail.com>
 *
 * Energy-ledger cpuidle governor gating deep idle states on paid residency.
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

#define AEGIS_VERSION			"0.2"

#define AEGIS_SHIFT			20
#define AEGIS_ONE			(1 << AEGIS_SHIFT)

/* Debt ratchet and calm recovery shifts */
#define AEGIS_DEBT_SHRINK_SHIFT		4
#define AEGIS_DEBT_GROW_SHIFT		2
#define AEGIS_DEAD_SHIFT		2
#define AEGIS_CALM_UP_SHIFT		2

#define AEGIS_DEFAULT_HEADROOM_MILLTHS	500
#define AEGIS_MAX_HEADROOM_MILLTHS	2000

#define AEGIS_COLD_CALM			AEGIS_ONE
#define AEGIS_RECOVERY_TIME_NS		(100ULL * NSEC_PER_MSEC)
#define AEGIS_DEEP_AFFINITY_NS		(5000000000ULL)
#define AEGIS_LONG_SLEEP_NS		(25ULL * NSEC_PER_MSEC)

struct aegis_stats {
	u64	selects;
	u64	wfi_settle;
	u64	misses;
	u64	deep_entries;
	u64	deep_affinity;
	u64	residency_ns;
};

struct aegis_cpu_data {
	s64	debt[CPUIDLE_STATE_MAX];
	u32	calm;
	u64	last_wake_ns;
	u64	last_miss_ns;
	bool	deep_affinity;
	unsigned int headroom_millths;
	bool	initialized;
	bool	reset_pending;
	struct aegis_stats stats;
};

static DEFINE_PER_CPU(struct aegis_cpu_data, aegis_data);

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
	aegis_decay_penalties(d, drv, now_ns);

	latency_req = cpuidle_governor_latency_req(dev->cpu);
	if (latency_req < 0)
		latency_req = PM_QOS_LATENCY_ANY_NS;

	window_s64 = ktime_to_ns(tick_nohz_get_sleep_length(&delta_tick));
	window_ns = window_s64 < 0 ? 0 : (u64)window_s64;

	in_deep_affinity = d->deep_affinity &&
			   (now_ns - d->last_wake_ns < AEGIS_DEEP_AFFINITY_NS);
	if (!in_deep_affinity && d->deep_affinity &&
	    (now_ns - d->last_wake_ns >= AEGIS_DEEP_AFFINITY_NS))
		d->deep_affinity = false;

	if (in_deep_affinity) {
		charge_scale = AEGIS_ONE;
		d->stats.deep_affinity++;
	} else {
		headroom = ((s64)d->headroom_millths << AEGIS_SHIFT) / 1000;
		charge_scale = AEGIS_ONE +
			      (((s64)headroom * (AEGIS_ONE - (s64)d->calm)) >>
			       AEGIS_SHIFT);
	}

	/* Search backwards from deepest state to state 1 */
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

		if (in_deep_affinity) {
			pay = (s64)st->target_residency_ns;
		} else {
			pay = ((s64)d->debt[i] * charge_scale) >> AEGIS_SHIFT;
			pay = clamp_t(s64, pay, (s64)st->target_residency_ns,
				      (s64)st->target_residency_ns * 3 / 2);
		}

		if ((s64)window_ns < pay)
			continue;

		idx = i;
		break;
	}

	/*
	 * Keep the scheduler tick stopped unless the selected state polls or
	 * the expected sleep window is shorter than one tick while the tick
	 * is still running. Clearing *stop_tick blindly on shallow or
	 * non-TIMER_STOP states wakes idle CPUs every HZ tick.
	 */
	if (((drv->states[idx].flags & CPUIDLE_FLAG_POLLING) ||
	     window_ns < TICK_NSEC) && !tick_nohz_tick_stopped()) {
		*stop_tick = false;

		if (delta_tick <= 0) {
			idx = 0;
		} else {
			while (idx > 0 &&
			       drv->states[idx].target_residency_ns > (u64)delta_tick) {
				idx--;
				while (idx > 0 && (dev->states_usage[idx].disable ||
				       (drv->states[idx].flags & CPUIDLE_FLAG_POLLING) ||
				       (drv->states[idx].flags & CPUIDLE_FLAG_UNUSABLE) ||
				       drv->states[idx].exit_latency_ns > (u64)latency_req))
					idx--;
			}
		}
	}

	if (!idx)
		d->stats.wfi_settle++;
	else if (drv->states[idx].flags & CPUIDLE_FLAG_TIMER_STOP)
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

	if (index >= 1 && measured >= AEGIS_LONG_SLEEP_NS) {
		d->deep_affinity = true;
		d->calm = AEGIS_ONE;
	}

	/* State 0 anti-trap */
	if (index == 0) {
		if (drv->state_count > 1 &&
		    measured >= (drv->states[1].target_residency_ns >> 1)) {
			s64 d1 = d->debt[1];
			s64 d1_lo = max_t(s64, (s64)drv->states[1].target_residency_ns, 1);
			d->debt[1] = max_t(s64, d1 - (d1 >> AEGIS_DEBT_SHRINK_SHIFT), d1_lo);
			d->calm = min_t(u32, d->calm + ((AEGIS_ONE - d->calm) >> 2), AEGIS_ONE);
		}
		return;
	}

	if (index >= drv->state_count || index >= CPUIDLE_STATE_MAX)
		return;

	debt = d->debt[index];
	lo = max_t(s64, (s64)drv->states[index].target_residency_ns, 1);
	hi = max_t(s64, (s64)drv->states[index].target_residency_ns * 3 / 2, lo);

	if ((s64)measured >= debt) {
		next = debt - (debt >> AEGIS_DEBT_SHRINK_SHIFT);
		d->debt[index] = max_t(s64, next, lo);
		d->calm = min_t(u32, d->calm + ((AEGIS_ONE - d->calm) >>
						 AEGIS_CALM_UP_SHIFT),
				AEGIS_ONE);
	} else if ((s64)measured < (debt >> AEGIS_DEAD_SHIFT)) {
		d->last_miss_ns = now_ns;
		d->stats.misses++;
		d->debt[index] = min_t(s64, debt + (debt >> 2), hi);

		if (d->deep_affinity)
			d->calm = max_t(u32, d->calm - (d->calm >> 3), AEGIS_ONE >> 1);
		else
			d->calm = max_t(u32, d->calm - (d->calm >> 2), AEGIS_ONE >> 2);
	} else {
		next = debt + (debt >> AEGIS_DEBT_GROW_SHIFT);
		d->debt[index] = min_t(s64, next, hi);
		if (d->debt[index] > lo && !d->last_miss_ns)
			d->last_miss_ns = now_ns;
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
