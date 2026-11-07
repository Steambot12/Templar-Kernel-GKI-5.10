// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.3 — Tri-cluster, Dual-profile Governor for Linux.
 *
 * Designed for heterogeneous multi-core architectures (DynamIQ / big.LITTLE).
 * Features two distinct operating profiles switched dynamically via sysfs:
 *   - Daily Profile (gaming_mode=0): Power-centric, adaptive idle stretching,
 *     climate-aware thermal ceiling, and smooth UI frame pacing.
 *   - Gaming Profile (gaming_mode=1): Sustained render floor tracking,
 *     directional EMA filtering, bounded downward slew, and anti-sawtooth cooling.
 *
 * Author: Templar Dev (Steambot12)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/topology.h>
#include <linux/rcupdate.h>
#include <linux/sched/rt.h>
#include <linux/sched/cpufreq.h>
#include <uapi/linux/sched/types.h>
#include <linux/tick.h>
#include <linux/timekeeping.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/irq_work.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/list.h>
#define CREATE_TRACE_POINTS
#include <trace/events/vorpal.h>

/* =====================================================================
 * Section 1: Governor Metadata & Sched-Core Integration
 * ===================================================================== */

#define CPUFREQ_VORPAL_NAME	"vorpal"
#define CPUFREQ_VORPAL_VERSION	"2.3-0"
#define CPUFREQ_VORPAL_AUTHOR	"Templar Dev"

#ifndef SCHED_FLAG_SUGOV
#define SCHED_FLAG_SUGOV	0x10000000
#endif

/*
 * Optional sched helpers for in-tree optimization.
 * Weak linkage allows Vorpal to compile cleanly as a 100% standalone governor
 * on any kernel tree without requiring out-of-tree modifications.
 */
extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin) __attribute__((weak));
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin) __attribute__((weak));
extern int rfx_setattr_sugov_gki510(struct task_struct *t) __attribute__((weak));

enum schedutil_type {
	FREQUENCY_UTIL,
	ENERGY_UTIL,
};
extern unsigned long schedutil_cpu_util(int cpu, unsigned long util_cfs,
					unsigned long max, enum schedutil_type type,
					struct task_struct *p);

/* =====================================================================
 * Section 2: Tunable Constants & Architectural Defaults
 * ===================================================================== */

/* Cluster capacity thresholds (normalized to 1024) */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* ---- Daily Profile Cadence (gaming_mode=0) ---- */
#define RFX_LITTLE_RATE_US		2000	/* 2ms evaluation cadence */
#define RFX_LITTLE_UP_US		0	/* Instant up-ramp (zero touch lag) */
#define RFX_LITTLE_DOWN_US		2500

#define RFX_BIG_RATE_US			2000	/* 2ms evaluation cadence for fluid 120Hz/90Hz UI */
#define RFX_BIG_UP_US			0	/* Instant up-ramp (zero touch lag) */
#define RFX_BIG_DOWN_US			2000

#define RFX_D_IDLE_EVAL_US		10000	/* 10ms evaluation when parked at fmin (fast touch wake) */
#define RFX_D_LIGHT_EVAL_OPP_PCT	25	/* Threshold below which cadence stretches */
#define RFX_D_LIGHT_EVAL_US		6000	/* 6ms light-load evaluation */
#define RFX_D_PARK_EXIT_PCT		12	/* 12% util to immediately unpark on touch */
#define RFX_D_PARK_EXIT_EVALS		1	/* Consecutive evals needed to unpark */

/* Daily cluster caps & sustained-load lift (% of effective ceiling) */
#define RFX_D_LITTLE_CAP_PCT		65	/* 65% baseline Little cap */
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	72	/* 72% sustained Little cap */
#define RFX_D_LITTLE_LIFT_PCT		58	/* Demand trigger to lift Little */
#define RFX_D_LITTLE_DROP_PCT		45

#define RFX_D_BIG_CAP_PCT		72	/* 72% baseline Big cap (~1.75 GHz for fluid UI) */
#define RFX_D_PRIME_CAP_PCT		65	/* 65% baseline Prime cap */
#define RFX_D_BIG_LIFT_PCT		55	/* Demand trigger to lift Big for heavy scrolling */
#define RFX_D_BIG_DROP_PCT		42
#define RFX_D_BIG_SUSTAINED_CAP_PCT	82	/* 82% sustained Big cap (~2.0 GHz: zero 120Hz frame drops) */
#define RFX_D_PRIME_SUSTAINED_CAP_PCT	75	/* 75% sustained Prime cap */

/* Daily timed wake-up floor (Little cluster wake transition) */
#define RFX_D_LITTLE_FLOOR_PCT		32
#define RFX_D_LITTLE_FLOOR_ARM_PCT	15
#define RFX_D_LITTLE_FLOOR_REARM_PCT	6
#define RFX_D_LITTLE_FLOOR_NS		(80 * NSEC_PER_MSEC)

/* Daily energy-aware round-down on descent */
#define RFX_D_ENERGY_AWARE		1
#define RFX_D_ENERGY_AWARE_MIN_PCT	25
#define RFX_D_LITTLE_MIN_SAMPLE_US	3000
#define RFX_D_BIG_MIN_SAMPLE_US		2000

/* ---- Gaming Profile Cadence & Controls (gaming_mode=1) ---- */
#define RFX_FAST_RATE_US		500	/* 500us (2000 eval/s): halved CPU tax vs 250us */
#define RFX_GAMING_DOWN_US		2500	/* 2.5ms paced descent delay */
#define RFX_GAMING_DOWN_PCT_PER_2MS	2	/* 2% per 2ms: silky smooth descent without stutter */
#define RFX_G_DOWN_FAST_PCT_DEFAULT	2	/* Uniform 2% descent rate */
#define RFX_G_DOWN_FAST_MS_DEFAULT	40	/* Fast window */
#define RFX_G_MIN_SAMPLE_US_DEFAULT	3000	/* 3ms dwell before drop */
#define RFX_G_ENERGY_AWARE_DEFAULT	1

/* Gaming floors (% of effective ceiling) */
#define RFX_G_PRIME_FLOOR_PCT		38	/* Spill tier standing floor */
#define RFX_G_BIG_FLOOR_PCT		44	/* Render tier baseline floor (calibrated sweet spot) */
#define RFX_G_LITTLE_FLOOR_PCT		28	/* Low-power background floor */
#define RFX_G_IDLE_FLOOR_PCT		26	/* Gated idle cluster floor (fast cool-down) */
#define RFX_G_WARMUP_FLOOR_PCT		58	/* Render tier burst lift floor */

/* Gaming cluster caps (% of effective ceiling: sustained, smooth, no spike) */
#define RFX_G_LITTLE_CAP_PCT		65	/* Little cluster cap: cuts background heat (~1.5W) */
#define RFX_G_PRIME_CAP_PCT		78	/* Prime core sustained cap: cuts peak heat (~2W) */
#define RFX_G_BIG_CAP_PCT		82	/* Big cluster sustained war cap: optimal V/f knee */

/* Gaming sustained thermal pacer (prevents runaway heat during war) */
#define RFX_G_SUSTAINED_TRIGGER_PCT	80	/* High compute threshold */
#define RFX_G_SUSTAINED_CLEAR_PCT	55	/* Relief threshold */
#define RFX_G_SUSTAINED_WINDOW_MS	300	/* 300ms continuous heavy load */

/* Gaming feature defaults */
#define RFX_G_HISPEED_PCT_DEFAULT	56	/* F1 hispeed render floor */
#define RFX_G_GO_HISPEED_PCT_DEFAULT	90	/* F1 trigger threshold */
#define RFX_G_HISPEED_HOLD_US_DEFAULT	3000	/* F1 hold duration (3ms) */

/* Gaming demand gate & idle latch (fast thermal cool-down) */
#define RFX_G_FLOOR_GATE_PCT		22
#define RFX_G_FLOOR_GATE_EXIT_PCT	42
#define RFX_G_GATE_DWELL_MS		60	/* Fast 60ms drop to idle floor when load clears */
#define RFX_G_GATE_EXIT_DWELL_MS	30

/* Gaming warmup & asset-load handling */
#define RFX_WARMUP_RAMP_DOWN_MS		60
#define RFX_WARMUP_REARM_RAMP_DOWN_MS	300
#define RFX_GAMING_WARMUP_NS		(200 * NSEC_PER_MSEC)	/* 200ms burst window */
#define RFX_GAMING_WARMUP_MAX_NS	(600 * NSEC_PER_MSEC)	/* 600ms bound (was 2s) */
#define RFX_GAMING_WARMUP_TRIGGER_PCT	60
#define RFX_GAMING_WARMUP_EXTEND_PCT	90	/* Was 85%: avoids perpetual war extensions */
#define RFX_GAMING_WARMUP_RELEASE_PCT	40
#define RFX_GAMING_WARMUP_RELEASE_NS	(40 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_HARD_CANCEL_PCT	95
#define RFX_GAMING_WARMUP_HARD_CANCEL_EVALS	2
#define RFX_GAMING_REARM_QUIET_NS	(2000 * NSEC_PER_MSEC)

/* Gaming frame-risk & scene-change damping */
#define RFX_G_RISK_ARM_PCT		85
#define RFX_G_RISK_CLEAR_PCT		50
#define RFX_G_RISK_BOOST_NS		(60 * NSEC_PER_MSEC)	/* 60ms boost (was 120ms) */
#define RFX_G_DESCEND_DROP_PCT		15
#define RFX_G_DESCEND_FROM_PCT		85
#define RFX_G_DESCEND_HOLD_NS		(20 * 500000ULL)	/* One frame period (10ms) */

/* Gaming thermal relief band */
#define RFX_G_COOL_ENTER_PCT		80
#define RFX_G_COOL_EXIT_PCT		85
#define RFX_G_COOL_STEADY_FLOOR_PCT	32	/* 32% steady relief floor */
#define RFX_G_COOL_DEEP_PCT		60
#define RFX_G_COOL_CAP_PCT		82	/* Stricter 82% cool cap */

/* Utilization EMA decay settings */
#define RFX_EMA_DECAY_PERIOD_NS		500000	/* 500us: aligned with fast eval rate */
#define RFX_EMA_GAMING_DIVISOR		100	/* Tau ~50ms (100 * 500us) */
#define RFX_EMA_MAX_STEPS		20	/* 10ms cap: covers two 120Hz frame gaps */

/* Headroom settings */
#define RFX_HEADROOM_DAILY_HIGH		12	/* 12% headroom for fluid UI scrolling */
#define RFX_HEADROOM_DAILY_MID		6	/* 6% mid-load headroom */
#define RFX_HEADROOM_GAMING		4	/* Calibrated margin (4%) */
#define RFX_HEADROOM_GAMING_GATE	80	/* Gate 80 */
#define RFX_SAT_TO_MAX_GAMING_PCT	100
#define RFX_SAT_TO_MAX_DAILY_PCT	88

/* Effective ceiling filter */
#define RFX_CEIL_RISE_PCT_PER_2MS	1
#define RFX_CEIL_FALL_DWELL_NS		(15 * NSEC_PER_MSEC)
#define RFX_CEIL_FALL_BYPASS_PCT	8

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/* =====================================================================
 * Section 3: Data Structures & Global State
 * ===================================================================== */

static atomic_t rfx_gaming = ATOMIC_INIT(0);

static LIST_HEAD(rfx_policy_list);
static DEFINE_SPINLOCK(rfx_policy_list_lock);

static inline bool rfx_gaming_enabled(void)
{
	return atomic_read(&rfx_gaming) != 0;
}

struct rfx_tunables {
	struct gov_attr_set attr_set;
	unsigned int rate_limit_us;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
};

struct rfx_policy {
	struct cpufreq_policy *policy;
	struct rfx_tunables *tunables;
	struct list_head tunables_hook;
	struct list_head gov_node;

	raw_spinlock_t update_lock;

	u64 last_upfreq_time;
	u64 last_downfreq_time;
	u64 last_eval_time;
	s64 freq_update_delay_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;

	unsigned int next_freq;
	unsigned int cached_raw_freq;
	unsigned int pending_raw_freq;
	unsigned int max_seen;

	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool limits_changed;
	bool need_freq_update;

	bool is_prime;
	bool is_little;
	bool gaming_attr;

	/* Warmup ramp tracking */
	unsigned int warmup_ramp_pct;
	u64 warmup_ramp_last_ns;
	u64 gaming_warmup_end_ns;
	u64 gaming_warmup_start_ns;
	bool gaming_warmup_pending;
	bool warmup_pending_entry;
	bool gaming_warmup_entry;
	u64 quiet_since_ns;

	/* Policy-wide smoothed utilization */
	unsigned long filt_util;
	u64 last_ema_ns;

	/* Gaming state flags */
	bool floor_gated;
	u64 gate_low_since_ns;
	u64 gate_exit_since_ns;
	unsigned int sat_consecutive;
	bool thermal_cooling;
	bool risk_high;
	u64 warmup_low_demand_since_ns;
	u64 descend_hold_end_ns;
	unsigned int prev_demand_pct;
	u64 hispeed_hold_end_ns;
	s64 min_sample_ns;
	u64 sustained_high_since_ns;
	bool gaming_sustained_cap;

	/* Daily state flags */
	bool little_cap_lifted;
	u64 little_floor_end_ns;
	unsigned int little_prev_demand;
	bool big_cap_lifted;
	bool parked;
	unsigned int park_exit_count;

	/* Ceiling filter state */
	unsigned int ceil_rise_pct;
	u64 ceil_rise_ref_ns;
	u64 ceil_fall_ns;

	/* Diagnostic fields */
	unsigned int dbg_demand_pct;
	unsigned int dbg_fceil_pct;
};

struct rfx_cpu {
	struct update_util_data update_util;
	struct rfx_policy *rfx_policy;
	unsigned int cpu;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;

	unsigned long util;
	unsigned long bwmin;
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu);

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

static inline struct gov_attr_set *rfx_to_gov_attr_set(struct kobject *kobj)
{
	return container_of(kobj, struct gov_attr_set, kobj);
}

/* =====================================================================
 * Section 4: Cluster Topology & Effective Ceiling Filter
 * ===================================================================== */

static inline bool rfx_cap_is_little(unsigned long cap)
{
	return cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool rfx_cap_is_top(unsigned long cap)
{
	return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

static int rfx_ntiers(void)
{
	static int ntiers;
	unsigned long caps[4];
	int n = 0, cpu, i;

	if (ntiers)
		return ntiers;

	for_each_possible_cpu(cpu) {
		unsigned long c = arch_scale_cpu_capacity(cpu);

		for (i = 0; i < n; i++) {
			if (caps[i] == c)
				break;
		}
		if (i == n && n < (int)ARRAY_SIZE(caps))
			caps[n++] = c;
	}

	if (n < 2)
		return n;
	ntiers = n;
	return ntiers;
}

static inline bool rfx_cap_is_prime(unsigned long cap)
{
	return rfx_cap_is_top(cap) && rfx_ntiers() >= 3;
}

static inline unsigned int rfx_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

static inline u64 rfx_elapsed(u64 time, u64 stamp)
{
	s64 delta = (s64)(time - stamp);

	return delta > 0 ? (u64)delta : 0;
}

/*
 * Determine available headroom percentage under platform thermal pressure
 * and sysfs policy limits.
 */
static unsigned int rfx_thermal_headroom_pct(struct rfx_policy *p,
					     unsigned long max_cap,
					     unsigned int *baseline)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int press_pct = 100, clamp_pct = 100;
	unsigned int pmax = READ_ONCE(pol->max);
	unsigned long press;

	if (pmax > p->max_seen)
		p->max_seen = pmax;
	*baseline = p->max_seen ? p->max_seen : pol->cpuinfo.max_freq;

	press = arch_scale_thermal_pressure(cpumask_first(pol->related_cpus));
	if (max_cap) {
		if (press >= max_cap)
			press_pct = 0;
		else if (press)
			press_pct = (unsigned int)(((u64)(max_cap - press) *
						    100 + max_cap - 1) / max_cap);
	}

	if (pmax && pmax < *baseline)
		clamp_pct = (unsigned int)(((u64)pmax * 100 + *baseline - 1) /
					   *baseline);

	return min(press_pct, clamp_pct);
}

/*
 * Paced rise filter for thermal recovery and dwell-filtered shallow falls.
 */
static unsigned int rfx_ceil_rise_filter(struct rfx_policy *p,
					 unsigned int pct, u64 time)
{
	u64 budget;

	if (pct <= p->ceil_rise_pct) {
		if (pct == p->ceil_rise_pct) {
			p->ceil_fall_ns = 0;
			return pct;
		}
		if (p->ceil_rise_pct - pct >= RFX_CEIL_FALL_BYPASS_PCT) {
			p->ceil_rise_pct = pct;
			p->ceil_rise_ref_ns = time;
			p->ceil_fall_ns = 0;
			return pct;
		}
		if (!p->ceil_fall_ns)
			p->ceil_fall_ns = time;
		if (rfx_elapsed(time, p->ceil_fall_ns) < RFX_CEIL_FALL_DWELL_NS)
			return p->ceil_rise_pct;
		p->ceil_rise_pct = pct;
		p->ceil_rise_ref_ns = time;
		p->ceil_fall_ns = 0;
		return pct;
	}

	p->ceil_fall_ns = 0;

	budget = rfx_elapsed(time, p->ceil_rise_ref_ns) / (2 * NSEC_PER_MSEC);
	if (!budget)
		return p->ceil_rise_pct;
	budget *= RFX_CEIL_RISE_PCT_PER_2MS;

	if (budget >= pct - p->ceil_rise_pct)
		p->ceil_rise_pct = pct;
	else
		p->ceil_rise_pct += budget;
	p->ceil_rise_ref_ns = time;
	return p->ceil_rise_pct;
}

/* =====================================================================
 * Section 5: EMA Utilization Smoothing & Headroom
 * ===================================================================== */

/*
 * Directional Exponential Moving Average:
 * Immediate rise for responsive burst tracking, time-normalized decay.
 */
static unsigned long rfx_ema(unsigned long old, unsigned long val, u64 time,
			     u64 *last_ns, bool gaming)
{
	u64 delta_ns;
	unsigned long diff;
	unsigned int steps;

	if (!old || val >= old || !gaming) {
		*last_ns = time;
		return val;
	}

	if (unlikely(!*last_ns))
		*last_ns = time - RFX_EMA_DECAY_PERIOD_NS;
	delta_ns = rfx_elapsed(time, *last_ns);

	steps = (unsigned int)min_t(u64, delta_ns / RFX_EMA_DECAY_PERIOD_NS,
				    RFX_EMA_MAX_STEPS);
	if (!steps)
		return old;

	if (steps < RFX_EMA_MAX_STEPS)
		*last_ns += (u64)steps * RFX_EMA_DECAY_PERIOD_NS;
	else
		*last_ns = time;

	while (steps--) {
		diff = old - val;
		if (!diff)
			break;
		old -= max_t(unsigned long, diff / RFX_EMA_GAMING_DIVISOR, 1);
	}

	return old;
}

static unsigned long rfx_apply_headroom(unsigned long util, unsigned long max_cap,
					bool gaming, bool little)
{
	unsigned int upct;

	if (!max_cap || util >= max_cap)
		return max_cap;

	upct = (unsigned int)(util * 100 / max_cap);
	if (upct >= (gaming ? RFX_SAT_TO_MAX_GAMING_PCT :
			      RFX_SAT_TO_MAX_DAILY_PCT))
		return max_cap;

	if (gaming) {
		if (upct <= RFX_HEADROOM_GAMING_GATE)
			return util;
		return min(util + max_cap * RFX_HEADROOM_GAMING *
				  (upct - RFX_HEADROOM_GAMING_GATE) /
				  ((100 - RFX_HEADROOM_GAMING_GATE) * 100),
			   max_cap);
	}

	if (little) {
		if (upct >= 65)
			return min(util + util * RFX_HEADROOM_DAILY_HIGH / 100, max_cap);
		if (upct >= 40)
			return min(util + util * RFX_HEADROOM_DAILY_MID / 100, max_cap);
		return util;
	}

	if (upct >= 70)
		return min(util + util * RFX_HEADROOM_DAILY_HIGH / 100, max_cap);
	if (upct >= 45)
		return min(util + util * RFX_HEADROOM_DAILY_MID / 100, max_cap);

	return util;
}

/* =====================================================================
 * Section 6: Target Frequency Decision Logic (Dual Profile)
 * ===================================================================== */

static void rfx_warmup_rearm_quiet(struct rfx_policy *p, unsigned int demand_pct,
				   u64 time)
{
	if (demand_pct >= RFX_GAMING_WARMUP_TRIGGER_PCT) {
		p->quiet_since_ns = 0;
		return;
	}

	if (!p->quiet_since_ns)
		p->quiet_since_ns = time;
	else if (!p->gaming_warmup_pending &&
		 rfx_elapsed(time, p->quiet_since_ns) >= RFX_GAMING_REARM_QUIET_NS) {
		p->gaming_warmup_pending = true;
		p->warmup_pending_entry = false;
	}
}

static void rfx_warmup_arm(struct rfx_policy *p, unsigned int demand_pct,
			   u64 time)
{
	if (!p->gaming_warmup_pending || p->thermal_cooling ||
	    demand_pct < RFX_GAMING_WARMUP_TRIGGER_PCT)
		return;

	p->gaming_warmup_pending = false;
	p->quiet_since_ns = 0;
	p->gaming_warmup_entry = p->warmup_pending_entry;
	p->gaming_warmup_start_ns = time;
	p->gaming_warmup_end_ns = time + RFX_GAMING_WARMUP_NS;
}

static void rfx_risk_rearm(struct rfx_policy *p, unsigned int demand_pct,
			   unsigned int warmup_fl, u64 time)
{
	if (demand_pct < RFX_G_RISK_ARM_PCT) {
		if (demand_pct <= RFX_G_RISK_CLEAR_PCT ||
		    time >= p->gaming_warmup_end_ns)
			p->risk_high = false;
		return;
	}

	if (p->risk_high || p->next_freq >= warmup_fl)
		return;

	p->risk_high = true;
	if (time + RFX_G_RISK_BOOST_NS > p->gaming_warmup_end_ns)
		p->gaming_warmup_end_ns = time + RFX_G_RISK_BOOST_NS;
}

static bool rfx_descend_hold(struct rfx_policy *p, unsigned int demand_pct,
			     bool render_band, bool cooling, u64 time)
{
	int drop = (int)p->prev_demand_pct - (int)demand_pct;

	if (render_band && !cooling &&
	    p->prev_demand_pct >= RFX_G_DESCEND_FROM_PCT &&
	    drop >= RFX_G_DESCEND_DROP_PCT)
		p->descend_hold_end_ns = time + RFX_G_DESCEND_HOLD_NS;
	p->prev_demand_pct = demand_pct;

	return render_band && !cooling && time < p->descend_hold_end_ns;
}

static unsigned int rfx_update_warmup_ramp(struct rfx_policy *p, bool active, u64 time)
{
	u64 delta_ns, ramp_ns;
	unsigned int step;

	if (active) {
		p->warmup_ramp_pct = 100;
		p->warmup_ramp_last_ns = time;
		return 100;
	}

	if (p->warmup_ramp_pct == 0)
		return 0;

	ramp_ns = (u64)(p->gaming_warmup_entry ? RFX_WARMUP_RAMP_DOWN_MS :
						 RFX_WARMUP_REARM_RAMP_DOWN_MS) *
		  NSEC_PER_MSEC;

	if (!p->warmup_ramp_last_ns)
		p->warmup_ramp_last_ns = time;
	delta_ns = rfx_elapsed(time, p->warmup_ramp_last_ns);

	step = (unsigned int)min_t(u64, (delta_ns * 100) / ramp_ns, 100);
	if (step > 0) {
		u64 consumed_ns = (u64)step * ramp_ns / 100;

		p->warmup_ramp_last_ns += consumed_ns;
		p->warmup_ramp_pct -= min(p->warmup_ramp_pct, step);
	}

	return p->warmup_ramp_pct;
}

static unsigned int rfx_cool_depth(unsigned int fceil_pct)
{
	if (fceil_pct <= RFX_G_COOL_DEEP_PCT)
		return 100;
	if (fceil_pct >= RFX_G_COOL_EXIT_PCT)
		return 0;
	return (RFX_G_COOL_EXIT_PCT - fceil_pct) * 100 /
	       (RFX_G_COOL_EXIT_PCT - RFX_G_COOL_DEEP_PCT);
}

static unsigned int rfx_target_freq(struct rfx_policy *p, unsigned long util,
				    unsigned long max_cap, u64 time, bool gaming)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int fmin = pol->cpuinfo.min_freq;
	bool little = p->is_little;
	bool prime = p->is_prime;
	unsigned int freq;
	unsigned long raw_util = util;
	unsigned int fmax, fceil, fceil_pct;

	if (unlikely(!pol->cpuinfo.max_freq || !max_cap))
		return pol->cur;

	fceil_pct = rfx_thermal_headroom_pct(p, max_cap, &fmax);
	if (unlikely(!fmax))
		return pol->cur;

	fceil = rfx_pct(fmax, rfx_ceil_rise_filter(p, fceil_pct, time));
	fceil = clamp(fceil, fmin, fmax);
	p->dbg_fceil_pct = fceil_pct;

	util = rfx_apply_headroom(util, max_cap, gaming, little);
	freq = (unsigned int)((u64)pol->cpuinfo.max_freq * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		bool warmup_active;
		unsigned int warmup_ramp_pct;
		unsigned int fl, warmup_fl, demand_pct;
		unsigned int depth;
		u64 down_step, slew_ns;
		unsigned int down_pct;
		bool hold;

		demand_pct = (unsigned int)(raw_util * 100 / max_cap);
		p->dbg_demand_pct = demand_pct;

		/* Autonomous sustained heavy-load thermal pacer: prevents thermal runaway during war */
		if (demand_pct >= RFX_G_SUSTAINED_TRIGGER_PCT) {
			if (!p->sustained_high_since_ns)
				p->sustained_high_since_ns = time;
			else if (rfx_elapsed(time, p->sustained_high_since_ns) >=
				 (u64)RFX_G_SUSTAINED_WINDOW_MS * NSEC_PER_MSEC)
				p->gaming_sustained_cap = true;
		} else if (demand_pct <= RFX_G_SUSTAINED_CLEAR_PCT) {
			p->sustained_high_since_ns = 0;
			p->gaming_sustained_cap = false;
		}

		if (fceil_pct < RFX_G_COOL_ENTER_PCT && !p->thermal_cooling) {
			p->thermal_cooling = true;
		} else if (fceil_pct >= RFX_G_COOL_EXIT_PCT && p->thermal_cooling) {
			p->thermal_cooling = false;
			p->descend_hold_end_ns = time + RFX_G_DESCEND_HOLD_NS;
		}

		rfx_warmup_rearm_quiet(p, demand_pct, time);
		rfx_warmup_arm(p, demand_pct, time);
		if (!little && !prime && !p->gaming_sustained_cap)
			rfx_risk_rearm(p, demand_pct,
				       rfx_pct(fceil, RFX_G_WARMUP_FLOOR_PCT),
				       time);
		hold = rfx_descend_hold(p, demand_pct, !little && !prime,
					p->thermal_cooling || p->gaming_sustained_cap,
					time);

		warmup_active = !little && p->gaming_warmup_end_ns &&
				time < p->gaming_warmup_end_ns;

		if (warmup_active) {
			if (demand_pct >= RFX_GAMING_WARMUP_HARD_CANCEL_PCT) {
				if (++p->sat_consecutive >=
				    RFX_GAMING_WARMUP_HARD_CANCEL_EVALS)
					p->gaming_warmup_end_ns = time;
			} else {
				p->sat_consecutive = 0;
			}

			if (demand_pct >= RFX_GAMING_WARMUP_EXTEND_PCT &&
			    !p->thermal_cooling && !p->gaming_sustained_cap &&
			    p->sat_consecutive < RFX_GAMING_WARMUP_HARD_CANCEL_EVALS) {
				u64 cap = p->gaming_warmup_start_ns +
					  RFX_GAMING_WARMUP_MAX_NS;
				u64 ext = time + RFX_EMA_DECAY_PERIOD_NS * 4;

				if (ext > cap)
					ext = cap;
				if (ext > p->gaming_warmup_end_ns)
					p->gaming_warmup_end_ns = ext;
				p->warmup_low_demand_since_ns = 0;
			} else if (!p->gaming_warmup_entry &&
				   demand_pct < RFX_GAMING_WARMUP_RELEASE_PCT) {
				if (!p->warmup_low_demand_since_ns)
					p->warmup_low_demand_since_ns = time;
				else if (rfx_elapsed(time,
						p->warmup_low_demand_since_ns) >=
					 RFX_GAMING_WARMUP_RELEASE_NS)
					p->gaming_warmup_end_ns = time;
			} else {
				p->warmup_low_demand_since_ns = 0;
			}
			warmup_active = time < p->gaming_warmup_end_ns;
		}

		if (prime)
			fl = rfx_pct(fceil, RFX_G_PRIME_FLOOR_PCT);
		else if (!little)
			fl = rfx_pct(fceil, RFX_G_BIG_FLOOR_PCT);
		else
			fl = rfx_pct(fceil, RFX_G_LITTLE_FLOOR_PCT);

		warmup_fl = (!little && !prime) ?
				rfx_pct(fceil, RFX_G_WARMUP_FLOOR_PCT) : fl;

		depth = rfx_cool_depth(fceil_pct);

		if (p->thermal_cooling) {
			unsigned int steady = rfx_pct(fceil,
						      RFX_G_COOL_STEADY_FLOOR_PCT);

			if (fl > steady)
				fl -= (fl - steady) * depth / 100;
			if (warmup_fl > steady)
				warmup_fl -= (warmup_fl - steady) * depth / 100;
		}

		if (!little && !prime && depth) {
			unsigned int walk = fceil -
				(fceil - rfx_pct(fceil, RFX_G_COOL_CAP_PCT)) * depth / 100;

			if (freq > walk)
				freq = walk;
		}

		/* Cluster-specific sustained thermal capping: rock-solid, smooth, no spike */
		if (little) {
			unsigned int lcap = rfx_pct(fceil, RFX_G_LITTLE_CAP_PCT);

			if (freq > lcap)
				freq = lcap;
		} else if (prime) {
			unsigned int pcap = rfx_pct(fceil, RFX_G_PRIME_CAP_PCT);

			if (freq > pcap)
				freq = pcap;
		} else if (p->gaming_sustained_cap) {
			unsigned int bcap = rfx_pct(fceil, RFX_G_BIG_CAP_PCT);

			if (warmup_fl > bcap)
				warmup_fl = bcap;
			if (fl > bcap)
				fl = bcap;
			if (freq > bcap)
				freq = bcap;
		}

		slew_ns = rfx_elapsed(time, max(p->last_upfreq_time,
						p->last_downfreq_time));
		slew_ns = min_t(u64, slew_ns,
				(u64)RFX_GAMING_DOWN_US * NSEC_PER_USEC);

		down_pct = RFX_GAMING_DOWN_PCT_PER_2MS;
		if (RFX_G_DOWN_FAST_MS_DEFAULT &&
		    rfx_elapsed(time, p->last_upfreq_time) <
			(u64)RFX_G_DOWN_FAST_MS_DEFAULT * NSEC_PER_MSEC)
			down_pct = RFX_G_DOWN_FAST_PCT_DEFAULT;

		down_step = (u64)rfx_pct(fceil, down_pct) *
			    slew_ns / (2 * NSEC_PER_MSEC);
		if (down_step < fceil && p->next_freq > (unsigned int)down_step &&
		    freq < p->next_freq - (unsigned int)down_step)
			freq = p->next_freq - (unsigned int)down_step;

		if (hold && freq < p->next_freq)
			freq = p->next_freq;

		warmup_ramp_pct = rfx_update_warmup_ramp(p, warmup_active, time);

		if (demand_pct < RFX_G_FLOOR_GATE_PCT) {
			if (!p->gate_low_since_ns)
				p->gate_low_since_ns = time;
			else if (rfx_elapsed(time, p->gate_low_since_ns) >=
				 (u64)RFX_G_GATE_DWELL_MS * NSEC_PER_MSEC)
				p->floor_gated = true;
		} else {
			p->gate_low_since_ns = 0;
			if (demand_pct >= RFX_G_FLOOR_GATE_EXIT_PCT) {
				p->floor_gated = false;
				p->gate_exit_since_ns = 0;
			} else if (demand_pct >= RFX_G_FLOOR_GATE_PCT) {
				if (!p->gate_exit_since_ns)
					p->gate_exit_since_ns = time;
				else if (rfx_elapsed(time, p->gate_exit_since_ns) >=
					 (u64)RFX_G_GATE_EXIT_DWELL_MS * NSEC_PER_MSEC)
					p->floor_gated = false;
			}
		}

		if (p->floor_gated)
			fl = rfx_pct(fceil, RFX_G_IDLE_FLOOR_PCT);
		else if (warmup_ramp_pct > 0 && warmup_fl > fl)
			fl = fl + (warmup_fl - fl) * warmup_ramp_pct / 100;

		if (!little && !prime && p->gaming_sustained_cap) {
			unsigned int bcap = rfx_pct(fceil, RFX_G_BIG_CAP_PCT);

			if (fl > bcap)
				fl = bcap;
		}

		if (freq < fl)
			freq = fl;

#if RFX_G_HISPEED_PCT_DEFAULT
		/* F1: hispeed render floor on scene-change bursts */
		if (!little && !prime && !p->thermal_cooling && !p->gaming_sustained_cap) {
			if (demand_pct >= RFX_G_GO_HISPEED_PCT_DEFAULT)
				p->hispeed_hold_end_ns = time +
				  (u64)RFX_G_HISPEED_HOLD_US_DEFAULT * NSEC_PER_USEC;
			if (p->hispeed_hold_end_ns && time < p->hispeed_hold_end_ns) {
				unsigned int hs = rfx_pct(fceil, RFX_G_HISPEED_PCT_DEFAULT);

				if (freq < hs)
					freq = hs;
			}
		}
#endif

		/* Final clamp to enforce cluster caps after floor adjustments */
		if (little) {
			unsigned int lcap = rfx_pct(fceil, RFX_G_LITTLE_CAP_PCT);

			if (freq > lcap)
				freq = lcap;
		} else if (prime) {
			unsigned int pcap = rfx_pct(fceil, RFX_G_PRIME_CAP_PCT);

			if (freq > pcap)
				freq = pcap;
		} else if (p->gaming_sustained_cap) {
			unsigned int bcap = rfx_pct(fceil, RFX_G_BIG_CAP_PCT);

			if (freq > bcap)
				freq = bcap;
		}
	} else {
		unsigned int cap, demand_pct;

		demand_pct = (unsigned int)(raw_util * 100 / max_cap);
		p->dbg_demand_pct = demand_pct;

		if (little) {
			cap = rfx_pct(fceil, RFX_D_LITTLE_CAP_PCT);

			if (demand_pct >= RFX_D_LITTLE_LIFT_PCT)
				p->little_cap_lifted = true;
			else if (demand_pct <= RFX_D_LITTLE_DROP_PCT)
				p->little_cap_lifted = false;

			if (p->little_cap_lifted)
				cap = rfx_pct(fceil, RFX_D_LITTLE_SUSTAINED_CAP_PCT);

			if (p->little_prev_demand < RFX_D_LITTLE_FLOOR_ARM_PCT &&
			    demand_pct >= RFX_D_LITTLE_FLOOR_ARM_PCT)
				p->little_floor_end_ns = time + RFX_D_LITTLE_FLOOR_NS;
			else if (demand_pct < RFX_D_LITTLE_FLOOR_REARM_PCT)
				p->little_floor_end_ns = 0;
			p->little_prev_demand = demand_pct;

			if (p->little_floor_end_ns && time < p->little_floor_end_ns &&
			    freq < rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT))
				freq = rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT);
		} else {
			cap = rfx_pct(fceil, prime ? RFX_D_PRIME_CAP_PCT :
						     RFX_D_BIG_CAP_PCT);

			if (demand_pct >= RFX_D_BIG_LIFT_PCT)
				p->big_cap_lifted = true;
			else if (demand_pct <= RFX_D_BIG_DROP_PCT)
				p->big_cap_lifted = false;

			if (p->big_cap_lifted)
				cap = rfx_pct(fceil, prime ?
					RFX_D_PRIME_SUSTAINED_CAP_PCT :
					RFX_D_BIG_SUSTAINED_CAP_PCT);
		}

		if (freq > cap)
			freq = cap;
	}

	freq = clamp(freq, fmin, fceil);

	if (freq == p->cached_raw_freq && !p->need_freq_update)
		return p->next_freq;

	p->pending_raw_freq = freq;

	if (pol->freq_table && freq < p->next_freq) {
		bool round_down;

		if (gaming)
			round_down = RFX_G_ENERGY_AWARE_DEFAULT;
		else
			round_down = RFX_D_ENERGY_AWARE &&
				freq >= rfx_pct(fceil, RFX_D_ENERGY_AWARE_MIN_PCT);

		if (round_down) {
			int idx = cpufreq_frequency_table_target(pol, freq,
								 CPUFREQ_RELATION_H);

			if (idx >= 0)
				return pol->freq_table[idx].frequency;
		}
	}
	return cpufreq_driver_resolve_freq(pol, freq);
}

/* =====================================================================
 * Section 7: IO-Wait Boosting & Scheduler Integration
 * ===================================================================== */

static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time, bool set)
{
	u64 delta_ns = rfx_elapsed(time, rfx_c->last_update);

	if (delta_ns <= TICK_NSEC)
		return false;

	rfx_c->iowait_boost = set ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time, unsigned int flags)
{
	bool set = flags & SCHED_CPUFREQ_IOWAIT;
	unsigned int cap;

	if (rfx_c->iowait_boost && rfx_iowait_reset(rfx_c, time, set))
		return;

	if (!set || rfx_c->iowait_boost_pending)
		return;
	rfx_c->iowait_boost_pending = true;

	if (rfx_c->iowait_boost) {
		cap = SCHED_CAPACITY_SCALE / 8;
		rfx_c->iowait_boost = min_t(unsigned int,
					    rfx_c->iowait_boost << 1, cap);
		return;
	}
	rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time,
				      unsigned long max_cap)
{
	if (likely(!rfx_c->iowait_boost))
		return 0;
	if (rfx_iowait_reset(rfx_c, time, false))
		return 0;
	if (!rfx_c->iowait_boost_pending) {
		rfx_c->iowait_boost >>= 1;
		if (rfx_c->iowait_boost < IOWAIT_BOOST_MIN) {
			rfx_c->iowait_boost = 0;
			return 0;
		}
	}
	rfx_c->iowait_boost_pending = false;

	/* Do not inflate util on parked policies to preserve deep idle / deep sleep */
	if (rfx_c->rfx_policy->parked)
		return 0;

	return rfx_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
}

static void rfx_get_util(struct rfx_cpu *rfx_c, unsigned long boost)
{
	if (rfx_get_util_gki510) {
		rfx_get_util_gki510(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
	} else {
		/*
		 * Fallback for upstream / other trees without GKI helper exports:
		 * map via standard exported schedutil_cpu_util().
		 */
		unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
		unsigned long util = schedutil_cpu_util(rfx_c->cpu, 0, max_cap,
							FREQUENCY_UTIL, NULL);

		if (boost > util)
			util = boost;
		util = util + (util >> 2);	/* 25% DVFS headroom */
		rfx_c->util = min(util, max_cap);
		rfx_c->bwmin = 0;
	}
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510 &&
	    rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

/* =====================================================================
 * Section 8: Rate Limiting & Lockless Fast-Path Evaluation
 * ===================================================================== */

static inline void rfx_set_down_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming) {
		p->down_rate_delay_ns = (s64)RFX_GAMING_DOWN_US * NSEC_PER_USEC;
		p->min_sample_ns = (s64)RFX_G_MIN_SAMPLE_US_DEFAULT * NSEC_PER_USEC;
	} else {
		p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
		p->min_sample_ns = (s64)(p->is_little ?
			RFX_D_LITTLE_MIN_SAMPLE_US :
			RFX_D_BIG_MIN_SAMPLE_US) * NSEC_PER_USEC;
	}
}

static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

static inline void rfx_set_eval_delay(struct rfx_policy *p, bool gaming)
{
	s64 base;

	if (gaming) {
		p->freq_update_delay_ns = (s64)RFX_FAST_RATE_US * NSEC_PER_USEC;
		return;
	}

	base = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;

	if (RFX_D_IDLE_EVAL_US && p->next_freq == p->policy->cpuinfo.min_freq) {
		p->freq_update_delay_ns = (s64)RFX_D_IDLE_EVAL_US * NSEC_PER_USEC;
		return;
	}

	if (RFX_D_LIGHT_EVAL_US && p->next_freq <
	    rfx_pct(p->policy->cpuinfo.max_freq, RFX_D_LIGHT_EVAL_OPP_PCT)) {
		unsigned int opp_pct = (unsigned int)((u64)p->next_freq * 100 /
					p->policy->cpuinfo.max_freq);
		s64 stretch = (s64)RFX_D_LIGHT_EVAL_US * NSEC_PER_USEC - base;

		base += stretch * (100 - opp_pct) / (100 - RFX_D_LIGHT_EVAL_OPP_PCT);
		base = min_t(s64, base, (s64)RFX_D_LIGHT_EVAL_US * NSEC_PER_USEC);
	}
	p->freq_update_delay_ns = base;
}

/*
 * Lockless fast-path gate: skips acquiring update_lock on unneeded scheduler ticks.
 */
static inline bool rfx_fast_should_update(struct rfx_policy *p, u64 time)
{
	s64 delta;

	if (unlikely(READ_ONCE(p->limits_changed) || READ_ONCE(p->need_freq_update)))
		return true;

	delta = (s64)(time - READ_ONCE(p->last_eval_time));
	return delta >= READ_ONCE(p->freq_update_delay_ns);
}

static bool rfx_should_update_freq(struct rfx_policy *p, u64 time)
{
	s64 delta;

	if (unlikely(!p || !p->policy))
		return false;
	if (!cpufreq_this_cpu_can_update(p->policy))
		return false;

	if (unlikely(READ_ONCE(p->limits_changed))) {
		WRITE_ONCE(p->limits_changed, false);
		p->need_freq_update = true;
		smp_mb();
		return true;
	}
	if (p->need_freq_update)
		return true;

	delta = (s64)(time - p->last_eval_time);
	return delta >= p->freq_update_delay_ns;
}

static bool rfx_commit_freq(struct rfx_policy *p, u64 time, unsigned int next_freq)
{
	s64 delta;

	if (p->need_freq_update) {
		p->need_freq_update = false;
		if (p->next_freq == next_freq)
			return false;
	} else if (p->next_freq == next_freq) {
		return false;
	}

	if (next_freq < p->next_freq) {
		delta = (s64)(time - p->last_downfreq_time);
		if (p->down_rate_delay_ns > 0 && delta < p->down_rate_delay_ns)
			return false;
		if (p->min_sample_ns > 0 &&
		    (s64)(time - p->last_upfreq_time) < p->min_sample_ns)
			return false;
		p->last_downfreq_time = time;
	} else {
		delta = (s64)(time - p->last_upfreq_time);
		if (p->up_rate_delay_ns > 0 && delta < p->up_rate_delay_ns)
			return false;
		p->last_upfreq_time = time;
	}

	p->cached_raw_freq = p->pending_raw_freq;
	p->next_freq = next_freq;
	return true;
}

/* =====================================================================
 * Section 9: Frequency Update Hook & Worker Thread
 * ===================================================================== */

static unsigned int rfx_next_freq(struct rfx_cpu *rfx_c, u64 time, bool gaming)
{
	struct rfx_policy *p = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	unsigned long max_util = 0;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *jc = per_cpu_ptr(&rfx_cpu, j);
		unsigned long jb, je;

		jb = rfx_iowait_apply(jc, time, max_cap);
		rfx_get_util(jc, jb);
		je = max(jc->util, jb);

		if (je > max_util)
			max_util = je;
	}

	p->filt_util = rfx_ema(p->filt_util, max_util, time, &p->last_ema_ns,
			       gaming);

	rfx_set_down_delay(p, gaming);
	rfx_pol_up_delay(p, gaming);

	if (!gaming) {
		unsigned int fmin = p->policy->cpuinfo.min_freq;

		if (p->filt_util < (max_cap >> 5)) {
			p->parked = true;
			p->park_exit_count = 0;
		} else if (p->parked) {
			if (p->filt_util >= (max_cap * RFX_D_PARK_EXIT_PCT / 100)) {
				if (++p->park_exit_count >= RFX_D_PARK_EXIT_EVALS)
					p->parked = false;
			} else {
				p->park_exit_count = 0;
			}
		}
		if (p->parked) {
			p->pending_raw_freq = fmin;
			return fmin;
		}
	}

	return rfx_target_freq(p, p->filt_util, max_cap, time, gaming);
}

static void rfx_update(struct update_util_data *hook, u64 time,
		       unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();
	unsigned long irqflags;
	unsigned int next_f;
	bool do_deferred = false;

	/* Lockless pre-check: exit early without acquiring spinlock */
	if (!rfx_fast_should_update(p, time))
		return;

	raw_spin_lock_irqsave(&p->update_lock, irqflags);

	if (!gaming)
		rfx_iowait_boost(rfx_c, time, flags);

	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);
	rfx_set_eval_delay(p, gaming);

	if (rfx_should_update_freq(p, time)) {
		p->last_eval_time = time;
		next_f = rfx_next_freq(rfx_c, time, gaming);
		if (rfx_commit_freq(p, time, next_f)) {
			trace_vorpal(rfx_c->cpu, atomic_read(&rfx_gaming),
				     p->next_freq, p->dbg_demand_pct,
				     p->dbg_fceil_pct);

			if (p->policy->fast_switch_enabled) {
				cpufreq_driver_fast_switch(p->policy,
							   p->next_freq);
			} else if (!p->work_in_progress) {
				p->work_in_progress = true;
				do_deferred = true;
			}
		}
	}

	raw_spin_unlock_irqrestore(&p->update_lock, irqflags);

	if (do_deferred)
		irq_work_queue(&p->irq_work);
}

static void rfx_work(struct kthread_work *work)
{
	struct rfx_policy *p = container_of(work, struct rfx_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&p->update_lock, flags);
	freq = p->next_freq;
	p->work_in_progress = false;
	raw_spin_unlock_irqrestore(&p->update_lock, flags);

	mutex_lock(&p->work_lock);
	__cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *p = container_of(irq_work, struct rfx_policy, irq_work);

	kthread_queue_work(&p->worker, &p->work);
}

/* =====================================================================
 * Section 10: Sysfs Control Nodes
 * ===================================================================== */

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->rate_limit_us);
}
static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->rate_limit_us = val;
	return count;
}
static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->up_rate_limit_us);
}
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->up_rate_limit_us = val;
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->down_rate_limit_us);
}
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->down_rate_limit_us = val;
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

static void rfx_reset_policy_locked(struct rfx_policy *p)
{
	p->warmup_ramp_pct = 0;
	p->warmup_ramp_last_ns = 0;
	p->gaming_warmup_end_ns = 0;
	p->gaming_warmup_start_ns = 0;
	p->gaming_warmup_pending = false;
	p->warmup_pending_entry = false;
	p->gaming_warmup_entry = false;
	p->quiet_since_ns = 0;
	p->thermal_cooling = false;
	p->floor_gated = false;
	p->gate_low_since_ns = 0;
	p->gate_exit_since_ns = 0;
	p->sat_consecutive = 0;
	p->warmup_low_demand_since_ns = 0;
	p->risk_high = false;
	p->little_cap_lifted = false;
	p->big_cap_lifted = false;
	p->little_floor_end_ns = 0;
	p->little_prev_demand = 0;
	p->parked = false;
	p->park_exit_count = 0;
	p->descend_hold_end_ns = 0;
	p->prev_demand_pct = 0;
	p->ceil_rise_pct = 100;
	p->ceil_rise_ref_ns = 0;
	p->ceil_fall_ns = 0;
	p->hispeed_hold_end_ns = 0;
	p->sustained_high_since_ns = 0;
	p->gaming_sustained_cap = false;
	p->filt_util = 0;
	p->last_ema_ns = 0;
	p->need_freq_update = true;
}

static void rfx_reset_all_policies(void)
{
	struct rfx_policy *p;
	unsigned long flags, pflags;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_for_each_entry(p, &rfx_policy_list, gov_node) {
		raw_spin_lock_irqsave(&p->update_lock, pflags);
		rfx_reset_policy_locked(p);
		raw_spin_unlock_irqrestore(&p->update_lock, pflags);
	}
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
}

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_gaming));
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 1)
		return -EINVAL;

	atomic_set(&rfx_gaming, val);

	if (!val) {
		rfx_reset_all_policies();
	} else {
		struct rfx_policy *p;
		unsigned long flags, pflags;

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);
			p->gaming_warmup_pending = true;
			p->warmup_pending_entry = true;
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
	}
	return count;
}
static struct governor_attr gaming_mode = __ATTR_RW(gaming_mode);

static struct attribute *rfx_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx);

static void rfx_tunables_free(struct kobject *kobj)
{
	kfree(to_rfx_tunables(rfx_to_gov_attr_set(kobj)));
}

static struct kobj_type rfx_ktype = {
	.default_groups = rfx_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};

static void rfx_gaming_attr(struct rfx_policy *p, bool want)
{
	struct kobject *kobj = &p->tunables->attr_set.kobj;

	if (want == p->gaming_attr)
		return;
	if (want) {
		if (!sysfs_add_file_to_group(kobj, &gaming_mode.attr, NULL))
			p->gaming_attr = true;
	} else if (have_governor_per_policy()) {
		sysfs_remove_file_from_group(kobj, &gaming_mode.attr, NULL);
		p->gaming_attr = false;
	}
}

static bool rfx_hosts_gaming_attr(struct cpufreq_policy *policy)
{
	return cpumask_test_cpu(cpumask_last(cpu_possible_mask),
				policy->related_cpus);
}

/* =====================================================================
 * Section 11: Governor Registration & Self-Checks
 * ===================================================================== */

static struct cpufreq_governor vorpal_gov;

static struct rfx_policy *rfx_policy_alloc(struct cpufreq_policy *policy)
{
	struct rfx_policy *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;
	p->policy = policy;
	raw_spin_lock_init(&p->update_lock);
	INIT_LIST_HEAD(&p->gov_node);
	return p;
}

static void rfx_policy_free(struct rfx_policy *p)
{
	kfree(p);
}

static int rfx_kthread_create(struct rfx_policy *p)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = p->policy;
	int ret;

	init_irq_work(&p->irq_work, rfx_irq_work);
	mutex_init(&p->work_lock);

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&p->work, rfx_work);
	kthread_init_worker(&p->worker);
	thread = kthread_create(kthread_worker_fn, &p->worker, "rfx_gov/%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("vorpal: kthread create failed %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	if (rfx_setattr_sugov_gki510) {
		ret = rfx_setattr_sugov_gki510(thread);
	} else {
		/* Portable in-driver DEADLINE/SUGOV initialization */
		struct sched_attr attr = {
			.size		= sizeof(struct sched_attr),
			.sched_policy	= SCHED_DEADLINE,
			.sched_flags	= SCHED_FLAG_SUGOV,
			.sched_nice	= 0,
			.sched_priority	= 0,
			.sched_runtime	=  1000000,
			.sched_deadline = 10000000,
			.sched_period	= 10000000,
		};
		ret = sched_setattr_nocheck(thread, &attr);
	}

	if (ret) {
		kthread_stop(thread);
		pr_warn("vorpal: failed to set SCHED_DEADLINE\n");
		return ret;
	}

	p->thread = thread;
	if (!policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, policy->related_cpus);

	wake_up_process(thread);
	return 0;
}

static void rfx_kthread_stop(struct rfx_policy *p)
{
	if (p->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&p->worker);
	kthread_stop(p->thread);
	mutex_destroy(&p->work_lock);
}

static struct rfx_tunables *rfx_tunables_alloc(struct rfx_policy *p)
{
	struct rfx_tunables *t;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (t) {
		gov_attr_set_init(&t->attr_set, &p->tunables_hook);
		if (!have_governor_per_policy())
			rfx_global_tunables = t;
	}
	return t;
}

static void rfx_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		rfx_global_tunables = NULL;
}

static int rfx_init(struct cpufreq_policy *policy)
{
	struct rfx_policy *p;
	struct rfx_tunables *t;
	unsigned long max_cap;
	int ret = 0;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	p = rfx_policy_alloc(policy);
	if (!p) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = rfx_kthread_create(p);
	if (ret)
		goto free_p;

	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	p->is_prime = rfx_cap_is_prime(max_cap);
	p->is_little = rfx_cap_is_little(max_cap);

	mutex_lock(&rfx_global_tunables_lock);

	if (rfx_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = p;
		p->tunables = rfx_global_tunables;
		gov_attr_set_get(&rfx_global_tunables->attr_set, &p->tunables_hook);
		goto out;
	}

	t = rfx_tunables_alloc(p);
	if (!t) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	if (p->is_little) {
		t->rate_limit_us = RFX_LITTLE_RATE_US;
		t->up_rate_limit_us = RFX_LITTLE_UP_US;
		t->down_rate_limit_us = RFX_LITTLE_DOWN_US;
	} else {
		t->rate_limit_us = RFX_BIG_RATE_US;
		t->up_rate_limit_us = RFX_BIG_UP_US;
		t->down_rate_limit_us = RFX_BIG_DOWN_US;
	}

	policy->governor_data = p;
	p->tunables = t;

	ret = kobject_init_and_add(&t->attr_set.kobj, &rfx_ktype,
				   get_governor_parent_kobj(policy),
				   "%s", vorpal_gov.name);
	if (ret)
		goto fail;

out:
	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
	mutex_unlock(&rfx_global_tunables_lock);
	return 0;

fail:
	kobject_put(&t->attr_set.kobj);
	policy->governor_data = NULL;
	rfx_clear_global_tunables();
stop_kthread:
	rfx_kthread_stop(p);
	mutex_unlock(&rfx_global_tunables_lock);
free_p:
	rfx_policy_free(p);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("vorpal: init failed error %d\n", ret);
	return ret;
}

static void rfx_exit(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	struct rfx_tunables *t = p->tunables;
	unsigned int count;

	mutex_lock(&rfx_global_tunables_lock);
	count = gov_attr_set_put(&t->attr_set, &p->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		rfx_clear_global_tunables();
	mutex_unlock(&rfx_global_tunables_lock);

	rfx_kthread_stop(p);
	rfx_policy_free(p);
	cpufreq_disable_fast_switch(policy);
}

static int rfx_start(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	unsigned long flags, max_cap;
	unsigned int cpu;
	u64 now = sched_clock();

	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	p->is_prime = rfx_cap_is_prime(max_cap);
	p->is_little = rfx_cap_is_little(max_cap);
	rfx_gaming_attr(p, rfx_hosts_gaming_attr(policy));

	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;

	p->last_upfreq_time = now;
	p->last_downfreq_time = now;
	p->last_eval_time = now;
	p->next_freq = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	p->cached_raw_freq = 0;
	p->pending_raw_freq = 0;
	p->ceil_rise_pct = 100;
	p->ceil_rise_ref_ns = now;
	p->ceil_fall_ns = 0;
	p->max_seen = policy->max;
	p->work_in_progress = false;
	p->limits_changed = false;
	p->filt_util = 0;
	p->last_ema_ns = 0;

	rfx_reset_policy_locked(p);
	p->need_freq_update = false;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_add(&p->gov_node, &rfx_policy_list);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = per_cpu_ptr(&rfx_cpu, cpu);

		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu = cpu;
		rfx_c->rfx_policy = p;
	}

	for_each_cpu(cpu, policy->cpus) {
		cpufreq_add_update_util_hook(cpu,
			&per_cpu_ptr(&rfx_cpu, cpu)->update_util, rfx_update);
	}
	return 0;
}

static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	unsigned long flags;
	unsigned int cpu;

	rfx_gaming_attr(p, false);

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_del(&p->gov_node);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&p->irq_work);
		kthread_cancel_work_sync(&p->work);
	}
}

static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&p->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&p->work_lock);
	}
	smp_wmb();
	WRITE_ONCE(p->limits_changed, true);
}

static struct cpufreq_governor vorpal_gov = {
	.name = CPUFREQ_VORPAL_NAME,
	.owner = THIS_MODULE,
	.flags = CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init = rfx_init,
	.exit = rfx_exit,
	.start = rfx_start,
	.stop = rfx_stop,
	.limits = rfx_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_VORPAL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &vorpal_gov;
}
#endif

static void __init rfx_selfcheck(void)
{
	const u64 t = 1ULL << 40;
	struct rfx_policy p = { };
	unsigned long gate, over;
	u64 ns;

	ns = 0;
	WARN_ON(rfx_ema(100, 200, t, &ns, true) != 200 || ns != t);

	ns = t;
	WARN_ON(rfx_ema(200, 100, t, &ns, false) != 100);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) != 1000);

	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) !=
		1000 - 1000 / RFX_EMA_GAMING_DIVISOR || ns != t);

	ns = t + NSEC_PER_MSEC;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) != 1000);

	ns = t - ((u64)(RFX_EMA_MAX_STEPS + 1) * RFX_EMA_DECAY_PERIOD_NS);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) !=
		1000 - RFX_EMA_MAX_STEPS * 1000 / RFX_EMA_GAMING_DIVISOR ||
		ns != t);

	gate = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE);
	over = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE + 2);
	WARN_ON(rfx_apply_headroom(gate, SCHED_CAPACITY_SCALE, true, false) != gate);
	WARN_ON(rfx_apply_headroom(over, SCHED_CAPACITY_SCALE, true, false) <= over);
	WARN_ON(rfx_apply_headroom(SCHED_CAPACITY_SCALE - 1, SCHED_CAPACITY_SCALE,
				   true, false) > SCHED_CAPACITY_SCALE);

	p.gaming_warmup_entry = true;
	WARN_ON(rfx_update_warmup_ramp(&p, true, t) != 100);
	WARN_ON(rfx_update_warmup_ramp(&p, false, t) != 100);
	WARN_ON(rfx_update_warmup_ramp(&p, false,
			t + (u64)RFX_WARMUP_RAMP_DOWN_MS * NSEC_PER_MSEC));
	p.warmup_ramp_pct = 100;
	p.warmup_ramp_last_ns = t;
	WARN_ON(rfx_update_warmup_ramp(&p, false, t + 250000) != 100 ||
		p.warmup_ramp_last_ns != t);

	memset(&p, 0, sizeof(p));
	p.gaming_warmup_entry = true;
	WARN_ON(rfx_update_warmup_ramp(&p, true, t) != 100);
	WARN_ON(rfx_update_warmup_ramp(&p, false,
		t + (u64)RFX_WARMUP_RAMP_DOWN_MS * NSEC_PER_MSEC) != 0);
	p.gaming_warmup_entry = false;
	p.warmup_ramp_pct = 100;
	p.warmup_ramp_last_ns = t;
	WARN_ON(rfx_update_warmup_ramp(&p, false,
		t + (u64)RFX_WARMUP_REARM_RAMP_DOWN_MS * NSEC_PER_MSEC / 2) != 50);
	WARN_ON(rfx_update_warmup_ramp(&p, false,
		t + (u64)RFX_WARMUP_REARM_RAMP_DOWN_MS * NSEC_PER_MSEC) != 0);

	memset(&p, 0, sizeof(p));
	p.warmup_pending_entry = true;
	p.gaming_warmup_pending = true;
	rfx_warmup_arm(&p, RFX_GAMING_WARMUP_TRIGGER_PCT, t);
	WARN_ON(!p.gaming_warmup_entry);
	p.gaming_warmup_end_ns = 0;
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1, t);
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1,
			       t + RFX_GAMING_REARM_QUIET_NS);
	WARN_ON(!p.gaming_warmup_pending || p.warmup_pending_entry);
	rfx_warmup_arm(&p, RFX_GAMING_WARMUP_TRIGGER_PCT, t + RFX_GAMING_REARM_QUIET_NS);
	WARN_ON(p.gaming_warmup_entry);

	WARN_ON(RFX_GAMING_WARMUP_HARD_CANCEL_PCT <= RFX_GAMING_WARMUP_EXTEND_PCT);
	{
		unsigned int sc = RFX_GAMING_WARMUP_HARD_CANCEL_EVALS;

		if (RFX_GAMING_WARMUP_EXTEND_PCT >= RFX_GAMING_WARMUP_HARD_CANCEL_PCT)
			sc++;
		else
			sc = 0;
		WARN_ON(sc != 0);
	}

	memset(&p, 0, sizeof(p));
	p.next_freq = 1;
	rfx_risk_rearm(&p, RFX_G_RISK_ARM_PCT, 100, t);
	WARN_ON(!p.risk_high ||
		p.gaming_warmup_end_ns != t + RFX_G_RISK_BOOST_NS);
	ns = p.gaming_warmup_end_ns;
	rfx_risk_rearm(&p, 100, 100, t + 1);
	WARN_ON(p.gaming_warmup_end_ns != ns);
	rfx_risk_rearm(&p, RFX_G_RISK_CLEAR_PCT + 1, 100, t + 1);
	WARN_ON(!p.risk_high);
	rfx_risk_rearm(&p, RFX_G_RISK_CLEAR_PCT, 100, t + 1);
	WARN_ON(p.risk_high);

	memset(&p, 0, sizeof(p));
	p.next_freq = 100;
	rfx_risk_rearm(&p, 100, 100, t);
	WARN_ON(p.risk_high || p.gaming_warmup_end_ns);

	memset(&p, 0, sizeof(p));
	p.next_freq = 1;
	p.gaming_warmup_end_ns = t + 10 * RFX_G_RISK_BOOST_NS;
	rfx_risk_rearm(&p, 100, 100, t);
	WARN_ON(p.gaming_warmup_end_ns != t + 10 * RFX_G_RISK_BOOST_NS);

	memset(&p, 0, sizeof(p));
	p.prev_demand_pct = 90;
	WARN_ON(!rfx_descend_hold(&p, 50, true, false, t));
	WARN_ON(p.descend_hold_end_ns != t + RFX_G_DESCEND_HOLD_NS);
	WARN_ON(!rfx_descend_hold(&p, 50, true, false, t + 1));
	WARN_ON(rfx_descend_hold(&p, 50, true, false, t + RFX_G_DESCEND_HOLD_NS));

	memset(&p, 0, sizeof(p));
	p.prev_demand_pct = 80;
	WARN_ON(rfx_descend_hold(&p, 70, true, false, t));
	WARN_ON(p.descend_hold_end_ns);

	p.prev_demand_pct = 50;
	WARN_ON(rfx_descend_hold(&p, 10, true, false, t));
	WARN_ON(p.descend_hold_end_ns);

	p.prev_demand_pct = 90;
	WARN_ON(rfx_descend_hold(&p, 10, true, true, t));
	WARN_ON(rfx_descend_hold(&p, 10, false, false, t));
	WARN_ON(p.descend_hold_end_ns);

	WARN_ON(rfx_cool_depth(RFX_G_COOL_EXIT_PCT) != 0);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_EXIT_PCT + 15) != 0);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_DEEP_PCT) != 100);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_DEEP_PCT - 10) != 100);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_DEEP_PCT +
		(RFX_G_COOL_EXIT_PCT - RFX_G_COOL_DEEP_PCT) / 5) != 80);
	for (ns = RFX_G_COOL_DEEP_PCT; ns <= RFX_G_COOL_EXIT_PCT; ns++)
		WARN_ON(rfx_cool_depth(ns - 1) < rfx_cool_depth(ns));

	memset(&p, 0, sizeof(p));
	p.ceil_rise_pct = 100;
	p.ceil_rise_ref_ns = t;
	WARN_ON(rfx_ceil_rise_filter(&p, 80, t) != 80);
	WARN_ON(p.ceil_fall_ns);
	WARN_ON(rfx_ceil_rise_filter(&p, 90, t + 2 * NSEC_PER_MSEC) != 81);
	WARN_ON(rfx_ceil_rise_filter(&p, 90, t + 4 * NSEC_PER_MSEC) != 82);
	WARN_ON(rfx_ceil_rise_filter(&p, 100, t + NSEC_PER_SEC) != 100);
	WARN_ON(rfx_ceil_rise_filter(&p, 60, t + NSEC_PER_SEC + 1) != 60);

	memset(&p, 0, sizeof(p));
	p.ceil_rise_pct = 100;
	p.ceil_rise_ref_ns = t;
	WARN_ON(rfx_ceil_rise_filter(&p, 98, t) != 100);
	WARN_ON(rfx_ceil_rise_filter(&p, 97, t + 5 * NSEC_PER_MSEC) != 100);
	WARN_ON(rfx_ceil_rise_filter(&p, 100, t + 6 * NSEC_PER_MSEC) != 100);
	WARN_ON(rfx_ceil_rise_filter(&p, 98, t + 7 * NSEC_PER_MSEC) != 100);
	WARN_ON(rfx_ceil_rise_filter(&p, 98, t + 23 * NSEC_PER_MSEC) != 98);
	WARN_ON(rfx_ceil_rise_filter(&p, 90, t + 24 * NSEC_PER_MSEC) != 90);

	memset(&p, 0, sizeof(p));
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1, t);
	WARN_ON(p.gaming_warmup_pending != true);
	p.gaming_warmup_pending = false;
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1, t);
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1,
			       t + RFX_GAMING_REARM_QUIET_NS - 1);
	WARN_ON(p.gaming_warmup_pending);
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1,
			       t + RFX_GAMING_REARM_QUIET_NS);
	WARN_ON(p.gaming_warmup_pending != true);
}

static int __init vorpal_gov_init(void)
{
	/* Deadband invariants & invariant validations */
	BUILD_BUG_ON(RFX_G_FLOOR_GATE_PCT >= RFX_G_FLOOR_GATE_EXIT_PCT);
	BUILD_BUG_ON((u64)RFX_G_GATE_DWELL_MS * NSEC_PER_MSEC > RFX_GAMING_WARMUP_NS);
	BUILD_BUG_ON(RFX_G_COOL_ENTER_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_DROP_PCT >= RFX_D_LITTLE_LIFT_PCT);
	BUILD_BUG_ON(RFX_D_BIG_DROP_PCT >= RFX_D_BIG_LIFT_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_WARMUP_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_BIG_FLOOR_PCT > RFX_G_WARMUP_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_WARMUP_FLOOR_PCT > 100);
	BUILD_BUG_ON(RFX_G_IDLE_FLOOR_PCT > RFX_G_LITTLE_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_COOL_STEADY_FLOOR_PCT > RFX_G_BIG_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_SUSTAINED_CLEAR_PCT >= RFX_G_SUSTAINED_TRIGGER_PCT);
	BUILD_BUG_ON(RFX_G_LITTLE_CAP_PCT <= RFX_G_LITTLE_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_CAP_PCT <= RFX_G_PRIME_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_BIG_CAP_PCT <= RFX_G_BIG_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_BIG_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_LITTLE_CAP_PCT > RFX_D_LITTLE_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_FLOOR_PCT > RFX_D_LITTLE_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_FLOOR_REARM_PCT >= RFX_D_LITTLE_FLOOR_ARM_PCT);
	BUILD_BUG_ON(RFX_D_BIG_CAP_PCT > RFX_D_BIG_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_CAP_PCT > RFX_D_PRIME_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_BIG_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_PRIME_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE > 1);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE_MIN_PCT > 100);
	BUILD_BUG_ON(RFX_D_PARK_EXIT_PCT * 32 <= 100);
	BUILD_BUG_ON(RFX_D_PARK_EXIT_EVALS < 1);
	BUILD_BUG_ON(RFX_LITTLE_CAP_THRESHOLD >= RFX_PRIME_CAP_THRESHOLD);
	BUILD_BUG_ON(RFX_EMA_GAMING_DIVISOR < 1 || RFX_EMA_MAX_STEPS < 1);
	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_ENTER_PCT);
	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_G_RISK_CLEAR_PCT >= RFX_G_RISK_ARM_PCT);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_RELEASE_PCT >= RFX_GAMING_WARMUP_TRIGGER_PCT);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_NS > RFX_GAMING_WARMUP_MAX_NS);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_HARD_CANCEL_PCT <= RFX_GAMING_WARMUP_EXTEND_PCT);
	BUILD_BUG_ON(!RFX_CEIL_RISE_PCT_PER_2MS);
	BUILD_BUG_ON(!RFX_CEIL_FALL_DWELL_NS || !RFX_CEIL_FALL_BYPASS_PCT);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_EXTEND_PCT < RFX_GAMING_WARMUP_TRIGGER_PCT);
	BUILD_BUG_ON(RFX_WARMUP_REARM_RAMP_DOWN_MS <= RFX_WARMUP_RAMP_DOWN_MS);
	BUILD_BUG_ON(RFX_G_DESCEND_FROM_PCT <= RFX_G_FLOOR_GATE_EXIT_PCT);
	BUILD_BUG_ON(RFX_G_DESCEND_DROP_PCT >= RFX_G_DESCEND_FROM_PCT);
	BUILD_BUG_ON(RFX_G_DESCEND_HOLD_NS >= RFX_GAMING_WARMUP_NS);
	BUILD_BUG_ON(RFX_G_COOL_CAP_PCT <= RFX_G_COOL_ENTER_PCT);
	BUILD_BUG_ON(RFX_G_COOL_CAP_PCT <= RFX_G_COOL_STEADY_FLOOR_PCT);
	BUILD_BUG_ON(RFX_HEADROOM_GAMING_GATE >= 100);
	BUILD_BUG_ON(RFX_SAT_TO_MAX_GAMING_PCT > 100);
	BUILD_BUG_ON(RFX_SAT_TO_MAX_DAILY_PCT > 100);

	BUILD_BUG_ON(RFX_FAST_RATE_US < 1);
	BUILD_BUG_ON(RFX_G_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_GO_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_DOWN_FAST_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_ENERGY_AWARE_DEFAULT > 1);

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	rfx_selfcheck();

	return cpufreq_register_governor(&vorpal_gov);
}

static void __exit vorpal_gov_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.3");
