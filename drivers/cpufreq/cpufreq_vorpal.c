// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.3 — Heterogeneous multi-core governor for Linux.
 *
 * Designed for asymmetric DynamIQ and big.LITTLE topologies with dual operating
 * profiles configurable via sysfs:
 *   - Daily (gaming_mode=0): Energy-oriented pacing with autonomous cluster
 *     parking, dormant idle scaling, and PM notifier integration.
 *   - Gaming (gaming_mode=1): Multi-threaded capacity tracking, burst-to-sustained
 *     OPP regulation, inter-frame valley stabilization, and proactive thermal
 *     headroom management.
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
#include <linux/suspend.h>
#ifdef CONFIG_INPUT
#include <linux/input.h>
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/vorpal.h>

/* Sched-core helpers and metadata */
#define CPUFREQ_VORPAL_NAME	"vorpal"
#define CPUFREQ_VORPAL_VERSION	"2.3"
#define CPUFREQ_VORPAL_AUTHOR	"Templar Dev"

#ifndef SCHED_FLAG_SUGOV
#define SCHED_FLAG_SUGOV	0x10000000
#endif

/*
 * Optional sched helpers for in-tree optimization.
 * Weak linkage allows Vorpal to compile cleanly as a standalone governor
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

/* Tunable constants & architectural defaults */

/* Cluster capacity thresholds (normalized to 1024) */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* ---- Daily Profile Cadence (gaming_mode=0) ---- */
#define RFX_LITTLE_RATE_US		2000
#define RFX_LITTLE_UP_US		500
#define RFX_LITTLE_DOWN_US		3000

#define RFX_BIG_RATE_US			2000
#define RFX_BIG_UP_US			500
#define RFX_BIG_DOWN_US			4000

#define RFX_D_IDLE_EVAL_US		12000
#define RFX_D_DORMANT_EVAL_US		20000
#define RFX_D_DORMANT_ENTER_NS		(60 * NSEC_PER_MSEC)
#define RFX_D_LIGHT_EVAL_OPP_PCT	30
#define RFX_D_LIGHT_EVAL_US		6000
#define RFX_D_PARK_ENTER_EVALS		3

/* Daily cluster caps & sustained-load lift (% of effective ceiling) */
#define RFX_D_LITTLE_CAP_PCT		62
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	76
#define RFX_D_LITTLE_LIFT_PCT		55
#define RFX_D_LITTLE_DROP_PCT		40

#define RFX_D_BIG_CAP_PCT		74
#define RFX_D_BIG_SUSTAINED_CAP_PCT	88
#define RFX_D_BIG_LIFT_PCT		55
#define RFX_D_BIG_DROP_PCT		38

#define RFX_D_PRIME_CAP_PCT		58
#define RFX_D_PRIME_SUSTAINED_CAP_PCT	78

/* Timed wake-up interaction floor for Little cluster */
#define RFX_D_LITTLE_FLOOR_PCT		28
#define RFX_D_LITTLE_FLOOR_ARM_PCT	20
#define RFX_D_LITTLE_FLOOR_REARM_PCT	8
#define RFX_D_LITTLE_FLOOR_NS		(50 * NSEC_PER_MSEC)

/* Daily energy-aware round-down on descent */
#define RFX_D_ENERGY_AWARE		1
#define RFX_D_ENERGY_AWARE_MIN_PCT	20
#define RFX_D_LITTLE_MIN_SAMPLE_US	3000
#define RFX_D_BIG_MIN_SAMPLE_US		4000

/* ---- Gaming Profile Cadence & Controls (gaming_mode=1) ---- */
#define RFX_FAST_RATE_US		250
#define RFX_GAMING_DOWN_US		3000
#define RFX_GAMING_DOWN_PCT_PER_2MS	1
#define RFX_G_DOWN_FAST_PCT_DEFAULT	2
#define RFX_G_DOWN_FAST_MS_DEFAULT	80
#define RFX_G_MIN_SAMPLE_US_DEFAULT	4000
#define RFX_G_ENERGY_AWARE_DEFAULT	0

/* Gaming floors (% of effective ceiling) */
#define RFX_G_PRIME_FLOOR_PCT		38
#define RFX_G_BIG_FLOOR_PCT		44
#define RFX_G_LITTLE_FLOOR_PCT		30
#define RFX_G_IDLE_FLOOR_PCT		28
#define RFX_G_WARMUP_FLOOR_PCT		68

/* Gaming cluster caps (% of effective ceiling) */
#define RFX_G_LITTLE_CAP_PCT		76
#define RFX_G_PRIME_CAP_PCT		74
#define RFX_G_BIG_CAP_PCT		86

/* Gaming burst caps (headroom for skill/crowd spikes without voltage-wall heat) */
#define RFX_G_BIG_BURST_CAP_PCT		96
#define RFX_G_PRIME_BURST_CAP_PCT	88

/* Gaming sustained thermal pacer */
#define RFX_G_SUSTAINED_TRIGGER_PCT	50
#define RFX_G_SUSTAINED_CLEAR_PCT	35
#define RFX_G_SUSTAINED_WINDOW_MS	35
#define RFX_G_SUSTAINED_CLEAR_DWELL_MS	150

/* Gaming thermal energy accumulator (leaky bucket) */
#define RFX_G_THERMAL_ACCUM_PCT		65
#define RFX_G_THERMAL_ACC_MAX		2500000
/* Progressive trim stage thresholds: lightest (0) ... deepest (3) */
#define RFX_G_THERMAL_STAGE0_ACC	300000
#define RFX_G_THERMAL_STAGE1_ACC	700000
#define RFX_G_THERMAL_STAGE2_ACC	1250000
#define RFX_G_THERMAL_STAGE3_ACC	2000000

/* Gaming feature defaults (calibrated for 1.0x unskewed PELT util) */
#define RFX_G_HISPEED_PCT_DEFAULT	56
#define RFX_G_GO_HISPEED_PCT_DEFAULT	74
#define RFX_G_HISPEED_HOLD_US_DEFAULT	4000
#define RFX_G_TOUCH_PCT_DEFAULT		58
#define RFX_G_LITTLE_TOUCH_PCT_DEFAULT	48
#define RFX_G_TOUCH_MS_DEFAULT		80

/* Gaming demand gate & idle latch */
#define RFX_G_FLOOR_GATE_PCT		22
#define RFX_G_FLOOR_GATE_EXIT_PCT	42
#define RFX_G_GATE_DWELL_MS		250
#define RFX_G_GATE_EXIT_DWELL_MS	50

/* Gaming warmup & asset-load handling */
#define RFX_WARMUP_RAMP_DOWN_MS		60
#define RFX_WARMUP_REARM_RAMP_DOWN_MS	300
#define RFX_GAMING_WARMUP_NS		(400 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_MAX_NS	(2000 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_TRIGGER_PCT	50
#define RFX_GAMING_WARMUP_EXTEND_PCT	68
#define RFX_GAMING_WARMUP_RELEASE_PCT	32
#define RFX_GAMING_WARMUP_RELEASE_NS	(100 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_HARD_CANCEL_PCT	86
#define RFX_GAMING_WARMUP_HARD_CANCEL_EVALS	2
#define RFX_GAMING_REARM_QUIET_NS	(3000 * NSEC_PER_MSEC)

/* Gaming frame-risk & inter-frame VSync bridge */
#define RFX_G_RISK_ARM_PCT		80
#define RFX_G_RISK_CLEAR_PCT		42
#define RFX_G_RISK_BOOST_NS		(120 * NSEC_PER_MSEC)
#define RFX_G_DESCEND_DROP_PCT		15
#define RFX_G_DESCEND_FROM_PCT		70
#define RFX_G_DESCEND_HOLD_NS		(10 * NSEC_PER_MSEC)	/* EMA_MAX_STEPS * DECAY_PERIOD: one frame-gap damper */

/* Gaming anti-spike render load band */
#define RFX_G_STABILIZE_DEMAND_PCT	52
#define RFX_G_STABILIZE_DWELL_NS	(35 * NSEC_PER_MSEC)

/* Gaming thermal relief band */
#define RFX_G_COOL_ENTER_PCT		80
#define RFX_G_COOL_EXIT_PCT		85
#define RFX_G_COOL_STEADY_FLOOR_PCT	36
#define RFX_G_COOL_DEEP_PCT		60
#define RFX_G_COOL_CAP_PCT		88
#define RFX_G_COOL_PRIME_CAP_PCT	60

/*
 * Utilization EMA decay:
 * tau = DIVISOR * DECAY_PERIOD = 50ms (gaming). The gaming eval cadence is
 * 2ms, so 40 periods per step spans ~two 120fps frame gaps; inter-frame
 * PELT troughs (8.3ms) therefore walk down a few decay steps instead of
 * collapsing the render floor, while real scene changes still settle in
 * ~40ms. Daily keeps its fast 16ms tau for snappy parking.
 */
#define RFX_EMA_DECAY_PERIOD_NS		250000
#define RFX_EMA_GAMING_DIVISOR		200
#define RFX_EMA_DAILY_DIVISOR		8
#define RFX_EMA_MAX_STEPS		40

/* Headroom settings (calibrated for 1.0x unskewed PELT util) */
#define RFX_HEADROOM_DAILY_HIGH		10
#define RFX_HEADROOM_DAILY_GATE_LITTLE	15
#define RFX_HEADROOM_DAILY_GATE_BIG	18
#define RFX_HEADROOM_GAMING		14
#define RFX_HEADROOM_GAMING_GATE	40
#define RFX_SAT_TO_MAX_GAMING_PCT	90
#define RFX_SAT_TO_MAX_DAILY_PCT	88

/* Effective ceiling filter */
#define RFX_CEIL_RISE_PCT_PER_2MS	1
#define RFX_CEIL_FALL_DWELL_NS		(15 * NSEC_PER_MSEC)
#define RFX_CEIL_FALL_BYPASS_PCT	8

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/* Data structures & global state */

static atomic_t rfx_gaming = ATOMIC_INIT(0);
static atomic64_t rfx_big_high_demand_since = ATOMIC64_INIT(0);
static atomic64_t rfx_input_ts = ATOMIC64_INIT(0);
static atomic64_t rfx_tap_ts = ATOMIC64_INIT(0);

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
	u64 sustained_low_since_ns;
	bool gaming_sustained_cap;
	u64 stabilize_end_ns;
	unsigned int stabilize_freq;

	/* Daily state flags */
	bool little_cap_lifted;
	u64 little_floor_end_ns;
	unsigned int little_prev_demand;
	bool big_cap_lifted;
	unsigned int big_unpark_count;
	bool parked;
	bool dormant;
	unsigned int park_enter_count;
	u64 park_since_ns;

	/* Ceiling filter state */
	unsigned int ceil_rise_pct;
	u64 ceil_rise_ref_ns;
	u64 ceil_fall_ns;

	/* Mathematical thermal leaky-bucket energy accumulator */
	u64 thermal_energy_acc;
	u64 last_acc_time;

	/* Rapid tap transient filter */
	u64 burst_high_since_ns;

	/* Deep quiescent state (autonomous screen-off / idle) */
	bool quiescent;
	u64 quiescent_since_ns;

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

/* Topology and thermal headroom tracking */

static unsigned long rfx_min_cap __read_mostly;
static unsigned long rfx_max_cap __read_mostly;

static int rfx_ntiers(void)
{
	static int ntiers;
	unsigned long caps[4];
	unsigned long min_c = ULONG_MAX, max_c = 0;
	int n = 0, cpu, i;

	if (ntiers)
		return ntiers;

	for_each_possible_cpu(cpu) {
		unsigned long c = arch_scale_cpu_capacity(cpu);

		if (c < min_c)
			min_c = c;
		if (c > max_c)
			max_c = c;

		for (i = 0; i < n; i++) {
			if (caps[i] == c)
				break;
		}
		if (i == n && n < (int)ARRAY_SIZE(caps))
			caps[n++] = c;
	}

	rfx_min_cap = min_c;
	rfx_max_cap = max_c;

	if (n < 2)
		return n;
	ntiers = n;
	return ntiers;
}

static inline bool rfx_cap_is_little(unsigned long cap)
{
	if (rfx_ntiers() < 2)
		return false;
	if (rfx_min_cap >= RFX_LITTLE_CAP_THRESHOLD)
		return false;
	return cap == rfx_min_cap;
}

static inline bool rfx_cap_is_prime(unsigned long cap)
{
	if (rfx_ntiers() < 3)
		return false;
	return cap == rfx_max_cap;
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

/*
 * Thermal pacing reads arch_scale_thermal_pressure() from the thermal framework.
 * No in-governor polling is used.
 */

/* Utilization filtering and headroom */

/*
 * Directional Exponential Moving Average:
 * Immediate rise for responsive burst tracking, time-normalized decay.
 */
static unsigned long rfx_ema(unsigned long old, unsigned long val, u64 time,
			     u64 *last_ns, bool gaming)
{
	u64 delta_ns, idle_reset_ns;
	unsigned long diff;
	unsigned int steps, max_steps, div;

	if (!old || val >= old) {
		*last_ns = time;
		return val;
	}

	if (unlikely(!*last_ns))
		*last_ns = time - RFX_EMA_DECAY_PERIOD_NS;
	delta_ns = rfx_elapsed(time, *last_ns);

	/* Reset filter on extended idle dwell */
	idle_reset_ns = (gaming ? 40 : 16) * NSEC_PER_MSEC;
	if (delta_ns >= idle_reset_ns) {
		*last_ns = time;
		return val;
	}

	max_steps = gaming ? RFX_EMA_MAX_STEPS : 64;
	steps = (unsigned int)min_t(u64, delta_ns / RFX_EMA_DECAY_PERIOD_NS,
				    max_steps);
	if (!steps)
		return old;

	if (steps < max_steps)
		*last_ns += (u64)steps * RFX_EMA_DECAY_PERIOD_NS;
	else
		*last_ns = time;

	div = gaming ? RFX_EMA_GAMING_DIVISOR : RFX_EMA_DAILY_DIVISOR;

	while (steps--) {
		diff = old - val;
		if (!diff)
			break;
		old -= max_t(unsigned long, diff / div, 1);
	}

	return max(old, val);
}

static unsigned long rfx_apply_headroom(unsigned long util, unsigned long max_cap,
					bool gaming, bool little)
{
	unsigned int upct, gate;

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

	/* Progressive continuous headroom for daily profile */
	gate = little ? RFX_HEADROOM_DAILY_GATE_LITTLE : RFX_HEADROOM_DAILY_GATE_BIG;
	if (upct <= gate)
		return util;

	return min(util + (unsigned long)(((u64)max_cap * RFX_HEADROOM_DAILY_HIGH *
					  (upct - gate) + (((100 - gate) * 100) / 2)) /
					  ((100 - gate) * 100)),
		   max_cap);
}

static inline bool rfx_ts_active(atomic64_t *stamp, u64 time, unsigned int window_ms)
{
	u64 ts = (u64)atomic64_read(stamp);

	return ts && rfx_elapsed(time, ts) < (u64)window_ms * NSEC_PER_MSEC;
}

static inline bool rfx_is_touch_active(u64 time, unsigned int window_ms)
{
	return rfx_ts_active(&rfx_input_ts, time, window_ms);
}

static inline bool rfx_is_tap_active(u64 time, unsigned int window_ms)
{
	return rfx_ts_active(&rfx_tap_ts, time, window_ms);
}

/* Frequency target decision logic */

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
	p->sat_consecutive = 0;
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
	p->sat_consecutive = 0;
	if (time >= p->gaming_warmup_end_ns)
		p->gaming_warmup_entry = false;
	if (time + RFX_G_RISK_BOOST_NS > p->gaming_warmup_end_ns)
		p->gaming_warmup_end_ns = time + RFX_G_RISK_BOOST_NS;
}

/*
 * Inter-Frame VSync Bridge:
 * Holds frequency across momentary inter-frame valleys where CPU waits on GPU.
 */
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

/*
 * Anti-Spike Frequency Stabilizer:
 * In heavy sustained workloads, prevent downward frequency oscillation
 * across short inter-frame dips.
 */
static unsigned int rfx_stabilize_opp(struct rfx_policy *p, unsigned int freq,
				      unsigned int demand_pct, u64 time)
{
	if (demand_pct >= RFX_G_STABILIZE_DEMAND_PCT) {
		if (freq >= p->stabilize_freq ||
		    (p->stabilize_end_ns && time >= p->stabilize_end_ns)) {
			p->stabilize_freq = freq;
			p->stabilize_end_ns = time + RFX_G_STABILIZE_DWELL_NS;
		}
	} else if (time >= p->stabilize_end_ns) {
		p->stabilize_end_ns = 0;
		p->stabilize_freq = 0;
	}

	/*
	 * Hold the stabilized OPP floor across brief inter-frame dips until
	 * the dwell window expires cleanly.
	 */
	return max(freq, p->stabilize_freq);
}

/*
 * Calculate effective cluster cap based on burst-to-sustained transition,
 * thermal energy accumulation, and proactive cooling depth.
 */
static unsigned int rfx_effective_cap(struct rfx_policy *p, unsigned int fceil,
				      unsigned int demand_pct, unsigned int depth,
				      u64 time)
{
	if (p->is_little) {
		unsigned int pct = RFX_G_LITTLE_CAP_PCT;

		if (demand_pct >= 82 && !p->thermal_cooling) {
			pct = 88U;
			if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE3_ACC)
				pct -= 4U;
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE2_ACC)
				pct -= 2U;
		} else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE3_ACC) {
			pct = max(pct - 8, 66U);
		} else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE2_ACC) {
			pct = max(pct - 5, 69U);
		} else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE1_ACC) {
			pct = max(pct - 3, 72U);
		} else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE0_ACC) {
			pct = max(pct - 1, 74U);
		}

		return rfx_pct(fceil, pct);
	} else if (p->is_prime) {
		s64 big_high = atomic64_read(&rfx_big_high_demand_since);
		bool tap_fast = rfx_is_tap_active(time, 60);
		bool big_sat = (big_high && rfx_elapsed(time, (u64)big_high) >= (8 * NSEC_PER_MSEC));
		bool prime_self_sustained = (p->sustained_high_since_ns &&
			rfx_elapsed(time, p->sustained_high_since_ns) >= (5 * NSEC_PER_MSEC) &&
			demand_pct >= 45);
		bool prime_burst = (demand_pct >= (tap_fast ? 58 : 70) || big_sat) &&
				   !p->thermal_cooling;
		unsigned int base_pct;
		unsigned int pct;
		unsigned int pcap;

		/*
		 * Prime Core Rapid-Tap & Thermal Protection:
		 * Hold brief idle/non-tap transients at 52% resting cap and
		 * normal sustained work at the 74% IPC sweet spot; unlock
		 * immediately on discrete button tap/press (>=35% demand) or
		 * heavy load / Big saturation without inflating steady drag.
		 */
		if (!big_sat && !prime_self_sustained &&
		    !(tap_fast && demand_pct >= 35) &&
		    demand_pct < RFX_GAMING_WARMUP_TRIGGER_PCT) {
			base_pct = 52U;
		} else if (prime_burst) {
			base_pct = RFX_G_PRIME_BURST_CAP_PCT;
		} else {
			base_pct = RFX_G_PRIME_CAP_PCT;
		}
		pct = base_pct;

		if (prime_burst) {
			if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE3_ACC)
				pct -= 4U;
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE2_ACC)
				pct -= 2U;
		} else if (pct > 52U) {
			if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE3_ACC)
				pct = max(pct - 7, 66U);
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE2_ACC)
				pct = max(pct - 5, 68U);
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE1_ACC)
				pct = max(pct - 3, 70U);
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE0_ACC)
				pct = max(pct - 1, 72U);
		}

		pcap = rfx_pct(fceil, pct);
		if (depth) {
			unsigned int p_walk = fceil -
				(fceil - rfx_pct(fceil, RFX_G_COOL_PRIME_CAP_PCT)) * depth / 100;

			pcap = min(pcap, p_walk);
		}
		return pcap;
	} else {
		/*
		 * Big cluster (primary render tier):
		 * Hysteresis burst gate: arm burst (96% fceil) after >=74%
		 * demand persists for >=4ms (or immediately >=62% during active
		 * touch), and hold cleanly until demand drops below 66%.
		 */
		unsigned int base_pct, pct, bcap;
		bool tap_active = rfx_is_tap_active(time, 60);
		bool burst_allowed = false;

		if (demand_pct >= 74) {
			if (!p->burst_high_since_ns)
				p->burst_high_since_ns = time;
			if (tap_active ||
			    rfx_elapsed(time, p->burst_high_since_ns) >= (4 * NSEC_PER_MSEC))
				burst_allowed = true;
		} else if (tap_active && demand_pct >= 62) {
			burst_allowed = true;
		} else if (demand_pct < 66) {
			p->burst_high_since_ns = 0;
		} else if (p->burst_high_since_ns &&
			   rfx_elapsed(time, p->burst_high_since_ns) >= (4 * NSEC_PER_MSEC)) {
			burst_allowed = true;
		}

		if (burst_allowed && !p->thermal_cooling) {
			pct = RFX_G_BIG_BURST_CAP_PCT;
			if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE3_ACC)
				pct -= 4U;
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE2_ACC)
				pct -= 2U;
		} else {
			base_pct = RFX_G_BIG_CAP_PCT;
			pct = base_pct;

			if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE3_ACC)
				pct = max(pct - 8, 78U);
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE2_ACC)
				pct = max(pct - 6, 80U);
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE1_ACC)
				pct = max(pct - 4, 82U);
			else if (p->thermal_energy_acc >= RFX_G_THERMAL_STAGE0_ACC)
				pct = max(pct - 2, 84U);
		}

		bcap = rfx_pct(fceil, pct);
		if (depth) {
			unsigned int b_walk = fceil -
				(fceil - rfx_pct(fceil, RFX_G_COOL_CAP_PCT)) * depth / 100;

			bcap = min(bcap, b_walk);
		}
		return bcap;
	}
}

static unsigned int rfx_target_freq(struct rfx_policy *p, unsigned long util,
				    unsigned long max_cap, u64 time, bool gaming)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int fmin = max(pol->cpuinfo.min_freq, READ_ONCE(pol->min));
	bool little = p->is_little;
	bool prime = p->is_prime;
	unsigned int freq;
	unsigned long raw_util = util;
	unsigned int fmax, fceil, fceil_pct;
	unsigned int demand_pct;

	if (unlikely(!pol->cpuinfo.max_freq || !max_cap))
		return pol->cur;

	fceil_pct = rfx_thermal_headroom_pct(p, max_cap, &fmax);
	if (unlikely(!fmax))
		return pol->cur;
	if (fmin > fmax)
		fmin = fmax;

	demand_pct = (unsigned int)(raw_util * 100 / max_cap);
	p->dbg_demand_pct = demand_pct;

	fceil = rfx_pct(fmax, rfx_ceil_rise_filter(p, fceil_pct, time));
	fceil = clamp(fceil, fmin, fmax);
	p->dbg_fceil_pct = fceil_pct;

	util = rfx_apply_headroom(util, max_cap, gaming, little);
	freq = (unsigned int)((u64)pol->cpuinfo.max_freq * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		bool warmup_active;
		unsigned int warmup_ramp_pct;
		unsigned int fl, warmup_fl, eff_cap;
		unsigned int depth;
		u64 down_step;
		unsigned int down_pct;
		bool hold;
		bool touch_active = rfx_is_touch_active(time, RFX_G_TOUCH_MS_DEFAULT);

		/*
		 * Leaky-bucket thermal energy accumulator:
		 * Tracks sustained load to apply progressive cap trims during
		 * prolonged high demand.  Accumulation starts well below
		 * saturation and the leak is fast, so brief bursts decay to
		 * zero between actions; rapid repeated taps build load
		 * gradually instead of slamming the caps hard.
		 */
		{
			u64 delta_ms = p->last_acc_time ?
				(rfx_elapsed(time, p->last_acc_time) / NSEC_PER_MSEC) : 0;

			if (delta_ms > 0 && delta_ms < 5000) {
				p->last_acc_time += (u64)delta_ms * NSEC_PER_MSEC;
				if (demand_pct >= RFX_G_THERMAL_ACCUM_PCT)
					p->thermal_energy_acc +=
						(u64)(demand_pct -
							RFX_G_THERMAL_ACCUM_PCT + 1) * delta_ms;
				else {
					/* Leaks faster the cooler the core is */
					u64 leak = (u64)(RFX_G_THERMAL_ACCUM_PCT - demand_pct) * delta_ms;

					if (p->thermal_energy_acc > leak)
						p->thermal_energy_acc -= leak;
					else
						p->thermal_energy_acc = 0;
				}
				if (p->thermal_energy_acc > RFX_G_THERMAL_ACC_MAX)
					p->thermal_energy_acc = RFX_G_THERMAL_ACC_MAX;
			} else if (!p->last_acc_time || delta_ms >= 5000) {
				p->last_acc_time = time;
				p->thermal_energy_acc = 0;
			}
		}

		/*
		 * Sustained load pacer with pulse-train hysteresis:
		 * Smoothly transitions from burst to sustained caps across rapid tapping intervals.
		 */
		if (demand_pct >= RFX_G_SUSTAINED_TRIGGER_PCT) {
			p->sustained_low_since_ns = 0;
			if (!p->sustained_high_since_ns)
				p->sustained_high_since_ns = time;
			else if (rfx_elapsed(time, p->sustained_high_since_ns) >=
				 (u64)RFX_G_SUSTAINED_WINDOW_MS * NSEC_PER_MSEC)
				p->gaming_sustained_cap = true;
		} else if (demand_pct <= RFX_G_SUSTAINED_CLEAR_PCT) {
			if (!p->sustained_low_since_ns)
				p->sustained_low_since_ns = time;
			else if (rfx_elapsed(time, p->sustained_low_since_ns) >=
				 (u64)RFX_G_SUSTAINED_CLEAR_DWELL_MS * NSEC_PER_MSEC) {
				p->sustained_high_since_ns = 0;
				p->gaming_sustained_cap = false;
				p->sustained_low_since_ns = 0;
			}
		} else {
			p->sustained_low_since_ns = 0;
		}

		if (fceil_pct < RFX_G_COOL_ENTER_PCT && !p->thermal_cooling) {
			p->thermal_cooling = true;
		} else if (fceil_pct >= RFX_G_COOL_EXIT_PCT && p->thermal_cooling) {
			p->thermal_cooling = false;
			p->descend_hold_end_ns = time + RFX_G_DESCEND_HOLD_NS;
		}

		/*
		 * Thermal cooling engages the sustained sweet-spot caps immediately:
		 * entering the cooling band mid-burst would otherwise hold burst
		 * caps up to 30% above the sustained sweet spot and keep the cores
		 * hot while the ceiling is already shrinking.
		 */
		if (p->thermal_cooling)
			p->gaming_sustained_cap = true;

		rfx_warmup_rearm_quiet(p, demand_pct, time);
		rfx_warmup_arm(p, demand_pct, time);
		if (!little && !prime && !p->thermal_cooling)
			rfx_risk_rearm(p, demand_pct,
				       rfx_pct(fceil, RFX_G_WARMUP_FLOOR_PCT),
				       time);

		hold = rfx_descend_hold(p, demand_pct, !little,
					p->thermal_cooling,
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
			    !p->thermal_cooling &&
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

		/* Calculate unified effective cluster cap across thermal and burst policies */
		eff_cap = rfx_effective_cap(p, fceil, demand_pct, depth, time);

		if (warmup_fl > eff_cap)
			warmup_fl = eff_cap;
		if (fl > eff_cap)
			fl = eff_cap;
		if (freq > eff_cap)
			freq = eff_cap;
		if (p->stabilize_freq > eff_cap)
			p->stabilize_freq = eff_cap;

		{
			u64 down_slew_ns;

			/*
			 * Upward frequency climbs in gaming commit unthrottled so
			 * sudden skill/particle bursts hit target OPP in one eval;
			 * bounded down-slew owns smooth descent across frame gaps.
			 */
			down_slew_ns = rfx_elapsed(time, p->last_downfreq_time);
			down_slew_ns = min_t(u64, down_slew_ns, 20 * NSEC_PER_MSEC);

			down_pct = RFX_GAMING_DOWN_PCT_PER_2MS;
			if (p->next_freq > rfx_pct(fceil, RFX_G_BIG_CAP_PCT) ||
			    (RFX_G_DOWN_FAST_MS_DEFAULT &&
			     rfx_elapsed(time, p->last_upfreq_time) <
				(u64)RFX_G_DOWN_FAST_MS_DEFAULT * NSEC_PER_MSEC))
				down_pct = RFX_G_DOWN_FAST_PCT_DEFAULT;

			down_step = (u64)rfx_pct(fceil, down_pct) *
				    down_slew_ns / (2 * NSEC_PER_MSEC);
			if (down_step < fceil && p->next_freq > (unsigned int)down_step &&
			    freq < p->next_freq - (unsigned int)down_step)
				freq = p->next_freq - (unsigned int)down_step;
		}

		if (hold && freq < p->next_freq) {
			unsigned int hold_floor = max(fl, rfx_pct(fceil, prime ? 26U : 42U));

			freq = min(p->next_freq, max(freq, hold_floor));
		}

		warmup_ramp_pct = rfx_update_warmup_ramp(p, warmup_active, time);

		if (touch_active) {
			p->gate_low_since_ns = 0;
			p->floor_gated = false;
			p->gate_exit_since_ns = 0;
		} else if (demand_pct < RFX_G_FLOOR_GATE_PCT) {
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
			} else {
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

		if (fl > eff_cap)
			fl = eff_cap;

		if (freq < fl)
			freq = fl;

		/* F1 hispeed + F2 touch boost (active in gaming_mode=1 only) */
		if (!p->thermal_cooling) {
			unsigned int boost_fl = 0;

			if (little && touch_active) {
				boost_fl = rfx_pct(fceil, RFX_G_LITTLE_TOUCH_PCT_DEFAULT);
			} else if (!little && !prime) {
				if (RFX_G_HISPEED_PCT_DEFAULT) {
					if (demand_pct >= RFX_G_GO_HISPEED_PCT_DEFAULT)
						p->hispeed_hold_end_ns = time +
						  (u64)RFX_G_HISPEED_HOLD_US_DEFAULT * NSEC_PER_USEC;
					if (p->hispeed_hold_end_ns &&
					    time < p->hispeed_hold_end_ns)
						boost_fl = rfx_pct(fceil,
								   RFX_G_HISPEED_PCT_DEFAULT);
				}
				if (touch_active)
					boost_fl = max(boost_fl,
						rfx_pct(fceil, RFX_G_TOUCH_PCT_DEFAULT));
			}

			if (boost_fl > eff_cap)
				boost_fl = eff_cap;
			if (freq < boost_fl)
				freq = boost_fl;
		}

		/* Anti-spike stabilizer filter for sustained heavy workloads */
		freq = rfx_stabilize_opp(p, freq, demand_pct, time);

		/* Final clamp to enforce cluster caps with smooth slew-paced transition */
		if (freq > eff_cap)
			freq = (p->next_freq > eff_cap && down_step &&
				p->next_freq > (unsigned int)down_step) ?
				max(p->next_freq - (unsigned int)down_step, eff_cap) : eff_cap;

		p->stabilize_freq = min_t(unsigned int, p->stabilize_freq, eff_cap);
	} else {
		/* Daily profile */
		unsigned int cap;

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

			if (p->little_floor_end_ns &&
			    time < p->little_floor_end_ns &&
			    freq < rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT))
				freq = rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT);

			p->little_prev_demand = demand_pct;
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

		/* Burst headroom: for high-demand bursts (>=84%), allow full ceiling to race-to-sleep */
		if (demand_pct >= 84)
			cap = fceil;

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
			int idx_h = cpufreq_frequency_table_target(pol, freq,
								   CPUFREQ_RELATION_H);
			int idx_l = cpufreq_frequency_table_target(pol, freq,
								   CPUFREQ_RELATION_L);

			if (idx_h >= 0 && idx_l >= 0) {
				unsigned int cand_h = pol->freq_table[idx_h].frequency;
				unsigned int cand_l = pol->freq_table[idx_l].frequency;
				unsigned int max_under_pct = gaming ? 8 : 18;

				if (cand_h && cand_h < cand_l && freq >= cand_h &&
				    (freq - cand_h) * 100 / freq <= max_under_pct &&
				    ((cand_l - freq) * 2 >= (cand_l - cand_h) ||
				     (!gaming && (cand_l - freq) * 3 >= (cand_l - cand_h))))
					return cand_h;
				return cand_l;
			}
		}
	}
	return cpufreq_driver_resolve_freq(pol, freq);
}

/* IO-wait boosting */

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
	unsigned int max_boost = SCHED_CAPACITY_SCALE / 4;

	if (rfx_c->iowait_boost && rfx_iowait_reset(rfx_c, time, set))
		return;

	if (!set || rfx_c->iowait_boost_pending)
		return;
	rfx_c->iowait_boost_pending = true;

	if (rfx_c->iowait_boost) {
		rfx_c->iowait_boost = min_t(unsigned int,
					    rfx_c->iowait_boost << 1,
					    max_boost);
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

	/* In daily mode, restrict I/O boost to Little cores only to prevent waking Big/Prime */
	if (!rfx_c->rfx_policy->is_little)
		return 0;

	/* On active I/O boost, unpark Little cluster to service disk wait immediately */
	if (rfx_c->iowait_boost >= (SCHED_CAPACITY_SCALE / 4)) {
		rfx_c->rfx_policy->parked = false;
		rfx_c->rfx_policy->dormant = false;
	}

	return (unsigned long)rfx_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
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

/* Rate limiting & fast-path evaluation */

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
	if (gaming) {
		/*
		 * Commit upward OPP steps immediately in gaming so skill and
		 * crowded-scene bursts track demand without staircase lag;
		 * burst filters and directional EMA shape the ceiling/decay.
		 */
		p->up_rate_delay_ns = 0;
	} else {
		p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	}
}

static inline void rfx_set_eval_delay(struct rfx_policy *p, bool gaming)
{
	s64 base;

	if (gaming) {
		WRITE_ONCE(p->freq_update_delay_ns, (s64)RFX_FAST_RATE_US * NSEC_PER_USEC);
		return;
	}

	if (p->quiescent) {
		WRITE_ONCE(p->freq_update_delay_ns, (s64)28000 * NSEC_PER_USEC);
		return;
	}

	if (p->dormant) {
		WRITE_ONCE(p->freq_update_delay_ns, (s64)RFX_D_DORMANT_EVAL_US * NSEC_PER_USEC);
		return;
	}

	if (RFX_D_IDLE_EVAL_US && (p->parked || p->next_freq <= p->policy->min)) {
		WRITE_ONCE(p->freq_update_delay_ns, (s64)RFX_D_IDLE_EVAL_US * NSEC_PER_USEC);
		return;
	}

	base = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;

	if (RFX_D_LIGHT_EVAL_US &&
	    base < (s64)RFX_D_LIGHT_EVAL_US * NSEC_PER_USEC &&
	    p->next_freq <
	    rfx_pct(p->policy->cpuinfo.max_freq, RFX_D_LIGHT_EVAL_OPP_PCT)) {
		unsigned int opp_pct = (unsigned int)((u64)p->next_freq * 100 /
					p->policy->cpuinfo.max_freq);
		s64 stretch = (s64)RFX_D_LIGHT_EVAL_US * NSEC_PER_USEC - base;

		base += stretch * (RFX_D_LIGHT_EVAL_OPP_PCT - opp_pct) /
			RFX_D_LIGHT_EVAL_OPP_PCT;
		base = min_t(s64, base, (s64)RFX_D_LIGHT_EVAL_US * NSEC_PER_USEC);
	}
	WRITE_ONCE(p->freq_update_delay_ns, base);
}

/*
 * Instant touch/UI wake bypass: keeps deep 12-28ms idle polling silent when
 * background noise stays under unpark thresholds, while waking within 500us on
 * boosted top-app / SurfaceFlinger / load spikes to eliminate first-frame lag.
 */
static inline bool rfx_wake_urgent(struct rfx_policy *p, struct rfx_cpu *rfx_c,
				   s64 delta, unsigned int flags)
{
	unsigned long util, bwmin, max_cap, thresh;
	s64 cooldown_ns;

	if (likely(!READ_ONCE(p->quiescent) && !READ_ONCE(p->dormant) &&
		   !READ_ONCE(p->parked)))
		return false;

	if (p->is_little && (flags & SCHED_CPUFREQ_IOWAIT))
		return true;

	cooldown_ns = p->policy->fast_switch_enabled ?
		      (500 * NSEC_PER_USEC) : (1500 * NSEC_PER_USEC);
	if (delta < cooldown_ns)
		return false;

	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	if (!max_cap)
		return false;

	if (rfx_get_util_gki510) {
		rfx_get_util_gki510(rfx_c->cpu, 0, &util, &bwmin);
	} else {
		util = schedutil_cpu_util(rfx_c->cpu, 0, max_cap,
					  FREQUENCY_UTIL, NULL);
		bwmin = 0;
	}

	if (p->is_little)
		thresh = max_cap >> 3;		/* 12.5% */
	else if (p->is_prime)
		thresh = max_cap * 38 / 100;	/* 38% */
	else
		thresh = max_cap * 26 / 100;	/* 26% */

	return util >= thresh || bwmin > 0;
}

/*
 * Lockless fast-path gate: skips acquiring update_lock on unneeded scheduler ticks.
 */
static inline bool rfx_fast_should_update(struct rfx_policy *p, struct rfx_cpu *rfx_c,
					  u64 time, unsigned int flags)
{
	s64 delta;

	if (unlikely(READ_ONCE(p->limits_changed) || READ_ONCE(p->need_freq_update)))
		return true;

	delta = (s64)(time - READ_ONCE(p->last_eval_time));
	if (delta >= READ_ONCE(p->freq_update_delay_ns))
		return true;

	return rfx_wake_urgent(p, rfx_c, delta, flags);
}

static bool rfx_should_update_freq(struct rfx_policy *p, struct rfx_cpu *rfx_c,
				   u64 time, unsigned int flags)
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
	if (delta >= p->freq_update_delay_ns)
		return true;

	return rfx_wake_urgent(p, rfx_c, delta, flags);
}

static bool rfx_commit_freq(struct rfx_policy *p, u64 time, unsigned int next_freq)
{
	bool ignore_rate_limit = false;
	s64 delta;

	if (p->need_freq_update) {
		p->need_freq_update = false;
		ignore_rate_limit = true;
		if (p->next_freq == next_freq)
			return false;
	} else if (p->next_freq == next_freq) {
		return false;
	}

	if (next_freq < p->next_freq) {
		/* Bypass rate limits on forced updates or when actively parked in daily profile */
		if (!ignore_rate_limit && (rfx_gaming_enabled() || !p->parked)) {
			delta = (s64)(time - p->last_downfreq_time);
			if (p->down_rate_delay_ns > 0 && delta < p->down_rate_delay_ns)
				return false;
			if (p->min_sample_ns > 0 &&
			    (s64)(time - p->last_upfreq_time) < p->min_sample_ns)
				return false;
		}
		p->last_downfreq_time = time;
	} else {
		if (!ignore_rate_limit) {
			delta = (s64)(time - p->last_upfreq_time);
			if (p->up_rate_delay_ns > 0 && delta < p->up_rate_delay_ns)
				return false;
		}
		p->last_upfreq_time = time;
	}

	p->cached_raw_freq = p->pending_raw_freq;
	p->next_freq = next_freq;
	return true;
}

/* Frequency update entry & worker */

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

	{
		unsigned long prev_filt_util = p->filt_util;

		p->filt_util = rfx_ema(p->filt_util, max_util, time,
				       &p->last_ema_ns, gaming);

		if (gaming && !p->is_little && !p->is_prime) {
			unsigned int cur_demand = (unsigned int)(p->filt_util * 100 / max_cap);

			if (cur_demand >= 78) {
				if (!atomic64_read(&rfx_big_high_demand_since))
					atomic64_set(&rfx_big_high_demand_since, (s64)time);
			} else {
				atomic64_set(&rfx_big_high_demand_since, 0);
			}
		}

		rfx_set_down_delay(p, gaming);
		rfx_pol_up_delay(p, gaming);

		if (gaming) {
			p->parked = false;
			p->dormant = false;
			p->quiescent = false;
			p->quiescent_since_ns = 0;
		} else {
			unsigned int fmin = max(p->policy->cpuinfo.min_freq,
						READ_ONCE(p->policy->min));

			if (p->is_prime) {
				unsigned long prime_unpark_thresh = max_cap * 38 / 100; /* 38% */

				p->quiescent = false;
				p->quiescent_since_ns = 0;
				if (max_util >= prime_unpark_thresh) {
					p->park_enter_count = 0;
					if (p->parked) {
						if (++p->big_unpark_count >= 2 ||
						    max_util >= (max_cap * 55 / 100)) {
							p->parked = false;
							p->dormant = false;
							p->park_since_ns = 0;
							p->big_unpark_count = 0;
						}
					}
				} else {
					p->big_unpark_count = 0;
					if (++p->park_enter_count >= 2) {
						p->parked = true;
						p->dormant = true;
					}
				}
			} else if (!p->is_little) {
				unsigned long big_park_thresh = max_cap * 16 / 100;
				unsigned long unpark_thresh = max_cap * 26 / 100;

				p->quiescent = false;
				p->quiescent_since_ns = 0;
				if (max_util >= unpark_thresh) {
					p->park_enter_count = 0;
					if (p->parked) {
						/*
						 * Swift 1-eval unpark on decisive touch (>=30%)
						 * or sudden jump from quiet idle (<15% prior EMA).
						 */
						if (++p->big_unpark_count >= 2 ||
						    max_util >= (max_cap * 30 / 100) ||
						    prev_filt_util < (max_cap * 15 / 100)) {
							p->parked = false;
							p->dormant = false;
							p->park_since_ns = 0;
							p->big_unpark_count = 0;
						}
					}
				} else if (max_util < big_park_thresh) {
					p->big_unpark_count = 0;
					if (++p->park_enter_count >= RFX_D_PARK_ENTER_EVALS) {
						p->parked = true;
						p->dormant = true;
					}
				} else {
					/* In deadband (16% - 26%): retain park state */
					p->park_enter_count = 0;
				}
			} else {
				/* Little cluster: autonomous deep quiescent idle tracking */
				if (max_util < (max_cap >> 4)) {
					if (!p->quiescent_since_ns)
						p->quiescent_since_ns = time;
					else if (rfx_elapsed(time, p->quiescent_since_ns) >=
						 (350 * NSEC_PER_MSEC))
						p->quiescent = true;
				} else {
					p->quiescent = false;
					p->quiescent_since_ns = 0;
				}

				if (max_util < (max_cap >> 3)) {
					if (!p->parked) {
						p->parked = true;
						p->park_since_ns = time;
					} else if (!p->dormant && p->park_since_ns) {
						u64 dwell = rfx_elapsed(time, p->park_since_ns);

						if (dwell >= RFX_D_DORMANT_ENTER_NS)
							p->dormant = true;
					}
					p->filt_util = max_util;
					p->park_enter_count = 0;
				} else {
					p->parked = false;
					p->dormant = false;
					p->park_since_ns = 0;
					p->park_enter_count = 0;
				}
			}

			if (p->parked) {
				p->little_cap_lifted = false;
				p->big_cap_lifted = false;
				p->down_rate_delay_ns = 0;
				p->pending_raw_freq = fmin;
				return cpufreq_driver_resolve_freq(p->policy, fmin);
			}
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

	/* Lockless pre-check: skip spinlock if evaluation cadence has not elapsed */
	if (!rfx_fast_should_update(p, rfx_c, time, flags))
		return;

	raw_spin_lock_irqsave(&p->update_lock, irqflags);

	if (!gaming && (p->is_little || !p->parked))
		rfx_iowait_boost(rfx_c, time, flags);

	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (rfx_should_update_freq(p, rfx_c, time, flags)) {
		WRITE_ONCE(p->last_eval_time, time);
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
		rfx_set_eval_delay(p, gaming);
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

/* Sysfs interface */

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
	p->big_unpark_count = 0;
	p->little_floor_end_ns = 0;
	p->little_prev_demand = 0;
	p->parked = true;
	p->dormant = true;
	p->park_enter_count = 0;
	p->park_since_ns = 0;
	p->descend_hold_end_ns = 0;
	p->prev_demand_pct = 0;
	p->ceil_rise_pct = 100;
	p->ceil_rise_ref_ns = 0;
	p->ceil_fall_ns = 0;
	p->hispeed_hold_end_ns = 0;
	p->sustained_high_since_ns = 0;
	p->sustained_low_since_ns = 0;
	p->gaming_sustained_cap = false;
	p->thermal_energy_acc = 0;
	p->last_acc_time = 0;
	p->burst_high_since_ns = 0;
	p->quiescent = false;
	p->quiescent_since_ns = 0;
	p->stabilize_end_ns = 0;
	p->stabilize_freq = 0;
	p->filt_util = 0;
	p->last_ema_ns = 0;
	/* Keep hardware frequency tracking accurate across mode transitions */
	p->next_freq = p->policy->cur ? p->policy->cur : p->policy->min;
	p->cached_raw_freq = 0;
	p->pending_raw_freq = p->next_freq;
	WRITE_ONCE(p->need_freq_update, true);
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

	atomic64_set(&rfx_big_high_demand_since, 0);
}

static int rfx_pm_notifier(struct notifier_block *nb, unsigned long action,
			   void *data)
{
	struct rfx_policy *p;
	unsigned long flags, pflags;

	switch (action) {
	case PM_SUSPEND_PREPARE:
		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);
			p->parked = true;
			p->dormant = true;
			p->next_freq = p->policy->min;
			p->cached_raw_freq = p->policy->min;
			p->pending_raw_freq = p->policy->min;
			WRITE_ONCE(p->need_freq_update, true);
			if (p->policy->fast_switch_enabled) {
				cpufreq_driver_fast_switch(p->policy, p->policy->min);
			} else if (!p->work_in_progress) {
				p->work_in_progress = true;
				irq_work_queue(&p->irq_work);
			}
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
		atomic64_set(&rfx_input_ts, 0);
		atomic64_set(&rfx_tap_ts, 0);
		break;
	case PM_POST_SUSPEND:
		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);
			if (rfx_gaming_enabled()) {
				p->parked = false;
				p->dormant = false;
				p->gaming_warmup_pending = true;
				p->warmup_pending_entry = true;
			}
			WRITE_ONCE(p->need_freq_update, true);
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
		atomic64_set(&rfx_input_ts, 0);
		atomic64_set(&rfx_tap_ts, 0);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block rfx_pm_nb = {
	.notifier_call = rfx_pm_notifier,
};


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
		atomic64_set(&rfx_input_ts, 0);
		atomic64_set(&rfx_tap_ts, 0);
		rfx_reset_all_policies();
	} else {
		struct rfx_policy *p;
		unsigned long flags, pflags;

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);
			p->parked = false;
			p->dormant = false;
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

/* Governor registration & init */

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
		mutex_destroy(&p->work_lock);
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
		mutex_destroy(&p->work_lock);
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
		t->up_rate_limit_us = p->is_prime ? 1000 : RFX_BIG_UP_US;
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
	p->max_seen = policy->max;
	p->work_in_progress = false;
	p->limits_changed = false;

	rfx_reset_policy_locked(p);
	p->ceil_rise_ref_ns = now;
	p->next_freq = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	p->cached_raw_freq = p->next_freq;
	p->pending_raw_freq = p->next_freq;
	if (rfx_gaming_enabled()) {
		p->parked = false;
		p->dormant = false;
		p->gaming_warmup_pending = true;
		p->warmup_pending_entry = true;
	}
	WRITE_ONCE(p->need_freq_update, true);

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
	WARN_ON(rfx_ema(200, 100, t, &ns, false) != 200);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) != 1000);

	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, false) !=
		1000 - (1000 + RFX_EMA_DAILY_DIVISOR - 1) / RFX_EMA_DAILY_DIVISOR || ns != t);

	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) !=
		1000 - (1000 + RFX_EMA_GAMING_DIVISOR - 1) / RFX_EMA_GAMING_DIVISOR || ns != t);

	ns = t + NSEC_PER_MSEC;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) != 1000);

	ns = t - ((u64)(RFX_EMA_MAX_STEPS + 1) * RFX_EMA_DECAY_PERIOD_NS);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) != 839 || ns != t);

	ns = t - (20 * NSEC_PER_MSEC);
	WARN_ON(rfx_ema(1000, 0, t, &ns, false) != 0 || ns != t);

	gate = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE);
	over = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE + 2);
	WARN_ON(rfx_apply_headroom(gate, SCHED_CAPACITY_SCALE, true, false) != gate);
	WARN_ON(rfx_apply_headroom(over, SCHED_CAPACITY_SCALE, true, false) <= over);
	WARN_ON(rfx_apply_headroom(SCHED_CAPACITY_SCALE - 1, SCHED_CAPACITY_SCALE,
				   true, false) > SCHED_CAPACITY_SCALE);

	gate = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_DAILY_GATE_BIG);
	over = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_DAILY_GATE_BIG + 2);
	WARN_ON(rfx_apply_headroom(gate, SCHED_CAPACITY_SCALE, false, false) != gate);
	WARN_ON(rfx_apply_headroom(over, SCHED_CAPACITY_SCALE, false, false) <= over);
	WARN_ON(rfx_apply_headroom(SCHED_CAPACITY_SCALE - 1, SCHED_CAPACITY_SCALE,
				   false, false) > SCHED_CAPACITY_SCALE);

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

	/* Stabilizer: holds floor within dwell window, releases on expiry even at high demand */
	memset(&p, 0, sizeof(p));
	p.stabilize_freq = 500;
	p.stabilize_end_ns = t + RFX_G_STABILIZE_DWELL_NS;
	/* heavy demand within dwell: freq below the floor is pulled back up */
	WARN_ON(rfx_stabilize_opp(&p, 400, RFX_G_STABILIZE_DEMAND_PCT + 1, t) != 500);
	/* freq above the floor updates the setpoint and re-arms the dwell */
	WARN_ON(rfx_stabilize_opp(&p, 600, RFX_G_STABILIZE_DEMAND_PCT + 1, t) != 600);
	WARN_ON(!(p.stabilize_freq == 600 && p.stabilize_end_ns));
	/* within dwell window across inter-frame dip: holds stabilized frequency */
	WARN_ON(rfx_stabilize_opp(&p, 400, 30,
				  t + RFX_G_STABILIZE_DWELL_NS / 2) != 600);
	WARN_ON(p.stabilize_freq != 600);
	/* after dwell expiry under heavy demand: settles cleanly to lower demand */
	WARN_ON(rfx_stabilize_opp(&p, 450, RFX_G_STABILIZE_DEMAND_PCT + 1,
				  t + RFX_G_STABILIZE_DWELL_NS + 1) != 450);
	WARN_ON(p.stabilize_freq != 450);
	/* ...and clears completely on dwell expiry under low demand */
	WARN_ON(rfx_stabilize_opp(&p, 400, 30,
				  t + RFX_G_STABILIZE_DWELL_NS * 3) == 450);
	WARN_ON(p.stabilize_freq);

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
	WARN_ON(p.gaming_warmup_pending);
	p.gaming_warmup_pending = false;
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1, t);
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1,
			       t + RFX_GAMING_REARM_QUIET_NS - 1);
	WARN_ON(p.gaming_warmup_pending);
	rfx_warmup_rearm_quiet(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1,
			       t + RFX_GAMING_REARM_QUIET_NS);
	WARN_ON(p.gaming_warmup_pending != true);
}

#ifdef CONFIG_INPUT
static bool rfx_input_registered __read_mostly;

static void rfx_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	u64 now;

	if (!rfx_gaming_enabled())
		return;

	if (type == EV_KEY) {
		if (code == KEY_POWER || code == KEY_SLEEP ||
		    code == KEY_MUTE || code == KEY_VOLUMEDOWN ||
		    code == KEY_VOLUMEUP)
			return;
		now = sched_clock();
		atomic64_set(&rfx_input_ts, (s64)now);
		if (value)
			atomic64_set(&rfx_tap_ts, (s64)now);
		return;
	}

	if (type == EV_ABS) {
		if (code == ABS_MT_TRACKING_ID && value >= 0) {
			now = sched_clock();
			atomic64_set(&rfx_input_ts, (s64)now);
			atomic64_set(&rfx_tap_ts, (s64)now);
		} else if (code == ABS_MT_POSITION_X ||
			   code == ABS_MT_POSITION_Y) {
			atomic64_set(&rfx_input_ts, (s64)sched_clock());
		}
	}
}

static int rfx_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "vorpal";

	error = input_register_handle(handle);
	if (error)
		goto err_free;
	error = input_open_device(handle);
	if (error)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return error;
}

static void rfx_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id rfx_input_ids[] = {
	{	/* touchscreen */
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
				BIT_MASK(ABS_MT_POSITION_X) },
	},
	{	/* keys / gamepad buttons */
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler rfx_input_handler = {
	.event		= rfx_input_event,
	.connect	= rfx_input_connect,
	.disconnect	= rfx_input_disconnect,
	.name		= "vorpal",
	.id_table	= rfx_input_ids,
};
#endif

static int __init vorpal_gov_init(void)
{
	int ret;

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
	BUILD_BUG_ON(RFX_G_BIG_CAP_PCT > RFX_G_BIG_BURST_CAP_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_CAP_PCT > RFX_G_PRIME_BURST_CAP_PCT);
	BUILD_BUG_ON(RFX_G_BIG_BURST_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_BURST_CAP_PCT > 100);
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
	BUILD_BUG_ON(RFX_D_PARK_ENTER_EVALS < 1);
	BUILD_BUG_ON(RFX_LITTLE_CAP_THRESHOLD >= RFX_PRIME_CAP_THRESHOLD);
	BUILD_BUG_ON(RFX_EMA_GAMING_DIVISOR < 1 || RFX_EMA_DAILY_DIVISOR < 1 || RFX_EMA_MAX_STEPS < 1);
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
	BUILD_BUG_ON(RFX_G_COOL_PRIME_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_G_COOL_PRIME_CAP_PCT <= RFX_G_PRIME_FLOOR_PCT);
	BUILD_BUG_ON(RFX_HEADROOM_DAILY_GATE_LITTLE >= 100);
	BUILD_BUG_ON(RFX_HEADROOM_DAILY_GATE_BIG >= 100);
	BUILD_BUG_ON(RFX_HEADROOM_GAMING_GATE >= 100);
	BUILD_BUG_ON(RFX_SAT_TO_MAX_GAMING_PCT > 100);
	BUILD_BUG_ON(RFX_SAT_TO_MAX_DAILY_PCT > 100);

	BUILD_BUG_ON(RFX_FAST_RATE_US < 1);
	BUILD_BUG_ON(RFX_G_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_GO_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_TOUCH_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_LITTLE_TOUCH_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_TOUCH_PCT_DEFAULT < RFX_G_BIG_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_LITTLE_TOUCH_PCT_DEFAULT < RFX_G_LITTLE_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_DOWN_FAST_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_THERMAL_STAGE0_ACC >= RFX_G_THERMAL_STAGE1_ACC);
	BUILD_BUG_ON(RFX_G_THERMAL_STAGE1_ACC >= RFX_G_THERMAL_STAGE2_ACC);
	BUILD_BUG_ON(RFX_G_THERMAL_STAGE2_ACC >= RFX_G_THERMAL_STAGE3_ACC);
	BUILD_BUG_ON(RFX_G_THERMAL_STAGE3_ACC > RFX_G_THERMAL_ACC_MAX);
	BUILD_BUG_ON(RFX_G_ENERGY_AWARE_DEFAULT > 1);

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	rfx_selfcheck();

	register_pm_notifier(&rfx_pm_nb);
#ifdef CONFIG_INPUT
	ret = input_register_handler(&rfx_input_handler);
	if (ret)
		pr_warn("vorpal: input handler registration failed %d\n", ret);
	else
		rfx_input_registered = true;
#endif
	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret) {
#ifdef CONFIG_INPUT
		if (rfx_input_registered) {
			input_unregister_handler(&rfx_input_handler);
			rfx_input_registered = false;
		}
#endif
		unregister_pm_notifier(&rfx_pm_nb);
	}
	return ret;
}

static void __exit vorpal_gov_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
#ifdef CONFIG_INPUT
	if (rfx_input_registered) {
		input_unregister_handler(&rfx_input_handler);
		rfx_input_registered = false;
	}
#endif
	unregister_pm_notifier(&rfx_pm_nb);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.3");
