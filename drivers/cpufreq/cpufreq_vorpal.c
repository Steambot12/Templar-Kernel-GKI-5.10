// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor — schedutil-derived, dual-profile (gaming/daily).
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
#include <linux/input.h>
#include <linux/suspend.h>

#define CREATE_TRACE_POINTS
#include <trace/events/vorpal.h>

#define CPUFREQ_VORPAL_NAME     "vorpal"
#define CPUFREQ_VORPAL_VERSION  "2.3"
#define CPUFREQ_VORPAL_AUTHOR   "Steambot12"

extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern unsigned long rfx_get_uclamp_min_gki510(int cpu);
extern unsigned long rfx_get_uclamp_max_gki510(int cpu);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);
extern int rfx_setattr_sugov_gki510(struct task_struct *t);

#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

#define RFX_LITTLE_RATE_US		3000
#define RFX_LITTLE_UP_US		1200
#define RFX_LITTLE_DOWN_US		2000

#define RFX_BIG_RATE_US			3000
#define RFX_BIG_UP_US			3500
#define RFX_BIG_DOWN_US			2500

/* Asymmetric EMA filter for gaming mode: attack 90%, decay 25% */
#define RFX_G_EMA_ATTACK_PCT		90
#define RFX_G_EMA_DECAY_PCT		25

/* Sustained gaming baseline floors & efficiency ceilings */
#define RFX_G_PRIME_FLOOR_PCT		24
#define RFX_G_BIG_FLOOR_PCT		32
#define RFX_G_LITTLE_FLOOR_PCT		22
#define RFX_G_LITTLE_CAP_PCT		78

/* Proactive hysteretic thermal dampening stages based on thermal pressure */
#define RFX_THERMAL_WARM_ARM_DEPTH	14
#define RFX_THERMAL_WARM_CLEAR_DEPTH	8
#define RFX_THERMAL_WARM_DWELL_NS	((u64)2000 * NSEC_PER_MSEC)

#define RFX_THERMAL_HOT_ARM_DEPTH	28
#define RFX_THERMAL_HOT_CLEAR_DEPTH	18
#define RFX_THERMAL_HOT_DWELL_NS	((u64)3500 * NSEC_PER_MSEC)

#define RFX_THERMAL_CRIT_ARM_DEPTH	48
#define RFX_THERMAL_CRIT_CLEAR_DEPTH	32
#define RFX_THERMAL_CRIT_DWELL_NS	((u64)5000 * NSEC_PER_MSEC)

#define RFX_THERMAL_WARM_CAP_PRIME_PCT	74
#define RFX_THERMAL_WARM_CAP_BIG_PCT	82
#define RFX_THERMAL_HOT_CAP_PRIME_PCT	66
#define RFX_THERMAL_HOT_CAP_BIG_PCT	74
#define RFX_THERMAL_CRIT_CAP_PRIME_PCT	58
#define RFX_THERMAL_CRIT_CAP_BIG_PCT	66

/* Warmup/entry ramps for gaming mode */
#define RFX_G_TOUCH_PCT_DEFAULT		40
#define RFX_G_TOUCH_MS_DEFAULT		100
#define RFX_G_ENERGY_AWARE_DEFAULT	0

#define RFX_G_PRIME_TOUCH_PCT		32
#define RFX_G_PRIME_CAP_PCT		82
#define RFX_G_BIG_CAP_PCT		88
#define RFX_G_BURST_UNLOCK_PCT		72

#define RFX_G_EVAL_US_DEFAULT		1200
#define RFX_G_HISPEED_PCT_DEFAULT	65

#define RFX_G_GO_HISPEED_PCT_DEFAULT	68
#define RFX_G_HISPEED_HOLD_US_DEFAULT	16000

#define RFX_D_LITTLE_CAP_PCT		38
#define RFX_D_LITTLE_FLOOR_PCT		18
#define RFX_D_LITTLE_FLOOR_REARM_PCT	6
#define RFX_D_LITTLE_FLOOR_NS		(24 * NSEC_PER_MSEC)
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	44
#define RFX_D_LITTLE_LIFT_PCT		52
#define RFX_D_LITTLE_DROP_PCT		32

#define RFX_D_BIG_LIFT_PCT		52
#define RFX_D_BIG_DROP_PCT		32
#define RFX_D_TOUCH_FLOOR_MIN_DEMAND	25
#define RFX_D_BIG_TOUCH_FLOOR_PCT	18
#define RFX_D_BIG_SUSTAINED_CAP_PCT	52

/* Daily sweet-spot efficiency shaping: Big 48%, Prime 38%, unlock at >= 82% */
#define RFX_D_SWEET_CAP_BIG_PCT		48
#define RFX_D_SWEET_CAP_PRIME_PCT	38
#define RFX_D_BURST_UNLOCK_PCT		82

/* Prime quiet gate: keep Cortex-X at low OPP during light daily loads */
#define RFX_D_PRIME_QUIET_PCT		38
#define RFX_D_PRIME_QUIET_KNEE_PCT	18

/* Sub-linear light-load mapping below knee */
#define RFX_D_SUBLINEAR_KNEE_PCT	38

/* Idle floor snap: park cluster to policy->min when idle */
#define RFX_D_IDLE_FLOOR_SNAP_PCT	14

/* Time-delta zero-snap across quiet gaps */
#define RFX_D_TIME_DECAY_SNAP_NS		(6 * NSEC_PER_MSEC)
#define RFX_D_TIME_DECAY_SNAP_UTIL_PCT	16

/* Fast downward rate limits for daily mode */
#define RFX_D_DOWN_US			1000
#define RFX_D_TOUCH_DOWN_US		1500

/* Baseline iowait boost caps per cluster tier */
#define RFX_D_IOWAIT_LITTLE_PCT	25
#define RFX_D_IOWAIT_BIG_PCT	10
#define RFX_D_IOWAIT_PRIME_PCT	0

#define RFX_D_IDLE_EVAL_US		35000
#define RFX_D_LIGHT_EVAL_OPP_PCT	30
#define RFX_D_LIGHT_EVAL_US		60000
#define RFX_D_ENERGY_AWARE		1
#define RFX_D_ENERGY_AWARE_MIN_PCT	0
#define RFX_D_PARK_EXIT_PCT		20
#define RFX_D_PARK_EXIT_EVALS		2
#define RFX_D_TOUCH_WINDOW_MS		48

/* Standby / prolonged quiescence threshold (3.5s without touch in daily mode) */
#define RFX_D_STANDBY_QUIET_NS		((u64)3500 * NSEC_PER_MSEC)

#define RFX_EMA_DECAY_PERIOD_NS		250000

#define RFX_EMA_DAILY_DIVISOR		4

#define RFX_EMA_MAX_STEPS		40

#define RFX_HEADROOM_DAILY_LITTLE	6
#define RFX_HEADROOM_DAILY_TOUCH	10
#define RFX_HEADROOM_DAILY_BIG		5

#define RFX_HEADROOM_GAMING		15

#define RFX_CEIL_RISE_PCT_PER_2MS	1

#define RFX_CEIL_FALL_DWELL_NS		(15 * NSEC_PER_MSEC)
#define RFX_CEIL_FALL_BYPASS_PCT	8

#define RFX_G_COOL_ENTER_PCT		65
#define RFX_G_COOL_EXIT_PCT		72

#define RFX_G_COOL_STEADY_FLOOR_PCT	50

#define RFX_G_COOL_DEEP_PCT		60

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/* Quiescent eco states: stage 1 (light idle) and stage 2 (deep idle / screen-off) */
#define RFX_ECO_INPUT_QUIET_NS		((u64)1200 * NSEC_PER_MSEC)
#define RFX_ECO_DEEP_INPUT_QUIET_NS	((u64)2500 * NSEC_PER_MSEC)
#define RFX_ECO_LITTLE_CAP_PCT		28
#define RFX_ECO_BIG_CAP_PCT		28
#define RFX_ECO_DEEP_LITTLE_CAP_PCT	18
#define RFX_ECO_DEEP_BIG_CAP_PCT	10
#define RFX_ECO_UCMIN_PCT		25
#define RFX_ECO_DEEP_UCMIN_PCT		12
#define RFX_ECO_EXIT_UCMIN_PCT		30
#define RFX_ECO_LITE_DEMAND_PCT		35
#define RFX_ECO_DEEP_DEMAND_PCT		18
#define RFX_ECO_IOWAIT_LITTLE_PCT	12

static atomic_t rfx_gaming ____cacheline_aligned = ATOMIC_INIT(0);

bool vorpal_is_gaming_mode(void)
{
	return READ_ONCE(rfx_gaming.counter) != 0;
}
EXPORT_SYMBOL_GPL(vorpal_is_gaming_mode);

static inline bool rfx_gaming_enabled(void)
{
	return vorpal_is_gaming_mode();
}

static atomic64_t rfx_interactive_ts = ATOMIC64_INIT(0);
static bool rfx_suspended;

/* Time since the last interactive boost, 0 while no boost has been seen */
static inline u64 rfx_input_quiet_ns(u64 time)
{
	u64 ts = (u64)atomic64_read(&rfx_interactive_ts);

	if (!ts || time < ts)
		return 0;
	return time - ts;
}

static inline bool rfx_is_standby(u64 time)
{
	if (unlikely(READ_ONCE(rfx_suspended)))
		return true;

	if (!rfx_gaming_enabled()) {
		u64 quiet_ns = rfx_input_quiet_ns(time);
		if (quiet_ns >= RFX_D_STANDBY_QUIET_NS)
			return true;
	}
	return false;
}

static inline bool rfx_touch_active(u64 time, u64 window_ns)
{
	u64 ts;

	if (rfx_is_standby(time))
		return false;

	ts = (u64)atomic64_read(&rfx_interactive_ts);
	return (ts != 0 && time >= ts && (time - ts) < window_ns);
}

static LIST_HEAD(rfx_policy_list);
static DEFINE_SPINLOCK(rfx_policy_list_lock);

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

	unsigned long filt_util;
	u64 last_ema_ns;

	bool little_cap_lifted;
	u64 little_floor_end_ns;
	bool big_cap_lifted;
	bool parked;
	unsigned int park_exit_count;
	/* Universal quiescent eco stage latches (daily mode only) */
	bool eco_lite_active;
	bool eco_deep_active;
	bool dl_bw_exceeded;
	unsigned long max_dl_bw;
	bool thermal_cooling;

	/* Hysteretic thermal dampening: ratcheted stage level + ref time */
	unsigned int thermal_stage;
	u64 thermal_stage_ref_ns;

	unsigned int ceil_rise_pct;
	u64 ceil_rise_ref_ns;
	u64 ceil_fall_ns;

	u64 hispeed_hold_end_ns;
	unsigned int dbg_demand_pct;
	unsigned int dbg_fceil_pct;
	int dbg_pr;
	int dbg_warmup_active;
};

struct rfx_cpu {
	struct update_util_data update_util;
	struct rfx_policy *rfx_policy;
	unsigned int cpu;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;
	u64 last_iowait_decay;

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

static unsigned long rfx_caps_sorted[4];
static int rfx_num_tiers;
static DEFINE_SPINLOCK(rfx_topo_lock);

static void rfx_init_topology(void)
{
	unsigned long flags;
	int cpu, i, j;
	unsigned long c;

	spin_lock_irqsave(&rfx_topo_lock, flags);
	if (rfx_num_tiers > 1) {
		spin_unlock_irqrestore(&rfx_topo_lock, flags);
		return;
	}

	rfx_num_tiers = 0;
	for_each_possible_cpu(cpu) {
		c = arch_scale_cpu_capacity(cpu);
		for (i = 0; i < rfx_num_tiers; i++) {
			if (rfx_caps_sorted[i] == c)
				break;
		}
		if (i == rfx_num_tiers && rfx_num_tiers < (int)ARRAY_SIZE(rfx_caps_sorted))
			rfx_caps_sorted[rfx_num_tiers++] = c;
	}

	for (i = 0; i < rfx_num_tiers - 1; i++) {
		for (j = i + 1; j < rfx_num_tiers; j++) {
			if (rfx_caps_sorted[i] > rfx_caps_sorted[j]) {
				unsigned long tmp = rfx_caps_sorted[i];
				rfx_caps_sorted[i] = rfx_caps_sorted[j];
				rfx_caps_sorted[j] = tmp;
			}
		}
	}
	spin_unlock_irqrestore(&rfx_topo_lock, flags);
}

static int rfx_ntiers(void)
{
	if (unlikely(!rfx_num_tiers))
		rfx_init_topology();
	return rfx_num_tiers;
}

static inline bool rfx_cap_is_little(unsigned long cap)
{
	if (unlikely(!rfx_num_tiers))
		rfx_init_topology();
	if (cap > (unsigned long)RFX_LITTLE_CAP_THRESHOLD)
		return false;
	if (rfx_num_tiers <= 1)
		return true;
	return cap == rfx_caps_sorted[0];
}

static inline bool rfx_cap_is_top(unsigned long cap)
{
	if (unlikely(!rfx_num_tiers))
		rfx_init_topology();
	if (rfx_num_tiers <= 1)
		return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
	return cap == rfx_caps_sorted[rfx_num_tiers - 1];
}

static inline bool rfx_cap_is_prime(unsigned long cap)
{
	if (unlikely(!rfx_num_tiers))
		rfx_init_topology();
	if (cap < (unsigned long)896)
		return false;
	return rfx_num_tiers >= 3 && rfx_cap_is_top(cap);
}

static inline unsigned int rfx_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

static unsigned int rfx_thermal_headroom_pct(struct rfx_policy *p,
					     unsigned long max_cap,
					     unsigned int *baseline,
					     unsigned long *out_press)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int press_pct = 100, clamp_pct = 100;
	unsigned int pmax = READ_ONCE(pol->max);
	unsigned long press;

	if (pmax > p->max_seen)
		p->max_seen = pmax;
	*baseline = p->max_seen ? p->max_seen : pol->cpuinfo.max_freq;

	press = arch_scale_thermal_pressure(cpumask_first(pol->related_cpus));
	if (out_press)
		*out_press = press;

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

	/*
	 * In gaming mode, seamlessly integrate CFS thermal pressure into
	 * the ceiling calculation rather than waiting for severe hardware
	 * cooling-device drops (clamp_pct). By scaling progressively as
	 * thermal pressure rises, we smooth the V/F curve and prevent
	 * sudden 50% thermal throttling cliffs that cause frame drops.
	 */
	if (rfx_gaming_enabled()) {
		if (press_pct < 100) {
			unsigned int scaled_press = 100 - ((100 - press_pct) * 3 / 4);

			return min(clamp_pct, scaled_press);
		}
		return clamp_pct;
	}

	return min(press_pct, clamp_pct);
}

static inline u64 rfx_elapsed(u64 time, u64 stamp)
{
	s64 delta = (s64)(time - stamp);

	return delta > 0 ? (u64)delta : 0;
}

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
		if (rfx_elapsed(time, p->ceil_fall_ns) <
		    RFX_CEIL_FALL_DWELL_NS)
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

static unsigned long rfx_ema(unsigned long old, unsigned long val, u64 time,
			     u64 *last_ns, bool smooth, bool gaming,
			     unsigned long max)
{
	u64 delta_ns;
	unsigned int steps;

	if (!smooth) {
		*last_ns = time;
		return val;
	}

	/*
	 * Gaming mode uses an asymmetric per-sample filter: a sample above
	 * the filtered value attacks fast so frame bursts are seen
	 * immediately, a sample at or below the value decays slowly so
	 * inter-frame idle gaps cannot collapse the perceived utilisation.
	 */
	if (gaming) {
		unsigned int alpha;

		if (!old)
			old = val;

		if (val > old)
			alpha = RFX_G_EMA_ATTACK_PCT;
		else
			alpha = RFX_G_EMA_DECAY_PCT;

		*last_ns = time;
		return (unsigned long)((u64)val * alpha +
				       (u64)old * (100 - alpha)) / 100;
	}

	/* Daily mode: periodic decay toward the newest sample */
	if (!old || val >= old) {
		*last_ns = time;
		return val;
	}

	if (unlikely(!*last_ns))
		*last_ns = time - RFX_EMA_DECAY_PERIOD_NS;
	delta_ns = rfx_elapsed(time, *last_ns);

	/*
	 * Time-delta zero-snap: after a long quiet gap (> 10ms, e.g. a
	 * brief timer/wakelock wake followed by idle), the divisor-based
	 * decay lags and would hold the cluster above its floor on the
	 * next evaluation. With near-zero demand, collapse the filter
	 * straight to the newest sample so the target snaps to the idle
	 * floor on the very next tick.
	 */
	if (delta_ns > RFX_D_TIME_DECAY_SNAP_NS &&
	    val * 100 < RFX_D_TIME_DECAY_SNAP_UTIL_PCT * max) {
		*last_ns = time;
		return val;
	}

	steps = (unsigned int)min_t(u64, delta_ns / RFX_EMA_DECAY_PERIOD_NS,
				    RFX_EMA_MAX_STEPS);
	if (!steps)
		return old;

	if (steps < RFX_EMA_MAX_STEPS)
		*last_ns += (u64)steps * RFX_EMA_DECAY_PERIOD_NS;
	else
		*last_ns = time;

	while (steps--) {
		unsigned long diff = old - val;

		if (!diff)
			break;
		old -= max_t(unsigned long, diff / RFX_EMA_DAILY_DIVISOR, 1);
	}

	return old;
}

static unsigned long rfx_apply_headroom(unsigned long util, unsigned long max_cap,
					bool gaming, bool little, bool touch)
{
	if (!max_cap || util >= max_cap)
		return max_cap;

	if (gaming)
		return min(util + (util * RFX_HEADROOM_GAMING / 100), max_cap);

	if (touch)
		return min(util + (util * RFX_HEADROOM_DAILY_TOUCH / 100), max_cap);

	if (little)
		return min(util + (util * RFX_HEADROOM_DAILY_LITTLE / 100), max_cap);

	/* Big/Prime non-touch daily: gentle headroom for micro-tasks */
	return min(util + (util * RFX_HEADROOM_DAILY_BIG / 100), max_cap);
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
 * Proactive thermal ratchet for gaming mode. Escalates fast when depth
 * crosses a stage's arm band (25/50/75) and releases only when depth
 * stays below the current stage's clear band for its dwell window
 * (2/3/4s). Pressure that recovers after a frequency drop cannot
 * de-escalate immediately, which is what keeps the temperature curve
 * slow and stable instead of tracking pressure spikes.
 */
static void rfx_thermal_stage_update(struct rfx_policy *p, unsigned int depth,
				    u64 time)
{
	unsigned int stage = p->thermal_stage;

	/* Escalate: any arm band at or below current depth takes effect now */
	if (depth >= RFX_THERMAL_CRIT_ARM_DEPTH)
		stage = 3;
	else if (depth >= RFX_THERMAL_HOT_ARM_DEPTH)
		stage = max(stage, 2);
	else if (depth >= RFX_THERMAL_WARM_ARM_DEPTH)
		stage = max(stage, 1);

	/*
	 * De-escalate one level at a time: require the current stage's clear
	 * band plus its dwell timer before dropping back, so single
	 * pressure drops cannot chase the temperature back up.
	 */
	switch (stage) {
	case 1:
		if (depth <= RFX_THERMAL_WARM_CLEAR_DEPTH &&
		    time >= p->thermal_stage_ref_ns + RFX_THERMAL_WARM_DWELL_NS) {
			stage--;
			p->thermal_stage_ref_ns = time;
		}
		break;
	case 2:
		if (depth <= RFX_THERMAL_HOT_CLEAR_DEPTH &&
		    time >= p->thermal_stage_ref_ns + RFX_THERMAL_HOT_DWELL_NS) {
			stage--;
			p->thermal_stage_ref_ns = time;
		}
		break;
	case 3:
		if (depth <= RFX_THERMAL_CRIT_CLEAR_DEPTH &&
		    time >= p->thermal_stage_ref_ns + RFX_THERMAL_CRIT_DWELL_NS) {
			stage--;
			p->thermal_stage_ref_ns = time;
		}
		break;
	default:
		break;
	}

	if (stage == 0)
		p->thermal_stage_ref_ns = 0;
	else if (stage != p->thermal_stage)
		p->thermal_stage_ref_ns = time;

	p->thermal_stage = stage;
}

/* Effective thermal cap percentage of fceil for the cluster's tier */
static unsigned int rfx_thermal_cap_pct(struct rfx_policy *p)
{
	unsigned int stage = p->thermal_stage;
	bool prime = p->is_prime;

	switch (stage) {
	case 3:
		return prime ? RFX_THERMAL_CRIT_CAP_PRIME_PCT :
			       RFX_THERMAL_CRIT_CAP_BIG_PCT;
	case 2:
		return prime ? RFX_THERMAL_HOT_CAP_PRIME_PCT :
			       RFX_THERMAL_HOT_CAP_BIG_PCT;
	case 1:
		return prime ? RFX_THERMAL_WARM_CAP_PRIME_PCT :
			       RFX_THERMAL_WARM_CAP_BIG_PCT;
	}
	return 100;
}

static unsigned int rfx_target_freq(struct rfx_policy *p, unsigned long util,
				    unsigned long max_uclamp_min,
				    unsigned long max_uclamp_max,
				    unsigned long max_cap, u64 time, bool gaming)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int fmin = pol->min;
	bool little = p->is_little;
	bool prime = p->is_prime;
	bool touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);
	unsigned int freq;
	unsigned long raw_util = util;
	unsigned long thermal_press = 0;
	unsigned int th_pct = 0;

	unsigned int fmax;
	unsigned int fceil;
	unsigned int fceil_pct;

	if (unlikely(!pol->cpuinfo.max_freq || !max_cap))
		return pol->cur;

	fceil_pct = rfx_thermal_headroom_pct(p, max_cap, &fmax, &thermal_press);
	if (unlikely(!fmax))
		return pol->cur;

	if (max_cap)
		th_pct = (unsigned int)(thermal_press * 100 / max_cap);

	if (gaming) {
		/* Fast, responsive ceiling tracking in gaming mode for immediate recovery */
		fceil = rfx_pct(fmax, fceil_pct);
		p->ceil_rise_pct = fceil_pct;
		p->ceil_rise_ref_ns = time;
		p->ceil_fall_ns = 0;
	} else {
		fceil = rfx_pct(fmax, rfx_ceil_rise_filter(p, fceil_pct, time));
	}
	fceil = clamp(fceil, fmin, fmax);
	p->dbg_fceil_pct = fceil_pct;

	p->dbg_pr = thermal_press;

	util = rfx_apply_headroom(util, max_cap, gaming, little, touch);

	freq = (unsigned int)((u64)pol->cpuinfo.max_freq * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		unsigned int fl, demand_pct;
		unsigned int depth;
		unsigned int boost_fl = 0;
		unsigned int tier_cap;
		bool is_2tier = (rfx_ntiers() == 2);

		demand_pct = (unsigned int)(raw_util * 100 / max_cap);
		p->dbg_demand_pct = demand_pct;
		depth = rfx_cool_depth(fceil_pct);

		/* Gaming efficiency ceilings: Prime 82%, Big 88% (90% 2-tier), Little 78% */
		if (demand_pct >= RFX_G_BURST_UNLOCK_PCT)
			tier_cap = 100;
		else if (prime)
			tier_cap = RFX_G_PRIME_CAP_PCT;
		else if (!little)
			tier_cap = is_2tier ? 90 : RFX_G_BIG_CAP_PCT;
		else
			tier_cap = RFX_G_LITTLE_CAP_PCT;

		fceil = min(fceil, rfx_pct(pol->cpuinfo.max_freq, tier_cap));

		if (fceil_pct < RFX_G_COOL_ENTER_PCT && !p->thermal_cooling)
			p->thermal_cooling = true;
		if (fceil_pct >= RFX_G_COOL_EXIT_PCT && p->thermal_cooling)
			p->thermal_cooling = false;

		if (!little) {
			rfx_thermal_stage_update(p, depth, time);
			if (p->thermal_stage)
				fceil = min(fceil, rfx_pct(fmax, rfx_thermal_cap_pct(p)));
		}

		/* Sustained sweet-spot gaming floors: Prime 24%, Big 32% (28% 2-tier), Little 22% */
		if (prime)
			fl = rfx_pct(fceil, RFX_G_PRIME_FLOOR_PCT);
		else if (!little)
			fl = rfx_pct(fceil, is_2tier ? 28 : RFX_G_BIG_FLOOR_PCT);
		else
			fl = rfx_pct(fceil, RFX_G_LITTLE_FLOOR_PCT);

		/* Scale floor down during active thermal cooling */
		if (p->thermal_cooling && depth && fl) {
			unsigned int steady = rfx_pct(fceil,
						      RFX_G_COOL_STEADY_FLOOR_PCT);

			if (fl > steady)
				fl -= (fl - steady) * (depth / 2) / 100;
		}

		/* Hispeed hold: demand >= 78% holds 56% floor for 16ms */
		if (!little && demand_pct >= RFX_G_GO_HISPEED_PCT_DEFAULT)
			p->hispeed_hold_end_ns = time +
				(u64)RFX_G_HISPEED_HOLD_US_DEFAULT * NSEC_PER_USEC;

		if (!little && p->hispeed_hold_end_ns && time < p->hispeed_hold_end_ns) {
			unsigned int hs_fl = min(rfx_pct(fceil, RFX_G_HISPEED_PCT_DEFAULT), fceil);
			boost_fl = max(boost_fl, hs_fl);
		}

#if RFX_G_TOUCH_PCT_DEFAULT && RFX_G_TOUCH_MS_DEFAULT
		if (touch) {
			unsigned int touch_pct;

			if (little) {
				touch_pct = 28;
				boost_fl = max(boost_fl,
					       min(rfx_pct(fceil, touch_pct), fceil));
			} else if (demand_pct >= 20) {
				if (prime)
					touch_pct = RFX_G_PRIME_TOUCH_PCT;
				else
					touch_pct = is_2tier ? 34 : 36;

				boost_fl = max(boost_fl,
					       min(rfx_pct(fceil, touch_pct), fceil));
			}
		}
#endif

		if (freq < boost_fl)
			freq = boost_fl;
		if (freq < fl)
			freq = fl;

		if (max_uclamp_min) {
			unsigned int uclamp_freq = (unsigned int)((u64)pol->cpuinfo.max_freq *
						   max_uclamp_min / max_cap);

			uclamp_freq = clamp(uclamp_freq, fmin, fceil);
			if (freq < uclamp_freq)
				freq = uclamp_freq;
		}

		/* Thermal Pressure Jump Smoothing */
		if (!little && th_pct >= 10 && freq > p->next_freq) {
			unsigned int max_up_step = (demand_pct >= RFX_G_BURST_UNLOCK_PCT) ?
				rfx_pct(fmax, 40) : rfx_pct(fmax, 25);

			if (freq > p->next_freq + max_up_step)
				freq = p->next_freq + max_up_step;
		}

		p->dbg_warmup_active = (boost_fl > 0);
	} else {
		unsigned int cap, demand_pct;
		unsigned int raw_cpu_pct = 0;
		unsigned int ucmin_pct =
			(unsigned int)(max_uclamp_min * 100 / max_cap);
		unsigned int ema_pct =
			(unsigned int)((u64)p->filt_util * 100 / max_cap);
		unsigned int big_cap_pct = touch ? 50 : RFX_D_SWEET_CAP_BIG_PCT;
		unsigned int prime_cap_pct = touch ? 38 : RFX_D_SWEET_CAP_PRIME_PCT;
		unsigned int little_cap_pct = touch ? 42 : RFX_D_LITTLE_CAP_PCT;

		/* Universal quiescent eco state: stage 1 (light idle), stage 2 (deep idle) */
		if (touch || p->dl_bw_exceeded || ucmin_pct >= RFX_ECO_EXIT_UCMIN_PCT ||
		    rfx_input_quiet_ns(time) < RFX_ECO_INPUT_QUIET_NS) {
			p->eco_lite_active = false;
			p->eco_deep_active = false;
		} else {
			if (p->eco_deep_active) {
				if (ucmin_pct >= RFX_ECO_DEEP_UCMIN_PCT ||
				    p->dl_bw_exceeded ||
				    ema_pct >= RFX_ECO_DEEP_DEMAND_PCT ||
				    rfx_input_quiet_ns(time) <
				    RFX_ECO_DEEP_INPUT_QUIET_NS)
					p->eco_deep_active = false;
			} else if (rfx_input_quiet_ns(time) >=
				   RFX_ECO_DEEP_INPUT_QUIET_NS &&
				   !p->dl_bw_exceeded &&
				   ucmin_pct < RFX_ECO_DEEP_UCMIN_PCT &&
				   ema_pct < RFX_ECO_DEEP_DEMAND_PCT) {
				p->eco_deep_active = true;
			}
			if (p->eco_lite_active) {
				if (ucmin_pct >= RFX_ECO_UCMIN_PCT ||
				    p->dl_bw_exceeded ||
				    ema_pct >= RFX_ECO_LITE_DEMAND_PCT ||
				    rfx_input_quiet_ns(time) <
				    RFX_ECO_INPUT_QUIET_NS)
					p->eco_lite_active = false;
			} else if (!p->eco_deep_active &&
				   rfx_input_quiet_ns(time) >=
				   RFX_ECO_INPUT_QUIET_NS &&
				   !p->dl_bw_exceeded &&
				   ucmin_pct < RFX_ECO_UCMIN_PCT &&
				   ema_pct < RFX_ECO_LITE_DEMAND_PCT) {
				p->eco_lite_active = true;
			}
		}

		demand_pct = (unsigned int)(raw_util * 100 / max_cap);
		p->dbg_demand_pct = demand_pct;

		if (p->dl_bw_exceeded) {
			unsigned long raw_cpu = (raw_util > p->max_dl_bw) ?
						(raw_util - p->max_dl_bw) : 0;
			raw_cpu_pct = (unsigned int)(raw_cpu * 100 / max_cap);
		}

		if (little) {
			cap = rfx_pct(fceil, little_cap_pct);

			if (demand_pct >= RFX_D_LITTLE_LIFT_PCT)
				p->little_cap_lifted = true;
			else if (demand_pct <= RFX_D_LITTLE_DROP_PCT || !touch)
				p->little_cap_lifted = false;
			if (p->little_cap_lifted || touch)
				cap = rfx_pct(fceil, touch ? 42 :
					      RFX_D_LITTLE_SUSTAINED_CAP_PCT);

			if (touch && demand_pct >= RFX_D_TOUCH_FLOOR_MIN_DEMAND)
				p->little_floor_end_ns = time + RFX_D_LITTLE_FLOOR_NS;
			else if (!touch || demand_pct < RFX_D_LITTLE_FLOOR_REARM_PCT)
				p->little_floor_end_ns = 0;

			/* Little floor: demand-gated touch floor and multimedia floor */
			if (touch && (p->little_floor_end_ns && time < p->little_floor_end_ns)) {
				unsigned int touch_fl = rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT);
				if (freq < touch_fl)
					freq = touch_fl;
			} else if (p->dl_bw_exceeded || ucmin_pct >= 8 ||
				   (p->little_floor_end_ns && time < p->little_floor_end_ns)) {
				unsigned int media_fl = rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT);
				if (freq < media_fl)
					freq = media_fl;
			}

			/* Eco stage Little capping under light demand */
			if (demand_pct < RFX_D_BURST_UNLOCK_PCT &&
			    p->eco_deep_active)
				cap = rfx_pct(fceil, RFX_ECO_DEEP_LITTLE_CAP_PCT);
			else if (demand_pct < RFX_D_BURST_UNLOCK_PCT &&
				 p->eco_lite_active)
				cap = min(cap, rfx_pct(fceil, RFX_ECO_LITTLE_CAP_PCT));

			/* DL bandwidth cap for Little cluster */
			if (p->dl_bw_exceeded && raw_cpu_pct < RFX_D_BURST_UNLOCK_PCT)
				cap = min(cap, rfx_pct(fceil, RFX_D_LITTLE_SUSTAINED_CAP_PCT));
		} else {
			if (demand_pct >= RFX_D_BIG_LIFT_PCT)
				p->big_cap_lifted = true;
			else if (demand_pct <= RFX_D_BIG_DROP_PCT || !touch)
				p->big_cap_lifted = false;

			/* Daily Big/Prime sweet-spot cap: Big 48%, Prime 38%; unlock >= 82% */
			if (demand_pct >= RFX_D_BURST_UNLOCK_PCT &&
			    (!p->dl_bw_exceeded || raw_cpu_pct >= RFX_D_BURST_UNLOCK_PCT))
				cap = fceil;
			else if (prime && demand_pct < RFX_D_PRIME_QUIET_PCT &&
				   max_uclamp_min * 100 <
				   (unsigned long)RFX_D_PRIME_QUIET_PCT *
				   max_cap) {
				cap = rfx_pct(fceil, RFX_D_PRIME_QUIET_KNEE_PCT);
			} else {
				unsigned int target_cap_pct = prime ? prime_cap_pct : big_cap_pct;

				if (!prime && (p->big_cap_lifted || (touch && demand_pct >= 40)))
					target_cap_pct = (rfx_ntiers() == 2) ? 56 : RFX_D_BIG_SUSTAINED_CAP_PCT;

				cap = rfx_pct(fceil, target_cap_pct);
			}

			if (p->dl_bw_exceeded && raw_cpu_pct < RFX_D_BURST_UNLOCK_PCT) {
				unsigned int dl_cap = rfx_pct(fceil,
					prime ? prime_cap_pct :
						big_cap_pct);
				if (cap > dl_cap)
					cap = dl_cap;
			}

			/* Sub-linear light load quadratic mapping */
			if (!touch && !p->dl_bw_exceeded && ucmin_pct < 8) {
				unsigned int ema_pct =
					(unsigned int)((u64)p->filt_util *
						    100 / max_cap);

				if (ema_pct < RFX_D_SUBLINEAR_KNEE_PCT) {
					unsigned int eff =
						(unsigned int)((u64)ema_pct *
							ema_pct /
							RFX_D_SUBLINEAR_KNEE_PCT);
					unsigned int sub_cap =
						rfx_pct(fceil, eff);

					if (freq > sub_cap)
						freq = sub_cap;
				}
			}

			if (touch && !prime && demand_pct >= RFX_D_TOUCH_FLOOR_MIN_DEMAND) {
				unsigned int touch_fl = rfx_pct(fceil, RFX_D_BIG_TOUCH_FLOOR_PCT);

				if (freq < touch_fl)
					freq = touch_fl;
			}

			/* Multimedia protection floor on Big cluster (16%) */
			if (!prime && (p->dl_bw_exceeded || ucmin_pct >= 12)) {
				unsigned int media_fl = rfx_pct(fceil, 16);

				if (freq < media_fl)
					freq = media_fl;
			}

			/* Standby / Eco stage Big/Prime capping under light demand */
			if (demand_pct < RFX_D_BURST_UNLOCK_PCT) {
				if (rfx_is_standby(time) || p->eco_deep_active)
					cap = prime ? 0 : min(cap, rfx_pct(fceil,
							RFX_ECO_DEEP_BIG_CAP_PCT));
				else if (p->eco_lite_active)
					cap = prime ? 0 :
						min(cap, rfx_pct(fceil,
							    RFX_ECO_BIG_CAP_PCT));
			}
		}

		if (freq > cap)
			freq = cap;

		if (max_uclamp_min && (touch || p->dl_bw_exceeded || demand_pct >= 15 || ucmin_pct >= 20)) {
			unsigned int uclamp_freq = (unsigned int)((u64)pol->cpuinfo.max_freq *
						   max_uclamp_min / max_cap);
			unsigned int uclamp_cap;

			if (demand_pct >= RFX_D_BURST_UNLOCK_PCT)
				uclamp_cap = fceil;
			else if (rfx_is_standby(time))
				uclamp_cap = prime ? fmin : (little ? rfx_pct(fceil, RFX_ECO_DEEP_LITTLE_CAP_PCT) :
							    rfx_pct(fceil, RFX_ECO_DEEP_BIG_CAP_PCT));
			else if (prime)
				uclamp_cap = rfx_pct(fceil, prime_cap_pct);
			else if (little)
				uclamp_cap = rfx_pct(fceil, (p->little_cap_lifted || touch) ?
						     RFX_D_LITTLE_SUSTAINED_CAP_PCT : little_cap_pct);
			else
				uclamp_cap = rfx_pct(fceil, (p->big_cap_lifted || touch) ?
						     ((rfx_ntiers() == 2) ? 56 : RFX_D_BIG_SUSTAINED_CAP_PCT) : big_cap_pct);

			uclamp_freq = clamp(uclamp_freq, fmin, uclamp_cap);
			if (freq < uclamp_freq)
				freq = uclamp_freq;
		}
	}

	/* Android UCLAMP_MAX enforcement across all modes */
	if (max_uclamp_max && max_uclamp_max < max_cap) {
		unsigned int uclamp_max_freq = (unsigned int)((u64)pol->cpuinfo.max_freq *
					       max_uclamp_max / max_cap);
		uclamp_max_freq = clamp(uclamp_max_freq, fmin, fceil);
		if (freq > uclamp_max_freq)
			freq = uclamp_max_freq;
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

			if (idx >= 0) {
				pol->cached_resolved_idx = idx;
				return pol->freq_table[idx].frequency;
			}
		}
	}
	return cpufreq_driver_resolve_freq(pol, freq);
}

static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time, bool set)
{
	u64 delta_ns = rfx_elapsed(time, rfx_c->last_update);
	u64 timeout = rfx_gaming_enabled() ? ((u64)TICK_NSEC * 4) : TICK_NSEC;

	if (delta_ns <= timeout)
		return false;

	rfx_c->iowait_boost = set ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time, unsigned int flags)
{
	bool set = flags & SCHED_CPUFREQ_IOWAIT;
	struct rfx_policy *p = rfx_c->rfx_policy;
	unsigned int cap;

	/*
	 * Filter I/O wait boost on Big and Prime clusters.
	 * Prime cluster NEVER receives iowait boost in either gaming or daily mode.
	 * In daily mode, background I/O daemons must not wake Big from low OPP,
	 * and quiescent eco state bars iowait from waking Big out of policy->min.
	 */
	if (p && (p->is_prime ||
		 (!rfx_gaming_enabled() && !p->is_little &&
		  (p->eco_lite_active || p->eco_deep_active)))) {
		rfx_c->iowait_boost = 0;
		rfx_c->iowait_boost_pending = false;
		return;
	}

	if (rfx_c->iowait_boost && rfx_iowait_reset(rfx_c, time, set))
		return;

	if (!set)
		return;

	if (rfx_c->iowait_boost_pending)
		return;
	rfx_c->iowait_boost_pending = true;

	if (rfx_c->iowait_boost) {
		if (rfx_gaming_enabled()) {
			if (p && p->is_little)
				cap = SCHED_CAPACITY_SCALE / 4;
			else
				cap = rfx_pct(SCHED_CAPACITY_SCALE, 16); /* Capped to 16% on Big during gaming */
		} else if (p && p->is_little) {
			/*
			 * Baseline Little tier: soft background I/O boost,
			 * tightened to the eco cap while a quiescent eco
			 * stage is active.
			 */
			unsigned int eco_cap = rfx_pct(SCHED_CAPACITY_SCALE,
					RFX_ECO_IOWAIT_LITTLE_PCT);

			if (p->eco_lite_active || p->eco_deep_active)
				cap = min(eco_cap, rfx_pct(SCHED_CAPACITY_SCALE,
						RFX_D_IOWAIT_LITTLE_PCT));
			else
				cap = rfx_pct(SCHED_CAPACITY_SCALE,
					      RFX_D_IOWAIT_LITTLE_PCT);
		} else {
			/*
			 * Baseline Big tier (only reached daily and outside
			 * the eco state, since eco-active and Prime
			 * clusters are filtered above): a small boost
			 * keeps background Big work moving without
			 * dragging Big into high-voltage OPPs. No policy
			 * pointer yet keeps the conservative legacy cap.
			 */
			cap = p ? rfx_pct(SCHED_CAPACITY_SCALE, RFX_D_IOWAIT_BIG_PCT)
				: SCHED_CAPACITY_SCALE / 8;
		}

		rfx_c->iowait_boost = min_t(unsigned int,
					    rfx_c->iowait_boost << 1, cap);
		rfx_c->last_iowait_decay = time;
		return;
	}
	rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
	rfx_c->last_iowait_decay = time;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time,
				      unsigned long max_cap)
{
	if (likely(!rfx_c->iowait_boost))
		return 0;
	if (rfx_iowait_reset(rfx_c, time, false))
		return 0;
	if (!rfx_c->iowait_boost_pending) {
		rfx_c->last_iowait_decay = time;
		rfx_c->iowait_boost >>= 1;
		if (rfx_c->iowait_boost < IOWAIT_BOOST_MIN) {
			rfx_c->iowait_boost = 0;
			return 0;
		}
	} else {
		rfx_c->last_iowait_decay = time;
	}
	rfx_c->iowait_boost_pending = false;
	return rfx_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
}

static void rfx_get_util(struct rfx_cpu *rfx_c, unsigned long boost)
{
	rfx_get_util_gki510(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin)) {
		WRITE_ONCE(rfx_c->rfx_policy->limits_changed, true);
		rfx_c->rfx_policy->need_freq_update = true;
	}
}

static inline void rfx_set_down_delay(struct rfx_policy *p, bool gaming, u64 time)
{
	if (gaming) {
		/* 16ms delay bridges 60/90/120Hz frame intervals without mid-frame drop */
		p->down_rate_delay_ns = (s64)16000 * NSEC_PER_USEC;
	} else {
		bool touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

		if (p->parked) {
			/* Idle park: zero downward delay for instant deep C-state entry */
			p->down_rate_delay_ns = 0;
			return;
		}

		if (touch)
			p->down_rate_delay_ns = (s64)RFX_D_TOUCH_DOWN_US * NSEC_PER_USEC;
		else
			p->down_rate_delay_ns = (s64)RFX_D_DOWN_US * NSEC_PER_USEC;
	}
}

static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming, unsigned int demand_pct)
{
	if (gaming) {
		/* Instant ramp-up (0us) in gaming mode for Big/Prime cores or demand >= 20% */
		if (!p->is_little || demand_pct >= 20)
			p->up_rate_delay_ns = 0;
		else
			p->up_rate_delay_ns = (s64)250 * NSEC_PER_USEC;
	} else {
		bool touch = rfx_touch_active(sched_clock(), (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

		/* Daily mode: 0us on touch with demand for zero-latency UI dispatch */
		if (touch && (p->is_little || demand_pct >= 20))
			p->up_rate_delay_ns = 0;
		else
			p->up_rate_delay_ns = (s64)500 * NSEC_PER_USEC;
	}
}

static inline void rfx_set_eval_delay(struct rfx_policy *p, bool gaming, u64 time)
{
	s64 base;
	bool touch;

	if (gaming) {
		p->freq_update_delay_ns = p->policy->fast_switch_enabled ?
			(s64)RFX_G_EVAL_US_DEFAULT * NSEC_PER_USEC :
			(s64)max_t(unsigned int, RFX_G_EVAL_US_DEFAULT, 2500U) * NSEC_PER_USEC;
		return;
	}

	if (unlikely(READ_ONCE(rfx_suspended))) {
		p->freq_update_delay_ns = (s64)500000 * NSEC_PER_USEC;
		return;
	}

	if (unlikely(rfx_is_standby(time))) {
		if (p->parked) {
			p->freq_update_delay_ns = p->is_little ?
				(s64)100000 * NSEC_PER_USEC :
				(s64)250000 * NSEC_PER_USEC;
			return;
		}
	}
	base = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	if (!p->policy->fast_switch_enabled)
		base = max_t(s64, base, (s64)2500 * NSEC_PER_USEC);
	touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

	if (RFX_D_IDLE_EVAL_US && p->parked && !touch) {
		p->freq_update_delay_ns =
			(s64)RFX_D_IDLE_EVAL_US * NSEC_PER_USEC;
		return;
	}

	/*
	 * Deep eco stage: stretch the evaluation period so background
	 * wakelocks do not churn the policy, capped at the idle-eval
	 * window since detection is input-quiescent only (no panel
	 * state) and video/audio wakeups must stay responsive. The
	 * DL bandwidth path forces an immediate pass regardless, and
	 * a touch or uclamp_min spike exits the stage on that pass.
	 */
	if (p->eco_deep_active && !touch) {
		p->freq_update_delay_ns =
			(s64)RFX_D_IDLE_EVAL_US * NSEC_PER_USEC;
		return;
	}

	/*
	 * Light eco stage (media/video playback): stretch evaluation
	 * period to 8ms when touch is inactive to reduce governor
	 * overhead during sustained streaming.
	 */
	if (p->eco_lite_active && !touch) {
		p->freq_update_delay_ns =
			(s64)max_t(unsigned int, p->tunables->rate_limit_us, 8000U) * NSEC_PER_USEC;
		return;
	}

	if (RFX_D_LIGHT_EVAL_US && !touch && p->next_freq <
	    rfx_pct(p->policy->cpuinfo.max_freq, RFX_D_LIGHT_EVAL_OPP_PCT)) {
		unsigned int opp_pct = (unsigned int)((u64)p->next_freq * 100 /
					p->policy->cpuinfo.max_freq);
		s64 stretch = (s64)RFX_D_LIGHT_EVAL_US * NSEC_PER_USEC -
			      base;

		base += stretch * (100 - opp_pct) /
			(100 - RFX_D_LIGHT_EVAL_OPP_PCT);
		base = min_t(s64, base, (s64)RFX_D_LIGHT_EVAL_US * NSEC_PER_USEC);
	}
	p->freq_update_delay_ns = base;
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
	unsigned int deadband_hz;

	if (p->next_freq == next_freq) {
		p->need_freq_update = false;
		return false;
	}

	/*
	 * Pure OPP-Index Deadband Filter:
	 * If the policy has a frequency table, verify if the target frequency
	 * resolves to the exact same OPP table index as p->next_freq. If so,
	 * skip the update completely without touching hardware registers.
	 */
	if (!p->need_freq_update && p->next_freq) {
		if (p->policy->freq_table) {
			int cur_idx = cpufreq_frequency_table_target(p->policy, p->next_freq,
								     CPUFREQ_RELATION_L);
			int next_idx = cpufreq_frequency_table_target(p->policy, next_freq,
								      CPUFREQ_RELATION_L);
			if (cur_idx >= 0 && cur_idx == next_idx) {
				p->need_freq_update = false;
				return false;
			}
		} else {
			unsigned int db_pct = 4;

			deadband_hz = (unsigned int)((u64)p->policy->cpuinfo.max_freq * db_pct / 100);
			if (abs((int)next_freq - (int)p->next_freq) < (int)deadband_hz) {
				p->need_freq_update = false;
				return false;
			}
		}
	}

	p->need_freq_update = false;

	if (next_freq < p->next_freq) {
		delta = (s64)(time - p->last_downfreq_time);
		if (p->down_rate_delay_ns > 0 && delta < p->down_rate_delay_ns)
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

static unsigned int rfx_next_freq(struct rfx_cpu *rfx_c, u64 time, bool gaming)
{
	struct rfx_policy *p = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	unsigned long max_util = 0;
	unsigned long max_uclamp_min = 0;
	unsigned long max_uclamp_max = 0;
	unsigned long max_dl_bw = 0;
	bool dl_bw_exceeded = false;
	unsigned int j;
	bool touch;

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *jc = per_cpu_ptr(&rfx_cpu, j);
		unsigned long jb, je, uc_min, uc_max;

		if (rfx_dl_bw_exceeded_gki510(j, jc->bwmin))
			dl_bw_exceeded = true;
		if (jc->bwmin > max_dl_bw)
			max_dl_bw = jc->bwmin;

		jb = rfx_iowait_apply(jc, time, max_cap);
		rfx_get_util(jc, jb);
		uc_min = rfx_get_uclamp_min_gki510(j);
		if (uc_min > max_uclamp_min)
			max_uclamp_min = uc_min;
		uc_max = rfx_get_uclamp_max_gki510(j);
		if (uc_max > max_uclamp_max)
			max_uclamp_max = uc_max;
		je = max(jc->util, jb);
		if (gaming) {
			if (uc_min > je)
				je = min(uc_min, max_cap);
		}

		if (je > max_util)
			max_util = je;
	}

	p->dl_bw_exceeded = dl_bw_exceeded;
	p->max_dl_bw = max_dl_bw;

	/* Detect interactive user touch / UI boost via standard Android UCLAMP */
	if (max_uclamp_min >= (max_cap * 8 / 100))
		atomic64_set(&rfx_interactive_ts, (s64)time);

	touch = !gaming && rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

	p->filt_util = rfx_ema(p->filt_util, max_util, time, &p->last_ema_ns,
			       true, gaming, max_cap);

	rfx_set_down_delay(p, gaming, time);
	rfx_pol_up_delay(p, gaming, max_cap ? (unsigned int)(max_util * 100 / max_cap) : 0);

	if (!gaming) {
		unsigned int fmin = p->policy->min;
		unsigned int park_thresh =
			(unsigned int)(max_cap * RFX_D_IDLE_FLOOR_SNAP_PCT / 100);

		/* Instant Suspend Park: keep all clusters parked to fmin during PM suspend */
		if (unlikely(READ_ONCE(rfx_suspended))) {
			p->parked = true;
			p->park_exit_count = 0;
			p->pending_raw_freq = fmin;
			p->down_rate_delay_ns = 0;
			return fmin;
		}

		/* Instant Standby / Quiescent Park: keep Big & Prime hard parked to fmin */
		if (unlikely(rfx_is_standby(time))) {
			if (!p->is_little) {
				if (max_util >= (max_cap * 75 / 100) || p->dl_bw_exceeded) {
					/* Genuine sustained heavy background task: allow unpark */
				} else {
					p->parked = true;
					p->park_exit_count = 0;
					p->pending_raw_freq = fmin;
					p->down_rate_delay_ns = 0;
					return fmin;
				}
			} else {
				/* Little cluster in standby: snap to fmin on light activity (< 10%) */
				if (max_util == 0 || p->filt_util < (max_cap * 10 / 100)) {
					p->parked = true;
					p->park_exit_count = 0;
					p->pending_raw_freq = fmin;
					p->down_rate_delay_ns = 0;
					return fmin;
				}
			}
		}

		/* Fast Idle Snap to policy->min: park immediately when util drops to 0 */
		if (max_util == 0 || (p->filt_util < (max_cap * 3 / 100) && max_uclamp_min == 0)) {
			p->parked = true;
			p->park_exit_count = 0;
			p->pending_raw_freq = fmin;
			p->down_rate_delay_ns = 0;
			return fmin;
		}

		/* Deep Eco Stage: park Big/Prime during quiescent deep idle */
		if (p->eco_deep_active && !touch &&
		    (max_util < (max_cap * RFX_D_BURST_UNLOCK_PCT / 100))) {
			if (!p->is_little) {
				p->parked = true;
				p->park_exit_count = 0;
				p->pending_raw_freq = fmin;
				return fmin;
			}
		}

		if (p->is_prime) {
			/* Prime conservation: keep Cortex-X parked unless heavy demand >= 75% */
			unsigned int prime_snap = (cpumask_weight(p->policy->cpus) > 1 && p->is_prime) ? 70 : 80;

			if (p->filt_util < (max_cap * prime_snap / 100) && max_uclamp_min < (max_cap * 50 / 100)) {
				p->parked = true;
				p->park_exit_count = 0;
				p->pending_raw_freq = fmin;
				return fmin;
			}
			if (++p->park_exit_count >= RFX_D_PARK_EXIT_EVALS)
				p->parked = false;
		} else if (!p->is_little) {
			/* Big conservation: park when idle (< 20%) or keep parked if no touch activity */
			if (!touch && p->filt_util < (max_cap * 20 / 100) && max_uclamp_min < (max_cap * 20 / 100)) {
				p->parked = true;
				p->park_exit_count = 0;
				p->pending_raw_freq = fmin;
				return fmin;
			}
			if (touch) {
				/* Unpark on touch only if there is genuine active demand on Big (>= 30%) */
				if (max_util >= (max_cap * 30 / 100) ||
				    p->filt_util >= (max_cap * 30 / 100)) {
					p->parked = false;
					p->park_exit_count = RFX_D_PARK_EXIT_EVALS;
				} else if (p->parked) {
					/* Completely idle or light during touch: stay parked at fmin */
					p->pending_raw_freq = fmin;
					return fmin;
				}
			} else if (p->parked) {
				/* Without touch, unpark Big only for genuine heavy load (>= 50%) */
				if (p->filt_util >= (max_cap * 50 / 100)) {
					if (++p->park_exit_count >= 2)
						p->parked = false;
				} else {
					p->park_exit_count = 0;
					p->pending_raw_freq = fmin;
					return fmin;
				}
			}
		} else {
			/* Little cluster: park to fmin when true idle */
			if (!touch && p->filt_util < park_thresh && max_uclamp_min == 0) {
				p->parked = true;
				p->park_exit_count = 0;
				p->pending_raw_freq = fmin;
				return fmin;
			}
			if (touch) {
				p->parked = false;
				p->park_exit_count = RFX_D_PARK_EXIT_EVALS;
			} else if (p->parked) {
				if (p->filt_util >= (max_cap * RFX_D_PARK_EXIT_PCT / 100)) {
					if (++p->park_exit_count >= RFX_D_PARK_EXIT_EVALS)
						p->parked = false;
				} else {
					p->park_exit_count = 0;
				}
			}
		}

		if (p->parked) {
			p->pending_raw_freq = fmin;
			return fmin;
		}
	}

	return rfx_target_freq(p, p->filt_util, max_uclamp_min, max_uclamp_max, max_cap, time, gaming);
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

	raw_spin_lock_irqsave(&p->update_lock, irqflags);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);
	rfx_set_eval_delay(p, gaming, time);

	if (rfx_should_update_freq(p, time)) {
		p->last_eval_time = time;
		next_f = rfx_next_freq(rfx_c, time, gaming);
		next_f = cpufreq_driver_resolve_freq(p->policy, next_f);
		if (rfx_commit_freq(p, time, next_f)) {

			unsigned long cap = arch_scale_cpu_capacity(rfx_c->cpu);
			int pr_pct = p->dbg_pr ?
				(int)((u64)p->dbg_pr * 100 / cap) : 0;

			trace_vorpal(rfx_c->cpu, atomic_read(&rfx_gaming),
				     p->next_freq, p->dbg_demand_pct,
				     p->dbg_fceil_pct, pr_pct,
				     p->dbg_warmup_active, p->is_prime);

			if (p->policy->fast_switch_enabled) {
				unsigned int actual_f;

				actual_f = cpufreq_driver_fast_switch(p->policy,
								      p->next_freq);
				if (actual_f) {
					p->policy->cur = actual_f;
					p->next_freq = actual_f;
				}
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
	freq = clamp(freq, p->policy->min, p->policy->max);
	__cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);

	raw_spin_lock_irqsave(&p->update_lock, flags);
	p->policy->cur = freq;
	if (p->work_in_progress) {
		irq_work_queue(&p->irq_work);
	} else {
		p->next_freq = freq;
	}
	raw_spin_unlock_irqrestore(&p->update_lock, flags);
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *p = container_of(irq_work, struct rfx_policy, irq_work);

	kthread_queue_work(&p->worker, &p->work);
}

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

/*
 * Profile state flush helpers. On a 1->0 (gaming to daily) transition all
 * gaming-specific latches and holds are zeroed so they cannot influence
 * the next daily-mode evaluation. On a 0->1 (daily to gaming) transition
 * all daily-specific park/eco caps are flushed. Both helpers must be
 * called under p->update_lock.
 */
static void rfx_flush_gaming_state_locked(struct rfx_policy *p)
{
	p->thermal_stage = 0;
	p->thermal_stage_ref_ns = 0;
	p->hispeed_hold_end_ns = 0;
	p->ceil_rise_pct = 100;
	p->ceil_rise_ref_ns = 0;
	p->ceil_fall_ns = 0;
}

static void rfx_flush_daily_state_locked(struct rfx_policy *p)
{
	p->parked = false;
	p->park_exit_count = 0;
	p->little_cap_lifted = false;
	p->big_cap_lifted = false;
	p->eco_lite_active = false;
	p->eco_deep_active = false;
	p->dl_bw_exceeded = false;
	p->max_dl_bw = 0;
	p->little_floor_end_ns = 0;
}

static void rfx_reset_policy_locked(struct rfx_policy *p)
{
	/* Full reset: used on policy start/stop and as the base for both
	 * flush helpers. Zeroes all transient state fields. */
	rfx_flush_gaming_state_locked(p);
	rfx_flush_daily_state_locked(p);
	p->thermal_cooling = false;
	p->ceil_rise_pct = 100;
	p->ceil_rise_ref_ns = 0;
	p->ceil_fall_ns = 0;
	p->filt_util = 0;
	p->last_ema_ns = 0;
	p->need_freq_update = true;
}

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", atomic_read(&rfx_gaming));
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	unsigned int val;
	int prev;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	prev = atomic_xchg(&rfx_gaming, val);
	if (prev == (int)val)
		return count;
	smp_mb();

	if (!val) {
		/* 1->0 transition: flush every gaming latch so nothing can
		 * bleed into the daily profile, and restore the daily
		 * rate-limit windows. */
		struct rfx_policy *p;
		unsigned long flags, pflags;

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			/* Full profile flush (gaming + daily latches). */
			rfx_reset_policy_locked(p);

			p->up_rate_delay_ns = (s64)500 * NSEC_PER_USEC;
			p->down_rate_delay_ns =
				(s64)p->tunables->down_rate_limit_us *
					 NSEC_PER_USEC;
			p->freq_update_delay_ns =
				(s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
			p->need_freq_update = true;
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
	} else {
		/* 0->1 transition: full flush, then arm the gaming rate-limit
		 * windows. */
		struct rfx_policy *p;
		unsigned long flags, pflags;

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);

			p->down_rate_delay_ns = (s64)16000 * NSEC_PER_USEC;
			p->up_rate_delay_ns = 0;
			p->freq_update_delay_ns = p->policy->fast_switch_enabled ?
				(s64)RFX_G_EVAL_US_DEFAULT * NSEC_PER_USEC :
				(s64)max_t(unsigned int, RFX_G_EVAL_US_DEFAULT, 2500U) * NSEC_PER_USEC;

			p->need_freq_update = true;
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
	&gaming_mode.attr,
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

	ret = rfx_setattr_sugov_gki510(thread);
	if (ret) {
		struct sched_param param = { .sched_priority = MAX_RT_PRIO / 2 };

		ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
		if (ret) {
			kthread_stop(thread);
			pr_warn("vorpal: failed to set SCHED_FIFO priority\n");
			return ret;
		}
	}

	p->thread = thread;

	if (!policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, policy->related_cpus);

	wake_up_process(thread);
	return 0;
}

static void rfx_kthread_stop(struct rfx_policy *p)
{
	if (!p->policy->fast_switch_enabled && p->thread) {
		kthread_flush_worker(&p->worker);
		kthread_stop(p->thread);
		p->thread = NULL;
	}
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

	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;

	p->last_upfreq_time = now;
	p->last_downfreq_time = now;
	p->last_eval_time = now;
	p->next_freq = policy->cur > 0 ? policy->cur : policy->min;
	if (policy->fast_switch_enabled)
		cpufreq_driver_resolve_freq(policy, p->next_freq);
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

	for_each_cpu(cpu, policy->cpus)
		cpufreq_add_update_util_hook(cpu,
			&per_cpu_ptr(&rfx_cpu, cpu)->update_util, rfx_update);
	return 0;
}

static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	unsigned long flags;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_del(&p->gov_node);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	irq_work_sync(&p->irq_work);
	kthread_cancel_work_sync(&p->work);
}

static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;

	if (!p)
		return;

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
	u64 ns;

	ns = 0;
	WARN_ON(rfx_ema(100, 200, t, &ns, true, false, 1000) != 200 || ns != t);

	ns = t;
	WARN_ON(rfx_ema(200, 100, t, &ns, false, false, 1000) != 100);

	/* Gaming: per-sample asymmetric blend, time-independent */
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true, 1000) != 750 || ns != t);
	WARN_ON(rfx_ema(100, 200, t, &ns, true, true, 1000) != 165);
	WARN_ON(rfx_ema(0, 200, t, &ns, true, true, 1000) != 200);
	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true, 1000) != 750 || ns != t);

	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false, 1000) !=
		1000 - 1000 / RFX_EMA_DAILY_DIVISOR || ns != t);

	ns = t + NSEC_PER_MSEC;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true, 1000) != 750);

	/* Daily zero-snap: a long quiet gap with near-zero demand collapses
	 * the filter straight to the newest sample. */
	ns = t - RFX_D_TIME_DECAY_SNAP_NS - 1;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false, 1000) != 0 || ns != t);

	/* Exactly at the snap boundary the snap does not fire (strict >);
	 * the step clamp still leaves a non-zero residual. */
	ns = t - RFX_D_TIME_DECAY_SNAP_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false, 1000) == 0 || ns != t);

	/* Demand at/over the snap util threshold keeps the divisor decay
	 * even across a long quiet gap. */
	ns = t - RFX_D_TIME_DECAY_SNAP_NS - 1;
	WARN_ON(rfx_ema(1000, RFX_D_TIME_DECAY_SNAP_UTIL_PCT * 10, t,
			&ns, true, false, 1000) == 0 || ns != t - 1);

	/* Step clamp: catch-up is bounded to RFX_EMA_MAX_STEPS decay
	 * steps, so a full-range value does not fully drain in one
	 * evaluation and the clamp pins last_ns to the evaluation time. */
	ns = t - ((u64)(RFX_EMA_MAX_STEPS + 1) * RFX_EMA_DECAY_PERIOD_NS);
	{
		unsigned long res = rfx_ema(1000, 0, t, &ns, true, false, 1000);

		WARN_ON(res >= 1000 || ns != t);
	}

	WARN_ON(rfx_apply_headroom(500, SCHED_CAPACITY_SCALE, true, false, false) <= 500);
	WARN_ON(rfx_apply_headroom(900, SCHED_CAPACITY_SCALE, true, false, false) != SCHED_CAPACITY_SCALE);
	WARN_ON(rfx_apply_headroom(SCHED_CAPACITY_SCALE - 1, SCHED_CAPACITY_SCALE,
				   true, false, false) != SCHED_CAPACITY_SCALE);

	WARN_ON(rfx_cool_depth(RFX_G_COOL_EXIT_PCT) != 0);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_EXIT_PCT + 15) != 0);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_DEEP_PCT) != 100);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_DEEP_PCT - 10) != 100);
	WARN_ON(rfx_cool_depth(RFX_G_COOL_DEEP_PCT +
		(RFX_G_COOL_EXIT_PCT - RFX_G_COOL_DEEP_PCT) / 2) != 50);
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
}

static int rfx_pm_notifier(struct notifier_block *nb, unsigned long action, void *data)
{
	struct rfx_policy *p;
	unsigned long flags;

	switch (action) {
	case PM_SUSPEND_PREPARE:
		WRITE_ONCE(rfx_suspended, true);
		smp_wmb();
		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			p->parked = true;
			p->park_exit_count = 0;
			p->pending_raw_freq = p->policy->min;
			p->next_freq = p->policy->min;
			p->down_rate_delay_ns = 0;
			p->freq_update_delay_ns = (s64)500000 * NSEC_PER_USEC;
			WRITE_ONCE(p->need_freq_update, true);
			if (p->policy->fast_switch_enabled)
				cpufreq_driver_fast_switch(p->policy, p->policy->min);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
		break;
	case PM_POST_SUSPEND:
		WRITE_ONCE(rfx_suspended, false);
		smp_wmb();
		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			p->park_exit_count = 0;
			WRITE_ONCE(p->need_freq_update, true);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block rfx_pm_nb = {
	.notifier_call = rfx_pm_notifier,
};

static int __init vorpal_gov_init(void)
{
	int ret;

	BUILD_BUG_ON(RFX_G_COOL_ENTER_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_DROP_PCT >= RFX_D_LITTLE_LIFT_PCT);
	BUILD_BUG_ON(RFX_D_BIG_DROP_PCT >= RFX_D_BIG_LIFT_PCT);
	BUILD_BUG_ON(RFX_G_LITTLE_FLOOR_PCT > 100);
	BUILD_BUG_ON(RFX_G_EMA_ATTACK_PCT > 100 || RFX_G_EMA_DECAY_PCT > 100);
	BUILD_BUG_ON(RFX_G_EMA_ATTACK_PCT <= RFX_G_EMA_DECAY_PCT);
	BUILD_BUG_ON(RFX_THERMAL_WARM_ARM_DEPTH > RFX_THERMAL_HOT_ARM_DEPTH ||
		     RFX_THERMAL_HOT_ARM_DEPTH > RFX_THERMAL_CRIT_ARM_DEPTH);
	BUILD_BUG_ON(RFX_THERMAL_WARM_CLEAR_DEPTH >= RFX_THERMAL_WARM_ARM_DEPTH ||
		     RFX_THERMAL_HOT_CLEAR_DEPTH >= RFX_THERMAL_HOT_ARM_DEPTH ||
		     RFX_THERMAL_CRIT_CLEAR_DEPTH >= RFX_THERMAL_CRIT_ARM_DEPTH);
	BUILD_BUG_ON(RFX_THERMAL_WARM_CLEAR_DEPTH > RFX_THERMAL_HOT_CLEAR_DEPTH ||
		     RFX_THERMAL_HOT_CLEAR_DEPTH > RFX_THERMAL_CRIT_CLEAR_DEPTH);
	BUILD_BUG_ON(RFX_THERMAL_WARM_DWELL_NS >= RFX_THERMAL_HOT_DWELL_NS ||
		     RFX_THERMAL_HOT_DWELL_NS >= RFX_THERMAL_CRIT_DWELL_NS);
	/* Prime is throttled harder than Big at every thermal stage */
	BUILD_BUG_ON(RFX_THERMAL_WARM_CAP_PRIME_PCT >= RFX_THERMAL_WARM_CAP_BIG_PCT);
	BUILD_BUG_ON(RFX_THERMAL_HOT_CAP_PRIME_PCT >= RFX_THERMAL_HOT_CAP_BIG_PCT);
	BUILD_BUG_ON(RFX_THERMAL_CRIT_CAP_PRIME_PCT >= RFX_THERMAL_CRIT_CAP_BIG_PCT);
	/* Higher stages throttle harder: caps must strictly decrease warm>hot>crit */
	BUILD_BUG_ON(RFX_THERMAL_WARM_CAP_PRIME_PCT <= RFX_THERMAL_HOT_CAP_PRIME_PCT ||
		     RFX_THERMAL_HOT_CAP_PRIME_PCT <= RFX_THERMAL_CRIT_CAP_PRIME_PCT);
	BUILD_BUG_ON(RFX_THERMAL_WARM_CAP_BIG_PCT <= RFX_THERMAL_HOT_CAP_BIG_PCT ||
		     RFX_THERMAL_HOT_CAP_BIG_PCT <= RFX_THERMAL_CRIT_CAP_BIG_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_CAP_PCT > RFX_D_LITTLE_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_FLOOR_PCT > RFX_D_LITTLE_CAP_PCT);

	BUILD_BUG_ON(RFX_D_LITTLE_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_SWEET_CAP_BIG_PCT > 100 ||
		     RFX_D_SWEET_CAP_PRIME_PCT > 100);
	BUILD_BUG_ON(RFX_D_BURST_UNLOCK_PCT > 100);
	BUILD_BUG_ON(RFX_D_SUBLINEAR_KNEE_PCT >= RFX_D_BURST_UNLOCK_PCT ||
		     RFX_D_SUBLINEAR_KNEE_PCT < 1);
	BUILD_BUG_ON(RFX_D_PRIME_QUIET_PCT >= RFX_D_BURST_UNLOCK_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_QUIET_PCT > 100);
	BUILD_BUG_ON(RFX_D_PRIME_QUIET_KNEE_PCT > RFX_D_SWEET_CAP_PRIME_PCT);
	BUILD_BUG_ON(RFX_D_IDLE_FLOOR_SNAP_PCT >= RFX_D_PRIME_QUIET_PCT);
	BUILD_BUG_ON(RFX_D_IDLE_FLOOR_SNAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_TIME_DECAY_SNAP_UTIL_PCT <= RFX_D_IDLE_FLOOR_SNAP_PCT);
	BUILD_BUG_ON(RFX_D_TIME_DECAY_SNAP_UTIL_PCT > 100);
	BUILD_BUG_ON(RFX_D_TOUCH_FLOOR_MIN_DEMAND > 100);
	BUILD_BUG_ON(RFX_D_BIG_TOUCH_FLOOR_PCT > RFX_D_SWEET_CAP_BIG_PCT);
	BUILD_BUG_ON(RFX_D_IOWAIT_LITTLE_PCT > 100 ||
		     RFX_D_IOWAIT_BIG_PCT > RFX_D_IOWAIT_LITTLE_PCT ||
		     RFX_D_IOWAIT_PRIME_PCT > RFX_D_IOWAIT_BIG_PCT ||
		     RFX_D_IOWAIT_PRIME_PCT > 0);
	BUILD_BUG_ON(RFX_D_DOWN_US > RFX_D_TOUCH_DOWN_US ||
		     RFX_D_TOUCH_DOWN_US > 10000);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE > 1);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE_MIN_PCT > 100);

	BUILD_BUG_ON(RFX_D_PARK_EXIT_PCT * 16 <= 100);
	BUILD_BUG_ON(RFX_D_PARK_EXIT_EVALS < 1);
	BUILD_BUG_ON(RFX_LITTLE_CAP_THRESHOLD >= RFX_PRIME_CAP_THRESHOLD);
	BUILD_BUG_ON(RFX_EMA_DAILY_DIVISOR < 1 || RFX_EMA_MAX_STEPS < 1);
	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_ENTER_PCT);

	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_EXIT_PCT);

	BUILD_BUG_ON(!RFX_CEIL_RISE_PCT_PER_2MS);
	BUILD_BUG_ON(!RFX_CEIL_FALL_DWELL_NS || !RFX_CEIL_FALL_BYPASS_PCT);

	BUILD_BUG_ON(RFX_G_EVAL_US_DEFAULT < 1);
	BUILD_BUG_ON(RFX_G_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_GO_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_TOUCH_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_TOUCH_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_PRIME_TOUCH_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_TOUCH_PCT > RFX_G_PRIME_CAP_PCT);
	BUILD_BUG_ON(RFX_G_ENERGY_AWARE_DEFAULT > 1);
	BUILD_BUG_ON(RFX_G_BURST_UNLOCK_PCT > 100 || RFX_G_BURST_UNLOCK_PCT < 50);
	BUILD_BUG_ON(RFX_D_BIG_SUSTAINED_CAP_PCT > 100 ||
		     RFX_D_BIG_SUSTAINED_CAP_PCT < RFX_D_SWEET_CAP_BIG_PCT);

	/* Eco stage gating assertions */
	BUILD_BUG_ON(RFX_ECO_DEEP_INPUT_QUIET_NS <= RFX_ECO_INPUT_QUIET_NS);
	BUILD_BUG_ON(RFX_ECO_DEEP_UCMIN_PCT >= RFX_ECO_UCMIN_PCT);
	BUILD_BUG_ON(RFX_ECO_UCMIN_PCT >= RFX_ECO_EXIT_UCMIN_PCT);
	BUILD_BUG_ON(RFX_ECO_DEEP_DEMAND_PCT >= RFX_ECO_LITE_DEMAND_PCT);
	BUILD_BUG_ON(RFX_ECO_DEEP_LITTLE_CAP_PCT >= RFX_ECO_LITTLE_CAP_PCT);
	BUILD_BUG_ON(RFX_ECO_DEEP_BIG_CAP_PCT >= RFX_ECO_BIG_CAP_PCT);

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	rfx_init_topology();
	rfx_selfcheck();

	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret)
		return ret;

	register_pm_notifier(&rfx_pm_nb);

	return 0;
}

static void __exit vorpal_gov_exit(void)
{
	unregister_pm_notifier(&rfx_pm_nb);
	cpufreq_unregister_governor(&vorpal_gov);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.3");
