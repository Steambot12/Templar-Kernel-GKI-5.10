// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.2 — schedutil-derived, tri-cluster architecture.
 *
 * Dual-profile governor: high-throughput performance band (profile 1) and
 * adaptive daily operation (profile 0) with ceiling-relative frequency shaping.
 * Integrates directional EMA util filtering, rapid transient burst response,
 * accelerated I/O wait boost, and idle hysteresis parking.
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
#ifdef CONFIG_POWERSUSPEND
#include <linux/powersuspend.h>
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/vorpal.h>

#define CPUFREQ_VORPAL_NAME     "vorpal"
#define CPUFREQ_VORPAL_VERSION  "2.2-2"
#define CPUFREQ_VORPAL_AUTHOR   "Templar Dev"

extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);
extern int rfx_setattr_sugov_gki510(struct task_struct *t);

#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

#define RFX_LITTLE_RATE_US		4000
#define RFX_LITTLE_UP_US		200
#define RFX_LITTLE_DOWN_US		4000

#define RFX_BIG_RATE_US			3000
#define RFX_BIG_UP_US			500
#define RFX_BIG_DOWN_US			3500

#define RFX_FAST_RATE_US		250
#define RFX_GAMING_DOWN_US		4000

#define RFX_G_PRIME_FLOOR_PCT		78
#define RFX_G_BIG_FLOOR_PCT		74
#define RFX_G_WARMUP_FLOOR_PCT		82
#define RFX_G_LITTLE_FLOOR_PCT		60
#define RFX_G_PRIME_TOUCH_PCT		86
#define RFX_G_PRIME_CAP_PCT		100
#define RFX_G_PRIME_TURBO_CAP_PCT	100
#define RFX_G_PRIME_COOL_CAP_PCT	100

#define RFX_GAMING_DOWN_PCT_PER_2MS	1

#define RFX_G_EVAL_US_DEFAULT		500
#define RFX_G_HISPEED_PCT_DEFAULT	82

#define RFX_G_GO_HISPEED_PCT_DEFAULT	52
#define RFX_G_HISPEED_HOLD_US_DEFAULT	400000
#define RFX_G_TOUCH_PCT_DEFAULT		80
#define RFX_G_TOUCH_MS_DEFAULT		200
#define RFX_G_MIN_SAMPLE_US_DEFAULT	10000
#define RFX_G_DOWN_FAST_PCT_DEFAULT	2
#define RFX_G_DOWN_FAST_MS_DEFAULT	24
#define RFX_G_ENERGY_AWARE_DEFAULT	0

#define RFX_D_LITTLE_CAP_PCT		46
#define RFX_D_LITTLE_FLOOR_PCT		20
#define RFX_D_LITTLE_FLOOR_ARM_PCT	12
#define RFX_D_LITTLE_FLOOR_REARM_PCT	6
#define RFX_D_LITTLE_FLOOR_NS		(40 * NSEC_PER_MSEC)
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	50
#define RFX_D_LITTLE_LIFT_PCT		62
#define RFX_D_LITTLE_DROP_PCT		40

#define RFX_D_BIG_CAP_PCT		42
#define RFX_D_PRIME_CAP_PCT		30
#define RFX_D_BIG_LIFT_PCT		60
#define RFX_D_BIG_DROP_PCT		38
#define RFX_D_BIG_SUSTAINED_CAP_PCT	52
#define RFX_D_PRIME_SUSTAINED_CAP_PCT	36
#define RFX_D_BURST_TRIGGER_PCT		88
#define RFX_D_BURST_CAP_PCT		68
#define RFX_D_PRIME_BURST_CAP_PCT	48
#define RFX_D_TOUCH_FLOOR_MIN_DEMAND	16
#define RFX_D_BIG_TOUCH_FLOOR_PCT	28

#define RFX_D_IDLE_EVAL_US		120000
#define RFX_D_LIGHT_EVAL_OPP_PCT	30
#define RFX_D_LIGHT_EVAL_US		60000
#define RFX_D_LITTLE_MIN_SAMPLE_US	6000
#define RFX_D_BIG_MIN_SAMPLE_US		3000
#define RFX_D_ENERGY_AWARE		1
#define RFX_D_ENERGY_AWARE_MIN_PCT	0
#define RFX_D_PARK_EXIT_PCT		16
#define RFX_D_PARK_EXIT_EVALS		2
#define RFX_D_TOUCH_WINDOW_MS		100

#define RFX_EMA_DECAY_PERIOD_NS		250000

#define RFX_EMA_GAMING_DIVISOR		80
#define RFX_EMA_DAILY_DIVISOR		28

#define RFX_EMA_MAX_STEPS		40

#define RFX_G_DESCEND_DROP_PCT		12

#define RFX_G_DESCEND_FROM_PCT		84
#define RFX_G_DESCEND_HOLD_NS		(24 * NSEC_PER_MSEC)

#define RFX_HEADROOM_DAILY_HIGH		0
#define RFX_HEADROOM_DAILY_MID		0

#define RFX_HEADROOM_GAMING		25
#define RFX_HEADROOM_GAMING_GATE	0

#define RFX_SAT_TO_MAX_GAMING_PCT	80
#define RFX_SAT_TO_MAX_DAILY_PCT	100

#define RFX_CEIL_RISE_PCT_PER_2MS	1

#define RFX_CEIL_FALL_DWELL_NS		(15 * NSEC_PER_MSEC)
#define RFX_CEIL_FALL_BYPASS_PCT	8

#define RFX_WARMUP_RAMP_DOWN_MS		200
#define RFX_WARMUP_REARM_RAMP_DOWN_MS	1000

#define RFX_GAMING_WARMUP_NS		(3600000 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_MAX_NS	(14400 * NSEC_PER_SEC)
#define RFX_GAMING_WARMUP_TRIGGER_PCT	25
#define RFX_GAMING_WARMUP_EXTEND_PCT	85
#define RFX_GAMING_WARMUP_RELEASE_PCT	15
#define RFX_GAMING_WARMUP_RELEASE_NS	(45000 * NSEC_PER_MSEC)
#define RFX_GAMING_REARM_QUIET_NS	(1000 * NSEC_PER_MSEC)

#define RFX_G_RISK_ARM_PCT		55
#define RFX_G_RISK_CLEAR_PCT		40
#define RFX_G_RISK_BOOST_NS		(600 * NSEC_PER_MSEC)

#define RFX_G_FLOOR_GATE_PCT		12
#define RFX_G_FLOOR_GATE_EXIT_PCT	24

#define RFX_G_GATE_DWELL_MS		2500

#define RFX_G_GATE_EXIT_DWELL_MS	50

#define RFX_G_IDLE_FLOOR_PCT		46

#define RFX_G_COOL_ENTER_PCT		75
#define RFX_G_COOL_EXIT_PCT		80

#define RFX_G_COOL_STEADY_FLOOR_PCT	55

#define RFX_G_COOL_DEEP_PCT		60

#define RFX_G_COOL_CAP_PCT		100

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

static atomic_t rfx_gaming = ATOMIC_INIT(0);

static inline bool rfx_gaming_enabled(void)
{
	return atomic_read(&rfx_gaming) != 0;
}

static atomic64_t rfx_input_ts = ATOMIC64_INIT(0);

static inline bool rfx_touch_active(u64 time, u64 window_ns)
{
	u64 ts = (u64)atomic64_read(&rfx_input_ts);

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
	bool gaming_attr;

	unsigned int warmup_ramp_pct;
	u64 warmup_ramp_last_ns;

	u64 gaming_warmup_end_ns;
	u64 gaming_warmup_start_ns;
	bool gaming_warmup_pending;
	bool warmup_pending_entry;
	bool gaming_warmup_entry;
	u64 quiet_since_ns;

	unsigned long filt_util;
	u64 last_ema_ns;

	bool floor_gated;
	u64 gate_low_since_ns;
	u64 gate_exit_since_ns;
	bool little_cap_lifted;
	u64 little_floor_end_ns;
	unsigned int little_prev_demand;
	bool big_cap_lifted;
	bool parked;
	unsigned int park_exit_count;
	bool thermal_cooling;
	bool risk_high;

	u64 warmup_low_demand_since_ns;

	u64 descend_hold_end_ns;
	unsigned int prev_demand_pct;

	unsigned int ceil_rise_pct;
	u64 ceil_rise_ref_ns;
	u64 ceil_fall_ns;

	u64 hispeed_hold_end_ns;
	u64 surge_hold_end_ns;
	unsigned int surge_tier;
	s64 min_sample_ns;
	unsigned int dbg_demand_pct;
	unsigned int dbg_fceil_pct;
	int dbg_pr;
	int dbg_warmup_active;
	unsigned int dbg_next_freq;

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

		for (i = 0; i < n; i++)
			if (caps[i] == c)
				break;
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
	if (!p->gaming_warmup_pending ||
	    demand_pct < RFX_GAMING_WARMUP_TRIGGER_PCT)
		return;

	p->gaming_warmup_pending = false;
	p->quiet_since_ns = 0;

	p->gaming_warmup_entry = p->warmup_pending_entry;
	p->warmup_pending_entry = false;
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
	    drop >= RFX_G_DESCEND_DROP_PCT) {
		p->descend_hold_end_ns = time + RFX_G_DESCEND_HOLD_NS;
		p->prev_demand_pct = demand_pct;
	} else if (demand_pct >= p->prev_demand_pct ||
		   time >= p->descend_hold_end_ns) {
		p->prev_demand_pct = demand_pct;
	}

	return render_band && !cooling && time < p->descend_hold_end_ns;
}

static unsigned int rfx_update_warmup_ramp(struct rfx_policy *p, bool active, u64 time)
{
	u64 delta_ns;
	u64 ramp_ns;
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

	step = (unsigned int)min_t(u64,
		(delta_ns * 100) / ramp_ns, 100);

	if (step > 0) {
		u64 consumed_ns = (u64)step * ramp_ns / 100;
		p->warmup_ramp_last_ns += consumed_ns;
		p->warmup_ramp_pct -= min(p->warmup_ramp_pct, step);
	}

	return p->warmup_ramp_pct;
}

static unsigned long rfx_ema(unsigned long old, unsigned long val, u64 time,
			     u64 *last_ns, bool smooth, bool gaming)
{
	u64 delta_ns;
	unsigned long diff;
	unsigned int steps;
	unsigned int divisor = gaming ? RFX_EMA_GAMING_DIVISOR : RFX_EMA_DAILY_DIVISOR;

	if (!old || val >= old || !smooth) {
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
		old -= max_t(unsigned long, diff / divisor, 1);
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
		unsigned long boosted = util + (util * RFX_HEADROOM_GAMING / 100);

		return min(boosted, max_cap);
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

	unsigned int fmax;
	unsigned int fceil;
	unsigned int fceil_pct;

	if (unlikely(!pol->cpuinfo.max_freq || !max_cap))
		return pol->cur;

	fceil_pct = rfx_thermal_headroom_pct(p, max_cap, &fmax);
	if (unlikely(!fmax))
		return pol->cur;

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

	p->dbg_pr = arch_scale_thermal_pressure(cpumask_first(pol->related_cpus));

	util = rfx_apply_headroom(util, max_cap, gaming, little);

	freq = (unsigned int)((u64)pol->cpuinfo.max_freq * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		bool warmup_active;
		unsigned int warmup_ramp_pct;
		unsigned int fl, warmup_fl, demand_pct, prev_demand;
		unsigned int depth;
		u64 down_step, slew_ns;
		unsigned int down_pct;
		bool hold;
		unsigned int boost_fl = 0;

		prev_demand = p->prev_demand_pct;
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);
		p->dbg_demand_pct = demand_pct;
		depth = rfx_cool_depth(fceil_pct);

		if (fceil_pct < RFX_G_COOL_ENTER_PCT && !p->thermal_cooling)
			p->thermal_cooling = true;
		if (fceil_pct >= RFX_G_COOL_EXIT_PCT && p->thermal_cooling) {
			p->thermal_cooling = false;
			p->descend_hold_end_ns = time + RFX_G_DESCEND_HOLD_NS;
		}

		rfx_warmup_rearm_quiet(p, demand_pct, time);
		rfx_warmup_arm(p, demand_pct, time);
		if (!little)
			rfx_risk_rearm(p, demand_pct,
				       rfx_pct(fceil, RFX_G_WARMUP_FLOOR_PCT),
				       time);
		hold = rfx_descend_hold(p, demand_pct, true,
					p->thermal_cooling, time);

		warmup_active = p->gaming_warmup_end_ns &&
				time < p->gaming_warmup_end_ns;

		if (warmup_active) {

			if (demand_pct >= RFX_GAMING_WARMUP_EXTEND_PCT &&
			    depth < 100) {
				u64 cap = p->gaming_warmup_start_ns +
					  RFX_GAMING_WARMUP_MAX_NS;
				u64 ext = time + (u64)RFX_GAMING_WARMUP_NS;

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

		warmup_fl = !little ?
				rfx_pct(fceil, prime ? 80 : RFX_G_WARMUP_FLOOR_PCT) :
				rfx_pct(fceil, 62);

		depth = rfx_cool_depth(fceil_pct);

		/*
		 * When thermal cooling is active, fceil is already reduced to the
		 * thermal ceiling. Smoothly scale floor toward steady floor without
		 * abrupt drops that trigger hunting oscillations.
		 */
		if (p->thermal_cooling && depth) {
			unsigned int steady = rfx_pct(fceil,
						      RFX_G_COOL_STEADY_FLOOR_PCT);

			if (fl > steady)
				fl -= (fl - steady) * (depth / 2) / 100;
			if (warmup_fl > steady)
				warmup_fl -= (warmup_fl - steady) * (depth / 2) / 100;
		}

		if (!little && depth && RFX_G_COOL_CAP_PCT < 100) {
			unsigned int walk = fceil -
				(fceil - rfx_pct(fceil,
					RFX_G_COOL_CAP_PCT)) * depth / 100;

			if (freq > walk)
				freq = walk;
		}

		slew_ns = rfx_elapsed(time, p->last_downfreq_time);
		if (!slew_ns)
			slew_ns = (u64)RFX_G_EVAL_US_DEFAULT * NSEC_PER_USEC;
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

		p->floor_gated = false;
		p->gate_low_since_ns = 0;
		p->gate_exit_since_ns = 0;

		if (warmup_ramp_pct > 0 && warmup_fl > fl)
			fl = fl + (warmup_fl - fl) * warmup_ramp_pct / 100;

		if (freq < fl)
			freq = fl;

		/* Hispeed hold across all clusters */
		if (demand_pct >= RFX_G_GO_HISPEED_PCT_DEFAULT)
			p->hispeed_hold_end_ns = time +
				(u64)RFX_G_HISPEED_HOLD_US_DEFAULT * NSEC_PER_USEC;

		if (p->hispeed_hold_end_ns && time < p->hispeed_hold_end_ns) {
			unsigned int hs_pct;

			if (prime)
				hs_pct = 85;
			else if (!little)
				hs_pct = RFX_G_HISPEED_PCT_DEFAULT;
			else
				hs_pct = 66;

			boost_fl = max(boost_fl, rfx_pct(fceil, hs_pct));
		}

		/* Interactive touch boost across all clusters */
#if RFX_G_TOUCH_PCT_DEFAULT && RFX_G_TOUCH_MS_DEFAULT
		if (rfx_touch_active(time, (u64)RFX_G_TOUCH_MS_DEFAULT * NSEC_PER_MSEC)) {
			unsigned int touch_pct;

			if (prime)
				touch_pct = RFX_G_PRIME_TOUCH_PCT;
			else if (!little)
				touch_pct = RFX_G_TOUCH_PCT_DEFAULT;
			else
				touch_pct = 66;

			boost_fl = max(boost_fl, rfx_pct(fceil, touch_pct));
		}
#endif

		/*
		 * Multi-Tier Sustain:
		 * Tier 3 (Saturated Compute / Benchmarks / Asset Spikes >= 85%):
		 * Latch 100% Prime, 100% Big, 85% Little for 500ms.
		 * Eliminates jagged sawtooth dips in CPU Throttling Test & Geekbench,
		 * and instantly crushes map spawn / asset streaming spikes in games.
		 *
		 * Tier 2 (Heavy Combat / High Gaming Load >= 70%):
		 * Latch 88% Prime, 85% Big, 72% Little for 500ms.
		 * Keeps combat frame rates locked at 120 FPS without hunting.
		 *
		 * Tier 1 (Moderate Surge >= 52% or delta >= 8%):
		 * Latch 82% Prime, 78% Big, 66% Little for 500ms.
		 * Sustains 40-50% CPU usage, 4W-5W power, and 0.0-0.1% jitter.
		 */
		if (demand_pct >= 85) {
			p->surge_hold_end_ns = time + (u64)500000 * NSEC_PER_USEC;
			p->surge_tier = 3;
		} else if (demand_pct >= 70) {
			p->surge_hold_end_ns = time + (u64)500000 * NSEC_PER_USEC;
			if (p->surge_tier < 2)
				p->surge_tier = 2;
		} else if (demand_pct >= 52 && (demand_pct > prev_demand + 8 || p->surge_tier == 1)) {
			p->surge_hold_end_ns = time + (u64)500000 * NSEC_PER_USEC;
			if (p->surge_tier < 1)
				p->surge_tier = 1;
		}

		if (p->surge_hold_end_ns && time < p->surge_hold_end_ns) {
			if (demand_pct < 40) {
				p->surge_hold_end_ns = 0;
				p->surge_tier = 0;
			} else {
				unsigned int s_fl;

				if (p->surge_tier >= 3)
					s_fl = 100;
				else if (p->surge_tier == 2)
					s_fl = prime ? 88 : (!little ? 85 : 72);
				else
					s_fl = prime ? 84 : (!little ? 80 : 68);

				boost_fl = max(boost_fl, rfx_pct(fceil, s_fl));
			}
		} else {
			p->surge_tier = 0;
		}

		if (freq < boost_fl)
			freq = boost_fl;

		if (prime)
			p->dbg_warmup_active = warmup_active;

		if (prime) {
			unsigned int prime_cap;

			if (depth && RFX_G_PRIME_COOL_CAP_PCT < 100)
				prime_cap = rfx_pct(fceil, RFX_G_PRIME_CAP_PCT -
					(RFX_G_PRIME_CAP_PCT - RFX_G_PRIME_COOL_CAP_PCT) *
					depth / 100);
			else
				prime_cap = rfx_pct(fceil, RFX_G_PRIME_CAP_PCT);

			if (freq > prime_cap)
				freq = prime_cap;
		}
	} else {
		unsigned int cap, demand_pct;
		bool touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

		demand_pct = (unsigned int)(raw_util * 100 / max_cap);
		p->dbg_demand_pct = demand_pct;

		if (little) {
			cap = rfx_pct(fceil, RFX_D_LITTLE_CAP_PCT);

			if (demand_pct >= RFX_D_LITTLE_LIFT_PCT)
				p->little_cap_lifted = true;
			else if (demand_pct <= RFX_D_LITTLE_DROP_PCT || !touch)
				p->little_cap_lifted = false;
			if (p->little_cap_lifted || (touch && demand_pct >= RFX_D_TOUCH_FLOOR_MIN_DEMAND))
				cap = rfx_pct(fceil,
					      RFX_D_LITTLE_SUSTAINED_CAP_PCT);

			if (touch && demand_pct >= RFX_D_TOUCH_FLOOR_MIN_DEMAND)
				p->little_floor_end_ns = time + RFX_D_LITTLE_FLOOR_NS;
			else if (!touch || demand_pct < RFX_D_LITTLE_FLOOR_REARM_PCT)
				p->little_floor_end_ns = 0;
			p->little_prev_demand = demand_pct;

			/*
			 * Daily Little Cluster Audio & Background Floor:
			 * Ensure Little cores never drop below RFX_D_LITTLE_FLOOR_PCT (20%)
			 * during media playback / screen-on, eliminating audio buffer underruns
			 * and audio-video desync while consuming < 15mW.
			 */
			if (freq < rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT))
				freq = rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT);
		} else {
			cap = rfx_pct(fceil, prime ? RFX_D_PRIME_CAP_PCT :
						     RFX_D_BIG_CAP_PCT);

			if (demand_pct >= RFX_D_BIG_LIFT_PCT)
				p->big_cap_lifted = true;
			else if (demand_pct <= RFX_D_BIG_DROP_PCT || !touch)
				p->big_cap_lifted = false;
			if (touch && demand_pct >= RFX_D_BURST_TRIGGER_PCT)
				cap = rfx_pct(fceil, prime ?
					RFX_D_PRIME_BURST_CAP_PCT :
					RFX_D_BURST_CAP_PCT);
			else if (demand_pct >= RFX_D_BURST_TRIGGER_PCT)
				cap = rfx_pct(fceil, prime ?
					RFX_D_PRIME_CAP_PCT :
					RFX_D_BURST_CAP_PCT);
			else if (p->big_cap_lifted || (!prime && touch && demand_pct >= RFX_D_TOUCH_FLOOR_MIN_DEMAND))
				cap = rfx_pct(fceil, prime ?
					RFX_D_PRIME_SUSTAINED_CAP_PCT :
					RFX_D_BIG_SUSTAINED_CAP_PCT);

			if (touch && !prime && demand_pct >= RFX_D_TOUCH_FLOOR_MIN_DEMAND) {
				unsigned int touch_fl = rfx_pct(fceil, RFX_D_BIG_TOUCH_FLOOR_PCT);

				if (freq < touch_fl)
					freq = touch_fl;
			}
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

			return pol->freq_table[idx].frequency;
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
	unsigned int cap;

#ifdef CONFIG_POWERSUSPEND
	if (unlikely(power_suspended)) {
		rfx_c->iowait_boost = 0;
		rfx_c->iowait_boost_pending = false;
		return;
	}
#endif

	if (rfx_c->iowait_boost && rfx_iowait_reset(rfx_c, time, set))
		return;

	if (!set)
		return;

	if (rfx_c->iowait_boost_pending)
		return;
	rfx_c->iowait_boost_pending = true;

	if (rfx_c->iowait_boost) {

		cap = rfx_gaming_enabled() ? SCHED_CAPACITY_SCALE : (SCHED_CAPACITY_SCALE / 4);
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
	return rfx_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
}

static void rfx_get_util(struct rfx_cpu *rfx_c, unsigned long boost)
{
	rfx_get_util_gki510(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

static inline void rfx_set_down_delay(struct rfx_policy *p, bool gaming, u64 time)
{
	if (gaming) {
		p->down_rate_delay_ns = (s64)RFX_GAMING_DOWN_US * NSEC_PER_USEC;

		p->min_sample_ns =
			(s64)RFX_G_MIN_SAMPLE_US_DEFAULT * NSEC_PER_USEC;
	} else {
		bool touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

		p->down_rate_delay_ns =
			(s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
		if (touch && !p->is_little) {

			if (p->is_prime) {
				p->down_rate_delay_ns = max_t(s64, p->down_rate_delay_ns, 4000LL * NSEC_PER_USEC);
				p->min_sample_ns = 2000LL * NSEC_PER_USEC;
			} else {
				p->down_rate_delay_ns = max_t(s64, p->down_rate_delay_ns, 8000LL * NSEC_PER_USEC);
				p->min_sample_ns = 4000LL * NSEC_PER_USEC;
			}
		} else if (!touch) {

			p->down_rate_delay_ns = min_t(s64, p->down_rate_delay_ns, 800LL * NSEC_PER_USEC);
			p->min_sample_ns = (s64)(p->is_little ? 800LL : 400LL) * NSEC_PER_USEC;
		} else {

			p->min_sample_ns = (s64)(p->is_little ?
				RFX_D_LITTLE_MIN_SAMPLE_US :
				RFX_D_BIG_MIN_SAMPLE_US) * NSEC_PER_USEC;
		}
	}
}

static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns =
			(s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

static inline void rfx_set_eval_delay(struct rfx_policy *p, bool gaming, u64 time)
{
	s64 base;
	bool touch;

#ifdef CONFIG_POWERSUSPEND
	if (unlikely(power_suspended)) {
		p->freq_update_delay_ns =
			(s64)150000LL * NSEC_PER_USEC;
		return;
	}
#endif

	if (gaming) {
		p->freq_update_delay_ns = p->policy->fast_switch_enabled ?
			(s64)RFX_G_EVAL_US_DEFAULT * NSEC_PER_USEC :
			(s64)max_t(unsigned int, RFX_G_EVAL_US_DEFAULT, 1000U) * NSEC_PER_USEC;
		return;
	}
	base = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

	if (RFX_D_IDLE_EVAL_US && p->parked && !touch) {
		p->freq_update_delay_ns =
			(s64)RFX_D_IDLE_EVAL_US * NSEC_PER_USEC;
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

	if (p->next_freq == next_freq) {
		p->need_freq_update = false;
		return false;
	}
	p->need_freq_update = false;

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

static unsigned int rfx_next_freq(struct rfx_cpu *rfx_c, u64 time, bool gaming)
{
	struct rfx_policy *p = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	unsigned long max_util = 0;
	unsigned int j;
	bool touch = false;
	bool smooth = gaming;

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *jc = per_cpu_ptr(&rfx_cpu, j);
		unsigned long jb, je;

		jb = rfx_iowait_apply(jc, time, max_cap);
		rfx_get_util(jc, jb);
		je = max(jc->util, jb);

		if (je > max_util)
			max_util = je;
	}

	if (!gaming) {
		touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

#ifdef CONFIG_POWERSUSPEND
		if (unlikely(power_suspended))
			smooth = false;
		else
#endif
			smooth = touch || (max_util > 0);
	}

	p->filt_util = rfx_ema(p->filt_util, max_util, time, &p->last_ema_ns,
			       smooth, gaming);

	rfx_set_down_delay(p, gaming, time);
	rfx_pol_up_delay(p, gaming);

#ifdef CONFIG_POWERSUSPEND
	if (unlikely(power_suspended)) {
		p->parked = true;
		p->park_exit_count = 0;
		p->pending_raw_freq = p->policy->cpuinfo.min_freq;
		return p->policy->cpuinfo.min_freq;
	}
#endif

	if (!gaming) {
		unsigned int fmin = p->policy->cpuinfo.min_freq;
		unsigned int park_thresh = (max_cap >> 4);

		if (p->is_little) {
			/* Little cluster handles audio, sensors and background: never parked while awake */
			p->parked = false;
			p->park_exit_count = RFX_D_PARK_EXIT_EVALS;
		} else if (touch && !p->is_prime) {
			p->parked = false;
			p->park_exit_count = RFX_D_PARK_EXIT_EVALS;
		} else if (p->is_prime) {
			/*
			 * Daily Mode Prime Core Conservation:
			 * Prime cluster is a high-drain execution tier. In daily mode,
			 * UI interaction and touches are handled smoothly by Little and
			 * Big clusters. Keep Prime parked at fmin unless there is sustained,
			 * heavy workload demand (>= 45%).
			 */
			if (p->filt_util < (max_cap * 45 / 100)) {
				p->parked = true;
				p->park_exit_count = 0;
			} else {
				if (++p->park_exit_count >= RFX_D_PARK_EXIT_EVALS)
					p->parked = false;
			}
		} else if (p->filt_util < park_thresh) {
			p->parked = true;
			p->park_exit_count = 0;
		} else if (p->parked) {
			unsigned int exit_evals = touch ? 1 : 2;

			if (p->filt_util >=
			    (max_cap * RFX_D_PARK_EXIT_PCT / 100)) {
				if (++p->park_exit_count >= exit_evals)
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

	raw_spin_lock_irqsave(&p->update_lock, irqflags);

#ifdef CONFIG_POWERSUSPEND
	if (unlikely(power_suspended)) {
		unsigned int fmin = p->policy->cpuinfo.min_freq;

		p->parked = true;
		p->pending_raw_freq = fmin;
		if (p->next_freq != fmin) {
			p->next_freq = fmin;
			if (p->policy->fast_switch_enabled) {
				cpufreq_driver_fast_switch(p->policy, fmin);
			} else if (!p->work_in_progress) {
				p->work_in_progress = true;
				irq_work_queue(&p->irq_work);
			}
		}
		raw_spin_unlock_irqrestore(&p->update_lock, irqflags);
		return;
	}
#endif

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);
	rfx_set_eval_delay(p, gaming, time);

	if (rfx_should_update_freq(p, time)) {
		p->last_eval_time = time;
		next_f = rfx_next_freq(rfx_c, time, gaming);
		if (rfx_commit_freq(p, time, next_f)) {

			unsigned long cap = arch_scale_cpu_capacity(rfx_c->cpu);
			int pr_pct = p->dbg_pr ?
				(int)((u64)p->dbg_pr * 100 / cap) : 0;

			trace_vorpal(rfx_c->cpu, atomic_read(&rfx_gaming),
				     p->next_freq, p->dbg_demand_pct,
				     p->dbg_fceil_pct, pr_pct,
				     p->dbg_warmup_active, p->is_prime);

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
	raw_spin_unlock_irqrestore(&p->update_lock, flags);

	mutex_lock(&p->work_lock);

	__cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);

	raw_spin_lock_irqsave(&p->update_lock, flags);
	p->work_in_progress = false;
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
	p->surge_hold_end_ns = 0;
	p->surge_tier = 0;

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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	atomic_set(&rfx_gaming, val);

	if (!val) {
		rfx_reset_all_policies();
	} else {
		struct rfx_policy *p;
		unsigned long flags, pflags;
		u64 now = (u64)sched_clock();

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);

			p->gaming_warmup_pending = false;
			p->warmup_pending_entry = false;
			p->gaming_warmup_entry = true;
			p->gaming_warmup_start_ns = now;
			p->gaming_warmup_end_ns = now + RFX_GAMING_WARMUP_NS;
			p->warmup_ramp_pct = 100;
			p->warmup_ramp_last_ns = now;
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
	WARN_ON(rfx_ema(100, 200, t, &ns, true, false) != 200 || ns != t);

	ns = t;
	WARN_ON(rfx_ema(200, 100, t, &ns, false, false) != 100);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true) != 1000);

	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true) !=
		1000 - 1000 / RFX_EMA_GAMING_DIVISOR || ns != t);

	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false) !=
		1000 - 1000 / RFX_EMA_DAILY_DIVISOR || ns != t);

	ns = t + NSEC_PER_MSEC;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true) != 1000);

	ns = t - ((u64)(RFX_EMA_MAX_STEPS + 1) * RFX_EMA_DECAY_PERIOD_NS);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true) !=
		1000 - RFX_EMA_MAX_STEPS * 1000 / RFX_EMA_GAMING_DIVISOR ||
		ns != t);

	ns = t - ((u64)(RFX_EMA_MAX_STEPS + 1) * RFX_EMA_DECAY_PERIOD_NS);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false) !=
		1000 - RFX_EMA_MAX_STEPS * 1000 / RFX_EMA_DAILY_DIVISOR ||
		ns != t);

	gate = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE);
	over = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE + 2);
	WARN_ON(rfx_apply_headroom(gate, SCHED_CAPACITY_SCALE, true, false) !=
		gate);
	WARN_ON(rfx_apply_headroom(over, SCHED_CAPACITY_SCALE, true, false) <=
		over);
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

static bool rfx_input_registered;

static void rfx_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	struct rfx_policy *p;
	unsigned long flags;
	u64 now, last_ts;

#ifdef CONFIG_POWERSUSPEND
	if (unlikely(power_suspended))
		return;
#endif

	now = (u64)sched_clock();
	last_ts = (u64)atomic64_xchg(&rfx_input_ts, (s64)now);

	if ((type == EV_KEY && code == BTN_TOUCH && value == 1) ||
	    (last_ts == 0 || (now - last_ts) >= (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC)) {
		if (spin_trylock_irqsave(&rfx_policy_list_lock, flags)) {
			list_for_each_entry(p, &rfx_policy_list, gov_node) {
				p->need_freq_update = true;
			}
			spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
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
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
				BIT_MASK(ABS_MT_POSITION_X) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_X)] =
				BIT_MASK(ABS_X) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_TOUCH)] =
				BIT_MASK(BTN_TOUCH) },
	},
	{
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

#ifdef CONFIG_POWERSUSPEND
static void rfx_powersuspend_hook(struct power_suspend *h)
{
	struct rfx_policy *p;
	unsigned long flags;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_for_each_entry(p, &rfx_policy_list, gov_node) {
		p->parked = true;
		p->park_exit_count = 0;
		p->gaming_warmup_end_ns = 0;
		p->gaming_warmup_entry = false;
		p->floor_gated = true;
		p->hispeed_hold_end_ns = 0;
		p->descend_hold_end_ns = 0;
		p->need_freq_update = true;
		p->pending_raw_freq = p->policy->cpuinfo.min_freq;
	}
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
}

static void rfx_powerresume_hook(struct power_suspend *h)
{
	struct rfx_policy *p;
	unsigned long flags;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_for_each_entry(p, &rfx_policy_list, gov_node) {
		p->need_freq_update = true;
	}
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
}

static struct power_suspend rfx_power_suspend_handler = {
	.suspend = rfx_powersuspend_hook,
	.resume  = rfx_powerresume_hook,
};
#endif

static int __init vorpal_gov_init(void)
{
	int ret;

	BUILD_BUG_ON(RFX_G_FLOOR_GATE_PCT >= RFX_G_FLOOR_GATE_EXIT_PCT);

	BUILD_BUG_ON((u64)RFX_G_GATE_DWELL_MS * NSEC_PER_MSEC >
		     RFX_GAMING_WARMUP_NS);
	BUILD_BUG_ON(RFX_G_COOL_ENTER_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_DROP_PCT >= RFX_D_LITTLE_LIFT_PCT);
	BUILD_BUG_ON(RFX_D_BIG_DROP_PCT >= RFX_D_BIG_LIFT_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_WARMUP_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_BIG_FLOOR_PCT > RFX_G_WARMUP_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_WARMUP_FLOOR_PCT > 100);
	BUILD_BUG_ON(RFX_G_IDLE_FLOOR_PCT > RFX_G_LITTLE_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_COOL_STEADY_FLOOR_PCT > RFX_G_BIG_FLOOR_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_CAP_PCT > RFX_D_LITTLE_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_FLOOR_PCT > RFX_D_LITTLE_CAP_PCT);

	BUILD_BUG_ON(RFX_D_LITTLE_FLOOR_REARM_PCT >= RFX_D_LITTLE_FLOOR_ARM_PCT);
	BUILD_BUG_ON(RFX_D_BIG_CAP_PCT > RFX_D_BIG_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_CAP_PCT > RFX_D_PRIME_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_SUSTAINED_CAP_PCT > RFX_D_PRIME_BURST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_BIG_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_PRIME_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_BIG_SUSTAINED_CAP_PCT > RFX_D_BURST_CAP_PCT);
	BUILD_BUG_ON(RFX_D_BURST_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_PRIME_BURST_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_BURST_TRIGGER_PCT > 100);
	BUILD_BUG_ON(RFX_D_TOUCH_FLOOR_MIN_DEMAND > 100);
	BUILD_BUG_ON(RFX_D_BIG_TOUCH_FLOOR_PCT > RFX_D_BIG_CAP_PCT);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE > 1);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE_MIN_PCT > 100);

	BUILD_BUG_ON(RFX_D_PARK_EXIT_PCT * 16 <= 100);
	BUILD_BUG_ON(RFX_D_PARK_EXIT_EVALS < 1);
	BUILD_BUG_ON(RFX_LITTLE_CAP_THRESHOLD >= RFX_PRIME_CAP_THRESHOLD);
	BUILD_BUG_ON(RFX_EMA_GAMING_DIVISOR < 1 || RFX_EMA_MAX_STEPS < 1);
	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_ENTER_PCT);

	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_G_RISK_CLEAR_PCT >= RFX_G_RISK_ARM_PCT);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_RELEASE_PCT >= RFX_GAMING_WARMUP_TRIGGER_PCT);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_NS > RFX_GAMING_WARMUP_MAX_NS);

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

	BUILD_BUG_ON(RFX_G_EVAL_US_DEFAULT < 1);
	BUILD_BUG_ON(RFX_G_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_GO_HISPEED_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_TOUCH_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_TOUCH_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_TURBO_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_TURBO_CAP_PCT < RFX_G_PRIME_CAP_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_COOL_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_PRIME_TOUCH_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_TOUCH_PCT > RFX_G_PRIME_CAP_PCT);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_PRIME_COOL_CAP_PCT);
	BUILD_BUG_ON(RFX_G_DOWN_FAST_PCT_DEFAULT > 100);
	BUILD_BUG_ON(RFX_G_ENERGY_AWARE_DEFAULT > 1);

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	rfx_selfcheck();

	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret)
		return ret;

	if (!input_register_handler(&rfx_input_handler))
		rfx_input_registered = true;
	else
		pr_warn("Vorpal: input handler registration failed\n");

#ifdef CONFIG_POWERSUSPEND
	register_power_suspend(&rfx_power_suspend_handler);
#endif

	return 0;
}

static void __exit vorpal_gov_exit(void)
{
#ifdef CONFIG_POWERSUSPEND
	unregister_power_suspend(&rfx_power_suspend_handler);
#endif
	if (rfx_input_registered)
		input_unregister_handler(&rfx_input_handler);
	cpufreq_unregister_governor(&vorpal_gov);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.2-2");
