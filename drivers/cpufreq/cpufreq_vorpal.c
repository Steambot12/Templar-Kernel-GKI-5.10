// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.3 — schedutil-derived, tri-cluster architecture.
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

#define CREATE_TRACE_POINTS
#include <trace/events/vorpal.h>

#define CPUFREQ_VORPAL_NAME     "vorpal"
#define CPUFREQ_VORPAL_VERSION  "2.3"
#define CPUFREQ_VORPAL_AUTHOR   "Templar Dev"

extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern unsigned long rfx_get_uclamp_min_gki510(int cpu);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);
extern int rfx_setattr_sugov_gki510(struct task_struct *t);

#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

#define RFX_LITTLE_RATE_US		4000
#define RFX_LITTLE_UP_US		1200
#define RFX_LITTLE_DOWN_US		4000

#define RFX_BIG_RATE_US			3000
#define RFX_BIG_UP_US			1500
#define RFX_BIG_DOWN_US			3500

/* Gaming-mode rate-limit overrides for Big/Prime: keep upward bursts
 * latency-free, but hold the downward transition across most of a
 * 90/120Hz frame window so a single low-util sample cannot oscillate
 * the frequency.
 */
#define RFX_G_GAMING_UP_RATE_US		1000
#define RFX_GAMING_DOWN_OVERRIDE_US	16000

/* Asymmetric EMA filter for gaming mode: a raw sample above the filtered
 * value attacks fast (alpha 72/28), a sample at or below decays slower
 * (alpha 22/78). This keeps inter-frame idle gaps from collapsing the
 * Prime/Big utilisation while allowing prompt recovery to efficient OPPs.
 */
#define RFX_G_EMA_ATTACK_PCT		72
#define RFX_G_EMA_DECAY_PCT		22

#define RFX_G_PRIME_FLOOR_PCT		42
#define RFX_G_BIG_FLOOR_PCT		44
#define RFX_G_LITTLE_FLOOR_PCT		32
#define RFX_G_LITTLE_CAP_PCT		72

/* Soft ceiling: transient spikes in gaming mode hold the cluster at this
 * percentage of the thermal ceiling instead of jumping to the top OPP.
 * 100% is released only after sustained saturation (see sat_count).
 */
#define RFX_G_SOFT_CEIL_PRIME_PCT	74
#define RFX_G_SOFT_CEIL_BIG_PCT		76

/* Saturation gate: demand windows held at/above these counts release the
 * top OPP; below the release count the counter decays to zero.
 */
#define RFX_G_SAT_ENTER_PCT		92
#define RFX_G_SAT_RAW_PCT		94
#define RFX_G_SAT_UNLOCK_COUNT		6
#define RFX_G_SAT_COUNT_MAX		10

/* Anti-sawtooth step-down cap: after a big/prime frequency drop in gaming
 * mode, the governor descends at most this percentage of max frequency
 * per evaluation, so a brief thread migration cannot cliff the cluster
 * from the top OPP back to the floor in one tick. A short descent hold
 * latches across evaluation periods so the drop steps down steadily
 * instead of oscillating.
 */
#define RFX_G_STEP_DOWN_PCT		5
#define RFX_G_STEP_DOWN_HOLD_NS		(36 * NSEC_PER_MSEC)

/* Proactive hysteretic thermal dampening in gaming mode. The stage input
 * is the rfx_cool_depth thermal-pressure depth (0-100) of the
 * cluster's frequency ceiling -- no polling, no new thermal-zone
 * plumbing. Stages arm early and release slowly (per-stage dwell with
 * hysteresis) so sustained gaming settles on a lower OPP long before
 * the SoC runs hot; pressure that recovers after a frequency drop
 * cannot re-escalate a released stage within its dwell window.
 */
#define RFX_THERMAL_WARM_ARM_DEPTH	10
#define RFX_THERMAL_WARM_CLEAR_DEPTH	5
#define RFX_THERMAL_WARM_DWELL_NS	((u64)2000 * NSEC_PER_MSEC)

#define RFX_THERMAL_HOT_ARM_DEPTH	25
#define RFX_THERMAL_HOT_CLEAR_DEPTH	16
#define RFX_THERMAL_HOT_DWELL_NS	((u64)3000 * NSEC_PER_MSEC)

#define RFX_THERMAL_CRIT_ARM_DEPTH	45
#define RFX_THERMAL_CRIT_CLEAR_DEPTH	30
#define RFX_THERMAL_CRIT_DWELL_NS	((u64)4000 * NSEC_PER_MSEC)

#define RFX_THERMAL_WARM_CAP_PRIME_PCT	70
#define RFX_THERMAL_WARM_CAP_BIG_PCT	74
#define RFX_THERMAL_HOT_CAP_PRIME_PCT	64
#define RFX_THERMAL_HOT_CAP_BIG_PCT	68
#define RFX_THERMAL_CRIT_CAP_PRIME_PCT	58
#define RFX_THERMAL_CRIT_CAP_BIG_PCT	62

/* Gaming floor-band hysteresis (demand pct of capacity). The floor is
 * armed when smoothed demand crosses ARM; it only disarms when demand
 * stays below CLEAR, so a single low tick cannot release the band.
 */
#define RFX_G_PRIME_GATE_ARM_PCT	16
#define RFX_G_PRIME_GATE_CLEAR_PCT	8
#define RFX_G_BIG_GATE_ARM_PCT		24
#define RFX_G_BIG_GATE_CLEAR_PCT	14
#define RFX_G_PRIME_GATE_HOLD_NS	(1000 * NSEC_PER_MSEC)
#define RFX_G_BIG_GATE_HOLD_NS		(1000 * NSEC_PER_MSEC)

/* Warmup/entry ramps for gaming mode */
#define RFX_G_WARMUP_FLOOR_PCT		48
#define RFX_G_TOUCH_PCT_DEFAULT		58
#define RFX_G_TOUCH_MS_DEFAULT		100
#define RFX_G_MIN_SAMPLE_US_DEFAULT	8333
#define RFX_G_DOWN_FAST_PCT_DEFAULT	2
#define RFX_G_DOWN_FAST_MS_DEFAULT	24
#define RFX_G_ENERGY_AWARE_DEFAULT	0
#define RFX_G_IDLE_FLOOR_PCT		25

#define RFX_G_PRIME_TOUCH_PCT		56
#define RFX_G_PRIME_CAP_PCT		100
#define RFX_G_PRIME_TURBO_CAP_PCT	100
#define RFX_G_PRIME_COOL_CAP_PCT	100

#define RFX_G_SURGE_UTIL_PCT		86
#define RFX_G_FRAME_HOLD_NS		(24000ULL * NSEC_PER_USEC)
#define RFX_G_DOWN_RATE_LIMIT_NS	(16000ULL * NSEC_PER_USEC)
#define RFX_G_MIN_SAMPLE_NS		(8333ULL * NSEC_PER_USEC)

#define RFX_G_EVAL_US_DEFAULT		500
#define RFX_G_HISPEED_PCT_DEFAULT	70

#define RFX_G_GO_HISPEED_PCT_DEFAULT	74
#define RFX_G_HISPEED_HOLD_US_DEFAULT	40000

#define RFX_D_LITTLE_CAP_PCT		35
#define RFX_D_LITTLE_FLOOR_PCT		20
#define RFX_D_LITTLE_FLOOR_ARM_PCT	12
#define RFX_D_LITTLE_FLOOR_REARM_PCT	6
#define RFX_D_LITTLE_FLOOR_NS		(16 * NSEC_PER_MSEC)
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	38
#define RFX_D_LITTLE_LIFT_PCT		62
#define RFX_D_LITTLE_DROP_PCT		40

#define RFX_D_BIG_LIFT_PCT		60
#define RFX_D_BIG_DROP_PCT		38
#define RFX_D_TOUCH_FLOOR_MIN_DEMAND	16
#define RFX_D_BIG_TOUCH_FLOOR_PCT	16

/*
 * Daily Big/Prime efficiency shaping. Under normal baseline load the
 * target holds at the linear V/F knee (50% of fmax Big, 42% of fmax
 * Prime). Raw demand >= 92% (app cold launch, camera burst) unlocks the
 * top OPP so launches stay snappy.
 */
#define RFX_D_SWEET_CAP_BIG_PCT		50
#define RFX_D_SWEET_CAP_PRIME_PCT	42
#define RFX_D_BURST_UNLOCK_PCT		92

/* Prime quiet gate: light Prime demand without a high uclamp_min holds
 * Prime at the efficient knee; baseline UI scrolling is served by
 * Little/Big. */
#define RFX_D_PRIME_QUIET_PCT		50
#define RFX_D_PRIME_QUIET_KNEE_PCT	20

/* Sub-linear baseline light-load mapping for Big/Prime: below this
 * filtered-demand knee the target rises quadratically (continuous with
 * the proportional mapping at the knee) so light work stays in the
 * low-voltage OPP region. */
#define RFX_D_SUBLINEAR_KNEE_PCT	50

/* Idle floor snap: filtered demand under this (no touch) parks the
 * cluster at policy->min so the deepest C-states are reachable. */
#define RFX_D_IDLE_FLOOR_SNAP_PCT	10

/* Time-delta zero-snap: after a quiet gap longer than this with raw
 * demand still under the snap util threshold, collapse the filtered
 * value to the newest sample so a brief wake/idle cycle cannot hold
 * the cluster above its floor. */
#define RFX_D_TIME_DECAY_SNAP_NS		(8 * NSEC_PER_MSEC)
#define RFX_D_TIME_DECAY_SNAP_UTIL_PCT	12

/* Crisp baseline down-shed: 1.5-2ms downward transitions so bursts shed
 * quickly while keeping deep-C residency high. Upward stays 1200-1500us. */
#define RFX_D_DOWN_US			1500
#define RFX_D_TOUCH_DOWN_US		2000
#define RFX_D_UP_MAX_US			1800

/* Baseline iowait boost caps per tier, in % of SCHED_CAPACITY_SCALE.
 * Background SQLite/storage I/O must not drag Big/Prime into
 * high-voltage OPPs: Little gets a soft boost, Big a small one, Prime
 * none. */
#define RFX_D_IOWAIT_LITTLE_PCT	25
#define RFX_D_IOWAIT_BIG_PCT	10
#define RFX_D_IOWAIT_PRIME_PCT	0

#define RFX_D_IDLE_EVAL_US		150000
#define RFX_D_LIGHT_EVAL_OPP_PCT	30
#define RFX_D_LIGHT_EVAL_US		60000
#define RFX_D_LITTLE_MIN_SAMPLE_US	6000
#define RFX_D_BIG_MIN_SAMPLE_US		3000
#define RFX_D_ENERGY_AWARE		1
#define RFX_D_ENERGY_AWARE_MIN_PCT	0
#define RFX_D_PARK_EXIT_PCT		16
#define RFX_D_PARK_EXIT_EVALS		2
#define RFX_D_TOUCH_WINDOW_MS		64

#define RFX_EMA_DECAY_PERIOD_NS		250000

#define RFX_EMA_DAILY_DIVISOR		16

#define RFX_EMA_MAX_STEPS		40

#define RFX_G_DESCEND_DROP_PCT		5

#define RFX_G_DESCEND_FROM_PCT		74
#define RFX_G_DESCEND_HOLD_NS		(36 * NSEC_PER_MSEC)

#define RFX_HEADROOM_DAILY_HIGH		0
#define RFX_HEADROOM_DAILY_MID		0

#define RFX_HEADROOM_GAMING		6
#define RFX_HEADROOM_GAMING_GATE	0

#define RFX_SAT_TO_MAX_GAMING_PCT	95
#define RFX_SAT_TO_MAX_DAILY_PCT	100

#define RFX_CEIL_RISE_PCT_PER_2MS	1

#define RFX_CEIL_FALL_DWELL_NS		(15 * NSEC_PER_MSEC)
#define RFX_CEIL_FALL_BYPASS_PCT	8

#define RFX_WARMUP_RAMP_DOWN_MS		200
#define RFX_WARMUP_REARM_RAMP_DOWN_MS	1000

#define RFX_GAMING_WARMUP_NS		(15000 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_TRIGGER_PCT	25
#define RFX_GAMING_REARM_QUIET_NS	(1000 * NSEC_PER_MSEC)

#define RFX_G_RISK_ARM_PCT		55
#define RFX_G_RISK_CLEAR_PCT		40
#define RFX_G_RISK_BOOST_NS		(600 * NSEC_PER_MSEC)

#define RFX_G_FLOOR_GATE_PCT		12
#define RFX_G_FLOOR_GATE_EXIT_PCT	24

#define RFX_G_GATE_DWELL_MS		2500

#define RFX_G_GATE_EXIT_DWELL_MS	50

#define RFX_G_COOL_ENTER_PCT		65
#define RFX_G_COOL_EXIT_PCT		72

#define RFX_G_COOL_STEADY_FLOOR_PCT	50

#define RFX_G_COOL_DEEP_PCT		60

#define RFX_G_COOL_CAP_PCT		100

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/* Daily universal quiescent eco state (in-governor replacement for the
 * out-of-tree powersuspend driver). Armed purely from native signals --
 * input inactivity, uclamp_min and cluster demand -- so screen-off
 * quiescence and deep-idle residency work on any mainline GKI 5.10
 * tree. Strictly daily: never evaluated while gaming_mode is 1, so a
 * game that idles between touches or lingers on a killcam keeps the
 * full performance band.
 *
 * Stage 1 (light): after 2.5s of no touch with low demand, cap the
 * Little/Big tiers at their low-voltage OPPs (38% Little, 40% Big) and hold
 * Prime at policy->min.
 * Stage 2 (deep): after 10s, park Big/Prime at policy->min, cap
 * Little at 30%, and lengthen the evaluation period so
 * background wakelocks settle into the deepest C-states.
 *
 * Instant exit: the eco latches clear on the next evaluation the
 * moment any touch input lands (rfx_input_ts), uclamp_min crosses the
 * exit threshold, or gaming_mode is written 1, so UI response and game
 * entry are lag-free.
 */
#define RFX_ECO_INPUT_QUIET_NS		((u64)2000 * NSEC_PER_MSEC)
#define RFX_ECO_DEEP_INPUT_QUIET_NS	((u64)8000 * NSEC_PER_MSEC)
#define RFX_ECO_LITTLE_CAP_PCT		35
#define RFX_ECO_BIG_CAP_PCT		38
#define RFX_ECO_DEEP_LITTLE_CAP_PCT	25
#define RFX_ECO_DEEP_BIG_CAP_PCT	16
/* Arm requires the max uclamp_min across the policy to sit under these
 * percents of capacity: a foreground RenderThread/video request keeps the
 * eco state disarmed. */
#define RFX_ECO_UCMIN_PCT		25
#define RFX_ECO_DEEP_UCMIN_PCT		12
/* Any uclamp_min at/over this percent of capacity wakes eco instantly. */
#define RFX_ECO_EXIT_UCMIN_PCT		35
/* Filtered-demand residency gates: stage 1 under 32% of capacity,
 * stage 2 under 10%. */
#define RFX_ECO_LITE_DEMAND_PCT		32
#define RFX_ECO_DEEP_DEMAND_PCT		10
/* iowait boost cap for Little in either eco stage: background I/O keeps a
 * soft boost so screen-on reads stay responsive. */
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

static atomic64_t rfx_input_ts = ATOMIC64_INIT(0);

static inline bool rfx_touch_active(u64 time, u64 window_ns)
{
	u64 ts = (u64)atomic64_read(&rfx_input_ts);

	return (ts != 0 && time >= ts && (time - ts) < window_ns);
}

/* Time since the last touch input event, 0 while input has not ever
 * been seen. Base of the eco-state quiet timers. */
static inline u64 rfx_input_quiet_ns(u64 time)
{
	u64 ts = (u64)atomic64_read(&rfx_input_ts);

	if (!ts || time < ts)
		return 0;
	return time - ts;
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
	bool prev_gaming_mode;

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

	bool little_cap_lifted;
	u64 little_floor_end_ns;
	unsigned int little_prev_demand;
	bool big_cap_lifted;
	bool parked;
	unsigned int park_exit_count;
	/* Universal quiescent eco stage latches (daily mode only) */
	bool eco_lite_active;
	bool eco_deep_active;
	bool dl_bw_exceeded;
	unsigned long max_dl_bw;
	bool thermal_cooling;
	bool risk_high;

	u64 warmup_low_demand_since_ns;

	u64 descend_hold_end_ns;
	unsigned int prev_demand_pct;

	/* Gaming floor-band hysteresis: band latched until demand stays in
	 * the clear zone for the hold window, so a single low tick does not
	 * release the floor.
	 */
	bool floor_gate_active;
	u64 floor_gate_low_since_ns;
	u64 step_down_end_ns;

	/* Sustained saturation gate: consecutive high-demand windows */
	u8 sat_count;

	/* Hysteretic thermal dampening: ratcheted stage level + ref time */
	unsigned int thermal_stage;
	u64 thermal_stage_ref_ns;

	unsigned int dbg_thermal_stage;

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

	/*
	 * In gaming mode, rely strictly on clamp_pct (actual policy->max limits
	 * enforced by thermal cooling devices) rather than instantaneous CFS
	 * thermal pressure fluctuations. This prevents premature 600MHz frequency
	 * collapses when the device reaches normal gaming temperatures (45-50°C).
	 */
	if (rfx_gaming_enabled())
		return clamp_pct;

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

/*
 * Gaming floor-band hysteresis. The floor band latches when smoothed
 * (EMA) demand crosses the ARM threshold, and releases only after demand
 * stays below the CLEAR threshold for the tier's hold window. Little
 * cores have no gated band (their floor is unconditional).
 */
static void rfx_floor_gate_update(struct rfx_policy *p, unsigned int ema_pct,
				  u64 time)
{
	unsigned int arm, clear, hold_ns;

	if (p->is_little)
		return;

	if (p->is_prime) {
		arm = RFX_G_PRIME_GATE_ARM_PCT;
		clear = RFX_G_PRIME_GATE_CLEAR_PCT;
		hold_ns = RFX_G_PRIME_GATE_HOLD_NS;
	} else {
		arm = RFX_G_BIG_GATE_ARM_PCT;
		clear = RFX_G_BIG_GATE_CLEAR_PCT;
		hold_ns = RFX_G_BIG_GATE_HOLD_NS;
	}

	if (!p->floor_gate_active) {
		if (ema_pct >= arm) {
			p->floor_gate_active = true;
			p->floor_gate_low_since_ns = 0;
		}
		return;
	}

	if (ema_pct >= clear) {
		p->floor_gate_low_since_ns = 0;
		return;
	}

	if (!p->floor_gate_low_since_ns)
		p->floor_gate_low_since_ns = time;
	else if (rfx_elapsed(time, p->floor_gate_low_since_ns) >= hold_ns) {
		p->floor_gate_active = false;
		p->floor_gate_low_since_ns = 0;
	}
}

static unsigned int rfx_target_freq(struct rfx_policy *p, unsigned long util,
				    unsigned long max_uclamp_min,
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
		unsigned int fl, demand_pct;
		unsigned int depth;
		unsigned int ema_pct;
		unsigned int top_cap_pct;
		unsigned int floor_gate_on;
		unsigned int boost_fl = 0;

		demand_pct = (unsigned int)(raw_util * 100 / max_cap);
		ema_pct = (unsigned int)(util * 100 / max_cap);
		p->dbg_demand_pct = demand_pct;
		depth = rfx_cool_depth(fceil_pct);

		if (fceil_pct < RFX_G_COOL_ENTER_PCT && !p->thermal_cooling)
			p->thermal_cooling = true;
		if (fceil_pct >= RFX_G_COOL_EXIT_PCT && p->thermal_cooling)
			p->thermal_cooling = false;

		/* Proactive thermal ratchet: caps Big/Prime below the soft
		 * ceiling so sustained load settles early without FPS cliffs.
		 * Little cores are left at full ceiling: their power
		 * contribution is small and they must stay available.
		 */
		if (!little)
			rfx_thermal_stage_update(p, depth, time);

		/*
		 * Sustained saturation gate. A cluster only earns the top OPP
		 * after two consecutive evaluation windows of heavy demand
		 * (filtered util >= 90% or raw util >= 92%); the count caps at
		 * 10 and decays one step per evaluation below release, so a
		 * single spike cannot hold the unlock open.
		 */
		if (!little) {
			if (ema_pct >= RFX_G_SAT_ENTER_PCT ||
			    demand_pct >= RFX_G_SAT_RAW_PCT) {
				if (p->sat_count < RFX_G_SAT_COUNT_MAX)
					p->sat_count++;
			} else if (p->sat_count) {
				p->sat_count--;
			}
		}

		/*
		 * Sweet-spot floor band with hysteresis. The floor is armed
		 * only while smoothed demand holds above the tier arm
		 * threshold (22% Prime / 35% Big), and is released only
		 * after demand stays below the tier release threshold
		 * (14% Prime / 25% Big) for the hold window, so a single
		 * low tick does not release the band.
		 */
		rfx_floor_gate_update(p, ema_pct, time);
		floor_gate_on = p->floor_gate_active;

		/*
		 * Target 1: Smart Dynamic Floor (Sweet Spot OPP).
		 * Prime: 56% gated band; Big: 48% gated band; Little: 40%
		 * unconditional. Gating keeps idle cores free to drop to
		 * fmin when demand is genuinely low in gaming mode.
		 */
		if (prime && floor_gate_on)
			fl = rfx_pct(fceil, RFX_G_PRIME_FLOOR_PCT);
		else if (!little && floor_gate_on)
			fl = rfx_pct(fceil, RFX_G_BIG_FLOOR_PCT);
		else if (little)
			fl = rfx_pct(fceil, RFX_G_LITTLE_FLOOR_PCT);
		else
			fl = rfx_pct(fceil, RFX_G_IDLE_FLOOR_PCT);

		/*
		 * When thermal cooling is active, fceil is already reduced to the
		 * thermal ceiling. Smoothly scale floor toward steady floor.
		 */
		if (p->thermal_cooling && depth && fl) {
			unsigned int steady = rfx_pct(fceil,
						      RFX_G_COOL_STEADY_FLOOR_PCT);

			if (fl > steady)
				fl -= (fl - steady) * (depth / 2) / 100;
		}

		/*
		 * Effective top cap. In gaming mode Big/Prime are clamped to
		 * the tier soft ceiling (84% Prime / 88% Big) unless the
		 * saturation gate has released 100% of the ceiling. The
		 * active thermal stage cap is always applied, so the
		 * effective ceiling is min(soft_or_unlocked, thermal_stage).
		 */
		top_cap_pct = prime ? RFX_G_SOFT_CEIL_PRIME_PCT :
				     RFX_G_SOFT_CEIL_BIG_PCT;
		if (!little && p->sat_count >= RFX_G_SAT_UNLOCK_COUNT)
			top_cap_pct = 100;
		top_cap_pct = min(top_cap_pct, rfx_thermal_cap_pct(p));
		if (!little)
			fceil = rfx_pct(fceil, top_cap_pct);
		else
			fceil = rfx_pct(fceil, RFX_G_LITTLE_CAP_PCT);

		/*
		 * Fast Burst Ramp-Up & Frame-Time Aligned Hold:
		 * When demand on Big/Prime exceeds the surge threshold (> 82%),
		 * immediately scale to the effective ceiling and latch a 2-frame
		 * window (~16.6ms) to bridge the 120Hz inter-frame render gap
		 * and GPU synchronization wait without sawtooth jitter. The
		 * surge floor respects the same effective ceiling as the
		 * transient load path above, so it can never overshoot the
		 * soft ceiling or an active thermal stage cap.
		 */
		if (!little && demand_pct >= RFX_G_SURGE_UTIL_PCT) {
			p->surge_hold_end_ns = time + RFX_G_FRAME_HOLD_NS;
			boost_fl = fceil;
		} else if (p->surge_hold_end_ns && time < p->surge_hold_end_ns) {
			boost_fl = fceil;
		}

		/* Hispeed hold across clusters in gaming mode */
		if (!little && demand_pct >= RFX_G_GO_HISPEED_PCT_DEFAULT)
			p->hispeed_hold_end_ns = time +
				(u64)RFX_G_HISPEED_HOLD_US_DEFAULT * NSEC_PER_USEC;

		if (!little && p->hispeed_hold_end_ns && time < p->hispeed_hold_end_ns) {
			unsigned int hs_fl = min(rfx_pct(fceil, RFX_G_HISPEED_PCT_DEFAULT), fceil);
			boost_fl = max(boost_fl, hs_fl);
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
				touch_pct = 42;

			/* Touch boost also respects the effective ceiling */
			boost_fl = max(boost_fl,
				       min(rfx_pct(fceil, touch_pct), fceil));
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

		p->dbg_warmup_active = (boost_fl > 0);
		p->dbg_thermal_stage = p->thermal_stage;

		/*
		 * Anti-sawtooth step-down cap. When the target drops below
		 * the cached frequency, the governor may descend at most
		 * RFX_G_STEP_DOWN_PCT of policy->max per evaluation. The
		 * 36ms descent hold re-arms the cap across consecutive
		 * evaluations, so a brief thread migration can only step
		 * the frequency down in small increments instead of
		 * cliffing it to the floor in one tick.
		 */
		if (!little && p->policy->max && freq < p->next_freq) {
			unsigned int max_drop = rfx_pct(p->policy->max,
							RFX_G_STEP_DOWN_PCT);
			unsigned int floor_step;

			if (max_drop >= p->next_freq)
				max_drop = p->next_freq - 1;
			floor_step = p->next_freq - max_drop;

			if (p->step_down_end_ns && time >= p->step_down_end_ns)
				p->step_down_end_ns = 0;

			if (!p->step_down_end_ns)
				p->step_down_end_ns = time + RFX_G_STEP_DOWN_HOLD_NS;

			if (freq < floor_step)
				freq = floor_step;
		}

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
		unsigned int raw_cpu_pct = 0;
		bool touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);
		unsigned int ucmin_pct =
			(unsigned int)(max_uclamp_min * 100 / max_cap);
		unsigned int ema_pct =
			(unsigned int)((u64)p->filt_util * 100 / max_cap);

		/*
		 * Universal quiescent eco state (daily only). Strictly gated
		 * on input inactivity, low uclamp_min and filtered-demand
		 * residency; gaming_mode=1 short-circuits all of this at the
		 * top of rfx_next_freq, so a paused game can never enter the
		 * eco state.
		 *
		 * Instant exit on any touch (0 latency) or uclamp_min spike >= 35%.
		 */
		if (touch || ucmin_pct >= RFX_ECO_EXIT_UCMIN_PCT ||
		    rfx_input_quiet_ns(time) < RFX_ECO_INPUT_QUIET_NS) {
			p->eco_lite_active = false;
			p->eco_deep_active = false;
		} else {
			if (p->eco_deep_active) {
				if (ucmin_pct >= RFX_ECO_DEEP_UCMIN_PCT ||
				    ema_pct >= RFX_ECO_DEEP_DEMAND_PCT ||
				    rfx_input_quiet_ns(time) <
				    RFX_ECO_DEEP_INPUT_QUIET_NS)
					p->eco_deep_active = false;
			} else if (rfx_input_quiet_ns(time) >=
				   RFX_ECO_DEEP_INPUT_QUIET_NS &&
				   ucmin_pct < RFX_ECO_DEEP_UCMIN_PCT &&
				   ema_pct < RFX_ECO_DEEP_DEMAND_PCT) {
				p->eco_deep_active = true;
			}
			if (p->eco_lite_active) {
				if (ucmin_pct >= RFX_ECO_UCMIN_PCT ||
				    ema_pct >= RFX_ECO_LITE_DEMAND_PCT ||
				    rfx_input_quiet_ns(time) <
				    RFX_ECO_INPUT_QUIET_NS)
					p->eco_lite_active = false;
			} else if (!p->eco_deep_active &&
				   rfx_input_quiet_ns(time) >=
				   RFX_ECO_INPUT_QUIET_NS &&
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
			 * Ensure Little cores maintain floor during touch interaction or active demand window
			 * to prevent audio/video buffer underruns, while allowing full parking to fmin when idle.
			 */
			if ((touch || (p->little_floor_end_ns && time < p->little_floor_end_ns)) &&
			    freq < rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT))
				freq = rfx_pct(fceil, RFX_D_LITTLE_FLOOR_PCT);

			/*
			 * Eco Little caps: stage 1 caps Little at 38% of fmax;
			 * stage 2 (deep) caps Little at 30% of fmax.
			 * Applied under light/moderate demand (< RFX_D_BURST_UNLOCK_PCT)
			 * so a genuine cold-launch burst unlocks the full capability.
			 */
			if (demand_pct < RFX_D_BURST_UNLOCK_PCT &&
			    p->eco_deep_active)
				cap = rfx_pct(fceil, RFX_ECO_DEEP_LITTLE_CAP_PCT);
			else if (demand_pct < RFX_D_BURST_UNLOCK_PCT &&
				 p->eco_lite_active)
				cap = min(cap, rfx_pct(fceil, RFX_ECO_LITTLE_CAP_PCT));

			/*
			 * DL bandwidth cap: prevent dl_bw_exceeded from blowing past
			 * Daily Sweet-Spot sustained cap unless raw CPU utilization
			 * itself exceeds RFX_D_BURST_UNLOCK_PCT.
			 */
			if (p->dl_bw_exceeded && raw_cpu_pct < RFX_D_BURST_UNLOCK_PCT)
				cap = min(cap, rfx_pct(fceil, RFX_D_LITTLE_SUSTAINED_CAP_PCT));
		} else {
			/*
			 * Daily Big/Prime tier shaping. The cluster is held
			 * at the efficient sweet-spot cap -- 56% of fmax for
			 * Big, 48% for Prime -- so light-to-moderate UI
			 * scrolling and video playback stay on the linear
			 * V/F knee instead of climbing into high-voltage
			 * OPPs. Raw demand >= 92% (app cold launch, camera
			 * burst) unlocks the top OPP so launches stay
			 * snappy without lag.
			 *
			 * If dl_bw_exceeded is true (e.g. video/audio playback in
			 * IG Reels), cap DL-driven boosts to the Sweet-Spot unless
			 * raw CPU utilization itself exceeds RFX_D_BURST_UNLOCK_PCT.
			 */
			if (demand_pct >= RFX_D_BURST_UNLOCK_PCT &&
			    (!p->dl_bw_exceeded || raw_cpu_pct >= RFX_D_BURST_UNLOCK_PCT))
				cap = fceil;
			else if (prime && demand_pct < RFX_D_PRIME_QUIET_PCT &&
				   max_uclamp_min * 100 <
				   (unsigned long)RFX_D_PRIME_QUIET_PCT *
				   max_cap) {
				/*
				 * Prime quiet gate: with light Prime demand
				 * and no high uclamp_min from the
				 * top-app/RenderThread, hold Prime at the
				 * efficient knee so the power-hungry core
				 * is not dragged up by background loads.
				 */
				cap = rfx_pct(fceil, RFX_D_PRIME_QUIET_KNEE_PCT);
			} else
				cap = rfx_pct(fceil,
					prime ? RFX_D_SWEET_CAP_PRIME_PCT :
						     RFX_D_SWEET_CAP_BIG_PCT);

			if (p->dl_bw_exceeded && raw_cpu_pct < RFX_D_BURST_UNLOCK_PCT) {
				unsigned int dl_cap = rfx_pct(fceil,
					prime ? RFX_D_SWEET_CAP_PRIME_PCT :
						RFX_D_SWEET_CAP_BIG_PCT);
				if (cap > dl_cap)
					cap = dl_cap;
			}

			/*
			 * Legacy lift-latch bookkeeping: the latch no longer
			 * shapes the cap under the sweet-spot model, but
			 * reset paths still clear it, so keep the
			 * arming/release semantics identical to before.
			 */
			if (demand_pct >= RFX_D_BIG_LIFT_PCT)
				p->big_cap_lifted = true;
			else if (demand_pct <= RFX_D_BIG_DROP_PCT || !touch)
				p->big_cap_lifted = false;

			/*
			 * Sub-linear light-load mapping: below the knee the
			 * target rises quadratically with filtered demand
			 * (continuous with the proportional mapping at the
			 * knee), so light work stays in the low-voltage
			 * region. uclamp_min is already folded into
			 * p->filt_util, so video playback never drops below
			 * its requested floor. Skipped while touching: UI
			 * scrolling must never feel laggy.
			 */
			if (!touch) {
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

			/*
			 * Eco Big/Prime caps: stage 1 holds Prime at
			 * policy->min (cap 0) and caps Big at 40% of
			 * fmax; stage 2 keeps Prime parked at policy->min
			 * and pins Big at the sustained-burst limit
			 * of 18%. Both apply under light demand
			 * only, so a cold-launch burst unlocks the
			 * top OPP while active.
			 */
			if (demand_pct < RFX_D_BURST_UNLOCK_PCT) {
				if (p->eco_deep_active)
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
	struct rfx_policy *p = rfx_c->rfx_policy;
	unsigned int cap;

	/*
	 * Filter I/O wait boost on Big and Prime clusters. In 120 FPS games,
	 * render threads blocking on GPU sync fences (dma_fence/sync_file)
	 * set SCHED_CPUFREQ_IOWAIT. Falsely boosting Big/Prime induces massive
	 * 10%->100% utilisation spikes and frame pacing jitter. In daily mode,
	 * background I/O daemons must not wake Big/Prime from low OPP, and
	 * the quiescent eco state additionally bars iowait from waking the
	 * Big cluster out of policy->min.
	 */
	if (p && (p->is_prime ||
		 (rfx_gaming_enabled() && !p->is_little) ||
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
		if (rfx_gaming_enabled())
			cap = SCHED_CAPACITY_SCALE / 4;
		else if (p && p->is_little) {
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
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();

	if (likely(!rfx_c->iowait_boost))
		return 0;
	if (rfx_iowait_reset(rfx_c, time, false))
		return 0;
	if (!rfx_c->iowait_boost_pending) {
		bool do_decay = true;

		if (gaming && p) {
			s64 down_delay = p->down_rate_delay_ns > 0 ?
					 p->down_rate_delay_ns : (20LL * NSEC_PER_MSEC);
			if (rfx_elapsed(time, rfx_c->last_iowait_decay) < (u64)down_delay)
				do_decay = false;
		}

		if (do_decay) {
			rfx_c->last_iowait_decay = time;
			rfx_c->iowait_boost >>= 1;
			if (rfx_c->iowait_boost < IOWAIT_BOOST_MIN) {
				rfx_c->iowait_boost = 0;
				return 0;
			}
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
		rfx_c->rfx_policy->need_freq_update = true;
		rfx_c->rfx_policy->dl_bw_exceeded = true;
	}
}

static inline void rfx_set_down_delay(struct rfx_policy *p, bool gaming, u64 time)
{
	if (gaming) {
		/*
		 * Frame-time aligned down-rate limit for 120Hz display pacing:
		 * 12ms delay (~1.5 frames) and 8.33ms min sample (1 frame) bridge
		 * inter-frame GPU fence wait without dropping frequency mid-frame.
		 *
		 * Big/Prime take a longer 20ms hold in gaming mode: a downward
		 * transition longer than one full 90/120Hz frame window, so a
		 * single low-util sample cannot oscillate the frequency within
		 * a frame. Little keeps the 12ms pacing limit.
		 */
		if (!p->is_little)
			p->down_rate_delay_ns =
				(s64)RFX_GAMING_DOWN_OVERRIDE_US * NSEC_PER_USEC;
		else
			p->down_rate_delay_ns = (s64)RFX_G_DOWN_RATE_LIMIT_NS;
		p->min_sample_ns = (s64)RFX_G_MIN_SAMPLE_NS;
	} else {
		bool touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);

		if (p->parked) {
			/* Idle park: zero downward delay for instant deep
			 * C-state entry. */
			p->down_rate_delay_ns = 0;
			p->min_sample_ns = 0;
			return;
		}

		p->down_rate_delay_ns =
			(s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
		/*
		 * Crisp baseline down-shed: clamp the sysfs-configured down
		 * limit to 3ms, or 4ms on an active touch window so a
		 * short interaction can still hold the frequency without
		 * oscillation. The long multi-frame hold is reserved for
		 * the interactive pacing profile.
		 */
		if (touch && !p->is_little)
			p->down_rate_delay_ns = min_t(s64, p->down_rate_delay_ns,
					(s64)RFX_D_TOUCH_DOWN_US * NSEC_PER_USEC);
		else
			p->down_rate_delay_ns = min_t(s64, p->down_rate_delay_ns,
					(s64)RFX_D_DOWN_US * NSEC_PER_USEC);
		if (touch && !p->is_little) {
			p->min_sample_ns = (s64)(p->is_prime ? 2000LL : 4000LL) *
					    NSEC_PER_USEC;
		} else if (!touch) {
			p->min_sample_ns = (s64)(p->is_little ? 800LL : 400LL) *
					    NSEC_PER_USEC;
		} else {
			p->min_sample_ns = (s64)(p->is_little ?
				RFX_D_LITTLE_MIN_SAMPLE_US :
				RFX_D_BIG_MIN_SAMPLE_US) * NSEC_PER_USEC;
		}
	}
}

static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming) {
		/*
		 * Fast upward response in gaming mode: frame bursts must never
		 * lag. Big/Prime bypass the up-rate limit entirely on heavy
		 * demand; otherwise hold the transition under the 1000us cap
		 * so latency stays low without chasing single-sample noise.
		 */
		if (p->dbg_demand_pct >= RFX_G_SURGE_UTIL_PCT && !p->is_little)
			p->up_rate_delay_ns = 0;
		else if (p->is_little)
			p->up_rate_delay_ns =
				(s64)min((unsigned int)RFX_LITTLE_UP_US,
					 (unsigned int)RFX_G_GAMING_UP_RATE_US) *
					 NSEC_PER_USEC;
		else
			p->up_rate_delay_ns =
				(s64)min(RFX_G_GAMING_UP_RATE_US,
					 p->tunables->up_rate_limit_us) *
					 NSEC_PER_USEC;
	} else {
		unsigned int base_up = p->is_little ? RFX_LITTLE_UP_US : RFX_BIG_UP_US;

		/*
		 * Baseline upward latency stays under the 1800us cap so
		 * launches and scroll kicks never lag, regardless of a
		 * (possibly slow) sysfs-configured up rate limit.
		 */
		p->up_rate_delay_ns =
			(s64)min(base_up, (unsigned int)RFX_D_UP_MAX_US) * NSEC_PER_USEC;
	}
}

static inline void rfx_set_eval_delay(struct rfx_policy *p, bool gaming, u64 time)
{
	s64 base;
	bool touch;

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
	unsigned long max_uclamp_min = 0;
	unsigned long max_dl_bw = 0;
	bool dl_bw_exceeded = p->dl_bw_exceeded;
	unsigned int j;
	bool touch = false;
	bool smooth = gaming;

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *jc = per_cpu_ptr(&rfx_cpu, j);
		unsigned long jb, je, uc_min;

		if (rfx_dl_bw_exceeded_gki510(j, jc->bwmin))
			dl_bw_exceeded = true;
		if (jc->bwmin > max_dl_bw)
			max_dl_bw = jc->bwmin;

		jb = rfx_iowait_apply(jc, time, max_cap);
		rfx_get_util(jc, jb);
		uc_min = rfx_get_uclamp_min_gki510(j);
		if (uc_min > max_uclamp_min)
			max_uclamp_min = uc_min;
		je = max(jc->util, jb);
		if (uc_min > je)
			je = min(uc_min, max_cap);

		if (je > max_util)
			max_util = je;
	}

	p->dl_bw_exceeded = dl_bw_exceeded;
	p->max_dl_bw = max_dl_bw;

	/*
	 * Prime Cluster Utilisation Smoothing in Gaming Mode:
	 * Dampen micro-transient spikes while retaining instant response
	 * for heavy render thread migrations (>= 85% capacity).
	 */
	if (gaming && p->is_prime && max_util > p->filt_util && p->filt_util > 0) {
		if (max_util < (max_cap * 85 / 100))
			max_util = (p->filt_util + max_util * 3) / 4;
	}

	if (!gaming) {
		touch = rfx_touch_active(time, (u64)RFX_D_TOUCH_WINDOW_MS * NSEC_PER_MSEC);
		smooth = true;
	}

	p->filt_util = rfx_ema(p->filt_util, max_util, time, &p->last_ema_ns,
			       smooth, gaming, max_cap);

	rfx_set_down_delay(p, gaming, time);
	rfx_pol_up_delay(p, gaming);

	if (!gaming) {
		unsigned int fmin = p->policy->cpuinfo.min_freq;
		/*
		 * Idle floor snap: below RFX_D_IDLE_FLOOR_SNAP_PCT of
		 * capacity with no active touch, snap the target to
		 * policy->min with zero downward hold so the deepest
		 * C-states are reachable after a brief wakeup.
		 */
		unsigned int park_thresh =
			(unsigned int)(max_cap * RFX_D_IDLE_FLOOR_SNAP_PCT / 100);

		/*
		 * Aggressive Idle Ramp-Down:
		 * When load drops below idle threshold, immediately park policy
		 * and bypass down-rate delays to allow rapid deep C-state entry.
		 */
		if (p->filt_util < park_thresh && !touch) {
			p->parked = true;
			p->park_exit_count = 0;
			p->pending_raw_freq = fmin;
			return fmin;
		}

		if (p->is_little) {
			/* Little cluster: park to fmin when idle to enable 120ms eval delay */
			if (!touch && p->filt_util < park_thresh) {
				p->parked = true;
				p->park_exit_count = 0;
			} else {
				if (++p->park_exit_count >= RFX_D_PARK_EXIT_EVALS)
					p->parked = false;
			}
		} else if (touch && !p->is_prime) {
			p->parked = false;
			p->park_exit_count = RFX_D_PARK_EXIT_EVALS;
		} else if (p->is_prime) {
			/*
			 * Daily Mode Prime Core Conservation:
			 * Prime cluster is a high-drain execution tier. In daily mode,
			 * UI interaction and touches are handled smoothly by Little and
			 * Big clusters. Keep Prime parked at fmin unless there is sustained,
			 * heavy workload demand (>= 75%).
			 */
			if (p->filt_util < (max_cap * 75 / 100)) {
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

	return rfx_target_freq(p, p->filt_util, max_uclamp_min, max_cap, time, gaming);
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
	if (p->next_freq != freq) {
		p->work_in_progress = true;
		irq_work_queue(&p->irq_work);
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
	p->floor_gate_active = false;
	p->floor_gate_low_since_ns = 0;
	p->step_down_end_ns = 0;
	p->descend_hold_end_ns = 0;
	p->sat_count = 0;
	p->thermal_stage = 0;
	p->thermal_stage_ref_ns = 0;
	p->gaming_warmup_end_ns = 0;
	p->warmup_ramp_pct = 0;
	p->warmup_ramp_last_ns = 0;
	p->hispeed_hold_end_ns = 0;
	p->surge_hold_end_ns = 0;
	p->surge_tier = 0;
	p->prev_demand_pct = 0;
	p->gaming_warmup_pending = false;
	p->warmup_pending_entry = false;
	p->gaming_warmup_entry = false;
	p->quiet_since_ns = 0;
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
	p->little_prev_demand = 0;
}

static void rfx_reset_policy_locked(struct rfx_policy *p)
{
	/* Full reset: used on policy start/stop and as the base for both
	 * flush helpers. Zeroes all transient state fields. */
	rfx_flush_gaming_state_locked(p);
	rfx_flush_daily_state_locked(p);
	p->thermal_cooling = false;
	p->warmup_low_demand_since_ns = 0;
	p->risk_high = false;
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

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	atomic_set(&rfx_gaming, val);
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

			{
				unsigned int base_up = p->is_little ?
					RFX_LITTLE_UP_US : RFX_BIG_UP_US;

				p->up_rate_delay_ns =
					(s64)min(base_up, (unsigned int)RFX_D_UP_MAX_US) *
					NSEC_PER_USEC;
			}
			p->down_rate_delay_ns =
				(s64)p->tunables->down_rate_limit_us *
					 NSEC_PER_USEC;
			p->min_sample_ns = 0;
			p->freq_update_delay_ns =
				(s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
			p->prev_gaming_mode = false;
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
	} else {
		/* 0->1 transition: flush every daily latch, then arm the
		 * gaming warmup and rate-limit windows. */
		struct rfx_policy *p;
		unsigned long flags, pflags;
		u64 now = (u64)sched_clock();

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);
			rfx_flush_daily_state_locked(p);

			p->gaming_warmup_pending = false;
			p->warmup_pending_entry = false;
			p->gaming_warmup_entry = true;
			p->gaming_warmup_start_ns = now;
			p->gaming_warmup_end_ns = now + RFX_GAMING_WARMUP_NS;
			p->warmup_ramp_pct = 100;
			p->warmup_ramp_last_ns = now;

			if (!p->is_little)
				p->down_rate_delay_ns =
					(s64)RFX_GAMING_DOWN_OVERRIDE_US *
						NSEC_PER_USEC;
			else
				p->down_rate_delay_ns =
					(s64)RFX_G_DOWN_RATE_LIMIT_NS;
			if (p->is_little)
				p->up_rate_delay_ns =
					(s64)min((unsigned int)RFX_LITTLE_UP_US,
						 (unsigned int)RFX_G_GAMING_UP_RATE_US) *
						 NSEC_PER_USEC;
			else
				p->up_rate_delay_ns =
					(s64)min(RFX_G_GAMING_UP_RATE_US,
						p->tunables->up_rate_limit_us) *
						NSEC_PER_USEC;
			p->min_sample_ns = (s64)RFX_G_MIN_SAMPLE_NS;
			p->freq_update_delay_ns =
				(s64)RFX_G_EVAL_US_DEFAULT * NSEC_PER_USEC;

			p->prev_gaming_mode = true;
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
	WARN_ON(rfx_ema(100, 200, t, &ns, true, false, 1000) != 200 || ns != t);

	ns = t;
	WARN_ON(rfx_ema(200, 100, t, &ns, false, false, 1000) != 100);

	/* Gaming: per-sample asymmetric blend, time-independent */
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true, 1000) != 780 || ns != t);
	WARN_ON(rfx_ema(100, 200, t, &ns, true, true, 1000) != 172);
	WARN_ON(rfx_ema(0, 200, t, &ns, true, true, 1000) != 200);
	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true, 1000) != 780 || ns != t);

	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false, 1000) !=
		1000 - 1000 / RFX_EMA_DAILY_DIVISOR || ns != t);

	ns = t + NSEC_PER_MSEC;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, true, 1000) != 780);

	/* Daily zero-snap: a >8ms quiet gap with near-zero demand
	 * collapses the filter straight to the newest sample. */
	ns = t - RFX_D_TIME_DECAY_SNAP_NS - 1;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false, 1000) != 0 || ns != t);

	/* Exactly at the 8ms boundary the snap does not fire (strict
	 * >); the step clamp still leaves a non-zero residual. */
	ns = t - RFX_D_TIME_DECAY_SNAP_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true, false, 1000) == 0 || ns != t);

	/* Demand at/over the snap util threshold (10% of capacity) keeps
	 * the divisor decay even across a long quiet gap. */
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
	p.prev_demand_pct = 70;
	WARN_ON(rfx_descend_hold(&p, 60, true, false, t));
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
	BUILD_BUG_ON(RFX_G_LITTLE_FLOOR_PCT > 100);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_SOFT_CEIL_PRIME_PCT);
	BUILD_BUG_ON(RFX_G_BIG_FLOOR_PCT > RFX_G_SOFT_CEIL_BIG_PCT);
	BUILD_BUG_ON(RFX_G_SOFT_CEIL_PRIME_PCT > 100);
	BUILD_BUG_ON(RFX_G_SOFT_CEIL_BIG_PCT > 100);
	BUILD_BUG_ON(RFX_G_SAT_UNLOCK_COUNT > RFX_G_SAT_COUNT_MAX);
	BUILD_BUG_ON(RFX_G_SAT_ENTER_PCT > RFX_G_SAT_RAW_PCT);
	BUILD_BUG_ON(RFX_G_EMA_ATTACK_PCT > 100 || RFX_G_EMA_DECAY_PCT > 100);
	BUILD_BUG_ON(RFX_G_EMA_ATTACK_PCT <= RFX_G_EMA_DECAY_PCT);
	BUILD_BUG_ON(RFX_G_STEP_DOWN_PCT >= 100);
	BUILD_BUG_ON(RFX_G_GAMING_UP_RATE_US > 1000);
	BUILD_BUG_ON(RFX_GAMING_DOWN_OVERRIDE_US <
		     (unsigned int)((u64)RFX_G_DOWN_RATE_LIMIT_NS / NSEC_PER_USEC));
	BUILD_BUG_ON(RFX_G_PRIME_GATE_ARM_PCT <= RFX_G_PRIME_GATE_CLEAR_PCT);
	BUILD_BUG_ON(RFX_G_BIG_GATE_ARM_PCT <= RFX_G_BIG_GATE_CLEAR_PCT);
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

	BUILD_BUG_ON(RFX_D_LITTLE_FLOOR_REARM_PCT >= RFX_D_LITTLE_FLOOR_ARM_PCT);
	/* Daily Big/Prime shaping: the sweet-spot caps live inside the
	 * linear V/F knee, the burst unlock must stay at fmax or below,
	 * and the Prime quiet gate must not clamp during an app launch or
	 * camera burst (i.e. below the burst unlock threshold). */
	BUILD_BUG_ON(RFX_D_LITTLE_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_SWEET_CAP_BIG_PCT > 100 ||
		     RFX_D_SWEET_CAP_PRIME_PCT > 100);
	BUILD_BUG_ON(RFX_D_BURST_UNLOCK_PCT > 100);
	/* The sub-linear knee must sit inside the burst unlock so the
	 * quadratic mapping can only tighten, never loosen, the ceiling. */
	BUILD_BUG_ON(RFX_D_SUBLINEAR_KNEE_PCT >= RFX_D_BURST_UNLOCK_PCT ||
		     RFX_D_SUBLINEAR_KNEE_PCT < 1);
	BUILD_BUG_ON(RFX_D_PRIME_QUIET_PCT >= RFX_D_BURST_UNLOCK_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_QUIET_PCT > 100);
	BUILD_BUG_ON(RFX_D_PRIME_QUIET_KNEE_PCT > RFX_D_SWEET_CAP_PRIME_PCT);
	/* The idle park threshold must sit below the Prime quiet gate so
	 * a snapped-to-idle sample still reaches the park path. */
	BUILD_BUG_ON(RFX_D_IDLE_FLOOR_SNAP_PCT >= RFX_D_PRIME_QUIET_PCT);
	BUILD_BUG_ON(RFX_D_IDLE_FLOOR_SNAP_PCT > 100);
	/* The zero-snap util threshold must sit above the idle park
	 * threshold: the snap only collapses the filter for near-zero
	 * demand. */
	BUILD_BUG_ON(RFX_D_TIME_DECAY_SNAP_UTIL_PCT <= RFX_D_IDLE_FLOOR_SNAP_PCT);
	BUILD_BUG_ON(RFX_D_TIME_DECAY_SNAP_UTIL_PCT > 100);
	BUILD_BUG_ON(RFX_D_TOUCH_FLOOR_MIN_DEMAND > 100);
	BUILD_BUG_ON(RFX_D_BIG_TOUCH_FLOOR_PCT > RFX_D_SWEET_CAP_BIG_PCT);
	/* Baseline iowait caps must stay monotone per tier: Little gets
	 * the most, Big less, Prime none. */
	BUILD_BUG_ON(RFX_D_IOWAIT_LITTLE_PCT > 100 ||
		     RFX_D_IOWAIT_BIG_PCT > RFX_D_IOWAIT_LITTLE_PCT ||
		     RFX_D_IOWAIT_PRIME_PCT > RFX_D_IOWAIT_BIG_PCT ||
		     RFX_D_IOWAIT_PRIME_PCT > 0);
	BUILD_BUG_ON(RFX_D_DOWN_US > RFX_D_TOUCH_DOWN_US ||
		     RFX_D_TOUCH_DOWN_US > 10000);
	/* Upward latency stays under the 2000us cap so cold launches and
	 * scroll kicks never lag. */
	BUILD_BUG_ON(RFX_D_UP_MAX_US > 2000);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE > 1);
	BUILD_BUG_ON(RFX_D_ENERGY_AWARE_MIN_PCT > 100);

	BUILD_BUG_ON(RFX_D_PARK_EXIT_PCT * 16 <= 100);
	BUILD_BUG_ON(RFX_D_PARK_EXIT_EVALS < 1);
	BUILD_BUG_ON(RFX_LITTLE_CAP_THRESHOLD >= RFX_PRIME_CAP_THRESHOLD);
	BUILD_BUG_ON(RFX_EMA_DAILY_DIVISOR < 1 || RFX_EMA_MAX_STEPS < 1);
	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_ENTER_PCT);

	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_G_RISK_CLEAR_PCT >= RFX_G_RISK_ARM_PCT);

	BUILD_BUG_ON(!RFX_CEIL_RISE_PCT_PER_2MS);
	BUILD_BUG_ON(!RFX_CEIL_FALL_DWELL_NS || !RFX_CEIL_FALL_BYPASS_PCT);
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

	/* Eco stage gating assertions */
	BUILD_BUG_ON(RFX_ECO_DEEP_INPUT_QUIET_NS <= RFX_ECO_INPUT_QUIET_NS);
	BUILD_BUG_ON(RFX_ECO_DEEP_UCMIN_PCT >= RFX_ECO_UCMIN_PCT);
	BUILD_BUG_ON(RFX_ECO_UCMIN_PCT >= RFX_ECO_EXIT_UCMIN_PCT);
	BUILD_BUG_ON(RFX_ECO_DEEP_DEMAND_PCT >= RFX_ECO_LITE_DEMAND_PCT);
	BUILD_BUG_ON(RFX_ECO_DEEP_LITTLE_CAP_PCT >= RFX_ECO_LITTLE_CAP_PCT);
	BUILD_BUG_ON(RFX_ECO_DEEP_BIG_CAP_PCT >= RFX_ECO_BIG_CAP_PCT);

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

	return 0;
}

static void __exit vorpal_gov_exit(void)
{
	if (rfx_input_registered)
		input_unregister_handler(&rfx_input_handler);
	cpufreq_unregister_governor(&vorpal_gov);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.3");
