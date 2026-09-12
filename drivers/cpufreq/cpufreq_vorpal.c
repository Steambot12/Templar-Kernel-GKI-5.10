// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.2 — schedutil-derived, tri-cluster.
 *
 * Two profiles: gaming (high band) and daily (ceiling-relative caps/floors).
 * Policy-wide directional EMA util, load-proportional headroom, latched thermal
 * net. Every floor and cap percent is a percentage of the effective ceiling
 * (fceil), never of hardware fmax.
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
#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
#endif

#define CPUFREQ_VORPAL_NAME     "vorpal"
#define CPUFREQ_VORPAL_VERSION  "2.2"
#define CPUFREQ_VORPAL_AUTHOR   "Templar Dev"

/* Sched-core helpers (owned by core sched): util getter, DL-bandwidth check,
 * SUGOV DL class setter for the slow-path worker. */
extern void rfx_get_util_gki510(int cpu, unsigned long boost, bool bound_rt,
				unsigned long *util, unsigned long *bwmin);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);
extern int rfx_setattr_sugov_gki510(struct task_struct *t);

/* ===================================================================== */
/* Tunable defaults (KMI-safe: plain #defines).                          */
/* ===================================================================== */

/* Cluster identification by arch capacity. */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* DAILY eval rate limits (us), at rest only: gaming and the DL bypass
 * override. up=0 = commit on the first eval that sees the rise. */
#define RFX_LITTLE_RATE_US		2000
#define RFX_LITTLE_UP_US		100
#define RFX_LITTLE_DOWN_US		2200

#define RFX_BIG_RATE_US			2000
#define RFX_BIG_UP_US			0
#define RFX_BIG_DOWN_US			2000

/* Gaming eval rate.
 *
 * Fast-switch drivers commit inside the util hook, so the gate may sit well
 * inside a frame -- an evaluation that changes nothing costs a comparison.
 *
 * A slow-switch driver commits from a SCHED_DEADLINE worker instead: each
 * accepted commit is a wakeup of the highest-priority class on the render
 * cluster plus a firmware round trip, and the reserved runtime is a small
 * fraction of one period. At the fast-switch cadence that budget is spent
 * inside a frame, after which the worker is throttled to the next period
 * boundary -- so the commit that matters lands a frame late, having preempted
 * the render thread on the way. Gate that path near its own transition cost:
 * up-rate stays 0, so a rise still commits on the first eval that sees it. */
#define RFX_FAST_RATE_US		400
#define RFX_SLOW_SWITCH_RATE_US		2000

/* Gaming down-rate gate. NOT rate-neutral -- the slew window resets on a
 * commit in either direction, this gate only on a downward one, so widening
 * it ratchets the clock up. */
#define RFX_GAMING_DOWN_US		3000

/* Gaming floors, percent of the effective ceiling. NO cluster is capped: every
 * cluster tracks demand up to fceil. Floors only cover a cold landing, and they
 * are the gaming resting-power dial.
 *
 * Do not lower a *_FLOOR_PCT on a tier that may render, and do not raise one
 * either: the extra heat lowers fceil and the render cluster leaves fmax. */
/* On both target devices the top tier is the SPILL tier (render = middle
 * tier), so its floor is pure resting power -- the heat that pushes the
 * die over the limiter's step threshold and starts the spike cycle:
 * burst chase -> power spike -> limiter step -> cpu sag -> gpu sag. */
/* Values tightened per external tuning guidance: floors lowered to cut
 * resting power while the frame is carried by demand + up-rate-0. */
#define RFX_G_PRIME_FLOOR_PCT		52
#define RFX_G_BIG_FLOOR_PCT		50
/* Warmup floor, both render tiers: spawn/asset load only, never steady state. */
#define RFX_G_WARMUP_FLOOR_PCT		72
/* Little never renders, so this floor is pure resting power: at the V/f knee
 * (== idle floor), never above it. Demand and up-rate-0 still cover a frame. */
#define RFX_G_LITTLE_FLOOR_PCT		32

/* Max downward slew, percent of ceiling per 2ms elapsed (so a half percent
 * per ms is expressible in integers). Bounds the depth a short lull can dig:
 * a recovery frame re-materialises demand over ~1ms of PELT, so every point
 * of dip is a frame-time tax on the next frame. The EMA owns descent shape --
 * the pair is tuned together, never loosen both. */
#define RFX_GAMING_DOWN_PCT_PER_2MS	1

/* ---- Daily shaping, percent of the effective ceiling. Caps only, slid
 * between a base and a sustained endpoint by demand: PELT alone carries the
 * rise, so a stepped ceiling would be followed step-for-step by the clock. ---- */
/* Little daily cap: just above the V/f knee; bottom of the slide. */
#define RFX_D_LITTLE_CAP_PCT		56
/* Top of the slide. Little is not pinned across the whole band, and clocking
 * a merely-busy cluster down does not buy idle -- the same lesson the
 * two-tier part taught about derating. */
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	72
/* Slide band, skewed 1.25x: the sustained cap is reached at ~61% real demand,
 * the base cap at ~36%. Little reads the same band as the Big/Prime pair. */
#define RFX_D_LITTLE_LIFT_PCT		80
#define RFX_D_LITTLE_DROP_PCT		45
/* Big/Prime share one band; a sustained cap may never exceed 100. The band
 * reads the same 1.25x-skewed demand as the gaming gates. Idle and light
 * load sit on the base cap. */
#define RFX_D_BIG_CAP_PCT		64
#define RFX_D_PRIME_CAP_PCT		64
#define RFX_D_BIG_LIFT_PCT		85
#define RFX_D_BIG_DROP_PCT		68
#define RFX_D_BIG_SUSTAINED_CAP_PCT	76
#define RFX_D_PRIME_SUSTAINED_CAP_PCT	76

/* ---- Gaming frame-paced boost: one short additive window per demand STEP
 * (not per level). Sits on top of demand + up-rate-0, which already carry a
 * frame; this covers only the onset rise of a frame burst. Reference is
 * resampled every SAMPLE_NS so demand that has settled high cannot keep
 * re-arming -- a step, not a level, opens the window.
 *
 * The window must stay well UNDER one frame period. At a period equal to the
 * window (8ms window at ~120Hz), every frame's render spike is a fresh step
 * against the last resample, the boost re-armed every frame, and the "onset"
 * lift became a de-facto permanent one -- measured as ~5C above the no-boost
 * baseline, which moves the first limiter event earlier exactly where frames
 * are at risk. Half a period keeps the per-frame onset response while halving
 * the duty of the lift. ---- */
#define RFX_G_FRAME_BOOST_DELTA_PCT	18
#define RFX_G_FRAME_BOOST_ARM_PCT	55
#define RFX_G_FRAME_BOOST_CLEAR_PCT	(RFX_G_FRAME_BOOST_ARM_PCT - 15)
#define RFX_G_FRAME_BOOST_NS		(4 * NSEC_PER_MSEC)
#define RFX_G_FRAME_BOOST_SAMPLE_NS	(4 * NSEC_PER_MSEC)
#define RFX_G_FRAME_BOOST_MAX_PCT	8

/* ---- Down-only commit hysteresis: a target within the band of the last
 * COMMITTED frequency is suppressed so util noise cannot walk the OPP back
 * and forth. Rises always pass -- a deadband on the way up is a frame latency
 * tax. Gaming band is wider (frame pacing); daily band only damps the OPP
 * ping-pong a scroll produces when demand sits between two levels. See
 * rfx_target_hysteresis() for why the reference is next_freq, not a stored
 * target. ---- */
#define RFX_G_TARGET_DOWN_DEADBAND_PCT	3
#define RFX_D_TARGET_DOWN_DEADBAND_PCT	2

/* ---- Daily UI-interaction boost: replaces the cap-release burst. One window
 * per demand step into the ARM region, then a cooldown so a duty-cycling load
 * cannot hold it open -- two release mechanisms at once would double-lift the
 * same interaction.
 *
 * ARM must sit ABOVE the band a screen-on-but-idle phone rests in, not at its
 * edge. Demand reads ~1.25x real, so a low ARM plus a step small enough for
 * any background timer to clear makes the window open on housekeeping instead
 * of on interaction -- at which point the lift is a second cap, paid for
 * continuously, and the cap shape below it stops being the power dial.
 *
 * The band has no floors, so at full burst the tier cap is the whole shortfall:
 * a saturating interaction runs a quarter under ceiling with nothing to lift it
 * back. Raising the cap pays for that on every sustained load as well, which is
 * the trade that already failed twice. So the window carries it instead.
 *
 * Shape: a scroll is not one event, it is a train of one-or-two-frame events
 * spaced by the frame period, and the detector must re-fire across the train
 * -- a step small enough for the per-frame demand oscillation to clear, a
 * sample window short enough to catch it, and a cooldown that lets the NEXT
 * frame's step re-open the window instead of spacing them a third of a second
 * apart. The lift itself is small and short: one frame's worth, expressed as a
 * cap lift where the band binds, not an additive kick that fires regardless
 * of whether demand was even pressing the cap. ---- */
#define RFX_D_UI_ARM_PCT		40
#define RFX_D_UI_CLEAR_PCT		14
#define RFX_D_UI_STEP_PCT		12
#define RFX_D_UI_BOOST_PCT		8
#define RFX_D_UI_BOOST_NS		(10 * NSEC_PER_MSEC)
#define RFX_D_UI_COOLDOWN_NS		(80 * NSEC_PER_MSEC)
#define RFX_D_UI_SAMPLE_NS		(16 * NSEC_PER_MSEC)

/* ---- Util EMA: rise instant, decay time-normalised, so the time constant is
 * independent of eval rate. Period = interval removing 1/DIVISOR of the
 * remaining error. ---- */
#define RFX_EMA_DECAY_PERIOD_NS		250000	/* one gaming eval */
/* Gaming decay: tau ~25ms. Must span more than one frame gap or the
 * inter-frame trough collapses the render floor every frame; a faster decay
 * tracked intra-frame duty instead of the scene and measured as an FPS drop.
 * The slew bound must stay LOOSER than this filter. */
#define RFX_EMA_GAMING_DIVISOR		100
#define RFX_EMA_MAX_STEPS		32	/* cap: 8ms, one frame gap */

/* ---- Headroom above demand, percent. Stacks on the 25% DVFS margin already
 * applied by rfx_get_util_gki510, so it only raises the resting OPP.
 * Daily has none. The margin already covers OPP granularity, so the tiered
 * curve this replaces only ever added a resting bin -- and it sat exactly in
 * the demand band that light browsing and scrolling run in, which is the load
 * the daily profile is judged on. Nothing here touches a cap, so no clip edge
 * moves and no latency is traded for it. Only gaming keeps a ramp. ---- */
/* Gaming headroom, phased in linearly from the GATE: below it the resting OPP
 * is untouched, above it a frame is near budget and this closes the gap. Flat
 * at every level was resting-power cost; zero at every level cost the frame. */
#define RFX_HEADROOM_GAMING		5
#define RFX_HEADROOM_GAMING_GATE	78

/* Util percent at which we stop interpolating and request fmax outright.
 * Gaming 100 disables the shortcut: any lower value makes the render tier
 * JUMP to fmax early and pin flat there -- top voltage step, no FPS gained.
 * Daily 95: the last OPP is a battery cost and the caps shape the top. */
#define RFX_SAT_TO_MAX_GAMING_PCT	100
#define RFX_SAT_TO_MAX_DAILY_PCT	95

/* ---- Thermal emergency net. HW LMH (thermal_pressure) and the vendor HAL
 * (policy->max) are the real controllers; this is one hard latched net for
 * when the vendor engine is absent. One trip, one release, 7C apart. ---- */
#define RFX_THERMAL_POLL_GAMING_MS	100
#define RFX_THERMAL_POLL_IDLE_MS	5000	/* deferrable: free in deep sleep */
#define RFX_THERMAL_POLL_WARM_MS	2000
#define RFX_TEMP_WARM_MC		70000
#define RFX_TEMP_EMERGENCY_MC		95000	/* junction; LMH acts far below */
#define RFX_TEMP_EMERGENCY_CLEAR_MC	88000
#define RFX_EMERGENCY_CAP_PCT		70

/* Warmup ramp: instant rise, linear decay back to the baseline floor. */
#define RFX_WARMUP_RAMP_DOWN_MS	60

/* Gaming warmup lifts the render floors for spawn + asset load. Extends while
 * demand stays >EXTEND_PCT up to MAX_NS, releases early below RELEASE_PCT.
 * The window is NOT anchored to the gaming_mode write: that write happens
 * from the launcher, an unbounded interval before the game process exists,
 * so a write-anchored window expired on launcher idle and the spawn burst
 * arrived on the bare baseline floor -- which is why lengthening the window
 * measured no change. The write arms a PENDING window; it starts on the
 * first demand crossing TRIGGER (a spawn-sized burst, not launcher
 * activity), never while the cooling latch holds, and is one-shot per
 * gaming_mode entry. Still capped at MAX_NS from the arm instant, so it
 * cannot pin every cluster through the hottest phase. */
#define RFX_GAMING_WARMUP_NS		(180 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_MAX_NS	(220 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_TRIGGER_PCT	60
#define RFX_GAMING_WARMUP_EXTEND_PCT	90
#define RFX_GAMING_WARMUP_RELEASE_PCT	40
#define RFX_GAMING_WARMUP_RELEASE_NS	(100 * NSEC_PER_MSEC)

/* Frame-risk re-arm of that same window. ARM sits above ordinary busy-scene
 * demand, CLEAR well below it, so one crossing yields one window; the window is
 * one sub-frame burst -- long enough to carry a late frame past its deadline,
 * short enough that the RAMP_DOWN decay, not the window, dominates the thermal
 * cost. Demand here reads ~1.25x real (see the caveat at demand_pct) and these
 * were tuned WITH that skew, so skew and numbers are a matched pair. */
#define RFX_G_RISK_ARM_PCT		92
#define RFX_G_RISK_CLEAR_PCT		78
#define RFX_G_RISK_BOOST_NS		(20 * NSEC_PER_MSEC)

/* Gaming demand gate -- the only demand threshold in the gaming band. Below
 * GATE a cluster is idle: floor releases, no lift may arm; it rejoins above
 * GATE_EXIT. Every lift reads the floor_gated latch, never demand directly.
 * One gate for every role and every threshold -- which tier renders is a
 * per-frame EAS decision the governor cannot see. */
#define RFX_G_FLOOR_GATE_PCT		25
#define RFX_G_FLOOR_GATE_EXIT_PCT	35

/* Floor for a gated (idle) cluster: at the V/f knee -- from fmin the OPP
 * transition plus rate gate turn a cold climb into a visible hitch. */
#define RFX_G_IDLE_FLOOR_PCT		32

/* Cluster cool-down band, hysteretic. Below ENTER the platform limiter is
 * taking capacity, so floors drop for relief and return at EXIT. A limiter
 * that reports continuously (vs step-wise) parks fceil between the two
 * thresholds; EXIT clears it well above ENTER so the band does not flap.
 *
 * ENTER is the sensitive one. The latch it sets collapses every floor toward
 * the relief value, suppresses the frame boost and freezes the warmup window,
 * so one noisy sample from a continuously-reporting limiter pulls the clock
 * DOWN while demand is climbing. Raising ENTER made a single early blip -- the
 * loudest part of a session, before the die has settled -- enough to trip it.
 * Back to the pair that measured clean. */
#define RFX_G_COOL_ENTER_PCT		80
#define RFX_G_COOL_EXIT_PCT		88

/* Relief floor once the platform is taking capacity. */
#define RFX_G_COOL_STEADY_FLOOR_PCT	46

/* Depth at which relief is fully applied: between ENTER and DEEP floors slide
 * down proportionally, so the clock walks with the ceiling instead of
 * stepping to the relief floor. */
#define RFX_G_COOL_DEEP_PCT		65

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

/* ===================================================================== */
/* Global state                                                          */
/* ===================================================================== */

/* Master gaming switch, written by gaming_mode sysfs (Prime cluster only). */
static atomic_t rfx_gaming = ATOMIC_INIT(0);

static inline bool rfx_gaming_enabled(void)
{
	return atomic_read(&rfx_gaming) != 0;
}


/* Emergency thermal cap percent (100 = inactive). Latched with hysteresis. */
static atomic_t rfx_emergency_cap_pct = ATOMIC_INIT(100);
/* Userspace-fed temperature fallback (milli-Celsius); 0 = unavailable. */
static atomic_t rfx_temp_mc = ATOMIC_INIT(0);

/* All live policies, so gaming-off can reset every cluster (not just Prime). */
static LIST_HEAD(rfx_policy_list);
static DEFINE_SPINLOCK(rfx_policy_list_lock);

/* ===================================================================== */
/* Data structures                                                       */
/* ===================================================================== */

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
	struct list_head gov_node;	/* on rfx_policy_list */

	raw_spinlock_t update_lock;

	u64 last_upfreq_time;
	u64 last_downfreq_time;
	u64 last_eval_time;		/* stamped on every evaluation, not just commits */
	s64 freq_update_delay_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;

	unsigned int next_freq;
	unsigned int cached_raw_freq;	/* raw request behind the last commit */
	unsigned int pending_raw_freq;	/* raw request awaiting the rate gate */
	unsigned int max_seen;		/* high-water policy->max = unthrottled baseline */

	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;
	bool work_requeue;		/* request landed mid-run: requeue, don't drop */

	bool limits_changed;
	bool need_freq_update;

	bool is_prime;			/* PRIME band applies (3+ tiers only) */
	bool is_little;
	bool gaming_attr;		/* this policy hosts the gaming_mode node */

	/* Warmup ramp (smooth release, not binary) */
	unsigned int warmup_ramp_pct;	/* current ramp level 0-100 */
	u64 warmup_ramp_last_ns;		/* previous ramp evaluation */

	u64 gaming_warmup_end_ns;	/* floor lift after gaming_mode=1 */
	u64 gaming_warmup_start_ns;	/* arm time — anchors the absolute cap */
	bool gaming_warmup_pending;	/* armed at the write, starts on the burst */

	/*
	 * Cluster-wide smoothed util, owned by the policy: a shared policy
	 * commits one frequency, so a per-CPU EMA made the committed value
	 * depend on which CPU ticked last (frequency jitter, micro-stutter).
	 */
	unsigned long filt_util;
	u64 last_ema_ns;			/* timestamp of last EMA update */

	bool floor_gated;		/* gaming: floor released to idle, hysteretic */
	bool thermal_cooling;		/* gaming: floors dropped to idle, hysteretic */

	/* adaptive warmup — early-release tracking */
	u64 warmup_low_demand_since_ns;	/* when demand first fell below release threshold */
	bool risk_high;			/* gaming: frame-risk edge consumed, hysteretic */

	/* gaming frame-paced boost — step detector over a resampled reference */
	u64 gaming_boost_end_ns;	/* additive lift live until this time */
	unsigned int gaming_boost_pct;	/* lift magnitude, percent of fceil */
	unsigned int boost_prev_demand_pct;
	u64 boost_sample_ns;
	bool boost_armed;

	/* daily UI-interaction boost — one window per step, then cooldown */
	u64 daily_ui_boost_end_ns;
	u64 daily_ui_cooldown_end_ns;
	unsigned int daily_ui_prev_demand_pct;
	u64 daily_ui_sample_ns;
	bool daily_ui_armed;
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

/*
 * Cluster identification against arch_scale_cpu_capacity() (biggest CPU = 1024).
 * is_prime means the PRIME band applies: on a two-tier SoC the fastest tier IS
 * the render cluster and takes the BIG band instead.
 */
static inline bool rfx_cap_is_little(unsigned long cap)
{
	return cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool rfx_cap_is_top(unsigned long cap)
{
	return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

/* Distinct capacity tiers. Cached only once a real topology is visible:
 * unnormalized capacities all read 1024 (one tier), and caching that would
 * latch it for the boot. */
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
		return n;	/* unnormalized or single-cluster: do not cache */
	ntiers = n;
	return ntiers;
}

static inline bool rfx_cap_is_prime(unsigned long cap)
{
	return rfx_cap_is_top(cap) && rfx_ntiers() >= 3;
}

/* fmax * pct / 100 */
static inline unsigned int rfx_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

/*
 * Effective ceiling: percent remaining, plus the unthrottled baseline it
 * applies to. Two throttle channels -- reading only the first was the
 * portability gap:
 *   thermal_pressure - cpufreq_cooling / LMH. Present on QCOM.
 *   policy->max      - vendor thermal HAL. The only channel on MTK.
 * Measured against the high-water policy->max, not the live value: a statically
 * low baseline (MTK per-core OPP split) must read as no throttle.
 *
 * Both ratios round UP, no deadband: reading a small real loss as zero raises
 * every floor computed against fceil and the platform clamps harder.
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
 * Elapsed ns between a stored @stamp and hook @time. SIGNED: sibling CPUs
 * snapshot their own rq_clock, so a stamp can read ahead of ours and unsigned
 * subtraction turns that into ~584 years.
 */
static inline u64 rfx_elapsed(u64 time, u64 stamp)
{
	s64 delta = (s64)(time - stamp);

	return delta > 0 ? (u64)delta : 0;
}

/* ===================================================================== */
/* Helpers                                                               */
/* ===================================================================== */

/*
 * Deferred warmup arm. One shot per gaming_mode entry: the crossing consumes
 * the pending flag, so a busy launcher can spend it, but a second write
 * re-arms. Never arms while the cooling latch holds -- same rule as the
 * extend path: no floor ride through a limiter event.
 */
static void rfx_warmup_arm(struct rfx_policy *p, unsigned int demand_pct,
			   u64 time)
{
	if (!p->gaming_warmup_pending || p->thermal_cooling ||
	    demand_pct < RFX_GAMING_WARMUP_TRIGGER_PCT)
		return;

	p->gaming_warmup_pending = false;
	p->gaming_warmup_start_ns = time;
	p->gaming_warmup_end_ns = time + RFX_GAMING_WARMUP_NS;
}

/* Frame-risk re-arm. The warmup window is otherwise armed exactly once, per
 * gaming_mode entry (on the first burst, see rfx_warmup_arm()), so once it
 * lapses the render clusters spend the rest of the
 * session on the bare baseline floor with no transient response left at all --
 * and the demand band between "busy" and "saturated" is precisely where a frame
 * is at risk while the saturation shortcut has not yet pinned the clock.
 *
 * One crossing arms one window: demand must fall back under CLEAR, or the window
 * must lapse, before another can arm. Re-arming while still saturated would make
 * the lifted floor the steady state for the whole session -- and a saturated
 * cluster is a heavy scene, not a missed frame, already at the ceiling through
 * the saturation shortcut, so a lift there buys no clock and only adds heat.
 *
 * Extends only, never shortens, so an edge inside the initial warmup cannot cut
 * it short. A re-armed window cannot itself be stretched by the extend path,
 * whose cap stays anchored to the original arm.
 */
static void rfx_risk_rearm(struct rfx_policy *p, unsigned int demand_pct,
			   unsigned int warmup_fl, u64 time)
{
	if (demand_pct < RFX_G_RISK_ARM_PCT) {
		/* Clear on demand under CLEAR, or on window lapse. Without the
		 * lapse test, demand parked between CLEAR and ARM -- where a busy
		 * scene sits between frames -- latches the edge forever after the
		 * first window and blocks every re-arm. */
		if (demand_pct <= RFX_G_RISK_CLEAR_PCT ||
		    time >= p->gaming_warmup_end_ns)
			p->risk_high = false;
		return;
	}

	/* Nothing to gain: already committed at or above the floor this window
	 * would install, so arming cannot raise this OPP -- only pin it. */
	if (p->risk_high || p->next_freq >= warmup_fl)
		return;

	p->risk_high = true;
	if (time + RFX_G_RISK_BOOST_NS > p->gaming_warmup_end_ns)
		p->gaming_warmup_end_ns = time + RFX_G_RISK_BOOST_NS;
}

/* Warmup ramp: 100 while the window holds, then a linear decay over
 * RFX_WARMUP_RAMP_DOWN_MS back to the baseline floor. */
static unsigned int rfx_update_warmup_ramp(struct rfx_policy *p, bool active, u64 time)
{
	u64 delta_ns;
	unsigned int step;

	if (active) {
		p->warmup_ramp_pct = 100;
		p->warmup_ramp_last_ns = time;
		return 100;
	}

	if (p->warmup_ramp_pct == 0)
		return 0;

	if (!p->warmup_ramp_last_ns)
		p->warmup_ramp_last_ns = time;
	delta_ns = rfx_elapsed(time, p->warmup_ramp_last_ns);

	step = (unsigned int)min_t(u64,
		(delta_ns * 100) / ((u64)RFX_WARMUP_RAMP_DOWN_MS * NSEC_PER_MSEC), 100);

	/* Advance by the time the step consumed, not to `time`: one ramp
	 * percent is 0.6ms but gaming updates arrive every ~250us, so the
	 * division floors to zero on most calls and advancing to `time` drops
	 * the remainder, stalling the decay. */
	if (step > 0) {
		u64 consumed_ns = (u64)step *
			((u64)RFX_WARMUP_RAMP_DOWN_MS * NSEC_PER_MSEC) /
			100;
		p->warmup_ramp_last_ns += consumed_ns;
		p->warmup_ramp_pct -= min(p->warmup_ramp_pct, step);
	}

	return p->warmup_ramp_pct;
}

/* ===================================================================== */
/* Util smoothing                                                        */
/* ===================================================================== */

/*
 * Directional EMA: instant rise, time-normalised decay. Each period removes
 * 1/RFX_EMA_GAMING_DIVISOR of the remaining error.
 */
static unsigned long rfx_ema(unsigned long old, unsigned long val, u64 time,
			     u64 *last_ns, bool gaming)
{
	u64 delta_ns;
	unsigned long diff;
	unsigned int steps;

	/* Seed, instant rise, or daily instant fall: nothing pending, so the
	 * reference moves to now. */
	if (!old || val >= old || !gaming) {
		*last_ns = time;
		return val;
	}

	/* Unseeded reference: worth one period, as an absolute stamp. Clamped:
	 * sched_clock near zero must not wrap backwards into the far future. */
	if (unlikely(!*last_ns))
		*last_ns = time > RFX_EMA_DECAY_PERIOD_NS ?
			time - RFX_EMA_DECAY_PERIOD_NS : 0;
	delta_ns = rfx_elapsed(time, *last_ns);

	steps = (unsigned int)min_t(u64, delta_ns / RFX_EMA_DECAY_PERIOD_NS,
				    RFX_EMA_MAX_STEPS);
	if (!steps)
		return old;	/* sub-period: hold, keep the remainder */

	/* Advance by periods CONSUMED, not to @time: dropping the remainder
	 * stretches the time constant. At the step cap the excess is discarded
	 * by design, so the reference goes to @time. */
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

/*
 * Request slightly more capacity than measured, so we land on an OPP with room
 * to spare. Gaming uses a phased linear ramp; daily gets none -- see the note
 * above RFX_HEADROOM_GAMING.
 */
static unsigned long rfx_apply_headroom(unsigned long util, unsigned long max_cap,
					bool gaming)
{
	unsigned int upct;

	if (!max_cap || util >= max_cap)
		return max_cap;

	upct = (unsigned int)(util * 100 / max_cap);
	if (upct >= (gaming ? RFX_SAT_TO_MAX_GAMING_PCT :
			      RFX_SAT_TO_MAX_DAILY_PCT))
		return max_cap;

	if (!gaming || upct <= RFX_HEADROOM_GAMING_GATE)
		return util;

	/* One expression: truncating to whole percent first would drop the bottom
	 * of the ramp. */
	return min(util + max_cap * RFX_HEADROOM_GAMING *
			  (upct - RFX_HEADROOM_GAMING_GATE) /
			  ((100 - RFX_HEADROOM_GAMING_GATE) * 100),
		   max_cap);
}

/*
 * Daily ceiling for a tier, slid linearly from its base cap to its sustained
 * cap between DROP and LIFT percent of demand. Both endpoints are the values
 * the profile was tuned around; only the path between them is new. A binary
 * latch made the ceiling STEP between the same two values, and the daily band
 * has no descent filter, so the clock followed that step in both directions --
 * a square wave on the interactive workload the profile is judged on.
 * Monotonic: BUILD_BUG_ONs pin DROP < LIFT and base <= sustained.
 */
static unsigned int rfx_daily_cap_pct(unsigned int demand_pct, unsigned int base,
				      unsigned int lift, unsigned int drop,
				      unsigned int sustained)
{
	if (demand_pct <= drop)
		return base;
	if (demand_pct >= lift)
		return sustained;
	return base + (sustained - base) * (demand_pct - drop) / (lift - drop);
}

/* Threshold step test, overflow-safe: prev near 100 must not wrap. */
static bool rfx_pct_step_reached(unsigned int now, unsigned int prev,
				 unsigned int delta)
{
	if (prev >= 100 || delta > 100 - prev)
		return false;
	return now >= prev + delta;
}

/*
 * Arm the gaming frame boost on a demand step into the ARM region.
 *
 * Same resampled-reference safety as the burst it replaces in spirit: a load
 * that has settled high re-seeds the reference to its own level every sample,
 * so its step is zero and it cannot re-arm -- the window is transient by
 * construction, never a resting floor. Only an actual rise arms, so this
 * costs the onset of a frame burst and nothing after it.
 *
 * boost_armed consumes the step: without it the same rise re-arms on every
 * evaluation until the next resample. It clears when demand falls back under
 * CLEAR or the window lapses, so one crossing yields one window.
 */
static void rfx_gaming_frame_boost(struct rfx_policy *p,
				   unsigned int demand_pct, u64 time)
{
	if (rfx_elapsed(time, p->boost_sample_ns) >=
	    RFX_G_FRAME_BOOST_SAMPLE_NS) {
		p->boost_sample_ns = time;
		p->boost_prev_demand_pct = demand_pct;
		return;
	}

	if (p->boost_armed ||
	    demand_pct < RFX_G_FRAME_BOOST_ARM_PCT ||
	    !rfx_pct_step_reached(demand_pct, p->boost_prev_demand_pct,
				  RFX_G_FRAME_BOOST_DELTA_PCT))
		return;

	p->boost_armed = true;
	p->boost_sample_ns = time;
	p->boost_prev_demand_pct = demand_pct;
	p->gaming_boost_end_ns = time + RFX_G_FRAME_BOOST_NS;
	p->gaming_boost_pct = RFX_G_FRAME_BOOST_MAX_PCT;
}

/* Disarm once demand has clearly left the interaction region or the window
 * has run out. */
static void rfx_gaming_frame_boost_clear(struct rfx_policy *p,
					 unsigned int demand_pct, u64 time)
{
	if (demand_pct < RFX_G_FRAME_BOOST_CLEAR_PCT ||
	    time >= p->gaming_boost_end_ns) {
		p->boost_armed = false;
		p->gaming_boost_pct = 0;
	}
}

/*
 * Daily UI boost: one additive window per demand step, then a cooldown.
 *
 * The cooldown is the whole reason this replaced the cap-release burst: a
 * duty-cycling load steps up once per cycle, and without the cooldown it
 * would re-open the window every cycle and hold the clock near the ceiling
 * permanently. ARM sits above the band idle scrolling lives in; CLEAR ends
 * the armed state once the interaction is over.
 */
static void rfx_daily_ui_boost(struct rfx_policy *p,
			       unsigned int demand_pct, u64 time)
{
	if (rfx_elapsed(time, p->daily_ui_sample_ns) >= RFX_D_UI_SAMPLE_NS) {
		p->daily_ui_sample_ns = time;
		p->daily_ui_prev_demand_pct = demand_pct;
		return;
	}

	if (time < p->daily_ui_cooldown_end_ns ||
	    p->daily_ui_armed ||
	    demand_pct < RFX_D_UI_ARM_PCT ||
	    !rfx_pct_step_reached(demand_pct, p->daily_ui_prev_demand_pct,
				  RFX_D_UI_STEP_PCT))
		return;

	p->daily_ui_armed = true;
	p->daily_ui_sample_ns = time;
	p->daily_ui_prev_demand_pct = demand_pct;
	p->daily_ui_boost_end_ns = time + RFX_D_UI_BOOST_NS;
	p->daily_ui_cooldown_end_ns = time + RFX_D_UI_COOLDOWN_NS;
}

/* Disarm once the interaction has left the region, or once the spacing the
 * cooldown defines has elapsed. Against the COOLDOWN deadline, not the window:
 * the window is the shorter of the two by invariant, so clearing on it left the
 * flag meaning nothing and the cooldown as the only gate -- and any later
 * shortening of the cooldown would then have silently turned one window per
 * interaction into a duty cycle. */
static void rfx_daily_ui_boost_clear(struct rfx_policy *p,
				     unsigned int demand_pct, u64 time)
{
	if (demand_pct <= RFX_D_UI_CLEAR_PCT ||
	    time >= p->daily_ui_cooldown_end_ns) {
		p->daily_ui_armed = false;
	}
}

/*
 * Down-only target hysteresis. The reference is p->next_freq (last COMMITTED
 * frequency), NOT a stored target: a stored-target reference makes every
 * suppressed update re-store its own target, so the reference only ever
 * ratchets to the high-water mark and the clock never comes down through a
 * sub-band descent -- a floor by construction. Against next_freq, a real
 * descent crosses the band within one commit and lands; only noise smaller
 * than the band is held, which is the point. Both profiles use it: gaming
 * wide (frame pacing), daily narrow (scroll OPP oscillation).
 */
static unsigned int rfx_target_hysteresis(struct rfx_policy *p,
					  unsigned int target,
					  unsigned int fceil, bool gaming)
{
	unsigned int band;

	band = rfx_pct(fceil, gaming ? RFX_G_TARGET_DOWN_DEADBAND_PCT :
				       RFX_D_TARGET_DOWN_DEADBAND_PCT);

	if (target < p->next_freq &&
	    p->next_freq - target < band)
		return p->next_freq;
	return target;
}

/* ===================================================================== */
/* Thermal emergency clamp (final clamp)                                 */
/* ===================================================================== */

/*
 * Last clamp before OPP resolution. Flat and latched. Normal throttling is the
 * platform's job; this only fires at RFX_TEMP_EMERGENCY_MC.
 */
static unsigned int rfx_thermal_clamp(unsigned int freq, unsigned int fmax)
{
	int pct = atomic_read(&rfx_emergency_cap_pct);

	if (likely(pct >= 100))
		return freq;

	return min(freq, rfx_pct(fmax, pct));
}

/* ===================================================================== */
/* Frequency decision                                                    */
/* ===================================================================== */

/*
 * Pure-ish frequency selection from a (smoothed) util value. Order:
 *   1. headroom -> base freq from util/capacity
 *   2. profile shaping (gaming band + bounded slew OR daily caps/floors)
 *   3. thermal step clamp (final ceiling)
 *   4. resolve to a real OPP (cached to skip redundant table walks)
 */
static unsigned int rfx_target_freq(struct rfx_policy *p, unsigned long util,
				    unsigned long max_cap, u64 time, bool gaming)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int fmin = pol->cpuinfo.min_freq;
	bool little = p->is_little;
	bool prime = p->is_prime;
	unsigned int freq;
	unsigned long raw_util = util;	/* demand before headroom inflation */
	/* One ceiling for both bands: every floor/cap below is a percentage of
	 * THIS, so the shape slides down under a clamp instead of colliding with
	 * it. fmax is the unthrottled baseline -- on MTK it sits below
	 * cpuinfo.max_freq. */
	unsigned int fmax;
	unsigned int fceil;
	unsigned int fceil_pct;

	if (unlikely(!pol->cpuinfo.max_freq || !max_cap))
		return pol->cur;

	fceil_pct = rfx_thermal_headroom_pct(p, max_cap, &fmax);
	if (unlikely(!fmax))
		return pol->cur;
	fceil = rfx_pct(fmax, fceil_pct);
	fceil = clamp(fceil, fmin, fmax);

	util = rfx_apply_headroom(util, max_cap, gaming);

	/* arch capacity 1024 is defined against cpuinfo max; only the
	 * percentage shape uses fmax. */
	freq = (unsigned int)((u64)pol->cpuinfo.max_freq * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		bool warmup_active;
		unsigned int warmup_ramp_pct;
		unsigned int fl, warmup_fl, demand_pct;
		u64 down_step, slew_ns;

		/*
		 * Demand before headroom inflation. CAVEAT: raw_util already
		 * carries the 25% DVFS margin, so demand_pct reads ~1.25x real
		 * demand -- every threshold was tuned WITH that skew.
		 */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		/* Cooling latch is updated HERE, above the warmup window: that
		 * window asks the same question this latch answers -- is the
		 * platform taking capacity -- and must read the latch rather than
		 * keep a second raw threshold of its own. Floor relief below still
		 * reads exactly the value it did before. */
		if (fceil_pct < RFX_G_COOL_ENTER_PCT)
			p->thermal_cooling = true;
		else if (fceil_pct >= RFX_G_COOL_EXIT_PCT)
			p->thermal_cooling = false;

		/* Deferred arm first: a crossing that would also trip the risk
		 * path must find a live, properly anchored window to extend --
		 * arming risk first would anchor the cap to a zero start. */
		if (!little)
			rfx_warmup_arm(p, demand_pct, time);

		/* Little never renders, so a warmup floor there is heat plus
		 * capacity EAS then packs work onto -- it neither arms a window
		 * nor rides one. Arming runs before the window is read, so an
		 * edge takes effect on this evaluation rather than the next. */
		if (!little)
			rfx_risk_rearm(p, demand_pct,
				       rfx_pct(fceil, RFX_G_WARMUP_FLOOR_PCT),
				       time);

		warmup_active = !little && p->gaming_warmup_end_ns &&
				time < p->gaming_warmup_end_ns;

		/* Adaptive warmup: extend while Big/Prime demand holds above
		 * EXTEND_PCT (absolute cap MAX_NS from arm), release early below
		 * RELEASE_PCT for RELEASE_NS. No extension once the limiter is
		 * taking capacity: the floor buys spawn headroom, and riding it
		 * through a throttle adds heat exactly where fceil is falling.
		 *
		 * That condition reads the cooling LATCH, not fceil_pct directly.
		 * A raw `fceil_pct >= COOL_EXIT` here was unreachable on a platform
		 * whose limiter reports continuously: any nonzero thermal_pressure
		 * pulls fceil_pct under the exit threshold, so the window could
		 * never extend there while it extended freely on a platform that
		 * throttles in steps. Same constant, opposite behaviour per SoC. */
		if (warmup_active) {
			if (demand_pct >= RFX_GAMING_WARMUP_EXTEND_PCT &&
			    !p->thermal_cooling) {
				u64 cap = p->gaming_warmup_start_ns +
					  RFX_GAMING_WARMUP_MAX_NS;
				u64 ext = time + RFX_EMA_DECAY_PERIOD_NS * 4;

				if (ext > cap)
					ext = cap;
				if (ext > p->gaming_warmup_end_ns)
					p->gaming_warmup_end_ns = ext;
				p->warmup_low_demand_since_ns = 0;
			} else if (demand_pct < RFX_GAMING_WARMUP_RELEASE_PCT) {
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

		/* Baseline floor per role; warmup_fl is the same value on Little
		 * so the lift below is a no-op there. */
		if (prime)
			fl = rfx_pct(fceil, RFX_G_PRIME_FLOOR_PCT);
		else if (!little)	/* Big: demand-tracked, uncapped */
			fl = rfx_pct(fceil, RFX_G_BIG_FLOOR_PCT);
		else			/* Little: compositor / audio / input */
			fl = rfx_pct(fceil, RFX_G_LITTLE_FLOOR_PCT);
		warmup_fl = little ? fl : rfx_pct(fceil, RFX_G_WARMUP_FLOOR_PCT);

		/* Once the platform has taken capacity, holding floors defeats
		 * thermal relief and makes the HW limiter sawtooth the clock.
		 * Latch itself is updated above, before the warmup window reads it. */
		if (p->thermal_cooling) {
			unsigned int steady = rfx_pct(fceil,
						      RFX_G_COOL_STEADY_FLOOR_PCT);
			unsigned int depth;

			/* Relief scales with throttle depth: 0 at ENTER, full at
			 * DEEP, so the clock walks down with the ceiling. */
			depth = fceil_pct >= RFX_G_COOL_ENTER_PCT ? 0 :
				fceil_pct <= RFX_G_COOL_DEEP_PCT ? 100 :
				(RFX_G_COOL_ENTER_PCT - fceil_pct) * 100 /
				(RFX_G_COOL_ENTER_PCT - RFX_G_COOL_DEEP_PCT);

			if (fl > steady)
				fl -= (fl - steady) * depth / 100;
			if (warmup_fl > steady)
				warmup_fl -= (warmup_fl - steady) * depth / 100;
		}

		/*
		 * Bounded slew, measured from last commit (not last eval) so
		 * budget accumulates correctly, capped at the down-rate period
		 * or budget grows while the gate blocks commits. Floors only
		 * raise, so order-independent.
		 */
		slew_ns = rfx_elapsed(time, max(p->last_upfreq_time,
						p->last_downfreq_time));
		slew_ns = min_t(u64, slew_ns,
				(u64)RFX_GAMING_DOWN_US * NSEC_PER_USEC);
		down_step = (u64)rfx_pct(fceil, RFX_GAMING_DOWN_PCT_PER_2MS) *
			    slew_ns / (2 * NSEC_PER_MSEC);
		if (down_step < fceil && p->next_freq > (unsigned int)down_step &&
		    freq < p->next_freq - (unsigned int)down_step)
			freq = p->next_freq - (unsigned int)down_step;

		warmup_ramp_pct = rfx_update_warmup_ramp(p, warmup_active, time);

		/* Idle latch: enter below GATE, leave only above GATE_EXIT.
		 * Role-independent; every lift reads this, never demand_pct. */
		if (demand_pct < RFX_G_FLOOR_GATE_PCT)
			p->floor_gated = true;
		else if (demand_pct >= RFX_G_FLOOR_GATE_EXIT_PCT)
			p->floor_gated = false;

		if (p->floor_gated)
			fl = rfx_pct(fceil, RFX_G_IDLE_FLOOR_PCT);
		else if (warmup_ramp_pct > 0 && warmup_fl > fl)
			fl = fl + (warmup_fl - fl) * warmup_ramp_pct / 100;

		if (freq < fl)
			freq = fl;

		/* Frame-paced boost: additive, before the thermal clamp so the
		 * emergency cap stays the last word, and suppressed while the
		 * platform is taking capacity -- boosting into a falling ceiling
		 * is heat for nothing. */
		rfx_gaming_frame_boost(p, demand_pct, time);
		rfx_gaming_frame_boost_clear(p, demand_pct, time);
		if (p->gaming_boost_pct && time < p->gaming_boost_end_ns &&
		    !p->thermal_cooling) {
			unsigned int boost_fl;

			boost_fl = freq + rfx_pct(fceil, p->gaming_boost_pct);
			freq = min(boost_fl, fceil);
		}
	} else {
		unsigned int cap, cap_pct, demand_pct;

		/* Raw demand, before headroom: post-headroom util is stepped by
		 * tier, so a crossing jumps the value with no load change. Same
		 * 1.25x skew as the gaming band. */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		/* One cap per tier, slid between its two endpoints rather than
		 * stepped between them. Big/Prime share one band. No floors:
		 * demand plus PELT already hold the clock where the work is. */
		if (little)
			cap_pct = rfx_daily_cap_pct(demand_pct,
					RFX_D_LITTLE_CAP_PCT,
					RFX_D_LITTLE_LIFT_PCT,
					RFX_D_LITTLE_DROP_PCT,
					RFX_D_LITTLE_SUSTAINED_CAP_PCT);
		else if (prime)
			cap_pct = rfx_daily_cap_pct(demand_pct,
					RFX_D_PRIME_CAP_PCT,
					RFX_D_BIG_LIFT_PCT,
					RFX_D_BIG_DROP_PCT,
					RFX_D_PRIME_SUSTAINED_CAP_PCT);
		else
			cap_pct = rfx_daily_cap_pct(demand_pct,
					RFX_D_BIG_CAP_PCT,
					RFX_D_BIG_LIFT_PCT,
					RFX_D_BIG_DROP_PCT,
					RFX_D_BIG_SUSTAINED_CAP_PCT);

		/* One window per demand step into the interaction region,
		 * cooldown-gated; replaces the cap-release burst -- running both
		 * would double-lift the same interaction. */
		rfx_daily_ui_boost(p, demand_pct, time);
		rfx_daily_ui_boost_clear(p, demand_pct, time);

		/* Boost lifts the CEILING, not the clock: additive-after-clamp
		 * spent the window even when demand sat well under the cap, where
		 * the lift bought nothing. As a cap lift it only exists where the
		 * band actually binds -- the top of a scroll or an inflate -- and
		 * the clock still has to be demanded to reach it. Suppressed while
		 * the limiter is taking capacity, same question the gaming cooling
		 * latch answers -- read live from fceil_pct here, because that
		 * latch is only ever written in the gaming branch. */
		if (time < p->daily_ui_boost_end_ns &&
		    fceil_pct >= RFX_G_COOL_ENTER_PCT)
			cap_pct = min(cap_pct + RFX_D_UI_BOOST_PCT, 100U);

		cap = rfx_pct(fceil, cap_pct);

		if (freq > cap)
			freq = cap;
	}

	/* Down-only hysteresis before the clamps: rises pass untouched, a
	 * sub-band descent holds one commit so util noise cannot oscillate the
	 * OPP. Against next_freq, so a real descent still lands. Band is wider
	 * in gaming (frame pacing) than daily (scroll oscillation). */
	freq = rfx_target_hysteresis(p, freq, fceil, gaming);
	freq = rfx_thermal_clamp(freq, fceil);
	freq = clamp(freq, fmin, fceil);

	/*
	 * The raw request becomes the cache key only once committed. Writing it
	 * here poisoned the cache whenever the rate gate rejected a commit: the
	 * next tick hit the cache and returned a stale next_freq.
	 */
	if (freq == p->cached_raw_freq && !p->need_freq_update)
		return p->next_freq;
	p->pending_raw_freq = freq;
	return cpufreq_driver_resolve_freq(pol, freq);
}

/* ===================================================================== */
/* IO-wait boost (unchanged behaviour from schedutil lineage)            */
/* ===================================================================== */

static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time, bool set)
{
	s64 delta_ns = time - rfx_c->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	rfx_c->iowait_boost = set ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time, unsigned int flags)
{
	bool set = flags & SCHED_CPUFREQ_IOWAIT;
	unsigned long max_cap;
	unsigned int cap;

	/* Reset boost if the CPU has been idle long enough. */
	if (rfx_c->iowait_boost && rfx_iowait_reset(rfx_c, time, set))
		return;

	/* Boost only tasks waking up after IO. */
	if (!set)
		return;

	/* Double at most once per boost consumption. */
	if (rfx_c->iowait_boost_pending)
		return;
	rfx_c->iowait_boost_pending = true;

	/*
	 * Per-cluster ceiling: Little modest, Big/Prime enough for the CPU side
	 * of a completion at the V/f knee. No gaming tier: it enters util before
	 * the 25% margin, so it outranked every gaming floor while blind to the
	 * idle gate and the cooling band.
	 */
	if (rfx_c->iowait_boost) {
		max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
		cap = rfx_cap_is_little(max_cap) ? SCHED_CAPACITY_SCALE / 6
						 : SCHED_CAPACITY_SCALE / 4;
		rfx_c->iowait_boost = min_t(unsigned int,
					    rfx_c->iowait_boost << 1, cap);
		return;
	}
	rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time,
				      unsigned long max_cap)
{
	/* Fast path: no boost active, skip all computation */
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
	/* RT is bounded for every profile. See rfx_get_util_gki510(). */
	rfx_get_util_gki510(rfx_c->cpu, boost, true, &rfx_c->util,
			    &rfx_c->bwmin);
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

/* ===================================================================== */
/* Rate limiting                                                         */
/* ===================================================================== */

/* Set the active down-rate-limit for this update (long while gaming). */
static inline void rfx_set_down_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->down_rate_delay_ns = (s64)RFX_GAMING_DOWN_US * NSEC_PER_USEC;
	else
		p->down_rate_delay_ns =
			(s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
}

/* up-rate-limit: ZERO while gaming, every cluster, no exception -- a nonzero
 * up-rate inside a frame budget is a frame-time tax, i.e. an FPS cap. */
static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns =
			(s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

/* Eval delay for this update. Set BEFORE rfx_should_update_freq, so it may only
 * depend on state known without util. */
static inline void rfx_set_eval_delay(struct rfx_policy *p, bool gaming)
{
	if (!gaming) {
		p->freq_update_delay_ns =
			(s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
		return;
	}

	p->freq_update_delay_ns = (s64)(p->policy->fast_switch_enabled ?
					RFX_FAST_RATE_US :
					RFX_SLOW_SWITCH_RATE_US) * NSEC_PER_USEC;
}

/*
 * Evaluation gate. Measures from last_eval_time (stamped on every evaluation),
 * not from last commit -- rfx_commit_freq() skips stamping when freq is
 * unchanged, so commit-based gating is permanently open when gaming floors pin
 * the clock.
 */
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

/* Commit next_freq subject to directional up/down rate limits. */
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
		p->last_downfreq_time = time;
	} else {
		delta = (s64)(time - p->last_upfreq_time);
		if (p->up_rate_delay_ns > 0 && delta < p->up_rate_delay_ns)
			return false;
		p->last_upfreq_time = time;
	}

	/*
	 * Commit accepted: the raw request behind it is now the valid cache key.
	 * Promoting here keeps a gate-rejected update from being dropped.
	 */
	p->cached_raw_freq = p->pending_raw_freq;
	p->next_freq = next_freq;
	return true;
}

/* ===================================================================== */
/* Update hooks                                                          */
/* ===================================================================== */

/*
 * Largest capacity in the policy. Within a cluster every CPU reads the same
 * value, so this is normally the triggering CPU's own capacity -- but a shared
 * policy must not let the divisor depend on which member happened to tick.
 */
static unsigned long rfx_policy_max_cap(struct cpufreq_policy *policy)
{
	unsigned long cap, max_cap = 0;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus) {
		cap = arch_scale_cpu_capacity(cpu);
		if (cap > max_cap)
			max_cap = cap;
	}
	return max_cap;
}

static unsigned int rfx_next_freq(struct rfx_cpu *rfx_c, u64 time, bool gaming)
{
	struct rfx_policy *p = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = rfx_policy_max_cap(policy);
	unsigned long max_util = 0;
	unsigned int j;

	/*
	 * Aggregate max util across the policy's CPUs, then filter once. The EMA
	 * lives on the policy: a per-CPU filter let the committed value flip with
	 * whichever CPU ticked last -- jitter with no change in load. Normalise
	 * by the policy's own ceiling, not the triggering CPU's capacity: util
	 * is aggregated across the policy, so the divisor must be too.
	 */
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

	return rfx_target_freq(p, p->filt_util, max_cap, time, gaming);
}

/*
 * One hook for every policy, single-CPU or shared: rfx_reset_all_policies()
 * writes filt_util, max_seen and every latch from a sysfs write on another
 * CPU, so every path runs under update_lock.
 */
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
	rfx_set_eval_delay(p, gaming);

	if (rfx_should_update_freq(p, time)) {
		p->last_eval_time = time;
		next_f = rfx_next_freq(rfx_c, time, gaming);
		if (rfx_commit_freq(p, time, next_f)) {
			/* Inside update_lock: the call may not run twice in
			 * parallel for one policy. */
			if (p->policy->fast_switch_enabled) {
				cpufreq_driver_fast_switch(p->policy,
							   p->next_freq);
			} else if (p->work_in_progress) {
				/* Latest request already queued behind the
				 * running worker: flag it so the worker
				 * re-runs instead of dropping the request
				 * that landed mid-run. */
				p->work_requeue = true;
			} else {
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
	bool again;

	raw_spin_lock_irqsave(&p->update_lock, flags);
	freq = p->next_freq;
	/* Keep work_in_progress while requeueing: releasing it here and
	 * re-taking under a second lock section would race an update that
	 * grabs the token between the two, double-queueing the work. The
	 * token returns with the requeue itself. */
	again = p->work_requeue;
	p->work_requeue = false;
	if (!again)
		p->work_in_progress = false;
	raw_spin_unlock_irqrestore(&p->update_lock, flags);

	mutex_lock(&p->work_lock);
	/* __cpufreq_driver_target, not cpufreq_driver_target: the latter takes
	 * policy->rwsem, and rfx_limits() runs holding that rwsem while it takes
	 * work_lock -- opposite order, deadlock. Reachable on any driver without
	 * fast_switch (MTK). */
	__cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);

	/* A request landed while this worker ran: requeue for the latest
	 * next_freq instead of dropping it, then hand the token back. */
	if (again) {
		raw_spin_lock_irqsave(&p->update_lock, flags);
		p->work_in_progress = false;
		raw_spin_unlock_irqrestore(&p->update_lock, flags);
		kthread_queue_work(&p->worker, &p->work);
	}
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *p = container_of(irq_work, struct rfx_policy, irq_work);

	kthread_queue_work(&p->worker, &p->work);
}

/* ===================================================================== */
/* Thermal poller (slow path, may sleep -> never in the util hook)       */
/* ===================================================================== */

#ifdef CONFIG_THERMAL
static struct thermal_zone_device *rfx_tz;
static char rfx_tz_name[THERMAL_NAME_LENGTH];
#endif
static struct delayed_work rfx_thermal_work;

/*
 * Emergency cap changed: force the next evaluation on every policy, or a
 * quiet cluster keeps its old frequency until something wakes it. need_freq_
 * update is consumed by rfx_should_update_freq(); it does not call the driver
 * from here -- the commit path owns that, in the right locking context.
 */
static void rfx_mark_all_policies_dirty(void)
{
	struct rfx_policy *p;
	unsigned long flags, pflags;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);

	list_for_each_entry(p, &rfx_policy_list, gov_node) {
		raw_spin_lock_irqsave(&p->update_lock, pflags);
		p->need_freq_update = true;
		raw_spin_unlock_irqrestore(&p->update_lock, pflags);
	}

	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
}

static void rfx_thermal_fn(struct work_struct *w)
{
	int t_mc = 0;
	bool have = false;
	unsigned int delay_ms;

#ifdef CONFIG_THERMAL
	if (READ_ONCE(rfx_tz) && !thermal_zone_get_temp(READ_ONCE(rfx_tz), &t_mc))
		have = true;
#endif
	if (!have) {
		t_mc = atomic_read(&rfx_temp_mc);
		if (t_mc > 0)
			have = true;
	}

	/* Latched net, 7C hysteresis: trip once, hold until the die cools,
	 * release once. */
	if (have) {
		if (atomic_read(&rfx_emergency_cap_pct) >= 100) {
			if (t_mc >= RFX_TEMP_EMERGENCY_MC) {
				atomic_set(&rfx_emergency_cap_pct,
					   RFX_EMERGENCY_CAP_PCT);
				rfx_mark_all_policies_dirty();
				pr_warn_ratelimited("vorpal: thermal emergency %d mC, cap %d%%\n",
						    t_mc, RFX_EMERGENCY_CAP_PCT);
			}
		} else if (t_mc <= RFX_TEMP_EMERGENCY_CLEAR_MC) {
			atomic_set(&rfx_emergency_cap_pct, 100);
			rfx_mark_all_policies_dirty();
			pr_info("vorpal: thermal emergency cleared %d mC\n", t_mc);
		}
	} else {
		/* No source configured: the poll can never do anything, so stop
		 * re-arming. Both sysfs stores re-arm when a source appears. */
		atomic_set(&rfx_emergency_cap_pct, 100);
		return;
	}


	if (rfx_gaming_enabled())
		delay_ms = RFX_THERMAL_POLL_GAMING_MS;
	else if (t_mc >= RFX_TEMP_WARM_MC)
		delay_ms = RFX_THERMAL_POLL_WARM_MS;
	else
		delay_ms = RFX_THERMAL_POLL_IDLE_MS;
	queue_delayed_work(system_power_efficient_wq, &rfx_thermal_work,
			   msecs_to_jiffies(delay_ms));
}

/* ===================================================================== */
/* sysfs                                                                 */
/* ===================================================================== */

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

/* sysfs rate-limit bound: a value past 1s is not a rate limit, and an
 * unbounded (s64)val * NSEC_PER_USEC invite. */
#define RFX_MAX_RATE_LIMIT_US		1000000U

static int rfx_validate_rate(unsigned int val)
{
	return val <= RFX_MAX_RATE_LIMIT_US ? 0 : -ERANGE;
}

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
	if (rfx_validate_rate(val))
		return -ERANGE;
	t->rate_limit_us = val;
	/* No push into every policy: each update calls rfx_set_eval_delay()
	 * before the gate reads freq_update_delay_ns. */
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
	if (rfx_validate_rate(val))
		return -ERANGE;
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
	if (rfx_validate_rate(val))
		return -ERANGE;
	t->down_rate_limit_us = val;
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

/*
 * Clear every transient latch and window on one policy. Called on both profile
 * edges: neither profile's residue may shape the other. Caller holds
 * p->update_lock.
 */
static void rfx_reset_policy_locked(struct rfx_policy *p)
{
	p->warmup_ramp_pct = 0;
	p->warmup_ramp_last_ns = 0;
	p->gaming_warmup_end_ns = 0;
	p->gaming_warmup_start_ns = 0;
	p->gaming_warmup_pending = false;
	p->thermal_cooling = false;
	p->floor_gated = false;
	p->warmup_low_demand_since_ns = 0;
	p->risk_high = false;
	p->gaming_boost_end_ns = 0;
	p->gaming_boost_pct = 0;
	p->boost_prev_demand_pct = 0;
	p->boost_sample_ns = 0;
	p->boost_armed = false;
	p->daily_ui_boost_end_ns = 0;
	p->daily_ui_cooldown_end_ns = 0;
	p->daily_ui_prev_demand_pct = 0;
	p->daily_ui_sample_ns = 0;
	p->daily_ui_armed = false;
	p->need_freq_update = true;
}

/* Reset transient residue on every live policy (all clusters). */
static void rfx_reset_all_policies(void)
{
	struct rfx_policy *p;
	unsigned long flags, pflags;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);

	list_for_each_entry(p, &rfx_policy_list, gov_node) {
		raw_spin_lock_irqsave(&p->update_lock, pflags);
		rfx_reset_policy_locked(p);
		/* Do not carry saturated gaming demand into the daily profile. */
		p->filt_util = 0;
		p->last_ema_ns = 0;
		raw_spin_unlock_irqrestore(&p->update_lock, pflags);
	}
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
}

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", rfx_gaming_enabled());
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
		/* Drop the 100ms gaming thermal poll back to idle rate. */
		mod_delayed_work(system_power_efficient_wq, &rfx_thermal_work,
				 msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));
	} else {
		struct rfx_policy *p;
		unsigned long flags, pflags;

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			rfx_reset_policy_locked(p);
			/* Warmup is pending, not running: the game does not
			 * exist yet at the moment of this write. */
			p->gaming_warmup_pending = true;
			raw_spin_unlock_irqrestore(&p->update_lock, pflags);
		}
		spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

		/* Sample temperature sooner once gaming begins. */
		mod_delayed_work(system_power_efficient_wq, &rfx_thermal_work,
				 msecs_to_jiffies(RFX_THERMAL_POLL_GAMING_MS));
	}
	return count;
}
static struct governor_attr gaming_mode = __ATTR_RW(gaming_mode);

static ssize_t temp_mc_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_temp_mc));
}
static ssize_t temp_mc_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&rfx_temp_mc, val);
	/* Re-arm: the poller stops itself while no source is configured. */
	if (val > 0)
		mod_delayed_work(system_power_efficient_wq, &rfx_thermal_work, 0);
	return count;
}
static struct governor_attr temp_mc = __ATTR_RW(temp_mc);

static ssize_t thermal_zone_show(struct gov_attr_set *attr_set, char *buf)
{
#ifdef CONFIG_THERMAL
	return sprintf(buf, "%s\n", rfx_tz_name[0] ? rfx_tz_name : "(none)");
#else
	return sprintf(buf, "(no CONFIG_THERMAL)\n");
#endif
}
static ssize_t thermal_zone_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
#ifdef CONFIG_THERMAL
	struct thermal_zone_device *tz;
	char name[THERMAL_NAME_LENGTH];

	strscpy(name, buf, sizeof(name));
	strim(name);
	tz = thermal_zone_get_zone_by_name(name);
	if (IS_ERR(tz))
		return -EINVAL;
	WRITE_ONCE(rfx_tz, tz);
	strscpy(rfx_tz_name, name, sizeof(rfx_tz_name));
	/* Re-arm: the poller stops itself while no source is configured. */
	mod_delayed_work(system_power_efficient_wq, &rfx_thermal_work, 0);
	return count;
#else
	return -ENODEV;
#endif
}
static struct governor_attr thermal_zone = __ATTR_RW(thermal_zone);

static struct attribute *rfx_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&temp_mc.attr,
	&thermal_zone.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx);

static void rfx_tunables_free(struct kobject *kobj)
{
	kfree(to_rfx_tunables(rfx_to_gov_attr_set(kobj)));
}

/*
 * One attr set shape for every cluster. gaming_mode/temp_mc/thermal_zone are
 * global state; gaming_mode is added at start() rather than declared here --
 * see rfx_gaming_attr().
 */
static struct kobj_type rfx_ktype = {
	.default_groups = rfx_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};

/*
 * Place gaming_mode on the top cluster only. Not in rfx_attrs[]: capacities
 * are unnormalized when the first kobject is built. Removal is per-policy
 * only -- with one shared kobject a non-top policy stopping would delete the
 * only activation path from a running top one.
 */
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

/*
 * Top cluster by highest possible CPU, not by capacity: start() for policy0
 * runs before capacities normalize (all read 1024). CPU numbering ascends with
 * capacity on every DynamIQ part. related_cpus -- cpus holds only online.
 */
static bool rfx_hosts_gaming_attr(struct cpufreq_policy *policy)
{
	return cpumask_test_cpu(cpumask_last(cpu_possible_mask),
				policy->related_cpus);
}

static struct cpufreq_governor vorpal_gov;

/* ===================================================================== */
/* Allocation / kthread                                                  */
/* ===================================================================== */

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

/*
 * DVFS worker for the slow path (any driver without fast_switch -- notably
 * mediatek-cpufreq, so every commit on MTK goes through here).
 *
 * SCHED_DEADLINE + SCHED_FLAG_SUGOV, not SCHED_FIFO: a DL task needs the
 * clock this worker is about to raise, so it must not be preemptible by one,
 * and an RT worker shares rt_rq bandwidth a runaway vendor RT thread can
 * throttle. The flag is the upstream escape hatch (dl_entity_is_special):
 * DL class, fake bandwidth, no admission control, still allowed to sleep.
 */
static int rfx_kthread_create(struct rfx_policy *p)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = p->policy;
	int ret;

	/* Deferred-path only, but initialise before the fast-switch return: a
	 * zeroed mutex is a crash, not a warning. */
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
	/* Bind to the cluster only when DVFS must run on it. Never
	 * set_cpus_allowed_ptr() here -- on a DL task that lands in
	 * set_cpus_allowed_dl()->__dl_sub(), which is not special-cased. */
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

/* ===================================================================== */
/* Governor callbacks                                                    */
/* ===================================================================== */

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

	/* Provisional: capacities may not be normalized yet on the notifier
	 * path, so start() re-derives the roles. */
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
	if (!count) {
		rfx_clear_global_tunables();
		atomic_set(&rfx_gaming, 0);
	}
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

	/* Re-derive roles here: on the cpufreq-notifier topology path
	 * capacities are normalized only after the last policy is created, so
	 * an early init() sees 1024 everywhere. */
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
	/* Unthrottled baseline; only ratchets up. */
	p->max_seen = policy->max;
	p->work_in_progress = false;
	p->limits_changed = false;
	p->filt_util = 0;
	p->last_ema_ns = 0;

	/* Not yet on the policy list, so nothing can race the hook here. */
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

/* ===================================================================== */
/* gaming_mode ownership                                                 */
/* ===================================================================== */

/*
 * gaming_mode is USER-OWNED: nothing in this driver ever writes it, and there
 * is deliberately no PM/suspend auto-clear.
 */

/*
 * Self-check for the two time-domain helpers: an unsigned rq_clock delta
 * wrapping on a sibling's stamp, and a periodic decay advancing to @time
 * instead of by the periods consumed.
 */
static void __init rfx_selfcheck(void)
{
	const u64 t = 1ULL << 40;
	struct rfx_policy p = { };
	unsigned long gate, over;
	unsigned int d, c, prev;
	u64 end;
	u64 ns;

	/* Rise is instant and re-seeds the reference. */
	ns = 0;
	WARN_ON(rfx_ema(100, 200, t, &ns, true) != 200 || ns != t);

	/* Daily falls instantly; gaming holds inside one period. */
	ns = t;
	WARN_ON(rfx_ema(200, 100, t, &ns, false) != 100);
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) != 1000);

	/* One period removes 1/DIVISOR of the error, reference advances by it. */
	ns = t - RFX_EMA_DECAY_PERIOD_NS;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) !=
		1000 - 1000 / RFX_EMA_GAMING_DIVISOR || ns != t);

	/* Stamp from a sibling CPU reading ahead must decay nothing, not wrap. */
	ns = t + NSEC_PER_MSEC;
	WARN_ON(rfx_ema(1000, 0, t, &ns, true) != 1000);

	/* Gaming headroom ramp: nothing at the gate, something above it (a ramp
	 * truncated to whole percent would give zero there), never past
	 * capacity. Two percent over, since one lands back on the gate once
	 * rfx_pct and the upct divide have both truncated. */
	gate = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE);
	over = rfx_pct(SCHED_CAPACITY_SCALE, RFX_HEADROOM_GAMING_GATE + 2);
	WARN_ON(rfx_apply_headroom(gate, SCHED_CAPACITY_SCALE, true) != gate);
	WARN_ON(rfx_apply_headroom(over, SCHED_CAPACITY_SCALE, true) <= over);
	WARN_ON(rfx_apply_headroom(SCHED_CAPACITY_SCALE - 1, SCHED_CAPACITY_SCALE,
				   true) > SCHED_CAPACITY_SCALE);
	/* Daily has no ramp: the value must come back untouched at every level,
	 * including the band the ramp used to occupy. */
	WARN_ON(rfx_apply_headroom(over, SCHED_CAPACITY_SCALE, false) != over);
	WARN_ON(rfx_apply_headroom(rfx_pct(SCHED_CAPACITY_SCALE, 80),
				   SCHED_CAPACITY_SCALE, false) !=
		rfx_pct(SCHED_CAPACITY_SCALE, 80));

	/* Ramp: instant to 100, zero at RAMP_DOWN_MS, and a sub-percent step
	 * keeps its remainder instead of stalling at 100 forever. */
	WARN_ON(rfx_update_warmup_ramp(&p, true, t) != 100);
	WARN_ON(rfx_update_warmup_ramp(&p, false, t) != 100);
	WARN_ON(rfx_update_warmup_ramp(&p, false,
			t + (u64)RFX_WARMUP_RAMP_DOWN_MS * NSEC_PER_MSEC));
	p.warmup_ramp_pct = 100;
	p.warmup_ramp_last_ns = t;
	WARN_ON(rfx_update_warmup_ramp(&p, false, t + 250000) != 100 ||
		p.warmup_ramp_last_ns != t);

	/* Frame-risk latch: one crossing arms one window; a second crossing while
	 * still saturated must NOT re-arm (that would make the lift the steady
	 * state); demand parked between CLEAR and ARM stays latched while the
	 * window lives; and the window extends but never shortens. */
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

	/* Nothing to gain: already committed at the floor the window installs. */
	memset(&p, 0, sizeof(p));
	p.next_freq = 100;
	rfx_risk_rearm(&p, 100, 100, t);
	WARN_ON(p.risk_high || p.gaming_warmup_end_ns);

	/* A live longer window must survive a fresh edge. */
	memset(&p, 0, sizeof(p));
	p.next_freq = 1;
	p.gaming_warmup_end_ns = t + 10 * RFX_G_RISK_BOOST_NS;
	rfx_risk_rearm(&p, 100, 100, t);
	WARN_ON(p.gaming_warmup_end_ns != t + 10 * RFX_G_RISK_BOOST_NS);

	/* Daily slide: both endpoints must land exactly on the tuned caps (this
	 * replaces a step, so an off-by-one at either end is a real regression),
	 * and the path between them must be monotonic -- a non-monotonic slide
	 * would put back the very edge it removes. */
	WARN_ON(rfx_daily_cap_pct(0, RFX_D_BIG_CAP_PCT, RFX_D_BIG_LIFT_PCT,
				  RFX_D_BIG_DROP_PCT,
				  RFX_D_BIG_SUSTAINED_CAP_PCT) !=
		RFX_D_BIG_CAP_PCT);
	WARN_ON(rfx_daily_cap_pct(100, RFX_D_BIG_CAP_PCT, RFX_D_BIG_LIFT_PCT,
				  RFX_D_BIG_DROP_PCT,
				  RFX_D_BIG_SUSTAINED_CAP_PCT) !=
		RFX_D_BIG_SUSTAINED_CAP_PCT);
	WARN_ON(rfx_daily_cap_pct(RFX_D_BIG_DROP_PCT, RFX_D_BIG_CAP_PCT,
				  RFX_D_BIG_LIFT_PCT, RFX_D_BIG_DROP_PCT,
				  RFX_D_BIG_SUSTAINED_CAP_PCT) !=
		RFX_D_BIG_CAP_PCT);
	WARN_ON(rfx_daily_cap_pct(RFX_D_BIG_LIFT_PCT, RFX_D_BIG_CAP_PCT,
				  RFX_D_BIG_LIFT_PCT, RFX_D_BIG_DROP_PCT,
				  RFX_D_BIG_SUSTAINED_CAP_PCT) !=
		RFX_D_BIG_SUSTAINED_CAP_PCT);
	for (d = 0, prev = 0; d <= 100; d++) {
		c = rfx_daily_cap_pct(d, RFX_D_BIG_CAP_PCT, RFX_D_BIG_LIFT_PCT,
				      RFX_D_BIG_DROP_PCT,
				      RFX_D_BIG_SUSTAINED_CAP_PCT);
		WARN_ON(d && c < prev);
		prev = c;
	}

	/* Daily UI boost. Settled high must not re-arm (resample zeroes the
	 * step); a step into the ARM region opens exactly one window; the
	 * cooldown blocks the immediate re-arm that a duty-cycling load would
	 * otherwise get every cycle. */
	memset(&p, 0, sizeof(p));
	rfx_daily_ui_boost(&p, 100, t);
	WARN_ON(p.daily_ui_boost_end_ns || p.daily_ui_prev_demand_pct != 100);

	/* A step into the ARM region arms exactly one window + cooldown. */
	memset(&p, 0, sizeof(p));
	p.daily_ui_sample_ns = t;
	p.daily_ui_prev_demand_pct = RFX_D_UI_ARM_PCT - RFX_D_UI_STEP_PCT;
	rfx_daily_ui_boost(&p, RFX_D_UI_ARM_PCT, t + 1);
	WARN_ON(p.daily_ui_boost_end_ns != t + 1 + RFX_D_UI_BOOST_NS);
	WARN_ON(p.daily_ui_cooldown_end_ns != t + 1 + RFX_D_UI_COOLDOWN_NS);

	/* Settled high: the resample zeroes the step, so no second window.
	 * Clear the cooldown and the armed flag first -- both block earlier
	 * than the step test, so leaving them set passes this on whichever
	 * guard happens to be widest instead of on the resample. */
	p.daily_ui_boost_end_ns = 0;
	p.daily_ui_cooldown_end_ns = 0;
	p.daily_ui_armed = false;
	rfx_daily_ui_boost(&p, 100, t + 1 + RFX_D_UI_SAMPLE_NS);
	rfx_daily_ui_boost(&p, 100, t + 2 + RFX_D_UI_SAMPLE_NS);
	WARN_ON(p.daily_ui_boost_end_ns);

	/* Step inside the cooldown: still blocked. */
	memset(&p, 0, sizeof(p));
	p.daily_ui_sample_ns = t;
	p.daily_ui_prev_demand_pct = RFX_D_UI_ARM_PCT - RFX_D_UI_STEP_PCT;
	p.daily_ui_cooldown_end_ns = t + RFX_D_UI_COOLDOWN_NS;
	rfx_daily_ui_boost(&p, RFX_D_UI_ARM_PCT, t + 1);
	WARN_ON(p.daily_ui_boost_end_ns);

	/* Below ARM: nothing to lift. */
	memset(&p, 0, sizeof(p));
	p.daily_ui_sample_ns = t;
	rfx_daily_ui_boost(&p, RFX_D_UI_ARM_PCT - 1, t + 1);
	WARN_ON(p.daily_ui_boost_end_ns);

	/* The armed flag spans the cooldown, not the window: a load that stays
	 * in the region must still be armed after the window closes, or the
	 * cooldown is the only thing spacing the windows apart. */
	memset(&p, 0, sizeof(p));
	p.daily_ui_sample_ns = t;
	p.daily_ui_prev_demand_pct = RFX_D_UI_ARM_PCT - RFX_D_UI_STEP_PCT;
	rfx_daily_ui_boost(&p, RFX_D_UI_ARM_PCT, t + 1);
	rfx_daily_ui_boost_clear(&p, RFX_D_UI_ARM_PCT,
				 t + 1 + RFX_D_UI_BOOST_NS);
	WARN_ON(!p.daily_ui_armed);
	rfx_daily_ui_boost_clear(&p, RFX_D_UI_ARM_PCT,
				 t + 1 + RFX_D_UI_COOLDOWN_NS);
	WARN_ON(p.daily_ui_armed);

	/* Gaming frame boost: one step arms one window, boost_armed consumes
	 * the step so the same rise cannot re-arm, clear exits the armed
	 * state below CLEAR. */
	memset(&p, 0, sizeof(p));
	p.boost_sample_ns = t;
	p.boost_prev_demand_pct = RFX_G_FRAME_BOOST_ARM_PCT -
				  RFX_G_FRAME_BOOST_DELTA_PCT;
	rfx_gaming_frame_boost(&p, RFX_G_FRAME_BOOST_ARM_PCT, t + 1);
	WARN_ON(!p.boost_armed ||
		p.gaming_boost_end_ns != t + 1 + RFX_G_FRAME_BOOST_NS ||
		p.gaming_boost_pct != RFX_G_FRAME_BOOST_MAX_PCT);

	/* Same rise again before the resample: consumed, no re-arm. */
	end = p.gaming_boost_end_ns;
	rfx_gaming_frame_boost(&p, RFX_G_FRAME_BOOST_ARM_PCT + 5, t + 2);
	WARN_ON(p.gaming_boost_end_ns != end);

	/* Deferred warmup arm: nothing at the write, nothing below TRIGGER,
	 * nothing while cooling holds, one window on the crossing, and the
	 * crossing consumes the pending flag. */
	memset(&p, 0, sizeof(p));
	p.gaming_warmup_pending = true;
	rfx_warmup_arm(&p, RFX_GAMING_WARMUP_TRIGGER_PCT - 1, t + 1);
	WARN_ON(p.gaming_warmup_end_ns);
	p.thermal_cooling = true;
	rfx_warmup_arm(&p, 100, t + 2);
	WARN_ON(p.gaming_warmup_end_ns);
	p.thermal_cooling = false;
	rfx_warmup_arm(&p, RFX_GAMING_WARMUP_TRIGGER_PCT, t + 3);
	WARN_ON(!p.gaming_warmup_end_ns ||
		p.gaming_warmup_end_ns != t + 3 + RFX_GAMING_WARMUP_NS ||
		p.gaming_warmup_start_ns != t + 3 ||
		p.gaming_warmup_pending);
	rfx_warmup_arm(&p, 100, t + 4);
	WARN_ON(p.gaming_warmup_end_ns != t + 3 + RFX_GAMING_WARMUP_NS);

	/* Demand falls under CLEAR: disarmed. */
	rfx_gaming_frame_boost_clear(&p, RFX_G_FRAME_BOOST_CLEAR_PCT - 1, t + 3);
	WARN_ON(p.boost_armed || p.gaming_boost_pct);

	/* Overflow safety: prev near 100 must not wrap to a "reached" verdict. */
	WARN_ON(rfx_pct_step_reached(100, 99, 18));

	/* Target hysteresis: a sub-band descent holds at the last COMMITTED
	 * frequency, a real descent (>= band) passes, and rises are untouched.
	 * Daily band is narrower than gaming's, so a gaming-band descent passes
	 * in daily while a daily-band descent still holds. */
	memset(&p, 0, sizeof(p));
	p.next_freq = 500000;
	WARN_ON(rfx_target_hysteresis(&p, 495000, 1000000, true) != 500000);
	WARN_ON(rfx_target_hysteresis(&p, 460000, 1000000, true) != 460000);
	WARN_ON(rfx_target_hysteresis(&p, 550000, 1000000, true) != 550000);
	WARN_ON(rfx_target_hysteresis(&p, 495000, 1000000, false) != 500000);
	WARN_ON(rfx_target_hysteresis(&p, 460000, 1000000, false) != 460000);
	WARN_ON(rfx_target_hysteresis(&p, 550000, 1000000, false) != 550000);
}

static int __init vorpal_gov_init(void)
{
	int ret;

	/* Deadbands: every hysteretic pair must have its exit above its entry,
	 * every floor at or below the boost it decays from, every daily floor at
	 * or below the cap that clamps it. An inversion here is a latch that can
	 * never release (or never engage) and is invisible at runtime.
	 * DROP < LIFT additionally keeps the daily slide's divisor non-zero and
	 * its direction monotonic; base <= sustained keeps the endpoints sane. */
	BUILD_BUG_ON(RFX_G_FLOOR_GATE_PCT >= RFX_G_FLOOR_GATE_EXIT_PCT);
	BUILD_BUG_ON(RFX_G_COOL_ENTER_PCT >= RFX_G_COOL_EXIT_PCT);
	BUILD_BUG_ON(RFX_G_RISK_CLEAR_PCT >= RFX_G_RISK_ARM_PCT);
	/* The step detector windows: the release must outlive one sample, or
	 * it ends before the detector could ever extend it; a zero step would
	 * arm on flat demand; CLEAR must sit under ARM or the armed state can
	 * never release; the cooldown must hold the window shut between
	 * interactions. */
	BUILD_BUG_ON(RFX_D_UI_BOOST_NS >= RFX_D_UI_COOLDOWN_NS);
	BUILD_BUG_ON(!RFX_D_UI_STEP_PCT);
	BUILD_BUG_ON(RFX_D_UI_CLEAR_PCT >= RFX_D_UI_ARM_PCT);
	BUILD_BUG_ON(RFX_G_FRAME_BOOST_CLEAR_PCT >= RFX_G_FRAME_BOOST_ARM_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_DROP_PCT >= RFX_D_LITTLE_LIFT_PCT);
	BUILD_BUG_ON(RFX_D_BIG_DROP_PCT >= RFX_D_BIG_LIFT_PCT);
	BUILD_BUG_ON(RFX_TEMP_EMERGENCY_CLEAR_MC >= RFX_TEMP_EMERGENCY_MC);
	BUILD_BUG_ON(RFX_G_PRIME_FLOOR_PCT > RFX_G_WARMUP_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_BIG_FLOOR_PCT > RFX_G_WARMUP_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_WARMUP_FLOOR_PCT > 100);
	/* The deferred arm must trigger on a burst, not on the resting band:
	 * at or below the floor-gate exit it would fire on background activity
	 * and spend the one shot before the game exists. Above EXTEND it can
	 * never distinguish from a scene the extend path already covers. */
	BUILD_BUG_ON(RFX_GAMING_WARMUP_TRIGGER_PCT <= RFX_G_FLOOR_GATE_EXIT_PCT);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_TRIGGER_PCT > RFX_GAMING_WARMUP_EXTEND_PCT);
	BUILD_BUG_ON(RFX_GAMING_WARMUP_NS > RFX_GAMING_WARMUP_MAX_NS);
	BUILD_BUG_ON(RFX_G_IDLE_FLOOR_PCT > RFX_G_LITTLE_FLOOR_PCT);
	BUILD_BUG_ON(RFX_G_COOL_STEADY_FLOOR_PCT > RFX_G_BIG_FLOOR_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_CAP_PCT > RFX_D_LITTLE_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_BIG_CAP_PCT > RFX_D_BIG_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_PRIME_CAP_PCT > RFX_D_PRIME_SUSTAINED_CAP_PCT);
	BUILD_BUG_ON(RFX_D_LITTLE_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_BIG_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_D_PRIME_SUSTAINED_CAP_PCT > 100);
	BUILD_BUG_ON(RFX_LITTLE_CAP_THRESHOLD >= RFX_PRIME_CAP_THRESHOLD);
	BUILD_BUG_ON(RFX_EMA_GAMING_DIVISOR < 1 || RFX_EMA_MAX_STEPS < 1);
	BUILD_BUG_ON(RFX_EMERGENCY_CAP_PCT >= 100);
	BUILD_BUG_ON(RFX_G_COOL_DEEP_PCT >= RFX_G_COOL_ENTER_PCT);
	/* Gate at 100 would divide by zero in the headroom ramp. */
	BUILD_BUG_ON(RFX_HEADROOM_GAMING_GATE >= 100);
	/* Above 100 the shortcut is unreachable and the constant reads as a
	 * threshold that was never applied. 100 means "disabled" deliberately. */
	BUILD_BUG_ON(RFX_SAT_TO_MAX_GAMING_PCT > 100);
	BUILD_BUG_ON(RFX_SAT_TO_MAX_DAILY_PCT > 100);

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	rfx_selfcheck();

	INIT_DEFERRABLE_WORK(&rfx_thermal_work, rfx_thermal_fn);
	queue_delayed_work(system_power_efficient_wq, &rfx_thermal_work,
			   msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));

	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret)
		cancel_delayed_work_sync(&rfx_thermal_work);
	return ret;
}

static void __exit vorpal_gov_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
	cancel_delayed_work_sync(&rfx_thermal_work);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.2");
