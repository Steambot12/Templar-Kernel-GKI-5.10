// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.2 — schedutil-derived, tri-cluster.
 *
 * Two profiles: gaming (locked high band, frame-pacing) and daily
 * (ceiling-relative caps/floors, power-efficient). Cluster-wide EMA util
 * (fast rise / slow decay), load-proportional headroom, in-kernel frame-risk
 * detection with a global cross-cluster boost, and a latched thermal net.
 * Tuning constants and their rationale live inline where they bite.
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
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/list.h>
#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
#endif

#define CPUFREQ_VORPAL_NAME     "vorpal"
#define CPUFREQ_VORPAL_VERSION  "2.2"
#define CPUFREQ_VORPAL_AUTHOR   "Templar Dev"

/* Core-sched util getter / deadline-bandwidth check (owned by core sched). */
extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);

/* ===================================================================== */
/* Tunable defaults (KMI-safe: plain #defines, no struct-layout changes). */
/* ===================================================================== */

/* Cluster identification by arch capacity. */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* Per-cluster DAILY eval rate limits (us). up=0 = scale up instantly.
 * Interaction/gaming override via the fast + UI gates below, so these apply
 * only at rest. Little at 3ms: daily housekeeping tracks PELT's ~32ms
 * half-life, so finer sampling is pure governor overhead. */
#define RFX_LITTLE_RATE_US		3000
#define RFX_LITTLE_UP_US		200
#define RFX_LITTLE_DOWN_US		3000

/* Big/Prime at rest track PELT's ~32ms half-life, so 3ms sampling is as fine as
 * the signal: every eval re-aggregates util across the policy from the scheduler
 * hot path, and at 1.5ms that was ~670 aggregations/s/policy of pure governor
 * overhead at rest. Interaction (RFX_UI_RATE_US), gaming (RFX_FAST_RATE_US) and
 * the DL-bandwidth bypass all override it, so nothing latency-bound is sampled
 * at this rate. Down-rate 2.5ms reaches the idle OPP faster than the eval gate
 * alone; gaming overrides down via RFX_GAMING_DOWN_US. */
#define RFX_BIG_RATE_US			3000
#define RFX_BIG_UP_US			0
#define RFX_BIG_DOWN_US			2500

#define RFX_PRIME_RATE_US		3000
#define RFX_PRIME_UP_US			0
#define RFX_PRIME_DOWN_US		2500

/* Eval rate while gaming (daily interaction uses RFX_UI_RATE_US below).
 * 500us = 16 evals per 120fps frame. At 250us it was 33 per frame, and each one
 * re-aggregates util across every CPU in the policy -- governor work billed to
 * the same cores that render, i.e. avg-CPU and watts with no frame in it. The
 * EMA time constant is anchored to RFX_EMA_DECAY_PERIOD_NS, not to this, so the
 * filter shape is unchanged; up-rate is 0, so a frame still gets fmax on the
 * next eval, <=0.5ms into an 8.3ms frame. */
#define RFX_FAST_RATE_US		500

/* Eval rate during a DAILY interaction window. 1500us: a scroll only needs the
 * util rise seen promptly, and this window is re-armed by every touch event, so
 * its cadence is what light-active use actually pays -- ~1.1kHz across the whole
 * post-touch window on every interaction-path cluster. 1.5ms is still ~5 evals
 * per 120Hz frame and 11 per 60Hz frame, and PELT (half-life ~32ms) moves ~2%
 * across it, so the rise is not missed; the up-rate gate is 0 on Big/Prime, so
 * the first eval that sees it commits. */
#define RFX_UI_RATE_US			1500

/* Gaming down-rate gate. Gates every downward commit, so it sets descent
 * granularity: step = RFX_GAMING_DOWN_PCT_PER_MS * this. Must be chosen with
 * the slew bound; 4ms (half a 120fps frame) * 2%/ms = 8% max step. */
#define RFX_GAMING_DOWN_US		4000

/* Gaming floors, percent of the effective ceiling. NO cluster is capped: every
 * cluster tracks its own demand (freq = fmax*util/max_cap) up to fceil, so a
 * heavy frame is never throttled into a util pile-up (the 96%-capped-Big bug:
 * util couldn't drain -> 100% load, and misfit migration can't shed the
 * overflow fast enough). The floor only covers a cold landing so a frame that
 * lands on an idle-ish cluster recovers in fewer OPP steps; the envelope filter
 * (not a static floor) holds the render clock between frames, so floors stay
 * modest -- a high floor just pays voltage for the whole session. Frame (boost)
 * floors stay high for real frame-miss recovery.
 *
 * These two are the gaming resting-power dial (avg CPU / watts): a heavy frame
 * still reaches fmax via the saturation shortcut regardless of the floor, so
 * cutting them only lowers the light/normal-frame resting OPP, not the FPS
 * ceiling. Prime 64->58: on a 3-tier SoC Prime is the EAS-spill cluster, so it
 * churns the top OPPs while carrying only a fraction of the frame -- the highest
 * voltage on the die bought with no frame in it. */
#define RFX_G_PRIME_FLOOR_PCT		58
/* PRIME frame floor 92->80. On a 3-tier part this is the cluster the trace shows
 * sawtoothing between its band floor and near-fmax all session while the render
 * clusters sit pinned -- amplitude bought by a rescue floor for frames it mostly
 * is not carrying. 80 halves the swing (heat, and the fceil the render clusters
 * live under) and cannot throttle it: demand still tracks to fceil uncapped, the
 * floor only covers a cold landing. Lower, never higher (3 reverts). */
#define RFX_G_PRIME_FRAME_PCT		80
#define RFX_G_BIG_FLOOR_PCT		58
#define RFX_G_BIG_FRAME_PCT		90
/* Little (compositor/input/audio): demand-tracked floor, not pinned max --
 * ~30-40% during play, so pinning 100% was pure heat. <25% idle gate releases. */
#define RFX_G_LITTLE_FLOOR_PCT		60
#define RFX_G_LITTLE_FLOOR_BOOST_PCT	80

/* Max downward slew, percent of ceiling per ms elapsed (time-proportional,
 * not per-update, since update spacing varies with load). Its only job is to
 * stop one eval stepping off a cliff; the EMA owns descent SHAPE.
 *
 * 2, not 1: the EMA sheds ~3 points of error per ms early in a drop, so at 1%/ms
 * the slew -- not the filter -- bound every descent, and the clock ratcheted.
 * With a burst arriving each frame at high refresh nothing ever came back down:
 * that is what pinned the render clusters flat at fmax and left the spill tier
 * sawtoothing across the top of its range at low duty. At 2 the slew binds only
 * the first third of a large drop and the EMA governs the rest, as designed. A
 * single commit is still bounded to 2 * the 4ms gate = 8% of ceiling, ~one OPP
 * step, so this is not a cliff. Rises are untouched. */
#define RFX_GAMING_DOWN_PCT_PER_MS	2

/* ---- Daily frequency shaping, percent of the effective ceiling ---- */
/* Little daily cap 65%: sits just above the V/f knee so light scrolling
 * stays on one voltage step instead of toggling across it. */
#define RFX_D_LITTLE_CAP_PCT		65
#define RFX_D_LITTLE_BOOST_CAP_PCT	80
/* Little interaction floor, window-scoped (no touch/UI burst → no floor).
 * Holds Little off fmin between scroll frames so each frame doesn't start at
 * ~300MHz waiting for DVFS. 32% is one voltage step below the knee. */
#define RFX_D_LITTLE_UI_FLOOR_PCT	32
/* Little sustained-load cap 80%: knee sits at 75-80%, so long background
 * work (media scan, sync) keeps throughput at lower voltage. */
#define RFX_D_LITTLE_SUSTAINED_CAP_PCT	80
/* Sustained-cap latch: long background work, not burst responsiveness.
 * Skewed 1.25x, so real demand is on at 58%, off at 44%. */
#define RFX_D_LITTLE_LIFT_PCT		72	/* latch on: sustained heavy load */
#define RFX_D_LITTLE_DROP_PCT		55	/* latch off: back to housekeeping */
/*
 * Daily Big/Prime caps keep unperceived background work off the top OPPs. A
 * touch/UI window lifts to *_BOOST_CAP_PCT; the sustained latch (>=LIFT_PCT,
 * released below DROP_PCT) lifts to *_SUSTAINED_CAP_PCT for long foreground
 * work. Prime's boost cap is deliberately flat at its base, so bursty daily
 * work never touches the top OPPs -- on the fastest tier those are the power
 * cliff.
 *
 * The SUSTAINED caps are the heavy-compute ceiling, which lands in daily mode
 * (gaming_mode is never auto-written). The latch only engages at sustained real
 * demand >=~52%, i.e. compile/export/sustained-compute -- never the idle or
 * light-active daily paths that own the battery targets -- so these can sit high
 * without a daily-power cost. Higher clock here is also race-to-idle. Trim if a
 * sustained-load thermal ceiling appears before the workload completes.
 */
#define RFX_D_BIG_CAP_PCT		70
#define RFX_D_BIG_BOOST_CAP_PCT		80
#define RFX_D_PRIME_CAP_PCT		68
#define RFX_D_PRIME_BOOST_CAP_PCT	68
#define RFX_D_BIG_LIFT_PCT		65
#define RFX_D_BIG_DROP_PCT		55
#define RFX_D_BIG_SUSTAINED_CAP_PCT	94
#define RFX_D_PRIME_SUSTAINED_CAP_PCT	85

/* Cold-start burst floors, clamped to cap. Cover the first ~2ms of an app
 * launch until real demand is visible (200ms window). */
#define RFX_D_BIG_BURST_FLOOR_PCT	45
#define RFX_D_PRIME_BURST_FLOOR_PCT	42

/* Daily burst delta: 12% catches animation ramps without false-firing on
 * video/background work. */
#define RFX_D_RAMP_DELTA_PCT		12
/* Ramp/cold-start reference sample cadence. Deltas above are measured against
 * a FIXED 16ms base (one 60Hz frame), not the variable eval spacing, else the
 * detector goes blind under the finger. Downward movement tracked immediately. */
#define RFX_D_RAMP_SAMPLE_NS		(16 * NSEC_PER_MSEC)
/* Cold-start: 40% demand jump from <=10% base. Catches app launches without
 * false-firing on background/sensor work. */
#define RFX_D_COLDSTART_DELTA_PCT	40
#define RFX_D_COLDSTART_BASE_PCT	10
/* UI boost 150ms: covers animation (140-160ms); scroll-momentum tail trimmed
 * to cut the boost-cap hold under continuous scrolling (daily heat). 220ms
 * held the 80% cap through the whole fling, so passive reels autoplay paid
 * scroll heat; 150ms still clears a tap+fling without pinning the cap. */
#define RFX_D_UI_BOOST_NS		(150 * NSEC_PER_MSEC)
/* Cold-start boost 200ms: spawn + initial layout + first render. */
#define RFX_D_COLDSTART_BOOST_NS	(200 * NSEC_PER_MSEC)

/* Touch window 230ms: keyboard popup (180-220ms); scroll-momentum tail
 * trimmed to cut boost-cap hold under continuous scrolling (daily heat). */
#define RFX_INPUT_WINDOW_NS		(230 * NSEC_PER_MSEC)

/* ---- Util EMA: rise instant, decay time-normalised. Decay step is scaled by
 * elapsed wall-clock, so the time constant is independent of eval rate
 * (500us gaming .. 3ms daily). Period = interval to remove
 * 1/RFX_EMA_GAMING_DIVISOR of the remaining error; longer gaps repeat the step,
 * bounded iterations. */
#define RFX_EMA_DECAY_PERIOD_NS		250000	/* decay reference period */
/* Gaming decay: 1/50 of the error per period is a ~12.5ms time constant, still
 * longer than the 8.33ms frame gap at 120fps, so filt_util rides the scene
 * envelope and does NOT track the render thread's intra-frame duty (at 1/8, tau
 * ~1.9ms, it decayed 99% inside one frame gap and the inter-frame trough
 * collapsed the render floor every frame -- never go back toward that).
 *
 * 100 -> 50: at tau 25ms the filter spanned three frame periods, so with a
 * render burst arriving every 8.3ms it only shed ~28% of the error between
 * peaks and became a peak-HOLD -- the trace shows both render clusters flat at
 * fmax for a 7-minute session at 55-80% load, which is the monotonic temperature
 * ramp, and the spill tier sawtoothing because its brief bursts were held three
 * frames wide. One frame and a half of hold is what the envelope needs; three is
 * a latch. The trough-collapse this used to guard against is now covered
 * structurally by RFX_G_FLOOR_GATE_DWELL_NS (20ms), not by the filter shape, so
 * the tau no longer has to carry that job. Rise stays instant and up-rate stays
 * 0, so no frame is served any later -- only the resting point between frames
 * comes down. */
#define RFX_EMA_GAMING_DIVISOR		50
/* Step cap: 32 periods = 8ms, one frame gap. Bounds util-hook work without
 * truncating the decay over the gaps that matter. */
#define RFX_EMA_MAX_STEPS		32

/* ---- Headroom (extra capacity above demand) percent. Stacks on top of the
 * 25% DVFS margin rfx_get_util_gki510 already applied. Small residual covers
 * PELT's slower-than-WALT decay; saturation shortcut still reaches fmax on a
 * real frame, so this only lowers the resting OPP. */
#define RFX_HEADROOM_DAILY_HIGH		4
#define RFX_HEADROOM_DAILY_MID		2
/* Gaming headroom on top of the 25% margin. Applied every eval, so it sets the
 * resting OPP -- the steady-state power lever. At 5% a sustained 70-80% render
 * load still pinned Big flat at fmax for a whole session while FPS stayed
 * engine-bound -- i.e. the top OPP bought heat, not frames. 2% (~27% total) lets
 * that band interpolate one-two OPPs below fmax; a real >=78% frame still
 * inflates past the saturation trigger and reaches fmax, and the independent
 * frame-risk detector (RFX_RISK_SATURATION_PCT) still boosts genuine misses, so
 * FPS holds while resting watts drop. Do not raise back toward 5+ without a
 * measurement showing FPS (not just CPU%) actually falling. */
#define RFX_HEADROOM_GAMING		2

/* Util percent at which we stop interpolating and request fmax outright.
 * Gaming 98%: the on-device trace showed 95 still shortcut the sustained
 * 70-80% render load to fmax (76% real x1.25 margin = 95 == trigger), pinning
 * Big/Little flat at the top OPP for the whole session while FPS stayed
 * engine-bound at ~117 -- clock that bought heat, not frames. 98 lets the
 * 70-78% real band (inflated 87-97) ride one-two OPPs down the table instead of
 * pinning; only >=78.4% real still inflates to >=98 and saturates, so genuinely
 * heavy frames reach fmax unchanged and the frame-risk detector still covers
 * misses -- only the resting point of a steady heavy scene moved. The envelope
 * filter makes the reading a scene measure, not a per-frame spike, so this does
 * not chatter. Daily 95%: last OPP is a battery cost. */
#define RFX_SAT_TO_MAX_GAMING_PCT	98
#define RFX_SAT_TO_MAX_DAILY_PCT	95

/* ---- Thermal emergency net. HW LMH (via thermal_pressure/fceil) + vendor HAL
 * (via core-enforced policy->max) are the real controllers. This is a single
 * hard net for when the vendor engine is absent/asleep: one trip, one release,
 * 7C apart, so it cannot oscillate. */
#define RFX_THERMAL_POLL_GAMING_MS	100
/* Idle poll: die time constant is ~seconds; 5s detects runaway in <2 constants.
 * Work is deferrable (free in deep sleep), so this only trims screen-on-idle
 * wakeup current. */
#define RFX_THERMAL_POLL_IDLE_MS	5000
/* Warm tier: charging + active use can climb 3-5C between 5s polls; 2s catches
 * it 2.5x sooner. Daily only; gaming already polls at 100ms. */
#define RFX_THERMAL_POLL_WARM_MS	2000
#define RFX_TEMP_WARM_MC		70000
#define RFX_TEMP_EMERGENCY_MC		95000	/* junction; LMH acts far below */
#define RFX_TEMP_EMERGENCY_CLEAR_MC	88000
#define RFX_EMERGENCY_CAP_PCT		70

/* ---- Frame pacing ---- */
/* ~4 frames at 120fps: covers the at-risk frame plus settle. Up-rate 0 recovers
 * a cluster in <1 frame, so a longer hold only pins idle clusters as heat. */
#define RFX_FRAME_BOOST_NS		(33 * NSEC_PER_MSEC)

/* In-kernel frame-risk detection vs the effective ceiling: >90% servable = next
 * frame at risk; must fall under 75% before another window can arm. On the
 * inflated demand scale, so ~72%/60% real -- above Big's ordinary band, which
 * 80/68 sat inside and re-armed continuously. */
#define RFX_RISK_SATURATION_PCT		90
#define RFX_RISK_CLEAR_PCT		75

/* Arm dwell. One eval's crossing is a burst, not a late frame: up-rate is 0, so
 * that cluster is served on the next eval anyway, while the window it arms lifts
 * EVERY cluster's floor for a whole span. Sub-ms spill bursts on an intermittent
 * cluster were re-arming it toward a permanent high floor. 2 evals, ~12% of a
 * frame -- long enough to need persistence, short enough that a real miss is
 * still rescued inside the same frame. */
#define RFX_RISK_ARM_DWELL_NS		(1 * NSEC_PER_MSEC)

/* Frame boost ramp: instant rise, gentle decay back to baseline floors. Also
 * carries the warmup release, so it must outlast a few frames. 40ms (~5 frames
 * at 120fps) continues the direction that measured good at 120->60: a shorter
 * decay shortens the whole span, so the same duty backstop allows a re-arm
 * sooner -- more rescue coverage for a rough patch, less lifted time. */
#define RFX_FRAME_BOOST_RAMP_DOWN_MS	40

/* Floors stay lifted for window + ramp tail, not just the window -- that full
 * span is what a duty cycle has to be measured against. Min spacing between
 * arms: expiry clears risk_high, so demand parked just above SATURATION re-arms
 * forever. 2x span caps the lifted duty cycle at 50%; measuring 3x the window
 * alone left it at 94%, i.e. permanently lifted. Kept at 2x: the arm edge in
 * rfx_frame_risk_check is what limits how often a window is legitimately armed,
 * and this backstop only has to bound the pathological case. */
#define RFX_FRAME_BOOST_SPAN_NS		(RFX_FRAME_BOOST_NS + \
					 (u64)RFX_FRAME_BOOST_RAMP_DOWN_MS * \
					 NSEC_PER_MSEC)
#define RFX_FRAME_BOOST_MIN_GAP_NS	(RFX_FRAME_BOOST_SPAN_NS * 2)

/* Gaming warmup: lifts floors to frame-boost level (not the cap) to cover spawn
 * + shader/asset load. Longer windows pinned all three clusters through the
 * hottest phase, tripping LMH; up-rate is 0 anyway, so warmup only pre-warms. */
#define RFX_GAMING_WARMUP_NS		(300 * NSEC_PER_MSEC)
/* Adaptive warmup: extends while Big/Prime demand stays >EXTEND_PCT, up to
 * MAX_NS; early release below RELEASE_PCT for RELEASE_NS. EXTEND_PCT is on the
 * inflated demand scale -- 60 there (~48% real) extended unconditionally.
 * MAX_NS is deliberately short: this window is anchored to the sysfs write, so
 * a longer one is spent on the load phase and pins every cluster through the
 * hottest part of the session (reverted twice). What covers the first rendered
 * frames instead is the re-armable rise detector, RFX_G_RAMP_DELTA_PCT. */
#define RFX_GAMING_WARMUP_MAX_NS	(600 * NSEC_PER_MSEC)
#define RFX_GAMING_WARMUP_EXTEND_PCT	90
#define RFX_GAMING_WARMUP_RELEASE_PCT	40
#define RFX_GAMING_WARMUP_RELEASE_NS	(100 * NSEC_PER_MSEC)

/* Gaming floor demand gate: below this util a cluster's floor releases and
 * normal DVFS applies, so an idle cluster doesn't burn floor voltage as heat.
 * EXIT is the deadband -- an intermittent-render cluster sits on the 25
 * boundary, and a single threshold toggled its floor every few evals. */
#define RFX_G_FLOOR_GATE_PCT		25
#define RFX_G_FLOOR_GATE_EXIT_PCT	35
/* Same gate, PRIME band. Its EMA parks in the 25-60 band all session, so at
 * 25/35 it never released and rested on its band floor at low duty -- the
 * session heat source, and heat is what drops fceil under the render clusters.
 * 45/55 releases it to the idle floor unless it is really carrying work. */
#define RFX_G_PRIME_FLOOR_GATE_PCT	45
#define RFX_G_PRIME_FLOOR_GATE_EXIT_PCT	55
/* Time deadband on the same gate. The value deadband can't tell an idle cluster
 * from a render cluster whose inter-frame trough dips under GATE. 20ms outlasts
 * a 60fps frame gap. Excludes a 3-tier Prime: holding that floor is the
 * forbidden direction. */
#define RFX_G_FLOOR_GATE_DWELL_NS	(20 * NSEC_PER_MSEC)

/* Demand a cluster must show to follow the GLOBAL (shared) frame boost up to its
 * frame floor. Without this, one cluster's risk crossing would hold all three at
 * frame-floor voltage for the whole session. 35% separates "carrying part of the
 * frame" from "ticking over"; below it the baseline band floor still applies. Set
 * low on purpose: Little (compositor/input/audio) is latency-critical but light
 * (~30-40%) and rarely hits 45%, so a higher gate would exclude the one cluster
 * always in the frame.
 *
 * PRIME needs a higher gate, not an exclusion: on a 3-tier SoC it mostly receives
 * EAS spill and sat across the 35 boundary all session, following every boost at
 * top-of-die voltage with no frame in it. 60 (~48% real) means it must actually be
 * carrying work -- which is the case a role flag cannot tell apart. */
#define RFX_G_BOOST_FOLLOW_PCT		35
#define RFX_G_PRIME_BOOST_FOLLOW_PCT	60

/* Floor for a gated (idle) cluster. Not zero: from fmin a cluster must climb the
 * whole range when work lands, and the OPP transition + rate gate turn that into
 * a visible hitch (freq trace shows the jump arriving AFTER the late frame). 38%
 * sits at the V/f knee -- near-free on an empty cluster, one fewer OPP step on
 * the recovery ramp. */
#define RFX_G_IDLE_FLOOR_PCT		38

/* Gaming rise detect: a scene change / level load steps demand faster than the
 * floor band tracks, and the opening seconds of a session are one long rise
 * after the warmup window has expired. Catching a 15-point step and lifting to
 * boost_fl for two evals closes that gap wherever it happens -- render cluster
 * included, which is what a 2-tier part needs. Gated on the same per-role
 * follow_pct as every other lift, so a spill burst on an idle-ish Prime cannot
 * arm it (that lift is the forbidden direction). Re-armable, unlike warmup. */
#define RFX_G_RAMP_DELTA_PCT		15
#define RFX_G_RAMP_HOLD_NS		(1 * NSEC_PER_MSEC)	/* 2 evals */
/* Reference cadence for that delta -- the same correction the daily detector
 * needed (RFX_D_RAMP_SAMPLE_NS), for the same reason. Measured against the
 * PREVIOUS EVAL the test was "15 points inside 500us", which only an
 * instantaneous jump can satisfy: a real scene ramp spread over 20ms moves ~0.4
 * points per eval and was never detected, so the detector could not do the job
 * it was added for. 8ms is one frame at 120fps, so the test reads "demand
 * stepped 15 points within a frame" -- a scene change, not eval noise -- and the
 * lift stays bounded because the reference refreshes on that cadence and on
 * nothing else (snapping it down on a fall is what turned the daily detector
 * into an unbounded peak-to-trough comparator). */
#define RFX_G_RAMP_SAMPLE_NS		(8 * NSEC_PER_MSEC)

/* Cluster cool-down band, hysteretic. Below ENTER the platform limiter is
 * taking capacity, so floors drop for relief; they return at EXIT. The 5-point
 * deadband stops toggling as the HAL steps policy->max across an OPP boundary. */
#define RFX_G_COOL_ENTER_PCT		80
#define RFX_G_COOL_EXIT_PCT		85

/* Cooling floors, split by role (do NOT collapse both to idle: boost_fl at 38%
 * made the risk detector's `next_freq >= boost_fl` short-circuit fire every
 * eval, so frame-boost never armed while throttled -- exactly when frames miss).
 *   STEADY - sustained-load relief floor, below the band but above idle.
 *   BOOST  - reduced but still armable so frame-risk can rescue a late frame.
 *            72, not 66: at 66 a late frame during throttle is rescued from a
 *            lower OPP, so recovery misses the deadline (the 0.5->1.3% jank). */
#define RFX_G_COOL_STEADY_FLOOR_PCT	52
#define RFX_G_COOL_BOOST_FLOOR_PCT	72

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

/* Last input event timestamp (daily touch boost). */
static atomic64_t rfx_input_ts_ns = ATOMIC64_INIT(0);

/* Emergency thermal cap percent (100 = inactive). Latched with hysteresis. */
static atomic_t rfx_emergency_cap_pct = ATOMIC_INIT(100);
/* Userspace-fed temperature fallback (milli-Celsius); 0 = unavailable. */
static atomic_t rfx_temp_mc = ATOMIC_INIT(0);

/*
 * Frame-miss boost deadline (ns since boot), GLOBAL so every cluster reacts to
 * a dropped frame - not just Prime. A missed frame means SOME cluster was too
 * slow; since the render thread's placement is not known in-kernel, all of
 * Prime/Big/Little lift their floor together until this deadline.
 */
static atomic64_t rfx_frame_boost_end_ns = ATOMIC64_INIT(0);
/* Arm time of that window: enforces RFX_FRAME_BOOST_MIN_GAP_NS. */
static atomic64_t rfx_frame_boost_arm_ns = ATOMIC64_INIT(0);

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

	bool limits_changed;
	bool need_freq_update;

	bool is_prime;			/* PRIME band applies (3+ tiers only) */
	bool is_top;			/* fastest tier: hosts the gaming_mode node */
	bool is_little;

	unsigned int prev_upct;		/* daily ramp reference demand% */
	u64 prev_upct_ns;		/* when that reference was sampled */
	u64 ui_boost_end_ns;		/* daily: UI render-burst floor hold */
	u64 coldstart_boost_end_ns;	/* daily: cold-start burst boost */

	/* Frame boost ramp (smooth transition, not binary) */
	unsigned int frame_boost_ramp_pct;	/* current ramp level 0-100 */
	u64 frame_boost_ramp_last_ns;		/* previous ramp evaluation */

	u64 gaming_warmup_end_ns;	/* floor lift after gaming_mode=1 */
	u64 gaming_warmup_start_ns;	/* arm time — anchors the absolute cap */

	/*
	 * Cluster-wide smoothed util. Owned by the policy, not by the CPU that
	 * happened to run the hook: a shared policy commits one frequency, so a
	 * per-CPU EMA made the committed value depend on which CPU ticked last
	 * (visible as frequency jitter and micro-stutter).
	 */
	unsigned long filt_util;
	u64 last_ema_ns;			/* timestamp of last EMA update */

	bool risk_high;			/* frame-risk edge state */
	u64 risk_since_ns;		/* gaming: demand saturated since (arm dwell) */
	bool floor_gated;		/* gaming: floor released to idle, hysteretic */
	u64 floor_low_since_ns;		/* gaming: demand below GATE since (dwell) */
	bool little_cap_lifted;		/* daily: sustained-load cap lift latch */
	bool big_cap_lifted;		/* daily: sustained-load cap lift for Big/Prime */
	bool thermal_cooling;		/* gaming: floors dropped to idle, hysteretic */

	/* Demand rise detection (gaming, all clusters) */
	u64 ramp_end_ns;
	unsigned int prev_gaming_demand_pct;
	u64 prev_gaming_demand_ns;	/* when that reference was sampled */

	/* adaptive warmup — early-release tracking */
	u64 warmup_low_demand_since_ns;	/* when demand first fell below release threshold */

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
 * Two separate questions, previously one flag:
 *   is_top   - fastest tier; hosts rfx_prime_ktype, the only attr set carrying
 *              gaming_mode, so exactly one policy must be true.
 *   is_prime - PRIME band applies. That band models an intermittent EAS-spill
 *              cluster; on a two-tier SoC the fastest tier IS the render cluster,
 *              so it takes the BIG band instead.
 */
static inline bool rfx_cap_is_little(unsigned long cap)
{
	return cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool rfx_cap_is_top(unsigned long cap)
{
	return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

/* Distinct capacity tiers. Fixed before any governor attaches, so cache it. */
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

	ntiers = n ? n : 3;
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
 * Effective ceiling: percent remaining, plus the unthrottled baseline it applies
 * to. Two throttle channels; reading only the first was the portability gap:
 *   thermal_pressure - cpufreq_cooling / LMH. Present on QCOM.
 *   policy->max      - vendor thermal HAL. The only channel on MTK, where
 *                      cpufreq_cooling never registers.
 * Measured against the high-water policy->max, not the live value: a statically
 * low baseline (MTK per-core OPP split) must read as no throttle, and folding
 * the live value in directly is the earlier double-clamped-daily-caps regression.
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
			press_pct = (unsigned int)((u64)(max_cap - press) *
						   100 / max_cap);
	}

	if (pmax && pmax < *baseline)
		clamp_pct = (unsigned int)((u64)pmax * 100 / *baseline);

	return min(press_pct, clamp_pct);
}

/*
 * Timestamps compared against the util hook's @time must come from
 * sched_clock(), not ktime_get_ns(): @time is rq_clock (sched_clock-derived),
 * while CLOCK_MONOTONIC has a different epoch and NTP rate correction, so
 * mixing them makes (time - ts) meaningless and can wrap to a huge value -- a
 * window that silently never opens. Touch boost, warmup and the rate gates all
 * live in the sched_clock domain; only the thermal poller keeps ktime (it
 * compares against itself).
 */
static inline bool rfx_input_active(u64 time)
{
	u64 ts = (u64)atomic64_read(&rfx_input_ts_ns);

	return ts && (time - ts) < RFX_INPUT_WINDOW_NS;
}

/* ===================================================================== */
/* Helpers                                                               */
/* ===================================================================== */

/*
 * Frame boost ramp: instant rise to 100 when a boost arms (zero-latency miss
 * recovery), then a linear decay over RFX_FRAME_BOOST_RAMP_DOWN_MS so the
 * floor slides back to baseline instead of stepping off a cliff.
 */
static unsigned int rfx_update_frame_boost_ramp(struct rfx_policy *p, bool boost_active, u64 time)
{
	u64 delta_ns;
	unsigned int step;

	if (boost_active) {
		p->frame_boost_ramp_pct = 100;
		p->frame_boost_ramp_last_ns = time;
		return 100;
	}

	if (p->frame_boost_ramp_pct == 0)
		return 0;

	if (!p->frame_boost_ramp_last_ns)
		p->frame_boost_ramp_last_ns = time;
	delta_ns = time - p->frame_boost_ramp_last_ns;

	step = (unsigned int)min_t(u64,
		(delta_ns * 100) / ((u64)RFX_FRAME_BOOST_RAMP_DOWN_MS * NSEC_PER_MSEC), 100);

	/*
	 * Advance the timestamp by exactly the time the step consumed, not to
	 * `time`. One ramp percent is 0.6ms but gaming updates arrive every
	 * ~250us, so the division floors to zero on most calls; deferring the
	 * advance accumulates the sub-step remainder until delta_ns crosses the
	 * 0.6ms boundary. Advancing to `time` dropped that remainder, leaving a
	 * stale ramp level when two boost windows overlap -- a micro-stutter.
	 */
	if (step > 0) {
		u64 consumed_ns = (u64)step *
			((u64)RFX_FRAME_BOOST_RAMP_DOWN_MS * NSEC_PER_MSEC) /
			100;
		p->frame_boost_ramp_last_ns += consumed_ns;
		p->frame_boost_ramp_pct -= min(p->frame_boost_ramp_pct, step);
	}

	return p->frame_boost_ramp_pct;
}

/* ===================================================================== */
/* Util smoothing                                                        */
/* ===================================================================== */

/*
 * Directional EMA: instant rise, time-normalised decay. The slow decay is the
 * anti-yoyo filter (stops the clock chasing micro-dips between frames) but must
 * still reach the bottom, so the step scales with elapsed time. Each 250us
 * reference period removes 1/RFX_EMA_GAMING_DIVISOR of the error; longer gaps
 * repeat the step, capped to bound update-hook work.
 */
static unsigned long rfx_ema(unsigned long old, unsigned long val,
			     u64 delta_ns, bool gaming)
{
	unsigned long diff;
	unsigned int steps;

	if (!old)
		return val;
	if (val >= old)
		return val;	/* instant rise */

	/* Daily: instant fall. Gentle decay rode the peak of bursty light loads
	 * (scroll/video), pinning a high resting OPP through inter-frame dips;
	 * the down-rate gate already bounds churn. Gaming keeps the decay below. */
	if (!gaming)
		return val;

	/*
	 * Gaming: 1/RFX_EMA_GAMING_DIVISOR of the remaining error per period.
	 * The decay must span frames, not chase within one -- see the divisor.
	 * Step cap keeps the time constant honest across a whole frame gap while
	 * still bounding work in the util hook.
	 */
	steps = (unsigned int)min_t(u64, delta_ns / RFX_EMA_DECAY_PERIOD_NS,
				    RFX_EMA_MAX_STEPS);
	if (!steps)
		return old;	/* sub-period: hold, don't decay */

	while (steps--) {
		diff = old - val;
		if (!diff)
			break;
		old -= max_t(unsigned long, diff / RFX_EMA_GAMING_DIVISOR, 1);
	}

	return old;
}

/*
 * Headroom: request slightly more capacity than measured so we land on an OPP
 * with room to spare (avoids running pinned at 100% util, which is both a
 * latency and a load-percent problem). Gaming uses fixed constant headroom
 * (eliminates multiplicative variance); daily uses a tiered curve that adds
 * little at low util (battery) and more as util climbs (responsiveness).
 */
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

	if (gaming)
		return min(util + (max_cap * RFX_HEADROOM_GAMING / 100), max_cap);

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

	/* Below 45%: no headroom. WALT adds nothing at low util, and the 25%
	 * DVFS margin from the util getter already covers OPP granularity.
	 * The old 2% residual held a higher voltage step through every idle
	 * evaluation — pure standby drain with no responsiveness benefit. */
	return util;
}

/* ===================================================================== */
/* Thermal emergency clamp (final clamp)                                 */
/* ===================================================================== */

/*
 * Last clamp before OPP resolution. A flat, latched cap - no walking, no
 * proportional target, no per-cluster stepping. Normal throttling is the
 * platform's job (LMH via thermal_pressure/fceil, thermal HAL via core-enforced
 * policy->max); this only fires if the die reaches RFX_TEMP_EMERGENCY_MC, which
 * on a healthy device never happens.
 */
static unsigned int rfx_thermal_clamp(unsigned int freq, unsigned int fmax)
{
	int pct = atomic_read(&rfx_emergency_cap_pct);

	if (likely(pct >= 100))
		return freq;

	return min(freq, rfx_pct(fmax, pct));
}

/* ===================================================================== */
/* Frame pacing                                                          */
/* ===================================================================== */

static inline bool rfx_frame_boost_active(u64 time)
{
	u64 end = (u64)atomic64_read(&rfx_frame_boost_end_ns);

	return end && time < end;
}

/*
 * Frame-risk detector. Arms one boost window when raw cluster demand crosses
 * RFX_RISK_SATURATION_PCT. Demand must fall below RISK_CLEAR_PCT (or the
 * boost window must expire) before another can arm. Little excluded.
 * Measures raw demand (before headroom inflation).
 */
static void rfx_frame_risk_check(struct rfx_policy *p, unsigned int demand_pct,
				 unsigned int boost_fl, u64 time)
{
	if (likely(p->is_little))
		return;

	if (demand_pct < RFX_RISK_SATURATION_PCT) {
		/*
		 * Clear risk_high on demand < CLEAR OR boost window expired.
		 * Without the expiry check, demand parked in (CLEAR, SATURATION)
		 * -- where a busy scene sits between frames -- latches risk_high
		 * forever after the first boost, blocking recovery re-arm (the
		 * residual jank under sustained load). This is the only re-arm
		 * path, which is what keeps one crossing to one window.
		 */
		p->risk_since_ns = 0;
		if (demand_pct <= RFX_RISK_CLEAR_PCT ||
		    !rfx_frame_boost_active(time))
			p->risk_high = false;
		return;
	}

	/*
	 * One crossing arms one window: demand must fall back under CLEAR before
	 * another can arm. Re-arming while still saturated made the boost the
	 * steady state on any device whose sustained gaming demand sits above
	 * SATURATION -- floors, and every other mechanism keyed off the boost,
	 * latched to their emergency values for the whole session. Sustained
	 * saturation is a heavy scene, not a missed frame, and such a cluster is
	 * already at fmax through the saturation shortcut, so a boost adds no
	 * clock -- only heat on the clusters it pins.
	 */
	if (p->risk_high)
		return;

	/*
	 * Nothing to gain: already committed at/above the floor a boost would
	 * install, so arming can't raise this OPP -- only pin the others. Beats a
	 * `policy->max < fmax` test that disabled the detector whenever anything
	 * held the policy down (thermal HAL, perf daemon) -- i.e. while throttled,
	 * which is when frames miss. "Is there clock left to win?" regardless of
	 * how the clamp got there.
	 *
	 * Tested at the ARM EDGE only, then latched by risk_since_ns. It reads
	 * next_freq, the PREVIOUS commit, while up_rate_delay_ns is 0 in gaming --
	 * so the first eval of a saturation already commits this cluster near
	 * fceil, and every later eval of the same crossing sees next_freq above
	 * boost_fl. Re-tested each eval it therefore cancelled every arm the dwell
	 * below was still counting out: the two were mutually exclusive and the
	 * detector could not fire at all. Edge-tested it asks the question that
	 * was intended -- "did a cluster that was running LOW just saturate?" --
	 * which is the cold-landing spill that actually misses frames, while a
	 * cluster already pinned at fceil still never arms.
	 */
	if (!p->risk_since_ns) {
		if (p->next_freq >= boost_fl)
			return;
		/*
		 * Time deadband on the arm edge (see RFX_RISK_ARM_DWELL_NS):
		 * saturation must persist, so a sub-ms burst cannot arm a global
		 * floor lift.
		 */
		p->risk_since_ns = time;
		return;
	}
	if (time - p->risk_since_ns < RFX_RISK_ARM_DWELL_NS)
		return;

	/* Duty-cycle backstop: one window per MIN_GAP, whatever demand does. */
	{
		u64 armed = (u64)atomic64_read(&rfx_frame_boost_arm_ns);

		if (armed && time - armed < RFX_FRAME_BOOST_MIN_GAP_NS)
			return;
	}

	p->risk_high = true;
	p->risk_since_ns = 0;
	atomic64_set(&rfx_frame_boost_arm_ns, time);
	atomic64_set(&rfx_frame_boost_end_ns, time + RFX_FRAME_BOOST_NS);
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
	/*
	 * One ceiling for both bands, honouring every throttle channel (see
	 * rfx_thermal_headroom_pct). Every floor, cap and frame percent below is
	 * a percentage of THIS, so the whole shape slides down under a clamp
	 * instead of colliding with it. A floor computed against hardware fmax
	 * would sit above the clamp, pin the cluster flat, and make the platform
	 * clamp harder -- the "heats even with a cooler" ratchet, seen only on
	 * ROMs/SoCs whose throttle channel this governor wasn't reading.
	 *
	 * fmax = unthrottled baseline (high-water policy->max), not
	 * cpuinfo.max_freq: on MTK the baseline sits below cpuinfo max.
	 */
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

	util = rfx_apply_headroom(util, max_cap, gaming, little);

	/* arch capacity 1024 is defined against cpuinfo max, so demand->freq uses
	 * that; only the percentage shape below uses fmax. */
	freq = (unsigned int)((u64)pol->cpuinfo.max_freq * util / max_cap);
	freq = clamp(freq, fmin, fceil);

	if (gaming) {
		bool fboost_active, warmup_active;
		unsigned int fboost_ramp_pct;
		unsigned int fl, boost_fl, demand_pct, follow_pct;
		u64 down_step, slew_ns;

		/*
		 * Demand before rfx_apply_headroom's inflation -- what the risk
		 * detector and floor gate judge against (inflated `util` is not a
		 * load measurement; feeding it here fired every threshold ~20
		 * points early).
		 *
		 * CAVEAT, do NOT re-tune thresholds without reading this: raw_util
		 * is filt_util, and rfx_get_util_gki510 already applied the 25% DVFS
		 * margin, so demand_pct reads ~1.25x measured demand (saturating).
		 * Every threshold below (gate, follow, lift/drop latches, cold-start
		 * and ramp deltas) was tuned empirically WITH that skew, so skew and
		 * numbers are a matched pair -- fix both together or neither.
		 */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		warmup_active = p->gaming_warmup_end_ns && time < p->gaming_warmup_end_ns;

		/*
		 * Adaptive warmup. Extend while Big/Prime demand stays above the
		 * extend threshold, capped at RFX_GAMING_WARMUP_MAX_NS from the
		 * original start; end early if demand drops below the release
		 * threshold for RELEASE_NS.
		 */
		if (warmup_active && !little) {
			if (demand_pct >= RFX_GAMING_WARMUP_EXTEND_PCT) {
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
				else if (time - p->warmup_low_demand_since_ns >=
					 RFX_GAMING_WARMUP_RELEASE_NS)
					p->gaming_warmup_end_ns = time;
			} else {
				p->warmup_low_demand_since_ns = 0;
			}
			warmup_active = time < p->gaming_warmup_end_ns;
		}

		/*
		 * One parameterised band for all three clusters. The three
		 * copies this replaces were identical apart from their
		 * percentages, and each had to be fixed separately - the exact
		 * shape that lets a bug survive in one branch after being fixed
		 * in the others.
		 */
		if (prime) {
			fl = rfx_pct(fceil, RFX_G_PRIME_FLOOR_PCT);
			boost_fl = rfx_pct(fceil, RFX_G_PRIME_FRAME_PCT);
			follow_pct = RFX_G_PRIME_BOOST_FOLLOW_PCT;
		} else if (!little) {		/* Big: demand-tracked, uncapped */
			fl = rfx_pct(fceil, RFX_G_BIG_FLOOR_PCT);
			boost_fl = rfx_pct(fceil, RFX_G_BIG_FRAME_PCT);
			follow_pct = RFX_G_BOOST_FOLLOW_PCT;
		} else {			/* Little: compositor / audio / input */
			fl = rfx_pct(fceil, RFX_G_LITTLE_FLOOR_PCT);
			boost_fl = rfx_pct(fceil, RFX_G_LITTLE_FLOOR_BOOST_PCT);
			follow_pct = RFX_G_BOOST_FOLLOW_PCT;
		}

		/*
		 * Once the platform has removed capacity, holding gaming floors or
		 * a global boost defeats thermal relief and makes the HW limiter
		 * saw-tooth the clock. Let every cluster cool to the idle floor;
		 * normal demand and up-rate stay intact. The entry/exit split (see
		 * RFX_G_COOL_ENTER_PCT) is the deadband against boundary toggling.
		 */
		if (fceil_pct < RFX_G_COOL_ENTER_PCT)
			p->thermal_cooling = true;
		else if (fceil_pct >= RFX_G_COOL_EXIT_PCT)
			p->thermal_cooling = false;

		if (p->thermal_cooling) {
			unsigned int steady = rfx_pct(fceil,
						      RFX_G_COOL_STEADY_FLOOR_PCT);
			unsigned int boost = rfx_pct(fceil,
						     RFX_G_COOL_BOOST_FLOOR_PCT);

			fl = min(fl, steady);
			boost_fl = min(boost_fl, boost);
		}

		/*
		 * The emergency net clamps the final result, so hand the detector
		 * the reachable (clamped) floor, not the nominal one -- else it arms
		 * windows that resolve to the clamp it already sits on.
		 */
		rfx_frame_risk_check(p, demand_pct,
				     rfx_thermal_clamp(boost_fl, fceil), time);

		/*
		 * Bounded slew: fall limited to RFX_GAMING_DOWN_PCT_PER_MS of
		 * ceiling per ms, measured in ns from last commit (not last eval) so
		 * budget accumulates correctly. Cap the window at the down-rate gate
		 * period: otherwise budget grows unbounded while the gate blocks
		 * commits, then the first accepted commit dumps the whole step as a
		 * frequency cliff (a jank frame at high FPS). Capped, one gate period
		 * = one max step, smooth even when commits are sparse.
		 */
		slew_ns = time - max(p->last_upfreq_time, p->last_downfreq_time);
		slew_ns = min_t(u64, slew_ns,
				(u64)RFX_GAMING_DOWN_US * NSEC_PER_USEC);
		down_step = (u64)rfx_pct(fceil, RFX_GAMING_DOWN_PCT_PER_MS) *
			    slew_ns / NSEC_PER_MSEC;
		if (down_step < fceil && p->next_freq > (unsigned int)down_step &&
		    freq < p->next_freq - (unsigned int)down_step)
			freq = p->next_freq - (unsigned int)down_step;

		fboost_active = rfx_frame_boost_active(time);
		/*
		 * Warmup feeds the same ramp instead of holding a hard floor: at
		 * expiry the old branch stepped the floor from boost_fl straight
		 * to fl in one eval, and that cliff landed exactly as the first
		 * real frames arrived (the start-of-session FPS notch). Through
		 * the ramp it decays over RAMP_DOWN_MS instead.
		 */
		fboost_ramp_pct = rfx_update_frame_boost_ramp(p,
					fboost_active || warmup_active, time);

		/*
		 * Idle clusters release to the idle floor. Two deadbands: value
		 * (GATE/GATE_EXIT, per role) and, on render-bearing clusters, time
		 * (DWELL) so an inter-frame trough can't collapse the render floor.
		 */
		if (demand_pct >= (prime ? RFX_G_PRIME_FLOOR_GATE_EXIT_PCT :
					   RFX_G_FLOOR_GATE_EXIT_PCT)) {
			p->floor_gated = false;
			p->floor_low_since_ns = 0;
		} else if (demand_pct < (prime ? RFX_G_PRIME_FLOOR_GATE_PCT :
						 RFX_G_FLOOR_GATE_PCT)) {
			if (prime) {
				p->floor_gated = true;
			} else if (!p->floor_low_since_ns) {
				p->floor_low_since_ns = time;
			} else if (time - p->floor_low_since_ns >=
				   RFX_G_FLOOR_GATE_DWELL_NS) {
				p->floor_gated = true;
			}
		} else {
			p->floor_low_since_ns = 0;
		}

		if (p->floor_gated)
			fl = rfx_pct(fceil, RFX_G_IDLE_FLOOR_PCT);
		else if (fboost_ramp_pct > 0 && demand_pct >= follow_pct)
			fl = fl + (boost_fl - fl) * fboost_ramp_pct / 100;

		if (freq < fl)
			freq = fl;

		/*
		 * Rise detect. A scene change / level load steps demand faster
		 * than the EMA and rate gates track, and the opening seconds are
		 * one long rise past warmup expiry. Same demand gate as every
		 * other lift, so an idle-ish spill cluster cannot arm it.
		 *
		 * Reference refreshes on a fixed cadence and on nothing else --
		 * against the previous eval (~500us apart under load) a 15-point
		 * step needs an instantaneous jump, so a real ramp spread over
		 * frames was never seen; snapping it down on a fall instead would
		 * make it an unbounded peak-to-trough comparator (the daily
		 * detector's old bug). See RFX_G_RAMP_SAMPLE_NS.
		 */
		if (demand_pct > p->prev_gaming_demand_pct + RFX_G_RAMP_DELTA_PCT &&
		    demand_pct >= follow_pct) {
			p->ramp_end_ns = time + RFX_G_RAMP_HOLD_NS;
			/*
			 * Consume the step: snap the reference UP to what just
			 * armed. Without this every eval inside the sample window
			 * still sees the same step and re-pushes ramp_end_ns, so a
			 * "2 eval" lift became a whole sample period. Snapping up
			 * only -- never down onto a trough -- keeps it bounded.
			 */
			p->prev_gaming_demand_pct = demand_pct;
			p->prev_gaming_demand_ns = time;
		} else if (!p->prev_gaming_demand_ns ||
			   time - p->prev_gaming_demand_ns >= RFX_G_RAMP_SAMPLE_NS) {
			p->prev_gaming_demand_pct = demand_pct;
			p->prev_gaming_demand_ns = time;
		}

		if (p->ramp_end_ns && time < p->ramp_end_ns && freq < boost_fl)
			freq = boost_fl;
	} else {
		bool ui_active, coldstart_active;
		unsigned int demand_pct;

		/*
		 * Raw demand, before headroom -- same correction the gaming band
		 * needs. Post-headroom `util` is not a load measurement: the daily
		 * tiers are stepped (+10% over 65/70, +5% over 40/45), so a tier
		 * crossing jumps the value up to 7 points with no load change, and
		 * the thresholds below were reading that (the 72% Little latch
		 * really fired at 66%). Same 1.25x DVFS-margin skew as the gaming
		 * band applies; see that note before re-tuning.
		 */
		demand_pct = (unsigned int)(raw_util * 100 / max_cap);

		/*
		 * Cold-start: a 0->high demand jump arms aggressive floors.
		 * Touch-gated like the ramp below: a launch always follows a tap,
		 * while a radio/RX wakeup on an idle device has the identical
		 * signature (idle base, one big step) and was arming the burst
		 * floor on every one -- the cellular-idle drain path.
		 */
		if (rfx_input_active(time) &&
		    demand_pct >= RFX_D_COLDSTART_DELTA_PCT &&
		    p->prev_upct <= RFX_D_COLDSTART_BASE_PCT)
			p->coldstart_boost_end_ns = time + RFX_D_COLDSTART_BOOST_NS;

		/*
		 * Render burst re-arms the UI window — gated on an active touch.
		 * Autoplay video decode spikes look identical to an animation
		 * ramp; ungated, passive watching (reels) pins the boost cap =
		 * daily heat with zero responsiveness gain. Touch-driven
		 * animation still boosts (input window covers the tap).
		 */
		if (rfx_input_active(time) &&
		    demand_pct > p->prev_upct &&
		    demand_pct - p->prev_upct >= RFX_D_RAMP_DELTA_PCT)
			p->ui_boost_end_ns = time + RFX_D_UI_BOOST_NS;

		/*
		 * Refresh the reference on a fixed cadence (RFX_D_RAMP_SAMPLE_NS)
		 * and nothing else. Snapping it down when demand fell turned the
		 * detector into an unbounded peak-to-trough comparator: any
		 * oscillating load (video, sync, sensor batch) re-armed at its own
		 * minimum, so the next upswing read as a 12-point ramp, each false
		 * positive pinning the Little floor and dragging all clusters to the
		 * 250us rate until the device never left the interaction state.
		 * Against a fixed 16ms reference the test means what it says: demand
		 * is 12 points up from one frame ago -- real scroll, not noise.
		 */
		if (!p->prev_upct_ns ||
		    time - p->prev_upct_ns >= RFX_D_RAMP_SAMPLE_NS) {
			p->prev_upct = demand_pct;
			p->prev_upct_ns = time;
		}

		coldstart_active = p->coldstart_boost_end_ns && time < p->coldstart_boost_end_ns;
		ui_active = rfx_input_active(time) ||
			    (p->ui_boost_end_ns && time < p->ui_boost_end_ns);

		if (little) {
			unsigned int cap = ui_active ?
				rfx_pct(fceil, RFX_D_LITTLE_BOOST_CAP_PCT) :
				rfx_pct(fceil, RFX_D_LITTLE_CAP_PCT);

			/*
			 * Sustained heavy load relaxes the cap: the 65%
			 * battery cap otherwise strangles every long
			 * multithread workload (compile, media scan,
			 * untouched-screen game) once the touch and burst
			 * windows expire. It relaxes to 80%, not to fmax -
			 * see RFX_D_LITTLE_SUSTAINED_CAP_PCT. Light use never
			 * reaches this demand, so idle battery is unaffected.
			 */
			if (demand_pct >= RFX_D_LITTLE_LIFT_PCT)
				p->little_cap_lifted = true;
			else if (demand_pct <= RFX_D_LITTLE_DROP_PCT)
				p->little_cap_lifted = false;
			if (p->little_cap_lifted)
				cap = max(cap, rfx_pct(fceil,
					RFX_D_LITTLE_SUSTAINED_CAP_PCT));

			if (freq > cap)
				freq = cap;

			/*
			 * Interaction floor, applied AFTER the cap so a lifted cap
			 * is never undercut and the floor never exceeds it. Touch/
			 * UI window only; see RFX_D_LITTLE_UI_FLOOR_PCT for why the
			 * input-pipeline cluster needs a floor, not just a cap.
			 */
			if (ui_active) {
				unsigned int fl = min(cap,
					rfx_pct(fceil, RFX_D_LITTLE_UI_FLOOR_PCT));

				if (freq < fl)
					freq = fl;
			}
		} else {
			/*
			 * Big/Prime daily cap, same model as Little: base cap
			 * keeps background work off the top OPPs; touch/UI lifts
			 * it for burst; a sustained-load latch lifts to 88% for
			 * long foreground work; cold-start floor applied after
			 * the cap (clamped to it).
			 */
			unsigned int cap, base_cap_pct, boost_cap_pct;

			if (prime) {
				base_cap_pct = RFX_D_PRIME_CAP_PCT;
				boost_cap_pct = RFX_D_PRIME_BOOST_CAP_PCT;
			} else {
				base_cap_pct = RFX_D_BIG_CAP_PCT;
				boost_cap_pct = RFX_D_BIG_BOOST_CAP_PCT;
			}

			cap = ui_active ?
				rfx_pct(fceil, boost_cap_pct) :
				rfx_pct(fceil, base_cap_pct);

			/* Sustained-load latch (Big/Prime share one latch). */
			if (demand_pct >= RFX_D_BIG_LIFT_PCT)
				p->big_cap_lifted = true;
			else if (demand_pct <= RFX_D_BIG_DROP_PCT)
				p->big_cap_lifted = false;
			if (p->big_cap_lifted)
				cap = max(cap, rfx_pct(fceil, prime ?
					RFX_D_PRIME_SUSTAINED_CAP_PCT :
					RFX_D_BIG_SUSTAINED_CAP_PCT));

			if (freq > cap)
				freq = cap;

			/* Cold-start burst floor, clamped to the cap. */
			if (coldstart_active) {
				unsigned int fl = rfx_pct(fceil, prime ?
						RFX_D_PRIME_BURST_FLOOR_PCT :
						RFX_D_BIG_BURST_FLOOR_PCT);

				fl = min(fl, cap);
				if (freq < fl)
					freq = fl;
			}
		}
	}

	freq = rfx_thermal_clamp(freq, fceil);
	freq = clamp(freq, fmin, fceil);

	/*
	 * Cache the raw request only once committed (see rfx_commit_freq).
	 * Writing it here unconditionally poisoned the cache when the rate gate
	 * rejected a commit: the next tick recomputed the same value, hit the
	 * cache, returned the stale next_freq, and lost the pending change until
	 * demand moved again -- flat segments in the trace with no load reason.
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
	 * Escalate toward a per-cluster ceiling. Little stays modest (IO
	 * completions there are housekeeping). Big/Prime split by profile:
	 * gaming 60% covers CPU-side work (decompression, parsing) without
	 * paying for the IO wait itself; daily 25% covers sqlite/log/image work
	 * at the V/f knee without a high OPP for ms-scale waits.
	 */
	if (rfx_c->iowait_boost) {
		max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
		if (rfx_cap_is_little(max_cap))
			cap = SCHED_CAPACITY_SCALE / 6;
		else if (rfx_gaming_enabled())
			cap = SCHED_CAPACITY_SCALE * 3 / 5;	/* 60% */
		else
			cap = SCHED_CAPACITY_SCALE / 4;		/* 25% */
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
	rfx_get_util_gki510(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
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
 * up-rate inside an 8.3ms frame budget is a frame-time tax, i.e. an FPS cap (a
 * 2ms gate on the fastest tier measured out at ~100fps). Tunable otherwise. */
static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns =
			(s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

/*
 * Evaluation delay for this update. Set BEFORE rfx_should_update_freq, so it
 * depends only on state known up front -- correct, since every reason for a
 * sub-ms cadence (gaming, touch window, UI-burst tail) is known without util.
 * Everything else, including idle Little, uses its tunable, so sysfs
 * rate_limit_us still means what it says.
 *
 * The daily interaction rate is scoped to the clusters in the interaction path:
 * a 3-tier Prime only carries daily spill, and its daily cap is flat at its base,
 * so ~1.1kHz sampling there bought no scroll responsiveness -- only governor work
 * in the scheduler hot path for the whole post-touch window. Keyed on the role,
 * not the tier, so on a 2-tier part the top cluster still gets the fast rate.
 */
static inline void rfx_set_eval_delay(struct rfx_policy *p, bool gaming, u64 time)
{
	if (gaming)
		p->freq_update_delay_ns = (s64)RFX_FAST_RATE_US * NSEC_PER_USEC;
	else if (!p->is_prime &&
		 (rfx_input_active(time) ||
		  (p->ui_boost_end_ns && time < p->ui_boost_end_ns)))
		p->freq_update_delay_ns = (s64)RFX_UI_RATE_US * NSEC_PER_USEC;
	else
		p->freq_update_delay_ns =
			(s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
}

/*
 * Evaluation gate. Measures from last_eval_time (stamped on every evaluation),
 * not from last commit — rfx_commit_freq() skips stamping when freq is
 * unchanged, so commit-based gating is permanently open when gaming floors
 * pin the clock. That was the 114fps overhead.
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
	 * Promoting here (rather than in rfx_target_freq) is what keeps a
	 * gate-rejected update from being silently dropped.
	 */
	p->cached_raw_freq = p->pending_raw_freq;
	p->next_freq = next_freq;
	return true;
}

/* ===================================================================== */
/* Update hooks                                                          */
/* ===================================================================== */

static void rfx_update_single_freq(struct update_util_data *hook, u64 time,
				   unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();
	unsigned long max_cap, boost, eff, irqflags;
	unsigned int next_f;
	bool do_deferred = false;
	u64 ema_delta;

	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);
	rfx_set_eval_delay(p, gaming, time);

	if (likely(!rfx_should_update_freq(p, time)))
		return;

	p->last_eval_time = time;
	boost = rfx_iowait_apply(rfx_c, time, max_cap);
	rfx_get_util(rfx_c, boost);
	eff = max(rfx_c->util, boost);
	ema_delta = p->last_ema_ns ? time - p->last_ema_ns : RFX_EMA_DECAY_PERIOD_NS;
	p->last_ema_ns = time;
	p->filt_util = rfx_ema(p->filt_util, eff, ema_delta, gaming);

	rfx_set_down_delay(p, gaming);
	rfx_pol_up_delay(p, gaming);

	next_f = rfx_target_freq(p, p->filt_util, max_cap, time, gaming);

	if (!rfx_commit_freq(p, time, next_f))
		return;

	if (p->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(p->policy, p->next_freq);
	} else {
		raw_spin_lock_irqsave(&p->update_lock, irqflags);
		if (!p->work_in_progress) {
			p->work_in_progress = true;
			do_deferred = true;
		}
		raw_spin_unlock_irqrestore(&p->update_lock, irqflags);

		if (do_deferred)
			irq_work_queue(&p->irq_work);
	}
}

static unsigned int rfx_next_freq_shared(struct rfx_cpu *rfx_c, u64 time,
					 bool gaming)
{
	struct rfx_policy *p = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	unsigned long max_util = 0;
	unsigned int j;
	u64 ema_delta;

	/*
	 * Aggregate max util across the policy's CPUs, then filter once. The EMA
	 * lives on the policy, not the CPU that ran the hook: a shared policy
	 * commits one frequency, so a per-CPU filter let the committed value flip
	 * with whichever CPU ticked last -- jitter with no change in load.
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

	ema_delta = p->last_ema_ns ? time - p->last_ema_ns : RFX_EMA_DECAY_PERIOD_NS;
	p->last_ema_ns = time;
	p->filt_util = rfx_ema(p->filt_util, max_util, ema_delta, gaming);

	rfx_set_down_delay(p, gaming);
	rfx_pol_up_delay(p, gaming);

	return rfx_target_freq(p, p->filt_util, max_cap, time, gaming);
}

static void rfx_update_shared(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();
	unsigned long irqflags;
	unsigned int next_f;
	bool do_deferred = false;
	bool do_fast_switch = false;

	raw_spin_lock_irqsave(&p->update_lock, irqflags);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);
	rfx_set_eval_delay(p, gaming, time);

	if (rfx_should_update_freq(p, time)) {
		p->last_eval_time = time;
		next_f = rfx_next_freq_shared(rfx_c, time, gaming);
		if (rfx_commit_freq(p, time, next_f)) {
			if (p->policy->fast_switch_enabled) {
				do_fast_switch = true;
			} else {
				if (!p->work_in_progress) {
					p->work_in_progress = true;
					do_deferred = true;
				}
			}
		}
	}

	raw_spin_unlock_irqrestore(&p->update_lock, irqflags);

	if (do_fast_switch)
		cpufreq_driver_fast_switch(p->policy, p->next_freq);

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
	cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);
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

	/*
	 * Latching emergency net with 7C hysteresis: trip once, hold until the
	 * die genuinely cools, release once -- it cannot chatter. The
	 * proportional relay it replaces re-derived a temperature target every
	 * 50ms against a 6ms/1% walk and fought both HW LMH and its own effect
	 * on temperature -- the shark-tooth sustain graph.
	 */
	if (have) {
		if (atomic_read(&rfx_emergency_cap_pct) >= 100) {
			if (t_mc >= RFX_TEMP_EMERGENCY_MC) {
				atomic_set(&rfx_emergency_cap_pct,
					   RFX_EMERGENCY_CAP_PCT);
				pr_warn_ratelimited("vorpal: thermal emergency %d mC, cap %d%%\n",
						    t_mc, RFX_EMERGENCY_CAP_PCT);
			}
		} else if (t_mc <= RFX_TEMP_EMERGENCY_CLEAR_MC) {
			atomic_set(&rfx_emergency_cap_pct, 100);
			pr_info("vorpal: thermal emergency cleared %d mC\n", t_mc);
		}
	} else {
		atomic_set(&rfx_emergency_cap_pct, 100);
	}

	/*
	 * Adaptive poll interval. Gaming: 100ms (unchanged). Daily warm
	 * (≥70°C): 2000ms for faster thermal response during charging + use.
	 * Daily cool: 5000ms (unchanged).
	 */
	if (rfx_gaming_enabled())
		delay_ms = RFX_THERMAL_POLL_GAMING_MS;
	else if (have && t_mc >= RFX_TEMP_WARM_MC)
		delay_ms = RFX_THERMAL_POLL_WARM_MS;
	else
		delay_ms = RFX_THERMAL_POLL_IDLE_MS;
	queue_delayed_work(system_power_efficient_wq, &rfx_thermal_work,
			   msecs_to_jiffies(delay_ms));
}

/* ===================================================================== */
/* Input handler (daily touch boost; off during gaming)                  */
/* ===================================================================== */

static void rfx_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	if (rfx_gaming_enabled())
		return;
	if (type == EV_ABS || type == EV_KEY)
		atomic64_set(&rfx_input_ts_ns, sched_clock());
}

static int rfx_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "vorpal";

	err = input_register_handle(handle);
	if (err)
		goto err_free;
	err = input_open_device(handle);
	if (err)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return err;
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
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
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

/* ===================================================================== */
/* sysfs                                                                 */
/* ===================================================================== */

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
	/*
	 * No need to push the new value into every policy here: every update
	 * calls rfx_set_eval_delay() before rfx_should_update_freq() reads
	 * freq_update_delay_ns, so the store below was overwritten before it
	 * was ever used.
	 */
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

/* Reset transient gaming residue on every live policy (all clusters). */
static void rfx_reset_all_policies(void)
{
	struct rfx_policy *p;
	unsigned long flags, pflags;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);

	atomic64_set(&rfx_frame_boost_end_ns, 0);
	atomic64_set(&rfx_frame_boost_arm_ns, 0);

	list_for_each_entry(p, &rfx_policy_list, gov_node) {
		raw_spin_lock_irqsave(&p->update_lock, pflags);
		p->frame_boost_ramp_pct = 0;
		p->frame_boost_ramp_last_ns = 0;
		p->gaming_warmup_end_ns = 0;
		p->gaming_warmup_start_ns = 0;
		p->risk_high = false;
		p->risk_since_ns = 0;
		p->thermal_cooling = false;
		p->floor_gated = false;
		p->floor_low_since_ns = 0;
		p->little_cap_lifted = false;
		p->big_cap_lifted = false;
		p->ramp_end_ns = 0;
		p->prev_gaming_demand_pct = 0;
		p->prev_gaming_demand_ns = 0;
		p->warmup_low_demand_since_ns = 0;
		/* Daily latches: exit == fresh daily, no stale boost window. */
		p->prev_upct = 0;
		p->prev_upct_ns = 0;
		p->ui_boost_end_ns = 0;
		p->coldstart_boost_end_ns = 0;
		/* Do not carry saturated gaming demand into the daily profile. */
		p->filt_util = 0;
		p->last_ema_ns = 0;
		p->need_freq_update = true;
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
		u64 now = sched_clock();

		spin_lock_irqsave(&rfx_policy_list_lock, flags);
		list_for_each_entry(p, &rfx_policy_list, gov_node) {
			raw_spin_lock_irqsave(&p->update_lock, pflags);
			p->need_freq_update = true;
			/* Warmup floor lift covers process spawn / asset load. */
			p->gaming_warmup_end_ns = now + RFX_GAMING_WARMUP_NS;
			p->gaming_warmup_start_ns = now;
			/* Clear daily state so it cannot shape gaming decisions. */
			p->little_cap_lifted = false;
			p->big_cap_lifted = false;
			p->ui_boost_end_ns = 0;
			p->coldstart_boost_end_ns = 0;
			p->prev_upct = 0;
			p->prev_upct_ns = 0;
			/* Stale latches from a previous session. */
			p->risk_high = false;
			p->risk_since_ns = 0;
			p->thermal_cooling = false;
			p->floor_gated = false;
			p->floor_low_since_ns = 0;
			p->ramp_end_ns = 0;
			p->prev_gaming_demand_pct = 0;
			p->prev_gaming_demand_ns = 0;
			p->warmup_low_demand_since_ns = 0;
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
	return count;
#else
	return -ENODEV;
#endif
}
static struct governor_attr thermal_zone = __ATTR_RW(thermal_zone);

static struct attribute *rfx_little_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_little);

static struct attribute *rfx_big_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_big);

static struct attribute *rfx_prime_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&gaming_mode.attr,
	&temp_mc.attr,
	&thermal_zone.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_prime);

static void rfx_tunables_free(struct kobject *kobj)
{
	kfree(to_rfx_tunables(rfx_to_gov_attr_set(kobj)));
}

static struct kobj_type rfx_little_ktype = {
	.default_groups = rfx_little_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};
static struct kobj_type rfx_big_ktype = {
	.default_groups = rfx_big_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};
static struct kobj_type rfx_prime_ktype = {
	.default_groups = rfx_prime_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};

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

static int rfx_kthread_create(struct rfx_policy *p)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = p->policy;
	struct sched_param sp = { .sched_priority = MAX_RT_PRIO / 2 };
	int ret;

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

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &sp);
	if (ret) {
		kthread_stop(thread);
		pr_warn("vorpal: failed to set SCHED_FIFO\n");
		return ret;
	}

	p->thread = thread;
	if (policy->dvfs_possible_from_any_cpu)
		set_cpus_allowed_ptr(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&p->irq_work, rfx_irq_work);
	mutex_init(&p->work_lock);
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
	struct kobj_type *ktype;
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
	p->is_top = rfx_cap_is_top(max_cap);
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
		ktype = &rfx_little_ktype;
	} else if (p->is_top) {
		/* is_top, not is_prime: on a two-tier SoC the band is BIG but
		 * gaming_mode must still exist somewhere. */
		t->rate_limit_us = RFX_PRIME_RATE_US;
		t->up_rate_limit_us = RFX_PRIME_UP_US;
		t->down_rate_limit_us = RFX_PRIME_DOWN_US;
		ktype = &rfx_prime_ktype;
	} else {
		t->rate_limit_us = RFX_BIG_RATE_US;
		t->up_rate_limit_us = RFX_BIG_UP_US;
		t->down_rate_limit_us = RFX_BIG_DOWN_US;
		ktype = &rfx_big_ktype;
	}

	policy->governor_data = p;
	p->tunables = t;

	ret = kobject_init_and_add(&t->attr_set.kobj, ktype,
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
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned long flags;
	unsigned int cpu;
	u64 now = sched_clock();

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
	p->need_freq_update = false;
	p->prev_upct = 0;
	p->prev_upct_ns = 0;
	p->ui_boost_end_ns = 0;
	p->coldstart_boost_end_ns = 0;
	p->filt_util = 0;
	p->last_ema_ns = 0;

	p->frame_boost_ramp_pct = 0;
	p->frame_boost_ramp_last_ns = 0;
	p->gaming_warmup_end_ns = 0;
	p->gaming_warmup_start_ns = 0;
	p->risk_high = false;
	p->risk_since_ns = 0;
	p->little_cap_lifted = false;
	p->big_cap_lifted = false;
	p->thermal_cooling = false;
	p->floor_gated = false;
	p->floor_low_since_ns = 0;
	p->ramp_end_ns = 0;
	p->prev_gaming_demand_pct = 0;
	p->prev_gaming_demand_ns = 0;
	p->warmup_low_demand_since_ns = 0;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_add(&p->gov_node, &rfx_policy_list);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = per_cpu_ptr(&rfx_cpu, cpu);

		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu = cpu;
		rfx_c->rfx_policy = p;
	}

	uu = policy_is_shared(policy) ? rfx_update_shared : rfx_update_single_freq;
	for_each_cpu(cpu, policy->cpus)
		cpufreq_add_update_util_hook(cpu, &per_cpu_ptr(&rfx_cpu, cpu)->update_util, uu);
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
 * gaming_mode is USER-OWNED: nothing in this driver ever writes it, and there is
 * deliberately no PM/suspend auto-clear. Cost: a session left at 1 keeps the
 * gaming eval rate and idle floors across post-suspend wakes.
 */

static int __init vorpal_gov_init(void)
{
	int ret;

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	INIT_DEFERRABLE_WORK(&rfx_thermal_work, rfx_thermal_fn);
	queue_delayed_work(system_power_efficient_wq, &rfx_thermal_work,
			   msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));

	if (input_register_handler(&rfx_input_handler))
		pr_warn("vorpal: input handler register failed (touch boost off)\n");

	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret) {
		input_unregister_handler(&rfx_input_handler);
		cancel_delayed_work_sync(&rfx_thermal_work);
	}
	return ret;
}

static void __exit vorpal_gov_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
	input_unregister_handler(&rfx_input_handler);
	cancel_delayed_work_sync(&rfx_thermal_work);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR("Steambot12");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.2");
