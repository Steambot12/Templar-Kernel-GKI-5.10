// SPDX-License-Identifier: GPL-2.0
/*
 * nap.c - Neural Adaptive Predictor cpuidle governor
 *
 * Author: Masahito Suzuki
 *
 * An MLP trunk (8->8, ReLU) feeds a shared score s = w_out . h + b_out into a
 * proportional-odds ordinal survival head: for each idle-state boundary k the
 * predicted probability that the upcoming idle reaches states[k]'s target
 * residency is q_k = sigmoid(s - thr_ord[k-1]).  q_k is shrunk toward a decayed
 * histogram of realized idles, and the deepest feasible state whose survival
 * still meets the confidence level wins.  Weights are seeded to reproduce the
 * deepest-state-that-fits default, then refined online by deferred SGD.
 *
 * Backported from the 6.18 original.  All arithmetic is Q12.20 fixed point
 * rather than SSE2/AVX2 float: cpuidle_select() runs with interrupts disabled,
 * where kernel-mode FPU/SIMD is not usable on every architecture, and integer
 * math keeps the result identical across CPUs.
 */

#define pr_fmt(fmt) "nap: " fmt

#include <linux/bitops.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/math64.h>
#include <linux/percpu.h>
#include <linux/pm_qos.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/tick.h>

#define NAP_VERSION		"0.5.0"

#define NAP_INPUT_SIZE		8
#define NAP_HIDDEN_SIZE		8
#define NAP_NUM_CUTS		(CPUIDLE_STATE_MAX - 1)
#define NAP_HISTORY_SIZE	8

/* Robustness floor: decay 1 - 2^-8 per idle, i.e. a 256-idle window. */
#define NAP_FLOOR_WIN_SHIFT	8
/* NN survival counts as this many pseudo-observations against the floor. */
#define NAP_PRIOR_K		16

#define NAP_DEFAULT_LR_MILLTHS		1	/* 0.001 */
#define NAP_DEFAULT_INTERVAL		4	/* learn every 4 reflects */
#define NAP_DEFAULT_CLAMP_MILLTHS	1000	/* 1.0 */
#define NAP_DEFAULT_CONF_MILLTHS	500	/* 0.5 */

/*
 * POLL short-circuit: dev->poll_limit_ns floor and margin.  Both 1 us, the
 * granularity at which a polling state samples its own timeout.
 */
#define NAP_POLL_LIMIT_MIN_NS		1000ULL
#define NAP_POLL_LIMIT_MARGIN_NS	1000ULL

/* Bounds staleness of the cached shallowest-valid-state lookup. */
#define NAP_MIN_STATE_REFRESH_JIFFIES	HZ

/* ================================================================
 * Q12.20 fixed-point math
 * ================================================================ */

#define NAP_FX_SHIFT	20
#define NAP_FX_ONE	(1 << NAP_FX_SHIFT)
/* ~1024.0.  Keeps a*b within s64 and (e << SHIFT) within s32 for e <= 63. */
#define NAP_FX_MAX	((s32)0x3fffffff)

/* log2(1+m), m in [0,1): minimax cubic through both endpoints, err < 9e-4 */
#define NAP_L2_C1	1491982
#define NAP_L2_C2	(-610361)
#define NAP_L2_C3	166954
/* 2^f - 1, f in [0,1): same form, err < 1.5e-4 */
#define NAP_E2_C1	729694
#define NAP_E2_C2	235788
#define NAP_E2_C3	83094
#define NAP_LOG2E	1512775

static inline s32 nap_fx_sat(s64 v)
{
	if (v > NAP_FX_MAX)
		return NAP_FX_MAX;
	if (v < -NAP_FX_MAX)
		return -NAP_FX_MAX;
	return (s32)v;
}

/* Truncation is toward zero on both signs: an asymmetric shift would bias
 * every SGD update negative by up to 2^-20 and drift the weights.
 */
static inline s32 nap_fx_mul(s32 a, s32 b)
{
	s64 p = (s64)a * b;

	return nap_fx_sat(p < 0 ? -((-p) >> NAP_FX_SHIFT) : p >> NAP_FX_SHIFT);
}

static inline s32 nap_fx_clamp(s32 v, s32 lo, s32 hi)
{
	if (v < lo)
		return lo;
	if (v > hi)
		return hi;
	return v;
}

/* A sysfs tunable in thousandths to Q12.20.  Exact, unlike ONE/1000. */
static inline s32 nap_fx_from_millths(unsigned int millths)
{
	return (s32)(((u32)millths << NAP_FX_SHIFT) / 1000u);
}

/* log2(x) for x >= 1; 0 for x == 0. */
static s32 nap_fx_log2(u64 x)
{
	s64 m, p;
	int e;

	if (unlikely(!x))
		return 0;

	e = fls64(x) - 1;
	m = (e >= NAP_FX_SHIFT) ? (s64)(x >> (e - NAP_FX_SHIFT))
				: (s64)(x << (NAP_FX_SHIFT - e));
	m -= NAP_FX_ONE;

	p = NAP_L2_C3;
	p = NAP_L2_C2 + ((m * p) >> NAP_FX_SHIFT);
	p = NAP_L2_C1 + ((m * p) >> NAP_FX_SHIFT);
	p = (m * p) >> NAP_FX_SHIFT;

	return nap_fx_sat(((s64)e << NAP_FX_SHIFT) + p);
}

/* sigmoid(x) = 1 / (1 + 2^(-x * log2e)), result in [0, NAP_FX_ONE]. */
static s32 nap_fx_sigmoid(s32 x)
{
	s64 t = -(((s64)x * NAP_LOG2E) >> NAP_FX_SHIFT);
	s64 f, p;
	int e;

	if (t >= 24 * NAP_FX_ONE)
		return 0;
	if (t <= -24 * NAP_FX_ONE)
		return NAP_FX_ONE;

	e = (int)(t >> NAP_FX_SHIFT);		/* floors, t may be negative */
	f = t - ((s64)e << NAP_FX_SHIFT);

	p = NAP_E2_C3;
	p = NAP_E2_C2 + ((f * p) >> NAP_FX_SHIFT);
	p = NAP_E2_C1 + ((f * p) >> NAP_FX_SHIFT);
	p = NAP_FX_ONE + ((f * p) >> NAP_FX_SHIFT);

	p = (e >= 0) ? (p << e) : (p >> -e);

	return (s32)div64_u64((u64)NAP_FX_ONE << NAP_FX_SHIFT,
			      (u64)NAP_FX_ONE + (u64)p);
}

/* ================================================================
 * Per-CPU data
 * ================================================================ */

/*
 * Trunk: input[8] -> hidden[8] (ReLU), then a shared score
 *   s = w_out . hidden + b_out
 * feeding the ordinal head q_k = sigmoid(s - thr_ord[k-1]).
 *
 * w_h1[j][i] is the weight from input j to hidden neuron i (column-major, as
 * in the original, so a row is one input's fan-out).
 */
struct nap_weights {
	s32 w_h1[NAP_INPUT_SIZE][NAP_HIDDEN_SIZE];
	s32 b_h1[NAP_HIDDEN_SIZE];
	s32 w_out[NAP_HIDDEN_SIZE];
	s32 b_out;
	s32 thr_ord[NAP_NUM_CUTS];
};

struct nap_stats {
	u64 total_selects;
	u64 total_residency_ns;
	u64 overshoot_count;
	u64 learn_count;
};

struct nap_cpu_data {
	/* log2(residency) ring, feeds the avg/min/max features */
	s32	log_history[NAP_HISTORY_SIZE];
	int	hist_idx;
	int	hist_count;

	u64	prev_idle_exit;
	s64	last_predicted_ns;
	s64	last_prediction_error;

	/* POLL short-circuit */
	bool	short_circuited;
	int	cached_min_state;
	s64	cached_min_state_latency;
	unsigned long cached_min_state_jiffies;

	unsigned long last_learn_jiffies;
	unsigned int  learn_jiffies_min;	/* 0 = disabled */

	/* forward-pass state consumed by the next learn */
	s32	nn_output;		/* shared ordinal score s */
	s32	hidden_out[NAP_HIDDEN_SIZE];
	s32	features[NAP_INPUT_SIZE];

	/* log2(target_residency) per state: threshold seed and timer clamp */
	s32	log2_tres[CPUIDLE_STATE_MAX];
	/* decayed per-bin idle histogram: robustness-floor survival */
	u32	bin_count[CPUIDLE_STATE_MAX];

	bool	needs_learn;
	bool	have_sample;
	bool	reset_pending;		/* set by sysfs, consumed by select */
	u64	learn_actual_ns;

	struct nap_weights weights;

	unsigned int learning_rate_millths;
	unsigned int max_grad_norm_millths;
	unsigned int conf_millths;
	unsigned int learn_interval;
	unsigned int learn_counter;
	bool	initialized;

	struct nap_stats stats;
};

static DEFINE_PER_CPU(struct nap_cpu_data, nap_data);

/* ================================================================
 * Forward pass and online learning
 * ================================================================ */

static void nap_nn_forward(struct nap_cpu_data *d)
{
	const struct nap_weights *w = &d->weights;
	s64 acc;
	int i, j;

	for (i = 0; i < NAP_HIDDEN_SIZE; i++) {
		acc = w->b_h1[i];
		for (j = 0; j < NAP_INPUT_SIZE; j++)
			acc += ((s64)d->features[j] * w->w_h1[j][i]) >>
			       NAP_FX_SHIFT;
		d->hidden_out[i] = acc > 0 ? nap_fx_sat(acc) : 0;   /* ReLU */
	}

	acc = w->b_out;
	for (i = 0; i < NAP_HIDDEN_SIZE; i++)
		acc += ((s64)d->hidden_out[i] * w->w_out[i]) >> NAP_FX_SHIFT;

	d->nn_output = nap_fx_sat(acc);
}

/*
 * SGD on the trunk and score head.  d_out is the score gradient
 * g = sum_k (q_k - y_k) computed by the caller; every update is
 * element-wise gradient-clamped, matching the original.
 */
static void nap_nn_learn(struct nap_cpu_data *d, s32 d_out, s32 lr)
{
	struct nap_weights *w = &d->weights;
	s32 clamp_val = nap_fx_from_millths(d->max_grad_norm_millths);
	s32 d_hid[NAP_HIDDEN_SIZE];
	int i, j;

	/* d_hid[i] = relu'(h[i]) * w_out[i] * d_out, using pre-update w_out */
	for (i = 0; i < NAP_HIDDEN_SIZE; i++)
		d_hid[i] = d->hidden_out[i] > 0 ?
			   nap_fx_mul(w->w_out[i], d_out) : 0;

	for (i = 0; i < NAP_HIDDEN_SIZE; i++)
		w->w_out[i] -= nap_fx_mul(lr,
			nap_fx_clamp(nap_fx_mul(d->hidden_out[i], d_out),
				     -clamp_val, clamp_val));

	w->b_out -= nap_fx_mul(lr, nap_fx_clamp(d_out, -clamp_val, clamp_val));

	for (j = 0; j < NAP_INPUT_SIZE; j++)
		for (i = 0; i < NAP_HIDDEN_SIZE; i++)
			w->w_h1[j][i] -= nap_fx_mul(lr,
				nap_fx_clamp(nap_fx_mul(d->features[j],
							d_hid[i]),
					     -clamp_val, clamp_val));

	for (i = 0; i < NAP_HIDDEN_SIZE; i++)
		w->b_h1[i] -= nap_fx_mul(lr, nap_fx_clamp(d_hid[i],
							  -clamp_val,
							  clamp_val));
}

/* ================================================================
 * Initialization
 * ================================================================ */

#define NAP_PRNG_SEED	42u
/* sqrt(6 / (fan_in + fan_out)) for 8 -> 8 */
#define NAP_XAVIER_FX	642119

static inline s32 nap_prng_fx(u32 *state)
{
	*state = *state * 1664525u + 1013904223u;
	/* s32 -> [-1, 1) in Q12.20 */
	return (s32)((s64)(s32)*state >> (31 - NAP_FX_SHIFT));
}

/*
 * Hidden neuron 0 is a pass-through for feature 0 = log2(sleep_length), and
 * w_out[0] = 1, so the untrained score is log2(sleep_length).  Paired with the
 * threshold seeding below, that reproduces the deepest-state-that-fits default
 * before any learning happens.  The other neurons are Xavier-initialized with
 * near-zero output weights, contributing almost nothing until trained.
 */
static void nap_init_weights(struct nap_weights *w)
{
	u32 rng = NAP_PRNG_SEED;
	int i, j;

	memset(w, 0, sizeof(*w));

	for (j = 0; j < NAP_INPUT_SIZE; j++)
		for (i = 0; i < NAP_HIDDEN_SIZE; i++)
			w->w_h1[j][i] = nap_fx_mul(nap_prng_fx(&rng),
						   NAP_XAVIER_FX);

	for (i = 0; i < NAP_HIDDEN_SIZE; i++)
		w->w_out[i] = nap_fx_mul(nap_prng_fx(&rng),
					 NAP_FX_ONE / 100);

	for (j = 0; j < NAP_INPUT_SIZE; j++)
		w->w_h1[j][0] = 0;
	w->w_h1[0][0] = NAP_FX_ONE;
	w->w_out[0] = NAP_FX_ONE;
}

/*
 * log2_tres[k] is boundary k's location in score space: it seeds thr_ord[k-1],
 * bounds that threshold's learned drift, and clamps the score against the next
 * timer in the decision layer.
 */
static void nap_init_state_data(struct nap_cpu_data *d,
				struct cpuidle_driver *drv)
{
	int i;

	memset(d->log2_tres, 0, sizeof(d->log2_tres));
	memset(d->bin_count, 0, sizeof(d->bin_count));

	for (i = 0; i < drv->state_count; i++)
		d->log2_tres[i] =
			nap_fx_log2(max_t(u64, drv->states[i].target_residency_ns, 1));

	/*
	 * Seed each threshold at its own boundary so q_k crosses 0.5 exactly
	 * when the score reaches that state's target residency.
	 */
	for (i = 1; i < drv->state_count && i - 1 < NAP_NUM_CUTS; i++)
		d->weights.thr_ord[i - 1] = d->log2_tres[i];
}

/* ================================================================
 * Feature extraction
 * ================================================================ */

struct nap_logring {
	s32 avg;
	s32 min;
	s32 max;
};

static void nap_logring_stats(const struct nap_cpu_data *d,
			      struct nap_logring *s)
{
	int i, n = d->hist_count;
	s64 sum;

	if (!n) {
		s->avg = 0;
		s->min = 0;
		s->max = 0;
		return;
	}

	sum = d->log_history[0];
	s->min = d->log_history[0];
	s->max = d->log_history[0];

	for (i = 1; i < n; i++) {
		s32 v = d->log_history[i];

		sum += v;
		if (v < s->min)
			s->min = v;
		if (v > s->max)
			s->max = v;
	}

	s->avg = (s32)div_s64(sum, n);
}

static void nap_extract_features(struct nap_cpu_data *d,
				 struct cpuidle_driver *drv,
				 u64 sleep_length_ns, s64 latency_req)
{
	u64 deepest_lat = drv->states[drv->state_count - 1].exit_latency_ns;
	s64 err_us = div_s64(d->last_prediction_error, 1000);
	u64 now = local_clock();
	struct nap_logring lr;
	s32 mag;
	int prev;

	d->features[0] = nap_fx_log2(max_t(u64, sleep_length_ns, 1));

	prev = (d->hist_idx - 1 + NAP_HISTORY_SIZE) % NAP_HISTORY_SIZE;
	d->features[1] = d->hist_count ? d->log_history[prev] : 0;

	nap_logring_stats(d, &lr);
	d->features[2] = lr.avg;
	d->features[3] = lr.min;
	d->features[4] = lr.max;

	/* sign-preserving log2(|prediction error in us| + 1) */
	mag = nap_fx_log2((err_us < 0 ? -err_us : err_us) + 1);
	d->features[5] = err_us < 0 ? -mag : mag;

	/* busy time since the last idle exit; 0 before the first one */
	d->features[6] = nap_fx_log2(now > d->prev_idle_exit ?
				     now - d->prev_idle_exit : 1);

	/* PM QoS headroom over the deepest state's exit latency */
	if (latency_req < PM_QOS_LATENCY_ANY_NS && deepest_lat)
		d->features[7] = nap_fx_sat(
			(s64)nap_fx_log2(max_t(s64, latency_req, 1)) -
			nap_fx_log2(deepest_lat));
	else
		d->features[7] = 0;

	d->last_predicted_ns = sleep_length_ns;
}

/* ================================================================
 * Per-idle feedback
 * ================================================================ */

/*
 * Every idle updates the decayed floor histogram.  Only every learn_interval
 * (needs_learn) do the ordinal thresholds and the trunk get a gradient step,
 * using the previous pass's score, activations and features.  Under the shared-
 * score proportional-odds model the gradient w.r.t. the score is the scalar
 * g = sum_k (q_k - y_k), so one backprop drives every boundary.  The loss is
 * symmetric; responsiveness bias lives in the decision layer, not here.
 */
static void nap_update(struct nap_cpu_data *d, struct cpuidle_driver *drv)
{
	int k, label_bin = 0;

	if (d->needs_learn) {
		s32 lr = nap_fx_from_millths(d->learning_rate_millths);
		s32 clamp_val = nap_fx_from_millths(d->max_grad_norm_millths);
		s32 g = 0;

		for (k = 1; k < drv->state_count; k++) {
			s32 th = d->weights.thr_ord[k - 1];
			s32 q = nap_fx_sigmoid(d->nn_output - th);
			s32 err = q - (d->learn_actual_ns >=
				       drv->states[k].target_residency_ns ?
				       NAP_FX_ONE : 0);

			g += err;
			d->weights.thr_ord[k - 1] = nap_fx_clamp(
				th + nap_fx_clamp(nap_fx_mul(lr, err),
						  -clamp_val, clamp_val),
				d->log2_tres[k] - 6 * NAP_FX_ONE,
				d->log2_tres[k] + 6 * NAP_FX_ONE);
		}

		nap_nn_learn(d, g, lr);
		d->stats.learn_count++;
		d->needs_learn = false;
	}

	for (k = 1; k < drv->state_count; k++)
		if (d->learn_actual_ns >= drv->states[k].target_residency_ns)
			label_bin = k;

	for (k = 0; k < drv->state_count; k++)
		d->bin_count[k] -= d->bin_count[k] >> NAP_FLOOR_WIN_SHIFT;
	d->bin_count[label_bin] += NAP_FX_ONE;

	d->have_sample = false;
}

/*
 * For each boundary k the survival q_k is a Beta-Binomial shrinkage of the NN
 * survival sigmoid(s - thr_ord) -- a prior worth NAP_PRIOR_K pseudo-
 * observations -- toward the decayed histogram (the data): the NN drives cold
 * start, the floor takes over as it fills.  A running minimum enforces a
 * monotone non-increasing survival curve, and the next timer event caps the
 * reachable depth.  Confidence is the single responsiveness dial: take the
 * deepest feasible state whose survival still meets it.
 */
static int nap_decide(struct nap_cpu_data *d, struct cpuidle_driver *drv,
		      struct cpuidle_device *dev, s64 latency_req)
{
	s32 conf = nap_fx_from_millths(d->conf_millths);
	u32 suffix[CPUIDLE_STATE_MAX];
	u64 total = 0;
	s32 qmin = NAP_FX_ONE;
	int k, m = 0, idx = 0;

	for (k = 0; k < drv->state_count; k++)
		total += d->bin_count[k];

	suffix[drv->state_count - 1] = d->bin_count[drv->state_count - 1];
	for (k = drv->state_count - 2; k >= 0; k--)
		suffix[k] = suffix[k + 1] + d->bin_count[k];

	for (k = 1; k < drv->state_count; k++) {
		s32 q_nn = nap_fx_sigmoid(d->nn_output -
					  d->weights.thr_ord[k - 1]);
		s32 q = (s32)div64_u64(((u64)NAP_PRIOR_K * q_nn + suffix[k]) <<
				       NAP_FX_SHIFT,
				       (u64)NAP_PRIOR_K * NAP_FX_ONE + total);

		/* cannot idle past the next timer */
		if (d->log2_tres[k] > d->features[0])
			q = 0;
		if (q < qmin)
			qmin = q;

		if (qmin < conf)
			break;
		m = k;
	}

	for (k = m; k >= 1; k--) {
		if (dev->states_usage[k].disable)
			continue;
		if (drv->states[k].exit_latency_ns > (u64)latency_req)
			continue;
		idx = k;
		break;
	}

	return idx;
}

/* ================================================================
 * Governor callbacks
 * ================================================================ */

/* Shallowest enabled state meeting the latency request, 0 if none. */
static int nap_find_min_valid_state(struct cpuidle_driver *drv,
				    struct cpuidle_device *dev,
				    s64 latency_req)
{
	int i;

	for (i = 1; i < drv->state_count; i++) {
		if (dev->states_usage[i].disable)
			continue;
		if (drv->states[i].exit_latency_ns > (u64)latency_req)
			continue;
		return i;
	}
	return 0;
}

/*
 * Cached wrapper: invalidated when latency_req changes (immediate PM QoS
 * propagation) or every NAP_MIN_STATE_REFRESH_JIFFIES, which bounds staleness
 * from rare sysfs / runtime state-disable events.
 */
static inline int nap_get_min_valid_state(struct nap_cpu_data *d,
					  struct cpuidle_driver *drv,
					  struct cpuidle_device *dev,
					  s64 latency_req)
{
	if (unlikely(latency_req != d->cached_min_state_latency ||
		     time_after(jiffies, d->cached_min_state_jiffies +
					 NAP_MIN_STATE_REFRESH_JIFFIES))) {
		d->cached_min_state = nap_find_min_valid_state(drv, dev,
							       latency_req);
		d->cached_min_state_latency = latency_req;
		d->cached_min_state_jiffies = jiffies;
	}
	return d->cached_min_state;
}

/*
 * Polling budget for the short-circuit path: predicted wake time plus a 1 us
 * margin so a slightly late wake does not retrigger select/enter/reflect,
 * capped at the shallowest state's target residency (past which that state
 * would have been the better choice).
 */
static inline u64 nap_poll_limit(u64 sleep_length_ns, u64 min_state_target_ns)
{
	return clamp_t(u64, sleep_length_ns + NAP_POLL_LIMIT_MARGIN_NS,
		       NAP_POLL_LIMIT_MIN_NS, min_state_target_ns);
}

static int nap_select(struct cpuidle_driver *drv, struct cpuidle_device *dev,
		      bool *stop_tick)
{
	struct nap_cpu_data *d = this_cpu_ptr(&nap_data);
	ktime_t delta_tick;
	u64 sleep_length_ns;
	s64 latency_req;
	int idx, min_state;

	if (unlikely(!d->initialized || drv->state_count <= 1)) {
		*stop_tick = false;
		d->short_circuited = true;
		return 0;
	}

	latency_req = cpuidle_governor_latency_req(dev->cpu);
	sleep_length_ns = ktime_to_ns(tick_nohz_get_sleep_length(&delta_tick));
	min_state = nap_get_min_valid_state(d, drv, dev, latency_req);

	/*
	 * Fast path: with no C-state able to amortize its target residency
	 * within the predicted sleep length the answer is deterministically
	 * the shallowest state, so skip inference entirely.  nap_reflect drops
	 * these events, keeping them out of the training distribution.
	 */
	if (min_state == 0 ||
	    sleep_length_ns < drv->states[min_state].target_residency_ns) {
		if (min_state > 0)
			dev->poll_limit_ns = nap_poll_limit(sleep_length_ns,
				drv->states[min_state].target_residency_ns);
		else
			dev->poll_limit_ns = max_t(u64, sleep_length_ns,
						   NAP_POLL_LIMIT_MIN_NS);

		*stop_tick = !(drv->states[0].flags & CPUIDLE_FLAG_POLLING);
		d->short_circuited = true;
		d->stats.total_selects++;
		return 0;
	}

	d->short_circuited = false;

	if (unlikely(d->reset_pending)) {
		nap_init_weights(&d->weights);
		nap_init_state_data(d, drv);
		d->have_sample = false;
		d->needs_learn = false;
		d->hist_idx = 0;
		d->hist_count = 0;
		d->reset_pending = false;
	}

	if (d->have_sample)
		nap_update(d, drv);

	nap_extract_features(d, drv, sleep_length_ns, latency_req);
	nap_nn_forward(d);
	idx = nap_decide(d, drv, dev, latency_req);

	/*
	 * Don't stop the tick for a polling state or an idle shorter than the
	 * tick period; if it stays running, keep the target residency within
	 * the time to the closest timer.
	 */
	if (((drv->states[idx].flags & CPUIDLE_FLAG_POLLING) ||
	     sleep_length_ns < TICK_NSEC) && !tick_nohz_tick_stopped()) {
		*stop_tick = false;

		while (idx > 0 &&
		       drv->states[idx].target_residency_ns > delta_tick) {
			idx--;
			while (idx > 0 && dev->states_usage[idx].disable)
				idx--;
		}
	} else {
		*stop_tick = true;
	}

	d->stats.total_selects++;

	return idx;
}

static void nap_reflect(struct cpuidle_device *dev, int index)
{
	struct nap_cpu_data *d = this_cpu_ptr(&nap_data);
	struct cpuidle_driver *drv = cpuidle_get_cpu_driver(dev);
	u64 measured_ns = dev->last_residency_ns;

	dev->last_state_idx = index;

	if (unlikely(!drv || !d->initialized))
		return;

	/*
	 * A short-circuited idle never reached the network, so its residency is
	 * not part of the training distribution: account it and nothing else.
	 */
	if (d->short_circuited) {
		d->stats.total_residency_ns += measured_ns;
		return;
	}

	d->log_history[d->hist_idx] = nap_fx_log2(max_t(u64, measured_ns, 1));
	d->hist_idx = (d->hist_idx + 1) % NAP_HISTORY_SIZE;
	if (d->hist_count < NAP_HISTORY_SIZE)
		d->hist_count++;

	d->last_prediction_error = d->last_predicted_ns - (s64)measured_ns;
	d->prev_idle_exit = local_clock();

	d->learn_actual_ns = measured_ns;
	d->have_sample = true;

	/*
	 * Throttle the weight update with a dual gate: the per-N-reflect
	 * counter AND a jiffies floor.  The time gate caps the learning rate
	 * on workloads with very rapid idle bursts; learn_jiffies_min == 0
	 * disables it and restores counter-only behavior.
	 */
	if (++d->learn_counter >= d->learn_interval &&
	    time_after_eq(jiffies, d->last_learn_jiffies +
				   d->learn_jiffies_min)) {
		d->learn_counter = 0;
		d->last_learn_jiffies = jiffies;
		d->needs_learn = true;
	}

	d->stats.total_residency_ns += measured_ns;
	if (index > 0 && measured_ns < drv->states[index].target_residency_ns)
		d->stats.overshoot_count++;
}

static int nap_enable_device(struct cpuidle_driver *drv,
			     struct cpuidle_device *dev)
{
	struct nap_cpu_data *d = per_cpu_ptr(&nap_data, dev->cpu);
	struct nap_stats stats = d->stats;

	memset(d, 0, sizeof(*d));
	d->stats = stats;

	d->learning_rate_millths = NAP_DEFAULT_LR_MILLTHS;
	d->max_grad_norm_millths = NAP_DEFAULT_CLAMP_MILLTHS;
	d->conf_millths = NAP_DEFAULT_CONF_MILLTHS;
	d->learn_interval = NAP_DEFAULT_INTERVAL;
	d->learn_jiffies_min = 1;

	/* force a first-call refresh of the min-valid-state cache */
	d->cached_min_state_latency = S64_MIN;
	d->cached_min_state_jiffies = jiffies - NAP_MIN_STATE_REFRESH_JIFFIES;

	nap_init_weights(&d->weights);
	nap_init_state_data(d, drv);
	d->initialized = true;

	return 0;
}

/* ================================================================
 * sysfs: /sys/devices/system/cpu/nap/
 * ================================================================ */

static ssize_t stats_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	u64 sel = 0, res = 0, over = 0, learn = 0;
	int cpu, len = 0;

	for_each_possible_cpu(cpu) {
		struct nap_cpu_data *d = per_cpu_ptr(&nap_data, cpu);

		sel   += d->stats.total_selects;
		res   += d->stats.total_residency_ns;
		over  += d->stats.overshoot_count;
		learn += d->stats.learn_count;
	}

	len += sysfs_emit_at(buf, len, "total_selects: %llu\n", sel);
	len += sysfs_emit_at(buf, len, "total_residency_ms: %llu\n",
			     div_u64(res, NSEC_PER_MSEC));
	len += sysfs_emit_at(buf, len, "overshoot_count: %llu\n", over);
	len += sysfs_emit_at(buf, len, "overshoot_rate_permil: %llu\n",
			     sel ? div64_u64(over * 1000, sel) : 0);
	len += sysfs_emit_at(buf, len, "learn_count: %llu\n", learn);
	return len;
}

static ssize_t version_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%s\n", NAP_VERSION);
}

#define NAP_TUNABLE(name, field, lo, hi)				\
static ssize_t name##_show(struct kobject *kobj,			\
			   struct kobj_attribute *attr, char *buf)	\
{									\
	int cpu = cpumask_first(cpu_online_mask);			\
									\
	if (cpu >= nr_cpu_ids)						\
		return sysfs_emit(buf, "0\n");				\
	return sysfs_emit(buf, "%u\n", per_cpu_ptr(&nap_data, cpu)->field); \
}									\
static ssize_t name##_store(struct kobject *kobj,			\
			    struct kobj_attribute *attr,		\
			    const char *buf, size_t count)		\
{									\
	unsigned int val;						\
	int cpu;							\
									\
	if (kstrtouint(buf, 10, &val) || val < (lo) || val > (hi))	\
		return -EINVAL;						\
									\
	for_each_possible_cpu(cpu)					\
		per_cpu_ptr(&nap_data, cpu)->field = val;		\
									\
	return count;							\
}

NAP_TUNABLE(learning_rate,  learning_rate_millths, 1, 100)
NAP_TUNABLE(learn_interval, learn_interval,        1, 10000)
/*
 * confidence: survival level a state must clear, in millths.  Higher demands
 * more certainty before going deeper (biases responsiveness), lower biases
 * energy.  This is the single responsiveness dial.
 */
NAP_TUNABLE(confidence,     conf_millths,          1, 999)

static ssize_t reset_weights_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	cpumask_var_t mask;
	int cpu;

	/*
	 * Only flag the CPU; it reinitializes its own weights from inside
	 * nap_select(), so no remote writer ever races the idle path.
	 * Accepts "all" or a cpulist ("0-3,5").
	 */
	if (sysfs_streq(buf, "all")) {
		for_each_possible_cpu(cpu)
			per_cpu_ptr(&nap_data, cpu)->reset_pending = true;
		return count;
	}

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	if (cpulist_parse(buf, mask)) {
		free_cpumask_var(mask);
		return -EINVAL;
	}

	for_each_cpu(cpu, mask)
		per_cpu_ptr(&nap_data, cpu)->reset_pending = true;

	free_cpumask_var(mask);
	return count;
}

static ssize_t reset_stats_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int cpu;

	for_each_possible_cpu(cpu)
		memset(&per_cpu_ptr(&nap_data, cpu)->stats, 0,
		       sizeof(struct nap_stats));

	return count;
}

static struct kobj_attribute version_attr	 = __ATTR_RO(version);
static struct kobj_attribute stats_attr		 = __ATTR_RO(stats);
static struct kobj_attribute learning_rate_attr	 = __ATTR_RW(learning_rate);
static struct kobj_attribute learn_interval_attr = __ATTR_RW(learn_interval);
static struct kobj_attribute confidence_attr	 = __ATTR_RW(confidence);
static struct kobj_attribute reset_weights_attr	 = __ATTR_WO(reset_weights);
static struct kobj_attribute reset_stats_attr	 = __ATTR_WO(reset_stats);

static struct attribute *nap_attrs[] = {
	&version_attr.attr,
	&stats_attr.attr,
	&learning_rate_attr.attr,
	&learn_interval_attr.attr,
	&confidence_attr.attr,
	&reset_weights_attr.attr,
	&reset_stats_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(nap);

static struct kobject *nap_kobj;

/* ================================================================
 * Self-test for the fixed-point kernels (CONFIG_CPU_IDLE_GOV_NAP_SELFTEST)
 * ================================================================ */

#ifdef CONFIG_CPU_IDLE_GOV_NAP_SELFTEST
static void __init nap_selftest(void)
{
	static const struct { u64 x; s32 want; } log2_vec[] __initconst = {
		{ 1, 0 }, { 2, NAP_FX_ONE }, { 1024, 10 * NAP_FX_ONE },
		{ 1000000000ULL, 31348724 }, { U64_MAX, 67108861 },
	};
	s32 prev;
	int i, fail = 0;

	/* exactness at powers of two, and < 1e-3 elsewhere */
	for (i = 0; i < ARRAY_SIZE(log2_vec); i++) {
		s32 got = nap_fx_log2(log2_vec[i].x);

		if (abs(got - log2_vec[i].want) > NAP_FX_ONE / 1024) {
			pr_err("selftest: log2(%llu) = %d, want %d\n",
			       log2_vec[i].x, got, log2_vec[i].want);
			fail++;
		}
	}

	/* sigmoid: centred, bounded, saturating and monotone */
	if (nap_fx_sigmoid(0) != NAP_FX_ONE / 2) {
		pr_err("selftest: sigmoid(0) = %d\n", nap_fx_sigmoid(0));
		fail++;
	}
	if (nap_fx_sigmoid(64 * NAP_FX_ONE) != NAP_FX_ONE ||
	    nap_fx_sigmoid(-64 * NAP_FX_ONE) != 0) {
		pr_err("selftest: sigmoid does not saturate\n");
		fail++;
	}
	prev = -1;
	for (i = -32 * NAP_FX_ONE; i <= 32 * NAP_FX_ONE; i += 4093) {
		s32 q = nap_fx_sigmoid(i);

		if (q < prev || q < 0 || q > NAP_FX_ONE) {
			pr_err("selftest: sigmoid(%d) = %d breaks monotonicity\n",
			       i, q);
			fail++;
			break;
		}
		prev = q;
	}

	/* saturating multiply must not wrap, and must round toward zero */
	if (nap_fx_mul(NAP_FX_MAX, NAP_FX_MAX) != NAP_FX_MAX ||
	    nap_fx_mul(NAP_FX_MAX, -NAP_FX_MAX) != -NAP_FX_MAX ||
	    nap_fx_mul(-1, 1) != 0 || nap_fx_mul(1, -1) != 0) {
		pr_err("selftest: fx_mul saturation/rounding\n");
		fail++;
	}

	if (fail)
		pr_err("selftest: %d failure(s)\n", fail);
	else
		pr_info("selftest: fixed-point kernels ok\n");
}
#else
static inline void nap_selftest(void) { }
#endif

/* ================================================================
 * Registration
 * ================================================================ */

static struct cpuidle_governor nap_governor = {
	.name	= "nap",
	.rating	= 18,
	.enable	= nap_enable_device,
	.select	= nap_select,
	.reflect = nap_reflect,
};

static int __init nap_init(void)
{
	int ret;

	if (IS_ENABLED(CONFIG_CPU_IDLE_GOV_NAP_SELFTEST))
		nap_selftest();

	ret = cpuidle_register_governor(&nap_governor);
	if (ret)
		return ret;

	/* cpu_dev_init() runs in driver_init(), before any initcall. */
	nap_kobj = kobject_create_and_add("nap", &cpu_subsys.dev_root->kobj);
	if (nap_kobj && sysfs_create_groups(nap_kobj, nap_groups)) {
		kobject_put(nap_kobj);
		nap_kobj = NULL;
	}
	if (!nap_kobj)
		pr_warn("nap: sysfs init failed, continuing without tunables\n");

	return 0;
}
postcore_initcall(nap_init);
