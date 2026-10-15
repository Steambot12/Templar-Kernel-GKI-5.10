/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TCP BBRplus Congestion Control
 *
 * Enhanced BBR with drain-to-target cycling, tuned for low-latency gaming
 * and sustained throughput on mobile (4G/5G/Wi-Fi).
 *
 * Originally introduced by:
 *   - dog250 <yyforkitty@gmail.com>
 *   - cx9208
 *
 * Upstream 5.10 port:
 *   - UJX6N <https://github.com/UJX6N/bbrplus-5.10>
 *
 * GKI 5.10 compatibility + tuning:
 *   - Steambot12 <wilfridus.it@gmail.com>
 */
#include <linux/module.h>
#include <net/tcp.h>
#include <linux/inet_diag.h>
#include <linux/inet.h>
#include <linux/random.h>
#include <linux/win_minmax.h>

#define BW_SCALE 24
#define BW_UNIT (1 << BW_SCALE)
#define BBR_SCALE 8
#define BBR_UNIT (1 << BBR_SCALE)

enum bbr_mode {
	BBR_STARTUP,
	BBR_DRAIN,
	BBR_PROBE_BW,
	BBR_PROBE_RTT,
};

struct bbr {
	u32	min_rtt_us;
	u32	min_rtt_stamp;
	u32	probe_rtt_done_stamp;
	struct minmax bw;
	u32	rtt_cnt;
	u32	next_rtt_delivered;
	u64	cycle_mstamp;
	
	u32	mode:3,
		prev_ca_state:3,
		packet_conservation:1,
		restore_cwnd:1,
		round_start:1,
		cycle_len:4,
		idle_restart:1,
		probe_rtt_round_done:1,
		unused:8,
		lt_is_sampling:1,
		lt_rtt_cnt:7,
		lt_use_bw:1;
	u32	lt_bw;
	u32	lt_last_delivered;
	u32	lt_last_stamp;
	u32	lt_last_lost;
	u32	pacing_gain:10,
		cwnd_gain:10,
		full_bw_reached:1,
		full_bw_cnt:2,
		cycle_idx:3,
		has_seen_rtt:1,
		unused_b:5;
	u32	prior_cwnd;
	u32	full_bw;
	u64	ack_epoch_mstamp;
	u16	extra_acked[2];
	u32	ack_epoch_acked:20,
		extra_acked_win_rtts:5,
		extra_acked_win_idx:1,
		unused1:6;
};

#define CYCLE_LEN	8

static const int bbr_bw_rtts		= CYCLE_LEN + 2;
static const u32 bbr_min_rtt_win_sec	= 10;
static const u32 bbr_probe_rtt_mode_ms	= 40;
static const int bbr_min_tso_rate	= 1200000;
static const int bbr_high_gain		= BBR_UNIT * 2885 / 1000 + 1;
static const int bbr_drain_gain		= BBR_UNIT * 1000 / 2885;
static const int bbr_cwnd_gain		= BBR_UNIT * 2;

enum bbr_pacing_gain_phase {
	BBR_BW_PROBE_UP     = 0,
	BBR_BW_PROBE_DOWN   = 1,
	BBR_BW_PROBE_CRUISE = 2,
};

static const int bbr_pacing_gain[] = {
	BBR_UNIT * 5 / 4,
	BBR_UNIT * 3 / 4,
	BBR_UNIT,
};

static const u32 bbr_cycle_rand		= 7;
static const u32 bbr_cwnd_min_target	= 4;
static const u32 bbr_full_bw_thresh	= BBR_UNIT * 5 / 4;
static const u32 bbr_full_bw_cnt	= 3;
static const u32 bbr_lt_intvl_min_rtts	= 4;
static const u32 bbr_lt_loss_thresh	= 50;
static const u32 bbr_lt_bw_ratio	= BBR_UNIT / 8;
static const u32 bbr_lt_bw_diff		= 4000 / 8;
static const u32 bbr_lt_bw_max_rtts	= 64;
static const int bbr_extra_acked_gain	= BBR_UNIT;
static const u32 bbr_extra_acked_win_rtts = 10;
static const u32 bbr_ack_epoch_acked_reset_thresh = 1U << 20;
static const u32 bbr_extra_acked_max_us	= 100 * 1000;
static const int bbr_probe_rtt_cwnd_gain = BBR_UNIT * 1 / 2;

static bool bbrplus_snd_wnd_test(const struct tcp_sock *tp,
				  const struct sk_buff *skb,
				  unsigned int cur_mss)
{
	u32 end_seq = TCP_SKB_CB(skb)->end_seq;

	if (skb->len > cur_mss)
		end_seq = TCP_SKB_CB(skb)->seq + cur_mss;
	return !after(end_seq, tcp_wnd_end(tp));
}

static bool bbr_full_bw_reached(const struct sock *sk)
{
	const struct bbr *bbr = inet_csk_ca(sk);

	return bbr->full_bw_reached;
}

static void bbr_set_cycle_idx(struct sock *sk, int cycle_idx)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->cycle_idx = cycle_idx;
	bbr->pacing_gain = bbr->lt_use_bw ?
			   BBR_UNIT : bbr_pacing_gain[bbr->cycle_idx];
}

static u32 bbr_max_bw(const struct sock *sk);
static u32 bbr_inflight(struct sock *sk, u32 bw, int gain);

static void bbr_drain_to_target_cycling(struct sock *sk,
					const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 elapsed_us =
		tcp_stamp_us_delta(tp->delivered_mstamp, bbr->cycle_mstamp);
	u32 inflight, bw;

	if (bbr->mode != BBR_PROBE_BW)
		return;

	if (elapsed_us > bbr->cycle_len * bbr->min_rtt_us) {
		bbr->cycle_mstamp = tp->delivered_mstamp;
		bbr->cycle_len = CYCLE_LEN - prandom_u32_max(bbr_cycle_rand);
		bbr_set_cycle_idx(sk, BBR_BW_PROBE_UP);
		return;
	}

	if (bbr->pacing_gain == BBR_UNIT)
		return;

	inflight = rs->prior_in_flight;
	bw = bbr_max_bw(sk);

	if (bbr->pacing_gain < BBR_UNIT) {
		if (inflight <= bbr_inflight(sk, bw, BBR_UNIT * 95 / 100))
			bbr_set_cycle_idx(sk, BBR_BW_PROBE_CRUISE);
		return;
	}

	if (elapsed_us > bbr->min_rtt_us &&
	    (inflight >= bbr_inflight(sk, bw, bbr->pacing_gain) ||
	     rs->losses ||
	     rs->is_app_limited ||
	     !tcp_send_head(sk) ||
	     !bbrplus_snd_wnd_test(tp, tcp_send_head(sk), tp->mss_cache)))
		bbr_set_cycle_idx(sk, BBR_BW_PROBE_DOWN);
}

static u16 bbr_extra_acked(const struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	return max(bbr->extra_acked[0], bbr->extra_acked[1]);
}

static u32 bbr_max_bw(const struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	return minmax_get(&bbr->bw);
}

static u32 bbr_bw(const struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	return bbr->lt_use_bw ? bbr->lt_bw : bbr_max_bw(sk);
}

/* Pace at ~1% below estimated bw for balanced throughput and latency.
 * Matches BBRv3 tuning while BBRplus's drain-to-target cycling and 10s
 * min_rtt window provide additional stability.
 */
static const int bbr_pacing_margin_percent = 1;

static u64 bbr_rate_bytes_per_sec(struct sock *sk, u64 rate, int gain)
{
	rate *= tcp_mss_to_mtu(sk, tcp_sk(sk)->mss_cache);
	rate *= gain;
	rate >>= BBR_SCALE;
	rate *= USEC_PER_SEC;
	rate >>= BW_SCALE;
	if (bbr_pacing_margin_percent)
		rate -= (rate * bbr_pacing_margin_percent) / 100;
	return rate;
}

static u32 bbr_bw_to_pacing_rate(struct sock *sk, u32 bw, int gain)
{
	u64 rate = bw;

	rate = bbr_rate_bytes_per_sec(sk, rate, gain);
	rate = min_t(u64, rate, sk->sk_max_pacing_rate);
	return rate;
}

static void bbr_init_pacing_rate_from_rtt(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u64 bw;
	u32 rtt_us;

	if (tp->srtt_us) {
		rtt_us = max(tp->srtt_us >> 3, 1U);
		bbr->has_seen_rtt = 1;
	} else {
		rtt_us = USEC_PER_MSEC;
	}
	bw = (u64)tp->snd_cwnd * BW_UNIT;
	do_div(bw, rtt_us);
	sk->sk_pacing_rate = bbr_bw_to_pacing_rate(sk, bw, bbr_high_gain);
}

static void bbr_set_pacing_rate(struct sock *sk, u32 bw, int gain)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 rate = bbr_bw_to_pacing_rate(sk, bw, gain);

	if (unlikely(!bbr->has_seen_rtt && tp->srtt_us))
		bbr_init_pacing_rate_from_rtt(sk);
	if (bbr_full_bw_reached(sk) || rate > sk->sk_pacing_rate)
		sk->sk_pacing_rate = rate;
}

static u32 bbr_tso_segs_goal(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	u32 min_segs, bytes, segs;

	min_segs = sk->sk_pacing_rate < (bbr_min_tso_rate >> 3) ? 1 : 2;

	bytes = min_t(unsigned long,
		      sk->sk_pacing_rate >> READ_ONCE(sk->sk_pacing_shift),
		      sk->sk_gso_max_size - 1 - MAX_TCP_HEADER);
	segs = max_t(u32, bytes / tp->mss_cache, min_segs);
	return min_t(u32, segs, 0x7FU);
}

static u32 bbr_min_tso_segs(struct sock *sk)
{
	return sk->sk_pacing_rate < (bbr_min_tso_rate >> 3) ? 1 : 2;
}

static void bbr_save_cwnd(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr->prev_ca_state < TCP_CA_Recovery && bbr->mode != BBR_PROBE_RTT)
		bbr->prior_cwnd = tp->snd_cwnd;
	else
		bbr->prior_cwnd = max(bbr->prior_cwnd, tp->snd_cwnd);
}

static void bbr_cwnd_event(struct sock *sk, enum tcp_ca_event event)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	if (event == CA_EVENT_TX_START && tp->app_limited) {
		bbr->idle_restart = 1;
		bbr->ack_epoch_mstamp = tp->tcp_mstamp;
		bbr->ack_epoch_acked = 0;
		if (bbr->mode == BBR_PROBE_BW)
			bbr_set_pacing_rate(sk, bbr_bw(sk), BBR_UNIT);
	}
}

static u32 bbr_bdp(struct sock *sk, u32 bw, int gain)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 bdp;
	u64 w;

	if (unlikely(bbr->min_rtt_us == ~0U))
		return TCP_INIT_CWND;

	w = (u64)bw * bbr->min_rtt_us;
	bdp = (((w * gain) >> BBR_SCALE) + BW_UNIT - 1) / BW_UNIT;
	return bdp;
}

static u32 bbr_quantization_budget(struct sock *sk, u32 cwnd)
{
	cwnd += 3 * bbr_tso_segs_goal(sk);
	return max_t(u32, cwnd, bbr_cwnd_min_target);
}

static u32 bbr_inflight(struct sock *sk, u32 bw, int gain)
{
	u32 inflight;

	inflight = bbr_bdp(sk, bw, gain);
	inflight = bbr_quantization_budget(sk, inflight);
	return inflight;
}

static u32 bbr_ack_aggregation_cwnd(struct sock *sk)
{
	u32 max_aggr_cwnd, aggr_cwnd = 0;

	if (bbr_extra_acked_gain && bbr_full_bw_reached(sk)) {
		max_aggr_cwnd = ((u64)bbr_bw(sk) * bbr_extra_acked_max_us)
				/ BW_UNIT;
		aggr_cwnd = (bbr_extra_acked_gain * bbr_extra_acked(sk))
			    >> BBR_SCALE;
		aggr_cwnd = min(aggr_cwnd, max_aggr_cwnd);
	}
	return aggr_cwnd;
}

static bool bbr_set_cwnd_to_recover_or_restore(
	struct sock *sk, const struct rate_sample *rs,
	u32 acked, u32 *new_cwnd)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u8 prev_state = bbr->prev_ca_state;
	u8 state = inet_csk(sk)->icsk_ca_state;
	u32 cwnd = tp->snd_cwnd;

	if (rs->losses > 0)
		cwnd = max_t(s32, cwnd - rs->losses, 1);

	if (state == TCP_CA_Recovery && prev_state != TCP_CA_Recovery) {
		bbr->packet_conservation = 1;
		bbr->next_rtt_delivered = tp->delivered;
		cwnd = tcp_packets_in_flight(tp) + acked;
	} else if (prev_state >= TCP_CA_Recovery && state < TCP_CA_Recovery) {
		bbr->restore_cwnd = 1;
		bbr->packet_conservation = 0;
	}
	bbr->prev_ca_state = state;

	if (bbr->restore_cwnd) {
		cwnd = max(cwnd, bbr->prior_cwnd);
		bbr->restore_cwnd = 0;
	}

	if (bbr->packet_conservation) {
		*new_cwnd = max(cwnd, tcp_packets_in_flight(tp) + acked);
		*new_cwnd = max_t(u32, *new_cwnd, bbr_cwnd_min_target);
		return true;
	}
	*new_cwnd = max_t(u32, cwnd, bbr_cwnd_min_target);
	return false;
}

static u32 bbr_probe_rtt_cwnd(struct sock *sk)
{
	u32 cwnd;

	if (bbr_probe_rtt_cwnd_gain == 0)
		return bbr_cwnd_min_target;

	cwnd = bbr_bdp(sk, bbr_bw(sk), bbr_probe_rtt_cwnd_gain);
	cwnd = bbr_quantization_budget(sk, cwnd);
	return max(cwnd, bbr_cwnd_min_target);
}

static void bbr_set_cwnd(struct sock *sk, const struct rate_sample *rs,
			 u32 acked, u32 bw, int gain)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 cwnd, target_cwnd = 0;

	if (!acked)
		return;

	cwnd = tp->snd_cwnd;
	if (bbr_set_cwnd_to_recover_or_restore(sk, rs, acked, &cwnd))
		goto done;

	target_cwnd = bbr_bdp(sk, bw, gain);
	target_cwnd += bbr_ack_aggregation_cwnd(sk);
	target_cwnd = bbr_quantization_budget(sk, target_cwnd);

	if (bbr_full_bw_reached(sk))
		cwnd = min(cwnd + acked, target_cwnd);
	else if (cwnd < target_cwnd || tp->delivered < TCP_INIT_CWND)
		cwnd = cwnd + acked;
	cwnd = max(cwnd, bbr_cwnd_min_target);

done:
	tp->snd_cwnd = min(cwnd, tp->snd_cwnd_clamp);
	if (bbr->mode == BBR_PROBE_RTT)
		tp->snd_cwnd = min(tp->snd_cwnd, bbr_probe_rtt_cwnd(sk));
}

static void bbr_update_cycle_phase(struct sock *sk,
				   const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr->mode == BBR_PROBE_BW)
		bbr_drain_to_target_cycling(sk, rs);
}

static void bbr_reset_startup_mode(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->mode = BBR_STARTUP;
	bbr->pacing_gain = bbr_high_gain;
	bbr->cwnd_gain   = bbr_high_gain;
}

static void bbr_reset_probe_bw_mode(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->mode = BBR_PROBE_BW;
	bbr->cwnd_gain = bbr_cwnd_gain;
	bbr->cycle_len = CYCLE_LEN - prandom_u32_max(bbr_cycle_rand);
	bbr->cycle_mstamp = tcp_sk(sk)->delivered_mstamp;
	bbr_set_cycle_idx(sk, BBR_BW_PROBE_CRUISE);
}

static void bbr_reset_mode(struct sock *sk)
{
	if (!bbr_full_bw_reached(sk))
		bbr_reset_startup_mode(sk);
	else
		bbr_reset_probe_bw_mode(sk);
}

static void bbr_reset_lt_bw_sampling_interval(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->lt_last_stamp = div_u64(tp->delivered_mstamp, USEC_PER_MSEC);
	bbr->lt_last_delivered = tp->delivered;
	bbr->lt_last_lost = tp->lost;
	bbr->lt_rtt_cnt = 0;
}

static void bbr_reset_lt_bw_sampling(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->lt_bw = 0;
	bbr->lt_use_bw = 0;
	bbr->lt_is_sampling = false;
	bbr_reset_lt_bw_sampling_interval(sk);
}

static void bbr_lt_bw_interval_done(struct sock *sk, u32 bw)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 diff;

	if (bbr->lt_bw) {
		diff = abs(bw - bbr->lt_bw);
		if ((diff * BBR_UNIT <= bbr_lt_bw_ratio * bbr->lt_bw) ||
		    (bbr_rate_bytes_per_sec(sk, diff, BBR_UNIT) <=
		     bbr_lt_bw_diff)) {
			bbr->lt_bw = (bw + bbr->lt_bw) >> 1;
			bbr->lt_use_bw = 1;
			bbr->pacing_gain = BBR_UNIT;
			bbr->lt_rtt_cnt = 0;
			return;
		}
	}
	bbr->lt_bw = bw;
	bbr_reset_lt_bw_sampling_interval(sk);
}

static void bbr_lt_bw_sampling(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u32 lost, delivered;
	u64 bw;
	u32 t;

	if (bbr->lt_use_bw) {
		if (bbr->mode == BBR_PROBE_BW && bbr->round_start &&
		    ++bbr->lt_rtt_cnt >= bbr_lt_bw_max_rtts) {
			bbr_reset_lt_bw_sampling(sk);
			bbr_reset_probe_bw_mode(sk);
		}
		return;
	}

	if (!bbr->lt_is_sampling) {
		if (!rs->losses)
			return;
		bbr_reset_lt_bw_sampling_interval(sk);
		bbr->lt_is_sampling = true;
	}

	if (rs->is_app_limited) {
		bbr_reset_lt_bw_sampling(sk);
		return;
	}

	if (bbr->round_start)
		bbr->lt_rtt_cnt++;
	if (bbr->lt_rtt_cnt < bbr_lt_intvl_min_rtts)
		return;
	if (bbr->lt_rtt_cnt > 4 * bbr_lt_intvl_min_rtts) {
		bbr_reset_lt_bw_sampling(sk);
		return;
	}

	if (!rs->losses)
		return;

	lost = tp->lost - bbr->lt_last_lost;
	delivered = tp->delivered - bbr->lt_last_delivered;
	if (!delivered || (lost << BBR_SCALE) < bbr_lt_loss_thresh * delivered)
		return;

	t = tcp_stamp_us_delta(tp->delivered_mstamp,
			       (u64)bbr->lt_last_stamp * USEC_PER_MSEC);
	if (t < USEC_PER_MSEC)
		return;
	bw = (u64)delivered * BW_UNIT;
	do_div(bw, t);
	bbr_lt_bw_interval_done(sk, bw);
}

static void bbr_update_bw(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	u64 bw;

	bbr->round_start = 0;
	if (rs->delivered < 0 || rs->interval_us <= 0)
		return;

	if (!before(rs->prior_delivered, bbr->next_rtt_delivered)) {
		bbr->next_rtt_delivered = tp->delivered;
		bbr->rtt_cnt++;
		bbr->round_start = 1;
		bbr->packet_conservation = 0;
	}

	bbr_lt_bw_sampling(sk, rs);

	bw = div64_long((u64)rs->delivered * BW_UNIT, rs->interval_us);

	if (!rs->is_app_limited || bw >= bbr_max_bw(sk))
		minmax_running_max(&bbr->bw, bbr_bw_rtts, bbr->rtt_cnt, bw);
}

static void bbr_check_full_bw_reached(struct sock *sk,
				      const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 bw_thresh;

	if (bbr_full_bw_reached(sk) || !bbr->round_start || rs->is_app_limited)
		return;

	bw_thresh = (u64)bbr->full_bw * bbr_full_bw_thresh >> BBR_SCALE;
	if (bbr_max_bw(sk) >= bw_thresh) {
		bbr->full_bw = bbr_max_bw(sk);
		bbr->full_bw_cnt = 0;
		return;
	}
	++bbr->full_bw_cnt;
	bbr->full_bw_reached = bbr->full_bw_cnt >= bbr_full_bw_cnt;
}

static void bbr_check_drain(struct sock *sk, const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);

	if (bbr->mode == BBR_STARTUP && bbr_full_bw_reached(sk)) {
		bbr->mode = BBR_DRAIN;
		bbr->pacing_gain = bbr_drain_gain;
		bbr->cwnd_gain = bbr_high_gain;
	}
	if (bbr->mode == BBR_DRAIN &&
	    tcp_packets_in_flight(tcp_sk(sk)) <=
	    bbr_inflight(sk, bbr_max_bw(sk), BBR_UNIT))
		bbr_reset_probe_bw_mode(sk);
}

static void bbr_update_ack_aggregation(struct sock *sk,
				       const struct rate_sample *rs)
{
	u32 epoch_us, expected_acked, extra_acked;
	struct bbr *bbr = inet_csk_ca(sk);
	struct tcp_sock *tp = tcp_sk(sk);

	if (!bbr_extra_acked_gain || rs->acked_sacked <= 0 ||
	    rs->delivered < 0 || rs->interval_us <= 0)
		return;

	if (bbr->round_start) {
		bbr->extra_acked_win_rtts = min(0x1FU,
					bbr->extra_acked_win_rtts + 1);
		if (bbr->extra_acked_win_rtts >= bbr_extra_acked_win_rtts) {
			bbr->extra_acked_win_rtts = 0;
			bbr->extra_acked_win_idx =
				bbr->extra_acked_win_idx ? 0 : 1;
			bbr->extra_acked[bbr->extra_acked_win_idx] = 0;
		}
	}

	epoch_us = tcp_stamp_us_delta(tp->delivered_mstamp,
				      bbr->ack_epoch_mstamp);
	expected_acked = ((u64)bbr_bw(sk) * epoch_us) / BW_UNIT;

	if (bbr->ack_epoch_acked <= expected_acked ||
	    (bbr->ack_epoch_acked + rs->acked_sacked >=
	     bbr_ack_epoch_acked_reset_thresh)) {
		bbr->ack_epoch_acked = 0;
		bbr->ack_epoch_mstamp = tp->delivered_mstamp;
		expected_acked = 0;
	}

	bbr->ack_epoch_acked = min_t(u32,
				     bbr->ack_epoch_acked + rs->acked_sacked,
				     bbr_ack_epoch_acked_reset_thresh - 1);
	extra_acked = bbr->ack_epoch_acked - expected_acked;
	extra_acked = min(extra_acked, tp->snd_cwnd);
	if (extra_acked > bbr->extra_acked[bbr->extra_acked_win_idx])
		bbr->extra_acked[bbr->extra_acked_win_idx] = extra_acked;
}

static void bbr_update_min_rtt(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);
	bool filter_expired;

	filter_expired = after(tcp_jiffies32,
			       bbr->min_rtt_stamp + bbr_min_rtt_win_sec * HZ);
	if (rs->rtt_us >= 0 &&
	    (rs->rtt_us <= bbr->min_rtt_us || filter_expired)) {
		bbr->min_rtt_us = rs->rtt_us;
		bbr->min_rtt_stamp = tcp_jiffies32;
	}

	if (bbr_probe_rtt_mode_ms > 0 && filter_expired &&
	    !bbr->idle_restart && bbr->mode != BBR_PROBE_RTT) {
		bbr->mode = BBR_PROBE_RTT;
		bbr->pacing_gain = BBR_UNIT;
		bbr->cwnd_gain = BBR_UNIT;
		bbr_save_cwnd(sk);
		bbr->probe_rtt_done_stamp = 0;
	}

	if (bbr->mode == BBR_PROBE_RTT) {
		tp->app_limited =
			(tp->delivered + tcp_packets_in_flight(tp)) ? : 1;
		if (!bbr->probe_rtt_done_stamp &&
		    tcp_packets_in_flight(tp) <= bbr_probe_rtt_cwnd(sk)) {
			bbr->probe_rtt_done_stamp = tcp_jiffies32 +
				msecs_to_jiffies(bbr_probe_rtt_mode_ms);
			bbr->probe_rtt_round_done = 0;
			bbr->next_rtt_delivered = tp->delivered;
		} else if (bbr->probe_rtt_done_stamp) {
			if (bbr->round_start)
				bbr->probe_rtt_round_done = 1;
			if (bbr->probe_rtt_round_done &&
			    after(tcp_jiffies32, bbr->probe_rtt_done_stamp)) {
				bbr->min_rtt_stamp = tcp_jiffies32;
				bbr->restore_cwnd = 1;
				bbr_reset_mode(sk);
			}
		}
	}
	if (rs->delivered > 0)
		bbr->idle_restart = 0;
}

static void bbr_update_model(struct sock *sk, const struct rate_sample *rs)
{
	bbr_update_bw(sk, rs);
	bbr_update_ack_aggregation(sk, rs);
	bbr_update_cycle_phase(sk, rs);
	bbr_check_full_bw_reached(sk, rs);
	bbr_check_drain(sk, rs);
	bbr_update_min_rtt(sk, rs);
}

static void bbr_main(struct sock *sk, const struct rate_sample *rs)
{
	struct bbr *bbr = inet_csk_ca(sk);
	u32 bw;

	bbr_update_model(sk, rs);
	bw = bbr_bw(sk);
	bbr_set_pacing_rate(sk, bw, bbr->pacing_gain);
	bbr_set_cwnd(sk, rs, rs->acked_sacked, bw, bbr->cwnd_gain);
}

static void bbr_init(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	struct bbr *bbr = inet_csk_ca(sk);

	BUILD_BUG_ON(sizeof(struct bbr) > ICSK_CA_PRIV_SIZE);

	bbr->prior_cwnd = 0;
	bbr->rtt_cnt = 0;
	bbr->next_rtt_delivered = tp->delivered;
	bbr->prev_ca_state = TCP_CA_Open;
	bbr->packet_conservation = 0;
	bbr->probe_rtt_done_stamp = 0;
	bbr->probe_rtt_round_done = 0;
	bbr->min_rtt_us = tcp_min_rtt(tp);
	bbr->min_rtt_stamp = tcp_jiffies32;
	minmax_reset(&bbr->bw, bbr->rtt_cnt, 0);
	bbr->has_seen_rtt = 0;
	bbr_init_pacing_rate_from_rtt(sk);
	bbr->restore_cwnd = 0;
	bbr->round_start = 0;
	bbr->idle_restart = 0;
	bbr->full_bw_reached = 0;
	bbr->full_bw = 0;
	bbr->full_bw_cnt = 0;
	bbr->cycle_mstamp = 0;
	bbr->cycle_idx = 0;
	bbr->cycle_len = CYCLE_LEN;
	bbr_reset_lt_bw_sampling(sk);
	bbr_reset_startup_mode(sk);
	bbr->ack_epoch_mstamp = tp->tcp_mstamp;
	bbr->ack_epoch_acked = 0;
	bbr->extra_acked_win_rtts = 0;
	bbr->extra_acked_win_idx = 0;
	bbr->extra_acked[0] = 0;
	bbr->extra_acked[1] = 0;
	cmpxchg(&sk->sk_pacing_status, SK_PACING_NONE, SK_PACING_NEEDED);
}

static u32 bbr_sndbuf_expand(struct sock *sk)
{
	return 4;
}

static u32 bbr_undo_cwnd(struct sock *sk)
{
	struct bbr *bbr = inet_csk_ca(sk);

	bbr->full_bw = 0;
	bbr->full_bw_cnt = 0;
	bbr_reset_lt_bw_sampling(sk);
	return tcp_sk(sk)->snd_cwnd;
}

static u32 bbr_ssthresh(struct sock *sk)
{
	bbr_save_cwnd(sk);
	return tcp_sk(sk)->snd_ssthresh;
}

static size_t bbr_get_info(struct sock *sk, u32 ext, int *attr,
			   union tcp_cc_info *info)
{
	if (ext & (1 << (INET_DIAG_BBRINFO - 1)) ||
	    ext & (1 << (INET_DIAG_VEGASINFO - 1))) {
		const struct tcp_sock *tp = tcp_sk(sk);
		struct bbr *bbr = inet_csk_ca(sk);
		u64 bw = bbr_bw(sk);

		bw = bw * tp->mss_cache * USEC_PER_SEC >> BW_SCALE;
		memset(&info->bbr, 0, sizeof(info->bbr));
		info->bbr.bbr_bw_lo       = (u32)bw;
		info->bbr.bbr_bw_hi       = (u32)(bw >> 32);
		info->bbr.bbr_min_rtt     = bbr->min_rtt_us;
		info->bbr.bbr_pacing_gain = bbr->pacing_gain;
		info->bbr.bbr_cwnd_gain   = bbr->cwnd_gain;
		*attr = INET_DIAG_BBRINFO;
		return sizeof(info->bbr);
	}
	return 0;
}

static void bbr_set_state(struct sock *sk, u8 new_state)
{
	struct bbr *bbr = inet_csk_ca(sk);

	if (new_state == TCP_CA_Loss) {
		struct rate_sample rs = { .losses = 1 };

		bbr->prev_ca_state = TCP_CA_Loss;
		bbr->full_bw = 0;
		bbr->round_start = 1;
		bbr_lt_bw_sampling(sk, &rs);
	}
}

static struct tcp_congestion_ops tcp_bbrplus_cong_ops __read_mostly = {
	.flags		= TCP_CONG_NON_RESTRICTED,
	.name		= "bbrplus",
	.owner		= THIS_MODULE,
	.init		= bbr_init,
	.cong_control	= bbr_main,
	.sndbuf_expand	= bbr_sndbuf_expand,
	.undo_cwnd	= bbr_undo_cwnd,
	.cwnd_event	= bbr_cwnd_event,
	.ssthresh	= bbr_ssthresh,
	.min_tso_segs	= bbr_min_tso_segs,
	.get_info	= bbr_get_info,
	.set_state	= bbr_set_state,
};

static int __init bbr_register(void)
{
	BUILD_BUG_ON(sizeof(struct bbr) > ICSK_CA_PRIV_SIZE);
	return tcp_register_congestion_control(&tcp_bbrplus_cong_ops);
}

static void __exit bbr_unregister(void)
{
	tcp_unregister_congestion_control(&tcp_bbrplus_cong_ops);
}

module_init(bbr_register);
module_exit(bbr_unregister);

MODULE_AUTHOR("Van Jacobson <vanj@google.com>");
MODULE_AUTHOR("Neal Cardwell <ncardwell@google.com>");
MODULE_AUTHOR("Yuchung Cheng <ycheng@google.com>");
MODULE_AUTHOR("dog250 (BBRplus enhancements)");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("TCP BBRplus — low-latency tuning, GKI 5.10");
