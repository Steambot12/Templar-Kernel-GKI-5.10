/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Legacy CFS sleeper credit — superseded by EEVDF's PLACE_LAG which
 * preserves the full fairness lag across sleep, not just 50%.
 * Kept false; no code reads it when EEVDF is active.
 */
SCHED_FEAT(GENTLE_FAIR_SLEEPERS, false)

/*
 * Legacy CFS initial placement debit — superseded by EEVDF's
 * PLACE_DEADLINE_INITIAL which gives new tasks half a virtual slice.
 * Kept false; no code reads it when EEVDF is active.
 */
SCHED_FEAT(START_DEBIT, false)

/*
 * EEVDF: Preserve lag across sleeps — entity wakes with the same
 * fairness debt/credit it had when it went to sleep.
 */
SCHED_FEAT(PLACE_LAG, true)

/*
 * EEVDF: Give newly forked/execed tasks only half a slice worth of
 * deadline, so they do not immediately preempt long-running tasks.
 */
SCHED_FEAT(PLACE_DEADLINE_INITIAL, true)

/*
 * EEVDF: Let the current task keep running at tick time if it hasn't
 * exhausted its slice yet — __pick_eevdf() returns curr immediately
 * instead of doing an O(log n) tree walk, saving ~50 cycles per tick
 * on lightly loaded cores.
 *
 * This does NOT block wakeup preemption: check_preempt_wakeup() fires
 * before pick_next_entity() and grants preemption via deadline
 * comparison + vruntime sleeper assist.  RUN_TO_PARITY only skips the
 * tick-driven re-evaluation, which matters for CPU-bound background
 * tasks — exactly the workload where fewer context switches save power
 * without affecting UI latency.
 */
SCHED_FEAT(RUN_TO_PARITY, true)

/*
 * Prefer to schedule the task we woke last (assuming it failed
 * wakeup-preemption), since its likely going to consume data we
 * touched, increases cache locality.
 */
SCHED_FEAT(NEXT_BUDDY, false)

/*
 * Prefer to schedule the task that ran last (when we did
 * wake-preempt) as that likely will touch the same data, increases
 * cache locality.
 */
SCHED_FEAT(LAST_BUDDY, true)

/*
 * Consider buddies to be cache hot, decreases the likelyness of a
 * cache buddy being migrated away, increases cache locality.
 */
SCHED_FEAT(CACHE_HOT_BUDDY, true)

/*
 * Allow wakeup-time preemption of the current task:
 */
SCHED_FEAT(WAKEUP_PREEMPTION, true)

SCHED_FEAT(HRTICK, false)
SCHED_FEAT(DOUBLE_TICK, false)

/*
 * Decrement CPU capacity based on time not spent running tasks
 */
SCHED_FEAT(NONTASK_CAPACITY, true)

/*
 * Queue remote wakeups on the target CPU and process them
 * using the scheduler IPI. Reduces rq->lock contention/bounces.
 */
SCHED_FEAT(TTWU_QUEUE, true)

/*
 * When doing wakeups, attempt to limit superfluous scans of the LLC domain.
 */
SCHED_FEAT(SIS_AVG_CPU, false)
SCHED_FEAT(SIS_PROP, true)

/*
 * Issue a WARN when we do multiple update_rq_clock() calls
 * in a single rq->lock section. Default disabled because the
 * annotations are not complete.
 */
SCHED_FEAT(WARN_DOUBLE_CLOCK, false)

#ifdef HAVE_RT_PUSH_IPI
/*
 * In order to avoid a thundering herd attack of CPUs that are
 * lowering their priorities at the same time, and there being
 * a single CPU that has an RT task that can migrate and is waiting
 * to run, where the other CPUs will try to take that CPUs
 * rq lock and possibly create a large contention, sending an
 * IPI to that CPU and let that CPU push the RT task to where
 * it should go may be a better scenario.
 *
 * This is best for PREEMPT_RT, but for non-RT it can cause issues
 * when preemption is disabled for long periods of time. Have
 * it only default enabled for PREEMPT_RT.
 */
# ifdef CONFIG_PREEMPT_RT
SCHED_FEAT(RT_PUSH_IPI, true)
# else
SCHED_FEAT(RT_PUSH_IPI, false)
# endif
#endif

SCHED_FEAT(RT_RUNTIME_SHARE, false)
SCHED_FEAT(LB_MIN, false)
SCHED_FEAT(ATTACH_AGE_LOAD, true)

SCHED_FEAT(WA_IDLE, true)
SCHED_FEAT(WA_WEIGHT, true)
SCHED_FEAT(WA_BIAS, true)

/*
 * UtilEstimation. Use estimated CPU utilization.
 */
SCHED_FEAT(UTIL_EST, true)
SCHED_FEAT(UTIL_EST_FASTUP, true)

SCHED_FEAT(ALT_PERIOD, true)
SCHED_FEAT(BASE_SLICE, true)
