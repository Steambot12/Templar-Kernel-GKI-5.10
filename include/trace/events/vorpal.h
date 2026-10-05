/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM vorpal

#if !defined(_TRACE_VORPAL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_VORPAL_H

#include <linux/tracepoint.h>

/*
 * One event per committed frequency change. demand/fceil are percentages of
 * capacity / the effective ceiling as the governor saw them at the decision;
 * gaming is the gaming_mode level (0 daily, 1/2 gaming tiers).
 */
TRACE_EVENT(vorpal,
	TP_PROTO(unsigned int cpu, int gaming, unsigned int freq,
		 unsigned int demand_pct, unsigned int fceil_pct),

	TP_ARGS(cpu, gaming, freq, demand_pct, fceil_pct),

	TP_STRUCT__entry(
		__field(unsigned int, cpu)
		__field(int, gaming)
		__field(unsigned int, freq)
		__field(unsigned int, demand_pct)
		__field(unsigned int, fceil_pct)
	),

	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->gaming = gaming;
		__entry->freq = freq;
		__entry->demand_pct = demand_pct;
		__entry->fceil_pct = fceil_pct;
	),

	TP_printk("cpu=%u gaming=%d freq=%u demand=%u fceil=%u",
		  __entry->cpu, __entry->gaming, __entry->freq,
		  __entry->demand_pct, __entry->fceil_pct)
);

#endif /* _TRACE_VORPAL_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
