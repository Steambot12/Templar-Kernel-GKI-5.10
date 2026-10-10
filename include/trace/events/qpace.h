/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM qpace

#if !defined(_TRACE_QPACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_QPACE_H
#include <linux/tracepoint.h>

TRACE_EVENT(start_qpace_urgent_decompress,

	TP_PROTO(u64 src, u64 dst, unsigned int size),
	TP_ARGS(src, dst, size),

	 TP_STRUCT__entry(
		 __field(u64, src)
		 __field(u64, dst)
		 __field(unsigned int, size)
	 ),

	 TP_fast_assign(
		 __entry->src = src;
		 __entry->dst = dst;
		 __entry->size = size;
	 ),

	 TP_printk("src=0x%llx dst=0x%llx size=%u",
		   __entry->src, __entry->dst, __entry->size)
);

TRACE_EVENT(end_qpace_urgent_decompress,

	 TP_PROTO(u64 src, u64 dst, unsigned int size, int ret),
	 TP_ARGS(src, dst, size, ret),

	 TP_STRUCT__entry(
		 __field(u64, src)
		 __field(u64, dst)
		 __field(unsigned int, size)
		 __field(int, ret)
	 ),

	 TP_fast_assign(
		 __entry->src = src;
		 __entry->dst = dst;
		 __entry->size = size;
		 __entry->ret = ret;
	 ),

	 TP_printk("src=0x%llx dst=0x%llx size=%u ret=%d",
		   __entry->src, __entry->dst, __entry->size, __entry->ret)
);

TRACE_EVENT(start_qpace_urgent_compress,

	 TP_PROTO(u64 src, u64 dst),
	 TP_ARGS(src, dst),

	 TP_STRUCT__entry(
		 __field(u64, src)
		 __field(u64, dst)
	 ),

	 TP_fast_assign(
		 __entry->src = src;
		 __entry->dst = dst;
	 ),

	 TP_printk("src=0x%llx dst=0x%llx",
		   __entry->src, __entry->dst)
);

TRACE_EVENT(end_qpace_urgent_compress,

	 TP_PROTO(u64 src, u64 dst, int ret),
	 TP_ARGS(src, dst, ret),

	 TP_STRUCT__entry(
		 __field(u64, src)
		 __field(u64, dst)
		 __field(int, ret)
	 ),

	 TP_fast_assign(
		 __entry->src = src;
		 __entry->dst = dst;
		 __entry->ret = ret;
	 ),

	 TP_printk("src=0x%llx dst=0x%llx ret=%d",
		   __entry->src, __entry->dst, __entry->ret)
);

#endif /* _TRACE_QPACE_H */

#include <trace/define_trace.h>
