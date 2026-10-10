/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 ARM Ltd.
 */

#include <linux/linkage.h>
#include <linux/const.h>
#include <asm/assembler.h>
#include <asm/page.h>

/*
 * Clear page @dest
 *
 * Parameters:
 *	x0 - dest
 */
SYM_FUNC_START_PI(clear_page)
	mrs	x1, dczid_el0
	tbnz	x1, #4, 2f	/* Branch if DC ZVA is prohibited */
	and	w1, w1, #0xf
	mov	x2, #4
	lsl	x1, x2, x1

1:	dc	zva, x0
	add	x0, x0, x1
	tst	x0, #(PAGE_SIZE - 1)
	b.ne	1b
	ret

2:	/*
	 * BACKPORT (Linux 6.12):
	 * Unroll fallback zeroing loop to 128 bytes per iteration using 8 pairs
	 * of non-temporal stores (STNP). Halves loop branch overhead from 64 to 32
	 * iterations per 4KB page, maximizing memory bus saturation on ARMv8/v9 cores.
	 */
	stnp	xzr, xzr, [x0]
	stnp	xzr, xzr, [x0, #16]
	stnp	xzr, xzr, [x0, #32]
	stnp	xzr, xzr, [x0, #48]
	stnp	xzr, xzr, [x0, #64]
	stnp	xzr, xzr, [x0, #80]
	stnp	xzr, xzr, [x0, #96]
	stnp	xzr, xzr, [x0, #112]
	add	x0, x0, #128
	tst	x0, #(PAGE_SIZE - 1)
	b.ne	2b
	ret
SYM_FUNC_END_PI(clear_page)
EXPORT_SYMBOL(clear_page)
