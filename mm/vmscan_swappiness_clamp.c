// SPDX-License-Identifier: GPL-2.0
/*
 * Swappiness clamp for per-cgroup reclaim.
 *
 * get_scan_count() uses the MEMCG's swappiness, not the global
 * /proc/sys/vm/swappiness — on devices whose userspace arms high
 * per-cgroup values, the global knob can never reduce swap-out
 * pressure. This listener clamps the effective value seen by the
 * reclaim path so per-cgroup requests above the cap read as the cap.
 * Values below the cap pass through untouched; the OOM-priority
 * SCAN_EQUAL path (swappiness != 0) is unaffected, so late-reclaim
 * behaviour is identical.
 */

#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/tracepoint.h>
#include <trace/hooks/vmscan.h>

/* 0 = derive from RAM size at init; 0 written at runtime = clamp off. */
static unsigned int swappiness_cap;

/*
 * The useful cap is a function of how much RAM the device has, and one image
 * ships to all of them.
 *
 * On a large-memory device, holding anon resident is the win: apps stay warm,
 * reopen without a decompress fault, and the compressed swap device stops
 * churning. There is room to spend on that.
 *
 * On a small-memory device the same cap defers pressure rather than removing
 * it. Anon that is not compressed is anon that is simply present, so the
 * reserve drains, and the next allocation spike gets resolved by the userspace
 * low-memory killer taking a whole process -- much worse than the decompress
 * fault the cap was avoiding. Below the cap nothing changes either way: values
 * under it still pass through untouched.
 *
 * Thresholds are read against usable RAM, which always sits some way below the
 * nominal size once carveouts are taken, so each band catches the marketing
 * figure above it.
 */
static unsigned int __init swappiness_cap_for_ram(void)
{
	unsigned long gb = totalram_pages() >> (30 - PAGE_SHIFT);

	if (gb <= 6)
		return 100;	/* anon and file at equal reclaim cost */
	if (gb <= 8)
		return 80;
	return 60;
}

static void vh_tune_swappiness(void *unused, int *swappiness)
{
	unsigned int cap = READ_ONCE(swappiness_cap);

	if (cap && *swappiness > (int)cap)
		*swappiness = (int)cap;
}

static int __init vmscan_swappiness_clamp_init(void)
{
	int ret;

	if (!swappiness_cap)
		swappiness_cap = swappiness_cap_for_ram();

	ret = register_trace_android_vh_tune_swappiness(
			vh_tune_swappiness, NULL);
	if (ret) {
		pr_err("swappiness_clamp: hook registration failed: %d\n", ret);
		return ret;
	}
	pr_info("swappiness_clamp: effective reclaim swappiness capped at %u\n",
		swappiness_cap);
	return 0;
}
module_init(vmscan_swappiness_clamp_init);

static void __exit vmscan_swappiness_clamp_exit(void)
{
	unregister_trace_android_vh_tune_swappiness(
			vh_tune_swappiness, NULL);
	tracepoint_synchronize_unregister();
}
module_exit(vmscan_swappiness_clamp_exit);

module_param(swappiness_cap, uint, 0644);
MODULE_PARM_DESC(swappiness_cap,
		 "max effective reclaim swappiness (1-200); 0 = auto by RAM size at boot, 0 at runtime disables the clamp");

MODULE_DESCRIPTION("Clamp per-cgroup reclaim swappiness");
MODULE_LICENSE("GPL");
