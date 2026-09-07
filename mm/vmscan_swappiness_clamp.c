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
#include <linux/module.h>
#include <linux/tracepoint.h>
#include <trace/hooks/vmscan.h>

static unsigned int swappiness_cap = 60;

static void vh_tune_swappiness(void *unused, int *swappiness)
{
	if (*swappiness > (int)swappiness_cap)
		*swappiness = (int)swappiness_cap;
}

static int __init vmscan_swappiness_clamp_init(void)
{
	int ret;

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
MODULE_PARM_DESC(swappiness_cap, "max effective reclaim swappiness (0-100)");

MODULE_DESCRIPTION("Clamp per-cgroup reclaim swappiness");
MODULE_LICENSE("GPL");
