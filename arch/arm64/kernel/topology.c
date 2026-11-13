/*
 * arch/arm64/kernel/topology.c
 *
 * Copyright (C) 2011,2013,2014 Linaro Limited.
 *
 * Based on the arm32 version written by Vincent Guittot in turn based on
 * arch/sh/kernel/topology.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/acpi.h>
#include <linux/arch_topology.h>
#include <linux/cacheinfo.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/sched/isolation.h>

#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/topology.h>

#ifdef CONFIG_ACPI
static bool __init acpi_cpu_is_threaded(int cpu)
{
	int is_threaded = acpi_pptt_cpu_is_thread(cpu);

	/*
	 * if the PPTT doesn't have thread information, assume a homogeneous
	 * machine and return the current CPU's thread state.
	 */
	if (is_threaded < 0)
		is_threaded = read_cpuid_mpidr() & MPIDR_MT_BITMASK;

	return !!is_threaded;
}

/*
 * Propagate the topology information of the processor_topology_node tree to the
 * cpu_topology array.
 */
int __init parse_acpi_topology(void)
{
	int cpu, topology_id;

	if (acpi_disabled)
		return 0;

	for_each_possible_cpu(cpu) {
		int i, cache_id;

		topology_id = find_acpi_cpu_topology(cpu, 0);
		if (topology_id < 0)
			return topology_id;

		if (acpi_cpu_is_threaded(cpu)) {
			cpu_topology[cpu].thread_id = topology_id;
			topology_id = find_acpi_cpu_topology(cpu, 1);
			cpu_topology[cpu].core_id   = topology_id;
		} else {
			cpu_topology[cpu].thread_id  = -1;
			cpu_topology[cpu].core_id    = topology_id;
		}
		topology_id = find_acpi_cpu_topology_package(cpu);
		cpu_topology[cpu].package_id = topology_id;

		i = acpi_find_last_cache_level(cpu);

		if (i > 0) {
			/*
			 * this is the only part of cpu_topology that has
			 * a direct relationship with the cache topology
			 */
			cache_id = find_acpi_cpu_cache_topology(cpu, i);
			if (cache_id > 0)
				cpu_topology[cpu].llc_id = cache_id;
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_ARM64_AMU_EXTN

#undef pr_fmt
#define pr_fmt(fmt) "AMU: " fmt

static DEFINE_PER_CPU_READ_MOSTLY(unsigned long, arch_max_freq_scale) =  1UL << (2 * SCHED_CAPACITY_SHIFT);
static DEFINE_PER_CPU_READ_MOSTLY(unsigned long, arch_max_freq_ref);
static cpumask_var_t amu_fie_cpus;

/*
 * Upstream 16d1e27475f6 ("arm64: Provide an AMU-based version of
 * arch_freq_get_on_cpu") samples the AMU cycle counters into one
 * coherent struct. Backported: the 5.10 tree updates the FIE counters
 * in init_cpu_freq_invariance_counters() instead of
 * update_freq_counters_refs(), so both refs are stored there.
 */
struct amu_cntr_sample {
	u64		arch_const_cycles_prev;
	u64		arch_core_cycles_prev;
	unsigned long	last_scale_update;
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct amu_cntr_sample, cpu_amu_samples);

void init_cpu_freq_invariance_counters(void)
{
	struct amu_cntr_sample *amu_sample = this_cpu_ptr(&cpu_amu_samples);

	amu_sample->arch_core_cycles_prev =
		read_sysreg_s(SYS_AMEVCNTR0_CORE_EL0);
	amu_sample->arch_const_cycles_prev =
		read_sysreg_s(SYS_AMEVCNTR0_CONST_EL0);
}

static int validate_cpu_freq_invariance_counters(int cpu)
{
	u64 max_freq_hz, ratio;

	if (!cpu_has_amu_feat(cpu)) {
		pr_debug("CPU%d: counters are not supported.\n", cpu);
		return -EINVAL;
	}

	init_cpu_freq_invariance_counters();

	{
		struct amu_cntr_sample *amu_sample = per_cpu_ptr(&cpu_amu_samples, cpu);

		if (unlikely(!amu_sample->arch_const_cycles_prev ||
			     !amu_sample->arch_core_cycles_prev)) {
			pr_debug("CPU%d: cycle counters are not enabled.\n", cpu);
			return -EINVAL;
		}
	}

	/* Convert maximum frequency from KHz to Hz and validate */
	max_freq_hz = cpufreq_get_hw_max_freq(cpu) * 1000ULL;
	if (unlikely(!max_freq_hz)) {
		pr_debug("CPU%d: invalid maximum frequency.\n", cpu);
		return -EINVAL;
	}

	/*
	 * Pre-compute the fixed ratio between the frequency of the constant
	 * counter and the maximum frequency of the CPU.
	 *
	 *			      const_freq
	 * arch_max_freq_scale =   ---------------- * SCHED_CAPACITY_SCALE²
	 *			   cpuinfo_max_freq
	 *
	 * We use a factor of 2 * SCHED_CAPACITY_SHIFT -> SCHED_CAPACITY_SCALE²
	 * in order to ensure a good resolution for arch_max_freq_scale for
	 * very low arch timer frequencies (down to the KHz range which should
	 * be unlikely).
	 */
	ratio = (u64)arch_timer_get_rate() << (2 * SCHED_CAPACITY_SHIFT);
	ratio = div64_u64(ratio, max_freq_hz);
	if (!ratio) {
		WARN_ONCE(1, "System timer frequency too low.\n");
		return -EINVAL;
	}

	per_cpu(arch_max_freq_scale, cpu) = (unsigned long)ratio;

	/*
	 * Reference (maximum) frequency in KHz, used to convert the
	 * frequency scale factor back into a concrete frequency.
	 */
	per_cpu(arch_max_freq_ref, cpu) = cpufreq_get_hw_max_freq(cpu);

	return 0;
}

static inline bool
enable_policy_freq_counters(int cpu, cpumask_var_t valid_cpus)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

	if (!policy) {
		pr_debug("CPU%d: No cpufreq policy found.\n", cpu);
		return false;
	}

	if (cpumask_subset(policy->related_cpus, valid_cpus))
		cpumask_or(amu_fie_cpus, policy->related_cpus,
			   amu_fie_cpus);

	cpufreq_cpu_put(policy);

	return true;
}

static DEFINE_STATIC_KEY_FALSE(amu_fie_key);
#define amu_freq_invariant() static_branch_unlikely(&amu_fie_key)

static int __init init_amu_fie(void)
{
	cpumask_var_t valid_cpus;
	bool have_policy = false;
	int ret = 0;
	int cpu;

	if (!zalloc_cpumask_var(&valid_cpus, GFP_KERNEL))
		return -ENOMEM;

	if (!zalloc_cpumask_var(&amu_fie_cpus, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto free_valid_mask;
	}

	for_each_present_cpu(cpu) {
		if (validate_cpu_freq_invariance_counters(cpu))
			continue;
		cpumask_set_cpu(cpu, valid_cpus);
		have_policy |= enable_policy_freq_counters(cpu, valid_cpus);
	}

	/*
	 * If we are not restricted by cpufreq policies, we only enable
	 * the use of the AMU feature for FIE if all CPUs support AMU.
	 * Otherwise, enable_policy_freq_counters has already enabled
	 * policy cpus.
	 */
	if (!have_policy && cpumask_equal(valid_cpus, cpu_present_mask))
		cpumask_or(amu_fie_cpus, amu_fie_cpus, valid_cpus);

	if (!cpumask_empty(amu_fie_cpus)) {
		pr_info("CPUs[%*pbl]: counters will be used for FIE.",
			cpumask_pr_args(amu_fie_cpus));
		static_branch_enable(&amu_fie_key);
	}

	/*
	 * If the system is not fully invariant after AMU init, disable
	 * partial use of counters for frequency invariance.
	 */
	if (!topology_scale_freq_invariant())
		static_branch_disable(&amu_fie_key);

free_valid_mask:
	free_cpumask_var(valid_cpus);

	return ret;
}
late_initcall_sync(init_amu_fie);

bool arch_freq_counters_available(const struct cpumask *cpus)
{
	return amu_freq_invariant() &&
	       cpumask_subset(cpus, amu_fie_cpus);
}

void topology_scale_freq_tick(void)
{
	struct amu_cntr_sample *amu_sample = this_cpu_ptr(&cpu_amu_samples);
	u64 prev_core_cnt, prev_const_cnt;
	u64 core_cnt, const_cnt, scale;
	int cpu = smp_processor_id();

	if (!amu_freq_invariant())
		return;

	if (!cpumask_test_cpu(cpu, amu_fie_cpus))
		return;

	prev_const_cnt = amu_sample->arch_const_cycles_prev;
	prev_core_cnt = amu_sample->arch_core_cycles_prev;

	init_cpu_freq_invariance_counters();

	const_cnt = amu_sample->arch_const_cycles_prev;
	core_cnt = amu_sample->arch_core_cycles_prev;

	/*
	 * This should not happen unless the AMUs have been reset and the
	 * counter values have not been restored - unlikely
	 */
	if (unlikely(core_cnt <= prev_core_cnt ||
		     const_cnt <= prev_const_cnt))
		return;

	/*
	 *	    /\core    arch_max_freq_scale
	 * scale =  ------- * --------------------
	 *	    /\const   SCHED_CAPACITY_SCALE
	 *
	 * See validate_cpu_freq_invariance_counters() for details on
	 * arch_max_freq_scale and the use of SCHED_CAPACITY_SHIFT.
	 */
	scale = core_cnt - prev_core_cnt;
	scale *= this_cpu_read(arch_max_freq_scale);
	scale = div64_u64(scale >> SCHED_CAPACITY_SHIFT,
			  const_cnt - prev_const_cnt);

	scale = min_t(unsigned long, scale, SCHED_CAPACITY_SCALE);
	this_cpu_write(freq_scale, (unsigned long)scale);

	amu_sample->last_scale_update = jiffies;
}

#define AMU_SAMPLE_EXP_MS	20

static unsigned int arch_scale_freq_ref(int cpu)
{
	return per_cpu(arch_max_freq_ref, cpu);
}

unsigned int arch_freq_get_on_cpu(int cpu)
{
	struct amu_cntr_sample *amu_sample;
	u64 prev_const_cnt, prev_core_cnt;
	u64 core_cnt, const_cnt, scale;
	unsigned long last_update;
	unsigned int freq = 0;

	/*
	 * 5.10 API returns 0 when the average frequency cannot be read
	 * (upstream 38e480d4fca changed the prototype to signed int for
	 * error codes; that prerequisite is not part of this tree).
	 */
	if (!amu_freq_invariant() || !cpumask_test_cpu(cpu, amu_fie_cpus) ||
	    !arch_scale_freq_ref(cpu))
		return 0;

	amu_sample = per_cpu_ptr(&cpu_amu_samples, cpu);
	last_update = amu_sample->last_scale_update;

	/*
	 * For those CPUs that are in full dynticks mode, or those that have
	 * not seen tick for a while, sample the counters directly instead
	 * of waiting for the next tick (upstream walks to an active CPU in
	 * the same freq domain; 5.10 has no NOHZ_FULL here, so reading
	 * AMU registers in place is equivalent and cheaper).
	 */
	if (!housekeeping_cpu(cpu, HK_FLAG_TICK) ||
	    time_is_before_jiffies(last_update + msecs_to_jiffies(AMU_SAMPLE_EXP_MS))) {
		prev_const_cnt = amu_sample->arch_const_cycles_prev;
		prev_core_cnt = amu_sample->arch_core_cycles_prev;

		init_cpu_freq_invariance_counters();

		const_cnt = amu_sample->arch_const_cycles_prev;
		core_cnt = amu_sample->arch_core_cycles_prev;

		if (unlikely(core_cnt <= prev_core_cnt ||
			     const_cnt <= prev_const_cnt))
			return 0;

		/*
		 * Reversed computation to the one used in
		 * topology_scale_freq_tick() to determine freq_scale.
		 */
		scale = core_cnt - prev_core_cnt;
		scale *= per_cpu(arch_max_freq_scale, cpu);
		scale = div64_u64(scale >> SCHED_CAPACITY_SHIFT,
				  const_cnt - prev_const_cnt);
	} else {
		scale = topology_get_freq_scale(cpu);
	}

	freq = scale * arch_scale_freq_ref(cpu);
	freq >>= SCHED_CAPACITY_SHIFT;
	return freq;
}
#endif /* CONFIG_ARM64_AMU_EXTN */
