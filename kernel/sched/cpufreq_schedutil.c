/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#include "sched.h"

/* Hardcoded ratelimits */
#define RATE_DELAY_NS (500 * NSEC_PER_USEC)

struct sugov_policy {
	struct cpufreq_policy *policy;

	raw_spinlock_t update_lock;
	u64 last_freq_update_time;
	unsigned int next_freq;
	unsigned int cached_raw_freq;

	/* The next fields are only needed if fast switch cannot be used. */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;
	bool limits_changed;
	bool need_freq_update;
};

struct sugov_cpu {
	struct update_util_data update_util;
	struct sugov_policy *sg_policy;
	unsigned int cpu;
	unsigned long bw_min;
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

/************************ Governor internals ***********************/

static bool sugov_should_update_freq(unsigned int cpu, struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	if (unlikely(!cpufreq_this_cpu_can_update(sg_policy->policy)))
		return false;

	if (unlikely(sg_policy->limits_changed)) {
		sg_policy->limits_changed = false;
		sg_policy->need_freq_update = true;
		return true;
	}

	/* If the last frequency wasn't set yet then we can still amend it */
	if (sg_policy->work_in_progress)
		return true;

	delta_ns = time - sg_policy->last_freq_update_time;

	return RATE_DELAY_NS;
}

static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	if (sg_policy->need_freq_update)
		sg_policy->need_freq_update = false;
	else if (sg_policy->next_freq == next_freq)
		return false;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	return true;
}

static void sugov_deferred_update(struct sugov_policy *sg_policy)
{
	if (sg_policy->work_in_progress)
		return;

	sched_irq_work_queue(&sg_policy->irq_work);
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedutil policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy, unsigned long util, u64 time)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int idx, l_freq, h_freq;
	unsigned int *best_freq = &l_freq;
	unsigned int freq;

	freq = arch_scale_freq_ref(policy->cpu);
	freq = map_util_freq(util, freq, arch_scale_cpu_capacity(policy->cpu));
	freq = clamp_val(freq, policy->min, policy->max);

	if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
		return sg_policy->next_freq;

	sg_policy->cached_raw_freq = freq;

	idx = cpufreq_frequency_table_target(policy, freq, CPUFREQ_RELATION_L);
	l_freq = policy->freq_table[idx].frequency;

	if (l_freq == policy->min)
		goto found;

	idx = cpufreq_frequency_table_target(policy, freq, CPUFREQ_RELATION_H);
	h_freq = policy->freq_table[idx].frequency;
	if (l_freq <= h_freq)
		goto found;

	/*
	 * Use the frequency step below if the calculated frequency is <20%
	 * higher than it.
	 */
	if (mult_frac(100, freq - h_freq, l_freq - h_freq) < 20)
		best_freq = &h_freq;

found:
	return cpufreq_driver_resolve_freq(policy, *best_freq);
}

static __always_inline
unsigned long apply_dvfs_headroom(int cpu, unsigned long util, unsigned long max_cap)
{
	unsigned long headroom;

	if (!util || util >= max_cap)
		return util;

	/*
	 * Headroom is always capped to the maximum of 25% (big), 50% (small) of util.
	 * Scaled relatively to the unused capacity and minimum CPU freq always get maximum headroom:
	 *
	 *					( max_cap - util ) * util * offset * 256
	 * 		headroom = -------------------------------------------
	 *			   				SCHED_CAPACITY_SCALE²
	 *
	 * Offset: A constant to make sure first minimum freq get maximum headroom desired.
	 */
	headroom = max_cap - util;
	headroom *= util;

	/* Offset calculation */
	headroom += (headroom >> 2); // Set the initial 25% headroom
	if (cpumask_test_cpu(cpu, cpu_lp_mask))
		headroom *= 5; // Set the initial 50% headroom

	headroom *= SCHED_CAPACITY_SCALE >> 2; // Multiply by 256
	headroom >>= SCHED_CAPACITY_SHIFT * 2; // Divide by SCHED_CAPACITY_SCALE²

	return util + headroom;
}

unsigned long sugov_effective_cpu_perf(int cpu, unsigned long actual,
				 unsigned long min,
				 unsigned long max)
{
	/* Add dvfs headroom to actual utilization */
	actual = apply_dvfs_headroom(cpu, actual, max);
	/* Actually we don't need to target the max performance */
	if (actual < max)
		max = actual;

	/*
	 * Ensure at least minimum performance while providing more compute
	 * capacity when possible.
	 */
	return max(min, max);
}

static unsigned long sugov_get_util(struct cpufreq_policy *policy)
{
	unsigned int j;
	unsigned long best_util = 0;
	unsigned long best_min, best_max;

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long cpu_util, min, max;

		cpu_util = cpu_util_cfs_boost(j_sg_cpu->cpu);
		cpu_util = effective_cpu_util(j_sg_cpu->cpu, cpu_util, &min, &max);
		j_sg_cpu->bw_min = min;

		if (best_util >= cpu_util)
			continue;

		best_util = cpu_util;
		best_min = min;
		best_max = max;

		if (best_util >= best_max)
			break;
	}

	return sugov_effective_cpu_perf(policy->cpu, best_util, best_min, best_max);
}

/*
 * Make sugov_should_update_freq() ignore the rate limit when DL
 * has increased the utilization.
 */
static __always_inline
void ignore_dl_rate_limit(struct sugov_cpu *sg_cpu)
{
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_min)
		sg_cpu->sg_policy->limits_changed = true;
}

static void
sugov_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util;
	unsigned int next_freq;

	raw_spin_lock(&sg_policy->update_lock);

	ignore_dl_rate_limit(sg_cpu);

	if (!sugov_should_update_freq(sg_cpu->cpu, sg_policy, time))
		goto unlock;

	util = sugov_get_util(sg_policy->policy);
	next_freq = get_next_freq(sg_policy, util, time);

	if (sugov_update_next_freq(sg_policy, time, next_freq))
		sugov_deferred_update(sg_policy);

unlock:
	raw_spin_unlock(&sg_policy->update_lock);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
	int ret;

	mutex_lock(&sg_policy->work_lock);

	ret = __cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq, CPUFREQ_RELATION_L);
	if (ret)
		sg_policy->limits_changed = true;

	mutex_unlock(&sg_policy->work_lock);

	sg_policy->work_in_progress = false;
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	queue_kthread_work(&sg_policy->worker, &sg_policy->work);
}

/********************** cpufreq governor interface *********************/
#ifdef CONFIG_ENERGY_MODEL

static void rebuild_sd_workfn(struct work_struct *work)
{
	rebuild_sched_domains_energy();
}

static DECLARE_WORK(rebuild_sd_work, rebuild_sd_workfn);

/*
 * EAS shouldn't be attempted without sugov, so rebuild the sched_domains
 * on governor changes to make sure the scheduler knows about it.
 */
static void sugov_eas_rebuild_sd(void)
{
	/*
	 * When called from the cpufreq_register_driver() path, the
	 * cpu_hotplug_lock is already held, so use a work item to
	 * avoid nested locking in rebuild_sched_domains().
	 */
	schedule_work(&rebuild_sd_work);
}
#else
static inline void sugov_eas_rebuild_sd(void) { };
#endif

struct cpufreq_governor schedutil_gov;

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size = sizeof(struct sched_attr),
		.sched_policy = SCHED_DEADLINE,
		.sched_flags = SCHED_FLAG_SUGOV,
		.sched_nice = 0,
		.sched_priority = 0,
		/*
		 * Fake (unused) bandwidth; workaround to "fix"
		 * priority inheritance.
		 */
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	init_kthread_work(&sg_policy->work, sugov_work);
	init_kthread_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	if (!policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	flush_kthread_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto alloc_error;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	policy->governor_data = sg_policy;

	sugov_eas_rebuild_sd();
	return 0;

free_sg_policy:
	sugov_policy_free(sg_policy);

alloc_error:
	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static int sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	policy->governor_data = NULL;

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);

	sugov_eas_rebuild_sd();
	return 0;
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq = 0;
	sg_policy->cached_raw_freq = 0;
	sg_policy->work_in_progress = false;
	sg_policy->limits_changed = false;
	sg_policy->need_freq_update = false;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu = cpu;
		sg_cpu->sg_policy = sg_policy;

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util, sugov_update_shared);
	}

	return 0;
}

static int sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	irq_work_sync(&sg_policy->irq_work);
	kthread_cancel_work_sync(&sg_policy->work);

	return 0;
}

static int sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	mutex_lock(&sg_policy->work_lock);
	cpufreq_policy_apply_limits(policy);
	mutex_unlock(&sg_policy->work_lock);

	sg_policy->limits_changed = true;

	return 0;
}

static int cpufreq_schedutil_cb(struct cpufreq_policy *policy,
				unsigned int event)
{
	switch(event) {
	case CPUFREQ_GOV_POLICY_INIT:
		return sugov_init(policy);
	case CPUFREQ_GOV_POLICY_EXIT:
		return sugov_exit(policy);
	case CPUFREQ_GOV_START:
		return sugov_start(policy);
	case CPUFREQ_GOV_STOP:
		return sugov_stop(policy);
	case CPUFREQ_GOV_LIMITS:
		return sugov_limits(policy);
	default:
		BUG();
	}
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDUTIL
#ifndef CONFIG_ENERGY_MODEL
static
#endif
#endif
struct cpufreq_governor schedutil_gov = {
	.name = "schedutil",
	.governor = cpufreq_schedutil_cb,
	.owner = THIS_MODULE,
};

cpufreq_governor_init(schedutil_gov);