#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>

#include "sched.h"

struct queue_params {
	bool queued;
	u64 last_queue_ns;
	int (*try_lock_func)(void *);
	void *lock_ptr;
	void (*lock_func)(void *);
	void (*unlock_func)(void *);
};

struct sugov_policy {
	struct cpufreq_policy *policy;

	u64 last_update_ns;
	unsigned long max_freq_ratio;
	unsigned long prev_util;

	/* Queued spin/mutex lock stuffs */
	struct queue_params queue[2];
	struct mutex work_lock;
	raw_spinlock_t	update_lock;

	/* Worker stuffs */
	struct irq_work irq_work;
	struct kthread_work work;
	struct kthread_worker worker;
	struct task_struct *thread;
};

struct sugov_cpu {
	struct update_util_data update_util;
	struct sugov_policy *sg_policy;
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

/************************ Governor internals ***********************/

/*
 * Queueing a large number of threads to perform the same task is inefficient:
 * 1) Increasing lock contention.
 * 2) Adding unnecessary redundancy and overhead.
 *
 * This queued {mutex, spin}lock mechanism mitigates this by:
 * 1) Ensuring that only one thread extra,
 *	  apart from the running one, that is busy-waiting.
 * 2) Caching the latest queue time and utilise it
 * 	  once the thread exits the queue.
 *
 * When 'time' is NULL, the operation is for mutex lock.
 * When 'time' is !NULL, the operation is for spin lock.
 */
static void
sugov_queue_func(struct sugov_policy *sg_policy, u64 *time,
				 void (*critical_func)(struct sugov_policy *, u64))
{
	struct queue_params *q = &sg_policy->queue[!!time];

	if (q->queued) {
		q->last_queue_ns = *time; // save
		return;
	}

	if (!q->try_lock_func(q->lock_ptr)) {
		q->queued = true;
		q->lock_func(q->lock_ptr);
		q->queued = false;
		*time = q->last_queue_ns; // restore
	}

	critical_func(sg_policy, *time);

	q->unlock_func(q->lock_ptr);
}

static unsigned long apply_dvfs_headroom(unsigned long util)
{
	unsigned long headroom;

	headroom = SCHED_CAPACITY_SCALE - util;
	headroom *= util;
	headroom >>= (SCHED_CAPACITY_SHIFT + 1);

	return util + headroom;
}

static unsigned long
sugov_eval_cpu_perf(unsigned int cpu, unsigned long actual,
					unsigned long min)
{
	unsigned long max = get_actual_cpu_capacity(cpu);

	if (min >= max)
		return min;

	/* Add dvfs headroom to actual utilization */
	if (actual && actual < max)
		actual = apply_dvfs_headroom(actual);

	/* Actually we don't need to target the max performance */
	max = min(max, actual);

	/*
	 * Ensure at least minimum performance while providing more compute
	 * capacity when possible.
	 */
	return max(min, max);
}

unsigned long
sugov_effective_cpu_perf(int cpu, unsigned long actual,
						 unsigned long min, unsigned long max)
{
	return sugov_eval_cpu_perf(cpu, actual, min);
}

static unsigned long sugov_get_util(struct cpufreq_policy *policy)
{
	unsigned long best_util = 0, best_min = 0;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus) {
		unsigned long min, util;

		util = cpu_util_cfs_boost(cpu);
		util = effective_cpu_util(cpu, util, &min, NULL);

		best_util = max(best_util, util);
		best_min  = max(best_min, min);
	}

	return sugov_eval_cpu_perf(policy->cpu, best_util, best_min);
}

static unsigned int
sugov_resolve_freq(struct cpufreq_policy *policy, unsigned int next_freq)
{
	unsigned int idx, l_freq, h_freq, *best_freq = &l_freq;

	idx = cpufreq_frequency_table_target(policy, next_freq, CPUFREQ_RELATION_L);

	l_freq = policy->freq_table[idx].frequency;
	if (l_freq == policy->min)
		goto done;

	idx = cpufreq_frequency_table_target(policy, next_freq, CPUFREQ_RELATION_H);

	h_freq = policy->freq_table[idx].frequency;
	if (l_freq <= h_freq)
		goto done;

	/*
	 * Use the frequency step be*low if the calculated frequency is <20%
	 * higher than it.
	 */
	if (mult_frac(100, next_freq - h_freq, l_freq - h_freq) < 20)
		best_freq = &h_freq;

done:
	return *best_freq;
}

static inline
bool sugov_exceed_limit(struct sugov_policy *sg_policy, u64 time)
{
	const u64 rate_limit_ns = 4000UL * NSEC_PER_USEC;

	return time - sg_policy->last_update_ns > rate_limit_ns;
}

static void
sugov_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util;
	unsigned int freq;

	if (!sugov_exceed_limit(sg_policy, time))
		return;

	util = sugov_get_util(policy);
	sg_policy->prev_util = util;

	/* Calculate the target frequency */
	freq = util * sg_policy->max_freq_ratio;
	freq = sugov_resolve_freq(policy, freq);

	if (freq == policy->cur)
		return;

	sg_policy->last_update_ns = time;

	cpufreq_driver_resolve_freq(policy, freq);
	sched_irq_work_queue(&sg_policy->irq_work);
}

static void
sugov_update_shared(struct update_util_data *hook, u64 time,
					unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu,
											update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;

	sugov_queue_func(sg_policy, &time, sugov_update_freq);
}

static void sugov_do_work(struct sugov_policy *sg_policy, u64 time)
{
	__cpufreq_driver_target(sg_policy->policy, 0, CPUFREQ_RELATION_L);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);

	sugov_queue_func(sg_policy, NULL, sugov_do_work);
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
	struct cpufreq_policy *policy = sg_policy->policy;

	init_kthread_work(&sg_policy->work, sugov_work);
	init_kthread_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	sched_set_fifo(thread);

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

#define DEFINE_SPINLOCK_WRAPPER(func_name, wrapped_func, ret_type) \
static ret_type func_name(raw_spinlock_t *lock) { \
	return wrapped_func(lock); \
}

DEFINE_SPINLOCK_WRAPPER(do_spin_trylock, raw_spin_trylock, int)
DEFINE_SPINLOCK_WRAPPER(do_spin_lock, raw_spin_lock, void)
DEFINE_SPINLOCK_WRAPPER(do_spin_unlock, raw_spin_unlock, void)

#define SETUP_QUEUE_PARAMS(sg_policy, i, tmp) \
{ \
	tmp.queued = false; \
	tmp.last_queue_ns = 0; \
	\
	if (i) { \
		tmp.lock_ptr = (void *)&(sg_policy)->update_lock; \
		tmp.try_lock_func = (int (*)(void *))do_spin_trylock; \
		tmp.lock_func = (void (*)(void *))do_spin_lock; \
		tmp.unlock_func = (void (*)(void *))do_spin_unlock; \
	} else { \
		tmp.lock_ptr = (void *)&(sg_policy)->work_lock; \
		tmp.try_lock_func = (int (*)(void *))mutex_trylock; \
		tmp.lock_func = (void (*)(void *))mutex_lock; \
		tmp.unlock_func = (void (*)(void *))mutex_unlock; \
	} \
} \

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned long max_cap = arch_scale_cpu_capacity(policy->cpu);
	unsigned int i, cpu;

	sg_policy->last_update_ns = 0;
	sg_policy->prev_util	  = 0;
	sg_policy->max_freq_ratio = policy->cpuinfo.max_freq / max_cap;

	for (i = 0; i < 2; i++) {
		struct queue_params tmp;

		SETUP_QUEUE_PARAMS(sg_policy, i, tmp);
		sg_policy->queue[i] = tmp;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
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

static void sugov_do_limits(struct sugov_policy *sg_policy, u64 time)
{
	cpufreq_policy_apply_limits(sg_policy->policy);
}

static int sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	sugov_queue_func(sg_policy, NULL, sugov_do_limits);

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
