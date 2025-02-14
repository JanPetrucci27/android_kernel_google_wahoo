/* Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
 * Copyright (C) 2006-2007 Adam Belay <abelay@novell.com>
 * Copyright (C) 2009 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/cpu.h>
#include <linux/of.h>
#include <linux/irqchip/msm-mpm-irq.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/suspend.h>
#include <linux/pm_qos.h>
#include <linux/of_platform.h>
#include <linux/smp.h>
#include <linux/remote_spinlock.h>
#include <linux/msm_remote_spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/coresight-cti.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/cpu_pm.h>
#include <linux/arm-smccc.h>
#include <linux/psci.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include <soc/qcom/rpm-notifier.h>
#include <soc/qcom/event_timer.h>
#include <soc/qcom/lpm-stats.h>
#include <soc/qcom/jtag.h>
#include <asm/cputype.h>
#include <asm/arch_timer.h>
#include <asm/cacheflush.h>
#include <asm/suspend.h>
#include <asm/cpuidle.h>
#include "lpm-levels.h"
#include "lpm-workarounds.h"
#include <trace/events/power.h>
#define CREATE_TRACE_POINTS
#include <trace/events/trace_msm_low_power.h>
#include "../../drivers/clk/msm/clock.h"

#define SCLK_HZ (32768)
#define SCM_HANDOFF_LOCK_ID "S:7"
#define PSCI_POWER_STATE(reset) (reset << 30)
#define PSCI_AFFINITY_LEVEL(lvl) ((lvl & 0x3) << 24)

static remote_spinlock_t scm_handoff_lock;

struct lpm_cluster *lpm_root_node;

static DEFINE_PER_CPU(struct lpm_cluster*, cpu_cluster);

static int lpm_cpu_callback(struct notifier_block *cpu_nb,
				unsigned long action, void *hcpu);

static void cluster_unprepare(struct lpm_cluster *cluster,
		const struct cpumask *cpu, int child_idx);
static void cluster_prepare(struct lpm_cluster *cluster,
		const struct cpumask *cpu, int child_idx);

static struct notifier_block __refdata lpm_cpu_nblk = {
	.notifier_call = lpm_cpu_callback,
};

static const bool print_parsed_dt = false;

/*
 * lpm_probe needs this to successfully create cpu levels..
 */
static bool dummy_param = true;
module_param_named(dummy_param, dummy_param, bool, 0444);

static int lpm_cpu_callback(struct notifier_block *cpu_nb,
	unsigned long action, void *hcpu)
{
	unsigned long cpu = (unsigned long) hcpu;
	struct lpm_cluster *cluster = per_cpu(cpu_cluster, (unsigned int) cpu);

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_DYING:
		cluster_prepare(cluster, get_cpu_mask((unsigned int) cpu),
					NR_LPM_LEVELS);
		break;
	case CPU_STARTING:
		cluster_unprepare(cluster, get_cpu_mask((unsigned int) cpu),
					NR_LPM_LEVELS);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

int set_l2_mode(struct low_power_ops *ops, int mode, bool notify_rpm)
{
	int lpm = mode;
	int rc = 0;
	struct low_power_ops *cpu_ops = per_cpu(cpu_cluster,
			smp_processor_id())->lpm_dev;

	if (cpu_ops->tz_flag & MSM_SCM_L2_OFF ||
			cpu_ops->tz_flag & MSM_SCM_L2_GDHS)
		coresight_cti_ctx_restore();

	switch (mode) {
	case MSM_SPM_MODE_STANDALONE_POWER_COLLAPSE:
	case MSM_SPM_MODE_POWER_COLLAPSE:
	case MSM_SPM_MODE_FASTPC:
		cpu_ops->tz_flag = MSM_SCM_L2_OFF;
		coresight_cti_ctx_save();
		break;
	case MSM_SPM_MODE_GDHS:
		cpu_ops->tz_flag = MSM_SCM_L2_GDHS;
		coresight_cti_ctx_save();
		break;
	case MSM_SPM_MODE_CLOCK_GATING:
	case MSM_SPM_MODE_RETENTION:
	case MSM_SPM_MODE_DISABLED:
		cpu_ops->tz_flag = MSM_SCM_L2_ON;
		break;
	default:
		cpu_ops->tz_flag = MSM_SCM_L2_ON;
		lpm = MSM_SPM_MODE_DISABLED;
		break;
	}
	rc = msm_spm_config_low_power_mode(ops->spm, lpm, notify_rpm);

	if (rc)
		pr_err("%s: Failed to set L2 low power mode %d, ERR %d",
				__func__, lpm, rc);

	return rc;
}

int set_l3_mode(struct low_power_ops *ops, int mode, bool notify_rpm)
{
	struct low_power_ops *cpu_ops = per_cpu(cpu_cluster,
			smp_processor_id())->lpm_dev;

	switch (mode) {
	case MSM_SPM_MODE_STANDALONE_POWER_COLLAPSE:
	case MSM_SPM_MODE_POWER_COLLAPSE:
	case MSM_SPM_MODE_FASTPC:
		cpu_ops->tz_flag |= MSM_SCM_L3_PC_OFF;
		break;
	default:
		break;
	}
	return msm_spm_config_low_power_mode(ops->spm, mode, notify_rpm);
}

int set_system_mode(struct low_power_ops *ops, int mode, bool notify_rpm)
{
	return msm_spm_config_low_power_mode(ops->spm, mode, notify_rpm);
}

static int set_device_mode(struct lpm_cluster *cluster, int ndevice,
		struct lpm_cluster_level *level)
{
	struct low_power_ops *ops;

	if (use_psci)
		return 0;

	ops = &cluster->lpm_dev[ndevice];
	if (ops && ops->set_mode)
		return ops->set_mode(ops, level->mode[ndevice],
				level->notify_rpm);
	else
		return -EINVAL;
}

static uint64_t get_cluster_sleep_time(struct lpm_cluster *cluster,
		struct cpumask *mask)
{
	int cpu;
	int next_cpu = raw_smp_processor_id();
	ktime_t next_event;
	struct cpumask online_cpus_in_cluster;

	next_event = KTIME_MAX;
	if (mask)
		cpumask_copy(mask, cpumask_of(raw_smp_processor_id()));

	cpumask_and(&online_cpus_in_cluster,
			&cluster->num_children_in_sync, cpu_online_mask);

	for_each_cpu(cpu, &online_cpus_in_cluster) {
		ktime_t *next_event_c;

		next_event_c = get_next_event_cpu(cpu);
		if (*next_event_c < next_event) {
			next_event = *next_event_c;
			next_cpu = cpu;
		}
	}

	if (mask)
		cpumask_copy(mask, cpumask_of(next_cpu));

	if (ktime_to_us(next_event) > ktime_to_us(ktime_get()))
		return ktime_to_us(ktime_sub(next_event, ktime_get()));
	else
		return 0;
}

static int cluster_select(struct lpm_cluster *cluster)
{
	int best_level = -1;
	int i;
	uint32_t sleep_us;

	if (!cluster)
		return -EINVAL;

	sleep_us = (uint32_t)get_cluster_sleep_time(cluster, NULL);

	for (i = 0; i < cluster->nlevels; i++) {
		struct lpm_cluster_level *level = &cluster->levels[i];
		struct power_params *pwr_params = &level->pwr;

		if (!lpm_cluster_mode_allow(cluster, i))
			continue;

		if (level->last_core_only &&
			cpumask_weight(cpu_online_mask) > 1)
			continue;

		if (!cpumask_equal(&cluster->num_children_in_sync,
					&level->num_cpu_votes))
			continue;

		if (sleep_us < (pwr_params->exit_latency +
						pwr_params->entry_latency))
			break;

		if (level->notify_rpm && msm_rpm_waiting_for_ack())
			continue;

		best_level = i;
	}

	return best_level;
}

static void cluster_notify(struct lpm_cluster *cluster,
		struct lpm_cluster_level *level, bool enter)
{
	if (level->is_reset && enter)
		cpu_cluster_pm_enter(cluster->aff_level);
	else if (level->is_reset && !enter)
		cpu_cluster_pm_exit(cluster->aff_level);
}

static int cluster_configure(struct lpm_cluster *cluster, int idx)
{
	struct lpm_cluster_level *level = &cluster->levels[idx];
	struct cpumask online_cpus;
	int ret, i;
	
	cpumask_and(&online_cpus, &cluster->num_children_in_sync,
					cpu_online_mask);

	if (!cpumask_equal(&cluster->num_children_in_sync, &cluster->child_cpus)
			|| is_IPI_pending(&online_cpus)) {
		return -EPERM;
	}

	for (i = 0; i < cluster->ndevices; i++) {
		ret = set_device_mode(cluster, i, level);
		if (ret)
			goto failed_set_mode;
	}

	if (level->notify_rpm) {
		struct cpumask nextcpu, *cpumask;
		uint64_t us;
		uint64_t sec;
		uint64_t nsec;

		us = get_cluster_sleep_time(cluster, &nextcpu);
		cpumask = level->disable_dynamic_routing ? NULL : &nextcpu;

		ret = msm_rpm_enter_sleep(0, cpumask);
		if (ret) {
			pr_info("Failed msm_rpm_enter_sleep() rc = %d\n", ret);
			goto failed_set_mode;
		}

		us = us + 1;
		sec = us;
		do_div(sec, USEC_PER_SEC);
		nsec = us - sec * USEC_PER_SEC;

		sec = sec * SCLK_HZ;
		if (nsec > 0) {
			nsec = nsec * NSEC_PER_USEC;
			do_div(nsec, NSEC_PER_SEC/SCLK_HZ);
		}
		us = sec + nsec;
		msm_mpm_enter_sleep(us, false, cpumask);
	}

	/* Notify cluster enter event after successfully config completion */
	cluster_notify(cluster, level, true);

	cluster->last_level = idx;
	return 0;

failed_set_mode:

	for (i = 0; i < cluster->ndevices; i++) {
		int rc = 0;
		level = &cluster->levels[cluster->default_level];
		rc = set_device_mode(cluster, i, level);
		BUG_ON(rc);
	}
	return ret;
}

static void cluster_prepare(struct lpm_cluster *cluster,
		const struct cpumask *cpu, int child_idx)
{
	int i;

	if (!cluster)
		return;

	if (cluster->min_child_level > child_idx)
		return;

	raw_spin_lock(&cluster->sync_lock);
	cpumask_or(&cluster->num_children_in_sync, cpu,
			&cluster->num_children_in_sync);

	for (i = 0; i < cluster->nlevels; i++) {
		struct lpm_cluster_level *lvl = &cluster->levels[i];

		if (child_idx >= lvl->min_child_level)
			cpumask_or(&lvl->num_cpu_votes, cpu,
					&lvl->num_cpu_votes);
	}

	/*
	 * cluster_select() does not make any configuration changes. So its ok
	 * to release the lock here. If a core wakes up for a rude request,
	 * it need not wait for another to finish its cluster selection and
	 * configuration process
	 */

	if (!cpumask_equal(&cluster->num_children_in_sync,
				&cluster->child_cpus))
		goto failed;

	i = cluster_select(cluster);
	if (i < 0)
		goto failed;

	if (cluster_configure(cluster, i))
		goto failed;

	cluster_prepare(cluster->parent, &cluster->num_children_in_sync, i);

failed:
	raw_spin_unlock(&cluster->sync_lock);
	return;
}

static void cluster_unprepare(struct lpm_cluster *cluster,
		const struct cpumask *cpu, int child_idx)
{
	struct lpm_cluster_level *level;
	bool first_cpu;
	int last_level, i, ret;

	if (!cluster)
		return;

	if (cluster->min_child_level > child_idx)
		return;

	raw_spin_lock(&cluster->sync_lock);
	last_level = cluster->default_level;
	first_cpu = cpumask_equal(&cluster->num_children_in_sync,
				&cluster->child_cpus);
	cpumask_andnot(&cluster->num_children_in_sync,
			&cluster->num_children_in_sync, cpu);

	for (i = 0; i < cluster->nlevels; i++) {
		struct lpm_cluster_level *lvl = &cluster->levels[i];

		if (child_idx >= lvl->min_child_level)
			cpumask_andnot(&lvl->num_cpu_votes,
					&lvl->num_cpu_votes, cpu);
	}

	if (!first_cpu || cluster->last_level == cluster->default_level)
		goto unlock_return;

	level = &cluster->levels[cluster->last_level];
	if (level->notify_rpm) {
		msm_rpm_exit_sleep();

		/* If RPM bumps up CX to turbo, unvote CX turbo vote
		 * during exit of rpm assisted power collapse to
		 * reduce the power impact
		 */

		lpm_wa_cx_unvote_send();
		msm_mpm_exit_sleep(false);
	}

	last_level = cluster->last_level;
	cluster->last_level = cluster->default_level;

	for (i = 0; i < cluster->ndevices; i++) {
		level = &cluster->levels[cluster->default_level];
		ret = set_device_mode(cluster, i, level);

		BUG_ON(ret);

	}

	cluster_notify(cluster, &cluster->levels[last_level], false);

	cluster_unprepare(cluster->parent, &cluster->child_cpus,
			last_level);
unlock_return:
	raw_spin_unlock(&cluster->sync_lock);
}

static inline void cpu_prepare(struct lpm_cluster *cluster, int cpu_index)
{
	/*
	 * Save JTAG registers for 8996v1.0 & 8996v2.x in C4 LPM
	 */
	if (cluster->cpu->levels[cpu_index].jtag_save_restore)
		msm_jtag_save_state();
}

static inline void cpu_unprepare(struct lpm_cluster *cluster, int cpu_index)
{
	/*
	 * Restore JTAG registers for 8996v1.0 & 8996v2.x in C4 LPM
	 */
	if (cluster->cpu->levels[cpu_index].jtag_save_restore)
		msm_jtag_restore_state();
}

static int get_cluster_id(struct lpm_cluster *cluster, int *aff_lvl)
{
	int state_id = 0;

	if (!cluster)
		return 0;

	raw_spin_lock(&cluster->sync_lock);

	if (!cpumask_equal(&cluster->num_children_in_sync,
				&cluster->child_cpus))
		goto unlock_and_return;

	state_id |= get_cluster_id(cluster->parent, aff_lvl);

	if (cluster->last_level != cluster->default_level) {
		struct lpm_cluster_level *level
			= &cluster->levels[cluster->last_level];

		state_id |= (level->psci_id & cluster->psci_mode_mask)
					<< cluster->psci_mode_shift;
		(*aff_lvl)++;
	}
unlock_and_return:
	raw_spin_unlock(&cluster->sync_lock);
	return state_id;
}

#if !defined(CONFIG_CPU_V7)
static void psci_enter_sleep(struct lpm_cluster *cluster, int idx)
{
	int affinity_level = 0, state_id, power_state;

	/*
	 * idx = 0 is the default LPM state
	 */
	if (!idx) {
		cpu_do_idle();
		return;
	}

#if !defined(CONFIG_ARM_PSCI)
	if (cluster->cpu->levels[idx].hyp_psci) {
		__invoke_psci_fn_smc(0xC4000021, 0, 0, 0);
		return;
	}
#endif

	state_id = get_cluster_id(cluster, &affinity_level);
	power_state = PSCI_POWER_STATE(cluster->cpu->levels[idx].is_reset);
	affinity_level = PSCI_AFFINITY_LEVEL(affinity_level);
	state_id |= (power_state | affinity_level
		| cluster->cpu->levels[idx].psci_id);
	!arm_cpuidle_suspend(state_id);
}
#else
static void psci_enter_sleep(struct lpm_cluster *cluster, int idx)
{
	WARN_ONCE(true, "PSCI cpu_suspend ops not supported\n");
}
#endif

static int lpm_cpuidle_select(struct cpuidle_driver *drv,
							  struct cpuidle_device *dev)
{
	return 0;
}

static void lpm_cpuidle_reflect(struct cpuidle_device *dev,
		 int idx)
{
}

static int lpm_cpuidle_enter(struct cpuidle_device *dev,
		struct cpuidle_driver *drv, int idx)
{
	if (!need_resched())
		cpu_do_idle();

	return idx;
}

#ifdef CONFIG_CPU_IDLE_MULTIPLE_DRIVERS
static int cpuidle_register_cpu(struct cpuidle_driver *drv,
		struct cpumask *mask)
{
	struct cpuidle_device *device;
	int cpu, ret;


	if (!mask || !drv)
		return -EINVAL;

	drv->cpumask = mask;
	ret = cpuidle_register_driver(drv);
	if (ret) {
		pr_err("Failed to register cpuidle driver %d\n", ret);
		goto failed_driver_register;
	}

	for_each_cpu(cpu, mask) {
		device = &per_cpu(cpuidle_dev, cpu);
		device->cpu = cpu;

		ret = cpuidle_register_device(device);
		if (ret) {
			pr_err("Failed to register cpuidle driver for cpu:%u\n",
					cpu);
			goto failed_driver_register;
		}
	}
	return ret;
failed_driver_register:
	for_each_cpu(cpu, mask)
		cpuidle_unregister_driver(drv);
	return ret;
}
#else
static int cpuidle_register_cpu(struct cpuidle_driver *drv,
		struct  cpumask *mask)
{
	return cpuidle_register(drv, NULL);
}
#endif

static struct cpuidle_governor lpm_governor = {
	.name =		"qcom",
	.rating =	30,
	.select =	lpm_cpuidle_select,
	.reflect =	lpm_cpuidle_reflect,
	.owner =	THIS_MODULE,
};

static int cluster_cpuidle_register(struct lpm_cluster *cl)
{
	int i = 0, ret = 0;
	unsigned cpu;
	struct lpm_cluster *p = NULL;

	if (!cl->cpu) {
		struct lpm_cluster *n;

		list_for_each_entry(n, &cl->child, list) {
			ret = cluster_cpuidle_register(n);
			if (ret)
				break;
		}
		return ret;
	}

	cl->drv = kzalloc(sizeof(*cl->drv), GFP_KERNEL);
	if (!cl->drv)
		return -ENOMEM;

	cl->drv->name = "msm_idle";

	for (i = 0; i < cl->cpu->nlevels; i++) {
		struct cpuidle_state *st = &cl->drv->states[i];
		struct lpm_cpu_level *cpu_level = &cl->cpu->levels[i];
		snprintf(st->name, CPUIDLE_NAME_LEN, "C%u\n", i);
		snprintf(st->desc, CPUIDLE_DESC_LEN, cpu_level->name);
		if (cpu_level->pwr.local_timer_stop)
			st->flags |= CPUIDLE_FLAG_TIMER_STOP;
		st->exit_latency = cpu_level->pwr.exit_latency + cpu_level->pwr.entry_latency;
		st->target_residency = cpu_level->pwr.min_residency;
		st->enter = lpm_cpuidle_enter;
	}

	cl->drv->state_count = cl->cpu->nlevels;
	cl->drv->safe_state_index = 0;
	for_each_cpu(cpu, &cl->child_cpus)
		per_cpu(cpu_cluster, cpu) = cl;

	for_each_possible_cpu(cpu) {
		if (cpu_online(cpu))
			continue;
		p = per_cpu(cpu_cluster, cpu);
		while (p) {
			int j;
			raw_spin_lock(&p->sync_lock);
			cpumask_set_cpu(cpu, &p->num_children_in_sync);
			for (j = 0; j < p->nlevels; j++)
				cpumask_copy(&p->levels[j].num_cpu_votes,
						&p->num_children_in_sync);
			raw_spin_unlock(&p->sync_lock);
			p = p->parent;
		}
	}
	ret = cpuidle_register_cpu(cl->drv, &cl->child_cpus);

	if (ret) {
		kfree(cl->drv);
		return -ENOMEM;
	}
	return 0;
}

/**
 * init_lpm - initializes the governor
 */
static int __init init_lpm(void)
{
	return cpuidle_register_governor(&lpm_governor);
}

postcore_initcall(init_lpm);

static void register_cpu_lpm_stats(struct lpm_cpu *cpu,
		struct lpm_cluster *parent)
{
	const char **level_name;
	int i;

	level_name = kzalloc(cpu->nlevels * sizeof(*level_name), GFP_KERNEL);

	if (!level_name)
		return;

	for (i = 0; i < cpu->nlevels; i++)
		level_name[i] = cpu->levels[i].name;

	lpm_stats_config_level("cpu", level_name, cpu->nlevels,
			parent->stats, &parent->child_cpus);

	kfree(level_name);
}

static void register_cluster_lpm_stats(struct lpm_cluster *cl,
		struct lpm_cluster *parent)
{
	const char **level_name;
	int i;
	struct lpm_cluster *child;

	if (!cl)
		return;

	level_name = kzalloc(cl->nlevels * sizeof(*level_name), GFP_KERNEL);

	if (!level_name)
		return;

	for (i = 0; i < cl->nlevels; i++)
		level_name[i] = cl->levels[i].level_name;

	cl->stats = lpm_stats_config_level(cl->cluster_name, level_name,
			cl->nlevels, parent ? parent->stats : NULL, NULL);

	kfree(level_name);

	if (cl->cpu) {
		register_cpu_lpm_stats(cl->cpu, cl);
		return;
	}

	list_for_each_entry(child, &cl->child, list)
		register_cluster_lpm_stats(child, cl);
}

static int lpm_suspend_prepare(void)
{
	msm_mpm_suspend_prepare();

	return 0;
}

static void lpm_suspend_wake(void)
{
	msm_mpm_suspend_wake();
}

static int lpm_suspend_enter(suspend_state_t state)
{
	int cpu = raw_smp_processor_id();
	struct lpm_cluster *cluster = per_cpu(cpu_cluster, cpu);
	struct lpm_cpu *lpm_cpu = cluster->cpu;
	const struct cpumask *cpumask = get_cpu_mask(cpu);
	int idx;

	if (need_resched())
		return 0;

	for (idx = lpm_cpu->nlevels - 1; idx >= 0; idx--) {
		if (lpm_cpu_mode_allow(cpu, idx))
			break;
	}

	if (idx < 0) {
		pr_err("Failed suspend\n");
		return 0;
	}

	cpu_prepare(cluster, idx);
	cluster_prepare(cluster, cpumask, idx);

	stop_critical_timings();
	psci_enter_sleep(cluster, idx);
	start_critical_timings();

	cluster_unprepare(cluster, cpumask, idx);
	cpu_unprepare(cluster, idx);

	return 0;
}

static const struct platform_suspend_ops lpm_suspend_ops = {
	.enter = lpm_suspend_enter,
	.valid = suspend_valid_only_mem,
	.prepare_late = lpm_suspend_prepare,
	.wake = lpm_suspend_wake,
};

static int lpm_probe(struct platform_device *pdev)
{
	int ret;
	struct kobject *module_kobj = NULL;

	get_online_cpus();
	lpm_root_node = lpm_of_parse_cluster(pdev);

	if (IS_ERR_OR_NULL(lpm_root_node)) {
		pr_err("%s(): Failed to probe low power modes\n", __func__);
		put_online_cpus();
		return PTR_ERR(lpm_root_node);
	}

	if (print_parsed_dt)
		cluster_dt_walkthrough(lpm_root_node);

	/*
	 * Register hotplug notifier before broadcast time to ensure there
	 * to prevent race where a broadcast timer might not be setup on for a
	 * core.  BUG in existing code but no known issues possibly because of
	 * how late lpm_levels gets initialized.
	 */
	suspend_set_ops(&lpm_suspend_ops);

	ret = remote_spin_lock_init(&scm_handoff_lock, SCM_HANDOFF_LOCK_ID);
	if (ret) {
		pr_err("%s: Failed initializing scm_handoff_lock (%d)\n",
			__func__, ret);
		put_online_cpus();
		return ret;
	}

	register_cluster_lpm_stats(lpm_root_node, NULL);

	ret = cluster_cpuidle_register(lpm_root_node);
	put_online_cpus();
	if (ret) {
		pr_err("%s()Failed to register with cpuidle framework\n",
				__func__);
		goto failed;
	}
	register_hotcpu_notifier(&lpm_cpu_nblk);
	module_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
	if (!module_kobj) {
		pr_err("%s: cannot find kobject for module %s\n",
			__func__, KBUILD_MODNAME);
		ret = -ENOENT;
		goto failed;
	}

	ret = create_cluster_lvl_nodes(lpm_root_node, module_kobj);
	if (ret) {
		pr_err("%s(): Failed to create cluster level nodes\n",
				__func__);
		goto failed;
	}

	return 0;
failed:
	free_cluster_node(lpm_root_node);
	lpm_root_node = NULL;
	return ret;
}

static struct of_device_id lpm_mtch_tbl[] = {
	{.compatible = "qcom,lpm-levels"},
	{},
};

static struct platform_driver lpm_driver = {
	.probe = lpm_probe,
	.driver = {
		.name = "lpm-levels",
		.owner = THIS_MODULE,
		.suppress_bind_attrs = true,
		.of_match_table = lpm_mtch_tbl,
	},
};

static int __init lpm_levels_module_init(void)
{
	int rc;
	rc = platform_driver_register(&lpm_driver);
	if (rc) {
		pr_info("Error registering %s\n", lpm_driver.driver.name);
		goto fail;
	}

fail:
	return rc;
}
late_initcall(lpm_levels_module_init);

enum msm_pm_l2_scm_flag lpm_cpu_pre_pc_cb(unsigned int cpu)
{
	struct lpm_cluster *cluster = per_cpu(cpu_cluster, cpu);
	enum msm_pm_l2_scm_flag retflag = MSM_SCM_L2_ON;

	/*
	 * No need to acquire the lock if probe isn't completed yet
	 * In the event of the hotplug happening before lpm probe, we want to
	 * flush the cache to make sure that L2 is flushed. In particular, this
	 * could cause incoherencies for a cluster architecture. This wouldn't
	 * affect the idle case as the idle driver wouldn't be registered
	 * before the probe function
	 */
	if (!cluster)
		return MSM_SCM_L2_OFF;

	/*
	 * Assumes L2 only. What/How parameters gets passed into TZ will
	 * determine how this function reports this info back in msm-pm.c
	 */
	raw_spin_lock(&cluster->sync_lock);

	if (!cluster->lpm_dev) {
		retflag = MSM_SCM_L2_OFF;
		goto unlock_and_return;
	}

	if (!cpumask_equal(&cluster->num_children_in_sync,
						&cluster->child_cpus))
		goto unlock_and_return;

	if (cluster->lpm_dev)
		retflag = cluster->lpm_dev->tz_flag;
	/*
	 * The scm_handoff_lock will be release by the secure monitor.
	 * It is used to serialize power-collapses from this point on,
	 * so that both Linux and the secure context have a consistent
	 * view regarding the number of running cpus (cpu_count).
	 *
	 * It must be acquired before releasing the cluster lock.
	 */
unlock_and_return:
	remote_spin_lock_rlock_id(&scm_handoff_lock,
				  REMOTE_SPINLOCK_TID_START + cpu);
	raw_spin_unlock(&cluster->sync_lock);
	return retflag;
}
