/*
 * kernel/sched/debug.c
 *
 * Print the CFS rbtree
 *
 * Copyright(C) 2007, Red Hat, Inc., Ingo Molnar
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <linux/mempolicy.h>
#include <linux/debugfs.h>

#include "sched.h"

static DEFINE_SPINLOCK(sched_debug_lock);

/*
 * This allows printing both to /proc/sched_debug and
 * to the console
 */
#define SEQ_printf(m, x...)			\
 do {						\
	if (m)					\
		seq_printf(m, x);		\
	else					\
		printk(x);			\
 } while (0)

/*
 * Ease the printing of nsec fields:
 */
static long long nsec_high(unsigned long long nsec)
{
	if ((long long)nsec < 0) {
		nsec = -nsec;
		do_div(nsec, 1000000);
		return -nsec;
	}
	do_div(nsec, 1000000);

	return nsec;
}

static unsigned long nsec_low(unsigned long long nsec)
{
	if ((long long)nsec < 0)
		nsec = -nsec;

	return do_div(nsec, 1000000);
}

#define SPLIT_NS(x) nsec_high(x), nsec_low(x)

#ifdef CONFIG_FAIR_GROUP_SCHED
static void print_cfs_group_stats(struct seq_file *m, int cpu, struct task_group *tg)
{
	struct sched_entity *se = tg->se[cpu];

#define P(F) \
	SEQ_printf(m, "  .%-30s: %lld\n", #F, (long long)F)
#define PN(F) \
	SEQ_printf(m, "  .%-30s: %lld.%06ld\n", #F, SPLIT_NS((long long)F))

	if (!se)
		return;

	PN(se->exec_start);
	PN(se->vruntime);
	PN(se->sum_exec_runtime);
#ifdef CONFIG_SCHEDSTATS
	if (schedstat_enabled()) {
		PN(se->statistics.wait_start);
		PN(se->statistics.sleep_start);
		PN(se->statistics.block_start);
		PN(se->statistics.sleep_max);
		PN(se->statistics.block_max);
		PN(se->statistics.exec_max);
		PN(se->statistics.slice_max);
		PN(se->statistics.wait_max);
		PN(se->statistics.wait_sum);
		P(se->statistics.wait_count);
	}
#endif
	P(se->load.weight);
#ifdef CONFIG_SMP
	P(se->avg.load_avg);
	P(se->avg.util_avg);
	P(se->avg.runnable_avg);
#endif
#undef PN
#undef P
}
#endif

#ifdef CONFIG_CGROUP_SCHED
static char group_path[PATH_MAX];

static char *task_group_path(struct task_group *tg)
{
	if (autogroup_path(tg, group_path, PATH_MAX))
		return group_path;

	cgroup_path(tg->css.cgroup, group_path, PATH_MAX);
	return group_path;
}
#endif

static void
print_task(struct seq_file *m, struct rq *rq, struct task_struct *p)
{
	if (task_current(rq, p))
		SEQ_printf(m, "R");
	else
		SEQ_printf(m, " ");

	SEQ_printf(m, "%15s %5d %9Ld.%06ld %c %9Ld.%06ld %c %9Ld.%06ld %9Ld.%06ld %9Ld %5d ",
		p->comm, task_pid_nr(p),
		SPLIT_NS(p->se.vruntime),
		entity_eligible(cfs_rq_of(&p->se), &p->se) ? 'E' : 'N',
		p->se.custom_slice ? 'S' : ' ',
		SPLIT_NS(p->se.slice),
		SPLIT_NS(p->se.sum_exec_runtime),
		(long long)(p->nvcsw + p->nivcsw),
		p->prio);

	SEQ_printf(m, "%9Ld.%06ld %9Ld.%06ld %9Ld.%06ld",
		0LL, 0L,
		SPLIT_NS(schedstat_val_or_zero(p->se.statistics.wait_sum)),
		SPLIT_NS(p->se.sum_exec_runtime),
		SPLIT_NS(schedstat_val_or_zero(p->se.statistics.sum_sleep_runtime)));

#ifdef CONFIG_NUMA_BALANCING
	SEQ_printf(m, " %d %d", task_node(p), task_numa_group_id(p));
#endif
#ifdef CONFIG_CGROUP_SCHED
	SEQ_printf(m, " %s", task_group_path(task_group(p)));
#endif

	SEQ_printf(m, "\n");
}

static void print_rq(struct seq_file *m, struct rq *rq, int rq_cpu)
{
	struct task_struct *g, *p;

	SEQ_printf(m,
	"\nrunnable tasks:\n"
	"            task   PID         tree-key  switches  prio"
	"     wait-time             sum-exec        sum-sleep\n"
	"------------------------------------------------------"
	"----------------------------------------------------\n");

	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (task_cpu(p) != rq_cpu)
			continue;

		print_task(m, rq, p);
	}
	rcu_read_unlock();
}

void print_cfs_rq(struct seq_file *m, int cpu, struct cfs_rq *cfs_rq)
{
	s64 left_vruntime = -1, min_vruntime, right_vruntime = -1, left_deadline = -1, spread;
	struct sched_entity *last, *first, *root;
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

#ifdef CONFIG_FAIR_GROUP_SCHED
	SEQ_printf(m, "\ncfs_rq[%d]:%s\n", cpu, task_group_path(cfs_rq->tg));
#else
	SEQ_printf(m, "\ncfs_rq[%d]:\n", cpu);
#endif

	raw_spin_lock_irqsave(&rq->lock, flags);
	root = __pick_root_entity(cfs_rq);
	if (root)
		left_vruntime = root->min_vruntime;
	first = __pick_first_entity(cfs_rq);
	if (first)
		left_deadline = first->deadline;
	last = __pick_last_entity(cfs_rq);
	if (last)
		right_vruntime = last->vruntime;
	min_vruntime = cfs_rq->min_vruntime;
	raw_spin_unlock_irqrestore(&rq->lock, flags);

	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", "left_deadline",
			SPLIT_NS(left_deadline));
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", "left_vruntime",
			SPLIT_NS(left_vruntime));
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", "min_vruntime",
			SPLIT_NS(min_vruntime));
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", "avg_vruntime",
			SPLIT_NS(avg_vruntime(cfs_rq)));
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", "right_vruntime",
			SPLIT_NS(right_vruntime));
	spread = right_vruntime - left_vruntime;
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", "spread", SPLIT_NS(spread));
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", "spread", SPLIT_NS(spread));
	SEQ_printf(m, "  .%-30s: %d\n", "nr_running", cfs_rq->nr_running);
	SEQ_printf(m, "  .%-30s: %d\n", "h_nr_running", cfs_rq->h_nr_running);
	SEQ_printf(m, "  .%-30s: %d\n", "h_nr_delayed", cfs_rq->h_nr_delayed);
	SEQ_printf(m, "  .%-30s: %d\n", "idle_nr_running",
			cfs_rq->idle_nr_running);
	SEQ_printf(m, "  .%-30s: %d\n", "idle_h_nr_running",
			cfs_rq->idle_h_nr_running);
	SEQ_printf(m, "  .%-30s: %ld\n", "load", cfs_rq->load.weight);
#ifdef CONFIG_SMP
	SEQ_printf(m, "  .%-30s: %lu\n", "load_avg",
			cfs_rq->avg.load_avg);
	SEQ_printf(m, "  .%-30s: %lu\n", "runnable_avg",
			cfs_rq->avg.runnable_avg);
	SEQ_printf(m, "  .%-30s: %lu\n", "util_avg",
			cfs_rq->avg.util_avg);
	SEQ_printf(m, "  .%-30s: %u\n", "util_est",
			cfs_rq->avg.util_est);
	SEQ_printf(m, "  .%-30s: %ld\n", "removed.load_avg",
			cfs_rq->removed.load_avg);
	SEQ_printf(m, "  .%-30s: %ld\n", "removed.util_avg",
			cfs_rq->removed.util_avg);
	SEQ_printf(m, "  .%-30s: %ld\n", "removed.runnable_avg",
			cfs_rq->removed.runnable_avg);
#ifdef CONFIG_FAIR_GROUP_SCHED
	SEQ_printf(m, "  .%-30s: %lu\n", "tg_load_avg_contrib",
			cfs_rq->tg_load_avg_contrib);
	SEQ_printf(m, "  .%-30s: %ld\n", "tg_load_avg",
			atomic_long_read(&cfs_rq->tg->load_avg));
#endif
#endif
#ifdef CONFIG_CFS_BANDWIDTH
	SEQ_printf(m, "  .%-30s: %d\n", "throttled",
			cfs_rq->throttled);
	SEQ_printf(m, "  .%-30s: %d\n", "throttle_count",
			cfs_rq->throttle_count);
#endif

#ifdef CONFIG_FAIR_GROUP_SCHED
	print_cfs_group_stats(m, cpu, cfs_rq->tg);
#endif
}

void print_rt_rq(struct seq_file *m, int cpu, struct rt_rq *rt_rq)
{
#ifdef CONFIG_RT_GROUP_SCHED
	SEQ_printf(m, "\nrt_rq[%d]:%s\n", cpu, task_group_path(rt_rq->tg));
#else
	SEQ_printf(m, "\nrt_rq[%d]:\n", cpu);
#endif

#define P(x) \
	SEQ_printf(m, "  .%-30s: %Ld\n", #x, (long long)(rt_rq->x))
#define PN(x) \
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", #x, SPLIT_NS(rt_rq->x))

	P(rt_nr_running);

#ifdef CONFIG_RT_GROUP_SCHED
	P(rt_throttled);
	PN(rt_time);
	PN(rt_runtime);
#endif

#undef PN
#undef P
}

void print_dl_rq(struct seq_file *m, int cpu, struct dl_rq *dl_rq)
{
	SEQ_printf(m, "\ndl_rq[%d]:\n", cpu);
	SEQ_printf(m, "  .%-30s: %ld\n", "dl_nr_running", dl_rq->dl_nr_running);
}

extern __read_mostly int sched_clock_running;

static void print_cpu(struct seq_file *m, int cpu)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long flags;

#ifdef CONFIG_X86
	{
		unsigned int freq = cpu_khz ? : 1;

		SEQ_printf(m, "cpu#%d, %u.%03u MHz\n",
			   cpu, freq / 1000, (freq % 1000));
	}
#else
	SEQ_printf(m, "cpu#%d\n", cpu);
#endif

#define P(x)								\
do {									\
	if (sizeof(rq->x) == 4)						\
		SEQ_printf(m, "  .%-30s: %ld\n", #x, (long)(rq->x));	\
	else								\
		SEQ_printf(m, "  .%-30s: %Ld\n", #x, (long long)(rq->x));\
} while (0)

#define PN(x) \
	SEQ_printf(m, "  .%-30s: %Ld.%06ld\n", #x, SPLIT_NS(rq->x))

	P(nr_running);
	P(nr_switches);
	P(nr_uninterruptible);
	PN(next_balance);
	SEQ_printf(m, "  .%-30s: %ld\n", "curr->pid", (long)(task_pid_nr(rq->curr)));
	PN(clock);
	PN(clock_task);
#undef P
#undef PN

#ifdef CONFIG_SCHEDSTATS
#define P(n) SEQ_printf(m, "  .%-30s: %d\n", #n, rq->n);
#define P64(n) SEQ_printf(m, "  .%-30s: %Ld\n", #n, rq->n);

#ifdef CONFIG_SMP
	P64(avg_idle);
	P64(max_idle_balance_cost);
#endif

	if (schedstat_enabled()) {
		P(yld_count);
		P(sched_count);
		P(sched_goidle);
		P(ttwu_count);
		P(ttwu_local);
	}

#undef P
#undef P64
#endif
	spin_lock_irqsave(&sched_debug_lock, flags);
	print_cfs_stats(m, cpu);
	print_rt_stats(m, cpu);
	print_dl_stats(m, cpu);

	print_rq(m, rq, cpu);
	spin_unlock_irqrestore(&sched_debug_lock, flags);
	SEQ_printf(m, "\n");
}

static const char *sched_tunable_scaling_names[] = {
	"none",
	"logaritmic",
	"linear"
};

static void sched_debug_header(struct seq_file *m)
{
	u64 ktime, sched_clk, cpu_clk;
	unsigned long flags;

	local_irq_save(flags);
	ktime = ktime_to_ns(ktime_get());
	sched_clk = sched_clock();
	cpu_clk = local_clock();
	local_irq_restore(flags);

	SEQ_printf(m, "Sched Debug Version: v0.11, %s %.*s\n",
		init_utsname()->release,
		(int)strcspn(init_utsname()->version, " "),
		init_utsname()->version);

#define P(x) \
	SEQ_printf(m, "%-40s: %Ld\n", #x, (long long)(x))
#define PN(x) \
	SEQ_printf(m, "%-40s: %Ld.%06ld\n", #x, SPLIT_NS(x))
	PN(ktime);
	PN(sched_clk);
	PN(cpu_clk);
	P(jiffies);
#ifdef CONFIG_HAVE_UNSTABLE_SCHED_CLOCK
	P(sched_clock_stable());
#endif
#undef PN
#undef P

	SEQ_printf(m, "\n");
	SEQ_printf(m, "sysctl_sched\n");

#define P(x) \
	SEQ_printf(m, "  .%-40s: %Ld\n", #x, (long long)(x))
#define PN(x) \
	SEQ_printf(m, "  .%-40s: %Ld.%06ld\n", #x, SPLIT_NS(x))
	PN(sysctl_sched_base_slice);
#undef PN
#undef P

	SEQ_printf(m, "  .%-40s: %d (%s)\n",
		"sysctl_sched_tunable_scaling",
		sysctl_sched_tunable_scaling,
		sched_tunable_scaling_names[sysctl_sched_tunable_scaling]);
	SEQ_printf(m, "\n");
}

static int sched_debug_show(struct seq_file *m, void *v)
{
	int cpu = (unsigned long)(v - 2);

	if (cpu != -1)
		print_cpu(m, cpu);
	else
		sched_debug_header(m);

	return 0;
}

void sysrq_sched_debug_show(void)
{
	int cpu;

	sched_debug_header(NULL);
	for_each_online_cpu(cpu) {
		/*
		 * Need to reset softlockup watchdogs on all CPUs, because
		 * another CPU might be blocked waiting for us to process
		 * an IPI or stop_machine.
		 */
		// touch_nmi_watchdog();
		// touch_all_softlockup_watchdogs();
		print_cpu(NULL, cpu);
	}
}

/*
 * This itererator needs some explanation.
 * It returns 1 for the header position.
 * This means 2 is cpu 0.
 * In a hotplugged system some cpus, including cpu 0, may be missing so we have
 * to use cpumask_* to iterate over the cpus.
 */
static void *sched_debug_start(struct seq_file *file, loff_t *offset)
{
	unsigned long n = *offset;

	if (n == 0)
		return (void *) 1;

	n--;

	if (n > 0)
		n = cpumask_next(n - 1, cpu_online_mask);
	else
		n = cpumask_first(cpu_online_mask);

	*offset = n + 1;

	if (n < nr_cpu_ids)
		return (void *)(unsigned long)(n + 2);
	return NULL;
}

static void *sched_debug_next(struct seq_file *file, void *data, loff_t *offset)
{
	(*offset)++;
	return sched_debug_start(file, offset);
}

static void sched_debug_stop(struct seq_file *file, void *data)
{
}

static const struct seq_operations sched_debug_sops = {
	.start = sched_debug_start,
	.next = sched_debug_next,
	.stop = sched_debug_stop,
	.show = sched_debug_show,
};

static int sched_debug_release(struct inode *inode, struct file *file)
{
	seq_release(inode, file);

	return 0;
}

static int sched_debug_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	ret = seq_open(filp, &sched_debug_sops);

	return ret;
}

static const struct file_operations sched_debug_fops = {
	.open		= sched_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= sched_debug_release,
};

enum dl_param {
	DL_RUNTIME = 0,
	DL_PERIOD,
};
static unsigned long fair_server_period_max = (1UL << 22) * NSEC_PER_USEC; /* ~4 seconds */
static unsigned long fair_server_period_min = (100) * NSEC_PER_USEC;     /* 100 us */
static ssize_t sched_fair_server_write(struct file *filp, const char __user *ubuf,
									   size_t cnt, loff_t *ppos, enum dl_param param)
{
	long cpu = (long) ((struct seq_file *) filp->private_data)->private;
	struct rq *rq = cpu_rq(cpu);
	u64 runtime, period;
	size_t err;
	int retval;
	u64 value;
	err = kstrtoull_from_user(ubuf, cnt, 10, &value);
	if (err)
		return err;
	scoped_guard (rq_lock_irqsave, rq) {
		runtime  = rq->fair_server.dl_runtime;
		period = rq->fair_server.dl_period;
		switch (param) {
			case DL_RUNTIME:
				if (runtime == value)
					break;
			runtime = value;
			break;
			case DL_PERIOD:
				if (value == period)
					break;
			period = value;
			break;
		}
		if (runtime > period ||
			period > fair_server_period_max ||
			period < fair_server_period_min) {
			return  -EINVAL;
			}
			if (rq->cfs.h_nr_running) {
				update_rq_clock(rq);
				dl_server_stop(&rq->fair_server);
			}
			retval = dl_server_apply_params(&rq->fair_server, runtime, period, 0);
		if (retval)
			cnt = retval;
		if (!runtime)
			printk_deferred("Fair server disabled in CPU %d, system may crash due to starvation.\n",
							cpu_of(rq));
			if (rq->cfs.h_nr_running)
				dl_server_start(&rq->fair_server);
	}
	*ppos += cnt;
	return cnt;
}
static size_t sched_fair_server_show(struct seq_file *m, void *v, enum dl_param param)
{
	unsigned long cpu = (unsigned long) m->private;
	struct rq *rq = cpu_rq(cpu);
	u64 value;
	switch (param) {
		case DL_RUNTIME:
			value = rq->fair_server.dl_runtime;
			break;
		case DL_PERIOD:
			value = rq->fair_server.dl_period;
			break;
	}
	seq_printf(m, "%llu\n", value);
	return 0;
}
static ssize_t
sched_fair_server_runtime_write(struct file *filp, const char __user *ubuf,
								size_t cnt, loff_t *ppos)
{
	return sched_fair_server_write(filp, ubuf, cnt, ppos, DL_RUNTIME);
}
static int sched_fair_server_runtime_show(struct seq_file *m, void *v)
{
	return sched_fair_server_show(m, v, DL_RUNTIME);
}
static int sched_fair_server_runtime_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sched_fair_server_runtime_show, inode->i_private);
}
static const struct file_operations fair_server_runtime_fops = {
	.open		= sched_fair_server_runtime_open,
	.write		= sched_fair_server_runtime_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static ssize_t
sched_fair_server_period_write(struct file *filp, const char __user *ubuf,
							   size_t cnt, loff_t *ppos)
{
	return sched_fair_server_write(filp, ubuf, cnt, ppos, DL_PERIOD);
}
static int sched_fair_server_period_show(struct seq_file *m, void *v)
{
	return sched_fair_server_show(m, v, DL_PERIOD);
}
static int sched_fair_server_period_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sched_fair_server_period_show, inode->i_private);
}
static const struct file_operations fair_server_period_fops = {
	.open		= sched_fair_server_period_open,
	.write		= sched_fair_server_period_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

__read_mostly bool sched_debug_enabled;

static void debugfs_fair_server_init(void)
{
	struct dentry *d_fair;
	unsigned long cpu;
	d_fair = debugfs_create_dir("fair_server", debugfs_sched);
	if (!d_fair)
		return;
	for_each_possible_cpu(cpu) {
		struct dentry *d_cpu;
		char buf[32];
		snprintf(buf, sizeof(buf), "cpu%lu", cpu);
		d_cpu = debugfs_create_dir(buf, d_fair);
		debugfs_create_file("runtime", 0644, d_cpu, (void *) cpu, &fair_server_runtime_fops);
		debugfs_create_file("period", 0644, d_cpu, (void *) cpu, &fair_server_period_fops);
	}

	debugfs_fair_server_init();
}

static int __init init_sched_debug_procfs(void)
{
	struct proc_dir_entry *pe;
	struct dentry *debug2;

	pe = proc_create("sched_debug", 0444, NULL, &sched_debug_fops);
	if (!pe)
		return -ENOMEM;

	debug2 = debugfs_create_bool("sched_debug2", 0444, NULL, &sched_debug_enabled);
	if (!debug2)
		return -ENOMEM;

	return 0;
}

__initcall(init_sched_debug_procfs);

#define __P(F) \
	SEQ_printf(m, "%-45s:%21Ld\n", #F, (long long)F)
#define P(F) \
	SEQ_printf(m, "%-45s:%21Ld\n", #F, (long long)p->F)
#define __PN(F) \
	SEQ_printf(m, "%-45s:%14Ld.%06ld\n", #F, SPLIT_NS((long long)F))
#define PN(F) \
	SEQ_printf(m, "%-45s:%14Ld.%06ld\n", #F, SPLIT_NS((long long)p->F))


#ifdef CONFIG_NUMA_BALANCING
void print_numa_stats(struct seq_file *m, int node, unsigned long tsf,
		unsigned long tpf, unsigned long gsf, unsigned long gpf)
{
	SEQ_printf(m, "numa_faults node=%d ", node);
	SEQ_printf(m, "task_private=%lu task_shared=%lu ", tsf, tpf);
	SEQ_printf(m, "group_private=%lu group_shared=%lu\n", gsf, gpf);
}
#endif


static void sched_show_numa(struct task_struct *p, struct seq_file *m)
{
#ifdef CONFIG_NUMA_BALANCING

	if (p->mm)
		P(mm->numa_scan_seq);

	P(numa_pages_migrated);
	P(numa_preferred_nid);
	P(total_numa_faults);
	SEQ_printf(m, "current_node=%d, numa_group_id=%d\n",
			task_node(p), task_numa_group_id(p));
	show_numa_stats(p, m);
#endif
}

void proc_sched_show_task(struct task_struct *p, struct seq_file *m)
{
	unsigned long nr_switches;

	SEQ_printf(m, "%s (%d, #threads: %d)\n", p->comm, task_pid_nr(p),
						get_nr_threads(p));
	SEQ_printf(m,
		"---------------------------------------------------------"
		"----------\n");
#define __P(F) \
	SEQ_printf(m, "%-45s:%21Ld\n", #F, (long long)F)
#define P(F) \
	SEQ_printf(m, "%-45s:%21Ld\n", #F, (long long)p->F)
#define __PN(F) \
	SEQ_printf(m, "%-45s:%14Ld.%06ld\n", #F, SPLIT_NS((long long)F))
#define PN(F) \
	SEQ_printf(m, "%-45s:%14Ld.%06ld\n", #F, SPLIT_NS((long long)p->F))

	PN(se.exec_start);
	PN(se.vruntime);
	PN(se.sum_exec_runtime);

	nr_switches = p->nvcsw + p->nivcsw;

#ifdef CONFIG_SCHEDSTATS
	P(se.nr_migrations);

	if (schedstat_enabled()) {
		u64 avg_atom, avg_per_cpu;

		PN(se.statistics.sum_sleep_runtime);
		PN(se.statistics.wait_start);
		PN(se.statistics.sleep_start);
		PN(se.statistics.block_start);
		PN(se.statistics.sleep_max);
		PN(se.statistics.block_max);
		PN(se.statistics.exec_max);
		PN(se.statistics.slice_max);
		PN(se.statistics.wait_max);
		PN(se.statistics.wait_sum);
		P(se.statistics.wait_count);
		PN(se.statistics.iowait_sum);
		P(se.statistics.iowait_count);
		P(se.statistics.nr_migrations_cold);
		P(se.statistics.nr_failed_migrations_affine);
		P(se.statistics.nr_failed_migrations_running);
		P(se.statistics.nr_failed_migrations_hot);
		P(se.statistics.nr_forced_migrations);
		P(se.statistics.nr_wakeups);
		P(se.statistics.nr_wakeups_sync);
		P(se.statistics.nr_wakeups_migrate);
		P(se.statistics.nr_wakeups_local);
		P(se.statistics.nr_wakeups_remote);
		P(se.statistics.nr_wakeups_affine);
		P(se.statistics.nr_wakeups_affine_attempts);
		P(se.statistics.nr_wakeups_passive);
		P(se.statistics.nr_wakeups_idle);
		/* eas */
		/* select_idle_sibling() */
		P(se.statistics.nr_wakeups_sis_attempts);
		P(se.statistics.nr_wakeups_sis_idle);
		P(se.statistics.nr_wakeups_sis_cache_affine);
		P(se.statistics.nr_wakeups_sis_suff_cap);
		P(se.statistics.nr_wakeups_sis_idle_cpu);
		P(se.statistics.nr_wakeups_sis_count);
		/* select_energy_cpu_brute() */
		P(se.statistics.nr_wakeups_secb_attempts);
		P(se.statistics.nr_wakeups_secb_idle_bt);
		P(se.statistics.nr_wakeups_secb_insuff_cap);
		P(se.statistics.nr_wakeups_secb_no_nrg_sav);
		P(se.statistics.nr_wakeups_secb_nrg_sav);
		P(se.statistics.nr_wakeups_secb_count);
		/* find_best_target() */
		P(se.statistics.nr_wakeups_fbt_attempts);
		P(se.statistics.nr_wakeups_fbt_no_cpu);
		P(se.statistics.nr_wakeups_fbt_no_sd);
		P(se.statistics.nr_wakeups_fbt_pref_idle);
		P(se.statistics.nr_wakeups_fbt_count);
		/* cas */
		/* select_task_rq_fair() */
		P(se.statistics.nr_wakeups_cas_attempts);
		P(se.statistics.nr_wakeups_cas_count);

		avg_atom = p->se.sum_exec_runtime;
		if (nr_switches)
			avg_atom = div64_ul(avg_atom, nr_switches);
		else
			avg_atom = -1LL;

		avg_per_cpu = p->se.sum_exec_runtime;
		if (p->se.nr_migrations) {
			avg_per_cpu = div64_u64(avg_per_cpu,
						p->se.nr_migrations);
		} else {
			avg_per_cpu = -1LL;
		}

		__PN(avg_atom);
		__PN(avg_per_cpu);
	}
#endif
	__P(nr_switches);
	SEQ_printf(m, "%-45s:%21Ld\n",
		   "nr_voluntary_switches", (long long)p->nvcsw);
	SEQ_printf(m, "%-45s:%21Ld\n",
		   "nr_involuntary_switches", (long long)p->nivcsw);

	P(se.load.weight);
#ifdef CONFIG_SMP
#ifdef CONFIG_SCHED_BORE
	P(se.burst_score);
#endif // CONFIG_SCHED_BORE
	P(se.avg.load_sum);
	P(se.avg.runnable_sum);
	P(se.avg.util_sum);
	P(se.avg.load_avg);
	P(se.avg.runnable_avg);
	P(se.avg.util_avg);
	P(se.avg.last_update_time);
	PM(se.avg.util_est, ~UTIL_AVG_UNCHANGED);
#endif
	P(policy);
	P(prio);
#undef PN
#undef __PN
#undef P
#undef __P

	{
		unsigned int this_cpu = raw_smp_processor_id();
		u64 t0, t1;

		t0 = cpu_clock(this_cpu);
		t1 = cpu_clock(this_cpu);
		SEQ_printf(m, "%-45s:%21Ld\n",
			   "clock-delta", (long long)(t1-t0));
	}

	sched_show_numa(p, m);
}

void proc_sched_set_task(struct task_struct *p)
{
#ifdef CONFIG_SCHEDSTATS
	memset(&p->se.statistics, 0, sizeof(p->se.statistics));
#endif
}
