/*
 *  Burst-Oriented Response Enhancer (BORE) CPU Scheduler
 *  Copyright (C) 2021-2024 Masahito Suzuki <firelzrd@gmail.com>
 */
#include <linux/cpuset.h>
// #include <linux/sched/task.h>
#include <linux/sched/bore.h>
#include "sched.h"

#ifdef CONFIG_SCHED_BORE
#ifdef CONFIG_SCHED_DEBUG
unsigned int sched_bore = 1;
#else
unsigned const int sched_bore = 1;
#endif

u8 __read_mostly sched_burst_parity_threshold 	= 2;
uint __read_mostly sched_deadline_boost_mask    = ENQUEUE_INITIAL | ENQUEUE_WAKEUP;

static const u8   sched_burst_exclude_kthreads = 0;
static const u8   sched_burst_smoothness_long  = 1;
static const u8   sched_burst_smoothness_short = 0;
static const u8   sched_burst_fork_atavistic   = 2;
static const u8   sched_burst_penalty_offset   = 24;
static const uint sched_burst_penalty_scale    = 1280;
static const uint sched_burst_cache_lifetime   = 75000000;
static const uint sched_burst_cache_stop_count = 64;

#define MAX_BURST_PENALTY (39U <<2)

static inline u32 log2plus1_u64_u32f8(u64 v) {
	u32 integral = fls64(v);
	u8  fractional = v << (64 - integral) >> 55;
	return integral << 8 | fractional;
}

static inline u32 calc_burst_penalty(u64 burst_time) {
	u32 greed, tolerance, penalty, scaled_penalty;

	greed = log2plus1_u64_u32f8(burst_time);
	tolerance = sched_burst_penalty_offset << 8;
	penalty = max(0, (s32)(greed - tolerance));
	scaled_penalty = penalty * sched_burst_penalty_scale >> 16;

	return min(MAX_BURST_PENALTY, scaled_penalty);
}

static inline u64 __scale_slice(u64 delta, u8 score)
{return mul_u64_u32_shr(delta, sched_prio_to_wmult[score], 22);}

static inline u64 __unscale_slice(u64 delta, u8 score)
{return mul_u64_u32_shr(delta, sched_prio_to_weight[score], 10);}

static void reweight_task_by_prio(struct task_struct *p, int prio) {
	struct sched_entity *se = &p->se;
	unsigned long weight = scale_load(sched_prio_to_weight[prio]);

	reweight_entity(cfs_rq_of(se), se, weight);
	se->load.inv_weight = sched_prio_to_wmult[prio];
}

static inline u8 effective_prio(struct task_struct *p) {
	u8 prio = p->static_prio - MAX_RT_PRIO;
	if (likely(sched_bore))
		prio += p->se.burst_score;
	return min_t(unsigned int, 39, prio);
}

void update_burst_score(struct sched_entity *se) {
	struct task_struct *p;
	u8 prev_prio, burst_score, new_prio;

	if (!entity_is_task(se)) return;

	p = task_of(se);
	prev_prio = effective_prio(p);

	burst_score = 0;
	if (!((p->flags & PF_KTHREAD) && likely(sched_burst_exclude_kthreads)))
		burst_score = se->burst_penalty >> 2;
	se->burst_score = burst_score;

	new_prio = effective_prio(p);
	if (new_prio != prev_prio)
		reweight_task_by_prio(p, new_prio);
}

void update_burst_penalty(struct sched_entity *se) {
	se->curr_burst_penalty = calc_burst_penalty(se->burst_time);
	se->burst_penalty = max(se->prev_burst_penalty, se->curr_burst_penalty);
	update_burst_score(se);
}

static inline u32 binary_smooth(u32 new, u32 old) {
	int increment = new - old;
	return (0 <= increment)?
		old + ( increment >> (int)sched_burst_smoothness_long):
		old - (-increment >> (int)sched_burst_smoothness_short);
}

static void revolve_burst_penalty(struct sched_entity *se) {
	se->prev_burst_penalty =
		binary_smooth(se->curr_burst_penalty, se->prev_burst_penalty);
	se->burst_time = 0;
	se->curr_burst_penalty = 0;
}

inline void restart_burst(struct sched_entity *se) {
	revolve_burst_penalty(se);
	se->burst_penalty = se->prev_burst_penalty;
	update_burst_score(se);
}

void restart_burst_rescale_deadline(struct sched_entity *se) {
	s64 vscaled, wremain, vremain = se->deadline - se->vruntime;
	struct task_struct *p = task_of(se);
	u8 prev_prio = effective_prio(p);
	u8 new_prio;

	restart_burst(se);
	new_prio = effective_prio(p);
	if (prev_prio > new_prio) {
		wremain = __unscale_slice(abs(vremain), prev_prio);
		vscaled = __scale_slice(wremain, new_prio);
		if (unlikely(vremain < 0))
			vscaled = -vscaled;
		se->deadline = se->vruntime + vscaled;
	}
}

static inline bool task_is_bore_eligible(struct task_struct *p)
{return p && p->sched_class == &fair_sched_class && !p->exit_state;}

#ifdef CONFIG_SCHED_DEBUG
static void reset_task_weights_bore(void) {
	struct task_struct *task;
	struct rq *rq;
	struct rq_flags rf;

	write_lock_irq(&tasklist_lock);
	for_each_process(task) {
		if (!task_is_bore_eligible(task)) continue;
		rq = task_rq(task);
		rq_pin_lock(rq, &rf);
		update_rq_clock(rq);
		reweight_task_by_prio(task, effective_prio(task));
		rq_unpin_lock(rq, &rf);
	}
	write_unlock_irq(&tasklist_lock);
}

int sched_bore_update_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp, loff_t *ppos) {
	int ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	reset_task_weights_bore();

	return 0;
}
#endif

#define for_each_child(p, t) \
	list_for_each_entry(t, &(p)->children, sibling)

static u32 count_children_max2(struct task_struct *p) {
	u32 cnt = 0;
	struct task_struct *child;
	for_each_child(p, child) {if (2 <= ++cnt) break;}
	return cnt;
}

static inline bool burst_cache_expired(struct sched_burst_cache *bc, u64 now)
{return (s64)(bc->timestamp + sched_burst_cache_lifetime - now) < 0;}

static void update_burst_cache(struct sched_burst_cache *bc,
	struct task_struct *p, u32 cnt, u32 sum, u64 now) {
	u8 avg = cnt ? sum / cnt : 0;
	bc->score = max(avg, p->se.burst_penalty);
	bc->count = cnt;
	bc->timestamp = now;
}

static inline void update_child_burst_direct(struct task_struct *p, u64 now) {
	u32 cnt = 0, sum = 0;
	struct task_struct *child;

	for_each_child(p, child) {
		if (!task_is_bore_eligible(child)) continue;
		cnt++;
		sum += child->se.burst_penalty;
	}

	update_burst_cache(&p->se.child_burst, p, cnt, sum, now);
}

static inline u8 inherit_burst_direct(
	struct task_struct *p, u64 now, u64 clone_flags) {
	struct task_struct *parent = p;

	if (clone_flags & CLONE_PARENT)
		parent = parent->real_parent;

	if (burst_cache_expired(&parent->se.child_burst, now))
		update_child_burst_direct(parent, now);

	return parent->se.child_burst.score;
}

static void update_child_burst_topological(
	struct task_struct *p, u64 now, u32 depth, u32 *acnt, u32 *asum) {
	u32 cnt = 0, dcnt = 0, sum = 0;
	struct task_struct *child, *dec;

	for_each_child(p, child) {
		dec = child;
		while ((dcnt = count_children_max2(dec)) == 1)
			dec = list_first_entry(&dec->children, struct task_struct, sibling);

		if (!dcnt || !depth) {
			if (!task_is_bore_eligible(dec)) continue;
			cnt++;
			sum += dec->se.burst_penalty;
			continue;
		}
		if (!burst_cache_expired(&dec->se.child_burst, now)) {
			cnt += dec->se.child_burst.count;
			sum += (u32)dec->se.child_burst.score * dec->se.child_burst.count;
			if (sched_burst_cache_stop_count <= cnt) break;
			continue;
		}
		update_child_burst_topological(dec, now, depth - 1, &cnt, &sum);
	}

	update_burst_cache(&p->se.child_burst, p, cnt, sum, now);
	*acnt += cnt;
	*asum += sum;
}

static inline u8 inherit_burst_topological(
	struct task_struct *p, u64 now, u64 clone_flags) {
	struct task_struct *anc = p;
	struct task_struct *next;
	u32 cnt = 0, sum = 0;
	u32 base_child_cnt = 0;

	if (clone_flags & CLONE_PARENT) {
		anc = anc->real_parent;
		base_child_cnt = 1;
	}

	for (;
		 anc != (next = anc->real_parent) &&
			count_children_max2(anc) <= base_child_cnt;) {
		anc = next;
		base_child_cnt = 1;
	}

	if (burst_cache_expired(&anc->se.child_burst, now))
		update_child_burst_topological(
			anc, now, sched_burst_fork_atavistic - 1, &cnt, &sum);

	return anc->se.child_burst.score;
}

static inline void update_tg_burst(struct task_struct *p, u64 now) {
	struct task_struct *task;
	u32 cnt = 0, sum = 0;

	for_each_thread(p, task) {
		if (!task_is_bore_eligible(task)) continue;
		cnt++;
		sum += task->se.burst_penalty;
	}

	update_burst_cache(&p->se.group_burst, p, cnt, sum, now);
}

static inline u8 inherit_burst_tg(struct task_struct *p, u64 now) {
	struct task_struct *parent = rcu_dereference(p->group_leader);
	if (burst_cache_expired(&parent->se.group_burst, now))
		update_tg_burst(parent, now);

	return parent->se.group_burst.score;
}

void sched_clone_bore(
	struct task_struct *p, struct task_struct *parent, u64 clone_flags) {
	struct sched_entity *se = &p->se;
	u64 now;
	u8 penalty;

	if (!task_is_bore_eligible(p)) return;

	if (clone_flags & CLONE_THREAD) {
		rcu_read_lock();
		now = jiffies_to_nsecs(jiffies);
 		penalty = inherit_burst_tg(parent, now);
		rcu_read_unlock();
	} else {
		read_lock(&tasklist_lock);
		now = jiffies_to_nsecs(jiffies);
		penalty = likely(sched_burst_fork_atavistic) ?
			inherit_burst_topological(parent, now, clone_flags):
			inherit_burst_direct(parent, now, clone_flags);
		read_unlock(&tasklist_lock);
	}

	se = &p->se;
	revolve_burst_penalty(se);
	se->burst_penalty = se->prev_burst_penalty =
		max(se->prev_burst_penalty, penalty);
	se->child_burst.timestamp = 0;
	se->group_burst.timestamp = 0;
}

void init_task_bore(struct task_struct *p) {
	p->se.burst_time = 0;
	p->se.prev_burst_penalty = 0;
	p->se.curr_burst_penalty = 0;
	p->se.burst_penalty = 0;
	p->se.burst_score = 0;
	memset(&p->se.child_burst, 0, sizeof(struct sched_burst_cache));
	memset(&p->se.group_burst, 0, sizeof(struct sched_burst_cache));
}

void __init sched_bore_init(void) {
	printk(KERN_INFO "BORE (Burst-Oriented Response Enhancer) CPU Scheduler modification %s by Masahito Suzuki", SCHED_BORE_VERSION);
    init_task_bore(&init_task);
}
#endif // CONFIG_SCHED_BORE
