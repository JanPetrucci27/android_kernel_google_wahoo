
#include <linux/sched.h>
// #include <linux/sched/cputime.h>

#ifndef _LINUX_SCHED_BORE_H
#define _LINUX_SCHED_BORE_H
#define SCHED_BORE_VERSION "5.7.14"

#ifdef CONFIG_SCHED_BORE
#ifdef CONFIG_SCHED_DEBUG
extern unsigned int sched_bore;
#else
extern unsigned const int sched_bore;
#endif

extern u8   __read_mostly sched_burst_parity_threshold;
extern uint __read_mostly sched_deadline_boost_mask;

extern void update_burst_score(struct sched_entity *se);
extern void update_burst_penalty(struct sched_entity *se);

extern void restart_burst(struct sched_entity *se);
extern void restart_burst_rescale_deadline(struct sched_entity *se);

extern void sched_clone_bore(
	struct task_struct *p, struct task_struct *parent, u64 clone_flags);

extern void init_task_bore(struct task_struct *p);
extern void sched_bore_init(void);

extern void reweight_entity(
	struct cfs_rq *cfs_rq, struct sched_entity *se, unsigned long weight);
#endif // CONFIG_SCHED_BORE
#endif // _LINUX_SCHED_BORE_H
