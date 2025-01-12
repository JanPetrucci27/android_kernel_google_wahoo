/*
 * Power Management Quality of Service (PM QoS) support base.
 *
 * Copyright (C) 2020 Intel Corporation
 *
 * Authors:
 *	Mark Gross <mgross@linux.intel.com>
 *	Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * Provided here is an interface for specifying PM QoS dependencies.  It allows
 * entities depending on QoS constraints to register their requests which are
 * aggregated as appropriate to produce effective constraints (target values)
 * that can be monitored by entities needing to respect them, either by polling
 * or through a built-in notification mechanism.
 *
 * There are lists of pm_qos_objects each one wrapping requests, notifiers
 *
 * In addition to the basic functionality, more specific interfaces for managing
 * global CPU latency QoS requests and frequency QoS requests are provided.
 *
 * Mark Gross <mgross@linux.intel.com>
 */

/*#define DEBUG*/

#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>

#include <linux/uaccess.h>
#include <linux/export.h>
#include <trace/events/power.h>

#define CPUMASK_ALL (BIT(NR_CPUS) - 1)

/*
 * locking rule: all changes to constraints or notifiers lists
 * or pm_qos_object list and pm_qos_objects need to happen with pm_qos_lock
 * held, taken with _irqsave.  One lock to rule them all
 */
static DEFINE_RAW_SPINLOCK(pm_qos_lock);

/**
 * pm_qos_read_value - Return the current effective constraint value.
 * @c: List of PM QoS constraint requests.
 */
s32 pm_qos_read_value(struct pm_qos_constraints *c)
{
	return READ_ONCE(c->target_value);
}

static int pm_qos_get_value(struct pm_qos_constraints *c)
{
	if (plist_head_empty(&c->list))
		return c->no_constraint_value;

	switch (c->type) {
	case PM_QOS_MIN:
		return plist_first(&c->list)->prio;

	case PM_QOS_MAX:
		return plist_last(&c->list)->prio;

	default:
		WARN(1, "Unknown PM QoS type in %s\n", __func__);
		return PM_QOS_DEFAULT_VALUE;
	}
}

static void pm_qos_set_value(struct pm_qos_constraints *c, s32 value)
{
	WRITE_ONCE(c->target_value, value);
}

static inline void pm_qos_set_value_for_cpus(struct pm_qos_request *new_req,
					    struct pm_qos_constraints *c,
					    unsigned long *cpus,
					    unsigned long new_cpus,
					    enum pm_qos_req_action new_action)
{
	struct pm_qos_request *req;
	unsigned long new_req_cpus;
	int cpu;
	
	if (new_cpus) {
		/* cpus_affine changed, so the old CPUs need to be refreshed */
		new_req_cpus = new_req->cpus_affine | new_cpus;
		new_req->cpus_affine = new_cpus;
	} else {
		new_req_cpus = new_req->cpus_affine;
	}

	if (new_action != PM_QOS_REMOVE_REQ) {
		bool changed = false;

		for_each_cpu(cpu, to_cpumask(&new_req_cpus)) {
			if (c->target_per_cpu[cpu] != new_req->node.prio) {
				changed = true;
				break;
			}
		}

		if (!changed)
			return;
	}

	plist_for_each_entry(req, &c->list, node) {
		unsigned long affected_cpus;

		affected_cpus = req->cpus_affine & new_req_cpus;
		if (!affected_cpus)
			continue;

		for_each_cpu(cpu, to_cpumask(&affected_cpus)) {
			if (c->target_per_cpu[cpu] != req->node.prio) {
				c->target_per_cpu[cpu] = req->node.prio;
				if (cpus)
					*cpus |= BIT(cpu);
			}
		}
		
		if (!(new_req_cpus &= ~affected_cpus))
			return;
	}

	for_each_cpu(cpu, to_cpumask(&new_req_cpus)) {
		if (c->target_per_cpu[cpu] != PM_QOS_CPU_LATENCY_DEFAULT_VALUE) {
			c->target_per_cpu[cpu] = PM_QOS_CPU_LATENCY_DEFAULT_VALUE;
			if (cpus)
				*cpus |= BIT(cpu);
		}
	}
}

static int __always_inline pm_qos_update_target_cpus(struct pm_qos_constraints *c,
				     struct pm_qos_request *req,
				     enum pm_qos_req_action action, int value,
				     unsigned long new_cpus)
{
	int prev_value, curr_value, new_value;
	struct plist_node *node = &req->node;
	unsigned long cpus = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&pm_qos_lock, flags);

	prev_value = pm_qos_get_value(c);
	if (value == PM_QOS_DEFAULT_VALUE)
		new_value = c->default_value;
	else
		new_value = value;

	switch (action) {
	case PM_QOS_REMOVE_REQ:
		plist_del(node, &c->list);
		break;
	case PM_QOS_UPDATE_REQ:
		/*
		 * To change the list, atomically remove, reinit with new value
		 * and add, then see if the aggregate has changed.
		 */
		plist_del(node, &c->list);
	case PM_QOS_ADD_REQ:
		plist_node_init(node, new_value);
		plist_add(node, &c->list);
		break;
	default:
		/* no action */
		;
	}

	curr_value = pm_qos_get_value(c);
	pm_qos_set_value(c, curr_value);
	pm_qos_set_value_for_cpus(req, c, &cpus, new_cpus, action);

	raw_spin_unlock_irqrestore(&pm_qos_lock, flags);

	/*
	 * if cpu mask bits are set, call the notifier call chain
	 * to update the new qos restriction for the cores
	 */
	if (!cpus && prev_value == curr_value)
		return 0;

	if (c->notifiers)
		blocking_notifier_call_chain(c->notifiers,
						     (unsigned long)curr_value,
						     &cpus);

	trace_pm_qos_update_target(action, prev_value, curr_value);
	
	return 1;
}

/**
 * pm_qos_update_target - Update a list of PM QoS constraint requests.
 * @c: List of PM QoS requests.
 * @node: Target list entry.
 * @action: Action to carry out (add, update or remove).
 * @value: New request value for the target list entry.
 *
 * Update the given list of PM QoS constraint requests, @c, by carrying an
 * @action involving the @node list entry and @value on it.
 *
 * The recognized values of @action are PM_QOS_ADD_REQ (store @value in @node
 * and add it to the list), PM_QOS_UPDATE_REQ (remove @node from the list, store
 * @value in it and add it to the list again), and PM_QOS_REMOVE_REQ (remove
 * @node from the list, ignore @value).
 *
 * Return: 1 if the aggregate constraint value has changed, 0  otherwise.
 */
int pm_qos_update_target(struct pm_qos_constraints *c, struct pm_qos_request *req,
			 enum pm_qos_req_action action, int value)
{
	return pm_qos_update_target_cpus(c, req, action, value, 0);
}

/**
 * pm_qos_flags_remove_req - Remove device PM QoS flags request.
 * @pqf: Device PM QoS flags set to remove the request from.
 * @req: Request to remove from the set.
 */
static void pm_qos_flags_remove_req(struct pm_qos_flags *pqf,
				    struct pm_qos_flags_request *req)
{
	s32 val = 0;

	list_del(&req->node);
	list_for_each_entry(req, &pqf->list, node)
		val |= req->flags;

	pqf->effective_flags = val;
}

/**
 * pm_qos_update_flags - Update a set of PM QoS flags.
 * @pqf: Set of PM QoS flags to update.
 * @req: Request to add to the set, to modify, or to remove from the set.
 * @action: Action to take on the set.
 * @val: Value of the request to add or modify.
 *
 * Return: 1 if the aggregate constraint value has changed, 0 otherwise.
 */
bool pm_qos_update_flags(struct pm_qos_flags *pqf,
			 struct pm_qos_flags_request *req,
			 enum pm_qos_req_action action, s32 val)
{
	s32 prev_value, curr_value;

	raw_spin_lock(&pm_qos_lock);

	prev_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	switch (action) {
	case PM_QOS_REMOVE_REQ:
		pm_qos_flags_remove_req(pqf, req);
		break;
	case PM_QOS_UPDATE_REQ:
		pm_qos_flags_remove_req(pqf, req);
	case PM_QOS_ADD_REQ:
		req->flags = val;
		INIT_LIST_HEAD(&req->node);
		list_add_tail(&req->node, &pqf->list);
		pqf->effective_flags |= val;
		break;
	default:
		/* no action */
		;
	}

	curr_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	raw_spin_unlock(&pm_qos_lock);

	trace_pm_qos_update_flags(action, prev_value, curr_value);

	return prev_value != curr_value;
}

static struct pm_qos_constraints cpu_latency_constraints = {
	.list = PLIST_HEAD_INIT(cpu_latency_constraints.list),
	.target_value = PM_QOS_CPU_LATENCY_DEFAULT_VALUE,
	.target_per_cpu = { [0 ... (NR_CPUS - 1)] =
		PM_QOS_CPU_LATENCY_DEFAULT_VALUE },
		.default_value = PM_QOS_CPU_LATENCY_DEFAULT_VALUE,
		.no_constraint_value = PM_QOS_CPU_LATENCY_DEFAULT_VALUE,
		.type = PM_QOS_MIN,
};

static inline bool cpu_latency_qos_value_invalid(s32 value)
{
	return value < 0 && value != PM_QOS_DEFAULT_VALUE;
}

/**
 * cpu_latency_qos_limit - Return current system-wide CPU latency QoS limit.
 */
s32 cpu_latency_qos_limit(void)
{
	return pm_qos_read_value(&cpu_latency_constraints);
}

int cpu_latency_qos_request_for_cpu(int cpu)
{
	return cpu_latency_constraints.target_per_cpu[cpu];
}
EXPORT_SYMBOL(cpu_latency_qos_request_for_cpu);

/**
 * cpu_latency_qos_request_active - Check the given PM QoS request.
 * @req: PM QoS request to check.
 *
 * Return: 'true' if @req has been added to the CPU latency QoS list, 'false'
 * otherwise.
 */
bool cpu_latency_qos_request_active(struct pm_qos_request *req)
{
	return req->qos == &cpu_latency_constraints;
}
EXPORT_SYMBOL_GPL(cpu_latency_qos_request_active);

#ifdef CONFIG_SMP
static void pm_qos_irq_release(struct kref *ref)
{
	struct irq_affinity_notify *notify = container_of(ref,
					struct irq_affinity_notify, kref);
	struct pm_qos_request *req = container_of(notify,
					struct pm_qos_request, irq_notify);
	struct pm_qos_constraints *c = &cpu_latency_constraints;

	pm_qos_update_target_cpus(c, req, PM_QOS_UPDATE_REQ,
				  c->default_value, CPUMASK_ALL);
}

static void pm_qos_irq_notify(struct irq_affinity_notify *notify,
		const cpumask_t *mask)
{
	struct pm_qos_request *req = container_of(notify,
					struct pm_qos_request, irq_notify);
	struct pm_qos_constraints *c = &cpu_latency_constraints;

	pm_qos_update_target_cpus(c, req, PM_QOS_UPDATE_REQ,
				  req->node.prio, *cpumask_bits(mask));
}
#endif

static void cpu_latency_qos_apply(struct pm_qos_request *req,
								   enum pm_qos_req_action action, s32 value)
{
	int ret = pm_qos_update_target(req->qos, req, action, value);
	if (ret > 0)
		wake_up_all_idle_cpus();
}

/**
 * cpu_latency_qos_add_request - Add new CPU latency QoS request.
 * @req: Pointer to a preallocated handle.
 * @value: Requested constraint value.
 *
 * Use @value to initialize the request handle pointed to by @req, insert it as
 * a new entry to the CPU latency QoS list and recompute the effective QoS
 * constraint for that list.
 *
 * Callers need to save the handle for later use in updates and removal of the
 * QoS request represented by it.
 */
void cpu_latency_qos_add_request(struct pm_qos_request *req, s32 value)
{
	if (!req || cpu_latency_qos_value_invalid(value))
		return;

	if (cpu_latency_qos_request_active(req)) {
		WARN(1, KERN_ERR "%s called for already added request\n", __func__);
		return;
	}

	switch (req->type) {
	case PM_QOS_REQ_AFFINE_CORES:
		if (!req->cpus_affine) {
			req->cpus_affine = CPUMASK_ALL;
			req->type = PM_QOS_REQ_ALL_CORES;
			WARN(1, KERN_ERR "Affine cores not set for request with affinity flag\n");
		}
		break;
#ifdef CONFIG_SMP
	case PM_QOS_REQ_AFFINE_IRQ:
		if (irq_can_set_affinity(req->irq)) {
			struct irq_desc *desc = irq_to_desc(req->irq);
			struct cpumask *mask;

			if (!desc)
				return;
			mask = desc->irq_data.common->affinity;

			/* Get the current affinity */
			req->cpus_affine = *cpumask_bits(mask);
			req->irq_notify.irq = req->irq;
			req->irq_notify.notify = pm_qos_irq_notify;
			req->irq_notify.release = pm_qos_irq_release;

		} else {
			req->type = PM_QOS_REQ_ALL_CORES;
			req->cpus_affine = CPUMASK_ALL;
			WARN(1, KERN_ERR "IRQ-%d not set for request with affinity flag\n",
					req->irq);
		}
		break;
#endif
	default:
		WARN(1, KERN_ERR "Unknown request type %d\n", req->type);
		/* fall through */
	case PM_QOS_REQ_ALL_CORES:
		req->cpus_affine = CPUMASK_ALL;
		break;
	}

	trace_pm_qos_add_request(value);

	req->qos = &cpu_latency_constraints;
	cpu_latency_qos_apply(req, PM_QOS_ADD_REQ, value);

#ifdef CONFIG_SMP
	if (req->type == PM_QOS_REQ_AFFINE_IRQ &&
			irq_can_set_affinity(req->irq)) {
		int ret = 0;

		ret = irq_set_affinity_notifier(req->irq,
					&req->irq_notify);
		if (ret) {
			WARN(1, "IRQ affinity notify set failed\n");
			req->type = PM_QOS_REQ_ALL_CORES;
			req->cpus_affine = CPUMASK_ALL;
			pm_qos_update_target(
				&cpu_latency_constraints,
				req, PM_QOS_UPDATE_REQ, value);
		}
	}
#endif
}
EXPORT_SYMBOL_GPL(cpu_latency_qos_add_request);

/**
 * cpu_latency_qos_update_request - Modify existing CPU latency QoS request.
 * @req : QoS request to update.
 * @new_value: New requested constraint value.
 *
 * Use @new_value to update the QoS request represented by @req in the CPU
 * latency QoS list along with updating the effective constraint value for that
 * list.
 */
void cpu_latency_qos_update_request(struct pm_qos_request *req, s32 new_value)
{
	if (!req || cpu_latency_qos_value_invalid(new_value))
		return;

	if (!cpu_latency_qos_request_active(req)) {
		WARN(1, KERN_ERR "%s called for unknown object\n", __func__);
		return;
	}

	trace_pm_qos_update_request(new_value);
	if (new_value == req->node.prio)
		return;

	cpu_latency_qos_apply(req, PM_QOS_UPDATE_REQ, new_value);
}
EXPORT_SYMBOL_GPL(cpu_latency_qos_update_request);

/**
 * cpu_latency_qos_remove_request - Remove existing CPU latency QoS request.
 * @req: QoS request to remove.
 *
 * Remove the CPU latency QoS request represented by @req from the CPU latency
 * QoS list along with updating the effective constraint value for that list.
 */
void cpu_latency_qos_remove_request(struct pm_qos_request *req)
{
	if (!req) /*guard against callers passing in null */
		return;
		/* silent return to keep pcm code cleaner */

	if (!cpu_latency_qos_request_active(req)) {
			WARN(1, KERN_ERR "%s called for unknown object\n", __func__);
		return;
	}

#ifdef CONFIG_SMP
	if (req->type == PM_QOS_REQ_AFFINE_IRQ) {
		int ret = 0;
		/* Get the current affinity */
		ret = irq_set_affinity_notifier(req->irq, NULL);
		if (ret)
			WARN(1, "IRQ affinity notify set failed\n");
	}
#endif

	trace_pm_qos_remove_request(PM_QOS_DEFAULT_VALUE);

	cpu_latency_qos_apply(req, PM_QOS_REMOVE_REQ, PM_QOS_DEFAULT_VALUE);
	memset(req, 0, sizeof(*req));
}
EXPORT_SYMBOL_GPL(cpu_latency_qos_remove_request);

/* User space interface to the CPU latency QoS via misc device. */

static int cpu_latency_qos_open(struct inode *inode, struct file *filp)
{
	struct pm_qos_request *req;

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	cpu_latency_qos_add_request(req, PM_QOS_DEFAULT_VALUE);
	filp->private_data = req;

	return 0;
}

static int cpu_latency_qos_release(struct inode *inode, struct file *filp)
{
	struct pm_qos_request *req = filp->private_data;

	filp->private_data = NULL;

	cpu_latency_qos_remove_request(req);
	kfree(req);

	return 0;
}

static ssize_t cpu_latency_qos_read(struct file *filp, char __user *buf,
				    size_t count, loff_t *f_pos)
{
	struct pm_qos_request *req = filp->private_data;
	unsigned long flags;
	s32 value;

	if (!req || !cpu_latency_qos_request_active(req))
		return -EINVAL;

	raw_spin_lock_irqsave(&pm_qos_lock, flags);
	value = pm_qos_get_value(&cpu_latency_constraints);
	raw_spin_unlock_irqrestore(&pm_qos_lock, flags);

	return simple_read_from_buffer(buf, count, f_pos, &value, sizeof(s32));
}

static ssize_t cpu_latency_qos_write(struct file *filp, const char __user *buf,
									 size_t count, loff_t *f_pos)
{
	s32 value;

	/* Don't let userspace impose restrictions on CPU idle levels */
	return count;

	if (count == sizeof(s32)) {
		if (copy_from_user(&value, buf, sizeof(s32)))
			return -EFAULT;
	} else {
		int ret;

		ret = kstrtos32_from_user(buf, count, 16, &value);
		if (ret)
			return ret;
	}

	cpu_latency_qos_update_request(filp->private_data, value);

	return count;
}

static const struct file_operations cpu_latency_qos_fops = {
	.write = cpu_latency_qos_write,
	.read = cpu_latency_qos_read,
	.open = cpu_latency_qos_open,
	.release = cpu_latency_qos_release,
	.llseek = noop_llseek,
};

static struct miscdevice cpu_latency_qos_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cpu_dma_latency",
	.fops = &cpu_latency_qos_fops,
};

static int __init cpu_latency_qos_init(void)
{
	int ret;

	ret = misc_register(&cpu_latency_qos_miscdev);
	if (ret < 0)
		pr_err("%s: %s setup failed\n", __func__,
		       cpu_latency_qos_miscdev.name);

	return ret;
}
late_initcall(cpu_latency_qos_init);
