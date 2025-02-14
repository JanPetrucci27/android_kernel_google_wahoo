/*
 * EDAC PCI component
 *
 * Author: Dave Jiang <djiang@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/sysctl.h>
#include <linux/highmem.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <asm/page.h>

#include "edac_core.h"
#include "edac_module.h"

static DEFINE_MUTEX(edac_pci_ctls_mutex);
static LIST_HEAD(edac_pci_list);
static atomic_t pci_indexes = ATOMIC_INIT(0);

/*
 * edac_pci_alloc_ctl_info
 *
 *	The alloc() function for the 'edac_pci' control info
 *	structure. The chip driver will allocate one of these for each
 *	edac_pci it is going to control/register with the EDAC CORE.
 */
struct edac_pci_ctl_info *edac_pci_alloc_ctl_info(unsigned int sz_pvt,
						const char *edac_pci_name)
{
	struct edac_pci_ctl_info *pci;
	void *p = NULL, *pvt;
	unsigned int size;

	edac_dbg(1, "\n");

	pci = edac_align_ptr(&p, sizeof(*pci), 1);
	pvt = edac_align_ptr(&p, 1, sz_pvt);
	size = ((unsigned long)pvt) + sz_pvt;

	/* Alloc the needed control struct memory */
	pci = kzalloc(size, GFP_KERNEL);
	if (pci  == NULL)
		return NULL;

	/* Now much private space */
	pvt = sz_pvt ? ((char *)pci) + ((unsigned long)pvt) : NULL;

	pci->pvt_info = pvt;
	pci->op_state = OP_ALLOC;

	snprintf(pci->name, strlen(edac_pci_name) + 1, "%s", edac_pci_name);

	return pci;
}
EXPORT_SYMBOL_GPL(edac_pci_alloc_ctl_info);

/*
 * edac_pci_free_ctl_info()
 *
 *	Last action on the pci control structure.
 *
 *	call the remove sysfs information, which will unregister
 *	this control struct's kobj. When that kobj's ref count
 *	goes to zero, its release function will be call and then
 *	kfree() the memory.
 */
void edac_pci_free_ctl_info(struct edac_pci_ctl_info *pci)
{
	edac_dbg(1, "\n");

	edac_pci_remove_sysfs(pci);
}
EXPORT_SYMBOL_GPL(edac_pci_free_ctl_info);

/*
 * find_edac_pci_by_dev()
 * 	scans the edac_pci list for a specific 'struct device *'
 *
 *	return NULL if not found, or return control struct pointer
 */
static struct edac_pci_ctl_info *find_edac_pci_by_dev(struct device *dev)
{
	struct edac_pci_ctl_info *pci;
	struct list_head *item;

	edac_dbg(1, "\n");

	list_for_each(item, &edac_pci_list) {
		pci = list_entry(item, struct edac_pci_ctl_info, link);

		if (pci->dev == dev)
			return pci;
	}

	return NULL;
}

/*
 * add_edac_pci_to_global_list
 * 	Before calling this function, caller must assign a unique value to
 * 	edac_dev->pci_idx.
 * 	Return:
 * 		0 on success
 * 		1 on failure
 */
static int add_edac_pci_to_global_list(struct edac_pci_ctl_info *pci)
{
	struct list_head *item, *insert_before;
	struct edac_pci_ctl_info *rover;

	edac_dbg(1, "\n");

	insert_before = &edac_pci_list;

	/* Determine if already on the list */
	rover = find_edac_pci_by_dev(pci->dev);
	if (unlikely(rover != NULL))
		goto fail0;

	/* Insert in ascending order by 'pci_idx', so find position */
	list_for_each(item, &edac_pci_list) {
		rover = list_entry(item, struct edac_pci_ctl_info, link);

		if (rover->pci_idx >= pci->pci_idx) {
			if (unlikely(rover->pci_idx == pci->pci_idx))
				goto fail1;

			insert_before = item;
			break;
		}
	}

	list_add_tail_rcu(&pci->link, insert_before);
	return 0;

fail0:
	edac_printk(KERN_WARNING, EDAC_PCI,
		"%s (%s) %s %s already assigned %d\n",
		dev_name(rover->dev), edac_dev_name(rover),
		rover->mod_name, rover->ctl_name, rover->pci_idx);
	return 1;

fail1:
	edac_printk(KERN_WARNING, EDAC_PCI,
		"but in low-level driver: attempt to assign\n"
		"\tduplicate pci_idx %d in %s()\n", rover->pci_idx,
		__func__);
	return 1;
}

/*
 * del_edac_pci_from_global_list
 *
 *	remove the PCI control struct from the global list
 */
static void del_edac_pci_from_global_list(struct edac_pci_ctl_info *pci)
{
	list_del_rcu(&pci->link);

	/* these are for safe removal of devices from global list while
	 * NMI handlers may be traversing list
	 */
	synchronize_rcu();
	INIT_LIST_HEAD(&pci->link);
}

#if 0
/* Older code, but might use in the future */

/*
 * edac_pci_find()
 * 	Search for an edac_pci_ctl_info structure whose index is 'idx'
 *
 * If found, return a pointer to the structure
 * Else return NULL.
 *
 * Caller must hold pci_ctls_mutex.
 */
struct edac_pci_ctl_info *edac_pci_find(int idx)
{
	struct list_head *item;
	struct edac_pci_ctl_info *pci;

	/* Iterage over list, looking for exact match of ID */
	list_for_each(item, &edac_pci_list) {
		pci = list_entry(item, struct edac_pci_ctl_info, link);

		if (pci->pci_idx >= idx) {
			if (pci->pci_idx == idx)
				return pci;

			/* not on list, so terminate early */
			break;
		}
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(edac_pci_find);
#endif

/*
 * edac_pci_workq_function()
 *
 * 	periodic function that performs the operation
 *	scheduled by a workq request, for a given PCI control struct
 */
static void edac_pci_workq_function(struct work_struct *work_req)
{
	struct delayed_work *d_work = to_delayed_work(work_req);
	struct edac_pci_ctl_info *pci = to_edac_pci_ctl_work(d_work);
	int msec;
	unsigned long delay;

	edac_dbg(3, "checking\n");

	mutex_lock(&edac_pci_ctls_mutex);

	if (pci->op_state == OP_RUNNING_POLL) {
		/* we might be in POLL mode, but there may NOT be a poll func
		 */
		if ((pci->edac_check != NULL) && edac_pci_get_check_errors())
			pci->edac_check(pci);

		/* if we are on a one second period, then use round */
		msec = edac_pci_get_poll_msec();
		if (msec == 1000)
			delay = round_jiffies_relative(msecs_to_jiffies(msec));
		else
			delay = msecs_to_jiffies(msec);

		/* Reschedule only if we are in POLL mode */
		queue_delayed_work(edac_workqueue, &pci->work, delay);
	}

	mutex_unlock(&edac_pci_ctls_mutex);
}

/*
 * edac_pci_workq_setup()
 * 	initialize a workq item for this edac_pci instance
 * 	passing in the new delay period in msec
 *
 *	locking model:
 *		called when 'edac_pci_ctls_mutex' is locked
 */
static void edac_pci_workq_setup(struct edac_pci_ctl_info *pci,
				 unsigned int msec)
{
	edac_dbg(0, "\n");

	INIT_DELAYED_WORK(&pci->work, edac_pci_workq_function);
	queue_delayed_work(edac_workqueue, &pci->work,
			msecs_to_jiffies(edac_pci_get_poll_msec()));
}

/*
 * edac_pci_workq_teardown()
 * 	stop the workq processing on this edac_pci instance
 */
static void edac_pci_workq_teardown(struct edac_pci_ctl_info *pci)
{
	edac_dbg(0, "\n");

	pci->op_state = OP_OFFLINE;

	cancel_delayed_work_sync(&pci->work);
	flush_workqueue(edac_workqueue);
}

/*
 * edac_pci_reset_delay_period
 *
 *	called with a new period value for the workq period
 *	a) stop current workq timer
 *	b) restart workq timer with new value
 */
void edac_pci_reset_delay_period(struct edac_pci_ctl_info *pci,
				 unsigned long value)
{
	edac_dbg(0, "\n");

	edac_pci_workq_teardown(pci);

	/* need to lock for the setup */
	mutex_lock(&edac_pci_ctls_mutex);

	edac_pci_workq_setup(pci, value);

	mutex_unlock(&edac_pci_ctls_mutex);
}
EXPORT_SYMBOL_GPL(edac_pci_reset_delay_period);

/*
 * edac_pci_alloc_index: Allocate a unique PCI index number
 *
 * Return:
 *      allocated index number
 *
 */
int edac_pci_alloc_index(void)
{
	return atomic_inc_return(&pci_indexes) - 1;
}
EXPORT_SYMBOL_GPL(edac_pci_alloc_index);

/*
 * edac_pci_add_device: Insert the 'edac_dev' structure into the
 * edac_pci global list and create sysfs entries associated with
 * edac_pci structure.
 * @pci: pointer to the edac_device structure to be added to the list
 * @edac_idx: A unique numeric identifier to be assigned to the
 * 'edac_pci' structure.
 *
 * Return:
 *      0       Success
 *      !0      Failure
 */
int edac_pci_add_device(struct edac_pci_ctl_info *pci, int edac_idx)
{
	edac_dbg(0, "\n");

	pci->pci_idx = edac_idx;
	pci->start_time = jiffies;

	mutex_lock(&edac_pci_ctls_mutex);

	if (add_edac_pci_to_global_list(pci))
		goto fail0;

	if (edac_pci_create_sysfs(pci)) {
		edac_pci_printk(pci, KERN_WARNING,
				"failed to create sysfs pci\n");
		goto fail1;
	}

	if (pci->edac_check != NULL) {
		pci->op_state = OP_RUNNING_POLL;

		edac_pci_workq_setup(pci, 1000);
	} else {
		pci->op_state = OP_RUNNING_INTERRUPT;
	}

	edac_pci_printk(pci, KERN_INFO,
		"Giving out device to module %s controller %s: DEV %s (%s)\n",
		pci->mod_name, pci->ctl_name, pci->dev_name,
		edac_op_state_to_string(pci->op_state));

	mutex_unlock(&edac_pci_ctls_mutex);
	return 0;

	/* error unwind stack */
fail1:
	del_edac_pci_from_global_list(pci);
fail0:
	mutex_unlock(&edac_pci_ctls_mutex);
	return 1;
}
EXPORT_SYMBOL_GPL(edac_pci_add_device);

/*
 * edac_pci_del_device()
 * 	Remove sysfs entries for specified edac_pci structure and
 * 	then remove edac_pci structure from global list
 *
 * @dev:
 * 	Pointer to 'struct device' representing edac_pci structure
 * 	to remove
 *
 * Return:
 * 	Pointer to removed edac_pci structure,
 * 	or NULL if device not found
 */
struct edac_pci_ctl_info *edac_pci_del_device(struct device *dev)
{
	struct edac_pci_ctl_info *pci;

	edac_dbg(0, "\n");

	mutex_lock(&edac_pci_ctls_mutex);

	/* ensure the control struct is on the global list
	 * if not, then leave
	 */
	pci = find_edac_pci_by_dev(dev);
	if (pci  == NULL) {
		mutex_unlock(&edac_pci_ctls_mutex);
		return NULL;
	}

	pci->op_state = OP_OFFLINE;

	del_edac_pci_from_global_list(pci);

	mutex_unlock(&edac_pci_ctls_mutex);

	/* stop the workq timer */
	edac_pci_workq_teardown(pci);

	edac_printk(KERN_INFO, EDAC_PCI,
		"Removed device %d for %s %s: DEV %s\n",
		pci->pci_idx, pci->mod_name, pci->ctl_name, edac_dev_name(pci));

	return pci;
}
EXPORT_SYMBOL_GPL(edac_pci_del_device);

/*
 * edac_pci_generic_check
 *
 *	a Generic parity check API
 */
static void edac_pci_generic_check(struct edac_pci_ctl_info *pci)
{
	edac_dbg(4, "\n");
	edac_pci_do_parity_check();
}

/* free running instance index counter */
static int edac_pci_idx;
#define EDAC_PCI_GENCTL_NAME	"EDAC PCI controller"

struct edac_pci_gen_data {
	int edac_idx;
};

/*
 * edac_pci_create_generic_ctl
 *
 *	A generic constructor for a PCI parity polling device
 *	Some systems have more than one domain of PCI busses.
 *	For systems with one domain, then this API will
 *	provide for a generic poller.
 *
 *	This routine calls the edac_pci_alloc_ctl_info() for
 *	the generic device, with default values
 */
struct edac_pci_ctl_info *edac_pci_create_generic_ctl(struct device *dev,
						const char *mod_name)
{
	struct edac_pci_ctl_info *pci;
	struct edac_pci_gen_data *pdata;

	pci = edac_pci_alloc_ctl_info(sizeof(*pdata), EDAC_PCI_GENCTL_NAME);
	if (!pci)
		return NULL;

	pdata = pci->pvt_info;
	pci->dev = dev;
	dev_set_drvdata(pci->dev, pci);
	pci->dev_name = pci_name(to_pci_dev(dev));

	pci->mod_name = mod_name;
	pci->ctl_name = EDAC_PCI_GENCTL_NAME;
	if (edac_op_state == EDAC_OPSTATE_POLL)
		pci->edac_check = edac_pci_generic_check;

	pdata->edac_idx = edac_pci_idx++;

	if (edac_pci_add_device(pci, pdata->edac_idx) > 0) {
		edac_dbg(3, "failed edac_pci_add_device()\n");
		edac_pci_free_ctl_info(pci);
		return NULL;
	}

	return pci;
}
EXPORT_SYMBOL_GPL(edac_pci_create_generic_ctl);

/*
 * edac_pci_release_generic_ctl
 *
 *	The release function of a generic EDAC PCI polling device
 */
void edac_pci_release_generic_ctl(struct edac_pci_ctl_info *pci)
{
	edac_dbg(0, "pci mod=%s\n", pci->mod_name);

	edac_pci_del_device(pci->dev);
	edac_pci_free_ctl_info(pci);
}
EXPORT_SYMBOL_GPL(edac_pci_release_generic_ctl);
