/* Copyright (c) 2013-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/iopoll.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>

#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_panel.h"
#include "mdss_mdp.h"

#define DISP_ERR_RECOVERY_MS 50
#define STATUS_CHECK_INTERVAL_MS 3000
#define STATUS_CHECK_INTERVAL_MIN_MS 50
#define DSI_STATUS_CHECK_INIT -1
#define DSI_STATUS_CHECK_DISABLE 1

static uint32_t interval = STATUS_CHECK_INTERVAL_MS / 2;
static int32_t dsi_status_disable = DSI_STATUS_CHECK_INIT;
struct dsi_status_data *pstatus_data;

static void enable_status_irq(struct dsi_status_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;

	ctrl_pdata = container_of(dev_get_platdata(&pdata->mfd->pdev->dev),
				typeof(*ctrl_pdata), panel_data);

	atomic_set(&ctrl_pdata->te_irq_ready, 1);
	schedule_delayed_work(&pdata->check_status,
			msecs_to_jiffies(interval));
	enable_irq(gpio_to_irq(ctrl_pdata->disp_te_gpio));
}

static void disable_status_irq(struct dsi_status_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;

	ctrl_pdata = container_of(dev_get_platdata(&pdata->mfd->pdev->dev),
				typeof(*ctrl_pdata), panel_data);

	if (atomic_read(&ctrl_pdata->te_irq_ready)) {
		disable_irq(gpio_to_irq(ctrl_pdata->disp_te_gpio));
		atomic_set(&ctrl_pdata->te_irq_ready, 0);
	}
}

/*
 * check_dsi_ctrl_status() - Reads MFD structure and
 * calls platform specific DSI ctrl Status function.
 * @work  : dsi controller status data
 */
static void check_dsi_ctrl_status(struct work_struct *work)
{
	struct dsi_status_data *pdsi_status = NULL;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	unsigned long flag;

	pdsi_status = container_of(to_delayed_work(work),
		struct dsi_status_data, check_status);

	if (!pdsi_status) {
		pr_err("%s: DSI status data not available\n", __func__);
		return;
	}

	if (!pdsi_status->mfd) {
		pr_err("%s: FB data not available\n", __func__);
		return;
	}

	if (mdss_panel_is_power_off(pdsi_status->mfd->panel_power_state) ||
			pdsi_status->mfd->shutdown_pending) {
		pr_debug("%s: panel off\n", __func__);
		return;
	}
	
	ctrl_pdata = container_of(
				dev_get_platdata(&pdsi_status->mfd->pdev->dev),
				typeof(*ctrl_pdata), panel_data);

	if (!atomic_read(&ctrl_pdata->te_irq_ready)) {
		enable_status_irq(pdsi_status);
		return;
	}

	spin_lock_irqsave(&pstatus_data->te.spinlock, flag);

	pr_debug("now=%d, last=%d, vsync=%d\n",
			jiffies_to_msecs(jiffies),
			jiffies_to_msecs(pstatus_data->te.ts_last_check),
			jiffies_to_msecs(pstatus_data->te.ts_vsync));

	if (pstatus_data->te.err_fg == true ||
		time_after(pstatus_data->te.ts_last_check,
			   pstatus_data->te.ts_vsync)) {
		if (pstatus_data->te.err_fg == true) {
			pr_warn("<ESD TE> %s: ERR_FG triggered\n", __func__);
			pstatus_data->te.err_fg = false;
		} else
			pr_warn("<ESD TE> %s: Vsync doesn't come on time (%d %d)\n",
				__func__,
				jiffies_to_msecs(pstatus_data->te.ts_last_check),
				jiffies_to_msecs(pstatus_data->te.ts_vsync));

		/* change ts for next round of vsync check after panel dead */
		pstatus_data->te.ts_last_check =
				pstatus_data->te.ts_vsync = jiffies;

		if (pstatus_data->te.irq_enabled == true) {
			disable_irq_nosync(pstatus_data->te.irq);
			pstatus_data->te.irq_enabled = false;
		}
		spin_unlock_irqrestore(&pstatus_data->te.spinlock, flag);
		
		pdsi_status->mfd->mdp.check_dsi_status(work, interval);
	} else {
		pstatus_data->te.ts_last_check = jiffies;
		pr_debug("<ESD TE> %s: enable vsync IRQ\n", __func__);
		enable_irq(pstatus_data->te.irq);
		pstatus_data->te.irq_enabled = true;
		spin_unlock_irqrestore(&pstatus_data->te.spinlock, flag);
		mod_delayed_work(system_wq, &pstatus_data->check_status,
			msecs_to_jiffies(interval));
	}

}

void check_dsi_ctrl_status_ext(void)
{
	check_dsi_ctrl_status(&pstatus_data->check_status.work);
}

static void disable_vsync_irq(struct work_struct *work)
{
	struct dsi_status_data *pdata;

	pdata = container_of(work, typeof(*pdata), irq_done);
	disable_status_irq(pdata);
}

/*
 * hw_vsync_handler() - Interrupt handler for HW VSYNC signal.
 * @irq		: irq line number
 * @data	: Pointer to the device structure.
 *
 * This function is called whenever a HW vsync signal is received from the
 * panel. This changes the timestamp of HW vsync signal.
 */
irqreturn_t hw_vsync_handler(int irq, void *data)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata =
			(struct mdss_dsi_ctrl_pdata *)data;
	unsigned long flag;

	if (!ctrl_pdata) {
		pr_err("%s: DSI ctrl not available\n", __func__);
		return IRQ_HANDLED;
	}

	if (pstatus_data) {
		spin_lock_irqsave(&pstatus_data->te.spinlock, flag);
		pstatus_data->te.ts_vsync = jiffies;
		if (pstatus_data->te.irq_enabled) {
			pr_debug("<ESD TE> %s: disable vsync IRQ\n", __func__);
			pstatus_data->te.irq_enabled = false;
			disable_irq_nosync(pstatus_data->te.irq);
		}
		spin_unlock_irqrestore(&pstatus_data->te.spinlock, flag);
	} else
		pr_err("Pstatus data is NULL\n");
	
	queue_work(system_highpri_wq, &pstatus_data->irq_done);

	if (!atomic_read(&ctrl_pdata->te_irq_ready))
		atomic_inc(&ctrl_pdata->te_irq_ready);

	return IRQ_HANDLED;
}
/*
 * err_fg_handler() - Interrupt handler for ERR_FG signal.
 * @irq                : irq line number
 * @data       : Pointer to the device structure.
 *
 * This function is called whenever a ERR_FG signal is received from the
 * panel. Must trigger ESD workaround.
 */
irqreturn_t err_fg_handler(int irq, void *data)
{
	unsigned long flag;

	pr_info("%s: Handle ERR_FG\n", __func__);
	spin_lock_irqsave(&pstatus_data->te.spinlock, flag);
	if (pstatus_data)
		pstatus_data->te.err_fg = true;
	spin_unlock_irqrestore(&pstatus_data->te.spinlock, flag);

	return IRQ_HANDLED;
}

int disp_read_err_register(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct dcs_cmd_req cmdreq;
	/*
	 * setting a non-zero default value, if for some reason unable to read
	 * back the value it should be treated as failure
	 */
	char rx_buf[1] = {0xFF};
	char err_read_dcs_cmd[2] = {0x9F, 0x00}; /* DTYPE_DCS_READ */
	struct dsi_cmd_desc err_read_dsi_cmd_desc = {
		{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(err_read_dcs_cmd)},
		err_read_dcs_cmd
	};
	int ret;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &err_read_dsi_cmd_desc;
	cmdreq.cmds_cnt = 1;
	/* make sure there is no ongoing transfer while checking error status */
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_HS_MODE |
			CMD_CLK_CTRL | CMD_REQ_MDP_IDLE;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rx_buf;

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
	if (ret != cmdreq.rlen) {
		pr_warn("%s: unable to read panel status (%d)\n",
			__func__, ret);
		return -EINVAL;
	}

	pr_info("%s: Reg[0x%x]=0x%x\n", __func__,
		err_read_dcs_cmd[0], rx_buf[0]);

	return rx_buf[0];
}

void disp_err_recovery_work(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct delayed_work *dw = to_delayed_work(work);
	struct mdss_panel_info *pinfo;
	int err;

	ctrl_pdata = container_of(dw, struct mdss_dsi_ctrl_pdata, err_int_work);
	if (!ctrl_pdata) {
		pr_err("%s: invalid ctrl data\n", __func__);
		return;
	}

	pinfo = &ctrl_pdata->panel_data.panel_info;

	if (pinfo->panel_power_state !=  MDSS_PANEL_POWER_ON) {
		pr_err("%s: SKIP panel reset\n", __func__);
		return;
	}
	err = disp_read_err_register(ctrl_pdata);
	if (err)
		mdss_fb_report_panel_dead(pstatus_data->mfd);
}

irqreturn_t disp_err_detect_handler(int irq, void *data)
{
#if 0
	struct mdss_dsi_ctrl_pdata *pdata = (struct mdss_dsi_ctrl_pdata *)data;

	pr_info("%s: Handle disp ERR_DETECT\n", __func__);
	if (pdata->rdy_err_detect)
		schedule_delayed_work(&pdata->err_int_work,
			msecs_to_jiffies(DISP_ERR_RECOVERY_MS));
#endif
	return IRQ_HANDLED;
}

/*
 * disable_esd_thread() - Cancels work item for the esd check.
 */
void disable_esd_thread(void)
{
	if (pstatus_data &&
	    cancel_delayed_work(&pstatus_data->check_status))
		pr_debug("esd thread killed\n");
}

/*
 * fb_event_callback() - Call back function for the fb_register_client()
 *			 notifying events
 * @self  : notifier block
 * @event : The event that was triggered
 * @data  : Of type struct fb_event
 *
 * This function listens for FB_BLANK_UNBLANK and FB_BLANK_POWERDOWN events
 * from frame buffer. DSI status check work is either scheduled again after
 * PANEL_STATUS_CHECK_INTERVAL or cancelled based on the event.
 */
static int fb_event_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct dsi_status_data *pdata = container_of(self,
				struct dsi_status_data, fb_notifier);
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo;
	struct msm_fb_data_type *mfd;

	if (!evdata) {
		pr_err("%s: event data not available\n", __func__);
		return NOTIFY_BAD;
	}

	/* handle only mdss fb device */
	if (strncmp("mdssfb", evdata->info->fix.id, 6))
		return NOTIFY_DONE;

	mfd = evdata->info->par;
	ctrl_pdata = container_of(dev_get_platdata(&mfd->pdev->dev),
				struct mdss_dsi_ctrl_pdata, panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: DSI ctrl not available\n", __func__);
		return NOTIFY_BAD;
	}

	pinfo = &ctrl_pdata->panel_data.panel_info;
	if (pinfo->err_detect_enabled) {
		pdata->mfd = evdata->info->par;
		ctrl_pdata->rdy_err_detect = true;
	}

	if ((!(pinfo->esd_check_enabled) &&
			dsi_status_disable) ||
			(dsi_status_disable == DSI_STATUS_CHECK_DISABLE)) {
		pr_debug("ESD check is disabled.\n");
		cancel_delayed_work(&pdata->check_status);
		return NOTIFY_DONE;
	}

	pdata->mfd = evdata->info->par;
	if (event == FB_EVENT_BLANK) {
		int *blank = evdata->data;
		struct dsi_status_data *pdata = container_of(self,
				struct dsi_status_data, fb_notifier);
		pdata->mfd = evdata->info->par;

		switch (*blank) {
		case FB_BLANK_UNBLANK:
			pdata->te.irq = gpio_to_irq(ctrl_pdata->disp_te_gpio);
			pdata->te.ts_last_check =
				pdata->te.ts_vsync = jiffies;

			enable_status_irq(pdata);
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			cancel_work_sync(&pdata->irq_done);
			disable_status_irq(pdata);
			cancel_delayed_work(&pdata->check_status);
			break;
		default:
			pr_err("Unknown case in FB_EVENT_BLANK event\n");
			break;
		}
	}
	return 0;
}

static int param_dsi_status_disable(const char *val,
				    const struct kernel_param *kp)
{
	int ret = 0;
	int int_val;

	ret = kstrtos32(val, 0, &int_val);
	if (ret)
		return ret;

	pr_info("%s: Set DSI status disable to %d\n",
			__func__, int_val);
	*((int *)kp->arg) = int_val;
	return ret;
}

static int param_set_interval(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	int int_val;

	ret = kstrtos32(val, 0, &int_val);
	if (ret)
		return ret;
	if (int_val < STATUS_CHECK_INTERVAL_MIN_MS) {
		pr_err("%s: Invalid value %d used, ignoring\n",
						__func__, int_val);
		ret = -EINVAL;
	} else {
		pr_info("%s: Set check interval to %d msecs\n",
						__func__, int_val);
		*((int *)kp->arg) = int_val;
	}
	return ret;
}

int __init mdss_dsi_status_init(void)
{
	int rc = 0;

	pstatus_data = kzalloc(sizeof(struct dsi_status_data), GFP_KERNEL);
	if (!pstatus_data) {
		pr_err("%s: can't allocate memory\n", __func__);
		return -ENOMEM;
	}

	pstatus_data->fb_notifier.notifier_call = fb_event_callback;

	rc = fb_register_client(&pstatus_data->fb_notifier);
	if (rc < 0) {
		pr_err("%s: fb_register_client failed, returned with rc=%d\n",
								__func__, rc);
		kfree(pstatus_data);
		return -EPERM;
	}

	pstatus_data->te.irq = -1;
	pstatus_data->te.irq_enabled = false;
	pstatus_data->te.ts_vsync =
		pstatus_data->te.ts_last_check = jiffies;
	spin_lock_init(&pstatus_data->te.spinlock);

	pr_info("%s: DSI status check interval:%d\n", __func__,	interval);
	
	INIT_WORK(&pstatus_data->irq_done, disable_vsync_irq);
	INIT_DELAYED_WORK(&pstatus_data->check_status, check_dsi_ctrl_status);

	pr_debug("%s: DSI ctrl status work queue initialized\n", __func__);

	return rc;
}

void __exit mdss_dsi_status_exit(void)
{
	fb_unregister_client(&pstatus_data->fb_notifier);
	cancel_work_sync(&pstatus_data->irq_done);
	cancel_delayed_work_sync(&pstatus_data->check_status);
	kfree(pstatus_data);
	pr_debug("%s: DSI ctrl status work queue removed\n", __func__);
}

module_param_call(interval, param_set_interval, param_get_uint,
						&interval, 0644);
MODULE_PARM_DESC(interval,
		"Duration in milliseconds to send BTA command for checking"
		"DSI status periodically");

module_param_call(dsi_status_disable, param_dsi_status_disable, param_get_uint,
						&dsi_status_disable, 0644);
MODULE_PARM_DESC(dsi_status_disable,
		"Disable DSI status check");

module_init(mdss_dsi_status_init);
module_exit(mdss_dsi_status_exit);

MODULE_LICENSE("GPL v2");
