/* Copyright (c) 2007-2017, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/iommu.h>
#include <linux/qcom_iommu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk/msm-clk.h>

#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/of_platform.h>
#include <linux/msm_dma_iommu_mapping.h>

#include <linux/qcom_iommu.h>
#include <linux/mdss_smmu_ext.h>

#include <asm/dma-iommu.h>
#include "soc/qcom/secure_buffer.h"

#include "mdss.h"
#include "mdss_mdp.h"
#include "mdss_smmu.h"
#include "mdss_debug.h"

#define SZ_4G		0xF0000000

static DEFINE_MUTEX(mdp_iommu_lock);

static struct mdss_smmu_private smmu_private;

struct msm_smmu_notifier_data {
	struct list_head _user;
	msm_smmu_handler_t callback;
};

struct mdss_smmu_private *mdss_smmu_get_private(void)
{
	return &smmu_private;
}

void mdss_iommu_lock(void)
{
	mutex_lock(&mdp_iommu_lock);
}

void mdss_iommu_unlock(void)
{
	mutex_unlock(&mdp_iommu_lock);
}

static int mdss_smmu_secure_wait(int State, int request)
{
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	int rc = 0;
	/**
	  * Case1: MDP in Secure Display and Rotator in Non Secure
	  */
	if (!State && !request && mdss_get_sd_client_cnt()) {
		rc = wait_event_interruptible_timeout(mdata->secure_waitq,
				(mdss_get_sd_client_cnt() == 0),
				KOFF_TIMEOUT);
		if (rc <= 0) {
			pr_err("timed out waiting for Secure transtion: %d\n",
				mdss_get_sd_client_cnt());
			rc = -EINVAL;
		}
	}

	return rc;
}

static int mdss_smmu_secure_session_ctrl(int enable)
{
	int rc = 0;
	/**
	  * Currently client requests only enable/disable.
	  * TODO: Secure camera is hardcoded need to extend.
	  */
	rc = mdss_mdp_secure_session_ctrl(enable,
					  MDP_SECURE_CAMERA_OVERLAY_SESSION);
	if (rc)
		pr_err("%s: mdss_mdp_secure_session_ctrl failed : %d\n",
			__func__, rc);

	return rc;
}

static inline bool all_devices_probed(struct mdss_smmu_private *prv)
{
	struct device_node *child;
	struct mdss_smmu_client *tmp;
	int d_cnt = 0;
	int p_cnt = 0;

	if (!prv->pdev)
		return 0;

	for_each_child_of_node(prv->pdev, child) {
		if (is_mdss_smmu_compatible_device(child->name))
			d_cnt++;
	}

	list_for_each_entry(tmp, &prv->smmu_device_list, _client) {
		p_cnt++;
	}

	return (d_cnt && (d_cnt == p_cnt) ? true : false);
}

void mdss_iommu_notify_users(struct mdss_smmu_private *prv)
{
	struct msm_smmu_notifier_data *notify;
	struct mdss_smmu_client *client;

	/* Initiate callbacks for all the users who registered before probe */
	if (all_devices_probed(prv)) {
		list_for_each_entry(notify, &prv->user_list, _user) {
			list_for_each_entry(client,
			    &prv->smmu_device_list, _client)
				notify->callback(&client->base);
		}
	}
}

int mdss_smmu_request_mappings(msm_smmu_handler_t callback)
{
	struct mdss_smmu_client *client;
	struct msm_smmu_notifier_data *ndata;
	struct mdss_smmu_private *prv = mdss_smmu_get_private();
	int ret = 0;

	mutex_lock(&prv->smmu_reg_lock);

	if (!all_devices_probed(prv)) {
		ndata = kzalloc(sizeof(struct msm_smmu_notifier_data),
						GFP_KERNEL);
		if (!ndata) {
			ret = -ENOMEM;
			goto done;
		}
		ndata->callback = callback;
		list_add(&ndata->_user, &prv->user_list);
		goto done;
	}

	/* Probe already done mappings are available */
	list_for_each_entry(client, &prv->smmu_device_list, _client) {
		callback(&client->base);
	}

done:
	mutex_unlock(&prv->smmu_reg_lock);
	return ret;
}

static int mdss_smmu_util_parse_dt_clock(struct platform_device *pdev,
		struct dss_module_power *mp)
{
	u32 i = 0, rc = 0;
	const char *clock_name;
	u32 clock_rate;
	int num_clk;

	num_clk = of_property_count_strings(pdev->dev.of_node,
			"clock-names");
	if (num_clk <= 0) {
		pr_err("clocks are not defined\n");
		goto clk_err;
	}

	mp->num_clk = num_clk;
	mp->clk_config = devm_kzalloc(&pdev->dev,
			sizeof(struct dss_clk) * mp->num_clk, GFP_KERNEL);
	if (!mp->clk_config) {
		pr_err("clock configuration allocation failed\n");
		rc = -ENOMEM;
		mp->num_clk = 0;
		goto clk_err;
	}

	for (i = 0; i < mp->num_clk; i++) {
		of_property_read_string_index(pdev->dev.of_node, "clock-names",
							i, &clock_name);
		strlcpy(mp->clk_config[i].clk_name, clock_name,
				sizeof(mp->clk_config[i].clk_name));

		of_property_read_u32_index(pdev->dev.of_node, "clock-rate",
							i, &clock_rate);
		mp->clk_config[i].rate = clock_rate;

		if (!clock_rate)
			mp->clk_config[i].type = DSS_CLK_AHB;
		else
			mp->clk_config[i].type = DSS_CLK_PCLK;
	}

clk_err:
	return rc;
}

static int mdss_smmu_clk_register(struct platform_device *pdev,
		struct dss_module_power *mp)
{
	int i, ret;
	struct clk *clk;

	ret = mdss_smmu_util_parse_dt_clock(pdev, mp);
	if (ret) {
		pr_err("unable to parse clocks\n");
		return -EINVAL;
	}

	for (i = 0; i < mp->num_clk; i++) {
		clk = devm_clk_get(&pdev->dev,
				mp->clk_config[i].clk_name);
		if (IS_ERR(clk)) {
			pr_err("unable to get clk: %s\n",
					mp->clk_config[i].clk_name);
			return PTR_ERR(clk);
		}
		mp->clk_config[i].clk = clk;
	}
	return 0;
}

static int mdss_smmu_enable_power(struct mdss_smmu_client *mdss_smmu,
	bool enable)
{
	int rc = 0;
	struct dss_module_power *mp;

	if (!mdss_smmu)
		return -EINVAL;

	mp = &mdss_smmu->mp;

	if (!mp->num_vreg && !mp->num_clk)
		return 0;

	if (enable) {
		rc = msm_dss_enable_vreg(mp->vreg_config, mp->num_vreg, true);
		if (rc) {
			pr_err("vreg enable failed - rc:%d\n", rc);
			goto end;
		}
		mdss_update_reg_bus_vote(mdss_smmu->reg_bus_clt,
			VOTE_INDEX_LOW);
		rc = msm_dss_enable_clk(mp->clk_config, mp->num_clk, true);
		if (rc) {
			pr_err("clock enable failed - rc:%d\n", rc);
			mdss_update_reg_bus_vote(mdss_smmu->reg_bus_clt,
				VOTE_INDEX_DISABLE);
			msm_dss_enable_vreg(mp->vreg_config, mp->num_vreg,
				false);
			goto end;
		}
	} else {
		msm_dss_enable_clk(mp->clk_config, mp->num_clk, false);
		mdss_update_reg_bus_vote(mdss_smmu->reg_bus_clt,
			VOTE_INDEX_DISABLE);
		msm_dss_enable_vreg(mp->vreg_config, mp->num_vreg, false);
	}
end:
	return rc;
}

/*
 * mdss_smmu_attach_v2()
 *
 * Associates each configured VA range with the corresponding smmu context
 * bank device. Enables the clks as smmu_v2 requires voting it before the usage.
 * And iommu attach is done only once during the initial attach and it is never
 * detached as smmu v2 uses a feature called 'retention'.
 * Only detach the secure and non-secure contexts in case of secure display
 * case and secure contexts for secure camera use cases for the platforms
 * which have caps MDSS_CAPS_SEC_DETACH_SMMU enabled
 */
static int mdss_smmu_attach_v2(struct mdss_data_type *mdata)
{
	struct mdss_smmu_client *mdss_smmu;
	int i, rc = 0;

	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		if (!mdss_smmu_is_valid_domain_type(mdata, i))
			continue;
		mdss_smmu = mdss_smmu_get_cb(i);
		if (mdss_smmu && mdss_smmu->base.dev) {
			if (!mdss_smmu->handoff_pending) {
				rc = mdss_smmu_enable_power(mdss_smmu, true);
				if (rc) {
					pr_err("power enable failed - domain:[%d] rc:%d\n",
						i, rc);
					goto err;
				}
			}
			mdss_smmu->handoff_pending = false;

			if (!mdss_smmu->domain_attached &&
				mdss_smmu_is_valid_domain_condition(mdata,
					i, true)) {
				rc = arm_iommu_attach_device(
						    mdss_smmu->base.dev,
						    mdss_smmu->mmu_mapping);
				if (rc) {
					pr_err("iommu attach device failed for domain[%d] with err:%d\n",
						i, rc);
					mdss_smmu_enable_power(mdss_smmu,
						false);
					goto err;
				}
				mdss_smmu->domain_attached = true;
				if (mdss_smmu->domain_reattach) {
					pr_debug("iommu v2 domain[%i] remove extra vote\n",
							i);
					/* remove extra power vote */
					mdss_smmu_enable_power(mdss_smmu,
						false);
					mdss_smmu->domain_reattach = false;
				}
				pr_debug("iommu v2 domain[%i] attached\n", i);
			}
		} else {
			pr_err("iommu device not attached for domain[%d]\n", i);
			return -ENODEV;
		}
	}

	return 0;

err:
	for (i--; i >= 0; i--) {
		mdss_smmu = mdss_smmu_get_cb(i);
		if (mdss_smmu && mdss_smmu->base.dev) {
			arm_iommu_detach_device(mdss_smmu->base.dev);
			mdss_smmu_enable_power(mdss_smmu, false);
			mdss_smmu->domain_attached = false;
		}
	}

	return rc;
}

/*
 * mdss_smmu_detach_v2()
 *
 * Disables the clks only when it is not required to detach the iommu mapped
 * VA range (as long as not in secure display use case)
 * from the device in smmu_v2 as explained in the mdss_smmu_v2_attach
 */
static int mdss_smmu_detach_v2(struct mdss_data_type *mdata)
{
	struct mdss_smmu_client *mdss_smmu;
	int i;

	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		if (!mdss_smmu_is_valid_domain_type(mdata, i))
			continue;

		mdss_smmu = mdss_smmu_get_cb(i);
		if (mdss_smmu && mdss_smmu->base.dev) {
			if (!mdss_smmu->handoff_pending &&
				mdss_smmu->domain_attached &&
				mdss_smmu_is_valid_domain_condition(mdata,
					i, false)) {
				/*
				 * if entering in secure display or
				 * secure camera use case(for secured contexts
				 * leave the smmu clocks on and only detach the
				 * smmu contexts
				 */
				arm_iommu_detach_device(mdss_smmu->base.dev);
				mdss_smmu->domain_attached = false;
				/*
				 * since we are leaving clocks on, on
				 * re-attach do not vote for clocks
				 */
				mdss_smmu->domain_reattach = true;
				pr_debug("iommu v2 domain[%i] detached\n", i);
			} else {
				mdss_smmu_enable_power(mdss_smmu, false);
			}
		}
	}

	return 0;
}

static int mdss_smmu_get_domain_id_v2(u32 type)
{
	return type;
}

/*
 * mdss_smmu_dma_buf_attach_v2()
 *
 * Same as mdss_smmu_dma_buf_attach except that the device is got from
 * the configured smmu v2 context banks.
 */
static struct dma_buf_attachment *mdss_smmu_dma_buf_attach_v2(
		struct dma_buf *dma_buf, struct device *dev, int domain)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return NULL;
	}

	return dma_buf_attach(dma_buf, mdss_smmu->base.dev);
}

/*
 * mdss_smmu_map_dma_buf_v2()
 *
 * Maps existing buffer (by struct scatterlist) into SMMU context bank device.
 * From which we can take the virtual address and size allocated.
 * msm_map_dma_buf is depricated with smmu v2 and it uses dma_map_sg instead
 */
static int mdss_smmu_map_dma_buf_v2(struct dma_buf *dma_buf,
		struct sg_table *table, int domain, dma_addr_t *iova,
		unsigned long *size, int dir)
{
	int rc;
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return -EINVAL;
	}
	ATRACE_BEGIN("map_buffer");
	rc = msm_dma_map_sg_lazy(mdss_smmu->base.dev, table->sgl, table->nents,
				 dir, dma_buf);
	if (rc != table->nents) {
		pr_err("dma map sg failed\n");
		return -ENOMEM;
	}
	ATRACE_END("map_buffer");
	*iova = table->sgl->dma_address;
	*size = table->sgl->dma_length;
	return 0;
}

static void mdss_smmu_unmap_dma_buf_v2(struct sg_table *table, int domain,
		int dir, struct dma_buf *dma_buf)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return;
	}

	ATRACE_BEGIN("unmap_buffer");
	msm_dma_unmap_sg(mdss_smmu->base.dev, table->sgl, table->nents, dir,
		 dma_buf);
	ATRACE_END("unmap_buffer");
}

/*
 * mdss_smmu_dma_alloc_coherent_v2()
 *
 * Allocates buffer same as mdss_smmu_dma_alloc_coherent_v1, but in addition it
 * also maps to the SMMU domain with the help of the respective SMMU context
 * bank device
 */
static int mdss_smmu_dma_alloc_coherent_v2(struct device *dev, size_t size,
		dma_addr_t *phys, dma_addr_t *iova, void *cpu_addr,
		gfp_t gfp, int domain)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return -EINVAL;
	}

	cpu_addr = dma_alloc_coherent(mdss_smmu->base.dev, size, iova, gfp);
	if (!cpu_addr) {
		pr_err("dma alloc coherent failed!\n");
		return -ENOMEM;
	}
	*phys = iommu_iova_to_phys(mdss_smmu->mmu_mapping->domain,
			*iova);
	return 0;
}

static void mdss_smmu_dma_free_coherent_v2(struct device *dev, size_t size,
		void *cpu_addr, dma_addr_t phys, dma_addr_t iova, int domain)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return;
	}

	dma_free_coherent(mdss_smmu->base.dev, size, cpu_addr, iova);
}

/*
 * mdss_smmu_map_v1()
 *
 * Same as mdss_smmu_map_v1, just that it maps to the appropriate domain
 * referred by the smmu context bank handles.
 */
static int mdss_smmu_map_v2(int domain, phys_addr_t iova, phys_addr_t phys,
		int gfp_order, int prot)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return -EINVAL;
	}

	return iommu_map(mdss_smmu->mmu_mapping->domain,
			iova, phys, gfp_order, prot);
}

static void mdss_smmu_unmap_v2(int domain, unsigned long iova, int gfp_order)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return;
	}

	iommu_unmap(mdss_smmu->mmu_mapping->domain, iova, gfp_order);
}

/*
 * mdss_smmUdsi_alloc_buf_v2()
 *
 * Allocates the buffer and mapping is done later
 */
static char *mdss_smmu_dsi_alloc_buf_v2(struct device *dev, int size,
		dma_addr_t *dmap, gfp_t gfp)
{
	char *data;

	data = kzalloc(size, GFP_KERNEL | GFP_DMA);
	if (data)
		*dmap = (dma_addr_t) virt_to_phys(data);

	return data;
}

/*
 * mdss_smmu_dsi_map_buffer_v2()
 *
 * Maps the buffer allocated in mdss_smmu_dsi_alloc_buffer_v2 with the SMMU
 * domain and uses dma_map_single as msm_iommu_map_contig_buffer is depricated
 * in smmu v2.
 */
static int mdss_smmu_dsi_map_buffer_v2(phys_addr_t phys, unsigned int domain,
		unsigned long size, dma_addr_t *dma_addr, void *cpu_addr,
		int dir)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return -EINVAL;
	}

	*dma_addr = dma_map_single(mdss_smmu->base.dev, cpu_addr, size, dir);
	if (dma_mapping_error(mdss_smmu->base.dev, *dma_addr)) {
		pr_err("dma map single failed\n");
		return -ENOMEM;
	}
	return 0;
}

static void mdss_smmu_dsi_unmap_buffer_v2(dma_addr_t dma_addr, int domain,
		unsigned long size, int dir)
{
	struct mdss_smmu_client *mdss_smmu = mdss_smmu_get_cb(domain);
	if (!mdss_smmu) {
		pr_err("not able to get smmu context\n");
		return;
	}

	if (is_mdss_iommu_attached())
		dma_unmap_single(mdss_smmu->base.dev, dma_addr, size, dir);
}

int mdss_smmu_fault_handler(struct iommu_domain *domain, struct device *dev,
	unsigned long iova, int flags, void *user_data)
{
	struct mdss_smmu_client *mdss_smmu =
		(struct mdss_smmu_client *)user_data;
	u32 fsynr1, mid, i;

	if (!mdss_smmu || !mdss_smmu->mmu_base)
		goto end;

	fsynr1 = readl_relaxed(mdss_smmu->mmu_base + SMMU_CBN_FSYNR1);
	mid = fsynr1 & 0xff;
	pr_err("mdss_smmu: iova:0x%lx flags:0x%x fsynr1: 0x%x mid: 0x%x\n",
		iova, flags, fsynr1, mid);

	/* get domain id information */
	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		if (mdss_smmu == mdss_smmu_get_cb(i))
			break;
	}

	if (i == MDSS_IOMMU_MAX_DOMAIN)
		goto end;

	mdss_mdp_debug_mid(mid);
end:
	return -ENOSYS;
}

static void mdss_smmu_deinit_v2(struct mdss_data_type *mdata)
{
	int i;
	struct mdss_smmu_client *mdss_smmu;

	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		mdss_smmu = mdss_smmu_get_cb(i);
		if (mdss_smmu && mdss_smmu->base.dev)
			arm_iommu_release_mapping(mdss_smmu->mmu_mapping);
	}
}

static void mdss_smmu_ops_init(struct mdss_data_type *mdata)
{
	mdata->smmu_ops.smmu_attach = mdss_smmu_attach_v2;
	mdata->smmu_ops.smmu_detach = mdss_smmu_detach_v2;
	mdata->smmu_ops.smmu_get_domain_id = mdss_smmu_get_domain_id_v2;
	mdata->smmu_ops.smmu_dma_buf_attach =
				mdss_smmu_dma_buf_attach_v2;
	mdata->smmu_ops.smmu_map_dma_buf = mdss_smmu_map_dma_buf_v2;
	mdata->smmu_ops.smmu_unmap_dma_buf = mdss_smmu_unmap_dma_buf_v2;
	mdata->smmu_ops.smmu_dma_alloc_coherent =
				mdss_smmu_dma_alloc_coherent_v2;
	mdata->smmu_ops.smmu_dma_free_coherent =
				mdss_smmu_dma_free_coherent_v2;
	mdata->smmu_ops.smmu_map = mdss_smmu_map_v2;
	mdata->smmu_ops.smmu_unmap = mdss_smmu_unmap_v2;
	mdata->smmu_ops.smmu_dsi_alloc_buf = mdss_smmu_dsi_alloc_buf_v2;
	mdata->smmu_ops.smmu_dsi_map_buffer =
				mdss_smmu_dsi_map_buffer_v2;
	mdata->smmu_ops.smmu_dsi_unmap_buffer =
				mdss_smmu_dsi_unmap_buffer_v2;
	mdata->smmu_ops.smmu_deinit = mdss_smmu_deinit_v2;
}

/*
 * mdss_smmu_device_create()
 * @dev: mdss_mdp device
 *
 * For smmu_v2, each context bank is a seperate child device of mdss_mdp.
 * Platform devices are created for those smmu related child devices of
 * mdss_mdp here. This would facilitate probes to happen for these devices in
 * which the smmu mapping and initilization is handled.
 */
void mdss_smmu_device_create(struct device *dev)
{
	struct device_node *parent, *child;
	struct mdss_smmu_private *prv = mdss_smmu_get_private();

	parent = dev->of_node;
	for_each_child_of_node(parent, child) {
		if (is_mdss_smmu_compatible_device(child->name))
			of_platform_device_create(child, NULL, dev);
	}
	prv->pdev = parent;
}

int mdss_smmu_init(struct mdss_data_type *mdata, struct device *dev)
{
	mdata->mdss_util->iommu_lock = mdss_iommu_lock;
	mdata->mdss_util->iommu_unlock = mdss_iommu_unlock;
	mdata->mdss_util->iommu_ctrl = mdss_iommu_ctrl;
	mdata->mdss_util->secure_session_ctrl =
		mdss_smmu_secure_session_ctrl;

	mdss_smmu_device_create(dev);
	mdss_smmu_ops_init(mdata);

	return 0;
}

static struct mdss_smmu_domain mdss_mdp_unsec = {
	"mdp_0", MDSS_IOMMU_DOMAIN_UNSECURE, SZ_128K, (SZ_4G - SZ_128K)};
static struct mdss_smmu_domain mdss_rot_unsec = {
	NULL, MDSS_IOMMU_DOMAIN_ROT_UNSECURE, SZ_128K, (SZ_4G - SZ_128K)};
static struct mdss_smmu_domain mdss_mdp_sec = {
	"mdp_1", MDSS_IOMMU_DOMAIN_SECURE, SZ_128K, (SZ_4G - SZ_128K)};
static struct mdss_smmu_domain mdss_rot_sec = {
	NULL, MDSS_IOMMU_DOMAIN_ROT_SECURE, SZ_128K, (SZ_4G - SZ_128K)};

static const struct of_device_id mdss_smmu_dt_match[] = {
	{ .compatible = "qcom,smmu_mdp_unsec", .data = &mdss_mdp_unsec},
	{ .compatible = "qcom,smmu_rot_unsec", .data = &mdss_rot_unsec},
	{ .compatible = "qcom,smmu_mdp_sec", .data = &mdss_mdp_sec},
	{ .compatible = "qcom,smmu_rot_sec", .data = &mdss_rot_sec},
	{}
};
MODULE_DEVICE_TABLE(of, mdss_smmu_dt_match);

/*
 * mdss_smmu_probe()
 * @pdev: platform device
 *
 * Each smmu context acts as a separate device and the context banks are
 * configured with a VA range.
 * Registeres the clks as each context bank has its own clks, for which voting
 * has to be done everytime before using that context bank.
 */
int mdss_smmu_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_smmu_private *prv = mdss_smmu_get_private();
	struct mdss_smmu_client *mdss_smmu;
	int rc = 0;
	struct mdss_smmu_domain smmu_domain;
	const struct of_device_id *match;
	struct dss_module_power *mp;
	char name[MAX_CLIENT_NAME_LEN];
	const __be32 *address = NULL, *size = NULL;

	if (!mdata) {
		pr_err("probe failed as mdata is not initialized\n");
		return -EPROBE_DEFER;
	}

	match = of_match_device(mdss_smmu_dt_match, &pdev->dev);
	if (!match || !match->data) {
		pr_err("probe failed as match data is invalid\n");
		return -EINVAL;
	}

	smmu_domain = *(struct mdss_smmu_domain *) (match->data);
	if (smmu_domain.domain >= MDSS_IOMMU_MAX_DOMAIN) {
		pr_err("no matching device found\n");
		return -EINVAL;
	}

	if (of_find_property(pdev->dev.of_node, "iommus", NULL)) {
		dev = &pdev->dev;
	} else {
		/*
		 * For old iommu driver we query the context bank device
		 * rather than getting it from dt.
		 */
		dev = msm_iommu_get_ctx(smmu_domain.ctx_name);
		if (!dev) {
			pr_err("Invalid SMMU ctx for domain:%d\n",
				smmu_domain.domain);
			return -EINVAL;
		}
	}

	mdss_smmu = &mdata->mdss_smmu[smmu_domain.domain];
	mdss_smmu->base.domain = smmu_domain.domain;
	mp = &mdss_smmu->mp;
	mdss_smmu->base.is_secure = false;
	memset(mp, 0, sizeof(struct dss_module_power));

	if (of_find_property(pdev->dev.of_node,
		"gdsc-mmagic-mdss-supply", NULL)) {

		mp->vreg_config = devm_kzalloc(&pdev->dev,
			sizeof(struct dss_vreg), GFP_KERNEL);
		if (!mp->vreg_config)
			return -ENOMEM;

		strlcpy(mp->vreg_config->vreg_name, "gdsc-mmagic-mdss",
				sizeof(mp->vreg_config->vreg_name));
		mp->num_vreg = 1;
	}

	rc = msm_dss_config_vreg(&pdev->dev, mp->vreg_config,
		mp->num_vreg, true);
	if (rc) {
		pr_err("vreg config failed rc=%d\n", rc);
		return rc;
	}

	rc = mdss_smmu_clk_register(pdev, mp);
	if (rc) {
		pr_err("smmu clk register failed for domain[%d] with err:%d\n",
			smmu_domain.domain, rc);
		msm_dss_config_vreg(&pdev->dev, mp->vreg_config, mp->num_vreg,
			false);
		return rc;
	}

	snprintf(name, MAX_CLIENT_NAME_LEN, "smmu:%u", smmu_domain.domain);
	mdss_smmu->reg_bus_clt = mdss_reg_bus_vote_client_create(name);
	if (IS_ERR(mdss_smmu->reg_bus_clt)) {
		pr_err("mdss bus client register failed\n");
		msm_dss_config_vreg(&pdev->dev, mp->vreg_config, mp->num_vreg,
			false);
		return PTR_ERR(mdss_smmu->reg_bus_clt);
	}

	rc = mdss_smmu_enable_power(mdss_smmu, true);
	if (rc) {
		pr_err("power enable failed - domain:[%d] rc:%d\n",
			smmu_domain.domain, rc);
		goto bus_client_destroy;
	}

	mdss_smmu->mmu_mapping = arm_iommu_create_mapping(
		msm_iommu_get_bus(dev), smmu_domain.start, smmu_domain.size);
	if (IS_ERR(mdss_smmu->mmu_mapping)) {
		pr_err("iommu create mapping failed for domain[%d]\n",
			smmu_domain.domain);
		rc = PTR_ERR(mdss_smmu->mmu_mapping);
		goto disable_power;
	}

	if (smmu_domain.domain == MDSS_IOMMU_DOMAIN_SECURE ||
		smmu_domain.domain == MDSS_IOMMU_DOMAIN_ROT_SECURE) {
		int secure_vmid = VMID_CP_PIXEL;
		rc = iommu_domain_set_attr(mdss_smmu->mmu_mapping->domain,
			DOMAIN_ATTR_SECURE_VMID, &secure_vmid);
		if (rc) {
			pr_err("couldn't set secure pixel vmid\n");
			goto release_mapping;
		}
		mdss_smmu->base.is_secure = true;
	}

	if (!mdata->handoff_pending)
		mdss_smmu_enable_power(mdss_smmu, false);
	else
		mdss_smmu->handoff_pending = true;

	mdss_smmu->base.dev = dev;

	address = of_get_address_by_name(pdev->dev.of_node, "mmu_cb", 0, 0);
	if (address) {
		size = address + 1;
		mdss_smmu->mmu_base = ioremap(be32_to_cpu(*address),
			be32_to_cpu(*size));
		if (mdss_smmu->mmu_base)
			iommu_set_fault_handler(mdss_smmu->mmu_mapping->domain,
				mdss_smmu_fault_handler, mdss_smmu);
	} else {
		pr_debug("unable to map context bank base\n");
	}

	mdss_smmu->base.iommu_ctrl = mdata->mdss_util->iommu_ctrl;
	mdss_smmu->base.reg_lock = mdata->mdss_util->vbif_reg_lock;
	mdss_smmu->base.reg_unlock = mdata->mdss_util->vbif_reg_unlock;
	mdss_smmu->base.secure_session_ctrl =
		mdata->mdss_util->secure_session_ctrl;
	mdss_smmu->base.wait_for_transition = mdss_smmu_secure_wait;
	mdss_smmu->base.handoff_pending = mdata->mdss_util->mdp_handoff_pending;

	list_add(&mdss_smmu->_client, &prv->smmu_device_list);

	mdss_iommu_notify_users(prv);

	pr_info("iommu v2 domain[%d] mapping and clk register successful!\n",
			smmu_domain.domain);
	return 0;

release_mapping:
	arm_iommu_release_mapping(mdss_smmu->mmu_mapping);
disable_power:
	mdss_smmu_enable_power(mdss_smmu, false);
bus_client_destroy:
	mdss_reg_bus_vote_client_destroy(mdss_smmu->reg_bus_clt);
	mdss_smmu->reg_bus_clt = NULL;
	msm_dss_config_vreg(&pdev->dev, mp->vreg_config, mp->num_vreg,
			false);
	return rc;
}

int mdss_smmu_remove(struct platform_device *pdev)
{
	int i;
	struct mdss_smmu_client *mdss_smmu;

	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		mdss_smmu = mdss_smmu_get_cb(i);
		if (mdss_smmu && mdss_smmu->base.dev &&
			(mdss_smmu->base.dev == &pdev->dev))
			arm_iommu_release_mapping(mdss_smmu->mmu_mapping);
	}
	return 0;
}

static struct platform_driver mdss_smmu_driver = {
	.probe = mdss_smmu_probe,
	.remove = mdss_smmu_remove,
	.shutdown = NULL,
	.driver = {
		.name = "mdss_smmu",
		.of_match_table = mdss_smmu_dt_match,
	},
};

static int mdss_smmu_register_driver(void)
{
	struct mdss_smmu_private *prv = mdss_smmu_get_private();
	int ret;

	INIT_LIST_HEAD(&prv->smmu_device_list);
	INIT_LIST_HEAD(&prv->user_list);
	mutex_init(&prv->smmu_reg_lock);

	ret = platform_driver_register(&mdss_smmu_driver);
	if (ret)
		pr_err("mdss_smmu_register_driver() failed!\n");

	return ret;
}

static int __init mdss_smmu_driver_init(void)
{
	int ret;
	ret = mdss_smmu_register_driver();
	if (ret)
		pr_err("mdss_smmu_register_driver() failed!\n");

	return ret;
}
module_init(mdss_smmu_driver_init);

static void __exit mdss_smmu_driver_cleanup(void)
{
	struct mdss_smmu_private *prv = mdss_smmu_get_private();
	struct msm_smmu_notifier_data *node;
	struct list_head *pos, *q;

	list_for_each_safe(pos, q, &prv->user_list) {
		node = list_entry(pos, struct msm_smmu_notifier_data, _user);
		list_del(&node->_user);
		kfree(node);
	}
	platform_driver_unregister(&mdss_smmu_driver);
}
module_exit(mdss_smmu_driver_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MDSS SMMU driver");
