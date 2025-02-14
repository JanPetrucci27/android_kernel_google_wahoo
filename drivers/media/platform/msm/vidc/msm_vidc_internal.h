/* Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
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

#ifndef _MSM_VIDC_INTERNAL_H_
#define _MSM_VIDC_INTERNAL_H_

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/kref.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/msm_vidc.h>
#include <media/msm_media_info.h>

#include "vidc_hfi_api.h"

#define MSM_VIDC_DRV_NAME "msm_vidc_driver"
#define MSM_VIDC_VERSION KERNEL_VERSION(0, 0, 1);
#define MAX_DEBUGFS_NAME 50
#define DEFAULT_TIMEOUT 3
#define DEFAULT_HEIGHT 1088
#define DEFAULT_WIDTH 1920
#define MIN_SUPPORTED_WIDTH 32
#define MIN_SUPPORTED_HEIGHT 32
#define DEFAULT_FPS 15

/* Maintains the number of FTB's between each FBD over a window */
#define DCVS_FTB_WINDOW 32

#define V4L2_EVENT_VIDC_BASE  10

#define SYS_MSG_START HAL_SYS_INIT_DONE
#define SYS_MSG_END HAL_SYS_ERROR
#define SESSION_MSG_START HAL_SESSION_EVENT_CHANGE
#define SESSION_MSG_END HAL_SESSION_ERROR
#define SYS_MSG_INDEX(__msg) (__msg - SYS_MSG_START)
#define SESSION_MSG_INDEX(__msg) (__msg - SESSION_MSG_START)


#define MAX_NAME_LENGTH 64

#define EXTRADATA_IDX(__num_planes) ((__num_planes) ? (__num_planes) - 1 : 0)

#define NUM_MBS_PER_SEC(__height, __width, __fps) \
	(NUM_MBS_PER_FRAME(__height, __width) * __fps)

#define NUM_MBS_PER_FRAME(__height, __width) \
	((ALIGN(__height, 16) / 16) * (ALIGN(__width, 16) / 16))

enum vidc_ports {
	OUTPUT_PORT,
	CAPTURE_PORT,
	MAX_PORT_NUM
};

enum vidc_core_state {
	VIDC_CORE_UNINIT = 0,
	VIDC_CORE_INIT,
	VIDC_CORE_INIT_DONE,
	VIDC_CORE_INVALID
};

/* Do not change the enum values unless
 * you know what you are doing*/
enum instance_state {
	MSM_VIDC_CORE_UNINIT_DONE = 0x0001,
	MSM_VIDC_CORE_INIT,
	MSM_VIDC_CORE_INIT_DONE,
	MSM_VIDC_OPEN,
	MSM_VIDC_OPEN_DONE,
	MSM_VIDC_LOAD_RESOURCES,
	MSM_VIDC_LOAD_RESOURCES_DONE,
	MSM_VIDC_START,
	MSM_VIDC_START_DONE,
	MSM_VIDC_STOP,
	MSM_VIDC_STOP_DONE,
	MSM_VIDC_RELEASE_RESOURCES,
	MSM_VIDC_RELEASE_RESOURCES_DONE,
	MSM_VIDC_CLOSE,
	MSM_VIDC_CLOSE_DONE,
	MSM_VIDC_CORE_UNINIT,
	MSM_VIDC_CORE_INVALID
};

struct buf_info {
	struct list_head list;
	struct vb2_buffer *buf;
};

struct msm_vidc_list {
	struct list_head list;
	struct mutex lock;
};

static inline void INIT_MSM_VIDC_LIST(struct msm_vidc_list *mlist)
{
	mutex_init(&mlist->lock);
	INIT_LIST_HEAD(&mlist->list);
}

static inline void DEINIT_MSM_VIDC_LIST(struct msm_vidc_list *mlist)
{
	mutex_destroy(&mlist->lock);
}

enum buffer_owner {
	DRIVER,
	FIRMWARE,
	CLIENT,
	MAX_OWNER
};

struct internal_buf {
	struct list_head list;
	enum hal_buffer buffer_type;
	struct msm_smem *handle;
	enum buffer_owner buffer_ownership;
};

struct msm_vidc_format {
	char name[MAX_NAME_LENGTH];
	u8 description[32];
	u32 fourcc;
	int type;
	u32 (*get_frame_size)(int plane, u32 height, u32 width);
};

struct msm_vidc_drv {
	struct mutex lock;
	struct list_head cores;
	int num_cores;
	struct dentry *debugfs_root;
	int thermal_level;
	u32 platform_version;
};

struct msm_video_device {
	int type;
	struct video_device vdev;
};

struct session_prop {
	u32 width[MAX_PORT_NUM];
	u32 height[MAX_PORT_NUM];
	u32 num_planes[MAX_PORT_NUM];
	u32 extradata[MAX_PORT_NUM];
	u32 fps;
	u32 bitrate;
};

struct buf_queue {
	struct vb2_queue vb2_bufq;
	struct mutex lock;
};

enum profiling_points {
	SYS_INIT = 0,
	SESSION_INIT,
	LOAD_RESOURCES,
	FRAME_PROCESSING,
	FW_IDLE,
	MAX_PROFILING_POINTS,
};

struct buf_count {
	int etb;
	int ftb;
	int fbd;
	int ebd;
};

struct dcvs_stats {
	int num_ftb[DCVS_FTB_WINDOW];
	bool transition_turbo;
	int ftb_index;
	int ftb_counter;
	bool prev_freq_lowered;
	bool prev_freq_increased;
	int threshold_disp_buf_high;
	int threshold_disp_buf_low;
	int load;
	int load_low;
	int load_high;
	int min_threshold;
	int max_threshold;
	int etb_counter;
	bool is_power_save_mode;
	unsigned int extra_buffer_count;
	u32 supported_codecs;
};

struct profile_data {
	int start;
	int stop;
	int cumulative;
	char name[64];
	int sampling;
	int average;
};

struct msm_vidc_debug {
	struct profile_data pdata[MAX_PROFILING_POINTS];
	int profile;
	int samples;
};

enum msm_vidc_modes {
	VIDC_SECURE = BIT(0),
	VIDC_TURBO = BIT(1),
	VIDC_THUMBNAIL = BIT(2),
	VIDC_LOW_POWER = BIT(3),
	VIDC_REALTIME = BIT(4),
};

struct msm_vidc_core {
	struct list_head list;
	struct mutex lock;
	int id;
	struct hfi_device *device;
	struct msm_video_device vdev[MSM_VIDC_MAX_DEVICES];
	struct v4l2_device v4l2_dev;
	struct list_head instances;
	struct dentry *debugfs_root;
	enum vidc_core_state state;
	struct completion completions[SYS_MSG_END - SYS_MSG_START + 1];
	enum msm_vidc_hfi_type hfi_type;
	struct msm_vidc_platform_resources resources;
	u32 enc_codec_supported;
	u32 dec_codec_supported;
	u32 codec_count;
	struct msm_vidc_capability *capabilities;
	struct delayed_work fw_unload_work;
	bool smmu_fault_handled;
};

struct msm_vidc_inst {
	struct list_head list;
	struct mutex sync_lock, lock;
	struct msm_vidc_core *core;
	enum session_type session_type;
	void *session;
	struct session_prop prop;
	enum instance_state state;
	struct msm_vidc_format fmts[MAX_PORT_NUM];
	struct buf_queue bufq[MAX_PORT_NUM];
	struct msm_vidc_list pendingq;
	struct msm_vidc_list scratchbufs;
	struct msm_vidc_list persistbufs;
	struct msm_vidc_list pending_getpropq;
	struct msm_vidc_list outputbufs;
	struct msm_vidc_list registeredbufs;
	struct buffer_requirements buff_req;
	void *mem_client;
	struct v4l2_ctrl_handler ctrl_handler;
	struct completion completions[SESSION_MSG_END - SESSION_MSG_START + 1];
	struct v4l2_ctrl **cluster;
	struct v4l2_fh event_handler;
	bool in_reconfig;
	u32 reconfig_width;
	u32 reconfig_height;
	struct dentry *debugfs_root;
	void *priv;
	struct msm_vidc_debug debug;
	struct buf_count count;
	struct dcvs_stats dcvs;
	enum msm_vidc_modes flags;
	struct msm_vidc_capability capability;
	u32 buffer_size_limit;
	enum buffer_mode_type buffer_mode_set[MAX_PORT_NUM];
	atomic_t seq_hdr_reqs;
	struct v4l2_ctrl **ctrls;
	bool dcvs_mode;
	enum msm_vidc_pixel_depth bit_depth;
	struct kref kref;
	unsigned long instant_bitrate;
	u32 buffers_held_in_driver;
	atomic_t in_flush;
	u32 pic_struct;
	u32 colour_space;
	u32 operating_rate;
	bool allow_ubwc_linear_event;
};

extern struct msm_vidc_drv *vidc_driver;

struct msm_vidc_ctrl_cluster {
	struct v4l2_ctrl **cluster;
	struct list_head list;
};

struct msm_vidc_ctrl {
	u32 id;
	char name[MAX_NAME_LENGTH];
	enum v4l2_ctrl_type type;
	s32 minimum;
	s32 maximum;
	s32 default_value;
	u32 step;
	u32 menu_skip_mask;
	u32 flags;
	const char * const *qmenu;
};

void handle_cmd_response(enum hal_command_response cmd, void *data);
int msm_vidc_trigger_ssr(struct msm_vidc_core *core,
	enum hal_ssr_trigger_type type);
int msm_vidc_check_session_supported(struct msm_vidc_inst *inst);
int msm_vidc_check_scaling_supported(struct msm_vidc_inst *inst);
void msm_vidc_queue_v4l2_event(struct msm_vidc_inst *inst, int event_type);

struct buffer_info {
	struct list_head list;
	int type;
	int num_planes;
	int fd[VIDEO_MAX_PLANES];
	int buff_off[VIDEO_MAX_PLANES];
	int size[VIDEO_MAX_PLANES];
	unsigned long uvaddr[VIDEO_MAX_PLANES];
	ion_phys_addr_t device_addr[VIDEO_MAX_PLANES];
	struct msm_smem *handle[VIDEO_MAX_PLANES];
	enum v4l2_memory memory;
	u32 v4l2_index;
	bool pending_deletion;
	atomic_t ref_count;
	bool dequeued;
	bool inactive;
	bool mapped[VIDEO_MAX_PLANES];
	int same_fd_ref[VIDEO_MAX_PLANES];
	struct timeval timestamp;
};

struct buffer_info *device_to_uvaddr(struct msm_vidc_list *buf_list,
				ion_phys_addr_t device_addr);
int buf_ref_get(struct msm_vidc_inst *inst, struct buffer_info *binfo);
int buf_ref_put(struct msm_vidc_inst *inst, struct buffer_info *binfo);
int output_buffer_cache_invalidate(struct msm_vidc_inst *inst,
				struct buffer_info *binfo);
int qbuf_dynamic_buf(struct msm_vidc_inst *inst,
			struct buffer_info *binfo);
int unmap_and_deregister_buf(struct msm_vidc_inst *inst,
			struct buffer_info *binfo);

void msm_comm_handle_thermal_event(void);
void *msm_smem_new_client(enum smem_type mtype,
		void *platform_resources, enum session_type stype);
struct msm_smem *msm_smem_alloc(void *clt, size_t size, u32 align, u32 flags,
		enum hal_buffer buffer_type, int map_kernel);
void msm_smem_free(void *clt, struct msm_smem *mem);
void msm_smem_delete_client(void *clt);
int msm_smem_cache_operations(void *clt, struct msm_smem *mem,
		enum smem_cache_ops);
struct msm_smem *msm_smem_user_to_kernel(void *clt, int fd, u32 offset,
				enum hal_buffer buffer_type);
struct context_bank_info *msm_smem_get_context_bank(void *clt,
		bool is_secure, enum hal_buffer buffer_type);
void msm_vidc_fw_unload_handler(struct work_struct *work);
bool msm_smem_compare_buffers(void *clt, int fd, void *priv);
/* XXX: normally should be in msm_vidc.h, but that's meant for public APIs,
 * whereas this is private */
int msm_vidc_destroy(struct msm_vidc_inst *inst);
#endif
