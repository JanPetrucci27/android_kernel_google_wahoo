/*
 * Copyright (c) 2014-2018 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

#if !defined(__QDF_TRACE_H)
#define __QDF_TRACE_H

/**
 *  DOC: qdf_trace
 *  QCA driver framework trace APIs
 *  Trace, logging, and debugging definitions and APIs
 */

/* Include Files */
#include  <qdf_types.h>         /* For QDF_MODULE_ID... */
#include  <stdarg.h>            /* For va_list... */
#include  <qdf_status.h>
#include  <qdf_nbuf.h>
#include  <i_qdf_types.h>
#include <qdf_debugfs.h>


/* Type declarations */

#define FL(x)    "%s: %d: " x, __func__, __LINE__

typedef int (qdf_abstract_print)(void *priv, const char *fmt, ...);

/*
 * Log levels
 */
#define QDF_DEBUG_FUNCTRACE     0x01
#define QDF_DEBUG_LEVEL0        0x02
#define QDF_DEBUG_LEVEL1        0x04
#define QDF_DEBUG_LEVEL2        0x08
#define QDF_DEBUG_LEVEL3        0x10
#define QDF_DEBUG_ERROR         0x20
#define QDF_DEBUG_CFG           0x40

#ifdef CONFIG_MCL
/* By default Data Path module will have all log levels enabled, except debug
 * log level. Debug level will be left up to the framework or user space modules
 * to be enabled when issue is detected
 */
#define QDF_DATA_PATH_TRACE_LEVEL \
	((1 << QDF_TRACE_LEVEL_FATAL) | (1 << QDF_TRACE_LEVEL_ERROR) | \
	(1 << QDF_TRACE_LEVEL_WARN) | (1 << QDF_TRACE_LEVEL_INFO) | \
	(1 << QDF_TRACE_LEVEL_INFO_HIGH) | (1 << QDF_TRACE_LEVEL_INFO_MED) | \
	(1 << QDF_TRACE_LEVEL_INFO_LOW))

/* Preprocessor definitions and constants */
#define ASSERT_BUFFER_SIZE (512)

#define MAX_QDF_TRACE_RECORDS 4000
#define INVALID_QDF_TRACE_ADDR 0xffffffff
#define DEFAULT_QDF_TRACE_DUMP_COUNT 0

#include  <i_qdf_trace.h>
/*
 * first parameter to iwpriv command - dump_dp_trace
 * iwpriv wlan0 dump_dp_trace 0 0 -> dump full buffer
 * iwpriv wlan0 dump_dp_trace 1 0 -> enable live view mode
 * iwpriv wlan0 dump_dp_trace 2 0 -> clear dp trace buffer
 * iwpriv wlan0 dump_dp_trace 2 0 -> disable live view mode
 */
#define DUMP_DP_TRACE			0
#define ENABLE_DP_TRACE_LIVE_MODE	1
#define CLEAR_DP_TRACE_BUFFER		2
#define DISABLE_DP_TRACE_LIVE_MODE	3


#ifdef TRACE_RECORD
#define MTRACE(p) p

#else
#define MTRACE(p) {  }
#endif
#define NO_SESSION 0xFF

/**
 * typedef struct qdf_trace_record_s - keep trace record
 * @qtime: qtimer ticks
 * @time: user timestamp
 * @module: module name
 * @code: hold record of code
 * @session: hold record of session
 * @data: hold data
 * @pid: hold pid of the process
 */
typedef struct qdf_trace_record_s {
	uint64_t qtime;
	char time[18];
	uint8_t module;
	uint8_t code;
	uint16_t session;
	uint32_t data;
	uint32_t pid;
} qdf_trace_record_t, *tp_qdf_trace_record;

/**
 * typedef struct s_qdf_trace_data - MTRACE logs are stored in ring buffer
 * @head: position of first record
 * @tail: position of last record
 * @num: count of total record
 * @num_since_last_dump: count from last dump
 * @enable: config for controlling the trace
 * @dump_count: Dump after number of records reach this number
 */
typedef struct s_qdf_trace_data {
	uint32_t head;
	uint32_t tail;
	uint32_t num;
	uint16_t num_since_last_dump;
	uint8_t enable;
	uint16_t dump_count;
} t_qdf_trace_data;

#define CASE_RETURN_STRING(str) case ((str)): return (uint8_t *)(# str);

/* DP Trace Implementation */
#ifdef FEATURE_DP_TRACE
#define DPTRACE(p) p
#else
#define DPTRACE(p)
#endif

#define MAX_QDF_DP_TRACE_RECORDS       2000
#define QDF_DP_TRACE_RECORD_SIZE       40
#define INVALID_QDF_DP_TRACE_ADDR      0xffffffff
#define QDF_DP_TRACE_VERBOSITY_HIGH    3
#define QDF_DP_TRACE_VERBOSITY_MEDIUM  2
#define QDF_DP_TRACE_VERBOSITY_LOW     1
#define QDF_DP_TRACE_VERBOSITY_BASE    0

/**
 * enum QDF_DP_TRACE_ID - Generic ID to identify various events in data path
 * @QDF_DP_TRACE_INVALID - invalid
 * @QDF_DP_TRACE_DROP_PACKET_RECORD - record drop packet
 * @QDF_DP_TRACE_EAPOL_PACKET_RECORD - record EAPOL packet
 * @QDF_DP_TRACE_DHCP_PACKET_RECORD - record DHCP packet
 * @QDF_DP_TRACE_ARP_PACKET_RECORD - record ARP packet
 * @QDF_DP_TRACE_MGMT_PACKET_RECORD - record MGMT pacekt
 * @QDF_DP_TRACE_ICMP_PACKET_RECORD - record ICMP packet
 * @QDF_DP_TRACE_ICMPv6_PACKET_RECORD - record ICMPv6 packet
 * QDF_DP_TRACE_EVENT_RECORD - record events
 * @QDF_DP_TRACE_BASE_VERBOSITY - below this are part of base verbosity
 * @QDF_DP_TRACE_ICMP_PACKET_RECORD - record ICMP packets
 * @QDF_DP_TRACE_TX_PACKET_RECORD - record tx pkt
 * @QDF_DP_TRACE_RX_PACKET_RECORD - record rx pkt
 * @QDF_DP_TRACE_HDD_TX_TIMEOUT - HDD tx timeout
 * @QDF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT- SOFTAP HDD tx timeout
 * @QDF_DP_TRACE_FREE_PACKET_PTR_RECORD - tx completion ptr record
 * @QDF_DP_TRACE_LOW_VERBOSITY - below this are part of low verbosity
 * @QDF_DP_TRACE_HDD_TX_PACKET_PTR_RECORD - HDD layer ptr record
  * @QDF_DP_TRACE_RX_HDD_PACKET_PTR_RECORD - HDD RX record
 * @QDF_DP_TRACE_CE_PACKET_PTR_RECORD - CE layer ptr record
 * @QDF_DP_TRACE_CE_FAST_PACKET_PTR_RECORD- CE fastpath ptr record
 * @QDF_DP_TRACE_CE_FAST_PACKET_ERR_RECORD- CE fastpath error record
 * @QDF_DP_TRACE_RX_HTT_PACKET_PTR_RECORD - HTT RX record
 * @QDF_DP_TRACE_RX_OFFLOAD_HTT_PACKET_PTR_RECORD- HTT RX offload record
  * @QDF_DP_TRACE_MED_VERBOSITY - below this are part of med verbosity
 * @QDF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD -tx queue ptr record
 * @QDF_DP_TRACE_TXRX_PACKET_PTR_RECORD - txrx packet ptr record
 * @QDF_DP_TRACE_TXRX_FAST_PACKET_PTR_RECORD - txrx fast path record
 * @QDF_DP_TRACE_HTT_PACKET_PTR_RECORD - htt packet ptr record
 * @QDF_DP_TRACE_HTC_PACKET_PTR_RECORD - htc packet ptr record
 * @QDF_DP_TRACE_HIF_PACKET_PTR_RECORD - hif packet ptr record
 * @QDF_DP_TRACE_RX_TXRX_PACKET_PTR_RECORD - txrx packet ptr record
 * @QDF_DP_TRACE_HIGH_VERBOSITY - below this are part of high verbosity
 */
enum  QDF_DP_TRACE_ID {
	QDF_DP_TRACE_INVALID,
	QDF_DP_TRACE_DROP_PACKET_RECORD,
	QDF_DP_TRACE_EAPOL_PACKET_RECORD,
	QDF_DP_TRACE_DHCP_PACKET_RECORD,
	QDF_DP_TRACE_ARP_PACKET_RECORD,
	QDF_DP_TRACE_MGMT_PACKET_RECORD,
	QDF_DP_TRACE_EVENT_RECORD,
	QDF_DP_TRACE_BASE_VERBOSITY,
	QDF_DP_TRACE_ICMP_PACKET_RECORD,
	QDF_DP_TRACE_ICMPv6_PACKET_RECORD,
	QDF_DP_TRACE_TX_PACKET_RECORD,
	QDF_DP_TRACE_RX_PACKET_RECORD,
	QDF_DP_TRACE_HDD_TX_TIMEOUT,
	QDF_DP_TRACE_HDD_SOFTAP_TX_TIMEOUT,
	QDF_DP_TRACE_FREE_PACKET_PTR_RECORD,
	QDF_DP_TRACE_LOW_VERBOSITY,
	QDF_DP_TRACE_HDD_TX_PACKET_PTR_RECORD,
	QDF_DP_TRACE_RX_HDD_PACKET_PTR_RECORD,
	QDF_DP_TRACE_CE_PACKET_PTR_RECORD,
	QDF_DP_TRACE_CE_FAST_PACKET_PTR_RECORD,
	QDF_DP_TRACE_CE_FAST_PACKET_ERR_RECORD,
	QDF_DP_TRACE_RX_HTT_PACKET_PTR_RECORD,
	QDF_DP_TRACE_RX_OFFLOAD_HTT_PACKET_PTR_RECORD,
	QDF_DP_TRACE_MED_VERBOSITY,
	QDF_DP_TRACE_TXRX_QUEUE_PACKET_PTR_RECORD,
	QDF_DP_TRACE_TXRX_PACKET_PTR_RECORD,
	QDF_DP_TRACE_TXRX_FAST_PACKET_PTR_RECORD,
	QDF_DP_TRACE_HTT_PACKET_PTR_RECORD,
	QDF_DP_TRACE_HTC_PACKET_PTR_RECORD,
	QDF_DP_TRACE_HIF_PACKET_PTR_RECORD,
	QDF_DP_TRACE_RX_TXRX_PACKET_PTR_RECORD,
	QDF_DP_TRACE_HIGH_VERBOSITY,
	QDF_DP_TRACE_MAX
};

/**
 * qdf_proto_dir - direction
 * @QDF_TX: TX direction
 * @QDF_RX: RX direction
 * @QDF_NA: not applicable
 */
enum qdf_proto_dir {
	QDF_TX,
	QDF_RX,
	QDF_NA
};

/**
 * struct qdf_dp_trace_ptr_buf - pointer record buffer
 * @cookie: cookie value
 * @msdu_id: msdu_id
 * @status: completion status
 */
struct qdf_dp_trace_ptr_buf {
	uint64_t cookie;
	uint16_t msdu_id;
	uint16_t status;
};

/**
 * struct qdf_dp_trace_proto_buf - proto packet buffer
 * @sa: source address
 * @da: destination address
 * @vdev_id : vdev id
 * @type: packet type
 * @subtype: packet subtype
 * @dir: direction
 */
struct qdf_dp_trace_proto_buf {
	struct qdf_mac_addr sa;
	struct qdf_mac_addr da;
	uint8_t vdev_id;
	uint8_t type;
	uint8_t subtype;
	uint8_t dir;
};

/**
 * struct qdf_dp_trace_mgmt_buf - mgmt packet buffer
 * @vdev_id : vdev id
 * @type: packet type
 * @subtype: packet subtype
 */
struct qdf_dp_trace_mgmt_buf {
	uint8_t vdev_id;
	uint8_t type;
	uint8_t subtype;
};

/**
 * struct qdf_dp_trace_event_buf - event buffer
 * @vdev_id : vdev id
 * @type: packet type
 * @subtype: packet subtype
 */
struct qdf_dp_trace_event_buf {
	uint8_t vdev_id;
	uint8_t type;
	uint8_t subtype;
};

/**
 * struct qdf_dp_trace_data_buf - nbuf data buffer
 * @msdu_id : msdu_id of the packet (for TX, for RX = 0)
 */
struct qdf_dp_trace_data_buf {
	uint16_t msdu_id;
};

/**
 * struct qdf_dp_trace_record_s - Describes a record in DP trace
 * @time: time when it got stored
 * @code: Describes the particular event
 * @data: buffer to store data
 * @size: Length of the valid data stored in this record
 * @pid : process id which stored the data in this record
 */
struct qdf_dp_trace_record_s {
	u64 time;
	uint8_t code;
	uint8_t data[QDF_DP_TRACE_RECORD_SIZE];
	uint8_t size;
	uint32_t pid;
};

/**
 * struct qdf_dp_trace_data - Parameters to configure/control DP trace
 * @head: Position of first record
 * @tail: Position of last record
 * @num:  Current index
 * @proto_bitmap: defines which protocol to be traced
 * @no_of_record: defines every nth packet to be traced
 * @verbosity : defines verbosity level
 * @enable: enable/disable DP trace
 * @count: current packet number
 * @live_mode_config: configuration as received during initialization
 * @live_mode: current live mode, enabled or disabled, can be throttled based
 *             on throughput
 * force_live_mode: flag to enable live mode all the time for all packets.
 *                  This can be set/unset from userspace and overrides other
 *                  live mode flags.
 * @print_pkt_cnt: count of number of packets printed in live mode
 *.@high_tput_thresh: thresh beyond which live mode is turned off
 *.@thresh_time_limit: max time, in terms of BW timer intervals to wait,
 *          for determining if high_tput_thresh has been crossed. ~1s
 * @arp_req: stats for arp reqs
 * @arp_resp: stats for arp resps
 * @icmp_req: stats for icmp reqs
 * @icmp_resp: stats for icmp resps
 * @dhcp_disc: stats for dhcp discover msgs
 * @dhcp_req: stats for dhcp req msgs
 * @dhcp_off: stats for dhcp offer msgs
 * @dhcp_ack: stats for dhcp ack msgs
 * @dhcp_nack: stats for dhcp nack msgs
 * @dhcp_others: stats for other dhcp pkts types
 * @eapol_m1: stats for eapol m1
 * @eapol_m2: stats for eapol m2
 * @eapol_m3: stats for eapol m3
 * @eapol_m4: stats for eapol m4
 * @eapol_others: stats for other eapol pkt types
 * @icmpv6_req: stats for icmpv6 reqs
 * @icmpv6_resp: stats for icmpv6 resps
 *.@icmpv6_ns: stats for icmpv6 nss
 *.@icmpv6_na: stats for icmpv6 nas
 *.@icmpv6_rs: stats for icmpv6 rss
 *.@icmpv6_ra: stats for icmpv6 ras
 */
struct s_qdf_dp_trace_data {
	uint32_t head;
	uint32_t tail;
	uint32_t num;
	uint8_t proto_bitmap;
	uint8_t no_of_record;
	uint8_t verbosity;
	bool enable;
	bool live_mode_config;
	bool live_mode;
	uint32_t curr_pos;
	uint32_t saved_tail;
	bool force_live_mode;
	uint8_t print_pkt_cnt;
	uint8_t high_tput_thresh;
	uint16_t thresh_time_limit;
	/* Stats */
	uint32_t tx_count;
	uint32_t rx_count;
	uint16_t arp_req;
	uint16_t arp_resp;
	uint16_t dhcp_disc;
	uint16_t dhcp_req;
	uint16_t dhcp_off;
	uint16_t dhcp_ack;
	uint16_t dhcp_nack;
	uint16_t dhcp_others;
	uint16_t eapol_m1;
	uint16_t eapol_m2;
	uint16_t eapol_m3;
	uint16_t eapol_m4;
	uint16_t eapol_others;
	uint16_t icmp_req;
	uint16_t icmp_resp;
	uint16_t icmpv6_req;
	uint16_t icmpv6_resp;
	uint16_t icmpv6_ns;
	uint16_t icmpv6_na;
	uint16_t icmpv6_rs;
	uint16_t icmpv6_ra;
};

/**
 * struct qdf_dpt_debugfs_state - state to control read to debugfs file
 * @QDF_DPT_DEBUGFS_STATE_SHOW_STATE_INVALID: invalid state
 * @QDF_DPT_DEBUGFS_STATE_SHOW_STATE_INIT: initial state
 * @QDF_DPT_DEBUGFS_STATE_SHOW_IN_PROGRESS: read is in progress
 * @QDF_DPT_DEBUGFS_STATE_SHOW_COMPLETE:  read complete
 */

enum qdf_dpt_debugfs_state {
	QDF_DPT_DEBUGFS_STATE_SHOW_STATE_INVALID,
	QDF_DPT_DEBUGFS_STATE_SHOW_STATE_INIT,
	QDF_DPT_DEBUGFS_STATE_SHOW_IN_PROGRESS,
	QDF_DPT_DEBUGFS_STATE_SHOW_COMPLETE,
};

/* Function declarations and documenation */

/**
 * qdf_trace_set_level() - Set the trace level for a particular module
 * @level : trace level
 *
 * Trace level is a member of the QDF_TRACE_LEVEL enumeration indicating
 * the severity of the condition causing the trace message to be issued.
 * More severe conditions are more likely to be logged.
 *
 * This is an external API that allows trace levels to be set for each module.
 *
 * Return:  nothing
 */
void qdf_trace_set_level(QDF_MODULE_ID module, QDF_TRACE_LEVEL level);

/**
 * qdf_trace_get_level() - get the trace level
 * @level : trace level
 *
 * This is an external API that returns a bool value to signify if a
 * particular trace level is set for the specified module.
 * A member of the QDF_TRACE_LEVEL enumeration indicating the severity
 * of the condition causing the trace message to be issued.
 *
 * Note that individual trace levels are the only valid values
 * for this API.  QDF_TRACE_LEVEL_NONE and QDF_TRACE_LEVEL_ALL
 * are not valid input and will return false
 *
 * Return:
 *  false - the specified trace level for the specified module is OFF
 *  true - the specified trace level for the specified module is ON
 */
bool qdf_trace_get_level(QDF_MODULE_ID module, QDF_TRACE_LEVEL level);

typedef void (*tp_qdf_trace_cb)(void *p_mac, tp_qdf_trace_record, uint16_t);
typedef void (*tp_qdf_state_info_cb) (char **buf, uint16_t *size);
void qdf_register_debugcb_init(void);
void qdf_register_debug_callback(QDF_MODULE_ID module_id,
					tp_qdf_state_info_cb qdf_state_infocb);
QDF_STATUS qdf_state_info_dump_all(char *buf, uint16_t size,
			uint16_t *driver_dump_size);
void qdf_trace(uint8_t module, uint8_t code, uint16_t session, uint32_t data);
void qdf_trace_register(QDF_MODULE_ID, tp_qdf_trace_cb);
QDF_STATUS qdf_trace_spin_lock_init(void);
void qdf_trace_init(void);
void qdf_trace_enable(uint32_t, uint8_t enable);
void qdf_trace_dump_all(void *, uint8_t, uint8_t, uint32_t, uint32_t);


#ifdef FEATURE_DP_TRACE
#define QDF_DP_TRACE_RECORD_INFO_LIVE (0x1)
#define QDF_DP_TRACE_RECORD_INFO_THROTTLED (0x1 << 1)

bool qdf_dp_trace_log_pkt(uint8_t session_id, struct sk_buff *skb,
				enum qdf_proto_dir dir);
void qdf_dp_trace_init(bool live_mode_config, uint8_t thresh,
				uint16_t time_limit, uint8_t verbosity,
				uint8_t proto_bitmap);
void qdf_dp_trace_spin_lock_init(void);
void qdf_dp_trace_set_value(uint8_t proto_bitmap, uint8_t no_of_records,
			 uint8_t verbosity);
void qdf_dp_trace_set_track(qdf_nbuf_t nbuf, enum qdf_proto_dir dir);
void qdf_dp_trace(qdf_nbuf_t nbuf, enum QDF_DP_TRACE_ID code,
			uint8_t *data, uint8_t size, enum qdf_proto_dir dir);

/**
 * qdf_dpt_get_curr_pos_debugfs() - get curr position to start read
 * @file: debugfs file to read
 * @state: state to control read to debugfs file
 *
 * Return: curr pos
 */
uint32_t qdf_dpt_get_curr_pos_debugfs(qdf_debugfs_file_t file,
				enum qdf_dpt_debugfs_state state);
/**
 * qdf_dpt_dump_stats_debugfs() - dump DP Trace stats to debugfs file
 * @file: debugfs file to read
 * @curr_pos: curr position to start read
 *
 * Return: QDF_STATUS
 */
QDF_STATUS qdf_dpt_dump_stats_debugfs(qdf_debugfs_file_t file,
				      uint32_t curr_pos);

/**
 * qdf_dpt_set_value_debugfs() - dump DP Trace stats to debugfs file
 * @file: debugfs file to read
 * @curr_pos: curr position to start read
 *
 * Return: none
 */
void qdf_dpt_set_value_debugfs(uint8_t proto_bitmap, uint8_t no_of_record,
			    uint8_t verbosity);


void qdf_dp_trace_dump_all(uint32_t count);

/**
 * qdf_dp_trace_dump_stats() - dump DP Trace stats
 *
 * Return: none
 */
void qdf_dp_trace_dump_stats(void);
void qdf_dp_trace_throttle_live_mode(bool high_bw_request);
typedef void (*tp_qdf_dp_trace_cb)(struct qdf_dp_trace_record_s*,
					uint16_t index, u8 info);
void qdf_dp_display_record(struct qdf_dp_trace_record_s *record,
					uint16_t index, u8 info);
void qdf_dp_display_data_pkt_record(struct qdf_dp_trace_record_s *pRecord,
				uint16_t rec_index, u8 info);
void qdf_dp_trace_ptr(qdf_nbuf_t nbuf, enum QDF_DP_TRACE_ID code,
		uint8_t *data, uint8_t size, uint16_t msdu_id, uint16_t status);

void qdf_dp_trace_data_pkt(qdf_nbuf_t nbuf, enum QDF_DP_TRACE_ID code,
			   uint16_t msdu_id, enum qdf_proto_dir dir);
void qdf_dp_display_ptr_record(struct qdf_dp_trace_record_s *record,
			       uint16_t rec_index, u8 info);
uint8_t qdf_dp_get_proto_bitmap(void);
void
qdf_dp_trace_proto_pkt(enum QDF_DP_TRACE_ID code, uint8_t vdev_id,
		uint8_t *sa, uint8_t *da, enum qdf_proto_type type,
		enum qdf_proto_subtype subtype, enum qdf_proto_dir dir,
		bool print);
void qdf_dp_display_proto_pkt(
			struct qdf_dp_trace_record_s *record,
			uint16_t index, u8 info);
void qdf_dp_trace_disable_live_mode(void);
void qdf_dp_trace_enable_live_mode(void);
void qdf_dp_trace_clear_buffer(void);
void qdf_dp_trace_mgmt_pkt(enum QDF_DP_TRACE_ID code, uint8_t vdev_id,
		enum qdf_proto_type type, enum qdf_proto_subtype subtype);
void qdf_dp_display_mgmt_pkt(struct qdf_dp_trace_record_s *record,
			      uint16_t index, u8 info);
void qdf_dp_display_event_record(struct qdf_dp_trace_record_s *record,
			      uint16_t index, u8 info);
void qdf_dp_trace_record_event(enum QDF_DP_TRACE_ID code, uint8_t vdev_id,
		enum qdf_proto_type type, enum qdf_proto_subtype subtype);
#else
static inline
bool qdf_dp_trace_log_pkt(uint8_t session_id, struct sk_buff *skb,
				enum qdf_proto_dir dir)
{
	return false;
}
static inline
void qdf_dp_trace_init(bool live_mode_config, uint8_t thresh,
				uint16_t time_limit, uint8_t verbosity,
				uint8_t proto_bitmap)
{
}
static inline
void qdf_dp_trace_set_track(qdf_nbuf_t nbuf, enum qdf_proto_dir dir)
{
}
static inline
void qdf_dp_trace_set_value(uint8_t proto_bitmap, uint8_t no_of_records,
			 uint8_t verbosity)
{
}

static inline
void qdf_dp_trace_dump_all(uint32_t count)
{
}

static inline
uint32_t qdf_dpt_get_curr_pos_debugfs(qdf_debugfs_file_t file,
				      enum qdf_dpt_debugfs_state state)
{
}

static inline
QDF_STATUS qdf_dpt_dump_stats_debugfs(qdf_debugfs_file_t file,
				      uint32_t curr_pos)
{
}

static inline
void qdf_dpt_set_value_debugfs(uint8_t proto_bitmap, uint8_t no_of_record,
			    uint8_t verbosity)
{
}

static inline void qdf_dp_trace_dump_stats(void)
{
}

static inline
void qdf_dp_trace_disable_live_mode(void)
{
}

static inline
void qdf_dp_trace_enable_live_mode(void)
{
}

static inline
void qdf_dp_trace_throttle_live_mode(bool high_bw_request)
{
}

static inline
void qdf_dp_trace_clear_buffer(void)
{
}

#endif


void qdf_trace_hex_dump(QDF_MODULE_ID module, QDF_TRACE_LEVEL level,
			void *data, int buf_len);

void qdf_trace_display(void);

void qdf_trace_set_value(QDF_MODULE_ID module, QDF_TRACE_LEVEL level,
			 uint8_t on);

void qdf_trace_set_module_trace_level(QDF_MODULE_ID module, uint32_t level);

void __printf(3, 4) qdf_snprintf(char *str_buffer, unsigned int size,
		  char *str_format, ...);

#define QDF_SNPRINTF qdf_snprintf

#ifdef TSOSEG_DEBUG

static inline void qdf_tso_seg_dbg_bug(char *msg)
{
	qdf_print(msg);
	QDF_BUG(0);
};

static inline
int qdf_tso_seg_dbg_record(struct qdf_tso_seg_elem_t *tsoseg, short id)
{
	int rc = -1;
	unsigned int c;

	qdf_assert(tsoseg);

	if (id == TSOSEG_LOC_ALLOC) {
		c = qdf_atomic_read(&(tsoseg->dbg.cur));
		/* dont crash on the very first alloc on the segment */
		c &= 0x0f;
		/* allow only INIT and FREE ops before ALLOC */
		if (tsoseg->dbg.h[c].id >= id)
			qdf_tso_seg_dbg_bug("Rogue TSO seg alloc");
	}
	c = qdf_atomic_inc_return(&(tsoseg->dbg.cur));

	c &= 0x0f;
	tsoseg->dbg.h[c].ts = qdf_get_log_timestamp();
	tsoseg->dbg.h[c].id = id;
	rc = c;

	return rc;
};

static inline void
qdf_tso_seg_dbg_setowner(struct qdf_tso_seg_elem_t *tsoseg, void *owner)
{
	if (tsoseg != NULL)
		tsoseg->dbg.txdesc = owner;
};

static inline void
qdf_tso_seg_dbg_zero(struct qdf_tso_seg_elem_t *tsoseg)
{
	memset(tsoseg, 0, offsetof(struct qdf_tso_seg_elem_t, dbg));
	return;
};

#else
static inline
int qdf_tso_seg_dbg_record(struct qdf_tso_seg_elem_t *tsoseg, short id)
{
	return 0;
};
static inline void qdf_tso_seg_dbg_bug(char *msg)
{
};
static inline void
qdf_tso_seg_dbg_setowner(struct qdf_tso_seg_elem_t *tsoseg, void *owner)
{
};
static inline int
qdf_tso_seg_dbg_zero(struct qdf_tso_seg_elem_t *tsoseg)
{
	memset(tsoseg, 0, sizeof(struct qdf_tso_seg_elem_t));
	return 0;
};

#endif /* TSOSEG_DEBUG */
#else

#define DPTRACE(x)
#define qdf_trace_hex_dump(x, y, z, q)

#endif /* CONFIG_MCL */

#define QDF_SYMBOL_LEN __QDF_SYMBOL_LEN

/**
 * qdf_sprint_symbol() - prints the name of a symbol into a string buffer
 * @buffer: the string buffer to print into
 * @addr: address of the symbol to lookup and print
 *
 * Return: number of characters printed
 */
int qdf_sprint_symbol(char *buffer, void *addr);

#endif /* __QDF_TRACE_H */
