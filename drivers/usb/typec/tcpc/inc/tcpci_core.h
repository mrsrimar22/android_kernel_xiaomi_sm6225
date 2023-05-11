/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Richtek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_RT_TCPCI_CORE_H__
#define __LINUX_RT_TCPCI_CORE_H__

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/alarmtimer.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>
#include <linux/notifier.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <uapi/linux/sched/types.h>

#include "tcpm.h"
#include "tcpci_timer.h"
#include "tcpci_config.h"

#ifdef CONFIG_USB_POWER_DELIVERY
#include "pd_core.h"
#ifdef CONFIG_USB_PD_WAIT_BC12
#include <linux/power_supply.h>
#endif	/* CONFIG_USB_PD_WAIT_BC12 */
#endif

/* The switch of log message */
#define TYPEC_INFO_ENABLE	1
#define TYPEC_INFO2_ENABLE	1
#define PE_EVENT_DBG_ENABLE	0
#define PE_STATE_INFO_ENABLE	1
#define TCPC_INFO_ENABLE	1
#define TCPC_TIMER_DBG_ENABLE	0
#define PE_INFO_ENABLE		1
#define TCPC_DBG_ENABLE		0
#define TCPC_DBG2_ENABLE	0
#define DPM_INFO_ENABLE		1
#define DPM_INFO2_ENABLE	1
#define DPM_DBG_ENABLE		0
#define PD_ERR_ENABLE		1
#define PE_DBG_ENABLE		0
#define TYPEC_DBG_ENABLE	0

#define DP_INFO_ENABLE		1
#define DP_DBG_ENABLE		0

#define UVDM_INFO_ENABLE	1
#define TCPM_DBG_ENABLE		0

#ifdef CONFIG_USB_PD_ALT_MODE_RTDC
#define DC_INFO_ENABLE		1
#define DC_DBG_ENABLE		0
#endif	/* CONFIG_USB_PD_ALT_MODE_RTDC */

#define TCPC_ENABLE_ANYMSG	\
		((TCPC_DBG_ENABLE) | (TCPC_DBG2_ENABLE) | \
		 (DPM_DBG_ENABLE) | \
		 (PD_ERR_ENABLE) | (PE_INFO_ENABLE) | \
		 (PE_DBG_ENABLE) | (PE_EVENT_DBG_ENABLE) | \
		 (PE_STATE_INFO_ENABLE) | (TCPC_INFO_ENABLE) | \
		 (TCPC_TIMER_DBG_ENABLE) | (TYPEC_DBG_ENABLE) | \
		 (TYPEC_INFO_ENABLE) | \
		 (DP_INFO_ENABLE) | (DP_DBG_ENABLE) | \
		 (UVDM_INFO_ENABLE) | (TCPM_DBG_ENABLE))

/* Disable VDM DBG Msg */
#define PE_STATE_INFO_VDM_DIS	0
#define PE_EVT_INFO_VDM_DIS	0
#define PE_DBG_RESET_VDM_DIS	1

#define PD_WARN_ON(x)	WARN_ON_ONCE(x)

extern struct class *tcpc_class;
struct tcpc_device;

struct tcpc_desc {
	u8 role_def;
	u8 rp_lvl;
	u8 vconn_supply;
	char *name;
	u16 pid;
};

#define TCP_NOTIFY_PRIO_RT_PD_MANAGER	40
#define TCP_NOTIFY_PRIO_XM_PD_ADAPTER	30
#define TCP_NOTIFY_PRIO_CHARGER		20
#define TCP_NOTIFY_PRIO_POLICY_MANAGER	10

/*---------------------------------------------------------------------------*/

#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK
#define CONFIG_TYPEC_NOTIFY_ATTACHWAIT
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK */

#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC
#undef CONFIG_TYPEC_NOTIFY_ATTACHWAIT
#define CONFIG_TYPEC_NOTIFY_ATTACHWAIT
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK */

#ifdef CONFIG_TCPC_FORCE_DISCHARGE_EXT
#define CONFIG_TCPC_EXT_DISCHARGE
#endif	/* CONFIG_TCPC_FORCE_DISCHARGE_EXT */

/*---------------------------------------------------------------------------*/

/* TCPC Power Register Define */
#define TCPC_REG_POWER_STATUS_EXT_VSAFE0V	BIT(15)	/* extend */
#define TCPC_REG_POWER_STATUS_VBUS_PRES		BIT(2)

/* TCPC Alert Register Define */
#define TCPC_REG_ALERT_EXT_RA_DETACH		BIT(21)
#define TCPC_REG_ALERT_EXT_WATCHDOG		BIT(18)
#define TCPC_REG_ALERT_EXT_VBUS_80		BIT(17)
#define TCPC_REG_ALERT_EXT_WAKEUP		BIT(16)

#define TCPC_REG_ALERT_VBUS_DISCNCT		BIT(11)
#define TCPC_REG_ALERT_RX_BUF_OVF		BIT(10)
#define TCPC_REG_ALERT_FAULT			BIT(9)
#define TCPC_REG_ALERT_V_ALARM_LO		BIT(8)
#define TCPC_REG_ALERT_V_ALARM_HI		BIT(7)
#define TCPC_REG_ALERT_TX_SUCCESS		BIT(6)
#define TCPC_REG_ALERT_TX_DISCARDED		BIT(5)
#define TCPC_REG_ALERT_TX_FAILED		BIT(4)
#define TCPC_REG_ALERT_RX_HARD_RST		BIT(3)
#define TCPC_REG_ALERT_RX_STATUS		BIT(2)
#define TCPC_REG_ALERT_POWER_STATUS		BIT(1)
#define TCPC_REG_ALERT_CC_STATUS		BIT(0)

#define TCPC_REG_ALERT_RX_MASK	\
	(TCPC_REG_ALERT_RX_STATUS | TCPC_REG_ALERT_RX_BUF_OVF)

#define TCPC_REG_ALERT_RX_ALL_MASK	\
	(TCPC_REG_ALERT_RX_MASK | TCPC_REG_ALERT_RX_HARD_RST)

#define TCPC_REG_ALERT_HRESET_SUCCESS	\
	(TCPC_REG_ALERT_TX_SUCCESS | TCPC_REG_ALERT_TX_FAILED)

#define TCPC_REG_ALERT_TX_MASK	\
	(TCPC_REG_ALERT_TX_SUCCESS | TCPC_REG_ALERT_TX_FAILED | TCPC_REG_ALERT_TX_DISCARDED)

#define TCPC_REG_ALERT_TXRX_MASK	\
	(TCPC_REG_ALERT_TX_MASK | TCPC_REG_ALERT_RX_MASK)

/* TCPC Behavior Flags */
#define TCPC_FLAGS_RETRY_CRC_DISCARD		BIT(0)
#define TCPC_FLAGS_WAIT_HRESET_COMPLETE		BIT(1)	/* Always true */
#define TCPC_FLAGS_CHECK_CC_STABLE		BIT(2)
#define TCPC_FLAGS_LPM_WAKEUP_WATCHDOG		BIT(3)
#define TCPC_FLAGS_CHECK_RA_DETACH		BIT(4)
#define TCPC_FLAGS_PREFER_LEGACY2		BIT(5)
#define TCPC_FLAGS_DISABLE_LEGACY		BIT(6)

#define TCPC_FLAGS_PD_REV30			BIT(7)

#define TCPC_FLAGS_WATCHDOG_EN			BIT(8)
#define TCPC_FLAGS_WATER_DETECTION		BIT(9)
#define TCPC_FLAGS_CABLE_TYPE_DETECTION		BIT(10)
#define TCPC_FLAGS_VCONN_SAFE5V_ONLY		BIT(11)
#define TCPC_FLAGS_ALERT_V10			BIT(12)

#define TYPEC_CC_PULL(rp_lvl, res)		(((rp_lvl) & 0x03) << 3 | ((res) & 0x07))

enum tcpc_rp_lvl {
	TYPEC_RP_DFT,
	TYPEC_RP_1_5,
	TYPEC_RP_3_0,
	TYPEC_RP_RSV,
};

enum tcpc_cc_pull {
	TYPEC_CC_RA = 0,
	TYPEC_CC_RP = 1,
	TYPEC_CC_RD = 2,
	TYPEC_CC_OPEN = 3,
	TYPEC_CC_DRP = 4,		/* from Rd */

	TYPEC_CC_RP_DFT = TYPEC_CC_PULL(TYPEC_RP_DFT, TYPEC_CC_RP),
	TYPEC_CC_RP_1_5 = TYPEC_CC_PULL(TYPEC_RP_1_5, TYPEC_CC_RP),
	TYPEC_CC_RP_3_0 = TYPEC_CC_PULL(TYPEC_RP_3_0, TYPEC_CC_RP),

	TYPEC_CC_DRP_DFT = TYPEC_CC_PULL(TYPEC_RP_DFT, TYPEC_CC_DRP),
	TYPEC_CC_DRP_1_5 = TYPEC_CC_PULL(TYPEC_RP_1_5, TYPEC_CC_DRP),
	TYPEC_CC_DRP_3_0 = TYPEC_CC_PULL(TYPEC_RP_3_0, TYPEC_CC_DRP),
};

#define TYPEC_CC_PULL_GET_RES(pull)	((pull) & 0x07)
#define TYPEC_CC_PULL_GET_RP_LVL(pull)	(((pull) & 0x18) >> 3)

enum tcpm_rx_cap_type {
	TCPC_RX_CAP_SOP = 1 << 0,
	TCPC_RX_CAP_SOP_PRIME = 1 << 1,
	TCPC_RX_CAP_SOP_PRIME_PRIME = 1 << 2,
	TCPC_RX_CAP_SOP_DEBUG_PRIME = 1 << 3,
	TCPC_RX_CAP_SOP_DEBUG_PRIME_PRIME = 1 << 4,
	TCPC_RX_CAP_HARD_RESET = 1 << 5,
	TCPC_RX_CAP_CABLE_RESET = 1 << 6,
};

struct tcpc_ops {
	int (*init)(struct tcpc_device *tcpc, bool sw_reset);
	int (*init_alert_mask)(struct tcpc_device *tcpc);
	int (*alert_status_clear)(struct tcpc_device *tcpc, u32 mask);
	int (*fault_status_clear)(struct tcpc_device *tcpc, u8 status);
	int (*set_alert_mask)(struct tcpc_device *tcpc, u32 mask);
	int (*get_alert_mask)(struct tcpc_device *tcpc, u32 *mask);
	int (*get_alert_status)(struct tcpc_device *tcpc, u32 *alert);
	int (*get_power_status)(struct tcpc_device *tcpc, u16 *pwr_status);
	int (*get_fault_status)(struct tcpc_device *tcpc, u8 *status);
	int (*get_cc)(struct tcpc_device *tcpc, int *cc1, int *cc2);
	int (*set_cc)(struct tcpc_device *tcpc, int pull);
	int (*set_polarity)(struct tcpc_device *tcpc, int polarity);
	int (*set_low_rp_duty)(struct tcpc_device *tcpc, bool low_rp);
	int (*set_vconn)(struct tcpc_device *tcpc, int enable);
	int (*deinit)(struct tcpc_device *tcpc);
	int (*alert_vendor_defined_handler)(struct tcpc_device *tcpc);

	int (*is_vsafe0v)(struct tcpc_device *tcpc);

#ifdef CONFIG_WATER_DETECTION
	int (*is_water_detected)(struct tcpc_device *tcpc);
	int (*set_water_protection)(struct tcpc_device *tcpc, bool en);
	int (*set_usbid_polling)(struct tcpc_device *tcpc, bool en);
#endif	/* CONFIG_WATER_DETECTION */

#ifdef CONFIG_TCPC_LOW_POWER_MODE
	int (*is_low_power_mode)(struct tcpc_device *tcpc);
	int (*set_low_power_mode)(struct tcpc_device *tcpc, bool en, int pull);
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

	int (*set_watchdog)(struct tcpc_device *tcpc, bool en);

#ifdef CONFIG_TCPC_INTRST_EN
	int (*set_intrst)(struct tcpc_device *tcpc, bool en);
#endif	/* CONFIG_TCPC_INTRST_EN */

#ifdef CONFIG_TYPEC_CAP_AUTO_DISCHARGE
#ifdef CONFIG_TCPC_AUTO_DISCHARGE_IC
	int (*set_auto_discharge)(struct tcpc_device *tcpc, bool en);
#endif	/* CONFIG_TCPC_AUTO_DISCHARGE_IC */
#endif	/* CONFIG_TYPEC_CAP_AUTO_DISCHARGE */

#ifdef CONFIG_USB_POWER_DELIVERY
	int (*set_msg_header)(struct tcpc_device *tcpc,
			      u8 power_role, u8 data_role);
	int (*set_rx_enable)(struct tcpc_device *tcpc, u8 enable);
	int (*get_message)(struct tcpc_device *tcpc, u32 *payload,
			   u16 *head, enum tcpm_transmit_type *type);
	int (*protocol_reset)(struct tcpc_device *tcpc);
	int (*transmit)(struct tcpc_device *tcpc,
			enum tcpm_transmit_type type,
			u16 header, const u32 *data);
	int (*set_bist_test_mode)(struct tcpc_device *tcpc, bool en);
	int (*set_bist_carrier_mode)(struct tcpc_device *tcpc, u8 pattern);

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	int (*retransmit)(struct tcpc_device *tcpc);
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */

#ifdef CONFIG_TYPEC_CAP_FORCE_DISCHARGE
#ifdef CONFIG_TCPC_FORCE_DISCHARGE_IC
	int (*set_force_discharge)(struct tcpc_device *tcpc, bool en, int mv);
#endif	/* CONFIG_TCPC_FORCE_DISCHARGE_IC */
#endif	/* CONFIG_TYPEC_CAP_FORCE_DISCHARGE */

#endif	/* CONFIG_USB_POWER_DELIVERY */
};

#define TCPC_VBUS_SOURCE_0V		(0)
#define TCPC_VBUS_SOURCE_5V		(5000)

#define TCPC_VBUS_SINK_0V		(0)
#define TCPC_VBUS_SINK_5V		(5000)

#define TCPC_LOW_POWER_MODE_RETRY	5

/*
 * Confirm DUT is connected to legacy cable or not
 *	after suupect_counter > this threshold (0 = always check)
 */

#define TCPC_LEGACY_CABLE_SUSPECT_THD		1

/*
 * Try another s/w workaround after retry_counter more than this value
 * Try which soltuion first is determined by tcpc_flags
 */

#define TCPC_LEGACY_CABLE_RETRY_SOLUTION	2

struct tcpc_timer {
	struct hrtimer timer;
	struct tcpc_device *tcpc;
};

#ifdef CONFIG_TCPC_NOTIFICATION_NON_BLOCKING
struct tcpc_event_entry {
	struct tcp_notify tcp_noti;
	u8 type;
	u8 state;
};
#endif

/*
 * tcpc device
 */

#define TCPC_WAKE_LOCK_ATTACH	0
#define TCPC_WAKE_LOCK_POWER	1

struct tcpc_device {
	struct i2c_client *client;
	struct tcpc_ops *ops;
	void *drv_data;
	struct tcpc_desc desc;
	struct device dev;
	unsigned long wake_lock_flags;
	struct wakeup_source *alert_wake_lock;

	/* For tcpc timer & event */
	u32 timer_handle_index;
	struct tcpc_timer tcpc_timer[PD_TIMER_NR];

	struct alarm wake_up_timer;
	struct kthread_worker *wake_up_worker;
	struct kthread_work wake_up_work;
	struct wakeup_source *wakeup_wake_lock;

	struct srcu_notifier_head late_sync_notifier;

	struct mutex access_lock;
	struct mutex typec_lock;
	struct mutex timer_lock;
	spinlock_t timer_tick_lock;
	atomic_t pending_event;
	u64 timer_tick;
	wait_queue_head_t event_wait_que;
	wait_queue_head_t timer_wait_que;
	struct task_struct *event_task;
	struct task_struct *timer_task;

	struct workqueue_struct *evt_wq;
	struct srcu_notifier_head evt_nh[TCP_NOTIFY_IDX_NR];
	struct list_head mr_list;
	struct mutex mr_lock;

	/* For TCPC TypeC */
	u8 typec_state;
	u8 typec_role;
	u8 typec_role_new;
	u8 typec_attach_old;
	u8 typec_attach_new;
	u8 typec_local_cc;
	u8 typec_local_rp_level;
	u8 typec_remote_cc[2];
	u8 typec_remote_rp_level;
	u8 typec_wait_ps_change;
	bool typec_polarity;
	bool typec_drp_try_timeout;
	bool typec_lpm;
	bool typec_cable_only;
	bool typec_power_ctrl;
	bool typec_watchdog;
	bool typec_reach_vsafe0v;
	bool typec_is_attached_src;

	int typec_usb_sink_curr;

#ifdef CONFIG_TYPEC_CAP_CUSTOM_HV
	bool typec_during_custom_hv;
#endif	/* CONFIG_TYPEC_CAP_CUSTOM_HV */

	u8 typec_lpm_pull;
	u8 typec_lpm_retry;

#ifdef CONFIG_TYPEC_WAKEUP_ONCE_LOW_DUTY
	bool typec_wakeup_once;
	bool typec_low_rp_duty_cntdown;
#endif	/* CONFIG_TYPEC_WAKEUP_ONCE_LOW_DUTY */

#ifdef CONFIG_TYPEC_CHECK_LEGACY_CABLE
	u8 typec_legacy_cable;
#if TCPC_LEGACY_CABLE_SUSPECT_THD
	u8 typec_legacy_cable_suspect;
#endif	/* TCPC_LEGACY_CABLE_SUSPECT_THD */

#ifdef CONFIG_TYPEC_CHECK_LEGACY_CABLE2
	u8 typec_legacy_retry_wk;
#endif	/* CONFIG_TYPEC_CHECK_LEGACY_CABLE2 */
#endif	/* CONFIG_TYPEC_CHECK_LEGACY_CABLE */

#ifdef CONFIG_TYPEC_CAP_ROLE_SWAP
	u8 typec_during_role_swap;
#endif	/* CONFIG_TYPEC_CAP_ROLE_SWAP */

#ifdef CONFIG_TYPEC_CAP_AUTO_DISCHARGE
#ifdef CONFIG_TCPC_AUTO_DISCHARGE_IC
	bool typec_auto_discharge;
#endif	/* CONFIG_TCPC_AUTO_DISCHARGE_IC */
#endif	/* CONFIG_TYPEC_CAP_AUTO_DISCHARGE */

#ifdef CONFIG_TCPC_EXT_DISCHARGE
	bool typec_ext_discharge;
#endif	/* CONFIG_TCPC_EXT_DISCHARGE */

#ifdef CONFIG_TCPC_VCONN_SUPPLY_MODE
	u8 tcpc_vconn_supply;
#endif	/* CONFIG_TCPC_VCONN_SUPPLY_MODE */

#ifdef CONFIG_TCPC_SOURCE_VCONN
	bool tcpc_source_vconn;
#endif	/* CONFIG_TCPC_SOURCE_VCONN */

	u32 tcpc_flags;

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	struct dual_role_phy_instance *dr_usb;
	struct dual_role_phy_desc *dual_role_desc;
	u8 dual_role_supported_modes;
	u8 dual_role_mode;
	u8 dual_role_pr;
	u8 dual_role_dr;
	u8 dual_role_vconn;
#endif	/* CONFIG_DUAL_ROLE_USB_INTF */

#ifdef CONFIG_USB_POWER_DELIVERY
	/* Event */
	u8 pd_event_count;
	u8 pd_event_head_index;
	u8 pd_msg_buffer_allocated;

	u8 pd_last_vdm_msg_id;
	bool pd_pending_vdm_event;
	bool pd_pending_vdm_reset;
	bool pd_pending_vdm_good_crc;
	bool pd_pending_vdm_attention;
	bool pd_postpone_vdm_timeout;

	struct pd_msg pd_attention_vdm_msg;
	struct pd_event pd_vdm_event;

	struct pd_msg pd_msg_buffer[PD_MSG_BUF_SIZE];
	struct pd_event pd_event_ring_buffer[PD_EVENT_BUF_SIZE];

	u8 tcp_event_count;
	u8 tcp_event_head_index;
	struct tcp_dpm_event tcp_event_ring_buffer[TCP_EVENT_BUF_SIZE];

	bool pd_pe_running;
	bool pd_wait_pe_idle;
	bool pd_hard_reset_event_pending;
	bool pd_wait_hard_reset_complete;
	bool pd_wait_pr_swap_complete;
	bool pd_ping_event_pending;
	u8 pd_bist_mode;
	u8 pd_transmit_state;
	u8 pd_wait_vbus_once;

#ifdef CONFIG_USB_PD_DIRECT_CHARGE
	bool pd_during_direct_charge;
#endif	/* CONFIG_USB_PD_DIRECT_CHARGE */

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	bool pd_discard_pending;
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */

#ifdef CONFIG_TYPEC_CAP_FORCE_DISCHARGE
#ifdef CONFIG_TCPC_FORCE_DISCHARGE_IC
	bool pd_force_discharge;
#endif	/* CONFIG_TCPC_FORCE_DISCHARGE_IC */
#endif	/* CONFIG_TYPEC_CAP_FORCE_DISCHARGE */

#ifdef CONFIG_USB_PD_REV30
	u8 pd_retry_count;
#endif	/* CONFIG_USB_PD_REV30 */

#ifdef CONFIG_USB_PD_DISABLE_PE
	bool disable_pe; /* typec only */
#endif	/* CONFIG_USB_PD_DISABLE_PE */

	struct pd_port pd_port;
#ifdef CONFIG_USB_PD_REV30
	struct notifier_block bat_nb;
	struct work_struct bat_update_work;
	struct power_supply *bat_psy;
	u8 charging_status;
	int bat_soc;
#endif	/* CONFIG_USB_PD_REV30 */
#ifdef CONFIG_USB_PD_WAIT_BC12
	u8 pd_wait_bc12_count;
	struct power_supply *usb_psy;
#endif	/* CONFIG_USB_PD_WAIT_BC12 */
#endif	/* CONFIG_USB_POWER_DELIVERY */
	u8 vbus_level;
	bool vbus_safe0v;
	bool vbus_present;
	bool irq_enabled;
	bool pd_inited_flag;

	/* TypeC Shield Protection */
#ifdef CONFIG_WATER_DETECTION
	int usbid_calib;
#endif	/* CONFIG_WATER_DETECTION */
#ifdef CONFIG_CABLE_TYPE_DETECTION
	enum tcpc_cable_type typec_cable_type;
#endif	/* CONFIG_CABLE_TYPE_DETECTION */

#ifdef CONFIG_TCPC_NOTIFICATION_NON_BLOCKING
	struct kfifo notify_fifo;
	struct work_struct notify_work;
	spinlock_t fifo_lock;
#endif

	/* Resource flags - see TCPC_RES_*_INIT below */
	unsigned long res_owner_flags;
};

/*
 * Lifecycle tracking bits: set when the core initialises a
 * non-idempotent resource; cleared (via test_and_clear_bit) in
 * tcpc_device_release() to ensure each teardown runs exactly once.
 */
#define TCPC_RES_EVENT_INIT		BIT(0)
#define TCPC_RES_TIMER_INIT		BIT(1)
#define TCPC_RES_PD_CORE_INIT		BIT(2)

#define to_tcpc_device(obj) container_of(obj, struct tcpc_device, dev)

#ifdef CONFIG_USB_POWER_DELIVERY
static inline u8 pd_get_rev(struct pd_port *pd_port, u8 sop_type)
{
	u8 pd_rev = PD_REV20;
#ifdef CONFIG_USB_PD_REV30_SYNC_SPEC_REV
	struct pe_data *pe_data = &pd_port->pe_data;
	struct tcpc_device *tcpc = pd_port->tcpc;

	if (sop_type == TCPC_TX_SOP) {
		pd_rev = pd_port->pd_revision[0];
	} else {
		if (pe_data->explicit_contract || pe_data->cable_rev_discovered)
			pd_rev = pd_port->pd_revision[1];
		else if (tcpc->tcpc_flags & TCPC_FLAGS_PD_REV30)
			pd_rev = PD_REV30;
	}
#endif	/* CONFIG_USB_PD_REV30_SYNC_SPEC_REV */

	return pd_rev;
}

static inline bool pd_check_rev30(struct pd_port *pd_port)
{
	return pd_get_rev(pd_port, TCPC_TX_SOP) >= PD_REV30;
}
#endif	/* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_PD_DBG_INFO
#define __RT_DBG_INFO	pd_dbg_info
#else
#define __RT_DBG_INFO	pr_info
#endif	/* CONFIG_PD_DBG_INFO */

#ifdef CONFIG_TCPC_LOG_WITH_PORT_NAME
#define RT_DBG_INFO(fmt, args...)	\
	__RT_DBG_INFO(fmt, tcpc->desc.name, ##args)
#else
#define RT_DBG_INFO(fmt, args...)	__RT_DBG_INFO(fmt, ##args)
#endif	/* CONFIG_TCPC_LOG_WITH_PORT_NAME */

#if TYPEC_DBG_ENABLE
#define TYPEC_DBG(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TYPEC: " fmt, ##args)
#else
#define TYPEC_DBG(fmt, args...)		no_printk(fmt, ##args)
#endif	/* TYPEC_DBG_ENABLE */

#if TYPEC_INFO_ENABLE
#define TYPEC_INFO(fmt, args...)	\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TYPEC: " fmt, ##args)
#else
#define TYPEC_INFO(fmt, args...)	no_printk(fmt, ##args)
#endif	/* TYPEC_INFO_ENABLE */

#if TYPEC_INFO2_ENABLE
#define TYPEC_INFO2(fmt, args...)	\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TYPEC: " fmt, ##args)
#else
#define TYPEC_INFO2(fmt, args...)	no_printk(fmt, ##args)
#endif	/* TYPEC_INFO2_ENABLE */

#if TCPC_INFO_ENABLE
#define TCPC_INFO(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TCPC: " fmt, ##args)
#else
#define TCPC_INFO(fmt, args...)		no_printk(fmt, ##args)
#endif	/* TCPC_INFO_ENABLE */

#if TCPC_DBG_ENABLE
#define TCPC_DBG(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TCPC: " fmt, ##args)
#else
#define TCPC_DBG(fmt, args...)		no_printk(fmt, ##args)
#endif	/* TCPC_DBG_ENABLE */

#if TCPC_DBG2_ENABLE
#define TCPC_DBG2(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TCPC: " fmt, ##args)
#else
#define TCPC_DBG2(fmt, args...)		no_printk(fmt, ##args)
#endif	/* TCPC_DBG2_ENABLE */

#if TCPC_TIMER_DBG_ENABLE
#define TCPC_TIMER_DBG(fmt, args...)	\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TIMER: " fmt, ##args)
#else
#define TCPC_TIMER_DBG(fmt, args...)	no_printk(fmt, ##args)
#endif	/* TCPC_TIMER_DBG_ENABLE */

#define TCPC_ERR(fmt, args...)	\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TCPC-ERR: " fmt, ##args)

#define DP_ERR(fmt, args...)	\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DP-ERR: " fmt, ##args)

#if DPM_INFO_ENABLE
#define DPM_INFO(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DPM: " fmt, ##args)
#else
#define DPM_INFO(fmt, args...)		no_printk(fmt, ##args)
#endif	/* DPM_INFO_ENABLE */

#if DPM_INFO2_ENABLE
#define DPM_INFO2(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DPM: " fmt, ##args)
#else
#define DPM_INFO2(fmt, args...)		no_printk(fmt, ##args)
#endif	/* DPM_INFO2_ENABLE */

#if DPM_DBG_ENABLE
#define DPM_DBG(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DPM: " fmt, ##args)
#else
#define DPM_DBG(fmt, args...)		no_printk(fmt, ##args)
#endif	/* DPM_DBG_ENABLE */

#if PD_ERR_ENABLE
#define PD_ERR(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "PD-ERR: " fmt, ##args)
#else
#define PD_ERR(fmt, args...)		no_printk(fmt, ##args)
#endif	/* PD_ERR_ENABLE */

#if PE_INFO_ENABLE
#define PE_INFO(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "PE: " fmt, ##args)
#else
#define PE_INFO(fmt, args...)		no_printk(fmt, ##args)
#endif	/* PE_INFO_ENABLE */

#if PE_EVENT_DBG_ENABLE
#define PE_EVT_INFO(fmt, args...)	\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "PE-EVT: " fmt, ##args)
#else
#define PE_EVT_INFO(fmt, args...)	no_printk(fmt, ##args)
#endif	/* PE_EVENT_DBG_ENABLE */

#if PE_DBG_ENABLE
#define PE_DBG(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "PE: " fmt, ##args)
#else
#define PE_DBG(fmt, args...)		no_printk(fmt, ##args)
#endif	/* PE_DBG_ENABLE */

#if PE_STATE_INFO_ENABLE
#define PE_STATE_INFO(fmt, args...)	\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "PE: " fmt, ##args)
#else
#define PE_STATE_INFO(fmt, args...)	no_printk(fmt, ##args)
#endif	/* PE_STATE_INFO_ENABLE */

#if DP_INFO_ENABLE
#define DP_INFO(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DP: " fmt, ##args)
#else
#define DP_INFO(fmt, args...)		no_printk(fmt, ##args)
#endif	/* DP_INFO_ENABLE */

#if DP_DBG_ENABLE
#define DP_DBG(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DP: " fmt, ##args)
#else
#define DP_DBG(fmt, args...)		no_printk(fmt, ##args)
#endif	/* DP_DBG_ENABLE */

#if UVDM_INFO_ENABLE
#define UVDM_INFO(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "UVDM: " fmt, ##args)
#else
#define UVDM_INFO(fmt, args...)		no_printk(fmt, ##args)
#endif	/* UVDM_INFO_ENABLE */

#if TCPM_DBG_ENABLE
#define TCPM_DBG(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "TCPM: " fmt, ##args)
#else
#define TCPM_DBG(fmt, args...)		no_printk(fmt, ##args)
#endif	/* TCPM_DBG_ENABLE */

#ifdef CONFIG_USB_PD_ALT_MODE_RTDC

#if DC_INFO_ENABLE
#define DC_INFO(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DC> " fmt, ##args)
#else
#define DC_INFO(fmt, args...)		no_printk(fmt, ##args)
#endif	/* DC_INFO_ENABLE */

#if DC_DBG_ENABLE
#define DC_DBG(fmt, args...)		\
	RT_DBG_INFO(CONFIG_TCPC_DBG_PRESTR "DC> " fmt, ##args)
#else
#define DC_DBG(fmt, args...)		no_printk(fmt, ##args)
#endif	/* DC_DBG_ENABLE */

#endif	/* CONFIG_USB_PD_ALT_MODE_RTDC */
#endif	/* __LINUX_RT_TCPCI_CORE_H__ */
