// SPDX-License-Identifier: GPL-2.0-only
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

#include "inc/tcpci.h"
#include <linux/time.h>
#include <linux/slab.h>

#define TCPC_NOTIFY_OVERTIME	(20) /* ms */

#ifdef CONFIG_TCPC_NOTIFICATION_NON_BLOCKING
static void tcpc_notify_func(struct work_struct *work)
{
	struct tcpc_device *tcpc = container_of(work, struct tcpc_device, notify_work);
	struct tcpc_event_entry entry;
	unsigned long flags;
#ifdef CONFIG_PD_BEGUG_ON
	struct timeval begin, end;
	int timeval = 0;
#endif

	while (kfifo_len(&tcpc->notify_fifo) >= sizeof(struct tcpc_event_entry)) {
		spin_lock_irqsave(&tcpc->fifo_lock, flags);
		kfifo_out(&tcpc->notify_fifo, &entry, sizeof(struct tcpc_event_entry));
		spin_unlock_irqrestore(&tcpc->fifo_lock, flags);
#ifdef CONFIG_PD_BEGUG_ON
		do_gettimeofday(&begin);
		srcu_notifier_call_chain(&tcpc->evt_nh[entry.type], entry.state, &entry.tcp_noti);
		do_gettimeofday(&end);
		timeval = (timeval_to_ns(end) - timeval_to_ns(begin)) / 1000 / 1000;
		PD_WARN_ON(timeval > TCPC_NOTIFY_OVERTIME);
#else
		srcu_notifier_call_chain(&tcpc->evt_nh[entry.type], entry.state, &entry.tcp_noti);
#endif
	}
}

static int tcpc_check_notify_time(struct tcpc_device *tcpc,
				  struct tcp_notify *tcp_noti, u8 type, u8 state)
{
	struct tcpc_event_entry entry;
	unsigned long flags;
	int ret = 0;

	entry.tcp_noti = *tcp_noti;
	entry.type = type;
	entry.state = state;

	spin_lock_irqsave(&tcpc->fifo_lock, flags);
	if (kfifo_avail(&tcpc->notify_fifo) >= sizeof(struct tcpc_event_entry))
		kfifo_in(&tcpc->notify_fifo, &entry, sizeof(struct tcpc_event_entry));
	else
		ret = -ENOSPC;
	spin_unlock_irqrestore(&tcpc->fifo_lock, flags);

	if (ret)
		return ret;

	queue_work(tcpc->evt_wq, &tcpc->notify_work);
	return 0;
}
#else
static int tcpc_check_notify_time(struct tcpc_device *tcpc,
				  struct tcp_notify *tcp_noti, u8 type, u8 state)
{
#ifdef CONFIG_PD_BEGUG_ON
	struct timeval begin, end;
	int timeval = 0;
	int ret;

	do_gettimeofday(&begin);
	ret = srcu_notifier_call_chain(&tcpc->evt_nh[type], state, tcp_noti);
	do_gettimeofday(&end);
	timeval = (timeval_to_ns(end) - timeval_to_ns(begin)) / 1000 / 1000;
	PD_WARN_ON(timeval > TCPC_NOTIFY_OVERTIME);
	return ret;
#else
	return srcu_notifier_call_chain(&tcpc->evt_nh[type], state, tcp_noti);
#endif
}
#endif	/* CONFIG_TCPC_NOTIFICATION_NON_BLOCKING */

int tcpci_notify_init(struct tcpc_device *tcpc)
{
#ifdef CONFIG_TCPC_NOTIFICATION_NON_BLOCKING
	int ret;

	tcpc->evt_wq = alloc_ordered_workqueue("%s", WQ_HIGHPRI | WQ_MEM_RECLAIM,
					       tcpc->desc.name);
	if (!tcpc->evt_wq) {
		dev_err(&tcpc->dev, "%s: create notify workqueue fail\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&tcpc->notify_work, tcpc_notify_func);

	ret = kfifo_alloc(&tcpc->notify_fifo,
			  64 * sizeof(struct tcpc_event_entry),
			  GFP_KERNEL);
	if (ret) {
		destroy_workqueue(tcpc->evt_wq);
		tcpc->evt_wq = NULL;
		return ret;
	}
#endif	/* CONFIG_TCPC_NOTIFICATION_NON_BLOCKING */
	return 0;
}

void tcpci_notify_deinit(struct tcpc_device *tcpc)
{
#ifdef CONFIG_TCPC_NOTIFICATION_NON_BLOCKING
	if (tcpc->evt_wq) {
		cancel_work_sync(&tcpc->notify_work);
		destroy_workqueue(tcpc->evt_wq);
		tcpc->evt_wq = NULL;
	}
	kfifo_free(&tcpc->notify_fifo);
#endif	/* CONFIG_TCPC_NOTIFICATION_NON_BLOCKING */
}

int tcpci_check_vbus_valid_from_ic(struct tcpc_device *tcpc)
{
	u16 power_status;
	int vbus_level = tcpc->vbus_level;

	if (!tcpci_get_power_status(tcpc, &power_status)) {
		if (vbus_level != tcpc->vbus_level)
			TCPC_INFO("ps_changed %d -> %d\n",
				  vbus_level, tcpc->vbus_level);
	}

	return tcpci_check_vbus_valid(tcpc);
}

bool tcpci_check_vsafe0v(struct tcpc_device *tcpc)
{
	return tcpc->vbus_level == TCPC_VBUS_SAFE0V;
}

int tcpci_alert_status_clear(struct tcpc_device *tcpc, u32 mask)
{
	PD_WARN_ON(!tcpc->ops->alert_status_clear);

	return tcpc->ops->alert_status_clear(tcpc, mask);
}
EXPORT_SYMBOL(tcpci_alert_status_clear);

int tcpci_fault_status_clear(struct tcpc_device *tcpc, u8 status)
{
	PD_WARN_ON(!tcpc->ops->fault_status_clear);

	return tcpc->ops->fault_status_clear(tcpc, status);
}

int tcpci_set_alert_mask(struct tcpc_device *tcpc, u32 mask)
{
	if (tcpc->ops->set_alert_mask)
		return tcpc->ops->set_alert_mask(tcpc, mask);

	return 0;
}

int tcpci_get_alert_mask(struct tcpc_device *tcpc, u32 *mask)
{
	PD_WARN_ON(!tcpc->ops->get_alert_mask);

	return tcpc->ops->get_alert_mask(tcpc, mask);
}

int tcpci_get_alert_status(struct tcpc_device *tcpc, u32 *alert)
{
	PD_WARN_ON(!tcpc->ops->get_alert_status);

	return tcpc->ops->get_alert_status(tcpc, alert);
}

int tcpci_get_fault_status(struct tcpc_device *tcpc, u8 *fault)
{
	if (tcpc->ops->get_fault_status)
		return tcpc->ops->get_fault_status(tcpc, fault);

	*fault = 0;
	return 0;
}

int tcpci_get_power_status(struct tcpc_device *tcpc, u16 *pw_status)
{
	int ret;

	PD_WARN_ON(!tcpc->ops->get_power_status);

	ret = tcpc->ops->get_power_status(tcpc, pw_status);
	if (ret < 0)
		return ret;

	tcpci_vbus_level_init(tcpc, *pw_status);
	return 0;
}

int tcpci_init(struct tcpc_device *tcpc, bool sw_reset)
{
	u16 power_status;
	int ret;

	PD_WARN_ON(!tcpc->ops->init);

	ret = tcpc->ops->init(tcpc, sw_reset);
	if (ret < 0)
		return ret;

	return tcpci_get_power_status(tcpc, &power_status);
}
EXPORT_SYMBOL_GPL(tcpci_init);

int tcpci_init_alert_mask(struct tcpc_device *tcpc)
{
	if (tcpc->ops->init_alert_mask)
		return tcpc->ops->init_alert_mask(tcpc);

	return 0;
}

int tcpci_get_cc(struct tcpc_device *tcpc)
{
	int ret, cc1, cc2;

	PD_WARN_ON(!tcpc->ops->get_cc);

	ret = tcpc->ops->get_cc(tcpc, &cc1, &cc2);
	if (ret < 0)
		return ret;

	if (cc1 == tcpc->typec_remote_cc[0] &&
	    cc2 == tcpc->typec_remote_cc[1])
		return 0;

	tcpc->typec_remote_cc[0] = cc1;
	tcpc->typec_remote_cc[1] = cc2;

	return 1;
}

int tcpci_set_cc(struct tcpc_device *tcpc, int pull)
{
	bool apply_local_rp = false;

#ifdef CONFIG_TYPEC_CHECK_LEGACY_CABLE
	if (pull == TYPEC_CC_DRP && tcpc->typec_legacy_cable) {
		pull = TYPEC_CC_RP_1_5;

#ifdef CONFIG_TYPEC_CHECK_LEGACY_CABLE2
		if (tcpc->typec_legacy_cable == 2)
			pull = TYPEC_CC_RP;
		else if (tcpc->typec_legacy_retry_wk > 1)
			pull = TYPEC_CC_RP_3_0;
#endif	/* CONFIG_TYPEC_CHECK_LEGACY_CABLE2 */

		TCPC_DBG2("LC->Toggling (%d)\n", pull);
		return __tcpci_set_cc(tcpc, pull);
	}

	if (!tcpc->typec_legacy_cable)
		apply_local_rp = true;
#else
	apply_local_rp = true;
#endif	/* CONFIG_TYPEC_CHECK_LEGACY_CABLE */

#ifdef CONFIG_TYPEC_LEGACY3_ALWAYS_LOCAL_RP
	if (apply_local_rp) {
		u8 rp_lvl = TYPEC_CC_PULL_GET_RP_LVL(pull);
		u8 res = TYPEC_CC_PULL_GET_RES(pull);

		pull = TYPEC_CC_PULL(rp_lvl == TYPEC_RP_DFT ?
				     tcpc->typec_local_rp_level : rp_lvl, res);
	}
#endif	/* CONFIG_TYPEC_LEGACY3_ALWAYS_LOCAL_RP */

	return __tcpci_set_cc(tcpc, pull);
}

int tcpci_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	PD_WARN_ON(!tcpc->ops->set_polarity);

	return tcpc->ops->set_polarity(tcpc, polarity);
}

int tcpci_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
#ifdef CONFIG_TYPEC_CAP_LOW_RP_DUTY
	if (low_rp)
		TCPC_INFO("low_rp_duty\n");

	if (tcpc->ops->set_low_rp_duty)
		return tcpc->ops->set_low_rp_duty(tcpc, low_rp);
#endif	/* CONFIG_TYPEC_CAP_LOW_RP_DUTY */

	return 0;
}

int tcpci_set_vconn(struct tcpc_device *tcpc, int enable)
{
#ifdef CONFIG_TCPC_SOURCE_VCONN
	struct tcp_notify tcp_noti;

	if (tcpc->tcpc_source_vconn == enable)
		return 0;

	tcpc->tcpc_source_vconn = enable;

	tcp_noti.en_state.en = enable != 0;
	tcpc_check_notify_time(tcpc, &tcp_noti,
			       TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_SOURCE_VCONN);

	if (tcpc->ops->set_vconn)
		return tcpc->ops->set_vconn(tcpc, enable);
#endif	/* CONFIG_TCPC_SOURCE_VCONN */

	return 0;
}

int tcpci_is_low_power_mode(struct tcpc_device *tcpc)
{
#ifdef CONFIG_TCPC_LOW_POWER_MODE
	if (tcpc->ops->is_low_power_mode)
		return tcpc->ops->is_low_power_mode(tcpc);
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

	return 1;
}

int tcpci_set_low_power_mode(struct tcpc_device *tcpc, bool en, int pull)
{
#ifdef CONFIG_TCPC_LOW_POWER_MODE
	if (tcpc->ops->set_low_power_mode)
		return tcpc->ops->set_low_power_mode(tcpc, en, pull);
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

	return 0;
}
EXPORT_SYMBOL(tcpci_set_low_power_mode);

int tcpci_set_watchdog(struct tcpc_device *tcpc, bool en)
{
	if ((tcpc->tcpc_flags & TCPC_FLAGS_WATCHDOG_EN) &&
	    tcpc->ops->set_watchdog)
		return tcpc->ops->set_watchdog(tcpc, en);

	return 0;
}

int tcpci_alert_vendor_defined_handler(struct tcpc_device *tcpc)
{
	if (tcpc->ops->alert_vendor_defined_handler)
		return tcpc->ops->alert_vendor_defined_handler(tcpc);

	return 0;
}

int tcpci_is_vsafe0v(struct tcpc_device *tcpc)
{
	if (tcpc->ops->is_vsafe0v)
		return tcpc->ops->is_vsafe0v(tcpc);

	return -ENOTSUPP;
}

#ifdef CONFIG_WATER_DETECTION
int tcpci_is_water_detected(struct tcpc_device *tcpc)
{
	if (tcpc->ops->is_water_detected)
		return tcpc->ops->is_water_detected(tcpc);

	return 0;
}

int tcpci_set_water_protection(struct tcpc_device *tcpc, bool en)
{
	if (tcpc->ops->set_water_protection)
		return tcpc->ops->set_water_protection(tcpc, en);

	return 0;
}

int tcpci_set_usbid_polling(struct tcpc_device *tcpc, bool en)
{
	if (tcpc->ops->set_usbid_polling)
		return tcpc->ops->set_usbid_polling(tcpc, en);

	return 0;
}

int tcpci_notify_wd_status(struct tcpc_device *tcpc, bool water_detected)
{
	struct tcp_notify tcp_noti;

	tcp_noti.wd_status.water_detected = water_detected;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				      TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_WD_STATUS);
}
#endif	/* CONFIG_WATER_DETECTION */

#ifdef CONFIG_CABLE_TYPE_DETECTION
int tcpci_notify_cable_type(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	tcp_noti.cable_type.type = tcpc->typec_cable_type;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				      TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_CABLE_TYPE);
}
#endif	/* CONFIG_CABLE_TYPE_DETECTION */

#ifdef CONFIG_USB_POWER_DELIVERY

int tcpci_set_msg_header(struct tcpc_device *tcpc,
			 u8 power_role, u8 data_role)
{
	PD_WARN_ON(!tcpc->ops->set_msg_header);

	return tcpc->ops->set_msg_header(tcpc, power_role, data_role);
}

int tcpci_set_rx_enable(struct tcpc_device *tcpc, u8 enable)
{
	PD_WARN_ON(!tcpc->ops->set_rx_enable);

	return tcpc->ops->set_rx_enable(tcpc, enable);
}

int tcpci_protocol_reset(struct tcpc_device *tcpc)
{
	if (tcpc->ops->protocol_reset)
		return tcpc->ops->protocol_reset(tcpc);

	return 0;
}

int tcpci_get_message(struct tcpc_device *tcpc,
		      u32 *payload, u16 *head,
		      enum tcpm_transmit_type *type)
{
	PD_WARN_ON(!tcpc->ops->get_message);

	return tcpc->ops->get_message(tcpc, payload, head, type);
}

int tcpci_transmit(struct tcpc_device *tcpc,
		   enum tcpm_transmit_type type,
		   u16 header, const u32 *data)
{
	PD_WARN_ON(!tcpc->ops->transmit);

	return tcpc->ops->transmit(tcpc, type, header, data);
}

int tcpci_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	if (tcpc->ops->set_bist_test_mode)
		return tcpc->ops->set_bist_test_mode(tcpc, en);

	return 0;
}

int tcpci_set_bist_carrier_mode(struct tcpc_device *tcpc, u8 pattern)
{
	PD_WARN_ON(!tcpc->ops->set_bist_carrier_mode);

	if (pattern)	/* wait for GoodCRC */
		usleep_range(250, 300);

	return tcpc->ops->set_bist_carrier_mode(tcpc, pattern);
}

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
int tcpci_retransmit(struct tcpc_device *tcpc)
{
	PD_WARN_ON(!tcpc->ops->retransmit);

	return tcpc->ops->retransmit(tcpc);
}
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */
#endif	/* CONFIG_USB_POWER_DELIVERY */

int tcpci_notify_typec_state(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	tcp_noti.typec_state.polarity = tcpc->typec_polarity;
	tcp_noti.typec_state.old_state = tcpc->typec_attach_old;
	tcp_noti.typec_state.new_state = tcpc->typec_attach_new;
	tcp_noti.typec_state.rp_level = tcpc->typec_remote_rp_level;
	tcp_noti.typec_state.local_rp_level = tcpc->typec_local_rp_level;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_USB, TCP_NOTIFY_TYPEC_STATE);
}

int tcpci_notify_role_swap(struct tcpc_device *tcpc, u8 event, u8 role)
{
	struct tcp_notify tcp_noti;

	tcp_noti.swap_state.new_role = role;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MISC, event);
}

int tcpci_notify_pd_state(struct tcpc_device *tcpc, u8 connect)
{
	struct tcp_notify tcp_noti;

	tcp_noti.pd_state.connected = connect;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_USB, TCP_NOTIFY_PD_STATE);
}

int tcpci_set_intrst(struct tcpc_device *tcpc, bool en)
{
#ifdef CONFIG_TCPC_INTRST_EN
	if (tcpc->ops->set_intrst)
		tcpc->ops->set_intrst(tcpc, en);
#endif	/* CONFIG_TCPC_INTRST_EN */

	return 0;
}

int tcpci_enable_watchdog(struct tcpc_device *tcpc, bool en)
{
	if (!(tcpc->tcpc_flags & TCPC_FLAGS_WATCHDOG_EN))
		return 0;

	TCPC_DBG2("enable_WG: %d\n", en);

	if (tcpc->typec_watchdog == en)
		return 0;

	mutex_lock(&tcpc->access_lock);
	tcpc->typec_watchdog = en;

	if (tcpc->ops->set_watchdog)
		tcpc->ops->set_watchdog(tcpc, en);

#ifdef CONFIG_TCPC_INTRST_EN
	if (!en ||
	    (tcpc->alert_wake_lock && tcpc->alert_wake_lock->active))
		tcpci_set_intrst(tcpc, en);
#endif	/* CONFIG_TCPC_INTRST_EN */

	mutex_unlock(&tcpc->access_lock);

	return 0;
}

int tcpci_source_vbus(struct tcpc_device *tcpc,
		      u8 type, int mv, int ma)
{
	struct tcp_notify tcp_noti;

#ifdef CONFIG_USB_POWER_DELIVERY
	if (type >= TCP_VBUS_CTRL_PD &&
	    tcpc->pd_port.pe_data.pd_prev_connected)
		type |= TCP_VBUS_CTRL_PD_DETECT;
#endif	/* CONFIG_USB_POWER_DELIVERY */

	if (ma < 0) {
		if (mv != 0) {
			switch (tcpc->typec_local_rp_level) {
			case TYPEC_RP_3_0:
				ma = 3000;
				break;
			case TYPEC_RP_1_5:
				ma = 1500;
				break;
			case TYPEC_RP_DFT:
			default:
				ma = CONFIG_TYPEC_SRC_CURR_DFT;
				break;
			}
		} else {
			ma = 0;
		}
	}

	tcp_noti.vbus_state.ma = ma;
	tcp_noti.vbus_state.mv = mv;
	tcp_noti.vbus_state.type = type;

	tcpci_enable_watchdog(tcpc, mv != 0);
	TCPC_DBG("source_vbus: %d mV, %d mA\n", mv, ma);
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_SOURCE_VBUS);
}

int tcpci_sink_vbus(struct tcpc_device *tcpc,
		    u8 type, int mv, int ma)
{
	struct tcp_notify tcp_noti;

#ifdef CONFIG_USB_POWER_DELIVERY
	if (type >= TCP_VBUS_CTRL_PD &&
	    tcpc->pd_port.pe_data.pd_prev_connected)
		type |= TCP_VBUS_CTRL_PD_DETECT;
#endif	/* CONFIG_USB_POWER_DELIVERY */

	if (ma < 0) {
		if (mv != 0) {
			switch (tcpc->typec_remote_rp_level) {
			case TYPEC_CC_VOLT_SNK_1_5:
				ma = 1500;
				break;
			case TYPEC_CC_VOLT_SNK_3_0:
				ma = 3000;
				break;
			default:
			case TYPEC_CC_VOLT_SNK_DFT:
				ma = tcpc->typec_usb_sink_curr;
				break;
			}
#if CONFIG_TYPEC_SNK_CURR_LIMIT > 0
		if (ma > CONFIG_TYPEC_SNK_CURR_LIMIT)
			ma = CONFIG_TYPEC_SNK_CURR_LIMIT;
#endif	/* CONFIG_TYPEC_SNK_CURR_LIMIT */
		} else {
			ma = 0;
		}
	}

	tcp_noti.vbus_state.ma = ma;
	tcp_noti.vbus_state.mv = mv;
	tcp_noti.vbus_state.type = type;

	TCPC_DBG("sink_vbus: %d mV, %d mA\n", mv, ma);
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_SINK_VBUS);
}

int tcpci_disable_vbus_control(struct tcpc_device *tcpc)
{
#ifdef CONFIG_TYPEC_USE_DIS_VBUS_CTRL
	struct tcp_notify tcp_noti;

	TCPC_DBG("disable_vbus\n");
	tcpci_enable_watchdog(tcpc, false);

	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_DIS_VBUS_CTRL);
#else
	tcpci_sink_vbus(tcpc, TCP_VBUS_CTRL_REMOVE, TCPC_VBUS_SINK_0V, 0);
	tcpci_source_vbus(tcpc, TCP_VBUS_CTRL_REMOVE, TCPC_VBUS_SOURCE_0V, 0);
	return 0;
#endif	/* CONFIG_TYPEC_USE_DIS_VBUS_CTRL */
}

int tcpci_notify_attachwait_state(struct tcpc_device *tcpc, bool as_sink)
{
#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT
	u8 notify = 0;
	struct tcp_notify tcp_noti;

#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK
	if (as_sink)
		notify = TCP_NOTIFY_ATTACHWAIT_SNK;
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK */

#ifdef CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC
	if (!as_sink)
		notify = TCP_NOTIFY_ATTACHWAIT_SRC;
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC */

	if (notify == 0)
		return 0;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_VBUS, notify);
#else
	return 0;
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT */
}

int tcpci_enable_auto_discharge(struct tcpc_device *tcpc, bool en)
{
#ifdef CONFIG_TYPEC_CAP_AUTO_DISCHARGE
#ifdef CONFIG_TCPC_AUTO_DISCHARGE_IC
	if (tcpc->typec_auto_discharge != en) {
		tcpc->typec_auto_discharge = en;
		if (tcpc->ops->set_auto_discharge)
			return tcpc->ops->set_auto_discharge(tcpc, en);
	}
#endif	/* CONFIG_TCPC_AUTO_DISCHARGE_IC */
#endif	/* CONFIG_TYPEC_CAP_AUTO_DISCHARGE */

	return 0;
}

static int __tcpci_enable_force_discharge(struct tcpc_device *tcpc,
					  bool en, int mv)
{
#ifdef CONFIG_TYPEC_CAP_FORCE_DISCHARGE
#ifdef CONFIG_TCPC_FORCE_DISCHARGE_IC
	if (tcpc->pd_force_discharge != en) {
		tcpc->pd_force_discharge = en;
		if (tcpc->ops->set_force_discharge)
			return tcpc->ops->set_force_discharge(tcpc, en, mv);
	}
#endif	/* CONFIG_TCPC_FORCE_DISCHARGE_IC */
#endif	/* CONFIG_TYPEC_CAP_FORCE_DISCHARGE */

	return 0;
}

static int __tcpci_enable_ext_discharge(struct tcpc_device *tcpc, bool en)
{
#ifdef CONFIG_TCPC_EXT_DISCHARGE
	struct tcp_notify tcp_noti;

	if (tcpc->typec_ext_discharge != en) {
		tcpc->typec_ext_discharge = en;
		tcp_noti.en_state.en = en;
		TCPC_DBG("EXT-Discharge: %d\n", en);
		return tcpc_check_notify_time(tcpc, &tcp_noti,
					     TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_EXT_DISCHARGE);
	}
#endif	/* CONFIG_TCPC_EXT_DISCHARGE */

	return 0;
}

int tcpci_enable_force_discharge(struct tcpc_device *tcpc, bool en, int mv)
{
#ifdef CONFIG_TYPEC_CAP_FORCE_DISCHARGE
	int ret;

	ret = __tcpci_enable_force_discharge(tcpc, en, mv);
	if (ret < 0)
		return ret;

#ifdef CONFIG_TCPC_FORCE_DISCHARGE_EXT
	return __tcpci_enable_ext_discharge(tcpc, en);
#endif	/* CONFIG_TCPC_FORCE_DISCHARGE_EXT */

	return ret;
#else
	return 0;
#endif	/* CONFIG_TYPEC_CAP_FORCE_DISCHARGE */
}

#ifdef CONFIG_USB_POWER_DELIVERY

int tcpci_notify_hard_reset_state(struct tcpc_device *tcpc, u8 state)
{
	struct tcp_notify tcp_noti;

	tcp_noti.hreset_state.state = state;

	if (state >= TCP_HRESET_SIGNAL_SEND)
		tcpc->pd_wait_hard_reset_complete = true;
	else if (tcpc->pd_wait_hard_reset_complete)
		tcpc->pd_wait_hard_reset_complete = false;
	else
		return 0;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_HARD_RESET_STATE);
}

int tcpci_enter_mode(struct tcpc_device *tcpc,
		     u16 svid, u8 ops, u32 mode)
{
	struct tcp_notify tcp_noti;

	tcp_noti.mode_ctrl.svid = svid;
	tcp_noti.mode_ctrl.ops = ops;
	tcp_noti.mode_ctrl.mode = mode;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_ENTER_MODE);
}

int tcpci_exit_mode(struct tcpc_device *tcpc, u16 svid)
{
	struct tcp_notify tcp_noti;

	tcp_noti.mode_ctrl.svid = svid;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_EXIT_MODE);
}

#ifdef CONFIG_USB_PD_ALT_MODE

int tcpci_report_hpd_state(struct tcpc_device *tcpc, u32 dp_status)
{
	struct tcp_notify tcp_noti;
	struct dp_data *dp_data = pd_get_dp_data(&tcpc->pd_port);

	/* UFP_D to DFP_D only */

	if (PD_DP_CFG_DFP_D(dp_data->local_config)) {
		tcp_noti.ama_dp_hpd_state.irq = PD_VDO_DPSTS_HPD_IRQ(dp_status);
		tcp_noti.ama_dp_hpd_state.state = PD_VDO_DPSTS_HPD_LVL(dp_status);
		tcpc_check_notify_time(tcpc, &tcp_noti,
				       TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_HPD_STATE);
	}

	return 0;
}

int tcpci_dp_status_update(struct tcpc_device *tcpc, u32 dp_status)
{
	DP_INFO("Status0: 0x%x\n", dp_status);
	tcpci_report_hpd_state(tcpc, dp_status);
	return 0;
}

int tcpci_dp_configure(struct tcpc_device *tcpc, u32 dp_config)
{
	struct tcp_notify tcp_noti;

	DP_INFO("LocalCFG: 0x%x\n", dp_config);

	switch (dp_config & 0x03) {
	case 0:
		tcp_noti.ama_dp_state.sel_config = SW_USB;
		break;
	case MODE_DP_SNK:
		tcp_noti.ama_dp_state.sel_config = SW_DFP_D;
		tcp_noti.ama_dp_state.pin_assignment = (dp_config >> 8) & 0xff;
		break;
	case MODE_DP_SRC:
		tcp_noti.ama_dp_state.sel_config = SW_UFP_D;
		tcp_noti.ama_dp_state.pin_assignment = (dp_config >> 16) & 0xff;
		break;
	}
	if (tcp_noti.ama_dp_state.pin_assignment == 0)
		tcp_noti.ama_dp_state.pin_assignment = (dp_config >> 16) & 0xff;

	DP_INFO("pin assignment: 0x%x\n", tcp_noti.ama_dp_state.pin_assignment);
	tcp_noti.ama_dp_state.signal = (dp_config >> 2) & 0x0f;
	tcp_noti.ama_dp_state.polarity = tcpc->typec_polarity;
	tcp_noti.ama_dp_state.active = 1;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_STATE);
}

int tcpci_dp_attention(struct tcpc_device *tcpc, u32 dp_status)
{
	/* DFP_U : Not call this function during internal flow */
	struct tcp_notify tcp_noti;

	DP_INFO("Attention: 0x%x\n", dp_status);
	tcp_noti.ama_dp_attention.state = (u8)dp_status;
	tcpc_check_notify_time(tcpc, &tcp_noti,
			       TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_ATTENTION);
	return tcpci_report_hpd_state(tcpc, dp_status);
}

int tcpci_dp_notify_status_update_done(struct tcpc_device *tcpc,
				       u32 dp_status, bool ack)
{
	/* DFP_U : Not call this function during internal flow */
	DP_INFO("Status1: 0x%x, ack=%d\n", dp_status, ack);
	return 0;
}

int tcpci_dp_notify_config_start(struct tcpc_device *tcpc)
{
	/* DFP_U : Put signal & mux into the Safe State */
	struct tcp_notify tcp_noti;

	DP_INFO("ConfigStart\n");
	tcp_noti.ama_dp_state.sel_config = SW_USB;
	tcp_noti.ama_dp_state.active = 0;
	tcpc_check_notify_time(tcpc, &tcp_noti,
			       TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_STATE);
	return 0;
}

int tcpci_dp_notify_config_done(struct tcpc_device *tcpc,
				u32 local_cfg, u32 remote_cfg, bool ack)
{
	/* DFP_U : If DP success,
	 * internal flow will enter this function finally
	 */
	DP_INFO("ConfigDone, L:0x%x, R:0x%x, ack=%d\n",
		local_cfg, remote_cfg, ack);

	if (ack)
		tcpci_dp_configure(tcpc, local_cfg);

	return 0;
}
#endif	/* CONFIG_USB_PD_ALT_MODE */

#ifdef CONFIG_USB_PD_CUSTOM_VDM
int tcpci_notify_uvdm(struct tcpc_device *tcpc, bool ack)
{
	struct tcp_notify tcp_noti;
	struct pd_port *pd_port = &tcpc->pd_port;

	tcp_noti.uvdm_msg.ack = ack;

	if (ack) {
		tcp_noti.uvdm_msg.uvdm_cnt = pd_port->uvdm_cnt;
		tcp_noti.uvdm_msg.uvdm_svid = pd_port->uvdm_svid;
		tcp_noti.uvdm_msg.uvdm_data = pd_port->uvdm_data;
	}

	tcpc_check_notify_time(tcpc, &tcp_noti,
			       TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_UVDM);
	return 0;
}
#endif	/* CONFIG_USB_PD_CUSTOM_VDM */

#ifdef CONFIG_USB_PD_ALT_MODE_RTDC
int tcpci_dc_notify_en_unlock(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	DC_INFO("DirectCharge en_unlock\n");
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_DC_EN_UNLOCK);
}
#endif	/* CONFIG_USB_PD_ALT_MODE_RTDC */

/* ---- Policy Engine (PD30) ---- */

#ifdef CONFIG_USB_PD_REV30

#ifdef CONFIG_USB_PD_REV30_ALERT_REMOTE
int tcpci_notify_alert(struct tcpc_device *tcpc, u32 ado)
{
	struct tcp_notify tcp_noti;

	tcp_noti.alert_msg.ado = ado;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_ALERT);
}
#endif	/* CONFIG_USB_PD_REV30_ALERT_REMOTE */

#ifdef CONFIG_USB_PD_REV30_STATUS_REMOTE
int tcpci_notify_status(struct tcpc_device *tcpc, struct pd_status *sdb)
{
	struct tcp_notify tcp_noti;

	tcp_noti.status_msg.sdb = sdb;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_STATUS);
}
#endif	/* CONFIG_USB_PD_REV30_STATUS_REMOTE */

#ifdef CONFIG_USB_PD_REV30_BAT_INFO
int tcpci_notify_request_bat_info(struct tcpc_device *tcpc,
				  enum pd_battery_reference ref)
{
	struct tcp_notify tcp_noti;

	tcp_noti.request_bat.ref = ref;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
				     TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_REQUEST_BAT_INFO);
}
#endif	/* CONFIG_USB_PD_REV30_BAT_INFO */

#endif	/* CONFIG_USB_PD_REV30 */

#endif	/* CONFIG_USB_POWER_DELIVERY */
