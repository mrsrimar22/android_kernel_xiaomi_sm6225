// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt)	"[USBPD-PM]: %s: " fmt, __func__

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/usb/usbpd.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/printk.h>
#include "pd_policy_manager.h"

#define PCA_PPS_CMD_RETRY_COUNT		3
#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3

#define BATT_MAX_CHG_VOLT		4450
#define BATT_FAST_CHG_CURR		5800
#define BUS_OVP_THRESHOLD		12000
#define BUS_OVP_ALARM_THRESHOLD		9500

#define BUS_VOLT_INIT_UP		400

#define PM_WORK_RUN_INTERVAL		300
#define PM_WORK_FC2_TUNE_INTERVAL	900

#define JEITA_WARM_DISABLE_CP_THR		480
#define JEITA_COOL_DISABLE_CP_THR		100
#define JEITA_BYPASS_WARM_DISABLE_CP_THR	480
#define JEITA_BYPASS_COOL_DISABLE_CP_THR	100

enum {
	PM_ALGO_RET_OK,
	PM_ALGO_RET_THERM_FAULT,
	PM_ALGO_RET_OTHER_FAULT,
	PM_ALGO_RET_CHG_DISABLED,
	PM_ALGO_RET_TAPER_DONE,
};

#define pdpm_log(fmt, ...)						\
do {									\
	if (pdpm && __ratelimit(&pdpm->rl_slots[_PDPM_ID() &		\
					    (PDPM_RL_SLOTS - 1)]))	\
		pr_info(fmt, ##__VA_ARGS__);				\
} while (0)

#define pdpm_info(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define pdpm_err(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define pdpm_dbg(fmt, ...)	pr_debug(fmt, ##__VA_ARGS__)

#define DT_READ_U32_OR(np, key, def) \
	({ u32 _v; of_property_read_u32((np), (key), &_v) ? (def) : _v; })

static void usbpd_check_cp_psy(struct usbpd_pm *pdpm)
{
	if (pdpm->cp_psy)
		return;

	pdpm->cp_psy = pdpm->cfg.cp_sec_enable
		? power_supply_get_by_name("sc8551-master")
		: power_supply_get_by_name("sc8551-standalone");

	if (pdpm->cp_psy && pdpm->cfg.cp_sec_enable)
		pdpm_info("sc8551-master found\n");
	else if (!pdpm->cp_psy)
		pdpm_err("cp_psy not found\n");
}

static void usbpd_check_cp_sec_psy(struct usbpd_pm *pdpm)
{
	if (pdpm->cp_sec_psy)
		return;

	pdpm->cp_sec_psy = power_supply_get_by_name("sc8551-slave");
	if (pdpm->cp_sec_psy)
		pdpm_info("sc8551-slave found\n");
	else
		pdpm_err("cp_sec_psy not found\n");
}

static void usbpd_check_usb_psy(struct usbpd_pm *pdpm)
{
	if (pdpm->usb_psy)
		return;

	pdpm->usb_psy = power_supply_get_by_name("usb");
	if (!pdpm->usb_psy)
		pdpm_err("usb psy not found!\n");
}

static void usbpd_check_tcpc(struct usbpd_pm *pdpm)
{
	if (pdpm->tcpc)
		return;

	pdpm->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!pdpm->tcpc)
		pdpm_err("get tcpc dev fail\n");
}

static void usbpd_check_batt_psy(struct usbpd_pm *pdpm)
{
	if (pdpm->batt_psy)
		return;

	pdpm->batt_psy = power_supply_get_by_name("battery");
	if (!pdpm->batt_psy)
		pdpm_err("batt psy not found!\n");
}

static void usbpd_check_bms_psy(struct usbpd_pm *pdpm)
{
	if (pdpm->bms_psy)
		return;

	pdpm->bms_psy = power_supply_get_by_name("bms");
	if (!pdpm->bms_psy)
		pdpm_err("bms psy not found!\n");
}

static void usbpd_clear_psy(struct usbpd_pm *pdpm)
{
	if (pdpm->bms_psy)
		power_supply_put(pdpm->bms_psy);
	if (pdpm->batt_psy)
		power_supply_put(pdpm->batt_psy);
	if (pdpm->tcpc)
		tcpc_dev_put(pdpm->tcpc);
	if (pdpm->usb_psy)
		power_supply_put(pdpm->usb_psy);
	if (pdpm->cfg.cp_sec_enable && pdpm->cp_sec_psy)
		power_supply_put(pdpm->cp_sec_psy);
	if (pdpm->cp_psy)
		power_supply_put(pdpm->cp_psy);
}

static int pd_get_batt_current_thermal_level(struct usbpd_pm *pdpm, int *level)
{
	union power_supply_propval pval = {0,};
	int ret;

	usbpd_check_batt_psy(pdpm);
	if (!pdpm->batt_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->batt_psy,
					POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &pval);
	if (ret < 0) {
		pdpm_err("Couldn't get system temp level: %d\n", ret);
		return ret;
	}

	pdpm_log("system temp level: %d\n", pval.intval);
	*level = pval.intval;

	return ret;
}

static bool pd_disable_cp_by_jeita_status(struct usbpd_pm *pdpm)
{
	union power_supply_propval pval = {0,};
	int batt_temp, input_suspend = 0;
	const int warm_thres = JEITA_WARM_DISABLE_CP_THR;
	const int cool_thres = JEITA_COOL_DISABLE_CP_THR;
	int ret;

	usbpd_check_batt_psy(pdpm);
	if (!pdpm->batt_psy)
		return false;

	ret = power_supply_get_property(pdpm->batt_psy,
					POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
	if (!ret)
		input_suspend = pval.intval;

	pdpm_log("input_suspend: %d\n", input_suspend);
	if (input_suspend)
		return true;

	ret = power_supply_get_property(pdpm->batt_psy,
					POWER_SUPPLY_PROP_TEMP, &pval);
	if (ret < 0) {
		pdpm_err("Couldn't get batt temp prop: %d\n", ret);
		return false;
	}

	batt_temp = pval.intval;
	pdpm_log("batt_temp: %d\n", (batt_temp / 10));

	if ((batt_temp >= warm_thres || batt_temp <= cool_thres) &&
	    !pdpm->jeita_triggered)
		pdpm->jeita_triggered = true;
	else if (batt_temp < warm_thres - JEITA_HYSTERESIS &&
		 batt_temp > cool_thres + JEITA_HYSTERESIS &&
		 pdpm->jeita_triggered)
		pdpm->jeita_triggered = false;

	return pdpm->jeita_triggered;
}

static inline int check_typec_attached_snk(struct tcpc_device *tcpc)
{
	if (tcpm_inquire_typec_attach_state(tcpc) != TYPEC_ATTACHED_SNK)
		return -EINVAL;

	return 0;
}

static int usbpd_pps_enable_charging(struct usbpd_pm *pdpm, bool en,
				     u32 mv, u32 ma)
{
	int ret, cnt;

	if (check_typec_attached_snk(pdpm->tcpc) < 0)
		return -EINVAL;

	pdpm_log("en = %d, %dmv, %dma\n", en, mv, ma);

	for (cnt = 0; cnt < PCA_PPS_CMD_RETRY_COUNT; cnt++) {
		ret = en ? tcpm_set_apdo_charging_policy(pdpm->tcpc,
							 DPM_CHARGING_POLICY_PPS,
							 mv, ma, NULL)
			 : tcpm_reset_pd_charging_policy(pdpm->tcpc, NULL);
		if (ret == TCP_DPM_RET_SUCCESS)
			break;
	}

	if (ret != TCP_DPM_RET_SUCCESS)
		pdpm_err("fail(%d)\n", ret);

	return ret > 0 ? -ret : ret;
}

static bool usbpd_get_pps_status(struct usbpd_pm *pdpm)
{
	struct tcpm_power_cap_val apdo_cap = {0};
	u8 cap_idx = 0;
	int ret, i;

	if (check_typec_attached_snk(pdpm->tcpc) < 0)
		return false;

	for (i = 0; i < 100 && !pdpm->is_pps_en_unlock; i++)
		usleep_range(1000, 1100);

	if (!pdpm->is_pps_en_unlock) {
		pdpm_err("pps en still locked after 100ms\n");
		return false;
	}

	if (!tcpm_inquire_pd_pe_ready(pdpm->tcpc)) {
		pdpm_err("PD PE not ready\n");
		return false;
	}

	while (tcpm_inquire_pd_source_apdo(pdpm->tcpc,
					   TCPM_POWER_CAP_APDO_TYPE_PPS,
					   &cap_idx, &apdo_cap) == TCP_DPM_RET_SUCCESS) {
		pdpm_info("cap_idx[%d], %d mv ~ %d mv, %d ma, pl: %d\n",
			  cap_idx, apdo_cap.min_mv, apdo_cap.max_mv,
			  apdo_cap.ma, apdo_cap.pwr_limit);

		if (apdo_cap.max_mv < pdpm->cfg.min_adapter_volt_required ||
		    apdo_cap.ma < pdpm->cfg.min_adapter_curr_required)
			continue;

		pdpm_info("select potential cap_idx[%d]\n", cap_idx);
		pdpm->apdo_max_volt = apdo_cap.max_mv;
		pdpm->apdo_max_curr = apdo_cap.ma;
		break;
	}

	if (!pdpm->apdo_max_volt)
		return false;

	ret = usbpd_pps_enable_charging(pdpm, true, 9000, 2000);
	return ret == TCP_DPM_RET_SUCCESS;
}

static int usbpd_select_pdo(struct usbpd_pm *pdpm, u32 mv, u32 ma)
{
	int ret, cnt;

	if (!pdpm)
		return -EINVAL;

	if (nopmi_is_maxim_ic())
		return usbpd_select_pdo_maxim(pdpm->apdo_selected_pdo, mv, ma);

	if (check_typec_attached_snk(pdpm->tcpc) < 0)
		return -EINVAL;

	if (!tcpm_inquire_pd_connected(pdpm->tcpc)) {
		pdpm_err("pd not connected\n");
		return -EINVAL;
	}

	pdpm_log("%dmv, %dma\n", mv, ma);

	for (cnt = 0; cnt < PCA_PPS_CMD_RETRY_COUNT; cnt++) {
		ret = tcpm_dpm_pd_request(pdpm->tcpc, mv, ma, NULL);
		if (ret == TCP_DPM_RET_SUCCESS)
			break;
	}

	if (ret != TCP_DPM_RET_SUCCESS)
		pdpm_err("fail(%d)\n", ret);

	return ret > 0 ? -ret : ret;
}

static int pca_pps_tcp_notifier_call(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct usbpd_pm *pdpm = container_of(nb, struct usbpd_pm, tcp_nb);
	struct tcp_notify *noti = data;

	switch (event) {
	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			pdpm_info("detached\n");
			pdpm->is_pps_en_unlock = false;
			pdpm->hrst_cnt = 0;
			break;
		case PD_CONNECT_HARD_RESET:
			pdpm->hrst_cnt++;
			pdpm_info("pd hardreset, cnt = %d\n", pdpm->hrst_cnt);
			pdpm->is_pps_en_unlock = false;
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			if (pdpm->hrst_cnt < 5) {
				pdpm_info("en unlock\n");
				pdpm->is_pps_en_unlock = true;
			}
			break;
		default:
			break;
		}
		if (pdpm->usb_psy)
			power_supply_changed(pdpm->usb_psy);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static int pdpm_cp_set_charging(struct power_supply *psy, bool enable)
{
	union power_supply_propval val = { .intval = enable };
	int ret;

	ret = power_supply_set_property(psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (ret < 0)
		pr_err("failed to set charging enabled=%d, ret=%d\n",
		       enable, ret);
	return ret;
}

static int pdpm_cp_read_enabled(struct power_supply *psy, bool *enabled)
{
	union power_supply_propval val = {0,};
	int ret;

	ret = power_supply_get_property(psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		*enabled = !!val.intval;
	return ret;
}

static int usbpd_pm_enable_sw(struct usbpd_pm *pdpm, bool en)
{
	union power_supply_propval val = { .intval = en };
	int ret;

	pdpm_info("%d\n", en);
	usbpd_check_batt_psy(pdpm);
	if (!pdpm->batt_psy)
		return -ENODEV;

	ret = power_supply_set_property(pdpm->batt_psy,
					POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &val);
	if (ret < 0) {
		pdpm_err("failed to set charging enabled to %d, ret=%d\n", en, ret);
		return ret;
	}

	pdpm->sw.charge_enabled = en;
	return 0;
}

static int usbpd_pm_enable_cp(struct usbpd_pm *pdpm, bool enable)
{
	usbpd_check_cp_psy(pdpm);
	if (!pdpm->cp_psy)
		return -ENODEV;

	return pdpm_cp_set_charging(pdpm->cp_psy, enable);
}

static int usbpd_pm_enable_cp_sec(struct usbpd_pm *pdpm, bool enable)
{
	usbpd_check_cp_sec_psy(pdpm);
	if (!pdpm->cp_sec_psy)
		return -ENODEV;

	return pdpm_cp_set_charging(pdpm->cp_sec_psy, enable);
}

static int usbpd_pm_check_cp_enabled(struct usbpd_pm *pdpm)
{
	int ret;

	usbpd_check_cp_psy(pdpm);
	if (!pdpm->cp_psy)
		return -ENODEV;

	ret = pdpm_cp_read_enabled(pdpm->cp_psy, &pdpm->cp.charge_enabled);
	pdpm_log("%d\n", pdpm->cp.charge_enabled);
	return ret;
}

static int usbpd_pm_check_cp_sec_enabled(struct usbpd_pm *pdpm)
{
	int ret;

	usbpd_check_cp_sec_psy(pdpm);
	if (!pdpm->cp_sec_psy)
		return -ENODEV;

	ret = pdpm_cp_read_enabled(pdpm->cp_sec_psy, &pdpm->cp_sec.charge_enabled);
	pdpm_log("%d\n", pdpm->cp_sec.charge_enabled);
	return ret;
}

static int usbpd_pm_check_sw_enabled(struct usbpd_pm *pdpm)
{
	union power_supply_propval val = {0,};
	int ret;

	usbpd_check_batt_psy(pdpm);
	if (!pdpm->batt_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->batt_psy,
					POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->sw.charge_enabled = !!val.intval;

	pdpm_log("%d\n", pdpm->sw.charge_enabled);
	return ret;
}

static void usbpd_pm_update_cp_status(struct usbpd_pm *pdpm)
{
	struct cp_device *cp = &pdpm->cp;
	union power_supply_propval val = {0,};
	int ret;

	usbpd_check_cp_psy(pdpm);
	if (!pdpm->cp_psy)
		return;

	usbpd_check_bms_psy(pdpm);
	if (!pdpm->bms_psy)
		return;

	ret = power_supply_get_property(pdpm->bms_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		cp->ibat_curr = val.intval / 1000;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_BUS_VOLTAGE, &val);
	if (!ret)
		cp->vbus_volt = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_BUS_CURRENT, &val);
	if (!ret)
		cp->ibus_curr_cp = val.intval;

	cp->ibus_curr = cp->ibus_curr_cp;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_VBUS_ERROR_STATUS, &val);
	if (!ret) {
		pdpm_log(">>>>vbus error state: %02x\n", val.intval);
		cp->vbus_error_low = (val.intval >> 5) & 0x01;
		cp->vbus_error_high = (val.intval >> 4) & 0x01;
	}

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_BUS_TEMPERATURE, &val);
	if (!ret)
		cp->bus_temp = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_BATTERY_TEMPERATURE, &val);
	if (!ret)
		cp->bat_temp = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_DIE_TEMPERATURE, &val);
	if (!ret)
		cp->die_temp = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_BATTERY_PRESENT, &val);
	if (!ret)
		cp->batt_pres = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_VBUS_PRESENT, &val);
	if (!ret)
		cp->vbus_pres = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		cp->charge_enabled = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_ALARM_STATUS, &val);
	if (!ret) {
		cp->bat_ovp_alarm = !!(val.intval & BAT_OVP_ALARM_MASK);
		cp->bat_ocp_alarm = !!(val.intval & BAT_OCP_ALARM_MASK);
		cp->bus_ovp_alarm = !!(val.intval & BUS_OVP_ALARM_MASK);
		cp->bus_ocp_alarm = !!(val.intval & BUS_OCP_ALARM_MASK);
		cp->bat_ucp_alarm = !!(val.intval & BAT_UCP_ALARM_MASK);
		cp->bat_therm_alarm = !!(val.intval & BAT_THERM_ALARM_MASK);
		cp->bus_therm_alarm = !!(val.intval & BUS_THERM_ALARM_MASK);
		cp->die_therm_alarm = !!(val.intval & DIE_THERM_ALARM_MASK);
	}

	ret = power_supply_get_property(pdpm->cp_psy,
					POWER_SUPPLY_PROP_SC_FAULT_STATUS, &val);
	if (!ret) {
		cp->bat_ovp_fault = !!(val.intval & BAT_OVP_FAULT_MASK);
		cp->bat_ocp_fault = !!(val.intval & BAT_OCP_FAULT_MASK);
		cp->bus_ovp_fault = !!(val.intval & BUS_OVP_FAULT_MASK);
		cp->bus_ocp_fault = !!(val.intval & BUS_OCP_FAULT_MASK);
		cp->bat_therm_fault = !!(val.intval & BAT_THERM_FAULT_MASK);
		cp->bus_therm_fault = !!(val.intval & BUS_THERM_FAULT_MASK);
		cp->die_therm_fault = !!(val.intval & DIE_THERM_FAULT_MASK);
	}
}

static void usbpd_pm_update_cp_sec_status(struct usbpd_pm *pdpm)
{
	struct cp_device *cp_sec = &pdpm->cp_sec;
	union power_supply_propval val = {0,};
	int ret;

	if (!pdpm->cfg.cp_sec_enable)
		return;

	usbpd_check_cp_sec_psy(pdpm);
	if (!pdpm->cp_sec_psy)
		return;

	ret = power_supply_get_property(pdpm->cp_sec_psy,
					POWER_SUPPLY_PROP_SC_BUS_CURRENT, &val);
	if (!ret)
		cp_sec->ibus_curr = val.intval;

	ret = power_supply_get_property(pdpm->cp_sec_psy,
					POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		cp_sec->charge_enabled = val.intval;

	ret = power_supply_get_property(pdpm->cp_sec_psy,
					POWER_SUPPLY_PROP_SC_VBUS_ERROR_STATUS, &val);
	if (!ret) {
		pdpm_log(">>>>slave cp vbus error state: %02x\n", val.intval);
		cp_sec->vbus_error_low = (val.intval >> 5) & 0x01;
		cp_sec->vbus_error_high = (val.intval >> 4) & 0x01;
	}
}

static void usbpd_pm_evaluate_src_caps_maxim(struct usbpd_pm *pdpm)
{
	int i;

	pdpm->pdo = usbpd_fetch_pdo();
	if (!pdpm->pdo) {
		pdpm_err("fetch_pdo returned NULL\n");
		return;
	}

	pdpm->apdo_max_volt = pdpm->cfg.min_adapter_volt_required;
	pdpm->apdo_max_curr = pdpm->cfg.min_adapter_curr_required;

	for (i = 0; i < 7; i++) {
		pdpm_log("[SC manager] %d type %d\n", i, pdpm->pdo[i].apdo);

		if (!pdpm->pdo[i].apdo)
			continue;

		if (pdpm->pdo[i].max_voltage >= pdpm->apdo_max_volt &&
		    pdpm->pdo[i].max_current > pdpm->apdo_max_curr) {
			pdpm->apdo_max_volt = pdpm->pdo[i].max_voltage;
			pdpm->apdo_max_curr = pdpm->pdo[i].max_current;
			pdpm->apdo_selected_pdo = i;
			pdpm->pps_supported = true;
			pdpm_log("[SC manager] volt %d curr %d\n",
				 pdpm->apdo_max_volt, pdpm->apdo_max_curr);
		}
	}

	if (pdpm->pps_supported)
		pdpm_log("PPS supported, preferred APDO pos: %d, max volt: %d, current: %d\n",
			 pdpm->apdo_selected_pdo, pdpm->apdo_max_volt, pdpm->apdo_max_curr);
	else
		pdpm_log("Not qualified PPS adapter\n");
}

static void usbpd_pm_evaluate_src_caps(struct usbpd_pm *pdpm)
{
	pdpm->pps_supported = usbpd_get_pps_status(pdpm);

	if (pdpm->pps_supported)
		pdpm_log("PPS supported, preferred APDO pos: %d, max volt: %d, current: %d\n",
			 pdpm->apdo_selected_pdo, pdpm->apdo_max_volt, pdpm->apdo_max_curr);
	else
		pdpm_log("Not qualified PPS adapter\n");
}

static int usbpd_update_ibat_curr(struct usbpd_pm *pdpm)
{
	struct cp_device *cp = &pdpm->cp;
	union power_supply_propval val = {0,};
	int ret;

	usbpd_check_bms_psy(pdpm);
	if (!pdpm->bms_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->bms_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		cp->ibat_curr_sw = val.intval / 1000;

	ret = power_supply_get_property(pdpm->bms_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		cp->vbat_volt = val.intval / 1000;

	pdpm_log("ibat_curr_fg: %d, vbat_volt_fg: %d\n",
		 cp->ibat_curr_sw, cp->vbat_volt);

	return ret;
}

#define TAPER_TIMEOUT		10
#define IBUS_CHANGE_TIMEOUT	5

static int usbpd_pm_fc2_charge_algo(struct usbpd_pm *pdpm)
{
	struct cp_device *cp = &pdpm->cp;
	struct cp_device *cp_sec = &pdpm->cp_sec;
	const int bat_volt_lmt = pdpm->cfg.bat_volt_lp_lmt;
	const int fc2_steps = pdpm->cfg.fc2_steps;
	const int fc2_taper_curr = pdpm->cfg.fc2_taper_current;
	int steps = 0, sw_ctrl_steps, hw_ctrl_steps;
	int step_vbat = 0, step_ibus = 0, step_ibat = 0;
	int step_bat_reg, ibus_total;
	int fcc_vote_val, effective_fcc_taper;
	int fcc_ibatt_diff, sicl_ibus_diff, ibus_limit, fcc_limit;
	int volt_ceil, time_delta, thermal_level;

	pd_get_batt_current_thermal_level(pdpm, &thermal_level);
	time_delta = ktime_ms_delta(ktime_get(), pdpm->entry_bq_cv_time);

	if (!pdpm->fcc_votable) {
		pdpm_err("FCC votable not found\n");
		return PM_ALGO_RET_OTHER_FAULT;
	}
	fcc_vote_val = get_effective_result(pdpm->fcc_votable);
	fcc_limit = min(fcc_vote_val, pdpm->cfg.bat_curr_lp_lmt);
	ibus_limit = fcc_limit >> 1;

	if (cp->vbat_volt > bat_volt_lmt - BQ_TAPER_HYS_MV) {
		if (pdpm->ibus_lmt_change_timer++ > IBUS_CHANGE_TIMEOUT) {
			pdpm->ibus_lmt_change_timer = 0;
			ibus_limit -= 100;
			effective_fcc_taper = fcc_vote_val - BQ_TAPER_DECREASE_STEP_MA;
			if (effective_fcc_taper >= 2000)
				vote(pdpm->fcc_votable, BQ_TAPER_FCC_VOTER,
				     true, effective_fcc_taper);
			pdpm_log("bq set taper fcc to %dmA\n", effective_fcc_taper);
		}
	} else {
		pdpm->ibus_lmt_change_timer = 0;
	}

	if (cp->vbat_volt > bat_volt_lmt)
		step_vbat = -fc2_steps;
	else if (cp->vbat_volt < bat_volt_lmt - 10)
		step_vbat = fc2_steps;
	pdpm_log("vbat: %d, lmt: %d, step: %d\n",
		 cp->vbat_volt, bat_volt_lmt, step_vbat);

	if (cp->ibat_curr < fcc_limit)
		step_ibat = fc2_steps;
	else if (cp->ibat_curr > fcc_limit + 100)
		step_ibat = -fc2_steps;
	pdpm_log("ibat: %d, lmt: %d, step: %d\n",
		 cp->ibat_curr, fcc_limit, step_ibat);

	ibus_total = cp->ibus_curr;
	if (pdpm->cfg.cp_sec_enable)
		ibus_total += cp_sec->ibus_curr;

	if (ibus_total < ibus_limit - 130)
		step_ibus = fc2_steps;
	else if (ibus_total > ibus_limit - 80)
		step_ibus = -fc2_steps;
	pdpm_log("ibus: %d, lmt: %d, step: %d\n",
		 ibus_total, ibus_limit, step_ibus);

	step_bat_reg = fc2_steps;
	sw_ctrl_steps = min3(step_vbat, step_ibus, step_ibat);
	sw_ctrl_steps = min(sw_ctrl_steps, step_bat_reg);

	hw_ctrl_steps = (cp->bat_ocp_alarm || cp->bus_ocp_alarm || cp->bus_ovp_alarm)
		? -fc2_steps : fc2_steps;

	pdpm_log("sw_steps: %d, hw_steps: %d, m_vbush: %d, s_vbush: %d\n",
		 sw_ctrl_steps, hw_ctrl_steps,
		 cp->vbus_error_high, cp_sec->vbus_error_high);

	usbpd_pm_check_cp_enabled(pdpm);
	pdpm_log("cp enable: %d\n", cp->charge_enabled);
	if (pdpm->cfg.cp_sec_enable) {
		usbpd_pm_check_cp_sec_enabled(pdpm);
		pdpm_log("cp sec enable: %d\n", cp_sec->charge_enabled);
		if (!cp_sec->charge_enabled && cp->ibat_curr > 3000)
			usbpd_pm_enable_cp_sec(pdpm, true);
	}

	pdpm->is_temp_out_fc2_range = pd_disable_cp_by_jeita_status(pdpm);
	pdpm_log("is_temp_out_fc2_range = %d\n", pdpm->is_temp_out_fc2_range);

	if (cp->bat_therm_fault) {
		pdpm_log("bat_therm_fault: %d\n", cp->bat_therm_fault);
		return PM_ALGO_RET_THERM_FAULT;
	} else if (thermal_level >= MAX_THERMAL_LEVEL_FOR_CP ||
		   pdpm->is_temp_out_fc2_range) {
		pdpm_log("system thermal level too high or batt temp is out of fc2 range\n");
		return PM_ALGO_RET_CHG_DISABLED;
	} else if (cp->bat_ocp_fault || cp->bus_ocp_fault ||
		   cp->bat_ovp_fault || cp->bus_ovp_fault) {
		pdpm_log("ocp_fault: (bat: %d, bus: %d) ovp_fault: (bat: %d, bus: %d)\n",
			 cp->bat_ocp_fault, cp->bus_ocp_fault,
			 cp->bat_ovp_fault, cp->bus_ovp_fault);
		return PM_ALGO_RET_OTHER_FAULT;
	} else if (!cp->charge_enabled &&
		   (cp->vbus_error_low || cp->vbus_error_high)) {
		pdpm_log("cp.charge_enabled: %d %d %d\n",
			 cp->charge_enabled, cp->vbus_error_low, cp->vbus_error_high);
		return PM_ALGO_RET_CHG_DISABLED;
	}

	if (cp->vbat_volt > bat_volt_lmt - TAPER_VOL_HYS &&
	    cp->ibat_curr < fc2_taper_curr) {
		if (pdpm->fc2_taper_timer++ > TAPER_TIMEOUT) {
			pdpm_log("charge pump taper charging done\n");
			pdpm->fc2_taper_timer = 0;
			return PM_ALGO_RET_TAPER_DONE;
		}
	} else {
		pdpm->fc2_taper_timer = 0;
	}

	steps = min(sw_ctrl_steps, hw_ctrl_steps);
	if (cp->ibat_curr > 0 && ibus_total > 0 &&
	    time_delta < QUICK_RAISE_VOLT_INTERVAL_S) {
		fcc_ibatt_diff = abs(cp->ibat_curr - fcc_limit);
		sicl_ibus_diff = abs(ibus_total - ibus_limit);

		pdpm_log("fcc_ibatt_diff: %d, sicl_ibus_diff: %d\n",
			 fcc_ibatt_diff, sicl_ibus_diff);

		if (fcc_ibatt_diff > 1200 && sicl_ibus_diff > 500)
			steps *= 5;
		else if (fcc_ibatt_diff > 500 && fcc_ibatt_diff <= 1200 &&
			 sicl_ibus_diff > 250 && sicl_ibus_diff <= 500)
			steps *= 3;
	}

	pdpm_log(">>>>>>%d %d %d sw %d hw %d all %d\n",
		 step_vbat, step_ibat, step_ibus,
		 sw_ctrl_steps, hw_ctrl_steps, steps);

	pdpm->request_voltage += steps * 20;
	volt_ceil = pdpm->apdo_max_volt - (pdpm->apdo_max_volt > 10000 ? 1000 : 0);
	pdpm->request_voltage = min(pdpm->request_voltage, volt_ceil);

	return PM_ALGO_RET_OK;
}

static const char * const pm_str[] = {
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_FC2_ENTRY",
	"PD_PM_STATE_FC2_ENTRY_1",
	"PD_PM_STATE_FC2_ENTRY_2",
	"PD_PM_STATE_FC2_ENTRY_3",
	"PD_PM_STATE_FC2_TUNE",
	"PD_PM_STATE_FC2_EXIT",
};

static void usbpd_pm_move_state(struct usbpd_pm *pdpm, enum pm_state state)
{
	const char *old_str = (pdpm->state < ARRAY_SIZE(pm_str)) ? pm_str[pdpm->state] : "UNKNOWN";
	const char *new_str = (state < ARRAY_SIZE(pm_str)) ? pm_str[state] : "UNKNOWN";

	pdpm_info("state change: %s -> %s\n", old_str, new_str);
	pdpm->state = state;
}

static int pd_set_cv(struct usbpd_pm *pdpm, int val)
{
	int last_pd_cv, ret;

	if (!pdpm->fv_votable) {
		pdpm_err("fv_votable is null!\n");
		return -ENXIO;
	}

	last_pd_cv = get_effective_result(pdpm->fv_votable);
	if (last_pd_cv < 0) {
		pdpm_err("get_effective_result fail!\n");
		return last_pd_cv;
	}

	pdpm->pd_cv = val;
	pdpm_log("pd_cv: %d, last_pd_cv: %d\n", pdpm->pd_cv, last_pd_cv);

	if (pdpm->pd_cv == last_pd_cv) {
		pdpm_log("skip vote, pd_cv: %d, last_pd_cv: %d\n",
			 pdpm->pd_cv, last_pd_cv);
		return 0;
	}

	ret = vote(pdpm->fv_votable, JEITA_VOTER, true, pdpm->pd_cv);
	if (ret < 0)
		pdpm_err("vote pd_cv to %d fail, ret=%d\n", pdpm->pd_cv, ret);

	return ret;
}

static bool usbpd_pm_sm(struct usbpd_pm *pdpm)
{
	struct cp_device *cp = &pdpm->cp;
	struct cp_device *cp_sec = &pdpm->cp_sec;
	struct sw_device *sw = &pdpm->sw;
	const int bat_volt_lmt = pdpm->cfg.bat_volt_lp_lmt;
	int ret, cv_val, thermal_level;
	bool done = false;

	pdpm_log("state phase: %d\n", pdpm->state);
	pdpm_log("vbus_vol: %d, vbat_vol: %d, ibat_curr: %d\n",
		 cp->vbus_volt, cp->vbat_volt, cp->ibat_curr);

	switch (pdpm->state) {
	case PD_PM_STATE_ENTRY:
		pdpm->stop_sw = false;
		pdpm->recover = false;
		pd_get_batt_current_thermal_level(pdpm, &thermal_level);
		pdpm->is_temp_out_fc2_range = pd_disable_cp_by_jeita_status(pdpm);
		pdpm_log("is_temp_out_fc2_range: %d\n", pdpm->is_temp_out_fc2_range);

		if (cp->vbat_volt < pdpm->cfg.min_vbat_for_cp) {
			pdpm_log("pm_sm batt_volt: %d, waiting...\n", cp->vbat_volt);
		} else if (cp->vbat_volt > bat_volt_lmt - 100) {
			pdpm_log("pm_sm batt_volt: %d too high for cp\n", cp->vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		} else if (thermal_level >= MAX_THERMAL_LEVEL_FOR_CP ||
			   pdpm->is_temp_out_fc2_range) {
			pdpm_log("system thermal level too high or temp out of fc2 range\n");
		} else {
			pdpm_log("pm_sm batt_volt: %d is ok, start flash charging\n",
				 cp->vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY);
		}
		break;

	case PD_PM_STATE_FC2_ENTRY:
		if (pdpm->cfg.fc2_disable_sw) {
			usbpd_pm_enable_sw(pdpm, false);
			if (!sw->charge_enabled)
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		} else {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		}
		break;

	case PD_PM_STATE_FC2_ENTRY_1:
		pdpm->request_voltage = cp->vbat_volt * 2 + BUS_VOLT_INIT_UP;
		pdpm->request_current = min(pdpm->apdo_max_curr, pdpm->cfg.bus_curr_lp_lmt);
		usbpd_select_pdo(pdpm, pdpm->request_voltage, pdpm->request_current);
		pdpm_log("request_voltage: %d, request_current: %d\n",
			 pdpm->request_voltage, pdpm->request_current);
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_2);
		pdpm->tune_vbus_retry = 0;
		break;

	case PD_PM_STATE_FC2_ENTRY_2:
		pdpm_log("tune_vbus_retry %d, vbus_low: %d, vbus_high: %d\n",
			 pdpm->tune_vbus_retry, cp->vbus_error_low, cp->vbus_error_high);

		if (cp->vbus_error_low ||
		    cp->vbus_volt < cp->vbat_volt * 2 + BUS_VOLT_INIT_UP - 50) {
			pdpm->tune_vbus_retry++;
			pdpm->request_voltage += 20;
			usbpd_select_pdo(pdpm, pdpm->request_voltage, pdpm->request_current);
			pdpm_log("vbus low, request_volt: %d, request_curr: %d\n",
				 pdpm->request_voltage, pdpm->request_current);
		} else if (cp->vbus_error_high ||
			   cp->vbus_volt > cp->vbat_volt * 2 + BUS_VOLT_INIT_UP + 200) {
			pdpm->tune_vbus_retry++;
			pdpm->request_voltage -= 20;
			usbpd_select_pdo(pdpm, pdpm->request_voltage, pdpm->request_current);
			pdpm_log("vbus high, request_volt: %d, request_cur: %d\n",
				 pdpm->request_voltage, pdpm->request_current);
		} else {
			pdpm_log("adapter volt tune ok, retry %d times\n",
				 pdpm->tune_vbus_retry);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_3);
			break;
		}

		if (pdpm->tune_vbus_retry > 30) {
			pdpm_log("tune adapter volt fail, try recover\n");
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			if (nopmi_is_syv_ic())
				pdpm->recover = true;
		}
		break;

	case PD_PM_STATE_FC2_ENTRY_3:
		cv_val = 4608;
		ret = pd_set_cv(pdpm, cv_val);
		if (ret < 0)
			pdpm_err("set pd_cv fail, ret: %d\n", ret);

		usbpd_pm_check_cp_enabled(pdpm);
		if (!cp->charge_enabled) {
			usbpd_pm_enable_cp(pdpm, true);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pdpm->cfg.cp_sec_enable) {
			usbpd_pm_check_cp_sec_enabled(pdpm);
			if (!cp_sec->charge_enabled) {
				usbpd_pm_enable_cp_sec(pdpm, true);
				usbpd_pm_check_cp_sec_enabled(pdpm);
			}
		}

		if (cp->charge_enabled &&
		    (!pdpm->cfg.cp_sec_enable || cp_sec->charge_enabled)) {
			pdpm->entry_bq_cv_time = ktime_get();
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_TUNE);
			pdpm->ibus_lmt_change_timer = 0;
			pdpm->fc2_taper_timer = 0;
		}
		break;

	case PD_PM_STATE_FC2_TUNE:
		ret = usbpd_pm_fc2_charge_algo(pdpm);
		if (ret != PM_ALGO_RET_OK) {
			switch (ret) {
			case PM_ALGO_RET_THERM_FAULT:
				pdpm_log("Move to stop charging: %d\n", ret);
				pdpm->stop_sw = true;
				break;
			case PM_ALGO_RET_CHG_DISABLED:
				pdpm_log("Move to switch charging, try recover: %d\n", ret);
				pdpm->recover = true;
				break;
			case PM_ALGO_RET_OTHER_FAULT:
				pdpm_log("Move to switch charging: %d\n", ret);
				break;
			default:
				pdpm_log("Move to switch charging, taper done: %d\n", ret);
				break;
			}
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		}

		usbpd_select_pdo(pdpm, pdpm->request_voltage, pdpm->request_current);
		pdpm_log("request_voltage: %d, request_current: %d\n",
			 pdpm->request_voltage, pdpm->request_current);

		if (pdpm->cfg.cp_sec_enable &&
		    cp_sec->charge_enabled &&
		    cp->vbat_volt > bat_volt_lmt - 50 &&
		    (cp->ibus_curr < 750 || cp_sec->ibus_curr < 750)) {
			pdpm_log("second cp is disabled due to ibus < 750mA\n");
			usbpd_pm_enable_cp_sec(pdpm, false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
			pdpm->cp_sec_stopped = true;
		}
		break;

	case PD_PM_STATE_FC2_EXIT:
		usbpd_select_pdo(pdpm, 9000, 2000);
		if (pdpm->fcc_votable) {
			usleep_range(1000, 2000);
			vote(pdpm->fcc_votable, BQ_TAPER_FCC_VOTER, false, 0);
		}

		if (cp->charge_enabled) {
			usbpd_pm_enable_cp(pdpm, false);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pdpm->cfg.cp_sec_enable && cp_sec->charge_enabled) {
			usbpd_pm_enable_cp_sec(pdpm, false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}

		pdpm_log(">>>sw state %d %d\n", pdpm->stop_sw, sw->charge_enabled);
		if (pdpm->stop_sw && sw->charge_enabled)
			usbpd_pm_enable_sw(pdpm, false);
		else if (!pdpm->stop_sw && !sw->charge_enabled)
			usbpd_pm_enable_sw(pdpm, true);

		if (pdpm->recover) {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		} else {
			if (!nopmi_is_maxim_ic() && !nopmi_is_syv_ic())
				usbpd_pps_enable_charging(pdpm, false, 5000, 3000);
			done = true;
		}
		break;
	}

	return done;
}

static void usbpd_pm_workfunc(struct kthread_work *work)
{
	struct usbpd_pm *pdpm =
		container_of(work, struct usbpd_pm, pm_work.work);
	unsigned long interval;

	usbpd_pm_update_cp_status(pdpm);
	usbpd_pm_update_cp_sec_status(pdpm);
	usbpd_update_ibat_curr(pdpm);

	if (!usbpd_pm_sm(pdpm) && pdpm->pd_active) {
		interval = (pdpm->state == PD_PM_STATE_FC2_TUNE)
			? msecs_to_jiffies(PM_WORK_FC2_TUNE_INTERVAL)
			: msecs_to_jiffies(PM_WORK_RUN_INTERVAL);
		kthread_queue_delayed_work(pdpm->pm_worker,
					   &pdpm->pm_work, interval);
	}
}

static void usbpd_pm_disconnect(struct usbpd_pm *pdpm)
{
	usbpd_pm_enable_cp(pdpm, false);
	usbpd_pm_check_cp_enabled(pdpm);
	if (pdpm->cfg.cp_sec_enable) {
		usbpd_pm_enable_cp_sec(pdpm, false);
		usbpd_pm_check_cp_sec_enabled(pdpm);
	}
	kthread_cancel_delayed_work_sync(&pdpm->pm_work);

	if (!pdpm->sw.charge_enabled) {
		usbpd_pm_enable_sw(pdpm, true);
		usbpd_pm_check_sw_enabled(pdpm);
	}

	msleep(100);
	if (pdpm->fv_votable)
		vote(pdpm->fv_votable, JEITA_VOTER, true, 4450);
	if (pdpm->fcc_votable)
		vote(pdpm->fcc_votable, BQ_TAPER_FCC_VOTER, false, 0);

	pdpm->pps_supported = false;
	pdpm->apdo_selected_pdo = 0;
	pdpm->jeita_triggered = false;
	pdpm->is_temp_out_fc2_range = false;
	pdpm->cp_sec_stopped = false;
	pdpm->tune_vbus_retry = 0;
	pdpm->stop_sw = false;
	pdpm->recover = false;

	usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
}

static void usbpd_pd_contact(struct usbpd_pm *pdpm, bool connected)
{
	pdpm->pd_active = connected;
	pdpm_info("pd_active: %d\n", pdpm->pd_active);

	if (connected) {
		usleep_range(10000, 10500);
		if (nopmi_is_maxim_ic())
			usbpd_pm_evaluate_src_caps_maxim(pdpm);
		else
			usbpd_pm_evaluate_src_caps(pdpm);

		pdpm_info("start cp charging, pps_support: %d\n", pdpm->pps_supported);
		if (pdpm->pps_supported)
			kthread_queue_delayed_work(pdpm->pm_worker,
						   &pdpm->pm_work, 0);
		else
			pdpm->pd_active = false;
	} else {
		usbpd_pm_disconnect(pdpm);
	}
}

static int usbpd_check_plugout(struct usbpd_pm *pdpm)
{
	union power_supply_propval val = {0,};
	int ret;

	usbpd_check_usb_psy(pdpm);
	if (!pdpm->usb_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->usb_psy,
					POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret && !val.intval) {
		usbpd_pm_enable_cp(pdpm, false);
		usbpd_pm_check_cp_enabled(pdpm);
		if (pdpm->cfg.cp_sec_enable) {
			usbpd_pm_enable_cp_sec(pdpm, false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}
	}

	return ret;
}

static void usb_psy_change_work(struct kthread_work *work)
{
	union power_supply_propval propval = {0,};
	struct usbpd_pm *pdpm =
		container_of(work, struct usbpd_pm, usb_psy_change_work);
	int ret;

	usbpd_check_usb_psy(pdpm);
	if (!pdpm->usb_psy)
		goto out;

	usbpd_check_plugout(pdpm);

	ret = power_supply_get_property(pdpm->usb_psy,
					POWER_SUPPLY_PROP_PD_ACTIVE, &propval);
	if (ret < 0) {
		pdpm_err("get pd_active failed, ret=%d\n", ret);
		goto out;
	}

	pdpm_info("pre_pd_active: %d, now: %d\n", pdpm->pd_active, propval.intval);

	if (!pdpm->pd_active && propval.intval == 1) {
		if (nopmi_is_syv_ic())
			usbpd_select_pdo(pdpm, 9000, 2000);
	} else if (!pdpm->pd_active && propval.intval == 2) {
		usbpd_pd_contact(pdpm, true);
	} else if (pdpm->pd_active && !propval.intval) {
		usbpd_pd_contact(pdpm, false);
	}

out:
	atomic_set(&pdpm->psy_change_running, 0);
}

static int usbpd_psy_notifier_cb(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct usbpd_pm *pdpm = container_of(nb, struct usbpd_pm, pnb);
	struct power_supply *psy = data;

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	usbpd_check_usb_psy(pdpm);
	if (!pdpm->usb_psy)
		return NOTIFY_DONE;

	if (!nopmi_is_maxim_ic()) {
		usbpd_check_tcpc(pdpm);
		if (!pdpm->tcpc)
			return NOTIFY_DONE;
	}

	if (psy == pdpm->usb_psy) {
		pdpm_log("[CP manager] change_running: %d\n",
			 atomic_read(&pdpm->psy_change_running));

		if (atomic_cmpxchg(&pdpm->psy_change_running, 0, 1) == 0)
			kthread_queue_work(pdpm->pm_worker,
					   &pdpm->usb_psy_change_work);
	}

	return NOTIFY_OK;
}

static void usbpd_pm_parse_dt(struct usbpd_pm *pdpm)
{
	struct device_node *np = pdpm->dev->of_node;
	struct pdpm_config *cfg = &pdpm->cfg;

	cfg->bat_volt_lp_lmt = DT_READ_U32_OR(np, "bat_volt_lp_lmt", BATT_MAX_CHG_VOLT);
	cfg->bat_curr_lp_lmt = DT_READ_U32_OR(np, "bat_curr_lp_lmt", BATT_FAST_CHG_CURR);
	cfg->bus_volt_lp_lmt = DT_READ_U32_OR(np, "bus_volt_lp_lmt", BUS_OVP_THRESHOLD);
	cfg->bus_curr_lp_lmt = DT_READ_U32_OR(np, "bus_curr_lp_lmt", BATT_FAST_CHG_CURR >> 1);
	cfg->fc2_taper_current = DT_READ_U32_OR(np, "fc2_taper_current", 2300);
	cfg->fc2_steps = DT_READ_U32_OR(np, "fc2_steps", 1);
	cfg->min_adapter_volt_required = DT_READ_U32_OR(np, "min_adapter_volt_required", 9000);
	cfg->min_adapter_curr_required = DT_READ_U32_OR(np, "min_adapter_curr_required", 2000);
	cfg->min_vbat_for_cp = DT_READ_U32_OR(np, "min_vbat_for_cp", 3500);

	cfg->cp_sec_enable = of_property_read_bool(np, "cp_sec_enable");
	cfg->fc2_disable_sw = of_property_read_bool(np, "fc2_disable_sw");

	pr_info("cfg: vbat_lmt=%d ibat_lmt=%d vbus_lmt=%d ibus_lmt=%d\n",
		cfg->bat_volt_lp_lmt, cfg->bat_curr_lp_lmt,
		cfg->bus_volt_lp_lmt, cfg->bus_curr_lp_lmt);
	pr_info("cfg: taper=%d steps=%d adap_v=%d adap_i=%d vbat_min=%d\n",
		cfg->fc2_taper_current, cfg->fc2_steps,
		cfg->min_adapter_volt_required, cfg->min_adapter_curr_required,
		cfg->min_vbat_for_cp);
	pr_info("cfg: cp_sec_enable=%d fc2_disable_sw=%d\n",
		cfg->cp_sec_enable, cfg->fc2_disable_sw);
}

static int usbpd_pm_probe(struct platform_device *pdev)
{
	struct power_supply *cp_psy = NULL;
	struct power_supply *cp_sec_psy = NULL;
	struct power_supply *usb_psy = NULL;
	struct power_supply *batt_psy = NULL;
	struct power_supply *bms_psy = NULL;
	struct tcpc_device *tcpc = NULL;
	struct usbpd_pm *pdpm = NULL;
	bool cp_sec_enable = false;
	static int probe_cnt;
	int i, ret = 0;

	pr_info("start, probe_cnt: %d\n", ++probe_cnt);
	if (probe_cnt >= 50) {
		pr_err("stop: probe_cnt: %d, max probe_cnt is 50\n", probe_cnt);
		return -ENODEV;
	}

	cp_sec_enable = of_property_read_bool(pdev->dev.of_node, "cp_sec_enable");
	cp_psy = cp_sec_enable
		? power_supply_get_by_name("sc8551-master")
		: power_supply_get_by_name("sc8551-standalone");
	if (!cp_psy) {
		pr_err("cp_psy not ready, defer\n");
		return -EPROBE_DEFER;
	}

	if (cp_sec_enable) {
		cp_sec_psy = power_supply_get_by_name("sc8551-slave");
		if (!cp_sec_psy) {
			pr_err("cp_sec_psy not ready, defer\n");
			ret = -EPROBE_DEFER;
			goto err_put_cp;
		}
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb_psy not ready, defer\n");
		ret = -EPROBE_DEFER;
		goto err_put_cp_sec;
	}

	if (!nopmi_is_maxim_ic()) {
		tcpc = tcpc_dev_get_by_name("type_c_port0");
		if (!tcpc) {
			pr_err("tcpc not ready, defer\n");
			ret = -EPROBE_DEFER;
			goto err_put_usb;
		}
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("batt_psy not ready, defer\n");
		ret = -EPROBE_DEFER;
		goto err_put_tcpc;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("bms_psy not ready, defer\n");
		ret = -EPROBE_DEFER;
		goto err_put_batt;
	}

	pdpm = devm_kzalloc(&pdev->dev, sizeof(*pdpm), GFP_KERNEL);
	if (!pdpm) {
		ret = -ENOMEM;
		goto err_put_bms;
	}

	pdpm->dev = &pdev->dev;
	pdpm->pdev = pdev;
	platform_set_drvdata(pdev, pdpm);

	pdpm->cp_psy = cp_psy;
	pdpm->cp_sec_psy = cp_sec_psy;
	pdpm->usb_psy = usb_psy;
	pdpm->tcpc = tcpc;
	pdpm->batt_psy = batt_psy;
	pdpm->bms_psy = bms_psy;
	usbpd_pm_parse_dt(pdpm);

	for (i = 0; i < PDPM_RL_SLOTS; i++) {
		ratelimit_state_init(&pdpm->rl_slots[i], PDPM_RL_INTERVAL, PDPM_RL_BURST);
		pdpm->rl_slots[i].flags |= RATELIMIT_MSG_ON_RELEASE;
	}
	atomic_set(&pdpm->psy_change_running, 0);

	pdpm->pm_worker = kthread_create_worker(0, "pdpm_pm_worker");
	if (IS_ERR(pdpm->pm_worker)) {
		ret = PTR_ERR(pdpm->pm_worker);
		pr_err("failed to create pm_worker: %d\n", ret);
		goto err_free;
	}
	sched_set_fifo(pdpm->pm_worker->task);

	kthread_init_work(&pdpm->usb_psy_change_work, usb_psy_change_work);
	kthread_init_delayed_work(&pdpm->pm_work, usbpd_pm_workfunc);

	pdpm->pnb.notifier_call = usbpd_psy_notifier_cb;
	ret = power_supply_reg_notifier(&pdpm->pnb);
	if (ret < 0) {
		pr_err("power_supply_reg_notifier failed: %d\n", ret);
		goto err_destroy_wq;
	}

	if (!nopmi_is_maxim_ic()) {
		pdpm->tcp_nb.notifier_call = pca_pps_tcp_notifier_call;
		pdpm->tcp_nb.priority = TCP_NOTIFY_PRIO_POLICY_MANAGER;
		ret = register_tcp_dev_notifier(pdpm->tcpc, &pdpm->tcp_nb,
						TCP_NOTIFY_TYPE_USB);
		if (ret < 0) {
			pr_err("register tcpc notifier fail: %d\n", ret);
			goto err_unreg_psy_noti;
		}
	}

	pdpm->fcc_votable = find_votable("FCC");
	pdpm->fv_votable = find_votable("FV");

	pr_info("success\n");
	return 0;

err_unreg_psy_noti:
	power_supply_unreg_notifier(&pdpm->pnb);
err_destroy_wq:
	if (!IS_ERR_OR_NULL(pdpm->pm_worker))
		kthread_destroy_worker(pdpm->pm_worker);
err_free:
	platform_set_drvdata(pdev, NULL);
err_put_bms:
	power_supply_put(bms_psy);
err_put_batt:
	power_supply_put(batt_psy);
err_put_tcpc:
	if (!nopmi_is_maxim_ic())
		tcpc_dev_put(tcpc);
err_put_usb:
	power_supply_put(usb_psy);
err_put_cp_sec:
	if (cp_sec_enable)
		power_supply_put(cp_sec_psy);
err_put_cp:
	power_supply_put(cp_psy);

	pr_err("fail!\n");
	return ret;
}

static int usbpd_pm_remove(struct platform_device *pdev)
{
	struct usbpd_pm *pdpm = platform_get_drvdata(pdev);

	if (!pdpm)
		return 0;

	if (pdpm->tcpc)
		unregister_tcp_dev_notifier(pdpm->tcpc, &pdpm->tcp_nb,
					    TCP_NOTIFY_TYPE_USB);

	power_supply_unreg_notifier(&pdpm->pnb);

	kthread_cancel_delayed_work_sync(&pdpm->pm_work);
	kthread_cancel_work_sync(&pdpm->usb_psy_change_work);

	if (!IS_ERR_OR_NULL(pdpm->pm_worker))
		kthread_destroy_worker(pdpm->pm_worker);

	usbpd_clear_psy(pdpm);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id usbpd_pm_dt_match[] = {
	{ .compatible = "xiaomi,cp_manager", },
	{ },
};
MODULE_DEVICE_TABLE(of, usbpd_pm_dt_match);

static struct platform_driver usbpd_pm_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "cp_manager",
		.of_match_table	= of_match_ptr(usbpd_pm_dt_match),
	},
	.probe	= usbpd_pm_probe,
	.remove	= usbpd_pm_remove,
};

static int __init usbpd_pm_init(void)
{
	return platform_driver_register(&usbpd_pm_driver);
}

static void __exit usbpd_pm_exit(void)
{
	platform_driver_unregister(&usbpd_pm_driver);
}

late_initcall(usbpd_pm_init);
module_exit(usbpd_pm_exit);

MODULE_AUTHOR("Xiaomi PPM");
MODULE_DESCRIPTION("pd_policy_manager");
MODULE_LICENSE("GPL");
