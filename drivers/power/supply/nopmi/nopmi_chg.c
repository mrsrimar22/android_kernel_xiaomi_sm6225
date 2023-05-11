// SPDX-License-Identifier: GPL-2.0-only
/*
 * nopmi_chg - NOPMI charger parent driver
 */
#define pr_fmt(fmt) "[nopmi_chg]: %s: " fmt, __func__

#include "nopmi_chg.h"

#define PROBE_CNT_MAX			50
#define MAIN_CHG_SUSPEND_VOTER		"MAIN_CHG_SUSPEND_VOTER"
#define CHG_INPUT_SUSPEND_VOTER		"CHG_INPUT_SUSPEND_VOTER"
#define THERMAL_DAEMON_VOTER		"THERMAL_DAEMON_VOTER"
#define MAIN_ICL_MIN			100
#define WORKFUNC_GAP_MS			10000
#define CV_STEP_WORKFUNC_GAP_MS		2000
#define BATT_VOLT_LOW_MV		3300
#define BATT_FULL_DESIGN_UAH		5000000

static const struct step_config cc_cv_step[STEP_TABLE_MAX] = {
	{ 4192, 5400 },
	{ 4440, 3920 },
};

static int nopmi_set_prop_input_suspend(struct nopmi_chg *chg,
					const union power_supply_propval *val)
{
	bool suspend = !!val->intval;
	int ret;

	if (!chg->usb_icl_votable || !chg->fcc_votable) {
		pr_err("missing votable(s), cannot %s input suspend\n",
		       suspend ? "apply" : "clear");
		return -ENODEV;
	}

	if (!nopmi_is_maxim_ic() &&
	    !chg->chg_dis_votable) {
		pr_err("chg_dis_votable missing, cannot %s input suspend\n",
		       suspend ? "apply" : "clear");
		return -ENODEV;
	}

	pr_info("%s suspend votes\n", suspend ? "applying" : "clearing");

	ret = vote(chg->fcc_votable, CHG_INPUT_SUSPEND_VOTER, suspend, 0);
	ret |= vote(chg->usb_icl_votable, CHG_INPUT_SUSPEND_VOTER,
		    suspend, suspend ? MAIN_ICL_MIN : 0);
	if (chg->chg_dis_votable)
		ret |= vote(chg->chg_dis_votable, CHG_INPUT_SUSPEND_VOTER,
			    suspend, 0);

	if (ret < 0) {
		pr_err("couldn't %s input suspend votes, ret=%d\n",
		       suspend ? "apply" : "clear", ret);
		return ret;
	}

	chg->input_suspend = suspend;
	return 0;
}

static int nopmi_update_batt_prop(struct nopmi_chg *chg,
				  enum power_supply_property prop)
{
	union power_supply_propval val = {0, };
	int ret;

	if (!chg || !chg->bms_psy)
		return -EINVAL;

	ret = power_supply_get_property(chg->bms_psy, prop, &val);
	if (ret < 0) {
		pr_err("read batt prop %d fail, ret=%d\n", prop, ret);
		return ret;
	}

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		WRITE_ONCE(chg->batt_temp, val.intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		WRITE_ONCE(chg->batt_curr, val.intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		WRITE_ONCE(chg->batt_volt, val.intval);
		break;
	default:
		break;
	}

	return 0;
}

static int nopmi_set_prop_system_temp_level(struct nopmi_chg *chg,
					    const union power_supply_propval *val)
{
	bool want_icl_disabled;
	int want_fcc_ma;
	int ret;

	if (!chg)
		return -EINVAL;

	if (val->intval < 0 || chg->thermal_levels <= 0 ||
	    (u32)val->intval >= (u32)chg->thermal_levels)
		return -EINVAL;

	if (val->intval == chg->system_temp_level)
		return 0;

	chg->system_temp_level = val->intval;

	if (chg->system_temp_level == chg->thermal_levels - 1) {
		want_icl_disabled = true;
		want_fcc_ma = 0;
	} else {
		want_icl_disabled = false;
		want_fcc_ma = (chg->system_temp_level == 0)
			      ? 0
			      : chg->thermal_mitigation[chg->system_temp_level] / 1000;
	}

	if (!chg->last_thermal_icl_valid ||
	    chg->last_thermal_icl_disabled != want_icl_disabled) {
		ret = vote(chg->usb_icl_votable, THERMAL_DAEMON_VOTER,
			   want_icl_disabled, 0);
		if (ret < 0) {
			pr_err("icl vote failed, ret=%d\n", ret);
			return ret;
		}
		chg->last_thermal_icl_disabled = want_icl_disabled;
		chg->last_thermal_icl_valid = true;
	}

	ret = (want_fcc_ma == 0)
	      ? vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0)
	      : vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true, want_fcc_ma);
	if (ret < 0) {
		pr_err("fcc vote failed, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static enum power_supply_property nopmi_batt_props[] = {
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_INPUT_SUSPEND,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_SOC_DECIMAL,
	POWER_SUPPLY_PROP_SOC_DECIMAL_RATE,
	POWER_SUPPLY_PROP_SHUTDOWN_DELAY,
};

static int nopmi_batt_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct nopmi_chg *chg = power_supply_get_drvdata(psy);
	int batt_volt_mv;
	int ret = 0;

	if (nopmi_is_maxim_ic()) {
		switch (psp) {
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
			break;	/* fall through to shared handler */
		default:
			return max77729_batt_get_property(psy, psp, val);
		}
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = power_supply_get_property(chg->bbc_psy, psp, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		batt_volt_mv = READ_ONCE(chg->batt_volt) / 1000;
		ret = power_supply_get_property(chg->bbc_psy, psp, val);
		if (ret < 0)
			return ret;

		if (chg->input_suspend || batt_volt_mv < BATT_VOLT_LOW_MV)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if ((val->intval == POWER_SUPPLY_STATUS_DISCHARGING ||
			  val->intval == POWER_SUPPLY_STATUS_NOT_CHARGING) &&
			 chg->real_type > 0)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (chg->pd_active)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;

		if (val->intval != chg->last_batt_status) {
			chg->last_batt_status = val->intval;
			power_supply_changed(chg->batt_psy);
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = nopmi_update_batt_prop(chg, POWER_SUPPLY_PROP_TEMP);
		if (ret < 0)
			return ret;
		val->intval = READ_ONCE(chg->batt_temp);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = nopmi_update_batt_prop(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW);
		if (ret < 0)
			return ret;
		val->intval = READ_ONCE(chg->batt_volt);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = nopmi_update_batt_prop(chg, POWER_SUPPLY_PROP_CURRENT_NOW);
		if (ret < 0)
			return ret;
		val->intval = READ_ONCE(chg->batt_curr);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_SHUTDOWN_DELAY:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_SOC_DECIMAL:
	case POWER_SUPPLY_PROP_SOC_DECIMAL_RATE:
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = power_supply_get_property(chg->bms_psy, psp, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = BATT_FULL_DESIGN_UAH;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = !(get_client_vote(chg->usb_icl_votable,
						MAIN_CHG_SUSPEND_VOTER) == MAIN_ICL_MIN);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		val->intval = chg->input_suspend;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = chg->system_temp_level;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = chg->thermal_levels;
		break;
	default:
		pr_debug("prop %d not supported\n", psp);
		ret = -EINVAL;
		break;
	}

	if (ret < 0)
		return -ENODATA;

	return 0;
}

static int nopmi_batt_set_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct nopmi_chg *chg = power_supply_get_drvdata(psy);

	if (nopmi_is_maxim_ic()) {
		switch (psp) {
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
			break;	/* fall through to shared handler */
		default:
			return max77729_batt_set_property(psy, psp, val);
		}
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		if (val->intval == 0)
			vote(chg->usb_icl_votable, MAIN_CHG_SUSPEND_VOTER, true, MAIN_ICL_MIN);
		else
			vote(chg->usb_icl_votable, MAIN_CHG_SUSPEND_VOTER, false, 0);
		return 0;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		pr_info("set input_suspend: %d\n", val->intval);
		return nopmi_set_prop_input_suspend(chg, val);
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return nopmi_set_prop_system_temp_level(chg, val);
	default:
		return -EINVAL;
	}
}

static int nopmi_batt_prop_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	if (nopmi_is_maxim_ic())
		return batt_prop_is_writeable(psy, psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return 1;
	default:
		return 0;
	}
}

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= nopmi_batt_props,
	.num_properties		= ARRAY_SIZE(nopmi_batt_props),
	.get_property		= nopmi_batt_get_prop,
	.set_property		= nopmi_batt_set_prop,
	.property_is_writeable	= nopmi_batt_prop_is_writeable,
};

static int nopmi_init_batt_psy(struct nopmi_chg *chg)
{
	struct power_supply_config cfg = {
		.drv_data	 = chg,
		.of_node	 = chg->dev->of_node,
	};

	chg->batt_psy = devm_power_supply_register(chg->dev,
						   &batt_psy_desc,
						   &cfg);
	if (IS_ERR(chg->batt_psy)) {
		pr_err("couldn't register battery psy\n");
		return PTR_ERR(chg->batt_psy);
	}

	return 0;
}

static enum power_supply_property nopmi_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_PD_ACTIVE,
	POWER_SUPPLY_PROP_PD_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_PD_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_PD_CURRENT_MAX,
	POWER_SUPPLY_PROP_PD_USB_SUSPEND_SUPPORTED,
	POWER_SUPPLY_PROP_PD_IN_HARD_RESET,
	POWER_SUPPLY_PROP_TYPEC_MODE,
	POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
	POWER_SUPPLY_PROP_QUICK_CHARGE_TYPE,
	POWER_SUPPLY_PROP_MTBF_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static int nopmi_usb_get_prop(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct nopmi_chg *chg = power_supply_get_drvdata(psy);
	int batt_volt_mv;
	int ret = 0;

	if (nopmi_is_maxim_ic()) {
		switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			ret = nopmi_update_batt_prop(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW);
			if (ret < 0)
				return ret;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_QUICK_CHARGE_TYPE:
		case POWER_SUPPLY_PROP_MTBF_CURRENT:
		case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
			break;
		default:
			return max77729_usb_get_property(psy, psp, val);
		}
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = (chg->real_type > 0) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		batt_volt_mv = READ_ONCE(chg->batt_volt) / 1000;
		val->intval = (chg->usb_online > 0 &&
			       batt_volt_mv >= BATT_VOLT_LOW_MV) ? 1 : 0;

		if (val->intval != chg->last_usb_online) {
			chg->last_usb_online = val->intval;
			WRITE_ONCE(chg->is_awake, val->intval);
			queue_delayed_work(chg->usb_online_wq,
					   &chg->usb_online_work,
					   msecs_to_jiffies(100));
		}
		break;
	case POWER_SUPPLY_PROP_QUICK_CHARGE_TYPE:
		val->intval = nopmi_get_quick_charge_type(chg);
		break;
	case POWER_SUPPLY_PROP_MTBF_CURRENT:
		val->intval = chg->mtbf_cur;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		val->intval = get_effective_result(chg->usb_icl_votable);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = get_effective_result(chg->fv_votable);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = power_supply_get_property(chg->bbc_psy, psp, val);
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = chg->pd_active ? POWER_SUPPLY_TYPE_USB_PD : chg->real_type;
		break;
	case POWER_SUPPLY_PROP_PD_ACTIVE:
		val->intval = chg->pd_active;
		break;
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MIN:
		val->intval = chg->pd_min_vol;
		break;
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MAX:
		val->intval = chg->pd_max_vol;
		break;
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
		val->intval = chg->pd_cur_max;
		break;
	case POWER_SUPPLY_PROP_PD_USB_SUSPEND_SUPPORTED:
		val->intval = chg->pd_usb_suspend;
		break;
	case POWER_SUPPLY_PROP_PD_IN_HARD_RESET:
		val->intval = chg->pd_in_hard_reset;
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		val->intval = chg->typec_mode;
		break;
	case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
		val->intval = chg->cc_orientation;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_SCOPE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_POWER_NOW:
		break;
	default:
		pr_debug("prop %d not supported\n", psp);
		ret = -EINVAL;
		break;
	}

	if (ret < 0)
		return -ENODATA;

	return 0;
}

static int nopmi_usb_set_prop(struct power_supply *psy,
			      enum power_supply_property psp,
			      const union power_supply_propval *val)
{
	struct nopmi_chg *chg = power_supply_get_drvdata(psy);

	if (nopmi_is_maxim_ic()) {
		switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
		case POWER_SUPPLY_PROP_MTBF_CURRENT:
		case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
			break;	/* fall through to shared handler */
		case POWER_SUPPLY_PROP_REAL_TYPE:
			chg->real_type = val->intval;
			return max77729_usb_set_property(psy, psp, val);
		case POWER_SUPPLY_PROP_PD_ACTIVE:
			chg->pd_active = val->intval;
			chg->usb_online = chg->pd_active ? 1 : 0;
			return max77729_usb_set_property(psy, psp, val);
		default:
			return max77729_usb_set_property(psy, psp, val);
		}
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return power_supply_set_property(chg->bbc_psy, psp, val);
	case POWER_SUPPLY_PROP_PD_ACTIVE:
		chg->pd_active = val->intval;
		chg->usb_online = chg->pd_active ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MIN:
		chg->pd_min_vol = val->intval;
		break;
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MAX:
		chg->pd_max_vol = val->intval;
		break;
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
		chg->pd_cur_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_PD_USB_SUSPEND_SUPPORTED:
		chg->pd_usb_suspend = val->intval;
		break;
	case POWER_SUPPLY_PROP_PD_IN_HARD_RESET:
		chg->pd_in_hard_reset = val->intval;
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		chg->typec_mode = val->intval;
		break;
	case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
		chg->cc_orientation = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		chg->usb_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_MTBF_CURRENT:
		pr_info("MTBF current: %d\n", val->intval);
		chg->mtbf_cur = val->intval;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		chg->real_type = val->intval;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_POWER_NOW:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int nopmi_usb_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property psp)
{
	if (nopmi_is_maxim_ic())
		return usb_prop_is_writeable(psy, psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_POWER_NOW:
	case POWER_SUPPLY_PROP_PD_ACTIVE:
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MIN:
	case POWER_SUPPLY_PROP_PD_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_PD_CURRENT_MAX:
	case POWER_SUPPLY_PROP_REAL_TYPE:
	case POWER_SUPPLY_PROP_TYPEC_MODE:
	case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
	case POWER_SUPPLY_PROP_MTBF_CURRENT:
		return 1;
	default:
		return 0;
	}
}

static const struct power_supply_desc usb_psy_desc = {
	.name			= "usb",
	.type			= POWER_SUPPLY_TYPE_USB_PD,
	.properties		= nopmi_usb_props,
	.num_properties		= ARRAY_SIZE(nopmi_usb_props),
	.get_property		= nopmi_usb_get_prop,
	.set_property		= nopmi_usb_set_prop,
	.property_is_writeable	= nopmi_usb_prop_is_writeable,
};

static int nopmi_init_usb_psy(struct nopmi_chg *chg)
{
	struct power_supply_config cfg = {
		.drv_data	 = chg,
		.of_node	 = chg->dev->of_node,
	};

	chg->usb_psy = devm_power_supply_register(chg->dev,
						  &usb_psy_desc,
						  &cfg);
	if (IS_ERR(chg->usb_psy)) {
		pr_err("couldn't register USB psy\n");
		return PTR_ERR(chg->usb_psy);
	}

	return 0;
}

#define JEITA_PARSE_S32(np, key, field, def)			\
do {								\
	s32 _v;							\
	if (of_property_read_s32(np, key, &_v) >= 0) {		\
		(field) = _v;					\
	} else {						\
		pr_err("use default " #def ": %d\n", (def));	\
		(field) = (def);				\
	}							\
} while (0)

static int nopmi_parse_dt_jeita(struct nopmi_chg *chg,
				struct device_node *np)
{
	struct jeita_config *cfg = &chg->jeita_st->cfg;

	cfg->enable_sw_jeita = of_property_read_bool(np, "enable_sw_jeita");

	JEITA_PARSE_S32(np, "jeita_temp_above_t4_cv",
			cfg->cv_above_t4, JEITA_CV_ABOVE_T4);
	JEITA_PARSE_S32(np, "jeita_temp_t3_to_t4_cv",
			cfg->cv_t3_to_t4, JEITA_CV_T3_TO_T4);
	JEITA_PARSE_S32(np, "jeita_temp_t2_to_t3_cv",
			cfg->cv_t2_to_t3, JEITA_CV_T2_TO_T3);
	JEITA_PARSE_S32(np, "jeita_temp_t1p5_to_t2_cv",
			cfg->cv_t1p5_to_t2, JEITA_CV_T1P5_TO_T2);
	JEITA_PARSE_S32(np, "jeita_temp_t1_to_t1p5_cv",
			cfg->cv_t1_to_t1p5, JEITA_CV_T1_TO_T1P5);
	JEITA_PARSE_S32(np, "jeita_temp_t0_to_t1_cv",
			cfg->cv_t0_to_t1, JEITA_CV_T0_TO_T1);
	JEITA_PARSE_S32(np, "jeita_temp_tn1_to_t0_cv",
			cfg->cv_tn1_to_t0, JEITA_CV_TN1_TO_T0);
	JEITA_PARSE_S32(np, "jeita_temp_below_t0_cv",
			cfg->cv_below_t0, JEITA_CV_BELOW_T0);
	JEITA_PARSE_S32(np, "normal-charge-voltage",
			cfg->cv_normal, JEITA_CV_NORMAL);
	JEITA_PARSE_S32(np, "temp_t4_thres",
			cfg->t4, JEITA_T4_THRES);
	JEITA_PARSE_S32(np, "temp_t4_thres_minus_x_degree",
			cfg->t4_minus_x, JEITA_T4_THRES_MINUS_X);
	JEITA_PARSE_S32(np, "temp_t3_thres",
			cfg->t3, JEITA_T3_THRES);
	JEITA_PARSE_S32(np, "temp_t3_thres_minus_x_degree",
			cfg->t3_minus_x, JEITA_T3_THRES_MINUS_X);
	JEITA_PARSE_S32(np, "temp_t2_thres",
			cfg->t2, JEITA_T2_THRES);
	JEITA_PARSE_S32(np, "temp_t2_thres_plus_x_degree",
			cfg->t2_plus_x, JEITA_T2_THRES_PLUS_X);
	JEITA_PARSE_S32(np, "temp_t1p5_thres",
			cfg->t1p5, JEITA_T1P5_THRES);
	JEITA_PARSE_S32(np, "temp_t1p5_thres_plus_x_degree",
			cfg->t1p5_plus_x, JEITA_T1P5_THRES_PLUS_X);
	JEITA_PARSE_S32(np, "temp_t1_thres",
			cfg->t1, JEITA_T1_THRES);
	JEITA_PARSE_S32(np, "temp_t1_thres_plus_x_degree",
			cfg->t1_plus_x, JEITA_T1_THRES_PLUS_X);
	JEITA_PARSE_S32(np, "temp_t0_thres",
			cfg->t0, JEITA_T0_THRES);
	JEITA_PARSE_S32(np, "temp_t0_thres_plus_x_degree",
			cfg->t0_plus_x, JEITA_T0_THRES_PLUS_X);
	JEITA_PARSE_S32(np, "temp_tn1_thres",
			cfg->tn1, JEITA_TN1_THRES);
	JEITA_PARSE_S32(np, "temp_tn1_thres_plus_x_degree",
			cfg->tn1_plus_x, JEITA_TN1_THRES_PLUS_X);
	JEITA_PARSE_S32(np, "temp_neg_10_thres",
			cfg->tneg10, JEITA_NEG10_THRES);
	JEITA_PARSE_S32(np, "temp_t3_to_t4_fcc",
			cfg->fcc_t3_to_t4, JEITA_FCC_T3_TO_T4);
	JEITA_PARSE_S32(np, "temp_t2_to_t3_fcc",
			cfg->fcc_t2_to_t3, JEITA_FCC_T2_TO_T3);
	JEITA_PARSE_S32(np, "temp_t1p5_to_t2_fcc",
			cfg->fcc_t1p5_to_t2, JEITA_FCC_T1P5_TO_T2);
	JEITA_PARSE_S32(np, "temp_t1_to_t1p5_fcc",
			cfg->fcc_t1_to_t1p5, JEITA_FCC_T1_TO_T1P5);
	JEITA_PARSE_S32(np, "temp_t0_to_t1_fcc",
			cfg->fcc_t0_to_t1, JEITA_FCC_T0_TO_T1);
	JEITA_PARSE_S32(np, "temp_tn1_to_t0_fcc",
			cfg->fcc_tn1_to_t0, JEITA_FCC_TN1_TO_T0);

	return 0;
}

static int nopmi_parse_dt_thermal(struct nopmi_chg *chg,
				  struct device_node *np)
{
	int byte_len, ret;

	if (!of_find_property(np, "nopmi,thermal-mitigation", &byte_len)) {
		pr_err("missing thermal-mitigation\n");
		return -EINVAL;
	}

	if (byte_len % sizeof(u32)) {
		pr_err("invalid thermal-mitigation length: %d\n", byte_len);
		return -EINVAL;
	}

	chg->thermal_levels = byte_len / sizeof(u32);
	if (chg->thermal_levels > MAX_THERMAL_LEVELS) {
		pr_err("too many thermal levels: %d\n", chg->thermal_levels);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "nopmi,thermal-mitigation",
					 chg->thermal_mitigation,
					 chg->thermal_levels);
	if (ret < 0) {
		pr_err("failed to read thermal-mitigation, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

static int nopmi_parse_dt(struct nopmi_chg *chg)
{
	struct device_node *np = chg->dev->of_node;
	int ret;

	if (!np) {
		pr_err("device tree node missing\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "qcom,fv-max-uv",
				   &chg->dt.batt_profile_fv_uv);
	if (ret < 0)
		chg->dt.batt_profile_fv_uv = -EINVAL;
	else
		pr_info("fv-max-uv: %d\n", chg->dt.batt_profile_fv_uv);

	ret = nopmi_parse_dt_jeita(chg, np);
	if (ret < 0)
		return ret;

	return nopmi_parse_dt_thermal(chg, np);
}

static void nopmi_cv_step_monitor(struct nopmi_chg *chg)
{
	int batt_curr, batt_volt;
	u32 i, stepdown = 0, final_fcc, vote_fcc;
	int ret;

	if (!chg || !chg->bms_psy)
		return;

	ret = nopmi_update_batt_prop(chg, POWER_SUPPLY_PROP_CURRENT_NOW);
	if (ret < 0) {
		pr_err("fail get CURRENT_NOW\n");
		return;
	}
	batt_curr = READ_ONCE(chg->batt_curr) / 1000;

	ret = nopmi_update_batt_prop(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (ret < 0) {
		pr_err("fail get VOLTAGE_NOW\n");
		return;
	}
	batt_volt = READ_ONCE(chg->batt_volt) / 1000;

	pr_info("volt=%d mV, curr=%d mA\n", batt_volt, batt_curr);

	if (!chg->fcc_votable)
		return;

	if (batt_curr < 0) {
		if (chg->last_vote_fcc != 0) {
			vote(chg->fcc_votable, CC_CV_STEP_VOTER, false, 0);
			chg->last_vote_fcc = 0;
		}
		for (i = 0; i < STEP_TABLE_MAX; i++)
			chg->cv_step_count[i] = 0;
		return;
	}

	for (i = 0; i < STEP_TABLE_MAX; i++) {
		if (batt_volt >= cc_cv_step[i].volt_lim - CV_BATT_VOLT_HYSTERESIS &&
		    batt_curr > cc_cv_step[i].curr_lim) {
			if (++chg->cv_step_count[i] >= 2) {
				stepdown = 1;
				chg->cv_step_count[i] = 0;
				pr_info("stepdown at step %u\n", i);
			}
			break;
		}
		chg->cv_step_count[i] = 0;
	}

	if (!stepdown)
		return;

	final_fcc = get_effective_result(chg->fcc_votable);
	vote_fcc = (final_fcc - cc_cv_step[i].curr_lim < STEP_DOWN_CURR_MA)
		   ? cc_cv_step[i].curr_lim
		   : final_fcc - STEP_DOWN_CURR_MA;

	if (chg->last_vote_fcc == (s32)vote_fcc) {
		pr_debug("skip cccv vote: last=%u new=%u\n",
			 chg->last_vote_fcc, vote_fcc);
		return;
	}

	vote(chg->fcc_votable, CC_CV_STEP_VOTER, true, vote_fcc);
	chg->last_vote_fcc = vote_fcc;
	pr_info("step=%u vote_fcc=%umA final_fcc=%umA\n",
		i, vote_fcc, final_fcc);
}

static void nopmi_chg_handle_workfunc(struct nopmi_chg *chg, bool en)
{
	pm_stay_awake(chg->dev);

	if (en) {
		schedule_delayed_work(&chg->chg_work, 0);
		schedule_delayed_work(&chg->cv_step_work,
				      msecs_to_jiffies(CV_STEP_WORKFUNC_GAP_MS));
	} else {
		cancel_delayed_work_sync(&chg->chg_work);
		cancel_delayed_work_sync(&chg->cv_step_work);
		nopmi_cv_step_monitor(chg);
		stop_nopmi_chg_jeita_workfunc(chg);
	}

	pm_relax(chg->dev);
}

static void nopmi_usb_online_workfunc(struct work_struct *work)
{
	struct nopmi_chg *chg =
		container_of(work, struct nopmi_chg, usb_online_work.work);
	bool en = READ_ONCE(chg->is_awake);

	pr_info("USB %s\n", en ? "connected" : "disconnected");
	power_supply_changed(chg->usb_psy);
	nopmi_chg_handle_workfunc(chg, en);
}

static void nopmi_reschedule_work(struct nopmi_chg *chg,
				  struct delayed_work *dwork,
				  unsigned long delay_ms,
				  const char *name)
{
	if (READ_ONCE(chg->is_awake))
		schedule_delayed_work(dwork, msecs_to_jiffies(delay_ms));
	else
		pr_info("usb offline, stop %s\n", name);
}

static void nopmi_cv_step_workfunc(struct work_struct *work)
{
	struct nopmi_chg *chg =
		container_of(work, struct nopmi_chg, cv_step_work.work);

	nopmi_cv_step_monitor(chg);
	nopmi_reschedule_work(chg, &chg->cv_step_work,
			      CV_STEP_WORKFUNC_GAP_MS, "cv step monitor");
}

static void nopmi_chg_workfunc(struct work_struct *work)
{
	struct nopmi_chg *chg =
		container_of(work, struct nopmi_chg, chg_work.work);

	if (READ_ONCE(chg->is_awake))
		start_nopmi_chg_jeita_workfunc(chg);

	nopmi_reschedule_work(chg, &chg->chg_work,
			      WORKFUNC_GAP_MS, "nopmi monitor");
}

static int nopmi_chg_probe(struct platform_device *pdev)
{
	struct nopmi_chg *chg = NULL;
	struct power_supply *bms_psy = NULL;
	struct power_supply *bbc_psy = NULL;
	struct power_supply *cp_psy = NULL;
	static int probe_cnt;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	pr_info("start, probe_cnt: %d\n", ++probe_cnt);
	if (probe_cnt >= PROBE_CNT_MAX) {
		pr_err("probe count exceeded: %d >= %d\n",
		       probe_cnt, PROBE_CNT_MAX);
		return -ENODEV;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("get bms psy fail, defer\n");
		return -EPROBE_DEFER;
	}

	bbc_psy = power_supply_get_by_name("bbc");
	if (!bbc_psy) {
		pr_err("get bbc psy fail, defer\n");
		ret = -EPROBE_DEFER;
		goto err_put_bms;
	}

	cp_psy = power_supply_get_by_name("sc8551-standalone");
	if (!cp_psy) {
		pr_err("get cp psy fail, defer\n");
		ret = -EPROBE_DEFER;
		goto err_put_bbc;
	}

	chg = kzalloc(sizeof(*chg), GFP_KERNEL);
	if (!chg) {
		ret = -ENOMEM;
		goto err_put_cp;
	}

	chg->jeita_st = kzalloc(sizeof(*chg->jeita_st), GFP_KERNEL);
	if (!chg->jeita_st) {
		ret = -ENOMEM;
		goto err_free_chg;
	}

	chg->dev = &pdev->dev;
	chg->pdev = pdev;
	platform_set_drvdata(pdev, chg);

	chg->bms_psy = bms_psy;
	chg->bbc_psy = bbc_psy;
	chg->cp_psy = cp_psy;

	ret = nopmi_parse_dt(chg);
	if (ret < 0) {
		pr_err("Couldn't parse DT, ret=%d\n", ret);
		goto err_free_jeita;
	}

	chg->last_vote_fcc = -1;
	chg->last_thermal_icl_valid = false;
	chg->last_thermal_icl_disabled = false;
	chg->last_usb_online = 0;
	WRITE_ONCE(chg->batt_temp, 250);
	WRITE_ONCE(chg->batt_volt, 3800000);
	WRITE_ONCE(chg->batt_curr, 0);
	WRITE_ONCE(chg->is_awake, 0);

	chg->fcc_votable = find_votable("FCC");
	chg->fv_votable = find_votable("FV");
	chg->usb_icl_votable = find_votable("USB_ICL");

	if (nopmi_is_maxim_ic())
		chg->chgctrl_votable = find_votable("CHG_CTRL");
	else
		chg->chg_dis_votable = find_votable("CHG_DISABLE");

	if (nopmi_ic_is_known()) {
		device_set_wakeup_capable(chg->dev, true);
		ret = device_set_wakeup_enable(chg->dev, true);
		if (ret) {
			pr_err("Failed to enable wakeup, ret=%d\n", ret);
			device_set_wakeup_capable(chg->dev, false);
			goto err_free_jeita;
		}
	}

	INIT_DELAYED_WORK(&chg->usb_online_work, nopmi_usb_online_workfunc);
	INIT_DELAYED_WORK(&chg->chg_work, nopmi_chg_workfunc);
	INIT_DELAYED_WORK(&chg->cv_step_work, nopmi_cv_step_workfunc);

	chg->usb_online_wq = alloc_ordered_workqueue("nopmi_usb_online_wq",
						      WQ_MEM_RECLAIM);
	if (!chg->usb_online_wq) {
		pr_err("Failed to create usb_online_wq\n");
		ret = -ENOMEM;
		goto err_wake;
	}

	nopmi_chg_jeita_init(chg);

	ret = nopmi_init_batt_psy(chg);
	if (ret < 0) {
		pr_err("Couldn't init batt psy, ret=%d\n", ret);
		goto err_destroy_wq;
	}

	ret = nopmi_init_usb_psy(chg);
	if (ret < 0) {
		pr_err("Couldn't init usb psy, ret=%d\n", ret);
		goto err_destroy_wq;
	}

	pr_info("probe success\n");
	return 0;

err_destroy_wq:
	nopmi_chg_jeita_deinit(chg);
	if (chg->usb_online_wq)
		destroy_workqueue(chg->usb_online_wq);
err_wake:
	if (nopmi_ic_is_known()) {
		device_set_wakeup_enable(chg->dev, false);
		device_set_wakeup_capable(chg->dev, false);
	}
err_free_jeita:
	platform_set_drvdata(pdev, NULL);
	kfree(chg->jeita_st);
err_free_chg:
	kfree(chg);
err_put_cp:
	power_supply_put(cp_psy);
err_put_bbc:
	power_supply_put(bbc_psy);
err_put_bms:
	power_supply_put(bms_psy);

	pr_err("probe fail, ret=%d\n", ret);
	return ret;
}

static int nopmi_chg_remove(struct platform_device *pdev)
{
	struct nopmi_chg *chg = platform_get_drvdata(pdev);

	if (!chg)
		return 0;

	nopmi_chg_jeita_deinit(chg);
	cancel_delayed_work_sync(&chg->cv_step_work);
	cancel_delayed_work_sync(&chg->chg_work);
	cancel_delayed_work_sync(&chg->usb_online_work);
	if (chg->usb_online_wq)
		destroy_workqueue(chg->usb_online_wq);

	if (nopmi_ic_is_known())
		device_set_wakeup_enable(chg->dev, false);

	if (chg->cp_psy)
		power_supply_put(chg->cp_psy);
	if (chg->bbc_psy)
		power_supply_put(chg->bbc_psy);
	if (chg->bms_psy)
		power_supply_put(chg->bms_psy);

	platform_set_drvdata(pdev, NULL);
	kfree(chg->jeita_st);
	kfree(chg);

	return 0;
}

static const struct of_device_id nopmi_chg_dt_match[] = {
	{ .compatible = "qcom,nopmi-chg" },
	{ },
};
MODULE_DEVICE_TABLE(of, nopmi_chg_dt_match);

static struct platform_driver nopmi_chg_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "qcom,nopmi-chg",
		.of_match_table	= of_match_ptr(nopmi_chg_dt_match),
	},
	.probe	= nopmi_chg_probe,
	.remove	= nopmi_chg_remove,
};

module_platform_driver(nopmi_chg_driver);

MODULE_AUTHOR("WingTech Inc.");
MODULE_DESCRIPTION("NOPMI charger parent driver");
MODULE_LICENSE("GPL");
