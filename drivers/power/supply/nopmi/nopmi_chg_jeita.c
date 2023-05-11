// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[nopmi_chg_jeita]: %s: " fmt, __func__

#include "nopmi_chg.h"

static int jeita_get_batt_temp(struct nopmi_chg_jeita_st *jeita)
{
	union power_supply_propval prop = {0, };
	int ret;

	if (!jeita->parent->bms_psy) {
		pr_err("bms psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_get_property(jeita->parent->bms_psy,
					POWER_SUPPLY_PROP_TEMP,
					&prop);
	if (ret < 0) {
		pr_err("couldn't read temp, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("batt_temp: %d\n", prop.intval / 10);
	return prop.intval / 10;
}

static int jeita_get_charger_voltage(struct nopmi_chg_jeita_st *jeita)
{
	union power_supply_propval prop = {0, };
	int ret;

	if (!jeita->parent->bbc_psy) {
		pr_err("bbc psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_get_property(jeita->parent->bbc_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&prop);
	if (ret < 0) {
		pr_err("couldn't read voltage, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("charger_voltage: %d\n", prop.intval);
	return prop.intval;
}

static int jeita_get_batt_id(struct nopmi_chg_jeita_st *jeita)
{
	union power_supply_propval prop = {0, };
	int ret;

	if (!jeita->parent->bms_psy) {
		pr_err("bms psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_get_property(jeita->parent->bms_psy,
					POWER_SUPPLY_PROP_RESISTANCE_ID,
					&prop);
	if (ret < 0) {
		pr_err("couldn't read batt_id, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("batt_id: %d\n", prop.intval);
	return prop.intval;
}

static int jeita_get_pd_active(struct nopmi_chg_jeita_st *jeita)
{
	union power_supply_propval prop = {0, };
	int ret;

	if (!jeita->parent->usb_psy) {
		pr_err("usb psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_get_property(jeita->parent->usb_psy,
					POWER_SUPPLY_PROP_PD_ACTIVE,
					&prop);
	if (ret < 0) {
		pr_err("couldn't read pd_active, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("pd_active: %d\n", prop.intval);
	return prop.intval;
}

static int jeita_get_term_current(struct nopmi_chg_jeita_st *jeita)
{
	union power_supply_propval prop = {0, };
	int ret;

	if (!jeita->parent->usb_psy) {
		pr_err("usb psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_get_property(jeita->parent->usb_psy,
					POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
					&prop);
	if (ret < 0) {
		pr_err("couldn't read term_current, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("term_current: %d\n", prop.intval);
	return prop.intval;
}

static int jeita_set_charger_current(struct nopmi_chg_jeita_st *jeita, int cc)
{
	union power_supply_propval prop = { .intval = cc };
	int ret;

	if (!jeita->parent->bbc_psy) {
		pr_err("bbc psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_set_property(jeita->parent->bbc_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW,
					&prop);
	if (ret < 0) {
		pr_err("couldn't set current, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("set current: %d\n", cc);
	return 0;
}

static int jeita_set_charger_voltage(struct nopmi_chg_jeita_st *jeita, int cv)
{
	union power_supply_propval prop = { .intval = cv };
	int ret;

	if (!jeita->parent->bbc_psy) {
		pr_err("bbc psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_set_property(jeita->parent->bbc_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW,
					&prop);
	if (ret < 0) {
		pr_err("couldn't set voltage, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("set voltage: %d\n", cv);
	return 0;
}

static int jeita_set_term_current(struct nopmi_chg_jeita_st *jeita, int term)
{
	union power_supply_propval prop = { .intval = term };
	int ret;

	if (!jeita->parent->usb_psy) {
		pr_err("usb psy not available\n");
		return -EINVAL;
	}

	ret = power_supply_set_property(jeita->parent->usb_psy,
					POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
					&prop);
	if (ret < 0) {
		pr_err("couldn't set term_current, ret=%d\n", ret);
		return -EINVAL;
	}

	pr_info("set term_current: %d\n", term);
	return 0;
}

static void jeita_handle_current(struct nopmi_chg_jeita_st *jeita)
{
	struct nopmi_chg *chg = jeita->parent;
	struct jeita_config *cfg = &jeita->cfg;
	struct sw_jeita_data *sw = &jeita->sw;
	union power_supply_propval prop = {0, };
	int temp = jeita->battery_temp;
	int chg1_cv, pd_active, term_pre;
	bool cp_work_flag = false;
	int ret;

	sw->pre_sm = sw->sm;
	sw->charging = true;

	if (temp >= cfg->t4) {
		pr_err("[SW_JEITA] Over temperature (%d)\n", cfg->t4);
		sw->sm = TEMP_ABOVE_T4;
		sw->charging = false;
	} else if (temp < cfg->tn1) {
		pr_err("[SW_JEITA] Cold temperature (%d)\n", cfg->tn1);
		sw->sm = TEMP_BELOW_T0;
		sw->charging = false;
	} else if (temp > cfg->t3) {
		if (temp >= cfg->t4_minus_x) {
			pr_err("[SW_JEITA] %d <= temp < %d, charging blocked\n",
			       cfg->t4_minus_x, cfg->t4);
			sw->charging = false;
		} else {
			pr_err("[SW_JEITA] %d < temp <= %d\n", cfg->t3, cfg->t4);
			sw->sm = TEMP_T3_TO_T4;
			jeita->current_limit = cfg->fcc_t3_to_t4;
		}
	} else if (temp >= cfg->t2) {
		switch (sw->sm) {
		case TEMP_T3_TO_T4:
			if (temp >= cfg->t3_minus_x) {
				pr_err("[SW_JEITA] temp not recovered to normal mode\n");
				goto out_fcc;
			}
			break;
		case TEMP_T1P5_TO_T2:
			if (temp <= cfg->t2_plus_x) {
				pr_err("[SW_JEITA] temp not recovered to normal mode\n");
				goto out_fcc;
			}
			break;
		default:
			break;
		}
		pr_err("[SW_JEITA] %d <= temp < %d\n", cfg->t2, cfg->t3);
		sw->sm = TEMP_T2_TO_T3;
		jeita->current_limit = cfg->fcc_t2_to_t3;
	} else if (temp >= cfg->t1p5) {
		switch (sw->sm) {
		case TEMP_T1_TO_T1P5:
			if (temp <= cfg->t1p5_plus_x) {
				pr_err("[SW_JEITA] %d <= temp < %d\n",
				       cfg->t1p5_plus_x, cfg->t2);
				goto out_fcc;
			}
			break;
		case TEMP_T0_TO_T1:
			if (temp <= cfg->t1p5_plus_x) {
				pr_err("[SW_JEITA] %d <= temp < %d\n",
				       cfg->t1_plus_x, cfg->t1p5);
				goto out_fcc;
			}
			break;
		default:
			break;
		}
		pr_err("[SW_JEITA] %d <= temp < %d\n", cfg->t1p5, cfg->t2);
		sw->sm = TEMP_T1P5_TO_T2;
		jeita->current_limit = cfg->fcc_t1p5_to_t2;
	} else if (temp >= cfg->t1) {
		switch (sw->sm) {
		case TEMP_T0_TO_T1:
			if (temp <= cfg->t1_plus_x) {
				pr_err("[SW_JEITA] %d <= temp < %d\n",
				       cfg->t1_plus_x, cfg->t1p5);
				goto out_fcc;
			}
			break;
		case TEMP_BELOW_T0:
			if (temp <= cfg->t1_plus_x) {
				pr_err("[SW_JEITA] %d <= temp < %d, charging blocked\n",
				       cfg->tn1, cfg->tn1_plus_x);
				sw->charging = false;
				goto out_fcc;
			}
			break;
		case TEMP_TN1_TO_T0:
			if (temp <= cfg->t1_plus_x)
				goto out_fcc;
			break;
		default:
			break;
		}
		pr_err("[SW_JEITA] %d <= temp < %d\n", cfg->t1, cfg->t1p5);
		sw->sm = TEMP_T1_TO_T1P5;
		jeita->current_limit = cfg->fcc_t1_to_t1p5;
	} else if (temp >= cfg->t0) {
		switch (sw->sm) {
		case TEMP_BELOW_T0:
			if (temp <= cfg->t0_plus_x) {
				pr_err("[SW_JEITA] %d <= temp < %d, charging blocked\n",
				       cfg->tn1, cfg->tn1_plus_x);
				sw->charging = false;
				goto out_fcc;
			}
			break;
		case TEMP_TN1_TO_T0:
			if (temp <= cfg->t0_plus_x) {
				pr_err("[SW_JEITA] %d <= temp < %d\n",
				       cfg->t0_plus_x, cfg->tn1);
				goto out_fcc;
			}
			break;
		default:
			break;
		}
		pr_err("[SW_JEITA] %d <= temp < %d\n", cfg->t0, cfg->t1);
		sw->sm = TEMP_T0_TO_T1;
		jeita->current_limit = cfg->fcc_t0_to_t1;
	} else if (temp >= cfg->tn1) {
		if (sw->sm == TEMP_BELOW_T0 && temp <= cfg->tn1_plus_x) {
			pr_err("[SW_JEITA] %d <= temp < %d, charging blocked\n",
			       cfg->tn1, cfg->tn1_plus_x);
			sw->charging = false;
		} else {
			pr_err("[SW_JEITA] %d <= temp < %d\n",
			       cfg->t0, cfg->tn1);
			sw->sm = TEMP_TN1_TO_T0;
			jeita->current_limit = cfg->fcc_tn1_to_t0;
		}
	}

out_fcc:

	if (chg->fcc_votable)
		vote(chg->fcc_votable, JEITA_VOTER, true, jeita->current_limit);
	else
		jeita_set_charger_current(jeita, jeita->current_limit);

	if (chg->bms_psy) {
		ret = power_supply_get_property(chg->bms_psy,
						POWER_SUPPLY_PROP_FASTCHARGE_MODE,
						&prop);
		if (ret < 0)
			pr_err("get fastcharge_mode fail\n");
		else
			jeita->fast_charge_mode = prop.intval;
	}

	if (!nopmi_is_ffc_disabled() && jeita->fast_charge_mode &&
	    sw->sm != TEMP_T2_TO_T3) {
		prop.intval = 0;
		jeita->fast_charge_mode = 0;
		power_supply_set_property(chg->bms_psy,
					  POWER_SUPPLY_PROP_FASTCHARGE_MODE,
					  &prop);
	} else if (!nopmi_is_ffc_disabled() && !jeita->fast_charge_mode &&
		   sw->sm == TEMP_T2_TO_T3) {
		prop.intval = 1;
		jeita->fast_charge_mode = 1;
		power_supply_set_property(chg->bms_psy,
					  POWER_SUPPLY_PROP_FASTCHARGE_MODE,
					  &prop);
	}

	if (sw->sm != TEMP_T2_TO_T3) {
		switch (sw->sm) {
		case TEMP_ABOVE_T4:
			sw->cv = cfg->cv_above_t4;
			break;
		case TEMP_T3_TO_T4:
			sw->cv = cfg->cv_t3_to_t4;
			break;
		case TEMP_T1P5_TO_T2:
			sw->cv = cfg->cv_t1p5_to_t2;
			break;
		case TEMP_T1_TO_T1P5:
			sw->cv = cfg->cv_t1_to_t1p5;
			break;
		case TEMP_T0_TO_T1:
			sw->cv = cfg->cv_t0_to_t1;
			break;
		case TEMP_TN1_TO_T0:
			sw->cv = cfg->cv_tn1_to_t0;
			break;
		case TEMP_BELOW_T0:
			sw->cv = cfg->cv_below_t0;
			break;
		default:
			sw->cv = cfg->cv_normal;
			break;
		}
	} else {
		if (jeita->fast_charge_mode && !nopmi_is_ffc_disabled()) {
			sw->cv = nopmi_is_maxim_ic() ? 4470 : 4480;
		} else {
			sw->cv = cfg->cv_normal;
		}
	}

	if (chg->cp_psy) {
		ret = power_supply_get_property(chg->cp_psy,
						POWER_SUPPLY_PROP_CHARGING_ENABLED,
						&prop);
		if (ret < 0)
			pr_err("get cp CHARGING_ENABLED fail (%d)\n", ret);
		else
			cp_work_flag = !!prop.intval;
	} else {
		pr_err("cp psy not available\n");
	}

	if (cp_work_flag && !nopmi_is_maxim_ic()) {
		sw->cv = 4608;
		pr_info("cp active: cv overridden to %d\n", sw->cv);
	}

	chg1_cv = chg->fv_votable
		  ? get_effective_result(chg->fv_votable)
		  : jeita_get_charger_voltage(jeita);

	if (sw->cv != chg1_cv) {
		if (chg->fv_votable) {
			vote(chg->fv_votable, JEITA_VOTER, true, sw->cv);
		} else {
			ret = jeita_set_charger_voltage(jeita, sw->cv);
			if (ret < 0)
				pr_err("couldn't set cv to %d, ret=%d\n", sw->cv, ret);
		}
	}

	if (nopmi_is_maxim_ic())
		return;

	pd_active = jeita_get_pd_active(jeita);
	if (sw->sm == TEMP_T2_TO_T3 && pd_active == 2) {
		if (temp >= 35)
			sw->term_curr = (jeita->battery_id == 0 ||
					 jeita->battery_id == 1) ? 833 : 784;
		else
			sw->term_curr = 768;
	} else {
		sw->term_curr = 256;
	}

	term_pre = jeita_get_term_current(jeita);
	if (sw->term_curr != term_pre) {
		ret = jeita_set_term_current(jeita, sw->term_curr);
		if (ret < 0)
			pr_err("couldn't set term_curr to %d, ret=%d\n",
			       sw->term_curr, ret);
	}
}

static void jeita_handle(struct nopmi_chg_jeita_st *jeita)
{
	struct sw_jeita_data *sw = &jeita->sw;

	jeita->battery_temp = jeita_get_batt_temp(jeita);
	jeita_handle_current(jeita);

	if (!sw->charging) {
		sw->can_recharging = true;
		switch (nopmi_get_charger_ic_type()) {
		case NOPMI_CHARGER_IC_MAXIM:
			vote(jeita->parent->chgctrl_votable, JEITA_CHG_VOTER,
			     true, CHG_MODE_CHARGING_OFF);
			break;
		case NOPMI_CHARGER_IC_SYV:
			vote(jeita->parent->chg_dis_votable, JEITA_CHG_VOTER, true, 0);
			break;
		default:
			break;
		}
	} else if (sw->can_recharging) {
		switch (nopmi_get_charger_ic_type()) {
		case NOPMI_CHARGER_IC_MAXIM:
			vote(jeita->parent->chgctrl_votable, JEITA_CHG_VOTER,
			     false, CHG_MODE_CHARGING_OFF);
			break;
		case NOPMI_CHARGER_IC_SYV:
			vote(jeita->parent->chg_dis_votable, JEITA_CHG_VOTER, false, 0);
			break;
		default:
			break;
		}
		sw->can_recharging = false;
	}
}

static void nopmi_chg_jeita_workfunc(struct work_struct *work)
{
	struct nopmi_chg_jeita_st *jeita =
		container_of(work, struct nopmi_chg_jeita_st, work.work);

	pr_info("start\n");

	jeita->usb_present = nopmi_chg_is_usb_present(jeita->parent);
	if (!jeita->usb_present)
		return;

	if (jeita->cfg.enable_sw_jeita) {
		jeita_handle(jeita);
		jeita->sw_jeita_start = true;
	} else {
		jeita->sw_jeita_start = false;
	}
}

void start_nopmi_chg_jeita_workfunc(struct nopmi_chg *chg)
{
	struct nopmi_chg_jeita_st *jeita = chg->jeita_st;

	if (!jeita)
		return;

	schedule_delayed_work(&jeita->work, msecs_to_jiffies(JEITA_WORK_DELAY_MS));
}

void stop_nopmi_chg_jeita_workfunc(struct nopmi_chg *chg)
{
	struct nopmi_chg_jeita_st *jeita = chg->jeita_st;

	if (!jeita)
		return;

	cancel_delayed_work_sync(&jeita->work);

	if (chg->fcc_votable)
		vote(chg->fcc_votable, JEITA_VOTER, false, 0);
	if (chg->fv_votable)
		vote(chg->fv_votable, JEITA_VOTER, true, JEITA_CV_NORMAL);
}

static void jeita_state_init(struct nopmi_chg_jeita_st *jeita)
{
	struct jeita_config *cfg = &jeita->cfg;
	struct sw_jeita_data *sw = &jeita->sw;
	int temp;

	if (!cfg->enable_sw_jeita)
		return;

	temp = jeita_get_batt_temp(jeita);
	jeita->battery_temp = temp;

	if (temp >= cfg->t4)
		sw->sm = TEMP_ABOVE_T4;
	else if (temp > cfg->t3)
		sw->sm = TEMP_T3_TO_T4;
	else if (temp >= cfg->t2)
		sw->sm = TEMP_T2_TO_T3;
	else if (temp >= cfg->t1p5)
		sw->sm = TEMP_T1P5_TO_T2;
	else if (temp >= cfg->t1)
		sw->sm = TEMP_T1_TO_T1P5;
	else if (temp >= cfg->t0)
		sw->sm = TEMP_T0_TO_T1;
	else if (temp >= cfg->tn1)
		sw->sm = TEMP_TN1_TO_T0;
	else
		sw->sm = TEMP_BELOW_T0;

	pr_info("[SW_JEITA] init: temp=%d, sm=%d\n", temp, sw->sm);
}

int nopmi_chg_jeita_init(struct nopmi_chg *chg)
{
	struct nopmi_chg_jeita_st *jeita = chg->jeita_st;

	if (!jeita) {
		pr_err("jeita_st not allocated\n");
		return -EINVAL;
	}

	pr_info("entry\n");

	jeita->parent = chg;
	jeita->current_limit = JEITA_FCC_T2_TO_T3;
	jeita->fast_charge_mode = 0;

	INIT_DELAYED_WORK(&jeita->work, nopmi_chg_jeita_workfunc);
	jeita_state_init(jeita);

	if (!nopmi_is_maxim_ic())
		jeita->battery_id = jeita_get_batt_id(jeita);

	pr_info("done\n");
	return 0;
}

int nopmi_chg_jeita_deinit(struct nopmi_chg *chg)
{
	struct nopmi_chg_jeita_st *jeita = chg->jeita_st;

	if (!jeita)
		return 0;

	pr_info("entry\n");
	cancel_delayed_work_sync(&jeita->work);
	pr_info("done\n");
	return 0;
}
