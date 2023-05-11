// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[nopmi_chg_common]: %s: " fmt, __func__

#include "max77729_charger.h"
#include "bq2589x_charger.h"
#include "nopmi_chg_common.h"
#include "nopmi_chg.h"

static enum nopmi_charger_ic_type nopmi_charger_ic = NOPMI_CHARGER_IC_NONE;
static bool g_ffc_disable = true;

int nopmi_set_charger_ic_type(enum nopmi_charger_ic_type type)
{
	if (type < NOPMI_CHARGER_IC_NONE || type >= NOPMI_CHARGER_IC_MAX) {
		pr_err("invalid IC type: %d\n", type);
		return -EINVAL;
	}

	WRITE_ONCE(nopmi_charger_ic, type);
	pr_info("set type=%d\n", type);
	return 0;
}
EXPORT_SYMBOL_GPL(nopmi_set_charger_ic_type);

enum nopmi_charger_ic_type nopmi_get_charger_ic_type(void)
{
	return READ_ONCE(nopmi_charger_ic);
}
EXPORT_SYMBOL_GPL(nopmi_get_charger_ic_type);

void nopmi_set_ffc_disabled(bool disable)
{
	WRITE_ONCE(g_ffc_disable, disable);
}
EXPORT_SYMBOL_GPL(nopmi_set_ffc_disabled);

bool nopmi_is_ffc_disabled(void)
{
	return READ_ONCE(g_ffc_disable);
}
EXPORT_SYMBOL_GPL(nopmi_is_ffc_disabled);

int nopmi_chg_is_usb_present(struct nopmi_chg *chg)
{
	if (!chg)
		return 0;

	return (READ_ONCE(chg->real_type) > 0) ? 1 : 0;
}

static const struct quick_charge adapter_cap[] = {
	{ POWER_SUPPLY_TYPE_USB, QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_TYPE_USB_CDP, QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_TYPE_USB_ACA, QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_TYPE_USB_FLOAT, QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_TYPE_USB_PD, QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_TYPE_USB_DCP, QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_TYPE_USB_HVDCP, QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_TYPE_USB_HVDCP_3, QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_TYPE_WIRELESS, QUICK_CHARGE_FAST },
	{ 0, 0 },
};

int nopmi_get_quick_charge_type(struct nopmi_chg *chg)
{
	union power_supply_propval prop = {0, };
	enum power_supply_type chg_type;
	int pd_verified = 0;
	int ret;
	int i;

	if (!chg)
		return QUICK_CHARGE_NORMAL;

	if (READ_ONCE(chg->pd_active)) {
		chg_type = POWER_SUPPLY_TYPE_USB_PD;
	} else if (chg->cp_psy) {
		ret = power_supply_get_property(chg->cp_psy,
						POWER_SUPPLY_PROP_CHARGING_ENABLED,
						&prop);
		if (!ret && !!prop.intval)
			return QUICK_CHARGE_TURBE;

		chg_type = (enum power_supply_type)READ_ONCE(chg->real_type);
	} else {
		chg_type = (enum power_supply_type)READ_ONCE(chg->real_type);
	}

	pr_info("chg_type=%d\n", chg_type);

	if (READ_ONCE(chg->batt_temp) < 50 ||
	    READ_ONCE(chg->batt_temp) >= 480) {
		if (!chg->is_single_flash) {
			pr_info("batt temp out-of-range (%d)\n",
				READ_ONCE(chg->batt_temp) / 10);
			chg->is_single_flash = true;
		}
		return QUICK_CHARGE_NORMAL;
	}

	if (chg->is_single_flash) {
		pr_info("batt temp returned to normal (%d)\n",
			READ_ONCE(chg->batt_temp) / 10);
		chg->is_single_flash = false;
	}

	switch (nopmi_get_charger_ic_type()) {
	case NOPMI_CHARGER_IC_MAXIM:
		pd_verified = max77729_usbc_is_pd_verified();
		break;
	case NOPMI_CHARGER_IC_SYV:
		pd_verified = adapter_dev_get_pd_verified();
		break;
	default:
		pd_verified = 1;
		break;
	}

	if (chg_type == POWER_SUPPLY_TYPE_USB_PD && pd_verified)
		return QUICK_CHARGE_TURBE;

	for (i = 0; adapter_cap[i].adap_type != 0; i++) {
		if (chg_type == adapter_cap[i].adap_type)
			return adapter_cap[i].adap_cap;
	}

	return QUICK_CHARGE_NORMAL;
}
