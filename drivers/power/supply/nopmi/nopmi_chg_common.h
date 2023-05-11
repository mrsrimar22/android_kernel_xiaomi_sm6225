/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * nopmi_chg_common - Base types, IC detection, and stateless helpers
 */

#ifndef __NOPMI_CHG_COMMON_H__
#define __NOPMI_CHG_COMMON_H__

#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pmic-voter.h>

struct nopmi_chg;

enum nopmi_charger_ic_type {
	NOPMI_CHARGER_IC_NONE = 0,
	NOPMI_CHARGER_IC_MAXIM,
	NOPMI_CHARGER_IC_SYV,
	NOPMI_CHARGER_IC_SC,
	NOPMI_CHARGER_IC_MAX,
};

enum quick_charge_type {
	QUICK_CHARGE_NORMAL = 0,
	QUICK_CHARGE_FAST,
	QUICK_CHARGE_FLASH,
	QUICK_CHARGE_TURBE,
	QUICK_CHARGE_MAX,
};

struct quick_charge {
	enum power_supply_type adap_type;
	enum quick_charge_type adap_cap;
};

extern int max77729_usbc_is_pd_verified(void);
extern int adapter_dev_get_pd_verified(void);

int nopmi_set_charger_ic_type(enum nopmi_charger_ic_type type);
enum nopmi_charger_ic_type nopmi_get_charger_ic_type(void);
void nopmi_set_ffc_disabled(bool disable);
bool nopmi_is_ffc_disabled(void);

static inline bool nopmi_is_maxim_ic(void)
{
	return nopmi_get_charger_ic_type() == NOPMI_CHARGER_IC_MAXIM;
}
static inline bool nopmi_is_syv_ic(void)
{
	return nopmi_get_charger_ic_type() == NOPMI_CHARGER_IC_SYV;
}
static inline bool nopmi_is_sc_ic(void)
{
	return nopmi_get_charger_ic_type() == NOPMI_CHARGER_IC_SC;
}
static inline bool nopmi_ic_is_known(void)
{
	return nopmi_is_maxim_ic() || nopmi_is_syv_ic() || nopmi_is_sc_ic();
}
int nopmi_chg_is_usb_present(struct nopmi_chg *chg);
int nopmi_get_quick_charge_type(struct nopmi_chg *chg);

#endif /* __NOPMI_CHG_COMMON_H__ */
