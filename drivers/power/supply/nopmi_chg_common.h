/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * nopmi_chg_common.h - Base types, IC detection, and stateless helpers
 *
 * This header is the foundation of the nopmi charger driver stack.
 * It deliberately contains NO resource ownership — no PSY handles,
 * no votable pointers, no struct definitions tied to a specific driver
 * instance. It is safe to include from any layer.
 *
 */

#if !defined(__NOPMI_CHG_COMMON_H__)
#define __NOPMI_CHG_COMMON_H__

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pmic-voter.h>

#include "maxim/max77729_charger.h"
#include "nopmi/bq2589x_charger.h"
#include "../../usb/typec/tcpc/inc/tcpm.h"

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

int nopmi_chg_is_usb_present(struct power_supply *usb_psy);

char nopmi_set_charger_ic_type(NOPMI_CHARGER_IC_TYPE nopmi_type);
NOPMI_CHARGER_IC_TYPE nopmi_get_charger_ic_type(void);

int nopmi_set_charge_enable(bool en);
int nopmi_get_quick_charge_type(struct power_supply *usb_psy);

#endif /* __NOPMI_CHG_COMMON_H__ */
