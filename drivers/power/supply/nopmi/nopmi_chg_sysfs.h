/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * nopmi_chg_sysfs - qcom-battery sysfs class sub-module
 */

#ifndef __NOPMI_CHG_SYSFS_H__
#define __NOPMI_CHG_SYSFS_H__

#include <linux/device.h>

struct nopmi_chg;

struct nopmi_chg_sysfs {
	struct nopmi_chg	*parent;
	struct class		battery_class;
};

struct nopmi_chg_sysfs *nopmi_chg_sysfs_init(struct nopmi_chg *chg);
void nopmi_chg_sysfs_deinit(struct nopmi_chg_sysfs *sysfs);

#endif /* __NOPMI_CHG_SYSFS_H__ */
