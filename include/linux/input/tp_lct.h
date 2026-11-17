/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __TP_LCT_H__
#define __TP_LCT_H__

#include <linux/types.h>

struct device;
struct tp_lct_device;

#define TP_LCT_INFO_MAX_LEN		128

enum tp_lct_selftest_cmd {
	TP_LCT_SELFTEST_MMI = 0x00,
	TP_LCT_SELFTEST_I2C = 0x01,
	TP_LCT_SELFTEST_OPEN = 0x02,
	TP_LCT_SELFTEST_SHORT = 0x03,
};

/**
 * struct tp_lct_ops - Operations provided by a touchpanel driver
 * @set_gesture: Enable/disable gesture mode (/proc/tp_gesture)
 * @get_gesture: Query gesture mode state (/proc/tp_gesture)
 * @set_palm: Enable/disable palm rejection (/proc/tp_palm)
 * @get_palm: Query palm rejection state (/proc/tp_palm)
 * @set_work: Enable/disable work mode (/proc/tp_work)
 * @get_work: Query work mode state (/proc/tp_work)
 * @set_grip_angle: Set screen orientation angle (/proc/tp_grip_area)
 * @get_grip_angle: Get screen orientation angle (/proc/tp_grip_area)
 * @get_info: Format touchpanel info into @buf (/proc/tp_info)
 * @get_lockdown_info: Format lockdown info into @buf (/proc/tp_lockdown_info)
 * @selftest: Execute selftest command (/proc/tp_selftest); return 2=pass, 1=fail, 0=unknown
 *
 * Procfs nodes are always created; operations a driver leaves NULL make
 * the corresponding node return -EOPNOTSUPP.
 */
struct tp_lct_ops {
	int (*set_gesture)(struct tp_lct_device *lct, bool enable);
	int (*get_gesture)(struct tp_lct_device *lct);
	int (*set_palm)(struct tp_lct_device *lct, bool enable);
	int (*get_palm)(struct tp_lct_device *lct);
	int (*set_work)(struct tp_lct_device *lct, bool enable);
	int (*get_work)(struct tp_lct_device *lct);
	int (*set_grip_angle)(struct tp_lct_device *lct, int angle);
	int (*get_grip_angle)(struct tp_lct_device *lct);
	int (*get_info)(struct tp_lct_device *lct, char *buf, size_t len);
	int (*get_lockdown_info)(struct tp_lct_device *lct, char *buf,
				 size_t len);
	int (*selftest)(struct tp_lct_device *lct, u8 cmd);
};

struct tp_lct_device *tp_lct_device_register(struct device *parent,
					     const struct tp_lct_ops *ops,
					     void *drv_data);
void tp_lct_device_unregister(struct tp_lct_device *lct);
void *tp_lct_get_drvdata(struct tp_lct_device *lct);

#endif /* __TP_LCT_H__ */
