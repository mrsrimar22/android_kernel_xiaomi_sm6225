/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __XIAOMI__TOUCH_H__
#define __XIAOMI__TOUCH_H__

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/srcu.h>
#include <linux/list.h>
#include <linux/rculist.h>

/* CUR, DEFAULT, MIN, MAX */
#define VALUE_TYPE_SIZE		6
#define VALUE_GRIP_SIZE		9

enum touch_mode_cmd {
	SET_CUR_VALUE = 0,
	GET_CUR_VALUE,
	GET_DEF_VALUE,
	GET_MIN_VALUE,
	GET_MAX_VALUE,
	GET_MODE_VALUE,
	RESET_MODE,
};

enum touch_mode_type {
	TOUCH_GAME_MODE		= 0,
	TOUCH_ACTIVE_MODE	= 1,
	TOUCH_UP_THRESHOLD	= 2,
	TOUCH_TOLERANCE		= 3,
	TOUCH_WGH_MIN		= 4,
	TOUCH_WGH_MAX		= 5,
	TOUCH_WGH_STEP		= 6,
	TOUCH_EDGE_FILTER	= 7,
	TOUCH_PANEL_ORIENTATION	= 8,
	TOUCH_REPORT_RATE	= 9,
	TOUCH_FOD_ENABLE	= 10,
	TOUCH_AOD_ENABLE	= 11,
	TOUCH_RESIST_RF		= 12,
	TOUCH_IDLE_TIME		= 13,
	TOUCH_DOUBLETAP_MODE	= 14,
	TOUCH_MODE_NUM		= 15,
};

struct xiaomi_touch_interface {
	int touch_mode[TOUCH_MODE_NUM][VALUE_TYPE_SIZE];
	int (*set_mode_value)(struct xiaomi_touch_interface *iface, int mode, int value);
	int (*get_mode_value)(struct xiaomi_touch_interface *iface, int mode, int value_type);
	int (*get_mode_all)(struct xiaomi_touch_interface *iface, int mode, int *mode_value);
	int (*reset_mode)(struct xiaomi_touch_interface *iface, int mode);
	int (*palm_sensor_read)(void);
	int (*palm_sensor_write)(int on);
	void *priv;
};

struct xiaomi_touch {
	struct miscdevice	misc_dev;
	struct device		*dev;
	struct class		*class;
	struct attribute_group	attrs;
	struct mutex		mutex;
	struct mutex		palm_mutex;
	struct mutex		psensor_mutex;
	wait_queue_head_t	wait_queue;
};

struct xiaomi_touch_pdata {
	struct xiaomi_touch		touch_dev;
	struct xiaomi_touch_interface	*touch_data;
	int				palm_value;
	bool				palm_changed;
	const char			*name;
};

int xiaomitouch_register_modedata(struct xiaomi_touch_interface *data);
void xiaomitouch_unregister_modedata(void);
int update_palm_sensor_value(int value);

#endif /* __XIAOMI__TOUCH_H__ */
