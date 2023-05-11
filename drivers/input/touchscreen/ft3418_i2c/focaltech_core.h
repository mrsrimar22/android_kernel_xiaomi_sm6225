/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/preempt.h>
#include <linux/hardirq.h>
#include "focaltech_common.h"

#ifdef CONFIG_PM
#include <linux/pm_runtime.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_XIAOMI_TOUCHFEATURE
#include <xiaomi_touch.h>
#endif

/*****************************************************************************
 * Constants and macros
 *****************************************************************************/
#define FTS_MAX_POINTS_SUPPORT		10
#define FTS_MAX_KEYS			4
#define FTS_KEY_DIM			10
#define FTS_ONE_TCH_LEN			6
#define FTS_TOUCH_DATA_LEN		(FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN + 3)

#define FTS_GESTURE_POINTS_MAX		6
#define FTS_GESTURE_DATA_LEN		(FTS_GESTURE_POINTS_MAX * 4 + 4)

#define FTS_MAX_ID			0x0A
#define FTS_TOUCH_X_H_POS		3
#define FTS_TOUCH_X_L_POS		4
#define FTS_TOUCH_Y_H_POS		5
#define FTS_TOUCH_Y_L_POS		6
#define FTS_TOUCH_PRE_POS		7
#define FTS_TOUCH_AREA_POS		8
#define FTS_TOUCH_POINT_NUM		2
#define FTS_TOUCH_EVENT_POS		3
#define FTS_TOUCH_ID_POS		5
#define FTS_COORDS_ARR_SIZE		4
#define FTS_X_MIN_DISPLAY_DEFAULT	0
#define FTS_Y_MIN_DISPLAY_DEFAULT	0
#define FTS_X_MAX_DISPLAY_DEFAULT	1080
#define FTS_Y_MAX_DISPLAY_DEFAULT	2400
#define FTS_SET_ANGLE			0x8c

#define FTS_TOUCH_DOWN			0
#define FTS_TOUCH_UP			1
#define FTS_TOUCH_CONTACT		2
#define EVENT_DOWN(flag)	(((flag) == FTS_TOUCH_DOWN) || ((flag) == FTS_TOUCH_CONTACT))
#define EVENT_UP(flag)		((flag) == FTS_TOUCH_UP)
#define EVENT_NO_DOWN(data)	(!(data)->point_num)

#define FTX_MAX_COMPATIBLE_TYPE		4
#define FTX_MAX_COMMMAND_LENGTH		16

#define FTS_REG_MONITOR_MODE		0x8600
#define FTS_REG_THDIFF			0x8500
#define FTS_REG_SENSIVITY		0x8100
#define FTS_REG_EDGE_FILTER_LEVEL	0x8D00
#define FTS_REG_EDGE_FILTER_ORIENTATION	0x8C00

#define FTS_TIMEOUT_COMERR_PM		500

#define CHECK_TOUCH_VENDOR
#ifdef CHECK_TOUCH_VENDOR
extern char mtkfb_lcm_name[256];
#endif

/*****************************************************************************
 * Structs
 *****************************************************************************/
struct ftxxxx_proc {
	struct proc_dir_entry	*proc_entry;
	u8			 opmode;
	u8			 cmd_len;
	u8			 cmd[FTX_MAX_COMMMAND_LENGTH];
};

struct fts_ts_platform_data {
	struct gpio_desc *irq_gpio;
	struct gpio_desc *reset_gpio;
	bool have_key;
	u32 key_number;
	u32 keys[FTS_MAX_KEYS];
	u32 key_y_coords[FTS_MAX_KEYS];
	u32 key_x_coords[FTS_MAX_KEYS];
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 max_touch_number;
};

struct ts_event {
	int x;
	int y;
	int p;
	int flag;
	int id;
	int area;
};

/*
 * Forward declarations for submodule state structs.
 */
struct fts_gesture_data;
struct fts_esdcheck_data;
struct fts_ex_fun_data;
struct fts_prc_data;

struct fts_ts_data {
	struct fts_upgrade		*upg;
	struct i2c_client		*client;
	struct spi_device		*spi;
	struct device			*dev;
	struct input_dev		*input_dev;
	struct fts_ts_platform_data	*pdata;
	struct ts_ic_info		ic_info;
	struct workqueue_struct		*ts_workqueue;
	struct workqueue_struct		*pm_workqueue;
	struct work_struct		fwupg_work;
	struct work_struct		resume_work;
	struct work_struct		suspend_work;
	struct ftxxxx_proc		proc;
	spinlock_t			irq_lock;
	struct mutex			report_lock;
	struct mutex			bus_lock;
	struct mutex			state_lock;
	int				irq;
	int				log_level;
	int				fw_is_running;
	int				dummy_byte;
	struct completion		pm_completion;
	bool				pm_suspend;
	bool				suspended;
	bool				fw_loading;
	bool				irq_disabled;
	bool				power_disabled;
	bool				glove_mode;
	bool				cover_mode;
	bool				charger_mode;
	bool				gesture_mode;
	bool				aod_mode;

	/* multi-touch */
	struct ts_event			*events;
	u8				*bus_tx_buf;
	u8				*bus_rx_buf;
	int				bus_type;
	u8				*point_buf;
	int				pnt_buf_size;
	int				touchs;
	int				key_state;
	int				touch_point;
	int				point_num;
	struct regulator		*vdd;
	struct regulator		*vcc_i2c;
#if FTS_PINCTRL_EN
	struct pinctrl			*pinctrl;
	struct pinctrl_state		*pins_active;
	struct pinctrl_state		*pins_suspend;
	struct pinctrl_state		*pins_release;
#endif

	struct drm_panel		*active_panel;
	struct notifier_block		drm_notif;

#ifdef CONFIG_TOUCHSCREEN_XIAOMI_TOUCHFEATURE
	struct xiaomi_touch_interface xmi_iface;
	bool				gamemode_enabled;
#endif

	/*
	 * Submodule state pointers.
	 * Each submodule allocates its own struct in init() via devm_kzalloc,
	 * sets the ->ts_data back-pointer, then assigns here.
	 * NULL when the submodule is compiled out or init has not run.
	 */
	struct fts_gesture_data		*gesture;
	struct fts_esdcheck_data	*esdcheck;
	struct fts_ex_fun_data	*ex_fun;
	struct fts_prc_data		*prc;
};

enum _FTS_BUS_TYPE {
	BUS_TYPE_NONE,
	BUS_TYPE_I2C,
	BUS_TYPE_SPI,
	BUS_TYPE_SPI_V2,
};

/*****************************************************************************
 * Communication interface
 *****************************************************************************/
int fts_read(struct fts_ts_data *ts_data, u8 *cmd, u32 cmdlen,
	     u8 *data, u32 datalen);
int fts_read_reg(struct fts_ts_data *ts_data, u8 addr, u8 *value);
int fts_write(struct fts_ts_data *ts_data, u8 *writebuf, u32 writelen);
int fts_write_reg(struct fts_ts_data *ts_data, u8 addr, u8 value);
void fts_hid2std(struct fts_ts_data *ts_data);
int fts_bus_init(struct fts_ts_data *ts_data);
int fts_bus_exit(struct fts_ts_data *ts_data);

/*****************************************************************************
 * Core utilities
 *****************************************************************************/
int fts_wait_tp_to_valid(struct fts_ts_data *ts_data, bool can_sleep);
int fts_reset_proc(struct fts_ts_data *ts_data, int hdelayms);
void fts_release_all_finger(struct fts_ts_data *ts_data);
void fts_tp_state_recovery(struct fts_ts_data *ts_data);
void fts_irq_disable(struct fts_ts_data *ts_data);
void fts_irq_enable(struct fts_ts_data *ts_data);

/*****************************************************************************
 * Gesture
 *****************************************************************************/
int fts_gesture_init(struct fts_ts_data *ts_data);
int fts_gesture_exit(struct fts_ts_data *ts_data);
void fts_gesture_recovery(struct fts_gesture_data *gesture);
int fts_gesture_readdata(struct fts_gesture_data *gesture, u8 *data);
int fts_gesture_suspend(struct fts_gesture_data *gesture);
int fts_gesture_resume(struct fts_gesture_data *gesture);

/*****************************************************************************
 * Apk / sysfs debug channels
 *****************************************************************************/
int fts_create_apk_debug_channel(struct fts_ts_data *ts_data);
void fts_release_apk_debug_channel(struct fts_ts_data *ts_data);
int fts_create_sysfs(struct fts_ts_data *ts_data);
int fts_remove_sysfs(struct fts_ts_data *ts_data);

/*****************************************************************************
 * ESD check
 *****************************************************************************/
#if FTS_ESDCHECK_EN
int fts_esdcheck_init(struct fts_ts_data *ts_data);
int fts_esdcheck_exit(struct fts_ts_data *ts_data);
int fts_esdcheck_switch(struct fts_esdcheck_data *esd, bool enable);
int fts_esdcheck_proc_busy(struct fts_esdcheck_data *esd, bool proc_debug);
int fts_esdcheck_set_intr(struct fts_esdcheck_data *esd, bool intr);
int fts_esdcheck_suspend(struct fts_esdcheck_data *esd);
int fts_esdcheck_resume(struct fts_esdcheck_data *esd);
#endif

/*****************************************************************************
 * Point report check
 *****************************************************************************/
#if FTS_POINT_REPORT_CHECK_EN
int fts_point_report_check_init(struct fts_ts_data *ts_data);
int fts_point_report_check_exit(struct fts_ts_data *ts_data);
void fts_prc_queue_work(struct fts_prc_data *prc);
#endif

/*****************************************************************************
 * FW upgrade
 *****************************************************************************/
int fts_fwupg_init(struct fts_ts_data *ts_data);
int fts_fwupg_exit(struct fts_ts_data *ts_data);
int fts_upgrade_bin(struct fts_ts_data *ts_data, char *fw_name, bool force);
int fts_enter_test_environment(struct fts_ts_data *ts_data, bool test_state);
int fts_flash_read(struct fts_ts_data *ts_data, u32 addr, u8 *buf, u32 len);

/*****************************************************************************
 * Extra modes (glove / cover / charger)
 *****************************************************************************/
int fts_ex_mode_init(struct fts_ts_data *ts_data);
int fts_ex_mode_exit(struct fts_ts_data *ts_data);
int fts_ex_mode_recovery(struct fts_ts_data *ts_data);

#endif /* __LINUX_FOCALTECH_CORE_H__ */
