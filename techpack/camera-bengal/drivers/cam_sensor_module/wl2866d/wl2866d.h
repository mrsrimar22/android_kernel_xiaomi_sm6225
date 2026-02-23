/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __WL2866D_H
#define __WL2866D_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/srcu.h>

enum {
	OUT_DVDD1 = 0,
	OUT_DVDD2,
	OUT_AVDD1,
	OUT_AVDD2,
	VOL_ENABLE,
	VOL_DISABLE,
	DISCHARGE_ENABLE,
	DISCHARGE_DISABLE,
};

#define WL2866D_MAX_CONFIG_NUM 16

struct reg_value {
	u8 u8Add;
	u8 u8Val;
};

struct wl2866d_device {
	struct miscdevice	misc_dev;
	struct i2c_client	*i2c_client;
	struct device		*dev;
	struct gpio_desc	*en_gpiod;
	u8			chip_id;
	u8			id_reg;
	u8			id_val;
	u8			id_val1;
	u8			init_num;
	struct reg_value	inits[WL2866D_MAX_CONFIG_NUM];
	u32			offset;
	bool			on;
	struct mutex		lock;
	char			*io_buf;
	size_t			io_buf_size;
	char			misc_name[32];
};

/*
 * struct wl2866d_lock_ctx - opaque borrow token.
 *
 * Allocated on the caller's stack. Valid only between a matched
 * wl2866d_lock() / wl2866d_unlock() pair. Never store or pass across
 * thread boundaries.
 */
struct wl2866d_lock_ctx {
	int srcu_idx;
};

/**
 * wl2866d_lock() - begin a sleepable critical section against device removal.
 * @ctx: caller-allocated context, filled by this function.
 *
 * Enters an SRCU read-side section that prevents wl2866d_remove() from
 * completing until wl2866d_unlock() is called. Does NOT take wdev->lock;
 * that is acquired internally by wl2866d_camera_power_control().
 *
 * Returns: 0 if a live device is available, -ENODEV otherwise.
 *          On -ENODEV, wl2866d_unlock() must NOT be called.
 */
int wl2866d_lock(struct wl2866d_lock_ctx *ctx);

/**
 * wl2866d_unlock() - end the critical section started by wl2866d_lock().
 * @ctx: context previously filled by a successful wl2866d_lock().
 */
void wl2866d_unlock(struct wl2866d_lock_ctx *ctx);

/**
 * wl2866d_camera_power_control() - enable or disable one output rail.
 * @ctx:        valid lock context from wl2866d_lock().
 * @out_iotype: OUT_DVDD1 .. OUT_AVDD2.
 * @is_power_on: 0 = disable; non-zero = enable.
 *               For OUT_DVDD2, 1050000 or 1200000 (uV) override the default.
 *
 * Must be called between wl2866d_lock() and wl2866d_unlock().
 * Returns: 0 on success, negative errno on failure.
 */
int wl2866d_camera_power_control(struct wl2866d_lock_ctx *ctx,
				 unsigned int out_iotype,
				 int is_power_on);

#endif /* __WL2866D_H */
