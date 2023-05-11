// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <drm/drm_panel.h>
#include "focaltech_core.h"

#define FTS_DRIVER_NAME		"fts_ts"
#define FTS_IRQ_NAME		"fts_ts_irq"
#define INTERVAL_READ_REG	200	/* ms */
#define TIMEOUT_READ_REG	1000	/* ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV		2800000
#define FTS_VTG_MAX_UV		3300000
#define FTS_I2C_VTG_MIN_UV	1800000
#define FTS_I2C_VTG_MAX_UV	1800000
#endif

/*
 * fts_data: module-private pointer to the single active device instance.
 *
 * This pointer is intentionally NOT exported.
 * All submodules must receive ts_data through their init/API call parameters.
 */
static struct fts_ts_data *fts_data;

static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

#ifdef CONFIG_TOUCHSCREEN_XIAOMI_TOUCHFEATURE
static int fts_get_mode_value(struct xiaomi_touch_interface *iface,
			      int mode, int value_type);
static int fts_get_mode_all(struct xiaomi_touch_interface *iface,
			    int mode, int *value);
static int fts_reset_mode(struct xiaomi_touch_interface *iface, int mode);
static int fts_set_cur_value(struct xiaomi_touch_interface *iface,
			     int fts_mode, int fts_value);
static void fts_init_touchmode_data(struct xiaomi_touch_interface *iface);
#endif

/*****************************************************************************
 * Core utilities
 *****************************************************************************/
int fts_wait_tp_to_valid(struct fts_ts_data *ts_data, bool can_sleep)
{
	int cnt, ret;
	const int max_tries = TIMEOUT_READ_REG / INTERVAL_READ_REG;
	u8 idh = 0, idl = 0;
	u8 chip_idh, chip_idl;
	int last_err = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data not initialized");
		return -ENODEV;
	}

	chip_idh = ts_data->ic_info.ids.chip_idh;
	chip_idl = ts_data->ic_info.ids.chip_idl;

	for (cnt = 0; cnt < max_tries; cnt++) {
		ret = fts_read_reg(ts_data, FTS_REG_CHIP_ID, &idh);
		if (ret < 0) {
			last_err = ret;
			FTS_DEBUG("read CHIP_ID failed (%d), try %d/%d",
				  ret, cnt + 1, max_tries);
			goto out;
		}

		ret = fts_read_reg(ts_data, FTS_REG_CHIP_ID2, &idl);
		if (ret < 0) {
			last_err = ret;
			FTS_DEBUG("read CHIP_ID2 failed (%d), try %d/%d",
				  ret, cnt + 1, max_tries);
			goto out;
		}

		if (idh == chip_idh && idl == chip_idl) {
			FTS_DEBUG("TP Ready, Device ID:0x%02x%02x", idh, idl);
			return 0;
		}

		FTS_DEBUG("TP Not Ready, ReadData:0x%02x%02x (try %d/%d)",
			  idh, idl, cnt + 1, max_tries);
out:
		if (can_sleep) {
			if (msleep_interruptible(INTERVAL_READ_REG)) {
				FTS_ERROR("wait interrupted");
				return -EINTR;
			}
		} else {
			mdelay(INTERVAL_READ_REG);
		}
	}

	FTS_ERROR("TP not responding after %d ms; last i2c err=%d",
		  max_tries * INTERVAL_READ_REG, last_err);
	return -EIO;
}

void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
	if (!ts_data) {
		FTS_ERROR("ts_data not initialized");
		return;
	}

	fts_wait_tp_to_valid(ts_data, true);
	fts_ex_mode_recovery(ts_data);
	if (ts_data->gesture)
		fts_gesture_recovery(ts_data->gesture);
}

int fts_reset_proc(struct fts_ts_data *ts_data, int hdelayms)
{
	if (!ts_data || !ts_data->pdata) {
		FTS_ERROR("ts_data not initialized");
		return -ENODEV;
	}

	gpiod_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
	if (hdelayms)
		msleep(hdelayms);

	return 0;
}

void fts_irq_disable(struct fts_ts_data *ts_data)
{
	unsigned long irqflags;

	if (!ts_data || !ts_data->irq) {
		FTS_ERROR("ts_data not initialized");
		return;
	}

	spin_lock_irqsave(&ts_data->irq_lock, irqflags);
	if (!ts_data->irq_disabled) {
		disable_irq_nosync(ts_data->irq);
		ts_data->irq_disabled = true;
	}
	spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void fts_irq_enable(struct fts_ts_data *ts_data)
{
	unsigned long irqflags;

	if (!ts_data || !ts_data->irq) {
		FTS_ERROR("ts_data not initialized");
		return;
	}

	spin_lock_irqsave(&ts_data->irq_lock, irqflags);
	if (ts_data->irq_disabled) {
		enable_irq(ts_data->irq);
		ts_data->irq_disabled = false;
	}
	spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void fts_hid2std(struct fts_ts_data *ts_data)
{
	int ret;
	u8 buf[3] = {0xEB, 0xAA, 0x09};

	if (!ts_data || ts_data->bus_type != BUS_TYPE_I2C)
		return;

	ret = fts_write(ts_data, buf, 3);
	if (ret < 0) {
		FTS_ERROR("hid2std cmd write fail");
		return;
	}

	usleep_range(10000, 12000);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	ret = fts_read(ts_data, NULL, 0, buf, 3);
	if (ret < 0)
		FTS_ERROR("hid2std cmd read fail");
	else if (buf[0] == 0xEB && buf[1] == 0xAA && buf[2] == 0x08)
		FTS_DEBUG("hidi2c change to stdi2c successful");
	else
		FTS_DEBUG("hidi2c change to stdi2c not support or fail");
}

void fts_release_all_finger(struct fts_ts_data *ts_data)
{
	struct input_dev *input_dev;
	u32 finger_count, max_touches;

	if (!ts_data || !ts_data->input_dev || !ts_data->pdata) {
		FTS_ERROR("ts_data not initialized");
		return;
	}

	input_dev = ts_data->input_dev;
	max_touches = ts_data->pdata->max_touch_number;

	mutex_lock(&ts_data->report_lock);
	for (finger_count = 0; finger_count < max_touches; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	ts_data->touchs = 0;
	ts_data->key_state = 0;
	mutex_unlock(&ts_data->report_lock);
}

/*****************************************************************************
 * Chip identification
 *****************************************************************************/
static int fts_get_chip_types(struct fts_ts_data *ts_data,
			      u8 id_h, u8 id_l, bool fw_valid)
{
	int i;
	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	if (!ts_data) {
		FTS_ERROR("ts_data not initialized");
		return -ENODEV;
	}

	if (id_h == 0x0 || id_l == 0x0) {
		FTS_ERROR("id_h/id_l is 0");
		return -EINVAL;
	}

	FTS_DEBUG("verify id: 0x%02x%02x", id_h, id_l);
	for (i = 0; i < ctype_entries; i++) {
		if (fw_valid == VALID) {
			if (id_h == ctype[i].chip_idh && id_l == ctype[i].chip_idl)
				break;
		} else {
			if ((id_h == ctype[i].rom_idh && id_l == ctype[i].rom_idl) ||
			    (id_h == ctype[i].pb_idh && id_l == ctype[i].pb_idl) ||
			    (id_h == ctype[i].bl_idh && id_l == ctype[i].bl_idl))
				break;
		}
	}

	if (i >= ctype_entries)
		return -ENODATA;

	ts_data->ic_info.ids = ctype[i];
	return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
	int ret;
	u8 chip_id[2] = {0};
	u8 id_cmd[4] = {0};
	u32 id_cmd_len;

	if (!ts_data) {
		FTS_ERROR("ts_data not initialized");
		return -ENODEV;
	}

	id_cmd[0] = FTS_CMD_START1;
	id_cmd[1] = FTS_CMD_START2;
	ret = fts_write(ts_data, id_cmd, 2);
	if (ret < 0) {
		FTS_ERROR("start cmd write fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	id_cmd[0] = FTS_CMD_READ_ID;
	id_cmd[1] = 0x00;
	id_cmd[2] = 0x00;
	id_cmd[3] = 0x00;
	id_cmd_len = ts_data->ic_info.is_incell ?
		FTS_CMD_READ_ID_LEN_INCELL : FTS_CMD_READ_ID_LEN;

	ret = fts_read(ts_data, id_cmd, id_cmd_len, chip_id, 2);
	if (ret < 0 || chip_id[0] == 0x0 || chip_id[1] == 0x0) {
		FTS_ERROR("read boot id fail, read: 0x%02x%02x, ret=%d",
			  chip_id[0], chip_id[1], ret);
		return -EIO;
	}

	id[0] = chip_id[0];
	id[1] = chip_id[1];
	return 0;
}

static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int i;
	const int max_tries = TIMEOUT_READ_REG / INTERVAL_READ_REG;
	u8 chip_idh = 0, chip_idl = 0;
	u8 boot_id[2] = {0, 0};
	int last_i2c_err = 0;
	int ret;

	if (!ts_data) {
		FTS_ERROR("ts_data not initialized");
		return -ENODEV;
	}

	ts_data->ic_info.is_incell = FTS_CHIP_IDC;
	ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

	for (i = 0; i < max_tries; i++) {
		ret = fts_read_reg(ts_data, FTS_REG_CHIP_ID, &chip_idh);
		if (ret < 0) {
			last_i2c_err = ret;
			FTS_ERROR("read CHIP_ID failed (%d) try %d/%d",
				  ret, i + 1, max_tries);
			goto out;
		}

		ret = fts_read_reg(ts_data, FTS_REG_CHIP_ID2, &chip_idl);
		if (ret < 0) {
			last_i2c_err = ret;
			FTS_ERROR("read CHIP_ID2 failed (%d) try %d/%d",
				  ret, i + 1, max_tries);
			goto out;
		}

		if (chip_idh == 0x00 || chip_idl == 0x00) {
			FTS_DEBUG("chip id read invalid 0x%02x%02x try %d/%d",
				  chip_idh, chip_idl, i + 1, max_tries);
			goto out;
		}

		ret = fts_get_chip_types(ts_data, chip_idh, chip_idl, VALID);
		if (ret == 0) {
			ts_data->ic_info.ids.chip_idh = chip_idh;
			ts_data->ic_info.ids.chip_idl = chip_idl;
			FTS_INFO("get ic information, chip_id: 0x%02x%02x",
				 chip_idh, chip_idl);
			return 0;
		}
out:
		msleep(INTERVAL_READ_REG);
	}

	FTS_INFO("normal polling timed out after %d ms, last_i2c_err=%d",
		 max_tries * INTERVAL_READ_REG, last_i2c_err);
	if (ts_data->ic_info.hid_supported)
		fts_hid2std(ts_data);

	ret = fts_read_bootid(ts_data, boot_id);
	if (ret < 0) {
		FTS_ERROR("read boot id fail (%d)", ret);
		return ret;
	}

	ret = fts_get_chip_types(ts_data, boot_id[0], boot_id[1], INVALID);
	if (ret < 0) {
		FTS_ERROR("can't get ic information from boot id (%d)", ret);
		return ret;
	}

	ts_data->ic_info.ids.chip_idh = boot_id[0];
	ts_data->ic_info.ids.chip_idl = boot_id[1];
	FTS_INFO("get ic information from boot id, chip_id: 0x%02x%02x",
		 boot_id[0], boot_id[1]);
	return 0;
}

/*****************************************************************************
 * Touch reporting
 *****************************************************************************/
static void fts_show_touch_buffer(struct fts_ts_data *ts_data,
				  u8 *data, int datalen)
{
	char tmpbuf[FTS_TOUCH_DATA_LEN * 3 + 1];
	int count = 0;
	int i;

	if (!ts_data || ts_data->log_level < 3)
		return;

	for (i = 0; i < datalen; i++) {
		if (count >= (int)sizeof(tmpbuf) - 3)
			break;
		count += snprintf(tmpbuf + count, sizeof(tmpbuf) - count,
				  "%02X,", data[i]);
	}
	FTS_DEBUG("point buffer: %s", tmpbuf);
}

static int fts_input_report_key(struct fts_ts_data *data, int index)
{
	int i, x, y;
	int *x_dim, *y_dim;

	if (!data || !data->pdata || !data->input_dev)
		return -EINVAL;

	if (!data->pdata->have_key ||
	    data->pdata->key_number > FTS_MAX_KEYS ||
	    index < 0)
		return -EINVAL;

	x = data->events[index].x;
	y = data->events[index].y;
	x_dim = &data->pdata->key_x_coords[0];
	y_dim = &data->pdata->key_y_coords[0];

	for (i = 0; i < data->pdata->key_number; i++) {
		if ((x >= x_dim[i] - FTS_KEY_DIM && x <= x_dim[i] + FTS_KEY_DIM) &&
		    (y >= y_dim[i] - FTS_KEY_DIM && y <= y_dim[i] + FTS_KEY_DIM)) {
			bool down = EVENT_DOWN(data->events[index].flag);
			bool up = EVENT_UP(data->events[index].flag);
			unsigned int mask = 1u << i;

			if (down && !(data->key_state & mask)) {
				input_report_key(data->input_dev,
						 data->pdata->keys[i], 1);
				data->key_state |= mask;
				FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
			} else if (up && (data->key_state & mask)) {
				input_report_key(data->input_dev,
						 data->pdata->keys[i], 0);
				data->key_state &= ~mask;
				FTS_DEBUG("Key%d(%d,%d) UP!", i, x, y);
			}
			return 0;
		}
	}
	return -EINVAL;
}

static int fts_input_report_b(struct fts_ts_data *data)
{
	int i;
	int uppoint = 0;
	unsigned int touchs = 0;
	bool va_reported = false;
	u32 max_touch_num;
	struct ts_event *events;

	if (!data || !data->pdata || !data->input_dev || !data->events)
		return -EINVAL;

	max_touch_num = data->pdata->max_touch_number;
	events = data->events;

	for (i = 0; i < data->touch_point; i++) {
		if (fts_input_report_key(data, i) == 0)
			continue;

		va_reported = true;
		input_mt_slot(data->input_dev, events[i].id);

		if (EVENT_DOWN(events[i].flag)) {
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, true);
#if FTS_REPORT_PRESSURE_EN
			if (events[i].p <= 0)
				events[i].p = 0x3f;
			input_report_abs(data->input_dev,
					 ABS_MT_PRESSURE, events[i].p);
#endif
			if (events[i].area <= 0)
				events[i].area = 0x09;
			input_report_abs(data->input_dev,
					 ABS_MT_TOUCH_MAJOR, events[i].area);
			input_report_abs(data->input_dev,
					 ABS_MT_POSITION_X, events[i].x);
			input_report_abs(data->input_dev,
					 ABS_MT_POSITION_Y, events[i].y);

			touchs |= BIT(events[i].id);
			data->touchs |= BIT(events[i].id);

			if (data->log_level >= 2 ||
			    (data->log_level == 1 &&
			     events[i].flag == FTS_TOUCH_DOWN)) {
				FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
					  events[i].id, events[i].x, events[i].y,
					  events[i].p, events[i].area);
			}
		} else {
			uppoint++;
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(events[i].id);
			if (data->log_level >= 1)
				FTS_DEBUG("[B]P%d UP!", events[i].id);
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		unsigned int diff = data->touchs ^ touchs;

		for (i = 0; i < (int)max_touch_num; i++) {
			if (diff & BIT(i)) {
				if (data->log_level >= 1)
					FTS_DEBUG("[B]P%d UP!", i);
				va_reported = true;
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev,
							   MT_TOOL_FINGER, false);
			}
		}
	}

	data->touchs = touchs;

	if (va_reported) {
		if (EVENT_NO_DOWN(data) || !touchs) {
			if (data->log_level >= 1)
				FTS_DEBUG("[B]Points All Up!");
			input_report_key(data->input_dev, BTN_TOUCH, 0);
		} else {
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}

	input_sync(data->input_dev);
	return 0;
}

static int fts_read_touchdata(struct fts_ts_data *data)
{
	int ret;
	u8 *buf;
	u8 *gbuf;

	if (!data || !data->pdata || !data->events || !data->point_buf)
		return -EINVAL;

	buf = data->point_buf;
	memset(buf, 0xFF, data->pnt_buf_size);
	buf[0] = 0x01;

	ret = fts_read(data, buf, 1, buf + 1, FTS_TOUCH_DATA_LEN - 1);
	if (ret < 0) {
		FTS_ERROR("read touchdata failed, ret:%d", ret);
		return ret;
	}

	if (data->log_level >= 3)
		fts_show_touch_buffer(data, buf, FTS_TOUCH_DATA_LEN);

	if (READ_ONCE(data->suspended) &&
	    (data->gesture_mode || data->aod_mode)) {
		gbuf = buf + FTS_TOUCH_DATA_LEN;
		memset(gbuf, 0, FTS_GESTURE_DATA_LEN);

		ret = fts_read_reg(data, FTS_REG_GESTURE_EN, &gbuf[0]);
		if (ret < 0) {
			FTS_ERROR("read gesture en reg failed: %d", ret);
			goto out_touch;
		}

		if (gbuf[0] != ENABLE) {
			FTS_DEBUG("gesture not enabled in fw");
			goto out_touch;
		}

		gbuf[2] = FTS_REG_GESTURE_OUTPUT_ADDRESS;
		ret = fts_read(data, &gbuf[2], 1, &gbuf[2], FTS_GESTURE_DATA_LEN - 2);
		if (ret < 0) {
			FTS_ERROR("read gesture output data failed: %d", ret);
			goto out_touch;
		}

		if (data->gesture && fts_gesture_readdata(data->gesture, gbuf) == 0) {
			FTS_INFO("gesture handled in irq handler");
			return 1;
		}
	}

out_touch:
	return 0;
}

static int fts_read_parse_touchdata(struct fts_ts_data *data)
{
	int ret;
	int i;
	u8 pointid;
	int base;
	struct ts_event *events;
	int max_touch_num;
	u8 *buf;

	if (!data || !data->pdata || !data->events || !data->point_buf)
		return -EINVAL;

	events = data->events;
	max_touch_num = data->pdata->max_touch_number;
	buf = data->point_buf;

	if (data->pnt_buf_size <= FTS_TOUCH_POINT_NUM)
		return -EINVAL;

	ret = fts_read_touchdata(data);
	if (ret)
		return ret;

	data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	data->touch_point = 0;

	if (data->ic_info.is_incell) {
		if (data->pnt_buf_size > 6 && data->point_num == 0x0F &&
		    buf[2] == 0xFF && buf[3] == 0xFF &&
		    buf[4] == 0xFF && buf[5] == 0xFF &&
		    buf[6] == 0xFF) {
			FTS_DEBUG("touch buff is 0xff, need recovery state");
			fts_release_all_finger(data);
			fts_tp_state_recovery(data);
			return -EIO;
		}
	}

	if (data->point_num > max_touch_num) {
		FTS_DEBUG("invalid point_num(%d)", data->point_num);
		return -EIO;
	}

	for (i = 0; i < max_touch_num; i++) {
		base = FTS_ONE_TCH_LEN * i;
		if ((size_t)(FTS_TOUCH_ID_POS + base) >= data->pnt_buf_size)
			break;

		pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;

		if (pointid >= (u8)max_touch_num) {
			FTS_DEBUG("ID(%d) beyond max_touch_number", pointid);
			return -EINVAL;
		}

		data->touch_point++;
		events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) |
			       (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
		events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) |
			       (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
		events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
		events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
		events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
		events[i].p = buf[FTS_TOUCH_PRE_POS + base];

		if (EVENT_DOWN(events[i].flag) && data->point_num == 0) {
			FTS_DEBUG("abnormal touch data from fw");
			return -EIO;
		}
	}

	return 0;
}

static void fts_irq_read_report(struct fts_ts_data *ts_data)
{
	int ret;

	if (!ts_data)
		return;

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_set_intr(ts_data->esdcheck, true);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_prc_queue_work(ts_data->prc);
#endif

	ret = fts_read_parse_touchdata(ts_data);
	if (ret == 0) {
		mutex_lock(&ts_data->report_lock);
		fts_input_report_b(ts_data);
		mutex_unlock(&ts_data->report_lock);
	}

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_set_intr(ts_data->esdcheck, false);
#endif
}

static irqreturn_t fts_irq_thread_handler(int irq, void *data)
{
	struct fts_ts_data *ts_data = data;
	int ret;

	if (!ts_data)
		return IRQ_HANDLED;

	if (READ_ONCE(ts_data->irq_disabled) || READ_ONCE(ts_data->fw_loading))
		return IRQ_HANDLED;

	if (READ_ONCE(ts_data->suspended)) {
		if (!ts_data->gesture_mode && !ts_data->aod_mode)
			return IRQ_HANDLED;
	}

	if ((ts_data->gesture_mode || ts_data->aod_mode) &&
	    READ_ONCE(ts_data->pm_suspend)) {
		ret = wait_for_completion_timeout(&ts_data->pm_completion,
						  msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM));
		if (!ret) {
			FTS_ERROR("Bus don't resume from pm, timeout");
			return IRQ_HANDLED;
		}
	}

	fts_irq_read_report(ts_data);
	return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
	struct fts_ts_platform_data *pdata;
	int irqn = 0;
	int ret;

	if (!ts_data || !ts_data->dev || !ts_data->pdata) {
		FTS_ERROR("invalid ts_data or pdata");
		return -EINVAL;
	}

	pdata = ts_data->pdata;

	irqn = gpiod_to_irq(pdata->irq_gpio);
	if (irqn <= 0) {
		FTS_ERROR("gpiod_to_irq failed: %d", irqn);
		return irqn;
	}

	FTS_INFO("irq:%d, flag:0x%x",
		 irqn, IRQF_TRIGGER_FALLING | IRQF_ONESHOT);

	ret = devm_request_threaded_irq(ts_data->dev, irqn,
					NULL, fts_irq_thread_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					FTS_IRQ_NAME, ts_data);
	if (ret) {
		FTS_ERROR("devm_request_threaded_irq failed: %d", ret);
		return ret;
	}
	ts_data->irq = irqn;

	return 0;
}

static int fts_input_init(struct fts_ts_data *ts_data)
{
	int ret;
	int key_num;
	struct fts_ts_platform_data *pdata;
	struct input_dev *input_dev;
	static const int gesture_keys[] = {
		KEY_SLEEP, KEY_POWER,
		KEY_LEFT, KEY_RIGHT, KEY_UP, KEY_DOWN,
		KEY_WAKEUP, KEY_GOTO,
		KEY_O, KEY_W, KEY_M, KEY_E, KEY_C, KEY_Z, KEY_L, KEY_S, KEY_V,
	};

	if (!ts_data || !ts_data->pdata || !ts_data->dev) {
		FTS_ERROR("invalid ts_data/pdata/dev");
		return -EINVAL;
	}

	pdata = ts_data->pdata;
	input_dev = input_allocate_device();
	if (!input_dev) {
		FTS_ERROR("Failed to allocate memory for input device");
		return -ENOMEM;
	}

	input_dev->name = FTS_DRIVER_NAME;
	input_dev->id.bustype = (ts_data->bus_type == BUS_TYPE_I2C) ?
				BUS_I2C : BUS_SPI;
	input_dev->dev.parent = ts_data->dev;
	input_set_drvdata(input_dev, ts_data);

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	if (pdata->have_key && pdata->key_number > 0) {
		for (key_num = 0; key_num < pdata->key_number; key_num++)
			input_set_capability(input_dev, EV_KEY,
					     pdata->keys[key_num]);
	}

	/* Gesture keys: must be set before input_register_device; the input
	 * core drops EV_KEY events not present in keybit at event time.
	 */
	for (key_num = 0; key_num < ARRAY_SIZE(gesture_keys); key_num++)
		input_set_capability(input_dev, EV_KEY, gesture_keys[key_num]);

	input_mt_init_slots(input_dev, pdata->max_touch_number,
			    INPUT_MT_DIRECT);

	if (pdata->x_min <= pdata->x_max)
		input_set_abs_params(input_dev, ABS_MT_POSITION_X,
				     pdata->x_min, pdata->x_max, 0, 0);
	else
		FTS_ERROR("invalid x range: min(%d) > max(%d)",
			  pdata->x_min, pdata->x_max);

	if (pdata->y_min <= pdata->y_max)
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
				     pdata->y_min, pdata->y_max, 0, 0);
	else
		FTS_ERROR("invalid y range: min(%d) > max(%d)",
			  pdata->y_min, pdata->y_max);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

	ret = input_register_device(input_dev);
	if (ret) {
		FTS_ERROR("Input device registration failed: %d", ret);
		input_set_drvdata(input_dev, NULL);
		input_free_device(input_dev);
		return ret;
	}

	ts_data->input_dev = input_dev;
	return 0;
}

static void fts_input_exit(struct fts_ts_data *ts_data)
{
	if (!ts_data || !ts_data->input_dev)
		return;

	input_unregister_device(ts_data->input_dev);
}

static int fts_report_buffer_init(struct fts_ts_data *ts_data)
{
	int point_num;
	size_t events_num;

	if (!ts_data || !ts_data->pdata) {
		FTS_ERROR("invalid ts_data/pdata");
		return -EINVAL;
	}

	point_num = ts_data->pdata->max_touch_number;
	if (point_num <= 0 || point_num > 32) {
		FTS_ERROR("invalid max_touch_number: %d", point_num);
		return -EINVAL;
	}

	ts_data->pnt_buf_size = (size_t)FTS_TOUCH_DATA_LEN + FTS_GESTURE_DATA_LEN;
	ts_data->point_buf = kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
	if (!ts_data->point_buf) {
		FTS_ERROR("failed to alloc memory for point buf");
		return -ENOMEM;
	}

	events_num = (size_t)point_num * sizeof(struct ts_event);
	ts_data->events = kzalloc(events_num, GFP_KERNEL);
	if (!ts_data->events) {
		FTS_ERROR("failed to alloc memory for point events");
		kfree(ts_data->point_buf);
		return -ENOMEM;
	}

	return 0;
}

static void fts_report_buffer_exit(struct fts_ts_data *ts_data)
{
	if (!ts_data)
		return;

	kfree(ts_data->events);
	kfree(ts_data->point_buf);
	ts_data->pnt_buf_size = 0;
}

/*****************************************************************************
 * Power
 *****************************************************************************/
#if FTS_POWER_SOURCE_CUST_EN
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(ts->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		FTS_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		ts->pinctrl = NULL;
		return ret;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		ret = PTR_ERR(ts->pins_active);
		goto err_put;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		ret = PTR_ERR(ts->pins_suspend);
		goto err_put;
	}

	ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pins_release)) {
		ret = PTR_ERR(ts->pins_release);
		goto err_put;
	}

	return 0;

err_put:
	devm_pinctrl_put(ts->pinctrl);
	return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0)
			FTS_ERROR("Set normal pin state error: %d", ret);
	}
	return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0)
			FTS_ERROR("Set suspend pin state error: %d", ret);
	}
	return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
	int ret = 0;

	if (!ts->pinctrl)
		return 0;

	if (IS_ERR_OR_NULL(ts->pins_release)) {
		devm_pinctrl_put(ts->pinctrl);
		return -ENODEV;
	}

	ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
	if (ret < 0) {
		FTS_ERROR("Set gesture pin state error: %d", ret);
		devm_pinctrl_put(ts->pinctrl);
	}
	return ret;
}
#endif /* FTS_PINCTRL_EN */

#define FTS_POWER_ON_DELAY_MS		5
#define FTS_RESET_PULSE_MS		1
#define FTS_RESET_RELEASE_DELAY_MS	10

static int fts_power_source_ctrl(struct fts_ts_data *ts_data, bool enable)
{
	struct fts_ts_platform_data *pdata;
	int ret;

	if (!ts_data || !ts_data->pdata) {
		FTS_ERROR("invalid ts_data/pdata");
		return -EINVAL;
	}

	pdata = ts_data->pdata;
	mutex_lock(&ts_data->state_lock);

	if (enable) {
		if (!ts_data->power_disabled)
			goto out_unlock;

#if FTS_PINCTRL_EN
		fts_pinctrl_select_normal(ts_data);
#endif
		if (ts_data->vdd) {
			ret = regulator_enable(ts_data->vdd);
			if (ret) {
				FTS_ERROR("enable vdd failed: %d", ret);
				goto err_power_on;
			}
		}

		if (ts_data->vcc_i2c) {
			ret = regulator_enable(ts_data->vcc_i2c);
			if (ret) {
				FTS_ERROR("enable vcc_i2c failed: %d", ret);
				if (ts_data->vdd)
					regulator_disable(ts_data->vdd);
				goto err_power_on;
			}
		}

		msleep(FTS_POWER_ON_DELAY_MS);
		gpiod_set_value_cansleep(pdata->reset_gpio, 1);
		msleep(FTS_RESET_RELEASE_DELAY_MS);
		ts_data->power_disabled = false;
	} else {
		if (ts_data->power_disabled)
			goto out_unlock;

		gpiod_set_value_cansleep(pdata->reset_gpio, 0);
		msleep(FTS_RESET_PULSE_MS);

		if (ts_data->vcc_i2c)
			regulator_disable(ts_data->vcc_i2c);

		if (ts_data->vdd)
			regulator_disable(ts_data->vdd);

#if FTS_PINCTRL_EN
		fts_pinctrl_select_suspend(ts_data);
#endif
		ts_data->power_disabled = true;
	}

out_unlock:
	mutex_unlock(&ts_data->state_lock);
	return 0;

err_power_on:
	mutex_unlock(&ts_data->state_lock);
	FTS_ERROR("power on failed: %d", ret);
	return ret;
}

static int fts_power_source_init(struct fts_ts_data *ts_data)
{
	int ret;

	if (!ts_data || !ts_data->pdata)
		return -EINVAL;

	ts_data->vdd = devm_regulator_get_optional(ts_data->dev, "vdd");
	if (IS_ERR(ts_data->vdd)) {
		if (PTR_ERR(ts_data->vdd) == -ENODEV) {
			FTS_INFO("no vdd regulator");
			ts_data->vdd = NULL;
		} else {
			return dev_err_probe(ts_data->dev,
					     PTR_ERR(ts_data->vdd),
					     "failed to get vdd\n");
		}
	}

	ts_data->vcc_i2c = devm_regulator_get_optional(ts_data->dev, "vcc_i2c");
	if (IS_ERR(ts_data->vcc_i2c)) {
		if (PTR_ERR(ts_data->vcc_i2c) == -ENODEV) {
			FTS_INFO("no vcc_i2c regulator");
			ts_data->vcc_i2c = NULL;
		} else {
			return dev_err_probe(ts_data->dev,
					     PTR_ERR(ts_data->vcc_i2c),
					     "failed to get vcc_i2c\n");
		}
	} else if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
		ret = regulator_set_voltage(ts_data->vcc_i2c,
					    FTS_I2C_VTG_MIN_UV,
					    FTS_I2C_VTG_MAX_UV);
		if (ret) {
			FTS_ERROR("vcc_i2c set_vtg failed: %d", ret);
			devm_regulator_put(ts_data->vcc_i2c);
			ts_data->vcc_i2c = NULL;
			return ret;
		}
	}

#if FTS_PINCTRL_EN
	fts_pinctrl_init(ts_data);
#endif

	ts_data->power_disabled = true;
	ret = fts_power_source_ctrl(ts_data, true);
	if (ret)
		FTS_ERROR("fail to enable power: %d", ret);

	return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
	if (!ts_data)
		return -EINVAL;

	fts_power_source_ctrl(ts_data, false);

#if FTS_PINCTRL_EN
	fts_pinctrl_select_release(ts_data);
#endif

	return 0;
}

static int fts_power_source_suspend(struct fts_ts_data *ts_data)
{
	int ret = fts_power_source_ctrl(ts_data, false);

	if (ret < 0)
		FTS_ERROR("power off fail, ret=%d", ret);
	return ret;
}

static int fts_power_source_resume(struct fts_ts_data *ts_data)
{
	int ret = fts_power_source_ctrl(ts_data, true);

	if (ret < 0)
		FTS_ERROR("power on fail, ret=%d", ret);
	return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

/*****************************************************************************
 * GPIO / DT
 *****************************************************************************/
static int fts_get_dt_coords(struct device *dev, char *name,
			     struct fts_ts_platform_data *pdata)
{
	u32 coords[FTS_COORDS_ARR_SIZE];
	int ret;

	ret = of_property_read_u32_array(dev->of_node, name,
					 coords, FTS_COORDS_ARR_SIZE);
	if (ret < 0) {
		FTS_ERROR("Unable to read %s, please check dts", name);
		pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
		pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
		pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
		pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
		return ret;
	}

	pdata->x_min = coords[0];
	pdata->y_min = coords[1];
	pdata->x_max = coords[2];
	pdata->y_max = coords[3];

	FTS_INFO("display x(%d %d) y(%d %d)",
		 pdata->x_min, pdata->x_max, pdata->y_min, pdata->y_max);
	return 0;
}

static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int ret;

	fts_get_dt_coords(dev, "focaltech,display-coords", pdata);

	pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
	if (pdata->have_key) {
		ret = of_property_read_u32(np, "focaltech,key-number",
					   &pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key number undefined!");

		if (pdata->key_number > FTS_MAX_KEYS)
			pdata->key_number = FTS_MAX_KEYS;

		of_property_read_u32_array(np, "focaltech,keys",
					   pdata->keys, pdata->key_number);
		of_property_read_u32_array(np, "focaltech,key-x-coords",
					   pdata->key_x_coords, pdata->key_number);
		of_property_read_u32_array(np, "focaltech,key-y-coords",
					   pdata->key_y_coords, pdata->key_number);
	}

	pdata->reset_gpio = devm_gpiod_get(dev, "focaltech,reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pdata->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(pdata->reset_gpio),
				     "failed to get reset gpio\n");

	pdata->irq_gpio = devm_gpiod_get(dev, "focaltech,irq", GPIOD_IN);
	if (IS_ERR(pdata->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(pdata->irq_gpio),
				     "failed to get irq gpio\n");

	ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get max-touch-number, please check dts");
		pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	} else {
		if (temp_val < 2)
			pdata->max_touch_number = 2;
		else if (temp_val > FTS_MAX_POINTS_SUPPORT)
			pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
		else
			pdata->max_touch_number = temp_val;
	}

	FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
		 pdata->max_touch_number,
		 desc_to_gpio(pdata->irq_gpio),
		 desc_to_gpio(pdata->reset_gpio));

	return 0;
}

/*****************************************************************************
 * Workqueue PM handlers
 *****************************************************************************/
static void fts_resume_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data =
		container_of(work, struct fts_ts_data, resume_work);

	fts_ts_resume(ts_data->dev);
}

static void fts_suspend_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data =
		container_of(work, struct fts_ts_data, suspend_work);

	fts_ts_suspend(ts_data->dev);
}

/*****************************************************************************
 * DRM panel notifier
 *****************************************************************************/
static int fts_check_dt(struct fts_ts_data *ts_data)
{
	struct device_node *np = ts_data->dev->of_node;
	struct device_node *node;
	struct drm_panel *panel;
	int i, count;

	if (!np)
		return -EINVAL;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0) {
		FTS_ERROR("find drm_panel count(%d) fail", count);
		return -ENODEV;
	}

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		if (!node)
			continue;

		panel = of_drm_find_panel(node);
		of_node_put(node);

		if (IS_ERR(panel))
			continue;

		FTS_INFO("find drm_panel successfully");
		ts_data->active_panel = panel;
		return 0;
	}

	FTS_ERROR("no find drm_panel");
	return -ENODEV;
}

static int fts_check_default_tp(struct device_node *dt, const char *prop)
{
	const char **active_tp;
	const char *active;
	int count, tmp, score = 0;
	int ret, i;

	count = of_property_count_strings(dt->parent, prop);
	if (count <= 0 || count > 3)
		return -ENODEV;

	active_tp = kcalloc(count, sizeof(char *), GFP_KERNEL);
	if (!active_tp)
		return -ENOMEM;

	ret = of_property_read_string_array(dt->parent, prop, active_tp, count);
	if (ret < 0) {
		ret = -ENODEV;
		goto out;
	}

	for (i = 0; i < count; i++) {
		active = active_tp[i];
		if (active) {
			tmp = of_device_is_compatible(dt, active);
			if (tmp > 0)
				score++;
		}
	}

	ret = (score > 0) ? 0 : -ENODEV;
out:
	kfree(active_tp);
	return ret;
}

static int drm_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fts_ts_data *ts_data =
		container_of(self, struct fts_ts_data, drm_notif);
	struct drm_panel_notifier *evdata = data;
	int *blank = NULL;
	bool is_suspend = false;

	if (!evdata || !evdata->data) {
		FTS_ERROR("evdata or evdata->data is null");
		return NOTIFY_DONE;
	}

	if (event != DRM_PANEL_EVENT_BLANK) {
		FTS_DEBUG("event(%lu) skipped", event);
		return NOTIFY_DONE;
	}

	blank = evdata->data;

	switch (*blank) {
	case DRM_PANEL_BLANK_UNBLANK:
		is_suspend = false;
		break;
	case DRM_PANEL_BLANK_POWERDOWN:
	case DRM_PANEL_BLANK_LP1:
	case DRM_PANEL_BLANK_LP2:
		is_suspend = true;
		break;
	default:
		FTS_DEBUG("DRM BLANK(%d) not relevant, ignored", *blank);
		return NOTIFY_DONE;
	}

	if (is_suspend) {
		cancel_work_sync(&ts_data->resume_work);
		queue_work(ts_data->pm_workqueue, &ts_data->suspend_work);
	} else {
		cancel_work_sync(&ts_data->suspend_work);
		queue_work(ts_data->pm_workqueue, &ts_data->resume_work);
	}

	return NOTIFY_OK;
}

/*****************************************************************************
 * Xiaomi touch game mode
 *****************************************************************************/
#ifdef CONFIG_TOUCHSCREEN_XIAOMI_TOUCHFEATURE
static void fts_init_touchmode_data(struct xiaomi_touch_interface *iface)
{
	FTS_DEBUG("ENTER");

	iface->touch_mode[TOUCH_GAME_MODE][GET_MAX_VALUE] = 1;
	iface->touch_mode[TOUCH_GAME_MODE][GET_MIN_VALUE] = 0;
	iface->touch_mode[TOUCH_GAME_MODE][GET_DEF_VALUE] = 0;
	iface->touch_mode[TOUCH_GAME_MODE][SET_CUR_VALUE] = 0;
	iface->touch_mode[TOUCH_GAME_MODE][GET_CUR_VALUE] = 0;

	iface->touch_mode[TOUCH_ACTIVE_MODE][GET_MAX_VALUE] = 1;
	iface->touch_mode[TOUCH_ACTIVE_MODE][GET_MIN_VALUE] = 0;
	iface->touch_mode[TOUCH_ACTIVE_MODE][GET_DEF_VALUE] = 0;
	iface->touch_mode[TOUCH_ACTIVE_MODE][SET_CUR_VALUE] = 0;
	iface->touch_mode[TOUCH_ACTIVE_MODE][GET_CUR_VALUE] = 0;

	iface->touch_mode[TOUCH_UP_THRESHOLD][GET_MAX_VALUE] = 50;
	iface->touch_mode[TOUCH_UP_THRESHOLD][GET_MIN_VALUE] = 35;
	iface->touch_mode[TOUCH_UP_THRESHOLD][GET_DEF_VALUE] = 0;
	iface->touch_mode[TOUCH_UP_THRESHOLD][SET_CUR_VALUE] = 0;
	iface->touch_mode[TOUCH_UP_THRESHOLD][GET_CUR_VALUE] = 0;

	iface->touch_mode[TOUCH_TOLERANCE][GET_MAX_VALUE] = 255;
	iface->touch_mode[TOUCH_TOLERANCE][GET_MIN_VALUE] = 64;
	iface->touch_mode[TOUCH_TOLERANCE][GET_DEF_VALUE] = 0;
	iface->touch_mode[TOUCH_TOLERANCE][SET_CUR_VALUE] = 0;
	iface->touch_mode[TOUCH_TOLERANCE][GET_CUR_VALUE] = 0;

	iface->touch_mode[TOUCH_PANEL_ORIENTATION][GET_MAX_VALUE] = 3;
	iface->touch_mode[TOUCH_PANEL_ORIENTATION][GET_MIN_VALUE] = 0;
	iface->touch_mode[TOUCH_PANEL_ORIENTATION][GET_DEF_VALUE] = 0;
	iface->touch_mode[TOUCH_PANEL_ORIENTATION][SET_CUR_VALUE] = 0;
	iface->touch_mode[TOUCH_PANEL_ORIENTATION][GET_CUR_VALUE] = 0;

	iface->touch_mode[TOUCH_EDGE_FILTER][GET_MAX_VALUE] = 3;
	iface->touch_mode[TOUCH_EDGE_FILTER][GET_MIN_VALUE] = 0;
	iface->touch_mode[TOUCH_EDGE_FILTER][GET_DEF_VALUE] = 2;
	iface->touch_mode[TOUCH_EDGE_FILTER][SET_CUR_VALUE] = 0;
	iface->touch_mode[TOUCH_EDGE_FILTER][GET_CUR_VALUE] = 0;
}

static int fts_set_cur_value(struct xiaomi_touch_interface *iface,
			     int fts_mode, int fts_value)
{
	struct fts_ts_data *ts_data = (struct fts_ts_data *)iface->priv;
	u8 fts_game_value[2] = {0};
	u8 temp_value = 0, reg_value = 0;
	int ret = 0;

	if (fts_mode >= TOUCH_MODE_NUM || fts_mode < 0) {
		FTS_ERROR("fts_mode is error: %d", fts_mode);
		return -EINVAL;
	}

	fts_value = clamp(fts_value,
			  iface->touch_mode[fts_mode][GET_MIN_VALUE],
			  iface->touch_mode[fts_mode][GET_MAX_VALUE]);

	iface->touch_mode[fts_mode][SET_CUR_VALUE] = fts_value;
	FTS_DEBUG("fts_mode: %d, fts_value: %d", fts_mode, fts_value);

	switch (fts_mode) {
	case TOUCH_GAME_MODE:
	case TOUCH_ACTIVE_MODE:
		break;
	case TOUCH_UP_THRESHOLD:
		temp_value = iface->touch_mode[TOUCH_UP_THRESHOLD][SET_CUR_VALUE];
		if (temp_value > 35 && temp_value <= 40)
			reg_value = 0x28;
		else if (temp_value > 40 && temp_value <= 45)
			reg_value = 0x25;
		else if (temp_value > 45 && temp_value <= 50)
			reg_value = 0x23;
		else
			reg_value = 0x25;
		fts_game_value[0] = 0x81;
		fts_game_value[1] = reg_value;
		break;
	case TOUCH_TOLERANCE:
		temp_value = iface->touch_mode[TOUCH_TOLERANCE][SET_CUR_VALUE];
		if (temp_value > 64 && temp_value <= 128)
			reg_value = 0x70;
		else if (temp_value > 128 && temp_value <= 192)
			reg_value = 0x60;
		else if (temp_value > 192 && temp_value <= 255)
			reg_value = 0x40;
		else
			reg_value = 0x80;
		fts_game_value[0] = 0x85;
		fts_game_value[1] = reg_value;
		break;
	case TOUCH_EDGE_FILTER:
		temp_value = iface->touch_mode[TOUCH_EDGE_FILTER][SET_CUR_VALUE];
		reg_value = (temp_value <= 3) ? (temp_value + 1) : 0x01;
		fts_game_value[0] = 0x8D;
		fts_game_value[1] = reg_value;
		break;
	case TOUCH_PANEL_ORIENTATION:
		temp_value = iface->touch_mode[TOUCH_PANEL_ORIENTATION][SET_CUR_VALUE];
		if (temp_value == 0 || temp_value == 2)
			reg_value = 0x00;
		else if (temp_value == 1)
			reg_value = 0x01;
		else if (temp_value == 3)
			reg_value = 0x02;
		fts_game_value[0] = 0x8C;
		fts_game_value[1] = reg_value;
		break;
	case TOUCH_AOD_ENABLE:
		temp_value = iface->touch_mode[TOUCH_AOD_ENABLE][SET_CUR_VALUE];
		ts_data->aod_mode = temp_value ? ENABLE : DISABLE;
		FTS_DEBUG("aod_mode: %d", ts_data->aod_mode);
		break;
	default:
		break;
	}

	FTS_INFO("mode: %d, value: %d, game_value: 0x%x,0x%x",
		 fts_mode, fts_value, fts_game_value[0], fts_game_value[1]);

	iface->touch_mode[fts_mode][GET_CUR_VALUE] =
		iface->touch_mode[fts_mode][SET_CUR_VALUE];

	if (iface->touch_mode[TOUCH_GAME_MODE][SET_CUR_VALUE]) {
		ret = fts_write_reg(iface->priv, fts_game_value[0], fts_game_value[1]);
		if (ret < 0)
			FTS_ERROR("change game mode fail, ret=%d", ret);
	}

	return 0;
}

static int fts_get_mode_value(struct xiaomi_touch_interface *iface,
			      int mode, int value_type)
{
	if (mode < 0 || mode >= TOUCH_MODE_NUM) {
		FTS_ERROR("don't support mode: %d", mode);
		return -EINVAL;
	}
	return iface->touch_mode[mode][value_type];
}

static int fts_get_mode_all(struct xiaomi_touch_interface *iface,
			    int mode, int *value)
{
	if (mode < 0 || mode >= TOUCH_MODE_NUM) {
		FTS_ERROR("don't support mode: %d", mode);
		return -EINVAL;
	}
	value[0] = iface->touch_mode[mode][GET_CUR_VALUE];
	value[1] = iface->touch_mode[mode][GET_DEF_VALUE];
	value[2] = iface->touch_mode[mode][GET_MIN_VALUE];
	value[3] = iface->touch_mode[mode][GET_MAX_VALUE];
	FTS_INFO("mode: %d, value: (%d,%d,%d,%d)",
		 mode, value[0], value[1], value[2], value[3]);
	return 0;
}

static int fts_reset_mode(struct xiaomi_touch_interface *iface, int mode)
{
	int i;
	int ret = 0;

	FTS_INFO("mode: %d", mode);

	if (mode > 0 && mode < TOUCH_MODE_NUM) {
		iface->touch_mode[mode][SET_CUR_VALUE] =
			iface->touch_mode[mode][GET_DEF_VALUE];
		fts_set_cur_value(iface, mode,
				  iface->touch_mode[mode][SET_CUR_VALUE]);
	} else if (mode == 0) {
		for (i = TOUCH_MODE_NUM - 1; i >= 0; i--) {
			iface->touch_mode[i][SET_CUR_VALUE] =
				iface->touch_mode[i][GET_DEF_VALUE];
			fts_set_cur_value(iface, i,
					  iface->touch_mode[i][SET_CUR_VALUE]);
		}
		ret = fts_write_reg(iface->priv, 0x8D, 0x00);
		if (ret < 0)
			FTS_ERROR("set 8D to reset mode fail, ret=%d", ret);
	} else {
		FTS_ERROR("don't support mode: %d", mode);
	}

	return 0;
}
#endif /* CONFIG_TOUCHSCREEN_XIAOMI_TOUCHFEATURE */

/*****************************************************************************
 * PM ops
 *****************************************************************************/
static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	FTS_INFO("system enters into pm_suspend");
	WRITE_ONCE(ts_data->pm_suspend, true);
	/* suspend flag needs to visible across CPU */
	smp_wmb();
	reinit_completion(&ts_data->pm_completion);
	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	FTS_INFO("system resumes from pm_suspend");
	WRITE_ONCE(ts_data->pm_suspend, false);
	/* suspend flag needs to visible across CPU */
	smp_wmb();
	complete(&ts_data->pm_completion);
	return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};

/*****************************************************************************
 * Suspend / resume
 *****************************************************************************/
static int fts_ts_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);
	if (READ_ONCE(ts_data->suspended)) {
		mutex_unlock(&ts_data->state_lock);
		FTS_INFO("Already in suspend state");
		return 0;
	}
	if (ts_data->fw_loading) {
		mutex_unlock(&ts_data->state_lock);
		FTS_INFO("fw upgrade in process, can't suspend");
		return -EBUSY;
	}
	mutex_unlock(&ts_data->state_lock);

	FTS_DEBUG("Enter tp suspend");

	fts_release_all_finger(ts_data);
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_suspend(ts_data->esdcheck);
#endif

	if (ts_data->gesture_mode || ts_data->aod_mode) {
		fts_irq_disable(ts_data);
		ret = fts_gesture_suspend(ts_data->gesture);
		if (ret < 0) {
			FTS_ERROR("gesture suspend failed: %d", ret);
			fts_irq_enable(ts_data);
			goto err_suspend_mode;
		}

#if FTS_POWER_SOURCE_CUST_EN
		if (ts_data->vcc_i2c) {
			ret = regulator_disable(ts_data->vcc_i2c);
			if (ret < 0)
				FTS_ERROR("vcc_i2c disable failed: %d, continuing", ret);
		}
#endif
		fts_irq_enable(ts_data);
	} else {
		fts_irq_disable(ts_data);

		ret = fts_write_reg(ts_data, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
		if (ret < 0) {
			FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);
			goto err_suspend_mode;
		}

		if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
			ret = fts_power_source_suspend(ts_data);
			if (ret < 0) {
				FTS_ERROR("power enter suspend fail: %d", ret);
				goto err_suspend_power;
			}
#endif
		}
	}

	mutex_lock(&ts_data->state_lock);
	WRITE_ONCE(ts_data->suspended, true);
	mutex_unlock(&ts_data->state_lock);

	return 0;

err_suspend_power:
	fts_write_reg(ts_data, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_ACTIVE);
err_suspend_mode:
	fts_irq_enable(ts_data);
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_resume(ts_data->esdcheck);
#endif
	return ret;
}

static int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);
	if (!READ_ONCE(ts_data->suspended)) {
		mutex_unlock(&ts_data->state_lock);
		FTS_INFO("Already in awake state");
		return 0;
	}
	mutex_unlock(&ts_data->state_lock);

	FTS_DEBUG("Enter tp resume");

	if (ts_data->gesture_mode || ts_data->aod_mode) {
#if FTS_POWER_SOURCE_CUST_EN
		if (ts_data->vcc_i2c) {
			ret = regulator_enable(ts_data->vcc_i2c);
			if (ret < 0) {
				FTS_ERROR("vcc_i2c enable failed: %d", ret);
				return ret;
			}
			usleep_range(2000, 5500);
		}
#endif
		fts_wait_tp_to_valid(ts_data, true);

		ret = fts_gesture_resume(ts_data->gesture);
		if (ret < 0)
			FTS_ERROR("gesture resume failed: %d", ret);
	} else {
		if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
			ret = fts_power_source_resume(ts_data);
			if (ret < 0) {
				FTS_ERROR("power resume failed: %d", ret);
				return ret;
			}
#endif
			fts_reset_proc(ts_data, 200);
		}

		ret = fts_write_reg(ts_data, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_ACTIVE);
		if (ret < 0)
			FTS_ERROR("set TP to active mode fail, ret=%d", ret);
		else
			fts_tp_state_recovery(ts_data);

		fts_irq_enable(ts_data);
	}

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_resume(ts_data->esdcheck);
#endif
	fts_release_all_finger(ts_data);

	mutex_lock(&ts_data->state_lock);
	WRITE_ONCE(ts_data->suspended, false);
	mutex_unlock(&ts_data->state_lock);

	return 0;
}

/*****************************************************************************
 * Probe / remove entry points
 *****************************************************************************/
static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
	int ret;
	int pdata_size = sizeof(struct fts_ts_platform_data);

	FTS_INFO("%s", FTS_DRIVER_VERSION);
	ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
	if (!ts_data->pdata) {
		FTS_ERROR("allocate memory for platform_data fail");
		return -ENOMEM;
	}

	if (ts_data->dev->of_node) {
		ret = fts_parse_dt(ts_data->dev, ts_data->pdata);
		if (ret) {
			FTS_ERROR("device-tree parse fail");
			ret = -ENODEV;
			goto err_pdata;
		}

		if (fts_check_dt(ts_data)) {
			ret = fts_check_default_tp(ts_data->dev->of_node,
						   "qcom,i2c-touch-active");
			ret = ret ? -ENODEV : -EPROBE_DEFER;
			goto err_pdata;
		}
	} else {
		if (ts_data->dev->platform_data) {
			memcpy(ts_data->pdata, ts_data->dev->platform_data,
			       pdata_size);
		} else {
			FTS_ERROR("platform_data is null");
			ret = -ENODEV;
			goto err_pdata;
		}
	}

	spin_lock_init(&ts_data->irq_lock);
	mutex_init(&ts_data->report_lock);
	mutex_init(&ts_data->bus_lock);
	mutex_init(&ts_data->state_lock);

	ret = fts_bus_init(ts_data);
	if (ret) {
		FTS_ERROR("bus initialize fail");
		goto err_mutex_init;
	}

	ret = fts_input_init(ts_data);
	if (ret) {
		FTS_ERROR("input initialize fail");
		goto err_bus_init;
	}

	ret = fts_report_buffer_init(ts_data);
	if (ret) {
		FTS_ERROR("report buffer init fail");
		goto err_input_init;
	}

	ts_data->ts_workqueue = alloc_ordered_workqueue("fts_wq",
							WQ_MEM_RECLAIM);
	if (!ts_data->ts_workqueue) {
		FTS_ERROR("failed to create fts workqueue\n");
		ret = -ENOMEM;
		goto err_report_buffer;
	}

	INIT_WORK(&ts_data->resume_work, fts_resume_work);
	INIT_WORK(&ts_data->suspend_work, fts_suspend_work);
	ts_data->pm_workqueue = alloc_ordered_workqueue("fts_pm_wq",
							WQ_MEM_RECLAIM);
	if (!ts_data->pm_workqueue) {
		FTS_ERROR("failed to create fts pm workqueue\n");
		ret = -ENOMEM;
		goto err_wq_init;
	}

#if FTS_POWER_SOURCE_CUST_EN
	ret = fts_power_source_init(ts_data);
	if (ret) {
		FTS_ERROR("fail to get power(regulator)");
		goto err_pm_wq_init;
	}
#endif

#if (!FTS_CHIP_IDC)
	fts_reset_proc(ts_data, 200);
#endif

	ret = fts_get_ic_information(ts_data);
	if (ret) {
		FTS_ERROR("not focal IC, unregister driver");
		goto err_power_init;
	}

	ret = fts_create_apk_debug_channel(ts_data);
	if (ret) {
		FTS_ERROR("create apk debug node fail");
		goto err_power_init;
	}

	ret = fts_create_sysfs(ts_data);
	if (ret) {
		FTS_ERROR("create sysfs node fail");
		goto err_apk_debug;
	}

#if FTS_POINT_REPORT_CHECK_EN
	ret = fts_point_report_check_init(ts_data);
	if (ret) {
		FTS_ERROR("init point report check fail");
		goto err_create_sysfs;
	}
#endif

	ret = fts_gesture_init(ts_data);
	if (ret) {
		FTS_ERROR("init gesture fail");
		goto err_point_report;
	}

	ret = fts_ex_mode_init(ts_data);
	if (ret) {
		FTS_ERROR("init glove/cover/charger fail");
		goto err_gesture_init;
	}

#if FTS_ESDCHECK_EN
	ret = fts_esdcheck_init(ts_data);
	if (ret) {
		FTS_ERROR("init esd check fail");
		goto err_ex_mode_init;
	}
#endif

	ret = fts_irq_registration(ts_data);
	if (ret) {
		FTS_ERROR("request irq failed");
		goto err_esdcheck_init;
	}

	ret = fts_fwupg_init(ts_data);
	if (ret) {
		FTS_ERROR("init fw upgrade fail");
		goto err_irq_reg;
	}

	ts_data->drm_notif.notifier_call = drm_notifier_callback;
	if (ts_data->active_panel) {
		ret = drm_panel_notifier_register(ts_data->active_panel,
						  &ts_data->drm_notif);
		if (ret < 0) {
			FTS_ERROR("register notifier failed: %d", ret);
			goto err_fwupg_init;
		}
		FTS_INFO("register notifier success");
	}

	device_init_wakeup(ts_data->dev, true);
	init_completion(&ts_data->pm_completion);
	WRITE_ONCE(ts_data->pm_suspend, false);

#ifdef CONFIG_TOUCHSCREEN_XIAOMI_TOUCHFEATURE
	ts_data->xmi_iface.set_mode_value = fts_set_cur_value;
	ts_data->xmi_iface.get_mode_value = fts_get_mode_value;
	ts_data->xmi_iface.reset_mode = fts_reset_mode;
	ts_data->xmi_iface.get_mode_all = fts_get_mode_all;
	ts_data->xmi_iface.priv = ts_data;
	fts_init_touchmode_data(&ts_data->xmi_iface);

	ret = xiaomitouch_register_modedata(&ts_data->xmi_iface);
	if (ret == -ENODEV)
		FTS_INFO("xiaomi_touch not probed yet, game mode unavailable");
	else if (ret)
		FTS_ERROR("register touch mode data failed: %d", ret);
	ret = 0;
#endif

	return 0;

err_fwupg_init:
	fts_fwupg_exit(ts_data);
err_irq_reg:
err_esdcheck_init:
#if FTS_ESDCHECK_EN
	fts_esdcheck_exit(ts_data);
err_ex_mode_init:
#endif
	fts_ex_mode_exit(ts_data);
err_gesture_init:
	fts_gesture_exit(ts_data);
err_point_report:
#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_exit(ts_data);
err_create_sysfs:
#endif
	fts_remove_sysfs(ts_data);
err_apk_debug:
	fts_release_apk_debug_channel(ts_data);
err_power_init:
#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_exit(ts_data);
err_pm_wq_init:
#endif
	if (ts_data->pm_workqueue) {
		cancel_work_sync(&ts_data->resume_work);
		cancel_work_sync(&ts_data->suspend_work);
		flush_workqueue(ts_data->pm_workqueue);
		destroy_workqueue(ts_data->pm_workqueue);
	}
err_wq_init:
	if (ts_data->ts_workqueue) {
		flush_workqueue(ts_data->ts_workqueue);
		destroy_workqueue(ts_data->ts_workqueue);
	}
err_report_buffer:
	fts_report_buffer_exit(ts_data);
err_input_init:
	fts_input_exit(ts_data);
err_bus_init:
	fts_bus_exit(ts_data);
err_mutex_init:
	mutex_destroy(&ts_data->state_lock);
	mutex_destroy(&ts_data->bus_lock);
	mutex_destroy(&ts_data->report_lock);
err_pdata:
	kfree(ts_data->pdata);
	return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
	if (!ts_data)
		return 0;

#ifdef CONFIG_TOUCHSCREEN_XIAOMI_TOUCHFEATURE
	xiaomitouch_unregister_modedata();
#endif

	device_init_wakeup(ts_data->dev, false);

	if (ts_data->active_panel)
		drm_panel_notifier_unregister(ts_data->active_panel,
					      &ts_data->drm_notif);

	fts_fwupg_exit(ts_data);

	if (ts_data->pm_workqueue) {
		cancel_work_sync(&ts_data->resume_work);
		cancel_work_sync(&ts_data->suspend_work);
	}

#if FTS_ESDCHECK_EN
	fts_esdcheck_exit(ts_data);
#endif
	fts_ex_mode_exit(ts_data);
	fts_gesture_exit(ts_data);
#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_exit(ts_data);
#endif
	fts_remove_sysfs(ts_data);
	fts_release_apk_debug_channel(ts_data);

	if (ts_data->pm_workqueue) {
		flush_workqueue(ts_data->pm_workqueue);
		destroy_workqueue(ts_data->pm_workqueue);
	}

	if (ts_data->ts_workqueue) {
		flush_workqueue(ts_data->ts_workqueue);
		destroy_workqueue(ts_data->ts_workqueue);
	}

#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_exit(ts_data);
#endif

	fts_report_buffer_exit(ts_data);
	fts_input_exit(ts_data);
	fts_bus_exit(ts_data);

	mutex_destroy(&ts_data->state_lock);
	mutex_destroy(&ts_data->bus_lock);
	mutex_destroy(&ts_data->report_lock);
	kfree(ts_data->pdata);
	fts_data = NULL;

	return 0;
}

/*****************************************************************************
 * I2C driver registration
 *****************************************************************************/
static int fts_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct fts_ts_data *ts_data;

	FTS_INFO("Touch Screen (I2C BUS) driver probe");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FTS_ERROR("I2C not supported");
		return -ENODEV;
	}

	ts_data = devm_kzalloc(&client->dev, sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data)
		return -ENOMEM;

	fts_data = ts_data;
	ts_data->client = client;
	ts_data->dev = &client->dev;
	ts_data->log_level = 0;
	ts_data->fw_is_running = 0;
	ts_data->bus_type = BUS_TYPE_I2C;
	i2c_set_clientdata(client, ts_data);

	ret = fts_ts_probe_entry(ts_data);
	if (ret) {
		FTS_ERROR("Touch Screen (I2C BUS) driver probe fail");
		fts_data = NULL;
		return ret;
	}

	FTS_INFO("Touch Screen (I2C BUS) driver probe successfully");
	return 0;
}

static int fts_ts_remove(struct i2c_client *client)
{
	return fts_ts_remove_entry(i2c_get_clientdata(client));
}

static const struct of_device_id fts_dt_match[] = {
	{ .compatible = "focaltech,fts", },
	{ },
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static const struct i2c_device_id fts_ts_id[] = {
	{ FTS_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fts_ts_id);

static struct i2c_driver fts_ts_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= FTS_DRIVER_NAME,
		.pm		= &fts_dev_pm_ops,
		.of_match_table	= of_match_ptr(fts_dt_match),
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = fts_ts_id,
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
};

static int __init fts_ts_init(void)
{
	int ret;

#ifdef CHECK_TOUCH_VENDOR
	FTS_INFO("TP info: [Vendor]samsung [IC]ft3418");
#endif

	ret = i2c_add_driver(&fts_ts_driver);
	if (ret)
		FTS_ERROR("Focaltech touch screen driver init failed!");

	return ret;
}

static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

late_initcall(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
