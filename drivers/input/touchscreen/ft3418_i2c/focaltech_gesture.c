// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 */

/*****************************************************************************
 * 1.Included header files
 *****************************************************************************/
#include "focaltech_gesture.h"

/******************************************************************************
 * Private constant and macro definitions using #define
 *****************************************************************************/
#define KEY_GESTURE_LEFT		KEY_LEFT
#define KEY_GESTURE_RIGHT		KEY_RIGHT
#define KEY_GESTURE_UP			KEY_UP
#define KEY_GESTURE_DOWN		KEY_DOWN
#define KEY_GESTURE_DOUBLECLICK		KEY_WAKEUP
#define KEY_GESTURE_SINGLECLICK		KEY_GOTO
#define KEY_GESTURE_O			KEY_O
#define KEY_GESTURE_W			KEY_W
#define KEY_GESTURE_M			KEY_M
#define KEY_GESTURE_E			KEY_E
#define KEY_GESTURE_C			KEY_C
#define KEY_GESTURE_Z			KEY_Z
#define KEY_GESTURE_L			KEY_L
#define KEY_GESTURE_S			KEY_S
#define KEY_GESTURE_V			KEY_V

#define GESTURE_LEFT			0x20
#define GESTURE_RIGHT			0x21
#define GESTURE_UP			0x22
#define GESTURE_DOWN			0x23
#define GESTURE_DOUBLECLICK		0x24
#define GESTURE_SINGLECLICK		0x25
#define GESTURE_O			0x30
#define GESTURE_W			0x31
#define GESTURE_M			0x32
#define GESTURE_E			0x33
#define GESTURE_C			0x34
#define GESTURE_Z			0x41
#define GESTURE_L			0x44
#define GESTURE_S			0x46
#define GESTURE_V			0x54

#define GESTURE_RETRY_COUNT		5
#define GESTURE_RETRY_DELAY_US_MIN	1000
#define GESTURE_RETRY_DELAY_US_MAX	1500

/*****************************************************************************
 * Static function prototypes
 *****************************************************************************/

static int fts_gesture_write_mask(struct fts_ts_data *ts_data)
{
	int ret;

	ret = fts_write_reg(ts_data, 0xD1, 0xFF);
	if (ret < 0)
		return ret;

	ret = fts_write_reg(ts_data, 0xD2, 0xFF);
	if (ret < 0)
		return ret;

	ret = fts_write_reg(ts_data, 0xD5, 0xFF);
	if (ret < 0)
		return ret;

	ret = fts_write_reg(ts_data, 0xD6, 0xFF);
	if (ret < 0)
		return ret;

	ret = fts_write_reg(ts_data, 0xD7, 0xFF);
	if (ret < 0)
		return ret;

	ret = fts_write_reg(ts_data, 0xD8, 0xFF);

	return ret;
}

/*****************************************************************************
 * Sysfs interface
 *****************************************************************************/
static ssize_t fts_gesture_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	u8 val = 0;
	int count;

	if (!ts_data)
		return -ENODEV;

	mutex_lock(&ts_data->state_lock);
	fts_read_reg(ts_data, FTS_REG_GESTURE_EN, &val);
	count = snprintf(buf, PAGE_SIZE, "Gesture Mode:%s\nReg(0xD0)=%d\n",
			 ts_data->gesture_mode ? "On" : "Off", val);
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_gesture_mode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	if (!ts_data)
		return -ENODEV;

	mutex_lock(&ts_data->state_lock);
	if (FTS_SYSFS_ECHO_ON(buf))
		ts_data->gesture_mode = ENABLE;
	else if (FTS_SYSFS_ECHO_OFF(buf))
		ts_data->gesture_mode = DISABLE;
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_gesture_buf_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct fts_gesture_data *gesture;
	int i, count = 0;

	if (!ts_data || !ts_data->gesture)
		return -ENODEV;

	gesture = ts_data->gesture;

	mutex_lock(&ts_data->state_lock);
	count = snprintf(buf, PAGE_SIZE, "Gesture ID:%d\nPointNum:%d\nPoints:\n",
			 gesture->gesture_id, gesture->point_num);

	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "%d(%d,%d) ", i,
				  gesture->coordinate_x[i],
				  gesture->coordinate_y[i]);
		if ((i + 1) % 4 == 0)
			count += snprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	if (count > 0 && buf[count - 1] != '\n')
		count += snprintf(buf + count, PAGE_SIZE - count, "\n");

	mutex_unlock(&ts_data->state_lock);

	return count;
}

static DEVICE_ATTR_RW(fts_gesture_mode);
static DEVICE_ATTR_RO(fts_gesture_buf);

static struct attribute *fts_gesture_mode_attrs[] = {
	&dev_attr_fts_gesture_mode.attr,
	&dev_attr_fts_gesture_buf.attr,
	NULL,
};

static struct attribute_group fts_gesture_group = {
	.attrs = fts_gesture_mode_attrs,
};

/*****************************************************************************
 * Public API Functions
 *****************************************************************************/

static void fts_gesture_report(struct fts_ts_data *ts_data, int gesture_id)
{
	int key;
	struct input_dev *input_dev;

	if (!ts_data || !ts_data->input_dev)
		return;

	input_dev = ts_data->input_dev;

	switch (gesture_id) {
	case GESTURE_LEFT:
		key = KEY_GESTURE_LEFT;
		break;
	case GESTURE_RIGHT:
		key = KEY_GESTURE_RIGHT;
		break;
	case GESTURE_UP:
		key = KEY_GESTURE_UP;
		break;
	case GESTURE_DOWN:
		key = KEY_GESTURE_DOWN;
		break;
	case GESTURE_DOUBLECLICK:
		key = KEY_GESTURE_DOUBLECLICK;
		break;
	case GESTURE_SINGLECLICK:
		key = KEY_GESTURE_SINGLECLICK;
		break;
	case GESTURE_O:
		key = KEY_GESTURE_O;
		break;
	case GESTURE_W:
		key = KEY_GESTURE_W;
		break;
	case GESTURE_M:
		key = KEY_GESTURE_M;
		break;
	case GESTURE_E:
		key = KEY_GESTURE_E;
		break;
	case GESTURE_C:
		key = KEY_GESTURE_C;
		break;
	case GESTURE_Z:
		key = KEY_GESTURE_Z;
		break;
	case GESTURE_L:
		key = KEY_GESTURE_L;
		break;
	case GESTURE_S:
		key = KEY_GESTURE_S;
		break;
	case GESTURE_V:
		key = KEY_GESTURE_V;
		break;
	default:
		return;
	}

	if ((key == KEY_GESTURE_SINGLECLICK && !ts_data->aod_mode) ||
	    (key != KEY_GESTURE_SINGLECLICK && !ts_data->gesture_mode))
		return;

	input_report_key(input_dev, key, 1);
	input_sync(input_dev);
	input_report_key(input_dev, key, 0);
	input_sync(input_dev);
}

int fts_gesture_readdata(struct fts_gesture_data *gesture, u8 *data)
{
	struct fts_ts_data *ts_data;
	int i, idx;

	if (!gesture || !gesture->ts_data || !data)
		return -EINVAL;

	ts_data = gesture->ts_data;

	if (!READ_ONCE(ts_data->suspended) ||
	    (!ts_data->gesture_mode && !ts_data->aod_mode))
		return 1;

	memcpy(gesture->gesture_buf, data, FTS_GESTURE_DATA_LEN);

	gesture->gesture_id = data[2];
	gesture->point_num = data[3];

	for (i = 0; i < FTS_GESTURE_POINTS_MAX; i++) {
		idx = 4 * i + 4;
		gesture->coordinate_x[i] = (((u16)data[idx] & 0x0F) << 8) | data[idx + 1];
		gesture->coordinate_y[i] = (((u16)data[idx + 2] & 0x0F) << 8) | data[idx + 3];
	}

	fts_gesture_report(ts_data, gesture->gesture_id);
	return 0;
}

void fts_gesture_recovery(struct fts_gesture_data *gesture)
{
	struct fts_ts_data *ts_data;
	int ret;

	if (!gesture || !gesture->ts_data)
		return;

	ts_data = gesture->ts_data;

	if (!READ_ONCE(ts_data->suspended) ||
	    (!ts_data->gesture_mode && !ts_data->aod_mode))
		return;

	ret = fts_gesture_write_mask(ts_data);
	if (ret < 0) {
		FTS_ERROR("gesture recovery: write mask fail: %d", ret);
		return;
	}

	ret = fts_write_reg(ts_data, FTS_REG_GESTURE_EN, ENABLE);
	if (ret < 0)
		FTS_ERROR("gesture recovery: write en fail: %d", ret);
}

int fts_gesture_suspend(struct fts_gesture_data *gesture)
{
	struct fts_ts_data *ts_data;
	int i, ret;
	u8 state = 0;

	if (!gesture || !gesture->ts_data)
		return -EINVAL;

	ts_data = gesture->ts_data;

	for (i = 0; i < GESTURE_RETRY_COUNT; i++) {
		ret = fts_gesture_write_mask(ts_data);
		if (ret < 0) {
			FTS_ERROR("write gesture mask fail: %d", ret);
			return ret;
		}

		ret = fts_write_reg(ts_data, FTS_REG_GESTURE_EN, ENABLE);
		if (ret < 0) {
			FTS_ERROR("write gesture en fail: %d", ret);
			return ret;
		}

		usleep_range(GESTURE_RETRY_DELAY_US_MIN, GESTURE_RETRY_DELAY_US_MAX);

		ret = fts_read_reg(ts_data, FTS_REG_GESTURE_EN, &state);
		if (ret < 0) {
			FTS_ERROR("read gesture en fail: %d", ret);
			return ret;
		}

		if (state == ENABLE)
			break;
	}

	if (i >= GESTURE_RETRY_COUNT) {
		FTS_ERROR("IC did not enter gesture mode after %d tries, state:0x%x",
			  GESTURE_RETRY_COUNT, state);
		return -EIO;
	}

	if (device_may_wakeup(ts_data->dev))
		enable_irq_wake(ts_data->irq);

	FTS_INFO("gesture suspend: successfully");
	return 0;
}

int fts_gesture_resume(struct fts_gesture_data *gesture)
{
	struct fts_ts_data *ts_data;
	int i, ret;
	u8 state = 0xFF;

	if (!gesture || !gesture->ts_data)
		return -EINVAL;

	ts_data = gesture->ts_data;

	if (device_may_wakeup(ts_data->dev))
		disable_irq_wake(ts_data->irq);

	for (i = 0; i < GESTURE_RETRY_COUNT; i++) {
		ret = fts_write_reg(ts_data, FTS_REG_GESTURE_EN, DISABLE);
		if (ret < 0) {
			FTS_ERROR("write gesture en fail: %d", ret);
			return ret;
		}

		usleep_range(GESTURE_RETRY_DELAY_US_MIN, GESTURE_RETRY_DELAY_US_MAX);

		ret = fts_read_reg(ts_data, FTS_REG_GESTURE_EN, &state);
		if (ret < 0) {
			FTS_ERROR("read gesture en fail: %d", ret);
			return ret;
		}

		if (state == DISABLE)
			break;
	}

	if (i >= GESTURE_RETRY_COUNT) {
		FTS_ERROR("IC did not exit gesture mode after %d tries, state:0x%x",
			  GESTURE_RETRY_COUNT, state);
		return -EIO;
	}

	FTS_INFO("gesture resume: successfully");
	return 0;
}

static int fts_gesture_switch(struct input_dev *dev, unsigned int type,
			      unsigned int code, int value)
{
	struct fts_ts_data *ts_data = input_get_drvdata(dev);

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	if (type == EV_SYN && code == SYN_CONFIG) {
		switch (value) {
		case 4:
			ts_data->gesture_mode = false;
			ts_data->aod_mode = false;
			FTS_INFO("Gesture wake disabled via event");
			break;
		case 5:
			ts_data->gesture_mode = true;
			ts_data->aod_mode = true;
			FTS_INFO("Gesture wake enabled via event");
			break;
		case 0:
		case 1:
			break;
		default:
			FTS_DEBUG("Unknown SYN_CONFIG: %d", value);
			break;
		}
	}

	return 0;
}

#if IS_ENABLED(CONFIG_TP_COMMON)
static ssize_t fts_double_tap_show(struct tp_feature_entry *entry, char *buf)
{
	struct fts_gesture_data *gesture = container_of(entry,
							struct fts_gesture_data,
							dt_entry);
	struct fts_ts_data *ts_data = gesture->ts_data;
	bool enabled;

	if (!ts_data)
		return -ENODEV;

	mutex_lock(&ts_data->state_lock);
	enabled = ts_data->gesture_mode;
	mutex_unlock(&ts_data->state_lock);

	return sysfs_emit(buf, "%d\n", enabled ? 1 : 0);
}

static ssize_t fts_double_tap_store(struct tp_feature_entry *entry,
				    const char *buf, size_t count)
{
	struct fts_gesture_data *gesture = container_of(entry,
							struct fts_gesture_data,
							dt_entry);
	struct fts_ts_data *ts_data = gesture->ts_data;
	bool enable;
	int ret;

	if (!ts_data)
		return -ENODEV;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&ts_data->state_lock);
	ts_data->gesture_mode = enable;

	if (ts_data->suspended) {
		if (enable)
			fts_gesture_suspend(gesture);
		else
			fts_write_reg(ts_data, FTS_REG_GESTURE_EN, 0x00);
	}
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static const struct tp_feature_ops fts_double_tap_ops = {
	.show  = fts_double_tap_show,
	.store = fts_double_tap_store,
};
#endif

int fts_gesture_init(struct fts_ts_data *ts_data)
{
	struct fts_gesture_data *gesture;
	struct input_dev *input_dev;
	int ret;

	FTS_FUNC_ENTER();

	gesture = kzalloc(sizeof(*gesture), GFP_KERNEL);
	if (!gesture)
		return -ENOMEM;

	gesture->ts_data = ts_data;
	input_dev = ts_data->input_dev;
	ts_data->gesture = gesture;

	input_dev->event = fts_gesture_switch;

	ret = sysfs_create_group(&ts_data->dev->kobj, &fts_gesture_group);
	if (ret) {
		FTS_ERROR("gesture sysfs node create fail: %d", ret);
		kfree(ts_data->gesture);
		return ret;
	}

	ts_data->gesture_mode = FTS_GESTURE_EN;

#if IS_ENABLED(CONFIG_TP_COMMON)
	gesture->dt_entry.feature = TP_FEATURE_DOUBLE_TAP;
	gesture->dt_entry.ops = &fts_double_tap_ops;

	ret = tp_common_register_feature(&gesture->dt_entry);
	if (ret) {
		FTS_ERROR("tp_common register double_tap fail: %d", ret);
		sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);
		kfree(ts_data->gesture);
		return ret;
	}
#endif

	FTS_FUNC_EXIT();
	return 0;
}

int fts_gesture_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	if (ts_data) {
#if IS_ENABLED(CONFIG_TP_COMMON)
		if (ts_data->gesture)
			tp_common_unregister_feature(&ts_data->gesture->dt_entry);
#endif

		if (ts_data->dev)
			sysfs_remove_group(&ts_data->dev->kobj, &fts_gesture_group);

		kfree(ts_data->gesture);
	}
	FTS_FUNC_EXIT();
	return 0;
}
