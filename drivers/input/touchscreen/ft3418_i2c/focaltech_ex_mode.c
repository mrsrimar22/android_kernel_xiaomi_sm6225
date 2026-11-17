// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
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

/*****************************************************************************
 * File Name: focaltech_ex_mode.c
 *
 * Author: Focaltech Driver Team
 *
 * Created: 2016-08-31
 *
 * Abstract: Extra mode (glove, cover, charger, report rate) support
 *
 *****************************************************************************/

/*****************************************************************************
 * 1.Included header files
 *****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
 * 2.Private constant and macro definitions using #define
 *****************************************************************************/

/*****************************************************************************
 * 3.Private enumerations, structures and unions using typedef
 *****************************************************************************/
enum _ex_mode {
	MODE_GLOVE = 0,
	MODE_COVER,
	MODE_CHARGER,
	REPORT_RATE,
};

/*****************************************************************************
 * 4.Static variables
 *****************************************************************************/

/*****************************************************************************
 * 5.Global variable or extern global variabls/functions
 *****************************************************************************/

/*****************************************************************************
 * 6.Static function prototypes
 *****************************************************************************/

/*****************************************************************************
 * fts_ex_mode_switch - Enable/disable extra operating modes
 *
 * @ts_data: touchscreen driver data structure
 * @mode: enum _ex_mode value (glove, cover, charger, or report rate)
 * @value: 1 to enable, 0 to disable; report rate value for REPORT_RATE
 *
 * Return: 0 on success, negative errno on failure
 *****************************************************************************/
static int fts_ex_mode_switch(struct fts_ts_data *ts_data,
			      enum _ex_mode mode, u8 value)
{
	int ret = 0;
	u8 m_val = 0;
	u8 reg_addr = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -EINVAL;
	}

	if (value)
		m_val = 0x01;
	else
		m_val = 0x00;

	switch (mode) {
	case MODE_GLOVE:
		reg_addr = FTS_REG_GLOVE_MODE_EN;
		FTS_DEBUG("Glove mode: %s", value ? "ON" : "OFF");
		break;

	case MODE_COVER:
		reg_addr = FTS_REG_COVER_MODE_EN;
		FTS_DEBUG("Cover mode: %s", value ? "ON" : "OFF");
		break;

	case MODE_CHARGER:
		reg_addr = FTS_REG_CHARGER_MODE_EN;
		FTS_DEBUG("Charger mode: %s", value ? "ON" : "OFF");
		break;

	case REPORT_RATE:
		reg_addr = FTS_REG_REPORT_RATE;
		m_val = value;	/* rate value, not an on/off flag */
		FTS_DEBUG("Report rate: %d", value);
		break;

	default:
		FTS_ERROR("mode(%d) unsupport", mode);
		return -EINVAL;
	}

	ret = fts_write_reg(ts_data, reg_addr, m_val);
	if (ret < 0) {
		FTS_ERROR("write mode register(0x%02x) fail: %d",
			  reg_addr, ret);
		return ret;
	}

	return 0;
}

/*****************************************************************************
 * Glove mode sysfs interface
 *****************************************************************************/
static ssize_t fts_glove_mode_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int count = 0;
	u8 val = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);

	if (fts_read_reg(ts_data, FTS_REG_GLOVE_MODE_EN, &val) < 0)
		FTS_ERROR("read glove mode register failed");

	count = snprintf(buf, PAGE_SIZE, "Glove Mode:%s\n",
			 ts_data->glove_mode ? "On" : "Off");
	count += snprintf(buf + count, PAGE_SIZE - count,
			  "Glove Reg(0xC0):%d\n", val);

	mutex_unlock(&ts_data->state_lock);
	return count;
}

static ssize_t fts_glove_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);

	if (FTS_SYSFS_ECHO_ON(buf)) {
		if (!ts_data->glove_mode) {
			FTS_DEBUG("enter glove mode");
			ret = fts_ex_mode_switch(ts_data, MODE_GLOVE, ENABLE);
			if (!ret)
				ts_data->glove_mode = ENABLE;
		}
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		if (ts_data->glove_mode) {
			FTS_DEBUG("exit glove mode");
			ret = fts_ex_mode_switch(ts_data, MODE_GLOVE, DISABLE);
			if (!ret)
				ts_data->glove_mode = DISABLE;
		}
	}

	FTS_DEBUG("glove mode: %d", ts_data->glove_mode);
	mutex_unlock(&ts_data->state_lock);
	return count;
}

/*****************************************************************************
 * Cover mode sysfs interface
 *****************************************************************************/
static ssize_t fts_cover_mode_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int count = 0;
	u8 val = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);

	if (fts_read_reg(ts_data, FTS_REG_COVER_MODE_EN, &val) < 0)
		FTS_ERROR("read cover mode register failed");

	count = snprintf(buf, PAGE_SIZE, "Cover Mode:%s\n",
			 ts_data->cover_mode ? "On" : "Off");
	count += snprintf(buf + count, PAGE_SIZE - count,
			  "Cover Reg(0xC1):%d\n", val);

	mutex_unlock(&ts_data->state_lock);
	return count;
}

static ssize_t fts_cover_mode_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);

	if (FTS_SYSFS_ECHO_ON(buf)) {
		if (!ts_data->cover_mode) {
			FTS_DEBUG("enter cover mode");
			ret = fts_ex_mode_switch(ts_data, MODE_COVER, ENABLE);
			if (!ret)
				ts_data->cover_mode = ENABLE;
		}
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		if (ts_data->cover_mode) {
			FTS_DEBUG("exit cover mode");
			ret = fts_ex_mode_switch(ts_data, MODE_COVER, DISABLE);
			if (!ret)
				ts_data->cover_mode = DISABLE;
		}
	}

	FTS_DEBUG("cover mode: %d", ts_data->cover_mode);
	mutex_unlock(&ts_data->state_lock);
	return count;
}

/*****************************************************************************
 * Charger mode sysfs interface
 *****************************************************************************/
static ssize_t fts_charger_mode_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int count = 0;
	u8 val = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);

	if (fts_read_reg(ts_data, FTS_REG_CHARGER_MODE_EN, &val) < 0)
		FTS_ERROR("read charger mode register failed");

	count = snprintf(buf, PAGE_SIZE, "Charger Mode:%s\n",
			 ts_data->charger_mode ? "On" : "Off");
	count += snprintf(buf + count, PAGE_SIZE - count,
			  "Charger Reg(0x8B):%d\n", val);

	mutex_unlock(&ts_data->state_lock);
	return count;
}

static ssize_t fts_charger_mode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);

	if (FTS_SYSFS_ECHO_ON(buf)) {
		if (!ts_data->charger_mode) {
			FTS_DEBUG("enter charger mode");
			ret = fts_ex_mode_switch(ts_data, MODE_CHARGER, ENABLE);
			if (!ret)
				ts_data->charger_mode = ENABLE;
		}
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		if (ts_data->charger_mode) {
			FTS_DEBUG("exit charger mode");
			ret = fts_ex_mode_switch(ts_data, MODE_CHARGER, DISABLE);
			if (!ret)
				ts_data->charger_mode = DISABLE;
		}
	}

	FTS_DEBUG("charger mode: %d", ts_data->charger_mode);
	mutex_unlock(&ts_data->state_lock);
	return count;
}

/*****************************************************************************
 * Report rate sysfs interface
 *****************************************************************************/
static ssize_t fts_report_rate_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int count = 0;
	u8 val = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	mutex_lock(&ts_data->state_lock);

	if (fts_read_reg(ts_data, FTS_REG_REPORT_RATE, &val) < 0)
		FTS_ERROR("read report rate register failed");

	count = snprintf(buf, PAGE_SIZE, "Report Rate:%d\n",
			 ts_data->report_rate);
	count += snprintf(buf + count, PAGE_SIZE - count,
			  "Report Rate Reg(0x88):%d\n", val);

	mutex_unlock(&ts_data->state_lock);
	return count;
}

static ssize_t fts_report_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret = 0;
	int rate;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -ENODEV;
	}

	ret = kstrtoint(buf, 16, &rate);
	if (ret)
		return ret;

	if (rate < 0 || rate > 0xFF)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);

	if (rate != ts_data->report_rate) {
		ret = fts_ex_mode_switch(ts_data, REPORT_RATE, (u8)rate);
		if (!ret)
			ts_data->report_rate = rate;
	}

	FTS_DEBUG("report rate: %d", ts_data->report_rate);
	mutex_unlock(&ts_data->state_lock);
	return count;
}

/* read and write extra modes
 * read example: cat fts_glove_mode  --- read glove mode
 * write example: echo 1 > fts_glove_mode  --- write glove mode to 01
 */
static DEVICE_ATTR_RW(fts_glove_mode);
static DEVICE_ATTR_RW(fts_cover_mode);
static DEVICE_ATTR_RW(fts_charger_mode);
static DEVICE_ATTR_RW(fts_report_rate);

static struct attribute *fts_touch_mode_attrs[] = {
	&dev_attr_fts_glove_mode.attr,
	&dev_attr_fts_cover_mode.attr,
	&dev_attr_fts_charger_mode.attr,
	&dev_attr_fts_report_rate.attr,
	NULL,
};

static struct attribute_group fts_touch_mode_group = {
	.attrs = fts_touch_mode_attrs,
};

/*****************************************************************************
 * fts_ex_mode_recovery - Restore extra modes after device reset
 *
 * Called from fts_tp_state_recovery() to re-enable any extra modes that
 * were active before the reset. Primarily used for glove, cover, charger
 * and report rate modes.
 *
 * @ts_data: touchscreen driver data structure
 *
 * Return: 0 on success, negative errno on failure
 *****************************************************************************/
int fts_ex_mode_recovery(struct fts_ts_data *ts_data)
{
	int ret = 0;

	if (!ts_data) {
		FTS_ERROR("ts_data is null");
		return -EINVAL;
	}

	if (ts_data->glove_mode) {
		ret = fts_ex_mode_switch(ts_data, MODE_GLOVE, ENABLE);
		if (ret < 0)
			FTS_ERROR("restore glove mode fail");
	}

	if (ts_data->cover_mode) {
		ret = fts_ex_mode_switch(ts_data, MODE_COVER, ENABLE);
		if (ret < 0)
			FTS_ERROR("restore cover mode fail");
	}

	if (ts_data->usb_psy) {
		union power_supply_propval pval = {0};

		if (!power_supply_get_property(ts_data->usb_psy,
					       POWER_SUPPLY_PROP_PRESENT,
					       &pval))
			ts_data->charger_mode = pval.intval > 0;
	}

	ret = fts_ex_mode_switch(ts_data, MODE_CHARGER,
				 ts_data->charger_mode ? ENABLE : DISABLE);
	if (ret < 0)
		FTS_ERROR("restore charger mode fail");

	if (ts_data->report_rate > 0) {
		ret = fts_ex_mode_switch(ts_data, REPORT_RATE,
					 (u8)ts_data->report_rate);
		if (ret < 0)
			FTS_ERROR("restore report rate fail");
	}

	return ret;
}

static void fts_charger_mode_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data =
		container_of(work, struct fts_ts_data, charger_work);
	union power_supply_propval pval = {0};
	bool present;
	int ret;

	ret = power_supply_get_property(ts_data->usb_psy,
					POWER_SUPPLY_PROP_PRESENT,
					&pval);
	if (ret < 0)
		return;

	present = pval.intval > 0;

	mutex_lock(&ts_data->state_lock);
	if (ts_data->charger_mode != present &&
	    !ts_data->suspended &&
	    !READ_ONCE(ts_data->pm_suspend) &&
	    !READ_ONCE(ts_data->fw_loading)) {
		if (!fts_ex_mode_switch(ts_data, MODE_CHARGER,
					present ? ENABLE : DISABLE))
			ts_data->charger_mode = present;
	}
	mutex_unlock(&ts_data->state_lock);
}

static int fts_psy_notifier_callback(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct fts_ts_data *ts_data =
		container_of(nb, struct fts_ts_data, psy_notif);

	if (event != PSY_EVENT_PROP_CHANGED || data != ts_data->usb_psy)
		return NOTIFY_DONE;

	queue_work(system_power_efficient_wq, &ts_data->charger_work);
	return NOTIFY_OK;
}

/*****************************************************************************
 * fts_ex_mode_init - Initialize extra modes module
 *
 * Sets up the sysfs interface for extra modes. Called during driver probe.
 *
 * @ts_data: touchscreen driver data structure
 *
 * Return: 0 on success, negative errno on failure
 *****************************************************************************/
int fts_ex_mode_init(struct fts_ts_data *ts_data)
{
	int ret = 0;

	if (!ts_data || !ts_data->dev) {
		FTS_ERROR("ts_data/dev is null");
		return -EINVAL;
	}

	ts_data->usb_psy = power_supply_get_by_name("usb");
	if (!ts_data->usb_psy) {
		FTS_INFO("usb power supply not ready, deferring");
		return -EPROBE_DEFER;
	}

	/* Initialize all mode flags to disabled */
	ts_data->glove_mode = DISABLE;
	ts_data->cover_mode = DISABLE;
	ts_data->charger_mode = DISABLE;
	ts_data->report_rate = 0;

	/* Create sysfs group for mode control */
	ret = sysfs_create_group(&ts_data->dev->kobj, &fts_touch_mode_group);
	if (ret < 0) {
		FTS_ERROR("create sysfs(ex_mode) fail: %d", ret);
		power_supply_put(ts_data->usb_psy);
		return ret;
	}

	INIT_WORK(&ts_data->charger_work, fts_charger_mode_work);
	ts_data->psy_notif.notifier_call = fts_psy_notifier_callback;
	ret = power_supply_reg_notifier(&ts_data->psy_notif);
	if (ret < 0) {
		FTS_ERROR("register psy notifier fail: %d", ret);
		sysfs_remove_group(&ts_data->dev->kobj, &fts_touch_mode_group);
		power_supply_put(ts_data->usb_psy);
		return ret;
	}

	queue_work(system_power_efficient_wq, &ts_data->charger_work);

	FTS_DEBUG("extra modes module initialized successfully");
	return 0;
}

/*****************************************************************************
 * fts_ex_mode_exit - Clean up extra modes module
 *
 * Removes the sysfs interface. Called during driver removal.
 *
 * @ts_data: touchscreen driver data structure
 *
 * Return: 0 on success, negative errno on failure
 *****************************************************************************/
int fts_ex_mode_exit(struct fts_ts_data *ts_data)
{
	if (!ts_data || !ts_data->dev) {
		FTS_ERROR("ts_data/dev is null");
		return -EINVAL;
	}

	power_supply_unreg_notifier(&ts_data->psy_notif);
	cancel_work_sync(&ts_data->charger_work);
	sysfs_remove_group(&ts_data->dev->kobj, &fts_touch_mode_group);
	power_supply_put(ts_data->usb_psy);
	return 0;
}
