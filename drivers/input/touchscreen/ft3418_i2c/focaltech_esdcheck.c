// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

/*****************************************************************************
 * Included header files
 *****************************************************************************/
#include "focaltech_esdcheck.h"

#if FTS_ESDCHECK_EN
/*****************************************************************************
 * Static function prototypes
 *****************************************************************************/
#if LCD_ESD_PATCH
static int idc_esdcheck_lcderror(struct fts_esdcheck_data *esd)
{
	int ret = 0;
	u8 val = 0;
	struct fts_ts_data *ts_data;

	if (!esd || !esd->ts_data) {
		FTS_ERROR("esd/ts_data is null");
		return -EINVAL;
	}
	ts_data = esd->ts_data;

	FTS_DEBUG("check LCD ESD");
	if (esd->tp_need_recovery == 1 &&
	    esd->lcd_need_reset == 0) {
		esd->tp_need_recovery = 0;
		/* LCD reset, need recover TP state */
		fts_release_all_finger(ts_data);
		fts_tp_state_recovery(ts_data);
	}

	ret = fts_read_reg(ts_data, FTS_REG_ESD_SATURATE, &val);
	if (ret < 0) {
		FTS_ERROR("read reg0xED fail, ret:%d", ret);
		return -EIO;
	}

	if (val == 0xAA) {
		/*
		 * 1. Set flag esd->lcd_need_reset = 1;
		 * 2. LCD driver need reset(recovery) LCD and set esd->lcd_need_reset to 0
		 * 3. recover TP state
		 */
		FTS_INFO("LCD ESD, need execute LCD reset");
		esd->lcd_need_reset = 1;
		esd->tp_need_recovery = 1;
	}

	return 0;
}
#endif

static int fts_esdcheck_tp_reset(struct fts_esdcheck_data *esd)
{
	struct fts_ts_data *ts_data;

	FTS_FUNC_ENTER();

	if (!esd || !esd->ts_data) {
		FTS_ERROR("esd/ts_data is null");
		return -EINVAL;
	}
	ts_data = esd->ts_data;

	esd->flow_work_hold_cnt = 0;
	esd->hardware_reset_cnt++;

	fts_reset_proc(ts_data, 200);
	fts_release_all_finger(ts_data);
	fts_tp_state_recovery(ts_data);

	FTS_FUNC_EXIT();
	return 0;
}

static bool get_chip_id(struct fts_esdcheck_data *esd)
{
	int ret = 0;
	int i = 0;
	u8 reg_value = 0;
	u8 reg_addr = 0;
	u8 chip_id;
	struct fts_ts_data *ts_data;

	if (!esd || !esd->ts_data) {
		FTS_ERROR("esd/ts_data is null");
		return true; // Conservative: trigger reset on error
	}
	ts_data = esd->ts_data;
	chip_id = ts_data->ic_info.ids.chip_idh;

	for (i = 0; i < 3; i++) {
		reg_addr = FTS_REG_CHIP_ID;
		ret = fts_read(ts_data, &reg_addr, 1, &reg_value, 1);
		if (ret < 0) {
			FTS_ERROR("read chip id fail, ret:%d", ret);
			esd->nack_cnt++;
		} else {
			if (reg_value == chip_id)
				break;

			FTS_DEBUG("read chip_id: %x, retry: %d", reg_value, i);
			esd->dataerror_cnt++;
		}
		usleep_range(10000, 11000);
	}

	/* if can't get correct data in 3 times, then need hardware reset */
	if (i >= 3) {
		FTS_ERROR("read chip id 3 times fail, need execute TP reset");
		return true;
	}

	return false;
}

/*****************************************************************************
 *  Name: get_flow_cnt
 *  Brief: Read flow cnt(0x91)
 *  Input: esd - ESD check context
 *  Return: true  - Reg 0x91(flow cnt) abnormal: hold a value for 5 times
 *          false - Reg 0x91(flow cnt) normal
 *****************************************************************************/
static bool get_flow_cnt(struct fts_esdcheck_data *esd)
{
	int ret = 0;
	u8 reg_value = 0;
	u8 reg_addr = 0;
	struct fts_ts_data *ts_data;

	if (!esd || !esd->ts_data) {
		FTS_ERROR("esd/ts_data is null");
		return true; // Conservative: trigger reset on error
	}
	ts_data = esd->ts_data;

	reg_addr = FTS_REG_FLOW_WORK_CNT;
	ret = fts_read(ts_data, &reg_addr, 1, &reg_value, 1);
	if (ret < 0) {
		FTS_ERROR("read reg0x91 fail, ret:%d", ret);
		esd->nack_cnt++;
	} else {
		if (reg_value == esd->flow_work_cnt_last) {
			FTS_DEBUG("reg0x91, val: %x, last: %x", reg_value,
				  esd->flow_work_cnt_last);
			esd->flow_work_hold_cnt++;
		} else {
			esd->flow_work_hold_cnt = 0;
		}

		esd->flow_work_cnt_last = reg_value;
	}

	/* Flow Work Cnt keep a value for 5 times, need execute TP reset */
	if (esd->flow_work_hold_cnt >= 5) {
		FTS_DEBUG("reg0x91 keep a value for 5 times, need execute TP reset");
		return true;
	}

	return false;
}

static int esdcheck_algorithm(struct fts_esdcheck_data *esd)
{
	int ret = 0;
	u8 reg_value = 0;
	u8 reg_addr = 0;
	bool hardware_reset = false;
	struct fts_ts_data *ts_data;

	if (!esd || !esd->ts_data) {
		FTS_ERROR("esd/ts_data is null");
		return -EINVAL;
	}
	ts_data = esd->ts_data;

	/* 1. esdcheck is interrupt, then return */
	if (esd->intr == 1) {
		esd->intr_cnt++;
		if (esd->intr_cnt > ESDCHECK_INTRCNT_MAX)
			esd->intr = 0;
		else
			return 0;
	}

	/* 2. check power state, if suspend, no need check esd */
	if (esd->suspend == 1) {
		FTS_DEBUG("In suspend, not check esd");
		return 0;
	}

	/* 3. check proc_debug state, if 1-proc busy, no need check esd */
	if (esd->proc_debug == 1) {
		FTS_INFO("In apk/adb command mode, not check esd");
		return 0;
	}

	/* 4. In factory mode, can't check esd */
	reg_addr = FTS_REG_WORKMODE;
	ret = fts_read_reg(ts_data, reg_addr, &reg_value);
	if (ret < 0) {
		esd->nack_cnt++;
	} else if ((reg_value & 0x70) != FTS_REG_WORKMODE_WORK_VALUE) {
		FTS_DEBUG("not in work mode(%x), no check esd", reg_value);
		return 0;
	}

	/* 5. IDC esd check lcd default:close */
#if LCD_ESD_PATCH
	idc_esdcheck_lcderror(esd);
#endif

	/* 6. Get Chip ID */
	hardware_reset = get_chip_id(esd);

	/* 7. get Flow work cnt: 0x91 If no change for 5 times, then ESD and reset */
	if (!hardware_reset)
		hardware_reset = get_flow_cnt(esd);

	/* 8. If need hardware reset, then handle it here */
	if (hardware_reset) {
		FTS_DEBUG("NoACK=%d, Error Data=%d, Hardware Reset=%d",
			  esd->nack_cnt, esd->dataerror_cnt,
			  esd->hardware_reset_cnt);
		fts_esdcheck_tp_reset(esd);
	}

	return 0;
}

static void esdcheck_func(struct work_struct *work)
{
	struct fts_esdcheck_data *esd =
		container_of(work, struct fts_esdcheck_data, work.work);
	struct fts_ts_data *ts_data;

	if (!esd || !esd->ts_data) {
		FTS_ERROR("esd/ts_data is null");
		return;
	}
	ts_data = esd->ts_data;

	if (esd->mode == ENABLE) {
		esdcheck_algorithm(esd);
		queue_delayed_work(ts_data->ts_workqueue, &esd->work,
				   msecs_to_jiffies(ESDCHECK_WAIT_TIME));
	}
}

/*****************************************************************************
 * Public API functions
 *****************************************************************************/
int fts_esdcheck_set_intr(struct fts_esdcheck_data *esd, bool intr)
{
	if (!esd) {
		FTS_ERROR("esd is null");
		return -EINVAL;
	}

	esd->intr = intr;
	esd->intr_cnt = (u8)intr;
	return 0;
}

static int fts_esdcheck_get_status(struct fts_esdcheck_data *esd)
{
	if (!esd) {
		FTS_ERROR("esd is null");
		return -EINVAL;
	}

	return esd->mode;
}

/*****************************************************************************
 *  Name: fts_esdcheck_proc_busy
 *  Brief: When APK or ADB command access TP via driver, then need set proc_debug,
 *         then will not check ESD.
 *  Input: esd        - ESD check context
 *         proc_debug - true: proc busy, false: proc idle
 *  Return: 0 on success
 *****************************************************************************/
int fts_esdcheck_proc_busy(struct fts_esdcheck_data *esd, bool proc_debug)
{
	if (!esd) {
		FTS_ERROR("esd is null");
		return -EINVAL;
	}

	esd->proc_debug = proc_debug;
	return 0;
}

/*****************************************************************************
 *  Name: fts_esdcheck_switch
 *  Brief: FTS esd check function switch.
 *  Input: esd    - ESD check context
 *         enable - 1: Enable esd check, 0: Disable esd check
 *  Return: 0 on success
 *****************************************************************************/
int fts_esdcheck_switch(struct fts_esdcheck_data *esd, bool enable)
{
	struct fts_ts_data *ts_data;

	FTS_FUNC_ENTER();

	if (!esd || !esd->ts_data) {
		FTS_ERROR("esd/ts_data is null");
		return -EINVAL;
	}
	ts_data = esd->ts_data;

	if (esd->mode == ENABLE) {
		if (enable) {
			FTS_DEBUG("ESD check start");
			esd->flow_work_hold_cnt = 0;
			esd->flow_work_cnt_last = 0;
			esd->intr = 0;
			esd->intr_cnt = 0;
			queue_delayed_work(ts_data->ts_workqueue, &esd->work,
					   msecs_to_jiffies(ESDCHECK_WAIT_TIME));
		} else {
			FTS_DEBUG("ESD check stop");
			cancel_delayed_work_sync(&esd->work);
		}
	}

	FTS_FUNC_EXIT();
	return 0;
}

int fts_esdcheck_suspend(struct fts_esdcheck_data *esd)
{
	FTS_FUNC_ENTER();

	if (!esd) {
		FTS_ERROR("esd is null");
		return -EINVAL;
	}

	fts_esdcheck_switch(esd, DISABLE);
	esd->suspend = 1;
	esd->intr = 0;
	esd->intr_cnt = 0;

	FTS_FUNC_EXIT();
	return 0;
}

int fts_esdcheck_resume(struct fts_esdcheck_data *esd)
{
	FTS_FUNC_ENTER();

	if (!esd) {
		FTS_ERROR("esd is null");
		return -EINVAL;
	}

	fts_esdcheck_switch(esd, ENABLE);
	esd->suspend = 0;
	esd->intr = 0;
	esd->intr_cnt = 0;

	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
 * Sysfs interface
 *****************************************************************************/
static ssize_t fts_esd_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct fts_esdcheck_data *esd;

	if (!ts_data || !ts_data->esdcheck) {
		FTS_ERROR("ts_data/esdcheck is null");
		return -EINVAL;
	}

	esd = ts_data->esdcheck;

	mutex_lock(&ts_data->state_lock);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_DEBUG("enable esdcheck");
		esd->mode = ENABLE;
		fts_esdcheck_switch(esd, ENABLE);
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_DEBUG("disable esdcheck");
		fts_esdcheck_switch(esd, DISABLE);
		esd->mode = DISABLE;
	}
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_esd_mode_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int count;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct fts_esdcheck_data *esd;

	if (!ts_data || !ts_data->esdcheck) {
		FTS_ERROR("ts_data/esdcheck is null");
		return -EINVAL;
	}

	esd = ts_data->esdcheck;

	mutex_lock(&ts_data->state_lock);
	count = snprintf(buf, PAGE_SIZE, "Esd check: %s\n",
			 fts_esdcheck_get_status(esd) ? "On" : "Off");
	mutex_unlock(&ts_data->state_lock);

	return count;
}

/* sysfs esd node
 *   read example: cat  fts_esd_mode --- read esd mode
 *   write example:echo 01 > fts_esd_mode --- make esdcheck enable
 */
static DEVICE_ATTR_RW(fts_esd_mode);

static struct attribute *fts_esd_mode_attrs[] = {
	&dev_attr_fts_esd_mode.attr,
	NULL,
};

static struct attribute_group fts_esd_group = {
	.attrs = fts_esd_mode_attrs,
};

static int fts_create_esd_sysfs(struct device *dev)
{
	int ret = 0;

	ret = sysfs_create_group(&dev->kobj, &fts_esd_group);
	if (ret) {
		FTS_ERROR("create esd sysfs fail");
		return ret;
	}

	return 0;
}

/*****************************************************************************
 * Init/Exit - operate on ts_data, allocate/free the submodule struct
 *****************************************************************************/
int fts_esdcheck_init(struct fts_ts_data *ts_data)
{
	struct fts_esdcheck_data *esd;

	FTS_FUNC_ENTER();

	if (!ts_data || !ts_data->dev || !ts_data->ts_workqueue) {
		FTS_ERROR("ts_data/dev/workqueue is null");
		return -EINVAL;
	}

	esd = kzalloc(sizeof(*esd), GFP_KERNEL);
	if (!esd) {
		FTS_ERROR("allocate esd memory fail");
		return -ENOMEM;
	}

	esd->ts_data = ts_data;
	ts_data->esdcheck = esd;

	INIT_DELAYED_WORK(&esd->work, esdcheck_func);

	esd->mode = ENABLE;
	esd->intr = 0;
	esd->intr_cnt = 0;

	fts_esdcheck_switch(esd, ENABLE);
	fts_create_esd_sysfs(ts_data->dev);

	FTS_FUNC_EXIT();
	return 0;
}

int fts_esdcheck_exit(struct fts_ts_data *ts_data)
{
	struct fts_esdcheck_data *esd;

	FTS_FUNC_ENTER();

	if (!ts_data || !ts_data->esdcheck) {
		FTS_ERROR("ts_data/esdcheck is null");
		return -EINVAL;
	}

	esd = ts_data->esdcheck;

	cancel_delayed_work_sync(&esd->work);
	sysfs_remove_group(&ts_data->dev->kobj, &fts_esd_group);

	kfree(ts_data->esdcheck);

	FTS_FUNC_EXIT();
	return 0;
}

#endif /* FTS_ESDCHECK_EN */
