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

/*****************************************************************************
 *
 * File Name: focaltech_point_report_check.c
 *
 * Author: Focaltech Driver Team
 *
 * Created: 2016-11-16
 *
 * Abstract: point report check function
 *
 *****************************************************************************/

#include "focaltech_point_report_check.h"

#if FTS_POINT_REPORT_CHECK_EN
#define POINT_REPORT_CHECK_WAIT_TIME	200 /* unit:ms */

static void fts_prc_func(struct work_struct *work)
{
	struct fts_prc_data *prc = container_of(work, struct fts_prc_data, work.work);
	struct fts_ts_data *ts_data = prc->ts_data;
	struct input_dev *input_dev;
	u32 finger_count = 0;
	u32 max_touches;

	if (!ts_data || !ts_data->input_dev || !ts_data->pdata)
		return;

	input_dev = ts_data->input_dev;
	max_touches = ts_data->pdata->max_touch_number;

	FTS_FUNC_ENTER();
	mutex_lock(&ts_data->report_lock);

	for (finger_count = 0; finger_count < max_touches; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	mutex_unlock(&ts_data->report_lock);
	FTS_FUNC_EXIT();
}

void fts_prc_queue_work(struct fts_prc_data *prc)
{
	struct fts_ts_data *ts_data;

	if (!prc || !prc->ts_data || !prc->ts_data->ts_workqueue)
		return;

	ts_data = prc->ts_data;
	queue_delayed_work(ts_data->ts_workqueue, &prc->work,
			   msecs_to_jiffies(POINT_REPORT_CHECK_WAIT_TIME));
}

int fts_point_report_check_init(struct fts_ts_data *ts_data)
{
	struct fts_prc_data *prc;

	FTS_FUNC_ENTER();

	if (!ts_data || !ts_data->dev || !ts_data->ts_workqueue) {
		FTS_ERROR("invalid ts_data/dev/workqueue");
		return -EINVAL;
	}

	prc = kzalloc(sizeof(*prc), GFP_KERNEL);
	if (!prc) {
		FTS_ERROR("allocate prc memory fail");
		return -ENOMEM;
	}

	prc->ts_data = ts_data;
	ts_data->prc = prc;

	INIT_DELAYED_WORK(&prc->work, fts_prc_func);

	FTS_FUNC_EXIT();
	return 0;
}

int fts_point_report_check_exit(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	if (ts_data && ts_data->prc) {
		cancel_delayed_work_sync(&ts_data->prc->work);
		kfree(ts_data->prc);
	}

	FTS_FUNC_EXIT();
	return 0;
}

#endif /* FTS_POINT_REPORT_CHECK_EN */
