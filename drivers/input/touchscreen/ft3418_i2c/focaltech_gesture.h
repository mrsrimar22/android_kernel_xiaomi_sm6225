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

#ifndef __LINUX_FOCALTECH_GESTURE_H__
#define __LINUX_FOCALTECH_GESTURE_H__

#include "focaltech_core.h"
#if IS_ENABLED(CONFIG_TP_COMMON)
#include <linux/input/tp_common.h>
#endif

/**
 * struct fts_gesture_data - per-device gesture state
 * @ts_data:      back-pointer to parent; always valid after init
 * @gesture_id:   last decoded gesture ID
 * @point_num:    number of key-point coordinates reported
 * @coordinate_x: X coordinates of gesture key points
 * @coordinate_y: Y coordinates of gesture key points
 * @dt_entry:     tp_common feature entry for double tap sysfs node
 *
 * Allocated by fts_gesture_init() via devm_kzalloc; stored in
 * ts_data->gesture. All operational functions receive this struct
 * directly and reach the parent through ->ts_data.
 */
struct fts_gesture_data {
	struct fts_ts_data	*ts_data;
	u8			 gesture_id;
	u8			 point_num;
	u8			 gesture_buf[FTS_GESTURE_DATA_LEN];
	u16			 coordinate_x[FTS_GESTURE_POINTS_MAX];
	u16			 coordinate_y[FTS_GESTURE_POINTS_MAX];
#if IS_ENABLED(CONFIG_TP_COMMON)
	struct tp_feature_entry	 dt_entry;
#endif
};


#endif /* __LINUX_FOCALTECH_GESTURE_H__ */
