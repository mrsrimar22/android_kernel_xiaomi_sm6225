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

#ifndef __LINUX_FOCALTECH_ESDCHECK_H__
#define __LINUX_FOCALTECH_ESDCHECK_H__

#include "focaltech_core.h"

/*****************************************************************************
 * Private constant and macro definitions using #define
 *****************************************************************************/
#define ESDCHECK_WAIT_TIME	1000 /* ms */
#define LCD_ESD_PATCH		0
#define ESDCHECK_INTRCNT_MAX	2

/**
 * struct fts_esdcheck_data - per-device ESD check state
 * @ts_data:               back-pointer to parent; always valid after init
 * @work:                  delayed work; rescheduled by esdcheck_func itself
 * @mode:                  1 = ESD check active, 0 = disabled
 * @suspend:               1 = device suspended, skip check
 * @proc_debug:            1 = apk/adb path active, skip check
 * @intr:                  1 = interrupt in flight, rate-limit reset count
 * @unused:                bitfield padding
 * @intr_cnt:              interrupt count within current rate-limit window
 * @flow_work_hold_cnt:    consecutive identical flow-work counter readings
 * @flow_work_cnt_last:    previous flow-work counter value (reg 0x91)
 * @hardware_reset_cnt:    number of HW resets triggered by ESD
 * @nack_cnt:              I2C NACK error count
 * @dataerror_cnt:         register read data-error count
 *
 * Allocated by fts_esdcheck_init() via devm_kzalloc; stored in
 * ts_data->esdcheck. All operational functions receive this struct
 * directly and reach the parent through ->ts_data.
 */
struct fts_esdcheck_data {
	struct fts_ts_data	*ts_data;
	struct delayed_work	 work;

	u8 mode		: 1;
	u8 suspend	: 1;
	u8 proc_debug	: 1;
	u8 intr		: 1;
	u8 unused	: 4;

	u8 intr_cnt;
	u8 flow_work_hold_cnt;
	u8 flow_work_cnt_last;
	u32 hardware_reset_cnt;
	u32 nack_cnt;
	u32 dataerror_cnt;

#if LCD_ESD_PATCH
	int lcd_need_reset;
	int tp_need_recovery; /* LCD reset cause Tp reset */
#endif
};

#endif /* __LINUX_FOCALTECH_ESDCHECK_H__ */
