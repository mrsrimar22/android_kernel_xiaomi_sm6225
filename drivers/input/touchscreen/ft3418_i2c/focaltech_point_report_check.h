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

#ifndef __LINUX_FOCALTECH_PRC_H__
#define __LINUX_FOCALTECH_PRC_H__

#include "focaltech_core.h"

/**
 * struct fts_prc_data - per-device point-report-check state
 * @ts_data: back-pointer to parent; always valid after init
 * @work:    delayed work; forcibly lifts all touch slots if no further
 *           events arrive within the timeout window
 *
 * Allocated by fts_point_report_check_init() via devm_kzalloc; stored
 * in ts_data->prc. All operational functions receive this struct
 * directly and reach the parent through ->ts_data.
 */
struct fts_prc_data {
	struct fts_ts_data	*ts_data;
	struct delayed_work	 work;
};

#endif /* __LINUX_FOCALTECH_PRC_H__ */
