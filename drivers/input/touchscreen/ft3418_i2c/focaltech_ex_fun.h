/* SPDX-License-Identifier: GPL-2.0-only */
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

#ifndef __FTS_EX_FUN_H__
#define __FTS_EX_FUN_H__

/*****************************************************************************
 * Included header files
 *****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
 * Private constant and macro definitions using #define
 *****************************************************************************/
#define PROC_BUF_SIZE		256
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER		2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA			6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG		8
#define PROC_SET_SLAVE_ADDR		10
#define PROC_HW_RESET			11
#define PROC_READ_STATUS		12
#define PROC_SET_BOOT_MODE		13
#define PROC_ENTER_TEST_ENVIRONMENT	14
#define PROC_NAME			"ftxxxx-debug"

/*****************************************************************************
 * Private enumerations, structures and unions using typedef
 *****************************************************************************/
enum {
	RWREG_OP_READ = 0,
	RWREG_OP_WRITE = 1,
};

struct fts_ex_fun_data {
	struct fts_ts_data *ts_data;    /* back pointer to core driver data */
	int type;                       /* 0: read, 1: write */
	int reg;                        /* register address */
	int len;                        /* read/write length */
	int val;                        /* single value for len=1 */
	int res;                        /* result: 0=success, <0=error */
	char *opbuf;                    /* dynamically allocated on demand */
	bool dirty;                     /* flag: opbuf contains unread result */
};

#endif /* __FTS_EX_FUN_H__ */
