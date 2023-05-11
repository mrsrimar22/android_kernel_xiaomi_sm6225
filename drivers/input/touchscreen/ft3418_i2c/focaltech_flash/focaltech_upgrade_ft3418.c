// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * FocalTech fts TouchScreen driver.
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
 *
 * File Name: focaltech_upgrade_ft3418.c
 *
 * Author: Focaltech Driver Team
 *
 * Created: 2016-08-15
 *
 * Abstract:
 *
 * Reference:
 *
 *****************************************************************************/

/*****************************************************************************
 * 1.Included header files
 *****************************************************************************/
#include "focaltech_flash.h"

static int fts_ft3418_upgrade(struct fts_upgrade *upg, u8 *buf, u32 len)
{
	int ret = 0, ecc_in_host = 0, ecc_in_tp = 0, i = 0;
	u32 start_addr = 0;
	u8 cmd[4] = {0}, wbuf[7] = {0}, reg_val[4] = {0};

	if (!upg || !upg->ts_data)
		return -EINVAL;

	if (!buf || len < FTS_MIN_LEN)
		return -EINVAL;

	ret = fts_fwupg_enter_into_boot(upg);
	if (ret < 0)
		goto fw_reset;

	cmd[0] = FTS_CMD_FLASH_MODE;
	cmd[1] = FLASH_MODE_UPGRADE_VALUE;
	ret = fts_write(upg->ts_data, cmd, 2);
	if (ret < 0)
		goto fw_reset;

	cmd[0] = FTS_CMD_DATA_LEN;
	cmd[1] = BYTE_OFF_16(len);
	cmd[2] = BYTE_OFF_8(len);
	cmd[3] = BYTE_OFF_0(len);
	ret = fts_write(upg->ts_data, cmd, FTS_CMD_DATA_LEN_LEN);
	if (ret < 0)
		goto fw_reset;

	ret = fts_fwupg_erase(upg, FTS_ERASE_APP_DELAY);
	if (ret < 0)
		goto fw_reset;

	start_addr = upgrade_func_ft3418.appoff;
	ecc_in_host = fts_flash_write_buf(upg, start_addr, buf, len, 1);
	if (ecc_in_host < 0)
		goto fw_reset;

	wbuf[0] = FTS_CMD_ECC_INIT;
	ret = fts_write(upg->ts_data, wbuf, 1);
	if (ret < 0)
		goto fw_reset;

	wbuf[0] = FTS_CMD_ECC_CAL;
	wbuf[1] = BYTE_OFF_16(start_addr);
	wbuf[2] = BYTE_OFF_8(start_addr);
	wbuf[3] = BYTE_OFF_0(start_addr);
	wbuf[4] = BYTE_OFF_16(len);
	wbuf[5] = BYTE_OFF_8(len);
	wbuf[6] = BYTE_OFF_0(len);

	ret = fts_write(upg->ts_data, wbuf, 7);
	if (ret < 0)
		goto fw_reset;

	msleep(len / 256);

	for (i = 0; i < FTS_RETRIES_ECC_CAL; i++) {
		wbuf[0] = FTS_CMD_FLASH_STATUS;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_read(upg->ts_data, wbuf, 1, reg_val, 2);
		if (reg_val[0] == 0xF0 && reg_val[1] == 0x55)
			break;

		msleep(FTS_RETRIES_DELAY_ECC_CAL);
	}

	wbuf[0] = FTS_CMD_ECC_READ;
	ret = fts_read(upg->ts_data, wbuf, 1, reg_val, 1);
	if (ret < 0)
		goto fw_reset;

	ecc_in_tp = reg_val[0];
	if (ecc_in_tp != ecc_in_host)
		goto fw_reset;

	ret = fts_fwupg_reset_in_boot(upg);
	if (ret < 0)
		return -EIO;

	msleep(200);
	return 0;

fw_reset:
	ret = fts_fwupg_reset_in_boot(upg);
	return -EIO;
}

struct upgrade_func upgrade_func_ft3418 = {
	.ctype = {0x81},
	.fwveroff = 0x010E,
	.fwcfgoff = 0x1FFB0,
	.appoff = 0x0000,
	.pramboot_supported = false,
	.hid_supported = true,
	.upgrade = fts_ft3418_upgrade,
};
