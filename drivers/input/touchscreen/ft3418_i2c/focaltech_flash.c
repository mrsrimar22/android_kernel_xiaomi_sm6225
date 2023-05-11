// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 */

#include "focaltech_flash.h"

#define FTS_FW_REQUEST_SUPPORT		1
#define FTS_FW_NAME_PREX_WITH_REQUEST	"focaltech_ts_fw_"

u8 fw_file[] = {
#include FTS_UPGRADE_FW_FILE
};

u8 fw_file2[] = {
#include FTS_UPGRADE_FW2_FILE
};

u8 fw_file3[] = {
#include FTS_UPGRADE_FW3_FILE
};

struct upgrade_module module_list[] = {
	{FTS_MODULE_ID, FTS_MODULE_NAME, fw_file, sizeof(fw_file)},
	{FTS_MODULE2_ID, FTS_MODULE2_NAME, fw_file2, sizeof(fw_file2)},
	{FTS_MODULE3_ID, FTS_MODULE3_NAME, fw_file3, sizeof(fw_file3)},
};

struct upgrade_func *upgrade_func_list[] = {
	&upgrade_func_ft3418,
};

struct fts_write_context {
	u32 total_packets;
	u32 written_packets;
	u32 start_addr;
	u32 total_len;
	bool partial_write_detected;
};

static bool fts_fwupg_check_state(struct fts_upgrade *upg,
				  enum FW_STATUS rstate);

static int fts_fwupg_get_boot_state(struct fts_upgrade *upg,
				    enum FW_STATUS *fw_sts)
{
	int ret;
	u8 cmd[4];
	u32 cmd_len;
	struct ft_chip_t *ids;

	FTS_DEBUG("**********read boot id**********");
	if (!upg || !upg->func || !upg->ts_data || !fw_sts)
		return -EINVAL;

	*fw_sts = FTS_RUN_IN_ERROR;
	if (upg->func->hid_supported)
		fts_hid2std(upg->ts_data);

	cmd[0] = FTS_CMD_START1;
	cmd[1] = FTS_CMD_START2;
	if (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
		cmd_len = 1;
	else
		cmd_len = 2;

	ret = fts_write(upg->ts_data, cmd, cmd_len);
	if (ret < 0) {
		FTS_ERROR("write 55 cmd fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	cmd[0] = FTS_CMD_READ_ID;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	cmd[3] = 0x00;
	if (upg->ts_data->ic_info.is_incell ||
	    upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
		cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
	else
		cmd_len = FTS_CMD_READ_ID_LEN;

	ret = fts_read(upg->ts_data, cmd, cmd_len, cmd, 2);
	if (ret < 0) {
		FTS_ERROR("write 90 cmd fail");
		return ret;
	}

	ids = &upg->ts_data->ic_info.ids;
	if (cmd[0] == ids->rom_idh && cmd[1] == ids->rom_idl)
		*fw_sts = FTS_RUN_IN_ROM;
	else if (cmd[0] == ids->pb_idh && cmd[1] == ids->pb_idl)
		*fw_sts = FTS_RUN_IN_PRAM;
	else if (cmd[0] == ids->bl_idh && cmd[1] == ids->bl_idl)
		*fw_sts = FTS_RUN_IN_BOOTLOADER;

	return 0;
}

static int fts_fwupg_reset_to_boot(struct fts_upgrade *upg)
{
	int ret;
	u8 reg = FTS_REG_UPGRADE;

	if (upg && upg->func && upg->func->is_reset_register_BC)
		reg = FTS_REG_UPGRADE2;

	ret = fts_write_reg(upg->ts_data, reg, FTS_UPGRADE_AA);
	if (ret < 0)
		return ret;
	msleep(FTS_DELAY_UPGRADE_AA);

	ret = fts_write_reg(upg->ts_data, reg, FTS_UPGRADE_55);
	if (ret < 0)
		return ret;
	msleep(FTS_DELAY_UPGRADE_RESET);

	return 0;
}

static int fts_fwupg_reset_to_romboot(struct fts_upgrade *upg)
{
	int ret, i;
	u8 cmd = FTS_CMD_RESET;
	enum FW_STATUS state;

	ret = fts_write(upg->ts_data, &cmd, 1);
	if (ret < 0)
		return ret;
	mdelay(10);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		ret = fts_fwupg_get_boot_state(upg, &state);
		if (state == FTS_RUN_IN_ROM)
			break;
		mdelay(5);
	}

	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	return 0;
}

static u16 fts_crc16_calc_host(u8 *pbuf, u32 length)
{
	u16 ecc = 0;
	u32 i, j;

	for (i = 0; i < length; i += 2) {
		ecc ^= ((pbuf[i] << 8) | (pbuf[i + 1]));
		for (j = 0; j < 16; j++) {
			if (ecc & 0x01)
				ecc = (u16)((ecc >> 1) ^ AL2_FCS_COEF);
			else
				ecc >>= 1;
		}
	}
	return ecc;
}

static u16 fts_pram_ecc_calc_host(u8 *pbuf, u32 length)
{
	return fts_crc16_calc_host(pbuf, length);
}

static int fts_pram_ecc_cal_algo(struct fts_upgrade *upg,
				 u32 start_addr, u32 ecc_length)
{
	int ret, i;
	u8 val[2], tmp;
	u8 cmd[FTS_ROMBOOT_CMD_ECC_NEW_LEN];

	cmd[0] = FTS_ROMBOOT_CMD_ECC;
	cmd[1] = BYTE_OFF_16(start_addr);
	cmd[2] = BYTE_OFF_8(start_addr);
	cmd[3] = BYTE_OFF_0(start_addr);
	cmd[4] = BYTE_OFF_16(ecc_length);
	cmd[5] = BYTE_OFF_8(ecc_length);
	cmd[6] = BYTE_OFF_0(ecc_length);
	ret = fts_write(upg->ts_data, cmd, FTS_ROMBOOT_CMD_ECC_NEW_LEN);
	if (ret < 0)
		return ret;

	cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
	for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
		usleep_range(1000, 1500);
		ret = fts_read(upg->ts_data, cmd, 1, val, 1);
		if (ret < 0)
			return ret;

		if (upg->func->new_return_value_from_ic ||
		    upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
			tmp = FTS_ROMBOOT_CMD_ECC_FINISH_OK_A5;
		else
			tmp = FTS_ROMBOOT_CMD_ECC_FINISH_OK_00;

		if (tmp == val[0])
			break;
	}
	if (i >= FTS_ECC_FINISH_TIMEOUT)
		return -EIO;

	cmd[0] = FTS_ROMBOOT_CMD_ECC_READ;
	ret = fts_read(upg->ts_data, cmd, 1, val, 2);
	if (ret < 0)
		return ret;

	return ((u16)(val[0] << 8) + val[1]) & 0x0000FFFF;
}

static int fts_pram_ecc_cal_xor(struct fts_upgrade *upg)
{
	int ret;
	u8 reg_val;

	ret = fts_read_reg(upg->ts_data, FTS_ROMBOOT_CMD_ECC, &reg_val);
	if (ret < 0)
		return ret;

	return (int)reg_val;
}

static int fts_pram_ecc_cal(struct fts_upgrade *upg, u32 saddr, u32 len)
{
	if (!upg || !upg->func)
		return -EINVAL;

	if (upg->func->pram_ecc_check_mode == ECC_CHECK_MODE_CRC16 ||
	    upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
		return fts_pram_ecc_cal_algo(upg, saddr, len);
	else
		return fts_pram_ecc_cal_xor(upg);
}

static int fts_pram_write_buf(struct fts_upgrade *upg, u8 *buf, u32 len)
{
	int ret, ecc_in_host;
	u32 i, j, offset, remainder, packet_number, packet_len, cmdlen;
	u8 packet_buf[FTS_FLASH_PACKET_LENGTH + FTS_CMD_WRITE_LEN];
	u8 ecc_tmp = 0;

	if (!upg || !upg->func || !buf)
		return -EINVAL;

	if (len < PRAMBOOT_MIN_SIZE || len > PRAMBOOT_MAX_SIZE)
		return -EINVAL;

	packet_number = len / FTS_FLASH_PACKET_LENGTH;
	remainder = len % FTS_FLASH_PACKET_LENGTH;
	if (remainder > 0)
		packet_number++;
	packet_len = FTS_FLASH_PACKET_LENGTH;

	for (i = 0; i < packet_number; i++) {
		offset = i * FTS_FLASH_PACKET_LENGTH;
		if ((i == (packet_number - 1)) && remainder)
			packet_len = remainder;

		if (upg->ts_data->bus_type == BUS_TYPE_SPI_V2) {
			packet_buf[0] = FTS_ROMBOOT_CMD_SET_PRAM_ADDR;
			packet_buf[1] = BYTE_OFF_16(offset);
			packet_buf[2] = BYTE_OFF_8(offset);
			packet_buf[3] = BYTE_OFF_0(offset);

			ret = fts_write(upg->ts_data, packet_buf,
					FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN);
			if (ret < 0)
				return ret;

			packet_buf[0] = FTS_ROMBOOT_CMD_WRITE;
			cmdlen = 1;
		} else {
			packet_buf[0] = FTS_ROMBOOT_CMD_WRITE;
			packet_buf[1] = BYTE_OFF_16(offset);
			packet_buf[2] = BYTE_OFF_8(offset);
			packet_buf[3] = BYTE_OFF_0(offset);
			packet_buf[4] = BYTE_OFF_8(packet_len);
			packet_buf[5] = BYTE_OFF_0(packet_len);
			cmdlen = 6;
		}

		for (j = 0; j < packet_len; j++) {
			packet_buf[cmdlen + j] = buf[offset + j];
			if (upg->func->pram_ecc_check_mode == ECC_CHECK_MODE_XOR)
				ecc_tmp ^= packet_buf[cmdlen + j];
		}

		ret = fts_write(upg->ts_data, packet_buf, packet_len + cmdlen);
		if (ret < 0)
			return ret;
	}

	if (upg->func->pram_ecc_check_mode == ECC_CHECK_MODE_CRC16 ||
	    upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
		ecc_in_host = (int)fts_pram_ecc_calc_host(buf, len);
	else
		ecc_in_host = (int)ecc_tmp;

	return ecc_in_host;
}

static int fts_pram_start(struct fts_upgrade *upg)
{
	u8 cmd = FTS_ROMBOOT_CMD_START_APP;
	int ret = fts_write(upg->ts_data, &cmd, 1);

	if (ret < 0)
		return ret;

	msleep(FTS_DELAY_PRAMBOOT_START);
	return 0;
}

static int fts_pram_write_remap(struct fts_upgrade *upg)
{
	int ret, ecc_in_host, ecc_in_tp;
	u8 *pb_buf;
	u32 pb_len;

	if (!upg || !upg->func || !upg->func->pramboot)
		return -EINVAL;

	if (upg->func->pb_length < FTS_MIN_LEN)
		return -EINVAL;

	pb_buf = upg->func->pramboot;
	pb_len = upg->func->pb_length;

	ecc_in_host = fts_pram_write_buf(upg, pb_buf, pb_len);
	if (ecc_in_host < 0)
		return ecc_in_host;

	ecc_in_tp = fts_pram_ecc_cal(upg, 0, pb_len);
	if (ecc_in_tp < 0)
		return ecc_in_tp;

	if (ecc_in_host != ecc_in_tp)
		return -EIO;

	ret = fts_pram_start(upg);
	if (ret < 0)
		return ret;

	return 0;
}

static int fts_pram_init(struct fts_upgrade *upg)
{
	int ret;
	u8 reg_val;
	u8 wbuf[3];

	wbuf[0] = FTS_CMD_FLASH_TYPE;
	ret = fts_read(upg->ts_data, wbuf, 1, &reg_val, 1);
	if (ret < 0)
		return ret;

	wbuf[0] = FTS_CMD_FLASH_TYPE;
	wbuf[1] = reg_val;
	wbuf[2] = 0x00;
	ret = fts_write(upg->ts_data, wbuf, 3);
	if (ret < 0)
		return ret;

	return 0;
}

static int fts_pram_write_init(struct fts_upgrade *upg)
{
	int ret;
	bool state;
	enum FW_STATUS status;

	if (!upg || !upg->func)
		return -EINVAL;

	if (!upg->func->pramboot_supported)
		return -EINVAL;

	ret = fts_fwupg_get_boot_state(upg, &status);
	if (status != FTS_RUN_IN_ROM) {
		if (status == FTS_RUN_IN_PRAM) {
			ret = fts_pram_init(upg);
			if (ret < 0)
				return ret;
		}

		ret = fts_fwupg_reset_to_romboot(upg);
		if (ret < 0)
			return ret;
	}

	ret = fts_pram_write_remap(upg);
	if (ret < 0)
		return ret;

	state = fts_fwupg_check_state(upg, FTS_RUN_IN_PRAM);
	if (!state)
		return -EIO;

	ret = fts_pram_init(upg);
	if (ret < 0)
		return ret;

	return 0;
}

static bool fts_fwupg_check_fw_valid(struct fts_upgrade *upg)
{
	int ret = fts_wait_tp_to_valid(upg->ts_data, true);

	return (ret < 0) ? false : true;
}

static bool fts_fwupg_check_state(struct fts_upgrade *upg,
				  enum FW_STATUS rstate)
{
	int i;
	enum FW_STATUS cstate;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		fts_fwupg_get_boot_state(upg, &cstate);
		if (cstate == rstate)
			return true;
		msleep(FTS_DELAY_READ_ID);
	}
	return false;
}

int fts_fwupg_reset_in_boot(struct fts_upgrade *upg)
{
	int ret;
	u8 cmd = FTS_CMD_RESET;

	ret = fts_write(upg->ts_data, &cmd, 1);
	if (ret < 0)
		return ret;

	msleep(FTS_DELAY_UPGRADE_RESET);
	return 0;
}

int fts_fwupg_enter_into_boot(struct fts_upgrade *upg)
{
	int ret;
	bool fwvalid, state;

	if (!upg || !upg->func)
		return -EINVAL;

	fwvalid = fts_fwupg_check_fw_valid(upg);
	if (fwvalid) {
		ret = fts_fwupg_reset_to_boot(upg);
		if (ret < 0)
			return ret;
	} else if (upg->func->read_boot_id_need_reset) {
		ret = fts_fwupg_reset_in_boot(upg);
		if (ret < 0)
			return ret;
	}

	if (upg->func->pramboot_supported) {
		if (upg->func->write_pramboot_private)
			ret = upg->func->write_pramboot_private(upg);
		else
			ret = fts_pram_write_init(upg);

		if (ret < 0)
			return ret;
	} else {
		state = fts_fwupg_check_state(upg, FTS_RUN_IN_BOOTLOADER);
		if (!state)
			return -EIO;
	}

	return 0;
}

static bool fts_fwupg_check_flash_status(struct fts_upgrade *upg,
					 u16 flash_status,
					 int retries,
					 int retries_delay)
{
	int i;
	u8 cmd;
	u8 val[FTS_CMD_FLASH_STATUS_LEN] = {0};
	u16 read_status;

	for (i = 0; i < retries; i++) {
		cmd = FTS_CMD_FLASH_STATUS;
		fts_read(upg->ts_data, &cmd, 1,
			 val, FTS_CMD_FLASH_STATUS_LEN);
		read_status = (((u16)val[0]) << 8) + val[1];
		if (flash_status == read_status)
			return true;
		msleep(retries_delay);
	}
	return false;
}

int fts_fwupg_erase(struct fts_upgrade *upg, u32 delay)
{
	int ret;
	u8 cmd = FTS_CMD_ERASE_APP;
	bool flag;

	ret = fts_write(upg->ts_data, &cmd, 1);
	if (ret < 0)
		return ret;

	msleep(delay);
	flag = fts_fwupg_check_flash_status(upg,
					    FTS_CMD_FLASH_STATUS_ERASE_OK,
					    FTS_RETRIES_ERASE,
					    FTS_RETRIES_DELAY_ERASE);
	if (!flag)
		return -EIO;

	return 0;
}

int fts_fwupg_ecc_cal(struct fts_upgrade *upg, u32 saddr, u32 len)
{
	int ret, ecc, ecc_len;
	u32 i, cmdlen, packet_num, packet_len, remainder, addr, offset;
	u8 wbuf[FTS_CMD_ECC_CAL_LEN];
	u8 val[FTS_CMD_FLASH_STATUS_LEN];
	bool bflag;

	if (!upg || !upg->func)
		return -EINVAL;

	wbuf[0] = FTS_CMD_ECC_INIT;
	ret = fts_write(upg->ts_data, wbuf, 1);
	if (ret < 0)
		return ret;

	if (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0) {
		packet_num = 1;
		remainder = 0;
		packet_len = len;
	} else {
		packet_num = len / FTS_MAX_LEN_ECC_CALC;
		remainder = len % FTS_MAX_LEN_ECC_CALC;
		if (remainder)
			packet_num++;
		packet_len = FTS_MAX_LEN_ECC_CALC;
	}

	wbuf[0] = FTS_CMD_ECC_CAL;
	for (i = 0; i < packet_num; i++) {
		offset = FTS_MAX_LEN_ECC_CALC * i;
		addr = saddr + offset;
		wbuf[1] = BYTE_OFF_16(addr);
		wbuf[2] = BYTE_OFF_8(addr);
		wbuf[3] = BYTE_OFF_0(addr);

		if (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0) {
			wbuf[4] = BYTE_OFF_16(packet_len);
			wbuf[5] = BYTE_OFF_8(packet_len);
			wbuf[6] = BYTE_OFF_0(packet_len);
			cmdlen = FTS_CMD_ECC_CAL_LEN;
		} else {
			if ((i == (packet_num - 1)) && remainder)
				packet_len = remainder;
			wbuf[4] = BYTE_OFF_8(packet_len);
			wbuf[5] = BYTE_OFF_0(packet_len);
			cmdlen = FTS_CMD_ECC_CAL_LEN - 1;
		}

		ret = fts_write(upg->ts_data, wbuf, cmdlen);
		if (ret < 0)
			return ret;

		msleep(packet_len / 256);

		bflag = fts_fwupg_check_flash_status(upg,
						     FTS_CMD_FLASH_STATUS_ECC_OK,
						     FTS_RETRIES_ECC_CAL,
						     FTS_RETRIES_DELAY_ECC_CAL);
		if (!bflag)
			return -EIO;
	}

	if (upg->func->fw_ecc_check_mode == ECC_CHECK_MODE_CRC16 ||
	    upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
		ecc_len = 2;
	else
		ecc_len = 1;

	wbuf[0] = FTS_CMD_ECC_READ;
	ret = fts_read(upg->ts_data, wbuf, 1, val, ecc_len);
	if (ret < 0)
		return ret;

	if (upg->func->fw_ecc_check_mode == ECC_CHECK_MODE_CRC16 ||
	    upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
		ecc = (int)((u16)(val[0] << 8) + val[1]);
	else
		ecc = (int)val[0];

	return ecc;
}

int fts_flash_write_buf(struct fts_upgrade *upg,
			u32 saddr, u8 *buf,
			u32 len, u32 delay)
{
	int ret, ecc_in_host;
	u32 i, j, packet_number, packet_len, addr, offset, remainder, cmdlen;
	u8 packet_buf[FTS_FLASH_PACKET_LENGTH + FTS_CMD_WRITE_LEN];
	u8 ecc_tmp = 0;
	u8 cmd;
	u8 val[FTS_CMD_FLASH_STATUS_LEN];
	u16 read_status, wr_ok;
	struct fts_write_context write_ctx = {0};

	if (!upg || !upg->func || !buf || !len)
		return -EINVAL;

	packet_number = len / FTS_FLASH_PACKET_LENGTH;
	remainder = len % FTS_FLASH_PACKET_LENGTH;
	if (remainder > 0)
		packet_number++;
	packet_len = FTS_FLASH_PACKET_LENGTH;

	write_ctx.total_packets = packet_number;
	write_ctx.start_addr = saddr;
	write_ctx.total_len = len;

	for (i = 0; i < packet_number; i++) {
		offset = i * FTS_FLASH_PACKET_LENGTH;
		addr = saddr + offset;

		if ((i == (packet_number - 1)) && remainder)
			packet_len = remainder;

		if (upg->ts_data->bus_type == BUS_TYPE_SPI_V2) {
			packet_buf[0] = FTS_CMD_SET_WFLASH_ADDR;
			packet_buf[1] = BYTE_OFF_16(addr);
			packet_buf[2] = BYTE_OFF_8(addr);
			packet_buf[3] = BYTE_OFF_0(addr);
			ret = fts_write(upg->ts_data, packet_buf, FTS_LEN_SET_ADDR);
			if (ret < 0) {
				write_ctx.partial_write_detected = true;
				break;
			}

			packet_buf[0] = FTS_CMD_WRITE;
			cmdlen = 1;
		} else {
			packet_buf[0] = FTS_CMD_WRITE;
			packet_buf[1] = BYTE_OFF_16(addr);
			packet_buf[2] = BYTE_OFF_8(addr);
			packet_buf[3] = BYTE_OFF_0(addr);
			packet_buf[4] = BYTE_OFF_8(packet_len);
			packet_buf[5] = BYTE_OFF_0(packet_len);
			cmdlen = 6;
		}

		for (j = 0; j < packet_len; j++) {
			packet_buf[cmdlen + j] = buf[offset + j];
			ecc_tmp ^= packet_buf[cmdlen + j];
		}

		ret = fts_write(upg->ts_data, packet_buf, packet_len + cmdlen);
		if (ret < 0) {
			write_ctx.partial_write_detected = true;
			break;
		}
		mdelay(delay);

		wr_ok = FTS_CMD_FLASH_STATUS_WRITE_OK + addr / FTS_FLASH_PACKET_LENGTH;
		for (j = 0; j < FTS_RETRIES_WRITE; j++) {
			cmd = FTS_CMD_FLASH_STATUS;
			ret = fts_read(upg->ts_data, &cmd, 1,
				       val, FTS_CMD_FLASH_STATUS_LEN);
			if (ret < 0)
				continue;

			read_status = (((u16)val[0]) << 8) + val[1];
			if (wr_ok == read_status) {
				write_ctx.written_packets++;
				break;
			}
			mdelay(FTS_RETRIES_DELAY_WRITE);
		}

		if (j >= FTS_RETRIES_WRITE) {
			write_ctx.partial_write_detected = true;
			break;
		}
	}

	if (write_ctx.partial_write_detected)
		return -EIO;

	if (upg->func->fw_ecc_check_mode == ECC_CHECK_MODE_CRC16 ||
	    upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
		ecc_in_host = (int)fts_crc16_calc_host(buf, len);
	else
		ecc_in_host = (int)ecc_tmp;

	return ecc_in_host;
}

static int fts_flash_read_buf(struct fts_upgrade *upg,
			      u32 saddr, u8 *buf, u32 len)
{
	int ret;
	u32 i, packet_number, packet_len, addr, offset, remainder;
	u8 wbuf[FTS_CMD_READ_LEN_SPI];

	if (!upg || !buf || !len)
		return -EINVAL;

	packet_number = len / FTS_FLASH_PACKET_LENGTH;
	remainder = len % FTS_FLASH_PACKET_LENGTH;
	if (remainder > 0)
		packet_number++;
	packet_len = FTS_FLASH_PACKET_LENGTH;

	for (i = 0; i < packet_number; i++) {
		offset = i * FTS_FLASH_PACKET_LENGTH;
		addr = saddr + offset;
		if ((i == (packet_number - 1)) && remainder)
			packet_len = remainder;

		if (upg->ts_data->bus_type == BUS_TYPE_I2C) {
			wbuf[0] = FTS_CMD_READ;
			wbuf[1] = BYTE_OFF_16(addr);
			wbuf[2] = BYTE_OFF_8(addr);
			wbuf[3] = BYTE_OFF_0(addr);
			ret = fts_write(upg->ts_data, wbuf, FTS_CMD_READ_LEN);
			if (ret < 0)
				return ret;

			msleep(FTS_CMD_READ_DELAY);
			ret = fts_read(upg->ts_data, NULL, 0, buf + offset, packet_len);
			if (ret < 0)
				return ret;
		} else if (upg->ts_data->bus_type == BUS_TYPE_SPI_V2) {
			wbuf[0] = FTS_CMD_SET_RFLASH_ADDR;
			wbuf[1] = BYTE_OFF_16(addr);
			wbuf[2] = BYTE_OFF_8(addr);
			wbuf[3] = BYTE_OFF_0(addr);
			ret = fts_write(upg->ts_data, wbuf, FTS_LEN_SET_ADDR);
			if (ret < 0)
				return ret;

			msleep(FTS_CMD_READ_DELAY);
			wbuf[0] = FTS_CMD_READ;
			ret = fts_read(upg->ts_data, wbuf, 1, buf + offset, packet_len);
			if (ret < 0)
				return ret;
		} else if (upg->ts_data->bus_type == BUS_TYPE_SPI) {
			wbuf[0] = FTS_CMD_READ;
			wbuf[1] = BYTE_OFF_16(addr);
			wbuf[2] = BYTE_OFF_8(addr);
			wbuf[3] = BYTE_OFF_0(addr);
			wbuf[4] = BYTE_OFF_8(packet_len);
			wbuf[5] = BYTE_OFF_0(packet_len);
			ret = fts_read(upg->ts_data, wbuf, FTS_CMD_READ_LEN_SPI,
				       buf + offset, packet_len);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

int fts_flash_read(struct fts_ts_data *ts_data, u32 addr, u8 *buf, u32 len)
{
	int ret;
	struct fts_upgrade *upg;

	if (!ts_data || !ts_data->upg || !buf || len == 0)
		return -EINVAL;
	upg = ts_data->upg;

	ret = fts_fwupg_enter_into_boot(upg);
	if (ret < 0)
		goto read_flash_err;

	ret = fts_flash_read_buf(upg, addr, buf, len);
	if (ret < 0)
		goto read_flash_err;

read_flash_err:
	fts_fwupg_reset_in_boot(upg);
	return ret;
}

static int fts_read_file(char *file_name, u8 **file_buf)
{
	int ret;
	char file_path[FILE_NAME_LENGTH] = {0};
	struct file *filp;
	struct inode *inode;
	mm_segment_t old_fs;
	loff_t pos = 0, file_len;
	u8 *buffer;
	bool fs_changed = false;

	if (!file_name || !file_buf)
		return -EINVAL;

	*file_buf = NULL;
	snprintf(file_path, FILE_NAME_LENGTH, "%s%s",
		 FTS_FW_BIN_FILEPATH, file_name);
	filp = filp_open(file_path, O_RDONLY, 0);
	if (IS_ERR(filp))
		return -ENOENT;

	inode = filp->f_inode;
	file_len = inode->i_size;

	buffer = kvmalloc(file_len, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto read_file_error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fs_changed = true;

	ret = vfs_read(filp, buffer, file_len, &pos);

	set_fs(old_fs);
	fs_changed = false;

	if (ret < 0)
		goto read_file_error;

	*file_buf = buffer;
	filp_close(filp, NULL);
	return ret;

read_file_error:
	if (fs_changed)
		set_fs(old_fs);
	kvfree(buffer);
	if (filp && !IS_ERR(filp))
		filp_close(filp, NULL);
	*file_buf = NULL;
	return ret;
}

int fts_upgrade_bin(struct fts_ts_data *ts_data,
		    char *fw_name, bool force)
{
	int ret;
	u32 fw_file_len;
	u8 *fw_file_buf;
	struct fts_upgrade *upg;

	if (!ts_data || !ts_data->upg || !ts_data->upg->func)
		return -EINVAL;
	upg = ts_data->upg;

	ts_data->fw_loading = 1;
	fts_irq_disable(ts_data);
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_switch(ts_data->esdcheck, DISABLE);
#endif

	ret = fts_read_file(fw_name, &fw_file_buf);
	if (ret < 0 || ret < FTS_MIN_LEN)
		goto err_bin;

	fw_file_len = ret;
	if (force) {
		if (upg->func->force_upgrade)
			ret = upg->func->force_upgrade(upg, fw_file_buf, fw_file_len);
		else
			goto err_bin;
	} else {
#if FTS_AUTO_LIC_UPGRADE_EN
		if (upg->func->lic_upgrade)
			ret = upg->func->lic_upgrade(upg, fw_file_buf, fw_file_len);
#endif
		if (upg->func->upgrade)
			ret = upg->func->upgrade(upg, fw_file_buf, fw_file_len);
	}

	if (ret < 0) {
		fts_fwupg_reset_in_boot(upg);
		goto err_bin;
	}

	ret = 0;

err_bin:
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_switch(ts_data->esdcheck, ENABLE);
#endif
	fts_irq_enable(ts_data);
	ts_data->fw_loading = 0;

	kvfree(fw_file_buf);
	return ret;
}

int fts_enter_test_environment(struct fts_ts_data *ts_data, bool test_state)
{
	return 0;
}

static int fts_param_get_ver_in_tp(struct fts_upgrade *upg, u8 *ver)
{
	if (!ver)
		return -EINVAL;
	if (fts_read_reg(upg->ts_data, FTS_REG_IDE_PARA_VER_ID, ver) < 0)
		return -EIO;
	if (*ver == 0x00 || *ver == 0xFF)
		return -EIO;
	return 0;
}

static int fts_param_get_ver_in_host(struct fts_upgrade *upg, u8 *ver)
{
	if (!upg || !upg->func || !upg->fw || !ver)
		return -EINVAL;
	if (upg->fw_length < upg->func->paramcfgveroff)
		return -EINVAL;

	*ver = upg->fw[upg->func->paramcfgveroff];
	if (*ver == 0x00 || *ver == 0xFF)
		return -EIO;
	return 0;
}

static int fts_param_ide_in_host(struct fts_upgrade *upg)
{
	u32 off;

	if (!upg || !upg->func || !upg->fw)
		return -EINVAL;
	if (upg->fw_length < upg->func->paramcfgoff + FTS_FW_IDE_SIG_LEN)
		return 0;

	off = upg->func->paramcfgoff;
	if (memcmp(&upg->fw[off], FTS_FW_IDE_SIG, FTS_FW_IDE_SIG_LEN) == 0)
		return 1;

	return 0;
}

static int fts_param_ide_in_tp(struct fts_upgrade *upg, u8 *val)
{
	if (fts_read_reg(upg->ts_data, FTS_REG_IDE_PARA_STATUS, val) < 0)
		return -EIO;
	if (*val != 0xFF && (*val & 0x80) == 0x80)
		return 1;
	return 0;
}

static int fts_param_need_upgrade(struct fts_upgrade *upg)
{
	int ret, ide_in_host, ide_in_tp;
	u8 val, ver_in_host, ver_in_tp;

	if (!fts_fwupg_check_fw_valid(upg))
		return 1;

	ide_in_host = fts_param_ide_in_host(upg);
	if (ide_in_host < 0)
		return ide_in_host;

	ide_in_tp = fts_param_ide_in_tp(upg, &val);
	if (ide_in_tp < 0)
		return ide_in_tp;

	if (ide_in_host == 0 && ide_in_tp == 0) {
		return 0;
	} else if (ide_in_host != ide_in_tp) {
		return 1;
	} else if (ide_in_host == 1 && ide_in_tp == 1) {
		if ((val & 0x7F) != 0x00)
			return 2;

		ret = fts_param_get_ver_in_host(upg, &ver_in_host);
		if (ret < 0)
			return ret;

		ret = fts_param_get_ver_in_tp(upg, &ver_in_tp);
		if (ret < 0)
			return ret;

		if (ver_in_tp != ver_in_host)
			return 2;
	}
	return 0;
}

static int fts_fwupg_get_ver_in_tp(struct fts_upgrade *upg, u8 *ver)
{
	if (!ver)
		return -EINVAL;

	return fts_read_reg(upg->ts_data, FTS_REG_FW_VER, ver);
}

static int fts_fwupg_get_ver_in_host(struct fts_upgrade *upg, u8 *ver)
{
	if (!upg || !upg->func || !upg->fw || !ver)
		return -EINVAL;
	if (upg->fw_length < upg->func->fwveroff)
		return -EINVAL;

	*ver = upg->fw[upg->func->fwveroff];
	return 0;
}

static bool fts_fwupg_need_upgrade(struct fts_upgrade *upg)
{
	u8 fw_ver_in_host, fw_ver_in_tp;

	if (fts_fwupg_check_fw_valid(upg)) {
		if (fts_fwupg_get_ver_in_host(upg, &fw_ver_in_host) < 0)
			return false;
		if (fts_fwupg_get_ver_in_tp(upg, &fw_ver_in_tp) < 0)
			return false;
		if (fw_ver_in_tp != fw_ver_in_host)
			return true;
	} else {
		return true;
	}
	return false;
}

static int fts_fwupg_do_fw_upgrade(struct fts_upgrade *upg)
{
	int ret;

	if (!upg->func->upgrade)
		return -ENODATA;

	ret = upg->func->upgrade(upg, upg->fw, upg->fw_length);
	if (ret < 0)
		fts_fwupg_reset_in_boot(upg);

	return ret;
}

static int fts_fwupg_do_param_upgrade(struct fts_upgrade *upg)
{
	int ret;

	if (!upg->func->param_upgrade)
		return 0;

	ret = fts_param_need_upgrade(upg);
	if (ret <= 0)
		return 0;

	if (ret == 1)
		return fts_fwupg_do_fw_upgrade(upg);

	if (ret == 2) {
		ret = upg->func->param_upgrade(upg, upg->fw, upg->fw_length);
		if (ret < 0)
			fts_fwupg_reset_in_boot(upg);
		return ret;
	}
	return 0;
}

int fts_fwupg_upgrade(struct fts_upgrade *upg)
{
	int ret = 0;
	int upgrade_count = 0;
	bool upgrade_flag;

	if (!upg || !upg->func)
		return -EINVAL;

	upgrade_flag = fts_fwupg_need_upgrade(upg);

	do {
		upgrade_count++;
		if (upgrade_flag)
			ret = fts_fwupg_do_fw_upgrade(upg);
		else
			ret = fts_fwupg_do_param_upgrade(upg);

		if (ret == 0 || ret == -ENODATA)
			break;
	} while (upgrade_count < 2);

	return ret;
}

static void fts_fwupg_auto_upgrade(struct fts_upgrade *upg)
{
	if (!upg || !upg->ts_data)
		return;
	fts_fwupg_upgrade(upg);
}

static int fts_fwupg_get_vendorid(struct fts_upgrade *upg, int *vid)
{
	int ret;
	u8 vendor_id = 0, module_id = 0, cmd;
	u8 cfgbuf[FTS_HEADER_LEN];

	if (!upg || !upg->func || !upg->ts_data || !vid)
		return -EINVAL;

	if (fts_fwupg_check_fw_valid(upg)) {
		ret = fts_read_reg(upg->ts_data, FTS_REG_VENDOR_ID, &vendor_id);
		if (upg->ts_data->ic_info.is_incell)
			ret = fts_read_reg(upg->ts_data, FTS_REG_MODULE_ID, &module_id);
	} else {
		if (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0) {
			cmd = FTS_CMD_READ_FW_CONF;
			ret = fts_read(upg->ts_data, &cmd, 1,
				       cfgbuf, FTS_HEADER_LEN);
		} else {
			ret = fts_flash_read(upg->ts_data, upg->func->fwcfgoff,
					     cfgbuf, FTS_HEADER_LEN);
		}

		if ((cfgbuf[FTS_CONIFG_VENDORID_OFF] +
			cfgbuf[FTS_CONIFG_VENDORID_OFF + 1]) == 0xFF)
			vendor_id = cfgbuf[FTS_CONIFG_VENDORID_OFF];
		if (upg->ts_data->ic_info.is_incell) {
			if ((cfgbuf[FTS_CONIFG_MODULEID_OFF] +
				cfgbuf[FTS_CONIFG_MODULEID_OFF + 1]) == 0xFF)
				module_id = cfgbuf[FTS_CONIFG_MODULEID_OFF];
		}
	}

	if (ret < 0)
		return ret;

	*vid = (int)((module_id << 8) + vendor_id);
	return 0;
}

static int fts_fwupg_get_module_info(struct fts_upgrade *upg)
{
	int ret, i;
	struct upgrade_module *info = &module_list[0];

	if (!upg || !upg->ts_data)
		return -EINVAL;

	if (FTS_GET_MODULE_NUM > 1) {
		ret = fts_fwupg_get_vendorid(upg, &upg->module_id);
		if (ret < 0)
			return ret;

		for (i = 0; i < FTS_GET_MODULE_NUM; i++) {
			info = &module_list[i];
			if (upg->module_id == info->id)
				break;
		}

		if (i >= FTS_GET_MODULE_NUM)
			return -ENODATA;
	}

	upg->module_info = info;
	return 0;
}

static int fts_get_fw_file_via_request_firmware(struct fts_upgrade *upg)
{
	int ret;
	const struct firmware *fw = NULL;
	u8 *tmpbuf;
	char fwname[FILE_NAME_LENGTH];

	if (!upg || !upg->ts_data || !upg->ts_data->dev)
		return -EINVAL;

	snprintf(fwname, FILE_NAME_LENGTH, "%s%s.bin",
		 FTS_FW_NAME_PREX_WITH_REQUEST,
		 upg->module_info->vendor_name);

	ret = request_firmware(&fw, fwname, upg->ts_data->dev);
	if (ret == 0) {
		tmpbuf = kvmalloc(fw->size, GFP_KERNEL);
		if (!tmpbuf) {
			ret = -ENOMEM;
		} else {
			memcpy(tmpbuf, fw->data, fw->size);
			upg->fw = tmpbuf;
			upg->fw_length = fw->size;
			upg->fw_from_request = 1;
		}
	}

	if (fw)
		release_firmware(fw);
	return ret;
}

static int fts_get_fw_file_via_i(struct fts_upgrade *upg)
{
	upg->fw = upg->module_info->fw_file;
	upg->fw_length = upg->module_info->fw_len;
	upg->fw_from_request = 0;
	return 0;
}

static int fts_fwupg_get_fw_file(struct fts_upgrade *upg)
{
	int ret;
	bool get_fw_i_flag = false;

	if (!upg || !upg->ts_data)
		return -EINVAL;

	ret = fts_fwupg_get_module_info(upg);
	if (ret < 0 || !upg->module_info)
		return ret;

	if (FTS_FW_REQUEST_SUPPORT) {
		ret = fts_get_fw_file_via_request_firmware(upg);
		if (ret != 0)
			get_fw_i_flag = true;
	} else {
		get_fw_i_flag = true;
	}

	if (get_fw_i_flag)
		ret = fts_get_fw_file_via_i(upg);

	upg->lic = upg->fw;
	upg->lic_length = upg->fw_length;

	if (upg->fw_length < FTS_MIN_LEN)
		return -ENODATA;

	return ret;
}

static void fts_fwupg_init_ic_detail(struct fts_upgrade *upg)
{
	if (upg && upg->func && upg->func->init)
		upg->func->init(upg, upg->fw, upg->fw_length);
}

static void fts_fwupg_work(struct work_struct *work)
{
	int ret;
	struct fts_ts_data *ts_data =
		container_of(work, struct fts_ts_data, fwupg_work);
	struct fts_upgrade *upg = ts_data->upg;

#if !FTS_AUTO_UPGRADE_EN
	return;
#endif

	if (!upg)
		return;

	ts_data->fw_loading = 1;
	fts_irq_disable(ts_data);
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_switch(ts_data->esdcheck, DISABLE);
#endif

	ret = fts_fwupg_get_fw_file(upg);
	if (ret >= 0) {
		fts_fwupg_init_ic_detail(upg);
		fts_fwupg_auto_upgrade(upg);
	}

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_switch(ts_data->esdcheck, ENABLE);
#endif
	fts_irq_enable(ts_data);
	ts_data->fw_loading = 0;
}

int fts_fwupg_init(struct fts_ts_data *ts_data)
{
	int i, j, ic_stype;
	struct upgrade_func *func = upgrade_func_list[0];
	int func_count = ARRAY_SIZE(upgrade_func_list);
	struct fts_upgrade *upg;

	if (!ts_data || !ts_data->ts_workqueue || !ts_data->dev)
		return -EINVAL;

	if (func_count == 0)
		return -ENODATA;

	upg = kzalloc(sizeof(*upg), GFP_KERNEL);
	if (!upg)
		return -ENOMEM;

	ic_stype = ts_data->ic_info.ids.type;
	if (func_count == 1) {
		upg->func = func;
	} else {
		for (i = 0; i < func_count; i++) {
			func = upgrade_func_list[i];
			for (j = 0; j < FTX_MAX_COMPATIBLE_TYPE; j++) {
				if (func->ctype[j] == 0) {
					break;
				} else if (func->ctype[j] == ic_stype) {
					upg->func = func;
					break;
				}
			}
		}
	}

	if (!upg->func) {
		kfree(upg);
		return -ENODATA;
	}

	upg->ts_data = ts_data;
	ts_data->upg = upg;

	INIT_WORK(&ts_data->fwupg_work, fts_fwupg_work);
	queue_work(ts_data->ts_workqueue, &ts_data->fwupg_work);

	return 0;
}

int fts_fwupg_exit(struct fts_ts_data *ts_data)
{
	if (ts_data && ts_data->ts_workqueue)
		cancel_work_sync(&ts_data->fwupg_work);

	if (ts_data && ts_data->upg) {
		if (ts_data->upg->fw_from_request && ts_data->upg->fw)
			kvfree(ts_data->upg->fw);

		kfree(ts_data->upg);
	}
	return 0;
}
