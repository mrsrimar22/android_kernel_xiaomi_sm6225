/* SPDX-License-Identifier: GPL-2.0 */
/*
 * aw87xxx_bin_parse.h
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 */

#ifndef __AW87XXX_BIN_PARSE_H__
#define __AW87XXX_BIN_PARSE_H__

#include <linux/types.h>

#define BIN_NUM_MAX		100
#define HEADER_LEN		60

/*
 * header information
 */
enum aw_bin_header_version {
	HEADER_VERSION_1_0_0 = 0x01000000,
};

enum aw_data_type {
	DATA_TYPE_REGISTER = 0x00000000,
	DATA_TYPE_DSP_REG = 0x00000010,
	DATA_TYPE_DSP_CFG = 0x00000011,
	DATA_TYPE_SOC_REG = 0x00000020,
	DATA_TYPE_SOC_APP = 0x00000021,
	DATA_TYPE_MULTI_BINS = 0x00002000,
	DATA_TYPE_MONITOR_ANALOG = 0x00020000,
};

enum aw_data_version {
	DATA_VERSION_V1 = 0x00000001,	/* default little endian */
	DATA_VERSION_MAX,
};

struct bin_header_info {
	u32 header_len;
	u32 check_sum;
	u32 header_ver;
	u32 bin_data_type;
	u32 bin_data_ver;
	u32 bin_data_len;
	u32 ui_ver;
	u8 chip_type[8];
	u32 reg_byte_len;
	u32 data_byte_len;
	u32 device_addr;
	u32 valid_data_len;
	u32 valid_data_addr;
	u32 reg_num;
	u32 reg_data_byte_len;
	u32 download_addr;
	u32 app_version;
};

/*
 * function define
 */
struct aw_bin {
	u8 *p_addr;
	u32 all_bin_parse_num;
	u32 multi_bin_parse_num;
	u32 single_bin_parse_num;
	struct bin_header_info header_info[BIN_NUM_MAX];
	u32 len;
	u8 data[];
};

int aw87xxx_parsing_bin_file(struct aw_bin *bin);

#endif
