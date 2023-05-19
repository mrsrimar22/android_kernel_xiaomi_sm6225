/* SPDX-License-Identifier: GPL-2.0 */
/*
 * aw87xxx_acf_bin.h  aw87xxx pa module
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 */

#ifndef __AW87XXX_ACF_BIN_H__
#define __AW87XXX_ACF_BIN_H__

#include <linux/device.h>
#include <linux/types.h>

#define AW_PROJECT_NAME_MAX		24
#define AW_CUSTOMER_NAME_MAX		16
#define AW_CFG_VERSION_MAX		4

#define AW_REG_ADDR_BYTE		1
#define AW_REG_DATA_BYTE		1

#define AW_ACF_FILE_ID			0x0A15F908
#define AW_PROFILE_STR_MAX		32
#define AW_POWER_OFF_NAME_SUPPORT_COUNT	5

enum aw_cfg_hdr_version {
	AW_ACF_HDR_VER_0_0_0_1 = 0x00000001,
	AW_ACF_HDR_VER_1_0_0_0 = 0x01000000,
};

enum aw_acf_dde_type_id {
	AW_DEV_NONE_TYPE_ID = 0xFFFFFFFF,
	AW_DDE_DEV_TYPE_ID = 0x00000000,
	AW_DDE_SKT_TYPE_ID = 0x00000001,
	AW_DDE_DEV_DEFAULT_TYPE_ID = 0x00000002,
};

enum aw_raw_data_type_id {
	AW_BIN_TYPE_REG = 0x00000000,
	AW_BIN_TYPE_DSP,
	AW_BIN_TYPE_DSP_CFG,
	AW_BIN_TYPE_DSP_FW,
	AW_BIN_TYPE_HDR_REG,
	AW_BIN_TYPE_HDR_DSP_CFG,
	AW_BIN_TYPE_HDR_DSP_FW,
	AW_BIN_TYPE_MUTLBIN,
	AW_SKT_UI_PROJECT,
	AW_DSP_CFG,
	AW_MONITOR,
	AW_BIN_TYPE_MAX,
};

enum aw_dev_type {
	AW_DEV_TYPE_OK = 0,
	AW_DEV_TYPE_NONE = 1,
};

enum aw_profile_status {
	AW_PROFILE_WAIT = 0,
	AW_PROFILE_OK,
};

enum aw_acf_load_status {
	AW_ACF_WAIT = 0,
	AW_ACF_UPDATE,
};

enum aw_bin_dev_profile_id {
	AW_PROFILE_MUSIC = 0x0000,
	AW_PROFILE_VOICE,
	AW_PROFILE_VOIP,
	AW_PROFILE_RINGTONE,
	AW_PROFILE_RINGTONE_HS,
	AW_PROFILE_LOWPOWER,
	AW_PROFILE_BYPASS,
	AW_PROFILE_MMI,
	AW_PROFILE_FM,
	AW_PROFILE_NOTIFICATION,
	AW_PROFILE_RECEIVER,
	AW_PROFILE_OFF,
	AW_PROFILE_MAX,
};

#define FW_NAME_MAX			64
#define FW_MIN_SIZE			10

struct aw_data_container {
	u32 len;
	u8 *data;
};

struct aw_acf_hdr {
	int a_id;
	char project[AW_PROJECT_NAME_MAX];
	char custom[AW_CUSTOMER_NAME_MAX];
	u8 version[AW_CFG_VERSION_MAX];
	int author_id;
	int ddt_size;
	int dde_num;
	int ddt_offset;
	int hdr_version;
	int reserve[3];
};

struct aw_acf_dde {
	int type;
	char dev_name[AW_CUSTOMER_NAME_MAX];
	u16 dev_index;
	u16 dev_bus;
	u16 dev_addr;
	u16 dev_profile;
	int data_type;
	int data_size;
	int data_offset;
	int data_crc;
	int reserve[5];
};

struct aw_acf_dde_v_1_0_0_0 {
	u32 type;
	char dev_name[AW_CUSTOMER_NAME_MAX];
	u16 dev_index;
	u16 dev_bus;
	u16 dev_addr;
	u16 dev_profile;
	u32 data_type;
	u32 data_size;
	u32 data_offset;
	u32 data_crc;
	char dev_profile_str[AW_PROFILE_STR_MAX];
	u32 chip_id;
	u32 reserve[4];
};

struct aw_prof_desc {
	u32 prof_st;
	char *prof_name;
	char prof_name_buf[AW_PROFILE_STR_MAX];
	char dev_name[AW_CUSTOMER_NAME_MAX];
	struct aw_data_container data_container;
	bool data_owned;
};

struct aw_all_prof_info {
	struct aw_prof_desc prof_desc[AW_PROFILE_MAX];
};

struct aw_prof_info {
	int count;
	int status;
	int prof_type;
	char (*prof_name_list)[AW_PROFILE_STR_MAX];
	struct aw_prof_desc *prof_desc;
};

struct aw_device;

struct acf_bin_info {
	int load_count;
	int fw_size;
	u16 dev_index;
	char *fw_data;
	int product_cnt;
	const char * const *product_tab;
	struct aw_device *aw_dev;

	struct aw_acf_hdr acf_hdr;
	struct aw_prof_info prof_info;
};

void aw87xxx_acf_profile_free(struct device *dev,
			      struct acf_bin_info *acf_info);
int aw87xxx_acf_parse(struct device *dev,
		      struct acf_bin_info *acf_info);
struct aw_prof_desc *aw87xxx_acf_get_prof_desc_from_name(struct device *dev,
							 struct acf_bin_info *acf_info,
							 char *profile_name);
int aw87xxx_acf_get_prof_index_from_name(struct device *dev,
					 struct acf_bin_info *acf_info,
					 char *profile_name);
char *aw87xxx_acf_get_prof_name_from_index(struct device *dev,
					   struct acf_bin_info *acf_info,
					   int index);
int aw87xxx_acf_get_profile_count(struct device *dev,
				  struct acf_bin_info *acf_info);
char *aw87xxx_acf_get_prof_off_name(struct device *dev,
				    struct acf_bin_info *acf_info);
void aw87xxx_acf_init(struct aw_device *aw_dev,
		      struct acf_bin_info *acf_info,
		      int index);
int aw_parse_single_bin(struct device *dev,
			struct acf_bin_info *acf_info,
			struct aw_data_container aw_fw_data,
			int prof_index);

#endif
