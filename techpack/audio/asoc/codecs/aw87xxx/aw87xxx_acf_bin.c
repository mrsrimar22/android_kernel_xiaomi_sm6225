// SPDX-License-Identifier: GPL-2.0
/*
 * aw87xxx_acf_bin.c
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 */

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <asoc/aw87xxx_api.h>

#include "aw87xxx_device.h"
#include "aw87xxx_acf_bin.h"
#include "aw87xxx_bin_parse.h"
#include "aw87xxx_log.h"
#include "aw87xxx_monitor.h"

static const char * const g_profile_name[] = {
	"Music", "Voice", "Voip", "Ringtone", "Ringtone_hs", "Lowpower",
	"Bypass", "Mmi", "Fm", "Notification", "Receiver", "Off"
};

static const char * const g_power_off_name[] = {
	"Off", "OFF", "off", "oFF", "power_down"
};

static const char *aw_get_prof_name(int profile)
{
	if (profile < 0 || profile >= AW_PROFILE_MAX)
		return "NULL";

	return g_profile_name[profile];
}

/*
 * acf check
 */
static int aw_crc8_check(const u8 *data, u32 data_size)
{
	u8 crc_value = 0x00;
	u8 pdatabuf;
	int i;

	while (data_size--) {
		pdatabuf = *data++;
		for (i = 0; i < 8; i++) {
			if ((crc_value ^ pdatabuf) & 0x01) {
				crc_value ^= 0x18;
				crc_value >>= 1;
				crc_value |= 0x80;
			} else {
				crc_value >>= 1;
			}
			pdatabuf >>= 1;
		}
	}

	return crc_value;
}

static int aw_check_file_id(struct device *dev,
			    const char *fw_data,
			    int file_id)
{
	int acf_file_id = get_unaligned_le32(fw_data);

	if (acf_file_id != file_id) {
		AW_DEV_LOGE(dev, "file id[0x%x] check failed", acf_file_id);
		return -ENFILE;
	}

	return 0;
}

static int aw_check_header_size(struct device *dev,
				const char *fw_data,
				size_t fw_size)
{
	if (fw_size < sizeof(struct aw_acf_hdr)) {
		AW_DEV_LOGE(dev, "acf size check failed, smaller than hdr");
		return -ENOEXEC;
	}

	return 0;
}

/*
 * acf data check (shared by V0.0.0.1 and V1.0.0.0).
 *
 * Both DDE table variants share the same leading layout through data_crc,
 * so the checks walk it through a common prefix window and stride by the
 * real element size.
 */
struct aw_acf_dde_hdr {
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
};

static int aw_check_ddt_size(struct device *dev,
			     const char *fw_data,
			     size_t dde_size)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)fw_data;
	size_t req_size = acf_hdr->dde_num * dde_size;

	if (acf_hdr->ddt_size != req_size) {
		AW_DEV_LOGE(dev, "acf ddt size check failed");
		return -EINVAL;
	}

	return 0;
}

static int aw_check_data_size(struct device *dev,
			      const char *fw_data,
			      size_t fw_size,
			      size_t dde_size,
			      bool check_odd)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)fw_data;
	size_t data_size = 0;
	int i;

	if (acf_hdr->ddt_offset + acf_hdr->ddt_size > fw_size) {
		AW_DEV_LOGE(dev, "ddt out of bounds");
		return -EINVAL;
	}

	for (i = 0; i < acf_hdr->dde_num; ++i) {
		const struct aw_acf_dde_hdr *dde =
			(void *)(fw_data + acf_hdr->ddt_offset + i * dde_size);

		if (check_odd && dde->data_size % 2)
			AW_DEV_LOGE(dev,
				    "dde[%d].data_size[%u] dev[%s] type[%u] check failed",
				    i, dde->data_size, dde->dev_name,
				    dde->data_type);

		data_size += dde->data_size;
	}

	if (fw_size != data_size + sizeof(*acf_hdr) + acf_hdr->ddt_size) {
		AW_DEV_LOGE(dev, "acf size check failed");
		return -EINVAL;
	}

	return 0;
}

static int aw_check_data_crc(struct device *dev,
			     const char *fw_data,
			     size_t dde_size)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)fw_data;
	int i;

	for (i = 0; i < acf_hdr->dde_num; ++i) {
		const struct aw_acf_dde_hdr *dde =
			(void *)(fw_data + acf_hdr->ddt_offset + i * dde_size);
		u32 crc_val = aw_crc8_check((const u8 *)(fw_data + dde->data_offset),
					    dde->data_size);

		if (crc_val != dde->data_crc) {
			AW_DEV_LOGE(dev, "acf dde_crc check failed");
			return -EINVAL;
		}
	}

	return 0;
}

static int aw_check_profile_id(struct device *dev,
			       const char *fw_data,
			       size_t dde_size)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)fw_data;
	int i;

	for (i = 0; i < acf_hdr->dde_num; ++i) {
		const struct aw_acf_dde_hdr *dde =
			(void *)(fw_data + acf_hdr->ddt_offset + i * dde_size);

		if (dde->data_type == AW_MONITOR)
			continue;
		if (dde->dev_profile > AW_PROFILE_MAX) {
			AW_DEV_LOGE(dev, "parse profile_id[%u] failed",
				    dde->dev_profile);
			return -EINVAL;
		}
	}

	return 0;
}

static int aw_check_data(struct device *dev,
			 const char *fw_data,
			 size_t fw_size,
			 size_t dde_size,
			 bool check_odd,
			 bool check_profile)
{
	int ret;

	ret = aw_check_file_id(dev, fw_data, AW_ACF_FILE_ID);
	if (ret < 0)
		return ret;

	ret = aw_check_ddt_size(dev, fw_data, dde_size);
	if (ret < 0)
		return ret;

	ret = aw_check_data_size(dev, fw_data, fw_size, dde_size, check_odd);
	if (ret < 0)
		return ret;

	ret = aw_check_data_crc(dev, fw_data, dde_size);
	if (ret < 0)
		return ret;

	if (check_profile) {
		ret = aw_check_profile_id(dev, fw_data, dde_size);
		if (ret < 0)
			return ret;
	}

	AW_DEV_LOGI(dev, "acf firmware check succeed");

	return 0;
}

/*
 * acf check API
 */
static int aw_check_acf_firmware(struct device *dev,
				 const char *fw_data,
				 size_t size)
{
	struct aw_acf_hdr *acf_hdr;
	int ret;

	if (!fw_data) {
		AW_DEV_LOGE(dev, "fw_data is NULL");
		return -ENODATA;
	}

	ret = aw_check_header_size(dev, fw_data, size);
	if (ret < 0)
		return ret;

	acf_hdr = (struct aw_acf_hdr *)fw_data;
	AW_DEV_LOGI(dev, "project name: [%s]", acf_hdr->project);
	AW_DEV_LOGI(dev, "custom name: [%s]", acf_hdr->custom);
	AW_DEV_LOGI(dev, "version name: [%s]", acf_hdr->version);
	AW_DEV_LOGI(dev, "author_id: [%d]", acf_hdr->author_id);

	switch (acf_hdr->hdr_version) {
	case AW_ACF_HDR_VER_0_0_0_1:
		return aw_check_data(dev, fw_data, size,
				     sizeof(struct aw_acf_dde),
				     false, true);
	case AW_ACF_HDR_VER_1_0_0_0:
		return aw_check_data(dev, fw_data, size,
				     sizeof(struct aw_acf_dde_v_1_0_0_0),
				     true, false);
	default:
		AW_DEV_LOGE(dev, "unsupported hdr_version[0x%x]",
			    acf_hdr->hdr_version);
		return -EINVAL;
	}
}

/*
 * acf parse
 */
static int aw_parse_raw_reg(struct device *dev,
			    u8 *data, u32 data_len,
			    struct aw_prof_desc *prof_desc)
{
	u8 *owned;

	AW_DEV_LOGD(dev, "data_size: %u enter", data_len);

	owned = kmemdup(data, data_len, GFP_KERNEL);
	if (!owned)
		return -ENOMEM;

	if (prof_desc->data_owned)
		kfree(prof_desc->data_container.data);

	prof_desc->data_container.data = owned;
	prof_desc->data_container.len = data_len;
	prof_desc->data_owned = true;
	prof_desc->prof_st = AW_PROFILE_OK;

	return 0;
}

static int aw_parse_reg_with_hdr(struct device *dev,
				 u8 *data, u32 data_len,
				 struct aw_prof_desc *prof_desc)
{
	struct aw_bin *aw_bin;
	u8 *owned;
	u32 vlen;
	int ret;

	AW_DEV_LOGD(dev, "data_size: %u enter", data_len);

	aw_bin = kzalloc(size_add(data_len, sizeof(*aw_bin)), GFP_KERNEL);
	if (!aw_bin)
		return -ENOMEM;

	aw_bin->len = data_len;
	memcpy(aw_bin->data, data, data_len);

	ret = aw87xxx_parsing_bin_file(aw_bin);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "parse bin failed");
		goto parse_bin_failed;
	}

	if (aw_bin->all_bin_parse_num != 1 ||
	    aw_bin->header_info[0].bin_data_type != DATA_TYPE_REGISTER) {
		AW_DEV_LOGE(dev, "bin num or type error");
		ret = -EINVAL;
		goto parse_bin_failed;
	}

	vlen = aw_bin->header_info[0].valid_data_len;
	owned = kmemdup(data + aw_bin->header_info[0].valid_data_addr,
			vlen, GFP_KERNEL);
	if (!owned) {
		ret = -ENOMEM;
		goto parse_bin_failed;
	}

	if (prof_desc->data_owned)
		kfree(prof_desc->data_container.data);

	prof_desc->data_container.data = owned;
	prof_desc->data_container.len = vlen;
	prof_desc->data_owned = true;
	prof_desc->prof_st = AW_PROFILE_OK;
	ret = 0;

parse_bin_failed:
	kfree(aw_bin);
	return ret;
}

static int aw_dev_prof_parse_multi_bin(struct device *dev,
				       u8 *data, u32 data_len,
				       struct aw_prof_desc *prof_desc)
{
	struct aw_bin *aw_bin;
	u8 *owned;
	u32 vlen;
	int i, ret;

	aw_bin = kzalloc(size_add(data_len, sizeof(*aw_bin)), GFP_KERNEL);
	if (!aw_bin)
		return -ENOMEM;

	aw_bin->len = data_len;
	memcpy(aw_bin->data, data, data_len);

	ret = aw87xxx_parsing_bin_file(aw_bin);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "parse bin failed");
		goto parse_bin_failed;
	}

	for (i = 0; i < aw_bin->all_bin_parse_num; i++) {
		if (aw_bin->header_info[i].bin_data_type == DATA_TYPE_REGISTER) {
			vlen = aw_bin->header_info[i].valid_data_len;
			owned = kmemdup(data + aw_bin->header_info[i].valid_data_addr,
					vlen, GFP_KERNEL);
			if (!owned) {
				ret = -ENOMEM;
				goto parse_bin_failed;
			}

			if (prof_desc->data_owned)
				kfree(prof_desc->data_container.data);

			prof_desc->data_container.len = vlen;
			prof_desc->data_container.data = owned;
			prof_desc->data_owned = true;
			break;
		}
	}

	if (i == aw_bin->all_bin_parse_num) {
		AW_DEV_LOGE(dev, "expected data type not found");
		ret = -EINVAL;
		goto parse_bin_failed;
	}

	prof_desc->prof_st = AW_PROFILE_OK;
	ret = 0;

parse_bin_failed:
	kfree(aw_bin);
	return ret;
}

static int aw_parse_monitor_config(struct device *dev,
				   const char *monitor_data,
				   u32 data_len)
{
	int ret;

	if (!monitor_data || data_len == 0) {
		AW_DEV_LOGE(dev, "no data to parse");
		return -EBFONT;
	}

	ret = aw87xxx_monitor_bin_parse(dev,
					(const u8 *)monitor_data,
					data_len);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "monitor_config parse failed");
		return ret;
	}

	AW_DEV_LOGI(dev, "monitor_bin parse succeed");

	return 0;
}

static int aw_check_prof_str_is_off(const char *profile_name)
{
	int i;

	for (i = 0; i < AW_POWER_OFF_NAME_SUPPORT_COUNT; i++) {
		if (strstr(profile_name, g_power_off_name[i]))
			return 0;
	}

	return -EINVAL;
}

/*
 * V0.0.0.1 version acf parse
 */
static int aw_check_product_name_v_0_0_0_1(struct device *dev,
					   struct acf_bin_info *acf_info,
					   struct aw_acf_dde *prof_hdr)
{
	int i;

	for (i = 0; i < acf_info->product_cnt; i++) {
		if (!strncasecmp(acf_info->product_tab[i], prof_hdr->dev_name,
				 AW_CUSTOMER_NAME_MAX)) {
			AW_DEV_LOGD(dev, "bin_dev_name: %s", prof_hdr->dev_name);
			return 0;
		}
	}

	return -ENXIO;
}

static int aw_check_data_type_is_monitor_v_0_0_0_1(struct device *dev,
						   struct aw_acf_dde *prof_hdr)
{
	if (prof_hdr->data_type == AW_MONITOR) {
		AW_DEV_LOGD(dev, "bin data is monitor");
		return 0;
	}

	return -ENXIO;
}

static int aw_parse_data_by_sec_type_v_0_0_0_1(struct device *dev,
					       struct acf_bin_info *acf_info,
					       struct aw_acf_dde *prof_hdr,
					       struct aw_prof_desc *prof_desc)
{
	char *cfg_data = acf_info->fw_data + prof_hdr->data_offset;
	int ret = -1;

	switch (prof_hdr->data_type) {
	case AW_BIN_TYPE_REG:
		strscpy(prof_desc->dev_name, prof_hdr->dev_name,
			sizeof(prof_desc->dev_name));
		prof_desc->prof_name = (char *)aw_get_prof_name(prof_hdr->dev_profile);
		AW_DEV_LOGD(dev, "parse reg type data enter, profile=%s",
			    prof_desc->prof_name);
		ret = aw_parse_raw_reg(dev, (u8 *)cfg_data,
				       prof_hdr->data_size,
				       prof_desc);
		break;
	case AW_BIN_TYPE_HDR_REG:
		strscpy(prof_desc->dev_name, prof_hdr->dev_name,
			sizeof(prof_desc->dev_name));
		prof_desc->prof_name = (char *)aw_get_prof_name(prof_hdr->dev_profile);
		AW_DEV_LOGD(dev, "parse hdr_reg type data enter, profile=%s",
			    prof_desc->prof_name);
		ret = aw_parse_reg_with_hdr(dev, (u8 *)cfg_data,
					    prof_hdr->data_size,
					    prof_desc);
		break;
	case AW_BIN_TYPE_MUTLBIN:
		strscpy(prof_desc->dev_name, prof_hdr->dev_name,
			sizeof(prof_desc->dev_name));
		prof_desc->prof_name = (char *)aw_get_prof_name(prof_hdr->dev_profile);
		AW_DEV_LOGD(dev, "parse mutil type data enter, profile=%s",
			    prof_desc->prof_name);
		ret = aw_dev_prof_parse_multi_bin(dev, (u8 *)cfg_data,
						  prof_hdr->data_size,
						  prof_desc);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int aw_parse_dde_entry_v_0_0_0_1(struct device *dev,
					struct acf_bin_info *acf_info,
					struct aw_acf_dde *dde,
					struct aw_all_prof_info *all_prof_info)
{
	struct aw_prof_desc *prof_desc;
	char *cfg_data;
	int ret;

	ret = aw_check_product_name_v_0_0_0_1(dev, acf_info, dde);
	if (ret < 0)
		return 0;

	ret = aw_check_data_type_is_monitor_v_0_0_0_1(dev, dde);
	if (!ret) {
		cfg_data = acf_info->fw_data + dde->data_offset;
		ret = aw_parse_monitor_config(dev, cfg_data, dde->data_size);
		if (ret < 0)
			return ret;
		return 0;
	}

	prof_desc = &all_prof_info->prof_desc[dde->dev_profile];
	ret = aw_parse_data_by_sec_type_v_0_0_0_1(dev, acf_info, dde, prof_desc);
	if (ret < 0)
		return ret;

	return 1;
}

static int aw_parse_dev_type_v_0_0_0_1(struct device *dev,
				       struct acf_bin_info *acf_info,
				       struct aw_all_prof_info *all_prof_info)
{
	struct aw_acf_dde *acf_dde;
	int i, ret;
	int sec_num = 0;

	AW_DEV_LOGD(dev, "enter");

	acf_dde = (struct aw_acf_dde *)(acf_info->fw_data +
					acf_info->acf_hdr.ddt_offset);

	for (i = 0; i < acf_info->acf_hdr.dde_num; i++) {
		if (acf_info->aw_dev->i2c_bus == acf_dde[i].dev_bus &&
		    acf_info->aw_dev->i2c_addr == acf_dde[i].dev_addr &&
		    acf_dde[i].type == AW_DDE_DEV_TYPE_ID) {
			ret = aw_parse_dde_entry_v_0_0_0_1(dev, acf_info,
							   &acf_dde[i],
							   all_prof_info);
			if (ret < 0) {
				AW_DEV_LOGE(dev, "parse dev type data failed");
				return ret;
			}
			sec_num += ret;
		}
	}

	if (!sec_num) {
		AW_DEV_LOGD(dev, "dev type num is 0, use default");
		return AW_DEV_TYPE_NONE;
	}

	return AW_DEV_TYPE_OK;
}

static int aw_parse_default_type_v_0_0_0_1(struct device *dev,
					   struct acf_bin_info *acf_info,
					   struct aw_all_prof_info *all_prof_info)
{
	struct aw_acf_dde *acf_dde;
	int i, ret;
	int sec_num = 0;

	AW_DEV_LOGD(dev, "enter");

	acf_dde = (struct aw_acf_dde *)(acf_info->fw_data +
					acf_info->acf_hdr.ddt_offset);

	for (i = 0; i < acf_info->acf_hdr.dde_num; i++) {
		if (acf_info->dev_index == acf_dde[i].dev_index &&
		    acf_dde[i].type == AW_DDE_DEV_DEFAULT_TYPE_ID) {
			ret = aw_parse_dde_entry_v_0_0_0_1(dev, acf_info,
							   &acf_dde[i],
							   all_prof_info);
			if (ret < 0) {
				AW_DEV_LOGE(dev, "parse default data failed");
				return ret;
			}
			sec_num += ret;
		}
	}

	if (!sec_num) {
		AW_DEV_LOGE(dev, "dev default type failed, num[%d]", sec_num);
		return -EINVAL;
	}

	return 0;
}

static int aw_get_prof_count_v_0_0_0_1(struct device *dev,
				       struct acf_bin_info *acf_info,
				       struct aw_all_prof_info *all_prof_info)
{
	struct aw_prof_desc *prof_desc = all_prof_info->prof_desc;
	int i;
	int prof_count = 0;

	for (i = 0; i < AW_PROFILE_MAX; i++) {
		if (prof_desc[i].prof_st == AW_PROFILE_OK) {
			prof_count++;
		} else if (i == AW_PROFILE_OFF) {
			prof_count++;
			AW_DEV_LOGI(dev, "not found profile[Off], default");
		}
	}

	AW_DEV_LOGI(dev, "get profile count=[%d]", prof_count);
	return prof_count;
}

static int aw_set_prof_off_info_v_0_0_0_1(struct device *dev,
					  struct acf_bin_info *acf_info,
					  struct aw_all_prof_info *all_prof_info,
					  int index)
{
	struct aw_prof_desc *prof_desc = all_prof_info->prof_desc;
	struct aw_prof_info *prof_info = &acf_info->prof_info;

	if (index >= prof_info->count) {
		AW_DEV_LOGE(dev, "index[%d] is out of table, count[%d]",
			    index, prof_info->count);
		return -EINVAL;
	}

	if (prof_desc[AW_PROFILE_OFF].prof_st == AW_PROFILE_OK) {
		prof_info->prof_desc[index] = prof_desc[AW_PROFILE_OFF];
		AW_DEV_LOGI(dev, "product=[%s], profile=[%s]",
			    prof_info->prof_desc[index].dev_name,
			    aw_get_prof_name(AW_PROFILE_OFF));
	} else {
		memset(&prof_info->prof_desc[index].data_container, 0,
		       sizeof(struct aw_data_container));
		prof_info->prof_desc[index].prof_st = AW_PROFILE_WAIT;
		prof_info->prof_desc[index].prof_name =
			(char *)aw_get_prof_name(AW_PROFILE_OFF);
		AW_DEV_LOGI(dev, "set default power_off with no data");
	}

	return 0;
}

static int aw_get_vaild_prof_v_0_0_0_1(struct device *dev,
				       struct acf_bin_info *acf_info,
				       struct aw_all_prof_info *all_prof_info)
{
	struct aw_prof_desc *prof_desc = all_prof_info->prof_desc;
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int i, ret;
	int index = 0;

	prof_info->count = 0;
	ret = aw_get_prof_count_v_0_0_0_1(dev, acf_info, all_prof_info);
	if (ret < 0)
		return ret;

	prof_info->count = ret;
	prof_info->prof_desc = kcalloc(prof_info->count,
				       sizeof(*prof_info->prof_desc),
				       GFP_KERNEL);
	if (!prof_info->prof_desc)
		return -ENOMEM;

	for (i = 0; i < AW_PROFILE_MAX; i++) {
		if (i != AW_PROFILE_OFF &&
		    prof_desc[i].prof_st == AW_PROFILE_OK) {
			if (index >= prof_info->count) {
				AW_DEV_LOGE(dev, "index[%d] overflow count[%d]",
					    index, prof_info->count);
				return -ENOMEM;
			}
			prof_info->prof_desc[index] = prof_desc[i];
			AW_DEV_LOGI(dev, "product=[%s], profile=[%s]",
				    prof_info->prof_desc[index].dev_name,
				    aw_get_prof_name(i));
			index++;
		}
	}

	ret = aw_set_prof_off_info_v_0_0_0_1(dev,
					     acf_info,
					     all_prof_info,
					     index);
	if (ret < 0)
		return ret;

	AW_DEV_LOGD(dev, "get valid profile succeed");
	return 0;
}

static int aw_set_prof_name_list_v_0_0_0_1(struct device *dev,
					   struct acf_bin_info *acf_info)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int i;
	int count = prof_info->count;

	prof_info->prof_name_list = kcalloc(count,
					    AW_PROFILE_STR_MAX,
					    GFP_KERNEL);
	if (!prof_info->prof_name_list)
		return -ENOMEM;

	for (i = 0; i < count; ++i) {
		strscpy(prof_info->prof_name_list[i],
			prof_info->prof_desc[i].prof_name,
			AW_PROFILE_STR_MAX);
		AW_DEV_LOGI(dev, "index=[%d], profile_name=[%s]", i,
			    prof_info->prof_name_list[i]);
	}

	return 0;
}

static int aw_parse_acf_v_0_0_0_1(struct device *dev,
				  struct acf_bin_info *acf_info)
{
	struct aw_all_prof_info all_prof_info;
	int ret;

	AW_DEV_LOGD(dev, "enter");
	acf_info->prof_info.status = AW_ACF_WAIT;

	memset(&all_prof_info, 0, sizeof(all_prof_info));

	ret = aw_parse_dev_type_v_0_0_0_1(dev, acf_info, &all_prof_info);
	if (ret < 0) {
		return ret;
	} else if (ret == AW_DEV_TYPE_NONE) {
		AW_DEV_LOGD(dev, "dev type num 0, parse default dev type");
		ret = aw_parse_default_type_v_0_0_0_1(dev,
						      acf_info,
						      &all_prof_info);
		if (ret < 0)
			return ret;
	}

	ret = aw_get_vaild_prof_v_0_0_0_1(dev, acf_info, &all_prof_info);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "hdr_version[0x%x] parse failed",
			    acf_info->acf_hdr.hdr_version);
		return ret;
	}

	ret = aw_set_prof_name_list_v_0_0_0_1(dev, acf_info);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "creat prof_id_and_name_list failed");
		return ret;
	}

	acf_info->prof_info.status = AW_ACF_UPDATE;
	AW_DEV_LOGI(dev, "acf parse success");

	return 0;
}

/*
 * V1.0.0.0 version acf parse
 */
static int aw_check_product_name_v_1_0_0_0(struct device *dev,
					   struct acf_bin_info *acf_info,
					   struct aw_acf_dde_v_1_0_0_0 *prof_hdr)
{
	int i;

	for (i = 0; i < acf_info->product_cnt; i++) {
		if (!strncasecmp(acf_info->product_tab[i], prof_hdr->dev_name,
				 AW_CUSTOMER_NAME_MAX)) {
			AW_DEV_LOGI(dev, "bin_dev_name: %s", prof_hdr->dev_name);
			return 0;
		}
	}

	return -ENXIO;
}

static int aw_get_dde_type_info_v_1_0_0_0(struct device *dev,
					  struct acf_bin_info *acf_info)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)acf_info->fw_data;
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	struct aw_acf_dde_v_1_0_0_0 *acf_dde;
	int dev_num = 0;
	int default_num = 0;
	int i;

	acf_dde = (struct aw_acf_dde_v_1_0_0_0 *)(acf_info->fw_data +
						  acf_hdr->ddt_offset);

	prof_info->prof_type = AW_DEV_NONE_TYPE_ID;
	for (i = 0; i < acf_hdr->dde_num; i++) {
		if (acf_dde[i].type == AW_DDE_DEV_TYPE_ID)
			dev_num++;
		if (acf_dde[i].type == AW_DDE_DEV_DEFAULT_TYPE_ID)
			default_num++;
	}

	if (!dev_num && !default_num) {
		AW_DEV_LOGE(dev, "can't find scene");
		return -EINVAL;
	}

	if (dev_num)
		prof_info->prof_type = AW_DDE_DEV_TYPE_ID;
	else if (default_num)
		prof_info->prof_type = AW_DDE_DEV_DEFAULT_TYPE_ID;

	return 0;
}

static int aw_parse_get_dev_type_prof_count_v_1_0_0_0(struct device *dev,
						      struct acf_bin_info *acf_info)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)acf_info->fw_data;
	struct aw_acf_dde_v_1_0_0_0 *acf_dde;
	int i, ret;
	int found_off = 0;
	int count = acf_info->prof_info.count;

	acf_dde = (struct aw_acf_dde_v_1_0_0_0 *)(acf_info->fw_data +
						  acf_hdr->ddt_offset);

	for (i = 0; i < acf_hdr->dde_num; ++i) {
		if ((acf_dde[i].data_type == AW_BIN_TYPE_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_HDR_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_MUTLBIN) &&
		    acf_info->aw_dev->i2c_bus == acf_dde[i].dev_bus &&
		    acf_info->aw_dev->i2c_addr == acf_dde[i].dev_addr &&
		    acf_info->aw_dev->chipid == acf_dde[i].chip_id) {
			ret = aw_check_product_name_v_1_0_0_0(dev,
							      acf_info,
							      &acf_dde[i]);
			if (ret < 0)
				continue;

			ret = aw_check_prof_str_is_off(acf_dde[i].dev_profile_str);
			if (!ret)
				found_off = AW_PROFILE_OK;

			count++;
		}
	}

	if (!count) {
		AW_DEV_LOGE(dev, "can't find profile");
		return -EINVAL;
	}

	if (!found_off) {
		count++;
		AW_DEV_LOGD(dev, "set no config power off profile in count");
	}

	acf_info->prof_info.count = count;
	AW_DEV_LOGI(dev, "dev_type profile count is %d", count);

	return 0;
}

static int aw_parse_get_default_type_prof_count_v_1_0_0_0(struct device *dev,
							  struct acf_bin_info *acf_info)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)acf_info->fw_data;
	struct aw_acf_dde_v_1_0_0_0 *acf_dde;
	int i, ret;
	int found_off = 0;
	int count = acf_info->prof_info.count;

	acf_dde = (struct aw_acf_dde_v_1_0_0_0 *)(acf_info->fw_data +
						  acf_hdr->ddt_offset);

	for (i = 0; i < acf_hdr->dde_num; ++i) {
		if ((acf_dde[i].data_type == AW_BIN_TYPE_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_HDR_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_MUTLBIN) &&
		    acf_info->dev_index == acf_dde[i].dev_index &&
		    acf_info->aw_dev->chipid == acf_dde[i].chip_id) {
			ret = aw_check_product_name_v_1_0_0_0(dev,
							      acf_info,
							      &acf_dde[i]);
			if (ret < 0)
				continue;

			ret = aw_check_prof_str_is_off(acf_dde[i].dev_profile_str);
			if (!ret)
				found_off = AW_PROFILE_OK;

			count++;
		}
	}

	if (!count) {
		AW_DEV_LOGE(dev, "can't find profile");
		return -EINVAL;
	}

	if (!found_off) {
		count++;
		AW_DEV_LOGD(dev, "set no config power off profile in count");
	}

	acf_info->prof_info.count = count;
	AW_DEV_LOGI(dev, "default_type profile count is %d", count);

	return 0;
}

static int aw_parse_get_profile_count_v_1_0_0_0(struct device *dev,
						struct acf_bin_info *acf_info)
{
	int ret;

	ret = aw_get_dde_type_info_v_1_0_0_0(dev, acf_info);
	if (ret < 0)
		return ret;

	if (acf_info->prof_info.prof_type == AW_DDE_DEV_TYPE_ID) {
		ret = aw_parse_get_dev_type_prof_count_v_1_0_0_0(dev, acf_info);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "parse dev_type count failed");
			return ret;
		}
	} else if (acf_info->prof_info.prof_type == AW_DDE_DEV_DEFAULT_TYPE_ID) {
		ret = aw_parse_get_default_type_prof_count_v_1_0_0_0(dev, acf_info);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "parse default_type count failed");
			return ret;
		}
	} else {
		AW_DEV_LOGE(dev, "unsupport prof_type[0x%x]",
			    acf_info->prof_info.prof_type);
		return -EINVAL;
	}

	AW_DEV_LOGI(dev, "profile count is %d", acf_info->prof_info.count);
	return 0;
}

static int aw_parse_dev_type_prof_name_v_1_0_0_0(struct device *dev,
						 struct acf_bin_info *acf_info)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)acf_info->fw_data;
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	struct aw_acf_dde_v_1_0_0_0 *acf_dde;
	int i, ret;
	int list_idx = 0;

	acf_dde = (struct aw_acf_dde_v_1_0_0_0 *)(acf_info->fw_data +
						  acf_hdr->ddt_offset);

	for (i = 0; i < acf_hdr->dde_num; ++i) {
		if ((acf_dde[i].data_type == AW_BIN_TYPE_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_HDR_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_MUTLBIN) &&
		    acf_info->aw_dev->i2c_bus == acf_dde[i].dev_bus &&
		    acf_info->aw_dev->i2c_addr == acf_dde[i].dev_addr &&
		    acf_info->aw_dev->chipid == acf_dde[i].chip_id) {
			if (list_idx >= prof_info->count) {
				AW_DEV_LOGE(dev, "redundant profile [%s]",
					    acf_dde[i].dev_profile_str);
				return -EINVAL;
			}

			ret = aw_check_product_name_v_1_0_0_0(dev,
							      acf_info,
							      &acf_dde[i]);
			if (ret < 0)
				continue;

			strscpy(prof_info->prof_name_list[list_idx],
				acf_dde[i].dev_profile_str, AW_PROFILE_STR_MAX);
			AW_DEV_LOGI(dev, "profile_name=[%s]",
				    prof_info->prof_name_list[list_idx]);
			list_idx++;
		}
	}

	return 0;
}

static int aw_parse_default_type_prof_name_v_1_0_0_0(struct device *dev,
						     struct acf_bin_info *acf_info)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)acf_info->fw_data;
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	struct aw_acf_dde_v_1_0_0_0 *acf_dde;
	int i, ret;
	int list_idx = 0;

	acf_dde = (struct aw_acf_dde_v_1_0_0_0 *)(acf_info->fw_data +
						  acf_hdr->ddt_offset);

	for (i = 0; i < acf_hdr->dde_num; ++i) {
		if ((acf_dde[i].data_type == AW_BIN_TYPE_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_HDR_REG ||
		     acf_dde[i].data_type == AW_BIN_TYPE_MUTLBIN) &&
		    acf_info->dev_index == acf_dde[i].dev_index &&
		    acf_info->aw_dev->chipid == acf_dde[i].chip_id) {
			if (list_idx >= prof_info->count) {
				AW_DEV_LOGE(dev, "redundant profile[%s]",
					    acf_dde[i].dev_profile_str);
				return -EINVAL;
			}

			ret = aw_check_product_name_v_1_0_0_0(dev,
							      acf_info,
							      &acf_dde[i]);
			if (ret < 0)
				continue;

			strscpy(prof_info->prof_name_list[list_idx],
				acf_dde[i].dev_profile_str, AW_PROFILE_STR_MAX);
			AW_DEV_LOGI(dev, "profile_name=[%s]",
				    prof_info->prof_name_list[list_idx]);
			list_idx++;
		}
	}

	return 0;
}

static int aw_parse_prof_name_v_1_0_0_0(struct device *dev,
					struct acf_bin_info *acf_info)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int count = prof_info->count;
	int ret;

	prof_info->prof_name_list = kcalloc(count,
					    AW_PROFILE_STR_MAX,
					    GFP_KERNEL);
	if (!prof_info->prof_name_list)
		return -ENOMEM;

	if (prof_info->prof_type == AW_DDE_DEV_TYPE_ID) {
		ret = aw_parse_dev_type_prof_name_v_1_0_0_0(dev, acf_info);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "parse dev_type profile name failed");
			return ret;
		}
	} else if (prof_info->prof_type == AW_DDE_DEV_DEFAULT_TYPE_ID) {
		ret = aw_parse_default_type_prof_name_v_1_0_0_0(dev, acf_info);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "parse default_type name failed");
			return ret;
		}
	} else {
		AW_DEV_LOGE(dev, "unsupport prof_type[0x%x]",
			    prof_info->prof_type);
		return -EINVAL;
	}

	AW_DEV_LOGI(dev, "profile name parse succeed");
	return 0;
}

static int aw_search_prof_index_from_list_v_1_0_0_0(struct device *dev,
						    struct acf_bin_info *acf_info,
						    struct aw_prof_desc **prof_desc,
						    struct aw_acf_dde_v_1_0_0_0 *prof_hdr)
{
	char (*list)[AW_PROFILE_STR_MAX] = acf_info->prof_info.prof_name_list;
	int count = acf_info->prof_info.count;
	int i;

	if (!list) {
		AW_DEV_LOGE(dev, "prof_name_list pointer is empty");
		return -ENOMEM;
	}

	for (i = 0; i < count; i++) {
		if (!strncmp(list[i], prof_hdr->dev_profile_str, AW_PROFILE_STR_MAX)) {
			*prof_desc = &acf_info->prof_info.prof_desc[i];
			return 0;
		}
	}

	AW_DEV_LOGE(dev, "not find prof_id and prof_name in list");
	return -EINVAL;
}

static int aw_parse_data_by_sec_type_v_1_0_0_0(struct device *dev,
					       struct acf_bin_info *acf_info,
					       struct aw_acf_dde_v_1_0_0_0 *prof_hdr)
{
	char *cfg_data = acf_info->fw_data + prof_hdr->data_offset;
	struct aw_prof_desc *prof_desc = NULL;
	int ret;

	ret = aw_search_prof_index_from_list_v_1_0_0_0(dev,
						       acf_info,
						       &prof_desc,
						       prof_hdr);
	if (ret < 0)
		return ret;

	switch (prof_hdr->data_type) {
	case AW_BIN_TYPE_REG:
		strscpy(prof_desc->dev_name, prof_hdr->dev_name,
			sizeof(prof_desc->dev_name));
		strscpy(prof_desc->prof_name_buf, prof_hdr->dev_profile_str,
			AW_PROFILE_STR_MAX);
		prof_desc->prof_name = prof_desc->prof_name_buf;

		AW_DEV_LOGI(dev, "parse reg type, prod=[%s], prof=[%s]",
			    prof_hdr->dev_name, prof_desc->prof_name);
		ret = aw_parse_raw_reg(dev, (u8 *)cfg_data,
				       prof_hdr->data_size,
				       prof_desc);
		break;
	case AW_BIN_TYPE_HDR_REG:
		strscpy(prof_desc->dev_name, prof_hdr->dev_name,
			sizeof(prof_desc->dev_name));
		strscpy(prof_desc->prof_name_buf, prof_hdr->dev_profile_str,
			AW_PROFILE_STR_MAX);
		prof_desc->prof_name = prof_desc->prof_name_buf;

		AW_DEV_LOGI(dev, "parse hdr_reg type, prod=[%s], prof=[%s]",
			    prof_hdr->dev_name, prof_desc->prof_name);
		ret = aw_parse_reg_with_hdr(dev, (u8 *)cfg_data,
					    prof_hdr->data_size,
					    prof_desc);
		break;
	case AW_BIN_TYPE_MUTLBIN:
		strscpy(prof_desc->dev_name, prof_hdr->dev_name,
			sizeof(prof_desc->dev_name));
		strscpy(prof_desc->prof_name_buf, prof_hdr->dev_profile_str,
			AW_PROFILE_STR_MAX);
		prof_desc->prof_name = prof_desc->prof_name_buf;

		AW_DEV_LOGI(dev, "parse multi type, prod=[%s], prof=[%s]",
			    prof_hdr->dev_name, prof_desc->prof_name);
		ret = aw_dev_prof_parse_multi_bin(dev, (u8 *)cfg_data,
						  prof_hdr->data_size,
						  prof_desc);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int aw_parse_dev_type_v_1_0_0_0(struct device *dev,
				       struct acf_bin_info *acf_info)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)acf_info->fw_data;
	struct aw_acf_dde_v_1_0_0_0 *acf_dde;
	int parse_prof_count = 0;
	char *cfg_data;
	int i, ret;

	AW_DEV_LOGD(dev, "enter");

	acf_dde = (struct aw_acf_dde_v_1_0_0_0 *)(acf_info->fw_data +
						  acf_hdr->ddt_offset);

	for (i = 0; i < acf_hdr->dde_num; i++) {
		if (acf_dde[i].type == AW_DDE_DEV_TYPE_ID &&
		    acf_info->aw_dev->i2c_bus == acf_dde[i].dev_bus &&
		    acf_info->aw_dev->i2c_addr == acf_dde[i].dev_addr &&
		    acf_info->aw_dev->chipid == acf_dde[i].chip_id) {
			ret = aw_check_product_name_v_1_0_0_0(dev,
							      acf_info,
							      &acf_dde[i]);
			if (ret < 0)
				continue;

			if (acf_dde[i].data_type == AW_MONITOR) {
				cfg_data = acf_info->fw_data + acf_dde[i].data_offset;
				AW_DEV_LOGD(dev, "parse monitor type enter");
				ret = aw_parse_monitor_config(dev,
							      cfg_data,
							      acf_dde[i].data_size);
				if (ret < 0)
					return ret;
			} else {
				ret = aw_parse_data_by_sec_type_v_1_0_0_0(dev,
									  acf_info,
									  &acf_dde[i]);
				if (ret < 0)
					AW_DEV_LOGE(dev, "dev parse failed");
				else
					parse_prof_count++;
			}
		}
	}

	if (!parse_prof_count) {
		AW_DEV_LOGE(dev, "dev type parse failed");
		return -EINVAL;
	}

	return AW_DEV_TYPE_OK;
}

static int aw_parse_default_type_v_1_0_0_0(struct device *dev,
					   struct acf_bin_info *acf_info)
{
	struct aw_acf_hdr *acf_hdr = (struct aw_acf_hdr *)acf_info->fw_data;
	struct aw_acf_dde_v_1_0_0_0 *acf_dde;
	int parse_prof_count = 0;
	char *cfg_data;
	int i, ret;

	AW_DEV_LOGD(dev, "enter");

	acf_dde = (struct aw_acf_dde_v_1_0_0_0 *)(acf_info->fw_data +
						  acf_hdr->ddt_offset);

	for (i = 0; i < acf_hdr->dde_num; i++) {
		if (acf_dde[i].type == AW_DDE_DEV_DEFAULT_TYPE_ID &&
		    acf_info->dev_index == acf_dde[i].dev_index &&
		    acf_info->aw_dev->chipid == acf_dde[i].chip_id) {
			ret = aw_check_product_name_v_1_0_0_0(dev,
							      acf_info,
							      &acf_dde[i]);
			if (ret < 0)
				continue;

			if (acf_dde[i].data_type == AW_MONITOR) {
				cfg_data = acf_info->fw_data + acf_dde[i].data_offset;
				AW_DEV_LOGD(dev, "parse monitor type enter");
				ret = aw_parse_monitor_config(dev,
							      cfg_data,
							      acf_dde[i].data_size);
				if (ret < 0)
					return ret;
			} else {
				ret = aw_parse_data_by_sec_type_v_1_0_0_0(dev,
									  acf_info,
									  &acf_dde[i]);
				if (ret < 0)
					AW_DEV_LOGE(dev, "default parse fail");
				else
					parse_prof_count++;
			}
		}
	}

	if (!parse_prof_count) {
		AW_DEV_LOGE(dev, "default type parse failed");
		return -EINVAL;
	}

	return AW_DEV_TYPE_OK;
}

static int aw_parse_by_hdr_v_1_0_0_0(struct device *dev,
				     struct acf_bin_info *acf_info)
{
	if (acf_info->prof_info.prof_type == AW_DDE_DEV_TYPE_ID)
		return aw_parse_dev_type_v_1_0_0_0(dev, acf_info);
	else if (acf_info->prof_info.prof_type == AW_DDE_DEV_DEFAULT_TYPE_ID)
		return aw_parse_default_type_v_1_0_0_0(dev, acf_info);

	return 0;
}

static int aw_set_prof_off_info_v_1_0_0_0(struct device *dev,
					  struct acf_bin_info *acf_info)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int i, ret;

	for (i = 0; i < prof_info->count; ++i) {
		if (!prof_info->prof_desc[i].prof_st) {
			strscpy(prof_info->prof_name_list[i],
				g_power_off_name[0],
				AW_PROFILE_STR_MAX);
			strscpy(prof_info->prof_desc[i].prof_name_buf,
				prof_info->prof_name_list[i],
				AW_PROFILE_STR_MAX);
			prof_info->prof_desc[i].prof_name =
					prof_info->prof_desc[i].prof_name_buf;
			prof_info->prof_desc[i].prof_st = AW_PROFILE_WAIT;
			memset(&prof_info->prof_desc[i].data_container, 0,
			       sizeof(prof_info->prof_desc[i].data_container));
			return 0;
		}

		ret = aw_check_prof_str_is_off(prof_info->prof_name_list[i]);
		if (!ret) {
			AW_DEV_LOGD(dev, "found profile off, data_len=[%u]",
				    prof_info->prof_desc[i].data_container.len);
			return 0;
		}
	}

	AW_DEV_LOGE(dev, "index[%d] is out of table, count[%d]",
		    i, prof_info->count);
	return -EINVAL;
}

static int aw_parse_acf_v_1_0_0_0(struct device *dev,
				  struct acf_bin_info *acf_info)

{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int ret;

	ret = aw_parse_get_profile_count_v_1_0_0_0(dev, acf_info);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "get profile count failed");
		return ret;
	}

	ret = aw_parse_prof_name_v_1_0_0_0(dev, acf_info);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "get profile name failed");
		return ret;
	}

	prof_info->prof_desc = kcalloc(prof_info->count,
				       sizeof(*prof_info->prof_desc),
				       GFP_KERNEL);
	if (!prof_info->prof_desc)
		return -ENOMEM;

	ret = aw_parse_by_hdr_v_1_0_0_0(dev, acf_info);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "parse data failed");
		return ret;
	}

	ret = aw_set_prof_off_info_v_1_0_0_0(dev, acf_info);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "set profile off info failed");
		return ret;
	}

	prof_info->status = AW_ACF_UPDATE;
	AW_DEV_LOGI(dev, "acf parse succeed");

	return 0;
}

/*
 * acf parse API
 */
void aw87xxx_acf_profile_free(struct device *dev,
			      struct acf_bin_info *acf_info)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int i;
	int count = prof_info->count;

	prof_info->count = 0;
	prof_info->status = AW_ACF_WAIT;
	memset(&acf_info->acf_hdr, 0, sizeof(acf_info->acf_hdr));

	if (prof_info->prof_desc) {
		for (i = 0; i < count; i++) {
			if (prof_info->prof_desc[i].data_owned) {
				kfree(prof_info->prof_desc[i].data_container.data);
				prof_info->prof_desc[i].data_owned = false;
			}
		}
	}

	kfree(prof_info->prof_desc);
	kfree(prof_info->prof_name_list);
	kfree(acf_info->fw_data);
}

int aw_parse_single_bin(struct device *dev,
			struct acf_bin_info *acf_info,
			struct aw_data_container aw_fw_data,
			int prof_index)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	u32 header_version = 0;
	struct aw_bin *aw_bin;
	int i, ret;
	u8 *owned;

	AW_DEV_LOGD(dev, "enter");

	aw_bin = kzalloc(size_add(aw_fw_data.len, sizeof(*aw_bin)), GFP_KERNEL);
	if (!aw_bin)
		return -ENOMEM;

	aw_bin->len = aw_fw_data.len;
	memcpy(aw_bin->data, aw_fw_data.data, aw_fw_data.len);

	header_version = get_unaligned_le32(&aw_bin->data[4]);
	AW_DEV_LOGD(dev, "header_version=0x%x", header_version);

	if (header_version != HEADER_VERSION_1_0_0) {
		AW_DEV_LOGI(dev, "bin data type is raw");

		owned = kmemdup(aw_bin->data, aw_bin->len, GFP_KERNEL);
		if (!owned) {
			kfree(aw_bin);
			return -ENOMEM;
		}

		if (prof_info->prof_desc[prof_index].data_owned)
			kfree(prof_info->prof_desc[prof_index].data_container.data);

		prof_info->prof_desc[prof_index].data_container.data = owned;
		prof_info->prof_desc[prof_index].data_container.len = aw_bin->len;
		prof_info->prof_desc[prof_index].data_owned = true;
		prof_info->prof_desc[prof_index].prof_st = AW_PROFILE_OK;

		kfree(aw_bin);
		return 0;
	}

	ret = aw87xxx_parsing_bin_file(aw_bin);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "parse bin failed");
		goto parse_bin_failed;
	}

	for (i = 0; i < aw_bin->all_bin_parse_num; i++) {
		if (aw_bin->header_info[i].bin_data_type ==
		    DATA_TYPE_REGISTER) {
			u32 vlen = aw_bin->header_info[i].valid_data_len;
			owned = kmemdup(aw_fw_data.data +
					aw_bin->header_info[i].valid_data_addr,
					vlen, GFP_KERNEL);
			if (!owned) {
				ret = -ENOMEM;
				goto parse_bin_failed;
			}

			if (prof_info->prof_desc[prof_index].data_owned)
				kfree(prof_info->prof_desc[prof_index].data_container.data);

			prof_info->prof_desc[prof_index].data_container.len = vlen;
			prof_info->prof_desc[prof_index].data_container.data = owned;
			prof_info->prof_desc[prof_index].data_owned = true;
			break;
		}
	}

	if (i == aw_bin->all_bin_parse_num) {
		AW_DEV_LOGE(dev, "expected data type not found");
		ret = -EINVAL;
		goto parse_bin_failed;
	}

	prof_info->prof_desc[prof_index].prof_st = AW_PROFILE_OK;
	acf_info->prof_info.status = AW_ACF_UPDATE;
	AW_DEV_LOGD(dev, "done");
	ret = 0;

parse_bin_failed:
	kfree(aw_bin);
	return ret;
}

int aw87xxx_acf_parse(struct device *dev, struct acf_bin_info *acf_info)
{
	int ret;

	AW_DEV_LOGD(dev, "enter");
	acf_info->prof_info.status = AW_ACF_WAIT;

	ret = aw_check_acf_firmware(dev, acf_info->fw_data, acf_info->fw_size);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "load firmware check failed");
		goto err_free;
	}

	memcpy(&acf_info->acf_hdr, acf_info->fw_data,
	       sizeof(acf_info->acf_hdr));

	switch (acf_info->acf_hdr.hdr_version) {
	case AW_ACF_HDR_VER_0_0_0_1:
		ret = aw_parse_acf_v_0_0_0_1(dev, acf_info);
		break;
	case AW_ACF_HDR_VER_1_0_0_0:
		ret = aw_parse_acf_v_1_0_0_0(dev, acf_info);
		break;
	default:
		AW_DEV_LOGE(dev, "unsupported hdr_version[0x%x]",
			    acf_info->acf_hdr.hdr_version);
		ret = -EINVAL;
		break;
	}

	if (ret < 0)
		goto err_free;

	return 0;

err_free:
	aw87xxx_acf_profile_free(dev, acf_info);
	return ret;
}

struct aw_prof_desc *aw87xxx_acf_get_prof_desc_from_name(struct device *dev,
							 struct acf_bin_info *acf_info,
							 char *profile_name)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	struct aw_prof_desc *prof_desc = NULL;
	int i;

	AW_DEV_LOGD(dev, "enter");

	if (!acf_info->prof_info.status) {
		AW_DEV_LOGE(dev, "profile_cfg not load");
		return NULL;
	}

	for (i = 0; i < prof_info->count; i++) {
		if (!strncmp(profile_name,
			     prof_info->prof_desc[i].prof_name,
			     AW_PROFILE_STR_MAX)) {
			prof_desc = &prof_info->prof_desc[i];
			break;
		}
	}

	if (i == prof_info->count) {
		AW_DEV_LOGE(dev, "profile not found");
		return NULL;
	}

	AW_DEV_LOGI(dev, "get prof desc down");
	return prof_desc;
}

int aw87xxx_acf_get_prof_index_from_name(struct device *dev,
					 struct acf_bin_info *acf_info,
					 char *profile_name)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int i;

	if (!acf_info->prof_info.status) {
		AW_DEV_LOGE(dev, "profile_cfg not load");
		return -EINVAL;
	}

	for (i = 0; i < prof_info->count; i++) {
		if (!strncmp(profile_name,
			     prof_info->prof_name_list[i],
			     AW_PROFILE_STR_MAX))
			return i;
	}

	AW_DEV_LOGE(dev, "profile_index not found");
	return -EINVAL;
}

char *aw87xxx_acf_get_prof_name_from_index(struct device *dev,
					   struct acf_bin_info *acf_info,
					   int index)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;

	if (!acf_info->prof_info.status) {
		AW_DEV_LOGE(dev, "profile_cfg not load");
		return NULL;
	}

	if (index >= prof_info->count || index < 0) {
		AW_DEV_LOGE(dev, "profile_index out of table");
		return NULL;
	}

	return prof_info->prof_desc[index].prof_name;
}

int aw87xxx_acf_get_profile_count(struct device *dev,
				  struct acf_bin_info *acf_info)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;

	if (!acf_info->prof_info.status) {
		AW_DEV_LOGE(dev, "profile_cfg not load");
		return -EINVAL;
	}

	if (prof_info->count > 0)
		return prof_info->count;

	return -EINVAL;
}

char *aw87xxx_acf_get_prof_off_name(struct device *dev,
				    struct acf_bin_info *acf_info)
{
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	int i, ret;

	if (!acf_info->prof_info.status) {
		AW_DEV_LOGE(dev, "profile_cfg not load");
		return NULL;
	}

	for (i = 0; i < prof_info->count; i++) {
		ret = aw_check_prof_str_is_off(prof_info->prof_name_list[i]);
		if (!ret)
			return prof_info->prof_name_list[i];
	}

	return NULL;
}

void aw87xxx_acf_init(struct aw_device *aw_dev,
		      struct acf_bin_info *acf_info,
		      int index)
{
	acf_info->load_count = 0;
	acf_info->prof_info.status = AW_ACF_WAIT;
	acf_info->dev_index = index;
	acf_info->aw_dev = aw_dev;
	acf_info->product_cnt = aw_dev->product_cnt;
	acf_info->product_tab = aw_dev->product_tab;
	acf_info->prof_info.prof_desc = NULL;
	acf_info->fw_data = NULL;
	acf_info->fw_size = 0;
}
