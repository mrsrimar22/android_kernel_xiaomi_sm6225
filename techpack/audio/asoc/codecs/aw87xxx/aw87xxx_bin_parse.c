// SPDX-License-Identifier: GPL-2.0
/*
 * aw87xxx_bin_parse.c
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 */

#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "aw87xxx_bin_parse.h"
#include "aw87xxx_log.h"

/* "code version"-"excel version" */
#define AWINIC_CODE_VERSION "V0.0.7-V1.0.4"

static int aw_parse_bin_header_1_0_0(struct aw_bin *bin);

/*
 * check sum data
 */
static int aw_check_sum(struct aw_bin *bin, int bin_num)
{
	struct bin_header_info *hdr = &bin->header_info[bin_num];
	u32 sum_data = 0;
	u32 check_sum;
	u8 *p_check_sum;
	u32 i;

	AW_BIN_LOGD("enter");

	p_check_sum = &bin->data[hdr->valid_data_addr - hdr->header_len];
	AW_BIN_LOGD("p_check_sum = %p", p_check_sum);

	check_sum = get_unaligned_le32(p_check_sum);

	for (i = 4; i < hdr->bin_data_len + hdr->header_len; i++)
		sum_data += p_check_sum[i];

	AW_BIN_LOGD("bin_num=%d, check_sum=0x%x, sum_data=0x%x",
		    bin_num, check_sum, sum_data);

	if (sum_data != check_sum) {
		AW_BIN_LOGE("check sum error!");
		AW_BIN_LOGE("bin_num=%d, check_sum=0x%x, sum_data=0x%x",
			    bin_num, check_sum, sum_data);
		return -EINVAL;
	}

	return 0;
}

static int aw_check_data_version(struct aw_bin *bin, int bin_num)
{
	int i;

	AW_BIN_LOGD("enter");

	for (i = DATA_VERSION_V1; i < DATA_VERSION_MAX; i++) {
		if (bin->header_info[bin_num].bin_data_ver == i)
			return 0;
	}

	AW_BIN_LOGE("Unrecognized this bin data version");
	return -EINVAL;
}

static int aw_check_register_num_v1(struct aw_bin *bin, int bin_num)
{
	struct bin_header_info *hdr = &bin->header_info[bin_num];
	u32 check_register_num;
	u32 parse_register_num;
	u8 *p_check_sum;

	AW_BIN_LOGD("enter");

	p_check_sum = &bin->data[hdr->valid_data_addr];
	AW_BIN_LOGD("p_check_sum = %p", p_check_sum);

	parse_register_num = get_unaligned_le32(p_check_sum);
	check_register_num = (hdr->bin_data_len - 4) /
			     (hdr->reg_byte_len + hdr->data_byte_len);

	AW_BIN_LOGD("bin_num=%d, parse_reg_num=0x%x, check_reg_num=0x%x",
		    bin_num, parse_register_num, check_register_num);

	if (parse_register_num != check_register_num) {
		AW_BIN_LOGE("register num error!");
		AW_BIN_LOGE("bin_num=%d, parse_reg=0x%x, check_reg=0x%x",
			    bin_num, parse_register_num, check_register_num);
		return -EINVAL;
	}

	hdr->reg_num = parse_register_num;
	hdr->valid_data_len = hdr->bin_data_len - 4;
	hdr->valid_data_addr += 4;

	return 0;
}

static int aw_check_dsp_reg_num_v1(struct aw_bin *bin, int bin_num)
{
	struct bin_header_info *hdr = &bin->header_info[bin_num];
	u32 check_dsp_reg_num;
	u32 parse_dsp_reg_num;
	u8 *p_check_sum;

	AW_BIN_LOGD("enter");

	p_check_sum = &bin->data[hdr->valid_data_addr];
	AW_BIN_LOGD("p_check_sum = %p", p_check_sum);

	parse_dsp_reg_num = get_unaligned_le32(p_check_sum + 4);
	hdr->reg_data_byte_len = get_unaligned_le32(p_check_sum + 8);
	check_dsp_reg_num = (hdr->bin_data_len - 12) / hdr->reg_data_byte_len;

	AW_BIN_LOGD("bin_num=%d, parse_dsp_num=0x%x, check_dsp_num=0x%x",
		    bin_num, parse_dsp_reg_num, check_dsp_reg_num);

	if (parse_dsp_reg_num != check_dsp_reg_num) {
		AW_BIN_LOGE("dsp reg num is error");
		AW_BIN_LOGE("bin_num=%d, parse=0x%x, check=0x%x",
			    bin_num, parse_dsp_reg_num, check_dsp_reg_num);
		return -EINVAL;
	}

	hdr->download_addr = get_unaligned_le32(p_check_sum);
	hdr->reg_num = parse_dsp_reg_num;
	hdr->valid_data_len = hdr->bin_data_len - 12;
	hdr->valid_data_addr += 12;

	return 0;
}

static int aw_check_soc_app_num_v1(struct aw_bin *bin, int bin_num)
{
	struct bin_header_info *hdr = &bin->header_info[bin_num];
	u32 check_soc_app_num;
	u32 parse_soc_app_num;
	u8 *p_check_sum;

	AW_BIN_LOGD("enter");

	p_check_sum = &bin->data[hdr->valid_data_addr];
	AW_BIN_LOGD("p_check_sum = %p", p_check_sum);

	hdr->app_version = get_unaligned_le32(p_check_sum);
	parse_soc_app_num = get_unaligned_le32(p_check_sum + 8);
	check_soc_app_num = hdr->bin_data_len - 12;

	AW_BIN_LOGD("bin_num=%d, parse_soc_num=0x%x, check_soc_num=0x%x",
		    bin_num, parse_soc_app_num, check_soc_app_num);

	if (parse_soc_app_num != check_soc_app_num) {
		AW_BIN_LOGE("soc app num is error");
		AW_BIN_LOGE("bin_num=%d, parse=0x%x, check=0x%x",
			    bin_num, parse_soc_app_num, check_soc_app_num);
		return -EINVAL;
	}

	hdr->reg_num = parse_soc_app_num;
	hdr->download_addr = get_unaligned_le32(p_check_sum + 4);
	hdr->valid_data_len = hdr->bin_data_len - 12;
	hdr->valid_data_addr += 12;

	return 0;
}

/*
 * bin header 1_0_0
 */
static void aw_get_single_bin_header_1_0_0(struct aw_bin *bin)
{
	struct bin_header_info *hdr = &bin->header_info[bin->all_bin_parse_num];

	AW_BIN_LOGD("enter");

	hdr->header_len = 60;
	hdr->check_sum = get_unaligned_le32(bin->p_addr);
	hdr->header_ver = get_unaligned_le32(bin->p_addr + 4);
	hdr->bin_data_type = get_unaligned_le32(bin->p_addr + 8);
	hdr->bin_data_ver = get_unaligned_le32(bin->p_addr + 12);
	hdr->bin_data_len = get_unaligned_le32(bin->p_addr + 16);
	hdr->ui_ver = get_unaligned_le32(bin->p_addr + 20);

	memcpy(hdr->chip_type, bin->p_addr + 24, 8);

	hdr->reg_byte_len = get_unaligned_le32(bin->p_addr + 32);
	hdr->data_byte_len = get_unaligned_le32(bin->p_addr + 36);
	hdr->device_addr = get_unaligned_le32(bin->p_addr + 40);

	hdr->reg_num = 0;
	hdr->reg_data_byte_len = 0;
	hdr->download_addr = 0;
	hdr->app_version = 0;
	hdr->valid_data_len = 0;

	bin->all_bin_parse_num++;
}

static int aw_parse_each_of_multi_bins_1_0_0(u32 bin_num,
					     int bin_serial_num,
					     struct aw_bin *bin)
{
	u32 bin_start_addr;
	u32 valid_data_len;
	struct bin_header_info *prev_hdr;
	struct bin_header_info *curr_hdr;

	AW_BIN_LOGD("enter multi bin branch");

	prev_hdr = &bin->header_info[bin->all_bin_parse_num - 1];
	curr_hdr = &bin->header_info[bin->all_bin_parse_num];

	if (!bin_serial_num) {
		bin_start_addr = get_unaligned_le32(bin->p_addr + 64);
		bin->p_addr += 60 + bin_start_addr;
		curr_hdr->valid_data_addr = prev_hdr->valid_data_addr +
					    4 + 8 * bin_num + 60;
	} else {
		valid_data_len = prev_hdr->bin_data_len;
		bin->p_addr += 60 + valid_data_len;
		curr_hdr->valid_data_addr = prev_hdr->valid_data_addr +
					    valid_data_len + 60;
	}

	return aw_parse_bin_header_1_0_0(bin);
}

/*
 * Get the number of bins in multi bins, and set a for loop,
 * loop processing each bin data
 */
static int aw_get_multi_bin_header_1_0_0(struct aw_bin *bin)
{
	u32 bin_num;
	u32 i;
	int ret;

	AW_BIN_LOGD("enter multi bin branch");

	bin_num = get_unaligned_le32(bin->p_addr + 60);

	if (bin->multi_bin_parse_num == 1)
		bin->header_info[bin->all_bin_parse_num].valid_data_addr = 60;

	aw_get_single_bin_header_1_0_0(bin);

	for (i = 0; i < bin_num; i++) {
		AW_BIN_LOGD("enter multi bin for loop is %d", i);
		ret = aw_parse_each_of_multi_bins_1_0_0(bin_num, i, bin);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/*
 * If the bin framework header version is 1.0.0,
 * determine the data type of bin, and then perform different processing
 * according to the data type.
 * If it is a single bin data type, write data directly to structure array.
 * If it is a multi-bin data type, first obtain the number of bins,
 * and recursively call bin frame header processing function according to
 * bin number to process the frame header information separately.
 */
static int aw_parse_bin_header_1_0_0(struct aw_bin *bin)
{
	u32 bin_data_type;
	int ret;

	AW_BIN_LOGD("enter");

	bin_data_type = get_unaligned_le32(bin->p_addr + 8);
	AW_BIN_LOGD("bin_data_type 0x%x", bin_data_type);

	switch (bin_data_type) {
	case DATA_TYPE_REGISTER:
	case DATA_TYPE_DSP_REG:
	case DATA_TYPE_SOC_APP:
		AW_BIN_LOGD("enter single bin branch");
		bin->single_bin_parse_num++;
		AW_BIN_LOGD("bin->single_bin_parse_num is %d",
			    bin->single_bin_parse_num);

		if (!bin->multi_bin_parse_num)
			bin->header_info[bin->all_bin_parse_num].valid_data_addr = 60;

		aw_get_single_bin_header_1_0_0(bin);
		break;
	case DATA_TYPE_MULTI_BINS:
		AW_BIN_LOGD("enter multi bin branch");
		bin->multi_bin_parse_num++;
		AW_BIN_LOGD("bin->multi_bin_parse_num is %d",
			    bin->multi_bin_parse_num);

		ret = aw_get_multi_bin_header_1_0_0(bin);
		if (ret < 0)
			return ret;
		break;
	default:
		AW_BIN_LOGD("Unrecognized this bin data type 0x%x",
			    bin_data_type);
		break;
	}

	return 0;
}

/* get the bin's header version */
static int aw_check_bin_header_version(struct aw_bin *bin)
{
	u32 header_version;

	header_version = get_unaligned_le32(bin->p_addr + 4);
	AW_BIN_LOGD("header_version 0x%x", header_version);

	switch (header_version) {
	case HEADER_VERSION_1_0_0:
		return aw_parse_bin_header_1_0_0(bin);
	default:
		AW_BIN_LOGE("Unrecognized this bin header version");
		return -EINVAL;
	}
}

int aw87xxx_parsing_bin_file(struct aw_bin *bin)
{
	struct bin_header_info *hdr;
	int i, ret;

	AW_BIN_LOGD("code version: %s", AWINIC_CODE_VERSION);

	if (!bin) {
		AW_BIN_LOGE("bin is NULL");
		return -EINVAL;
	}

	bin->p_addr = bin->data;
	bin->all_bin_parse_num = 0;
	bin->multi_bin_parse_num = 0;
	bin->single_bin_parse_num = 0;

	ret = aw_check_bin_header_version(bin);
	if (ret < 0) {
		AW_BIN_LOGE("check bin header version error");
		return ret;
	}

	bin->p_addr = NULL;

	for (i = 0; i < bin->all_bin_parse_num; i++) {
		hdr = &bin->header_info[i];

		ret = aw_check_sum(bin, i);
		if (ret < 0) {
			AW_BIN_LOGE("check sum data error");
			return ret;
		}

		ret = aw_check_data_version(bin, i);
		if (ret < 0) {
			AW_BIN_LOGE("check data version error");
			return ret;
		}

		if (hdr->bin_data_ver == DATA_VERSION_V1) {
			if (hdr->bin_data_type == DATA_TYPE_REGISTER) {
				ret = aw_check_register_num_v1(bin, i);
				if (ret < 0) {
					AW_BIN_LOGE("check reg num error");
					return ret;
				}
			} else if (hdr->bin_data_type == DATA_TYPE_DSP_REG) {
				ret = aw_check_dsp_reg_num_v1(bin, i);
				if (ret < 0) {
					AW_BIN_LOGE("check dsp reg num error");
					return ret;
				}
			} else if (hdr->bin_data_type == DATA_TYPE_SOC_APP) {
				ret = aw_check_soc_app_num_v1(bin, i);
				if (ret < 0) {
					AW_BIN_LOGE("check soc app num error");
					return ret;
				}
			} else {
				hdr->valid_data_len = hdr->bin_data_len;
			}
		}
	}

	AW_BIN_LOGD("parsing success");

	return 0;
}
