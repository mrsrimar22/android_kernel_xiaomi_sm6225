/* SPDX-License-Identifier: GPL-2.0 */
/*
 * aw87xxx_core.h  aw87xxx pa module
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 */

#ifndef __AW87XXX_CORE_H__
#define __AW87XXX_CORE_H__

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/srcu.h>
#include <linux/rculist.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "aw87xxx_acf_bin.h"
#include "aw87xxx_device.h"
#include "aw87xxx_monitor.h"

#define AW_CFG_UPDATE_DELAY_TIMER	3000

#define AW87XXX_PRIVATE_KCONTROL_NUM	4
#define AW87XXX_PUBLIC_KCONTROL_NUM	2

#define AW_I2C_RETRIES			5
#define AW_I2C_RETRY_DELAY		2
#define AW_I2C_READ_MSG_NUM		2

#define AW87XXX_FW_NAME_MAX		64
#define AW_NAME_BUF_MAX			64
#define AW_LOAD_FW_RETRIES		3

#define AW_DEV_REG_RD_ACCESS		BIT(0)
#define AW_DEV_REG_WR_ACCESS		BIT(1)

#define AWRW_ADDR_BYTES			1
#define AWRW_DATA_BYTES			1
#define AWRW_HDR_LEN			24

enum awrw_flag {
	AWRW_FLAG_WRITE = 0,
	AWRW_FLAG_READ,
};

enum awrw_i2c_status {
	AWRW_I2C_ST_NONE = 0,
	AWRW_I2C_ST_READ,
	AWRW_I2C_ST_WRITE,
};

enum awrw_hdr_index {
	AWRW_HDR_WR_FLAG = 0,
	AWRW_HDR_ADDR_BYTES,
	AWRW_HDR_DATA_BYTES,
	AWRW_HDR_REG_NUM,
	AWRW_HDR_REG_ADDR,
	AWRW_HDR_MAX,
};

struct aw_i2c_packet {
	char status;
	u32 reg_num;
	u32 reg_addr;
	char *reg_data;
};

enum aw_bin_type {
	BIN_TYPE_ACF,
	BIN_TYPE_SINGLE,
};

struct aw87xxx {
	struct list_head node;
	char fw_name[AW87XXX_FW_NAME_MAX];
	int dev_index;
	char current_profile_buf[AW_PROFILE_STR_MAX];
	char *current_profile;
	char prof_off_name[AW_PROFILE_STR_MAX];
	struct device *dev;
	bool is_suspend;

	int bin_type;
	int support_prof_count;
	char **support_prof;
	void **single_bin_data;

	struct mutex reg_lock; /* state register lock */
	struct aw_device aw_dev;
	struct aw_i2c_packet i2c_packet;
	u8 preset_buf[128];

	struct delayed_work fw_load_work;
	struct acf_bin_info acf_info;

	struct snd_soc_component *codec;
	struct aw_monitor monitor;
};

int aw87xxx_add_codec_controls(struct snd_soc_component *codec);
int aw87xxx_update_profile(struct aw87xxx *aw87xxx, char *profile);
int aw87xxx_update_profile_esd(struct aw87xxx *aw87xxx, char *profile);

#endif
