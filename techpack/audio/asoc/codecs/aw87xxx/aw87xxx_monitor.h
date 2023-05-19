/* SPDX-License-Identifier: GPL-2.0 */
/*
 * aw87xxx_monitor.h
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 */

#ifndef __AW87XXX_MONITOR_H__
#define __AW87XXX_MONITOR_H__

#include <linux/device.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/workqueue.h>

/* #define AW_DEBUG */

#define AW_WAIT_DSP_OPEN_TIME			3000
#define AW_VBAT_CAPACITY_MIN			0
#define AW_VBAT_CAPACITY_MAX			100
#define AW_VMAX_INIT_VAL			0xFFFFFFFF
#define AW_VBAT_MAX				100
#define AW_VMAX_MAX				0
#define AW_DEFAULT_MONITOR_TIME			3000
#define AW_WAIT_TIME				3000
#define REG_STATUS_CHECK_MAX			5
#define AW_ESD_CHECK_DELAY			1000
#define AW_ESD_CHECK_DELAY_MAX			1500
#define AW_MONOTOR_ESD_ERR_CNT_MAX		3

#define AW_MONITOR_TIME_MIN			0
#define AW_MONITOR_TIME_MAX			50000

#define AW_ESD_ENABLE				true
#define AW_ESD_DISABLE				false
#define AW_ESD_ENABLE_STRLEN			16
#define MONITOR_EN_MASK				0x01

#define AW_TABLE_SIZE				sizeof(struct aw_table)

#define IPEAK_NONE				0xFF
#define GAIN_NONE				0xFF
#define VMAX_NONE				0xFFFFFFFF

enum aw_mon_logic {
	AW_MON_LOGIC_OR = 0,
	AW_MON_LOGIC_AND = 1,
};

enum aw_monitor_bit {
	MONITOR_EN_BIT = 0,
	MONITOR_LOGIC_BIT = 1,
	MONITOR_IPEAK_EN_BIT = 2,
	MONITOR_GAIN_EN_BIT = 3,
	MONITOR_VMAX_EN_BIT = 4,
	MONITOR_TEMP_EN_BIT = 5,
	MONITOR_VOL_EN_BIT = 6,
	MONITOR_TEMPERATURE_SOURCE_BIT = 7,
	MONITOR_VOLTAGE_SOURCE_BIT = 8,
	MONITOR_VOLTAGE_MODE_BIT = 9,
};

enum aw_sys_info {
	AW_SYS_VOLTAGE_NOW = 0,
	AW_SYS_CAPACITY = 1,
};

enum aw_info_type {
	AW_VOLTAGE_INFO = 0,
	AW_CAPACITY_INFO = 1,
	AW_TEMPERATURE_INFO = 2,
};

enum aw_src_type {
	AW_CHIP_INFO = 0,
	AW_PLATFORM_INFO = 1,
};

enum aw_monitor_init {
	AW_MONITOR_CFG_WAIT = 0,
	AW_MONITOR_CFG_OK = 1,
};

enum aw_monitor_hdr_info {
	AW_MONITOR_HDR_DATA_SIZE = 0x00000004,
	AW_MONITOR_HDR_DATA_BYTE_LEN = 0x00000004,
};

enum aw_monitor_data_ver {
	AW_MONITOR_DATA_VER = 0x00000001,
	AW_MONITOR_DATA_VER_MAX,
};

enum aw_monitor_hdr_ver {
	AW_MONITOR_HDR_VER_0_1_0 = 0x00010000,
	AW_MONITOR_HDR_VER_0_1_1 = 0x00010100,
	AW_MONITOR_HDR_VER_0_1_2 = 0x00010200,
};

enum aw_monitor_first_enter {
	AW_FIRST_ENTRY = 0,
	AW_NOT_FIRST_ENTRY = 1,
};

struct aw_container {
	int len;
	u8 data[];
};

struct aw_table {
	s16 min_val;
	s16 max_val;
	u16 ipeak;
	u16 gain;
	u32 vmax;
};

struct aw_bin_header {
	u32 check_sum;
	u32 header_ver;
	u32 bin_data_type;
	u32 bin_data_ver;
	u32 bin_data_size;
	u32 ui_ver;
	char product[8];
	u32 addr_byte_len;
	u32 data_byte_len;
	u32 device_addr;
	u32 reserve[4];
};

struct aw_monitor_header {
	u32 monitor_switch;
	u32 monitor_time;
	u32 monitor_count;
	u32 step_count;
	u32 reserve[4];
};

/* v0.1.2 */
struct aw_monitor_hdr {
	u32 check_sum;
	u32 monitor_ver;
	char chip_type[16];
	u32 ui_ver;
	u32 monitor_time;
	u32 monitor_count;
	u32 enable_flag;
	/*
	 * [bit 31:7]
	 * [bit 9: voltage mode]
	 * [bit 8: voltage source]
	 * [bit 7: temperature source]
	 * [bit 6: vol en]
	 * [bit 5: temp en]
	 * [bit 4: vmax en]
	 * [bit 3: gain en]
	 * [bit 2: ipeak en]
	 * [bit 1: & or | flag]
	 * [bit 0: monitor en]
	 */
	u32 temp_aplha;
	u32 temp_num;
	u32 single_temp_size;
	u32 temp_offset;
	u32 vol_aplha;
	u32 vol_num;
	u32 single_vol_size;
	u32 vol_offset;
	u32 reserver[3];
};

struct vmax_step_config {
	u32 vbat_min;
	u32 vbat_max;
	int vmax_vol;
};

struct aw_table_info {
	u8 table_num;
	struct aw_table *aw_table;
};

struct aw_monitor_cfg {
	u8 monitor_status;
	u32 monitor_switch;
	u32 monitor_time;
	u32 monitor_count;
	u32 logic_switch;
	u32 temp_switch;
	u32 temp_aplha;
	u32 vol_switch;
	u32 vol_aplha;
	u32 ipeak_switch;
	u32 gain_switch;
	u32 vmax_switch;
	u32 temp_source;
	u32 vol_source;
	u32 vol_mode;
	struct aw_table_info temp_info;
	struct aw_table_info vol_info;
};

struct aw_monitor_trace {
	int pre_val;
	int sum_val;
	struct aw_table aw_table;
};

struct aw_monitor {
	bool esd_enable;
	int dev_index;
	u8 first_entry;
	u8 timer_cnt;
	u32 vbat_sum;
	int custom_capacity;
	u32 pre_vmax;
	u32 esd_err_cnt;
	u8 samp_count;
	u32 version;

	int bin_status;
	struct aw_monitor_header monitor_hdr;
	struct vmax_step_config *vmax_cfg;
	struct aw_monitor_cfg monitor_cfg;

	struct aw_monitor_trace temp_trace;
	struct aw_monitor_trace vol_trace;
	struct aw_container *monitor_container;

	struct delayed_work with_dsp_work;
};

void aw87xxx_monitor_cfg_free(struct aw_monitor *monitor);
int aw87xxx_monitor_bin_parse(struct device *dev,
			      const u8 *monitor_data,
			      u32 data_len);
void aw87xxx_monitor_stop(struct aw_monitor *monitor);
void aw87xxx_monitor_start(struct aw_monitor *monitor);
int aw87xxx_monitor_no_dsp_get_vmax(struct aw_monitor *monitor,
				    int *vmax);
void aw87xxx_monitor_init(struct device *dev,
			  struct aw_monitor *monitor,
			  struct device_node *dev_node);
void aw87xxx_monitor_exit(struct aw_monitor *monitor);
int aw87xxx_dev_monitor_switch_set(struct aw_monitor *monitor,
				   u32 enable);

#endif
