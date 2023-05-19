// SPDX-License-Identifier: GPL-2.0
/*
 * aw87xxx_monitor.c
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 */

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "aw87xxx_core.h"
#include "aw87xxx_bin_parse.h"
#include "aw87xxx_device.h"
#include "aw87xxx_dsp.h"
#include "aw87xxx_log.h"
#include "aw87xxx_monitor.h"

#define AW_MONITOR_BIN_PARSE_VERSION	"V0.1.0"
#define AW87XXX_MONITOR_NAME		"aw87xxx_monitor.bin"

/*
 * aw87xxx monitor bin check
 */
static int aw_monitor_get_ctrl_info(struct device *dev,
				    u32 *switch_ctrl,
				    u32 *count,
				    u32 *time)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;

	switch (monitor->version) {
	case DATA_VERSION_V1:
		*time = monitor->monitor_hdr.monitor_time;
		*count = monitor->monitor_hdr.monitor_count;
		*switch_ctrl = monitor->monitor_hdr.monitor_switch;
		break;
	case AW_MONITOR_HDR_VER_0_1_2:
		*time = monitor->monitor_cfg.monitor_time;
		*count = monitor->monitor_cfg.monitor_count;
		*switch_ctrl = monitor->monitor_cfg.monitor_switch;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int aw_monitor_check_header_v_1_0_0(struct device *dev,
					   const u8 *data,
					   u32 data_len)
{
	struct aw_bin_header *header = (struct aw_bin_header *)data;
	int i;

	if (header->bin_data_type != DATA_TYPE_MONITOR_ANALOG) {
		AW_DEV_LOGE(dev, "monitor data_type check error!");
		return -EINVAL;
	}

	if (header->bin_data_size != AW_MONITOR_HDR_DATA_SIZE) {
		AW_DEV_LOGE(dev, "monitor data_size error!");
		return -EINVAL;
	}

	if (header->data_byte_len != AW_MONITOR_HDR_DATA_BYTE_LEN) {
		AW_DEV_LOGE(dev, "monitor data_byte_len error!");
		return -EINVAL;
	}

	for (i = 0; i < AW_MONITOR_DATA_VER_MAX; i++) {
		if (header->bin_data_ver == i) {
			AW_LOGD("monitor bin_data_ver[0x%x]", i);
			break;
		}
	}

	if (i == AW_MONITOR_DATA_VER_MAX)
		return -EINVAL;

	return 0;
}

static int aw_monitor_check_data_v1_size(struct device *dev,
					 const u8 *data,
					 int data_len)
{
	int bin_header_len = sizeof(struct aw_bin_header);
	int monitor_header_len = sizeof(struct aw_monitor_header);
	int monitor_data_len = sizeof(struct vmax_step_config);
	struct aw_monitor_header *monitor_hdr;
	int len;

	AW_DEV_LOGD(dev, "enter");

	if (data_len < bin_header_len + monitor_header_len) {
		AW_DEV_LOGE(dev, "bin len less than header sizes");
		return -EINVAL;
	}

	monitor_hdr = (struct aw_monitor_header *)(data + bin_header_len);
	len = data_len - bin_header_len - monitor_header_len;

	if (len < monitor_hdr->step_count * monitor_data_len) {
		AW_DEV_LOGE(dev, "bin data len is not enough, check failed");
		return -EINVAL;
	}

	AW_DEV_LOGD(dev, "succeed");

	return 0;
}

static int aw_monitor_check_data_size(struct device *dev,
				      const u8 *data,
				      int data_len)
{
	struct aw_bin_header *header = (struct aw_bin_header *)data;
	int ret;

	switch (header->bin_data_ver) {
	case AW_MONITOR_DATA_VER:
		ret = aw_monitor_check_data_v1_size(dev, data, data_len);
		if (ret < 0)
			return ret;
		break;
	default:
		AW_DEV_LOGE(dev, "bin data_ver[0x%x] non support",
			    header->bin_data_ver);
		return -EINVAL;
	}

	return 0;
}

static int aw_monitor_check_bin_header(struct device *dev,
				       const u8 *data,
				       int data_len)
{
	struct aw_bin_header *header;
	int ret;

	if (data_len < sizeof(*header)) {
		AW_DEV_LOGE(dev, "bin len is less than aw_bin_header");
		return -EINVAL;
	}

	header = (struct aw_bin_header *)data;

	switch (header->header_ver) {
	case HEADER_VERSION_1_0_0:
		ret = aw_monitor_check_header_v_1_0_0(dev, data, data_len);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "monitor bin header info error!");
			return ret;
		}
		break;
	default:
		AW_DEV_LOGE(dev, "bin version[0x%x] non support",
			    header->header_ver);
		return -EINVAL;
	}

	return 0;
}

static int aw_monitor_bin_check_sum(struct device *dev,
				    const u8 *data,
				    int data_len)
{
	u32 *check_sum = (u32 *)data;
	int data_sum = 0;
	int i;

	for (i = 4; i < data_len; i++)
		data_sum += data[i];

	if (*check_sum != data_sum) {
		AW_DEV_LOGE(dev, "check_sum[%u] != data_sum[%d]",
			    *check_sum, data_sum);
		return -ENOMEM;
	}

	AW_DEV_LOGD(dev, "succeed");

	return 0;
}

static int aw_monitor_bin_check(struct device *dev,
				const u8 *monitor_data,
				u32 data_len)
{
	int ret;

	if (!monitor_data || data_len == 0) {
		AW_DEV_LOGE(dev, "none data to parse");
		return -EINVAL;
	}

	ret = aw_monitor_bin_check_sum(dev, monitor_data, data_len);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "bin data check sum failed");
		return ret;
	}

	ret = aw_monitor_check_bin_header(dev, monitor_data, data_len);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "bin data len check failed");
		return ret;
	}

	ret = aw_monitor_check_data_size(dev, monitor_data, data_len);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "bin header info check failed");
		return ret;
	}

	return 0;
}

/*
 * aw87xxx monitor header bin parse
 */
static void aw_monitor_write_to_table_v1(struct device *dev,
					 struct vmax_step_config *vmax_step,
					 const u8 *vmax_data, u32 step_count)
{
	u32 i;

	for (i = 0; i < step_count; i++) {
		int idx = sizeof(*vmax_step) * i;

		vmax_step[i].vbat_min = get_unaligned_le32(&vmax_data[idx]);
		vmax_step[i].vbat_max = get_unaligned_le32(&vmax_data[idx + 4]);
		vmax_step[i].vmax_vol = get_unaligned_le32(&vmax_data[idx + 8]);

		AW_DEV_LOGI(dev, "vbat_min: %u, vbat_max: %u, vmax_vol: 0x%x",
			    vmax_step[i].vbat_min,
			    vmax_step[i].vbat_max,
			    vmax_step[i].vmax_vol);
	}
}

static int aw_monitor_parse_vol_data_v1(struct device *dev,
					struct aw_monitor *monitor,
					const u8 *monitor_data)
{
	struct vmax_step_config *vmax_step = NULL;
	const u8 *vmax_data;
	u32 step_count;

	step_count = monitor->monitor_hdr.step_count;
	if (step_count) {
		vmax_step = kcalloc(step_count, sizeof(*vmax_step), GFP_KERNEL);
		if (!vmax_step)
			return -ENOMEM;
	}

	vmax_data = monitor_data + sizeof(struct aw_bin_header) +
		    sizeof(struct aw_monitor_header);
	aw_monitor_write_to_table_v1(dev, vmax_step, vmax_data, step_count);
	kfree(monitor->vmax_cfg);
	monitor->vmax_cfg = vmax_step;

	AW_DEV_LOGI(dev, "vmax_data parse succeed");

	return 0;
}

static int aw_monitor_parse_data_v1(struct device *dev,
				    struct aw_monitor *monitor,
				    const u8 *monitor_data)
{
	struct aw_monitor_header *monitor_hdr = &monitor->monitor_hdr;
	int header_len;
	int ret;

	header_len = sizeof(struct aw_bin_header);
	memcpy(monitor_hdr, monitor_data + header_len, sizeof(*monitor_hdr));

	AW_DEV_LOGI(dev,
		    "switch: %u, time: %u(ms), count: %u, step_count: %u",
		    monitor_hdr->monitor_switch, monitor_hdr->monitor_time,
		    monitor_hdr->monitor_count, monitor_hdr->step_count);

	ret = aw_monitor_parse_vol_data_v1(dev, monitor, monitor_data);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "vmax_data parse failed");
		return ret;
	}

	monitor->bin_status = AW_MONITOR_CFG_OK;

	return 0;
}

static void aw_monitor_cfg_v_0_0_1_free(struct aw_monitor *monitor)
{
	monitor->bin_status = AW_MONITOR_CFG_WAIT;
	memset(&monitor->monitor_hdr, 0, sizeof(monitor->monitor_hdr));
	kfree(monitor->vmax_cfg);
}

static void aw_monitor_write_data_to_table(struct device *dev,
					   struct aw_table_info *table_info,
					   const u8 *offset_ptr)
{
	struct aw_table *aw_table = table_info->aw_table;
	int i;

	for (i = 0; i < table_info->table_num; i++) {
		int idx = i * sizeof(struct aw_table);

		aw_table[i].min_val = get_unaligned_le16(&offset_ptr[idx]);
		aw_table[i].max_val = get_unaligned_le16(&offset_ptr[idx + 2]);
		aw_table[i].ipeak = get_unaligned_le16(&offset_ptr[idx + 4]);
		aw_table[i].gain = get_unaligned_le16(&offset_ptr[idx + 6]);
		aw_table[i].vmax = get_unaligned_le32(&offset_ptr[idx + 8]);

		AW_DEV_LOGI(dev,
			    "min_val: %d, max_val: %d, ipeak: 0x%x, gain: 0x%x, vmax: 0x%x",
			    aw_table[i].min_val, aw_table[i].max_val,
			    aw_table[i].ipeak, aw_table[i].gain, aw_table[i].vmax);
	}
}

static int aw_monitor_parse_vol_data_v_0_1_2(struct device *dev,
					     const u8 *data)
{
	struct aw_monitor_hdr *monitor_hdr = (struct aw_monitor_hdr *)data;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor_cfg *monitor_cfg = &aw87xxx->monitor.monitor_cfg;
	struct aw_table_info *vol_info = &monitor_cfg->vol_info;

	kfree(vol_info->aw_table);

	vol_info->aw_table = kcalloc(monitor_hdr->vol_num,
				     sizeof(struct aw_table),
				     GFP_KERNEL);
	if (!vol_info->aw_table)
		return -ENOMEM;

	vol_info->table_num = monitor_hdr->vol_num;
	aw_monitor_write_data_to_table(dev,
				       vol_info,
				       &data[monitor_hdr->vol_offset]);

	return 0;
}

static int aw_monitor_parse_temp_data_v_0_1_2(struct device *dev,
					      const u8 *data)
{
	struct aw_monitor_hdr *monitor_hdr = (struct aw_monitor_hdr *)data;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor_cfg *monitor_cfg = &aw87xxx->monitor.monitor_cfg;
	struct aw_table_info *temp_info = &monitor_cfg->temp_info;

	kfree(temp_info->aw_table);

	temp_info->aw_table = kcalloc(monitor_hdr->temp_num,
				      sizeof(struct aw_table),
				      GFP_KERNEL);
	if (!temp_info->aw_table)
		return -ENOMEM;

	temp_info->table_num = monitor_hdr->temp_num;
	aw_monitor_write_data_to_table(dev,
				       temp_info,
				       &data[monitor_hdr->temp_offset]);

	return 0;
}

static void aw_monitor_parse_hdr_v_0_1_2(struct device *dev, const u8 *data)
{
	struct aw_monitor_hdr *monitor_hdr = (struct aw_monitor_hdr *)data;
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor_cfg *monitor_cfg = &aw87xxx->monitor.monitor_cfg;
	u32 flag = monitor_hdr->enable_flag;

	monitor_cfg->monitor_switch = (flag >> MONITOR_EN_BIT) &
				      MONITOR_EN_MASK;
	monitor_cfg->monitor_time = monitor_hdr->monitor_time;
	monitor_cfg->monitor_count = monitor_hdr->monitor_count;
	monitor_cfg->ipeak_switch = (flag >> MONITOR_IPEAK_EN_BIT) &
				    MONITOR_EN_MASK;
	monitor_cfg->logic_switch = (flag >> MONITOR_LOGIC_BIT) &
				    MONITOR_EN_MASK;
	monitor_cfg->gain_switch = (flag >> MONITOR_GAIN_EN_BIT) &
				   MONITOR_EN_MASK;
	monitor_cfg->vmax_switch = (flag >> MONITOR_VMAX_EN_BIT) &
				   MONITOR_EN_MASK;
	monitor_cfg->temp_switch = (flag >> MONITOR_TEMP_EN_BIT) &
				   MONITOR_EN_MASK;
	monitor_cfg->temp_aplha = monitor_hdr->temp_aplha;
	monitor_cfg->vol_switch = (flag >> MONITOR_VOL_EN_BIT) &
				  MONITOR_EN_MASK;
	monitor_cfg->vol_aplha = monitor_hdr->vol_aplha;
	monitor_cfg->temp_source = (flag >> MONITOR_TEMPERATURE_SOURCE_BIT) &
				   MONITOR_EN_MASK;
	monitor_cfg->vol_source = (flag >> MONITOR_VOLTAGE_SOURCE_BIT) &
				  MONITOR_EN_MASK;
	monitor_cfg->vol_mode = (flag >> MONITOR_VOLTAGE_MODE_BIT) &
				MONITOR_EN_MASK;

	AW_DEV_LOGI(dev, "chip name: %s", monitor_hdr->chip_type);
	AW_DEV_LOGI(dev, "ui ver: 0x%x", monitor_hdr->ui_ver);

	AW_DEV_LOGI(dev, "voltage mode: %d, vol src: %d, temp src: %d",
		    monitor_cfg->vol_mode,
		    monitor_cfg->vol_source,
		    monitor_cfg->temp_source);

	AW_DEV_LOGI(dev, "switch: %d, time: %d(ms), count: %d",
		    monitor_cfg->monitor_switch,
		    monitor_cfg->monitor_time,
		    monitor_cfg->monitor_count);

	AW_DEV_LOGI(dev, "logic: %d, ipeak: %d, gain: %d, vmax: %d",
		    monitor_cfg->logic_switch,
		    monitor_cfg->ipeak_switch,
		    monitor_cfg->gain_switch,
		    monitor_cfg->vmax_switch);

	AW_DEV_LOGI(dev, "temp_sw: %d, temp_alpha: %d, vol_sw: %d, vol_alpha: %d",
		    monitor_cfg->temp_switch,
		    monitor_cfg->temp_aplha,
		    monitor_cfg->vol_switch,
		    monitor_cfg->vol_aplha);
}

static int aw_monitor_check_fw_v_0_1_2(struct device *dev,
				       const u8 *data,
				       u32 data_len)
{
	struct aw_monitor_hdr *monitor_hdr = (struct aw_monitor_hdr *)data;
	u32 temp_size;
	u32 vol_size;

	if (data_len < sizeof(*monitor_hdr)) {
		AW_DEV_LOGE(dev, "params size[%u] < hdr size[%zu]!",
			    data_len, sizeof(*monitor_hdr));
		return -ENOMEM;
	}

	if (monitor_hdr->temp_offset > data_len) {
		AW_DEV_LOGE(dev, "temp_offset[%u] overflow file size[%u]!",
			    monitor_hdr->temp_offset, data_len);
		return -ENOMEM;
	}

	if (monitor_hdr->vol_offset > data_len) {
		AW_DEV_LOGE(dev, "vol_offset[%u] overflow file size[%u]!",
			    monitor_hdr->vol_offset, data_len);
		return -ENOMEM;
	}

	if (check_mul_overflow((u32)monitor_hdr->temp_num,
			       (u32)monitor_hdr->single_temp_size,
			       &temp_size)) {
		AW_DEV_LOGE(dev, "temp_size multiplication overflow!");
		return -ENOMEM;
	}

	if (temp_size > data_len) {
		AW_DEV_LOGE(dev, "temp_size[%u] overflow file size[%u]!",
			    temp_size, data_len);
		return -ENOMEM;
	}

	if (check_mul_overflow((u32)monitor_hdr->vol_num,
			       (u32)monitor_hdr->single_vol_size,
			       &vol_size)) {
		AW_DEV_LOGE(dev, "vol_size multiplication overflow!");
		return -ENOMEM;
	}

	if (vol_size > data_len) {
		AW_DEV_LOGE(dev, "vol_size[%u] overflow file size[%u]!",
			    vol_size, data_len);
		return -ENOMEM;
	}

	return 0;
}

static int aw_monitor_parse_data_v_0_1_2(struct device *dev,
					 const u8 *data,
					 u32 data_len)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	int ret;

	ret = aw_monitor_check_fw_v_0_1_2(dev, data, data_len);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "check monitor failed");
		return ret;
	}

	aw_monitor_parse_hdr_v_0_1_2(dev, data);

	ret = aw_monitor_parse_temp_data_v_0_1_2(dev, data);
	if (ret < 0)
		return ret;

	ret = aw_monitor_parse_vol_data_v_0_1_2(dev, data);
	if (ret < 0)
		return ret;

	monitor->bin_status = AW_MONITOR_CFG_OK;

	return 0;
}

static int aw_monitor_param_check_sum(struct device *dev,
				      const u8 *data,
				      u32 data_len)
{
	struct aw_monitor_hdr *monitor_hdr = (struct aw_monitor_hdr *)data;
	int check_sum = 0;
	u32 i;

	if (data_len < sizeof(*monitor_hdr)) {
		AW_DEV_LOGE(dev, "data size smaller than hdr");
		return -ENOMEM;
	}

	for (i = 4; i < data_len; i++)
		check_sum += data[i];

	if (monitor_hdr->check_sum != check_sum) {
		AW_DEV_LOGE(dev, "check_sum[%d] != actual check_sum[%d]",
			    monitor_hdr->check_sum, check_sum);
		return -ENOMEM;
	}

	return 0;
}

static int aw87xxx_monitor_bin_version_parse(struct device *dev,
					     const u8 *monitor_data)
{
	struct aw_bin_header *bin_header;
	struct aw_monitor_hdr *monitor_hdr;

	bin_header = (struct aw_bin_header *)monitor_data;
	switch (bin_header->bin_data_ver) {
	case DATA_VERSION_V1:
		return DATA_VERSION_V1;
	default:
		AW_DEV_LOGE(dev, "DATA_VERSION_V1 version Mismatched");
	}

	monitor_hdr = (struct aw_monitor_hdr *)monitor_data;
	switch (monitor_hdr->monitor_ver) {
	case AW_MONITOR_HDR_VER_0_1_2:
		return AW_MONITOR_HDR_VER_0_1_2;
	default:
		AW_DEV_LOGE(dev, "HDR_VER_0_1_2 version Mismatched");
	}

	AW_DEV_LOGE(dev, "unsupported monitor_hdr");

	return -EINVAL;
}

int aw87xxx_monitor_bin_parse(struct device *dev,
			      const u8 *monitor_data,
			      u32 data_len)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_container *monitor_cont;
	struct aw_monitor *monitor;
	int ret;

	if (!aw87xxx) {
		AW_DEV_LOGE(dev, "get struct aw87xxx failed");
		return -EINVAL;
	}

	monitor = &aw87xxx->monitor;
	monitor->bin_status = AW_MONITOR_CFG_WAIT;

	ret = aw87xxx_monitor_bin_version_parse(dev, monitor_data);
	switch (ret) {
	case DATA_VERSION_V1:
		AW_DEV_LOGD(dev, "DATA_VERSION_V1 enter");
		ret = aw_monitor_bin_check(dev, monitor_data, data_len);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "monitor bin check failed");
			return ret;
		}

		ret = aw_monitor_parse_data_v1(dev, monitor, monitor_data);
		if (ret < 0) {
			aw_monitor_cfg_v_0_0_1_free(monitor);
			return ret;
		}
		monitor->version = DATA_VERSION_V1;
		break;

	case AW_MONITOR_HDR_VER_0_1_2:
		AW_DEV_LOGD(dev, "AW_MONITOR_HDR_VER_0_1_2 enter");

		monitor_cont = kzalloc(size_add(data_len, sizeof(u32)), GFP_KERNEL);
		if (!monitor_cont)
			return -ENOMEM;

		kfree(monitor->monitor_container);
		monitor->monitor_container = monitor_cont;

		monitor_cont->len = data_len;
		memcpy(monitor_cont->data, monitor_data, data_len);

		AW_DEV_LOGD(dev, "len %d", monitor_cont->len);

		ret = aw_monitor_param_check_sum(dev, monitor_data, data_len);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "monitor bin check failed");
			return ret;
		}

		ret = aw_monitor_parse_data_v_0_1_2(dev, monitor_data, data_len);
		if (ret < 0)
			return ret;

		monitor->version = AW_MONITOR_HDR_VER_0_1_2;
		break;

	default:
		AW_DEV_LOGE(dev, "Unrecognized this bin data version[0x%x]", ret);
	}

	return 0;
}

/*
 * aw87xxx monitor get adjustment vmax of power
 */
static int aw_monitor_get_battery_status(struct device *dev,
					 int type,
					 int *value)
{
	const char *name = "battery";
	union power_supply_propval prop = { 0 };
	struct power_supply *psy;
	int ret;

	psy = power_supply_get_by_name(name);
	if (!psy) {
		AW_DEV_LOGE(dev, "no struct power supply name: %s", name);
		return -EINVAL;
	}

	ret = power_supply_get_property(psy, type, &prop);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "get type: %d failed", type);
		power_supply_put(psy);
		return -EINVAL;
	}

	*value = prop.intval;
	AW_DEV_LOGI(dev, "get type: %d, value: %d", type, *value);

	power_supply_put(psy);
	return 0;
}

static int aw_monitor_get_chip_temperature(struct device *dev, int *value)
{
	return 0;
}

static int aw_monitor_get_chip_voltage(struct device *dev, int *value)
{
	return 0;
}

static int aw_monitor_get_sys_temperature(struct device *dev, int *value)
{
	int temperature = 0;
	int ret;

	ret = aw_monitor_get_battery_status(dev,
					    POWER_SUPPLY_PROP_TEMP,
					    &temperature);
	if (ret < 0)
		return ret;

	*value = temperature / 10;
	AW_DEV_LOGI(dev, "sys temperature: %d", *value);

	return ret;
}

static int aw_monitor_get_sys_voltage(struct device *dev, int *value)
{
	int voltage = 0;
	int ret;

	ret = aw_monitor_get_battery_status(dev,
					    POWER_SUPPLY_PROP_VOLTAGE_NOW,
					    &voltage);
	if (ret < 0)
		return ret;

	/*
	 * Taking APQ8096/Mtk6765/SPRD platform as an example,
	 * the returned voltage unit is uV.
	 */
	*value = voltage / 1000;
	AW_DEV_LOGI(dev, "sys voltage: %d", *value);

	return ret;
}

static int aw_monitor_get_sys_capacity(struct device *dev, int *value)
{
	int ret;

	ret = aw_monitor_get_battery_status(dev,
					    POWER_SUPPLY_PROP_CAPACITY,
					    value);
	if (ret < 0)
		return ret;

	AW_DEV_LOGI(dev, "sys capacity: %d", *value);

	return ret;
}

static int aw_monitor_get_chip_runtime_info(struct device *dev,
					    u8 info_type,
					    int *value)
{
	switch (info_type) {
	case AW_VOLTAGE_INFO:
		aw_monitor_get_chip_voltage(dev, value);
		break;
	case AW_TEMPERATURE_INFO:
		aw_monitor_get_chip_temperature(dev, value);
		break;
	default:
		AW_DEV_LOGE(dev, "unsupported type: %d", info_type);
		return -EINVAL;
	}

	return 0;
}

static int aw_monitor_get_sys_runtime_info(struct device *dev,
					   u8 info_type,
					   int *value)
{
	AW_DEV_LOGI(dev, "info_type: %s",
		    (info_type == AW_TEMPERATURE_INFO) ? "temperature info" :
		    ((info_type == AW_VOLTAGE_INFO) ? "voltage info" :
		    "capacity info"));

	switch (info_type) {
	case AW_VOLTAGE_INFO:
		aw_monitor_get_sys_voltage(dev, value);
		break;
	case AW_TEMPERATURE_INFO:
		aw_monitor_get_sys_temperature(dev, value);
		break;
	case AW_CAPACITY_INFO:
		aw_monitor_get_sys_capacity(dev, value);
		break;
	default:
		AW_DEV_LOGE(dev, "unsupported type: %d", info_type);
		return -EINVAL;
	}

	return 0;
}

static int aw_monitor_get_runtime_info(struct device *dev,
				       u8 src_type,
				       u8 info_type,
				       int *value)
{
	AW_DEV_LOGI(dev, "source type: %s",
		    src_type ? "platform info" : "chip info");

	switch (src_type) {
	case AW_CHIP_INFO:
		aw_monitor_get_chip_runtime_info(dev, info_type, value);
		break;
	case AW_PLATFORM_INFO:
		aw_monitor_get_sys_runtime_info(dev, info_type, value);
		break;
	default:
		AW_DEV_LOGE(dev, "unsupported type: %d", info_type);
		return -EINVAL;
	}

	return 0;
}

static int aw_search_vmax_from_table(struct device *dev,
				     struct aw_monitor *monitor,
				     const int vbat_vol,
				     int *vmax_vol)
{
	struct aw_monitor_header *monitor_hdr = &monitor->monitor_hdr;
	struct vmax_step_config *vmax_cfg = monitor->vmax_cfg;
	int i;

	if (monitor->bin_status == AW_MONITOR_CFG_WAIT) {
		AW_DEV_LOGE(dev, "vmax_cfg not loaded or parse failed");
		return -ENODATA;
	}

	if (vbat_vol == AW_VBAT_MAX) {
		*vmax_vol = AW_VMAX_MAX;
		AW_DEV_LOGD(dev, "vbat=%d, setting vmax=0x%x",
			    vbat_vol, *vmax_vol);
		return 0;
	}

	for (i = 0; i < monitor_hdr->step_count; i++) {
		if (vbat_vol >= vmax_cfg[i].vbat_min &&
		    vbat_vol < vmax_cfg[i].vbat_max) {
			*vmax_vol = vmax_cfg[i].vmax_vol;
			AW_DEV_LOGD(dev,
				    "read setting vmax=0x%x step[%d]: min=%u, max=%u",
				    *vmax_vol, i,
				    vmax_cfg[i].vbat_min,
				    vmax_cfg[i].vbat_max);
			return 0;
		}
	}

	AW_DEV_LOGE(dev, "vmax_cfg not found");
	return -ENODATA;
}

/*
 * monitor_esd_func
 */
static int aw_chip_status_recover(struct aw87xxx *aw87xxx)
{
	struct aw_monitor *monitor = &aw87xxx->monitor;
	char *profile = aw87xxx->current_profile;
	int ret;

	AW_DEV_LOGD(aw87xxx->dev, "enter");

	ret = aw87xxx_update_profile_esd(aw87xxx, profile);
	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev, "load profile[%s] failed", profile);
		return ret;
	}

	AW_DEV_LOGI(aw87xxx->dev, "current prof[%s], dev_index[%d]",
		    profile, aw87xxx->dev_index);

	monitor->pre_vmax = AW_VMAX_INIT_VAL;
	monitor->first_entry = AW_FIRST_ENTRY;
	monitor->timer_cnt = 0;
	monitor->vbat_sum = 0;

	return 0;
}

static int aw_monitor_chip_esd_check_work(struct aw87xxx *aw87xxx)
{
	int ret = 0;
	int i;

	for (i = 0; i < REG_STATUS_CHECK_MAX; i++) {
		AW_DEV_LOGD(aw87xxx->dev, "reg_status_check[%d]", i);

		ret = aw87xxx_dev_esd_reg_status_check(&aw87xxx->aw_dev);
		if (ret < 0) {
			if (ret == -EINVAL)
				aw_chip_status_recover(aw87xxx);
		} else {
			AW_DEV_LOGD(aw87xxx->dev, "chip status check succeed");
			break;
		}
		usleep_range(AW_ESD_CHECK_DELAY, AW_ESD_CHECK_DELAY_MAX);
	}

	return ret;
}

/*
 * aw87xxx monitor work with dsp
 */
static int aw_monitor_update_vmax_to_dsp(struct device *dev,
					 struct aw_monitor *monitor,
					 int vmax_set)
{
	u32 enable = 0;
	int ret;

	if (monitor->pre_vmax != vmax_set) {
		ret = aw87xxx_dsp_get_rx_module_enable(&enable);
		if (!enable || ret < 0) {
			AW_DEV_LOGE(dev,
				    "get rx failed or rx disable, ret=%d, enable=%d",
				    ret, enable);
			return -EPERM;
		}

		ret = aw87xxx_dsp_set_vmax(vmax_set, monitor->dev_index);
		if (ret) {
			AW_DEV_LOGE(dev, "set dsp msg fail, ret=%d", ret);
			return ret;
		}

		AW_DEV_LOGI(dev, "set dsp vmax=0x%x success", vmax_set);
		monitor->pre_vmax = vmax_set;
	} else {
		AW_DEV_LOGI(dev, "vmax=0x%x no change", vmax_set);
	}

	return 0;
}

static int aw_monitor_with_dsp_vmax_work(struct device *dev,
					  struct aw_monitor *monitor)
{
	struct aw_monitor_header *monitor_hdr = &monitor->monitor_hdr;
	int vbat_capacity = 0;
	int ave_capacity = 0;
	int vmax_set = 0;
	int ret;

	AW_DEV_LOGD(dev, "enter with dsp monitor");

	ret = aw_monitor_get_sys_capacity(dev, &vbat_capacity);
	if (ret < 0)
		return ret;

	if (monitor->timer_cnt < monitor_hdr->monitor_count) {
		monitor->timer_cnt++;
		monitor->vbat_sum += vbat_capacity;
		AW_DEV_LOGI(dev, "timer_cnt=%u", monitor->timer_cnt);
	}

	if (monitor->timer_cnt >= monitor_hdr->monitor_count ||
	    monitor->first_entry == AW_FIRST_ENTRY) {
		monitor->first_entry = AW_NOT_FIRST_ENTRY;
		ave_capacity = monitor->vbat_sum / monitor->timer_cnt;

		if (monitor->custom_capacity)
			ave_capacity = monitor->custom_capacity;

		AW_DEV_LOGI(dev, "get average capacity=%d", ave_capacity);

		ret = aw_search_vmax_from_table(dev,
						monitor,
						ave_capacity,
						&vmax_set);
		if (ret < 0) {
			AW_DEV_LOGE(dev, "not find vmax_vol");
		} else {
			ret = aw_monitor_update_vmax_to_dsp(dev, monitor, vmax_set);
			if (ret < 0)
				return ret;
		}

		monitor->timer_cnt = 0;
		monitor->vbat_sum = 0;
	}

	return 0;
}

static int aw_monitor_get_temp_and_vol(struct device *dev)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	int current_temp = 0;
	int voltage = 0;
	int ret;

	ret = aw_monitor_get_runtime_info(dev,
					  monitor->monitor_cfg.vol_source,
					  monitor->monitor_cfg.vol_mode,
					  &voltage);
	if (ret < 0)
		return ret;

	ret = aw_monitor_get_runtime_info(dev,
					  monitor->monitor_cfg.temp_source,
					  AW_TEMPERATURE_INFO,
					  &current_temp);
	if (ret < 0)
		return ret;

	monitor->vol_trace.sum_val += voltage;
	monitor->temp_trace.sum_val += current_temp;
	monitor->samp_count++;

	return 0;
}

static int aw_monitor_first_get_data_from_table(struct device *dev,
						struct aw_table_info table_info,
						struct aw_monitor_trace *trace)
{
	int i;

	if (!table_info.aw_table) {
		AW_DEV_LOGE(dev, "table_info.aw_table is null");
		return -EINVAL;
	}

	for (i = 0; i < table_info.table_num; i++) {
		if (trace->sum_val >= table_info.aw_table[i].min_val) {
			memcpy(&trace->aw_table, &table_info.aw_table[i],
			       sizeof(struct aw_table));
			break;
		}
	}

	return 0;
}

static int aw_monitor_trace_data_from_table(struct device *dev,
					    struct aw_table_info table_info,
					    struct aw_monitor_trace *trace)
{
	int i;

	if (!table_info.aw_table) {
		AW_DEV_LOGE(dev, "table_info.aw_table is null");
		return -EINVAL;
	}

	for (i = 0; i < table_info.table_num; i++) {
		if (trace->sum_val >= table_info.aw_table[i].min_val &&
		    trace->sum_val <= table_info.aw_table[i].max_val) {
			memcpy(&trace->aw_table, &table_info.aw_table[i],
			       sizeof(struct aw_table));
			break;
		}
	}

	return 0;
}

static int aw_monitor_get_data_from_table(struct device *dev,
					  struct aw_table_info table_info,
					  struct aw_monitor_trace *data_trace,
					  u32 aplha)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;

	if (monitor->first_entry == AW_FIRST_ENTRY)
		return aw_monitor_first_get_data_from_table(dev,
							    table_info,
							    data_trace);

	if (!monitor->samp_count) {
		AW_DEV_LOGE(dev, "monitor->samp_count: 0 unsupported");
		return -EINVAL;
	}

	data_trace->sum_val = data_trace->sum_val / monitor->samp_count;
	data_trace->sum_val = ((int)aplha * data_trace->sum_val +
			       (1000 - (int)aplha) * data_trace->pre_val) / 1000;

	return aw_monitor_trace_data_from_table(dev, table_info, data_trace);
}

static int aw_monitor_get_data(struct device *dev)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;
	struct aw_monitor_trace *vol_trace = &monitor->vol_trace;
	struct aw_monitor_trace *temp_trace = &monitor->temp_trace;
	int ret;

	if (monitor_cfg->vol_switch) {
		ret = aw_monitor_get_data_from_table(dev,
						     monitor_cfg->vol_info,
						     vol_trace,
						     monitor_cfg->vol_aplha);
		if (ret < 0)
			return ret;
	} else {
		vol_trace->aw_table.ipeak = IPEAK_NONE;
		vol_trace->aw_table.gain = GAIN_NONE;
		vol_trace->aw_table.vmax = VMAX_NONE;
	}

	if (monitor_cfg->temp_switch) {
		ret = aw_monitor_get_data_from_table(dev,
						     monitor_cfg->temp_info,
						     temp_trace,
						     monitor_cfg->temp_aplha);
		if (ret < 0)
			return ret;
	} else {
		temp_trace->aw_table.ipeak = IPEAK_NONE;
		temp_trace->aw_table.gain = GAIN_NONE;
		temp_trace->aw_table.vmax = VMAX_NONE;
	}

	AW_DEV_LOGD(dev,
		    "filter_vol: %d, vol: ipeak=0x%x, gain=0x%x, vmax=0x%x",
		    monitor->vol_trace.sum_val, vol_trace->aw_table.ipeak,
		    vol_trace->aw_table.gain, vol_trace->aw_table.vmax);

	AW_DEV_LOGD(dev,
		    "filter_temp: %d, temp: ipeak=0x%x, gain=0x%x, vmax=0x%x",
		    monitor->temp_trace.sum_val, temp_trace->aw_table.ipeak,
		    temp_trace->aw_table.gain, temp_trace->aw_table.vmax);

	return 0;
}

static void aw_monitor_get_cfg(struct device *dev, struct aw_table *set_table)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_table *temp_data = &monitor->temp_trace.aw_table;
	struct aw_table *vol_data = &monitor->vol_trace.aw_table;

	if (temp_data->ipeak == IPEAK_NONE && vol_data->ipeak == IPEAK_NONE) {
		memcpy(set_table, temp_data, sizeof(*set_table));
	} else if (temp_data->ipeak == IPEAK_NONE) {
		memcpy(set_table, vol_data, sizeof(*set_table));
	} else if (vol_data->ipeak == IPEAK_NONE) {
		memcpy(set_table, temp_data, sizeof(*set_table));
	} else {
		if (monitor->monitor_cfg.logic_switch == AW_MON_LOGIC_OR) {
			set_table->ipeak = min(temp_data->ipeak, vol_data->ipeak);
			set_table->gain = min(temp_data->gain, vol_data->gain);
			set_table->vmax = min(temp_data->vmax, vol_data->vmax);
		} else {
			set_table->ipeak = max(temp_data->ipeak, vol_data->ipeak);
			set_table->gain = max(temp_data->gain, vol_data->gain);
			set_table->vmax = max(temp_data->vmax, vol_data->vmax);
		}
	}
}

static void aw_monitor_set_cm_volt(struct device *dev)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_monitor_trace *vol_trace = &monitor->vol_trace;
	struct aw_cm_volt_desc *desc = &aw87xxx->aw_dev.cm_volt_desc;
	u8 set_val;
	int ret;

	if (desc->addr == AW_REG_NONE)
		return;

	set_val = (vol_trace->sum_val < desc->threshold) ? desc->adjust : desc->init;

	ret = aw87xxx_dev_i2c_write_bits(&aw87xxx->aw_dev,
					 desc->addr,
					 desc->mask,
					 set_val);
	if (ret < 0)
		AW_DEV_LOGE(dev, "write cm volt failed");
}

static void aw_monitor_set_ipeak(struct device *dev, u16 ipeak)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;
	struct aw_ipeak_desc *desc = &aw87xxx->aw_dev.ipeak_desc;
	int ret;

	if (desc->reg == AW_REG_NONE)
		return;

	if (ipeak == IPEAK_NONE || !monitor_cfg->ipeak_switch)
		return;

	ret = aw87xxx_dev_i2c_write_bits(&aw87xxx->aw_dev,
					 desc->reg,
					 desc->mask,
					 (u8)ipeak);
	if (ret < 0)
		AW_DEV_LOGE(dev, "write ipeak failed");
	else
		AW_DEV_LOGI(dev, "set ipeak=0x%x", ipeak);
}

static void aw_monitor_set_vmax(struct device *dev, u32 vmax)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;

	if (vmax == VMAX_NONE || !monitor_cfg->vmax_switch)
		return;

	aw_monitor_update_vmax_to_dsp(dev, monitor, vmax);
}

static void aw_monitor_with_dsp_work(struct device *dev)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_table set_table;
	int ret;

	ret = aw_monitor_get_temp_and_vol(dev);
	if (ret < 0)
		return;

	if (monitor->samp_count < monitor->monitor_cfg.monitor_count &&
	    monitor->first_entry == AW_NOT_FIRST_ENTRY)
		return;

	ret = aw_monitor_get_data(dev);
	if (ret < 0)
		return;

	aw_monitor_get_cfg(dev, &set_table);

	AW_DEV_LOGD(dev, "set_ipeak=0x%x, set_gain=0x%x, set_vmax=0x%x",
		    set_table.ipeak, set_table.gain, set_table.vmax);

	aw_monitor_set_cm_volt(dev);
	aw_monitor_set_ipeak(dev, set_table.ipeak);
	aw_monitor_set_vmax(dev, set_table.vmax);

	monitor->samp_count = 0;
	monitor->temp_trace.pre_val = monitor->temp_trace.sum_val;
	monitor->temp_trace.sum_val = 0;

	monitor->vol_trace.pre_val = monitor->vol_trace.sum_val;
	monitor->vol_trace.sum_val = 0;

	monitor->first_entry = AW_NOT_FIRST_ENTRY;
}

static void aw_monitor_work_func(struct work_struct *work)
{
	struct aw87xxx *aw87xxx = container_of(work, struct aw87xxx,
					       monitor.with_dsp_work.work);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct device *dev = aw87xxx->dev;
	u32 monitor_switch = 0;
	u32 monitor_time = 0;
	u32 monitor_count = 0;
	int ret;

	AW_DEV_LOGD(dev, "enter");

	if (aw87xxx->is_suspend)
		return;

	if (monitor->esd_enable &&
	    monitor->esd_err_cnt < AW_MONOTOR_ESD_ERR_CNT_MAX) {
		ret = aw_monitor_chip_esd_check_work(aw87xxx);
		if (ret < 0) {
			monitor->esd_err_cnt++;
			AW_DEV_LOGI(dev, "esd check failed");
		}
	}

	aw_monitor_get_ctrl_info(dev,
				 &monitor_switch,
				 &monitor_count,
				 &monitor_time);

	if (monitor_switch &&
	    !aw87xxx->aw_dev.is_rec_mode &&
	    monitor->bin_status == AW_MONITOR_CFG_OK) {
		AW_DEV_LOGD(dev, "monitor version 0x%x", monitor->version);
		switch (monitor->version) {
		case DATA_VERSION_V1:
			ret = aw_monitor_with_dsp_vmax_work(dev, monitor);
			break;
		case AW_MONITOR_HDR_VER_0_1_2:
			aw_monitor_with_dsp_work(dev);
			ret = 0;
			break;
		default:
			AW_DEV_LOGE(dev, "INVALID version: %d", monitor->version);
			return;
		}

		if (ret < 0) {
			AW_DEV_LOGE(dev,
				    "monitor dsp work failed, ret=%d, reschedule sooner",
				    ret);
			monitor_time = 100;
		}
	}

	if (monitor->esd_enable ||
	    (monitor_switch &&
	     !aw87xxx->aw_dev.is_rec_mode &&
	     monitor->bin_status == AW_MONITOR_CFG_OK)) {
		schedule_delayed_work(&monitor->with_dsp_work,
				      msecs_to_jiffies(monitor_time));
	}
}

void aw87xxx_monitor_stop(struct aw_monitor *monitor)
{
	struct aw87xxx *aw87xxx = container_of(monitor, struct aw87xxx,
					       monitor);

	AW_DEV_LOGD(aw87xxx->dev, "enter");
	cancel_delayed_work_sync(&monitor->with_dsp_work);
}

void aw87xxx_monitor_start(struct aw_monitor *monitor)
{
	struct aw87xxx *aw87xxx = container_of(monitor, struct aw87xxx,
					       monitor);
	u32 monitor_switch = 0;
	u32 monitor_count = 0;
	u32 monitor_time = 0;
	int ret;

	ret = aw87xxx_dev_check_reg_is_rec_mode(&aw87xxx->aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev, "get reg current mode failed");
		return;
	}

	aw_monitor_get_ctrl_info(aw87xxx->dev,
				 &monitor_switch,
				 &monitor_count,
				 &monitor_time);

	AW_DEV_LOGI(aw87xxx->dev,
		    "esd_en: %d, is_rec_mode: %d",
		    monitor->esd_enable,
		    aw87xxx->aw_dev.is_rec_mode);
	AW_DEV_LOGI(aw87xxx->dev,
		    "switch: %u, count: %u, time: %u",
		    monitor_switch, monitor_count, monitor_time);

	if (monitor->esd_enable ||
	    (monitor_switch && !aw87xxx->aw_dev.is_rec_mode &&
	     monitor->bin_status == AW_MONITOR_CFG_OK)) {
		AW_DEV_LOGD(aw87xxx->dev, "enter");
		monitor->pre_vmax = AW_VMAX_INIT_VAL;
		monitor->first_entry = AW_FIRST_ENTRY;
		monitor->timer_cnt = 0;
		monitor->vbat_sum = 0;
		monitor->esd_err_cnt = 0;

		schedule_delayed_work(&monitor->with_dsp_work,
				      msecs_to_jiffies(50));
	}
}

/*
 * aw87xxx no dsp monitor func
 */
static int aw87xxx_hal_get_vmax(struct device *dev, int *vmax)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_table set_table;
	int ret;

	monitor->first_entry = AW_FIRST_ENTRY;
	monitor->samp_count = 0;
	monitor->temp_trace.sum_val = 0;
	monitor->temp_trace.pre_val = 0;
	monitor->vol_trace.sum_val = 0;
	monitor->vol_trace.pre_val = 0;

	ret = aw_monitor_get_temp_and_vol(dev);
	if (ret < 0)
		return ret;

	ret = aw_monitor_get_data(dev);
	if (ret < 0)
		return ret;

	aw_monitor_get_cfg(dev, &set_table);

	*vmax = set_table.vmax;

	return 0;
}

int aw87xxx_monitor_no_dsp_get_vmax(struct aw_monitor *monitor, int *vmax)
{
	struct aw87xxx *aw87xxx = container_of(monitor, struct aw87xxx,
					       monitor);
	struct device *dev = aw87xxx->dev;
	int vbat_capacity = 0;
	int vmax_vol = 0;
	int ret;

	if (monitor->version == AW_MONITOR_HDR_VER_0_1_2)
		return aw87xxx_hal_get_vmax(dev, vmax);

	ret = aw_monitor_get_sys_capacity(dev, &vbat_capacity);
	if (ret < 0)
		return ret;

	if (monitor->custom_capacity)
		vbat_capacity = monitor->custom_capacity;

	AW_DEV_LOGI(dev, "get_battery_capacity is [%d]", vbat_capacity);

	ret = aw_search_vmax_from_table(dev,
					monitor,
					vbat_capacity,
					&vmax_vol);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "not find vmax_vol");
		return ret;
	}

	*vmax = vmax_vol;
	return 0;
}

/*
 * aw87xxx monitor sysfs nodes
 */
static ssize_t esd_enable_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;

	AW_DEV_LOGI(aw87xxx->dev, "esd-enable=%s",
		    monitor->esd_enable ? "true" : "false");

	return sysfs_emit(buf, "esd-enable=%s\n",
			  monitor->esd_enable ? "true" : "false");
}

static ssize_t esd_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	bool enable;

	if (kstrtobool(buf, &enable)) {
		AW_DEV_LOGE(aw87xxx->dev, "input esd-enable error");
		return -EINVAL;
	}

	AW_DEV_LOGD(aw87xxx->dev, "input esd-enable=[%s]",
		    enable ? "true" : "false");

	monitor->esd_enable = enable ? AW_ESD_ENABLE : AW_ESD_DISABLE;
	AW_DEV_LOGI(dev, "set esd-enable=[%s]",
		    monitor->esd_enable ? "true" : "false");

	return len;
}

static ssize_t vbat_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	int vbat_capacity = 0;
	int ret;

	if (!monitor->custom_capacity) {
		ret = aw_monitor_get_sys_capacity(dev, &vbat_capacity);
		if (ret < 0) {
			AW_DEV_LOGE(aw87xxx->dev,
				    "get battery_capacity failed");
			return ret;
		}
		return sysfs_emit(buf, "vbat capacity=%d\n", vbat_capacity);
	}

	return sysfs_emit(buf, "vbat capacity=%d\n", monitor->custom_capacity);
}

static ssize_t vbat_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	u32 capacity;
	int ret;

	ret = kstrtouint(buf, 0, &capacity);
	if (ret < 0)
		return ret;

	AW_DEV_LOGI(aw87xxx->dev, "set capacity=%u", capacity);

	if (capacity >= AW_VBAT_CAPACITY_MIN &&
	    capacity <= AW_VBAT_CAPACITY_MAX) {
		monitor->custom_capacity = capacity;
	} else {
		AW_DEV_LOGE(aw87xxx->dev,
			    "vbat_set invalid, input value [%d-%d]",
			    AW_VBAT_CAPACITY_MIN, AW_VBAT_CAPACITY_MAX);
		return -EINVAL;
	}

	return len;
}

static ssize_t vmax_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	int vmax_get = 0;
	int ret;

	ret = aw87xxx_dsp_get_vmax(&vmax_get, aw87xxx->dev_index);
	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev,
			    "get dsp vmax fail, ret=%d", ret);
		return ret;
	}
	return sysfs_emit(buf, "get_vmax=%d\n", vmax_get);
}

static ssize_t vmax_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	u32 vmax_set;
	int ret;

	ret = kstrtouint(buf, 0, &vmax_set);
	if (ret < 0)
		return ret;

	AW_DEV_LOGI(aw87xxx->dev, "vmax_set=0x%x", vmax_set);

	ret = aw87xxx_dsp_set_vmax(vmax_set, aw87xxx->dev_index);
	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev,
			    "send dsp_msg error, ret=%d", ret);
		return ret;
	}
	usleep_range(2000, 2010);

	return count;
}

static ssize_t monitor_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	u32 monitor_switch = 0;
	u32 monitor_count = 0;
	u32 monitor_time = 0;

	aw_monitor_get_ctrl_info(dev,
				 &monitor_switch,
				 &monitor_count,
				 &monitor_time);

	return sysfs_emit(buf, "aw87xxx monitor switch: %u\n", monitor_switch);
}

int aw87xxx_dev_monitor_switch_set(struct aw_monitor *monitor, u32 enable)
{
	struct aw87xxx *aw87xxx = container_of(monitor, struct aw87xxx,
					       monitor);
	struct aw_monitor_header *monitor_hdr = &monitor->monitor_hdr;
	struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;

	AW_DEV_LOGI(aw87xxx->dev, "monitor switch set=%u", enable);

	if (monitor->bin_status == AW_MONITOR_CFG_WAIT) {
		AW_DEV_LOGE(aw87xxx->dev,
			    "bin parse failed or not loaded, set invalid");
		return -EINVAL;
	}

	if (enable) {
		monitor_hdr->monitor_switch = 1;
		monitor_cfg->monitor_switch = 1;
		monitor->pre_vmax = AW_VMAX_INIT_VAL;
		monitor->first_entry = AW_FIRST_ENTRY;
		monitor->timer_cnt = 0;
		monitor->vbat_sum = 0;
	} else {
		monitor_hdr->monitor_switch = 0;
		monitor_cfg->monitor_switch = 0;
	}

	return 0;
}

static ssize_t monitor_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	u32 enable;
	int ret;

	ret = kstrtouint(buf, 0, &enable);
	if (ret < 0)
		return ret;

	ret = aw87xxx_dev_monitor_switch_set(monitor, enable);
	if (ret)
		return ret;

	return count;
}

static ssize_t monitor_time_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	u32 monitor_switch = 0;
	u32 monitor_count = 0;
	u32 monitor_time = 0;

	aw_monitor_get_ctrl_info(dev,
				 &monitor_switch,
				 &monitor_count,
				 &monitor_time);

	return sysfs_emit(buf, "aw_monitor_timer=%u(ms)\n", monitor_time);
}

static ssize_t monitor_time_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_monitor_header *monitor_hdr = &monitor->monitor_hdr;
	struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;
	u32 timer_val;
	int ret;

	ret = kstrtouint(buf, 0, &timer_val);
	if (ret < 0)
		return ret;

	AW_DEV_LOGI(aw87xxx->dev, "input monitor timer=%u(ms)", timer_val);

	if (monitor->bin_status == AW_MONITOR_CFG_WAIT) {
		AW_DEV_LOGE(aw87xxx->dev,
			    "bin parse failed or not loaded, set invalid");
		return -EINVAL;
	}

	monitor_hdr->monitor_time = timer_val;
	monitor_cfg->monitor_time = timer_val;

	return count;
}

static ssize_t monitor_count_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	u32 monitor_switch = 0;
	u32 monitor_count = 0;
	u32 monitor_time = 0;

	aw_monitor_get_ctrl_info(dev,
				 &monitor_switch,
				 &monitor_count,
				 &monitor_time);

	return sysfs_emit(buf, "aw_monitor_count=%u\n", monitor_count);
}

static ssize_t monitor_count_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_monitor_header *monitor_hdr = &monitor->monitor_hdr;
	struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;
	u32 monitor_count;
	int ret;

	ret = kstrtouint(buf, 0, &monitor_count);
	if (ret < 0)
		return ret;

	AW_DEV_LOGI(aw87xxx->dev, "input monitor count=%u", monitor_count);

	if (monitor->bin_status == AW_MONITOR_CFG_WAIT) {
		AW_DEV_LOGE(aw87xxx->dev,
			    "bin parse failed or not loaded, set invalid");
		return -EINVAL;
	}

	monitor_hdr->monitor_count = monitor_count;
	monitor_cfg->monitor_count = monitor_count;

	return count;
}

static ssize_t rx_show(struct device *dev,
		       struct device_attribute *attr,
		       char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	u32 enable = 0;
	int ret;

	ret = aw87xxx_dsp_get_rx_module_enable(&enable);
	if (ret) {
		AW_DEV_LOGE(aw87xxx->dev, "dsp_msg error, ret=%d", ret);
		return ret;
	}

	return sysfs_emit(buf, "aw87xxx rx: %u\n", enable);
}

static ssize_t rx_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	u32 enable;
	int ret;

	ret = kstrtouint(buf, 0, &enable);
	if (ret < 0)
		return ret;

	AW_DEV_LOGI(aw87xxx->dev, "set rx enable=%u", enable);

	ret = aw87xxx_dsp_set_rx_module_enable(enable);
	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev, "dsp_msg error, ret=%d", ret);
		return ret;
	}

	return count;
}

static ssize_t monitor_params_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *monitor = &aw87xxx->monitor;
	struct aw_container *monitor_cont = monitor->monitor_container;
	int at = 0;
	int i;

	if (!monitor_cont || monitor->bin_status == AW_MONITOR_CFG_WAIT)
		return sysfs_emit(buf, "0\n");

	at += sysfs_emit_at(buf, at, "1 ");

	for (i = 0; i < monitor_cont->len; i++) {
		int written = sysfs_emit_at(buf, at, "%02x,", monitor_cont->data[i]);
		if (!written)
			break;
		at += written;
	}

	at += sysfs_emit_at(buf, at, "\n");

	return at;
}

static int aw_monitor_real_time_update_monitor(struct device *dev)
{
	const struct firmware *cont = NULL;
	int ret;

	ret = request_firmware(&cont, AW87XXX_MONITOR_NAME, dev);
	if (ret < 0) {
		AW_DEV_LOGE(dev, "failed to read %s", AW87XXX_MONITOR_NAME);
		release_firmware(cont);
		return ret;
	}

	ret = aw87xxx_monitor_bin_parse(dev, cont->data, (u32)cont->size);
	if (ret < 0)
		AW_DEV_LOGE(dev, "parse monitor firmware failed!");

	release_firmware(cont);

	return ret;
}

void aw87xxx_monitor_cfg_free(struct aw_monitor *monitor)
{
	struct aw_monitor_cfg *monitor_cfg = &monitor->monitor_cfg;

	aw_monitor_cfg_v_0_0_1_free(monitor);

	kfree(monitor_cfg->temp_info.aw_table);
	kfree(monitor_cfg->vol_info.aw_table);
	kfree(monitor->monitor_container);

	memset(monitor_cfg, 0, sizeof(*monitor_cfg));
}

static ssize_t monitor_update_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *aw_monitor = &aw87xxx->monitor;

	if (aw_monitor->version == AW_MONITOR_DATA_VER) {
		AW_DEV_LOGE(dev, "unsupported monitor version");
		return sysfs_emit(buf, "0\n");
	}

	if (aw_monitor->bin_status == AW_MONITOR_CFG_WAIT)
		return sysfs_emit(buf, "0\n");
	else if (aw_monitor->bin_status == AW_MONITOR_CFG_OK)
		return sysfs_emit(buf, "1\n");

	AW_DEV_LOGE(dev, "unsupported bin_status");
	return 0;
}

static ssize_t monitor_update_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_monitor *aw_monitor = &aw87xxx->monitor;
	u32 update;
	int ret;

	ret = kstrtouint(buf, 0, &update);
	if (ret < 0)
		return ret;

	AW_DEV_LOGI(dev, "monitor update=%u", update);

	if (update) {
		aw87xxx_monitor_stop(aw_monitor);
		aw87xxx_monitor_cfg_free(aw_monitor);

		ret = aw_monitor_real_time_update_monitor(dev);
		if (ret < 0)
			return ret;

		aw87xxx_monitor_start(aw_monitor);
	}

	return count;
}

static DEVICE_ATTR_RW(esd_enable);
static DEVICE_ATTR_RW(vbat);
static DEVICE_ATTR_RW(vmax);
static DEVICE_ATTR_RW(monitor);
static DEVICE_ATTR_RW(monitor_time);
static DEVICE_ATTR_RW(monitor_count);
static DEVICE_ATTR_RW(rx);
static DEVICE_ATTR_RW(monitor_update);
static DEVICE_ATTR_RO(monitor_params);

static struct attribute *aw_monitor_vol_adjust[] = {
	&dev_attr_esd_enable.attr,
	&dev_attr_vbat.attr,
	&dev_attr_vmax.attr,
	NULL
};

static const struct attribute_group aw_monitor_vol_adjust_group = {
	.attrs = aw_monitor_vol_adjust,
};

static struct attribute *aw_monitor_control[] = {
	&dev_attr_monitor.attr,
	&dev_attr_monitor_time.attr,
	&dev_attr_monitor_count.attr,
	&dev_attr_rx.attr,
	&dev_attr_monitor_update.attr,
	&dev_attr_monitor_params.attr,
	NULL
};

static const struct attribute_group aw_monitor_control_group = {
	.attrs = aw_monitor_control,
};

/*
 * aw87xxx monitor init
 */
static void aw_monitor_dtsi_parse(struct device *dev,
				  struct aw_monitor *monitor,
				  struct device_node *dev_node)
{
	const char *esd_enable;
	int ret;

	ret = of_property_read_string(dev_node, "esd-enable", &esd_enable);
	if (ret < 0) {
		AW_DEV_LOGI(dev, "esd_enable parse failed, default[disable]");
		monitor->esd_enable = AW_ESD_DISABLE;
	} else {
		if (!strcmp(esd_enable, "false"))
			monitor->esd_enable = AW_ESD_DISABLE;
		else
			monitor->esd_enable = AW_ESD_ENABLE;

		AW_DEV_LOGI(dev, "parse esd-enable=[%s]",
			    monitor->esd_enable ? "true" : "false");
	}
}

void aw87xxx_monitor_init(struct device *dev,
			  struct aw_monitor *monitor,
			  struct device_node *dev_node)
{
	struct aw87xxx *aw87xxx = container_of(monitor, struct aw87xxx,
					       monitor);
	int ret;

	monitor->dev_index = aw87xxx->dev_index;
	monitor->monitor_hdr.monitor_time = AW_DEFAULT_MONITOR_TIME;

	aw_monitor_dtsi_parse(dev, monitor, dev_node);

	ret = sysfs_create_group(&dev->kobj,
				 &aw_monitor_vol_adjust_group);
	if (ret < 0)
		AW_DEV_LOGE(dev,
			    "failed to create monitor vol_adjust sysfs");

	INIT_DELAYED_WORK(&monitor->with_dsp_work, aw_monitor_work_func);

	ret = sysfs_create_group(&dev->kobj,
				 &aw_monitor_control_group);
	if (ret < 0)
		AW_DEV_LOGE(dev,
			    "failed to create dsp control sysfs");

	if (!ret)
		AW_DEV_LOGI(dev, "monitor init succeed");
}

void aw87xxx_monitor_exit(struct aw_monitor *monitor)
{
	struct aw87xxx *aw87xxx = container_of(monitor, struct aw87xxx,
					       monitor);

	aw87xxx_monitor_stop(monitor);

	/* rm attr node */
	sysfs_remove_group(&aw87xxx->dev->kobj,
			    &aw_monitor_vol_adjust_group);
	sysfs_remove_group(&aw87xxx->dev->kobj,
			   &aw_monitor_control_group);
}
