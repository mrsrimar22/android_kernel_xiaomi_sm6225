// SPDX-License-Identifier: GPL-2.0
/*
 * aw87xxx_dsp.c
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <dsp/q6afe-v2.h>

#include "aw87xxx_dsp.h"
#include "aw87xxx_log.h"

static DEFINE_MUTEX(g_dsp_lock);
static u32 g_spin_value;

static u32 g_rx_topo_id;
static u32 g_rx_port_id;

static bool aw_check_dsp_ready(void)
{
	int i, ret;

	for (i = 0; i < 30; i++) {
		ret = afe_get_topology(g_rx_port_id);
		if (ret < 0) {
			AW_LOGE("get topology failed: %d", ret);
			return false;
		}
		if ((u32)ret == g_rx_topo_id)
			return true;
		if (i < 29)
			usleep_range(2000, 2500);
	}

	AW_LOGE("topo id mismatch: 0x%x (expected 0x%x)", ret, g_rx_topo_id);
	return false;
}

static int aw_write_data_to_dsp(int param_id, void *data, int data_size)
{
	int ret = -EINVAL;

	mutex_lock(&g_dsp_lock);
	if (aw_check_dsp_ready())
		ret = aw_send_afe_cal_apr(param_id, data, data_size, true);
	else
		AW_LOGE("DSP not ready, write aborted");
	mutex_unlock(&g_dsp_lock);
	return ret;
}

static int aw_read_data_from_dsp(int param_id, void *data, int data_size)
{
	int ret = -EINVAL;

	mutex_lock(&g_dsp_lock);
	if (aw_check_dsp_ready())
		ret = aw_send_afe_cal_apr(param_id, data, data_size, false);
	else
		AW_LOGE("DSP not ready, read aborted");
	mutex_unlock(&g_dsp_lock);
	return ret;
}

int aw87xxx_dsp_get_rx_module_enable(int *enable)
{
	if (!enable) {
		AW_LOGE("enable is NULL");
		return -EINVAL;
	}

	return aw_read_data_from_dsp(AWDSP_RX_SET_ENABLE,
				     enable,
				     sizeof(*enable));
}

int aw87xxx_dsp_set_rx_module_enable(int enable)
{
	switch (enable) {
	case AW_RX_MODULE_DISENABLE:
	case AW_RX_MODULE_ENABLE:
		AW_LOGD("set enable=%d", enable);
		break;
	default:
		AW_LOGE("unsupport enable=%d", enable);
		return -EINVAL;
	}

	return aw_write_data_to_dsp(AWDSP_RX_SET_ENABLE,
				    &enable,
				    sizeof(enable));
}

int aw87xxx_dsp_get_vmax(u32 *vmax, int dev_index)
{
	int param_id;

	switch (dev_index % AW_DSP_CHANNEL_MAX) {
	case AW_DSP_CHANNEL_0:
		param_id = AWDSP_RX_VMAX_0;
		break;
	case AW_DSP_CHANNEL_1:
		param_id = AWDSP_RX_VMAX_1;
		break;
	default:
		AW_LOGE("algo only support double PA, channel: %d unsupport",
			dev_index);
		return -EINVAL;
	}

	return aw_read_data_from_dsp(param_id, vmax, sizeof(*vmax));
}

int aw87xxx_dsp_set_vmax(u32 vmax, int dev_index)
{
	int param_id;

	switch (dev_index % AW_DSP_CHANNEL_MAX) {
	case AW_DSP_CHANNEL_0:
		param_id = AWDSP_RX_VMAX_0;
		break;
	case AW_DSP_CHANNEL_1:
		param_id = AWDSP_RX_VMAX_1;
		break;
	default:
		AW_LOGE("algo only support double PA, channel: %d unsupport",
			dev_index);
		return -EINVAL;
	}

	return aw_write_data_to_dsp(param_id, &vmax, sizeof(vmax));
}

int aw87xxx_dsp_set_spin(u32 ctrl_value)
{
	int ret;

	if (ctrl_value >= AW_SPIN_MAX) {
		AW_LOGE("spin[%d] unsupported", ctrl_value);
		return -EINVAL;
	}

	ret = aw_write_data_to_dsp(AW_MSG_ID_SPIN,
				   &ctrl_value,
				   sizeof(ctrl_value));
	if (ret) {
		AW_LOGE("spin[%d] set failed", ctrl_value);
		return ret;
	}

	g_spin_value = ctrl_value;

	return 0;
}

int aw87xxx_dsp_get_spin(void)
{
	return g_spin_value;
}

void aw87xxx_dsp_parse_topo_id_dt(struct aw_device *aw_dev)
{
	int ret;

	ret = of_property_read_u32(aw_dev->dev->of_node,
				   "aw-rx-topo-id",
				   &g_rx_topo_id);
	if (ret < 0) {
		g_rx_topo_id = AW_RX_DEFAULT_TOPO_ID;
		AW_DEV_LOGI(aw_dev->dev,
			    "read aw-rx-topo-id failed, use default");
	}

	AW_DEV_LOGI(aw_dev->dev, "rx-topo-id: 0x%x", g_rx_topo_id);
}

void aw87xxx_dsp_parse_port_id_dt(struct aw_device *aw_dev)
{
	int ret;

	ret = of_property_read_u32(aw_dev->dev->of_node,
				   "aw-rx-port-id",
				   &g_rx_port_id);
	if (ret < 0) {
		g_rx_port_id = AW_RX_DEFAULT_PORT_ID;
		AW_DEV_LOGI(aw_dev->dev,
			    "read aw-rx-port-id failed, use default");
	}

	AW_DEV_LOGI(aw_dev->dev, "rx-port-id: 0x%x", g_rx_port_id);
}
