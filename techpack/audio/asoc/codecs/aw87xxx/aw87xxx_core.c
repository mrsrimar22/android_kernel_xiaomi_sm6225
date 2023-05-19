// SPDX-License-Identifier: GPL-2.0
/*
 * aw87xxx.c  aw87xxx pa module
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 */

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <uapi/sound/asound.h>
#include <asoc/aw87xxx_api.h>
#include <asoc/msm-cdc-pinctrl.h>

#include "aw87xxx_core.h"
#include "aw87xxx_acf_bin.h"
#include "aw87xxx_bin_parse.h"
#include "aw87xxx_device.h"
#include "aw87xxx_dsp.h"
#include "aw87xxx_log.h"
#include "aw87xxx_monitor.h"

#define AW87XXX_I2C_NAME	"aw87xxx_pa"
#define AW87XXX_DRIVER_VERSION	"v4.0.9"
#define AW87XXX_FW_BIN_NAME	"aw87xxx_acf.bin"

DEFINE_STATIC_SRCU(g_aw87xxx_srcu);
static LIST_HEAD(g_aw87xxx_list);
static DEFINE_MUTEX(g_aw87xxx_list_lock);
static atomic_t g_aw87xxx_dev_cnt = ATOMIC_INIT(0);

static const char * const aw87xxx_monitor_switch[] = {
	"Disable", "Enable"
};

static const char * const aw87xxx_spin_switch[] = {
	"spin_0", "spin_90", "spin_180", "spin_270"
};

static struct aw87xxx *aw87xxx_get_by_index(int dev_index)
{
	struct aw87xxx *aw87xxx;

	list_for_each_entry_rcu(aw87xxx, &g_aw87xxx_list, node) {
		if (aw87xxx->dev_index == dev_index)
			return aw87xxx;
	}

	return NULL;
}

static void aw87xxx_update_voltage_max(struct aw87xxx *aw87xxx,
				       struct aw_data_container *data_container)
{
	struct aw_voltage_desc *vol_desc = &aw87xxx->aw_dev.vol_desc;
	int i;

	if (!data_container ||
	    data_container->len <= 2 ||
	    vol_desc->addr == AW_REG_NONE)
		return;

	for (i = 0; i < data_container->len; i += 2) {
		if (data_container->data[i] == vol_desc->addr) {
			vol_desc->vol_max = (data_container->data[i + 1] &
					     ~vol_desc->mask) >> vol_desc->start;
			AW_DEV_LOGD(aw87xxx->dev, "get voltage max=0x%02x",
				    vol_desc->vol_max);
			return;
		}
	}
}

static void aw87xxx_set_current_profile(struct aw87xxx *aw87xxx,
					const char *name)
{
	strscpy(aw87xxx->current_profile_buf, name, AW_PROFILE_STR_MAX);
}

static const struct {
	u8 reg;
	u8 val;
} g_aw87xxx_preset_off[] = {
	{ AW87XXX_PID_5A_REG_SYSCTRL_REG, 0x00 },
};

/* Receiver (Earpiece Call Mode) */
static const struct {
	u8 reg;
	u8 val;
} g_aw87xxx_preset_receiver[] = {
	{ AW87XXX_PID_5A_REG_DFT3R_REG, 0xB5 },
	{ AW87XXX_PID_5A_REG_test3_REG, 0x39 },
	{ AW87XXX_PID_5A_REG_test4_REG, 0xE5 },
	{ AW87XXX_PID_5A_REG_ENCR_REG, 0xC1 },
	{ AW87XXX_PID_5A_REG_ENCR_REG, 0xC1 },
	{ AW87XXX_PID_5A_REG_test3_REG, 0x7A },
	{ AW87XXX_PID_5A_REG_test4_REG, 0x6C },
	{ AW87XXX_PID_5A_REG_ENCR_REG, 0x81 },
	{ AW87XXX_PID_5A_REG_DFT7R_REG, 0x69 },
	{ AW87XXX_PID_5A_REG_SYSST_REG, 0xBC },
	{ AW87XXX_PID_5A_REG_BATSAFE_REG, 0x08 }, /* BATSAFE: 1ms, VTH=3.4V, EN=0, Level=5.0V */
	{ AW87XXX_PID_5A_REG_BSTOVR_REG, 0x0A }, /* BSTOVR */
	{ AW87XXX_PID_5A_REG_BSTCPR1_REG, 0x80 }, /* BSTCPR1 */
	{ AW87XXX_PID_5A_REG_BSTCPR2_REG, 0x08 }, /* BSTCPR2 */
	{ AW87XXX_PID_5A_REG_PAGR_REG, 0x00 }, /* PAGR */
	{ AW87XXX_PID_5A_REG_PAGC3OPR_REG, 0x93 }, /* PAGC3OPR */
	{ AW87XXX_PID_5A_REG_PAGC3PR_REG, 0x4E }, /* PAGC3PR */
	{ AW87XXX_PID_5A_REG_PAGC2OPR_REG, 0x0B }, /* PAGC2OPR */
	{ AW87XXX_PID_5A_REG_PAGC2PR_REG, 0x08 }, /* PAGC2PR */
	{ AW87XXX_PID_5A_REG_PAGC1PR_REG, 0x4B }, /* PAGC1PR */
	{ AW87XXX_PID_5A_REG_ADP_MODE_REG, 0x00 }, /* ADP_MODE */
	{ AW87XXX_PID_5A_REG_ADPBST_TIME1_REG, 0x77 }, /* ADPBST_TIME1 */
	{ AW87XXX_PID_5A_REG_ADPBST_TIME2_REG, 0x7A }, /* ADPBST_TIME2 */
	{ AW87XXX_PID_5A_REG_ADPBST_VTH_REG, 0x51 }, /* ADPBST_VTH */
	{ AW87XXX_PID_5A_REG_BOOST_PAR_REG, 0x40 }, /* BOOST_PAR */
	{ AW87XXX_PID_5A_REG_DFT1R_REG, 0x26 },
	{ AW87XXX_PID_5A_REG_DFT2R_REG, 0x15 },
	{ AW87XXX_PID_5A_REG_DFT4R_REG, 0x5A },
	{ AW87XXX_PID_5A_REG_DFT5R_REG, 0xD5 },
	{ AW87XXX_PID_5A_REG_DFT6R_REG, 0x57 },
	{ AW87XXX_PID_5A_REG_DFT8R_REG, 0x28 },
	{ AW87XXX_PID_5A_REG_DFT9R_REG, 0x22 },
	{ AW87XXX_PID_5A_REG_DFTAR_REG, 0xA4 },
	{ AW87XXX_PID_5A_REG_DFTBR_REG, 0x1C },
	{ AW87XXX_PID_5A_REG_DFTCR_REG, 0x9C },
	{ AW87XXX_PID_5A_REG_DFTDR_REG, 0xB3 },
	{ AW87XXX_PID_5A_REG_DFTER_REG, 0x44 },
	{ AW87XXX_PID_5A_REG_DFTFR_REG, 0x6C },
	{ AW87XXX_PID_5A_REG_SYSCTRL_REG, 0x7C }, /* SYSCTRL */
	{ AW_REG_NONE, 0x20 },
};

/* Base Speaker Preset Profile (PID 5A) */
static const struct {
	u8 reg;
	u8 val;
} g_aw87xxx_preset_speaker_base[] = {
	{ AW87XXX_PID_5A_REG_DFT3R_REG, 0xB5 },
	{ AW87XXX_PID_5A_REG_test3_REG, 0x39 },
	{ AW87XXX_PID_5A_REG_test4_REG, 0xE5 },
	{ AW87XXX_PID_5A_REG_ENCR_REG, 0xC1 },
	{ AW87XXX_PID_5A_REG_ENCR_REG, 0xC1 },
	{ AW87XXX_PID_5A_REG_test3_REG, 0x7A },
	{ AW87XXX_PID_5A_REG_test4_REG, 0x6C },
	{ AW87XXX_PID_5A_REG_ENCR_REG, 0x81 },
	{ AW87XXX_PID_5A_REG_DFT7R_REG, 0x69 },
	{ AW87XXX_PID_5A_REG_SYSST_REG, 0xBC },
	{ AW87XXX_PID_5A_REG_BATSAFE_REG, 0x4C }, /* BATSAFE: 200us, VTH=3.4V (Voice=0x5C/3.6V) */
	{ AW87XXX_PID_5A_REG_BSTOVR_REG, 0x0A }, /* BSTOVR: Dev0=9.0V (Dev1=0x04/7.5V) */
	{ AW87XXX_PID_5A_REG_BSTCPR1_REG, 0x80 }, /* BSTCPR1 */
	{ AW87XXX_PID_5A_REG_BSTCPR2_REG, 0x08 }, /* BSTCPR2 */
	{ AW87XXX_PID_5A_REG_PAGR_REG, 0x09 }, /* PAGR (13.5dB Gain) */
	{ AW87XXX_PID_5A_REG_PAGC3OPR_REG, 0x83 }, /* PAGC3OPR */
	{ AW87XXX_PID_5A_REG_PAGC3PR_REG, 0x4E }, /* PAGC3PR */
	{ AW87XXX_PID_5A_REG_PAGC2OPR_REG, 0x03 }, /* PAGC2OPR */
	{ AW87XXX_PID_5A_REG_PAGC2PR_REG, 0x08 }, /* PAGC2PR */
	{ AW87XXX_PID_5A_REG_PAGC1PR_REG, 0x4A }, /* PAGC1PR */
	{ AW87XXX_PID_5A_REG_ADP_MODE_REG, 0x03 }, /* ADP_MODE */
	{ AW87XXX_PID_5A_REG_ADPBST_TIME1_REG, 0x77 }, /* ADPBST_TIME1 */
	{ AW87XXX_PID_5A_REG_ADPBST_TIME2_REG, 0x7A }, /* ADPBST_TIME2 */
	{ AW87XXX_PID_5A_REG_ADPBST_VTH_REG, 0x51 }, /* ADPBST_VTH */
	{ AW87XXX_PID_5A_REG_BOOST_PAR_REG, 0x58 }, /* BOOST_PAR */
	{ AW87XXX_PID_5A_REG_DFT1R_REG, 0x26 },
	{ AW87XXX_PID_5A_REG_DFT2R_REG, 0x15 },
	{ AW87XXX_PID_5A_REG_DFT4R_REG, 0x5A },
	{ AW87XXX_PID_5A_REG_DFT5R_REG, 0xD5 },
	{ AW87XXX_PID_5A_REG_DFT6R_REG, 0x57 },
	{ AW87XXX_PID_5A_REG_DFT8R_REG, 0x28 },
	{ AW87XXX_PID_5A_REG_DFT9R_REG, 0x35 },
	{ AW87XXX_PID_5A_REG_DFTAR_REG, 0x98 },
	{ AW87XXX_PID_5A_REG_DFTBR_REG, 0x1C },
	{ AW87XXX_PID_5A_REG_DFTCR_REG, 0x9C },
	{ AW87XXX_PID_5A_REG_DFTDR_REG, 0x33 },
	{ AW87XXX_PID_5A_REG_DFTER_REG, 0x40 },
	{ AW87XXX_PID_5A_REG_DFTFR_REG, 0x6C },
	{ AW87XXX_PID_5A_REG_SYSCTRL_REG, 0x78 }, /* SYSCTRL */
	{ AW_REG_NONE, 0x40 },
};

static int aw87xxx_get_preset_profile(struct aw87xxx *aw87xxx,
				      const char *name,
				      struct aw_data_container *container)
{
	int i;

	if (!strncasecmp(name, "Off", AW_PROFILE_STR_MAX)) {
		container->data = (u8 *)g_aw87xxx_preset_off;
		container->len = sizeof(g_aw87xxx_preset_off);
		return 0;
	}

	if (!strncasecmp(name, "Receiver", AW_PROFILE_STR_MAX)) {
		container->data = (u8 *)g_aw87xxx_preset_receiver;
		container->len = sizeof(g_aw87xxx_preset_receiver);
		return 0;
	}

	/* Speaker mode: copy base table and apply per-mode/dev dynamic overrides */
	memcpy(aw87xxx->preset_buf, g_aw87xxx_preset_speaker_base,
	       sizeof(g_aw87xxx_preset_speaker_base));

	for (i = 0; i < sizeof(g_aw87xxx_preset_speaker_base); i += 2) {
		if (aw87xxx->preset_buf[i] == AW87XXX_PID_5A_REG_BATSAFE_REG) {
			if (!strncasecmp(name, "Voice", AW_PROFILE_STR_MAX) ||
			    !strncasecmp(name, "Voip", AW_PROFILE_STR_MAX))
				aw87xxx->preset_buf[i + 1] = 0x5C; /* VTH=3.6V for Voice */
		} else if (aw87xxx->preset_buf[i] == AW87XXX_PID_5A_REG_BSTOVR_REG) {
			if (aw87xxx->dev_index == 1)
				aw87xxx->preset_buf[i + 1] = 0x04; /* 7.5V Boost for Dev1 (Bottom) */
		}
	}

	container->data = aw87xxx->preset_buf;
	container->len = sizeof(g_aw87xxx_preset_speaker_base);
	return 0;
}

static int aw87xxx_power_down(struct aw87xxx *aw87xxx)
{
	struct aw_device *aw_dev = &aw87xxx->aw_dev;
	char *off_name = aw87xxx->prof_off_name;
	struct aw_data_container off_container = {
		.len = sizeof(g_aw87xxx_preset_off),
		.data = (u8 *)g_aw87xxx_preset_off,
	};
	int ret = 0;

	/* Already in target state, just refresh cache. */
	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGD(aw87xxx->dev, "chip already off, refresh cache only");
		goto out;
	}

	if (aw_dev->pwr_off_func)
		ret = aw_dev->pwr_off_func(aw_dev, &off_container);
	else
		ret = aw87xxx_dev_default_pwr_off(aw_dev, &off_container);

	if (ret < 0)
		AW_DEV_LOGE(aw87xxx->dev,
			    "off apply failed (ret=%d), reset already asserted",
			    ret);

out:
	aw87xxx_set_current_profile(aw87xxx, off_name);
	return ret;
}

static int aw87xxx_power_on(struct aw87xxx *aw87xxx, char *profile)
{
	struct aw_device *aw_dev = &aw87xxx->aw_dev;
	struct aw_data_container data_container;
	int ret;

	if (!strncmp(profile, aw87xxx->prof_off_name, AW_PROFILE_STR_MAX))
		return aw87xxx_power_down(aw87xxx);

	ret = aw87xxx_get_preset_profile(aw87xxx, profile, &data_container);
	if (ret < 0) {
		struct aw_prof_desc *prof_desc;
		prof_desc = aw87xxx_acf_get_prof_desc_from_name(aw87xxx->dev,
								&aw87xxx->acf_info,
								profile);
		if (!prof_desc || !prof_desc->prof_st) {
			AW_DEV_LOGE(aw87xxx->dev, "profile[%s] not found", profile);
			return -EINVAL;
		}
		data_container = prof_desc->data_container;
	}

	AW_DEV_LOGD(aw87xxx->dev, "load preset profile[%s] len=%u",
		    profile, data_container.len);

	if (aw_dev->pwr_on_func)
		ret = aw_dev->pwr_on_func(aw_dev, &data_container);
	else
		ret = aw87xxx_dev_default_pwr_on(aw_dev, &data_container);

	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev,
			    "load profile[%s] failed (ret=%d)",
			    profile, ret);
		aw87xxx_set_current_profile(aw87xxx, aw87xxx->prof_off_name);
		return ret;
	}

	aw87xxx_update_voltage_max(aw87xxx, &data_container);
	aw87xxx_set_current_profile(aw87xxx, profile);
	AW_DEV_LOGD(aw87xxx->dev, "load profile[%s] succeed", profile);

	return 0;
}

int aw87xxx_update_profile(struct aw87xxx *aw87xxx, char *profile)
{
	struct aw_device *aw_dev;
	int ret;

	if (!aw87xxx)
		return -EINVAL;

	aw_dev = &aw87xxx->aw_dev;

	mutex_lock(&aw87xxx->reg_lock);

	if (!strncmp(profile, aw87xxx->current_profile, AW_PROFILE_STR_MAX)) {
		bool want_off = !strncmp(profile,
					 aw87xxx->prof_off_name,
					 AW_PROFILE_STR_MAX);

		if (want_off || aw_dev->hwen_status == AW_DEV_HWEN_ON) {
			AW_DEV_LOGI(aw87xxx->dev,
				    "profile[%s] is already active!",
				    profile);
			mutex_unlock(&aw87xxx->reg_lock);
			return 0;
		}

		AW_DEV_LOGI(aw87xxx->dev,
			    "profile cache says [%s] but hwen=OFF, re-applying",
			    profile);
	}

	aw87xxx_monitor_stop(&aw87xxx->monitor);

	if (!strncmp(profile, aw87xxx->prof_off_name, AW_PROFILE_STR_MAX)) {
		ret = aw87xxx_power_down(aw87xxx);
	} else if (aw_dev->hwen_status == AW_DEV_HWEN_ON) {
		AW_DEV_LOGI(aw87xxx->dev, "warm switch [%s] -> [%s]",
			    aw87xxx->current_profile, profile);
		aw87xxx_dev_soft_reset(aw_dev);
		ret = aw87xxx_power_on(aw87xxx, profile);
		if (!ret)
			aw87xxx_monitor_start(&aw87xxx->monitor);
	} else {
		ret = aw87xxx_power_down(aw87xxx);
		if (ret < 0) {
			AW_DEV_LOGE(aw87xxx->dev, "load profile[%s] failed!",
				    aw87xxx->prof_off_name);
			mutex_unlock(&aw87xxx->reg_lock);
			return ret;
		}

		ret = aw87xxx_power_on(aw87xxx, profile);
		if (!ret)
			aw87xxx_monitor_start(&aw87xxx->monitor);
	}

	mutex_unlock(&aw87xxx->reg_lock);
	return ret;
}

int aw87xxx_update_profile_esd(struct aw87xxx *aw87xxx, char *profile)
{
	int ret;

	aw87xxx_dev_soft_reset(&aw87xxx->aw_dev);

	mutex_lock(&aw87xxx->reg_lock);
	if (!strncmp(profile, aw87xxx->prof_off_name, AW_PROFILE_STR_MAX))
		ret = aw87xxx_power_down(aw87xxx);
	else
		ret = aw87xxx_power_on(aw87xxx, profile);
	mutex_unlock(&aw87xxx->reg_lock);

	return ret;
}

/*
 * aw87xxx_power_on_by_index - Phase 1 of split power-up sequence (PRE_PMU).
 *
 * Asserts RESET HIGH and waits for the hardware settling delay, but does NOT
 * push any register configuration to the chip.  This gives the supply rail
 * and the GPIO line time to stabilise before the codec's POST_PMU event
 * issues I²C traffic via aw87xxx_set_profile_by_index().
 *
 * Locking: acquires reg_lock internally; must NOT be called with it held.
 *
 * Returns 0 on success, negative errno otherwise.
 */
int aw87xxx_power_on_by_index(int dev_index)
{
	struct aw87xxx *aw87xxx;
	struct aw_device *aw_dev;
	struct aw_prof_info *prof_info;
	int srcu_idx;
	int ret = 0;

	srcu_idx = srcu_read_lock(&g_aw87xxx_srcu);
	aw87xxx = aw87xxx_get_by_index(dev_index);
	if (!aw87xxx) {
		AW_LOGE("not found struct aw87xxx, dev_index[%d]", dev_index);
		ret = -EINVAL;
		goto unlock_srcu;
	}

	aw_dev = &aw87xxx->aw_dev;
	prof_info = &aw87xxx->acf_info.prof_info;

	mutex_lock(&aw87xxx->reg_lock);

	if (!prof_info->status) {
		AW_DEV_LOGE(aw87xxx->dev, "profile_cfg not loaded yet");
		ret = -EINVAL;
		goto unlock_reg;
	}

	if (aw_dev->hwen_status == AW_DEV_HWEN_ON) {
		AW_DEV_LOGI(aw87xxx->dev, "hw already on, skip PRE_PMU toggle");
		goto unlock_reg;
	}

	aw87xxx_dev_hw_pwr_ctrl(aw_dev, true);

	if (aw_dev->delay_desc.power_on_delay_ms > 0)
		mdelay(aw_dev->delay_desc.power_on_delay_ms);

	AW_DEV_LOGI(aw87xxx->dev, "PRE_PMU: hw asserted, dev_index=%d", dev_index);

unlock_reg:
	mutex_unlock(&aw87xxx->reg_lock);
unlock_srcu:
	srcu_read_unlock(&g_aw87xxx_srcu, srcu_idx);
	return ret;
}
EXPORT_SYMBOL_GPL(aw87xxx_power_on_by_index);

/*
 * aw87xxx_set_profile_by_index - Phase 2 of split power-up / warm-switch
 *                                 sequence (POST_PMU).
 *
 * Pushes register configuration for @profile to a chip that is already
 * powered (RESET HIGH, supply stable).  This is the right call-site for
 * POST_PMU events where the DAPM framework has guaranteed that supply
 * rails and clocks are stable.
 *
 * If the chip is currently OFF this function falls back to a full cold-start
 * via aw87xxx_update_profile() so callers don't need to special-case the
 * firmware-not-yet-loaded or early-probe scenarios.
 *
 * Locking: acquires reg_lock internally; must NOT be called with it held.
 *
 * Returns 0 on success, negative errno otherwise.
 */
int aw87xxx_set_profile_by_index(int dev_index, char *profile)
{
	struct aw87xxx *aw87xxx;
	struct aw_device *aw_dev;
	struct aw_prof_info *prof_info;
	struct aw_prof_desc *prof_desc;
	struct aw_data_container *data_container;
	struct aw_data_container preset_container;
	const char *prof_name;
	int srcu_idx;
	int ret = 0;

	srcu_idx = srcu_read_lock(&g_aw87xxx_srcu);
	aw87xxx = aw87xxx_get_by_index(dev_index);
	if (!aw87xxx || !profile) {
		AW_LOGE("not found struct aw87xxx, dev_index[%d]", dev_index);
		ret = -EINVAL;
		goto unlock_srcu;
	}

	aw_dev = &aw87xxx->aw_dev;
	prof_info = &aw87xxx->acf_info.prof_info;

	if (!strncmp(profile, aw87xxx->prof_off_name, AW_PROFILE_STR_MAX)) {
		ret = aw87xxx_update_profile(aw87xxx, profile);
		goto unlock_srcu;
	}

	mutex_lock(&aw87xxx->reg_lock);

	if (aw_dev->hwen_status != AW_DEV_HWEN_ON) {
		AW_DEV_LOGI(aw87xxx->dev,
			    "POST_PMU: hw not on yet, falling back to full update");
		mutex_unlock(&aw87xxx->reg_lock);
		ret = aw87xxx_update_profile(aw87xxx, profile);
		goto unlock_srcu;
	}

	/*
	 * PID 5A is fully isolated:
	 * uses hardened driver preset profiles directly
	 * without ACF profile validation
	 */
	if (aw_dev->chipid == AW_DEV_CHIPID_5A ||
	    aw87xxx_get_preset_profile(aw87xxx, profile, &preset_container) == 0) {
		ret = aw87xxx_get_preset_profile(aw87xxx, profile, &preset_container);
		if (ret < 0) {
			AW_DEV_LOGE(aw87xxx->dev,
				    "PID 5A preset profile[%s] invalid", profile);
			goto unlock_reg;
		}
		data_container = &preset_container;
		prof_name = profile;
	} else {
		if (!prof_info->status) {
			AW_DEV_LOGE(aw87xxx->dev, "profile_cfg not loaded yet");
			ret = -EINVAL;
			goto unlock_reg;
		}

		prof_desc = aw87xxx_acf_get_prof_desc_from_name(aw87xxx->dev,
								&aw87xxx->acf_info,
								profile);
		if (!prof_desc || !prof_desc->prof_st) {
			AW_DEV_LOGE(aw87xxx->dev,
				    "profile[%s] not found or empty", profile);
			ret = -EINVAL;
			goto unlock_reg;
		}
		data_container = &prof_desc->data_container;
		prof_name = prof_desc->prof_name;
	}

	aw87xxx_monitor_stop(&aw87xxx->monitor);

	AW_DEV_LOGI(aw87xxx->dev, "POST_PMU: applying profile[%s] len=%u",
		    profile, data_container->len);

	if (aw_dev->pwr_on_func)
		ret = aw_dev->pwr_on_func(aw_dev, data_container);
	else
		ret = aw87xxx_dev_default_pwr_on(aw_dev, data_container);

	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev,
			    "POST_PMU: profile[%s] apply failed (ret=%d)",
			    profile, ret);
		aw87xxx_set_current_profile(aw87xxx, aw87xxx->prof_off_name);
		goto unlock_reg;
	}

	aw87xxx_update_voltage_max(aw87xxx, data_container);
	aw87xxx_set_current_profile(aw87xxx, prof_name);
	aw87xxx_monitor_start(&aw87xxx->monitor);

	AW_DEV_LOGI(aw87xxx->dev, "POST_PMU: profile[%s] applied", profile);

unlock_reg:
	mutex_unlock(&aw87xxx->reg_lock);
unlock_srcu:
	srcu_read_unlock(&g_aw87xxx_srcu, srcu_idx);
	return ret;
}
EXPORT_SYMBOL_GPL(aw87xxx_set_profile_by_index);

/*
 * aw87xxx_soft_off_by_index - Phase 3 of split power-down sequence (PRE_PMD).
 *
 * Sends the soft-off register sequence to the chip via I²C but deliberately
 * leaves the RESET GPIO HIGH.  hwen_status remains AW_DEV_HWEN_ON so that
 * POST_PMD's call to aw87xxx_power_off_by_index() is NOT a no-op and can
 * assert RESET LOW after its own hardware settling delay.
 *
 * Locking: acquires reg_lock internally; must NOT be called with it held.
 *
 * Returns 0 on success, negative errno otherwise.
 */
int aw87xxx_soft_off_by_index(int dev_index)
{
	struct aw87xxx *aw87xxx;
	struct aw_device *aw_dev;
	struct aw_prof_desc *prof_desc;
	struct aw_data_container off_container = {
		.len = sizeof(g_aw87xxx_preset_off),
		.data = (u8 *)g_aw87xxx_preset_off,
	};
	struct aw_data_container *p_off = &off_container;
	int srcu_idx;
	int ret = 0;

	srcu_idx = srcu_read_lock(&g_aw87xxx_srcu);
	aw87xxx = aw87xxx_get_by_index(dev_index);
	if (!aw87xxx) {
		AW_LOGE("not found struct aw87xxx, dev_index[%d]", dev_index);
		ret = -EINVAL;
		goto unlock_srcu;
	}

	aw_dev = &aw87xxx->aw_dev;

	mutex_lock(&aw87xxx->reg_lock);

	if (aw_dev->hwen_status != AW_DEV_HWEN_ON) {
		AW_DEV_LOGI(aw87xxx->dev,
			    "PRE_PMD: hw already off, skip soft-off, dev_index=%d",
			    dev_index);
		goto unlock_reg;
	}

	aw87xxx_monitor_stop(&aw87xxx->monitor);

	if (aw_dev->chipid != AW_DEV_CHIPID_5A) {
		prof_desc = aw87xxx_acf_get_prof_desc_from_name(aw87xxx->dev,
								&aw87xxx->acf_info,
								aw87xxx->prof_off_name);
		if (prof_desc && prof_desc->prof_st)
			p_off = &prof_desc->data_container;
	}

	ret = aw87xxx_dev_soft_off_only(aw_dev, p_off);

	AW_DEV_LOGI(aw87xxx->dev,
		    "PRE_PMD: soft-off regs sent, dev_index=%d",
		    dev_index);

unlock_reg:
	mutex_unlock(&aw87xxx->reg_lock);
unlock_srcu:
	srcu_read_unlock(&g_aw87xxx_srcu, srcu_idx);
	return ret;
}
EXPORT_SYMBOL_GPL(aw87xxx_soft_off_by_index);

/*
 * aw87xxx_power_off_by_index - Phase 4 of split power-down sequence (POST_PMD).
 *
 * Asserts RESET LOW.  By the time POST_PMD fires, the PRE_PMD handler
 * (aw87xxx_soft_off_by_index) has already run the soft-off register sequence
 * via I²C while keeping RESET HIGH, so it is safe to pull the GPIO now.
 * hwen_status is still AW_DEV_HWEN_ON after PRE_PMD, so this call is never
 * a no-op under normal sequencing.
 *
 * Locking: acquires reg_lock internally; must NOT be called with it held.
 *
 * Returns 0 always (best-effort; errors are logged).
 */
int aw87xxx_power_off_by_index(int dev_index)
{
	struct aw87xxx *aw87xxx;
	struct aw_device *aw_dev;
	int srcu_idx;

	srcu_idx = srcu_read_lock(&g_aw87xxx_srcu);
	aw87xxx = aw87xxx_get_by_index(dev_index);
	if (!aw87xxx) {
		AW_LOGE("not found struct aw87xxx, dev_index[%d]", dev_index);
		srcu_read_unlock(&g_aw87xxx_srcu, srcu_idx);
		return -EINVAL;
	}

	aw_dev = &aw87xxx->aw_dev;

	mutex_lock(&aw87xxx->reg_lock);

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGI(aw87xxx->dev,
			    "POST_PMD: hw already off, dev_index=%d",
			    dev_index);
		mutex_unlock(&aw87xxx->reg_lock);
		srcu_read_unlock(&g_aw87xxx_srcu, srcu_idx);
		return 0;
	}

	aw87xxx_monitor_stop(&aw87xxx->monitor);
	aw87xxx_set_current_profile(aw87xxx, aw87xxx->prof_off_name);
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, false);

	AW_DEV_LOGI(aw87xxx->dev,
		    "POST_PMD: hw asserted low, dev_index=%d",
		    dev_index);

	mutex_unlock(&aw87xxx->reg_lock);
	srcu_read_unlock(&g_aw87xxx_srcu, srcu_idx);
	return 0;
}
EXPORT_SYMBOL_GPL(aw87xxx_power_off_by_index);

/* Kcontrols sections */
static int aw87xxx_switch_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;

	ucontrol->value.integer.value[0] = strncmp(aw87xxx->current_profile,
						   aw87xxx->prof_off_name,
						   AW_PROFILE_STR_MAX);
	return 0;
}

static int aw87xxx_profile_switch_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;
	char *profile_name;
	char *name;
	int count;

	if (!aw87xxx)
		return 0;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	count = aw87xxx_acf_get_profile_count(aw87xxx->dev, &aw87xxx->acf_info);

	if (count <= 0) {
		uinfo->value.enumerated.items = 0;
		return 0;
	}

	uinfo->value.enumerated.items = count;
	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	name = uinfo->value.enumerated.name;
	profile_name = aw87xxx_acf_get_prof_name_from_index(aw87xxx->dev,
							    &aw87xxx->acf_info,
							    uinfo->value.enumerated.item);

	if (!profile_name)
		strscpy(name, "NULL", sizeof(uinfo->value.enumerated.name));
	else
		strscpy(name, profile_name, sizeof(uinfo->value.enumerated.name));

	return 0;
}

static int aw87xxx_profile_switch_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;
	int index = ucontrol->value.integer.value[0];
	char *profile_name;
	int ret;

	if (!aw87xxx)
		return -EINVAL;

	profile_name = aw87xxx_acf_get_prof_name_from_index(aw87xxx->dev,
							    &aw87xxx->acf_info,
							    index);
	if (!profile_name)
		return -EINVAL;

	ret = aw87xxx_update_profile(aw87xxx, profile_name);
	if (ret < 0)
		return -EINVAL;

	return 0;
}

static int aw87xxx_profile_switch_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;
	int index;

	if (!aw87xxx || !aw87xxx->current_profile)
		return -EINVAL;

	index = aw87xxx_acf_get_prof_index_from_name(aw87xxx->dev,
						     &aw87xxx->acf_info,
						     aw87xxx->current_profile);
	if (index < 0)
		return -EINVAL;

	ucontrol->value.integer.value[0] = index;
	return 0;
}

static int aw87xxx_vmax_get_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = INT_MIN;
	uinfo->value.integer.max = AW_VMAX_MAX;

	return 0;
}

static int aw87xxx_vmax_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;
	int vmax_val = 0;
	int ret;

	if (!aw87xxx)
		return -EINVAL;

	ret = aw87xxx_monitor_no_dsp_get_vmax(&aw87xxx->monitor, &vmax_val);
	if (ret < 0)
		return -EINVAL;

	ucontrol->value.integer.value[0] = vmax_val;

	return 0;
}

static int aw87xxx_monitor_switch_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	int count = ARRAY_SIZE(aw87xxx_monitor_switch);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	strscpy(uinfo->value.enumerated.name,
		aw87xxx_monitor_switch[uinfo->value.enumerated.item],
		sizeof(uinfo->value.enumerated.name));

	return 0;
}

static int aw87xxx_monitor_switch_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;
	u32 ctrl_val = ucontrol->value.integer.value[0];
	int ret;

	ret = aw87xxx_dev_monitor_switch_set(&aw87xxx->monitor, ctrl_val);
	if (ret)
		return -EINVAL;

	return 0;
}

static int aw87xxx_monitor_switch_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;
	struct aw_monitor *aw_monitor = &aw87xxx->monitor;

	if (aw_monitor->version == AW_MONITOR_HDR_VER_0_1_2)
		ucontrol->value.integer.value[0] =
			aw_monitor->monitor_cfg.monitor_switch;
	else
		ucontrol->value.integer.value[0] =
			aw_monitor->monitor_hdr.monitor_switch;

	return 0;
}

static int aw87xxx_spin_switch_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	int count = ARRAY_SIZE(aw87xxx_spin_switch);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	strscpy(uinfo->value.enumerated.name,
		aw87xxx_spin_switch[uinfo->value.enumerated.item],
		sizeof(uinfo->value.enumerated.name));

	return 0;
}

static int aw87xxx_spin_switch_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	return aw87xxx_dsp_set_spin(ucontrol->value.integer.value[0]);
}

static int aw87xxx_spin_switch_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = aw87xxx_dsp_get_spin();
	return 0;
}

static int aw87xxx_hal_monitor_time_info(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = AW_MONITOR_TIME_MIN;
	uinfo->value.integer.max = AW_MONITOR_TIME_MAX;

	return 0;
}

static int aw87xxx_hal_monitor_time_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct aw87xxx *aw87xxx = (struct aw87xxx *)kcontrol->private_value;

	if (!aw87xxx)
		return -EINVAL;

	ucontrol->value.integer.value[0] =
		aw87xxx->monitor.monitor_hdr.monitor_time;

	return 0;
}

static int aw87xxx_kcontrol_dynamic_create(struct aw87xxx *aw87xxx,
					   struct snd_soc_component *codec)
{
	struct snd_kcontrol_new aw_kctl[AW87XXX_PRIVATE_KCONTROL_NUM] = {0};
	char kctl_name[AW87XXX_PRIVATE_KCONTROL_NUM][AW_NAME_BUF_MAX];
	int ret;

	aw87xxx->codec = codec;

	snprintf(kctl_name[0], AW_NAME_BUF_MAX,
		 "aw87xxx_profile_switch_%d",
		 aw87xxx->dev_index);
	aw_kctl[0].name = kctl_name[0];
	aw_kctl[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw_kctl[0].info = aw87xxx_profile_switch_info;
	aw_kctl[0].get = aw87xxx_profile_switch_get;
	aw_kctl[0].put = aw87xxx_profile_switch_put;
	aw_kctl[0].private_value = (unsigned long)aw87xxx;

	snprintf(kctl_name[1], AW_NAME_BUF_MAX,
		 "aw87xxx_vmax_get_%d",
		 aw87xxx->dev_index);
	aw_kctl[1].name = kctl_name[1];
	aw_kctl[1].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw_kctl[1].access = SNDRV_CTL_ELEM_ACCESS_READ;
	aw_kctl[1].info = aw87xxx_vmax_get_info;
	aw_kctl[1].get = aw87xxx_vmax_get;
	aw_kctl[1].private_value = (unsigned long)aw87xxx;

	snprintf(kctl_name[2], AW_NAME_BUF_MAX,
		 "aw87xxx_monitor_switch_%d",
		 aw87xxx->dev_index);
	aw_kctl[2].name = kctl_name[2];
	aw_kctl[2].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw_kctl[2].info = aw87xxx_monitor_switch_info;
	aw_kctl[2].get = aw87xxx_monitor_switch_get;
	aw_kctl[2].put = aw87xxx_monitor_switch_put;
	aw_kctl[2].private_value = (unsigned long)aw87xxx;

	snprintf(kctl_name[3], AW_NAME_BUF_MAX,
		 "aw87xxx_switch_%d",
		 aw87xxx->dev_index);
	aw_kctl[3].name = kctl_name[3];
	aw_kctl[3].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw_kctl[3].access = SNDRV_CTL_ELEM_ACCESS_READ;
	aw_kctl[3].info = aw87xxx_monitor_switch_info;
	aw_kctl[3].get = aw87xxx_switch_get;
	aw_kctl[3].private_value = (unsigned long)aw87xxx;

	ret = snd_soc_add_component_controls(codec, aw_kctl, ARRAY_SIZE(aw_kctl));
	if (ret < 0)
		AW_DEV_LOGE(aw87xxx->dev, "add codec controls failed: %d", ret);

	return ret;
}

static int aw87xxx_public_kcontrol_create(struct aw87xxx *aw87xxx,
					  struct snd_soc_component *codec)
{
	struct snd_kcontrol_new aw_kctl[AW87XXX_PUBLIC_KCONTROL_NUM] = {0};
	int ret;

	aw87xxx->codec = codec;

	aw_kctl[0].name = "aw87xxx_spin_switch";
	aw_kctl[0].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw_kctl[0].info = aw87xxx_spin_switch_info;
	aw_kctl[0].get = aw87xxx_spin_switch_get;
	aw_kctl[0].put = aw87xxx_spin_switch_put;
	aw_kctl[0].private_value = (unsigned long)aw87xxx;

	aw_kctl[1].name = "aw87xxx_hal_monitor_time";
	aw_kctl[1].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	aw_kctl[1].access = SNDRV_CTL_ELEM_ACCESS_READ;
	aw_kctl[1].info = aw87xxx_hal_monitor_time_info;
	aw_kctl[1].get = aw87xxx_hal_monitor_time_get;
	aw_kctl[1].private_value = (unsigned long)aw87xxx;

	ret = snd_soc_add_component_controls(codec, aw_kctl, ARRAY_SIZE(aw_kctl));
	if (ret < 0)
		AW_DEV_LOGE(aw87xxx->dev, "add public controls failed: %d", ret);

	return ret;
}

int aw87xxx_add_codec_controls(struct snd_soc_component *codec)
{
	struct aw87xxx *aw87xxx;
	int ret = 0;

	mutex_lock(&g_aw87xxx_list_lock);
	list_for_each_entry(aw87xxx, &g_aw87xxx_list, node) {
		ret = aw87xxx_kcontrol_dynamic_create(aw87xxx, codec);
		if (ret < 0)
			break;

		if (aw87xxx->dev_index == 0) {
			ret = aw87xxx_public_kcontrol_create(aw87xxx, codec);
			if (ret < 0)
				break;
		}
	}
	mutex_unlock(&g_aw87xxx_list_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(aw87xxx_add_codec_controls);

static void aw87xxx_fw_cfg_free(struct aw87xxx *aw87xxx)
{
	int i;

	if (aw87xxx->bin_type == BIN_TYPE_SINGLE &&
	    aw87xxx->single_bin_data) {
		for (i = 0; i < aw87xxx->support_prof_count; i++)
			kfree(aw87xxx->single_bin_data[i]);
		kfree(aw87xxx->single_bin_data);
	}

	aw87xxx_acf_profile_free(aw87xxx->dev, &aw87xxx->acf_info);
	aw87xxx_monitor_cfg_free(&aw87xxx->monitor);
}

static int aw87xxx_init_default_prof(struct aw87xxx *aw87xxx)
{
	char *profile;

	profile = aw87xxx_acf_get_prof_off_name(aw87xxx->dev,
						&aw87xxx->acf_info);
	if (!profile) {
		AW_DEV_LOGE(aw87xxx->dev, "get profile off name failed");
		return -EINVAL;
	}

	strscpy(aw87xxx->prof_off_name, profile, AW_PROFILE_STR_MAX);
	aw87xxx_set_current_profile(aw87xxx, aw87xxx->prof_off_name);

	return 0;
}

static int aw87xxx_acf_bin_load(struct aw87xxx *aw87xxx)
{
	struct acf_bin_info *acf_info = &aw87xxx->acf_info;
	const struct firmware *cont = NULL;
	int ret;

	ret = request_firmware(&cont, aw87xxx->fw_name, aw87xxx->dev);
	if (ret || !cont) {
		AW_DEV_LOGE(aw87xxx->dev, "load[%s] failed!", aw87xxx->fw_name);

		if (acf_info->load_count < AW_READ_CHIPID_RETRIES) {
			acf_info->load_count++;
			schedule_delayed_work(&aw87xxx->fw_load_work,
					      msecs_to_jiffies(1000));
			return -EBUSY;
		}

		acf_info->load_count = 0;
		return ret;
	}

	AW_DEV_LOGI(aw87xxx->dev, "loaded %s - size: %zu",
		    aw87xxx->fw_name, cont->size);

	mutex_lock(&aw87xxx->reg_lock);
	acf_info->fw_data = kzalloc(cont->size, GFP_KERNEL);
	if (!acf_info->fw_data) {
		ret = -ENOMEM;
		goto exit_alloc_failed;
	}

	memcpy(acf_info->fw_data, cont->data, cont->size);
	acf_info->fw_size = cont->size;

	ret = aw87xxx_acf_parse(aw87xxx->dev, &aw87xxx->acf_info);
	if (ret < 0)
		AW_DEV_LOGE(aw87xxx->dev, "fw_data parse failed");

exit_alloc_failed:
	mutex_unlock(&aw87xxx->reg_lock);
	release_firmware(cont);

	return ret;
}

static int aw87xxx_single_bin_load(struct aw87xxx *aw87xxx)
{
	struct acf_bin_info *acf_info = &aw87xxx->acf_info;
	struct aw_prof_info *prof_info = &acf_info->prof_info;
	struct aw_data_container *aw_fw_data;
	const struct firmware *cont = NULL;
	char fw_name[FW_NAME_MAX];
	int ret, i;

	prof_info->count = aw87xxx->support_prof_count;
	prof_info->prof_desc = kcalloc(prof_info->count,
				       sizeof(*prof_info->prof_desc),
				       GFP_KERNEL);
	if (!prof_info->prof_desc)
		return -ENOMEM;

	aw_fw_data = kcalloc(aw87xxx->support_prof_count,
			     sizeof(*aw_fw_data),
			     GFP_KERNEL);
	if (!aw_fw_data) {
		ret = -ENOMEM;
		goto err_free_desc;
	}

	aw87xxx->single_bin_data = kcalloc(aw87xxx->support_prof_count,
					   sizeof(void *),
					   GFP_KERNEL);
	if (!aw87xxx->single_bin_data) {
		ret = -ENOMEM;
		goto err_free_fw_data_arr;
	}

	for (i = 0; i < aw87xxx->support_prof_count; i++) {
		snprintf(fw_name, sizeof(fw_name),
			 "aw87xxx_%02d_%s_0x%02x.bin",
			 aw87xxx->dev_index,
			 aw87xxx->support_prof[i],
			 aw87xxx->aw_dev.chipid);

		ret = request_firmware(&cont, fw_name, aw87xxx->dev);
		if (ret || !cont) {
			AW_DEV_LOGE(aw87xxx->dev, "request %s failed", fw_name);

			if (acf_info->load_count < AW_READ_CHIPID_RETRIES) {
				acf_info->load_count++;
				schedule_delayed_work(&aw87xxx->fw_load_work,
						      msecs_to_jiffies(1000));
				ret = -EBUSY; /* Deferred */
			} else {
				acf_info->load_count = 0;
			}
			goto err_free_fw_data;
		}

		if (cont->size < FW_MIN_SIZE) {
			release_firmware(cont);
			ret = -EINVAL;
			goto err_free_fw_data;
		}

		aw_fw_data[i].data = kzalloc(cont->size, GFP_KERNEL);
		if (!aw_fw_data[i].data) {
			release_firmware(cont);
			ret = -ENOMEM;
			goto err_free_fw_data;
		}
		aw_fw_data[i].len = cont->size;
		memcpy(aw_fw_data[i].data, cont->data, cont->size);

		aw87xxx->single_bin_data[i] = aw_fw_data[i].data;
		release_firmware(cont);
	}

	mutex_lock(&aw87xxx->reg_lock);
	prof_info->prof_name_list = kcalloc(prof_info->count,
					    AW_PROFILE_STR_MAX,
					    GFP_KERNEL);
	if (!prof_info->prof_name_list) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	for (i = 0; i < prof_info->count; i++) {
		ret = aw_parse_single_bin(aw87xxx->dev, acf_info,
					  aw_fw_data[i], i);
		if (ret)
			goto err_free_name_list;

		strscpy(prof_info->prof_name_list[i],
			aw87xxx->support_prof[i],
			AW_PROFILE_STR_MAX);
		prof_info->prof_desc[i].prof_name = aw87xxx->support_prof[i];
	}
	mutex_unlock(&aw87xxx->reg_lock);
	kfree(aw_fw_data);

	return 0;

err_free_name_list:
	kfree(prof_info->prof_name_list);
err_unlock:
	mutex_unlock(&aw87xxx->reg_lock);
err_free_fw_data:
	for (i = 0; i < aw87xxx->support_prof_count; i++)
		kfree(aw87xxx->single_bin_data[i]);
	kfree(aw87xxx->single_bin_data);
err_free_fw_data_arr:
	kfree(aw_fw_data);
err_free_desc:
	kfree(prof_info->prof_desc);
	return ret;
}

static void aw87xxx_fw_load_work(struct work_struct *work)
{
	struct aw87xxx *aw87xxx = container_of(work, struct aw87xxx,
					       fw_load_work.work);
	int ret = -1;

	if (aw87xxx->bin_type == BIN_TYPE_ACF) {
		ret = aw87xxx_acf_bin_load(aw87xxx);
		if (ret == -EBUSY)
			return;
		if (ret) {
			ret = aw87xxx_single_bin_load(aw87xxx);
			if (ret)
				return;
		}
	} else if (aw87xxx->bin_type == BIN_TYPE_SINGLE) {
		ret = aw87xxx_single_bin_load(aw87xxx);
		if (ret)
			return;
	} else {
		AW_DEV_LOGE(aw87xxx->dev, "bin type error");
		return;
	}

	ret = aw87xxx_init_default_prof(aw87xxx);
	if (ret < 0) {
		aw87xxx_fw_cfg_free(aw87xxx);
		AW_DEV_LOGE(aw87xxx->dev, "aw87xxx_init_default_prof failed");
		return;
	}
	AW_DEV_LOGI(aw87xxx->dev, "acf parse succeed");
}

static void aw87xxx_fw_load_init(struct aw87xxx *aw87xxx)
{
	int cfg_timer_val = AW_CFG_UPDATE_DELAY_TIMER;

	snprintf(aw87xxx->fw_name, AW87XXX_FW_NAME_MAX,
		 "%s", AW87XXX_FW_BIN_NAME);
	aw87xxx_acf_init(&aw87xxx->aw_dev,
			 &aw87xxx->acf_info,
			 aw87xxx->dev_index);

	INIT_DELAYED_WORK(&aw87xxx->fw_load_work, aw87xxx_fw_load_work);
	schedule_delayed_work(&aw87xxx->fw_load_work,
			      msecs_to_jiffies(cfg_timer_val));
}

static ssize_t reg_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_device *aw_dev = &aw87xxx->aw_dev;
	ssize_t len = 0;
	u8 reg_val = 0;
	unsigned int i;

	for (i = 0; i <= aw_dev->reg_max_addr; i++) {
		if (aw87xxx_dev_i2c_read_byte(&aw87xxx->aw_dev, i, &reg_val) < 0)
			len += sysfs_emit_at(buf, len, "read reg[0x%x] failed\n", i);
		else
			len += sysfs_emit_at(buf, len, "reg: 0x%02X=0x%02X\n", i, reg_val);
	}
	return len;
}

static ssize_t reg_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	unsigned long reg_addr, reg_val;
	char *p = (char *)buf;
	int ret;

	while (*p && isspace(*p))
		p++;
	ret = kstrtoul(p, 16, &reg_addr);
	if (ret)
		return -EINVAL;

	while (*p && !isspace(*p))
		p++;
	while (*p && isspace(*p))
		p++;

	ret = kstrtoul(p, 16, &reg_val);
	if (ret)
		return -EINVAL;

	if (reg_addr > aw87xxx->aw_dev.reg_max_addr) {
		AW_DEV_LOGE(aw87xxx->dev, "reg[0x%lx] out of max[0x%x]",
			    reg_addr, aw87xxx->aw_dev.reg_max_addr);
		return -EINVAL;
	}

	ret = aw87xxx_dev_i2c_write_byte(&aw87xxx->aw_dev,
					 (u8)reg_addr,
					 (u8)reg_val);
	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev, "set [0x%lx]=0x%lx failed",
			    reg_addr, reg_val);
	}

	return len;
}

static ssize_t profile_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_prof_info *prof_info = &aw87xxx->acf_info.prof_info;
	ssize_t len = 0;
	unsigned int i;

	if (!prof_info->status)
		return sysfs_emit(buf, "profile_cfg not load\n");

	for (i = 0; i < prof_info->count; i++) {
		if (!strncmp(aw87xxx->current_profile,
			     prof_info->prof_name_list[i],
			     AW_PROFILE_STR_MAX)) {
			len += sysfs_emit_at(buf, len, ">%s\n",
					     prof_info->prof_name_list[i]);
		} else {
			len += sysfs_emit_at(buf, len, " %s\n",
					     prof_info->prof_name_list[i]);
		}
	}

	return len;
}

static ssize_t profile_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	char profile[AW_PROFILE_STR_MAX] = {0};
	int ret;

	if (!buf || len == 0 || len > AW_PROFILE_STR_MAX)
		return -EINVAL;

	if (sscanf(buf, "%31s", profile) != 1) {
		AW_DEV_LOGE(aw87xxx->dev, "invalid profile format");
		return -EINVAL;
	}

	ret = aw87xxx_update_profile(aw87xxx, profile);
	if (ret < 0) {
		AW_DEV_LOGE(aw87xxx->dev, "set profile[%s] failed", profile);
		return ret;
	}

	return len;
}

static ssize_t hwen_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	int hwen = aw87xxx->aw_dev.hwen_status;

	if (hwen >= AW_DEV_HWEN_INVALID)
		return sysfs_emit(buf, "hwen_status: invalid\n");
	else if (hwen == AW_DEV_HWEN_ON)
		return sysfs_emit(buf, "hwen_status: on\n");
	else if (hwen == AW_DEV_HWEN_OFF)
		return sysfs_emit(buf, "hwen_status: off\n");

	return 0;
}

static ssize_t hwen_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	unsigned int state;
	int ret;

	ret = kstrtouint(buf, 0, &state);
	if (ret)
		return ret;

	mutex_lock(&aw87xxx->reg_lock);
	if (state == AW_DEV_HWEN_OFF)
		aw87xxx_dev_hw_pwr_ctrl(&aw87xxx->aw_dev, false);
	else if (state == AW_DEV_HWEN_ON)
		aw87xxx_dev_hw_pwr_ctrl(&aw87xxx->aw_dev, true);
	mutex_unlock(&aw87xxx->reg_lock);

	return len;
}

static int aw87xxx_awrw_write(struct aw87xxx *aw87xxx,
			      const char *buf, size_t count)
{
	struct aw_i2c_packet *packet = &aw87xxx->i2c_packet;
	int data_str_size = 2 + 2 * AWRW_DATA_BYTES;
	unsigned int temp_data = 0;
	u8 *reg_data;
	int i, ret = -1;

	reg_data = kcalloc(packet->reg_num, sizeof(u8), GFP_KERNEL);
	if (!reg_data)
		return -ENOMEM;

	for (i = 0; i < packet->reg_num; i++) {
		ret = sscanf(buf + AWRW_HDR_LEN + 1 + i * (data_str_size + 1),
			     "0x%x", &temp_data);
		if (ret != 1) {
			kfree(reg_data);
			return -EINVAL;
		}
		reg_data[i] = (u8)temp_data;
	}

	ret = regmap_bulk_write(aw87xxx->aw_dev.regmap, packet->reg_addr,
				reg_data, packet->reg_num);
	kfree(reg_data);

	return (ret < 0) ? -EFAULT : 0;
}

static int aw87xxx_awrw_data_check(struct aw87xxx *aw87xxx,
				   int *data, size_t count)
{
	struct aw_i2c_packet *packet = &aw87xxx->i2c_packet;
	int act_data_len;
	int req_data_len;

	if (data[AWRW_HDR_ADDR_BYTES] != AWRW_ADDR_BYTES ||
	    data[AWRW_HDR_DATA_BYTES] != AWRW_DATA_BYTES) {
		return -EINVAL;
	}

	if (data[AWRW_HDR_WR_FLAG] == AWRW_FLAG_WRITE) {
		req_data_len = (2 + 2 * AWRW_DATA_BYTES + 1) * packet->reg_num;
		act_data_len = count - AWRW_HDR_LEN - 1;

		if (req_data_len > act_data_len)
			return -EINVAL;
	}

	return 0;
}

static int aw87xxx_awrw_parse_buf(struct aw87xxx *aw87xxx,
				  const char *buf, size_t count,
				  int *wr_status)
{
	unsigned int data[AWRW_HDR_MAX] = {0};
	struct aw_i2c_packet *packet = &aw87xxx->i2c_packet;

	if (sscanf(buf, "0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
		   &data[0], &data[1], &data[2], &data[3], &data[4]) == 5) {
		packet->reg_addr = data[AWRW_HDR_REG_ADDR];
		packet->reg_num = data[AWRW_HDR_REG_NUM];
		*wr_status = data[AWRW_HDR_WR_FLAG];

		return aw87xxx_awrw_data_check(aw87xxx, (int *)data, count);
	}

	return -EINVAL;
}

static ssize_t awrw_show(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	struct aw_i2c_packet *packet = &aw87xxx->i2c_packet;
	int data_len;
	size_t len = 0;
	int ret, i;
	u8 *reg_data;

	if (packet->status != AWRW_I2C_ST_READ)
		return -EINVAL;

	data_len = AWRW_DATA_BYTES * packet->reg_num;
	reg_data = kzalloc(data_len, GFP_KERNEL);
	if (!reg_data)
		return -ENOMEM;

	ret = aw87xxx_dev_i2c_read_msg(&aw87xxx->aw_dev,
				       packet->reg_addr,
				       reg_data, data_len);
	if (ret < 0) {
		kfree(reg_data);
		packet->status = AWRW_I2C_ST_NONE;
		return -EFAULT;
	}

	for (i = 0; i < data_len; i++)
		len += sysfs_emit_at(buf, len, "0x%02x,", reg_data[i]);

	kfree(reg_data);
	packet->status = AWRW_I2C_ST_NONE;

	return len;
}

static ssize_t awrw_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct aw87xxx *aw87xxx = dev_get_drvdata(dev);
	int wr_status = 0;
	int ret;

	if (count < AWRW_HDR_LEN)
		return -EINVAL;

	ret = aw87xxx_awrw_parse_buf(aw87xxx, buf, count, &wr_status);
	if (ret < 0)
		return ret;

	if (wr_status == AWRW_FLAG_WRITE) {
		ret = aw87xxx_awrw_write(aw87xxx, buf, count);
		if (ret < 0)
			return ret;
	} else if (wr_status == AWRW_FLAG_READ) {
		aw87xxx->i2c_packet.status = AWRW_I2C_ST_READ;
	} else {
		return -EINVAL;
	}

	return count;
}

static ssize_t drv_ver_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "driver_ver: %s\n", AW87XXX_DRIVER_VERSION);
}

static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_RW(profile);
static DEVICE_ATTR_RW(hwen);
static DEVICE_ATTR_RW(awrw);
static DEVICE_ATTR_RO(drv_ver);

static struct attribute *aw87xxx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_profile.attr,
	&dev_attr_hwen.attr,
	&dev_attr_awrw.attr,
	&dev_attr_drv_ver.attr,
	NULL
};

static const struct attribute_group aw87xxx_attribute_group = {
	.attrs = aw87xxx_attributes
};

static int aw87xxx_dtsi_dev_index_check(struct aw87xxx *cur_aw87xxx)
{
	struct aw87xxx *list_aw87xxx;
	int idx = srcu_read_lock(&g_aw87xxx_srcu);

	list_for_each_entry_rcu(list_aw87xxx, &g_aw87xxx_list, node) {
		if (list_aw87xxx != cur_aw87xxx &&
		    list_aw87xxx->dev_index == cur_aw87xxx->dev_index) {
			AW_DEV_LOGE(cur_aw87xxx->dev, "dev_index exist");
			srcu_read_unlock(&g_aw87xxx_srcu, idx);
			return -EINVAL;
		}
	}
	srcu_read_unlock(&g_aw87xxx_srcu, idx);

	return 0;
}

static bool aw87xxx_is_primary_ready(void)
{
	struct aw87xxx *aw87xxx;
	bool ready = false;
	int idx;

	idx = srcu_read_lock(&g_aw87xxx_srcu);
	list_for_each_entry_rcu(aw87xxx, &g_aw87xxx_list, node) {
		if (aw87xxx->dev_index == 0) {
			ready = true;
			break;
		}
	}
	srcu_read_unlock(&g_aw87xxx_srcu, idx);

	return ready;
}

static int aw87xxx_cdc_rst_pin_ctrl(struct aw_device *aw_dev, bool enable)
{
	if (enable)
		return msm_cdc_pinctrl_select_active_state(aw_dev->rst_pinctrl_np);

	return msm_cdc_pinctrl_select_sleep_state(aw_dev->rst_pinctrl_np);
}

static int aw87xxx_dtsi_parse(struct aw87xxx *aw87xxx,
			      struct device_node *dev_node)
{
	struct aw_voltage_desc *vol_desc = &aw87xxx->aw_dev.vol_desc;
	u32 voltage_min, dev_index;
	int count, i, ret;

	ret = of_property_read_u32(dev_node, "dev_index", &dev_index);
	if (ret < 0) {
		aw87xxx->dev_index = atomic_read(&g_aw87xxx_dev_cnt);
	} else {
		aw87xxx->dev_index = dev_index;
		aw87xxx->aw_dev.dev_index = dev_index;
	}

	ret = aw87xxx_dtsi_dev_index_check(aw87xxx);
	if (ret < 0)
		return ret;

	if (aw87xxx->dev_index > 0 && !aw87xxx_is_primary_ready())
		return -EPROBE_DEFER;

	aw87xxx->aw_dev.rst_pinctrl_np = of_parse_phandle(dev_node, "cdc-rst-pinctrl", 0);
	if (aw87xxx->aw_dev.rst_pinctrl_np) {
		aw87xxx->aw_dev.has_rst_gpio = false;
		aw87xxx->aw_dev.hwen_status = AW_DEV_HWEN_OFF;
		aw87xxx->aw_dev.rst_pin_ctrl = aw87xxx_cdc_rst_pin_ctrl;
	} else {
		struct gpio_desc *desc;

		desc = gpiod_get_optional(aw87xxx->dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(desc)) {
			ret = PTR_ERR(desc);
			if (ret != -EPROBE_DEFER)
				AW_DEV_LOGE(aw87xxx->dev, "failed to get reset-gpio: %d", ret);
			goto err_put_node;
		}

		if (!desc) {
			aw87xxx->aw_dev.has_rst_gpio = false;
			aw87xxx->aw_dev.hwen_status = AW_DEV_HWEN_INVALID;
		} else {
			aw87xxx->aw_dev.has_rst_gpio = true;
			aw87xxx->aw_dev.hwen_status = AW_DEV_HWEN_OFF;
			gpiod_put(desc);
		}
	}

	ret = of_property_read_u32(dev_node, "aw-voltage-min", &voltage_min);
	vol_desc->vol_min = (ret < 0) ? AW_BOOST_VOLTAGE_MIN : voltage_min;

	aw87xxx_dsp_parse_port_id_dt(&aw87xxx->aw_dev);
	aw87xxx_dsp_parse_topo_id_dt(&aw87xxx->aw_dev);

	count = of_property_count_strings(dev_node, "scene_name");
	if (count <= 0) {
		aw87xxx->bin_type = BIN_TYPE_ACF;
		aw87xxx->support_prof_count = 2;
		aw87xxx->support_prof = kcalloc(aw87xxx->support_prof_count,
						sizeof(char *),
						GFP_KERNEL);
		if (!aw87xxx->support_prof) {
			ret = -ENOMEM;
			goto err_put_node;
		}
		aw87xxx->support_prof[0] = "Music";
		aw87xxx->support_prof[1] = "Off";
	} else {
		aw87xxx->support_prof_count = count;
		aw87xxx->bin_type = BIN_TYPE_SINGLE;
		aw87xxx->support_prof = kcalloc(count,
						sizeof(char *),
						GFP_KERNEL);
		if (!aw87xxx->support_prof) {
			ret = -ENOMEM;
			goto err_put_node;
		}
		for (i = 0; i < count; i++) {
			ret = of_property_read_string_index(dev_node,
							    "scene_name", i,
							    (const char **)&aw87xxx->support_prof[i]);
			if (ret) {
				kfree(aw87xxx->support_prof);
				ret = -EINVAL;
				goto err_put_node;
			}
		}
	}
	return 0;

err_put_node:
	of_node_put(aw87xxx->aw_dev.rst_pinctrl_np);
	return ret;
}

static void aw87xxx_check_rst(struct aw87xxx *new_dev)
{
	struct aw87xxx *aw87xxx;
	int cnt = 0;
	int idx;

	if (atomic_read(&g_aw87xxx_dev_cnt) == 1) {
		new_dev->aw_dev.rst_list_flag = 0;
		return;
	}

	idx = srcu_read_lock(&g_aw87xxx_srcu);
	list_for_each_entry_rcu(aw87xxx, &g_aw87xxx_list, node) {
		if (aw87xxx == new_dev)
			continue;

		if (aw87xxx->aw_dev.has_rst_gpio && new_dev->aw_dev.has_rst_gpio) {
			new_dev->aw_dev.rst_list_flag = aw87xxx->aw_dev.rst_list_flag;
			srcu_read_unlock(&g_aw87xxx_srcu, idx);
			return;
		}

		if (cnt < aw87xxx->aw_dev.rst_list_flag)
			cnt = aw87xxx->aw_dev.rst_list_flag;
	}
	srcu_read_unlock(&g_aw87xxx_srcu, idx);

	new_dev->aw_dev.rst_list_flag = cnt + 1;
}

static const struct regmap_config aw87xxx_regmap_config = {
	.val_bits = 8,
	.reg_bits = 8,
	.max_register = AW_REG_NONE,
};

static int aw87xxx_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device_node *dev_node = client->dev.of_node;
	struct device_node *rst_np;
	struct aw87xxx *aw87xxx;
	int ret;

	rst_np = of_parse_phandle(dev_node, "cdc-rst-pinctrl", 0);
	if (rst_np && !msm_cdc_pinctrl_is_ready(rst_np)) {
		of_node_put(rst_np);
		return -EPROBE_DEFER;
	}
	of_node_put(rst_np);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	aw87xxx = devm_kzalloc(&client->dev, sizeof(*aw87xxx), GFP_KERNEL);
	if (!aw87xxx)
		return -ENOMEM;

	aw87xxx->dev = &client->dev;
	aw87xxx->aw_dev.dev = &client->dev;
	aw87xxx->aw_dev.i2c_bus = client->adapter->nr;
	aw87xxx->aw_dev.i2c_addr = client->addr;
	aw87xxx->aw_dev.i2c = client;
	aw87xxx->aw_dev.hwen_status = AW_DEV_HWEN_INVALID;
	aw87xxx->current_profile = aw87xxx->current_profile_buf;
	strscpy(aw87xxx->current_profile_buf, "", AW_PROFILE_STR_MAX);

	aw87xxx->aw_dev.regmap = devm_regmap_init_i2c(client, &aw87xxx_regmap_config);
	if (IS_ERR(aw87xxx->aw_dev.regmap))
		return PTR_ERR(aw87xxx->aw_dev.regmap);

	i2c_set_clientdata(client, aw87xxx);

	ret = aw87xxx_dtsi_parse(aw87xxx, dev_node);
	if (ret < 0)
		return ret;

	aw87xxx_dev_hw_pwr_ctrl(&aw87xxx->aw_dev, true);

	ret = aw87xxx_dev_init(&aw87xxx->aw_dev);
	if (ret < 0) {
		aw87xxx_dev_hw_pwr_ctrl(&aw87xxx->aw_dev, false);
		goto err_free_prof;
	}

	aw87xxx_dev_soft_reset(&aw87xxx->aw_dev);
	aw87xxx_dev_hw_pwr_ctrl(&aw87xxx->aw_dev, false);

	ret = sysfs_create_group(&aw87xxx->dev->kobj, &aw87xxx_attribute_group);
	if (ret < 0)
		AW_DEV_LOGE(aw87xxx->dev, "failed to create sysfs nodes");

	aw87xxx_monitor_init(aw87xxx->dev, &aw87xxx->monitor, dev_node);
	aw87xxx_fw_load_init(aw87xxx);

	mutex_lock(&g_aw87xxx_list_lock);
	list_add_tail_rcu(&aw87xxx->node, &g_aw87xxx_list);
	mutex_unlock(&g_aw87xxx_list_lock);

	atomic_inc(&g_aw87xxx_dev_cnt);
	aw87xxx_check_rst(aw87xxx);
	aw87xxx_dev_add_dev_list(&aw87xxx->aw_dev);

	AW_DEV_LOGI(aw87xxx->dev, "succeed, dev_index=[%d], dev_cnt=[%d]",
		    aw87xxx->dev_index, atomic_read(&g_aw87xxx_dev_cnt));

	return 0;

err_free_prof:
	kfree(aw87xxx->support_prof);
	return ret;
}

static int aw87xxx_i2c_remove(struct i2c_client *client)
{
	struct aw87xxx *aw87xxx = i2c_get_clientdata(client);

	if (!aw87xxx)
		return 0;

	cancel_delayed_work_sync(&aw87xxx->fw_load_work);

	mutex_lock(&g_aw87xxx_list_lock);
	list_del_rcu(&aw87xxx->node);
	mutex_unlock(&g_aw87xxx_list_lock);

	synchronize_srcu(&g_aw87xxx_srcu);

	aw87xxx_fw_cfg_free(aw87xxx);
	aw87xxx_monitor_exit(&aw87xxx->monitor);
	sysfs_remove_group(&aw87xxx->dev->kobj, &aw87xxx_attribute_group);

	kfree(aw87xxx->support_prof);
	of_node_put(aw87xxx->aw_dev.rst_pinctrl_np);

	aw87xxx_dev_remove_dev_list(&aw87xxx->aw_dev);
	atomic_dec(&g_aw87xxx_dev_cnt);

	return 0;
}

static void aw87xxx_i2c_shutdown(struct i2c_client *client)
{
	struct aw87xxx *aw87xxx = i2c_get_clientdata(client);

	aw87xxx_update_profile(aw87xxx, aw87xxx->prof_off_name);
}

static const struct i2c_device_id aw87xxx_i2c_id[] = {
	{ AW87XXX_I2C_NAME, 0 },
	{ }
};

static const struct of_device_id extpa_of_match[] = {
	{ .compatible = "awinic,aw87xxx_pa" },
	{ },
};

#ifdef CONFIG_PM
static int aw87xxx_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw87xxx *aw87xxx = i2c_get_clientdata(client);

	aw87xxx->is_suspend = false;

	return 0;
}

static int aw87xxx_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw87xxx *aw87xxx = i2c_get_clientdata(client);

	aw87xxx->is_suspend = true;

	return 0;
}

static const struct dev_pm_ops aw87xxx_dev_pm_ops = {
	.suspend = aw87xxx_i2c_suspend,
	.resume  = aw87xxx_i2c_resume,
};
#endif

static struct i2c_driver aw87xxx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AW87XXX_I2C_NAME,
		.of_match_table = extpa_of_match,
#ifdef CONFIG_PM
		.pm = &aw87xxx_dev_pm_ops,
#endif
	},
	.probe = aw87xxx_i2c_probe,
	.remove = aw87xxx_i2c_remove,
	.shutdown = aw87xxx_i2c_shutdown,
	.id_table = aw87xxx_i2c_id,
};

static int __init aw87xxx_pa_init(void)
{
	int ret;

	AW_LOGI("driver version: %s", AW87XXX_DRIVER_VERSION);

	ret = i2c_add_driver(&aw87xxx_i2c_driver);
	if (ret < 0) {
		AW_LOGE("Unable to register driver, ret=%d", ret);
		return ret;
	}

	return 0;
}

static void __exit aw87xxx_pa_exit(void)
{
	i2c_del_driver(&aw87xxx_i2c_driver);
}

module_init(aw87xxx_pa_init);
module_exit(aw87xxx_pa_exit);

MODULE_AUTHOR("Barry <zhaozhongbo@awinic.com>");
MODULE_DESCRIPTION("awinic aw87xxx pa driver");
MODULE_LICENSE("GPL v2");
