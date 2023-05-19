// SPDX-License-Identifier: GPL-2.0
/*
 * aw87xxx_device.c  aw87xxx pa module
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/timer.h>

#include "aw87xxx_core.h"
#include "aw87xxx_dsp.h"
#include "aw87xxx_log.h"

static LIST_HEAD(g_dev_list);
static DEFINE_MUTEX(g_dev_list_lock);

/* aw87xxx variable */
static const char * const g_aw_pid_9b_product[] = {
	"aw87319",
};

static const char * const g_aw_pid_18_product[] = {
	"aw87358",
};

static const char * const g_aw_pid_39_product[] = {
	"aw87329",
	"aw87339",
	"aw87349",
};

static const char * const g_aw_pid_59_3x9_product[] = {
	"aw87359",
	"aw87389",
};

static const char * const g_aw_pid_59_5x9_product[] = {
	"aw87509",
	"aw87519",
	"aw87529",
	"aw87539",
};

static const char * const g_aw_pid_5a_product[] = {
	"aw87549",
	"aw87559",
	"aw87569",
	"aw87579",
	"aw81509",
	"aw87579G",
};

static const char * const g_aw_pid_76_product[] = {
	"aw87390",
	"aw87320",
	"aw87401",
	"aw87360",
	"aw87390G",
};

static const char * const g_aw_pid_60_product[] = {
	"aw87560",
	"aw87561",
	"aw87562",
	"aw87501",
	"aw87550",
};

static const char * const g_aw_pid_c1_product[] = {
	"aw87391",
	"aw87392",
	"aw87393",
	"aw87402",
	"aw87392e",
};

static const char * const g_aw_pid_c2_product[] = {
	"aw87565",
	"aw87566",
	"aw81564",
	"aw87567",
	"aw87564",
	"aw87580",
};

static const char * const g_aw_pid_c2_product3[] = {
	"aw87568",
	"aw87571",
};

static const struct aw_product_tab g_aw_pid_c2_product_tab[] = {
	{ ARRAY_SIZE(g_aw_pid_c2_product), g_aw_pid_c2_product },
	{ ARRAY_SIZE(g_aw_pid_c2_product), g_aw_pid_c2_product },
	{ ARRAY_SIZE(g_aw_pid_c2_product), g_aw_pid_c2_product },
	{ ARRAY_SIZE(g_aw_pid_c2_product3), g_aw_pid_c2_product3 }
};

static const char * const g_aw_pid_23_product[] = {
	"aw87394",
	"aw87395",
	"aw87490",
};

static int aw87xxx_dev_get_chipid(struct aw_device *aw_dev);

/*
 * reading and writing of I2C bus via regmap
 */
int aw87xxx_dev_i2c_write_byte(struct aw_device *aw_dev,
			       u8 reg_addr,
			       u8 reg_data)
{
	int ret;

	ret = regmap_write(aw_dev->regmap, reg_addr, reg_data);
	if (ret < 0)
		AW_DEV_LOGE(aw_dev->dev, "regmap write error=%d", ret);

	return ret;
}

int aw87xxx_dev_i2c_read_byte(struct aw_device *aw_dev,
			      u8 reg_addr,
			      u8 *reg_data)
{
	unsigned int val;
	int ret;

	ret = regmap_read(aw_dev->regmap, reg_addr, &val);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "regmap read error=%d", ret);
		return ret;
	}

	*reg_data = (u8)val;
	return 0;
}

int aw87xxx_dev_i2c_read_msg(struct aw_device *aw_dev,
			     u8 reg_addr,
			     u8 *data_buf,
			     u32 data_len)
{
	int ret;

	ret = regmap_bulk_read(aw_dev->regmap, reg_addr, data_buf, data_len);
	if (ret < 0)
		AW_DEV_LOGE(aw_dev->dev, "regmap bulk read error=%d", ret);

	return ret;
}

int aw87xxx_dev_i2c_write_bits(struct aw_device *aw_dev,
			       u8 reg_addr,
			       u8 mask,
			       u8 reg_data)
{
	int ret;

	ret = regmap_update_bits(aw_dev->regmap, reg_addr, (u8)~mask, reg_data);
	if (ret < 0)
		AW_DEV_LOGE(aw_dev->dev, "regmap update bits error=%d", ret);

	return ret;
}

/*
 * aw87xxx device update profile data to registers
 */
static void aw87xxx_dev_reg_mute_bits_set(struct aw_device *aw_dev,
					  u8 *reg_val, bool enable)
{
	if (enable) {
		*reg_val &= aw_dev->mute_desc.mask;
		*reg_val |= aw_dev->mute_desc.enable;
	} else {
		*reg_val &= aw_dev->mute_desc.mask;
		*reg_val |= aw_dev->mute_desc.disable;
	}
}

#define AW87XXX_BULK_MAX_RUN	64

static int aw87xxx_dev_reg_update_bulk(struct aw_device *aw_dev,
				       struct aw_data_container *profile_data)
{
	u8 bulk_buf[AW87XXX_BULK_MAX_RUN];
	size_t run_len;
	u8 next_reg, next_val;
	int i = 0;
	int ret;

	while (i < profile_data->len) {
		u8 reg_addr = profile_data->data[i];
		u8 reg_val = profile_data->data[i + 1];

		if (reg_addr == aw_dev->cm_volt_desc.addr)
			aw_dev->cm_volt_desc.init = reg_val &
						    ~aw_dev->cm_volt_desc.mask;

		if (reg_addr == AW87XXX_DELAY_REG_ADDR) {
			AW_DEV_LOGI(aw_dev->dev, "delay %d ms", reg_val);
			usleep_range(reg_val * AW87XXX_REG_DELAY_TIME,
				     reg_val * AW87XXX_REG_DELAY_TIME + 10);
			i += 2;
			continue;
		}

		if (aw_dev->mute_desc.addr != AW_REG_NONE &&
		    reg_addr == aw_dev->mute_desc.addr)
			aw87xxx_dev_reg_mute_bits_set(aw_dev, &reg_val, true);

		bulk_buf[0] = reg_val;
		run_len = 1;

		while (i + run_len * 2 < profile_data->len &&
		       run_len < AW87XXX_BULK_MAX_RUN) {
			next_reg = profile_data->data[i + run_len * 2];
			next_val = profile_data->data[i + run_len * 2 + 1];

			if (next_reg != (u8)(reg_addr + run_len) ||
			    next_reg == AW87XXX_DELAY_REG_ADDR ||
			    next_reg == AW_REG_NONE)
				break;

			if (aw_dev->mute_desc.addr != AW_REG_NONE &&
			    next_reg == aw_dev->mute_desc.addr)
				aw87xxx_dev_reg_mute_bits_set(aw_dev, &next_val, true);

			if (next_reg == aw_dev->cm_volt_desc.addr)
				aw_dev->cm_volt_desc.init = next_val &
							    ~aw_dev->cm_volt_desc.mask;

			bulk_buf[run_len] = next_val;
			run_len++;
		}

		if (run_len >= 2) {
			ret = regmap_bulk_write(aw_dev->regmap, reg_addr,
						bulk_buf, run_len);
			if (ret < 0) {
				AW_DEV_LOGE(aw_dev->dev,
					    "bulk write [0x%02x..0x%02x] failed: %d",
					    reg_addr, (u8)(reg_addr + run_len - 1),
					    ret);
				return ret;
			}
			i += run_len * 2;
		} else {
			ret = regmap_write(aw_dev->regmap, reg_addr, reg_val);
			if (ret < 0) {
				AW_DEV_LOGE(aw_dev->dev,
					    "regmap write 0x%02x=0x%02x failed: %d",
					    reg_addr, reg_val, ret);
				return ret;
			}
			i += 2;
		}
	}

	return 0;
}

static int aw87xxx_dev_reg_update(struct aw_device *aw_dev,
				  struct aw_data_container *profile_data)
{
	int i, ret;
	u8 reg_addr = 0;
	u8 reg_val = 0;

	if (!profile_data)
		return -EINVAL;

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "hwen is off, can not update reg");
		return -EINVAL;
	}

	/* PID 5A uses optimized chunked bulk write */
	if (aw_dev->chipid == AW_DEV_CHIPID_5A)
		return aw87xxx_dev_reg_update_bulk(aw_dev, profile_data);

	for (i = 0; i < profile_data->len; i += 2) {
		reg_addr = profile_data->data[i];
		reg_val = profile_data->data[i + 1];
		AW_DEV_LOGI(aw_dev->dev, "reg=0x%02x, val=0x%02x",
			    reg_addr, reg_val);

		if (reg_addr == aw_dev->cm_volt_desc.addr) {
			aw_dev->cm_volt_desc.init = reg_val &
						    ~aw_dev->cm_volt_desc.mask;
		}

		/* delay ms */
		if (reg_addr == AW87XXX_DELAY_REG_ADDR) {
			AW_DEV_LOGI(aw_dev->dev, "delay %d ms", reg_val);
			usleep_range(reg_val * AW87XXX_REG_DELAY_TIME,
				     reg_val * AW87XXX_REG_DELAY_TIME + 10);
			continue;
		}

		if (aw_dev->mute_desc.addr != AW_REG_NONE &&
		    reg_addr == aw_dev->mute_desc.addr) {
			aw87xxx_dev_reg_mute_bits_set(aw_dev, &reg_val, true);
			AW_DEV_LOGD(aw_dev->dev,
				    "change mute_mask, val=0x%02x", reg_val);
		}

		ret = regmap_write(aw_dev->regmap, reg_addr, reg_val);
		if (ret < 0) {
			AW_DEV_LOGE(aw_dev->dev, "regmap write failed: %d", ret);
			return ret;
		}
	}

	return 0;
}

/*
 * aw87xxx device hadware and soft contols
 */
void aw87xxx_dev_add_dev_list(struct aw_device *aw_dev)
{
	mutex_lock(&g_dev_list_lock);
	list_add(&aw_dev->list, &g_dev_list);
	mutex_unlock(&g_dev_list_lock);
}

void aw87xxx_dev_remove_dev_list(struct aw_device *aw_dev)
{
	mutex_lock(&g_dev_list_lock);
	if (!list_empty(&aw_dev->list))
		list_del_init(&aw_dev->list);
	mutex_unlock(&g_dev_list_lock);
}

static bool aw87xxx_dev_gpio_is_valid(struct aw_device *aw_dev)
{
	return (aw_dev->rst_pin_ctrl != NULL) || aw_dev->has_rst_gpio;
}

void aw87xxx_dev_hw_pwr_ctrl(struct aw_device *aw_dev, bool enable)
{
	struct list_head *pos;
	struct aw_device *dev_check;
	int shared_is_active = 0;
	struct gpio_desc *desc;

	if (aw_dev->hwen_status == AW_DEV_HWEN_INVALID) {
		AW_DEV_LOGD(aw_dev->dev,
			    "no reset-pin, hardware pwd control invalid");
		return;
	}

	if (!aw87xxx_dev_gpio_is_valid(aw_dev)) {
		AW_DEV_LOGI(aw_dev->dev,
			    "hw already power %s", enable ? "on" : "off");
		return;
	}

	if (!enable && aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGD(aw_dev->dev,
			    "hw already off, skip duplicate ctrl");
		return;
	}

	if (!enable) {
		mutex_lock(&g_dev_list_lock);
		list_for_each(pos, &g_dev_list) {
			dev_check = list_entry(pos, struct aw_device, list);
			if (dev_check != aw_dev &&
			    aw_dev->rst_list_flag == dev_check->rst_list_flag &&
			    dev_check->hwen_status == AW_DEV_HWEN_ON) {
				shared_is_active = 1;
				break;
			}
		}
		mutex_unlock(&g_dev_list_lock);

		if (shared_is_active) {
			aw_dev->hwen_status = AW_DEV_HWEN_OFF;
			AW_DEV_LOGI(aw_dev->dev,
				    "hw power off (shared pin kept active by another device)");
			return;
		}
	}

	if (aw_dev->rst_pin_ctrl) {
		if (enable) {
			aw_dev->rst_pin_ctrl(aw_dev, true);
			mdelay(2);
			aw_dev->hwen_status = AW_DEV_HWEN_ON;
			AW_DEV_LOGI(aw_dev->dev, "hw power real on (pinctrl)");
		} else {
			aw_dev->rst_pin_ctrl(aw_dev, false);
			mdelay(2);
			aw_dev->hwen_status = AW_DEV_HWEN_OFF;
			AW_DEV_LOGI(aw_dev->dev, "hw power real off (pinctrl)");
		}
		return;
	}

	if (enable) {
		desc = gpiod_get_optional(aw_dev->dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(desc)) {
			AW_DEV_LOGE(aw_dev->dev,
				    "reset get failed on power on: %ld", PTR_ERR(desc));
			return;
		}
		if (desc) {
			mdelay(2);
			gpiod_put(desc);
		}
		aw_dev->hwen_status = AW_DEV_HWEN_ON;
		AW_DEV_LOGI(aw_dev->dev, "hw power real on (gpiod)");
	} else {
		desc = gpiod_get_optional(aw_dev->dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(desc)) {
			AW_DEV_LOGE(aw_dev->dev,
				    "reset get failed on power off: %ld", PTR_ERR(desc));
		} else if (desc) {
			mdelay(2);
			gpiod_put(desc);
			AW_DEV_LOGI(aw_dev->dev,
				    "hw power real off (gpiod)");
		}
		aw_dev->hwen_status = AW_DEV_HWEN_OFF;
	}
}

static int aw87xxx_dev_mute_ctrl(struct aw_device *aw_dev, bool enable)
{
	int ret;

	if (enable) {
		ret = aw87xxx_dev_i2c_write_bits(aw_dev,
						 aw_dev->mute_desc.addr,
						 aw_dev->mute_desc.mask,
						 aw_dev->mute_desc.enable);
		if (ret < 0)
			return ret;

		AW_DEV_LOGI(aw_dev->dev, "set mute down");
	} else {
		ret = aw87xxx_dev_i2c_write_bits(aw_dev,
						 aw_dev->mute_desc.addr,
						 aw_dev->mute_desc.mask,
						 aw_dev->mute_desc.disable);
		if (ret < 0)
			return ret;

		AW_DEV_LOGI(aw_dev->dev, "close mute down");
	}

	return 0;
}

void aw87xxx_dev_soft_reset(struct aw_device *aw_dev)
{
	int ret;

	AW_DEV_LOGD(aw_dev->dev, "enter");

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "hw is off, can not softrst");
		return;
	}

	ret = aw87xxx_dev_i2c_write_byte(aw_dev,
					 AW87XXX_CHIPIDL_REG,
					 AW87XXX_SW_RESET_PASSWORD);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "write failed, ret=%d", ret);
		return;
	}

	AW_DEV_LOGD(aw_dev->dev, "down");
}

int aw87xxx_dev_default_pwr_off(struct aw_device *aw_dev,
				struct aw_data_container *profile_data)
{
	int ret;

	AW_DEV_LOGD(aw_dev->dev, "enter");

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "hwen is already off");
		return 0;
	}

	if (aw_dev->soft_off_enable && profile_data) {
		ret = aw87xxx_dev_reg_update(aw_dev, profile_data);
		if (ret < 0) {
			AW_DEV_LOGE(aw_dev->dev,
				    "update profile[Off] fw config failed");
			goto reg_off_update_failed;
		}
	}

	if (aw_dev->delay_desc.power_off_delay_ms > 0)
		mdelay(aw_dev->delay_desc.power_off_delay_ms);

	aw87xxx_dev_hw_pwr_ctrl(aw_dev, false);
	AW_DEV_LOGD(aw_dev->dev, "down");
	return 0;

reg_off_update_failed:
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, false);
	return ret;
}

int aw87xxx_dev_soft_off_only(struct aw_device *aw_dev,
			      struct aw_data_container *profile_data)
{
	int ret = 0;

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGD(aw_dev->dev, "soft_off_only: hw already off, skip");
		return 0;
	}

	if (aw_dev->soft_off_enable && profile_data) {
		ret = aw87xxx_dev_reg_update(aw_dev, profile_data);
		if (ret < 0)
			AW_DEV_LOGE(aw_dev->dev,
				    "soft_off_only: reg write failed (ret=%d)",
				    ret);
	}

	return ret;
}

/*
 * aw87xxx device power on process function
 */
int aw87xxx_dev_default_pwr_on(struct aw_device *aw_dev,
			       struct aw_data_container *profile_data)
{
	int ret;

	/* hw power on */
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, true);

	if (aw_dev->delay_desc.power_on_delay_ms > 0)
		mdelay(aw_dev->delay_desc.power_on_delay_ms);

	if (aw_dev->mute_desc.addr != AW_REG_NONE) {
		/* open the mute */
		ret = aw87xxx_dev_mute_ctrl(aw_dev, true);
		if (ret < 0)
			goto pwr_on_failed;
	}

	ret = aw87xxx_dev_reg_update(aw_dev, profile_data);
	if (ret < 0)
		goto pwr_on_failed;

	if (aw_dev->mute_desc.addr != AW_REG_NONE) {
		/* close the mute */
		ret = aw87xxx_dev_mute_ctrl(aw_dev, false);
		if (ret < 0)
			goto pwr_on_failed;
	}

	return 0;

pwr_on_failed:
	/* roll back to a known-OFF state so caller does not have to */
	AW_DEV_LOGE(aw_dev->dev,
		    "pwr_on failed (ret=%d), rolling back to OFF",
		    ret);
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, false);
	return ret;
}

/*
 * aw87xxx chip esd status check
 */
int aw87xxx_dev_esd_reg_status_check(struct aw_device *aw_dev)
{
	int ret;
	u8 reg_val = 0;
	struct aw_esd_check_desc *esd_desc = &aw_dev->esd_desc;

	AW_DEV_LOGD(aw_dev->dev, "enter");

	if (!esd_desc->first_update_reg_addr) {
		AW_DEV_LOGE(aw_dev->dev,
			    "esd check info if not init, please check");
		return -EINVAL;
	}

	ret = aw87xxx_dev_i2c_read_byte(aw_dev,
					esd_desc->first_update_reg_addr,
					&reg_val);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read reg 0x%02x failed",
			    esd_desc->first_update_reg_addr);
		return ret;
	}

	AW_DEV_LOGD(aw_dev->dev, "0x%02x: default val=0x%02x, real val=0x%02x",
		    esd_desc->first_update_reg_addr,
		    esd_desc->first_update_reg_val, reg_val);

	if (reg_val == esd_desc->first_update_reg_val) {
		AW_DEV_LOGE(aw_dev->dev, "reg status check failed");
		return -EINVAL;
	}

	return 0;
}

int aw87xxx_dev_check_reg_is_rec_mode(struct aw_device *aw_dev)
{
	int ret;
	u8 reg_val = 0;
	struct aw_rec_mode_desc *rec_desc = &aw_dev->rec_desc;
	bool is_rec;

	if (!rec_desc->addr) {
		AW_DEV_LOGE(aw_dev->dev, "rec check info if not init");
		return -EINVAL;
	}

	ret = aw87xxx_dev_i2c_read_byte(aw_dev, rec_desc->addr, &reg_val);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read reg 0x%02x failed",
			    rec_desc->addr);
		return ret;
	}

	is_rec = !!(reg_val & ~rec_desc->mask);

	if ((rec_desc->enable && is_rec) || (!rec_desc->enable && !is_rec)) {
		AW_DEV_LOGI(aw_dev->dev, "reg status is receiver mode");
		aw_dev->is_rec_mode = AW_IS_REC_MODE;
	} else {
		aw_dev->is_rec_mode = AW_NOT_REC_MODE;
	}

	return 0;
}

static u8 aw_dev_check_product(struct aw_device *aw_dev,
			       const struct aw_mark_desc *mark,
			       const struct aw_product_tab *product,
			       u8 count)
{
	u8 reg_val = 0;
	u8 index;

	if (mark->addr != AW_REG_NONE) {
		aw87xxx_dev_i2c_write_bits(aw_dev,
					   aw_dev->en_desc.addr,
					   aw_dev->en_desc.mask,
					   aw_dev->en_desc.enable);
		mdelay(1);
		aw87xxx_dev_i2c_read_byte(aw_dev, mark->addr, &reg_val);
		aw87xxx_dev_i2c_write_bits(aw_dev,
					   aw_dev->en_desc.addr,
					   aw_dev->en_desc.mask,
					   aw_dev->en_desc.disable);

		reg_val = (reg_val & ~mark->mask) >> mark->start;
	}

	index = (reg_val >= count) ? 0 : reg_val;
	aw_dev->product_tab = product[index].product_tab;
	aw_dev->product_cnt = product[index].count;

	return reg_val;
}

/* aw87xxx_pid_9A attributes */

static int aw_dev_pid_9b_reg_update(struct aw_device *aw_dev,
				    struct aw_data_container *profile_data)
{
	int i, ret;
	u8 reg_val = 0;

	if (!profile_data)
		return -EINVAL;

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "dev is pwr_off, can not update reg");
		return -EINVAL;
	}

	if (profile_data->len != AW_PID_9B_BIN_REG_CFG_COUNT) {
		AW_DEV_LOGE(aw_dev->dev,
			    "reg_config count of bin is error");
		return -EINVAL;
	}

	ret = aw87xxx_dev_i2c_write_byte(aw_dev,
					 AW87XXX_PID_9B_ENCRYPTION_REG,
					 AW87XXX_PID_9B_ENCRYPTION_BOOST_OUTPUT_SET);
	if (ret < 0)
		return ret;

	for (i = 1; i < AW_PID_9B_BIN_REG_CFG_COUNT; i++) {
		AW_DEV_LOGI(aw_dev->dev, "reg=0x%02x, val=0x%02x",
			    i, profile_data->data[i]);

		/* delay ms */
		if (profile_data->data[i] == AW87XXX_DELAY_REG_ADDR) {
			AW_DEV_LOGI(aw_dev->dev, "delay %d ms",
				    profile_data->data[i + 1]);
			usleep_range(profile_data->data[i + 1] * AW87XXX_REG_DELAY_TIME,
				     profile_data->data[i + 1] * AW87XXX_REG_DELAY_TIME + 10);
			continue;
		}

		reg_val = profile_data->data[i];
		if (i == AW87XXX_PID_9B_SYSCTRL_REG) {
			aw87xxx_dev_reg_mute_bits_set(aw_dev, &reg_val, true);
			AW_DEV_LOGD(aw_dev->dev,
				    "change mute_mask, val=0x%02x",
				    reg_val);
		}

		ret = aw87xxx_dev_i2c_write_byte(aw_dev, i, reg_val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int aw_dev_pid_9b_pwr_on(struct aw_device *aw_dev,
				struct aw_data_container *data)
{
	int ret;

	/* hw power on */
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, true);

	/* open the mute */
	ret = aw87xxx_dev_mute_ctrl(aw_dev, true);
	if (ret < 0)
		goto pwr_on_failed;

	/* Update scene parameters in mute mode */
	ret = aw_dev_pid_9b_reg_update(aw_dev, data);
	if (ret < 0)
		goto pwr_on_failed;

	/* close the mute */
	ret = aw87xxx_dev_mute_ctrl(aw_dev, false);
	if (ret < 0)
		goto pwr_on_failed;

	return 0;

pwr_on_failed:
	/* Roll back to a known-OFF state so caller does not have to.
	 * Mirrors aw87xxx_dev_default_pwr_on() behavior for consistency.
	 */
	AW_DEV_LOGE(aw_dev->dev,
		    "pid_9b pwr_on failed (ret=%d), rolling back to OFF",
		    ret);
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, false);
	return ret;
}

static void aw_dev_init_defaults(struct aw_device *aw_dev)
{
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;
	aw_dev->delay_desc.power_on_delay_ms = 0;
	aw_dev->delay_desc.power_off_delay_ms = 0;
	aw_dev->en_desc.addr = AW_REG_NONE;
	aw_dev->mute_desc.addr = AW_REG_NONE;
	aw_dev->rec_desc.addr = AW_REG_NONE;
	aw_dev->esd_desc.first_update_reg_addr = AW_REG_NONE;
	aw_dev->esd_desc.first_update_reg_val = 0;
	aw_dev->ipeak_desc.reg = AW_REG_NONE;
	aw_dev->vol_desc.addr = AW_REG_NONE;
	aw_dev->cm_volt_desc.addr = AW_REG_NONE;
	aw_dev->ef_desc.count = 0;
	aw_dev->pwr_on_func = NULL;
}

static int aw_dev_pid_9b_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_9B_REG_MAX;
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_DISENABLE;
	aw_dev->pwr_on_func = aw_dev_pid_9b_pwr_on;

	aw_dev->mute_desc.addr = AW87XXX_PID_9B_SYSCTRL_REG;
	aw_dev->mute_desc.mask = AW87XXX_PID_9B_REG_EN_SW_MASK;
	aw_dev->mute_desc.enable = AW87XXX_PID_9B_REG_EN_SW_DISABLE_VALUE;
	aw_dev->mute_desc.disable = AW87XXX_PID_9B_REG_EN_SW_ENABLE_VALUE;

	aw_dev->rec_desc.addr = AW87XXX_PID_9B_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_9B_SPK_MODE_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_9B_SPK_MODE_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_9B_SPK_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_9B_SYSCTRL_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_9B_SYSCTRL_DEFAULT;

	return 0;
}

static int aw_dev_pid_9a_init(struct aw_device *aw_dev)
{
	int ret;

	ret = aw87xxx_dev_i2c_write_byte(aw_dev,
					 AW87XXX_PID_9B_ENCRYPTION_REG,
					 AW87XXX_PID_9B_ENCRYPTION_BOOST_OUTPUT_SET);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "write 0x64=0x2C error");
		return -EINVAL;
	}

	ret = aw87xxx_dev_get_chipid(aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read chipid is failed, ret=%d", ret);
		return ret;
	}

	if (aw_dev->chipid == AW_DEV_CHIPID_9B) {
		AW_DEV_LOGI(aw_dev->dev, "product is pid_9B class");
		aw_dev_pid_9b_init(aw_dev);
	} else {
		AW_DEV_LOGE(aw_dev->dev, "product is not pid_9B class");
		return -EINVAL;
	}

	return 0;
}

/* aw87xxx_pid_9b attributes end */

/* aw87xxx_pid_18 attributes */
static int aw_dev_pid_18_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_18_REG_MAX;

	aw_dev->mute_desc.addr = AW87XXX_PID_18_SYSCTRL_REG;
	aw_dev->mute_desc.mask = AW87XXX_PID_18_REG_EN_SW_MASK;
	aw_dev->mute_desc.enable = AW87XXX_PID_18_REG_EN_SW_DISABLE_VALUE;
	aw_dev->mute_desc.disable = AW87XXX_PID_18_REG_EN_SW_ENABLE_VALUE;

	aw_dev->rec_desc.addr = AW87XXX_PID_18_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_18_REG_REC_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_18_REG_REC_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_18_REG_REC_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_18_CLASSD_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_18_CLASSD_DEFAULT;

	aw_dev->vol_desc.addr = AW87XXX_PID_18_CPOC_REG;
	aw_dev->vol_desc.mask = AW87XXX_CP_OVP_MASK;
	aw_dev->vol_desc.start = AW87XXX_CP_OVP_START;

	return 0;
}

/* aw87xxx_pid_18 attributes end */

/* aw87xxx_pid_39 attributes */
static int aw_dev_pid_39_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_39_REG_MAX;

	aw_dev->rec_desc.addr = AW87XXX_PID_39_REG_MODECTRL;
	aw_dev->rec_desc.disable = AW87XXX_PID_39_REC_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_39_REC_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_39_REC_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_39_REG_MODECTRL;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_39_MODECTRL_DEFAULT;

	aw_dev->vol_desc.addr = AW87XXX_PID_39_REG_CPOVP;
	aw_dev->vol_desc.mask = AW87XXX_CP_OVP_MASK;
	aw_dev->vol_desc.start = AW87XXX_CP_OVP_START;

	return 0;
}

/* aw87xxx_pid_39 attributes end */

/* aw87xxx_pid_59 attributes */
static int aw_dev_pid_59_5x9_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_59_5X9_REG_MAX;
	aw_dev->product_tab = g_aw_pid_59_5x9_product;
	aw_dev->product_cnt = ARRAY_SIZE(g_aw_pid_59_5x9_product);

	aw_dev->rec_desc.addr = AW87XXX_PID_59_5X9_REG_SYSCTRL;
	aw_dev->rec_desc.disable = AW87XXX_PID_59_5X9_REC_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_59_5X9_REC_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_59_5X9_REC_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_59_5X9_REG_ENCR;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_59_5X9_ENCRY_DEFAULT;

	return 0;
}

static int aw_dev_pid_59_3x9_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_59_3X9_REG_MAX;
	aw_dev->product_tab = g_aw_pid_59_3x9_product;
	aw_dev->product_cnt = ARRAY_SIZE(g_aw_pid_59_3x9_product);

	aw_dev->rec_desc.addr = AW87XXX_PID_59_3X9_REG_MDCRTL;
	aw_dev->rec_desc.disable = AW87XXX_PID_59_3X9_SPK_MODE_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_59_3X9_SPK_MODE_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_59_3X9_SPK_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_59_3X9_REG_ENCR;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_59_3X9_ENCR_DEFAULT;

	aw_dev->vol_desc.addr = AW87XXX_PID_59_3X9_REG_CPOVP;
	aw_dev->vol_desc.mask = AW87XXX_CP_OVP_MASK;
	aw_dev->vol_desc.start = AW87XXX_CP_OVP_START;

	return 0;
}

static int aw_dev_pid_59_init(struct aw_device *aw_dev)
{
	if (aw87xxx_dev_gpio_is_valid(aw_dev))
		return aw_dev_pid_59_5x9_init(aw_dev);

	return aw_dev_pid_59_3x9_init(aw_dev);
}

/* aw87xxx_pid_59 attributes end */

static int aw_dev_pid_5a_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_5A_REG_MAX;

	aw_dev->rec_desc.addr = AW87XXX_PID_5A_REG_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_5A_REG_RCV_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_5A_REG_RCV_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_5A_REG_RCV_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_5A_REG_DFT3R_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_5A_DFT3R_DEFAULT;

	aw_dev->ipeak_desc.reg = AW87XXX_PID_5A_REG_BSTCPR2_REG;
	aw_dev->ipeak_desc.mask = AW87XXX_PID_5A_REG_BST_IPEAK_MASK;

	return 0;
}

/* aw87xxx_pid_5a attributes end */

/* aw87xxx_pid_76 attributes */
static int aw_dev_pid_76_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_76_REG_MAX;

	aw_dev->rec_desc.addr = AW87XXX_PID_76_MDCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_76_EN_SPK_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_76_EN_SPK_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_76_EN_SPK_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_76_DFT_ADP1_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_76_DFT_ADP1_CHECK;

	aw_dev->vol_desc.addr = AW87XXX_PID_76_CPOVP_REG;
	aw_dev->vol_desc.mask = AW87XXX_CP_OVP_MASK;
	aw_dev->vol_desc.start = AW87XXX_CP_OVP_START;

	return 0;
}

/* aw87xxx_pid_76 attributes end */

/* aw87xxx_pid_60 attributes */
static int aw_dev_pid_60_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_60_REG_MAX;

	aw_dev->rec_desc.addr = AW87XXX_PID_60_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_60_RCV_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_60_RCV_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_60_RCV_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_60_NG3_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_60_ESD_REG_VAL;

	return 0;
}

/* aw87xxx_pid_60 attributes end */

/* aw87xxx_pid_c1 attributes */
static int aw_dev_pid_c1_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_C1_REG_MAX;

	aw_dev->en_desc.addr = AW87XXX_PID_C1_SYSCTRL_REG;
	aw_dev->en_desc.disable = AW87XXX_PID_C1_EN_SE_DISABLE_VALUE;
	aw_dev->en_desc.enable = AW87XXX_PID_C1_EN_SE_ENABLE_VALUE;
	aw_dev->en_desc.mask = AW87XXX_PID_C1_EN_SW_MASK;

	aw_dev->rec_desc.addr = AW87XXX_PID_C1_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_C1_EN_SPK_SPK_MODE_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_C1_EN_SPK_SPK_MODE_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_C1_EN_SPK_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_C1_DFT_THGEN1_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_C1_DFT_THGEN1_CHECK;

	aw_dev->ef_desc.count = 2;
	aw_dev->ef_desc.sequence[0].reg = AW87XXX_PID_C1_EFRH2_REG;
	aw_dev->ef_desc.sequence[0].mask = AW87XXX_EF_LOCK_MASK;
	aw_dev->ef_desc.sequence[0].check_val = AW87XXX_EF_LOCK_ENABLE_VALUE;
	aw_dev->ef_desc.sequence[1].reg = AW87XXX_PID_C1_EFRL2_REG;
	aw_dev->ef_desc.sequence[1].mask = AW87XXX_EF_LOCK_MASK;
	aw_dev->ef_desc.sequence[1].check_val = AW87XXX_EF_LOCK_ENABLE_VALUE;

	return 0;
}

/* aw87xxx_pid_c1 attributes end */

/* aw87xxx_pid_c2 attributes */
static int aw_dev_pid_c2_init(struct aw_device *aw_dev)
{
	int ret;
	u8 reg_val;
	struct aw_mark_desc mark;

	aw_dev->reg_max_addr = AW87XXX_PID_C2_REG_MAX;
	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_C2_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_C2_POWER_OFF_DELAY_MS;

	aw_dev->en_desc.addr = AW87XXX_PID_C2_SYSCTRL_REG;
	aw_dev->en_desc.disable = AW87XXX_PID_C2_EN_SW_DISABLE_VALUE;
	aw_dev->en_desc.enable = AW87XXX_PID_C2_EN_SW_ENABLE_VALUE;
	aw_dev->en_desc.mask = AW87XXX_PID_C2_EN_SW_MASK;

	aw_dev->rec_desc.addr = AW87XXX_PID_C2_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_C2_RCV_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_C2_RCV_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_C2_RCV_MODE_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_C2_CP_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_C2_CP_CHECK;

	aw_dev->ipeak_desc.reg = AW87XXX_PID_C2_PEAKLIMIT_REG;
	aw_dev->ipeak_desc.mask = AW87XXX_PID_C2_BST_IPEAK_MASK;

	aw_dev->ef_desc.count = 2;
	aw_dev->ef_desc.sequence[0].reg = AW87XXX_PID_C2_EFRHH_REG;
	aw_dev->ef_desc.sequence[0].mask = AW87XXX_EF_LOCK_MASK;
	aw_dev->ef_desc.sequence[0].check_val = AW87XXX_EF_LOCK_ENABLE_VALUE;
	aw_dev->ef_desc.sequence[1].reg = AW87XXX_PID_C2_EFRHL_REG;
	aw_dev->ef_desc.sequence[1].mask = AW87XXX_EF_LOCK_MASK;
	aw_dev->ef_desc.sequence[1].check_val = AW87XXX_EF_LOCK_ENABLE_VALUE;

	mark.addr = AW87XXX_PID_C2_EFRHH_REG;
	mark.start = AW87XXX_PID_C2_EF_VERSION_ID_START_BIT;
	mark.mask = AW87XXX_PID_C2_EF_VERSION_ID_MASK;

	reg_val = aw_dev_check_product(aw_dev, &mark,
				       g_aw_pid_c2_product_tab,
				       ARRAY_SIZE(g_aw_pid_c2_product_tab));

	ret = aw87xxx_dev_i2c_read_byte(aw_dev,
					AW87XXX_PID_C2_VERSION_REG,
					&reg_val);
	if (ret < 0)
		return ret;

	reg_val = (reg_val & ~AW87XXX_PID_C2_VERSION_MASK) >>
		  AW87XXX_PID_C2_VERSION_START_BIT;
	aw_dev->version = reg_val;
	AW_DEV_LOGI(aw_dev->dev, "read c2 version=0x%x", reg_val);

	return ret;
}

/* aw87xxx_pid_c2 attributes end */

/* aw87xxx_pid_23 attributes */
static int aw_dev_pid_23_init(struct aw_device *aw_dev)
{
	aw_dev->reg_max_addr = AW87XXX_PID_23_REG_MAX;

	aw_dev->rec_desc.addr = AW87XXX_PID_23_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_23_EN_SPK_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_23_EN_SPK_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_23_EN_SPK_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_23_ESD_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_23_ESD_CHECK;

	return 0;
}

/* aw87xxx_pid_23 attributes end */

static const struct aw_dev_property g_aw_dev_property_registry[] = {
	{ AW_DEV_CHIPID_9A, NULL,
		0, aw_dev_pid_9a_init },
	{ AW_DEV_CHIPID_9B, g_aw_pid_9b_product,
		ARRAY_SIZE(g_aw_pid_9b_product), aw_dev_pid_9b_init },
	{ AW_DEV_CHIPID_18, g_aw_pid_18_product,
		ARRAY_SIZE(g_aw_pid_18_product), aw_dev_pid_18_init },
	{ AW_DEV_CHIPID_39, g_aw_pid_39_product,
		ARRAY_SIZE(g_aw_pid_39_product), aw_dev_pid_39_init },
	{ AW_DEV_CHIPID_59, NULL,
		0, aw_dev_pid_59_init },
	{ AW_DEV_CHIPID_5A, g_aw_pid_5a_product,
		ARRAY_SIZE(g_aw_pid_5a_product), aw_dev_pid_5a_init },
	{ AW_DEV_CHIPID_76, g_aw_pid_76_product,
		ARRAY_SIZE(g_aw_pid_76_product), aw_dev_pid_76_init },
	{ AW_DEV_CHIPID_60, g_aw_pid_60_product,
		ARRAY_SIZE(g_aw_pid_60_product), aw_dev_pid_60_init },
	{ AW_DEV_CHIPID_C1, g_aw_pid_c1_product,
		ARRAY_SIZE(g_aw_pid_c1_product), aw_dev_pid_c1_init },
	{ AW_DEV_CHIPID_C2, g_aw_pid_c2_product,
		ARRAY_SIZE(g_aw_pid_c2_product), aw_dev_pid_c2_init },
	{ AW_DEV_CHIPID_23, g_aw_pid_23_product,
		ARRAY_SIZE(g_aw_pid_23_product), aw_dev_pid_23_init },
};

static int aw_dev_check_chip_model(struct aw_device *aw_dev)
{
	const struct aw_dev_property *entry;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(g_aw_dev_property_registry); i++) {
		entry = &g_aw_dev_property_registry[i];
		if (aw_dev->chipid == entry->id) {
			aw_dev_init_defaults(aw_dev);

			if (entry->product_tab) {
				aw_dev->product_tab = entry->product_tab;
				aw_dev->product_cnt = entry->product_cnt;
			}
			ret = entry->dev_init_func(aw_dev);

			if (ret < 0)
				AW_DEV_LOGE(aw_dev->dev,
					    "product is pid_%x init failed",
					    entry->id);
			else
				AW_DEV_LOGI(aw_dev->dev,
					    "product is pid_%x class",
					    entry->id);

			return ret;
		}
	}

	return -EINVAL;
}

static int aw_dev_chip_init(struct aw_device *aw_dev)
{
	int ret;

	ret = aw_dev_check_chip_model(aw_dev);
	if (!ret)
		return 0;

	aw_dev->chipid &= 0xFF;
	ret = aw_dev_check_chip_model(aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev,
			    "unsupported device revision [0x%x]",
			    aw_dev->chipid);
		return -EINVAL;
	}

	return 0;
}

static int aw87xxx_dev_get_chipid(struct aw_device *aw_dev)
{
	int ret;
	u32 cnt;
	u8 reg_val_l = 0;
	u8 reg_val_h = 0;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		ret = aw87xxx_dev_i2c_read_byte(aw_dev,
						AW87XXX_CHIPIDL_REG,
						&reg_val_l);
		if (ret >= 0)
			break;

		AW_DEV_LOGE(aw_dev->dev,
			    "[%d] read low id is failed, ret=%d",
			    cnt, ret);
	}
	if (cnt == AW_READ_CHIPID_RETRIES) {
		AW_DEV_LOGE(aw_dev->dev, "read low id is failed");
		return -EINVAL;
	}

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		ret = aw87xxx_dev_i2c_read_byte(aw_dev,
						AW87XXX_CHIPIDH_REG,
						&reg_val_h);
		if (ret >= 0)
			break;

		AW_DEV_LOGE(aw_dev->dev,
			    "[%d] read high id is failed, ret=%d",
			    cnt, ret);
	}

	if (cnt == AW_READ_CHIPID_RETRIES) {
		AW_DEV_LOGE(aw_dev->dev, "read high id is failed");
		return -EINVAL;
	}

	aw_dev->chipid = (int)reg_val_l | ((int)reg_val_h << 8);
	AW_DEV_LOGI(aw_dev->dev, "read chipid[0x%x] succeed", aw_dev->chipid);

	return 0;
}

static int aw87xxx_dev_check_ef_lock(struct aw_device *aw_dev)
{
	struct aw_ef_desc *ef_desc = &aw_dev->ef_desc;
	u32 i;
	u8 reg_val = 0;

	if (!ef_desc->count)
		return 0;

	if (aw_dev->en_desc.addr != AW_REG_NONE) {
		aw87xxx_dev_i2c_write_bits(aw_dev,
					   aw_dev->en_desc.addr,
					   aw_dev->en_desc.mask,
					   aw_dev->en_desc.enable);
		mdelay(1);
	}

	for (i = 0; i < ef_desc->count; i++) {
		aw87xxx_dev_i2c_read_byte(aw_dev,
					  ef_desc->sequence[i].reg,
					  &reg_val);
		if ((reg_val & ~ef_desc->sequence[i].mask) !=
		    ef_desc->sequence[i].check_val) {
			AW_DEV_LOGD(aw_dev->dev,
				    "ef check failed: 0x%x=0x%x",
				    ef_desc->sequence[i].reg, reg_val);
		}
	}

	if (aw_dev->en_desc.addr != AW_REG_NONE) {
		aw87xxx_dev_i2c_write_bits(aw_dev,
					   aw_dev->en_desc.addr,
					   aw_dev->en_desc.mask,
					   aw_dev->en_desc.disable);
	}

	return 0;
}

int aw87xxx_dev_init(struct aw_device *aw_dev)
{
	int ret;

	ret = aw87xxx_dev_get_chipid(aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read chipid is failed, ret=%d", ret);
		return ret;
	}

	ret = aw_dev_chip_init(aw_dev);
	if (ret < 0)
		return ret;

	ret = aw87xxx_dev_check_ef_lock(aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "ef has not been locked");
		return ret;
	}

	return 0;
}
