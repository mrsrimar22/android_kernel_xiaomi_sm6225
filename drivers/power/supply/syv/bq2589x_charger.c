// SPDX-License-Identifier: GPL-2.0-only
/*
 * otg-gpio BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 */

#define pr_fmt(fmt)	"[bq2589x_chg]: %s: " fmt, __func__

#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/ratelimit.h>
#include <linux/printk.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <asm/unaligned.h>
#include "bq2589x_reg.h"
#include "bq2589x_charger.h"

#define PROFILE_CHG_VOTER	"PROFILE_CHG_VOTER"
#define MAIN_SET_VOTER		"MAIN_SET_VOTER"
#define JEITA_VOTER		"JEITA_VOTER"
#define CHG_FCC_CURR_MAX	5950
#define CHG_FV_CURR_MAX		4450
#define CHG_ICL_CURR_MAX	2950
#define NOTIFY_COUNT_MAX	40
#define NO_CHANGE_MAX		5
#define RETRY_MS		500
#define RETRY_TIMEOUT_MS	10000
#define POST_INTERVAL		(2 * HZ)
#define RESET_GAP		(5 * HZ)
#define MAIN_ICL_MIN		100
#define MAIN_FCC_MIN		500
#define PROBE_CNT_MAX		50

#define bq_log(fmt, ...)						\
do {									\
	if (bq && __ratelimit(&bq->rl_slots[_BQ_ID() &			\
					    (BQ_RL_SLOTS - 1)]))	\
		pr_info(fmt, ##__VA_ARGS__);				\
} while (0)

#define bq_err(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define bq_info(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define bq_dbg(fmt, ...)	pr_debug(fmt, ##__VA_ARGS__)

static const struct regmap_config bq2589x_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= BQ2589X_REG_14,
};

struct class *bq2589x_class;
EXPORT_SYMBOL_GPL(bq2589x_class);

int bq_charger_register_notifier(struct bq2589x *bq, struct notifier_block *nb)
{
	if (!bq)
		return -EINVAL;

	return srcu_notifier_chain_register(&bq->evt_notifier, nb);
}
EXPORT_SYMBOL_GPL(bq_charger_register_notifier);

int bq_charger_unregister_notifier(struct bq2589x *bq, struct notifier_block *nb)
{
	if (!bq)
		return -EINVAL;

	return srcu_notifier_chain_unregister(&bq->evt_notifier, nb);
}
EXPORT_SYMBOL_GPL(bq_charger_unregister_notifier);

static int bbc_match_device_by_name(struct device *dev, const void *data)
{
	const char *name = data;
	struct bq2589x *bq = dev_get_drvdata(dev);

	return bq && strcmp(bq->name, name) == 0;
}

struct bq2589x *bbc_dev_get_by_name(const char *name)
{
	struct device *dev;

	if (!bq2589x_class)
		return NULL;

	dev = class_find_device(bq2589x_class, NULL,
				(const void *)name,
				bbc_match_device_by_name);

	return dev ? dev_get_drvdata(dev) : NULL;
}
EXPORT_SYMBOL_GPL(bbc_dev_get_by_name);

void bbc_dev_put(struct bq2589x *bq)
{
	if (!bq)
		return;

	put_device(&bq->class_dev);
}
EXPORT_SYMBOL_GPL(bbc_dev_put);

static void bq2589x_class_dev_release(struct device *dev)
{
}

static int bq2589x_read_byte(struct bq2589x *bq, u8 reg, u8 *data)
{
	int ret;
	unsigned int val;

	ret = regmap_read(bq->regmap, reg, &val);
	if (ret < 0) {
		bq_err("read 0x%02x failed: %d\n", reg, ret);
		*data = 0;
		return ret;
	}

	*data = (u8)val;
	return 0;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

	ret = regmap_write(bq->regmap, reg, data);
	if (ret < 0)
		bq_err("write 0x%02x->0x%02x failed: %d\n", data, reg, ret);

	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;

	ret = regmap_update_bits(bq->regmap, reg, mask, data);
	if (ret < 0)
		bq_err("update_bits 0x%02x (mask=0x%02x val=0x%02x) failed: %d\n",
		       reg, mask, data, ret);

	return ret;
}

static int bq2589x_enable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
				   BQ2589X_OTG_CONFIG_MASK,
				   val);
}

static int bq2589x_disable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
				   BQ2589X_OTG_CONFIG_MASK,
				   val);
}

static int bq2589x_set_otg_current(struct bq2589x *bq, int curr)
{
	u8 temp;

	if (curr < 0)
		curr = 0;

	if (bq->part_no == SC89890H) {
		if (curr < 600)
			temp = SC89890H_BOOST_LIM_500MA;
		else if (curr < 900)
			temp = SC89890H_BOOST_LIM_750MA;
		else if (curr < 1300)
			temp = SC89890H_BOOST_LIM_1200MA;
		else if (curr < 1500)
			temp = SC89890H_BOOST_LIM_1400MA;
		else if (curr < 1700)
			temp = SC89890H_BOOST_LIM_1650MA;
		else if (curr < 1900)
			temp = SC89890H_BOOST_LIM_1875MA;
		else if (curr < 2200)
			temp = SC89890H_BOOST_LIM_2150MA;
		else
			temp = SC89890H_BOOST_LIM_2450MA;
	} else {
		if (curr <= 500)
			temp = BQ2589X_BOOST_LIM_500MA;
		else if (curr > 500 && curr <= 800)
			temp = BQ2589X_BOOST_LIM_700MA;
		else if (curr > 800 && curr <= 1200)
			temp = BQ2589X_BOOST_LIM_1100MA;
		else if (curr > 1200 && curr <= 1400)
			temp = BQ2589X_BOOST_LIM_1300MA;
		else if (curr > 1400 && curr <= 1700)
			temp = BQ2589X_BOOST_LIM_1600MA;
		else if (curr > 1700 && curr <= 1900)
			temp = BQ2589X_BOOST_LIM_1800MA;
		else if (curr > 1900 && curr <= 2200)
			temp = BQ2589X_BOOST_LIM_2100MA;
		else if (curr > 2200 && curr <= 2300)
			temp = BQ2589X_BOOST_LIM_2400MA;
		else
			temp = BQ2589X_BOOST_LIM_2400MA;
	}

	return bq2589x_update_bits(bq, BQ2589X_REG_0A,
				   BQ2589X_BOOST_LIM_MASK,
				   temp << BQ2589X_BOOST_LIM_SHIFT);
}

static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				  BQ2589X_CHG_CONFIG_MASK,
				  val);
	if (!ret)
		set_bit(BQ2589X_STAT_CHARGE_ENABLE_BIT, &bq->status);

	return ret;
}

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				  BQ2589X_CHG_CONFIG_MASK,
				  val);
	if (!ret)
		clear_bit(BQ2589X_STAT_CHARGE_ENABLE_BIT, &bq->status);

	return ret;
}

static int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_02, &val);
	if (ret < 0) {
		bq_err("failed to read register 0x02:%d\n", ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
		return 0;

	if (oneshot)
		return bq2589x_update_bits(bq, BQ2589X_REG_02,
					   BQ2589X_CONV_START_MASK,
					   BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);

	return bq2589x_update_bits(bq, BQ2589X_REG_02,
				   BQ2589X_CONV_RATE_MASK,
				   BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
}

static int bq2589x_adc_stop(struct bq2589x *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02,
				   BQ2589X_CONV_RATE_MASK,
				   BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}

static int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0E, &val);
	if (ret < 0) {
		bq_err("read battery voltage failed: %d\n", ret);
		return ret;
	}

	return BQ2589X_BATV_BASE +
		((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB;
}

static int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_11, &val);
	if (ret < 0) {
		bq_err("read vbus voltage failed: %d\n", ret);
		return ret;
	}

	return BQ2589X_VBUSV_BASE +
		((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB;
}

static int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_12, &val);
	if (ret < 0) {
		bq_err("read charge current failed: %d\n", ret);
		return ret;
	}

	return BQ2589X_ICHGR_BASE +
		((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB;
}

static int bq2589x_set_charge_current(struct bq2589x *bq, int curr)
{
	u8 ichg;

	if (curr < 0)
		curr = 0;

	if (bq->part_no == SC89890H)
		ichg = (curr - SC89890H_ICHG_BASE) / SC89890H_ICHG_LSB;
	else
		ichg = (curr - BQ2589X_ICHG_BASE) / BQ2589X_ICHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_04,
				   BQ2589X_ICHG_MASK,
				   ichg << BQ2589X_ICHG_SHIFT);
}

static int bq2589x_get_charge_current(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_04, &val);
	if (ret < 0) {
		bq_err("get charge current failed: %d\n", ret);
		return ret;
	}

	return ((val & BQ2589X_ICHG_MASK) >> BQ2589X_ICHG_SHIFT) *
		BQ2589X_ICHG_LSB + BQ2589X_ICHG_BASE;
}

static int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	if (bq->part_no == SC89890H) {
		if (curr > SC89890H_ITERM_MAX)
			curr = SC89890H_ITERM_MAX;
		iterm = (curr - SC89890H_ITERM_BASE) / SC89890H_ITERM_LSB;
	} else {
		iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;
	}

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
				   BQ2589X_ITERM_MASK,
				   iterm << BQ2589X_ITERM_SHIFT);
}

static int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	if (bq->part_no == SC89890H)
		iprechg = (curr - SC89890H_IPRECHG_BASE) / SC89890H_IPRECHG_LSB;
	else
		iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
				   BQ2589X_IPRECHG_MASK,
				   iprechg << BQ2589X_IPRECHG_SHIFT);
}

static int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val = (volt - BQ2589X_VREG_BASE) / BQ2589X_VREG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_06,
				   BQ2589X_VREG_MASK,
				   val << BQ2589X_VREG_SHIFT);
}

static int bq2589x_get_chargevoltage(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_06, &val);
	if (ret < 0) {
		bq_err("get charge voltage failed: %d\n", ret);
		return ret;
	}

	return ((val & BQ2589X_VREG_MASK) >> BQ2589X_VREG_SHIFT) *
		BQ2589X_VREG_LSB + BQ2589X_VREG_BASE;
}

static int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_0D,
				   BQ2589X_VINDPM_MASK,
				   val << BQ2589X_VINDPM_SHIFT);
}

static int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{
	u8 val;

	if (curr < BQ2589X_IINLIM_BASE)
		curr = BQ2589X_IINLIM_BASE;
	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_00,
				   BQ2589X_IINLIM_MASK,
				   val << BQ2589X_IINLIM_SHIFT);
}

static int bq2589x_get_input_current_limit(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_00, &val);
	if (ret < 0) {
		bq_err("get input current limit failed: %d\n", ret);
		return ret;
	}

	return ((val & BQ2589X_IINLIM_MASK) >> BQ2589X_IINLIM_SHIFT) *
		BQ2589X_IINLIM_LSB + BQ2589X_IINLIM_BASE;
}

static int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;

	if (bq->part_no == SC89890H) {
		val = (offset < 500) ? SC89890H_VINDPMOS_400MV :
				       SC89890H_VINDPMOS_600MV;
		return bq2589x_update_bits(bq, BQ2589X_REG_01,
					   SC89890H_VINDPMOS_MASK,
					   val << SC89890H_VINDPMOS_SHIFT);
	}

	val = (offset - BQ2589X_VINDPMOS_BASE) / BQ2589X_VINDPMOS_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_01,
				   BQ2589X_VINDPMOS_MASK,
				   val << BQ2589X_VINDPMOS_SHIFT);
}

static u8 bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 reg;
	int ret;
	union power_supply_propval propval = {0, };

	if (!bq)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &reg);
	if (ret < 0) {
		bq_err("failed to read register 0x0b: %d\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	switch ((reg & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT) {
	case BQ2589X_CHRG_STAT_IDLE:
		srcu_notifier_call_chain(&bq->evt_notifier,
					 BQ_CHG_EVENT_CHARGE_NOT_DONE, bq);
		bq_log("not charging\n");
		return POWER_SUPPLY_STATUS_DISCHARGING;
	case BQ2589X_CHRG_STAT_PRECHG:
		srcu_notifier_call_chain(&bq->evt_notifier,
					 BQ_CHG_EVENT_CHARGE_NOT_DONE, bq);
		bq_log("pre charging\n");
		return POWER_SUPPLY_STATUS_CHARGING;
	case BQ2589X_CHRG_STAT_FASTCHG:
		srcu_notifier_call_chain(&bq->evt_notifier,
					 BQ_CHG_EVENT_CHARGE_NOT_DONE, bq);
		bq_log("fast charging\n");
		return POWER_SUPPLY_STATUS_CHARGING;
	case BQ2589X_CHRG_STAT_CHGDONE:
		bq_log("charge done!\n");

		if (!bq->bms_psy)
			bq->bms_psy = power_supply_get_by_name("bms");
		if (!bq->bms_psy) {
			bq_err("bms_psy not available\n");
			return POWER_SUPPLY_STATUS_CHARGING;
		}

		ret = power_supply_get_property(bq->bms_psy,
						POWER_SUPPLY_PROP_CAPACITY,
						&propval);
		if (ret < 0) {
			bq_err("get battery cap fail: %d\n", ret);
			return POWER_SUPPLY_STATUS_CHARGING;
		}

		bq_log("battery cap: %d\n", propval.intval);
		if (propval.intval > 95) {
			srcu_notifier_call_chain(&bq->evt_notifier,
						 BQ_CHG_EVENT_CHARGE_DONE, bq);
			return POWER_SUPPLY_STATUS_FULL;
		}

		return POWER_SUPPLY_STATUS_CHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

static int bq2589x_set_otg(struct bq2589x *bq, bool enable)
{
	int ret;

	if (enable) {
		ret = bq2589x_disable_charger(bq);
		if (ret < 0) {
			bq_err("failed to disable charger: %d\n", ret);
			return ret;
		}

		ret = bq2589x_enable_otg(bq);
		if (ret < 0) {
			bq_err("failed to enable otg: %d\n", ret);
			return ret;
		}
	} else {
		ret = bq2589x_disable_otg(bq);
		if (ret < 0) {
			bq_err("failed to disable otg: %d\n", ret);
			return ret;
		}

		ret = bq2589x_enable_charger(bq);
		if (ret < 0) {
			bq_err("failed to enable charger: %d\n", ret);
			return ret;
		}
	}
	return 0;
}

static int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}

static __maybe_unused int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_07,
				   BQ2589X_WDT_MASK,
				   (u8)((timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB) << BQ2589X_WDT_SHIFT);
}

static int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}

static int bq2589x_force_dpdm(struct bq2589x *bq)
{
	int ret;
	u8 data = 0;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	if (bq->part_no == SC89890H && bq->vbus_type == BQ2589X_VBUS_USB_HVDCP) {
		ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &data);
		if (ret < 0) {
			bq_err("failed to read REG_0B: %d\n", ret);
			return ret;
		}

		if ((data & 0xE0) == 0x80) {
			bq2589x_write_byte(bq, BQ2589X_REG_01, 0x45);
			msleep(30);
			bq2589x_write_byte(bq, BQ2589X_REG_01, 0x25);
			msleep(30);
		}
	}

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
}

static int bq2589x_is_dpdm_done(struct bq2589x *bq, int *done)
{
	int ret;
	u8 data = 0, force_dpdm_bit = 0;

	if (!done)
		return -EINVAL;

	if (bq->part_no == SC89890H) {
		ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &data);
		if (ret < 0)
			return ret;

		*done = (data & BQ2589X_PG_STAT_MASK) ? 1 : 0;
		bq_dbg("SC89890H DPDM: REG_0B=0x%02x, PG_STAT=%d\n", data, *done);
	} else {
		ret = bq2589x_read_byte(bq, BQ2589X_REG_02, &data);
		if (ret < 0)
			return ret;

		force_dpdm_bit = (data & BQ2589X_FORCE_DPDM_MASK) >> BQ2589X_FORCE_DPDM_SHIFT;
		*done = (force_dpdm_bit == 0) ? 1 : 0;
		bq_dbg("BQ2589X DPDM: REG_02=0x%02x, FORCE_DPDM=%d, done=%d\n",
		       data, force_dpdm_bit, *done);
	}

	return ret;
}

static int bq2589x_force_dpdm_done(struct bq2589x *bq)
{
	int ret;
	int done = 0;
	int retry = 100;

	clear_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status);
	bq_info("force DPDM start\n");

	ret = bq2589x_force_dpdm(bq);
	if (ret < 0) {
		bq_err("failed to trigger DPDM: %d\n", ret);
		return ret;
	}

	while (retry-- > 0) {
		ret = bq2589x_is_dpdm_done(bq, &done);
		if (ret < 0) {
			bq_err("read DPDM done status failed: %d\n", ret);
			return ret;
		}

		if (done) {
			bq_info("DPDM done\n");
			return 0;
		}

		msleep(30);
	}

	bq_err("DPDM timeout (after ~3 seconds)\n");
	return -ETIMEDOUT;
}

static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &val);
	if (ret < 0) {
		bq_err("failed to read 0B byte, ret: %d\n", ret);
		return BQ2589X_VBUS_UNKNOWN;
	}

	return (val & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
}

static enum bq2589x_vbus_type bq2589x_get_vbus_valid(struct bq2589x *bq)
{
	enum bq2589x_vbus_type vbus_type = BQ2589X_VBUS_UNKNOWN;
	int ret;
	int retry = 3;

	ret = bq2589x_force_dpdm_done(bq);
	if (ret < 0) {
		bq_err("DPDM handshake failed: %d\n", ret);
		return vbus_type;
	}

	do {
		vbus_type = bq2589x_get_vbus_type(bq);
		if (vbus_type != BQ2589X_VBUS_UNKNOWN)
			break;

		usleep_range(1000, 1500);
	} while (--retry > 0);

	bq_info("vbus_type: %d\n", vbus_type);
	return vbus_type;
}

static int bq2589x_reset_chip(struct bq2589x *bq)
{
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
}

static __maybe_unused int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
}

static int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
}

static int bq2589x_pumpx_increase_volt(struct bq2589x *bq)
{
	u8 val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);
}

static int bq2589x_pumpx_increase_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_09, &val);
	if (ret < 0)
		return ret;

	return (val & BQ2589X_PUMPX_UP_MASK) ? 1 : 0;
}

static int bq2589x_pumpx_decrease_volt(struct bq2589x *bq)
{
	u8 val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);
}

static int bq2589x_pumpx_decrease_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_09, &val);
	if (ret < 0)
		return ret;

	return (val & BQ2589X_PUMPX_DOWN_MASK) ? 1 : 0;
}

static int bq2589x_force_ico(struct bq2589x *bq)
{
	u8 val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK, val);
}

static int bq2589x_check_force_ico_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_14, &val);
	if (ret < 0)
		return ret;

	return (val & BQ2589X_ICO_OPTIMIZED_MASK) ? 1 : 0;
}

static int bq2589x_enable_term(struct bq2589x *bq, bool enable)
{
	u8 val = enable ? (BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT) :
			  (BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT);

	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);
}

static int bq2589x_enable_auto_dpdm(struct bq2589x *bq, bool enable)
{
	u8 val = enable ? (BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT) :
			  (BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT);

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);
}

static int bq2589x_use_absolute_vindpm(struct bq2589x *bq, bool enable)
{
	u8 val = enable ? (BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT) :
			  (BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT);

	return bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);
}

static int bq2589x_enable_ico(struct bq2589x *bq, bool enable)
{
	u8 val = enable ? (BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT) :
			  (BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT);

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);
}

static __maybe_unused void bq2589x_dump_regs(struct bq2589x *bq)
{
	int addr, ret;
	u8 val;

	bq_dbg("dump_regs:\n");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, addr, &val);
		if (!ret)
			bq_dbg("Reg[%02x] = 0x%02x\n",
			       (unsigned int)addr, (unsigned int)val);
	}
}

static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;

	if (bq->part_no == SC89890H)
		bq2589x_update_bits(bq, BQ2589X_REG_00,
				    BQ2589X_ENILIM_MASK,
				    BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);

	bq2589x_enable_ico(bq, false);
	bq2589x_disable_watchdog_timer(bq);
	bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm);
	bq2589x_enable_term(bq, bq->cfg.enable_term);
	if (!bq->cfg.enable_auto_dpdm)
		bq->cfg.use_absolute_vindpm = true;

	bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);

	ret = bq2589x_set_vindpm_offset(bq, 600);
	if (ret < 0) {
		bq_err("failed to set vindpm offset: %d\n", ret);
		return ret;
	}

	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		bq_err("failed to set termination current: %d\n", ret);
		return ret;
	}

	ret = bq2589x_set_prechg_current(bq, 200);
	if (ret < 0) {
		bq_err("failed to set prechg current: %d\n", ret);
		return ret;
	}

	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		bq_err("failed to set charge voltage: %d\n", ret);
		return ret;
	}

	ret = bq2589x_enable_charger(bq);
	if (ret < 0) {
		bq_err("failed to enable charger: %d\n", ret);
		return ret;
	}

	bq2589x_update_bits(bq, BQ2589X_REG_00,
			    BQ2589X_ENILIM_MASK,
			    BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
	bq2589x_update_bits(bq, BQ2589X_REG_02, 0x8, 1 << 3);

	if (bq->part_no == SYV690) {
		bq_info("init syv690 HV_TYPE 9/12V\n");
		bq2589x_update_bits(bq, BQ2589X_REG_02, 0x4, 0 << 2);
	}

	bq2589x_adc_stop(bq);
	return 0;
}

static int bq2589x_get_charge_type(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	if (!bq)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &val);
	if (ret < 0) {
		bq_err("read REG_0B failed: %d\n", ret);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	bq_dbg("REG_0B=0x%x\n", val);
	val = (val & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case BQ2589X_CHRG_STAT_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2589X_CHRG_STAT_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2589X_CHRG_STAT_CHGDONE:
	case BQ2589X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static enum power_supply_property bq2589x_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
};

static int bq2589x_wall_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct bq2589x *bq = power_supply_get_drvdata(psy);
	int online = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2589x_get_charging_status(bq);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		online = READ_ONCE(bq->chg_online);
		if (READ_ONCE(bq->vbat_volt) < 3300)
			online = 0;
		val->intval = online;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = READ_ONCE(bq->chg_type);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = READ_ONCE(bq->vbus_volt);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = READ_ONCE(bq->chg_current);
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = READ_ONCE(bq->enabled);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		val->intval = bq->cfg.term_current;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_get_charge_type(bq);
		bq_log("CHARGE_TYPE: %d\n", val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2589x_wall_set_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     const union power_supply_propval *val)
{
	int ret = 0;
	struct bq2589x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_REAL_TYPE:
		WRITE_ONCE(bq->chg_type, val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		WRITE_ONCE(bq->chg_online, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		WRITE_ONCE(bq->vbus_volt, val->intval);
		ret = bq2589x_set_chargevoltage(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		WRITE_ONCE(bq->chg_current, val->intval);
		if (val->intval > 0)
			vote(bq->fcc_votable, MAIN_SET_VOTER, true, val->intval);
		else
			vote(bq->fcc_votable, MAIN_SET_VOTER, false, 0);
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		WRITE_ONCE(bq->enabled, val->intval);
		if (val->intval > 0)
			ret = bq2589x_enable_charger(bq);
		else
			ret = bq2589x_disable_charger(bq);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		bq->cfg.term_current = val->intval;
		ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int bq2589x_wall_prop_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_REAL_TYPE:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
}

static int bq2589x_psy_register(struct bq2589x *bq)
{
	struct power_supply_config wall_cfg = {};

	bq->wall.name = "bbc";
	bq->wall.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
	bq->wall.properties = bq2589x_charger_props;
	bq->wall.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	bq->wall.get_property = bq2589x_wall_get_property;
	bq->wall.set_property = bq2589x_wall_set_property;
	bq->wall.property_is_writeable = bq2589x_wall_prop_is_writeable;

	wall_cfg.drv_data = bq;
	wall_cfg.of_node = bq->dev->of_node;

	bq->wall_psy = devm_power_supply_register(bq->dev, &bq->wall, &wall_cfg);
	if (IS_ERR(bq->wall_psy)) {
		bq_err("failed to register wall psy\n");
		return PTR_ERR(bq->wall_psy);
	}

	bq_info("%s power supply register successfully\n", bq->wall.name);
	return 0;
}

static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	u8 addr, val;
	int idx = 0, ret;

	if (!bq)
		return -ENODEV;

	idx += scnprintf(buf + idx, PAGE_SIZE - idx, "Charger 1:\n");
	for (addr = 0x00; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, addr, &val);
		if (ret) {
			dev_warn(bq->dev, "read reg 0x%02x failed: %d\n", addr, ret);
			continue;
		}

		idx += scnprintf(buf + idx, PAGE_SIZE - idx,
				 "Reg[0x%02x] = 0x%02x\n", addr, val);
		if (idx >= PAGE_SIZE)
			break;
	}
	return idx;
}

static const u8 bq2589x_write_whitelist[] = {
	BQ2589X_REG_00, BQ2589X_REG_01, BQ2589X_REG_02, BQ2589X_REG_03,
	BQ2589X_REG_04, BQ2589X_REG_05, BQ2589X_REG_06, BQ2589X_REG_07,
	BQ2589X_REG_08, BQ2589X_REG_09, BQ2589X_REG_0A, BQ2589X_REG_14
};

static bool bq2589x_is_reg_writable(u8 reg)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bq2589x_write_whitelist); i++) {
		if (bq2589x_write_whitelist[i] == reg)
			return true;
	}

	return false;
}

static ssize_t registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	char tmp[32];
	unsigned long reg_ul, val_ul;
	int ret;
	char *p;

	if (!bq)
		return -ENODEV;
	if (count == 0 || count >= sizeof(tmp))
		return -EINVAL;

	memcpy(tmp, buf, count);
	tmp[count] = '\0';

	ret = kstrtoul(tmp, 0, &reg_ul);
	if (ret)
		return -EINVAL;

	p = tmp;
	while (*p && !isspace(*p))
		p++;
	while (*p && isspace(*p))
		p++;
	if (!*p)
		return -EINVAL;

	ret = kstrtoul(p, 0, &val_ul);
	if (ret)
		return -EINVAL;

	if (reg_ul > 0x14 || val_ul > 0xff ||
	    !bq2589x_is_reg_writable((u8)reg_ul)) {
		dev_err(bq->dev, "invalid/readonly reg/val: reg=0x%lx val=0x%lx\n",
			reg_ul, val_ul);
		return -EINVAL;
	}

	ret = bq2589x_write_byte(bq, (u8)reg_ul, (u8)val_ul);
	if (ret) {
		dev_err(bq->dev, "write reg 0x%02lx failed: %d\n", reg_ul, ret);
		return ret;
	}

	dev_info(bq->dev, "wrote 0x%02lx to reg 0x%02lx\n", val_ul, reg_ul);
	return count;
}

static DEVICE_ATTR_RW(registers);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};

static int bq2589x_parse_dt(struct device *dev, struct bq2589x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	if (!np) {
		bq_err("no device tree node\n");
		return -ENODEV;
	}

	bq->irq_gpiod = devm_gpiod_get(dev, "intr", GPIOD_IN);
	if (IS_ERR(bq->irq_gpiod)) {
		ret = PTR_ERR(bq->irq_gpiod);
		bq_err("devm_gpiod_get(intr) failed: %d\n", ret);
		return ret;
	}
	bq_info("intr descriptor acquired\n");

	ret = 0;
	ret |= of_property_read_u32(np, "ti,bq2589x,vbus-volt-high-level",
				    &bq->pe.high_volt_level);

	ret |= of_property_read_u32(np, "ti,bq2589x,vbus-volt-low-level",
				    &bq->pe.low_volt_level);

	ret |= of_property_read_u32(np, "ti,bq2589x,vbat-min-volt-to-tuneup",
				    &bq->pe.vbat_min_volt);

	bq->cfg.enable_auto_dpdm = of_property_read_bool(np, "ti,bq2589x,enable-auto-dpdm");
	bq->cfg.enable_term = of_property_read_bool(np, "ti,bq2589x,enable-termination");
	bq->cfg.enable_ico = of_property_read_bool(np, "ti,bq2589x,enable-ico");
	bq->cfg.use_absolute_vindpm = of_property_read_bool(np, "ti,bq2589x,use-absolute-vindpm");

	ret |= of_property_read_u32(np, "ti,bq2589x,charge-voltage",
				    &bq->cfg.charge_voltage);

	ret |= of_property_read_u32(np, "ti,bq2589x,charge-current",
				    &bq->cfg.charge_current);

	ret |= of_property_read_u32(np, "ti,bq2589x,charge-current-3500",
				    &bq->cfg.charge_current_3500);

	ret |= of_property_read_u32(np, "ti,bq2589x,charge-current-1500",
				    &bq->cfg.charge_current_1500);

	ret |= of_property_read_u32(np, "ti,bq2589x,charge-current-1000",
				    &bq->cfg.charge_current_1000);

	ret |= of_property_read_u32(np, "ti,bq2589x,charge-current-500",
				    &bq->cfg.charge_current_500);

	ret |= of_property_read_u32(np, "ti,bq2589x,input-current-2000",
				    &bq->cfg.input_current_2000);

	ret |= of_property_read_u32(np, "ti,bq2589x,term-current",
				    &bq->cfg.term_current);

	if (ret) {
		bq_err("missing mandatory device tree properties\n");
		return -EINVAL;
	}

	return 0;
}

static void bq2589x_usb_switch(struct bq2589x *bq, bool en)
{
	struct gpio_desc *sw;

	if (!bq)
		return;

	mutex_lock(&bq->usb_switch_lock);
	if (bq->usb_switch_flag != en) {
		/* lookup and release every time rather than holding it. */
		sw = gpiod_get_optional(bq->dev, "usb-switch",
				       en ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
		if (IS_ERR(sw)) {
			bq_err("usb_switch gpiod_get_optional failed: %ld\n", PTR_ERR(sw));
			mutex_unlock(&bq->usb_switch_lock);
			return;
		}

		if (sw) {
			gpiod_put(sw);
			bq->usb_switch_flag = en;
			bq_info("usb_switch set to %d\n", en);
		}
	}
	mutex_unlock(&bq->usb_switch_lock);
}

static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_14, &data);
	if (!ret) {
		bq->part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	}

	return ret;
}

static int bq2589x_read_batt_rsoc(struct bq2589x *bq)
{
	union power_supply_propval ret = {0, };

	if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");
	if (bq->batt_psy) {
		power_supply_get_property(bq->batt_psy,
					  POWER_SUPPLY_PROP_CAPACITY,
					  &ret);
		return ret.intval;
	}

	return 50;
}

static void bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	int vbus_volt, vindpm_volt, ret;

	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if (vbus_volt < 6000)
		vindpm_volt = (bq->vbus_type == BQ2589X_VBUS_USB_DCP) ? 4500 : 4600;
	else
		vindpm_volt = 8300;

	WRITE_ONCE(bq->vbus_volt, vbus_volt);

	ret = bq2589x_set_input_volt_limit(bq, vindpm_volt);
	if (ret < 0)
		bq_err("set absolute vindpm threshold %d failed: %d\n", vindpm_volt, ret);
	else
		bq_info("set absolute vindpm threshold %d successfully\n", vindpm_volt);
}

static void bq2589x_adapter_in_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_in_work);
	int ret;
	union power_supply_propval propval = {0, };

	switch (bq->vbus_type) {
	case BQ2589X_VBUS_USB_HVDCP:
		bq_info("charger_type: HVDCP\n");
		bq2589x_enable_ico(bq, !bq->cfg.enable_ico);
		bq2589x_set_input_volt_limit(bq, 8300);
		vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 1800);
		bq2589x_usb_switch(bq, true);
		break;

	case BQ2589X_VBUS_USB_DCP:
		bq_info("charger_type: DCP, pd_active=%d\n", bq->pd_active);
		vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true,
		     bq->cfg.input_current_2000);
		bq->pe_check_pending = true;
		bq->pe_check_wait = 0;
		bq2589x_usb_switch(bq, true);
		break;

	case BQ2589X_VBUS_USB_CDP:
		bq_info("charger_type: CDP\n");
		vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 1500);
		msleep(1000);
		bq2589x_usb_switch(bq, false);
		break;

	case BQ2589X_VBUS_USB_SDP:
		bq_info("charger_type: SDP, pd_active=%d\n", bq->pd_active);
		if (!bq->usb_psy)
			bq->usb_psy = power_supply_get_by_name("usb");
		if (bq->usb_psy) {
			ret = power_supply_get_property(bq->usb_psy,
							POWER_SUPPLY_PROP_MTBF_CURRENT,
							&propval);
			if (ret < 0)
				bq_err("get mtbf current fail\n");
		}

		if (!bq->pd_active)
			vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true,
			     (propval.intval >= 1500) ? propval.intval : 500);
		else
			vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 1500);
		bq2589x_usb_switch(bq, false);
		break;

	case BQ2589X_VBUS_NONSTAND:
	case BQ2589X_VBUS_UNKNOWN:
		bq_info("charger_type: FLOAT, pd_active=%d\n", bq->pd_active);
		if (!bq->usb_psy)
			bq->usb_psy = power_supply_get_by_name("usb");
		if (bq->usb_psy) {
			ret = power_supply_get_property(bq->usb_psy,
							POWER_SUPPLY_PROP_MTBF_CURRENT,
							&propval);
			if (ret < 0)
				bq_err("get mtbf current fail\n");
		}

		if (!bq->pd_active)
			vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true,
			     (propval.intval >= 1500) ? propval.intval : 1000);
		else
			vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true,
			     bq->cfg.input_current_2000);
		bq2589x_usb_switch(bq, false);
		break;

	default:
		bq_info("charger_type: Other, vbus_type is %d\n", bq->vbus_type);
		bq2589x_usb_switch(bq, false);
		bq->ico_pending = true;
		bq->ico_wait = 0;
		break;
	}

	if (bq->vbus_type == BQ2589X_VBUS_USB_SDP && !bq->pd_active)
		vote(bq->fcc_votable, MAIN_SET_VOTER, true, MAIN_FCC_MIN);
	else
		vote(bq->fcc_votable, MAIN_SET_VOTER, false, 0);

	bq->monitor_wait = 0;
	queue_delayed_work(bq->bg_wq, &bq->poll_work, 0);
}

static void bq2589x_adapter_out_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_out_work);
	int ret;

	ret = bq2589x_set_input_volt_limit(bq, 4600);
	if (ret < 0)
		bq_err("reset vindpm threshold to 4600 failed: %d\n", ret);
	else
		bq_info("reset vindpm threshold to 4600 successfully\n");

	vote(bq->fcc_votable, MAIN_SET_VOTER, true, MAIN_FCC_MIN);
	vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, MAIN_ICL_MIN);
}

static void bq2589x_charger_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, charger_work.work);
	u8 type_now;

	if (!bq->batt_psy)
		return;

	type_now = bq2589x_get_charging_status(bq);
	if (type_now > 0)
		power_supply_changed(bq->batt_psy);

	bq_info("type_now: %d\n", type_now);
}

static void bq2589x_ico_tick(struct bq2589x *bq)
{
	int ret, idpm;
	u8 status;

	if (bq->part_no == SYV690) {
		bq_info("SYV690 IC detected, skip ico\n");
		bq->ico_pending = false;
		return;
	}

	if (!bq->ico_issued) {
		ret = bq2589x_force_ico(bq);
		if (ret < 0) {
			bq->ico_wait = 1;
		} else {
			bq->ico_issued = true;
			bq->ico_wait = 3;
		}
	} else {
		bq->ico_issued = false;
		bq->ico_pending = false;
		ret = bq2589x_check_force_ico_done(bq);
		if (ret == 1) {
			ret = bq2589x_read_byte(bq, BQ2589X_REG_13, &status);
			if (!ret) {
				idpm = ((status & BQ2589X_IDPM_LIM_MASK) >>
					BQ2589X_IDPM_LIM_SHIFT) *
					BQ2589X_IDPM_LIM_LSB + BQ2589X_IDPM_LIM_BASE;
				bq_info("ICL from ICO: %d mA\n", idpm);
			}
		}
	}
}

static void bq2589x_pe_check_tick(struct bq2589x *bq)
{
	if (!bq->pe.enable) {
		bq->pe_check_pending = false;
		bq->ico_pending = true;
		bq->ico_wait = 0;
		return;
	}

	WRITE_ONCE(bq->vbat_volt, bq2589x_adc_read_battery_volt(bq));
	WRITE_ONCE(bq->rsoc, bq2589x_read_batt_rsoc(bq));

	if (READ_ONCE(bq->vbat_volt) > bq->pe.vbat_min_volt && READ_ONCE(bq->rsoc) < 95) {
		bq->pe.target_volt = bq->pe.high_volt_level;
		bq->pe.tune_up_volt = true;
		bq->pe.tune_down_volt = false;
		bq->pe.tune_done = false;
		bq->pe.tune_count = 0;
		bq->pe.tune_fail = false;
		bq->pe_check_pending = false;
		bq->tune_pending = true;
		bq->pe_tune_wait = 0;
	} else if (READ_ONCE(bq->rsoc) >= 95) {
		bq->pe_check_pending = false;
		bq->ico_pending = true;
		bq->ico_wait = 0;
	} else {
		bq->pe_check_wait = 2;
	}
}

static void bq2589x_time_delay_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, time_delay_work.work);
	int ret;
	u8 status;

	if (bq->vbus_type != BQ2589X_VBUS_OTG) {
		bq2589x_usb_switch(bq, true);
		bq->vbus_type = bq2589x_get_vbus_valid(bq);
	}

	if (bq->vbus_type != BQ2589X_VBUS_NONE &&
	    bq->vbus_type != BQ2589X_VBUS_USB_HVDCP &&
	    bq->vbus_type != BQ2589X_VBUS_USB_DCP) {
		bq2589x_usb_switch(bq, false);
		return;
	}

	ret = bq2589x_read_byte(bq, BQ2589X_REG_13, &status);
	if (!ret && (status & BQ2589X_VDPM_STAT_MASK) && !bq->pd_active) {
		if (bq->vbus_type == BQ2589X_VBUS_USB_HVDCP &&
		    READ_ONCE(bq->vbus_volt) < 8000) {
			bq_info("HVDCP VINDPM occurred, vbus: %d, reset vindpm!\n",
				READ_ONCE(bq->vbus_volt));
			bq2589x_adjust_absolute_vindpm(bq);
		}
	}
}

static void bq2589x_usb_changed_tick(struct bq2589x *bq)
{
	union power_supply_propval val = {0, };
	int chg_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (bq->usb_changed_last_jiffies &&
	    time_after(jiffies, bq->usb_changed_last_jiffies + RESET_GAP)) {
		bq->usb_changed_no_change_count = 0;
		bq->usb_changed_last_valid = false;
		bq->usb_changed_last_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}
	bq->usb_changed_last_jiffies = jiffies;

	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy) {
			bq_err("fail to get usb_psy\n");
			bq->usb_changed_pending = false;
			return;
		}
	}

	if (!power_supply_get_property(bq->usb_psy,
				       POWER_SUPPLY_PROP_REAL_TYPE,
				       &val))
		chg_type = val.intval;

	if (!bq->usb_changed_last_valid ||
	    chg_type != bq->usb_changed_last_chg_type) {
		bq->usb_changed_last_chg_type = chg_type;
		bq->usb_changed_last_valid = true;
		bq->usb_changed_no_change_count = NO_CHANGE_MAX;
	}

	if (bq->usb_changed_no_change_count > 0) {
		if (bq->usb_psy)
			power_supply_changed(bq->usb_psy);
		if (bq->wall_psy)
			power_supply_changed(bq->wall_psy);

		bq->usb_changed_no_change_count--;
		if (bq->usb_changed_no_change_count > 0)
			bq->usb_changed_wait = 2;	/* POST_INTERVAL */
		else
			bq->usb_changed_last_valid = false;
	} else {
		bq->usb_changed_last_valid = false;
		bq->usb_changed_pending = false;
	}
}

static void bq2589x_tune_tick(struct bq2589x *bq)
{
	int ret = 0;

	WRITE_ONCE(bq->vbus_volt, bq2589x_adc_read_vbus_volt(bq));

	if ((bq->pe.tune_up_volt && READ_ONCE(bq->vbus_volt) > bq->pe.target_volt) ||
	    (bq->pe.tune_down_volt && READ_ONCE(bq->vbus_volt) < bq->pe.target_volt)) {
		bq->pe.tune_done = true;
		bq2589x_adjust_absolute_vindpm(bq);
		bq->tune_pending = false;
		if (bq->pe.tune_up_volt) {
			bq->ico_pending = true;
			bq->ico_wait = 0;
		}
		return;
	}

	if (bq->pe.tune_count > 10) {
		bq->pe.tune_fail = true;
		bq2589x_adjust_absolute_vindpm(bq);
		bq->tune_pending = false;
		if (bq->pe.tune_up_volt) {
			bq->ico_pending = true;
			bq->ico_wait = 0;
		}
		return;
	}

	if (!bq->pumpx_cmd_issued) {
		if (bq->pe.tune_up_volt)
			ret = bq2589x_pumpx_increase_volt(bq);
		else if (bq->pe.tune_down_volt)
			ret = bq2589x_pumpx_decrease_volt(bq);

		if (ret < 0) {
			bq->pe_tune_wait = 1;
		} else {
			bq->pumpx_cmd_issued = true;
			bq->pe.tune_count++;
			bq->pe_tune_wait = 3;
		}
	} else {
		if (bq->pe.tune_up_volt)
			ret = bq2589x_pumpx_increase_volt_done(bq);
		else if (bq->pe.tune_down_volt)
			ret = bq2589x_pumpx_decrease_volt_done(bq);

		if (ret == 0) {
			bq2589x_adjust_absolute_vindpm(bq);
			bq->pumpx_cmd_issued = false;
		}
		bq->pe_tune_wait = 1;
	}
}

static void bq2589x_monitor_tick(struct bq2589x *bq)
{
	u8 status = 0;
	int ret = 0, rawfcc = 0, rawfv = 0, rawicl = 0, batt_temp = 0;
	union power_supply_propval propval = {0, };

	bq2589x_reset_watchdog_timer(bq);
	WRITE_ONCE(bq->rsoc, bq2589x_read_batt_rsoc(bq));
	WRITE_ONCE(bq->vbus_volt, bq2589x_adc_read_vbus_volt(bq));
	WRITE_ONCE(bq->vbat_volt, bq2589x_adc_read_battery_volt(bq));
	WRITE_ONCE(bq->chg_current, bq2589x_adc_read_charge_current(bq));

	rawfcc = bq2589x_get_charge_current(bq);
	rawfv = bq2589x_get_chargevoltage(bq);
	rawicl = bq2589x_get_input_current_limit(bq);

	if (!bq->bms_psy)
		bq->bms_psy = power_supply_get_by_name("bms");
	if (bq->bms_psy) {
		ret = power_supply_get_property(bq->bms_psy,
						POWER_SUPPLY_PROP_TEMP,
						&propval);
		if (ret < 0) {
			bq_err("get battery temp fail\n");
			batt_temp = 250;
		} else {
			batt_temp = propval.intval;
		}
		bq_info("batt_temp: %d\n", (batt_temp / 10));
	}

	if (batt_temp < 0)
		bq2589x_update_bits(bq, BQ2589X_REG_06,
				    BQ2589X_VRECHG_MASK,
				    BQ2589X_VRECHG_200MV << BQ2589X_VRECHG_SHIFT);
	else
		bq2589x_update_bits(bq, BQ2589X_REG_06,
				    BQ2589X_VRECHG_MASK,
				    BQ2589X_VRECHG_100MV << BQ2589X_VRECHG_SHIFT);

	ret = bq2589x_read_byte(bq, BQ2589X_REG_13, &status);
	if (!ret && (status & BQ2589X_VDPM_STAT_MASK))
		bq_info("VINDPM occurred\n");
	if (!ret && (status & BQ2589X_IDPM_STAT_MASK))
		bq_info("IINDPM occurred\n");

	if (bq->vbus_type == BQ2589X_VBUS_USB_DCP &&
	    READ_ONCE(bq->vbus_volt) > bq->pe.high_volt_level &&
	    READ_ONCE(bq->rsoc) > 95 && !bq->pe.tune_down_volt) {
		bq->pe.tune_down_volt = true;
		bq->pe.tune_up_volt = false;
		bq->pe.target_volt = bq->pe.low_volt_level;
		bq->pe.tune_done = false;
		bq->pe.tune_count = 0;
		bq->pe.tune_fail = false;
		bq->tune_pending = true;
		bq->pe_tune_wait = 0;
	}

	switch (bq->vbus_type) {
	case BQ2589X_VBUS_USB_HVDCP:
		bq2589x_enable_ico(bq, false);
		/* fallthrough */
	case BQ2589X_VBUS_USB_DCP:
		if (rawicl != 2000 && rawicl != MAIN_ICL_MIN)
			rerun_election(bq->usb_icl_votable);
		/* fallthrough */
	case BQ2589X_VBUS_USB_SDP:
	case BQ2589X_VBUS_USB_CDP:
	case BQ2589X_VBUS_NONSTAND:
	case BQ2589X_VBUS_UNKNOWN:
		if (rawfcc > get_effective_result_locked(bq->fcc_votable))
			rerun_election(bq->fcc_votable);
		if (rawfv > get_effective_result_locked(bq->fv_votable))
			rerun_election(bq->fv_votable);
		break;
	default:
		bq_info("unhandled vbus_type: %d\n", bq->vbus_type);
		break;
	}
}

#define MONITOR_WAIT_TICKS	10
#define USB_CHANGED_WAIT_TICKS	2	/* POST_INTERVAL */
static void bq2589x_poll_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, poll_work.work);
	bool plugin = test_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status);

	if (plugin && bq->monitor_wait > 0)
		bq->monitor_wait--;
	if (bq->ico_pending && bq->ico_wait > 0)
		bq->ico_wait--;
	if (bq->pe_check_pending && bq->pe_check_wait > 0)
		bq->pe_check_wait--;
	if (bq->tune_pending && bq->pe_tune_wait > 0)
		bq->pe_tune_wait--;
	if (bq->usb_changed_pending && bq->usb_changed_wait > 0)
		bq->usb_changed_wait--;

	if (plugin && bq->monitor_wait == 0) {
		bq2589x_monitor_tick(bq);
		bq->monitor_wait = MONITOR_WAIT_TICKS;
	}
	if (bq->ico_pending && bq->ico_wait == 0)
		bq2589x_ico_tick(bq);
	if (bq->pe_check_pending && bq->pe_check_wait == 0)
		bq2589x_pe_check_tick(bq);
	if (bq->tune_pending && bq->pe_tune_wait == 0)
		bq2589x_tune_tick(bq);
	if (bq->usb_changed_pending && bq->usb_changed_wait == 0)
		bq2589x_usb_changed_tick(bq);

	if (plugin || bq->ico_pending || bq->pe_check_pending ||
	    bq->tune_pending || bq->usb_changed_pending)
		queue_delayed_work(bq->bg_wq, &bq->poll_work, HZ);
}

static void bq2589x_start_charging_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, start_charging_work);
	int last_status = POWER_SUPPLY_STATUS_UNKNOWN, status;
	int ret = 0;
	unsigned long timeout_jiffies;

	if (!bq->bms_psy)
		bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");
	if (!bq->bms_psy || !bq->batt_psy)
		return;

	ret = bq2589x_enable_charger(bq);
	if (ret < 0) {
		bq_err("failed to enable charger: %d\n", ret);
		return;
	}
	srcu_notifier_call_chain(&bq->evt_notifier, BQ_CHG_EVENT_STOP_MONITOR, bq);

	timeout_jiffies = jiffies + 2 * HZ;
	while (time_before(jiffies, timeout_jiffies)) {
		status = bq2589x_get_charging_status(bq);
		bq_info("waiting for charging: status=%d, last=%d\n", status, last_status);
		if (status != last_status) {
			last_status = status;
			power_supply_changed(bq->batt_psy);
		}
		if (status == POWER_SUPPLY_STATUS_CHARGING) {
			power_supply_changed(bq->bms_psy);
			break;
		}
		msleep(200);
	}
	srcu_notifier_call_chain(&bq->evt_notifier, BQ_CHG_EVENT_START_MONITOR, bq);
}

static int bq2589x_set_charger_type(struct bq2589x *bq, enum power_supply_type chg_type)
{
	union power_supply_propval propval = {0, };
	bool online = (chg_type != POWER_SUPPLY_TYPE_UNKNOWN);
	int ret;

	WRITE_ONCE(bq->chg_online, online);
	WRITE_ONCE(bq->chg_type, chg_type);

	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy) {
			bq_err("fail to get usb_psy\n");
			return -ENODEV;
		}
	}

	if (!(bq->pd_active && !online)) {
		propval.intval = online;
		ret = power_supply_set_property(bq->usb_psy,
						POWER_SUPPLY_PROP_ONLINE,
						&propval);
		if (ret < 0)
			bq_err("inform power supply usb_online fail, ret=%d\n", ret);
	}

	bq_info("chg_type = %d\n", chg_type);
	propval.intval = chg_type;

	ret = power_supply_set_property(bq->usb_psy,
					POWER_SUPPLY_PROP_REAL_TYPE,
					&propval);
	if (ret < 0)
		bq_err("set prop REAL_TYPE fail, ret=%d\n", ret);

	power_supply_changed(bq->usb_psy);
	return ret;
}

static enum power_supply_type bq2589x_get_charger_type(struct bq2589x *bq)
{
	switch (bq->vbus_type) {
	case BQ2589X_VBUS_NONE:
		bq_info("charger_type: NONE\n");
		return POWER_SUPPLY_TYPE_UNKNOWN;
	case BQ2589X_VBUS_USB_HVDCP:
		bq_info("charger_type: HVDCP/Maxcharge\n");
		if (bq->part_no == SC89890H)
			bq2589x_write_byte(bq, BQ2589X_REG_01, 0xC9);
		return POWER_SUPPLY_TYPE_USB_HVDCP;
	case BQ2589X_VBUS_USB_DCP:
		bq_info("charger_type: DCP\n");
		return POWER_SUPPLY_TYPE_USB_DCP;
	case BQ2589X_VBUS_USB_CDP:
		bq_info("charger_type: CDP\n");
		return POWER_SUPPLY_TYPE_USB_CDP;
	case BQ2589X_VBUS_USB_SDP:
		bq_info("charger_type: SDP\n");
		return POWER_SUPPLY_TYPE_USB;
	case BQ2589X_VBUS_NONSTAND:
		bq_info("charger_type: FLOAT\n");
	case BQ2589X_VBUS_UNKNOWN:
		bq_info("charger_type: UNKNOWN\n");
		return POWER_SUPPLY_TYPE_USB_FLOAT;
	case BQ2589X_VBUS_OTG:
		bq_info("charger_type: OTG\n");
		return POWER_SUPPLY_TYPE_UNKNOWN;
	default:
		bq_info("charger_type: Other, vbus_type is %d\n", bq->vbus_type);
		return POWER_SUPPLY_TYPE_USB_FLOAT;
	}
}

static void bq2589x_handle_event(struct bq2589x *bq)
{
	u8 status = 0, fault = 0, vbus_status = 0, pg_status = 0;
	int ret;
	enum power_supply_type chg_type;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &status);
	if (ret)
		return;

	bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
	pg_status = (status & BQ2589X_PG_STAT_MASK) >> BQ2589X_PG_STAT_SHIFT;
	if (!pg_status)
		bq->vbus_type = BQ2589X_VBUS_NONE;

	chg_type = bq2589x_get_charger_type(bq);
	bq2589x_set_charger_type(bq, chg_type);

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0C, &fault);
	if (ret)
		return;

	bq_info("status=%02x, vbus=%d, chg_type=%d, fault=0x%02x\n",
		status, bq->vbus_type, chg_type, fault);

	if (bq->part_no == SC89890H) {
		ret = bq2589x_read_byte(bq, BQ2589X_REG_11, &vbus_status);
		if (ret)
			return;

		if (!(vbus_status & BQ2589X_VBUS_GD_MASK) &&
		    test_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status)) {
			bq2589x_usb_switch(bq, true);
			bq2589x_adc_stop(bq);
			clear_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status);
			queue_work(bq->event_wq, &bq->adapter_out_work);
			bq_info("adapter removed\n");
			queue_delayed_work(bq->bg_wq, &bq->charger_work, 0);
		} else if (bq->vbus_type != BQ2589X_VBUS_NONE &&
			   bq->vbus_type != BQ2589X_VBUS_OTG &&
			   !test_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status)) {
			set_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status);
			bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);
			bq2589x_adc_start(bq, false);
			if (bq->cfg.use_absolute_vindpm)
				bq2589x_adjust_absolute_vindpm(bq);
			bq2589x_update_bits(bq, BQ2589X_REG_00,
					    BQ2589X_ENILIM_MASK,
					    BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
			bq->usb_changed_pending = true;
			bq->usb_changed_wait = 0;
			queue_work(bq->event_wq, &bq->adapter_in_work);
			bq_info("adapter plugged in\n");
			queue_delayed_work(bq->bg_wq, &bq->charger_work, 100);
			queue_work(bq->event_wq, &bq->start_charging_work);
		}
	} else {
		if ((bq->vbus_type == BQ2589X_VBUS_NONE ||
		     bq->vbus_type == BQ2589X_VBUS_OTG) &&
		    test_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status)) {
			bq2589x_usb_switch(bq, true);
			bq2589x_adc_stop(bq);
			clear_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status);
			queue_work(bq->event_wq, &bq->adapter_out_work);
			bq_info("adapter removed\n");
			queue_delayed_work(bq->bg_wq, &bq->charger_work, 0);
		} else if (bq->vbus_type != BQ2589X_VBUS_NONE &&
			   bq->vbus_type != BQ2589X_VBUS_OTG &&
			   !test_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status)) {
			set_bit(BQ2589X_STAT_PLUGIN_BIT, &bq->status);
			bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);
			bq2589x_adc_start(bq, false);
			if (bq->cfg.use_absolute_vindpm)
				bq2589x_adjust_absolute_vindpm(bq);
			bq2589x_update_bits(bq, BQ2589X_REG_00,
					    BQ2589X_ENILIM_MASK,
					    BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
			bq->usb_changed_pending = true;
			bq->usb_changed_wait = 0;
			queue_work(bq->event_wq, &bq->adapter_in_work);
			bq_info("adapter plugged in\n");
			queue_delayed_work(bq->bg_wq, &bq->charger_work, 100);
			queue_work(bq->event_wq, &bq->start_charging_work);
		}
	}

	if ((status & BQ2589X_PG_STAT_MASK) && !test_bit(BQ2589X_STAT_PG_BIT, &bq->status))
		set_bit(BQ2589X_STAT_PG_BIT, &bq->status);
	else if (!(status & BQ2589X_PG_STAT_MASK) &&
		 test_bit(BQ2589X_STAT_PG_BIT, &bq->status))
		clear_bit(BQ2589X_STAT_PG_BIT, &bq->status);

	if (fault && !test_bit(BQ2589X_STAT_FAULT_BIT, &bq->status))
		set_bit(BQ2589X_STAT_FAULT_BIT, &bq->status);
	else if (!fault && test_bit(BQ2589X_STAT_FAULT_BIT, &bq->status))
		clear_bit(BQ2589X_STAT_FAULT_BIT, &bq->status);

	if (fault & BQ2589X_FAULT_BOOST_MASK) {
		bq2589x_usb_switch(bq, true);
		if (bq2589x_set_otg(bq, false))
			bq_err("failed to disable otg on fault\n");
	}

	if (fault & BQ2589X_FAULT_WDT_MASK) {
		bq2589x_reset_chip(bq);
		usleep_range(4500, 5500);
		bq2589x_init_device(bq);
	}
}

static irqreturn_t bq2589x_thread_irq(int irq, void *data)
{
	struct bq2589x *bq = data;

	if (!bq)
		return IRQ_HANDLED;

	pm_wakeup_ws_event(bq->bq_ws, 500, false);

	bq2589x_handle_event(bq);

	return IRQ_HANDLED;
}

#if defined(CONFIG_TCPC_RT1711H)
static int bq2589x_set_fast_charge_mode(struct bq2589x *bq, int pd_active)
{
	union power_supply_propval propval = {0, };
	int batt_verify = 0, batt_soc = 0, batt_temp = 0, ret = 0;

	if (!bq->bms_psy)
		bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->bms_psy) {
		bq_err("bms_psy not found\n");
		return -ENOENT;
	}

	ret = power_supply_get_property(bq->bms_psy,
					POWER_SUPPLY_PROP_CHIP_OK,
					&propval);
	if (ret < 0)
		bq_err("get battery chip ok fail\n");
	else
		batt_verify = propval.intval;

	ret = power_supply_get_property(bq->bms_psy,
					POWER_SUPPLY_PROP_CAPACITY,
					&propval);
	if (ret < 0)
		bq_err("get battery capacity fail\n");
	else
		batt_soc = propval.intval;

	ret = power_supply_get_property(bq->bms_psy,
					POWER_SUPPLY_PROP_TEMP,
					&propval);
	if (ret < 0)
		bq_err("get battery temp fail\n");
	else
		batt_temp = propval.intval;

	if (pd_active == 2 && batt_verify && batt_soc < 95) {
		nopmi_set_ffc_disabled(false);
		propval.intval = (batt_temp >= 150 && batt_temp <= 480) ? 1 : 0;
	} else {
		propval.intval = 0;
		nopmi_set_ffc_disabled(true);
	}

	ret = power_supply_set_property(bq->bms_psy,
					POWER_SUPPLY_PROP_FASTCHARGE_MODE,
					&propval);
	if (ret < 0)
		bq_err("set fastcharge mode fail!\n");

	return ret;
}

static void set_pd_active(struct bq2589x *bq, int pd_active)
{
	int ret = 0;
	union power_supply_propval val = {0, };

	if (!bq->usb_psy)
		bq->usb_psy = power_supply_get_by_name("usb");
	if (!bq->usb_psy) {
		bq_err("fail to get usb_psy\n");
		return;
	}

	if (pd_active)
		bq2589x_set_charger_type(bq, POWER_SUPPLY_TYPE_USB_PD);
	else
		bq2589x_set_charger_type(bq, POWER_SUPPLY_TYPE_UNKNOWN);

	bq->pd_active = pd_active;
	val.intval = pd_active;
	ret = power_supply_set_property(bq->usb_psy,
					POWER_SUPPLY_PROP_PD_ACTIVE,
					&val);
	if (ret < 0)
		bq_err("Couldn't set USB PD_ACTIVE status, ret=%d\n", ret);

	bq2589x_set_fast_charge_mode(bq, pd_active);
}

static int get_source_mode(struct tcp_notify *noti)
{
	if (noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC)
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;

	switch (noti->typec_state.rp_level) {
	case TYPEC_CC_VOLT_SNK_1_5:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case TYPEC_CC_VOLT_SNK_3_0:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	case TYPEC_CC_VOLT_SNK_DFT:
	default:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	}
}

static int bq2589x_set_cc_orientation(struct bq2589x *bq, int cc_orientation)
{
	int ret = 0;
	union power_supply_propval propval = {0, };

	if (!bq->usb_psy)
		bq->usb_psy = power_supply_get_by_name("usb");
	if (!bq->usb_psy) {
		bq_err("fail to get usb_psy\n");
		return -ENODEV;
	}

	propval.intval = cc_orientation;
	ret = power_supply_set_property(bq->usb_psy,
					POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
					&propval);
	if (ret < 0)
		bq_err("set prop CC_ORIENTATION fail: (%d)\n", ret);

	return ret;
}

static int bq2589x_set_typec_mode(struct bq2589x *bq, enum power_supply_typec_mode typec_mode)
{
	int ret = 0;
	union power_supply_propval propval = {0, };

	if (!bq->usb_psy)
		bq->usb_psy = power_supply_get_by_name("usb");
	if (!bq->usb_psy) {
		bq_err("fail to get usb_psy\n");
		return -ENODEV;
	}

	propval.intval = typec_mode;
	ret = power_supply_set_property(bq->usb_psy,
					POWER_SUPPLY_PROP_TYPEC_MODE,
					&propval);
	if (ret < 0)
		bq_err("set prop TYPEC_MODE fail: (%d)\n", ret);

	return ret;
}

static int pd_tcp_notifier_call(struct notifier_block *pnb, unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct bq2589x *bq = container_of(pnb, struct bq2589x, pd_nb);
	enum power_supply_typec_mode typec_mode = POWER_SUPPLY_TYPEC_NONE;
	bool typec_mode_valid = false;
	int ret, cc_orientation = 0;
	u8 status;

	switch (event) {
	case TCP_NOTIFY_SINK_VBUS:
		bq_log("TCP_NOTIFY_SINK_VBUS\n");
		break;
	case TCP_NOTIFY_PD_STATE:
		bq_info("noti->pd_state connected: %d\n", noti->pd_state.connected);
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			bq_info("disconnected\n");
			break;
		case PD_CONNECT_HARD_RESET:
			bq_info("hardreset\n");
			if (bq->pd_active)
				set_pd_active(bq, 0);
			break;
		case PD_CONNECT_PE_READY_SNK:
			bq_info("PD2.0 connect\n");
			set_pd_active(bq, 1);
			break;
		case PD_CONNECT_PE_READY_SNK_PD30:
			bq_info("PD3.0 connect\n");
			set_pd_active(bq, 1);
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			bq_info("PPS connect\n");
			set_pd_active(bq, 2);
			break;
		case PD_CONNECT_TYPEC_ONLY_SNK_DFT:
		case PD_CONNECT_TYPEC_ONLY_SNK:
			ret = bq2589x_read_byte(bq, BQ2589X_REG_13, &status);
			if (!ret && (status & BQ2589X_VDPM_STAT_MASK) && !bq->pd_active) {
				if (bq->vbus_type == BQ2589X_VBUS_USB_HVDCP &&
				    READ_ONCE(bq->vbus_volt) < 8000) {
					bq_info("HVDCP VINDPM occurred, vbus: %d, reset vindpm!\n",
						READ_ONCE(bq->vbus_volt));
					bq2589x_usb_switch(bq, true);
					queue_delayed_work(bq->bg_wq, &bq->time_delay_work,
							   3 * HZ);
				}
			}
			break;
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		cc_orientation = noti->typec_state.polarity ? 2 : 1;
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
		    (noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
		     noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		     noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC ||
		     noti->typec_state.new_state == TYPEC_ATTACHED_DBGACC_SNK)) {
			bq_info("USB Plug in, pol = %d, state = %d\n",
				cc_orientation, noti->typec_state.new_state);
			if (!bq->otg_attached &&
			    (bq->vbus_type == BQ2589X_VBUS_NONE ||
			     bq->vbus_type == BQ2589X_VBUS_UNKNOWN)) {
				bq2589x_init_device(bq);
				bq2589x_usb_switch(bq, true);
				bq->vbus_type = bq2589x_get_vbus_valid(bq);
			}
			typec_mode = get_source_mode(noti);
			typec_mode_valid = true;
			bq2589x_set_cc_orientation(bq, cc_orientation);
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			   (noti->typec_state.new_state == TYPEC_ATTACHED_SRC ||
			    noti->typec_state.new_state == TYPEC_ATTACHED_DEBUG)) {
			bq2589x_usb_switch(bq, false);
			bq->otg_attached = true;
			typec_mode = (noti->typec_state.new_state == TYPEC_ATTACHED_DEBUG) ?
				     POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY :
				     POWER_SUPPLY_TYPEC_SINK;
			typec_mode_valid = true;
			bq2589x_set_cc_orientation(bq, cc_orientation);
			bq_info("OTG Type-C Plug in, pol = %d\n", cc_orientation);
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			   noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			bq_info("Audio Accessory plug in\n");
			typec_mode = POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
			typec_mode_valid = true;
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO &&
			   noti->typec_state.new_state == TYPEC_UNATTACHED) {
			bq_info("Audio Accessory plug out\n");
			typec_mode = POWER_SUPPLY_TYPEC_NONE;
			typec_mode_valid = true;
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SRC ||
			    noti->typec_state.old_state == TYPEC_ATTACHED_DEBUG) &&
			   noti->typec_state.new_state == TYPEC_UNATTACHED) {
			typec_mode = POWER_SUPPLY_TYPEC_NONE;
			typec_mode_valid = true;
			bq->otg_attached = false;
			bq2589x_set_cc_orientation(bq, 0);
			bq_info("OTG Type-C Plug out\n");
			bq2589x_usb_switch(bq, true);
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
			    noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
			    noti->typec_state.old_state == TYPEC_ATTACHED_NORP_SRC ||
			    noti->typec_state.old_state == TYPEC_ATTACHED_DBGACC_SNK) &&
			   noti->typec_state.new_state == TYPEC_UNATTACHED) {
			bq_info("USB Plug out\n");
			typec_mode = POWER_SUPPLY_TYPEC_NONE;
			typec_mode_valid = true;
			bq2589x_set_cc_orientation(bq, 0);
			bq2589x_usb_switch(bq, false);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SRC &&
			   noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			bq->otg_attached = false;
			typec_mode = POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
			typec_mode_valid = true;
			bq_info("Source_to_Sink, pol = %d\n", cc_orientation);
			bq2589x_set_cc_orientation(bq, cc_orientation);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SNK &&
			   noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			bq->otg_attached = true;
			typec_mode = POWER_SUPPLY_TYPEC_SINK;
			typec_mode_valid = true;
			bq_info("Sink_to_Source, pol = %d\n", cc_orientation);
			bq2589x_set_cc_orientation(bq, cc_orientation);
		}
		if (typec_mode_valid)
			bq2589x_set_typec_mode(bq, typec_mode);
		break;
	case TCP_NOTIFY_EXT_DISCHARGE:
		bq_info("Ext discharge = %d\n", noti->en_state.en);
		if (noti->en_state.en) {
			bq2589x_usb_switch(bq, false);
		} else {
			bq2589x_usb_switch(bq, true);
			set_pd_active(bq, 0);
		}
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		bq_log("TCP_NOTIFY_SOURCE_VBUS\n");
		if (noti->vbus_state.mv == TCPC_VBUS_SOURCE_0V && bq->vbus_on) {
			bq_info("OTG VBUS Disabled\n");
			bq->vbus_on = false;
			if (bq2589x_set_otg(bq, false))
				bq_err("failed to disable otg on tcpc noti\n");
			bq2589x_usb_switch(bq, true);
		} else if (noti->vbus_state.mv == TCPC_VBUS_SOURCE_5V && !bq->vbus_on) {
			bq_info("OTG VBUS Enabled\n");
			bq->vbus_on = true;
			bq2589x_usb_switch(bq, false);
			if (bq2589x_set_otg(bq, true))
				bq_err("failed to enable otg on tcpc noti\n");
			bq2589x_set_otg_current(bq, bq->cfg.charge_current_1500);
		}
		break;
	}
	return NOTIFY_OK;
}

static int bq2589x_late_init_usb_switch(struct notifier_block *nb,
					unsigned long action, void *data)
{
	struct bq2589x *bq = container_of(nb, struct bq2589x, late_sync_nb);

	bq2589x_usb_switch(bq, true);
	bq->vbus_type = bq2589x_get_vbus_valid(bq);
	bq_info("USB Switch late init success\n");

	return NOTIFY_OK;
}
#endif

static int fcc_vote_callback(struct votable *votable, void *data,
			     int fcc_ma, const char *client)
{
	struct bq2589x *bq = data;
	int ret;

	bq_info("fcc: %d\n", fcc_ma);
	if (fcc_ma < 0)
		return 0;
	if (fcc_ma > BQ2589X_MAX_FCC)
		fcc_ma = BQ2589X_MAX_FCC;

	ret = bq2589x_set_charge_current(bq, fcc_ma);
	if (ret < 0)
		bq_err("failed to set charger current: (%d)\n", ret);

	return ret;
}

static int fv_vote_callback(struct votable *votable, void *data,
			    int fv_mv, const char *client)
{
	struct bq2589x *bq = data;
	int ret;

	bq_info("fv: %d\n", fv_mv);
	if (fv_mv < 0)
		return 0;

	ret = bq2589x_set_chargevoltage(bq, fv_mv);
	if (ret < 0)
		bq_err("failed to set charger voltage: (%d)\n", ret);

	return ret;
}

static int usb_icl_vote_callback(struct votable *votable, void *data,
				 int icl_ma, const char *client)
{
	struct bq2589x *bq = data;
	int ret;

	bq_info("icl: %d\n", icl_ma);
	if (icl_ma < 0)
		return 0;
	if (icl_ma > BQ2589X_MAX_ICL)
		icl_ma = BQ2589X_MAX_ICL;

	ret = bq2589x_set_input_current_limit(bq, icl_ma);
	if (ret < 0)
		bq_err("failed to set input current limit: (%d)\n", ret);

	return ret;
}

static int chg_dis_vote_callback(struct votable *votable, void *data,
				 int disable, const char *client)
{
	struct bq2589x *bq = data;
	int ret;

	bq_info("disable: %d\n", disable);
	ret = disable ? bq2589x_disable_charger(bq) : bq2589x_enable_charger(bq);
	if (ret < 0)
		bq_err("failed to %s charger: %d\n", disable ? "disable" : "enable", ret);

	return ret;
}

static int bq2589x_parse_irq(struct bq2589x *bq)
{
	int irqn, ret;

	irqn = gpiod_to_irq(bq->irq_gpiod);
	if (irqn <= 0) {
		bq_err("gpiod_to_irq failed: %d\n", irqn);
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(bq->dev, irqn,
					NULL, bq2589x_thread_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"bq2589x_charger_irq", bq);
	if (ret) {
		bq_err("request_threaded_irq failed for IRQ %d: %d\n", irqn, ret);
		return ret;
	}

	bq->irq = irqn;
	return 0;
}

static int bq2589x_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bq2589x *bq = NULL;
#if defined(CONFIG_TCPC_RT1711H)
	struct tcpc_device *tcpc_dev = NULL;
#endif
	int i, ret;
	static int probe_cnt;

	bq_info("enter, probe_cnt: %d\n", ++probe_cnt);
	if (probe_cnt >= PROBE_CNT_MAX) {
		bq_err("probe count exceeded: %d >= %d\n",
		       probe_cnt, PROBE_CNT_MAX);
		return -ENODEV;
	}

#if defined(CONFIG_TCPC_RT1711H)
	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc_dev) {
		bq_err("tcpc device not ready, defer\n");
		return -EPROBE_DEFER;
	}
#endif

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		ret = -ENOMEM;
		goto err_put_tcpc;
	}

	bq->name = BQ2589X_DEV_NAME;
	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	bq->regmap = devm_regmap_init_i2c(bq->client, &bq2589x_regmap_config);
	if (IS_ERR(bq->regmap)) {
		ret = PTR_ERR(bq->regmap);
		bq_err("failed to init regmap: %d\n", ret);
		goto err_free;
	}

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25890) {
		set_bit(BQ2589X_STAT_EXIST_BIT, &bq->status);
		bq_info("charger device bq25890 detected, revision: (%d)\n", bq->revision);
	} else if (!ret && bq->part_no == SYV690) {
		set_bit(BQ2589X_STAT_EXIST_BIT, &bq->status);
		bq_info("charger device SYV690 detected, revision: (%d)\n", bq->revision);
		nopmi_set_charger_ic_type(NOPMI_CHARGER_IC_SYV);
	} else if (!ret && bq->part_no == SC89890H) {
		set_bit(BQ2589X_STAT_EXIST_BIT, &bq->status);
		bq_info("charger device SC89890H detected, revision: (%d)\n", bq->revision);
	} else {
		bq_err("no bq25890 charger device found: (%d)\n", ret);
		ret = -ENODEV;
		goto err_free;
	}
	mutex_init(&bq->usb_switch_lock);

#if defined(CONFIG_TCPC_RT1711H)
	bq->tcpc_dev = tcpc_dev;
#endif
	bq->usb_psy = power_supply_get_by_name("usb");
	bq->batt_psy = power_supply_get_by_name("battery");
	bq->bms_psy = power_supply_get_by_name("bms");
	bq->class_dev.class = bq2589x_class;
	bq->class_dev.release = bq2589x_class_dev_release;
	dev_set_name(&bq->class_dev, "bq2589x");
	dev_set_drvdata(&bq->class_dev, bq);
	ret = device_register(&bq->class_dev);
	if (ret < 0) {
		bq_err("class_dev register failed: %d\n", ret);
		goto err_free_psy;
	}

	srcu_init_notifier_head(&bq->evt_notifier);
	bq->usb_changed_last_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	bq->usb_changed_no_change_count = 0;
	bq->usb_changed_last_valid = false;
	bq->usb_changed_last_jiffies = 0;
	bq->ico_issued = false;
	bq->pumpx_cmd_issued = false;
	for (i = 0; i < BQ_RL_SLOTS; i++) {
		ratelimit_state_init(&bq->rl_slots[i], BQ_RL_INTERVAL, BQ_RL_BURST);
		bq->rl_slots[i].flags |= RATELIMIT_MSG_ON_RELEASE;
	}

	ret = bq2589x_parse_dt(bq->dev, bq);
	if (ret) {
		bq_err("DT parse failed: %d\n", ret);
		goto err_free_dev;
	}

	ret = bq2589x_init_device(bq);
	if (ret) {
		bq_err("device init failure: (%d)\n", ret);
		goto err_free_dev;
	}

	device_set_wakeup_capable(bq->dev, true);
	ret = device_set_wakeup_enable(bq->dev, true);
	if (ret) {
		bq_err("enable wakeup failed: %d\n", ret);
		device_set_wakeup_capable(bq->dev, false);
		goto err_free_dev;
	}

	bq->bq_ws = wakeup_source_register(bq->dev, "bq2589x_ws");
	if (IS_ERR_OR_NULL(bq->bq_ws)) {
		ret = IS_ERR(bq->bq_ws) ? PTR_ERR(bq->bq_ws) : -ENOMEM;
		bq_err("wakeup_source_register failed: %d\n", ret);
		goto err_wake;
	}

	INIT_WORK(&bq->adapter_in_work, bq2589x_adapter_in_workfunc);
	INIT_WORK(&bq->adapter_out_work, bq2589x_adapter_out_workfunc);
	INIT_WORK(&bq->start_charging_work, bq2589x_start_charging_workfunc);
	INIT_DELAYED_WORK(&bq->poll_work, bq2589x_poll_workfunc);
	INIT_DELAYED_WORK(&bq->charger_work, bq2589x_charger_workfunc);
	INIT_DELAYED_WORK(&bq->time_delay_work, bq2589x_time_delay_workfunc);

	bq->event_wq = alloc_ordered_workqueue("bq2589x_event_wq", WQ_MEM_RECLAIM);
	if (!bq->event_wq) {
		ret = -ENOMEM;
		bq_err("failed to alloc event_wq\n");
		goto err_unreg_ws;
	}

	bq->bg_wq = alloc_ordered_workqueue("bq2589x_bg_wq", WQ_MEM_RECLAIM);
	if (!bq->bg_wq) {
		ret = -ENOMEM;
		bq_err("failed to alloc bg_wq\n");
		goto err_destroy_event_wq;
	}

	bq->fcc_votable = create_votable("FCC", VOTE_MIN, fcc_vote_callback, bq);
	if (IS_ERR(bq->fcc_votable)) {
		ret = PTR_ERR(bq->fcc_votable);
		goto err_destroy_bg_wq;
	}

	bq->chg_dis_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					     chg_dis_vote_callback, bq);
	if (IS_ERR(bq->chg_dis_votable)) {
		ret = PTR_ERR(bq->chg_dis_votable);
		goto err_destroy_fcc;
	}

	bq->fv_votable = create_votable("FV", VOTE_MIN, fv_vote_callback, bq);
	if (IS_ERR(bq->fv_votable)) {
		ret = PTR_ERR(bq->fv_votable);
		goto err_destroy_chg_dis;
	}

	bq->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					     usb_icl_vote_callback, bq);
	if (IS_ERR(bq->usb_icl_votable)) {
		ret = PTR_ERR(bq->usb_icl_votable);
		goto err_destroy_fv;
	}

	vote(bq->fcc_votable, PROFILE_CHG_VOTER, true, CHG_FCC_CURR_MAX);
	vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, CHG_ICL_CURR_MAX);
	vote(bq->fcc_votable, MAIN_SET_VOTER, true, MAIN_FCC_MIN);
	vote(bq->fv_votable, JEITA_VOTER, true, CHG_FV_CURR_MAX);
	vote(bq->chg_dis_votable, "BMS_FC_VOTER", false, 0);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		bq_err("failed to register sysfs: (%d)\n", ret);
		goto err_destroy_usb_icl;
	}

	ret = bq2589x_psy_register(bq);
	if (ret) {
		bq_err("psy_register failed\n");
		goto err_remove_sysfs;
	}

	bq->pe.enable = false;
	ret = bq2589x_parse_irq(bq);
	if (ret)
		goto err_remove_sysfs;

#if defined(CONFIG_TCPC_RT1711H)
	bq->pd_nb.notifier_call = pd_tcp_notifier_call;
	bq->pd_nb.priority = TCP_NOTIFY_PRIO_CHARGER;
	ret = register_tcp_dev_notifier(bq->tcpc_dev, &bq->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		bq_err("register tcpc notifier fail: %d\n", ret);
		goto err_remove_sysfs;
	}

	bq->late_sync_nb.notifier_call = bq2589x_late_init_usb_switch;
	ret = register_tcpc_late_sync_notifier(bq->tcpc_dev, &bq->late_sync_nb);
	if (ret < 0) {
		bq_err("register late_sync tcpc notifier fail: %d\n", ret);
		goto err_tcpc_notifier;
	}
#else
	bq2589x_usb_switch(bq, true);
	queue_delayed_work(bq->bg_wq, &bq->time_delay_work, 4 * HZ);
#endif

	bq_info("success\n");

	return 0;

#if defined(CONFIG_TCPC_RT1711H)
err_tcpc_notifier:
	if (bq->tcpc_dev)
		unregister_tcp_dev_notifier(bq->tcpc_dev, &bq->pd_nb, TCP_NOTIFY_TYPE_ALL);
#endif
err_remove_sysfs:
	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
err_destroy_usb_icl:
	if (!IS_ERR_OR_NULL(bq->usb_icl_votable))
		destroy_votable(bq->usb_icl_votable);
err_destroy_fv:
	if (!IS_ERR_OR_NULL(bq->fv_votable))
		destroy_votable(bq->fv_votable);
err_destroy_chg_dis:
	if (!IS_ERR_OR_NULL(bq->chg_dis_votable))
		destroy_votable(bq->chg_dis_votable);
err_destroy_fcc:
	if (!IS_ERR_OR_NULL(bq->fcc_votable))
		destroy_votable(bq->fcc_votable);
err_destroy_bg_wq:
	if (bq->bg_wq)
		destroy_workqueue(bq->bg_wq);
err_destroy_event_wq:
	if (bq->event_wq)
		destroy_workqueue(bq->event_wq);
err_unreg_ws:
	if (!IS_ERR_OR_NULL(bq->bq_ws))
		wakeup_source_unregister(bq->bq_ws);
err_wake:
	device_set_wakeup_enable(bq->dev, false);
	device_set_wakeup_capable(bq->dev, false);
err_free_dev:
	srcu_cleanup_notifier_head(&bq->evt_notifier);
	device_unregister(&bq->class_dev);
err_free_psy:
	if (bq->bms_psy) {
		power_supply_put(bq->bms_psy);
		bq->bms_psy = NULL;
	}
	if (bq->batt_psy) {
		power_supply_put(bq->batt_psy);
		bq->batt_psy = NULL;
	}
	if (bq->usb_psy) {
		power_supply_put(bq->usb_psy);
		bq->usb_psy = NULL;
	}
	mutex_destroy(&bq->usb_switch_lock);
err_free:
	i2c_set_clientdata(client, NULL);
err_put_tcpc:
#if defined(CONFIG_TCPC_RT1711H)
	tcpc_dev_put(tcpc_dev);
#endif

	bq_err("fail! (%d)\n", ret);
	return ret;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return 0;

	bq_info("entry\n");

	(void)bq2589x_set_otg(bq, false);
	bq2589x_exit_hiz_mode(bq);
	bq2589x_adc_stop(bq);
	usleep_range(4500, 5500);

#if defined(CONFIG_TCPC_RT1711H)
	if (bq->tcpc_dev) {
		unregister_tcpc_late_sync_notifier(bq->tcpc_dev, &bq->late_sync_nb);
		unregister_tcp_dev_notifier(bq->tcpc_dev, &bq->pd_nb, TCP_NOTIFY_TYPE_ALL);
	}
#endif

	if (bq->irq) {
		disable_irq(bq->irq);
		synchronize_irq(bq->irq);
	}

	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_work_sync(&bq->start_charging_work);
	cancel_delayed_work_sync(&bq->poll_work);
	cancel_delayed_work_sync(&bq->charger_work);
	cancel_delayed_work_sync(&bq->time_delay_work);

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);

	if (!IS_ERR_OR_NULL(bq->usb_icl_votable))
		destroy_votable(bq->usb_icl_votable);
	if (!IS_ERR_OR_NULL(bq->fv_votable))
		destroy_votable(bq->fv_votable);
	if (!IS_ERR_OR_NULL(bq->chg_dis_votable))
		destroy_votable(bq->chg_dis_votable);
	if (!IS_ERR_OR_NULL(bq->fcc_votable))
		destroy_votable(bq->fcc_votable);

	if (bq->bg_wq)
		destroy_workqueue(bq->bg_wq);

	if (bq->event_wq)
		destroy_workqueue(bq->event_wq);

	if (!IS_ERR_OR_NULL(bq->bq_ws))
		wakeup_source_unregister(bq->bq_ws);

	device_set_wakeup_enable(bq->dev, false);
	srcu_cleanup_notifier_head(&bq->evt_notifier);
	device_unregister(&bq->class_dev);

	if (bq->bms_psy)
		power_supply_put(bq->bms_psy);
	if (bq->batt_psy)
		power_supply_put(bq->batt_psy);
	if (bq->usb_psy)
		power_supply_put(bq->usb_psy);
#if defined(CONFIG_TCPC_RT1711H)
	if (bq->tcpc_dev)
		tcpc_dev_put(bq->tcpc_dev);
#endif

	mutex_destroy(&bq->usb_switch_lock);
	i2c_set_clientdata(client, NULL);

	bq_info("success\n");
	return 0;
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return;

	bq_info("entry\n");

	(void)bq2589x_set_otg(bq, false);
	bq2589x_exit_hiz_mode(bq);
	bq2589x_adc_stop(bq);
	usleep_range(4500, 5500);

	if (bq->irq) {
		disable_irq(bq->irq);
		synchronize_irq(bq->irq);
	}

	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_work_sync(&bq->start_charging_work);
	cancel_delayed_work_sync(&bq->poll_work);
	cancel_delayed_work_sync(&bq->charger_work);
	cancel_delayed_work_sync(&bq->time_delay_work);

	bq_info("success\n");
}

static int bq2589x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return 0;

	if (device_may_wakeup(dev))
		enable_irq_wake(bq->irq);

	bq_info("Suspend successfully!\n");
	return 0;
}

static int bq2589x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return 0;

	if (device_may_wakeup(dev))
		disable_irq_wake(bq->irq);

	if (bq->wall_psy)
		power_supply_changed(bq->wall_psy);

	bq_info("Resume successfully!\n");
	return 0;
}

static const struct dev_pm_ops bq2589x_pm_ops = {
	.suspend	= bq2589x_suspend,
	.resume		= bq2589x_resume,
};

static const struct of_device_id bq2589x_charger_match_table[] = {
	{ .compatible = "ti,bq2589x-1", },
	{ },
};
MODULE_DEVICE_TABLE(of, bq2589x_charger_match_table);

static const struct i2c_device_id bq2589x_charger_id[] = {
	{ "bq2589x-1", BQ25890 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bq2589x_charger_id);

static struct i2c_driver bq2589x_charger_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "bq2589x-1",
		.of_match_table	= of_match_ptr(bq2589x_charger_match_table),
		.pm		= &bq2589x_pm_ops,
	},
	.id_table	= bq2589x_charger_id,
	.probe		= bq2589x_charger_probe,
	.remove		= bq2589x_charger_remove,
	.shutdown	= bq2589x_charger_shutdown,
};

static int __init bq2589x_init(void)
{
	int ret;

	bq2589x_class = class_create(THIS_MODULE, "bq2589x");
	if (IS_ERR(bq2589x_class))
		return PTR_ERR(bq2589x_class);

	ret = i2c_add_driver(&bq2589x_charger_driver);
	if (ret)
		class_destroy(bq2589x_class);

	return ret;
}

static void __exit bq2589x_exit(void)
{
	i2c_del_driver(&bq2589x_charger_driver);
	class_destroy(bq2589x_class);
}

module_init(bq2589x_init);
module_exit(bq2589x_exit);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
