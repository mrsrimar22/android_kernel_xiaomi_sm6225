/*
 *otg-gpio BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#define PROFILE_CHG_VOTER		"PROFILE_CHG_VOTER"
#define MAIN_SET_VOTER			"MAIN_SET_VOTER"
//#define PD2SW_HITEMP_OCCURE_VOTER	"PD2SW_HITEMP_OCCURE_VOTER"
#define CHG_FCC_CURR_MAX		6000
#define CHG_ICL_CURR_MAX		3000
#define NOTIFY_COUNT_MAX		40
#define NO_CHANGE_MAX		5
#define RETRY_MS		500
#define RETRY_TIMEOUT_MS	10000
#define POST_INTERVAL		(2 * HZ)
#define RESET_GAP		(5 * HZ)
#define MAIN_ICL_MIN			100
extern bool g_ffc_disable;

enum print_reason {
	PR_INTERRUPT	= BIT(0),
	PR_REGISTER	= BIT(1),
	PR_OEM		= BIT(2),
	PR_DEBUG	= BIT(3),
};

static int debug_mask = PR_OEM;
module_param_named(debug_mask, debug_mask, int, 0600);

// Legacy
#define bq_dbg(...)	do { } while (0)

#define _BQ_RAW(a, b) a##b
#define _BQ_NAME(a, b) _BQ_RAW(a, b)

#if defined(__COUNTER__)
#define _BQ_ID() __COUNTER__
#else
#define _BQ_ID() __LINE__
#endif

#define _BQ_WRAPPER_ID(id, fmt, ...)					\
do {									\
	static DEFINE_RATELIMIT_STATE(_BQ_NAME(bq_rl_, id), 5 * HZ, 1);	\
	if (__ratelimit(&_BQ_NAME(bq_rl_, id)))				\
		pr_info(fmt, ##__VA_ARGS__);				\
} while (0)

#define bq_log(fmt, ...)	_BQ_WRAPPER_ID(_BQ_ID(), fmt, ##__VA_ARGS__)
#define bq_err(fmt, ...)	do { pr_err(fmt, ##__VA_ARGS__); } while (0)
#define bq_debug(fmt, ...)	do { pr_debug(fmt, ##__VA_ARGS__); } while (0)
#define bq_info(fmt, ...)	do { pr_info(fmt, ##__VA_ARGS__); } while (0)

static struct bq2589x *g_bq;
static struct pe_ctrl pe;
//int get_apdo_regain;
static bool vbus_on = false;

extern void start_fg_monitor_work(struct power_supply *psy);
extern void stop_fg_monitor_work(struct power_supply *psy);
extern char nopmi_set_charger_ic_type(NOPMI_CHARGER_IC_TYPE nopmi_type);

static int bq2589x_set_fast_charge_mode(struct bq2589x *bq, int pd_active)
{
	int rc = 0;
	union power_supply_propval propval = {0, };
	int batt_verify = 0, batt_soc = 0, batt_temp = 0;

	if (!bq->bms_psy)
		bq->bms_psy = power_supply_get_by_name("bms");
	if (bq->bms_psy) {
		rc = power_supply_get_property(bq->bms_psy, POWER_SUPPLY_PROP_CHIP_OK, &propval);
		if (rc < 0)
			bq_err("get battery chip ok fail\n");
		else
			batt_verify = propval.intval;

		rc = power_supply_get_property(bq->bms_psy, POWER_SUPPLY_PROP_CAPACITY, &propval);
		if (rc < 0)
			bq_err("get battery capacity fail\n");
		else
			batt_soc = propval.intval;

		rc = power_supply_get_property(bq->bms_psy, POWER_SUPPLY_PROP_TEMP, &propval);
		if (rc < 0)
			bq_err("get battery temp fail\n");
		else
			batt_temp = propval.intval;
	} else {
		bq_err("bms_psy not found\n");
		return -ENOENT;
	}

	/*If TA plug in with PPS, battery auth success and soc less than 95%, FFC flag will enabled.
		The temp is normal set fastcharge mode as 1 and jeita loop also handle fastcharge prop*/
	//bq_debug("batt_verify: %d, batt_soc: %d, batt_temp: %d\n", batt_verify, batt_soc, batt_temp);
	if ((pd_active == 2) && batt_verify && batt_soc < 95) {
		g_ffc_disable = false;
		propval.intval = (batt_temp >= 150 && batt_temp <= 480) ? 1 : 0;
	} else {
		/*If TA plug in without PPS, battery auth fail and soc exceed 95%, FFC will always be disabled*/
		propval.intval = 0;
		g_ffc_disable = true;
	}

	rc = power_supply_set_property(bq->bms_psy, POWER_SUPPLY_PROP_FASTCHARGE_MODE, &propval);
	if (rc < 0)
		bq_err("set fastcharge mode fail!\n");

	return rc;
}

static int __bq2589x_read_byte(struct bq2589x *bq, u8 reg, u8 *data)
{
	int ret, retry;
	const int max_retry = 3;

	for (retry = 1; retry <= max_retry; retry++) {
		ret = i2c_smbus_read_byte_data(bq->client, reg);
		if (ret >= 0) {
			*data = (u8)ret;
			return 0;
		}

		bq_err("read 0x%02x failed (try %d/%d): %d\n",
				reg, retry, max_retry, ret);

		if (retry < max_retry)
			udelay(200);
	}

	return ret;
}

static int __bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret, retry;
	const int max_retry = 3;

	for (retry = 1; retry <= max_retry; retry++) {
		ret = i2c_smbus_write_byte_data(bq->client, reg, data);
		if (ret >= 0)
			return 0;

		bq_err("write 0x%02x->0x%02x failed (try %d/%d): %d\n",
				data, reg, retry, max_retry, ret);

		if (retry < max_retry)
			udelay(200);
	}

	return ret;
}

static int bq2589x_read_byte(struct bq2589x *bq, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_write_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_read_byte(bq, reg, &tmp);
	if (ret) {
		bq_err("failed to read reg 0x%02x, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2589x_write_byte(bq, reg, tmp);
	if (ret)
		bq_err("failed to write reg 0x%02x, ret=%d\n", reg, ret);

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2589x_enable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
}

static int bq2589x_disable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
}
//EXPORT_SYMBOL_GPL(bq2589x_disable_otg);

#if 0
static int bq2589x_set_otg_volt(struct bq2589x *bq, int volt)
{
	u8 val = 0;

	if (bq->part_no == SC89890H) {
		if (volt < SC89890H_BOOSTV_BASE)
			volt = SC89890H_BOOSTV_BASE;
		if (volt > SC89890H_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * SC89890H_BOOSTV_LSB)
			volt = SC89890H_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * SC89890H_BOOSTV_LSB;

		val = ((volt - SC89890H_BOOSTV_BASE) / SC89890H_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;
	} else {
		if (volt < BQ2589X_BOOSTV_BASE)
			volt = BQ2589X_BOOSTV_BASE;
		if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
			volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;

		val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;
	}

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOSTV_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_volt);
#endif

static int bq2589x_set_otg_current(struct bq2589x *bq, int curr)
{
	u8 temp;

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
		else if (curr < 2500)
			temp = SC89890H_BOOST_LIM_2450MA;
		else
			temp = SC89890H_BOOST_LIM_1400MA;
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

	return bq2589x_update_bits(bq, BQ2589X_REG_0A, BQ2589X_BOOST_LIM_MASK, temp << BQ2589X_BOOST_LIM_SHIFT);
}
//EXPORT_SYMBOL_GPL(bq2589x_set_otg_current);

static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		bq->status |= BQ2589X_STATUS_CHARGE_ENABLE;

	return ret;
}

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if (ret == 0)
		bq->status &= ~BQ2589X_STATUS_CHARGE_ENABLE;

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_disable_charger);

/* interfaces that can be called by other module */
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
		return 0; /*is doing continuous scan*/

	if (oneshot)
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_adc_start);

static int bq2589x_adc_stop(struct bq2589x *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
//EXPORT_SYMBOL_GPL(bq2589x_adc_stop);

static int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0E, &val);
	if (ret < 0) {
		bq_err("read battery voltage failed: %d\n", ret);
		return ret;
	} else {
		volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB;
		return volt;
	}
}
//EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);

#if 0
int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0F, &val);
	if (ret < 0) {
		bq_err("read system voltage failed: %d\n", ret);
		return ret;
	} else {
		volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);
#endif

static int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_11, &val);
	if (ret < 0) {
		bq_err("read vbus voltage failed: %d\n", ret);
		return ret;
	} else {
		volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB;
		return volt;
	}
}
//EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

#if 0
int bq2589x_adc_read_temperature(struct bq2589x *bq)
{
	uint8_t val;
	int temp;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_10, &val);
	if (ret < 0) {
		bq_err("read temperature failed: %d\n", ret);
		return ret;
	} else {
		temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB;
		return temp;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);
#endif

static int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_12, &val);
	if (ret < 0) {
		bq_err("read charge current failed: %d\n", ret);
		return ret;
	} else {
		volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB);
		return volt;
	}
}
//EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

static int bq2589x_set_charge_current(struct bq2589x *bq, int curr)
{
	u8 ichg;

	if (bq->part_no == SC89890H) {
		ichg = (curr - SC89890H_ICHG_BASE) / SC89890H_ICHG_LSB;
	} else {
		ichg = (curr - BQ2589X_ICHG_BASE) / BQ2589X_ICHG_LSB;
	}

	return bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);
}

static int bq2589x_get_charge_current(struct bq2589x *bq)
{
	u8 val;
	int ret = 0;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_04, &val);
	if (ret < 0) {
		bq_err("failed to read register 0x00:%d\n", ret);
		return ret;
	}

	return ((val & BQ2589X_ICHG_MASK) >> BQ2589X_ICHG_SHIFT) * BQ2589X_ICHG_LSB + BQ2589X_ICHG_BASE;
}
//EXPORT_SYMBOL_GPL(bq2589x_set_charge_current);

static int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	if (bq->part_no == SC89890H) {
		if (curr > SC89890H_ITERM_MAX) {
			curr = SC89890H_ITERM_MAX;
		}
		iterm = (curr - SC89890H_ITERM_BASE) / SC89890H_ITERM_LSB;
	} else {
		iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;
	}

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}
//EXPORT_SYMBOL_GPL(bq2589x_set_term_current);

static int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	if (bq->part_no == SC89890H) {
		iprechg = (curr - SC89890H_IPRECHG_BASE) / SC89890H_IPRECHG_LSB;
	} else {
		iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;
	}

	return bq2589x_update_bits(bq, BQ2589X_REG_05, BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}
//EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);

static int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val;

	val = (volt - BQ2589X_VREG_BASE) / BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}
//EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);

static int bq2589x_get_chargevoltage(struct bq2589x *bq)
{
	u8 val;
	int ret = 0;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_06, &val);
	if (ret < 0) {
		bq_err("failed to read register 0x00:%d\n", ret);
		return ret;
	}
	return ((val & BQ2589X_VREG_MASK) >> BQ2589X_VREG_SHIFT) * BQ2589X_VREG_LSB + BQ2589X_VREG_BASE;
}
//EXPORT_SYMBOL_GPL(bq2589x_get_chargevoltage);

static int main_set_charge_voltage(int volt)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(g_bq))
		return -1;

	ret = bq2589x_set_chargevoltage(g_bq, volt);
	bq_debug("end main_set_charge_voltage, ret=%d\n", ret);

	return ret;
}

static int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val;

	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}
//EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);

static int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{
	u8 val;

	if (curr < BQ2589X_IINLIM_BASE)
		curr = BQ2589X_IINLIM_BASE;

	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}
//EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);

static int bq2589x_get_input_current_limit(struct bq2589x *bq)
{
	u8 val;
	int ret = 0;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_00, &val);
	if (ret < 0) {
		bq_err("failed to read register 0x00:%d\n", ret);
		return ret;
	}
	return ((val & BQ2589X_IINLIM_MASK) >> BQ2589X_IINLIM_SHIFT) * BQ2589X_IINLIM_LSB + BQ2589X_IINLIM_BASE;
}
//EXPORT_SYMBOL_GPL(bq2589x_get_input_current_limit);

static int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;

	if (bq->part_no == SC89890H) {
		if (offset < 500) {
			val = SC89890h_VINDPMOS_400MV;
		} else {
			val = SC89890h_VINDPMOS_600MV;
		}
		return bq2589x_update_bits(bq, BQ2589X_REG_01, SC89890H_VINDPMOS_MASK, val << SC89890H_VINDPMOS_SHIFT);
	} else {
		val = (offset - BQ2589X_VINDPMOS_BASE) / BQ2589X_VINDPMOS_LSB;
		return bq2589x_update_bits(bq, BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
	}

	return 0;
}
//EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);

static u8 bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 val = 0;
	int ret, cap = -1;
	union power_supply_propval propval = {0, };

	if (!bq)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &val);
	if (ret < 0) {
		bq_err("failed to read register 0x0b:%d\n", ret);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	if (val == BQ2589X_CHRG_STAT_IDLE) {
		bq_log("not charging\n");
		return POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (val == BQ2589X_CHRG_STAT_PRECHG) {
		bq_log("precharging\n");
		return POWER_SUPPLY_STATUS_CHARGING;
	} else if (val == BQ2589X_CHRG_STAT_FASTCHG) {
		bq_log("fast charging\n");
		return POWER_SUPPLY_STATUS_CHARGING;
	} else if (val == BQ2589X_CHRG_STAT_CHGDONE) {
		bq_log("charge done!\n");
		if (!bq->bms_psy)
			bq->bms_psy = power_supply_get_by_name("bms");
		if (bq->bms_psy) {
			ret = power_supply_get_property(bq->bms_psy, POWER_SUPPLY_PROP_CAPACITY, &propval);
			if (ret < 0) {
				bq_err("get battery cap fail\n");
			}
			cap = propval.intval;
			bq_log("battery cap: %d\n", cap);
		}

		if (cap > 95) {
			return POWER_SUPPLY_STATUS_FULL;
		} else {
			return POWER_SUPPLY_STATUS_CHARGING;
		}
	}

	return POWER_SUPPLY_STATUS_UNKNOWN;
}
//EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);

static void bq2589x_set_otg(struct bq2589x *bq, int enable)
{
	int ret;

	if (enable) {
		if (bq->part_no == SC89890H) {
			bq2589x_disable_charger(bq);
		}
		ret = bq2589x_enable_otg(bq);
		if (ret < 0) {
			bq_err("failed to enable otg-%d\n", ret);
			return;
		}
	} else {
		ret = bq2589x_disable_otg(bq);
		if (ret < 0) {
			bq_err("failed to disable otg-%d\n", ret);
			return;
		}
		if (bq->part_no == SC89890H) {
			bq2589x_enable_charger(bq);
		}
	}
}
//EXPORT_SYMBOL_GPL(bq2589x_set_otg);

static int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}

static __maybe_unused int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, (u8)((timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB) << BQ2589X_WDT_SHIFT);
}

static int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}

static int bq2589x_is_dpdm_done(struct bq2589x *bq, int *done)
{
	int ret = 0;
	u8 data = 0;

//modify by HTH-209427/HTH-209841 at 2022/05/12 begin
	if (bq->part_no == SC89890H) {
		ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &data);
		if (data & BQ2589X_PG_STAT_MASK) {
			*done = 0;
		} else {
			*done = 1;
		}
	} else {
		ret = bq2589x_read_byte(bq, BQ2589X_REG_02, &data);
		//bq_info("data(0x%x)\n", data);
		data &= (BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT);
		*done = (data >> BQ2589X_FORCE_DPDM_SHIFT);
	}

	return ret;
//modify by HTH-209427/HTH-209841 at 2022/05/12 end
}

static int bq2589x_force_dpdm(struct bq2589x *bq)
{
	u8 data = 0;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

//modify by HTH-209427/HTH-209841/HTH-234945/HTH-234948 at 2022/06/08 begin
	if (bq->part_no == SC89890H && bq->vbus_type == BQ2589X_VBUS_MAXC) {
		bq2589x_read_byte(bq, BQ2589X_REG_0B, &data);
		bq_info("0x0B = 0x%02x\n", data);
		if ((data & 0xE0) == 0x80) {
			bq2589x_write_byte(bq, BQ2589X_REG_01, 0x45);
			msleep(30);
			bq2589x_write_byte(bq, BQ2589X_REG_01, 0x25);
			msleep(30);
		}
	}
//modify by HTH-209427/HTH-209841/HTH-234945/HTH-234948 at 2022/06/08 end

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
}

/*void bq2589x_force_dpdm_done(struct bq2589x *bq)
{
	int retry = 0;
	int bc_count = 200;
	int done = 1;

	bq->status &= ~BQ2589X_STATUS_PLUGIN;
	bq2589x_force_dpdm(bq);

	while (retry++ < bc_count) {
		bq2589x_is_dpdm_done(bq, &done);
		msleep(20);
		if (!done) //already known charger type
			break;
	}
}*/

static int bq2589x_force_dpdm_done(struct bq2589x *bq)
{
	int ret = 0;
	int done = 1;
	int retry = 200; /* 200 * 20ms = ~4s total timeout */

	mutex_lock(&bq->dpdm_lock);

	bq->status &= ~BQ2589X_STATUS_PLUGIN;
	bq_info("force DPDM start\n");

	ret = bq2589x_force_dpdm(bq);
	if (ret < 0) {
		bq_err("failed to trigger DPDM: (%d)\n", ret);
		goto out_unlock;
	}

	do {
		ret = bq2589x_is_dpdm_done(bq, &done);
		if (ret < 0) {
			bq_err("read DPDM done status failed (%d)\n", ret);
			goto out_unlock;
		}

		if (!done) {
			bq_info("DPDM done\n");
			ret = 0;
			goto out_unlock;
		}

		if (--retry > 0)
			msleep(20);

	} while (retry > 0);

	bq_err("DPDM timeout\n");
	ret = 1;

out_unlock:
	mutex_unlock(&bq->dpdm_lock);
	return ret;
}

#if 1
static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &val);
	if (ret < 0) {
		bq_err("failed to read 0B byte, ret: %d\n", ret);
		return 0;
	}

	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}

static enum bq2589x_vbus_type bq2589x_get_vbus_valid(struct bq2589x *bq)
{
	enum bq2589x_vbus_type vbus_type = BQ2589X_VBUS_UNKNOWN;
	int dpdm_status;
	int retry = 3;

	dpdm_status = bq2589x_force_dpdm_done(bq);
	if (dpdm_status != 0) {
		bq_err("DPDM handshake failed (%d)\n", dpdm_status);
		return vbus_type;
	}

	do {
		vbus_type = bq2589x_get_vbus_type(bq);
		if (vbus_type != BQ2589X_VBUS_UNKNOWN)
			break;

		msleep(100);
	} while (--retry);

	bq_info("vbus_type: %d\n", vbus_type);
	return vbus_type;
}
#endif

static int bq2589x_reset_chip(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_RESET_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

#if 0
int bq2589x_enter_ship_mode(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enter_ship_mode);
#endif

static int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
}
//EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

static int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
}
//EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

#if 0
int bq2589x_get_hiz_mode(struct bq2589x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_00, &val);
	if (ret)
		return ret;

	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);

int bq2589x_pumpx_enable(struct bq2589x *bq, int enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);
#endif

static int bq2589x_pumpx_increase_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);

static int bq2589x_pumpx_increase_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_09, &val);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1; /* not finished*/
	else
		return 0; /* pumpx up finished*/
}
//EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);

static int bq2589x_pumpx_decrease_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);

static int bq2589x_pumpx_decrease_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_09, &val);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1; /* not finished*/
	else
		return 0; /* pumpx down finished*/
}
//EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);

static int bq2589x_force_ico(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_force_ico);

static int bq2589x_check_force_ico_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_14, &val);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1; /*finished*/
	else
		return 0; /* in progress*/
}
//EXPORT_SYMBOL_GPL(bq2589x_check_force_ico_done);

static int bq2589x_enable_term(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_enable_term);

static int bq2589x_enable_auto_dpdm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);

static int bq2589x_use_absolute_vindpm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_use_absolute_vindpm);

static int bq2589x_enable_ico(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;
}
//EXPORT_SYMBOL_GPL(bq2589x_enable_ico);

#if 0
static int bq2589x_read_idpm_limit(struct bq2589x *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_13, &val);
	if (ret < 0) {
		bq_err("read vbus voltage failed: %d\n", ret);
		return ret;
	} else {
		curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);
#endif

bool bq2589x_is_charge_done(void)
{
	int ret;
	u8 val;

	if (IS_ERR_OR_NULL(g_bq))
		return PTR_ERR(g_bq);

	ret = bq2589x_read_byte(g_bq, BQ2589X_REG_0B, &val);
	if (ret < 0) {
		bq_err("read REG0B failed: %d\n", ret);
		return false;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);

#if 1
static void bq2589x_dump_regs(struct bq2589x *bq)
{
	int addr, ret;
	u8 val;

	bq_debug("dump_regs:\n");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, addr, &val);
		if (ret == 0)
			bq_debug("Reg[%02x] = 0x%02x\n", (unsigned int)addr, (unsigned int)val);
	}
}
#endif

static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;

	/*common initialization*/
	if (bq->part_no == SC89890H) {
		bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENILIM_MASK,
				BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
//modify by HTH-209427/HTH-209841 at 2022/05/12 begin
		bq2589x_enable_ico(bq, false);
	} else {
//modify by HTH-234718 at 2022/05/25 begin
		bq2589x_enable_ico(bq, false);
//modify by HTH-234718 at 2022/05/25 end
	}
//modify by HTH-209427/HTH-209841 at 2022/05/12 end
	bq2589x_disable_watchdog_timer(bq);

	bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm);
	bq2589x_enable_term(bq, bq->cfg.enable_term);
	/*force use absolute vindpm if auto_dpdm not enabled*/
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

	main_set_charge_enable(true);
	//bq2589x_adc_start(bq, false);
	/*if (ret) {
		bq_err("failed to enable pumpx: %d\n", ret);
		return ret;
	}*/

	//bq2589x_set_watchdog_timer(bq, 160);
	bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENILIM_MASK, BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
	bq2589x_update_bits(bq, BQ2589X_REG_02, 0x8, 1 << 3);

	/* 2022.5.18 longcheer tangyanchang edit start */
	if (bq->part_no == SYV690) {
		bq_info("init syv690 HV_TYPE 9/12V\n");
		bq2589x_update_bits(bq, BQ2589X_REG_02, 0x4, 0 << 2); //HV_TYPE 0-9V/1-12V
	}
	/* 2022.5.18 longcheer tangyanchang edit end */

	//bq2589x_update_bits(bq, BQ2589X_REG_01, 0x2, 0 << 1);
	bq2589x_adc_stop(bq);

	return ret;
}

#if 1
static int bq2589x_charge_status(struct bq2589x *bq)
{
	u8 val = 0;

	if (!bq)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq2589x_read_byte(bq, BQ2589X_REG_0B, &val);
	bq_debug("REG_0B=0x%x\n", val);

	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
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
#endif

static enum power_supply_property bq2589x_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_TERM_CURRENT,
	//POWER_SUPPLY_PROP_BATT_CHARGE_TYPE, /* Batt Charge status output */
	POWER_SUPPLY_PROP_CHARGE_TYPE, /* Charger status output */
};
extern int get_prop_battery_charging_enabled(struct votable *usb_icl_votable,
					union power_supply_propval *val);

static int bq2589x_wall_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq2589x *bq = power_supply_get_drvdata(psy);
	int online = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2589x_get_charging_status(bq);
#if 0
		if (get_effective_result_locked(bq->fv_votable) < 4450) {
			if (val->intval == POWER_SUPPLY_STATUS_FULL) {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
		} else if (get_client_vote_locked(bq->usb_icl_votable, "MAIN_CHG_SUSPEND_VOTER") == MAIN_ICL_MIN) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
#endif
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		online = READ_ONCE(bq->chg_online);
		if (bq->vbat_volt < 3300)
			online = 0;
		val->intval = online;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = READ_ONCE(bq->chg_type);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq->vbus_volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq->chg_current;
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = bq->enabled;
		break;
	case POWER_SUPPLY_PROP_TERM_CURRENT:
		val->intval = bq->cfg.term_current;
		break;
	//case POWER_SUPPLY_PROP_BATT_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_charge_status(bq);
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
	//case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_REAL_TYPE:
		WRITE_ONCE(bq->chg_type, val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		WRITE_ONCE(bq->chg_online, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		bq->vbus_volt = val->intval;
		ret = main_set_charge_voltage(bq->vbus_volt);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		bq->chg_current = val->intval;
		ret = main_set_charge_current(bq->chg_current);
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		bq->enabled = val->intval;
		ret = main_set_charge_enable(bq->enabled);
		break;
	case POWER_SUPPLY_PROP_TERM_CURRENT:
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
	//case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_REAL_TYPE:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TERM_CURRENT:
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
	// bq->wall.type = POWER_SUPPLY_TYPE_MAINS;
	bq->wall.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
	bq->wall.properties = bq2589x_charger_props;
	bq->wall.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	bq->wall.get_property = bq2589x_wall_get_property;
	bq->wall.set_property = bq2589x_wall_set_property;
	bq->wall.property_is_writeable = bq2589x_wall_prop_is_writeable;
	bq->wall.external_power_changed = NULL;

	wall_cfg.drv_data = bq;
	wall_cfg.of_node = bq->dev->of_node;
	wall_cfg.num_supplicants = 0;

	bq->wall_psy = devm_power_supply_register(bq->dev, &bq->wall, &wall_cfg);
	if (IS_ERR(bq->wall_psy)) {
		bq_err("failed to register wall psy\n");
		return PTR_ERR(bq->wall_psy);
	}

	bq_info("%s power supply register successfully\n", bq->wall.name);

	return 0;
}

static void bq2589x_psy_unregister(struct bq2589x *bq)
{
	bq_info("start unregister\n");
}

/*static ssize_t bq2589x_show_registers(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	if (IS_ERR_OR_NULL(g_bq))
		return idx;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "Charger 1");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq, addr, &val);
		if (ret == 0) {
			//len = snprintf(tmpbuf, PAGE_SIZE - idx, "Reg[0x%.2x] = 0x%.2x\n", addr, val);
			len = snprintf(tmpbuf, PAGE_SIZE - idx, "Reg[0x%02x] = 0x%02x\n", (unsigned int)addr, (unsigned int)val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}
static DEVICE_ATTR(registers, S_IRUGO, bq2589x_show_registers, NULL);*/

static ssize_t bq2589x_show_registers(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	int idx = 0;
	int ret;

	if (IS_ERR_OR_NULL(g_bq))
		return -ENODEV;

	idx += scnprintf(buf + idx, PAGE_SIZE - idx, "Charger 1:\n");

	for (addr = 0x00; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq, addr, &val);
		if (ret) {
			dev_warn(g_bq->dev, "read reg 0x%02x failed: %d\n", addr, ret);
			continue;
		}

		idx += scnprintf(buf + idx, PAGE_SIZE - idx, "Reg[0x%02x] = 0x%02x\n", addr, val);
		if (idx >= PAGE_SIZE)
			break;
	}

	return idx;
}

static ssize_t bq2589x_store_registers(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	char tmp[32];
	unsigned long reg_ul;
	unsigned long val_ul;
	int ret;
	char *p;

	if (IS_ERR_OR_NULL(g_bq))
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

	if (reg_ul > 0x14 || val_ul > 0xff) {
		dev_err(g_bq->dev, "invalid reg/val: reg=0x%lx val=0x%lx\n", reg_ul, val_ul);
		return -EINVAL;
	}

	ret = bq2589x_write_byte(g_bq, (u8)reg_ul, (u8)val_ul);
	if (ret) {
		dev_err(g_bq->dev, "write reg 0x%02lx failed: %d\n", reg_ul, ret);
		return ret;
	}

	dev_info(g_bq->dev, "wrote 0x%02lx to reg 0x%02lx\n", val_ul, reg_ul);
	return count;
}

static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, bq2589x_show_registers, bq2589x_store_registers);

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

	// validate of
	if (!np) {
		bq_err("no device tree node\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-high-level", &pe.high_volt_level);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-low-level", &pe.low_volt_level);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,vbat-min-volt-to-tuneup", &pe.vbat_min_volt);
	if (ret)
		return ret;

	bq->cfg.enable_auto_dpdm = of_property_read_bool(np, "ti,bq2589x,enable-auto-dpdm");
	bq->cfg.enable_term = of_property_read_bool(np, "ti,bq2589x,enable-termination");
	bq->cfg.enable_ico = of_property_read_bool(np, "ti,bq2589x,enable-ico");
	bq->cfg.use_absolute_vindpm = of_property_read_bool(np, "ti,bq2589x,use-absolute-vindpm");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage", &bq->cfg.charge_voltage);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current", &bq->cfg.charge_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current-3500", &bq->cfg.charge_current_3500);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current-1500", &bq->cfg.charge_current_1500);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current-1000", &bq->cfg.charge_current_1000);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current-500", &bq->cfg.charge_current_500);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,input-current-2000", &bq->cfg.input_current_2000);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2589x,term-current", &bq->cfg.term_current);
	if (ret)
		return ret;

	// buggy ret usage
	/*bq->irq_gpio = of_get_named_gpio(np, "intr-gpios", 0);
	if (ret < 0) {
		bq_err("no intr_gpio info\n");
		return ret;
	} else {
		bq_info("intr_gpio info: %d\n", bq->irq_gpio);
	}

	bq->usb_switch1_gpio = of_get_named_gpio(np, "usb-switch1-gpios", 0);
	if (ret < 0) {
		bq_err("no usb-switch1 info\n");
		return ret;
	}*/
	// drop legacy, use devm instead
	bq->irq_gpiod = devm_gpiod_get(dev, "intr", GPIOD_IN);
	if (IS_ERR(bq->irq_gpiod)) {
		ret = PTR_ERR(bq->irq_gpiod);
		bq_err("devm_gpiod_get(intr) failed: %d\n", ret);
		return ret;
	}
	bq_info("intr descriptor acquired\n");

	bq->usb_switch1_gpiod = devm_gpiod_get(dev, "usb-switch1", GPIOD_OUT_LOW);
	if (IS_ERR(bq->usb_switch1_gpiod)) {
		ret = PTR_ERR(bq->usb_switch1_gpiod);
		bq_err("devm_gpiod_get(usb-switch1) failed: %d\n", ret);
		return ret;
	}
	bq_info("usb-switch1 descriptor acquired\n");

	return 0;
}

/*static void bq2589x_usb_switch(struct bq2589x *bq, bool en)
{
	int ret = 0;

	//msleep(5);
	bq_info("%d\n", en);
	mutex_lock(&bq->usb_switch_lock);
	ret = gpio_direction_output(bq->usb_switch1_gpio, en);
	bq->usb_switch_flag = en;
	mutex_unlock(&bq->usb_switch_lock);
}*/

static void bq2589x_usb_switch(struct bq2589x *bq, bool en)
{
	unsigned long flags;
	int new_flag = -1;

	if (!bq)
		return;

	/* nothing to do if already in desired state */
	if (bq->usb_switch_flag == en)
		return;

	/* descriptor-only path */
	if (!bq->usb_switch1_gpiod)
		return;

	if (in_atomic()) {
		spin_lock_irqsave(&bq->usb_switch_lock_spin, flags);
		if (bq->usb_switch_flag != en) {
			gpiod_set_value(bq->usb_switch1_gpiod, en);
			new_flag = !!gpiod_get_value(bq->usb_switch1_gpiod);
			bq->usb_switch_flag = new_flag;
		}
		spin_unlock_irqrestore(&bq->usb_switch_lock_spin, flags);
		if (new_flag >= 0)
			bq_info("usb_switch (atomic, desc) set to %d\n", new_flag);
	} else {
		mutex_lock(&bq->usb_switch_lock);
		if (bq->usb_switch_flag != en) {
			gpiod_set_value_cansleep(bq->usb_switch1_gpiod, en);
			new_flag = gpiod_get_value_cansleep(bq->usb_switch1_gpiod);
			bq->usb_switch_flag = !!new_flag;
		}
		mutex_unlock(&bq->usb_switch_lock);
		if (new_flag >= 0)
			bq_info("usb_switch (desc) set to %d\n", new_flag);
	}
}

static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, BQ2589X_REG_14, &data);
	if (ret == 0) {
		bq->part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	}

	return ret;
}

static int bq2589x_read_batt_rsoc(struct bq2589x *bq)
{
	union power_supply_propval ret = {0,};

	if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");
	if (bq->batt_psy) {
		power_supply_get_property(bq->batt_psy, POWER_SUPPLY_PROP_CAPACITY, &ret);
		return ret.intval;
	} else {
		return 50;
	}
}

static void bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	u16 vbus_volt = 0;
	u16 vindpm_volt = 0;
	int ret = 0;

	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if (vbus_volt < 6000)
		//vindpm for 5v charger
		if (bq->vbus_type == BQ2589X_VBUS_USB_DCP) {
			vindpm_volt = 4500;
		} else {
			vindpm_volt = 4600;
		}
	else
		vindpm_volt = 8300; //vindpm for 9v+ charger

	ret = bq2589x_set_input_volt_limit(bq, vindpm_volt);
	if (ret < 0)
		bq_err("set absolute vindpm threshold %d failed: %d\n", vindpm_volt, ret);
	else
		bq_info("set absolute vindpm threshold %d successfully\n", vindpm_volt);
}

int main_set_charge_enable(bool en)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(g_bq))
		return -1;

	bq_info("charge_enable: %d\n", en);
	if (en)
		ret = bq2589x_enable_charger(g_bq);
	else
		ret = bq2589x_disable_charger(g_bq);

	return ret;
}
EXPORT_SYMBOL_GPL(main_set_charge_enable);

int main_set_hiz_mode(bool en)
{
	if (IS_ERR_OR_NULL(g_bq))
		return -1;

	if (en)
		bq2589x_enter_hiz_mode(g_bq);
	else
		bq2589x_exit_hiz_mode(g_bq);

	return 0;
}
EXPORT_SYMBOL_GPL(main_set_hiz_mode);

int main_set_input_current_limit(int curr)
{
	if (IS_ERR_OR_NULL(g_bq))
		return -1;

	bq2589x_set_input_current_limit(g_bq, curr);
	return 0;
}
EXPORT_SYMBOL_GPL(main_set_input_current_limit);

int main_set_charge_current(int curr)
{
	if (IS_ERR_OR_NULL(g_bq))
		return -1;

	bq_info("charge_current: %d\n", curr);
	vote(g_bq->fcc_votable, MAIN_SET_VOTER, true, curr);

	return 0;
}
EXPORT_SYMBOL_GPL(main_set_charge_current);

int main_get_charge_type(void)
{
	u8 type;

	if (IS_ERR_OR_NULL(g_bq))
		return -1;

	type = g_bq->vbus_type; //bq2589x_get_vbus_type(g_bq); 2021.09.11 wsy edit for crash
	return (int)type;
}
EXPORT_SYMBOL_GPL(main_get_charge_type);

static void bq2589x_adapter_in_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_in_work);
	int ret;
	union power_supply_propval propval = {0, };

/*//modify by HTH-209427/HTH-209841 at 2022/05/12 begin
	bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);
//modify by HTH-209427/HTH-209841 at 2022/05/12 end
	bq2589x_adc_start(bq, false);*/
	switch (bq->vbus_type) {
	case BQ2589X_VBUS_MAXC:
		bq_info("charger_type: MAXC\n");
		bq2589x_enable_ico(bq, !bq->cfg.enable_ico);
		bq2589x_set_input_volt_limit(bq, 8300);
		vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 1800);
		/*if (bq->cfg.use_absolute_vindpm)
			bq2589x_adjust_absolute_vindpm(bq);*/
		//schedule_delayed_work(&bq->ico_work, 0);
		bq2589x_usb_switch(bq, true);
		break;
	case BQ2589X_VBUS_USB_DCP:
		bq_info("charger_type: DCP, pd_active=%d\n", bq->pd_active);
		vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, bq->cfg.input_current_2000);
		schedule_delayed_work(&bq->check_pe_tuneup_work, 0);
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
			ret = power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_MTBF_CUR, &propval);
			if (ret < 0) {
				bq_err("get mtbf current fail\n");
			}
		}

		//vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 500);
		if (!bq->pd_active) {
			if (propval.intval >= 1500) {
				vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, propval.intval);
			} else {
				vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 500);
			}
		} else {
			vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 1500);
		}
		bq2589x_usb_switch(bq, false);
		break;
	case BQ2589X_VBUS_NONSTAND:
	case BQ2589X_VBUS_UNKNOWN:
		bq_info("charger_type: FLOAT, pd_active=%d\n", bq->pd_active);
		if (!bq->usb_psy)
			bq->usb_psy = power_supply_get_by_name("usb");
		if (bq->usb_psy) {
			ret = power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_MTBF_CUR, &propval);
			if (ret < 0) {
				bq_err("get mtbf current fail\n");
			}
		}

		//vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 500);
		if (!bq->pd_active) {
			if (propval.intval >= 1500) {
				vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, propval.intval);
			} else {
				vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 1000);
			}
		} else {
			vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, bq->cfg.input_current_2000);
		}
		bq2589x_usb_switch(bq, false); //HTH-191813
		break;
	default:
		bq_info("charger_type: Other, vbus_type is %d\n", bq->vbus_type);
		bq2589x_usb_switch(bq, false);
		schedule_delayed_work(&bq->ico_work, 0);
		break;
	}

	if (bq->vbus_type == BQ2589X_VBUS_USB_SDP && !bq->pd_active) {
		vote(bq->fcc_votable, MAIN_SET_VOTER, true, 500);
	} else {
		vote(bq->fcc_votable, MAIN_SET_VOTER, false, 0);
	}
	//bq2589x_dump_regs(bq);
	//power_supply_changed(bq->usb_psy);
	//cancel_delayed_work_sync(&bq->monitor_work);
	schedule_delayed_work(&bq->monitor_work, 0);
}

static void bq2589x_adapter_out_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_out_work);
	int ret;

//modify by HTH-209427/HTH-209841 at 2022/05/12 begin
	ret = bq2589x_set_input_volt_limit(bq, 4600);
	if (ret < 0)
		bq_err("reset vindpm threshold to 4600 failed (%d)\n", ret);
	else
		bq_info("reset vindpm threshold to 4600 successfully\n");
	vote(bq->fcc_votable, MAIN_SET_VOTER, true, 500);
	vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, 0);
//modify by HTH-209427/HTH-209841 at 2022/05/12 end

	cancel_delayed_work_sync(&bq->monitor_work);
}

static void bq2589x_charger_workfunc(struct work_struct *work)
{
	u8 type_now = 0;
	struct bq2589x *bq = container_of(work, struct bq2589x, charger_work.work);

	if (!bq->batt_psy)
		return;

	type_now = bq2589x_get_charging_status(bq);
	if (type_now > 0)
		power_supply_changed(bq->batt_psy);

	bq_info("type_now: %d\n", type_now);
}

static void bq2589x_ico_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, ico_work.work);
	int ret;
	int idpm;
	u8 status;
	static bool ico_issued;

	if (bq->part_no == SYV690) {
		bq_info("SYV690 IC detected, skip ico\n");
		return;
	}

	if (!ico_issued) {
		ret = bq2589x_force_ico(bq);
		if (ret < 0) {
			schedule_delayed_work(&bq->ico_work, HZ); /* retry 1 second later*/
		} else {
			ico_issued = true;
			schedule_delayed_work(&bq->ico_work, 3 * HZ);
		}
	} else {
		ico_issued = false;
		ret = bq2589x_check_force_ico_done(bq);
		if (ret) { /*ico done*/
			ret = bq2589x_read_byte(bq, BQ2589X_REG_13, &status);
			if (ret == 0) {
				idpm = ((status & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB + BQ2589X_IDPM_LIM_BASE;
			}
		}
	}
}

static void bq2589x_check_pe_tuneup_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, check_pe_tuneup_work.work);

	if (!pe.enable) {
		schedule_delayed_work(&bq->ico_work, 0);
		return;
	}

	bq->vbat_volt = bq2589x_adc_read_battery_volt(bq);
	bq->rsoc = bq2589x_read_batt_rsoc(bq);

	if (bq->vbat_volt > pe.vbat_min_volt && bq->rsoc < 95) {
		pe.target_volt = pe.high_volt_level;
		pe.tune_up_volt = true;
		pe.tune_down_volt = false;
		pe.tune_done = false;
		pe.tune_count = 0;
		pe.tune_fail = false;
		schedule_delayed_work(&bq->pe_volt_tune_work, 0);
	} else if (bq->rsoc >= 95) {
		schedule_delayed_work(&bq->ico_work, 0);
	} else {
		/* wait battery voltage up enough to check again */
		schedule_delayed_work(&bq->check_pe_tuneup_work, 2 * HZ);
	}
}

//20220211 : Only for time delay
/*static void time_delay_work(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, time_delay_work.work);
	enum bq2589x_vbus_type vbus_type = BQ2589X_VBUS_UNKNOWN;

	vbus_type = bq2589x_get_vbus_type(bq);
	if (vbus_type == BQ2589X_VBUS_NONE || (vbus_type == BQ2589X_VBUS_UNKNOWN && bq->pd_active == 0)) {
		bq2589x_usb_switch(bq, true);
		bq2589x_force_dpdm_done(bq);
		mdelay(1000);
	}

	vbus_type = bq2589x_get_vbus_type(bq);
	if (vbus_type == BQ2589X_VBUS_NONE) {
		bq2589x_usb_switch(bq, false);
	}
	//bq2589x_usb_switch(bq, true);
	//bq2589x_force_dpdm_done(bq);
}*/

static void bq2589x_dpdm_work(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, dpdm_work.work);
	int dpdm_status;

	dpdm_status = bq2589x_force_dpdm_done(bq);
	if (dpdm_status == 0)
		bq_info("DPDM handshake done successfully\n");
	else if (dpdm_status == 1)
		bq_err("DPDM handshake timeout\n");
	else
		bq_err("DPDM handshake failed (%d)\n", dpdm_status);
}

static void time_delay_work(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, time_delay_work.work);
	int rc;
	u8 status;

	if (bq->vbus_type != BQ2589X_VBUS_OTG) {
		bq2589x_usb_switch(bq, true);
		bq->vbus_type = bq2589x_get_vbus_valid(bq);
	}

	if (bq->vbus_type == BQ2589X_VBUS_UNKNOWN)
		bq2589x_usb_switch(bq, false);

	rc = bq2589x_read_byte(bq, BQ2589X_REG_13, &status);
	if (rc == 0 && (status & BQ2589X_VDPM_STAT_MASK) && (bq->pd_active == 0)) {
		if ((bq->vbus_type == BQ2589X_VBUS_MAXC) && (bq->vbus_volt < 8000)) {
			//HVDCP && vbus<8v
			bq_info("HVDCP VINDPM occurred, vbus: %d, reset vindpm!\n", bq->vbus_volt);
			bq2589x_adjust_absolute_vindpm(bq);
		}
	}
}

static void bq2589x_usb_changed_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, usb_changed_work.work);
	union power_supply_propval val = {0, };
	int chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	static int no_change_count;
	static int last_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	static bool last_valid;
	static unsigned long last_jiffies;
	static unsigned long retry_deadline_jiffies;

	if (last_jiffies && time_after(jiffies, last_jiffies + RESET_GAP)) {
		no_change_count = 0;
		last_valid = false;
		last_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		retry_deadline_jiffies = 0;
	}
	last_jiffies = jiffies;

	if (!bq->usb_psy)
		bq->usb_psy = power_supply_get_by_name("usb");
	if (!bq->usb_psy) {
		if (!retry_deadline_jiffies)
			retry_deadline_jiffies = jiffies + msecs_to_jiffies(RETRY_TIMEOUT_MS);

		if (time_before(jiffies, retry_deadline_jiffies)) {
			schedule_delayed_work(&bq->usb_changed_work, msecs_to_jiffies(RETRY_MS));
			return;
		}

		retry_deadline_jiffies = 0;
		last_valid = false;
		no_change_count = 0;
		last_chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		return;
	}

	retry_deadline_jiffies = 0;
	if (power_supply_get_property(bq->usb_psy, POWER_SUPPLY_PROP_REAL_TYPE, &val) == 0)
		chg_type = val.intval;

	WRITE_ONCE(bq->chg_type, chg_type);

	if (!last_valid) {
		last_chg_type = chg_type;
		last_valid = true;
		no_change_count = NO_CHANGE_MAX;
		if (bq->usb_psy)
			power_supply_changed(bq->usb_psy);
		if (bq->wall_psy)
			power_supply_changed(bq->wall_psy);

		no_change_count--;
		if (no_change_count > 0)
			schedule_delayed_work(&bq->usb_changed_work, POST_INTERVAL);
		else
			last_valid = false;
		return;
	}

	if (chg_type != last_chg_type) {
		last_chg_type = chg_type;
		no_change_count = NO_CHANGE_MAX;
		if (bq->usb_psy)
			power_supply_changed(bq->usb_psy);
		if (bq->wall_psy)
			power_supply_changed(bq->wall_psy);

		no_change_count--;
		if (no_change_count > 0)
			schedule_delayed_work(&bq->usb_changed_work, POST_INTERVAL);
		else
			last_valid = false;
		return;
	}

	if (no_change_count > 0) {
		if (bq->usb_psy)
			power_supply_changed(bq->usb_psy);
		if (bq->wall_psy)
			power_supply_changed(bq->wall_psy);
		no_change_count--;
		if (no_change_count > 0)
			schedule_delayed_work(&bq->usb_changed_work, POST_INTERVAL);
		else
			last_valid = false;
	} else {
		last_valid = false;
	}
}

static void bq2589x_tune_volt_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, pe_volt_tune_work.work);
	int ret = 0;
	static bool pumpx_cmd_issued;

	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);

	if ((pe.tune_up_volt && bq->vbus_volt > pe.target_volt) ||
			(pe.tune_down_volt && bq->vbus_volt < pe.target_volt)) {
		pe.tune_done = true;
		bq2589x_adjust_absolute_vindpm(bq);
		if (pe.tune_up_volt)
			schedule_delayed_work(&bq->ico_work, 0);
		return;
	}

	if (pe.tune_count > 10) {
		pe.tune_fail = true;
		bq2589x_adjust_absolute_vindpm(bq);

		if (pe.tune_up_volt)
			schedule_delayed_work(&bq->ico_work, 0);
		return;
	}

	if (!pumpx_cmd_issued) {
		if (pe.tune_up_volt)
			ret = bq2589x_pumpx_increase_volt(bq);
		else if (pe.tune_down_volt)
			ret = bq2589x_pumpx_decrease_volt(bq);

		if (ret) {
			schedule_delayed_work(&bq->pe_volt_tune_work, HZ);
		} else {
			pumpx_cmd_issued = true;
			pe.tune_count++;
			schedule_delayed_work(&bq->pe_volt_tune_work, 3 * HZ);
		}
	} else {
		if (pe.tune_up_volt)
			ret = bq2589x_pumpx_increase_volt_done(bq);
		else if (pe.tune_down_volt)
			ret = bq2589x_pumpx_decrease_volt_done(bq);

		if (ret == 0) {
			bq2589x_adjust_absolute_vindpm(bq);
			pumpx_cmd_issued = 0;
		}

		schedule_delayed_work(&bq->pe_volt_tune_work, HZ);
	}
}

static void bq2589x_monitor_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, monitor_work.work);
	u8 status = 0;
	int ret = 0;
	int rawfcc = 0, rawfv = 0, rawicl = 0;
	int batt_temp;
	union power_supply_propval propval = {0, };

	//bq2589x_dump_regs(bq);
	bq2589x_reset_watchdog_timer(bq);
	bq->rsoc = bq2589x_read_batt_rsoc(bq);
	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	bq->vbat_volt = bq2589x_adc_read_battery_volt(bq);
	bq->chg_current = bq2589x_adc_read_charge_current(bq);
	rawfcc = bq2589x_get_charge_current(bq);
	rawfv = bq2589x_get_chargevoltage(bq);
	rawicl = bq2589x_get_input_current_limit(bq);

	if (!bq->bms_psy)
		bq->bms_psy = power_supply_get_by_name("bms");
	if (bq->bms_psy) {
		ret = power_supply_get_property(bq->bms_psy, POWER_SUPPLY_PROP_TEMP, &propval);
		if (ret < 0) {
			bq_err("get battery temp fail\n");
		}
		batt_temp = propval.intval;
		bq_info("batt_temp: %d\n", (batt_temp / 10));
	}

	if (batt_temp < 0) {
		bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VRECHG_MASK, BQ2589X_VRECHG_200MV << BQ2589X_VRECHG_SHIFT);
	} else {
		bq2589x_update_bits(bq, BQ2589X_REG_06, BQ2589X_VRECHG_MASK, BQ2589X_VRECHG_100MV << BQ2589X_VRECHG_SHIFT);
	}

	ret = bq2589x_read_byte(bq, BQ2589X_REG_13, &status);
	if (ret == 0 && (status & BQ2589X_VDPM_STAT_MASK))
		bq_info("VINDPM occurred\n");
	if (ret == 0 && (status & BQ2589X_IDPM_STAT_MASK))
		bq_info("IINDPM occurred\n");

	if (bq->vbus_type == BQ2589X_VBUS_USB_DCP && bq->vbus_volt > pe.high_volt_level && bq->rsoc > 95 && !pe.tune_down_volt) {
		pe.tune_down_volt = true;
		pe.tune_up_volt = false;
		pe.target_volt = pe.low_volt_level;
		pe.tune_done = false;
		pe.tune_count = 0;
		pe.tune_fail = false;
		schedule_delayed_work(&bq->pe_volt_tune_work, 0);
	}

	switch (bq->vbus_type) {
	case BQ2589X_VBUS_MAXC:
		bq2589x_enable_ico(bq, false);
		if (rawicl != 2000 && rawicl != MAIN_ICL_MIN) {
			rerun_election(bq->usb_icl_votable);
		}
	case BQ2589X_VBUS_USB_DCP:
		//if (rawicl != 2000 && rawicl != MAIN_ICL_MIN) {
		if (rawicl > 2000) {
			rerun_election(bq->usb_icl_votable);
		}
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

	schedule_delayed_work(&bq->monitor_work, 10 * HZ);
}

static void bq2589x_start_charging_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, start_charging_work);
	int last_status = POWER_SUPPLY_STATUS_UNKNOWN;
	int status;
	bool stopped_fg_monitor = false;
	unsigned long timeout_jiffies;

	if (!bq)
		return;

	if (!bq->bms_psy)
		bq->bms_psy = power_supply_get_by_name("bms");
	if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");

	if (!bq->bms_psy || !bq->batt_psy)
		return;

	bq2589x_enable_charger(bq);

	if (bq->bms_psy) {
		stopped_fg_monitor = true;
		stop_fg_monitor_work(bq->bms_psy);
	}

	timeout_jiffies = jiffies + msecs_to_jiffies(10000);
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

	if (stopped_fg_monitor)
		start_fg_monitor_work(bq->bms_psy);
}

/*static int bq2589x_set_charger_type(struct bq2589x *bq, enum power_supply_type chg_type)
{
	int ret = 0;
	union power_supply_propval propval = {0, };

	if (bq->wall_psy == NULL) {
		bq->wall_psy = power_supply_get_by_name("bbc");
		if (bq->wall_psy == NULL) {
			bq_err("fail to get bbc_psy\n");
			return -ENODEV;
		}
	}

	if (bq->usb_psy == NULL) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (bq->usb_psy == NULL) {
			bq_err("fail to get usb_psy\n");
			return -ENODEV;
		}
	}

	if (chg_type != POWER_SUPPLY_TYPE_UNKNOWN)
		propval.intval = true;
	else
		propval.intval = false;

	ret = power_supply_set_property(bq->wall_psy, POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		bq_err("inform power supply chg_online failed: %d\n", ret);

	if (chg_type != POWER_SUPPLY_TYPE_UNKNOWN)
		propval.intval = true;
	else
		propval.intval = false;

	if (bq->pd_active && (propval.intval == false)) {
		//fix CtoC disconnection
	} else {
		ret = power_supply_set_property(bq->usb_psy, POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0)
			bq_err("inform power supply usb_online failed: %d\n", ret);
	}

	propval.intval = chg_type;
	ret = power_supply_set_property(bq->wall_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		bq_err("inform power supply charge type failed: %d\n", ret);

	propval.intval = chg_type;
	ret = power_supply_set_property(bq->usb_psy, POWER_SUPPLY_PROP_REAL_TYPE, &propval);
	if (ret < 0)
		bq_err("set prop REAL_TYPE fail: %d\n", ret);

	power_supply_changed(bq->usb_psy);
	return ret;
}*/

static int bq2589x_set_charger_type(struct bq2589x *bq, enum power_supply_type chg_type)
{
	int ret = 0;
	union power_supply_propval propval = {0, };

	if (chg_type != POWER_SUPPLY_TYPE_UNKNOWN) {
		WRITE_ONCE(bq->chg_online, true);
		propval.intval = true;
	} else {
		WRITE_ONCE(bq->chg_online, false);
		propval.intval = false;
	}
	WRITE_ONCE(bq->chg_type, chg_type);

	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy) {
			bq_err("fail to get usb_psy\n");
			return -ENODEV;
		}
	}

	if (bq->pd_active && !propval.intval) {
		//fix CtoC disconnection
	} else {
		ret = power_supply_set_property(bq->usb_psy, POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0)
			bq_err("inform power supply usb_online fail, ret=%d\n", ret);
	}

	bq_info("chg_type = %d\n", chg_type);
	propval.intval = chg_type;

	ret = power_supply_set_property(bq->usb_psy, POWER_SUPPLY_PROP_REAL_TYPE, &propval);
	if (ret < 0)
		bq_err("set prop REAL_TYPE fail, ret=%d\n", ret);

	power_supply_changed(bq->usb_psy);
	return ret;
}

static enum power_supply_type bq2589x_get_charger_type(struct bq2589x *bq)
{
	enum power_supply_type chg_type = POWER_SUPPLY_TYPE_UNKNOWN;

	switch (bq->vbus_type) {
	case BQ2589X_VBUS_NONE:
		bq_info("charger_type: NONE\n");
		chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case BQ2589X_VBUS_MAXC:
		bq_info("charger_type: HVDCP/Maxcharge\n");
		chg_type = POWER_SUPPLY_TYPE_USB_HVDCP;

		if (bq->part_no == SC89890H) {
//modify by HTH-223146 at 2022/06/23 begin
			//bq2589x_set_input_current_limit(bq, 2000);
//modify by HTH-223146 at 2022/06/23 end
			bq2589x_write_byte(bq, BQ2589X_REG_01, 0xC9);
		}
//modify by HTH-209427/HTH-209841/HTH-234945/HTH-234948 at 2022/06/08 begin
		/*if (bq->cfg.use_absolute_vindpm) {
			bq2589x_adjust_absolute_vindpm(bq);
		}*/
//modify by HTH-209427/HTH-209841/HTH-234945/HTH-234948 at 2022/06/08 end
		break;
	case BQ2589X_VBUS_USB_DCP:
		bq_info("charger_type: DCP\n");
		chg_type = POWER_SUPPLY_TYPE_USB_DCP;
//modify by HTH-223146 at 2022/06/23 begin
		/*if (bq->part_no == SC89890H) {
			bq2589x_set_input_current_limit(bq, 2000);
		}*/
//modify by HTH-223146 at 2022/06/23 begin
		break;
	case BQ2589X_VBUS_USB_CDP:
		bq_info("charger_type: CDP\n");
		chg_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case BQ2589X_VBUS_USB_SDP:
		bq_info("charger_type: SDP\n");
		chg_type = POWER_SUPPLY_TYPE_USB;
		break;
	case BQ2589X_VBUS_NONSTAND:
		bq_info("charger_type: FLOAT\n");
	case BQ2589X_VBUS_UNKNOWN:
		bq_info("charger_type: UNKNOWN\n");
		chg_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		break;
	case BQ2589X_VBUS_OTG:
		bq_info("charger_type: OTG\n");
		chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	default:
		bq_info("charger_type: Other, vbus_type is %d\n", bq->vbus_type);
		chg_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		break;
	}

	return chg_type;
}

static void bq2589x_charger_irq_workfunc(struct work_struct *work)
{
	//struct bq2589x *bq = container_of(work, struct bq2589x, irq_work.work);
	struct bq2589x *bq = container_of(to_delayed_work(work), struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	u8 vbus_status = 0;
	u8 charge_status = 0;
	u8 pg_status = 0;
	int ret;
	enum power_supply_type chg_type = POWER_SUPPLY_TYPE_UNKNOWN;
	//static int count = 0;
	//int prev_chg_type;

	//count++;
	//bq_debug("start count:%d\n", count);
	//bq_err("wsy irq_works bq2589x_usb_switch gpio_value=%d\n", gpio_get_value(bq->usb_switch1_gpio));

	bq_debug("irq_work running\n");

	mutex_lock(&bq->irq_complete);
	if (bq->shutting_down) {
		mutex_unlock(&bq->irq_complete);
		goto out;
	}

	atomic_set(&bq->irq_pending, 0);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		bq_debug("IRQ triggered before device-resume, masking IRQ\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(bq->irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		goto out;
	}
	bq->irq_waiting = false;
	mutex_unlock(&bq->irq_complete);

	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, BQ2589X_REG_0B, &status);
	if (ret)
		goto out;

	//bq2589x_dump_regs(bq);
	//prev_chg_type = bq->vbus_type;
	bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
	pg_status = (status & BQ2589X_PG_STAT_MASK) >> BQ2589X_PG_STAT_SHIFT;
	if (!pg_status)
		bq->vbus_type = BQ2589X_VBUS_NONE;

	// adapter in handle usb_switch
	/*if (bq->vbus_type == BQ2589X_VBUS_USB_CDP || bq->vbus_type == BQ2589X_VBUS_USB_SDP) {
		bq2589x_usb_switch(bq, false);
	}

	if ((bq->vbus_type == BQ2589X_VBUS_NONSTAND || bq->vbus_type == BQ2589X_VBUS_UNKNOWN)) {
		//bq2589x_usb_switch(bq, true);
		//bq2589x_force_dpdm_done(bq);
		//bq2589x_adc_start(bq, false);
		//goto out;
	}*/

	chg_type = bq2589x_get_charger_type(bq);
	bq2589x_set_charger_type(bq, chg_type);

	/*if (prev_chg_type == bq->vbus_type) {
		bq_err("prev_chg_type == new_chg_type\n");
		goto out;
	}*/

	ret = bq2589x_read_byte(bq, BQ2589X_REG_0C, &fault);
	if (ret)
		goto out;

	bq_info("status: %d, vbus_type: %d, chg_type: %d, fault: %d\n",
			status, bq->vbus_type, chg_type, fault);

	/*if (!bq->batt_psy)
		bq->batt_psy = power_supply_get_by_name("battery");*/
//modify by HTH-234945/HTH-234948 at 2022/06/08 begin
	if (bq->part_no == SC89890H) {
		ret = bq2589x_read_byte(bq, BQ2589X_REG_11, &vbus_status);
		if (ret)
			goto out;

		if (!(vbus_status & BQ2589X_VBUS_GD_MASK) && (bq->status & BQ2589X_STATUS_PLUGIN)) {
			bq2589x_usb_switch(bq, false);
			usleep_range(4500, 5500); // rest for power output
			bq2589x_usb_switch(bq, true);
			bq2589x_adc_stop(bq);
			bq->status &= ~BQ2589X_STATUS_PLUGIN;
			schedule_work(&bq->adapter_out_work);
			bq_info("adapter removed\n");
			schedule_delayed_work(&bq->charger_work, 0);
		} else if (bq->vbus_type != BQ2589X_VBUS_NONE && (bq->vbus_type != BQ2589X_VBUS_OTG) && !(bq->status & BQ2589X_STATUS_PLUGIN)) {
			bq->status |= BQ2589X_STATUS_PLUGIN;
			bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);
			bq2589x_adc_start(bq, false);

			if (bq->cfg.use_absolute_vindpm)
				bq2589x_adjust_absolute_vindpm(bq);

			bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENILIM_MASK,
					BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
			schedule_delayed_work(&bq->usb_changed_work, 0);
			schedule_work(&bq->adapter_in_work);
			bq_info("adapter plugged in\n");
			schedule_delayed_work(&bq->charger_work, 100);
			schedule_work(&bq->start_charging_work);
		}
	} else {
		if (((bq->vbus_type == BQ2589X_VBUS_NONE) || (bq->vbus_type == BQ2589X_VBUS_OTG)) && (bq->status & BQ2589X_STATUS_PLUGIN)) {
			bq2589x_usb_switch(bq, false);
			usleep_range(4500, 5500); // rest for power output
			bq2589x_usb_switch(bq, true);
			bq2589x_adc_stop(bq);
			bq->status &= ~BQ2589X_STATUS_PLUGIN;
			schedule_work(&bq->adapter_out_work);
			bq_info("adapter removed\n");
			schedule_delayed_work(&bq->charger_work, 0);
		} else if (bq->vbus_type != BQ2589X_VBUS_NONE && (bq->vbus_type != BQ2589X_VBUS_OTG) && !(bq->status & BQ2589X_STATUS_PLUGIN)) {
			bq->status |= BQ2589X_STATUS_PLUGIN;
			bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);
			bq2589x_adc_start(bq, false);

			if (bq->cfg.use_absolute_vindpm)
				bq2589x_adjust_absolute_vindpm(bq);

			bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENILIM_MASK,
					BQ2589X_ENILIM_DISABLE << BQ2589X_ENILIM_SHIFT);
			schedule_delayed_work(&bq->usb_changed_work, 0);
			schedule_work(&bq->adapter_in_work);
			bq_info("adapter plugged in\n");
			schedule_delayed_work(&bq->charger_work, 100);
			schedule_work(&bq->start_charging_work);
		}
	}
//modify by HTH-234945/HTH-234948 at 2022/06/08 end

	if ((status & BQ2589X_PG_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PG))
		bq->status |= BQ2589X_STATUS_PG;
	else if (!(status & BQ2589X_PG_STAT_MASK) && (bq->status & BQ2589X_STATUS_PG))
		bq->status &= ~BQ2589X_STATUS_PG;

	if (fault && !(bq->status & BQ2589X_STATUS_FAULT))
		bq->status |= BQ2589X_STATUS_FAULT;
	else if (!fault && (bq->status & BQ2589X_STATUS_FAULT))
		bq->status &= ~BQ2589X_STATUS_FAULT;

	charge_status = (status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	if (charge_status == BQ2589X_CHRG_STAT_IDLE) {
		bq_info("not charging\n");
	} else if (charge_status == BQ2589X_CHRG_STAT_PRECHG) {
		bq_info("precharging\n");
	} else if (charge_status == BQ2589X_CHRG_STAT_FASTCHG) {
		bq_info("fast charging\n");
	} else if (charge_status == BQ2589X_CHRG_STAT_CHGDONE) {
		bq_info("charge done!\n");
		if (bq->wall_psy) {
			power_supply_changed(bq->wall_psy);
		}
	}

	if (fault & 0x40) {
		bq2589x_usb_switch(bq, true);
		bq2589x_set_otg(bq, false);
		bq2589x_enable_charger(bq);
	}

	if (fault & 0x80) {
		bq2589x_reset_chip(bq);
		usleep_range(4500, 5500);
		bq2589x_init_device(bq);
	}

out:
	if (atomic_xchg(&bq->is_awake, 0)) {
		if (bq->bq_ws)
			__pm_relax(bq->bq_ws);
	}
}

static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	if (!bq)
		return IRQ_NONE;

	if (!atomic_xchg(&bq->is_awake, 1)) {
		if (bq->bq_ws)
			__pm_stay_awake(bq->bq_ws);
	}

	if (!atomic_xchg(&bq->irq_pending, 1))
		mod_delayed_work(bq->irq_wq, &bq->irq_work, 0);

	return IRQ_HANDLED;
}

static void determine_initial_status(struct bq2589x *bq)
{
	if (bq->irq)
		bq2589x_charger_interrupt(bq->irq, bq);
}

/*static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;
	//static int irq_count = 0;

	//irq_count++;
	//bq_info("irq_count: %d\n", irq_count);
	//schedule_work(&bq->irq_work);
	schedule_delayed_work(&bq->irq_work, msecs_to_jiffies(5));
	return IRQ_HANDLED;
}*/

#if defined(CONFIG_TCPC_RT1711H)
static void set_pd_active(struct bq2589x *bq, int pd_active)
{
	int rc = 0;
	union power_supply_propval val = {0, };

	if (!bq->usb_psy)
		bq->usb_psy = power_supply_get_by_name("usb");
	if (bq->usb_psy) {
		if (pd_active)
			bq2589x_set_charger_type(bq, POWER_SUPPLY_TYPE_USB_PD);
		else
			bq2589x_set_charger_type(bq, POWER_SUPPLY_TYPE_UNKNOWN);
		bq->pd_active = pd_active;
		val.intval = pd_active;
		rc = power_supply_set_property(bq->usb_psy, POWER_SUPPLY_PROP_PD_ACTIVE, &val);
		if (rc < 0)
			bq_err("couldn't set USB present status, rc=%d\n", rc);
		bq2589x_set_fast_charge_mode(bq, pd_active);
	}
}

static int get_source_mode(struct tcp_notify *noti)
{
	if (noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC || noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC)
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;

	switch (noti->typec_state.rp_level) {
	case TYPEC_CC_VOLT_SNK_1_5:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case TYPEC_CC_VOLT_SNK_3_0:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	case TYPEC_CC_VOLT_SNK_DFT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int bq2589x_set_cc_orientation(struct bq2589x *bq, int cc_orientation)
{
	int ret = 0;
	union power_supply_propval propval = {0, };

	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy) {
			bq_err("fail to get usb_psy\n");
			return -ENODEV;
		}
	}

	propval.intval = cc_orientation;
	ret = power_supply_set_property(bq->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION, &propval);
	if (ret < 0)
		bq_err("set prop CC_ORIENTATION fail: (%d)\n", ret);

	return ret;
}

static int bq2589x_set_typec_mode(struct bq2589x *bq, enum power_supply_typec_mode typec_mode)
{
	int ret = 0;
	union power_supply_propval propval = {0, };

	if (!bq->usb_psy) {
		bq->usb_psy = power_supply_get_by_name("usb");
		if (!bq->usb_psy) {
			bq_err("fail to get usb_psy\n");
			return -ENODEV;
		}
	}

	propval.intval = typec_mode;
	ret = power_supply_set_property(bq->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &propval);
	if (ret < 0)
		bq_err("set prop TYPEC_MODE fail: (%d)\n", ret);

	return ret;
}

static int pd_tcp_notifier_call(struct notifier_block *pnb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct bq2589x *bq = container_of(pnb, struct bq2589x, pd_nb);
	enum power_supply_typec_mode typec_mode = POWER_SUPPLY_TYPEC_NONE;
	int cc_orientation = 0;

	//bq_info("event:%d\n", event);
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
			//get_apdo_regain = 1;
			set_pd_active(bq, 2);
			//schedule_delayed_work(&bq->period_work, msecs_to_jiffies(5000));
			break;
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
				(noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
				noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
				noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC)) {
			bq_info("USB Plug in, pol = %d, state = %d\n",
					noti->typec_state.polarity, noti->typec_state.new_state);
			if (bq->vbus_type == BQ2589X_VBUS_NONE || bq->vbus_type == BQ2589X_VBUS_UNKNOWN) {
				bq_info("vbus_type is %d, force dpdm\n", bq->vbus_type);
				bq2589x_init_device(bq);
				bq2589x_usb_switch(bq, true);
				//bq2589x_force_dpdm_done(bq);
				// we can't access blocking contex here, use wq instead
				schedule_delayed_work(&bq->dpdm_work, 0);
			}
			typec_mode = get_source_mode(noti);
			cc_orientation = noti->typec_state.polarity;
			bq2589x_set_cc_orientation(bq, cc_orientation);
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
				noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			typec_mode = POWER_SUPPLY_TYPEC_SINK;
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
				noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
				noti->typec_state.old_state == TYPEC_ATTACHED_NORP_SRC ||
				noti->typec_state.old_state == TYPEC_ATTACHED_SRC) &&
				noti->typec_state.new_state == TYPEC_UNATTACHED) {
			typec_mode = POWER_SUPPLY_TYPEC_NONE;
			set_pd_active(bq, 0);
			bq2589x_usb_switch(bq, false);
			bq_info("USB Plug out\n");
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SRC &&
				noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			bq_info("Source_to_Sink\n");
			typec_mode = POWER_SUPPLY_TYPEC_SINK;
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SNK &&
				noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			typec_mode = get_source_mode(noti);
			bq_info("Sink_to_Source\n");
		}

		if (typec_mode >= POWER_SUPPLY_TYPEC_NONE && typec_mode <= POWER_SUPPLY_TYPEC_NON_COMPLIANT)
			bq2589x_set_typec_mode(bq, typec_mode);
		break;
	case TCP_NOTIFY_EXT_DISCHARGE:
		bq2589x_usb_switch(bq, true);
		bq_info("Ext USB Plug out\n");
		break;
	case TCP_NOTIFY_SOURCE_VBUS:
		bq_log("TCP_NOTIFY_SOURCE_VBUS\n");
		//2021.09.11 wsy crash this case
		if ((noti->vbus_state.mv == TCPC_VBUS_SOURCE_0V) && (vbus_on)) {
			/* disable OTG power output */
			bq_info("OTG Plug out\n");
			vbus_on = false;
			bq2589x_set_otg(bq, false);
			bq2589x_usb_switch(bq, true);
		} else if ((noti->vbus_state.mv == TCPC_VBUS_SOURCE_5V) && (!vbus_on)) {
			/* enable OTG power output */
			bq_info("OTG Plug in\n");
			vbus_on = true;
			bq2589x_usb_switch(bq, false);
			bq2589x_set_otg(bq, true);
			bq2589x_set_otg_current(bq, bq->cfg.charge_current_1500);
		}
		break;
	}

	return NOTIFY_OK;
}
#endif

static int fcc_vote_callback(struct votable *votable, void *data,
			int fcc_ua, const char *client)
{
	struct bq2589x *bq = data;
	int rc;

	if (fcc_ua < 0)
		return 0;

	if (fcc_ua > BQ2589X_MAX_FCC)
		fcc_ua = BQ2589X_MAX_FCC;

	rc = bq2589x_set_charge_current(bq, fcc_ua);
	if (rc < 0) {
		bq_err("failed to set charge current\n");
		return rc;
	}

	return 0;
}

static int chg_dis_vote_callback(struct votable *votable, void *data,
			int disable, const char *client)
{
	struct bq2589x *bq = data;
	int rc;

	if (disable) {
		rc = bq2589x_disable_charger(bq);
	} else {
		rc = bq2589x_enable_charger(bq);
	}

	if (rc < 0) {
		bq_err("failed to disable: (%d)\n", rc);
		return rc;
	}

	bq_info("disable: %d\n", disable);
	return 0;
}

static int fv_vote_callback(struct votable *votable, void *data,
			int fv_mv, const char *client)
{
	struct bq2589x *bq = data;
	int rc;

	if (fv_mv < 0)
		return 0;

	rc = bq2589x_set_chargevoltage(bq, fv_mv);
	if (rc < 0) {
		bq_err("failed to set charge voltage\n");
		return rc;
	}

	bq_info("fv: %d\n", fv_mv);
	return 0;
}

static int usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ma, const char *client)
{
	int rc;

	bq_info("iclb: %d\n", icl_ma);
	if (icl_ma < 0)
		return 0;

	if (icl_ma > BQ2589X_MAX_ICL)
		icl_ma = BQ2589X_MAX_ICL;

	rc = main_set_input_current_limit(icl_ma);
	if (rc < 0) {
		bq_err("failed to set input current limit\n");
		return rc;
	}

	return 0;
}

/*
static int chgctrl_vote_callback(struct votable *votable, void *data,
			int disable, const char *client)
{
	struct bq2589x *bq = data;
	int rc;

	bq_info("chgctrl_vote_callback chgctrl disable: %d\n", disable);
	if (disable)
		rc = bq2589x_disable_charger(bq);
	else
		rc = bq2589x_enable_charger(bq);

	return 0;
}*/ //for ovp lead reboot

/*static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int irqn;
	int ret;
	enum bq2589x_vbus_type vbus_type = BQ2589X_VBUS_UNKNOWN;

	pr_info("entry\n");
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		pr_err("out of memory!!\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->usb_switch_lock);
	mutex_init(&bq->dpdm_lock);
	i2c_set_clientdata(client, bq);

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25890) {
		bq->status |= BQ2589X_STATUS_EXIST;
		//bq->is_bq25890h = true;
		pr_info("charger device bq25890 detected, revision: (%d)\n", bq->revision);
	} else if (!ret && bq->part_no == SYV690) {
		bq->status |= BQ2589X_STATUS_EXIST;
		//bq->is_bq25890h = false;
		pr_info("charger device SYV690 detected, revision: (%d)\n", bq->revision);
		nopmi_set_charger_ic_type(NOPMI_CHARGER_IC_SYV);
	} else if (!ret && bq->part_no == SC89890H) {
		bq->status |= BQ2589X_STATUS_EXIST;
		//bq->is_bq25890h = false;
		pr_info("charger device SC89890H detected, revision: (%d)\n", bq->revision);
	} else {
		pr_err("no bq25890 charger device found: (%d)\n", ret);
		ret = -ENODEV;
		goto err_free;
	}

#if defined(CONFIG_TCPC_RT1711H)
	bq->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!bq->tcpc_dev) {
		pr_err("tcpc device not ready, defer\n");
		ret = -EPROBE_DEFER;
		goto err_free;
	}
#endif
	bq->usb_psy = power_supply_get_by_name("usb");
	bq->batt_psy = power_supply_get_by_name("battery");
	bq->bms_psy = power_supply_get_by_name("bms");

	if (client->dev.of_node)
		bq2589x_parse_dt(&client->dev, bq);

	ret = bq2589x_init_device(bq);
	if (ret) {
		pr_err("device init failure: (%d)\n", ret);
		goto err_init;
	}

	ret = gpio_request(bq->irq_gpio, "bq2589x irq pin");
	if (ret) {
		pr_err("%d gpio request failed\n", bq->irq_gpio);
		goto err_gpio;
	}

	irqn = gpio_to_irq(bq->irq_gpio);
	if (irqn < 0) {
		pr_err("%d gpio_to_irq failed\n", irqn);
		ret = irqn;
		goto err_gpio;
	}
	client->irq = irqn;

	ret = bq2589x_psy_register(bq);
	if (ret)
		goto err_psy;

	g_bq = bq;
	bq->is_awake = false;

	INIT_WORK(&bq->adapter_in_work, bq2589x_adapter_in_workfunc);
	INIT_WORK(&bq->adapter_out_work, bq2589x_adapter_out_workfunc);
	INIT_WORK(&bq->start_charging_work, bq2589x_start_charging_workfunc);
	INIT_DELAYED_WORK(&bq->irq_work, bq2589x_charger_irq_workfunc);
	INIT_DELAYED_WORK(&bq->monitor_work, bq2589x_monitor_workfunc);
	INIT_DELAYED_WORK(&bq->ico_work, bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->charger_work, bq2589x_charger_workfunc);
	INIT_DELAYED_WORK(&bq->pe_volt_tune_work, bq2589x_tune_volt_workfunc);
	INIT_DELAYED_WORK(&bq->usb_changed_work, bq2589x_usb_changed_workfunc);
	INIT_DELAYED_WORK(&bq->check_pe_tuneup_work, bq2589x_check_pe_tuneup_workfunc);
	INIT_DELAYED_WORK(&bq->time_delay_work, time_delay_work);
	INIT_DELAYED_WORK(&bq->dpdm_work, bq2589x_dpdm_work);
	//INIT_DELAYED_WORK(&bq->period_work, bq2589x_period_workfunc);

	bq->fcc_votable = create_votable("FCC", VOTE_MIN, fcc_vote_callback, bq);
	if (IS_ERR(bq->fcc_votable)) {
		ret = PTR_ERR(bq->fcc_votable);
		bq->fcc_votable = NULL;
		goto destroy_votable;
	}

	bq->chg_dis_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY, chg_dis_vote_callback, bq);
	if (IS_ERR(bq->chg_dis_votable)) {
		ret = PTR_ERR(bq->chg_dis_votable);
		bq->chg_dis_votable = NULL;
		goto destroy_votable;
	}

	bq->fv_votable = create_votable("FV", VOTE_MIN, fv_vote_callback, bq);
	if (IS_ERR(bq->fv_votable)) {
		ret = PTR_ERR(bq->fv_votable);
		bq->fv_votable = NULL;
		goto destroy_votable;
	}

	bq->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN, usb_icl_vote_callback, bq);
	if (IS_ERR(bq->usb_icl_votable)) {
		ret = PTR_ERR(bq->usb_icl_votable);
		bq->usb_icl_votable = NULL;
		goto destroy_votable;
	}

#if 0
	bq->chgctrl_votable = create_votable("CHG_CTRL", VOTE_MIN, chgctrl_vote_callback, bq);
	if (IS_ERR(bq->chgctrl_votable)) {
		ret = PTR_ERR(bq->chgctrl_votable);
		bq->chgctrl_votable = NULL;
		goto destroy_votable;
	}
#endif //for ovp lead reboot

	vote(bq->fcc_votable, MAIN_SET_VOTER, true, 500);
	vote(bq->fcc_votable, PROFILE_CHG_VOTER, true, CHG_FCC_CURR_MAX);
	vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, CHG_ICL_CURR_MAX);
	vote(bq->chg_dis_votable, "BMS_FC_VOTER", false, 0);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		pr_err("failed to register sysfs: (%d)\n", ret);
		goto err_sysfs;
	}

//modify by HTH-209427/HTH-209841 at 2022/05/12 begin
	pe.enable = false; //PE adjuested to the front of the interrupt
//modify by HTH-209427/HTH-209841 at 2022/05/12 end
	ret = request_irq(client->irq, bq2589x_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "bq2589x_charger1_irq", bq);
	if (ret) {
		pr_err("request IRQ %d failed: (%d)\n", client->irq, ret);
		goto err_irq;
	} else {
		pr_info("IRQ interrupt: (%d)\n", client->irq);
	}
	//schedule_work(&bq->irq_work); // 2020.09.15 change for zsa in case of adapter has been in when power off

#if defined(CONFIG_TCPC_RT1711H)
	bq->pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(bq->tcpc_dev, &bq->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		pr_err("register tcpc notifer fail\n");
		ret = -EINVAL;
		goto err_get_tcpc_dev;
	}
#endif

	enable_irq_wake(irqn);
	bq2589x_dump_regs(bq);

	vbus_type = bq2589x_get_vbus_type(bq);
	if (vbus_type != BQ2589X_VBUS_OTG) {
		bq2589x_usb_switch(bq, true);
		vbus_type = bq2589x_get_vbus_valid(bq);
	}

	if (vbus_type == BQ2589X_VBUS_UNKNOWN || vbus_type == BQ2589X_VBUS_NONE) {
		pr_info("retry bc12 delay 4s!\n");
		schedule_delayed_work(&bq->time_delay_work, msecs_to_jiffies(4000));
	}
	bq->vbus_type = vbus_type;

	pr_info("success\n");
	return 0;

#if defined(CONFIG_TCPC_RT1711H)
err_get_tcpc_dev:
	disable_irq(bq->client->irq);
	free_irq(bq->client->irq, bq);
#endif
err_irq:
	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
err_sysfs:
destroy_votable:
	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_work_sync(&bq->start_charging_work);
	cancel_delayed_work_sync(&bq->irq_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->charger_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->usb_changed_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	cancel_delayed_work_sync(&bq->time_delay_work);
	cancel_delayed_work_sync(&bq->dpdm_work);
	//cancel_delayed_work_sync(&bq->period_work);

	//if (!IS_ERR_OR_NULL(bq->chgctrl_votable))
		//destroy_votable(bq->chgctrl_votable);
	if (!IS_ERR_OR_NULL(bq->usb_icl_votable))
		destroy_votable(bq->usb_icl_votable);
	if (!IS_ERR_OR_NULL(bq->fv_votable))
		destroy_votable(bq->fv_votable);
	if (!IS_ERR_OR_NULL(bq->chg_dis_votable))
		destroy_votable(bq->chg_dis_votable);
	if (!IS_ERR_OR_NULL(bq->fcc_votable))
		destroy_votable(bq->fcc_votable);

	bq2589x_psy_unregister(bq);
err_psy:
	gpio_free(bq->irq_gpio);
err_gpio:
err_init:
	if (bq->bms_psy)
		power_supply_put(bq->bms_psy);
	if (bq->batt_psy)
		power_supply_put(bq->batt_psy);
	if (bq->usb_psy)
		power_supply_put(bq->usb_psy);
err_free:
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->usb_switch_lock);
	mutex_destroy(&bq->dpdm_lock);
	g_bq = NULL;
	//devm_kfree(&client->dev, bq);
	pr_err("fail!!\n");
	return ret;
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	pr_info("entry\n");
	//bq2589x_disable_otg(bq);
	bq2589x_set_otg(bq, false);
	bq2589x_exit_hiz_mode(bq);
	bq2589x_adc_stop(bq);
	bq2589x_psy_unregister(bq);
	msleep(5);

	if (bq->client->irq) {
		disable_irq_wake(bq->client->irq);
		disable_irq(bq->client->irq);
		bq->client->irq = 0;
	}
	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);

	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_work_sync(&bq->start_charging_work);
	cancel_delayed_work_sync(&bq->irq_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->charger_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->usb_changed_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	cancel_delayed_work_sync(&bq->time_delay_work);
	cancel_delayed_work_sync(&bq->dpdm_work);
	// cancel_delayed_work_sync(&bq->period_work);

	// legacy think, will be remove later
	if (bq->usb_switch1_requested && bq->usb_switch1_gpio >= 0) {
		gpio_free(bq->usb_switch1_gpio);
		bq->usb_switch1_requested = false;
		bq->usb_switch1_gpio = -1;
	}
	if (bq->irq_requested && bq->irq_gpio >= 0) {
		gpio_free(bq->irq_gpio);
		bq->irq_requested = false;
		bq->irq_gpio = -1;
	}

	// g_bq = NULL;
}*/

static void bq2589x_wq_destroy(void *data)
{
	struct workqueue_struct *wq = data;
	destroy_workqueue(wq);
}

static int bq2589x_charger_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int irqn;
	int ret;

	bq_info("entry\n");
	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		bq_err("out of memory!!\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->usb_switch_lock);
	mutex_init(&bq->dpdm_lock);
	mutex_init(&bq->irq_complete);
	spin_lock_init(&bq->usb_switch_lock_spin);
	i2c_set_clientdata(client, bq);

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25890) {
		bq->status |= BQ2589X_STATUS_EXIST;
		bq_info("charger device bq25890 detected, revision: (%d)\n", bq->revision);
	} else if (!ret && bq->part_no == SYV690) {
		bq->status |= BQ2589X_STATUS_EXIST;
		bq_info("charger device SYV690 detected, revision: (%d)\n", bq->revision);
		nopmi_set_charger_ic_type(NOPMI_CHARGER_IC_SYV);
	} else if (!ret && bq->part_no == SC89890H) {
		bq->status |= BQ2589X_STATUS_EXIST;
		bq_info("charger device SC89890H detected, revision: (%d)\n", bq->revision);
	} else {
		bq_err("no bq25890 charger device found: (%d)\n", ret);
		ret = -ENODEV;
		goto err_free;
	}

#if defined(CONFIG_TCPC_RT1711H)
	bq->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!bq->tcpc_dev) {
		bq_err("tcpc device not ready, defer\n");
		ret = -EPROBE_DEFER;
		goto err_free;
	}
#endif
	bq->usb_psy = power_supply_get_by_name("usb");
	bq->batt_psy = power_supply_get_by_name("battery");
	bq->bms_psy = power_supply_get_by_name("bms");

	atomic_set(&bq->irq_pending, 0);
	atomic_set(&bq->is_awake, 0);
	bq->irq_gpiod = NULL;
	bq->usb_switch1_gpiod = NULL;
	bq->irq = 0;
	bq->usb_switch_flag = 0;
	bq->resume_completed = true;
	bq->irq_disabled = false;
	bq->irq_waiting = false;
	bq->shutting_down = false;
	g_bq = bq;

	if (client->dev.of_node) {
		ret = bq2589x_parse_dt(&client->dev, bq);
		if (ret) {
			bq_err("DT parse failed: %d\n", ret);
			goto err_free;
		}
	}

	ret = bq2589x_init_device(bq);
	if (ret) {
		bq_err("device init failure: (%d)\n", ret);
		goto err_init;
	}

	if (!bq->irq_gpiod) {
		bq_err("irq descriptor unavailable\n");
		ret = -EINVAL;
		goto err_gpio;
	}

	irqn = gpiod_to_irq(bq->irq_gpiod);
	if (irqn < 0) {
		bq_err("gpiod_to_irq failed: %d\n", irqn);
		ret = irqn;
		goto err_gpio;
	}

	ret = bq2589x_psy_register(bq);
	if (ret) {
		bq_err("psy_register failed\n");
		goto err_psy;
	}

	bq->bq_ws = wakeup_source_register(bq->dev, "bq2589x_ws");
	if (!bq->bq_ws) {
		bq_err("wakeup_source_register failed\n");
		ret = -ENOMEM;
		goto err_psy;
	}

	INIT_WORK(&bq->adapter_in_work, bq2589x_adapter_in_workfunc);
	INIT_WORK(&bq->adapter_out_work, bq2589x_adapter_out_workfunc);
	INIT_WORK(&bq->start_charging_work, bq2589x_start_charging_workfunc);
	INIT_DELAYED_WORK(&bq->irq_work, bq2589x_charger_irq_workfunc);
	INIT_DELAYED_WORK(&bq->monitor_work, bq2589x_monitor_workfunc);
	INIT_DELAYED_WORK(&bq->ico_work, bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->charger_work, bq2589x_charger_workfunc);
	INIT_DELAYED_WORK(&bq->pe_volt_tune_work, bq2589x_tune_volt_workfunc);
	INIT_DELAYED_WORK(&bq->usb_changed_work, bq2589x_usb_changed_workfunc);
	INIT_DELAYED_WORK(&bq->check_pe_tuneup_work, bq2589x_check_pe_tuneup_workfunc);
	INIT_DELAYED_WORK(&bq->time_delay_work, time_delay_work);
	INIT_DELAYED_WORK(&bq->dpdm_work, bq2589x_dpdm_work);

	bq->irq_wq = create_singlethread_workqueue("bq2589x-irq-wq");
	if (!bq->irq_wq) {
		bq_err("failed to create irq workqueue\n");
		ret = -ENOMEM;
		goto err_ws;
	}

	ret = devm_add_action_or_reset(bq->dev, bq2589x_wq_destroy, bq->irq_wq);
	if (ret)
		goto err_ws;

	bq->fcc_votable = create_votable("FCC", VOTE_MIN, fcc_vote_callback, bq);
	if (IS_ERR(bq->fcc_votable)) {
		ret = PTR_ERR(bq->fcc_votable);
		bq->fcc_votable = NULL;
		goto destroy_votable;
	}

	bq->chg_dis_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY, chg_dis_vote_callback, bq);
	if (IS_ERR(bq->chg_dis_votable)) {
		ret = PTR_ERR(bq->chg_dis_votable);
		bq->chg_dis_votable = NULL;
		goto destroy_votable;
	}

	bq->fv_votable = create_votable("FV", VOTE_MIN, fv_vote_callback, bq);
	if (IS_ERR(bq->fv_votable)) {
		ret = PTR_ERR(bq->fv_votable);
		bq->fv_votable = NULL;
		goto destroy_votable;
	}

	bq->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN, usb_icl_vote_callback, bq);
	if (IS_ERR(bq->usb_icl_votable)) {
		ret = PTR_ERR(bq->usb_icl_votable);
		bq->usb_icl_votable = NULL;
		goto destroy_votable;
	}

	vote(bq->fcc_votable, MAIN_SET_VOTER, true, 500);
	vote(bq->fcc_votable, PROFILE_CHG_VOTER, true, CHG_FCC_CURR_MAX);
	vote(bq->usb_icl_votable, PROFILE_CHG_VOTER, true, CHG_ICL_CURR_MAX);
	vote(bq->chg_dis_votable, "BMS_FC_VOTER", false, 0);

	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		bq_err("failed to register sysfs: (%d)\n", ret);
		goto err_sysfs;
	}

	pe.enable = false;
	ret = devm_request_threaded_irq(bq->dev, irqn,
				NULL, bq2589x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2589x_charger1_irq", bq);
	if (ret) {
		bq_err("request_threaded_irq failed for IRQ %d: %d\n", irqn, ret);
		goto err_irq;
	}
	bq->irq = irqn;

#if defined(CONFIG_TCPC_RT1711H)
	bq->pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(bq->tcpc_dev, &bq->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		bq_err("register tcpc notifer fail\n");
		ret = -EINVAL;
		goto err_get_tcpc_dev;
	}
#endif

	enable_irq_wake(bq->irq);
	determine_initial_status(bq);
	usleep_range(4500, 5500);

	/* usb_switch1: use descriptor API only */
	if (bq->usb_switch1_gpiod) {
		bq->usb_switch_flag = !!gpiod_get_value_cansleep(bq->usb_switch1_gpiod);
	} else {
		bq->usb_switch_flag = 0;
	}
	bq_info("initial usb_switch_flag: %d\n", bq->usb_switch_flag);

	schedule_delayed_work(&bq->time_delay_work, msecs_to_jiffies(4000));

	bq2589x_dump_regs(bq);
	bq_info("success\n");
	return 0;

/* error unwind: only undo what was created up to that point */
#if defined(CONFIG_TCPC_RT1711H)
err_get_tcpc_dev:
	if (bq->irq)
		bq->irq = 0;
#endif
err_irq:
	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
err_sysfs:
destroy_votable:
	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_work_sync(&bq->start_charging_work);
	cancel_delayed_work_sync(&bq->irq_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->charger_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->usb_changed_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	cancel_delayed_work_sync(&bq->time_delay_work);
	cancel_delayed_work_sync(&bq->dpdm_work);

	if (!IS_ERR_OR_NULL(bq->usb_icl_votable))
		destroy_votable(bq->usb_icl_votable);
	if (!IS_ERR_OR_NULL(bq->fv_votable))
		destroy_votable(bq->fv_votable);
	if (!IS_ERR_OR_NULL(bq->chg_dis_votable))
		destroy_votable(bq->chg_dis_votable);
	if (!IS_ERR_OR_NULL(bq->fcc_votable))
		destroy_votable(bq->fcc_votable);

	bq2589x_psy_unregister(bq);
err_psy:
err_gpio:
err_ws:
	if (bq->bq_ws) {
		wakeup_source_unregister(bq->bq_ws);
		bq->bq_ws = NULL;
	}
err_init:
	if (bq->bms_psy)
		power_supply_put(bq->bms_psy);
	if (bq->batt_psy)
		power_supply_put(bq->batt_psy);
	if (bq->usb_psy)
		power_supply_put(bq->usb_psy);
err_free:
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->usb_switch_lock);
	mutex_destroy(&bq->dpdm_lock);
	mutex_destroy(&bq->irq_complete);
	i2c_set_clientdata(client, NULL);
	g_bq = NULL;

	bq_err("fail! (%d)\n", ret);
	return ret;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return 0;

	bq_info("entry\n");

	bq2589x_set_otg(bq, false);
	bq2589x_exit_hiz_mode(bq);
	bq2589x_adc_stop(bq);
	usleep_range(4500, 5500);

	if (bq->irq) {
		disable_irq_wake(bq->irq);
		disable_irq(bq->irq);
	}

#if defined(CONFIG_TCPC_RT1711H)
	if (bq->tcpc_dev)
		unregister_tcp_dev_notifier(bq->tcpc_dev, &bq->pd_nb, TCP_NOTIFY_TYPE_ALL);
#endif

	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_work_sync(&bq->start_charging_work);
	cancel_delayed_work_sync(&bq->irq_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->charger_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	cancel_delayed_work_sync(&bq->usb_changed_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->time_delay_work);
	cancel_delayed_work_sync(&bq->dpdm_work);

	if (bq->irq_wq)
		flush_workqueue(bq->irq_wq);

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);

	if (bq->bq_ws) {
		wakeup_source_unregister(bq->bq_ws);
		bq->bq_ws = NULL;
	}

	if (!IS_ERR_OR_NULL(bq->usb_icl_votable))
		destroy_votable(bq->usb_icl_votable);
	if (!IS_ERR_OR_NULL(bq->fv_votable))
		destroy_votable(bq->fv_votable);
	if (!IS_ERR_OR_NULL(bq->chg_dis_votable))
		destroy_votable(bq->chg_dis_votable);
	if (!IS_ERR_OR_NULL(bq->fcc_votable))
		destroy_votable(bq->fcc_votable);

	bq2589x_psy_unregister(bq);
	if (bq->bms_psy)
		power_supply_put(bq->bms_psy);
	if (bq->batt_psy)
		power_supply_put(bq->batt_psy);
	if (bq->usb_psy)
		power_supply_put(bq->usb_psy);

	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->usb_switch_lock);
	mutex_destroy(&bq->dpdm_lock);
	i2c_set_clientdata(client, NULL);
	g_bq = NULL;

	bq_info("success\n");
	return 0;
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return;

	bq_info("entry\n");

	mutex_lock(&bq->irq_complete);
	bq->shutting_down = true;
	mutex_unlock(&bq->irq_complete);

	bq2589x_set_otg(bq, false);
	bq2589x_exit_hiz_mode(bq);
	bq2589x_adc_stop(bq);
	usleep_range(4500, 5500);

	if (bq->irq) {
		disable_irq_wake(bq->irq);
		disable_irq(bq->irq);
	}

	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_work_sync(&bq->start_charging_work);
	cancel_delayed_work_sync(&bq->irq_work);
	cancel_delayed_work_sync(&bq->monitor_work);
	cancel_delayed_work_sync(&bq->ico_work);
	cancel_delayed_work_sync(&bq->charger_work);
	cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	cancel_delayed_work_sync(&bq->usb_changed_work);
	cancel_delayed_work_sync(&bq->check_pe_tuneup_work);
	cancel_delayed_work_sync(&bq->time_delay_work);
	cancel_delayed_work_sync(&bq->dpdm_work);

	if (bq->irq_wq)
		flush_workqueue(bq->irq_wq);

	bq_info("success\n");
}

static int bq2589x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return 0;

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	bq_info("Suspend successfully!\n");
	return 0;
}

static int bq2589x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return 0;

	mutex_lock(&bq->irq_complete);
	if (bq->irq_waiting || atomic_read(&bq->irq_pending)) {
		bq_err("Aborting suspend, an interrupt was detected while suspending\n");
		mutex_unlock(&bq->irq_complete);
		return -EBUSY;
	}
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq2589x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (!bq)
		return 0;

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_disabled) {
		enable_irq(bq->irq);
		bq->irq_disabled = false;
	}

	atomic_set(&bq->irq_pending, 0);
	if (bq->irq_waiting)
		mod_delayed_work(bq->irq_wq, &bq->irq_work, 0);
	mutex_unlock(&bq->irq_complete);

	if (bq->wall_psy)
		power_supply_changed(bq->wall_psy);

	bq_info("Resume successfully!\n");
	return 0;
}

static const struct dev_pm_ops bq2589x_pm_ops = {
	.suspend = bq2589x_suspend,
	.suspend_noirq = bq2589x_suspend_noirq,
	.resume = bq2589x_resume,
};

static struct of_device_id bq2589x_charger_match_table[] = {
	{.compatible = "ti,bq2589x-1",},
	{},
};

static const struct i2c_device_id bq2589x_charger_id[] = {
	{ "bq2589x-1", BQ25890 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger_id);

static struct i2c_driver bq2589x_charger_driver = {
	.driver		= {
		.name	= "bq2589x-1",
		.owner	= THIS_MODULE,
		.of_match_table = bq2589x_charger_match_table,
		.pm = &bq2589x_pm_ops,
	},
	.id_table	= bq2589x_charger_id,
	.probe		= bq2589x_charger_probe,
	.remove	= bq2589x_charger_remove,
	.shutdown	= bq2589x_charger_shutdown,
};

module_i2c_driver(bq2589x_charger_driver);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
