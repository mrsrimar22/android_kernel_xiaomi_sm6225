// SPDX-License-Identifier: GPL-2.0-only
/*
 * SC8551 battery charging driver
 */

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
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_irq.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <asm/unaligned.h>
#include "sc8551_reg.h"

enum sc8551_adc_channel {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VAC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBUS,
	ADC_TBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

#define SC8551_ROLE_STANDALONE		0
#define SC8551_ROLE_SLAVE		1
#define SC8551_ROLE_MASTER		2

enum {
	SC_NONE = 0,
	SC8551 = 8551,
	SC_MAX = 0xFFFF,
};

enum {
	SC8551_STANDALONE,
	SC8551_SLAVE,
	SC8551_MASTER,
};

static int sc8551_mode_data[] = {
	[SC8551_STANDALONE]	= SC8551_ROLE_STANDALONE,
	[SC8551_MASTER]		= SC8551_ROLE_MASTER,
	[SC8551_SLAVE]		= SC8551_ROLE_SLAVE,
};

#define SC_RL_SLOTS	16
#define SC_RL_INTERVAL	(5 * HZ)
#define SC_RL_BURST	1

#ifdef __COUNTER__
#define _SC_ID()	__COUNTER__
#else
#define _SC_ID()	__LINE__
#endif

struct sc8551_cfg {
	bool	bat_ovp_disable;
	bool	bat_ocp_disable;
	bool	bat_ovp_alm_disable;
	bool	bat_ocp_alm_disable;

	int	bat_ovp_th;
	int	bat_ovp_alm_th;
	int	bat_ocp_th;
	int	bat_ocp_alm_th;

	bool	bus_ovp_alm_disable;
	bool	bus_ocp_disable;
	bool	bus_ocp_alm_disable;

	int	bus_ovp_th;
	int	bus_ovp_alm_th;
	int	bus_ocp_th;
	int	bus_ocp_alm_th;

	bool	bat_ucp_alm_disable;

	int	bat_ucp_alm_th;
	int	ac_ovp_th;

	bool	bat_therm_disable;
	bool	bus_therm_disable;
	bool	die_therm_disable;

	int	bat_therm_th; /* in % */
	int	bus_therm_th; /* in % */
	int	die_therm_th; /* in degC */

	int	sense_r_mohm;
};

struct sc8551 {
	struct device		*dev;
	struct i2c_client	*client;
	struct regmap		*regmap;

	int			part_no;
	int			revision;
	int			chip_vendor;

	int			mode;

	struct mutex		data_lock;

	bool			batt_present;
	bool			vbus_present;

	bool			usb_present;
	bool			charge_enabled;

	int			vbus_error;

	/* ADC reading */
	int			vbat_volt;
	int			vbus_volt;

	int			ibat_curr;
	int			ibus_curr;

	int			bat_temp;
	int			bus_temp;
	int			die_temp;

	/* alarm/fault status */
	bool			bat_ovp_fault;
	bool			bat_ocp_fault;
	bool			bus_ovp_fault;
	bool			bus_ocp_fault;

	bool			bat_ovp_alarm;
	bool			bat_ocp_alarm;
	bool			bus_ovp_alarm;
	bool			bus_ocp_alarm;

	bool			bat_ucp_alarm;

	bool			bat_therm_alarm;
	bool			bus_therm_alarm;
	bool			die_therm_alarm;

	bool			bat_therm_fault;
	bool			bus_therm_fault;
	bool			die_therm_fault;

	int			prev_alarm;
	int			prev_fault;

	struct sc8551_cfg	cfg;

	int			skip_writes;
	int			skip_reads;

	struct delayed_work	monitor_work;
	struct gpio_desc	*irq_gpiod;
	int			irq;

	struct dentry		*debug_root;

	struct power_supply_desc	psy_desc;
	struct power_supply_config	psy_cfg;
	struct power_supply		*fc2_psy;

	struct wakeup_source	*sc_ws;
	struct ratelimit_state	rl_slots[SC_RL_SLOTS];
};

static inline const char *sc8551_role_str(int mode)
{
	switch (mode) {
	case SC8551_ROLE_MASTER:
		return "MASTER";
	case SC8551_ROLE_SLAVE:
		return "SLAVE";
	default:
		return "STANDALONE";
	}
}

#define sc_log(fmt, ...)						\
do {									\
	if (sc && __ratelimit(&sc->rl_slots[_SC_ID() &			\
					    (SC_RL_SLOTS - 1)]))	\
		pr_info("[sc8551-%s]: %s: " fmt,			\
			sc8551_role_str(sc->mode), __func__,		\
			##__VA_ARGS__);					\
} while (0)

#define sc_err(fmt, ...)						\
	pr_err("[sc8551-%s]: %s: " fmt, sc8551_role_str(sc->mode),	\
	       __func__, ##__VA_ARGS__)

#define sc_info(fmt, ...)						\
	pr_info("[sc8551-%s]: %s: " fmt, sc8551_role_str(sc->mode),	\
		__func__, ##__VA_ARGS__)

#define sc_dbg(fmt, ...)						\
	pr_debug("[sc8551-%s]: %s: " fmt, sc8551_role_str(sc->mode),	\
		 __func__, ##__VA_ARGS__)

static const struct regmap_config sc8551_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SC8551_REG_35,
};

static int sc8551_read_byte(struct sc8551 *sc, u8 reg, u8 *data)
{
	int ret;
	unsigned int val;

	if (sc->skip_reads) {
		*data = 0;
		return 0;
	}

	ret = regmap_read(sc->regmap, reg, &val);
	if (ret < 0) {
		sc_err("read 0x%02x failed: %d\n", reg, ret);
		*data = 0;
		return ret;
	}

	*data = (u8)val;
	return 0;
}

static int sc8551_write_byte(struct sc8551 *sc, u8 reg, u8 data)
{
	int ret;

	if (sc->skip_writes)
		return 0;

	ret = regmap_write(sc->regmap, reg, data);
	if (ret < 0)
		sc_err("write 0x%02x->0x%02x failed: %d\n", data, reg, ret);

	return ret;
}

static int sc8551_update_bits(struct sc8551 *sc, u8 reg, u8 mask, u8 data)
{
	int ret;

	if (sc->skip_reads || sc->skip_writes)
		return 0;

	ret = regmap_update_bits(sc->regmap, reg, mask, data);
	if (ret < 0)
		sc_err("update_bits 0x%02x (mask=0x%02x val=0x%02x) failed: %d\n",
		       reg, mask, data, ret);

	return ret;
}

static int sc8551_enable_charge(struct sc8551 *sc, bool enable)
{
	int ret;
	u8 val;

	val = enable ? SC8551_CHG_ENABLE : SC8551_CHG_DISABLE;
	val <<= SC8551_CHG_EN_SHIFT;

	ret = sc8551_update_bits(sc, SC8551_REG_0C, SC8551_CHG_EN_MASK, val);
	if (!ret)
		sc_log("charge: %s\n", enable ? "enabled" : "disabled");
	else
		sc_err("failed to %s charge: ret=%d\n",
		       enable ? "enable" : "disable", ret);

	return ret;
}

static int sc8551_check_charge_enabled(struct sc8551 *sc, bool *enabled)
{
	int ret;
	u8 val;

	ret = sc8551_read_byte(sc, SC8551_REG_0C, &val);
	if (!ret) {
		*enabled = !!(val & SC8551_CHG_EN_MASK);
		sc_log("charge status: %s\n", *enabled ? "enabled" : "disabled");
	} else {
		sc_err("failed to read charger status, ret=%d\n", ret);
	}

	return ret;
}

static int sc8551_enable_wdt(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_WATCHDOG_ENABLE : SC8551_WATCHDOG_DISABLE;
	val <<= SC8551_WATCHDOG_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_0B, SC8551_WATCHDOG_DIS_MASK, val);
}

static __maybe_unused int sc8551_set_wdt(struct sc8551 *sc, int ms)
{
	u8 val;

	if (ms == 500)
		val = SC8551_WATCHDOG_0P5S;
	else if (ms == 1000)
		val = SC8551_WATCHDOG_1S;
	else if (ms == 5000)
		val = SC8551_WATCHDOG_5S;
	else
		val = SC8551_WATCHDOG_30S;

	val <<= SC8551_WATCHDOG_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_0B, SC8551_WATCHDOG_MASK, val);
}

static int sc8551_set_reg_reset(struct sc8551 *sc)
{
	u8 val;

	sc_info("mode: %d, reg reset\n", sc->mode);

	val = SC8551_REG_RST_ENABLE;
	val <<= SC8551_REG_RST_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_0B, SC8551_REG_RST_MASK, val);
}

static int sc8551_enable_batovp(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BAT_OVP_ENABLE : SC8551_BAT_OVP_DISABLE;
	val <<= SC8551_BAT_OVP_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_00, SC8551_BAT_OVP_DIS_MASK, val);
}

static int sc8551_set_batovp_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BAT_OVP_BASE)
		threshold = SC8551_BAT_OVP_BASE;

	val = (threshold - SC8551_BAT_OVP_BASE) / SC8551_BAT_OVP_LSB;
	val <<= SC8551_BAT_OVP_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_00, SC8551_BAT_OVP_MASK, val);
}

static int sc8551_enable_batovp_alarm(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BAT_OVP_ALM_ENABLE : SC8551_BAT_OVP_ALM_DISABLE;
	val <<= SC8551_BAT_OVP_ALM_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_01, SC8551_BAT_OVP_ALM_DIS_MASK, val);
}

static int sc8551_set_batovp_alarm_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BAT_OVP_ALM_BASE)
		threshold = SC8551_BAT_OVP_ALM_BASE;

	val = (threshold - SC8551_BAT_OVP_ALM_BASE) / SC8551_BAT_OVP_ALM_LSB;
	val <<= SC8551_BAT_OVP_ALM_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_01, SC8551_BAT_OVP_ALM_MASK, val);
}

static int sc8551_enable_batocp(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BAT_OCP_ENABLE : SC8551_BAT_OCP_DISABLE;
	val <<= SC8551_BAT_OCP_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_02, SC8551_BAT_OCP_DIS_MASK, val);
}

static int sc8551_set_batocp_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BAT_OCP_BASE)
		threshold = SC8551_BAT_OCP_BASE;

	val = (threshold - SC8551_BAT_OCP_BASE) / SC8551_BAT_OCP_LSB;
	val <<= SC8551_BAT_OCP_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_02, SC8551_BAT_OCP_MASK, val);
}

static int sc8551_enable_batocp_alarm(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BAT_OCP_ALM_ENABLE : SC8551_BAT_OCP_ALM_DISABLE;
	val <<= SC8551_BAT_OCP_ALM_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_03, SC8551_BAT_OCP_ALM_DIS_MASK, val);
}

static int sc8551_set_batocp_alarm_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BAT_OCP_ALM_BASE)
		threshold = SC8551_BAT_OCP_ALM_BASE;

	val = (threshold - SC8551_BAT_OCP_ALM_BASE) / SC8551_BAT_OCP_ALM_LSB;
	val <<= SC8551_BAT_OCP_ALM_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_03, SC8551_BAT_OCP_ALM_MASK, val);
}

static int sc8551_set_busovp_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BUS_OVP_BASE)
		threshold = SC8551_BUS_OVP_BASE;

	val = (threshold - SC8551_BUS_OVP_BASE) / SC8551_BUS_OVP_LSB;
	val <<= SC8551_BUS_OVP_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_06, SC8551_BUS_OVP_MASK, val);
}

static int sc8551_enable_busovp_alarm(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BUS_OVP_ALM_ENABLE : SC8551_BUS_OVP_ALM_DISABLE;
	val <<= SC8551_BUS_OVP_ALM_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_07, SC8551_BUS_OVP_ALM_DIS_MASK, val);
}

static int sc8551_set_busovp_alarm_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BUS_OVP_ALM_BASE)
		threshold = SC8551_BUS_OVP_ALM_BASE;

	val = (threshold - SC8551_BUS_OVP_ALM_BASE) / SC8551_BUS_OVP_ALM_LSB;
	val <<= SC8551_BUS_OVP_ALM_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_07, SC8551_BUS_OVP_ALM_MASK, val);
}

static int sc8551_enable_busocp(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BUS_OCP_ENABLE : SC8551_BUS_OCP_DISABLE;
	val <<= SC8551_BUS_OCP_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_08, SC8551_BUS_OCP_DIS_MASK, val);
}

static int sc8551_set_busocp_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BUS_OCP_BASE)
		threshold = SC8551_BUS_OCP_BASE;

	val = (threshold - SC8551_BUS_OCP_BASE) / SC8551_BUS_OCP_LSB;
	val <<= SC8551_BUS_OCP_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_08, SC8551_BUS_OCP_MASK, val);
}

static int sc8551_enable_busocp_alarm(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BUS_OCP_ALM_ENABLE : SC8551_BUS_OCP_ALM_DISABLE;
	val <<= SC8551_BUS_OCP_ALM_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_09, SC8551_BUS_OCP_ALM_DIS_MASK, val);
}

static int sc8551_set_busocp_alarm_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BUS_OCP_ALM_BASE)
		threshold = SC8551_BUS_OCP_ALM_BASE;

	val = (threshold - SC8551_BUS_OCP_ALM_BASE) / SC8551_BUS_OCP_ALM_LSB;
	val <<= SC8551_BUS_OCP_ALM_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_09, SC8551_BUS_OCP_ALM_MASK, val);
}

static int sc8551_enable_batucp_alarm(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_BAT_UCP_ALM_ENABLE : SC8551_BAT_UCP_ALM_DISABLE;
	val <<= SC8551_BAT_UCP_ALM_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_04, SC8551_BAT_UCP_ALM_DIS_MASK, val);
}

static int sc8551_set_batucp_alarm_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_BAT_UCP_ALM_BASE)
		threshold = SC8551_BAT_UCP_ALM_BASE;

	val = (threshold - SC8551_BAT_UCP_ALM_BASE) / SC8551_BAT_UCP_ALM_LSB;
	val <<= SC8551_BAT_UCP_ALM_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_04, SC8551_BAT_UCP_ALM_MASK, val);
}

static int sc8551_set_acovp_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	if (threshold < SC8551_AC_OVP_BASE)
		threshold = SC8551_AC_OVP_BASE;

	if (threshold == SC8551_AC_OVP_6P5V)
		val = 0x07;
	else
		val = (threshold - SC8551_AC_OVP_BASE) / SC8551_AC_OVP_LSB;

	val <<= SC8551_AC_OVP_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_05, SC8551_AC_OVP_MASK, val);
}

static int sc8551_set_vdrop_th(struct sc8551 *sc, int threshold)
{
	u8 val;

	val = (threshold == 300) ? SC8551_VDROP_THRESHOLD_300MV :
				   SC8551_VDROP_THRESHOLD_400MV;
	val <<= SC8551_VDROP_THRESHOLD_SET_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_05, SC8551_VDROP_THRESHOLD_SET_MASK, val);
}

static int sc8551_set_vdrop_deglitch(struct sc8551 *sc, int us)
{
	u8 val;

	val = (us == 8) ? SC8551_VDROP_DEGLITCH_8US : SC8551_VDROP_DEGLITCH_5MS;
	val <<= SC8551_VDROP_DEGLITCH_SET_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_05, SC8551_VDROP_DEGLITCH_SET_MASK, val);
}

static int sc8551_enable_bat_therm(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_TSBAT_ENABLE : SC8551_TSBAT_DISABLE;
	val <<= SC8551_TSBAT_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_0C, SC8551_TSBAT_DIS_MASK, val);
}

static int sc8551_set_bat_therm_th(struct sc8551 *sc, u8 threshold)
{
	return sc8551_write_byte(sc, SC8551_REG_29, threshold);
}

static int sc8551_enable_bus_therm(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_TSBUS_ENABLE : SC8551_TSBUS_DISABLE;
	val <<= SC8551_TSBUS_DIS_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_0C, SC8551_TSBUS_DIS_MASK, val);
}

static int sc8551_set_bus_therm_th(struct sc8551 *sc, u8 threshold)
{
	return sc8551_write_byte(sc, SC8551_REG_28, threshold);
}

static int sc8551_set_die_therm_th(struct sc8551 *sc, int threshold_c)
{
	u8 delta, val;

	if (threshold_c < SC8551_TDIE_ALM_BASE)
		threshold_c = SC8551_TDIE_ALM_BASE;

	delta = (u8)(threshold_c - SC8551_TDIE_ALM_BASE);
	val = delta << 1;
	val <<= SC8551_TDIE_ALM_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_2A, SC8551_TDIE_ALM_MASK, val);
}

static int sc8551_enable_adc(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_ADC_ENABLE : SC8551_ADC_DISABLE;
	val <<= SC8551_ADC_EN_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_14, SC8551_ADC_EN_MASK, val);
}

static int sc8551_set_adc_scanrate(struct sc8551 *sc, bool oneshot)
{
	u8 val;

	val = oneshot ? SC8551_ADC_RATE_ONESHOT : SC8551_ADC_RATE_CONTINOUS;
	val <<= SC8551_ADC_RATE_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_14, SC8551_ADC_RATE_MASK, val);
}

#define ADC_REG_BASE SC8551_REG_16
static int __sc8551_get_adc_data(struct sc8551 *sc, int channel, int *result)
{
	int ret;
	u8 buf[2];
	u16 val;

	ret = regmap_bulk_read(sc->regmap, ADC_REG_BASE + (channel << 1), buf, 2);
	if (ret < 0) {
		sc_err("bulk read channel %d failed: %d\n", channel, ret);
		return ret;
	}

	if (channel == ADC_IBUS || channel == ADC_IBAT)
		buf[0] &= 0x03;
	else if (channel == ADC_VBUS || channel == ADC_VAC || channel == ADC_VOUT || channel == ADC_VBAT)
		buf[0] &= 0x0F;
	else
		buf[0] &= 0x01;

	val = ((u16)buf[0] << 8) | buf[1];

	if (sc->chip_vendor == SC8551) {
		switch (channel) {
		case ADC_VAC:
			*result = (int)((val << 2) + val);
			break;
		case ADC_TDIE:
			*result = (int)(val >> 1);
			break;
		case ADC_VOUT:
			*result = (int)((val * 5) >> 2);
			break;
		case ADC_IBAT:
			*result = (int)((val * 25) >> 3);
			break;
		case ADC_IBUS:
			*result = (int)((val * 25) >> 4);
			break;
		case ADC_VBUS:
			*result = (int)((val * 15) >> 2);
			break;
		case ADC_VBAT:
			*result = (int)(((u64)val * 125750) / 100000);
			break;
		case ADC_TBUS:
		case ADC_TBAT:
			*result = (int)(((u64)val * 9766) / 100000);
			break;
		default:
			*result = (int)val;
			break;
		}
	} else {
		*result = (int)val;
	}

	return 0;
}

static int sc8551_get_adc_data(struct sc8551 *sc, int channel, int *result)
{
	int ret;

	if (channel < 0 || channel >= ADC_MAX_NUM)
		return -EINVAL;

	mutex_lock(&sc->data_lock);
	ret = __sc8551_get_adc_data(sc, channel, result);
	mutex_unlock(&sc->data_lock);

	return ret;
}

static int sc8551_set_adc_scan(struct sc8551 *sc, int channel, bool enable)
{
	u8 reg, mask, shift, val;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	if (channel == ADC_IBUS) {
		reg = SC8551_REG_14;
		shift = SC8551_IBUS_ADC_DIS_SHIFT;
		mask = SC8551_IBUS_ADC_DIS_MASK;
	} else {
		reg = SC8551_REG_15;
		shift = 8 - channel;
		mask = 1 << shift;
	}

	val = enable ? (0 << shift) : (1 << shift);

	return sc8551_update_bits(sc, reg, mask, val);
}

static int sc8551_set_alarm_int_mask(struct sc8551 *sc, u8 mask)
{
	int ret;
	u8 val;

	ret = sc8551_read_byte(sc, SC8551_REG_0F, &val);
	if (ret)
		return ret;

	val |= mask;
	return sc8551_write_byte(sc, SC8551_REG_0F, val);
}

static __maybe_unused int sc8551_clear_alarm_int_mask(struct sc8551 *sc, u8 mask)
{
	int ret;
	u8 val;

	ret = sc8551_read_byte(sc, SC8551_REG_0F, &val);
	if (ret)
		return ret;

	val &= ~mask;
	return sc8551_write_byte(sc, SC8551_REG_0F, val);
}

static __maybe_unused int sc8551_set_fault_int_mask(struct sc8551 *sc, u8 mask)
{
	int ret;
	u8 val;

	ret = sc8551_read_byte(sc, SC8551_REG_12, &val);
	if (ret)
		return ret;

	val |= mask;
	return sc8551_write_byte(sc, SC8551_REG_12, val);
}

static __maybe_unused int sc8551_clear_fault_int_mask(struct sc8551 *sc, u8 mask)
{
	int ret;
	u8 val;

	ret = sc8551_read_byte(sc, SC8551_REG_12, &val);
	if (ret)
		return ret;

	val &= ~mask;
	return sc8551_write_byte(sc, SC8551_REG_12, val);
}

static int sc8551_set_sense_resistor(struct sc8551 *sc, int r_mohm)
{
	u8 val;

	if (r_mohm == 2)
		val = SC8551_SET_IBAT_SNS_RES_2MHM;
	else if (r_mohm == 5)
		val = SC8551_SET_IBAT_SNS_RES_5MHM;
	else
		return -EINVAL;

	val <<= SC8551_SET_IBAT_SNS_RES_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_2B, SC8551_SET_IBAT_SNS_RES_MASK, val);
}

static int sc8551_enable_regulation(struct sc8551 *sc, bool enable)
{
	u8 val;

	val = enable ? SC8551_EN_REGULATION_ENABLE : SC8551_EN_REGULATION_DISABLE;
	val <<= SC8551_EN_REGULATION_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_2B, SC8551_EN_REGULATION_MASK, val);
}

static int sc8551_set_ss_timeout(struct sc8551 *sc, int timeout)
{
	u8 val;

	switch (timeout) {
	case 0:
		val = SC8551_SS_TIMEOUT_DISABLE;
		break;
	case 12:
		val = SC8551_SS_TIMEOUT_12P5MS;
		break;
	case 25:
		val = SC8551_SS_TIMEOUT_25MS;
		break;
	case 50:
		val = SC8551_SS_TIMEOUT_50MS;
		break;
	case 100:
		val = SC8551_SS_TIMEOUT_100MS;
		break;
	case 400:
		val = SC8551_SS_TIMEOUT_400MS;
		break;
	case 1500:
		val = SC8551_SS_TIMEOUT_1500MS;
		break;
	case 100000:
		val = SC8551_SS_TIMEOUT_100000MS;
		break;
	default:
		val = SC8551_SS_TIMEOUT_DISABLE;
		break;
	}

	val <<= SC8551_SS_TIMEOUT_SET_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_2B, SC8551_SS_TIMEOUT_SET_MASK, val);
}

static int sc8551_set_ibat_reg_th(struct sc8551 *sc, int th_ma)
{
	u8 val;

	if (th_ma == 200)
		val = SC8551_IBAT_REG_200MA;
	else if (th_ma == 300)
		val = SC8551_IBAT_REG_300MA;
	else if (th_ma == 400)
		val = SC8551_IBAT_REG_400MA;
	else
		val = SC8551_IBAT_REG_500MA;

	val <<= SC8551_IBAT_REG_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_2C, SC8551_IBAT_REG_MASK, val);
}

static int sc8551_set_vbat_reg_th(struct sc8551 *sc, int th_mv)
{
	u8 val;

	if (th_mv == 50)
		val = SC8551_VBAT_REG_50MV;
	else if (th_mv == 100)
		val = SC8551_VBAT_REG_100MV;
	else if (th_mv == 150)
		val = SC8551_VBAT_REG_150MV;
	else
		val = SC8551_VBAT_REG_200MV;

	val <<= SC8551_VBAT_REG_SHIFT;

	return sc8551_update_bits(sc, SC8551_REG_2C, SC8551_VBAT_REG_MASK, val);
}

static int sc8551_check_vbus_error_status(struct sc8551 *sc)
{
	int ret;
	u8 data;

	ret = sc8551_read_byte(sc, SC8551_REG_0A, &data);
	if (!ret) {
		sc_log("vbus error >>>>%02x\n", data);
		sc->vbus_error = data;
	}

	return ret;
}

static int sc8551_detect_device(struct sc8551 *sc)
{
	int ret;
	u8 data;

	ret = sc8551_read_byte(sc, SC8551_REG_13, &data);
	if (ret) {
		pr_err("[sc8551_chg]: %s: device detect fail, ret=%d\n", __func__, ret);
		return ret;
	}

	sc->part_no = (data & SC8551_DEV_ID_MASK);
	sc->part_no >>= SC8551_DEV_ID_SHIFT;
	if (data == SC8551_DEVICE_ID || data == SC8551A_DEVICE_ID)
		sc->chip_vendor = SC8551;

	pr_info("[sc8551_chg]: %s: PART_INFO: 0x%x\n", __func__, data);
	return 0;
}

static int sc8551_parse_dt(struct sc8551 *sc, struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;

	if (!np) {
		sc_err("no device tree node\n");
		return -ENODEV;
	}

	sc->irq_gpiod = devm_gpiod_get(dev, "intr", GPIOD_IN);
	if (IS_ERR(sc->irq_gpiod)) {
		ret = PTR_ERR(sc->irq_gpiod);
		sc_err("devm_gpiod_get(intr) failed: %d\n", ret);
		return ret;
	}
	sc_info("intr descriptor acquired\n");

	sc->cfg.bat_ovp_disable =
		of_property_read_bool(np, "sc,sc8551,bat-ovp-disable");
	sc->cfg.bat_ocp_disable =
		of_property_read_bool(np, "sc,sc8551,bat-ocp-disable");
	sc->cfg.bat_ovp_alm_disable =
		of_property_read_bool(np, "sc,sc8551,bat-ovp-alarm-disable");
	sc->cfg.bat_ocp_alm_disable =
		of_property_read_bool(np, "sc,sc8551,bat-ocp-alarm-disable");
	sc->cfg.bus_ocp_disable =
		of_property_read_bool(np, "sc,sc8551,bus-ocp-disable");
	sc->cfg.bus_ovp_alm_disable =
		of_property_read_bool(np, "sc,sc8551,bus-ovp-alarm-disable");
	sc->cfg.bus_ocp_alm_disable =
		of_property_read_bool(np, "sc,sc8551,bus-ocp-alarm-disable");
	sc->cfg.bat_ucp_alm_disable =
		of_property_read_bool(np, "sc,sc8551,bat-ucp-alarm-disable");
	sc->cfg.bat_therm_disable =
		of_property_read_bool(np, "sc,sc8551,bat-therm-disable");
	sc->cfg.bus_therm_disable =
		of_property_read_bool(np, "sc,sc8551,bus-therm-disable");

	ret = of_property_read_u32(np, "sc,sc8551,bat-ovp-threshold",
				   &sc->cfg.bat_ovp_th);
	if (ret) {
		sc_err("failed to read bat-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bat-ovp-alarm-threshold",
				   &sc->cfg.bat_ovp_alm_th);
	if (ret) {
		sc_err("failed to read bat-ovp-alarm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bat-ocp-threshold",
				   &sc->cfg.bat_ocp_th);
	if (ret) {
		sc_err("failed to read bat-ocp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bat-ocp-alarm-threshold",
				   &sc->cfg.bat_ocp_alm_th);
	if (ret) {
		sc_err("failed to read bat-ocp-alarm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bus-ovp-threshold",
				   &sc->cfg.bus_ovp_th);
	if (ret) {
		sc_err("failed to read bus-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bus-ovp-alarm-threshold",
				   &sc->cfg.bus_ovp_alm_th);
	if (ret) {
		sc_err("failed to read bus-ovp-alarm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bus-ocp-threshold",
				   &sc->cfg.bus_ocp_th);
	if (ret) {
		sc_err("failed to read bus-ocp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bus-ocp-alarm-threshold",
				   &sc->cfg.bus_ocp_alm_th);
	if (ret) {
		sc_err("failed to read bus-ocp-alarm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bat-ucp-alarm-threshold",
				   &sc->cfg.bat_ucp_alm_th);
	if (ret) {
		sc_err("failed to read bat-ucp-alarm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bat-therm-threshold",
				   &sc->cfg.bat_therm_th);
	if (ret) {
		sc_err("failed to read bat-therm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,bus-therm-threshold",
				   &sc->cfg.bus_therm_th);
	if (ret) {
		sc_err("failed to read bus-therm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,die-therm-threshold",
				   &sc->cfg.die_therm_th);
	if (ret) {
		sc_err("failed to read die-therm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,ac-ovp-threshold",
				   &sc->cfg.ac_ovp_th);
	if (ret) {
		sc_err("failed to read ac-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8551,sense-resistor-mohm",
				   &sc->cfg.sense_r_mohm);
	if (ret) {
		sc_err("failed to read sense-resistor-mohm\n");
		return ret;
	}

	return 0;
}

static int sc8551_init_protection(struct sc8551 *sc)
{
	int ret;

	ret = sc8551_enable_batovp(sc, !sc->cfg.bat_ovp_disable);
	sc_info("%s bat ovp %s\n",
		sc->cfg.bat_ovp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_batocp(sc, !sc->cfg.bat_ocp_disable);
	sc_info("%s bat ocp %s\n",
		sc->cfg.bat_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_batovp_alarm(sc, !sc->cfg.bat_ovp_alm_disable);
	sc_info("%s bat ovp alarm %s\n",
		sc->cfg.bat_ovp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_batocp_alarm(sc, !sc->cfg.bat_ocp_alm_disable);
	sc_info("%s bat ocp alarm %s\n",
		sc->cfg.bat_ocp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_batucp_alarm(sc, !sc->cfg.bat_ucp_alm_disable);
	sc_info("%s bat ucp alarm %s\n",
		sc->cfg.bat_ucp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_busovp_alarm(sc, !sc->cfg.bus_ovp_alm_disable);
	sc_info("%s bus ovp alarm %s\n",
		sc->cfg.bus_ovp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_busocp(sc, !sc->cfg.bus_ocp_disable);
	sc_info("%s bus ocp %s\n",
		sc->cfg.bus_ocp_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_busocp_alarm(sc, !sc->cfg.bus_ocp_alm_disable);
	sc_info("%s bus ocp alarm %s\n",
		sc->cfg.bus_ocp_alm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_bat_therm(sc, !sc->cfg.bat_therm_disable);
	sc_info("%s bat therm %s\n",
		sc->cfg.bat_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_enable_bus_therm(sc, !sc->cfg.bus_therm_disable);
	sc_info("%s bus therm %s\n",
		sc->cfg.bus_therm_disable ? "disable" : "enable",
		!ret ? "successfullly" : "failed");

	ret = sc8551_set_batovp_th(sc, sc->cfg.bat_ovp_th);
	sc_info("set bat ovp th %d %s\n", sc->cfg.bat_ovp_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_batovp_alarm_th(sc, sc->cfg.bat_ovp_alm_th);
	sc_info("set bat ovp alarm threshold %d %s\n", sc->cfg.bat_ovp_alm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_batocp_th(sc, sc->cfg.bat_ocp_th);
	sc_info("set bat ocp threshold %d %s\n", sc->cfg.bat_ocp_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_batocp_alarm_th(sc, sc->cfg.bat_ocp_alm_th);
	sc_info("set bat ocp alarm threshold %d %s\n", sc->cfg.bat_ocp_alm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_busovp_th(sc, sc->cfg.bus_ovp_th);
	sc_info("set bus ovp threshold %d %s\n", sc->cfg.bus_ovp_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_busovp_alarm_th(sc, sc->cfg.bus_ovp_alm_th);
	sc_info("set bus ovp alarm threshold %d %s\n", sc->cfg.bus_ovp_alm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_busocp_th(sc, sc->cfg.bus_ocp_th);
	sc_info("set bus ocp threshold %d %s\n", sc->cfg.bus_ocp_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_busocp_alarm_th(sc, sc->cfg.bus_ocp_alm_th);
	sc_info("set bus ocp alarm th %d %s\n", sc->cfg.bus_ocp_alm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_batucp_alarm_th(sc, sc->cfg.bat_ucp_alm_th);
	sc_info("set bat ucp threshold %d %s\n", sc->cfg.bat_ucp_alm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_bat_therm_th(sc, sc->cfg.bat_therm_th);
	sc_info("set bat therm threshold %d %s\n", sc->cfg.bat_therm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_bus_therm_th(sc, sc->cfg.bus_therm_th);
	sc_info("set bus therm threshold %d %s\n", sc->cfg.bus_therm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_die_therm_th(sc, sc->cfg.die_therm_th);
	sc_info("set die therm threshold %d %s\n", sc->cfg.die_therm_th,
		!ret ? "successfully" : "failed");

	ret = sc8551_set_acovp_th(sc, sc->cfg.ac_ovp_th);
	sc_info("set ac ovp threshold %d %s\n", sc->cfg.ac_ovp_th,
		!ret ? "successfully" : "failed");

	return 0;
}

static int sc8551_init_adc(struct sc8551 *sc)
{
	sc8551_set_adc_scanrate(sc, false);
	sc8551_set_adc_scan(sc, ADC_IBUS, true);
	sc8551_set_adc_scan(sc, ADC_VBUS, true);
	sc8551_set_adc_scan(sc, ADC_VOUT, true);
	sc8551_set_adc_scan(sc, ADC_VBAT, true);
	sc8551_set_adc_scan(sc, ADC_IBAT, true);
	sc8551_set_adc_scan(sc, ADC_TBUS, true);
	sc8551_set_adc_scan(sc, ADC_TBAT, true);
	sc8551_set_adc_scan(sc, ADC_TDIE, true);
	sc8551_set_adc_scan(sc, ADC_VAC, true);

	/* improve adc accuracy */
	sc8551_write_byte(sc, SC8551_REG_34, 0x01);
	sc8551_enable_adc(sc, true);

	return 0;
}

static int sc8551_init_int_src(struct sc8551 *sc)
{
	int ret;

	ret = sc8551_set_alarm_int_mask(sc, ADC_DONE | BAT_OVP_ALARM);
	if (ret) {
		sc_err("failed to set alarm mask: %d\n", ret);
		return ret;
	}

	return ret;
}

static int sc8551_init_regulation(struct sc8551 *sc)
{
	sc8551_set_ibat_reg_th(sc, 300);
	sc8551_set_vbat_reg_th(sc, 100);

	sc8551_set_vdrop_deglitch(sc, 5000);
	sc8551_set_vdrop_th(sc, 400);

	sc8551_enable_regulation(sc, false);

	sc8551_write_byte(sc, SC8551_REG_2E, 0x08);
	sc8551_write_byte(sc, SC8551_REG_34, 0x01);

	return 0;
}

static int sc8551_disable_vbus_range(struct sc8551 *sc, bool disable)
{
	u8 val = disable ? SC8551_VBUS_RANGE_DISABLE : SC8551_VBUS_RANGE_ENABLE;

	val <<= SC8551_VBUS_RANGE_DIS_SHIFT;
	return sc8551_update_bits(sc, SC8551_REG_35, SC8551_VBUS_RANGE_DIS_MASK, val);
}

static int sc8551_init_device(struct sc8551 *sc)
{
	sc_info("entry\n");
	sc8551_set_reg_reset(sc);
	sc8551_enable_wdt(sc, false);

	sc8551_set_ss_timeout(sc, 100000);
	sc8551_set_sense_resistor(sc, sc->cfg.sense_r_mohm);

	sc8551_init_protection(sc);
	sc8551_init_adc(sc);
	sc8551_init_int_src(sc);
	sc8551_init_regulation(sc);

	if (sc->mode == SC8551_ROLE_SLAVE)
		sc8551_disable_vbus_range(sc, true);

	return 0;
}

static int sc8551_set_present(struct sc8551 *sc, bool present)
{
	sc->usb_present = present;

	if (present)
		sc8551_init_device(sc);

	return 0;
}

static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc8551 *sc = dev_get_drvdata(dev);
	u8 addr, val;
	int idx = 0, ret;

	if (!sc)
		return -ENODEV;

	idx += scnprintf(buf + idx, PAGE_SIZE - idx, "sc8551:\n");

	for (addr = 0x00; addr <= 0x31; addr++) {
		ret = sc8551_read_byte(sc, addr, &val);
		if (ret) {
			dev_warn(sc->dev, "read reg 0x%02x failed: %d\n", addr, ret);
			continue;
		}

		idx += scnprintf(buf + idx, PAGE_SIZE - idx, "Reg[0x%02x] = 0x%02x\n",
				 addr, val);
		if (idx >= PAGE_SIZE)
			break;
	}

	return idx;
}

static ssize_t registers_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sc8551 *sc = dev_get_drvdata(dev);
	char tmp[32];
	unsigned long reg_ul, val_ul;
	int ret;
	char *p;

	if (!sc)
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

	if (reg_ul > 0x31 || val_ul > 0xff) {
		dev_err(sc->dev, "invalid reg/val: reg=0x%lx val=0x%lx\n", reg_ul, val_ul);
		return -EINVAL;
	}

	ret = sc8551_write_byte(sc, (u8)reg_ul, (u8)val_ul);
	if (ret) {
		dev_err(sc->dev, "write reg 0x%02lx failed: %d\n", reg_ul, ret);
		return ret;
	}

	dev_info(sc->dev, "wrote 0x%02lx to reg 0x%02lx\n", val_ul, reg_ul);
	return count;
}

static DEVICE_ATTR_RW(registers);

static struct attribute *sc8551_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group sc8551_attr_group = {
	.attrs = sc8551_attributes,
};

static void sc8551_check_alarm_status(struct sc8551 *sc);
static void sc8551_check_fault_status(struct sc8551 *sc);

static enum power_supply_property sc8551_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_SC_BATTERY_PRESENT,
	POWER_SUPPLY_PROP_SC_VBUS_PRESENT,
	POWER_SUPPLY_PROP_SC_BATTERY_VOLTAGE,
	POWER_SUPPLY_PROP_SC_BATTERY_CURRENT,
	POWER_SUPPLY_PROP_SC_BATTERY_TEMPERATURE,
	POWER_SUPPLY_PROP_SC_BUS_VOLTAGE,
	POWER_SUPPLY_PROP_SC_BUS_CURRENT,
	POWER_SUPPLY_PROP_SC_BUS_TEMPERATURE,
	POWER_SUPPLY_PROP_SC_DIE_TEMPERATURE,
	POWER_SUPPLY_PROP_SC_ALARM_STATUS,
	POWER_SUPPLY_PROP_SC_FAULT_STATUS,
	POWER_SUPPLY_PROP_SC_VBUS_ERROR_STATUS,
	POWER_SUPPLY_PROP_SC_CHIP_VENDOR,
};

static int sc8551_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sc8551 *sc = power_supply_get_drvdata(psy);
	int result, ret;
	u8 reg_val;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		sc8551_check_charge_enabled(sc, &sc->charge_enabled);
		val->intval = sc->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sc->usb_present;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_PRESENT:
		ret = sc8551_read_byte(sc, SC8551_REG_0D, &reg_val);
		if (!ret)
			sc->batt_present = !!(reg_val & VBAT_INSERT);
		val->intval = sc->batt_present;
		break;
	case POWER_SUPPLY_PROP_SC_VBUS_PRESENT:
		ret = sc8551_read_byte(sc, SC8551_REG_0D, &reg_val);
		if (!ret)
			sc->vbus_present = !!(reg_val & VBUS_INSERT);
		val->intval = sc->vbus_present;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_VOLTAGE:
		ret = sc8551_get_adc_data(sc, ADC_VBAT, &result);
		if (!ret)
			sc->vbat_volt = result;
		val->intval = sc->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_CURRENT:
		ret = sc8551_get_adc_data(sc, ADC_IBAT, &result);
		if (!ret)
			sc->ibat_curr = result;
		val->intval = sc->ibat_curr;
		break;
	case POWER_SUPPLY_PROP_SC_BATTERY_TEMPERATURE:
		ret = sc8551_get_adc_data(sc, ADC_TBAT, &result);
		if (!ret)
			sc->bat_temp = result;
		val->intval = sc->bat_temp;
		break;
	case POWER_SUPPLY_PROP_SC_BUS_VOLTAGE:
		ret = sc8551_get_adc_data(sc, ADC_VBUS, &result);
		if (!ret)
			sc->vbus_volt = result;
		val->intval = sc->vbus_volt;
		break;
	case POWER_SUPPLY_PROP_SC_BUS_CURRENT:
		ret = sc8551_get_adc_data(sc, ADC_IBUS, &result);
		if (!ret)
			sc->ibus_curr = result;
		val->intval = sc->ibus_curr;
		break;
	case POWER_SUPPLY_PROP_SC_BUS_TEMPERATURE:
		ret = sc8551_get_adc_data(sc, ADC_TBUS, &result);
		if (!ret)
			sc->bus_temp = result;
		val->intval = sc->bus_temp;
		break;
	case POWER_SUPPLY_PROP_SC_DIE_TEMPERATURE:
		ret = sc8551_get_adc_data(sc, ADC_TDIE, &result);
		if (!ret)
			sc->die_temp = result;
		val->intval = sc->die_temp;
		break;
	case POWER_SUPPLY_PROP_SC_ALARM_STATUS:
		sc8551_check_alarm_status(sc);
		val->intval = ((sc->bat_ovp_alarm << BAT_OVP_ALARM_SHIFT)
			| (sc->bat_ocp_alarm << BAT_OCP_ALARM_SHIFT)
			| (sc->bat_ucp_alarm << BAT_UCP_ALARM_SHIFT)
			| (sc->bus_ovp_alarm << BUS_OVP_ALARM_SHIFT)
			| (sc->bus_ocp_alarm << BUS_OCP_ALARM_SHIFT)
			| (sc->bat_therm_alarm << BAT_THERM_ALARM_SHIFT)
			| (sc->bus_therm_alarm << BUS_THERM_ALARM_SHIFT)
			| (sc->die_therm_alarm << DIE_THERM_ALARM_SHIFT));
		break;
	case POWER_SUPPLY_PROP_SC_FAULT_STATUS:
		sc8551_check_fault_status(sc);
		val->intval = ((sc->bat_ovp_fault << BAT_OVP_FAULT_SHIFT)
			| (sc->bat_ocp_fault << BAT_OCP_FAULT_SHIFT)
			| (sc->bus_ovp_fault << BUS_OVP_FAULT_SHIFT)
			| (sc->bus_ocp_fault << BUS_OCP_FAULT_SHIFT)
			| (sc->bat_therm_fault << BAT_THERM_FAULT_SHIFT)
			| (sc->bus_therm_fault << BUS_THERM_FAULT_SHIFT)
			| (sc->die_therm_fault << DIE_THERM_FAULT_SHIFT));
		break;
	case POWER_SUPPLY_PROP_SC_VBUS_ERROR_STATUS:
		sc8551_check_vbus_error_status(sc);
		val->intval = sc->vbus_error;
		break;
	case POWER_SUPPLY_PROP_SC_CHIP_VENDOR:
		val->intval = sc->chip_vendor;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sc8551_charger_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct sc8551 *sc = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		sc8551_enable_charge(sc, val->intval);
		sc8551_check_charge_enabled(sc, &sc->charge_enabled);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		sc8551_set_present(sc, !!val->intval);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sc8551_charger_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_PRESENT:
		return 1;
	default:
		break;
	}
	return 0;
}

static int sc8551_psy_register(struct sc8551 *sc)
{
	struct power_supply_config sc_psy_config = {};

	sc_psy_config.drv_data = sc;
	sc_psy_config.of_node = sc->dev->of_node;

	if (sc->mode == SC8551_ROLE_MASTER)
		sc->psy_desc.name = "sc8551-master";
	else if (sc->mode == SC8551_ROLE_SLAVE)
		sc->psy_desc.name = "sc8551-slave";
	else
		sc->psy_desc.name = "sc8551-standalone";

	sc->psy_desc.type = POWER_SUPPLY_TYPE_CHARGE_PUMP;
	sc->psy_desc.properties = sc8551_charger_props;
	sc->psy_desc.num_properties = ARRAY_SIZE(sc8551_charger_props);
	sc->psy_desc.get_property = sc8551_charger_get_property;
	sc->psy_desc.set_property = sc8551_charger_set_property;
	sc->psy_desc.property_is_writeable = sc8551_charger_is_writeable;

	sc->fc2_psy = devm_power_supply_register(sc->dev, &sc->psy_desc, &sc_psy_config);
	if (IS_ERR(sc->fc2_psy)) {
		sc_err("failed to register fc2_psy\n");
		return PTR_ERR(sc->fc2_psy);
	}

	sc_info("%s power supply register successfully\n", sc->psy_desc.name);
	return 0;
}

static void sc8551_check_alarm_status(struct sc8551 *sc)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;

	mutex_lock(&sc->data_lock);

	ret = sc8551_read_byte(sc, SC8551_REG_08, &flag);
	if (!ret && (flag & SC8551_IBUS_UCP_FALL_FLAG_MASK))
		sc_dbg("UCP_FLAG = 0x%02X\n",
		       !!(flag & SC8551_IBUS_UCP_FALL_FLAG_MASK));

	ret = sc8551_read_byte(sc, SC8551_REG_2D, &flag);
	if (!ret && (flag & SC8551_VDROP_OVP_FLAG_MASK))
		sc_dbg("VDROP_OVP_FLAG = 0x%02X\n",
		       !!(flag & SC8551_VDROP_OVP_FLAG_MASK));

	/* read to clear alarm flag */
	ret = sc8551_read_byte(sc, SC8551_REG_0E, &flag);
	if (!ret && flag)
		sc_dbg("INT_FLAG = 0x%02X\n", flag);

	ret = sc8551_read_byte(sc, SC8551_REG_0D, &stat);
	if (!ret && stat != sc->prev_alarm) {
		sc_dbg("INT_STAT = 0X%02x\n", stat);
		sc->prev_alarm = stat;
		sc->bat_ovp_alarm = !!(stat & BAT_OVP_ALARM);
		sc->bat_ocp_alarm = !!(stat & BAT_OCP_ALARM);
		sc->bus_ovp_alarm = !!(stat & BUS_OVP_ALARM);
		sc->bus_ocp_alarm = !!(stat & BUS_OCP_ALARM);
		sc->batt_present = !!(stat & VBAT_INSERT);
		sc->vbus_present = !!(stat & VBUS_INSERT);
		sc->bat_ucp_alarm = !!(stat & BAT_UCP_ALARM);
	}

	ret = sc8551_read_byte(sc, SC8551_REG_08, &stat);
	if (!ret && (stat & 0x50))
		sc_dbg("Reg[05]BUS_UCPOVP = 0x%02X\n", stat);

	ret = sc8551_read_byte(sc, SC8551_REG_0A, &stat);
	if (!ret && (stat & 0x02))
		sc_dbg("Reg[0A]CONV_OCP = 0x%02X\n", stat);

	mutex_unlock(&sc->data_lock);
}

static void sc8551_check_fault_status(struct sc8551 *sc)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;

	mutex_lock(&sc->data_lock);

	ret = sc8551_read_byte(sc, SC8551_REG_10, &stat);
	if (!ret && stat)
		sc_log("Reg[10]FAULT_STAT = 0x%02X\n", stat);

	ret = sc8551_read_byte(sc, SC8551_REG_11, &flag);
	if (!ret && flag)
		sc_log("Reg[11]FAULT_FLAG = 0x%02X\n", flag);

	if (!ret && flag != sc->prev_fault) {
		sc->prev_fault = flag;
		sc->bat_ovp_fault = !!(flag & BAT_OVP_FAULT);
		sc->bat_ocp_fault = !!(flag & BAT_OCP_FAULT);
		sc->bus_ovp_fault = !!(flag & BUS_OVP_FAULT);
		sc->bus_ocp_fault = !!(flag & BUS_OCP_FAULT);
		sc->bat_therm_fault = !!(flag & TS_BAT_FAULT);
		sc->bus_therm_fault = !!(flag & TS_BUS_FAULT);
		sc->die_therm_fault = !!(flag & TS_DIE_FAULT);
		sc->die_therm_alarm = !!(flag & TS_DIE_FAULT);
		sc->bat_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
		sc->bus_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
	}

	mutex_unlock(&sc->data_lock);
}

static irqreturn_t sc8551_thread_irq(int irq, void *data)
{
	struct sc8551 *sc = data;

	if (!sc)
		return IRQ_HANDLED;

	pm_wakeup_ws_event(sc->sc_ws, 500, false);

	return IRQ_HANDLED;
}

static int sc8551_parse_irq(struct sc8551 *sc)
{
	int irqn, ret;

	irqn = gpiod_to_irq(sc->irq_gpiod);
	if (irqn <= 0) {
		pr_err("[sc8551_chg]: %s: gpiod_to_irq failed: %d\n", __func__, irqn);
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(sc->dev, irqn,
					NULL, sc8551_thread_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"sc8551_charger_irq", sc);
	if (ret < 0) {
		pr_err("[sc8551_chg]: %s: request irq=%d failed, ret=%d\n",
		       __func__, irqn, ret);
		return ret;
	}

	sc->irq = irqn;
	return 0;
}

static const struct of_device_id sc8551_charger_match_table[] = {
	{
		.compatible = "sc,sc8551-standalone",
		.data = &sc8551_mode_data[SC8551_STANDALONE],
	},
	{
		.compatible = "sc,sc8551-master",
		.data = &sc8551_mode_data[SC8551_MASTER],
	},
	{
		.compatible = "sc,sc8551-slave",
		.data = &sc8551_mode_data[SC8551_SLAVE],
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sc8551_charger_match_table);

static int sc8551_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sc8551 *sc;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	int i, ret;

	pr_info("[sc8551_chg]: %s: entry\n", __func__);
	sc = devm_kzalloc(&client->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc) {
		pr_err("[sc8551_chg]: %s: out of memory\n", __func__);
		return -ENOMEM;
	}

	sc->dev = &client->dev;
	sc->client = client;
	i2c_set_clientdata(client, sc);

	sc->regmap = devm_regmap_init_i2c(sc->client, &sc8551_regmap_config);
	if (IS_ERR(sc->regmap)) {
		pr_err("[sc8551_chg]: %s: failed to init regmap: %ld\n",
		       __func__, PTR_ERR(sc->regmap));
		return PTR_ERR(sc->regmap);
	}

	mutex_init(&sc->data_lock);

	ret = sc8551_detect_device(sc);
	if (ret) {
		pr_err("[sc8551_chg]: %s: no sc8551 device found!\n", __func__);
		ret = -ENODEV;
		goto err_out;
	}

	match = of_match_node(sc8551_charger_match_table, node);
	if (!match) {
		pr_err("[sc8551_chg]: %s: device tree match not found!\n", __func__);
		ret = -ENODEV;
		goto err_out;
	}

	sc->mode = *(int *)match->data;
	for (i = 0; i < SC_RL_SLOTS; i++) {
		ratelimit_state_init(&sc->rl_slots[i], SC_RL_INTERVAL, SC_RL_BURST);
		sc->rl_slots[i].flags |= RATELIMIT_MSG_ON_RELEASE;
	}

	ret = sc8551_parse_dt(sc, sc->dev);
	if (ret) {
		pr_err("[sc8551_chg]: %s: parse_dt failed: %d\n", __func__, ret);
		ret = -EIO;
		goto err_out;
	}

	ret = sc8551_init_device(sc);
	if (ret) {
		pr_err("[sc8551_chg]: %s: failed to init device: %d\n", __func__, ret);
		goto err_out;
	}

	device_set_wakeup_capable(sc->dev, true);
	ret = device_set_wakeup_enable(sc->dev, true);
	if (ret) {
		pr_err("[sc8551_chg]: %s: enable wakeup failed: %d\n", __func__, ret);
		device_set_wakeup_capable(sc->dev, false);
		goto err_out;
	}

	sc->sc_ws = wakeup_source_register(sc->dev, "sc8551_ws");
	if (IS_ERR_OR_NULL(sc->sc_ws)) {
		ret = IS_ERR(sc->sc_ws) ? PTR_ERR(sc->sc_ws) : -ENOMEM;
		pr_err("[sc8551_chg]: %s: wakeup_source_register failed: %d\n",
		       __func__, ret);
		goto err_wake;
	}

	ret = sysfs_create_group(&sc->dev->kobj, &sc8551_attr_group);
	if (ret) {
		pr_err("[sc8551_chg]: %s: failed to create sysfs group: %d\n",
		       __func__, ret);
		goto err_unreg_ws;
	}

	ret = sc8551_psy_register(sc);
	if (ret) {
		pr_err("[sc8551_chg]: %s: psy_register failed: %d\n", __func__, ret);
		goto err_remove_sysfs;
	}

	ret = sc8551_parse_irq(sc);
	if (ret)
		goto err_remove_sysfs;

	pr_info("[sc8551_chg]: %s: successfully, part_num: %d\n", __func__, sc->part_no);
	return 0;

err_remove_sysfs:
	sysfs_remove_group(&sc->dev->kobj, &sc8551_attr_group);
err_unreg_ws:
	if (!IS_ERR_OR_NULL(sc->sc_ws))
		wakeup_source_unregister(sc->sc_ws);
err_wake:
	device_set_wakeup_enable(sc->dev, false);
	device_set_wakeup_capable(sc->dev, false);
err_out:
	mutex_destroy(&sc->data_lock);
	i2c_set_clientdata(client, NULL);
	pr_err("[sc8551_chg]: %s: probe failed: %d\n", __func__, ret);
	return ret;
}

static int sc8551_charger_remove(struct i2c_client *client)
{
	struct sc8551 *sc = i2c_get_clientdata(client);

	if (!sc)
		return 0;

	sc8551_enable_adc(sc, false);
	if (sc->irq) {
		disable_irq(sc->irq);
		synchronize_irq(sc->irq);
	}

	sysfs_remove_group(&sc->dev->kobj, &sc8551_attr_group);
	if (!IS_ERR_OR_NULL(sc->sc_ws))
		wakeup_source_unregister(sc->sc_ws);

	device_set_wakeup_enable(sc->dev, false);
	mutex_destroy(&sc->data_lock);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static void sc8551_charger_shutdown(struct i2c_client *client)
{
	struct sc8551 *sc = i2c_get_clientdata(client);

	if (!sc)
		return;

	sc8551_enable_adc(sc, false);
	if (sc->irq) {
		disable_irq(sc->irq);
		synchronize_irq(sc->irq);
	}
}

static int sc8551_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8551 *sc = i2c_get_clientdata(client);

	if (!sc)
		return 0;

	if (device_may_wakeup(dev))
		enable_irq_wake(sc->irq);

	pr_info("[sc8551_chg]: %s: Suspend successfully!\n", __func__);
	return 0;
}

static int sc8551_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8551 *sc = i2c_get_clientdata(client);

	if (!sc)
		return 0;

	if (device_may_wakeup(dev))
		disable_irq_wake(sc->irq);

	if (sc->fc2_psy)
		power_supply_changed(sc->fc2_psy);

	pr_info("[sc8551_chg]: %s: Resume successfully!\n", __func__);
	return 0;
}

static const struct dev_pm_ops sc8551_pm_ops = {
	.suspend	= sc8551_suspend,
	.resume		= sc8551_resume,
};

static const struct i2c_device_id sc8551_charger_id[] = {
	{ "sc8551-standalone", SC8551_ROLE_STANDALONE },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc8551_charger_id);

static struct i2c_driver sc8551_charger_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "sc8551-charger",
		.of_match_table	= of_match_ptr(sc8551_charger_match_table),
		.pm		= &sc8551_pm_ops,
	},
	.id_table	= sc8551_charger_id,
	.probe		= sc8551_charger_probe,
	.remove		= sc8551_charger_remove,
	.shutdown	= sc8551_charger_shutdown,
};

module_i2c_driver(sc8551_charger_driver);

MODULE_DESCRIPTION("SC SC8551 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aiden-yu@southchip.com");
