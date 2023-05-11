// SPDX-License-Identifier: GPL-2.0-only
/*
 * Fuelgauge battery driver
 * Copyright (C) 2018 Siliconmitus
 */

#define pr_fmt(fmt)	"[sm5602_fg]: %s: " fmt, __func__

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
#include <linux/ctype.h>
#include <linux/atomic.h>
#include <linux/param.h>
#include <linux/ratelimit.h>
#include <linux/printk.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/idr.h>
#include <linux/acpi.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/debugfs.h>
#include <linux/regmap.h>
#include <linux/random.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <asm/unaligned.h>
#include "sm5602_fuelgauge.h"

#define INVALID_REG_ADDR	0xFF
#define PROBE_CNT_MAX	50

static const u8 sm5602_regs[SM_FG_REG_COUNT] = {
	0x00, /* DEVICE_ID */
	0x01, /* CNTL */
	0x02, /* INT */
	0x03, /* INT_MASK */
	0x04, /* STATUS */
	0x05, /* SOC */
	0x06, /* OCV */
	0x07, /* VOLTAGE */
	0x08, /* CURRENT */
	0x09, /* TEMPERATURE_IN */
	0x0A, /* TEMPERATURE_EX */
	0x0C, /* V_L_ALARM */
	0x0D, /* V_H_ALARM */
	0x0E, /* A_H_ALARM */
	0x0F, /* T_IN_H_ALARM */
	0x10, /* SOC_L_ALARM */
	0x11, /* FG_OP_STATUS */
	0x12, /* TOPOFFSOC */
	0x13, /* PARAM_CTRL */
	0x14, /* SHUTDOWN */
	0x1A, /* VIT_PERIOD */
	0x1B, /* CURRENT_RATE */
	0x62, /* BAT_CAP */
	0x73, /* CURR_OFFSET */
	0x74, /* CURR_SLOPE */
	0x90, /* MISC */
	0x91, /* RESET */
	0x95, /* RSNS_SEL */
	0x96, /* VOL_COMP */
};

static const char * const device2str[] = {
	"sm5602",
};

#ifdef ENABLE_NTC_COMPENSATION
#define SCALE_TABLE_LEN 50
#define LUT_TABLE_LEN 249

static const u32 LUT_UV[LUT_TABLE_LEN] = {
	0, 5000, 10000, 15000, 20000,
	25000, 30000, 35000, 40000, 45000,
	50000, 55000, 60000, 65000, 70000,
	75000, 80000, 85000, 90000, 95000,
	100000, 105000, 110000, 115000, 120000,
	125000, 130000, 135000, 140000, 145000,
	150000, 155000, 160000, 165000, 170000,
	175000, 180000, 185000, 190000, 195000,
	200000, 205000, 210000, 215000, 220000,
	225000, 230000, 235000, 240000, 245000,
	250000, 255000, 260000, 265000, 270000,
	275000, 280000, 285000, 290000, 295000,
	300000, 305000, 310000, 315000, 320000,
	325000, 330000, 335000, 340000, 345000,
	350000, 355000, 360000, 365000, 370000,
	375000, 380000, 385000, 390000, 395000,
	400000, 405000, 410000, 415000, 420000,
	425000, 430000, 435000, 440000, 445000,
	450000, 455000, 460000, 465000, 470000,
	475000, 480000, 485000, 490000, 495000,
	500000, 505000, 510000, 515000, 520000,
	525000, 530000, 535000, 540000, 545000,
	550000, 555000, 560000, 565000, 570000,
	575000, 580000, 585000, 590000, 595000,
	600000, 605000, 610000, 615000, 620000,
	625000, 630000, 635000, 640000, 645000,
	650000, 655000, 660000, 665000, 670000,
	675000, 680000, 685000, 690000, 695000,
	700000, 705000, 710000, 715000, 720000,
	725000, 730000, 735000, 740000, 745000,
	750000, 755000, 760000, 765000, 770000,
	775000, 780000, 785000, 790000, 795000,
	800000, 805000, 810000, 815000, 820000,
	825000, 830000, 835000, 840000, 845000,
	850000, 855000, 860000, 865000, 870000,
	875000, 880000, 885000, 890000, 895000,
	900000, 905000, 910000, 915000, 920000,
	925000, 930000, 935000, 940000, 945000,
	950000, 955000, 960000, 965000, 970000,
	975000, 980000, 985000, 990000, 995000,
	1000000, 1005000, 1010000, 1015000, 1020000,
	1025000, 1030000, 1035000, 1040000, 1045000,
	1050000, 1055000, 1060000, 1065000, 1070000,
	1075000, 1080000, 1085000, 1090000, 1095000,
	1100000, 1105000, 1110000, 1115000, 1120000,
	1125000, 1130000, 1135000, 1140000, 1145000,
	1150000, 1155000, 1160000, 1165000, 1170000,
	1175000, 1180000, 1185000, 1190000, 1195000,
	1200000, 1205000, 1210000, 1215000, 1220000,
	1225000, 1230000, 1235000, 1240000
};

static const u16 LUT_ADC[LUT_TABLE_LEN] = {
	0x0D18, 0x0DEF, 0x0ED4, 0x0FB5, 0x109C,
	0x117F, 0x1267, 0x1348, 0x1430, 0x1516,
	0x15FC, 0x16DF, 0x17C2, 0x18AA, 0x198B,
	0x1A70, 0x1B54, 0x1C3C, 0x1D1F, 0x1E02,
	0x1EEC, 0x1FD1, 0x20B2, 0x2198, 0x227D,
	0x235E, 0x2448, 0x2529, 0x2610, 0x26F0,
	0x27DF, 0x28C0, 0x29A7, 0x2A8B, 0x2B6D,
	0x2C53, 0x2D39, 0x2E20, 0x2F04, 0x2FEA,
	0x30CB, 0x31B3, 0x3298, 0x337E, 0x3461,
	0x3547, 0x362C, 0x3712, 0x37F5, 0x38D9,
	0x39C1, 0x3AA5, 0x3B88, 0x3C6F, 0x3D51,
	0x3E3B, 0x3F1E, 0x4003, 0x40E8, 0x41CE,
	0x42B1, 0x4397, 0x447C, 0x4562, 0x4644,
	0x472C, 0x4811, 0x48F3, 0x49D6, 0x4ABE,
	0x4B9F, 0x4C88, 0x4D6B, 0x4E52, 0x4F36,
	0x501A, 0x5101, 0x51E6, 0x52CB, 0x53AE,
	0x5492, 0x5578, 0x565A, 0x5745, 0x5826,
	0x590D, 0x59F2, 0x5AD9, 0x5BBE, 0x5CA1,
	0x5D85, 0x5E6B, 0x5F4E, 0x6036, 0x6118,
	0x61FB, 0x62E4, 0x63C5, 0x64AB, 0x6591,
	0x6678, 0x675C, 0x6840, 0x6925, 0x6A0B,
	0x6AEC, 0x6BD1, 0x6CBA, 0x6D9E, 0x6E85,
	0x6F85, 0x7061, 0x7148, 0x722F, 0x730A,
	0x73F6, 0x74D9, 0x75B0, 0x76A1, 0x778B,
	0x809F, 0x817D, 0x826A, 0x834C, 0x843B,
	0x8520, 0x85F5, 0x86E0, 0x87C5, 0x88B5,
	0x8997, 0x8A76, 0x8B5B, 0x8C3D, 0x8D23,
	0x8E08, 0x8EF4, 0x8FDA, 0x90C0, 0x91A2,
	0x928F, 0x9371, 0x942A, 0x950C, 0x95F7,
	0x96D4, 0x97BD, 0x989E, 0x9950, 0x9A6A,
	0x9B4F, 0x9C31, 0x9D1B, 0x9DFB, 0x9EDB,
	0x9FBF, 0xA0AE, 0xA191, 0xA273, 0xA35B,
	0xA445, 0xA530, 0xA61B, 0xA6FA, 0xA7E5,
	0xA8BB, 0xA9A4, 0xAA9F, 0xAB6F, 0xAC5C,
	0xAD38, 0xAE11, 0xAF2A, 0xAFE9, 0xB0D7,
	0xB1B9, 0xB296, 0xB381, 0xB467, 0xB553,
	0xB637, 0xB717, 0xB802, 0xB8E4, 0xB9CA,
	0xBAAD, 0xBB91, 0xBC79, 0xBD5B, 0xBE47,
	0xBF20, 0xC00B, 0xC0F0, 0xC1DB, 0xC2C5,
	0xC3A0, 0xC47F, 0xC566, 0xC653, 0xC72D,
	0xC815, 0xC8F0, 0xC9E1, 0xCACA, 0xCBA8,
	0xCC8D, 0xCD76, 0xCE5E, 0xCF36, 0xD021,
	0xD110, 0xD1F2, 0xD2D3, 0xD3BB, 0xD4AA,
	0xD58D, 0xD671, 0xD759, 0xD83B, 0xD926,
	0xDA0E, 0xDAEF, 0xDBD5, 0xDCBD, 0xDD9C,
	0xDE7E, 0xDF67, 0xE04B, 0xE12D, 0xE210,
	0xE2FB, 0xE3E2, 0xE4CE, 0xE5A5, 0xE68E,
	0xE775, 0xE858, 0xE935, 0xEA22, 0xEB05,
	0xEBEF, 0xECCB, 0xEDD5, 0xEE9E, 0xEF6F,
	0xF050, 0xF127, 0xF208, 0xF2E7
};

static const u32 scale_thresholds_ma[SCALE_TABLE_LEN] = {
	0, 122, 245, 367, 490, 612, 735, 857, 980, 1102,
	1224, 1347, 1469, 1592, 1714, 1837, 1959, 2082, 2204, 2327,
	2449, 2571, 2694, 2816, 2939, 3061, 3184, 3306, 3429, 3551,
	3673, 3796, 3918, 4041, 4163, 4286, 4408, 4531, 4653, 4776,
	4898, 5020, 5143, 5265, 5388, 5510, 5633, 5755, 5878, 6000
};

static const u32 scale_x1000_table[SCALE_TABLE_LEN] = {
	850, 855, 860, 865, 870, 876, 881, 886, 891, 896,
	901, 906, 911, 916, 921, 927, 932, 937, 942, 947,
	952, 957, 962, 967, 972, 978, 983, 988, 993, 998,
	1003, 1008, 1013, 1018, 1023, 1029, 1034, 1039, 1044, 1049,
	1054, 1059, 1064, 1069, 1074, 1080, 1085, 1090, 1095, 1100
};
#endif /* ENABLE_NTC_COMPENSATION */

#define sm_log(fmt, ...)							\
do {										\
	if (sm && __ratelimit(&sm->rl_slots[_SM_ID() & (SM_RL_SLOTS - 1)]))	\
		pr_info(fmt, ##__VA_ARGS__);					\
} while (0)

#define sm_err(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define sm_info(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define sm_dbg(fmt, ...)	pr_debug(fmt, ##__VA_ARGS__)

static const struct regmap_config sm5602_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 16,
	.val_format_endian	= REGMAP_ENDIAN_LITTLE,
	.max_register	= 0xE0,
};

static bool fg_init(struct sm_fg_chip *sm);
static bool fg_reg_init(struct sm_fg_chip *sm);
static bool fg_check_reg_init_need(struct sm_fg_chip *sm);
static int fg_read_current(struct sm_fg_chip *sm);
static void fg_monitor_workfunc(struct kthread_work *work);

static int fg_read_word(struct sm_fg_chip *sm, u8 reg, u16 *val)
{
	int ret;
	unsigned int rval;

	if (sm->skip_reads) {
		*val = 0;
		return 0;
	}

	ret = regmap_read(sm->regmap, reg, &rval);
	if (ret < 0) {
		sm_err("read 0x%02x failed: %d\n", reg, ret);
		*val = 0;
		return ret;
	}

	*val = (u16)rval;
	return 0;
}

static int fg_write_word(struct sm_fg_chip *sm, u8 reg, u16 val)
{
	int ret;

	if (sm->skip_writes)
		return 0;

	ret = regmap_write(sm->regmap, reg, val);
	if (ret < 0)
		sm_err("write 0x%04x->0x%02x failed: %d\n", val, reg, ret);

	return ret;
}

static int fg_update_bits(struct sm_fg_chip *sm, u8 reg, u16 mask, u16 val)
{
	int ret;

	if (sm->skip_reads || sm->skip_writes)
		return 0;

	ret = regmap_update_bits(sm->regmap, reg, mask, val);
	if (ret < 0)
		sm_err("update_bits 0x%02x (mask=0x%04x val=0x%04x) failed: %d\n",
		       reg, mask, val, ret);

	return ret;
}

#define FG_STATUS_SLEEP			BIT(10)
#define FG_STATUS_BATT_PRESENT		BIT(9)
#define FG_STATUS_SOC_UPDATE		BIT(8)
#define FG_STATUS_TOPOFF		BIT(7)
#define FG_STATUS_LOW_SOC2		BIT(6)
#define FG_STATUS_LOW_SOC1		BIT(5)
#define FG_STATUS_HIGH_CURRENT		BIT(4)
#define FG_STATUS_HIGH_TEMPERATURE	BIT(3)
#define FG_STATUS_LOW_TEMPERATURE	BIT(2)
#define FG_STATUS_HIGH_VOLTAGE		BIT(1)
#define FG_STATUS_LOW_VOLTAGE		BIT(0)
#define FG_OP_STATUS_CHG_DISCHG		BIT(15)

static int fg_read_status(struct sm_fg_chip *sm)
{
	int ret;
	u16 flags1, flags2;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_STATUS], &flags1);
	if (ret < 0)
		return ret;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &flags2);
	if (ret < 0)
		return ret;

	mutex_lock(&sm->data_lock);
	sm->batt_present	= !!(flags1 & FG_STATUS_BATT_PRESENT);
	sm->batt_ot		= !!(flags1 & FG_STATUS_HIGH_TEMPERATURE);
	sm->batt_ut		= !!(flags1 & FG_STATUS_LOW_TEMPERATURE);
	sm->batt_fc		= !!(flags1 & FG_STATUS_TOPOFF);
	sm->batt_soc1		= !!(flags1 & FG_STATUS_LOW_SOC2);
	sm->batt_socp		= !!(flags1 & FG_STATUS_LOW_SOC1);
	sm->batt_dsg		= !(flags2 & FG_OP_STATUS_CHG_DISCHG);
	mutex_unlock(&sm->data_lock);

	return 0;
}

static bool do_update(struct sm_fg_chip *sm,
		      enum fg_update_idx idx,
		      unsigned long interval_ms)
{
	ktime_t now = ktime_get_boottime();
	s64 old = atomic64_read(&sm->last_update_time[idx]);

	if (old && ktime_ms_delta(now, (ktime_t)old) < interval_ms)
		return false;

	return atomic64_cmpxchg(&sm->last_update_time[idx], old, now) == old;
}


static int fg_read_soc(struct sm_fg_chip *sm)
{
	int ret, soc, raw_soc;
	u32 raw;
	u16 data = 0;

	if (!do_update(sm, FG_UPDATE_SOC, 1000))
		return 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_SOC], &data);
	if (ret < 0) {
		sm_err("Couldn't read SOC, ret=%d\n", ret);
		return ret;
	}

	raw = data & FG_REG_MAG_MASK;
	raw_soc = (raw * 10U) >> 8;

	if (data & FG_REG_SIGN_BIT) {
		sm_err("data: 0x%04x, raw_soc: %d\n", (u32)data, raw_soc);
		raw_soc = -raw_soc;
	}

	soc = clamp(raw_soc, 0, 1000);

	mutex_lock(&sm->data_lock);
	sm->batt_soc = soc;
	mutex_unlock(&sm->data_lock);

	return 0;
}

static int fg_get_soc_decimal(struct sm_fg_chip *sm)
{
	int batt_soc;

	mutex_lock(&sm->data_lock);
	batt_soc = sm->batt_soc;
	mutex_unlock(&sm->data_lock);

	return batt_soc % 100;
}

static int fg_get_soc_decimal_rate(struct sm_fg_chip *sm)
{
	int soc, i;

	if (!sm->dec_rate_seq || sm->dec_rate_len < 2)
		return 0;

	mutex_lock(&sm->data_lock);
	soc = sm->batt_soc / 10;
	mutex_unlock(&sm->data_lock);
	if (soc < sm->dec_rate_seq[0])
		return sm->dec_rate_seq[1];

	for (i = 2; i < sm->dec_rate_len; i += 2) {
		if (soc < sm->dec_rate_seq[i])
			return sm->dec_rate_seq[i - 1];
	}

	return sm->dec_rate_seq[sm->dec_rate_len - 1];
}

static int fg_read_ocv(struct sm_fg_chip *sm)
{
	int ret;
	u32 low, high, ocv;
	u16 data = 0;

	if (!do_update(sm, FG_UPDATE_OCV, 1000))
		return 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_OCV], &data);
	if (ret < 0) {
		sm_err("Couldn't read OCV, ret=%d\n", ret);
		return ret;
	}

	low = data & FG_REG_OCV_LOW_MASK;
	high = data >> 12;
	ocv = ((low * 1000U) >> 11) + (high * 2000U);

	mutex_lock(&sm->data_lock);
	sm->batt_ocv = ocv;
	mutex_unlock(&sm->data_lock);

	return 0;
}

#ifdef ENABLE_NTC_COMPENSATION
static u32 adc_to_uv(int len, u16 adc_code, const u16 *adc_table, const u32 *uv_table)
{
	int i;

	if (adc_code < adc_table[0])
		return uv_table[0];
	if (adc_code > adc_table[len - 1])
		return uv_table[len - 1];

	for (i = 0; i < len - 1; i++) {
		if (adc_code >= adc_table[i] && adc_code <= adc_table[i + 1])
			break;
	}

	if (i >= len - 1)
		return uv_table[len - 1];
	if (adc_table[i + 1] == adc_table[i])
		return uv_table[i];

	return uv_table[i] + (u32)div_u64((u64)(uv_table[i + 1] - uv_table[i]) * (u64)(adc_code - adc_table[i]), (adc_table[i + 1] - adc_table[i]));
}

static u16 uv_to_adc(int len, u32 meas_uv, const u32 *uv_table, const u16 *adc_table)
{
	int i;

	if (meas_uv < uv_table[0])
		return adc_table[0];
	if (meas_uv > uv_table[len - 1])
		return adc_table[len - 1];

	for (i = 0; i < len - 1; i++) {
		if (meas_uv >= uv_table[i] && meas_uv <= uv_table[i + 1])
			break;
	}

	if (i >= len - 1)
		return adc_table[len - 1];
	if (uv_table[i + 1] == uv_table[i])
		return adc_table[i];

	return adc_table[i] + (u16)div_u64((u64)(adc_table[i + 1] - adc_table[i]) * (u64)(meas_uv - uv_table[i]), (uv_table[i + 1] - uv_table[i]));
}

static u32 ir_drop_uv(u32 curr_ma, u32 rtrace_uohm)
{
	return (u32)div_u64((u64)curr_ma * (u64)rtrace_uohm, 1000ULL);
}

static u32 get_scale_x1000(u32 curr_mA)
{
	int i;

	for (i = SCALE_TABLE_LEN - 1; i >= 0; --i) {
		if (curr_mA >= scale_thresholds_ma[i])
			return scale_x1000_table[i];
	}

	return 0;
}

static u16 compensate_ntc(struct sm_fg_chip *sm, u16 raw_adc)
{
	int len = sizeof(LUT_UV) / sizeof(u32);
	u32 curr = (u32)abs(sm->batt_curr);
	u32 rtrace = sm->rtrace;
	u32 meas_uv, corrected_uv;
	u32 scale_x1000_val;
	u64 ir_uv, corr_candidate, corr_prev;
	const u64 max_step_uv = 10000ULL;

	meas_uv = adc_to_uv(len, raw_adc, LUT_ADC, LUT_UV);
	sm_dbg("DBG_COMP: raw_adc=0x%04x, meas_uV=%u, curr=%u, rtrace=%u\n",
	       (u32)raw_adc, meas_uv, curr, rtrace);

	ir_uv = ir_drop_uv(curr, rtrace);
	scale_x1000_val = get_scale_x1000(curr);
	corr_candidate = div_u64(ir_uv * (u64)scale_x1000_val, 1000ULL);

	corr_prev = sm->prev_corr_uv;
	if (corr_candidate > corr_prev) {
		u64 delta = corr_candidate - corr_prev;
		if (delta > max_step_uv)
			corr_candidate = corr_prev + max_step_uv;
	} else if (corr_prev > corr_candidate) {
		u64 delta = corr_prev - corr_candidate;
		if (delta > max_step_uv)
			corr_candidate = corr_prev - max_step_uv;
	}

	sm->prev_corr_uv = corr_candidate;

	/* Charging (batt_curr >= 0): subtract IR drop, Discharging: add IR drop */
	if (sm->batt_curr >= 0) {
		corrected_uv = meas_uv > (u32)corr_candidate ?
			       meas_uv - (u32)corr_candidate : 0;
	} else {
		corrected_uv = meas_uv + (u32)corr_candidate;
	}

	sm_dbg("DBG_COMP: ir_uv=%llu, scale=%u, corr_uv=%llu, temp_uV_after_corr=%u, corr_mV=%llu\n",
	       ir_uv, scale_x1000_val, corr_candidate,
	       corrected_uv, div_u64(corr_candidate, 1000ULL));

	return uv_to_adc(len, corrected_uv, LUT_UV, LUT_ADC);
}
#endif

static int __calculate_battery_temp_ex(struct sm_fg_chip *sm, u16 uval)
{
	int i, temp;
	u16 val = 0;

	if (uval >= FG_TEMP_EX_INVALID_LOW && uval <= FG_TEMP_EX_INVALID_HIGH) {
		sm_err("sp_range uval = 0x%x\n", uval);
		return sm->batt_temp;
	}

	/* Transform raw register value (domain A) to table domain (domain B = XOR 0x8000). */
	val = uval ^ 0x8000;

#ifdef ENABLE_NTC_COMPENSATION
	sm_info("DBG_COMP: NTC_COMP=ON, rtrace_cfg=%u\n", sm->rtrace);
	val = compensate_ntc(sm, val);
#else
	sm_info("DBG_COMP: NTC_COMP=OFF\n");
#endif

	if (val >= sm->battery_temp_table[0]) { /* NTC table should be sorted descending */
		temp = EX_TEMP_MIN * 10;
	} else if (val <= sm->battery_temp_table[FG_TEMP_TABLE_CNT_MAX - 1]) {
		temp = EX_TEMP_MAX * 10;
	} else {
		temp = EX_TEMP_MAX * 10;
		for (i = 0; i < FG_TEMP_TABLE_CNT_MAX; i++) {
			if (val >= sm->battery_temp_table[i]) {
				temp = (EX_TEMP_MIN * 10) + i;
				break;
			}
		}
	}

#ifdef ENABLE_NTC_COMPENSATION
	sm_info("DBG_COMP: raw_uval=0x%04x, final_val=0x%04x, temp=%d\n",
		(u32)uval, (u32)val, temp);
#else
	sm_info("DBG_COMP: (NTC OFF) raw_uval=0x%04x, final_val=0x%04x, temp=%d\n",
		(u32)uval, (u32)val, temp);
#endif

	return temp;
}

#ifdef ENABLE_TEMP_AVG
#define MIN_TEMP_SAMPLE_WINDOW	5
#define MAX_TEMP_SAMPLE_WINDOW	BATT_TEMP_AVG_SAMPLES
static void calculate_average_temperature(struct sm_fg_chip *sm)
{
	int curr = sm->param.batt_temp;
	int prev = sm->param.batt_temp_prev;
	int delta, abs_delta;
	int sum = 0, count = 0;
	int i, idx;

	if (curr == -EINVAL)
		return;

	if (sm->param.batt_temp_avg == -EINVAL) {
		sm->param.batt_temp_avg = curr;
		sm->param.batt_temp_prev = curr;
		sm->param.batt_temp_samples_index = 0;
		sm->param.batt_temp_samples_num = 1;
		sm->param.batt_temp_avg_samples[0] = curr;
		return;
	}

	delta = curr - prev;
	abs_delta = abs(delta);
	sm->param.batt_temp_prev = curr;

	sm->param.batt_temp_avg_samples[sm->param.batt_temp_samples_index] = curr;
	sm->param.batt_temp_samples_index =
		(sm->param.batt_temp_samples_index + 1) % BATT_TEMP_AVG_SAMPLES;

	if (sm->param.batt_temp_samples_num < BATT_TEMP_AVG_SAMPLES)
		sm->param.batt_temp_samples_num++;

	if (sm->param.batt_temp_samples_num < MIN_TEMP_SAMPLE_WINDOW) {
		sm->param.batt_temp_avg = curr;
		return;
	}

	if (abs_delta <= 5)
		count = MIN_TEMP_SAMPLE_WINDOW;
	else
		count = MAX_TEMP_SAMPLE_WINDOW;

	if (count > sm->param.batt_temp_samples_num)
		count = sm->param.batt_temp_samples_num;

	for (i = 0; i < count; i++) {
		idx = (sm->param.batt_temp_samples_index - 1 - i + BATT_TEMP_AVG_SAMPLES) % BATT_TEMP_AVG_SAMPLES;
		sum += sm->param.batt_temp_avg_samples[idx];
	}

	sm->param.batt_temp_avg = DIV_ROUND_CLOSEST(sum, count);

	sm_log("raw_temp: %d, avg_temp: %d, delta: %d, abs: %d, count: %d\n",
	       curr, sm->param.batt_temp_avg, delta, abs_delta, count);
}
#endif

static int fg_read_temperature(struct sm_fg_chip *sm,
			       enum sm_fg_temperature_type temperature_type)
{
	int ret;
	int temp;
	u16 data = 0;

	if (!do_update(sm, FG_UPDATE_TEMP, 1000))
		return 0;

	switch (temperature_type) {
	case TEMPERATURE_IN:
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_IN], &data);
		if (ret < 0) {
			sm_err("Couldn't read TEMP_IN, ret=%d\n", ret);
			return ret;
		}

		temp = data & FG_REG_TEMP_IN_MAG_MASK;
		if (data & FG_REG_SIGN_BIT)
			temp = -temp;

		sm_log("temp_in: %d\n", temp);
		break;
	case TEMPERATURE_EX:
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_TEMPERATURE_EX], &data);
		if (ret < 0) {
			sm_err("Couldn't read TEMP_EX, ret=%d\n", ret);
			return ret;
		}

		temp = __calculate_battery_temp_ex(sm, data);
		if (temp >= 600) {
			sm_err("temp >= 60 exceeded, schedule overtemp timer\n");
			if (!sm->overtemp_allow_restart) {
				temp = 600;
				if (!sm->overtemp_delay_on) {
					sm->overtemp_delay_on = true;
					mod_timer(&sm->overtemp_timer,
						  jiffies + 20 * HZ);
				}
			}
		} else if (temp < 600 && sm->overtemp_delay_on) {
			sm_err("temp is < 60, cleanup overtemp timer\n");
			del_timer_sync(&sm->overtemp_timer);
			sm->overtemp_delay_on = false;
			sm->overtemp_allow_restart = false;
		}

		sm_log("temp_ex: %d\n", temp);
		break;
	default:
		return -ENODATA;
	}

	mutex_lock(&sm->data_lock);
	sm->batt_temp = temp;
#ifdef ENABLE_TEMP_AVG
	sm->param.batt_temp = temp;
	calculate_average_temperature(sm);
#endif
	mutex_unlock(&sm->data_lock);

	return 0;
}

static int fg_read_volt(struct sm_fg_chip *sm)
{
	int ret, volt, scaled;
	u32 raw;
	u16 data = 0;

	if (!do_update(sm, FG_UPDATE_VOLT, 1000))
		return 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_VOLTAGE], &data);
	if (ret < 0) {
		sm_err("Couldn't read VOLT, ret=%d\n", ret);
		return ret;
	}

	raw = data & FG_REG_MAG_MASK;
	scaled = (raw * 94U) >> 10;

	if (data & FG_REG_SIGN_BIT)
		scaled = -scaled;

	volt = scaled + 2700;

	mutex_lock(&sm->data_lock);
	sm->batt_volt = volt;
	mutex_unlock(&sm->data_lock);

	return 0;
}

static int fg_get_cycle(struct sm_fg_chip *sm)
{
	int ret, cycle;
	u16 data = 0;

	if (!do_update(sm, FG_UPDATE_CYCLE, 1000))
		return 0;

	ret = fg_read_word(sm, FG_REG_SOC_CYCLE, &data);
	if (ret < 0) {
		sm_err("Couldn't read CYCLE, ret=%d\n", ret);
		return ret;
	}

	cycle = data & FG_REG_CYCLE_MASK;

	mutex_lock(&sm->data_lock);
	sm->batt_soc_cycle = cycle;
	mutex_unlock(&sm->data_lock);

	return 0;
}

#ifdef ENABLE_CURRENT_AVG
#define MIN_CURRENT_SAMPLE_WINDOW	3
#define MAX_CURRENT_SAMPLE_WINDOW	BATT_MA_AVG_SAMPLES
#define CURRENT_THRESHOLD_HIGH		4500
static void calculate_average_current(struct sm_fg_chip *sm)
{
	int curr = sm->param.batt_ma;
	int prev = sm->param.batt_ma_prev;
	int delta, abs_delta;
	int sum = 0;
	int count = 0;
	int i, idx;

	if (sm->param.batt_ma_avg == -EINVAL ||
	    (curr > 0 && sm->param.batt_ma_avg < 0) ||
	    (curr < 0 && sm->param.batt_ma_avg > 0)) {
		sm->param.batt_ma_avg = curr;
		sm->param.batt_ma_prev = curr;
		sm->param.batt_ma_samples_index = 0;
		sm->param.batt_ma_samples_num = 1;
		sm->param.batt_ma_avg_samples[0] = curr;
		return;
	}

	delta = curr - prev;
	abs_delta = abs(delta);
	sm->param.batt_ma_prev = curr;

	sm->param.batt_ma_avg_samples[sm->param.batt_ma_samples_index] = curr;
	sm->param.batt_ma_samples_index =
		(sm->param.batt_ma_samples_index + 1) % BATT_MA_AVG_SAMPLES;

	if (sm->param.batt_ma_samples_num < BATT_MA_AVG_SAMPLES)
		sm->param.batt_ma_samples_num++;

	if (sm->param.batt_ma_samples_num < MIN_CURRENT_SAMPLE_WINDOW) {
		sm->param.batt_ma_avg = curr;
		return;
	}

	if (curr <= CURRENT_THRESHOLD_HIGH)
		count = MIN_CURRENT_SAMPLE_WINDOW;
	else
		count = MAX_CURRENT_SAMPLE_WINDOW;

	if (count > sm->param.batt_ma_samples_num)
		count = sm->param.batt_ma_samples_num;

	if (count <= 0)
		return;

	for (i = 0; i < count; i++) {
		idx = (sm->param.batt_ma_samples_index - 1 - i + BATT_MA_AVG_SAMPLES) % BATT_MA_AVG_SAMPLES;
		sum += sm->param.batt_ma_avg_samples[idx];
	}

	if (sum >= 0)
		sm->param.batt_ma_avg = (sum + count / 2) / count;
	else
		sm->param.batt_ma_avg = (sum - count / 2) / count;

	sm_log("raw_ma: %d, avg_ma: %d, delta: %d, abs: %d, count: %d\n",
	       curr, sm->param.batt_ma_avg, delta, abs_delta, count);
}
#endif

static int fg_read_current(struct sm_fg_chip *sm)
{
	int ret, curr;
	unsigned int rsns;
	u16 data = 0;
	u32 raw;

	if (!do_update(sm, FG_UPDATE_CURRENT, 1000))
		return 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_CURRENT], &data);
	if (ret < 0) {
		sm_err("Couldn't read CURRENT, ret=%d\n", ret);
		return ret;
	}

	if (sm->batt_rsns == -EINVAL) {
		sm_err("Couldn't read batt_rsns, force 10 mohm\n");
		rsns = 10;
	} else {
		rsns = (sm->batt_rsns == 0) ? 5 : (u32)(sm->batt_rsns * 10);
	}

	if (rsns == 0) {
		sm_err("invalid shunt (%u mohm)\n", rsns);
		return -EINVAL;
	}

	raw = data & FG_REG_MAG_MASK;
	curr = (int)((((raw * 10000U) + 2048U) >> 12) / rsns);

	if (data & FG_REG_SIGN_BIT)
		curr = -curr;

	mutex_lock(&sm->data_lock);
	sm->batt_curr = curr;
#ifdef ENABLE_CURRENT_AVG
	sm->param.batt_ma = curr;
	calculate_average_current(sm);
#endif
	mutex_unlock(&sm->data_lock);

	return 0;
}

static int fg_read_fcc(struct sm_fg_chip *sm)
{
	int ret, fcc;
	u32 raw;
	u16 data = 0;

	if (!do_update(sm, FG_UPDATE_FCC, 1000))
		return 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_BAT_CAP], &data);
	if (ret < 0) {
		sm_err("Couldn't read FCC, ret=%d\n", ret);
		return ret;
	}

	raw = data & FG_REG_MAG_MASK;
	fcc = (raw * 125U) >> 8;

	mutex_lock(&sm->data_lock);
	sm->batt_fcc = fcc;
	mutex_unlock(&sm->data_lock);

	return 0;
}

static int fg_read_rmc(struct sm_fg_chip *sm)
{
	int ret, rmc;
	u32 raw;
	u16 data = 0;

	if (!do_update(sm, FG_UPDATE_RMC, 1000))
		return 0;

	ret = fg_read_word(sm, FG_REG_RMC, &data);
	if (ret < 0) {
		sm_err("Couldn't read RMC, ret=%d\n", ret);
		return ret;
	}

	raw = data & FG_REG_MAG_MASK;
	rmc = (raw * 125U) >> 8;

	mutex_lock(&sm->data_lock);
	sm->batt_rmc = rmc;
	mutex_unlock(&sm->data_lock);

	return 0;
}

#define FG_SOFT_RESET	0x1A6
static int fg_reset(struct sm_fg_chip *sm)
{
	int ret;

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_RESET], FG_SOFT_RESET);
	if (ret < 0) {
		sm_err("Couldn't reset, ret=%d\n", ret);
		return ret;
	}
	msleep(800);

	return 0;
}

static int get_battery_status(struct sm_fg_chip *sm)
{
	union power_supply_propval pval = {0, };
	int ret;

	if (!sm->batt_psy)
		sm->batt_psy = power_supply_get_by_name("battery");
	if (sm->batt_psy) {
		ret = power_supply_get_property(sm->batt_psy,
					       POWER_SUPPLY_PROP_STATUS,
					       &pval);
		if (ret) {
			sm_err("battery does not export status (%d)\n", ret);
			return POWER_SUPPLY_STATUS_UNKNOWN;
		}
		return pval.intval;
	}

	sm_err("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static bool is_battery_charging(struct sm_fg_chip *sm)
{
	return get_battery_status(sm) == POWER_SUPPLY_STATUS_CHARGING;
}

static int fg_param_unlock(struct sm_fg_chip *sm)
{
	int ret;
	u16 val = FG_PARAM_UNLOCK_CODE |
		  ((sm->battery_table_num & 0x0003) << 6) |
		  (FG_TABLE_LEN - 1);

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL], val);
	if (ret < 0)
		sm_err("Failed to unlock param window, ret=%d\n", ret);

	return ret;
}

static int fg_param_lock(struct sm_fg_chip *sm)
{
	int ret;
	u16 val = FG_PARAM_LOCK_CODE |
		  ((sm->battery_table_num & 0x0003) << 6) |
		  (FG_TABLE_LEN - 1);

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL], val);
	if (ret < 0)
		sm_err("Failed to re-lock param window, ret=%d\n", ret);

	return ret;
}

#ifdef ENABLE_TEMBASE_ZDSCON
static void fg_tembase_zdscon(struct sm_fg_chip *sm)
{
	u16 data = 0;
	int hminman_value, ret;
	int fg_temp_gap = sm->batt_temp - sm->temp_std;
	int abs_gap;
	int tmp;

	if (fg_temp_gap >= 0)
		return;

	abs_gap = -fg_temp_gap;

	ret = fg_read_word(sm, FG_REG_RS_3, &data);
	if (ret < 0) {
		sm_err("Couldn't read rs_3 reg, ret=%d\n", ret);
		return;
	}

	if (abs_gap <= ZDSCON_ACT_TEMP_GAP || sm->is_charging) {
		if (data != sm->rs_value[3]) {
			ret = fg_param_unlock(sm);
			if (ret < 0)
				return;

			ret = fg_write_word(sm, FG_REG_RS_3, sm->rs_value[3]);
			if (ret < 0) {
				sm_err("Couldn't write rs_3 reg, ret=%d\n", ret);
				fg_param_lock(sm);
				return;
			}

			fg_param_lock(sm);
		}

		ret = fg_read_word(sm, FG_REG_RS_0, &data);
		if (ret < 0 || data != sm->rs_value[0]) {
			ret = fg_param_unlock(sm);
			if (ret < 0)
				return;

			ret = fg_write_word(sm, FG_REG_RS_0, sm->rs_value[0]);
			if (ret < 0)
				sm_err("Couldn't write rs_0 reg, ret=%d\n", ret);

			fg_param_lock(sm);
		}

		sm_info("hminman restore to 0x%x, temp(%d)\n",
			sm->rs_value[3], sm->batt_temp);
		return;
	}

	tmp = (int)sm->rs_value[3] + ((int)(abs_gap - ZDSCON_ACT_TEMP_GAP) * HMINMAN_T_VALUE_FACT) / T_GAP_DENOM;
	tmp -= ((int)abs(sm->batt_curr) * HMINMAN_I_VALUE_FACT) / I_GAP_DENOM;

	tmp = clamp(tmp, (int)sm->rs_value[3], 0xFFFF);
	hminman_value = tmp;

	if (data == (u16)hminman_value)
		return;

	ret = fg_param_unlock(sm);
	if (ret < 0)
		return;

	ret = fg_write_word(sm, FG_REG_RS_3, (u16)hminman_value);
	if (ret < 0) {
		sm_err("Couldn't write rs_3 reg, ret=%d\n", ret);
		fg_param_lock(sm);
		return;
	}

	ret = fg_read_word(sm, FG_REG_RS_0, &data);
	if (ret < 0 || data != (u16)(hminman_value + 2)) {
		ret = fg_write_word(sm, FG_REG_RS_0, (u16)(hminman_value + 2));
		if (ret < 0)
			sm_err("Couldn't write rs_0 reg, ret=%d\n", ret);
	}

	fg_param_lock(sm);

	sm_info("hminman set 0x%x -> 0x%x, temp(%d) curr(%d)\n",
		data, hminman_value, sm->batt_temp, sm->batt_curr);
}
#endif

static void fg_vbatocv_check(struct sm_fg_chip *sm)
{
	int topoff = sm->topoff;
	int topoff_margin = sm->topoff_margin;
	int high_bound, low_bound;
	int rs0 = 0, rs2 = 0;
	u16 data0 = 0, data2 = 0;
	u16 rs2_target;

	if (sm->fast_mode) {
		high_bound = topoff * 3 + topoff_margin;
		low_bound = topoff - topoff_margin;
	} else {
		high_bound = topoff + topoff_margin;
		low_bound = topoff - (topoff_margin * 2);
	}

	sm_info("fast_mode: %d, high_bound: %d, low_bound: %d\n",
		sm->fast_mode, high_bound, low_bound);

	rs0 = fg_read_word(sm, FG_REG_RS_0, &data0);
	if (rs0 < 0) {
		sm_err("Couldn't read rs_0 reg (%d)\n", rs0);
		return;
	}

	rs2 = fg_read_word(sm, FG_REG_RS_2, &data2);
	if (rs2 < 0) {
		sm_err("Couldn't read rs_2 reg (%d)\n", rs2);
		return;
	}

#ifdef ENABLE_VLCM_MODE
	if ((abs(sm->batt_curr) < 50) ||
	    (sm->is_charging &&
	    sm->batt_curr < high_bound &&
	    sm->batt_curr > low_bound &&
	    sm->batt_soc >= SM_RAW_SOC_FULL)) {
#else
	if (sm->is_charging &&
	    sm->batt_curr < high_bound &&
	    sm->batt_curr > low_bound &&
	    sm->batt_soc >= SM_RAW_SOC_FULL) {
#endif
		if (abs(sm->batt_ocv - sm->batt_volt) > 30)
			sm->iocv_error_count = min(sm->iocv_error_count + 1, 6);

		sm_info("iocv_error_count: %d\n", sm->iocv_error_count);
	} else {
		sm->iocv_error_count = 0;
	}

	if (sm->iocv_error_count > 5) {
		sm_info("p_v - v = (%d)\n", sm->p_batt_voltage - sm->batt_volt);
		if (abs(sm->p_batt_voltage - sm->batt_volt) > 15) {
			sm->iocv_error_count = 0;
			goto update_prev;
		}
		rs2_target = data0;
		sm_info("mode change to RS m mode 0x%x\n", rs2_target);
	} else if (sm->p_batt_voltage < sm->n_tempoff &&
		    sm->batt_volt < sm->n_tempoff &&
		    !sm->is_charging) {
		if (sm->p_batt_voltage < (sm->n_tempoff - sm->n_tempoff_offset) &&
		    sm->batt_volt < (sm->n_tempoff - sm->n_tempoff_offset)) {
			rs2_target = (u16)(data0 >> 1);
			sm_info("mode change to normal temp RS m mode >> 1 0x%x\n", rs2_target);
		} else {
			rs2_target = data0;
			sm_info("mode change to normal temp RS m mode 0x%x\n", rs2_target);
		}
	} else {
		rs2_target = sm->rs_value[2];
	}

	if (rs2 < 0 || data2 != rs2_target) {
		sm_info("mode change to RS a mode\n");

		if (!fg_param_unlock(sm)) {
			fg_write_word(sm, FG_REG_RS_2, rs2_target);
			fg_param_lock(sm);
		}
	}

update_prev:
	sm->p_batt_voltage = sm->batt_volt;
	sm->p_batt_current = sm->batt_curr;
}

static int fg_cal_carc(struct sm_fg_chip *sm)
{
	int ret;
	int curr_cal = 0, p_curr_cal = 0, n_curr_cal = 0;
	int p_delta_cal = 0, n_delta_cal = 0;
	int p_fg_delta_cal = 0, n_fg_delta_cal = 0;
	int temp_curr_offset = 0, temp_gap = 0, fg_temp_gap = 0;
	int base = 0, gap = 0;
	u16 tmp = 0;
#ifdef ENABLE_MIX_COMP
	u16 temp_aging_ctrl = 0;
#endif
	union power_supply_propval pval = {0, };

	sm->is_charging = is_battery_charging(sm);
	if (!sm->cp_psy)
		sm->cp_psy = power_supply_get_by_name("sc8551-standalone");
	if (sm->cp_psy) {
		ret = power_supply_get_property(sm->cp_psy,
						POWER_SUPPLY_PROP_CHARGING_ENABLED,
						&pval);
		if (ret < 0)
			sm_err("get sc8551_psy charge property error: %d\n", ret);
		else
			sm->cp_work_flag = !!pval.intval;
	} else {
		sm_err("sc8551_psy not found\n");
	}
#ifdef ENABLE_TEMBASE_ZDSCON
	fg_tembase_zdscon(sm);
#endif
	fg_vbatocv_check(sm);

	fg_temp_gap = sm->batt_temp - sm->temp_std;

	temp_curr_offset = sm->curr_offset & 0x00FF;
	base = (temp_curr_offset & 0x0080) ? -(temp_curr_offset & 0x007F) : temp_curr_offset;

	if (sm->en_high_fg_temp_offset && fg_temp_gap > 0)
		base += (fg_temp_gap / (sm->high_fg_temp_offset_denom * 10)) * sm->high_fg_temp_offset_fact;
	else if (sm->en_low_fg_temp_offset && fg_temp_gap < 0)
		base += ((-fg_temp_gap) / (sm->low_fg_temp_offset_denom * 10)) * sm->low_fg_temp_offset_fact;

	base = clamp(base, -127, 127);
	if (base < 0)
		base = ((-base) & 0x7F) | 0x0080;
	else
		base &= 0x00FF;

	tmp = (u16)((base & 0x00FF) | ((base & 0x00FF) << 8));
	n_curr_cal = (sm->curr_slope & 0xFF00) >> 8;
	p_curr_cal = sm->curr_slope & 0x00FF;

	if (sm->en_high_fg_temp_cal && fg_temp_gap > 0) {
		gap = fg_temp_gap;
		p_fg_delta_cal = (gap / (sm->high_fg_temp_p_cal_denom * 10)) * sm->high_fg_temp_p_cal_fact;
		n_fg_delta_cal = (gap / (sm->high_fg_temp_n_cal_denom * 10)) * sm->high_fg_temp_n_cal_fact;
	} else if (sm->en_low_fg_temp_cal && fg_temp_gap < 0) {
		gap = -fg_temp_gap;
		p_fg_delta_cal = (gap / (sm->low_fg_temp_p_cal_denom * 10)) * sm->low_fg_temp_p_cal_fact;
		n_fg_delta_cal = (gap / (sm->low_fg_temp_n_cal_denom * 10)) * sm->low_fg_temp_n_cal_fact;
	}
	p_curr_cal += p_fg_delta_cal;
	n_curr_cal += n_fg_delta_cal;

	temp_gap = fg_temp_gap;
	if (sm->en_high_temp_cal && temp_gap > 0) {
		gap = temp_gap;
		p_delta_cal = (gap / (sm->high_temp_p_cal_denom * 10)) * sm->high_temp_p_cal_fact;
		n_delta_cal = (gap / (sm->high_temp_n_cal_denom * 10)) * sm->high_temp_n_cal_fact;
	} else if (sm->en_low_temp_cal && (temp_gap < 0)) {
		gap = -temp_gap;
		p_delta_cal = (gap / (sm->low_temp_p_cal_denom * 10)) * sm->low_temp_p_cal_fact;
		n_delta_cal = (gap / (sm->low_temp_n_cal_denom * 10)) * sm->low_temp_n_cal_fact;
	}
	p_curr_cal += p_delta_cal;
	n_curr_cal += n_delta_cal;

	if (sm->fast_mode)
		p_curr_cal += sm->fcm_offset;

	p_curr_cal = clamp(p_curr_cal, 0, 255);
	n_curr_cal = clamp(n_curr_cal, 0, 255);
	curr_cal = ((n_curr_cal & 0xFF) << 8) | (p_curr_cal & 0xFF);

	sm_dbg("curr_slope: base p=0x%02x n=0x%02x, delta_fg p=%d n=%d, delta_ext p=%d n=%d, fast=%d\n",
	       sm->curr_slope & 0xFF, (sm->curr_slope >> 8) & 0xFF,
	       p_fg_delta_cal, n_fg_delta_cal,
	       p_delta_cal, n_delta_cal,
	       sm->fast_mode);

	ret = fg_param_unlock(sm);
	if (ret < 0)
		return ret;

	ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, tmp);
	if (ret < 0) {
		sm_err("Failed to write CURR_IN_OFFSET, ret=%d\n", ret);
		goto cal_lock;
	}
	sm_dbg("CURR_IN_OFFSET [0x%x] = 0x%x\n", FG_REG_CURR_IN_OFFSET, tmp);

	ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, (u16)curr_cal);
	if (ret < 0) {
		sm_err("Failed to write CURR_IN_SLOPE, ret=%d\n", ret);
		goto cal_lock;
	}
	sm_dbg("CURR_IN_SLOPE [0x%x] = 0x%x\n", FG_REG_CURR_IN_SLOPE, curr_cal);

#ifdef ENABLE_MIX_COMP
	ret = fg_read_word(sm, FG_REG_AGING_CTRL, &temp_aging_ctrl);
	if (ret < 0) {
		sm_err("Couldn't read FG_REG_AGING_CTRL, ret=%d\n", ret);
		goto cal_lock;
	}
	if (sm->batt_temp < 80 && !sm->is_charging &&
	    ((sm->batt_soc < 100 && sm->batt_soc > 20) ||
	    (sm->batt_soc < 300 && sm->batt_soc > 200) ||
	    (sm->batt_soc < 500 && sm->batt_soc > 400))) {
		if (sm->aging_ctrl == temp_aging_ctrl)
			ret = fg_write_word(sm, FG_REG_AGING_CTRL, (sm->aging_ctrl & 0xFFFE));
	} else {
		if (sm->aging_ctrl != temp_aging_ctrl)
			ret = fg_write_word(sm, FG_REG_AGING_CTRL, sm->aging_ctrl);
	}
	if (ret < 0) {
		sm_err("Couldn't write FG_REG_AGING_CTRL, ret=%d\n", ret);
		goto cal_lock;
	}
	sm_dbg("FG_REG_AGING_CTRL=0x%x\n", temp_aging_ctrl);
#endif

cal_lock:
	fg_param_lock(sm);
	return ret;
}

static int fg_get_batt_status(struct sm_fg_chip *sm)
{
	int batt_soc, batt_curr;
	bool batt_present, batt_fc, batt_dsg;
	int soc;

	mutex_lock(&sm->data_lock);
	batt_soc = sm->batt_soc;
	batt_curr = sm->batt_curr;
	batt_present = sm->batt_present;
	batt_fc = sm->batt_fc;
	batt_dsg = sm->batt_dsg;
	mutex_unlock(&sm->data_lock);

	soc = clamp(batt_soc / 10, 0, 100);

	if (!batt_present)
		return POWER_SUPPLY_STATUS_UNKNOWN;
	else if (batt_fc && sm->bq_batt_fc && (soc > 95))
		return POWER_SUPPLY_STATUS_FULL;
	else if (batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int fg_get_batt_capacity_level(struct sm_fg_chip *sm)
{
	bool batt_present, batt_fc, batt_soc1, batt_socp;

	mutex_lock(&sm->data_lock);
	batt_present = sm->batt_present;
	batt_fc = sm->batt_fc;
	batt_soc1 = sm->batt_soc1;
	batt_socp = sm->batt_socp;
	mutex_unlock(&sm->data_lock);

	if (!batt_present)
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
	else if (batt_fc)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (batt_soc1)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (batt_socp)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
}

static int fg_get_batt_health(struct sm_fg_chip *sm)
{
	int batt_temp;
	bool batt_present, batt_ot, batt_ut;

	mutex_lock(&sm->data_lock);
	batt_temp = sm->batt_temp;
	batt_present = sm->batt_present;
	batt_ot = sm->batt_ot;
	batt_ut = sm->batt_ut;
	mutex_unlock(&sm->data_lock);

	if (!batt_present)
		return POWER_SUPPLY_HEALTH_UNKNOWN;
#ifdef ENABLE_NTC_COMPENSATION
	else if (batt_temp >= OVERHEAT_TH_DEG)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (batt_temp <= COLD_TH_DEG)
		return POWER_SUPPLY_HEALTH_COLD;
#else
	else if (batt_ot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (batt_ut)
		return POWER_SUPPLY_HEALTH_COLD;
#endif
	else if (batt_temp >= 425)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (batt_temp <= 150)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static int get_battery_id(struct sm_fg_chip *sm)
{
	int battery_id = BATTERY_VENDOR_UNKNOWN;
	union power_supply_propval pval = {0, };
	u8 p0_buf[16];
	int ret;

	if (!sm->verify_psy)
		sm->verify_psy = power_supply_get_by_name("batt_verify");
	if (!sm->verify_psy) {
		sm_err("batt_verify psy not found\n");
		return battery_id;
	}

	ret = power_supply_get_property(sm->verify_psy,
					POWER_SUPPLY_PROP_CHIP_OK,
					&pval);
	if (ret < 0) {
		sm_err("get CHIP_OK error: (%d)\n", ret);
		return battery_id;
	}

	if (pval.intval) {
		ret = power_supply_get_property(sm->verify_psy,
						POWER_SUPPLY_PROP_PAGE0_DATA,
						&pval);
		if (ret < 0) {
			sm_err("get PAGE0_DATA error: (%d)\n", ret);
		} else {
			memcpy(p0_buf, pval.arrayval, 16);
			sm_dbg("PAGE0 raw: %*ph\n", 16, p0_buf);
			if (p0_buf[0] == 'N') {
				battery_id = BATTERY_VENDOR_NVT;
			} else if (p0_buf[0] == 0xFF || p0_buf[0] == 'C' || p0_buf[0] == 'V') {
				battery_id = BATTERY_VENDOR_GY;
			} else if (p0_buf[0] == 'L' || p0_buf[0] == 'X' || p0_buf[0] == 'S') {
				battery_id = BATTERY_VENDOR_XWD;
			} else {
				sm_err("Unknown PAGE0 signature: 0x%02x\n", p0_buf[0]);
				battery_id = BATTERY_VENDOR_UNKNOWN;
			}
		}
	}

	sm_info("battery_id = %d\n", battery_id);
	sm->battery_id = battery_id;
	return battery_id;
}

static enum power_supply_property fg_props[] = {
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_ROMID,
	POWER_SUPPLY_PROP_DS_STATUS,
	POWER_SUPPLY_PROP_PAGE0_DATA,
	POWER_SUPPLY_PROP_CHIP_OK,
#endif
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_SHUTDOWN_DELAY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_SOC_DECIMAL,
	POWER_SUPPLY_PROP_SOC_DECIMAL_RATE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_FASTCHARGE_MODE,
	POWER_SUPPLY_PROP_BATTERY_TYPE,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_SOH,
};

#define SHUTDOWN_DELAY_VOL	3400
#define SHUTDOWN_VOL	3300
static int fg_get_property(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	struct sm_fg_chip *sm = power_supply_get_drvdata(psy);
	union power_supply_propval b_val = {0, };
	bool usb_present;
	int ret;
	int vbat_mv;
	int vbat_curr;
	int vbat_soc;

	switch (psp) {
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	case POWER_SUPPLY_PROP_AUTHENTIC:
		ret = power_supply_get_property(sm->verify_psy,
						POWER_SUPPLY_PROP_AUTHEN_RESULT,
						&b_val);
		val->intval = b_val.intval;
		break;
	case POWER_SUPPLY_PROP_ROMID:
		ret = power_supply_get_property(sm->verify_psy,
						POWER_SUPPLY_PROP_ROMID,
						&b_val);
		memcpy(val->arrayval, b_val.arrayval, 8);
		break;
	case POWER_SUPPLY_PROP_DS_STATUS:
		ret = power_supply_get_property(sm->verify_psy,
						POWER_SUPPLY_PROP_DS_STATUS,
						&b_val);
		memcpy(val->arrayval, b_val.arrayval, 8);
		break;
	case POWER_SUPPLY_PROP_PAGE0_DATA:
		ret = power_supply_get_property(sm->verify_psy,
						POWER_SUPPLY_PROP_PAGE0_DATA,
						&b_val);
		memcpy(val->arrayval, b_val.arrayval, 16);
		break;
	case POWER_SUPPLY_PROP_CHIP_OK:
		ret = power_supply_get_property(sm->verify_psy,
						POWER_SUPPLY_PROP_CHIP_OK,
						&b_val);
		val->intval = b_val.intval;
		break;
#endif
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_batt_status(sm);
		break;
	case POWER_SUPPLY_PROP_SHUTDOWN_DELAY:
		val->intval = sm->shutdown_delay;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&sm->data_lock);
		usb_present = sm->usb_present;
		mutex_unlock(&sm->data_lock);

		if (usb_present && sm->cp_psy && sm->cp_work_flag) {
			ret = power_supply_get_property(sm->cp_psy,
							POWER_SUPPLY_PROP_SC_BATTERY_VOLTAGE,
							&b_val);
			val->intval = b_val.intval * 1000;
			break;
		}

		ret = fg_read_volt(sm);
		mutex_lock(&sm->data_lock);
		val->intval = sm->batt_volt * 1000;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		mutex_lock(&sm->data_lock);
		val->intval = sm->batt_present;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = fg_read_current(sm);
		mutex_lock(&sm->data_lock);
#ifdef ENABLE_CURRENT_AVG
		vbat_curr = sm->param.batt_ma_avg * 1000;
#else
		vbat_curr = sm->batt_curr * 1000;
#endif
		mutex_unlock(&sm->data_lock);
		val->intval = clamp(vbat_curr, -10000000, 10000000);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (sm->fake_soc >= 0) {
			val->intval = sm->fake_soc;
			break;
		}

		ret = fg_read_soc(sm);
		mutex_lock(&sm->data_lock);
		if (sm->param.batt_soc >= 0)
			vbat_soc = sm->param.batt_soc / 10;
		else if ((!ret) && (sm->param.batt_soc == -EINVAL))
			vbat_soc = (sm->batt_soc >= 970) ? 100 : (sm->batt_soc * 10 + 96) / 97;
		else
			vbat_soc = 50;
		mutex_unlock(&sm->data_lock);

		val->intval = clamp(vbat_soc, 0, 100);

		if (sm->shutdown_delay_enable) {
			if (val->intval == 0) {
				sm->is_charging = is_battery_charging(sm);
				vbat_mv = sm->batt_volt;

				if (vbat_mv > SHUTDOWN_DELAY_VOL) {
					val->intval = 1;
					if (sm->is_charging)
						sm->shutdown_delay = false;
				} else if (vbat_mv > SHUTDOWN_VOL) {
					if (!sm->is_charging) {
						sm->shutdown_delay = true;
						val->intval = 1;
					} else {
						sm->shutdown_delay = false;
						val->intval = 1;
					}
				} else {
					sm->shutdown_delay = false;
					val->intval = 0;
				}
			} else {
				sm->shutdown_delay = false;
			}
			if (sm->last_shutdown_delay != sm->shutdown_delay) {
				sm->last_shutdown_delay = sm->shutdown_delay;
				if (sm->batt_psy)
					power_supply_changed(sm->batt_psy);
				if (sm->fg_psy)
					power_supply_changed(sm->fg_psy);
			}
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(sm);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (sm->fake_temp != -EINVAL) {
			val->intval = sm->fake_temp;
			break;
		}

		if (sm->en_temp_in)
			ret = fg_read_temperature(sm, TEMPERATURE_IN);
		else if (sm->en_temp_ex)
			ret = fg_read_temperature(sm, TEMPERATURE_EX);
		else
			ret = -ENODATA;

		mutex_lock(&sm->data_lock);
		if (ret == -ENODATA)
			val->intval = 250;
#ifdef ENABLE_TEMP_AVG
		else if (sm->param.batt_temp >= (EX_TEMP_MIN * 10) &&
			 sm->param.batt_temp <= (EX_TEMP_MAX * 10))
			val->intval = sm->param.batt_temp_avg;
#endif
		else
			val->intval = sm->batt_temp;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		if (sm->battery_id == BATTERY_VENDOR_UNKNOWN)
			sm->battery_id = get_battery_id(sm);
		val->intval = sm->battery_id;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg_read_fcc(sm);
		mutex_lock(&sm->data_lock);
		val->intval = sm->batt_fcc * 1000;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_SOC_DECIMAL:
		val->intval = fg_get_soc_decimal(sm);
		break;
	case POWER_SUPPLY_PROP_SOC_DECIMAL_RATE:
		val->intval = fg_get_soc_decimal_rate(sm);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = fg_get_batt_health(sm);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_FASTCHARGE_MODE:
		val->intval = sm->fast_mode;
		break;
	case POWER_SUPPLY_PROP_BATTERY_TYPE:
		if (sm->battery_id == BATTERY_VENDOR_UNKNOWN)
			sm->battery_id = get_battery_id(sm);
		switch (sm->battery_id) {
		case BATTERY_VENDOR_NVT:
			val->strval = "M376-NVT-5000mAh";
			break;
		case BATTERY_VENDOR_GY:
			val->strval = "M376-GuanYu-5000mAh";
			break;
		case BATTERY_VENDOR_XWD:
			val->strval = "M376-Sunwoda-5000mAh";
			break;
		default:
			val->strval = "M376-unknown-5000mAh";
			break;
		}
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg_get_cycle(sm);
		mutex_lock(&sm->data_lock);
		val->intval = sm->batt_soc_cycle;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = fg_read_rmc(sm);
		mutex_lock(&sm->data_lock);
		val->intval = sm->batt_rmc * 1000;
		mutex_unlock(&sm->data_lock);
		break;
	case POWER_SUPPLY_PROP_SOH:
		val->intval = 100;
		break;
	default:
		sm_dbg("unsupported_property psp=%d\n", psp);
		return -EINVAL;
	}

	return 0;
}

static int fg_set_property(struct power_supply *psy,
			   enum power_supply_property prop,
			   const union power_supply_propval *val)
{
	struct sm_fg_chip *sm = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_FASTCHARGE_MODE:
		sm->fast_mode = !!val->intval;
		break;
	case POWER_SUPPLY_PROP_CHIP_OK:
		sm->fake_chip_ok = !!val->intval;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fg_prop_is_writeable(struct power_supply *psy,
				enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_FASTCHARGE_MODE:
	case POWER_SUPPLY_PROP_CHIP_OK:
		return 1;
	default:
		return 0;
	}
}

static int fg_psy_register(struct sm_fg_chip *sm)
{
	struct power_supply_config fg_psy_cfg = {};

	sm->fg_psy_d.name = "bms";
	sm->fg_psy_d.type = POWER_SUPPLY_TYPE_BMS;
	sm->fg_psy_d.properties = fg_props;
	sm->fg_psy_d.num_properties = ARRAY_SIZE(fg_props);
	sm->fg_psy_d.get_property = fg_get_property;
	sm->fg_psy_d.set_property = fg_set_property;
	sm->fg_psy_d.property_is_writeable = fg_prop_is_writeable;

	fg_psy_cfg.drv_data = sm;
	fg_psy_cfg.of_node = sm->dev->of_node;

	sm->fg_psy = devm_power_supply_register(sm->dev, &sm->fg_psy_d, &fg_psy_cfg);
	if (IS_ERR(sm->fg_psy)) {
		sm_err("failed to register fg_psy\n");
		return PTR_ERR(sm->fg_psy);
	}

	sm_info("%s power supply register successfully\n", sm->fg_psy_d.name);

	return 0;
}

static const u8 fg_dump_regs[] = {
	0x00, 0x01, 0x03, 0x04,
	0x05, 0x06, 0x07, 0x08,
	0x09, 0x0A, 0x0C, 0x0D,
	0x0E, 0x0F, 0x10, 0x11,
	0x12, 0x13, 0x14, 0x1A,
	0x1B, 0x1C, 0x62, 0x73,
	0x74, 0x90, 0x91, 0x95,
	0x96
};

static int fg_dump_debug(struct seq_file *m, void *data)
{
	struct sm_fg_chip *sm = m->private;
	int i;
	int ret;
	u16 val = 0;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		ret = fg_read_word(sm, fg_dump_regs[i], &val);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%04X\n", fg_dump_regs[i], val);
	}

	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct sm_fg_chip *sm = inode->i_private;

	return single_open(file, fg_dump_debug, sm);
}

static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct sm_fg_chip *sm)
{
	sm->debug_root = debugfs_create_dir("sm_fg", NULL);

	if (!sm->debug_root) {
		sm_err("failed to create debug dir\n");
		return;
	}

	debugfs_create_file("registers", S_IFREG | 0444,
			    sm->debug_root, sm, &reg_debugfs_ops);
	debugfs_create_u32("fake_soc", S_IFREG | 0644,
			   sm->debug_root, (u32 *)&sm->fake_soc);
	debugfs_create_u32("fake_temp", S_IFREG | 0644,
			   sm->debug_root, (u32 *)&sm->fake_temp);
	debugfs_create_u8("skip_reads", 0644,
			  sm->debug_root, (u8 *)&sm->skip_reads);
	debugfs_create_u8("skip_writes", 0644,
			  sm->debug_root, (u8 *)&sm->skip_writes);
}

static ssize_t registers_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int i, ret, idx = 0;
	u16 val = 0;
	u8 reg;

	if (IS_ERR_OR_NULL(sm))
		return -ENODEV;

	idx += scnprintf(buf + idx, PAGE_SIZE - idx, "FG registers:\n");

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		reg = fg_dump_regs[i];

		ret = fg_read_word(sm, reg, &val);
		if (ret) {
			dev_warn(sm->dev, "read reg 0x%02x failed: %d\n", reg, ret);
			idx += scnprintf(buf + idx, PAGE_SIZE - idx,
					"Reg[0x%02x] = <err %d>\n", reg, ret);
		} else {
			idx += scnprintf(buf + idx, PAGE_SIZE - idx,
					"Reg[0x%02x] = 0x%04X\n", reg, val);
		}

		if (idx >= PAGE_SIZE)
			break;
	}

	return idx;
}

static ssize_t registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	char tmp[48];
	unsigned long reg_ul = 0, val_ul = 0;
	char *p;
	int ret, i;
	bool ok = false;

	if (IS_ERR_OR_NULL(sm))
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

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		if (reg_ul == fg_dump_regs[i]) {
			ok = true;
			break;
		}
	}
	if (!ok) {
		dev_err(sm->dev, "invalid reg: 0x%lx not in dump list\n", reg_ul);
		return -EINVAL;
	}

	if (val_ul > 0xffff) {
		dev_err(sm->dev, "invalid val: 0x%lx\n", val_ul);
		return -EINVAL;
	}

	ret = fg_write_word(sm, (u8)reg_ul, (u16)val_ul);
	if (ret) {
		dev_err(sm->dev, "write reg 0x%04lx failed: %d\n", reg_ul, ret);
		return ret;
	}

	dev_info(sm->dev, "wrote 0x%04lx to reg 0x%04lx\n", val_ul, reg_ul);
	return count;
}

static ssize_t batt_rmc_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int rmc;

	if (IS_ERR_OR_NULL(sm))
		return -ENODEV;

	fg_read_rmc(sm);
	mutex_lock(&sm->data_lock);
	rmc = sm->batt_rmc;
	mutex_unlock(&sm->data_lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", rmc);
}

static ssize_t batt_fcc_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int fcc;

	if (IS_ERR_OR_NULL(sm))
		return -ENODEV;

	fg_read_fcc(sm);
	mutex_lock(&sm->data_lock);
	fcc = sm->batt_fcc;
	mutex_unlock(&sm->data_lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", fcc);
}

static ssize_t batt_volt_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int volt;

	if (IS_ERR_OR_NULL(sm))
		return -ENODEV;

	fg_read_volt(sm);
	mutex_lock(&sm->data_lock);
	volt = sm->batt_volt;
	mutex_unlock(&sm->data_lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", volt);
}

static ssize_t batt_curr_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int curr;

	if (IS_ERR_OR_NULL(sm))
		return -ENODEV;

	fg_read_current(sm);
	mutex_lock(&sm->data_lock);
#ifdef ENABLE_CURRENT_AVG
	curr = sm->param.batt_ma_avg;
#else
	curr = sm->batt_curr;
#endif
	mutex_unlock(&sm->data_lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", curr);
}

static ssize_t batt_cycle_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sm_fg_chip *sm = i2c_get_clientdata(client);
	int cycle;

	if (IS_ERR_OR_NULL(sm))
		return -ENODEV;

	fg_get_cycle(sm);
	mutex_lock(&sm->data_lock);
	cycle = sm->batt_soc_cycle;
	mutex_unlock(&sm->data_lock);
	return scnprintf(buf, PAGE_SIZE, "%d\n", cycle);
}

static DEVICE_ATTR_RO(batt_rmc);
static DEVICE_ATTR_RO(batt_fcc);
static DEVICE_ATTR_RO(batt_volt);
static DEVICE_ATTR_RO(batt_curr);
static DEVICE_ATTR_RO(batt_cycle);
static DEVICE_ATTR_RW(registers);

static struct attribute *fg_attributes[] = {
	&dev_attr_batt_rmc.attr,
	&dev_attr_batt_fcc.attr,
	&dev_attr_batt_volt.attr,
	&dev_attr_batt_curr.attr,
	&dev_attr_batt_cycle.attr,
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};

static void fg_smooth_tracking(struct sm_fg_chip *sm)
{
	int raw_soc = sm->param.batt_raw_soc;
	unsigned long long tmp = raw_soc * 100ULL + 960;
	int orig_soc = (raw_soc <= 0) ? 0 : (int)(tmp / 97);
	int target_soc, delta_soc, bucket_soc;
	int curr_soc, new_soc, soc_delta = 0, soc_changed = 0;
	ktime_t now = ktime_get_boottime();
	int allowed_steps = 0, delta_ms = 0;
	int unit_time_ms = 100; /* 100ms per 0.1% (1s per 1.0%) */
	int batt_ma = sm->batt_curr;
	int batt_temp = sm->batt_temp;
	int charging_status = get_battery_status(sm);

	if (raw_soc >= 970) {
		target_soc = 1000;
	} else if (orig_soc <= 990) {
		target_soc = orig_soc;
	} else {
		delta_soc = orig_soc - 990;
		bucket_soc = delta_soc >> 1;
		target_soc = 990 + bucket_soc;
	}

	if (!sm->soc_smooth_initialized) {
		sm->param.batt_soc = target_soc;
		sm->last_soc_change_time = ktime_to_ms(now);
		sm->soc_smooth_initialized = true;
		return;
	}

	/* Update debouncing counters */
	if ((batt_ma < 0) && (batt_ma > -9) && (sm->ibat_pos9_count < 1440))
		sm->ibat_pos9_count++;
	else if ((batt_ma <= -9) && (sm->ibat_pos_count < 3))
		sm->ibat_pos_count++;
	else if (batt_ma >= 0) {
		sm->ibat_pos_count = 0;
		sm->ibat_pos9_count = 0;
	}

	curr_soc = sm->param.batt_soc;

	/* Low temperature adjustments (batt_temp in tenths of a degree, so 150 = 15.0C) */
	if ((charging_status == POWER_SUPPLY_STATUS_DISCHARGING ||
	     charging_status == POWER_SUPPLY_STATUS_NOT_CHARGING) &&
	    !sm->batt_rmc && batt_temp < 150 && curr_soc >= 10) {
		if (batt_ma > -300)
			unit_time_ms = 2000; /* 20s per 1% */
		else if (batt_ma > -600)
			unit_time_ms = 1500; /* 15s per 1% */
		else
			unit_time_ms = 1000; /* 10s per 1% */
	} else if (charging_status == POWER_SUPPLY_STATUS_CHARGING) {
		unit_time_ms = 100; /* 100ms per 0.1% (1s per 1.0%) */
	}

	soc_delta = abs(target_soc - curr_soc);

	/* Smooth only if gap > 10 (1%), OR low voltage, OR cold temperature */
	if (soc_delta >= 10 ||
	    (sm->batt_volt < 3300 && target_soc > 0) ||
	    (unit_time_ms != 100) ||
	    charging_status == POWER_SUPPLY_STATUS_CHARGING) {
		delta_ms = ktime_to_ms(now) - sm->last_soc_change_time;
		if (delta_ms < 0) {
			sm->last_soc_change_time = ktime_to_ms(now);
			delta_ms = 0;
		}

		allowed_steps = delta_ms / unit_time_ms;
		soc_changed = min(soc_delta, allowed_steps); /* Limit change to target delta */

		if (soc_changed > 0) {
			if (charging_status == POWER_SUPPLY_STATUS_CHARGING && target_soc > curr_soc)
				new_soc = curr_soc + soc_changed;
			else if (charging_status == POWER_SUPPLY_STATUS_DISCHARGING && target_soc < curr_soc)
				new_soc = curr_soc - soc_changed;
			else
				new_soc = curr_soc;
		} else {
			new_soc = curr_soc;
		}
	} else {
		new_soc = target_soc; /* Instant update if < 1% jump under normal conditions */
	}

	/* Avoid mismatches between charging status and soc changes (debounce) */
	if (((charging_status == POWER_SUPPLY_STATUS_DISCHARGING) && (new_soc > curr_soc)) ||
	    ((charging_status == POWER_SUPPLY_STATUS_CHARGING) && (new_soc < curr_soc) &&
	     (sm->ibat_pos_count < 3) && (sm->ibat_pos9_count < 1440) &&
	     (delta_ms > 10000))) {
		new_soc = curr_soc;
	}

	if (new_soc != curr_soc) {
		/* Advance last_change_time by exact steps consumed to prevent drift */
		if (soc_delta >= 10 || (sm->batt_volt < 3300 && target_soc > 0) || (unit_time_ms != 100) ||
		    charging_status == POWER_SUPPLY_STATUS_CHARGING) {
			sm->last_soc_change_time += (s64)soc_changed * unit_time_ms;
		} else {
			sm->last_soc_change_time = ktime_to_ms(now); /* Reset base on instant update */
		}
		curr_soc = new_soc;
	}

	if (curr_soc > 1000)
		curr_soc = 1000;
	if (curr_soc <= 0)
		curr_soc = 0;

	sm_info("soc: %d.%d, target: %d.%d, raw: %d.%d, delta_ms: %d, allowed: %d, step: %d\n",
		curr_soc / 10, curr_soc % 10, target_soc / 10, target_soc % 10,
		raw_soc / 10, raw_soc % 10, delta_ms, delta_ms / unit_time_ms, soc_changed);

	sm->param.batt_soc = curr_soc;
}

static int fg_refresh_status(struct sm_fg_chip *sm)
{
	bool prev_batt_present, prev_batt_fc, prev_batt_ot, prev_batt_ut;
	bool cur_batt_present, cur_batt_fc, cur_batt_ot, cur_batt_ut;
	int prev_batt_soc, prev_batt_temp;
	union power_supply_propval pval = {0,};
	int cp_vbat = 0, ret;
	int batt_soc, batt_volt, batt_curr;
	bool usb_present;
	int interval;

	mutex_lock(&sm->data_lock);
	prev_batt_present = sm->batt_present;
	prev_batt_fc = sm->batt_fc;
	prev_batt_ot = sm->batt_ot;
	prev_batt_ut = sm->batt_ut;
	prev_batt_soc = sm->batt_soc;
	prev_batt_temp = sm->batt_temp;
	mutex_unlock(&sm->data_lock);

	fg_read_status(sm);

	mutex_lock(&sm->data_lock);
	cur_batt_present = sm->batt_present;
	cur_batt_fc = sm->batt_fc;
	cur_batt_ot = sm->batt_ot;
	cur_batt_ut = sm->batt_ut;
	mutex_unlock(&sm->data_lock);

	if (!cur_batt_present) {
		if (prev_batt_present) {
			sm_info("Battery removed\n");
			mutex_lock(&sm->data_lock);
			sm->batt_soc = -ENODATA;
			sm->batt_fcc = -ENODATA;
			sm->batt_volt = -ENODATA;
			sm->batt_curr = -ENODATA;
			sm->batt_temp = -ENODATA;
			mutex_unlock(&sm->data_lock);
			power_supply_changed(sm->fg_psy);
		}
		return MONITOR_WORK_10S;
	}

	if (!prev_batt_present)
		sm_info("Battery inserted\n");

	if (cur_batt_present != prev_batt_present ||
	    cur_batt_fc != prev_batt_fc ||
	    cur_batt_ot != prev_batt_ot ||
	    cur_batt_ut != prev_batt_ut)
		power_supply_changed(sm->fg_psy);

	fg_read_soc(sm);
	fg_read_ocv(sm);
	fg_read_fcc(sm);
	fg_read_volt(sm);
	fg_read_current(sm);
	fg_get_cycle(sm);
	fg_read_rmc(sm);
	if (sm->en_temp_in)
		fg_read_temperature(sm, TEMPERATURE_IN);
	else if (sm->en_temp_ex)
		fg_read_temperature(sm, TEMPERATURE_EX);

	fg_cal_carc(sm);

	mutex_lock(&sm->data_lock);
	sm->param.batt_raw_soc = sm->batt_soc;
	fg_smooth_tracking(sm);
	mutex_unlock(&sm->data_lock);

	mutex_lock(&sm->data_lock);
	batt_soc = sm->batt_soc;
	batt_volt = sm->batt_volt;
	batt_curr = sm->batt_curr;
	usb_present = sm->usb_present;
	mutex_unlock(&sm->data_lock);

	if (usb_present && sm->cp_psy && sm->cp_work_flag) {
		ret = power_supply_get_property(sm->cp_psy,
					       POWER_SUPPLY_PROP_SC_BATTERY_VOLTAGE,
					       &pval);
		if (!ret)
			cp_vbat = pval.intval;
	}

	sm_log("raw_soc: %d, volt: %d, cp_volt: %d, current: %d, fast_mode: %d\n",
	       batt_soc, batt_volt, cp_vbat, batt_curr, sm->fast_mode);
	sm_log("cycle_count: %d, charge_counter: %d, fcc: %d, ocv: %d\n",
	       sm->batt_soc_cycle, sm->batt_rmc, sm->batt_fcc, sm->batt_ocv);

	if (sm->fast_mode ||
	    (usb_present && batt_soc >= SM_RAW_SOC_FULL))
		interval = MONITOR_WORK_1S;
	else if (usb_present)
		interval = MONITOR_WORK_5S;
	else
		interval = MONITOR_WORK_10S;

	if (sm->fg_psy)
		power_supply_changed(sm->fg_psy);
	if (sm->batt_psy)
		power_supply_changed(sm->batt_psy);

	return interval;
}

static void fg_update_low_battery(struct sm_fg_chip *sm)
{
	union power_supply_propval pval = {0,};
	bool usb_online = false;
	int batt_soc, batt_volt;
	int ret;

	if (sm->usb_psy) {
		ret = power_supply_get_property(sm->usb_psy,
					       POWER_SUPPLY_PROP_ONLINE,
					       &pval);
		if (!ret)
			usb_online = !!pval.intval;
		else
			sm_err("Couldn't get prop usb online!\n");
	}

	if (usb_online) {
		sm->low_batt_check_ts = 0;
		sm->low_battery_power = false;
		return;
	}

	mutex_lock(&sm->data_lock);
	batt_soc = sm->batt_soc;
	batt_volt = sm->batt_volt;
	mutex_unlock(&sm->data_lock);

	if (batt_soc < 10 && batt_volt < 3400) {
		if (!sm->low_battery_power) {
			if (!sm->low_batt_check_ts)
				sm->low_batt_check_ts = jiffies + 5 * HZ;
			else if (time_after_eq(jiffies, sm->low_batt_check_ts)) {
				sm->low_battery_power = true;
				mutex_lock(&sm->data_lock);
				sm->param.batt_soc = 0;
				sm->batt_soc = 0;
				mutex_unlock(&sm->data_lock);
			}
		}
	} else {
		sm->low_batt_check_ts = 0;
		sm->low_battery_power = false;
	}

	sm_log("low_battery_power: %d\n", sm->low_battery_power);
}

#define SM5602_FFC_TERM_WAM_TEMP	350
#define SM5602_COLD_TEMP_TERM		0
#define SM5602_FFC_FULL_FV		8940
#define SM5602_NOR_FULL_FV		8880
#define BAT_FULL_CHECK_TIME		1
static void fg_check_full_status(struct sm_fg_chip *sm)
{
	int full_volt;
	int batt_soc, batt_volt, batt_curr;
	bool usb_present;

	if (!sm->chg_dis_votable)
		sm->chg_dis_votable = find_votable("CHG_DISABLE");
	if (!sm->chg_dis_votable)
		return;

	if (!sm->fv_votable)
		sm->fv_votable = find_votable("FV");
	if (!sm->fv_votable)
		return;

	mutex_lock(&sm->data_lock);
	usb_present = sm->usb_present;
	mutex_unlock(&sm->data_lock);

	if (!usb_present) {
		if (sm->batt_sw_fc) {
			vote(sm->chg_dis_votable, BMS_FC_VOTER, false, 0);
			sm->batt_sw_fc = false;
			sm_info("USB removed: reset full state\n");
		}
		sm->full_check = 0;
		return;
	}

	full_volt = get_effective_result(sm->fv_votable) - 20;

	mutex_lock(&sm->data_lock);
	batt_soc = sm->batt_soc;
	batt_volt = sm->batt_volt;
	batt_curr = sm->batt_curr;
	mutex_unlock(&sm->data_lock);

	if (batt_soc >= SM_RAW_SOC_FULL &&
	    batt_volt > full_volt &&
	    batt_curr > 0 &&
	    !sm->batt_sw_fc) {
		if (++sm->full_check >= BAT_FULL_CHECK_TIME) {
			sm->batt_sw_fc = true;
			vote(sm->chg_dis_votable, BMS_FC_VOTER, true, 0);
			sm_info("charge termination -> batt_sw_fc=%d\n",
				sm->batt_sw_fc);
		}
	} else {
		sm->full_check = 0;
	}
}

#define BAT_WARM_TEMP	480
static void fg_check_recharge_status(struct sm_fg_chip *sm)
{
	int batt_soc, batt_temp;

	if (!sm->chg_dis_votable)
		sm->chg_dis_votable = find_votable("CHG_DISABLE");
	if (!sm->chg_dis_votable)
		return;

	mutex_lock(&sm->data_lock);
	batt_soc = sm->batt_soc;
	batt_temp = sm->batt_temp;
	mutex_unlock(&sm->data_lock);

	if (batt_soc <= SM_RECHARGE_SOC &&
	    batt_temp < BAT_WARM_TEMP &&
	    sm->batt_sw_fc) {
		sm->batt_sw_fc = false;
		vote(sm->chg_dis_votable, BMS_FC_VOTER, false, 0);
		sm_info("force recharging\n");
	}
}

static void fg_monitor_workfunc(struct kthread_work *work)
{
	struct sm_fg_chip *sm = container_of(work, struct sm_fg_chip, monitor_work.work);
	int interval;

	fg_init(sm);

	interval = fg_refresh_status(sm);
	fg_update_low_battery(sm);
	fg_check_full_status(sm);
	fg_check_recharge_status(sm);

	kthread_queue_delayed_work(sm->main_worker,
				   &sm->monitor_work,
				   interval * HZ);
}

static int sm_fg_charger_event(struct notifier_block *nb, unsigned long event, void *v)
{
	struct sm_fg_chip *sm = container_of(nb, struct sm_fg_chip, chg_nb);

	switch (event) {
	case BQ_CHG_EVENT_START_MONITOR:
		kthread_queue_delayed_work(sm->main_worker, &sm->monitor_work, 0);
		break;
	case BQ_CHG_EVENT_STOP_MONITOR:
		kthread_cancel_delayed_work_sync(&sm->monitor_work);
		break;
	case BQ_CHG_EVENT_CHARGE_DONE:
		sm->bq_batt_fc = true;
		break;
	case BQ_CHG_EVENT_CHARGE_NOT_DONE:
		sm->bq_batt_fc = false;
		break;
	}
	return NOTIFY_OK;
}

static bool fg_check_reg_init_need(struct sm_fg_chip *sm)
{
	int ret;
	u16 data = 0;
	u16 param_ver = 0;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
	if (ret < 0) {
		sm_err("Failed to read FG_OP_STATUS(%d)\n", ret);
		return true;
	}
	sm_dbg("FG_OP_STATUS = 0x%x\n", data);

	ret = fg_read_word(sm, FG_PARAM_VERSION, &param_ver);
	if (ret < 0) {
		sm_err("Failed to read FG_PARAM_VERSION(%d)\n", ret);
		return true;
	}

	sm_dbg("param_version = 0x%x, common=%u, battery=%u\n",
	       param_ver, sm->common_param_version, sm->battery_param_version);

	if (((data & FG_INIT_DISABLED_MASK) == FG_INIT_DISABLED_BIT) &&
	    (((param_ver & COMMON_PARAM_MASK) >> COMMON_PARAM_SHIFT) >= sm->common_param_version) &&
	    ((param_ver & BATTERY_PARAM_MASK) >= sm->battery_param_version)) {
		sm_dbg("FG_OP_STATUS: 0x%x, no init needed\n", data);
		return false;
	}

	sm_err("FG_OP_STATUS: 0x%x, init required\n", data);
	return true;
}

static int fg_read_v_i_sample(struct sm_fg_chip *sm, int idx, u16 *v_out, int *i_out)
{
	u16 v_ret, i_ret;
	int ret;

	ret = fg_write_word(sm, FG_REG_SRADDR, idx);
	if (ret < 0) {
		sm_err("Failed to write FG_REG_SRADDR idx=0x%x, ret=%d\n", idx, ret);
		return ret;
	}
	usleep_range(15000, 16000);
	ret = fg_read_word(sm, FG_REG_SRDATA, &v_ret);
	if (ret >= 0 && (v_ret & FG_IOCV_BUSY_BIT)) {
		usleep_range(15000, 16000);
		ret = fg_read_word(sm, FG_REG_SRDATA, &v_ret);
	}
	if (ret < 0) {
		sm_err("Failed to read v idx=0x%x, ret=%d\n", idx, ret);
		return ret;
	}

	ret = fg_write_word(sm, FG_REG_SRADDR, idx + FG_IOCV_CURR_IDX_OFFSET);
	if (ret < 0) {
		sm_err("Failed to write FG_REG_SRADDR idx=0x%x, ret=%d\n",
		       idx + FG_IOCV_CURR_IDX_OFFSET, ret);
		return ret;
	}
	usleep_range(15000, 16000);
	ret = fg_read_word(sm, FG_REG_SRDATA, &i_ret);
	if (ret >= 0 && (i_ret & FG_IOCV_BUSY_BIT)) {
		usleep_range(15000, 16000);
		ret = fg_read_word(sm, FG_REG_SRDATA, &i_ret);
	}
	if (ret < 0) {
		sm_err("Failed to read i idx=0x%x, ret=%d\n", idx, ret);
		return ret;
	}

	*v_out = v_ret;
	*i_out = (i_ret & FG_IOCV_CURR_SIGN_BIT) ?
		  -(int)(i_ret & FG_IOCV_CURR_MAG_MASK) :
		  (int)i_ret;
	return 0;
}

static int fg_build_i_offset(int i_set, bool *sign_negative)
{
	int i_offset = i_set + 4;

	if (i_offset <= 0) {
		*sign_negative = true;
#ifdef IGNORE_N_I_OFFSET
		i_offset = 0;
#else
		i_offset = -i_offset;
#endif
	} else {
		*sign_negative = false;
	}

	i_offset >>= 1;
	if (!*sign_negative)
		i_offset |= FG_I_OFFSET_POS_BIT;
	i_offset |= i_offset << 8;

	return i_offset;
}

static int fg_calculate_iocv(struct sm_fg_chip *sm)
{
	bool only_lb = false, sign_i_offset = false;
	int roop_start = 0, roop_max = 0, i = 0;
	int cb_last_index = 0, cb_pre_last_index = 0;
	int lb_v_buffer[FG_INIT_B_LEN + 1] = { };
	int lb_i_buffer[FG_INIT_B_LEN + 1] = { };
	int cb_v_buffer[FG_INIT_B_LEN + 1] = { };
	int cb_i_buffer[FG_INIT_B_LEN + 1] = { };
	int i_offset_margin = FG_IOCV_I_OFFSET_MARGIN, i_vset_margin = FG_IOCV_I_VSET_MARGIN;
	int v_max = 0, v_min = 0, v_sum = 0;
	int i_max = 0, i_min = 0, i_sum = 0;
	int lb_v_avg = 0, lb_i_avg = 0, lb_v_set = 0, lb_i_set = 0;
	int cb_v_avg = 0, cb_i_avg = 0, cb_v_set = 0, cb_i_set = 0;
	int lb_i_p_v_min = 0, lb_i_n_v_max = 0;
	int cb_i_p_v_min = 0, cb_i_n_v_max = 0;
	int i_offset = 0, ret = 0;
	u16 v_ret = 0;
	int i_ret = 0;
	u16 data = 0;

	ret = fg_read_word(sm, FG_REG_END_V_IDX, &data);
	if (ret < 0) {
		sm_err("Failed to read FG_REG_END_V_IDX, ret=%d\n", ret);
		return ret;
	}
	sm_info("iocv_status_read = addr: 0x%x, data: 0x%x\n", FG_REG_END_V_IDX, data);

	only_lb = !(data & FG_IOCV_CB_PRESENT_BIT);

	roop_max = data & FG_IOCV_IDX_CNT_MASK;
	if (roop_max > FG_INIT_B_LEN)
		roop_max = FG_INIT_B_LEN;

	if (roop_max < 3) {
		sm_err("Insufficient samples for iocv calculation: %d\n", roop_max);
		return -EINVAL;
	}

	roop_start = FG_REG_START_LB_V;
	for (i = roop_start; i < roop_start + roop_max; i++) {
		ret = fg_read_v_i_sample(sm, i, &v_ret, &i_ret);
		if (ret < 0)
			return ret;

		lb_v_buffer[i - roop_start] = v_ret;
		lb_i_buffer[i - roop_start] = i_ret;

		if (i == roop_start) {
			v_max = v_min = v_sum = v_ret;
			i_max = i_min = i_sum = i_ret;
		} else {
			v_max = max_t(int, v_max, v_ret);
			v_min = min_t(int, v_min, v_ret);
			v_sum += v_ret;
			i_max = max_t(int, i_max, i_ret);
			i_min = min_t(int, i_min, i_ret);
			i_sum += i_ret;
		}

		if (abs(i_ret) > i_vset_margin) {
			if (i_ret > 0)
				lb_i_p_v_min = lb_i_p_v_min ? min(lb_i_p_v_min, (int)v_ret) : v_ret;
			else
				lb_i_n_v_max = lb_i_n_v_max ? max(lb_i_n_v_max, (int)v_ret) : v_ret;
		}
	}

	v_sum -= v_max + v_min;
	i_sum -= i_max + i_min;

	lb_v_avg = v_sum / (roop_max - 2);
	lb_i_avg = i_sum / (roop_max - 2);

	if (abs(lb_i_buffer[roop_max - 1]) < i_vset_margin) {
		if (abs(lb_i_buffer[roop_max - 2]) < i_vset_margin) {
			lb_v_set = max(lb_v_buffer[roop_max - 2], lb_v_buffer[roop_max - 1]);
			if (abs(lb_i_buffer[roop_max - 3]) < i_vset_margin)
				lb_v_set = max(lb_v_buffer[roop_max - 3], lb_v_set);
		} else {
			lb_v_set = lb_v_buffer[roop_max - 1];
		}
	} else {
		lb_v_set = lb_v_avg;
	}

	if (lb_i_n_v_max > 0)
		lb_v_set = max(lb_i_n_v_max, lb_v_set);

	if (roop_max > 3)
		lb_i_set = (lb_i_buffer[2] + lb_i_buffer[3]) / 2;

	if ((abs(lb_i_buffer[roop_max - 1]) < i_offset_margin) &&
	    (abs(lb_i_set) < i_offset_margin)) {
		lb_i_set = max(lb_i_buffer[roop_max - 1], lb_i_set);
	} else if (abs(lb_i_buffer[roop_max - 1]) < i_offset_margin) {
		lb_i_set = lb_i_buffer[roop_max - 1];
	} else if (abs(lb_i_set) >= i_offset_margin) {
		lb_i_set = 0;
	}

	i_offset = fg_build_i_offset(lb_i_set, &sign_i_offset);

	if (!only_lb) {
		roop_start = FG_REG_START_CB_V;
		roop_max = 6;
		for (i = roop_start; i < roop_start + roop_max; i++) {
			ret = fg_read_v_i_sample(sm, i, &v_ret, &i_ret);
			if (ret < 0)
				return ret;

			cb_v_buffer[i - roop_start] = v_ret;
			cb_i_buffer[i - roop_start] = i_ret;

			if (i == roop_start) {
				v_max = v_min = v_sum = v_ret;
				i_max = i_min = i_sum = i_ret;
			} else {
				v_max = max_t(int, v_max, v_ret);
				v_min = min_t(int, v_min, v_ret);
				v_sum += v_ret;
				i_max = max_t(int, i_max, i_ret);
				i_min = min_t(int, i_min, i_ret);
				i_sum += i_ret;
			}

			if (abs(i_ret) > i_vset_margin) {
				if (i_ret > 0)
					cb_i_p_v_min = cb_i_p_v_min ? min(cb_i_p_v_min, (int)v_ret) : v_ret;
				else
					cb_i_n_v_max = cb_i_n_v_max ? max(cb_i_n_v_max, (int)v_ret) : v_ret;
			}
		}

		v_sum -= v_max + v_min;
		i_sum -= i_max + i_min;

		cb_v_avg = v_sum / (roop_max - 2);
		cb_i_avg = i_sum / (roop_max - 2);

		cb_last_index = (data & FG_IOCV_IDX_CNT_MASK) - 7;
		if (cb_last_index < 0)
			cb_last_index = 5;

		for (i = roop_max; i > 0; i--) {
			if (abs(cb_i_buffer[cb_last_index]) < i_vset_margin) {
				cb_v_set = cb_v_buffer[cb_last_index];
				if (abs(cb_i_buffer[cb_last_index]) < i_offset_margin)
					cb_i_set = cb_i_buffer[cb_last_index];

				cb_pre_last_index = cb_last_index - 1;
				if (cb_pre_last_index < 0)
					cb_pre_last_index = 5;

				if (abs(cb_i_buffer[cb_pre_last_index]) < i_vset_margin) {
					cb_v_set = max(cb_v_buffer[cb_pre_last_index], cb_v_set);
					if (abs(cb_i_buffer[cb_pre_last_index]) < i_offset_margin)
						cb_i_set = max(cb_i_buffer[cb_pre_last_index], cb_i_set);
				}
				break;
			}
			cb_last_index--;
			if (cb_last_index < 0)
				cb_last_index = 5;
		}

		if (cb_v_set == 0) {
			cb_v_set = cb_v_avg;
			if (cb_i_set == 0)
				cb_i_set = cb_i_avg;
		}

		if (cb_i_n_v_max > 0)
			cb_v_set = max(cb_i_n_v_max, cb_v_set);

		if (abs(cb_i_set) < i_offset_margin && cb_i_set > lb_i_set)
			i_offset = fg_build_i_offset(cb_i_set, &sign_i_offset);
	}

	if (abs(cb_i_set) > i_vset_margin || only_lb) {
		ret = max(lb_v_set, cb_i_n_v_max);
		cb_i_set = lb_i_avg;
	} else {
		ret = cb_v_set;
		cb_i_set = cb_i_avg;
	}

#ifdef ENABLE_IOCV_ADJ
	if (ret < IOCV_MAX_ADJ_LEVEL && ret > IOCV_MIN_ADJ_LEVEL &&
	    abs(cb_i_set) < IOCI_MAX_ADJ_LEVEL && abs(cb_i_set) > IOCI_MIN_ADJ_LEVEL) {
		cb_v_set = ret;
		ret = ret - (((cb_i_set * IOCV_I_SLOPE) + IOCV_I_OFFSET) / 1000);
		sm_info("first boot vbat-soc adjust 1st_v=0x%x, 2nd_v=0x%x, all_i=0x%x\n",
			cb_v_set, ret, cb_i_set);
	}
#endif

	if (ret > sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN - 1])
		ret = sm->battery_table[BATTERY_TABLE0][FG_TABLE_LEN - 1];
	else if (ret < sm->battery_table[BATTERY_TABLE0][0])
		ret = sm->battery_table[BATTERY_TABLE0][0] + FG_IOCV_OCV_MIN_OFFSET;

	return ret;
}

static bool fg_reg_init(struct sm_fg_chip *sm)
{
	bool ok = false;
	int i, j, value, ret, cnt = 0;
	u8 table_reg;
	u16 data, data_int_mask;
	u16 cntl_mask, cntl_val;

	sm_info("start\n");

	mutex_lock(&sm->data_lock);

	cntl_mask = ENABLE_EN_TEMP_IN | ENABLE_EN_TEMP_EX | ENABLE_EN_BATT_DET;
	cntl_val = (sm->en_temp_in ? ENABLE_EN_TEMP_IN : 0) |
		   (sm->en_temp_ex ? ENABLE_EN_TEMP_EX : 0) |
		   (sm->en_batt_det ? ENABLE_EN_BATT_DET : 0);

	ret = fg_update_bits(sm, sm->regs[SM_FG_REG_CNTL], cntl_mask, cntl_val);
	if (ret < 0) {
		sm_err("Failed to update CNTL, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("CNTL[0x%02x] en_temp_in=%d en_temp_ex=%d en_batt_det=%d\n",
		sm->regs[SM_FG_REG_CNTL],
		sm->en_temp_in, sm->en_temp_ex, sm->en_batt_det);

	/* IRQ Mask */
	if (sm->fg_irq_set == -EINVAL) {
		sm_err("fg_irq_set is invalid\n");
	} else {
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_INT_MASK], &data_int_mask);
		if (ret < 0) {
			sm_err("Failed to read INT_MASK, ret=%d\n", ret);
			goto unlock;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK],
				    (u16)(0x4000 | (data_int_mask | (u16)sm->fg_irq_set)));
		if (ret < 0) {
			sm_err("Failed to write 0x4000 | INIT_MASK, ret=%d\n", ret);
			goto unlock;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_INT_MASK],
				    (u16)(0x07FF & (data_int_mask | (u16)sm->fg_irq_set)));
		if (ret < 0) {
			sm_err("Failed to write INIT_MASK, ret=%d\n", ret);
			goto unlock;
		}
	}

	/* Low SOC1 */
	if (sm->low_soc1 == -EINVAL) {
		sm_err("low_soc1 is invalid\n");
	} else {
		ret = fg_update_bits(sm, sm->regs[SM_FG_REG_SOC_L_ALARM],
				     FG_SOC1_MASK,
				     (u16)(sm->low_soc1 & FG_SOC1_MASK));
		if (ret < 0) {
			sm_err("Failed to write SOC_L_ALARM (LOW_SOC1), ret=%d\n", ret);
			goto unlock;
		}
	}

	/* Low SOC2 */
	if (sm->low_soc2 == -EINVAL) {
		sm_err("low_soc2 is invalid\n");
	} else {
		ret = fg_update_bits(sm, sm->regs[SM_FG_REG_SOC_L_ALARM],
				     FG_SOC2_MASK,
				     (u16)((sm->low_soc2 << FG_SOC2_SHIFT) & FG_SOC2_MASK));
		if (ret < 0) {
			sm_err("Failed to write SOC_L_ALARM (LOW_SOC2), ret=%d\n", ret);
			goto unlock;
		}
	}

	/* V L ALARM */
	if (sm->v_l_alarm == -EINVAL) {
		sm_err("v_l_alarm is invalid\n");
	} else {
		if (sm->v_l_alarm >= 2000 && sm->v_l_alarm < 3000) {
			data = (u16)(FG_VALARM_RANGE_MASK & (((u16)(sm->v_l_alarm / 10)) << 8));
		} else if (sm->v_l_alarm >= 3000 && sm->v_l_alarm < 4000) {
			data = (u16)(FG_VALARM_RANGE_HI | (((u16)(sm->v_l_alarm / 10)) << 8));
		} else {
			sm_err("Failed to calculate V_L_ALARM\n");
			goto unlock;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_V_L_ALARM], data);
		if (ret < 0) {
			sm_err("Failed to write V_L_ALARM, ret=%d\n", ret);
			goto unlock;
		}
	}

	/* V H ALARM */
	if (sm->v_h_alarm == -EINVAL) {
		sm_err("v_h_alarm is invalid\n");
	} else {
		if (sm->v_h_alarm >= 3000 && sm->v_h_alarm < 4000) {
			data = (u16)(FG_VALARM_RANGE_MASK & (((u16)(sm->v_h_alarm / 10)) << 8));
		} else if (sm->v_h_alarm >= 4000 && sm->v_h_alarm < 5000) {
			data = (u16)(FG_VALARM_RANGE_HI | (((u16)(sm->v_h_alarm / 10)) << 8));
		} else {
			sm_err("Failed to calculate V_H_ALARM\n");
			goto unlock;
		}
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_V_H_ALARM], data);
		if (ret < 0) {
			sm_err("Failed to write V_H_ALARM, ret=%d\n", ret);
			goto unlock;
		}
	}

	/* T IN H/L ALARM */
	if (sm->t_h_alarm_in == -EINVAL || sm->t_l_alarm_in == -EINVAL) {
		sm_err("t_h_alarm_in || sm->t_l_alarm_in is invalid\n");
	} else {
		data = 0;
		if (sm->t_h_alarm_in < 0)
			data |= (u16)(FG_TEMP_IN_SIGN_HI | ((((u16)(-sm->t_h_alarm_in)) & FG_TEMP_IN_MAG7_MASK) << 8));
		else
			data |= (u16)(((u16)(sm->t_h_alarm_in) & FG_TEMP_IN_MAG7_MASK) << 8);

		if (sm->t_l_alarm_in < 0)
			data |= (u16)(FG_TEMP_IN_SIGN_LO | (u16)((-sm->t_l_alarm_in) & FG_TEMP_IN_MAG7_MASK));
		else
			data |= (u16)(sm->t_l_alarm_in & FG_TEMP_IN_MAG7_MASK);

		ret = fg_write_word(sm, sm->regs[SM_FG_REG_T_IN_H_ALARM], data);
		if (ret < 0) {
			sm_err("Failed to write SM_FG_REG_T_IN_H_ALARM, ret=%d\n", ret);
			goto unlock;
		}
	}

	do {
		ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL],
				    FG_PARAM_UNLOCK_CODE |
				    ((sm->battery_table_num & 0x0003) << 6) |
				    (FG_TABLE_LEN - 1));
		if (ret < 0) {
			sm_err("Failed to write param_ctrl unlock, ret=%d\n", ret);
			goto unlock;
		}
		sm_info("Param Unlock\n");
		msleep(60);
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_FG_OP_STATUS], &data);
		if (ret < 0)
			sm_err("Failed to read FG_OP_STATUS, ret = %d\n", ret);
		else
			sm_info("FG_OP_STATUS = 0x%x\n", data);
		cnt++;
	} while (((data & FG_OP_STATUS_PARAM_RDY) != FG_OP_STATUS_PARAM_RDY) && cnt <= 3);

	/* VIT_PERIOD write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);
	if (ret < 0) {
		sm_err("Failed to write VIT PERIOD, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("Write VIT_PERIOD = 0x%x:0x%x\n",
		sm->regs[SM_FG_REG_VIT_PERIOD], sm->vit_period);

	/* Aging ctrl write */
	ret = fg_write_word(sm, FG_REG_AGING_CTRL, sm->aging_ctrl);
	if (ret < 0) {
		sm_err("Failed to write FG_REG_AGING_CTRL, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("Write FG_REG_AGING_CTRL = 0x%x:0x%x\n",
		FG_REG_AGING_CTRL, sm->aging_ctrl);

	/* SOC Cycle ctrl write */
	ret = fg_write_word(sm, FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);
	if (ret < 0) {
		sm_err("Failed to write FG_REG_SOC_CYCLE_CFG, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("Write FG_REG_SOC_CYCLE_CFG = 0x%x:0x%x\n",
		FG_REG_SOC_CYCLE_CFG, sm->cycle_cfg);

	/* RSNS write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns);
	if (ret < 0) {
		sm_err("Failed to write SM_FG_REG_RSNS_SEL, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("Write SM_FG_REG_RSNS_SEL = 0x%x:0x%x\n",
		sm->regs[SM_FG_REG_RSNS_SEL], sm->batt_rsns);

	/* Battery_Table write */
	for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
		table_reg = 0xA0 + (i * FG_TABLE_LEN);
		for (j = 0; j < FG_TABLE_LEN; j++) {
			ret = fg_write_word(sm, (table_reg + j), sm->battery_table[i][j]);
			if (ret < 0) {
				sm_err("Failed to write Battery Table, ret=%d\n", ret);
				goto unlock;
			}
		}
	}

	for (j = 0; j < FG_ADD_TABLE_LEN; j++) {
		table_reg = 0xD0 + j;
		ret = fg_write_word(sm, table_reg, sm->battery_table[i][j]);
		if (ret < 0) {
			sm_err("Failed to write Battery Table, ret=%d\n", ret);
			goto unlock;
		}
	}

	/* RS write */
	ret = fg_write_word(sm, FG_REG_RS, sm->rs);
	if (ret < 0) {
		sm_err("Failed to write RS, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("RS = 0x%x:0x%x\n", FG_REG_RS, sm->rs);

	/* alpha write */
	ret = fg_write_word(sm, FG_REG_ALPHA, sm->alpha);
	if (ret < 0) {
		sm_err("Failed to write FG_REG_ALPHA, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("ALPHA = 0x%x:0x%x\n", FG_REG_ALPHA, sm->alpha);

	/* beta write */
	ret = fg_write_word(sm, FG_REG_BETA, sm->beta);
	if (ret < 0) {
		sm_err("Failed to write FG_REG_BETA, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("BETA = 0x%x:0x%x\n", FG_REG_BETA, sm->beta);

	/* RS_* write */
	ret = fg_write_word(sm, FG_REG_RS_0, sm->rs_value[0]);
	if (ret < 0) {
		sm_err("Failed to write RS_0, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("RS = 0x%x:0x%x\n", FG_REG_RS_0, sm->rs_value[0]);

	ret = fg_write_word(sm, FG_REG_RS_1, sm->rs_value[1]);
	if (ret < 0) {
		sm_err("Failed to write RS_1, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("RS_1 = 0x%x:0x%x\n", FG_REG_RS_1, sm->rs_value[1]);

	ret = fg_write_word(sm, FG_REG_RS_2, sm->rs_value[2]);
	if (ret < 0) {
		sm_err("Failed to write RS_2, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("RS_2 = 0x%x:0x%x\n", FG_REG_RS_2, sm->rs_value[2]);

	ret = fg_write_word(sm, FG_REG_RS_3, sm->rs_value[3]);
	if (ret < 0) {
		sm_err("Failed to write RS_3, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("RS_3 = 0x%x:0x%x\n", FG_REG_RS_3, sm->rs_value[3]);

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);
	if (ret < 0) {
		sm_err("Failed to write CURRENT_RATE, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("CURRENT_RATE = 0x%x:0x%x\n",
		sm->regs[SM_FG_REG_CURRENT_RATE], sm->mix_value);

	sm_info("RS_0 = 0x%x, RS_1 = 0x%x, RS_2 = 0x%x, RS_3 = 0x%x, CURRENT_RATE = 0x%x\n",
		sm->rs_value[0],
		sm->rs_value[1],
		sm->rs_value[2],
		sm->rs_value[3],
		sm->mix_value);

	/* VOLT_CAL write */
	ret = fg_write_word(sm, FG_REG_VOLT_CAL, sm->volt_cal);
	if (ret < 0) {
		sm_err("Failed to write FG_REG_VOLT_CAL, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("FG_REG_VOLT_CAL = 0x%x:0x%x\n",
		FG_REG_VOLT_CAL, sm->volt_cal);

	/* CAL write */
	ret = fg_write_word(sm, FG_REG_CURR_IN_OFFSET, sm->curr_offset);
	if (ret < 0) {
		sm_err("Failed to write CURR_IN_OFFSET, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("CURR_IN_OFFSET = 0x%x:0x%x\n",
		FG_REG_CURR_IN_OFFSET, sm->curr_offset);

	ret = fg_write_word(sm, FG_REG_CURR_IN_SLOPE, sm->curr_slope);
	if (ret < 0) {
		sm_err("Failed to write CURR_IN_SLOPE, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("BETA = 0x%x:0x%x\n",
		FG_REG_CURR_IN_SLOPE, sm->curr_slope);

	/* BAT CAP write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_BAT_CAP], sm->cap);
	if (ret < 0) {
		sm_err("Failed to write BAT_CAP, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("BAT_CAP = 0x%x:0x%x\n",
		sm->regs[SM_FG_REG_BAT_CAP], sm->cap);

	/* MISC write */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_MISC], sm->misc);
	if (ret < 0) {
		sm_err("Failed to write REG_MISC, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("REG_MISC 0x%x:0x%x\n",
		sm->regs[SM_FG_REG_MISC], sm->misc);

	/* TOPOFF SOC */
	ret = fg_write_word(sm, sm->regs[SM_FG_REG_TOPOFFSOC], sm->topoff_soc);
	if (ret < 0) {
		sm_err("Failed to write SM_FG_REG_TOPOFFSOC, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("SM_REG_TOPOFFSOC 0x%x:0x%x\n",
		sm->regs[SM_FG_REG_TOPOFFSOC], sm->topoff_soc);

	/* IOCV manual-mode */
	ret = fg_update_bits(sm, sm->regs[SM_FG_REG_CNTL],
			     ENABLE_IOCV_MAN_MODE,
			     sm->iocv_man_mode ? ENABLE_IOCV_MAN_MODE : 0);
	if (ret < 0) {
		sm_err("Failed to update CNTL (iocv_man_mode), ret=%d\n", ret);
		goto unlock;
	}
	sm_info("CNTL[0x%02x] iocv_man_mode=%d\n",
		sm->regs[SM_FG_REG_CNTL], sm->iocv_man_mode);

	/* Parameter Version */
	ret = fg_write_word(sm, FG_PARAM_VERSION,
			    (sm->common_param_version << 8) | sm->battery_param_version);
	if (ret < 0) {
		sm_err("Failed to write FG_PARAM_VERSION, ret=%d\n", ret);
		goto unlock;
	}

	/* T EX L ALARM */
	if (sm->t_l_alarm_ex == -EINVAL) {
		sm_err("t_l_alarm_ex is invalid\n");
	} else {
		data = (sm->t_l_alarm_ex) >> 1;
		ret = fg_write_word(sm, FG_REG_SWADDR, FG_SWADDR_T_EX_L_ALARM);
		if (ret < 0) {
			sm_err("Failed to write FG_REG_SWADDR, ret=%d\n", ret);
			goto unlock;
		}
		ret = fg_write_word(sm, FG_REG_SWDATA, data);
		if (ret < 0) {
			sm_err("Failed to write FG_REG_SWDATA (T_EX_L), ret=%d\n", ret);
			goto unlock;
		}
		sm_info("write to T_EX_L_ALARM = 0x%x\n", data);
	}

	/* T EX H ALARM */
	if (sm->t_h_alarm_ex == -EINVAL) {
		sm_err("t_h_alarm_ex is invalid\n");
	} else {
		data = (sm->t_h_alarm_ex) >> 1;
		ret = fg_write_word(sm, FG_REG_SWADDR, FG_SWADDR_T_EX_H_ALARM);
		if (ret < 0) {
			sm_err("Failed to write FG_REG_SWADDR, ret=%d\n", ret);
			goto unlock;
		}
		ret = fg_write_word(sm, FG_REG_SWDATA, data);
		if (ret < 0) {
			sm_err("Failed to write FG_REG_SWDATA (T_EX_H), ret=%d\n", ret);
			goto unlock;
		}
		sm_info("write to T_EX_H_ALARM = 0x%x\n", data);
	}

	if (sm->iocv_man_mode) {
		value = fg_calculate_iocv(sm);
		usleep_range(10000, 11000);
		ret = fg_write_word(sm, FG_REG_SWADDR, FG_SWADDR_IOCV_MAN);
		if (ret < 0) {
			sm_err("Failed to write FG_REG_SWADDR, ret=%d\n", ret);
			goto unlock;
		}
		ret = fg_write_word(sm, FG_REG_SWDATA, value);
		if (ret < 0) {
			sm_err("Failed to write FG_REG_SWDATA (IOCV), ret=%d\n", ret);
			goto unlock;
		}
		sm_info("IOCV_MAN = 0x%x\n", value);
	}
	usleep_range(20000, 21000);

	ret = fg_write_word(sm, sm->regs[SM_FG_REG_PARAM_CTRL],
			    (FG_PARAM_LOCK_CODE | (sm->battery_table_num & 0x0003) << 6) |
			    (FG_TABLE_LEN - 1));
	if (ret < 0) {
		sm_err("Failed to write param_ctrl lock, ret=%d\n", ret);
		goto unlock;
	}
	sm_info("Param Lock\n");

	if (sm->en_temp_ex)
		msleep(300);
	else
		msleep(160);

	ok = true;
unlock:
	mutex_unlock(&sm->data_lock);
	return ok;
}

static int fg_get_device_id(struct sm_fg_chip *sm)
{
	int ret;
	u16 data;

	ret = fg_read_word(sm, sm->regs[SM_FG_REG_DEVICE_ID], &data);
	if (ret < 0) {
		sm_err("Failed to read DEVICE_ID, ret=%d\n", ret);
		return ret;
	}

	sm_info("revision_id: 0x%x\n", (data & FG_DEV_REVISION_MASK));
	sm_info("device_id: 0x%x\n", (data & FG_DEV_ID_MASK) >> FG_DEV_ID_SHIFT);

	return (int)data;
}

static bool fg_check_device_id(struct sm_fg_chip *sm)
{
	int id = fg_get_device_id(sm);

	if (id < 0)
		return false;

	return ((id & FG_DEV_ID_MASK) >> FG_DEV_ID_SHIFT) == FG_DEV_ID_SM5602;
}

static bool fg_init(struct sm_fg_chip *sm)
{
	int ret;
	u16 data = 0, cntl_ref = 0;
	bool ok = false;

	if (atomic_read(&sm->init_running))
		return true;

	atomic_set(&sm->init_running, 1);

	if (!fg_check_reg_init_need(sm)) {
		sm_dbg("skip fg_reg_init (no init needed)\n");
		/* Control register audit & restore */
		ret = fg_read_word(sm, sm->regs[SM_FG_REG_CNTL], &data);
		if (ret < 0) {
			sm_err("Failed to read CNTL in skip path, ret=%d\n", ret);
			goto out;
		}

		if (sm->en_temp_in)
			cntl_ref |= ENABLE_EN_TEMP_IN;
		if (sm->en_temp_ex)
			cntl_ref |= ENABLE_EN_TEMP_EX;
		if (sm->en_batt_det)
			cntl_ref |= ENABLE_EN_BATT_DET;
		if (sm->iocv_man_mode)
			cntl_ref |= ENABLE_IOCV_MAN_MODE;

		if (data != cntl_ref) {
			sm_err("CNTL mismatch! hw=0x%04x ref=0x%04x, restoring...\n",
			       data, cntl_ref);
			ret = fg_write_word(sm, sm->regs[SM_FG_REG_CNTL], cntl_ref);
			if (ret < 0) {
				sm_err("Failed to rewrite CNTL, ret=%d\n", ret);
				goto out;
			}
		}

		ok = true;
		goto out;
	}

	ret = fg_reset(sm);
	if (ret < 0) {
		sm_err("fg_reset failed (%d)\n", ret);
		goto out;
	}

	sm_info("performing fg_reg_init\n");
	if (!fg_reg_init(sm)) {
		sm_err("fg_reg_init failed\n");
		goto out;
	}

	ok = true;

out:
	atomic_set(&sm->init_running, 0);
	return ok;
}

#define PROPERTY_NAME_SIZE 128
static int fg_common_parse_dt(struct sm_fg_chip *sm)
{
	struct device *dev = &sm->client->dev;
	struct device_node *np = dev->of_node;
	int ret, count;

	if (!np) {
		sm_err("No common device tree node found\n");
		return -ENODEV;
	}

	sm->en_temp_ex = of_property_read_bool(np, "sm,en_temp_ex");
	sm_info("Temperature EX enabled = %d\n", sm->en_temp_ex);

	sm->en_temp_in = of_property_read_bool(np, "sm,en_temp_in");
	sm_info("Temperature IN enabled = %d\n", sm->en_temp_in);

	sm->en_batt_det = of_property_read_bool(np, "sm,en_batt_det");
	sm_info("Batt Det enabled = %d\n", sm->en_batt_det);

	ret = of_property_read_u32(np, "sm,misc", &sm->misc);
	if (ret < 0)
		sm->misc = FG_MISC_DEFAULT;

	sm->iocv_man_mode = of_property_read_bool(np, "sm,iocv_man_mode");
	sm_info("IOCV_MAN_MODE = %d\n", sm->iocv_man_mode);

	ret = of_property_read_u32(np, "sm,aging_ctrl", &sm->aging_ctrl);
	if (ret < 0)
		sm->aging_ctrl = -EINVAL;

	count = of_property_count_u32_elems(np, "sm,soc_decimal_rate");
	if (count <= 0) {
		sm->dec_rate_seq = NULL;
		sm->dec_rate_len = 0;
	} else if (count < 2 || (count & 1)) {
		sm->dec_rate_seq = NULL;
		sm->dec_rate_len = 0;
	} else {
		sm->dec_rate_seq = devm_kcalloc(dev, count, sizeof(u32), GFP_KERNEL);
		if (!sm->dec_rate_seq) {
			sm->dec_rate_len = 0;
		} else {
			ret = of_property_read_u32_array(np, "sm,soc_decimal_rate", sm->dec_rate_seq, count);
			if (ret) {
				sm->dec_rate_seq = NULL;
				sm->dec_rate_len = 0;
			} else {
				sm->dec_rate_len = count;
				sm_info("decimal_rate = %d\n", sm->dec_rate_len);
			}
		}
	}

	ret = of_property_read_u32(np, "sm,cycle_cfg", &sm->cycle_cfg);
	if (ret < 0)
		sm->cycle_cfg = -EINVAL;

	ret = of_property_read_u32(np, "sm,rsns", &sm->batt_rsns);
	if (ret < 0)
		sm->batt_rsns = -EINVAL;

	ret = of_property_read_u32(np, "sm,fg_irq_set", &sm->fg_irq_set);
	if (ret < 0)
		sm->fg_irq_set = -EINVAL;

	ret = of_property_read_u32(np, "sm,low_soc1", &sm->low_soc1);
	if (ret < 0)
		sm->low_soc1 = -EINVAL;
	sm_info("low_soc1 = %d\n", sm->low_soc1);

	ret = of_property_read_u32(np, "sm,low_soc2", &sm->low_soc2);
	if (ret < 0)
		sm->low_soc2 = -EINVAL;
	sm_info("low_soc2 = %d\n", sm->low_soc2);

	ret = of_property_read_u32(np, "sm,v_l_alarm", &sm->v_l_alarm);
	if (ret < 0)
		sm->v_l_alarm = -EINVAL;
	sm_info("v_l_alarm = %d\n", sm->v_l_alarm);

	ret = of_property_read_u32(np, "sm,v_h_alarm", &sm->v_h_alarm);
	if (ret < 0)
		sm->v_h_alarm = -EINVAL;
	sm_info("v_h_alarm = %d\n", sm->v_h_alarm);

	ret = of_property_read_u32(np, "sm,t_l_alarm_in", &sm->t_l_alarm_in);
	if (ret < 0)
		sm->t_l_alarm_in = -EINVAL;
	sm_info("t_l_alarm_in = %d\n", sm->t_l_alarm_in);

	ret = of_property_read_u32(np, "sm,t_h_alarm_in", &sm->t_h_alarm_in);
	if (ret < 0)
		sm->t_h_alarm_in = -EINVAL;
	sm_info("t_h_alarm_in = %d\n", sm->t_h_alarm_in);

	ret = of_property_read_u32(np, "sm,t_l_alarm_ex", &sm->t_l_alarm_ex);
	if (ret < 0)
		sm->t_l_alarm_ex = -EINVAL;
	sm_info("t_l_alarm_ex = %d\n", sm->t_l_alarm_ex);

	ret = of_property_read_u32(np, "sm,t_h_alarm_ex", &sm->t_h_alarm_ex);
	if (ret < 0)
		sm->t_h_alarm_ex = -EINVAL;
	sm_info("t_h_alarm_ex = %d\n", sm->t_h_alarm_ex);

	ret = of_property_read_u32(np, "sm,battery_table_num", &sm->battery_table_num);
	if (ret < 0)
		sm->battery_table_num = -EINVAL;

	ret = of_property_read_u32(np, "sm,param_version", &sm->common_param_version);
	if (ret < 0)
		sm->common_param_version = -EINVAL;

	sm->shutdown_delay_enable = of_property_read_bool(np, "sm,shutdown-delay-enable");

#ifdef ENABLE_NTC_COMPENSATION
	ret = of_property_read_u32(np, "sm,rtrace", &sm->rtrace);
	if (ret < 0)
		sm->rtrace = 0;
#endif

	return 0;
}

static int fg_battery_parse_dt(struct sm_fg_chip *sm)
{
	struct device *dev = &sm->client->dev;
	struct device_node *root = dev->of_node;
	struct device_node *bp = NULL;
	char prop_name[PROPERTY_NAME_SIZE];
	int battery_id = BATTERY_VENDOR_UNKNOWN;
	int *battery_temp_table;
	int table[FG_TABLE_LEN] = {0};
	int rs_value[4] = {0};
	int topoff_soc[3] = {0};
	int temp_offset[6] = {0};
	int temp_cal[10] = {0};
	int ext_temp_cal[10] = {0};
	int battery_type[3] = {0};
	int set_tempoff[2] = {0};
	int ret = 0;
	int i, j;

	if (!root) {
		sm_err("No battery device tree node found\n");
		return -ENODEV;
	}

	bp = of_get_child_by_name(root, "battery_params");
	if (!bp) {
		sm_err("couldn't find child node \"battery_params\"\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(bp, "battery,id", &battery_id);
	if (ret < 0)
		sm_dbg("no battery_id property, get from authen\n");

	for (i = 0; i < 3; i++) {
		battery_id = get_battery_id(sm);

		if (battery_id != BATTERY_VENDOR_UNKNOWN)
			break;

		sm_dbg("battery_id unknown (%d), retrying...\n", i + 1);
		usleep_range(5000, 6000);
	}

	if (battery_id == BATTERY_VENDOR_UNKNOWN) {
		sm_err("battery id still unknown, use fallback\n");
		battery_id = 1;
	} else {
		sm_info("battery id = %d\n", battery_id);
	}

	for (i = BATTERY_TABLE0; i < BATTERY_TABLE2; i++) {
		snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s%d", battery_id, "battery_table", i);
		ret = of_property_read_u32_array(bp, prop_name, table, FG_TABLE_LEN);
		if (ret < 0) {
			sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
			continue;
		}
		for (j = 0; j < FG_TABLE_LEN; j++)
			sm->battery_table[i][j] = table[j];
	}

	i = BATTERY_TABLE2;
	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s%d", battery_id, "battery_table", i);
	ret = of_property_read_u32_array(bp, prop_name, table, FG_ADD_TABLE_LEN);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		for (j = 0; j < FG_ADD_TABLE_LEN; j++)
			sm->battery_table[i][j] = table[j];
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "rs");
	ret = of_property_read_u32_array(bp, prop_name, &sm->rs, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->rs);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "alpha");
	ret = of_property_read_u32_array(bp, prop_name, &sm->alpha, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->alpha);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "beta");
	ret = of_property_read_u32_array(bp, prop_name, &sm->beta, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->beta);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "rs_value");
	ret = of_property_read_u32_array(bp, prop_name, rs_value, 4);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		for (i = 0; i < 4; i++)
			sm->rs_value[i] = rs_value[i];
		sm_info("%s = <0x%x 0x%x 0x%x 0x%x>\n",
			prop_name, rs_value[0], rs_value[1], rs_value[2], rs_value[3]);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "vit_period");
	ret = of_property_read_u32_array(bp, prop_name, &sm->vit_period, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->vit_period);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "battery_type");
	ret = of_property_read_u32_array(bp, prop_name, battery_type, 3);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		sm->batt_v_max = battery_type[0];
		sm->min_cap = battery_type[1];
		sm->cap = battery_type[2];
		sm_info("%s = <%d %d %d>\n", prop_name, sm->batt_v_max, sm->min_cap, sm->cap);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "tempoff");
	ret = of_property_read_u32_array(bp, prop_name, set_tempoff, 2);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		sm->n_tempoff = set_tempoff[0];
		sm->n_tempoff_offset = set_tempoff[1];
		sm_info("%s = <%d %d>\n", prop_name, sm->n_tempoff, sm->n_tempoff_offset);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "max_voltage_uv");
	ret = of_property_read_u32(bp, prop_name, &sm->batt_max_voltage_uv);
	if (ret < 0)
		sm_err("couldn't find battery max voltage\n");

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "topoff_soc");
	ret = of_property_read_u32_array(bp, prop_name, topoff_soc, 3);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		sm->topoff_soc = topoff_soc[0];
		sm->topoff = topoff_soc[1];
		sm->topoff_margin = topoff_soc[2];
		sm_info("%s = <%d %d %d>\n", prop_name, sm->topoff_soc, sm->topoff, sm->topoff_margin);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "mix_value");
	ret = of_property_read_u32_array(bp, prop_name, &sm->mix_value, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <%d>\n", prop_name, sm->mix_value);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "volt_cal");
	ret = of_property_read_u32_array(bp, prop_name, &sm->volt_cal, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->volt_cal);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "curr_offset");
	ret = of_property_read_u32_array(bp, prop_name, &sm->curr_offset, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->curr_offset);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "curr_slope");
	ret = of_property_read_u32_array(bp, prop_name, &sm->curr_slope, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->curr_slope);

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_std");
	ret = of_property_read_u32_array(bp, prop_name, &sm->temp_std, 1);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		sm->temp_std *= 10;
		sm_info("%s = <%d>\n", prop_name, sm->temp_std);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_offset");
	ret = of_property_read_u32_array(bp, prop_name, temp_offset, 6);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		sm->en_high_fg_temp_offset = !!temp_offset[0];
		sm->high_fg_temp_offset_denom = temp_offset[1];
		sm->high_fg_temp_offset_fact = temp_offset[2];
		sm->en_low_fg_temp_offset = !!temp_offset[3];
		sm->low_fg_temp_offset_denom = temp_offset[4];
		sm->low_fg_temp_offset_fact = temp_offset[5];
		sm_info("%s = <%d, %d, %d, %d, %d, %d>\n", prop_name,
			sm->en_high_fg_temp_offset, sm->high_fg_temp_offset_denom, sm->high_fg_temp_offset_fact,
			sm->en_low_fg_temp_offset, sm->low_fg_temp_offset_denom, sm->low_fg_temp_offset_fact);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "temp_cal");
	ret = of_property_read_u32_array(bp, prop_name, temp_cal, 10);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		sm->en_high_fg_temp_cal = !!temp_cal[0];
		sm->high_fg_temp_p_cal_denom = temp_cal[1];
		sm->high_fg_temp_p_cal_fact = temp_cal[2];
		sm->high_fg_temp_n_cal_denom = temp_cal[3];
		sm->high_fg_temp_n_cal_fact = temp_cal[4];
		sm->en_low_fg_temp_cal = !!temp_cal[5];
		sm->low_fg_temp_p_cal_denom = temp_cal[6];
		sm->low_fg_temp_p_cal_fact = temp_cal[7];
		sm->low_fg_temp_n_cal_denom = temp_cal[8];
		sm->low_fg_temp_n_cal_fact = temp_cal[9];
		sm_info("%s = <%d, %d, %d, %d, %d, %d, %d, %d, %d, %d>\n", prop_name,
			sm->en_high_fg_temp_cal, sm->high_fg_temp_p_cal_denom, sm->high_fg_temp_p_cal_fact,
			sm->high_fg_temp_n_cal_denom, sm->high_fg_temp_n_cal_fact, sm->en_low_fg_temp_cal,
			sm->low_fg_temp_p_cal_denom, sm->low_fg_temp_p_cal_fact, sm->low_fg_temp_n_cal_denom, sm->low_fg_temp_n_cal_fact);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "ext_temp_cal");
	ret = of_property_read_u32_array(bp, prop_name, ext_temp_cal, 10);
	if (ret < 0) {
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	} else {
		sm->en_high_temp_cal = !!ext_temp_cal[0];
		sm->high_temp_p_cal_denom = ext_temp_cal[1];
		sm->high_temp_p_cal_fact = ext_temp_cal[2];
		sm->high_temp_n_cal_denom = ext_temp_cal[3];
		sm->high_temp_n_cal_fact = ext_temp_cal[4];
		sm->en_low_temp_cal = !!ext_temp_cal[5];
		sm->low_temp_p_cal_denom = ext_temp_cal[6];
		sm->low_temp_p_cal_fact = ext_temp_cal[7];
		sm->low_temp_n_cal_denom = ext_temp_cal[8];
		sm->low_temp_n_cal_fact = ext_temp_cal[9];
		sm_info("%s = <%d, %d, %d, %d, %d, %d, %d, %d, %d, %d>\n", prop_name,
			sm->en_high_temp_cal, sm->high_temp_p_cal_denom, sm->high_temp_p_cal_fact,
			sm->high_temp_n_cal_denom, sm->high_temp_n_cal_fact, sm->en_low_temp_cal,
			sm->low_temp_p_cal_denom, sm->low_temp_p_cal_fact, sm->low_temp_n_cal_denom, sm->low_temp_n_cal_fact);
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "fcm_offset");
	ret = of_property_read_u32_array(bp, prop_name, &sm->fcm_offset, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->fcm_offset);

	battery_temp_table = kcalloc(FG_TEMP_TABLE_CNT_MAX, sizeof(int), GFP_KERNEL);
	if (battery_temp_table) {
		snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "thermal_table");
		ret = of_property_read_u32_array(bp, prop_name, battery_temp_table, FG_TEMP_TABLE_CNT_MAX);
		if (ret < 0) {
			sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
		} else {
			for (i = 0; i < FG_TEMP_TABLE_CNT_MAX; i++)
				sm->battery_temp_table[i] = (u16)battery_temp_table[i];
		}
		kfree(battery_temp_table);
	} else {
		sm_err("failed to allocate memory for battery_temp_table\n");
	}

	snprintf(prop_name, PROPERTY_NAME_SIZE, "battery%d,%s", battery_id, "param_version");
	ret = of_property_read_u32_array(bp, prop_name, &sm->battery_param_version, 1);
	if (ret < 0)
		sm_err("couldn't get prop %s (%d)\n", prop_name, ret);
	else
		sm_info("%s = <0x%x>\n", prop_name, sm->battery_param_version);

	of_node_put(bp);
	return 0;
}

static bool hal_fg_init(struct sm_fg_chip *sm)
{
	int ret;
	bool ok;

	sm_info("start\n");
	if (sm->client->dev.of_node) {
		ret = fg_common_parse_dt(sm);
		if (ret) {
			sm_err("fg_common_parse_dt failed: %d\n", ret);
			goto out;
		}

		ret = fg_battery_parse_dt(sm);
		if (ret) {
			sm_err("fg_battery_parse_dt failed: %d\n", ret);
			goto out;
		}
	}

	ok = fg_init(sm);
	if (!ok) {
		sm_err("fg_init failed\n");
		goto out;
	}

	sm_info("hal_init done\n");
	return true;
out:
	sm_info("hal_init failed\n");
	return false;
}

static int sm5602_get_psy(struct sm_fg_chip *sm)
{
	if (sm->usb_psy && sm->batt_psy)
		return 0;

	sm->usb_psy = power_supply_get_by_name("usb");
	if (!sm->usb_psy) {
		sm_err("usb psy not ready, force probe\n");
		return -EINVAL;
	}

	sm->batt_psy = power_supply_get_by_name("battery");
	if (!sm->batt_psy) {
		sm_err("batt psy not ready, force probe\n");
		return -EINVAL;
	}

	return 0;
}

static void sm5602_put_psy(struct sm_fg_chip *sm)
{
	if (sm->batt_psy) {
		power_supply_put(sm->batt_psy);
		sm->batt_psy = NULL;
	}
	if (sm->usb_psy) {
		power_supply_put(sm->usb_psy);
		sm->usb_psy = NULL;
	}
	if (sm->cp_psy) {
		power_supply_put(sm->cp_psy);
		sm->cp_psy = NULL;
	}
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	if (sm->verify_psy) {
		power_supply_put(sm->verify_psy);
		sm->verify_psy = NULL;
	}
#endif
}

static void overtemp_timer_cb(struct timer_list *tm)
{
	struct sm_fg_chip *sm = from_timer(sm, tm, overtemp_timer);

	sm->overtemp_allow_restart = true;
}

static void usb_notify_workfunc(struct work_struct *work)
{
	struct sm_fg_chip *sm = container_of(work, struct sm_fg_chip, usb_notify_work);
	union power_supply_propval pval = {0, };
	bool prev_present;
	int ret;

	if (sm5602_get_psy(sm) < 0)
		return;

	ret = power_supply_get_property(sm->usb_psy,
					POWER_SUPPLY_PROP_PRESENT,
					&pval);
	if (ret < 0) {
		sm_err("failed get usb present\n");
		return;
	}

	mutex_lock(&sm->data_lock);
	prev_present = sm->usb_present;
	sm->usb_present = !!pval.intval;
	mutex_unlock(&sm->data_lock);

	if (sm->usb_present && !prev_present)
		sm_info("USB connected\n");
	else if (!sm->usb_present && prev_present)
		sm_info("USB disconnected\n");
}

static int sm5602_notifier_call(struct notifier_block *nb, unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct sm_fg_chip *sm = container_of(nb, struct sm_fg_chip, nb);

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_DONE;

	if (sm->usb_psy && psy != sm->usb_psy)
		return NOTIFY_DONE;

	schedule_work(&sm->usb_notify_work);

	return NOTIFY_OK;
}

static int sm_fg_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct power_supply *cp_psy = NULL;
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	struct power_supply *verify_psy = NULL;
#endif
	struct bq2589x *bq = NULL;
	struct sm_fg_chip *sm = NULL;
	const u8 *regs;
	static int probe_cnt;
	int i, ret;

	sm_info("enter, probe_cnt: %d\n", ++probe_cnt);
	if (probe_cnt >= PROBE_CNT_MAX) {
		sm_err("probe count exceeded: %d >= %d\n",
		       probe_cnt, PROBE_CNT_MAX);
		return -ENODEV;
	}

	bq = bbc_dev_get_by_name(BQ2589X_DEV_NAME);
	if (!bq) {
		sm_err("bq2589x not ready\n");
		return -EPROBE_DEFER;
	}

#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	verify_psy = power_supply_get_by_name("batt_verify");
	if (!verify_psy) {
		sm_err("batt_verify psy not ready, defer probe\n");
		ret = -EPROBE_DEFER;
		goto err_put_bq;
	}
#endif

	cp_psy = power_supply_get_by_name("sc8551-standalone");
	if (!cp_psy) {
		sm_err("cp_psy not ready, defer probe\n");
		ret = -EPROBE_DEFER;
		goto err_put_verify;
	}

	sm = devm_kzalloc(&client->dev, sizeof(*sm), GFP_KERNEL);
	if (!sm) {
		ret = -ENOMEM;
		goto err_put_cp;
	}

	sm->dev = &client->dev;
	sm->client = client;
	i2c_set_clientdata(client, sm);

	sm->regmap = devm_regmap_init_i2c(sm->client, &sm5602_regmap_config);
	if (IS_ERR(sm->regmap)) {
		ret = PTR_ERR(sm->regmap);
		sm_err("failed to init regmap: %d\n", ret);
		goto err_free;
	}

	sm->chip = id->driver_data;
	if (sm->chip == SM5602) {
		regs = sm5602_regs;
	} else {
		sm_err("unexpected fuel gauge: %d\n", sm->chip);
		regs = sm5602_regs;
	}

	memcpy(sm->regs, regs, ARRAY_SIZE(sm->regs) * sizeof(sm->regs[0]));
	mutex_init(&sm->data_lock);

	sm->bq = bq;
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	sm->verify_psy = verify_psy;
#endif
	sm->cp_psy = cp_psy;
	/* Non-Validated, Get it later if not ready */
	sm->usb_psy = power_supply_get_by_name("usb");
	sm->batt_psy = power_supply_get_by_name("battery");
	sm->batt_soc = -ENODATA;
	sm->batt_fcc = -ENODATA;
	sm->batt_volt = -ENODATA;
	sm->batt_curr = -ENODATA;
	sm->batt_temp = -ENODATA;
	sm->batt_rmc = -ENODATA;
	sm->batt_ocv = -ENODATA;
	sm->batt_soc_cycle = -ENODATA;
	sm->fake_soc = -EINVAL;
	sm->fake_temp = -EINVAL;
	sm->soc_smooth_initialized = false;
	sm->param.batt_soc = -EINVAL;
	sm->param.batt_temp_avg = -EINVAL;
	sm->param.batt_ma_avg = -EINVAL;
	atomic_set(&sm->init_running, 0);

	for (i = 0; i < SM_RL_SLOTS; i++) {
		ratelimit_state_init(&sm->rl_slots[i], SM_RL_INTERVAL, SM_RL_BURST);
		sm->rl_slots[i].flags |= RATELIMIT_MSG_ON_RELEASE;
	}

	if (!fg_check_device_id(sm)) {
		sm_err("device ID check failed\n");
		ret = -ENODEV;
		goto err_put_psy;
	}

	if (!hal_fg_init(sm)) {
		sm_err("Failed to Initialize Fuelgauge\n");
		ret = -EIO;
		goto err_put_psy;
	}

	kthread_init_delayed_work(&sm->monitor_work, fg_monitor_workfunc);
	timer_setup(&sm->overtemp_timer, overtemp_timer_cb, 0);
	INIT_WORK(&sm->usb_notify_work, usb_notify_workfunc);

	sm->main_worker = kthread_create_worker(0, "sm5602_fg");
	if (IS_ERR(sm->main_worker)) {
		ret = PTR_ERR(sm->main_worker);
		sm_err("failed to create main_worker: %d\n", ret);
		goto err_put_psy;
	}
	sched_set_fifo(sm->main_worker->task);

	sm_info("overtemp_delay_on: %d\n", sm->overtemp_delay_on);

	sm->fv_votable = find_votable("FV");
	sm->chg_dis_votable = find_votable("CHG_DISABLE");

	ret = sysfs_create_group(&sm->dev->kobj, &fg_attr_group);
	if (ret) {
		sm_err("Failed to register sysfs: %d\n", ret);
		goto err_wq;
	}

	create_debugfs_entry(sm);

	ret = fg_psy_register(sm);
	if (ret) {
		sm_err("fg_psy_register failed\n");
		goto err_sysfs;
	}

	sm->nb.notifier_call = sm5602_notifier_call;
	ret = power_supply_reg_notifier(&sm->nb);
	if (ret < 0) {
		sm_err("Couldn't register psy notifier, ret=%d\n", ret);
		goto err_sysfs;
	}

	sm->chg_nb.notifier_call = sm_fg_charger_event;
	ret = bq_charger_register_notifier(sm->bq, &sm->chg_nb);
	if (ret < 0) {
		sm_err("bq charger notifier register failed: %d\n", ret);
		goto err_batt_nb;
	}

	kthread_queue_delayed_work(sm->main_worker, &sm->monitor_work, 5 * HZ);
	sm_info("sm fuel gauge probe successfully, %s\n", device2str[sm->chip]);

	return 0;

err_batt_nb:
	power_supply_unreg_notifier(&sm->nb);
err_sysfs:
	if (!IS_ERR_OR_NULL(sm->debug_root))
		debugfs_remove_recursive(sm->debug_root);
	sysfs_remove_group(&sm->dev->kobj, &fg_attr_group);
err_wq:
	if (!IS_ERR_OR_NULL(sm->main_worker))
		kthread_destroy_worker(sm->main_worker);
err_put_psy:
	/* Validate put if ready */
	if (sm->batt_psy) {
		power_supply_put(sm->batt_psy);
		sm->batt_psy = NULL;
	}
	if (sm->usb_psy) {
		power_supply_put(sm->usb_psy);
		sm->usb_psy = NULL;
	}
	mutex_destroy(&sm->data_lock);
err_free:
	i2c_set_clientdata(client, NULL);
err_put_cp:
	power_supply_put(cp_psy);
err_put_verify:
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	power_supply_put(verify_psy);
err_put_bq:
#endif
	bbc_dev_put(bq);

	sm_err("sm fuel gauge probe failed\n");
	return ret;
}

static int sm_fg_remove(struct i2c_client *client)
{
	struct sm_fg_chip *sm = i2c_get_clientdata(client);

	if (!sm)
		return 0;

	bq_charger_unregister_notifier(sm->bq, &sm->chg_nb);
	power_supply_unreg_notifier(&sm->nb);

	kthread_cancel_delayed_work_sync(&sm->monitor_work);
	del_timer_sync(&sm->overtemp_timer);
	cancel_work_sync(&sm->usb_notify_work);

	if (!IS_ERR_OR_NULL(sm->debug_root))
		debugfs_remove_recursive(sm->debug_root);
	sysfs_remove_group(&sm->dev->kobj, &fg_attr_group);

	if (sm->main_worker)
		kthread_destroy_worker(sm->main_worker);

	sm5602_put_psy(sm);
	if (sm->bq)
		bbc_dev_put(sm->bq);

	mutex_destroy(&sm->data_lock);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static void sm_fg_shutdown(struct i2c_client *client)
{
	sm_info("sm fuel gauge driver shutdown!\n");
}

static const struct of_device_id sm_fg_match_table[] = {
	{ .compatible = "sm,sm5602", },
	{ },
};
MODULE_DEVICE_TABLE(of, sm_fg_match_table);

static const struct i2c_device_id sm_fg_id[] = {
	{ "sm5602", SM5602 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm_fg_id);

static struct i2c_driver sm_fg_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "sm5602",
		.of_match_table	= of_match_ptr(sm_fg_match_table),
	},
	.id_table	= sm_fg_id,
	.probe		= sm_fg_probe,
	.remove		= sm_fg_remove,
	.shutdown	= sm_fg_shutdown,
};

module_i2c_driver(sm_fg_driver);

MODULE_DESCRIPTION("SM SM5602 Gauge Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Siliconmitus");
