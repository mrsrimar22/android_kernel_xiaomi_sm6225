/* SPDX-License-Identifier: GPL-2.0 */
/*
 * aw87xxx_device.h  aw87xxx pa module
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef __AW87XXX_DEVICE_H__
#define __AW87XXX_DEVICE_H__

#include <linux/bits.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/regmap.h>
#include <sound/control.h>

#define AW_PRODUCT_NAME_LEN				8

#define AW_GPIO_HIGHT_LEVEL				1
#define AW_GPIO_LOW_LEVEL				0

#define AW_I2C_RETRIES					5
#define AW_I2C_RETRY_DELAY				2
#define AW_I2C_READ_MSG_NUM				2

#define AW_READ_CHIPID_RETRIES				5
#define AW_READ_CHIPID_RETRY_DELAY			2

#define AW_DEV_REG_INVALID_MASK				0xFF

#define AW_PID_9B_BIN_REG_CFG_COUNT			10

#define AW87XXX_DELAY_REG_ADDR				0xFE
#define AW87XXX_REG_DELAY_TIME				1000

#define AW_BOOST_VOLTAGE_MIN				0x00

#define AW_REG_NONE					0xFF

#define AW_LOCK_SEQUENCE_MAX				2

/*
 * aw87xxx register attributes
 */
#define AW87XXX_CHIPIDL_REG				0x00
#define AW87XXX_SW_RESET_PASSWORD			0xAA
#define AW87XXX_CHIPIDH_REG				0x01

#define AW87XXX_CP_OVP_START				0
#define AW87XXX_CP_OVP_LEN				4
#define AW87XXX_CP_OVP_MASK				((u8)~GENMASK(3, 0))

#define AW87XXX_EF_LOCK_STRAT_BIT			7
#define AW87XXX_EF_LOCK_BITS_LEN			1
#define AW87XXX_EF_LOCK_MASK				((u8)~GENMASK(7, 7))
#define AW87XXX_EF_LOCK_ENABLE				1
#define AW87XXX_EF_LOCK_ENABLE_VALUE			BIT(AW87XXX_EF_LOCK_STRAT_BIT)

#define AW87XXX_PID_9B_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_9B_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_9B_REG_MAX				0x63
#define AW87XXX_PID_9B_SYSCTRL_DEFAULT			0x03
#define AW87XXX_PID_9B_SYSCTRL_REG			0x01

#define AW87XXX_PID_9B_SPK_MODE_START_BIT		0
#define AW87XXX_PID_9B_SPK_MODE_BITS_LEN		1
#define AW87XXX_PID_9B_SPK_MODE_MASK			((u8)~GENMASK(0, 0))
#define AW87XXX_PID_9B_SPK_MODE_DISABLE			0
#define AW87XXX_PID_9B_SPK_MODE_DISABLE_VALUE		0
#define AW87XXX_PID_9B_SPK_MODE_ENABLE			1
#define AW87XXX_PID_9B_SPK_MODE_ENABLE_VALUE		BIT(0)

#define AW87XXX_PID_9B_REG_EN_SW_START_BIT		2
#define AW87XXX_PID_9B_REG_EN_SW_BITS_LEN		1
#define AW87XXX_PID_9B_REG_EN_SW_MASK			((u8)~GENMASK(2, 2))
#define AW87XXX_PID_9B_REG_EN_SW_DISABLE		0
#define AW87XXX_PID_9B_REG_EN_SW_DISABLE_VALUE		0
#define AW87XXX_PID_9B_REG_EN_SW_ENABLE			1
#define AW87XXX_PID_9B_REG_EN_SW_ENABLE_VALUE		BIT(2)

#define AW87XXX_PID_9B_ENCRYPTION_REG			0x64
#define AW87XXX_PID_9B_ENCRYPTION_BOOST_OUTPUT_SET	0x2C

#define AW87XXX_PID_18_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_18_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_18_REG_MAX				0x66
#define AW87XXX_PID_18_SYSCTRL_REG			0x03

#define AW87XXX_PID_18_REG_REC_MODE_START_BIT		1
#define AW87XXX_PID_18_REG_REC_MODE_BITS_LEN		1
#define AW87XXX_PID_18_REG_REC_MODE_MASK		((u8)~GENMASK(1, 1))
#define AW87XXX_PID_18_REG_REC_MODE_DISABLE		0
#define AW87XXX_PID_18_REG_REC_MODE_DISABLE_VALUE	0
#define AW87XXX_PID_18_REG_REC_MODE_ENABLE		1
#define AW87XXX_PID_18_REG_REC_MODE_ENABLE_VALUE	BIT(1)

#define AW87XXX_PID_18_REG_EN_SW_START_BIT		6
#define AW87XXX_PID_18_REG_EN_SW_BITS_LEN		1
#define AW87XXX_PID_18_REG_EN_SW_MASK			((u8)~GENMASK(6, 6))
#define AW87XXX_PID_18_REG_EN_SW_DISABLE		0
#define AW87XXX_PID_18_REG_EN_SW_DISABLE_VALUE		0
#define AW87XXX_PID_18_REG_EN_SW_ENABLE			1
#define AW87XXX_PID_18_REG_EN_SW_ENABLE_VALUE		BIT(6)

#define AW87XXX_PID_18_CLASSD_REG			0x05
#define AW87XXX_PID_18_CLASSD_DEFAULT			0x10
#define AW87XXX_PID_18_CPOC_REG				0x04

#define AW87XXX_PID_39_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_39_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_39_REG_MAX				0x64
#define AW87XXX_PID_39_REG_MODECTRL			0x02
#define AW87XXX_PID_39_MODECTRL_DEFAULT			0xA0

#define AW87XXX_PID_39_REC_MODE_START_BIT		3
#define AW87XXX_PID_39_REC_MODE_BITS_LEN		1
#define AW87XXX_PID_39_REC_MODE_MASK			((u8)~GENMASK(3, 3))
#define AW87XXX_PID_39_REC_MODE_DISABLE			0
#define AW87XXX_PID_39_REC_MODE_DISABLE_VALUE		0
#define AW87XXX_PID_39_REC_MODE_ENABLE			1
#define AW87XXX_PID_39_REC_MODE_ENABLE_VALUE		BIT(3)

#define AW87XXX_PID_39_REG_CPOVP			0x03

#define AW87XXX_PID_59_5X9_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_59_5X9_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_59_5X9_REG_MAX			0x69
#define AW87XXX_PID_59_5X9_REG_SYSCTRL			0x01

#define AW87XXX_PID_59_5X9_REC_MODE_START_BIT		3
#define AW87XXX_PID_59_5X9_REC_MODE_BITS_LEN		1
#define AW87XXX_PID_59_5X9_REC_MODE_MASK		((u8)~GENMASK(3, 3))
#define AW87XXX_PID_59_5X9_REC_MODE_DISABLE		0
#define AW87XXX_PID_59_5X9_REC_MODE_DISABLE_VALUE	0
#define AW87XXX_PID_59_5X9_REC_MODE_ENABLE		1
#define AW87XXX_PID_59_5X9_REC_MODE_ENABLE_VALUE	BIT(3)

#define AW87XXX_PID_59_5X9_REG_ENCR			0x69
#define AW87XXX_PID_59_5X9_ENCRY_DEFAULT		0x00

#define AW87XXX_PID_59_3X9_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_59_3X9_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_59_3X9_REG_MAX			0x70
#define AW87XXX_PID_59_3X9_REG_MDCRTL			0x02

#define AW87XXX_PID_59_3X9_SPK_MODE_START_BIT		2
#define AW87XXX_PID_59_3X9_SPK_MODE_BITS_LEN		1
#define AW87XXX_PID_59_3X9_SPK_MODE_MASK		((u8)~GENMASK(2, 2))
#define AW87XXX_PID_59_3X9_SPK_MODE_DISABLE		0
#define AW87XXX_PID_59_3X9_SPK_MODE_DISABLE_VALUE	0
#define AW87XXX_PID_59_3X9_SPK_MODE_ENABLE		1
#define AW87XXX_PID_59_3X9_SPK_MODE_ENABLE_VALUE	BIT(2)

#define AW87XXX_PID_59_3X9_REG_CPOVP			0x03
#define AW87XXX_PID_59_3X9_REG_ENCR			0x70
#define AW87XXX_PID_59_3X9_ENCR_DEFAULT			0x00

#define AW87XXX_PID_5A_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_5A_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_5A_REG_MAX				0x79

#define AW87XXX_PID_5A_REG_ID_REG			0x00
#define AW87XXX_PID_5A_REG_SYSCTRL_REG			0x01
#define AW87XXX_PID_5A_REG_BATSAFE_REG			0x02
#define AW87XXX_PID_5A_REG_BSTOVR_REG			0x03
#define AW87XXX_PID_5A_REG_BSTCPR1_REG			0x04
#define AW87XXX_PID_5A_REG_BSTCPR2_REG			0x05
#define AW87XXX_PID_5A_REG_PAGR_REG			0x06
#define AW87XXX_PID_5A_REG_PAGC3OPR_REG			0x07
#define AW87XXX_PID_5A_REG_PAGC3PR_REG			0x08
#define AW87XXX_PID_5A_REG_PAGC2OPR_REG			0x09
#define AW87XXX_PID_5A_REG_PAGC2PR_REG			0x0A
#define AW87XXX_PID_5A_REG_PAGC1PR_REG			0x0B
#define AW87XXX_PID_5A_REG_ADP_MODE_REG			0x0C
#define AW87XXX_PID_5A_REG_ADPBST_TIME1_REG		0x0D
#define AW87XXX_PID_5A_REG_ADPBST_TIME2_REG		0x0E
#define AW87XXX_PID_5A_REG_ADPBST_VTH_REG		0x0F
#define AW87XXX_PID_5A_REG_BOOST_PAR_REG		0x10
#define AW87XXX_PID_5A_REG_BOOST_VOUT_DET_REG		0x57
#define AW87XXX_PID_5A_REG_SYSST_REG			0x58
#define AW87XXX_PID_5A_REG_SYSINT_REG			0x59
#define AW87XXX_PID_5A_REG_DFT1R_REG			0x60
#define AW87XXX_PID_5A_REG_DFT2R_REG			0x61
#define AW87XXX_PID_5A_REG_DFT3R_REG			0x62
#define AW87XXX_PID_5A_REG_DFT4R_REG			0x63
#define AW87XXX_PID_5A_REG_DFT5R_REG			0x64
#define AW87XXX_PID_5A_REG_DFT6R_REG			0x65
#define AW87XXX_PID_5A_REG_DFT7R_REG			0x66
#define AW87XXX_PID_5A_REG_DFT8R_REG			0x67
#define AW87XXX_PID_5A_REG_DFT9R_REG			0x68
#define AW87XXX_PID_5A_REG_DFTAR_REG			0x69
#define AW87XXX_PID_5A_REG_DFTBR_REG			0x70
#define AW87XXX_PID_5A_REG_DFTCR_REG			0x71
#define AW87XXX_PID_5A_REG_DFTDR_REG			0x72
#define AW87XXX_PID_5A_REG_DFTER_REG			0x73
#define AW87XXX_PID_5A_REG_DFTFR_REG			0x74
#define AW87XXX_PID_5A_REG_test1_REG			0x75
#define AW87XXX_PID_5A_REG_test2_REG			0x76
#define AW87XXX_PID_5A_REG_ENCR_REG			0x77
#define AW87XXX_PID_5A_REG_test3_REG			0x78
#define AW87XXX_PID_5A_REG_test4_REG			0x79

#define AW87XXX_PID_5A_DFT3R_DEFAULT			0x02

#define AW87XXX_PID_5A_REG_RCV_MODE_START_BIT		2
#define AW87XXX_PID_5A_REG_RCV_MODE_BITS_LEN		1
#define AW87XXX_PID_5A_REG_RCV_MODE_MASK		((u8)~GENMASK(2, 2))
#define AW87XXX_PID_5A_REG_RCV_MODE_DISABLE		0
#define AW87XXX_PID_5A_REG_RCV_MODE_DISABLE_VALUE	0
#define AW87XXX_PID_5A_REG_RCV_MODE_ENABLE		1
#define AW87XXX_PID_5A_REG_RCV_MODE_ENABLE_VALUE	BIT(2)

#define AW87XXX_PID_5A_REG_BST_IPEAK_START_BIT		0
#define AW87XXX_PID_5A_REG_BST_IPEAK_BITS_LEN		4
#define AW87XXX_PID_5A_REG_BST_IPEAK_MASK		((u32)~GENMASK(3, 0))

#define AW87XXX_PID_76_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_76_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_76_REG_MAX				0x78
#define AW87XXX_PID_76_MDCTRL_REG			0x02

#define AW87XXX_PID_76_EN_SPK_START_BIT			2
#define AW87XXX_PID_76_EN_SPK_BITS_LEN			1
#define AW87XXX_PID_76_EN_SPK_MASK			((u8)~GENMASK(2, 2))
#define AW87XXX_PID_76_EN_SPK_DISABLE			0
#define AW87XXX_PID_76_EN_SPK_DISABLE_VALUE		0
#define AW87XXX_PID_76_EN_SPK_ENABLE			1
#define AW87XXX_PID_76_EN_SPK_ENABLE_VALUE		BIT(2)

#define AW87XXX_PID_76_CPOVP_REG			0x03
#define AW87XXX_PID_76_DFT_ADP1_REG			0x67
#define AW87XXX_PID_76_DFT_ADP1_CHECK			0x04

#define AW87XXX_PID_60_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_60_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_60_REG_MAX				0x7C
#define AW87XXX_PID_60_SYSCTRL_REG			0x01

#define AW87XXX_PID_60_RCV_MODE_START_BIT		1
#define AW87XXX_PID_60_RCV_MODE_BITS_LEN		1
#define AW87XXX_PID_60_RCV_MODE_MASK			((u8)~GENMASK(1, 1))
#define AW87XXX_PID_60_RCV_MODE_DISABLE			0
#define AW87XXX_PID_60_RCV_MODE_DISABLE_VALUE		0
#define AW87XXX_PID_60_RCV_MODE_ENABLE			1
#define AW87XXX_PID_60_RCV_MODE_ENABLE_VALUE		BIT(1)

#define AW87XXX_PID_60_NG3_REG				0x76
#define AW87XXX_PID_60_ESD_REG_VAL			0x91

#define AW87XXX_PID_C1_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_C1_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_C1_REG_MAX				0x7F
#define AW87XXX_PID_C1_SYSCTRL_REG			0x01

#define AW87XXX_PID_C1_EN_SW_START_BIT			0
#define AW87XXX_PID_C1_EN_SW_BITS_LEN			1
#define AW87XXX_PID_C1_EN_SW_MASK			((u8)~GENMASK(0, 0))
#define AW87XXX_PID_C1_EN_SE_DISABLE			0
#define AW87XXX_PID_C1_EN_SE_DISABLE_VALUE		0
#define AW87XXX_PID_C1_EN_SE_ENABLE			1
#define AW87XXX_PID_C1_EN_SE_ENABLE_VALUE		BIT(0)

#define AW87XXX_PID_C1_EN_SPK_START_BIT			3
#define AW87XXX_PID_C1_EN_SPK_BITS_LEN			1
#define AW87XXX_PID_C1_EN_SPK_MASK			((u8)~GENMASK(3, 3))
#define AW87XXX_PID_C1_EN_SPK_SPK_MODE_DISABLE		0
#define AW87XXX_PID_C1_EN_SPK_SPK_MODE_DISABLE_VALUE	0
#define AW87XXX_PID_C1_EN_SPK_SPK_MODE_ENABLE		1
#define AW87XXX_PID_C1_EN_SPK_SPK_MODE_ENABLE_VALUE	BIT(3)

#define AW87XXX_PID_C1_DFT_THGEN1_REG			0x64
#define AW87XXX_PID_C1_DFT_THGEN1_CHECK			0x0A
#define AW87XXX_PID_C1_EFRH2_REG			0x76
#define AW87XXX_PID_C1_EFRL2_REG			0x78

#define AW87XXX_PID_C2_POWER_ON_DELAY_MS		3
#define AW87XXX_PID_C2_POWER_OFF_DELAY_MS		5
#define AW87XXX_PID_C2_REG_MAX				0x7F
#define AW87XXX_PID_C2_CM_VOLT_MARK			3
#define AW87XXX_PID_C2_SYSCTRL_REG			0x01

#define AW87XXX_PID_C2_RCV_MODE_START_BIT		1
#define AW87XXX_PID_C2_RCV_MODE_BITS_LEN		1
#define AW87XXX_PID_C2_RCV_MODE_MASK			((u8)~GENMASK(1, 1))
#define AW87XXX_PID_C2_RCV_MODE_DISABLE			0
#define AW87XXX_PID_C2_RCV_MODE_DISABLE_VALUE		0
#define AW87XXX_PID_C2_RCV_MODE_ENABLE			1
#define AW87XXX_PID_C2_RCV_MODE_ENABLE_VALUE		BIT(1)

#define AW87XXX_PID_C2_EN_SW_START_BIT			5
#define AW87XXX_PID_C2_EN_SW_BITS_LEN			1
#define AW87XXX_PID_C2_EN_SW_MASK			((u8)~GENMASK(5, 5))
#define AW87XXX_PID_C2_EN_SW_DISABLE			0
#define AW87XXX_PID_C2_EN_SW_DISABLE_VALUE		0
#define AW87XXX_PID_C2_EN_SW_ENABLE			1
#define AW87XXX_PID_C2_EN_SW_ENABLE_VALUE		BIT(5)

#define AW87XXX_PID_C2_PEAKLIMIT_REG			0x03
#define AW87XXX_PID_C2_BST_IPEAK_START_BIT		0
#define AW87XXX_PID_C2_BST_IPEAK_BITS_LEN		4
#define AW87XXX_PID_C2_BST_IPEAK_MASK			((u32)~GENMASK(3, 0))

#define AW87XXX_PID_C2_CP_REG				0x21
#define AW87XXX_PID_C2_VERSION_BITS_LEN			2
#define AW87XXX_PID_C2_VERSION_START_BIT		5
#define AW87XXX_PID_C2_VERSION_REG			0x2A
#define AW87XXX_PID_C2_VERSION_MASK			((u8)~GENMASK(6, 5))
#define AW87XXX_PID_C2_EFRHH_REG			0x31
#define AW87XXX_PID_C2_EF_VERSION_ID_START_BIT		5
#define AW87XXX_PID_C2_EF_VERSION_ID_BITS_LEN		2
#define AW87XXX_PID_C2_EF_VERSION_ID_MASK		((u8)~GENMASK(6, 5))
#define AW87XXX_PID_C2_EFRHL_REG			0x32
#define AW87XXX_PID_C2_CP_CHECK				0x77

#define AW87XXX_PID_23_POWER_ON_DELAY_MS		0
#define AW87XXX_PID_23_POWER_OFF_DELAY_MS		0
#define AW87XXX_PID_23_REG_MAX				0x7F
#define AW87XXX_PID_23_SYSCTRL_REG			0x02

#define AW87XXX_PID_23_EN_SPK_START_BIT			3
#define AW87XXX_PID_23_EN_SPK_BITS_LEN			1
#define AW87XXX_PID_23_EN_SPK_MASK			((u8)~GENMASK(3, 3))
#define AW87XXX_PID_23_EN_SPK_DISABLE			0
#define AW87XXX_PID_23_EN_SPK_DISABLE_VALUE		0
#define AW87XXX_PID_23_EN_SPK_ENABLE			1
#define AW87XXX_PID_23_EN_SPK_ENABLE_VALUE		BIT(3)

#define AW87XXX_PID_23_CP_REG				0x03
#define AW87XXX_PID_23_ESD_REG				0x01
#define AW87XXX_PID_23_ESD_CHECK			0x00

/*
 * aw87xxx devices attributes
 */
struct aw_device;
struct aw_data_container;

enum aw_dev_chipid {
	AW_DEV_CHIPID_18 = 0x18,
	AW_DEV_CHIPID_39 = 0x39,
	AW_DEV_CHIPID_59 = 0x59,
	AW_DEV_CHIPID_5A = 0x5A,
	AW_DEV_CHIPID_9A = 0x9A,
	AW_DEV_CHIPID_9B = 0x9B,
	AW_DEV_CHIPID_76 = 0x76,
	AW_DEV_CHIPID_60 = 0x60,
	AW_DEV_CHIPID_C1 = 0xC1,
	AW_DEV_CHIPID_C2 = 0xC2,
	AW_DEV_CHIPID_23 = 0x23
};

enum aw_dev_hw_status {
	AW_DEV_HWEN_OFF = 0,
	AW_DEV_HWEN_ON,
	AW_DEV_HWEN_INVALID,
	AW_DEV_HWEN_STATUS_MAX,
};

enum aw_dev_soft_off_enable {
	AW_DEV_SOFT_OFF_DISENABLE = 0,
	AW_DEV_SOFT_OFF_ENABLE = 1,
};

enum aw_reg_receiver_mode {
	AW_NOT_REC_MODE = 0,
	AW_IS_REC_MODE = 1,
};

enum aw_reg_voltage_status {
	AW_VOLTAGE_LOW = 0,
	AW_VOLTAGE_HIGH,
};

struct aw_product_tab {
	u8 count;
	const char * const *product_tab;
};

struct aw_mark_desc {
	u8 addr;
	u8 start;
	u8 mask;
};

struct aw_dev_property {
	int id;
	const char * const *product_tab;
	int product_cnt;
	int (*dev_init_func)(struct aw_device *aw_dev);
};

struct aw_fld_check_unit {
	u32 reg;
	u32 mask;
	u32 check_val;
};

struct aw_delay_desc {
	u8 power_on_delay_ms;
	u8 power_off_delay_ms;
};

struct aw_enable_desc {
	u8 addr;
	u8 enable;
	u8 disable;
	u8 mask;
};

struct aw_mute_desc {
	u8 addr;
	u8 enable;
	u8 disable;
	u8 mask;
};

struct aw_esd_check_desc {
	u8 first_update_reg_addr;
	u8 first_update_reg_val;
};

struct aw_rec_mode_desc {
	u8 addr;
	u8 enable;
	u8 disable;
	u8 mask;
};

struct aw_voltage_desc {
	u8 addr;
	u8 mask;
	u8 start;
	u8 vol_max;
	u8 vol_min;
};

struct aw_ipeak_desc {
	u32 reg;
	u32 mask;
};

struct aw_ef_desc {
	u32 count;
	struct aw_fld_check_unit sequence[AW_LOCK_SEQUENCE_MAX];
};

struct aw_cm_volt_desc {
	u8 addr;
	u8 mask;
	u8 init;
	u8 adjust;
	u8 threshold;
};

struct aw_device {
	u8 i2c_addr;
	u8 soft_off_enable;
	u8 is_rec_mode;
	u8 power_on_disabled;
	u8 power_off_disabled;
	u8 version;
	int chipid;
	int hwen_status;
	int i2c_bus;
	bool has_rst_gpio;
	int reg_max_addr;
	int product_cnt;
	int rst_list_flag;
	int dev_index;
	const char * const *product_tab;

	struct device *dev;
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct aw_delay_desc delay_desc;
	struct aw_enable_desc en_desc;
	struct aw_mute_desc mute_desc;
	struct aw_esd_check_desc esd_desc;
	struct aw_rec_mode_desc rec_desc;
	struct aw_voltage_desc vol_desc;
	struct aw_ipeak_desc ipeak_desc;
	struct aw_ef_desc ef_desc;
	struct aw_cm_volt_desc cm_volt_desc;
	struct list_head list;

	struct device_node *rst_pinctrl_np;
	int (*pwr_on_func)(struct aw_device *aw_dev,
			   struct aw_data_container *data);
	int (*pwr_off_func)(struct aw_device *aw_dev,
			    struct aw_data_container *data);
	int (*rst_pin_ctrl)(struct aw_device *aw_dev, bool enable);
};

void aw87xxx_dev_add_dev_list(struct aw_device *aw_dev);
void aw87xxx_dev_remove_dev_list(struct aw_device *aw_dev);
int aw87xxx_dev_i2c_write_byte(struct aw_device *aw_dev,
			       u8 reg_addr,
			       u8 reg_data);
int aw87xxx_dev_i2c_read_byte(struct aw_device *aw_dev,
			      u8 reg_addr,
			      u8 *reg_data);
int aw87xxx_dev_i2c_read_msg(struct aw_device *aw_dev,
			     u8 reg_addr,
			     u8 *data_buf,
			     u32 data_len);
int aw87xxx_dev_i2c_write_bits(struct aw_device *aw_dev,
			       u8 reg_addr,
			       u8 mask,
			       u8 reg_data);
void aw87xxx_dev_soft_reset(struct aw_device *aw_dev);
void aw87xxx_dev_hw_pwr_ctrl(struct aw_device *aw_dev, bool enable);
int aw87xxx_dev_default_pwr_on(struct aw_device *aw_dev,
			       struct aw_data_container *profile_data);
int aw87xxx_dev_default_pwr_off(struct aw_device *aw_dev,
				struct aw_data_container *profile_data);
int aw87xxx_dev_soft_off_only(struct aw_device *aw_dev,
			      struct aw_data_container *profile_data);
int aw87xxx_dev_esd_reg_status_check(struct aw_device *aw_dev);
int aw87xxx_dev_check_reg_is_rec_mode(struct aw_device *aw_dev);
int aw87xxx_dev_init(struct aw_device *aw_dev);

#endif
