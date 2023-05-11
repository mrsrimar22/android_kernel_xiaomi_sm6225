// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Richtek Inc.
 *
 * Richtek RT1711H Type-C Port Control Driver
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/cpu.h>
#include <linux/version.h>
#include <linux/sched/clock.h>
#include <asm/unaligned.h>

#include "inc/tcpci.h"
#include "inc/tcpci_rt1711h.h"

#define RT1711H_DRV_VERSION	"2.0.7_G"

#define RT1711H_IRQ_WAKE_TIME	(500) /* ms */

static atomic_t rt1711h_ready = ATOMIC_INIT(0);

bool rt1711h_chip_is_ready(void)
{
	return atomic_read(&rt1711h_ready) == 1;
}
EXPORT_SYMBOL_GPL(rt1711h_chip_is_ready);

static int rt1711h_read_device(struct rt1711h_chip *chip, u8 reg, int len, void *dst)
{
	struct i2c_client *client = chip->client;
	int ret = 0;
	int i;
#if ENABLE_RT1711H_DBG
	u64 t1 = 0, t2 = 0;
#endif

	for (i = 0; i < 5; i++) {
#if ENABLE_RT1711H_DBG
		t1 = local_clock();
#endif
		ret = i2c_smbus_read_i2c_block_data(client, reg, len, dst);
#if ENABLE_RT1711H_DBG
		t2 = local_clock();
		RT1711H_INFO("%s: del = %lluus, reg = 0x%02X, len = %d\n",
			     __func__, (t2 - t1) / NSEC_PER_USEC, reg, len);
#endif
		if (ret >= 0)
			return ret;

		dev_err(chip->dev, "%s: reg 0x%02x len %d retry %d/5 fail(%d)\n",
			__func__, reg, len, i + 1, ret);

		if (i < 4)
			usleep_range(200, 300);
	}

	dev_err(chip->dev, "%s: reg 0x%02x len %d fail(%d)\n", __func__, reg, len, ret);
	return ret;
}

static int rt1711h_write_device(struct rt1711h_chip *chip, u8 reg, int len, const void *src)
{
	struct i2c_client *client = chip->client;
	int ret = 0;
	int i;
#if ENABLE_RT1711H_DBG
	u64 t1 = 0, t2 = 0;
#endif

	for (i = 0; i < 5; i++) {
#if ENABLE_RT1711H_DBG
		t1 = local_clock();
#endif
		ret = i2c_smbus_write_i2c_block_data(client, reg, len, src);
#if ENABLE_RT1711H_DBG
		t2 = local_clock();
		RT1711H_INFO("%s: del = %lluus, reg = %02X, len = %d\n",
			     __func__, (t2 - t1) / NSEC_PER_USEC, reg, len);
#endif
		if (!ret)
			return 0;

		dev_err(chip->dev, "%s: reg 0x%02x len %d retry %d/5 fail(%d)\n",
			__func__, reg, len, i + 1, ret);

		if (i < 4)
			usleep_range(200, 300);
	}

	dev_err(chip->dev, "%s: reg 0x%02x len %d fail(%d)\n", __func__, reg, len, ret);
	return ret;
}

static const struct regmap_config rt1711h_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int rt1711h_i2c_read8(struct tcpc_device *tcpc, u8 reg)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	u32 val = 0;
	int ret = 0, i;

	for (i = 0; i < 5; i++) {
		ret = regmap_read(chip->regmap, reg, &val);
		if (ret >= 0)
			return (u8)val;

		dev_err(chip->dev, "%s: reg 0x%02x retry %d/5 fail(%d)\n",
			__func__, reg, i + 1, ret);

		if (i < 4)
			usleep_range(200, 300);
	}

	dev_err(chip->dev, "%s: reg 0x%02x fail(%d)\n", __func__, reg, ret);
	return ret;
}

static int rt1711h_i2c_write8(struct tcpc_device *tcpc,
			      u8 reg, const u8 data)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int ret = 0, i;

	for (i = 0; i < 5; i++) {
		ret = regmap_write(chip->regmap, reg, data);
		if (!ret)
			return 0;

		dev_err(chip->dev, "%s: reg 0x%02x retry %d/5 fail(%d)\n",
			__func__, reg, i + 1, ret);

		if (i < 4)
			usleep_range(200, 300);
	}

	dev_err(chip->dev, "%s: reg 0x%02x fail(%d)\n", __func__, reg, ret);
	return ret;
}

static int rt1711h_i2c_read16(struct tcpc_device *tcpc, u8 reg)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	u8 buf[2] = {0};
	int ret = 0, i;

	for (i = 0; i < 5; i++) {
		ret = regmap_bulk_read(chip->regmap, reg, buf, 2);
		if (ret >= 0)
			return buf[0] | (buf[1] << 8);

		dev_err(chip->dev, "%s: reg 0x%02x retry %d/5 fail(%d)\n",
			__func__, reg, i + 1, ret);

		if (i < 4)
			usleep_range(200, 300);
	}

	dev_err(chip->dev, "%s: reg 0x%02x fail(%d)\n", __func__, reg, ret);
	return ret;
}

static int rt1711h_i2c_write16(struct tcpc_device *tcpc,
			       u8 reg, const u16 data)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	u8 buf[2];
	int ret = 0, i;

	buf[0] = data & 0xFF;
	buf[1] = (data >> 8) & 0xFF;

	for (i = 0; i < 5; i++) {
		ret = regmap_bulk_write(chip->regmap, reg, buf, 2);
		if (!ret)
			return 0;

		dev_err(chip->dev, "%s: reg 0x%02x retry %d/5 fail(%d)\n",
			__func__, reg, i + 1, ret);

		if (i < 4)
			usleep_range(200, 300);
	}

	dev_err(chip->dev, "%s: reg 0x%02x fail(%d)\n", __func__, reg, ret);
	return ret;
}

static int rt1711h_init_vbus_cal(struct tcpc_device *tcpc)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int ret = 0;
	u8 data = 0;
	s8 cal = 0;

	ret = rt1711h_i2c_write16(tcpc, RT1711H_REG_UNLOCK_PW_2, 0x6286);
	if (ret < 0)
		dev_notice(chip->dev, "%s: en test mode fail(%d)\n", __func__, ret);

	ret = rt1711h_i2c_read8(tcpc, RT1711H_REG_EFUSE5);
	if (ret < 0) {
		dev_notice(chip->dev, "%s: read efuse5 fail(%d)\n", __func__, ret);
		goto out;
	}

	data = ret;
	data = (data & RT1711H_REG_M_VBUS_CAL) >> RT1711H_REG_S_VBUS_CAL;
	cal = (data & BIT(2)) ? (data | GENMASK(7, 3)) : data;
	cal -= 2;
	if (cal < RT1711H_REG_MIN_VBUS_CAL)
		cal = RT1711H_REG_MIN_VBUS_CAL;
	data = ((u8)cal << RT1711H_REG_S_VBUS_CAL) | (ret & GENMASK(4, 0));

	ret = rt1711h_i2c_write8(tcpc, RT1711H_REG_EFUSE5, data);
	if (ret < 0)
		dev_notice(chip->dev, "%s: write efuse5 fail(%d)\n", __func__, ret);

out:
	ret = rt1711h_i2c_write16(tcpc, RT1711H_REG_UNLOCK_PW_2, 0x0000);
	if (ret < 0)
		dev_notice(chip->dev, "%s: dis test mode fail(%d)\n", __func__, ret);

	return ret;
}

static int rt1711h_init_alert_mask(struct tcpc_device *tcpc)
{
	u16 mask;

	mask = TCPC_V10_REG_ALERT_CC_STATUS | TCPC_V10_REG_ALERT_POWER_STATUS;

#ifdef CONFIG_USB_POWER_DELIVERY
	/* Need to handle RX overflow */
	mask |= TCPC_V10_REG_ALERT_TX_SUCCESS |
			TCPC_V10_REG_ALERT_TX_DISCARDED |
			TCPC_V10_REG_ALERT_TX_FAILED |
			TCPC_V10_REG_ALERT_RX_HARD_RST |
			TCPC_V10_REG_ALERT_RX_STATUS |
			TCPC_V10_REG_RX_OVERFLOW;
#endif

	mask |= TCPC_REG_ALERT_FAULT;

	return rt1711h_i2c_write16(tcpc, TCPC_V10_REG_ALERT_MASK, mask);
}

static int rt1711h_init_power_status_mask(struct tcpc_device *tcpc)
{
	const u8 mask = TCPC_V10_REG_POWER_STATUS_VBUS_PRES;

	return rt1711h_i2c_write8(tcpc, TCPC_V10_REG_POWER_STATUS_MASK, mask);
}

static int rt1711h_init_fault_mask(struct tcpc_device *tcpc)
{
	const u8 mask =
			TCPC_V10_REG_FAULT_STATUS_VCONN_OV |
			TCPC_V10_REG_FAULT_STATUS_VCONN_OC;

	return rt1711h_i2c_write8(tcpc, TCPC_V10_REG_FAULT_STATUS_MASK, mask);
}

static int rt1711h_init_rt_mask(struct tcpc_device *tcpc)
{
	u8 rt_mask = 0;
#ifdef CONFIG_TCPC_WATCHDOG_EN
	rt_mask |= RT1711H_REG_M_WATCHDOG;
#endif	/* CONFIG_TCPC_WATCHDOG_EN */
	rt_mask |= RT1711H_REG_M_VBUS_80;

#ifdef CONFIG_TYPEC_CAP_RA_DETACH
	if (tcpc->tcpc_flags & TCPC_FLAGS_CHECK_RA_DETACH)
		rt_mask |= RT1711H_REG_M_RA_DETACH;
#endif	/* CONFIG_TYPEC_CAP_RA_DETACH */

#ifdef CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG
	if (tcpc->tcpc_flags & TCPC_FLAGS_LPM_WAKEUP_WATCHDOG)
		rt_mask |= RT1711H_REG_M_WAKEUP;
#endif	/* CONFIG_TYPEC_CAP_LPM_WAKEUP_WATCHDOG */

	return rt1711h_i2c_write8(tcpc, RT1711H_REG_RT_MASK, rt_mask);
}

static irqreturn_t rt1711h_thread_irq(int irq, void *data)
{
	struct rt1711h_chip *chip = data;

	if (chip->tcpc) {
		pm_wakeup_ws_event(chip->rt1711h_ws, RT1711H_IRQ_WAKE_TIME, true);
		tcpci_lock_typec(chip->tcpc);
		tcpci_alert(chip->tcpc);
		tcpci_unlock_typec(chip->tcpc);
	}

	return IRQ_HANDLED;
}

static int rt1711h_init_alert(struct tcpc_device *tcpc)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int irqn = 0, ret = 0;
	char *name = NULL;

	/* Clear Alert Mask & Status */
	rt1711h_i2c_write16(tcpc, TCPC_V10_REG_ALERT_MASK, 0);
	rt1711h_i2c_write16(tcpc, TCPC_V10_REG_ALERT, 0xffff);

	name = devm_kasprintf(chip->dev,
			      GFP_KERNEL, "%s-IRQ",
			      chip->tcpc_desc->name);
	if (!name)
		return -ENOMEM;

	dev_info(chip->dev, "%s: name: %s\n", __func__,
		 chip->tcpc_desc->name);

	irqn = gpiod_to_irq(chip->irq_gpiod);
	if (irqn <= 0) {
		dev_err(chip->dev, "%s: gpiod_to_irq fail(%d)\n", __func__, irqn);
		return irqn;
	}

	dev_info(chip->dev, "%s: IRQ number: %d\n", __func__, irqn);

	ret = devm_request_threaded_irq(chip->dev, irqn,
					NULL, rt1711h_thread_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					name, chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: request irq fail(%d)\n", __func__, ret);
		return ret;
	}
	chip->irq = irqn;

	return 0;
}

int rt1711h_alert_status_clear(struct tcpc_device *tcpc, u32 mask)
{
	int ret;
	u16 mask_t1;
	u8 mask_t2;

	mask_t1 = (u16)mask;
	if (mask_t1) {
		ret = rt1711h_i2c_write16(tcpc, TCPC_V10_REG_ALERT, mask_t1);
		if (ret < 0)
			return ret;
	}

	mask_t2 = (u8)mask >> 16;
	if (mask_t2) {
		ret = rt1711h_i2c_write8(tcpc, RT1711H_REG_RT_INT, mask_t2);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int rt1711h_set_clock_gating(struct tcpc_device *tcpc, bool en)
{
	int ret = 0;

#ifdef CONFIG_TCPC_CLOCK_GATING
	int i = 0;
	u8 clk2 = RT1711H_REG_CLK_DIV_600K_EN |
			RT1711H_REG_CLK_DIV_300K_EN |
			RT1711H_REG_CLK_CK_300K_EN;
	u8 clk3 = RT1711H_REG_CLK_DIV_2P4M_EN;

	if (!en) {
		clk2 |= RT1711H_REG_CLK_BCLK2_EN | RT1711H_REG_CLK_BCLK_EN;
		clk3 |= RT1711H_REG_CLK_CK_24M_EN | RT1711H_REG_CLK_PCLK_EN;
	}

	if (en) {
		for (i = 0; i < 2; i++)
			ret = rt1711h_alert_status_clear(tcpc, TCPC_REG_ALERT_RX_ALL_MASK);
	}

	if (ret == 0)
		ret = rt1711h_i2c_write8(tcpc, RT1711H_REG_CLK_CTRL2, clk2);
	if (ret == 0)
		ret = rt1711h_i2c_write8(tcpc, RT1711H_REG_CLK_CTRL3, clk3);
#endif	/* CONFIG_TCPC_CLOCK_GATING */

	return ret;
}

static int rt1711h_init_cc_params(struct tcpc_device *tcpc, u8 cc_res)
{
	int rv = 0;

#ifdef CONFIG_USB_POWER_DELIVERY
#ifdef CONFIG_USB_PD_SNK_DFT_NO_GOOD_CRC
	u8 en, sel;
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);

	if (cc_res == TYPEC_CC_VOLT_SNK_DFT) {	/* 0.55 */
		en = 0;
		sel = 0x81;
	} else if (chip->did >= RT1715_DID_D) {	/* 0.35 & 0.75 */
		en = 1;
		sel = 0x81;
	} else {	/* 0.4 & 0.7 */
		en = 1;
		sel = 0x80;
	}

	rv = rt1711h_i2c_write8(tcpc, RT1711H_REG_BMCIO_RXDZEN, en);
	if (rv == 0)
		rv = rt1711h_i2c_write8(tcpc, RT1711H_REG_BMCIO_RXDZSEL, sel);
#endif	/* CONFIG_USB_PD_SNK_DFT_NO_GOOD_CRC */
#endif	/* CONFIG_USB_POWER_DELIVERY */

	return rv;
}

static int rt1711h_tcpc_init(struct tcpc_device *tcpc, bool sw_reset)
{
	int ret;
	bool retry_discard_old = false;
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);

	if (sw_reset) {
		ret = rt1711h_i2c_write8(tcpc, RT1711H_REG_SWRESET, 1);
		if (ret < 0)
			return ret;
		usleep_range(1000, 2000);
	}

#ifdef CONFIG_TCPC_I2CRST_EN
	if (chip->did != HUSB311_DID)
		rt1711h_i2c_write8(tcpc, RT1711H_REG_I2CRST_CTRL,
				   RT1711H_REG_I2CRST_SET(true, 0x0f));
#endif	/* CONFIG_TCPC_I2CRST_EN */

	/* UFP Both RD setting */
	/* DRP = 0, RpVal = 0 (Default), Rd, Rd */
	rt1711h_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL,
			   TCPC_V10_REG_ROLE_CTRL_RES_SET(0, 0, CC_RD, CC_RD));

	if (chip->did == RT1711H_DID_A)
		rt1711h_i2c_write8(tcpc, TCPC_V10_REG_FAULT_CTRL,
				   TCPC_V10_REG_FAULT_CTRL_DIS_VCONN_OV);

	/*
	 * CC Detect Debounce : 26.7*val us
	 * Transition window count : spec 12~20us, based on 2.4MHz
	 * DRP Toggle Cycle : 51.2 + 6.4*val ms
	 * DRP Duty Ctrl : dcSRC / 1024
	 */

	rt1711h_i2c_write8(tcpc, RT1711H_REG_TTCPC_FILTER, 10);
	rt1711h_i2c_write8(tcpc, RT1711H_REG_DRP_TOGGLE_CYCLE, 4);
	rt1711h_i2c_write16(tcpc, RT1711H_REG_DRP_DUTY_CTRL,
			    TCPC_NORMAL_RP_DUTY);

	rt1711h_i2c_write8(tcpc, RT1711H_REG_VCONN_CLIMITEN, 1);

	/* RX/TX Clock Gating (Auto Mode) */
	if (!sw_reset)
		rt1711h_set_clock_gating(tcpc, true);

	if (!(tcpc->tcpc_flags & TCPC_FLAGS_RETRY_CRC_DISCARD))
		retry_discard_old = true;

	rt1711h_i2c_write8(tcpc, RT1711H_REG_CONFIG_GPIO0, 0x80);

	/* For BIST, Change Transition Toggle Counter (Noise) from 3 to 7 */
	rt1711h_i2c_write8(tcpc, RT1711H_REG_PHY_CTRL1,
			   RT1711H_REG_PHY_CTRL1_SET(retry_discard_old, 7, 0, 1));

	tcpci_alert_status_clear(tcpc, 0xffffffff);

	rt1711h_init_vbus_cal(tcpc);
	rt1711h_init_power_status_mask(tcpc);
	rt1711h_init_alert_mask(tcpc);
	rt1711h_init_fault_mask(tcpc);
	rt1711h_init_rt_mask(tcpc);

	/* CK_300K from 320K, SHIPPING off, AUTOIDLE enable, TIMEOUT = 6.4ms */
	rt1711h_i2c_write8(tcpc, RT1711H_REG_IDLE_CTRL,
			   RT1711H_REG_IDLE_SET(0, 1, 1, 0));
	mdelay(1);

	return 0;
}

static int rt1711h_fault_status_vconn_ov(struct tcpc_device *tcpc)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);

	return regmap_update_bits(chip->regmap, RT1711H_REG_BMC_CTRL,
				  RT1711H_REG_DISCHARGE_EN, 0);
}

int rt1711h_fault_status_clear(struct tcpc_device *tcpc, u8 status)
{
	int ret = 0;

	if (status & TCPC_V10_REG_FAULT_STATUS_VCONN_OV) {
		ret = rt1711h_fault_status_vconn_ov(tcpc);
		if (ret < 0)
			return ret;
	}

	return rt1711h_i2c_write8(tcpc, TCPC_V10_REG_FAULT_STATUS, status);
}

int rt1711h_get_alert_mask(struct tcpc_device *tcpc, u32 *mask)
{
	int ret;
	u8 v2;

	ret = rt1711h_i2c_read16(tcpc, TCPC_V10_REG_ALERT_MASK);
	if (ret < 0)
		return ret;

	*mask = (u16)ret;

	ret = rt1711h_i2c_read8(tcpc, RT1711H_REG_RT_MASK);
	if (ret < 0)
		return ret;

	v2 = (u8)ret;
	*mask |= v2 << 16;

	return 0;
}

int rt1711h_get_alert_status(struct tcpc_device *tcpc, u32 *alert)
{
	int ret;
	u8 v2;

	ret = rt1711h_i2c_read16(tcpc, TCPC_V10_REG_ALERT);
	if (ret < 0)
		return ret;

	*alert = (u16)ret;

	ret = rt1711h_i2c_read8(tcpc, RT1711H_REG_RT_INT);
	if (ret < 0)
		return ret;

	v2 = (u8)ret;
	*alert |= v2 << 16;

	return 0;
}

static int rt1711h_get_power_status(struct tcpc_device *tcpc, u16 *pwr_status)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int pwr_stat, rt_stat, pwr_stat_recheck;

	pwr_stat = rt1711h_i2c_read8(tcpc, TCPC_V10_REG_POWER_STATUS);
	if (pwr_stat < 0)
		return pwr_stat;

	rt_stat = rt1711h_i2c_read8(tcpc, RT1711H_REG_RT_STATUS);
	if (rt_stat < 0)
		return rt_stat;

	pwr_stat_recheck = rt1711h_i2c_read8(tcpc, TCPC_V10_REG_POWER_STATUS);
	if (pwr_stat_recheck >= 0) {
		if ((pwr_stat ^ pwr_stat_recheck) & TCPC_V10_REG_POWER_STATUS_VBUS_PRES)
			dev_notice(chip->dev,
				   "%s: VBUS_PRES changed mid-read (0x%02x -> 0x%02x)\n",
				   __func__, pwr_stat, pwr_stat_recheck);
		pwr_stat = pwr_stat_recheck;
	}

	*pwr_status = 0;

	if (pwr_stat & TCPC_V10_REG_POWER_STATUS_VBUS_PRES)
		*pwr_status |= TCPC_REG_POWER_STATUS_VBUS_PRES;

	if (rt_stat & RT1711H_REG_VBUS_80)
		*pwr_status |= TCPC_REG_POWER_STATUS_EXT_VSAFE0V;

	return 0;
}

int rt1711h_get_fault_status(struct tcpc_device *tcpc, u8 *status)
{
	int ret;

	ret = rt1711h_i2c_read8(tcpc, TCPC_V10_REG_FAULT_STATUS);
	if (ret < 0)
		return ret;

	*status = (u8)ret;
	return 0;
}

static int rt1711h_get_cc(struct tcpc_device *tcpc, int *cc1, int *cc2)
{
	int status, role_ctrl, cc_role;
	bool act_as_sink, act_as_drp;

	status = rt1711h_i2c_read8(tcpc, TCPC_V10_REG_CC_STATUS);
	if (status < 0)
		return status;

	role_ctrl = rt1711h_i2c_read8(tcpc, TCPC_V10_REG_ROLE_CTRL);
	if (role_ctrl < 0)
		return role_ctrl;

	if (status & TCPC_V10_REG_CC_STATUS_DRP_TOGGLING) {
		*cc1 = TYPEC_CC_DRP_TOGGLING;
		*cc2 = TYPEC_CC_DRP_TOGGLING;
		return 0;
	}

	*cc1 = TCPC_V10_REG_CC_STATUS_CC1(status);
	*cc2 = TCPC_V10_REG_CC_STATUS_CC2(status);

	act_as_drp = !!(role_ctrl & TCPC_V10_REG_ROLE_CTRL_DRP);
	if (act_as_drp) {
		act_as_sink = TCPC_V10_REG_CC_STATUS_DRP_RESULT(status);
	} else {
		/* ROLE_CTRL shares CC1/CC2 bit layout with CC_STATUS */
		if (tcpc->typec_polarity)
			cc_role = TCPC_V10_REG_CC_STATUS_CC2(role_ctrl);
		else
			cc_role = TCPC_V10_REG_CC_STATUS_CC1(role_ctrl);

		if (cc_role == TYPEC_CC_RP)
			act_as_sink = false;
		else
			act_as_sink = true;
	}

	/*
	 * If status is not open, then OR in termination to convert to
	 * enum tcpc_cc_voltage_status.
	 */
	if (*cc1 != TYPEC_CC_VOLT_OPEN)
		*cc1 |= (act_as_sink << 2);

	if (*cc2 != TYPEC_CC_VOLT_OPEN)
		*cc2 |= (act_as_sink << 2);

	rt1711h_init_cc_params(tcpc, (u8)tcpc->typec_polarity ? *cc2 : *cc1);

	return 0;
}

static int rt1711h_enable_vsafe0v_detect(struct tcpc_device *tcpc, bool enable)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);

	return regmap_update_bits(chip->regmap, RT1711H_REG_RT_MASK,
				  RT1711H_REG_M_VBUS_80,
				  enable ? RT1711H_REG_M_VBUS_80 : 0);
}

static int rt1711h_set_cc(struct tcpc_device *tcpc, int pull)
{
	int ret;
	u8 data;
	int rp_lvl = TYPEC_CC_PULL_GET_RP_LVL(pull), pull1, pull2;
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);

	pull = TYPEC_CC_PULL_GET_RES(pull);
	dev_dbg(chip->dev, "%s: pull = 0x%02X\n", __func__, pull);
	if (pull == TYPEC_CC_DRP) {
		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(1, rp_lvl, TYPEC_CC_RD, TYPEC_CC_RD);
		if (chip->did == HUSB311_DID) {
			data &= 0xCF;
			data |= 0x20;
		}

		ret = rt1711h_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);
		if (ret == 0) {
			rt1711h_enable_vsafe0v_detect(tcpc, false);
			ret = rt1711h_i2c_write8(tcpc, TCPC_V10_REG_COMMAND,
						 TCPM_CMD_LOOK_CONNECTION);
		}
	} else {
#ifdef CONFIG_USB_POWER_DELIVERY
		if (pull == TYPEC_CC_RD && tcpc->pd_wait_pr_swap_complete)
			rt1711h_init_cc_params(tcpc, TYPEC_CC_VOLT_SNK_DFT);
#endif	/* CONFIG_USB_POWER_DELIVERY */

		pull1 = pull;
		pull2 = pull;

		if (pull == TYPEC_CC_RP && tcpc->typec_is_attached_src) {
			if (tcpc->typec_polarity)
				pull1 = TYPEC_CC_OPEN;
			else
				pull2 = TYPEC_CC_OPEN;
		}

		data = TCPC_V10_REG_ROLE_CTRL_RES_SET(0, rp_lvl, pull1, pull2);
		if (chip->did == HUSB311_DID) {
			data &= 0xCF;
			data |= 0x10;
		}
		ret = rt1711h_i2c_write8(tcpc, TCPC_V10_REG_ROLE_CTRL, data);
	}

	return ret;
}

static int rt1711h_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int data;

	data = rt1711h_init_cc_params(tcpc, tcpc->typec_remote_cc[polarity]);
	if (data)
		return data;

	return regmap_update_bits(chip->regmap, TCPC_V10_REG_TCPC_CTRL,
				  TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT,
				  polarity ? TCPC_V10_REG_TCPC_CTRL_PLUG_ORIENT : 0);
}

static int rt1711h_set_low_rp_duty(struct tcpc_device *tcpc, bool low_rp)
{
	u16 duty = low_rp ? TCPC_LOW_RP_DUTY : TCPC_NORMAL_RP_DUTY;

	return rt1711h_i2c_write16(tcpc, RT1711H_REG_DRP_DUTY_CTRL, duty);
}

static int rt1711h_set_vconn(struct tcpc_device *tcpc, int enable)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int rv;

	rv = regmap_update_bits(chip->regmap, TCPC_V10_REG_POWER_CTRL,
				TCPC_V10_REG_POWER_CTRL_VCONN,
				enable ? TCPC_V10_REG_POWER_CTRL_VCONN : 0);
	if (rv < 0)
		return rv;

	return rt1711h_i2c_write8(tcpc, RT1711H_REG_IDLE_CTRL,
			RT1711H_REG_IDLE_SET(0, 1, enable ? 0 : 1, 0));
}

#ifdef CONFIG_TCPC_LOW_POWER_MODE
static int rt1711h_is_low_power_mode(struct tcpc_device *tcpc)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int rv;

	rv = rt1711h_i2c_read8(tcpc, RT1711H_REG_BMC_CTRL);
	if (rv < 0)
		return rv;

	if (chip->did == HUSB311_DID) {
		dev_info(chip->dev, "%s: read HUSB311_REG_BMC_CTRL=0x%x\n",
			 __func__, rv);
		return ((rv & RT1711H_REG_BMCIO_OSC_EN) == 0);
	}

	return (rv & RT1711H_REG_BMCIO_LPEN) != 0;
}

static int rt1711h_set_low_power_mode(struct tcpc_device *tcpc, bool en, int pull)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int ret = 0;
	u8 data;

	ret = rt1711h_i2c_write8(tcpc, RT1711H_REG_IDLE_CTRL,
				 RT1711H_REG_IDLE_SET(0, 1, en ? 0 : 1, 0));
	if (ret < 0)
		return ret;

	rt1711h_enable_vsafe0v_detect(tcpc, !en);

	if (en) {
		data = RT1711H_REG_BMCIO_LPEN;

		if (pull & TYPEC_CC_RP)
			data |= RT1711H_REG_BMCIO_LPRPRD;

#ifdef CONFIG_TYPEC_CAP_NORP_SRC
		data |= RT1711H_REG_BMCIO_BG_EN | RT1711H_REG_VBUS_DET_EN;
#endif
		if (chip->did == HUSB311_DID) {
			data &= ~RT1711H_REG_BMCIO_OSC_EN;
			dev_info(chip->dev, "%s: write HUSB311_REG_BMC_CTRL=0x%x\n",
				 __func__, data);
		}

	} else {
		data = RT1711H_REG_BMCIO_BG_EN |
			RT1711H_REG_VBUS_DET_EN | RT1711H_REG_BMCIO_OSC_EN;

		if (chip->did == HUSB311_DID) {
			data |= RT1711H_REG_BMCIO_OSC_EN;
			dev_info(chip->dev, "%s: write HUSB311_REG_BMC_CTRL=0x%x\n",
				 __func__, data);
		}
	}

	return rt1711h_i2c_write8(tcpc, RT1711H_REG_BMC_CTRL, data);
}
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

#ifdef CONFIG_TCPC_WATCHDOG_EN
int rt1711h_set_watchdog(struct tcpc_device *tcpc, bool en)
{
	u8 data = RT1711H_REG_WATCHDOG_CTRL_SET(en, 7);

	return rt1711h_i2c_write8(tcpc, RT1711H_REG_WATCHDOG_CTRL, data);
}
#endif	/* CONFIG_TCPC_WATCHDOG_EN */

#ifdef CONFIG_TCPC_INTRST_EN
int rt1711h_set_intrst(struct tcpc_device *tcpc, bool en)
{
	return rt1711h_i2c_write8(tcpc, RT1711H_REG_INTRST_CTRL,
				  RT1711H_REG_INTRST_SET(en, 3));
}
#endif	/* CONFIG_TCPC_INTRST_EN */

static int rt1711h_tcpc_deinit(struct tcpc_device *tcpc)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);

#ifdef CONFIG_TCPC_SHUTDOWN_CC_DETACH
	rt1711h_set_cc(tcpc, TYPEC_CC_DRP);
	rt1711h_set_cc(tcpc, TYPEC_CC_OPEN);

	if (chip->did == HUSB311_DID) {
		tcpci_alert_status_clear(tcpc, 0xffffff);//0x10 0x11 0x98

		rt1711h_i2c_write16(tcpc, TCPC_V10_REG_ALERT_MASK, 0x0); //0x12 0x13
		rt1711h_i2c_write8(tcpc, TCPC_V10_REG_POWER_STATUS_MASK, 0x0); //0x14
		rt1711h_i2c_write8(tcpc, RT1711H_REG_RT_MASK, 0x0); //0x99
		rt1711h_i2c_write8(tcpc, RT1711H_REG_BMC_CTRL, 0x0); //0x90
		dev_info(chip->dev, "%s: HUSB311 deinit\n", __func__);
	} else {
		rt1711h_i2c_write8(tcpc, RT1711H_REG_I2CRST_CTRL,
				   RT1711H_REG_I2CRST_SET(true, 4));
	}

	rt1711h_i2c_write8(tcpc, RT1711H_REG_INTRST_CTRL,
			   RT1711H_REG_INTRST_SET(true, 0));
#else
	rt1711h_i2c_write8(tcpc, RT1711H_REG_SWRESET, 1);
#endif	/* CONFIG_TCPC_SHUTDOWN_CC_DETACH */

	return 0;
}

#ifdef CONFIG_USB_POWER_DELIVERY
static int rt1711h_set_msg_header(struct tcpc_device *tcpc,
				  u8 power_role, u8 data_role)
{
	u8 msg_hdr = TCPC_V10_REG_MSG_HDR_INFO_SET(data_role, power_role);

	return rt1711h_i2c_write8(tcpc, TCPC_V10_REG_MSG_HDR_INFO, msg_hdr);
}

static int rt1711h_protocol_reset(struct tcpc_device *tcpc)
{
	rt1711h_i2c_write8(tcpc, RT1711H_REG_PRL_FSM_RESET, 0);
	mdelay(1);
	rt1711h_i2c_write8(tcpc, RT1711H_REG_PRL_FSM_RESET, 1);
	return 0;
}

static int rt1711h_set_rx_enable(struct tcpc_device *tcpc, u8 enable)
{
	int ret = 0;

	if (enable)
		ret = rt1711h_set_clock_gating(tcpc, false);

	if (ret == 0)
		ret = rt1711h_i2c_write8(tcpc, TCPC_V10_REG_RX_DETECT, enable);

	if (ret == 0 && !enable) {
		rt1711h_protocol_reset(tcpc);
		ret = rt1711h_set_clock_gating(tcpc, true);
	}

	return ret;
}

static int rt1711h_get_message(struct tcpc_device *tcpc, u32 *payload,
			       u16 *msg_head, enum tcpm_transmit_type *frame_type)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int rv = 0;
	u8 cnt = 0, buf[4];

	rv = regmap_bulk_read(chip->regmap, TCPC_V10_REG_RX_BYTE_CNT, buf, 4);
	if (rv < 0)
		return rv;

	cnt = buf[0];
	*frame_type = buf[1];
	*msg_head = get_unaligned_le16(&buf[2]);

	if (*msg_head == 0x0) {
		tcpci_init(tcpc, true);
		dev_err(chip->dev, "%s: msg_head = 0x%x\n", __func__, *msg_head);
		return -EINVAL;
	}

	/* TCPC 1.0 ==> no need to subtract the size of msg_head */
	if (cnt > 3) {
		cnt -= 3; /* MSG_HDR */
		if (cnt > 28)
			cnt = 28;
		rv = regmap_bulk_read(chip->regmap, TCPC_V10_REG_RX_DATA, payload, cnt);
	}

	return rv;
}

static int rt1711h_set_bist_carrier_mode(struct tcpc_device *tcpc, u8 pattern)
{
	/* Don't support this function */
	return 0;
}

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
static int rt1711h_retransmit(struct tcpc_device *tcpc)
{
	return rt1711h_i2c_write8(tcpc, TCPC_V10_REG_TRANSMIT,
			TCPC_V10_REG_TRANSMIT_SET(tcpc->pd_retry_count, TCPC_TX_SOP));
}
#endif

#pragma pack(push, 1)
struct tcpc_transmit_packet {
	u8 cnt;
	u16 msg_header;
	u8 data[sizeof(u32) * 7];
};

#pragma pack(pop)

static int rt1711h_transmit(struct tcpc_device *tcpc,
			    enum tcpm_transmit_type type,
			    u16 header, const u32 *data)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);
	int rv;
	int data_cnt;
	struct tcpc_transmit_packet packet;

	if (type < TCPC_TX_HARD_RESET) {
		data_cnt = sizeof(u32) * PD_HEADER_CNT(header);

		packet.cnt = data_cnt + sizeof(u16);
		packet.msg_header = header;

		if (data_cnt > 0)
			memcpy(packet.data, (u8 *)data, data_cnt);

		rv = regmap_bulk_write(chip->regmap, TCPC_V10_REG_TX_BYTE_CNT,
				       (u8 *)&packet, packet.cnt + 1);
		if (rv < 0)
			return rv;
	}

	rv = rt1711h_i2c_write8(tcpc, TCPC_V10_REG_TRANSMIT,
				TCPC_V10_REG_TRANSMIT_SET(tcpc->pd_retry_count, type));
	return rv;
}

static int rt1711h_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	struct rt1711h_chip *chip = tcpc_get_dev_data(tcpc);

	return regmap_update_bits(chip->regmap, TCPC_V10_REG_TCPC_CTRL,
				  TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE,
				  en ? TCPC_V10_REG_TCPC_CTRL_BIST_TEST_MODE : 0);
}
#endif	/* CONFIG_USB_POWER_DELIVERY */

static struct tcpc_ops rt1711h_tcpc_ops = {
	.init = rt1711h_tcpc_init,
	.alert_status_clear = rt1711h_alert_status_clear,
	.fault_status_clear = rt1711h_fault_status_clear,
	.get_alert_mask = rt1711h_get_alert_mask,
	.get_alert_status = rt1711h_get_alert_status,
	.get_power_status = rt1711h_get_power_status,
	.get_fault_status = rt1711h_get_fault_status,
	.get_cc = rt1711h_get_cc,
	.set_cc = rt1711h_set_cc,
	.set_polarity = rt1711h_set_polarity,
	.set_low_rp_duty = rt1711h_set_low_rp_duty,
	.set_vconn = rt1711h_set_vconn,
	.deinit = rt1711h_tcpc_deinit,

#ifdef CONFIG_TCPC_LOW_POWER_MODE
	.is_low_power_mode = rt1711h_is_low_power_mode,
	.set_low_power_mode = rt1711h_set_low_power_mode,
#endif	/* CONFIG_TCPC_LOW_POWER_MODE */

#ifdef CONFIG_TCPC_WATCHDOG_EN
	.set_watchdog = rt1711h_set_watchdog,
#endif	/* CONFIG_TCPC_WATCHDOG_EN */

#ifdef CONFIG_TCPC_INTRST_EN
	.set_intrst = rt1711h_set_intrst,
#endif	/* CONFIG_TCPC_INTRST_EN */

#ifdef CONFIG_USB_POWER_DELIVERY
	.set_msg_header = rt1711h_set_msg_header,
	.set_rx_enable = rt1711h_set_rx_enable,
	.protocol_reset = rt1711h_protocol_reset,
	.get_message = rt1711h_get_message,
	.transmit = rt1711h_transmit,
	.set_bist_test_mode = rt1711h_set_bist_test_mode,
	.set_bist_carrier_mode = rt1711h_set_bist_carrier_mode,
#endif	/* CONFIG_USB_POWER_DELIVERY */

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	.retransmit = rt1711h_retransmit,
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */
};

static int rt_parse_dt(struct rt1711h_chip *chip)
{
	struct device *dev = chip->dev;

	chip->irq_gpiod = devm_gpiod_get(dev, "rt1711hpd,intr", GPIOD_IN);
	if (IS_ERR(chip->irq_gpiod)) {
		dev_err(dev, "%s: failed to get IRQ GPIO: %ld\n",
			__func__, PTR_ERR(chip->irq_gpiod));
		return PTR_ERR(chip->irq_gpiod);
	}

	return 0;
}

/*
 * In some platform pr_info may spend too much time on printing debug message.
 * So we use this function to test the printk performance.
 * If your platform cannot not pass this check function, please config
 * PD_DBG_INFO, this will provide the threaded debug message for you.
 */
#if TCPC_ENABLE_ANYMSG
static void check_printk_performance(void)
{
	int i;
	u64 t1, t2;
	u32 nsrem;

#ifdef CONFIG_PD_DBG_INFO
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pd_dbg_info("%d\n", i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pd_dbg_info("pd_dbg_info: t2-t1 = %lu\n",
			    (unsigned long)nsrem / 1000);
	}
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pr_info("%s: %d\n", __func__, i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pr_info("%s: pr_info: t2-t1 = %lu\n", __func__, (unsigned long)nsrem / 1000);
	}
#else
	for (i = 0; i < 10; i++) {
		t1 = local_clock();
		pr_info("%s: %d\n", __func__, i);
		t2 = local_clock();
		t2 -= t1;
		nsrem = do_div(t2, 1000000000);
		pr_info("%s: t2-t1 = %lu\n", __func__, (unsigned long)nsrem / 1000);
		PD_WARN_ON(nsrem > 100 * 1000);
	}
#endif	/* CONFIG_PD_DBG_INFO */
}
#endif	/* TCPC_ENABLE_ANYMSG */

static int rt1711h_tcpc_dev_init(struct rt1711h_chip *chip)
{
	struct tcpc_desc *desc;
	struct device *dev = chip->dev;
	struct device_node *np = dev->of_node;
	u32 val;
	char *name = "default";

	desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	if (of_property_read_u32(np, "rt-tcpc,role_def", &val) >= 0) {
		if (val >= TYPEC_ROLE_NR)
			desc->role_def = TYPEC_ROLE_DRP;
		else
			desc->role_def = val;
	} else {
		dev_err(dev, "%s: use default Role DRP\n", __func__);
		desc->role_def = TYPEC_ROLE_DRP;
	}

	if (of_property_read_u32(np, "rt-tcpc,rp_level", &val) >= 0) {
		switch (val) {
		case TYPEC_RP_DFT:
		case TYPEC_RP_1_5:
		case TYPEC_RP_3_0:
			desc->rp_lvl = val;
			if (chip->did != RT1715_DID_D) // not rt1711, use 2---->3.0
				desc->rp_lvl = 2;
			break;
		default:
			break;
		}
	}

#ifdef CONFIG_TCPC_VCONN_SUPPLY_MODE
	if (of_property_read_u32(np, "rt-tcpc,vconn_supply", &val) >= 0) {
		if (val >= TCPC_VCONN_SUPPLY_NR)
			desc->vconn_supply = TCPC_VCONN_SUPPLY_ALWAYS;
		else
			desc->vconn_supply = val;
	} else {
		dev_err(dev, "%s: use default VconnSupply\n", __func__);
		desc->vconn_supply = TCPC_VCONN_SUPPLY_ALWAYS;
	}
#endif	/* CONFIG_TCPC_VCONN_SUPPLY_MODE */

	if (of_property_read_string(np, "rt-tcpc,name", (char const **)&name) < 0)
		dev_err(dev, "%s: use default name\n", __func__);

	desc->name = kstrdup(name, GFP_KERNEL);
	if (!desc->name) {
		kfree(desc);
		return -ENOMEM;
	}

	desc->pid = chip->pid;
	chip->tcpc_desc = desc;

	chip->tcpc = tcpc_device_register(dev, desc, &rt1711h_tcpc_ops, chip);
	if (IS_ERR_OR_NULL(chip->tcpc)) {
		kfree(desc->name);
		kfree(desc);
		chip->tcpc_desc = NULL;
		return -EINVAL;
	}

#ifdef CONFIG_USB_PD_DISABLE_PE
	chip->tcpc->disable_pe = of_property_read_bool(np, "rt-tcpc,disable_pe");
#endif	/* CONFIG_USB_PD_DISABLE_PE */

	chip->tcpc->tcpc_flags =
			TCPC_FLAGS_LPM_WAKEUP_WATCHDOG |
			TCPC_FLAGS_VCONN_SAFE5V_ONLY;

	if (chip->did > RT1711H_DID_B)
		chip->tcpc->tcpc_flags |= TCPC_FLAGS_CHECK_RA_DETACH;

#ifdef CONFIG_USB_PD_RETRY_CRC_DISCARD
	if (chip->did > RT1715_DID_D)
		chip->tcpc->tcpc_flags |= TCPC_FLAGS_RETRY_CRC_DISCARD;
#endif	/* CONFIG_USB_PD_RETRY_CRC_DISCARD */

#ifdef CONFIG_USB_PD_REV30
	if (chip->did >= RT1715_DID_D || chip->did  == HUSB311_DID)
		chip->tcpc->tcpc_flags |= TCPC_FLAGS_PD_REV30;

	if (chip->tcpc->tcpc_flags & TCPC_FLAGS_PD_REV30)
		dev_info(dev, "%s: PD_REV30\n", __func__);
	else
		dev_info(dev, "%s: PD_REV20\n", __func__);
#endif	/* CONFIG_USB_PD_REV30 */

	chip->tcpc->tcpc_flags |= TCPC_FLAGS_ALERT_V10;

	return 0;
}

static void rt1711h_tcpc_dev_deinit(struct rt1711h_chip *chip)
{
	if (chip->tcpc)
		tcpc_device_unregister(chip->dev, chip->tcpc);

	kfree(chip->tcpc_desc->name);
	kfree(chip->tcpc_desc);
}

#define RICHTEK_1711_VID	0x29cf
#define RICHTEK_1711_PID	0x1711
#define HUSB_311_VID	0x2e99
#define HUSB_311_PID	0x0311

static int rt1711h_check_revision(struct rt1711h_chip *chip)
{
	u16 vid, pid, did;
	int ret;
	u8 data = 1;

	ret = rt1711h_read_device(chip, TCPC_V10_REG_VID, 2, &vid);
	if (ret < 0) {
		dev_err(chip->dev, "%s: read chip ID fail(%d)\n", __func__, ret);
		return -EIO;
	}

	if (vid != RICHTEK_1711_VID && vid != HUSB_311_VID) {
		dev_err(chip->dev, "%s: VID check failed, VID=0x%04x\n", __func__, vid);
		return -ENODEV;
	}

	ret = rt1711h_read_device(chip, TCPC_V10_REG_PID, 2, &pid);
	if (ret < 0) {
		dev_err(chip->dev, "%s: read product ID fail(%d)\n", __func__, ret);
		return -EIO;
	}

	if (pid != RICHTEK_1711_PID && pid != HUSB_311_PID) {
		dev_err(chip->dev, "%s: PID check failed, PID=0x%04x\n", __func__, pid);
		return -ENODEV;
	}

	ret = rt1711h_write_device(chip, RT1711H_REG_SWRESET, 1, &data);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	ret = rt1711h_read_device(chip, TCPC_V10_REG_DID, 2, &did);
	if (ret < 0) {
		dev_err(chip->dev, "%s: read device ID fail(%d)\n", __func__, ret);
		return -EIO;
	}

	chip->vid = vid;
	chip->pid = pid;
	chip->did = did;

	return 0;
}

static int rt1711h_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct rt1711h_chip *chip;
	int ret = 0;
	bool use_dt = client->dev.of_node;

	dev_info(&client->dev, "%s: (%s)\n", __func__, RT1711H_DRV_VERSION);
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_I2C_BLOCK |
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: I2C not supported\n", __func__);
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);

	if (!use_dt) {
		dev_err(chip->dev, "%s: no dts node\n", __func__);
		ret = -ENODEV;
		goto err_free;
	}

	ret = rt_parse_dt(chip);
	if (ret < 0)
		goto err_free;

	ret = rt1711h_check_revision(chip);
	if (ret < 0)
		goto err_free;

#if TCPC_ENABLE_ANYMSG
	check_printk_performance();
#endif	/* TCPC_ENABLE_ANYMSG */

	dev_info(chip->dev, "%s: %s_chipID: 0x%04x\n",
		 __func__, chip->client->name, chip->did);

	chip->regmap = devm_regmap_init_i2c(chip->client, &rt1711h_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(chip->dev, "%s: failed to initialize regmap\n", __func__);
		goto err_free;
	}

	device_init_wakeup(chip->dev, true);

	chip->rt1711h_ws = wakeup_source_register(chip->dev, "rt1711h_wake");
	if (IS_ERR_OR_NULL(chip->rt1711h_ws)) {
		ret = IS_ERR(chip->rt1711h_ws) ? PTR_ERR(chip->rt1711h_ws) : -ENOMEM;
		dev_err(chip->dev, "%s: wakeup_source_register failed: %d\n",
			__func__, ret);
		goto err_ws_reg;
	}

	ret = rt1711h_tcpc_dev_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "%s: rt1711 tcpc dev init fail\n", __func__);
		goto err_tcpc_reg;
	}

	ret = rt1711h_init_alert(chip->tcpc);
	if (ret < 0) {
		dev_err(chip->dev, "%s: rt1711 init alert fail\n", __func__);
		goto err_irq_init;
	}

	atomic_set(&rt1711h_ready, 1);

	dev_info(chip->dev, "%s: OK\n", __func__);
	return 0;

err_irq_init:
	rt1711h_tcpc_dev_deinit(chip);
err_tcpc_reg:
	if (!IS_ERR_OR_NULL(chip->rt1711h_ws))
		wakeup_source_unregister(chip->rt1711h_ws);
err_ws_reg:
	device_init_wakeup(chip->dev, false);
err_free:
	i2c_set_clientdata(client, NULL);
	dev_err(chip->dev, "%s: Failed\n", __func__);
	return ret;
}

static int rt1711h_i2c_remove(struct i2c_client *client)
{
	struct rt1711h_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return 0;

	atomic_set(&rt1711h_ready, 0);

	if (chip->irq) {
		disable_irq(chip->irq);
		synchronize_irq(chip->irq);
	}

	rt1711h_tcpc_dev_deinit(chip);

	if (!IS_ERR_OR_NULL(chip->rt1711h_ws))
		wakeup_source_unregister(chip->rt1711h_ws);

	device_init_wakeup(chip->dev, false);

	i2c_set_clientdata(client, NULL);

	return 0;
}

static void rt1711h_shutdown(struct i2c_client *client)
{
	struct rt1711h_chip *chip = i2c_get_clientdata(client);
	u8 rst = 0x01;

	/* Please reset IC here */
	if (chip) {
		if (chip->irq) {
			disable_irq(chip->irq);
			synchronize_irq(chip->irq);
		}
		tcpm_shutdown(chip->tcpc);
	} else {
		i2c_smbus_write_i2c_block_data(client, RT1711H_REG_SWRESET, 1, &rst);
	}
}

static int rt1711h_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rt1711h_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return 0;

	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);

	return 0;
}

static int rt1711h_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rt1711h_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return 0;

	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);

	return 0;
}

static const struct dev_pm_ops rt1711h_pm_ops = {
	.suspend	= rt1711h_i2c_suspend,
	.resume	= rt1711h_i2c_resume,
};

enum rt_chip_id {
	TYPE_RT1711H = 0,
	TYPE_RT1715,
	TYPE_RT1716,
};

static const struct i2c_device_id rt1711h_id_table[] = {
	{"rt1711h", TYPE_RT1711H},
	{"rt1715", TYPE_RT1715},
	{"rt1716", TYPE_RT1716},
	{},
};
MODULE_DEVICE_TABLE(i2c, rt1711h_id_table);

static const struct of_device_id rt_match_table[] = {
	{.compatible = "richtek,rt1711h", .data = (void *)TYPE_RT1711H},
	{.compatible = "richtek,rt1715", .data = (void *)TYPE_RT1715},
	{.compatible = "richtek,rt1716", .data = (void *)TYPE_RT1716},
	{},
};
MODULE_DEVICE_TABLE(of, rt_match_table);

static struct i2c_driver rt1711h_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "usb_type_c",
		.of_match_table = of_match_ptr(rt_match_table),
		.pm		= &rt1711h_pm_ops,
	},
	.id_table	= rt1711h_id_table,
	.probe		= rt1711h_i2c_probe,
	.remove	= rt1711h_i2c_remove,
	.shutdown	= rt1711h_shutdown,
};

module_i2c_driver(rt1711h_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_DESCRIPTION("RT1711H TCPC Driver");
MODULE_VERSION(RT1711H_DRV_VERSION);

/**** Release Note ****
 * 2.0.7_G
 * (1) Revise suspend/resume flow for IRQ
 *
 * 2.0.6_G
 * (1) Revert Vconn OC to shutdown mode
 * (2) Revise IRQ handling
 *
 * 2.0.5_G
 * (1) Utilize rt-regmap to reduce I2C accesses
 * (2) Decrease VBUS present threshold (VBUS_CAL) by 60mV (2LSBs)
 *
 * 2.0.4_G
 * (1) Mask vSafe0V IRQ before entering low power mode
 * (2) Disable auto idle mode before entering low power mode
 * (3) Reset Protocol FSM and clear RX alerts twice before clock gating
 *
 * 2.0.3_G
 * (1) Single Rp as Attatched.SRC for Ellisys TD.4.9.4
 *
 * 2.0.2_G
 * (1) Replace wake_lock with wakeup_source
 * (2) Move down the shipping off
 * (3) Add support for NoRp.SRC
 * (4) Reg0x71[7] = 1'b1 to workaround unstable VDD Iq in low power mode
 * (5) Add get_alert_mask of tcpc_ops
 *
 * 2.0.1_G
 * First released PD3.0 Driver
 */
