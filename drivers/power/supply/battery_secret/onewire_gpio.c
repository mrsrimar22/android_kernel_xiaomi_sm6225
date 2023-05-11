// SPDX-License-Identifier: GPL-2.0-only
/*
 * onewire_gpio - Bit-bang 1-Wire bus driver using Linux GPIO Consumer API (gpiod)
 *
 * Copyright (c) 2016 Xiaomi Inc.
 *
 */
#define pr_fmt(fmt) "[Onewire]: %s: " fmt, __func__

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irqflags.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "onewire_gpio.h"

#define DRV_STRENGTH_16MA	GENMASK(8, 6)
#define GPIO_ENABLE		BIT(12)
#define GPIO_OUTPUT		BIT(9)
#define GPIO_INPUT		0
#define GPIO_PULL_UP		0x3
#define OUTPUT_HIGH		0x2
#define OUTPUT_LOW		0x0

#define TLMM_GPIO108_CFG_ADDR	0x0096C000
#define TLMM_GPIO_INOUT_OFFSET	0x04

struct onewire_gpio_data {
	struct platform_device *pdev;
	struct device *dev;

	struct gpio_desc *gpiod;
	void __iomem *gpio_cfg_reg;
	void __iomem *gpio_in_out_reg;

	u32 onewire_gpio_cfg_addr;
	u32 onewire_gpio_inout_offset;

	struct pinctrl *ow_gpio_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_sleep;

	char bus_label[32];
};

static struct class *onewire_class;

static int match_bus_by_name(struct device *dev, const void *data)
{
	struct onewire_gpio_data *od = dev_get_drvdata(dev);

	return od && strcmp(od->bus_label, (const char *)data) == 0;
}

struct onewire_gpio_data *onewire_bus_get(const char *name)
{
	struct device *dev;

	if (!onewire_class || !name)
		return NULL;

	dev = class_find_device(onewire_class, NULL, (const void *)name,
				match_bus_by_name);

	return dev ? dev_get_drvdata(dev) : NULL;
}
EXPORT_SYMBOL(onewire_bus_get);

void onewire_bus_put(struct onewire_gpio_data *od)
{
	if (od && od->dev)
		put_device(od->dev);
}
EXPORT_SYMBOL(onewire_bus_put);

static inline void ow_pin_output(struct onewire_gpio_data *od)
{
	writel(GPIO_ENABLE | DRV_STRENGTH_16MA | GPIO_OUTPUT | GPIO_PULL_UP,
	       od->gpio_cfg_reg);
}

static inline void ow_pin_input(struct onewire_gpio_data *od)
{
	writel(GPIO_ENABLE | DRV_STRENGTH_16MA | GPIO_INPUT | GPIO_PULL_UP,
	       od->gpio_cfg_reg);
}

static inline void ow_pin_high(struct onewire_gpio_data *od)
{
	writel(OUTPUT_HIGH, od->gpio_in_out_reg);
}

static inline void ow_pin_low(struct onewire_gpio_data *od)
{
	writel(OUTPUT_LOW, od->gpio_in_out_reg);
}

static inline unsigned int ow_pin_read(struct onewire_gpio_data *od)
{
	return readl(od->gpio_in_out_reg) & 0x01;
}

/**
 * ow_read_bit - read one 1-Wire bit slot.
 *
 * IRQs are disabled only for the timing-critical window: from the master's
 * falling edge through the 500 ns sample point. The post-sample recovery
 * delay (udelay(5) + pin idle + udelay(6)) runs with IRQs enabled.
 */
static u8 ow_read_bit(struct onewire_gpio_data *od)
{
	unsigned long flags;
	unsigned int val;

	local_irq_save(flags);
	ow_pin_output(od);
	ow_pin_low(od);
	udelay(1);
	ow_pin_input(od);
	ndelay(500);
	val = ow_pin_read(od);
	local_irq_restore(flags);

	udelay(5);
	ow_pin_high(od);
	ow_pin_output(od);
	udelay(6);

	return (u8)val;
}

/**
 * ow_write_bit - write one 1-Wire bit slot.
 *
 * IRQs are disabled for the drive-low -> drive-high window (~11 us).
 */
static void ow_write_bit(struct onewire_gpio_data *od, u8 bitval)
{
	unsigned long flags;

	ow_pin_output(od);
	udelay(5);
	local_irq_save(flags);
	ow_pin_low(od);
	udelay(1);
	if (bitval)
		ow_pin_high(od);
	udelay(10);
	local_irq_restore(flags);

	ow_pin_high(od);
	udelay(6);
}

static int ow_gpio_runtime_suspend(struct device *dev)
{
	struct onewire_gpio_data *od = dev_get_drvdata(dev);
	int ret;

	ret = pinctrl_select_state(od->ow_gpio_pinctrl,
				   od->pinctrl_state_sleep);
	if (ret)
		dev_err(dev, "failed to select sleep state: %d\n", ret);
	return ret;
}

static int ow_gpio_runtime_resume(struct device *dev)
{
	struct onewire_gpio_data *od = dev_get_drvdata(dev);
	int ret;

	ret = pinctrl_select_state(od->ow_gpio_pinctrl,
				   od->pinctrl_state_active);
	if (ret)
		dev_err(dev, "failed to select active state: %d\n", ret);
	return ret;
}

static const struct dev_pm_ops ow_gpio_pm_ops = {
	SET_RUNTIME_PM_OPS(ow_gpio_runtime_suspend, ow_gpio_runtime_resume, NULL)
};

static void ow_gpio_activate(struct onewire_gpio_data *od)
{
	pm_runtime_get_sync(&od->pdev->dev);
}

static void ow_gpio_sleep(struct onewire_gpio_data *od)
{
	pm_runtime_mark_last_busy(&od->pdev->dev);
	pm_runtime_put_autosuspend(&od->pdev->dev);
}

u8 onewire_reset(struct onewire_gpio_data *od)
{
	unsigned long flags;
	u8 presence;

	if (!od)
		return (u8)-ENODEV;

	ow_gpio_activate(od);
	ow_pin_output(od);
	ow_pin_low(od);
	udelay(50);
	local_irq_save(flags);
	ow_pin_high(od);
	ow_pin_input(od);
	udelay(7);
	presence = (u8)ow_pin_read(od);
	local_irq_restore(flags);
	udelay(50);
	ow_gpio_sleep(od);

	return presence;
}
EXPORT_SYMBOL(onewire_reset);

u8 onewire_read_byte(struct onewire_gpio_data *od)
{
	unsigned long flags;
	u8 i, value = 0;

	if (!od)
		return (u8)-ENODEV;

	ow_gpio_activate(od);
	local_irq_save(flags);
	for (i = 0; i < 8; i++) {
		if (ow_read_bit(od))
			value |= BIT(i);
	}
	local_irq_restore(flags);
	ow_gpio_sleep(od);

	return value;
}
EXPORT_SYMBOL(onewire_read_byte);

void onewire_write_byte(struct onewire_gpio_data *od, u8 val)
{
	unsigned long flags;
	u8 i;

	if (!od)
		return;

	ow_gpio_activate(od);
	ow_pin_output(od);
	local_irq_save(flags);
	for (i = 0; i < 8; i++)
		ow_write_bit(od, (val >> i) & 0x01);
	local_irq_restore(flags);
	ow_gpio_sleep(od);
}
EXPORT_SYMBOL(onewire_write_byte);

void onewire_software_reset(struct onewire_gpio_data *od)
{
	unsigned long flags;

	if (!od)
		return;

	ow_gpio_activate(od);
	ow_pin_output(od);
	ow_pin_low(od);
	local_irq_save(flags);
	udelay(480);
	ow_pin_high(od);
	ow_pin_input(od);
	udelay(70);
	(void)ow_pin_read(od);
	local_irq_restore(flags);
	udelay(410);
	ow_gpio_sleep(od);
}
EXPORT_SYMBOL(onewire_software_reset);

static ssize_t ow_gpio_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct onewire_gpio_data *od = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ow_pin_read(od));
}

static ssize_t ow_gpio_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct onewire_gpio_data *od = dev_get_drvdata(dev);
	u8 rom_id[8] = {};
	u8 result;
	u8 i;
	int val, ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	switch (val) {
	case 0:
		ow_pin_low(od);
		pr_info("pin: LOW\n");
		break;
	case 1:
		ow_pin_high(od);
		pr_info("pin: HIGH\n");
		break;
	case 2:
		ow_pin_output(od);
		pr_info("pin: OUTPUT\n");
		break;
	case 3:
		ow_pin_input(od);
		pr_info("pin: INPUT\n");
		break;
	case 4:
		result = onewire_reset(od);
		pr_info("ow_reset: %s (0x%02x)\n",
			result ? "no device" : "present", result);
		break;
	case 5:
		result = ow_read_bit(od);
		pr_info("read_bit: 0x%02x\n", result);
		break;
	case 6:
		ow_write_bit(od, 0x01);
		pr_info("write_bit: 1\n");
		break;
	case 7:
		result = onewire_reset(od);
		pr_info("ow_reset: %s (0x%02x)\n",
			result ? "no device" : "present", result);
		onewire_write_byte(od, 0x33);
		for (i = 0; i < 8; i++)
			rom_id[i] = onewire_read_byte(od);
		pr_info("RomID = %8phC\n", rom_id);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(ow_gpio, 0660, ow_gpio_show, ow_gpio_store);

static struct attribute *onewire_gpio_attrs[] = {
	&dev_attr_ow_gpio.attr,
	NULL,
};

static const struct attribute_group onewire_gpio_group = {
	.attrs = onewire_gpio_attrs,
};

static const struct attribute_group *onewire_gpio_groups[] = {
	&onewire_gpio_group,
	NULL,
};

static int onewire_gpio_parse_dt(struct device *dev,
				 struct onewire_gpio_data *od)
{
	struct device_node *np = dev->of_node;
	const char *label;
	u32 reg[2];
	int ret;

	ret = of_property_read_string(np, "label", &label);
	if (ret) {
		pr_err("missing 'label' property: %d\n", ret);
		return ret;
	}
	strscpy(od->bus_label, label, sizeof(od->bus_label));
	pr_debug("bus label: '%s'\n", od->bus_label);

	if (!of_property_read_u32_array(np, "xiaomi,onewire-cfg-addr", reg, 2)) {
		od->onewire_gpio_cfg_addr = reg[0];
		od->onewire_gpio_inout_offset = reg[1];
	} else {
		od->onewire_gpio_cfg_addr = TLMM_GPIO108_CFG_ADDR;
		od->onewire_gpio_inout_offset = TLMM_GPIO_INOUT_OFFSET;
	}

	return 0;
}

static int onewire_gpio_pinctrl_init(struct onewire_gpio_data *od)
{
	int ret;

	od->ow_gpio_pinctrl = devm_pinctrl_get(&od->pdev->dev);
	if (IS_ERR_OR_NULL(od->ow_gpio_pinctrl)) {
		ret = PTR_ERR(od->ow_gpio_pinctrl);
		pr_err("devm_pinctrl_get: %d\n", ret);
		return ret;
	}

	od->pinctrl_state_active = pinctrl_lookup_state(od->ow_gpio_pinctrl,
							"onewire_active");
	if (IS_ERR_OR_NULL(od->pinctrl_state_active)) {
		ret = PTR_ERR(od->pinctrl_state_active);
		pr_err("lookup onewire_active: %d\n", ret);
		return ret;
	}

	od->pinctrl_state_sleep = pinctrl_lookup_state(od->ow_gpio_pinctrl,
						       "onewire_sleep");
	if (IS_ERR_OR_NULL(od->pinctrl_state_sleep)) {
		ret = PTR_ERR(od->pinctrl_state_sleep);
		pr_err("lookup onewire_sleep: %d\n", ret);
		return ret;
	}

	return 0;
}

static int onewire_gpio_probe(struct platform_device *pdev)
{
	struct onewire_gpio_data *od;
	struct device *dev = &pdev->dev;
	int ret;

	pr_info("entry\n");

	if (!dev->of_node || !of_device_is_available(dev->of_node))
		return -ENODEV;

	od = devm_kzalloc(dev, sizeof(*od), GFP_KERNEL);
	if (!od)
		return -ENOMEM;

	ret = onewire_gpio_parse_dt(dev, od);
	if (ret)
		return ret;

	od->pdev = pdev;
	platform_set_drvdata(pdev, od);

	ret = onewire_gpio_pinctrl_init(od);
	if (ret) {
		pr_err("init pinctrl failed: %d\n", ret);
		return ret;
	}

	ret = pinctrl_select_state(od->ow_gpio_pinctrl,
				   od->pinctrl_state_active);
	if (ret) {
		pr_err("select active pinstate: %d\n", ret);
		return ret;
	}

	od->gpiod = gpiod_get_optional(dev, "xiaomi,onewire", GPIOD_OUT_HIGH);
	if (IS_ERR(od->gpiod)) {
		ret = PTR_ERR(od->gpiod);
		pr_err("failed to get 'onewire' gpiod: %d\n", ret);
		return ret;
	}
	if (!od->gpiod) {
		pr_err("missing required 'xiaomi,onewire-gpios' in dt\n");
		return -ENODEV;
	}

	od->gpio_cfg_reg = devm_ioremap(dev, od->onewire_gpio_cfg_addr, 0x8);
	if (!od->gpio_cfg_reg) {
		pr_err("ioremap gpio register block (0x%08x) failed\n",
		       od->onewire_gpio_cfg_addr);
		ret = -ENOMEM;
		goto err_put_gpiod;
	}
	od->gpio_in_out_reg = od->gpio_cfg_reg + od->onewire_gpio_inout_offset;

	pr_info("gpio_regs mapped at %pK (physical 0x%08x, offset 0x%x)\n",
		od->gpio_cfg_reg, od->onewire_gpio_cfg_addr,
		od->onewire_gpio_inout_offset);

	pm_runtime_set_autosuspend_delay(&pdev->dev, 500);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	if (!dev->parent || !dev->parent->parent) {
		pr_err("parent chain unavailable\n");
		ret = -ENODEV;
		goto err_disable_pm;
	}

	od->dev = device_create(onewire_class, dev->parent->parent,
				MKDEV(0, 0), od, "onewirectrl");
	if (IS_ERR(od->dev)) {
		ret = PTR_ERR(od->dev);
		pr_err("device_create: %d\n", ret);
		goto err_disable_pm;
	}

	ret = sysfs_create_link(&od->dev->kobj, &dev->kobj, "pltdev");
	if (ret) {
		pr_err("sysfs_create_link: %d\n", ret);
		goto err_destroy_dev;
	}

	ow_gpio_sleep(od);
	pr_info("success\n");
	return 0;

err_destroy_dev:
	device_destroy(onewire_class, MKDEV(0, 0));
err_disable_pm:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
err_put_gpiod:
	gpiod_put(od->gpiod);
	return ret;
}

static int onewire_gpio_remove(struct platform_device *pdev)
{
	struct onewire_gpio_data *od = platform_get_drvdata(pdev);

	if (!od)
		return 0;

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	if (od->dev) {
		sysfs_remove_link(&od->dev->kobj, "pltdev");
		device_destroy(onewire_class, MKDEV(0, 0));
	}

	if (od->gpiod)
		gpiod_put(od->gpiod);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id onewire_gpio_dt_match[] = {
	{ .compatible = "xiaomi,onewire_gpio", },
	{ },
};
MODULE_DEVICE_TABLE(of, onewire_gpio_dt_match);

static struct platform_driver onewire_gpio_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "onewire_gpio",
		.of_match_table	= of_match_ptr(onewire_gpio_dt_match),
		.pm		= &ow_gpio_pm_ops,
	},
	.probe	= onewire_gpio_probe,
	.remove	= onewire_gpio_remove,
};

static int __init onewire_gpio_init(void)
{
	int ret;

	pr_info("entry\n");

	onewire_class = class_create(THIS_MODULE, "onewire");
	if (IS_ERR(onewire_class)) {
		pr_err("class_create failed\n");
		return PTR_ERR(onewire_class);
	}
	onewire_class->dev_groups = onewire_gpio_groups;

	ret = platform_driver_register(&onewire_gpio_driver);
	if (ret) {
		pr_err("platform_driver_register failed: %d\n", ret);
		goto err_class;
	}

	pr_info("driver registered\n");
	return 0;

err_class:
	class_destroy(onewire_class);
	return ret;
}

static void __exit onewire_gpio_exit(void)
{
	pr_info("exit\n");
	platform_driver_unregister(&onewire_gpio_driver);
	class_destroy(onewire_class);
}

subsys_initcall(onewire_gpio_init);
module_exit(onewire_gpio_exit);

MODULE_AUTHOR("Xiaomi Inc.");
MODULE_DESCRIPTION("1-Wire bit-bang GPIO bus driver");
MODULE_LICENSE("GPL");
