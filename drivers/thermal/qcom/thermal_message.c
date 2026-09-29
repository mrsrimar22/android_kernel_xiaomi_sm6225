// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Xiaomi thermal message driver
 *
 * Exposes the "thermal_message" sysfs node used by the Xiaomi thermal HAL.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/cpu_cooling.h>
#include <drm/drm_panel.h>

#include "../thermal_core.h"

#define THERMAL_MESSAGE_NAME	"thermal_message"
#define CPU_LIMITS_PARAM_NUM	2

struct thermal_message {
	struct device *dev;
	struct device *msg_dev;
	struct drm_panel *prim_panel;
	struct notifier_block panel_nb;
	atomic_t screen_state;		/* 1: on; 0: off; -1: unknown */
	atomic_t switch_mode;
	atomic_t temp_state;
	char boost_buf[PAGE_SIZE];
	const char *board_sensor;
	char board_sensor_temp[PAGE_SIZE];
	struct mutex sysfs_lock;
};

static ssize_t screen_state_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct thermal_message *tm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&tm->screen_state));
}
static DEVICE_ATTR_RO(screen_state);

static ssize_t sconfig_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct thermal_message *tm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&tm->switch_mode));
}

static ssize_t sconfig_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct thermal_message *tm = dev_get_drvdata(dev);
	char *kbuf;
	int val;
	int ret;

	if (len == 0 || len > PAGE_SIZE - 1)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	memcpy(kbuf, buf, len);
	kbuf[len] = '\0';

	ret = kstrtoint(kbuf, 10, &val);
	kfree(kbuf);
	if (ret)
		return ret;

	atomic_set(&tm->switch_mode, val);

	return len;
}
static DEVICE_ATTR_RW(sconfig);

static ssize_t boost_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	struct thermal_message *tm = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&tm->sysfs_lock);
	ret = scnprintf(buf, PAGE_SIZE, "%s", tm->boost_buf);
	mutex_unlock(&tm->sysfs_lock);

	return ret;
}

static ssize_t boost_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t len)
{
	struct thermal_message *tm = dev_get_drvdata(dev);
	char *kbuf;
	ssize_t ret = len;

	if (len == 0 || len > PAGE_SIZE - 1)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	memcpy(kbuf, buf, len);
	kbuf[len] = '\0';

	mutex_lock(&tm->sysfs_lock);
	scnprintf(tm->boost_buf, PAGE_SIZE, "%s", kbuf);
	mutex_unlock(&tm->sysfs_lock);

	kfree(kbuf);
	return ret;
}
static DEVICE_ATTR_RW(boost);

static ssize_t temp_state_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct thermal_message *tm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&tm->temp_state));
}

static ssize_t temp_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct thermal_message *tm = dev_get_drvdata(dev);
	char *kbuf;
	int val;
	int ret;

	if (len == 0 || len > PAGE_SIZE - 1)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	memcpy(kbuf, buf, len);
	kbuf[len] = '\0';

	ret = kstrtoint(kbuf, 10, &val);
	kfree(kbuf);
	if (ret)
		return ret;

	atomic_set(&tm->temp_state, val);

	return len;
}
static DEVICE_ATTR_RW(temp_state);

static ssize_t cpu_limits_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	return 0;
}

static ssize_t cpu_limits_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	char *kbuf;
	unsigned int cpu;
	unsigned int max;
	int scanned;

	if (len == 0 || len > PAGE_SIZE - 1)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	memcpy(kbuf, buf, len);
	kbuf[len] = '\0';

	scanned = sscanf(kbuf, "cpu%u %u", &cpu, &max);
	kfree(kbuf);
	if (scanned != CPU_LIMITS_PARAM_NUM) {
		pr_err("Thermal: input param error, cannot parse param.\n");
		return -EINVAL;
	}

	cpu_limits_set_level(cpu, max);

	return len;
}
static DEVICE_ATTR_RW(cpu_limits);

static ssize_t board_sensor_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct thermal_message *tm = dev_get_drvdata(dev);
	const char *sensor;
	ssize_t ret;

	mutex_lock(&tm->sysfs_lock);
	sensor = tm->board_sensor ? tm->board_sensor : "invalid";
	if (!tm->board_sensor)
		pr_warn("Thermal: thermal_board_sensor invalid.\n");
	ret = scnprintf(buf, PAGE_SIZE, "%s", sensor);
	mutex_unlock(&tm->sysfs_lock);

	return ret;
}
static DEVICE_ATTR_RO(board_sensor);

static ssize_t board_sensor_temp_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct thermal_message *tm = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&tm->sysfs_lock);
	ret = scnprintf(buf, PAGE_SIZE, "%s", tm->board_sensor_temp);
	mutex_unlock(&tm->sysfs_lock);

	return ret;
}

static ssize_t board_sensor_temp_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct thermal_message *tm = dev_get_drvdata(dev);
	char *kbuf;
	ssize_t ret = len;

	if (len == 0 || len > PAGE_SIZE - 1)
		return -EINVAL;

	kbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	memcpy(kbuf, buf, len);
	kbuf[len] = '\0';

	mutex_lock(&tm->sysfs_lock);
	scnprintf(tm->board_sensor_temp, PAGE_SIZE, "%s", kbuf);
	mutex_unlock(&tm->sysfs_lock);

	kfree(kbuf);
	return ret;
}
static DEVICE_ATTR_RW(board_sensor_temp);

static struct attribute *thermal_message_attrs[] = {
	&dev_attr_screen_state.attr,
	&dev_attr_sconfig.attr,
	&dev_attr_boost.attr,
	&dev_attr_temp_state.attr,
	&dev_attr_cpu_limits.attr,
	&dev_attr_board_sensor.attr,
	&dev_attr_board_sensor_temp.attr,
	NULL,
};

static const struct attribute_group thermal_message_group = {
	.attrs = thermal_message_attrs,
};

static const struct attribute_group *thermal_message_groups[] = {
	&thermal_message_group,
	NULL,
};

static int screen_state_for_thermal_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct thermal_message *tm = container_of(nb, struct thermal_message,
						  panel_nb);
	struct drm_panel_notifier *evdata = data;
	int power_mode;

	if (val != DRM_PANEL_EVENT_BLANK || !evdata || !evdata->data)
		return NOTIFY_DONE;

	power_mode = *(int *)(evdata->data);

	switch (power_mode) {
	case DRM_PANEL_BLANK_LP1:
	case DRM_PANEL_BLANK_LP2:
	case DRM_PANEL_BLANK_POWERDOWN:
		pr_info("%s: panel off/doze (blank: %d)\n", __func__, power_mode);
		atomic_set(&tm->screen_state, 0);
		break;
	case DRM_PANEL_BLANK_UNBLANK:
		pr_info("%s: panel on\n", __func__);
		atomic_set(&tm->screen_state, 1);
		break;
	default:
		return NOTIFY_DONE;
	}

	sysfs_notify(&tm->dev->kobj, NULL, "screen_state");
	return NOTIFY_OK;
}

static int thermal_message_find_panel(struct thermal_message *tm)
{
	struct device_node *np = tm->dev->of_node;
	struct device_node *node;
	struct drm_panel *panel;
	int i, count, ret = -ENODEV;

	if (!np)
		return -ENODEV;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return -ENODEV;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		if (!node)
			continue;

		panel = of_drm_find_panel(node);
		of_node_put(node);

		if (!IS_ERR(panel)) {
			tm->prim_panel = panel;
			pr_info("Thermal: panel %d ready\n", i);
			return 0;
		}

		pr_info("Thermal: panel %d not ready (%ld)\n", i, PTR_ERR(panel));

		if (PTR_ERR(panel) == -EPROBE_DEFER)
			ret = -EPROBE_DEFER;
	}

	return ret;
}

static int thermal_message_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct thermal_message *tm;
	int ret;

	tm = devm_kzalloc(dev, sizeof(*tm), GFP_KERNEL);
	if (!tm)
		return -ENOMEM;

	tm->dev = dev;
	atomic_set(&tm->screen_state, -1);
	atomic_set(&tm->switch_mode, -1);
	atomic_set(&tm->temp_state, 0);
	mutex_init(&tm->sysfs_lock);
	platform_set_drvdata(pdev, tm);

	if (of_property_read_string(dev->of_node, "board-sensor",
				    &tm->board_sensor))
		pr_warn("Thermal: board-sensor missing\n");
	else
		pr_info("Thermal: board sensor: %s\n", tm->board_sensor);

	ret = thermal_message_find_panel(tm);
	if (ret)
		/* Panel is mandatory: let the driver core retry once it is up. */
		return ret;

	tm->panel_nb.notifier_call = screen_state_for_thermal_callback;
	ret = drm_panel_notifier_register(tm->prim_panel, &tm->panel_nb);
	if (ret < 0) {
		dev_warn(dev, "Thermal: drm_panel_notifier_register failed: %d\n",
			 ret);
		return ret;
	}

	tm->msg_dev = device_create_with_groups(&thermal_class, dev, 0, tm,
						thermal_message_groups,
						THERMAL_MESSAGE_NAME);
	if (IS_ERR(tm->msg_dev)) {
		ret = PTR_ERR(tm->msg_dev);
		tm->msg_dev = NULL;
		dev_err(dev, "Thermal: failed to create %s device: %d\n",
			THERMAL_MESSAGE_NAME, ret);
		drm_panel_notifier_unregister(tm->prim_panel, &tm->panel_nb);
		return ret;
	}

	dev_info(dev, "Thermal: %s registered\n", THERMAL_MESSAGE_NAME);
	return 0;
}

static int thermal_message_remove(struct platform_device *pdev)
{
	struct thermal_message *tm = platform_get_drvdata(pdev);

	/* Stop the panel notifier before tearing the sysfs node down. */
	if (tm->prim_panel) {
		drm_panel_notifier_unregister(tm->prim_panel, &tm->panel_nb);
		tm->prim_panel = NULL;
	}

	if (tm->msg_dev) {
		device_unregister(tm->msg_dev);
		tm->msg_dev = NULL;
	}

	return 0;
}

static const struct of_device_id thermal_message_of_match[] = {
	{ .compatible = "xiaomi,thermal-message" },
	{ }
};
MODULE_DEVICE_TABLE(of, thermal_message_of_match);

static struct platform_driver thermal_message_driver = {
	.probe = thermal_message_probe,
	.remove = thermal_message_remove,
	.driver = {
		.name = THERMAL_MESSAGE_NAME,
		.of_match_table = thermal_message_of_match,
	},
};
builtin_platform_driver(thermal_message_driver);

MODULE_DESCRIPTION("Xiaomi thermal message sysfs driver");
MODULE_LICENSE("GPL v2");
