// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[ant_check_div]: %s: " fmt, __func__

#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

struct ant_div_check_info {
	u32 irq_gpio;
	u32 irq_gpio_flags;
	int irq;
	bool irq_disabled;
	int ant_div_check_state;
	struct input_dev *ipdev;
	struct class ant_div_sys_class;
};

struct ant_div_check_info *global_ant_div_info;

/*function*/
static irqreturn_t ant_div_interrupt(int irq, void *data)
{
	struct ant_div_check_info *ant_div_info = data;
	int ant_div_gpio;

	ant_div_gpio = gpio_get_value_cansleep(ant_div_info->irq_gpio);
	pr_info("gpio = %d\n", ant_div_gpio);
	if (ant_div_gpio == ant_div_info->ant_div_check_state)
		return IRQ_HANDLED;

	ant_div_info->ant_div_check_state = ant_div_gpio;
	pr_err("report key %s\n", ant_div_gpio ? "connect" : "disconnect");

	if (ant_div_gpio) {
		input_report_key(ant_div_info->ipdev, DIV_ANT_CONNECT, 1);
		input_report_key(ant_div_info->ipdev, DIV_ANT_CONNECT, 0);
		input_sync(ant_div_info->ipdev);
	} else {
		input_report_key(ant_div_info->ipdev, DIV_ANT_UNCONNECT, 1);
		input_report_key(ant_div_info->ipdev, DIV_ANT_UNCONNECT, 0);
		input_sync(ant_div_info->ipdev);
	}

	return IRQ_HANDLED;
}

static void ant_div_disable_irq(struct ant_div_check_info *ant_div_info, bool disable)
{
	if (disable) {
		if (!ant_div_info->irq_disabled) {
			disable_irq(ant_div_info->irq);
			ant_div_info->irq_disabled = true;
		}
	} else {
		if (ant_div_info->irq_disabled) {
			enable_irq(ant_div_info->irq);
			ant_div_info->irq_disabled = false;
		}
	}
}

static int ant_div_parse_dt(struct device *dev, struct ant_div_check_info *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "ant_check_div_gpio", 0,
						  &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pr_info("irq_gpio = %d\n", pdata->irq_gpio);
	return 0;
}

static ssize_t ant_div_state_show(struct class *class,
				  struct class_attribute *attr,
				  char *buf)
{
	struct ant_div_check_info *ant_div_info =
		container_of(class, struct ant_div_check_info, ant_div_sys_class);
	int state;

	if (!ant_div_info)
		return -ENODEV;

	state = ant_div_info->ant_div_check_state ? 3 : 2;

	pr_info("state = %d, custome_state = %d\n",
		ant_div_info->ant_div_check_state, state);

	return sysfs_emit(buf, "%d\n", state);
}

static ssize_t ant_div_state_store(struct class *class,
				   struct class_attribute *attr,
				   const char *buf, size_t size)
{
	struct ant_div_check_info *ant_div_info =
		container_of(class, struct ant_div_check_info, ant_div_sys_class);
	int rc, state;

	if (!ant_div_info)
		return -ENODEV;

	rc = kstrtoint(buf, 10, &state);
	if (rc) {
		pr_err("kstrtoint failed, rc=%d\n", rc);
		return rc;
	}

	if (state == 0)
		ant_div_disable_irq(ant_div_info, true);
	else
		ant_div_disable_irq(ant_div_info, false);

	return size;
}

static struct class_attribute ant_div_state = __ATTR_RW(ant_div_state);

static int ant_div_register_class_dev(struct ant_div_check_info *ant_div_info)
{
	int err;

	ant_div_info->ant_div_sys_class.name = "ant_div_class";
	ant_div_info->ant_div_sys_class.owner = THIS_MODULE;

	err = class_register(&ant_div_info->ant_div_sys_class);
	if (err)
		return err;

	err = class_create_file(&ant_div_info->ant_div_sys_class, &ant_div_state);
	if (err) {
		class_unregister(&ant_div_info->ant_div_sys_class);
		return err;
	}

	return 0;
}

static int ant_div_probe(struct platform_device *pdev)
{
	int rc = 0;
	int err;
	struct ant_div_check_info *ant_div_info;

	pr_info("entry\n");
	if (!pdev->dev.of_node)
		return -ENODEV;

	ant_div_info = kzalloc(sizeof(*ant_div_info), GFP_KERNEL);
	if (!ant_div_info)
		return -ENOMEM;

	err = ant_div_parse_dt(&pdev->dev, ant_div_info);
	if (err) {
		pr_err("DT parsing failed\n");
		goto free_struct;
	}

	platform_set_drvdata(pdev, ant_div_info);

	rc = device_init_wakeup(&pdev->dev, true);
	if (rc) {
		pr_err("device_init_wakeup failed: %d\n", rc);
		goto put_drvdata_free;
	}

	/* input system config */
	ant_div_info->ipdev = input_allocate_device();
	if (!ant_div_info->ipdev) {
		pr_err("input_allocate_device fail\n");
		rc = -ENOMEM;
		goto free_wakeup;
	}

	ant_div_info->ipdev->name = "ant_div_check-input";
	input_set_capability(ant_div_info->ipdev, EV_KEY, DIV_ANT_CONNECT);
	input_set_capability(ant_div_info->ipdev, EV_KEY, DIV_ANT_UNCONNECT);

	rc = input_register_device(ant_div_info->ipdev);
	if (rc) {
		pr_err("input_register_device fail, rc=%d\n", rc);
		input_free_device(ant_div_info->ipdev);
		goto free_wakeup;
	}

	/* interrupt config */
	if (!gpio_is_valid(ant_div_info->irq_gpio)) {
		pr_err("irq gpio not provided\n");
		rc = -EINVAL;
		goto unregister_input;
	}

	rc = gpio_request(ant_div_info->irq_gpio, "ant_check_div");
	if (rc < 0) {
		pr_err("gpio_request fail, rc=%d\n", rc);
		goto unregister_input;
	}

	rc = gpio_direction_input(ant_div_info->irq_gpio);
	if (rc < 0) {
		pr_err("gpio_direction_input fail, rc=%d\n", rc);
		goto free_gpio;
	}

	ant_div_info->ant_div_check_state = gpio_get_value(ant_div_info->irq_gpio);
	pr_info("gpios = %d\n", ant_div_info->ant_div_check_state);

	ant_div_info->irq = gpio_to_irq(ant_div_info->irq_gpio);
	if (ant_div_info->irq < 0) {
		rc = ant_div_info->irq;
		pr_err("gpio_to_irq failed: %d\n", rc);
		goto free_gpio;
	}
	pr_info("irq = %d\n", ant_div_info->irq);

	rc = devm_request_threaded_irq(&pdev->dev, ant_div_info->irq,
				       NULL, ant_div_interrupt,
				       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				       "ant-div-switch-irq", ant_div_info);
	if (rc < 0) {
		pr_err("devm_request_threaded_irq fail, rc=%d\n", rc);
		goto free_gpio;
	}

	rc = ant_div_register_class_dev(ant_div_info);
	if (rc) {
		pr_err("ant_div_register_class_dev fail, rc=%d\n", rc);
		goto free_gpio;
	}

	global_ant_div_info = ant_div_info;
	pr_info("done\n");

	return 0;

free_gpio:
	gpio_free(ant_div_info->irq_gpio);
unregister_input:
	input_unregister_device(ant_div_info->ipdev);
free_wakeup:
	device_init_wakeup(&pdev->dev, false);
put_drvdata_free:
	platform_set_drvdata(pdev, NULL);
free_struct:
	kfree(ant_div_info);
	pr_err("failed\n");
	return rc;
}

static int ant_div_remove(struct platform_device *pdev)
{
	struct ant_div_check_info *ant = platform_get_drvdata(pdev);

	if (!ant)
		return 0;

	global_ant_div_info = NULL;
	class_remove_file(&ant->ant_div_sys_class, &ant_div_state);
	class_unregister(&ant->ant_div_sys_class);

	ant_div_disable_irq(ant, false);

	if (gpio_is_valid(ant->irq_gpio))
		gpio_free(ant->irq_gpio);

	input_unregister_device(ant->ipdev);

	device_init_wakeup(&pdev->dev, false);

	platform_set_drvdata(pdev, NULL);
	kfree(ant);

	return 0;
}

static int ant_div_suspend(struct device *dev)
{
	struct ant_div_check_info *ant = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(ant->irq);

	return 0;
}

static int ant_div_resume(struct device *dev)
{
	struct ant_div_check_info *ant = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(ant->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ant_div_pm_ops, ant_div_suspend, ant_div_resume);

static const struct of_device_id sn_match_table[] = {
	{ .compatible = "ant_check_div", },
	{ },
};
MODULE_DEVICE_TABLE(of, sn_match_table);

static struct platform_driver ant_div_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "ant_check_div",
		.pm		= &ant_div_pm_ops,
		.of_match_table	= of_match_ptr(sn_match_table),
	},
	.probe		= ant_div_probe,
	.remove		= ant_div_remove,
};

static int __init ant_div_init(void)
{
	return platform_driver_register(&ant_div_driver);
}

static void __exit ant_div_exit(void)
{
	platform_driver_unregister(&ant_div_driver);
}

module_init(ant_div_init);
module_exit(ant_div_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lisuyang");
