// SPDX-License-Identifier: GPL-2.0-only
/*************************************************************************
 * Author: zengrui
 * Created Time : 2021年10月09日 星期六 16时22分35秒
 * File Name: xiaomi_touch
 * Description:
 ************************************************************************/

#include "xiaomi_touch.h"

DEFINE_STATIC_SRCU(xmi_srcu);
static struct xiaomi_touch_pdata __rcu *g_xmi_pdata;
static DEFINE_MUTEX(xmi_pdata_lock);

static int xiaomi_touch_dev_open(struct inode *inode, struct file *file)
{
	struct xiaomi_touch_pdata *pdata;
	int idx;

	idx = srcu_read_lock(&xmi_srcu);
	pdata = srcu_dereference(g_xmi_pdata, &xmi_srcu);
	srcu_read_unlock(&xmi_srcu, idx);

	if (!pdata) {
		pr_err("%s: device not ready\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static ssize_t xiaomi_touch_dev_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	return 0;
}

static ssize_t xiaomi_touch_dev_write(struct file *file,
				      const char __user *buf,
				      size_t count, loff_t *pos)
{
	return 0;
}

static unsigned int xiaomi_touch_dev_poll(struct file *file, poll_table *wait)
{
	return 0;
}

static long xiaomi_touch_dev_ioctl(struct file *file,
				   unsigned int cmd,
				   unsigned long arg)
{
	struct xiaomi_touch_pdata *pdata;
	struct xiaomi_touch_interface *touch_data;
	struct xiaomi_touch *touch_dev;
	int buf[VALUE_TYPE_SIZE] = {0, };
	void __user *argp = (void __user *)arg;
	int user_cmd = _IOC_NR(cmd);
	int r = 0, err = 0;
	int idx;

	if (copy_from_user(buf, argp, sizeof(buf)))
		return -EFAULT;

	pr_info("%s: cmd: %d, mode: %d, value: %d\n",
		__func__, user_cmd, buf[1], buf[2]);

	idx = srcu_read_lock(&xmi_srcu);
	pdata = srcu_dereference(g_xmi_pdata, &xmi_srcu);
	if (!pdata) {
		err = -ENODEV;
		goto unlock_srcu;
	}

	touch_data = pdata->touch_data;
	touch_dev = &pdata->touch_dev;

	if (!touch_data) {
		err = -ENODEV;
		goto unlock_srcu;
	}

	mutex_lock(&touch_dev->mutex);

	switch (user_cmd) {
	case SET_CUR_VALUE:
		if (touch_data->set_mode_value) {
			r = touch_data->set_mode_value(touch_data, buf[1], buf[2]);
			if (r < 0)
				err = r;
			else
				buf[0] = r;
		} else {
			err = -ENOTTY;
		}
		break;
	case GET_CUR_VALUE:
	case GET_DEF_VALUE:
	case GET_MIN_VALUE:
	case GET_MAX_VALUE:
		if (touch_data->get_mode_value) {
			r = touch_data->get_mode_value(touch_data, buf[1], user_cmd);
			if (r < 0)
				err = r;
			else
				buf[0] = r;
		} else {
			err = -ENOTTY;
		}
		break;
	case RESET_MODE:
		if (touch_data->reset_mode) {
			r = touch_data->reset_mode(touch_data, buf[1]);
			if (r < 0)
				err = r;
			else
				buf[0] = r;
		} else {
			err = -ENOTTY;
		}
		break;
	case GET_MODE_VALUE:
		if (touch_data->get_mode_all) {
			r = touch_data->get_mode_all(touch_data, buf[1], buf);
			if (r < 0)
				err = r;
		} else {
			err = -ENOTTY;
		}
		break;
	default:
		pr_err("%s: unsupported ioctl cmd %d\n", __func__, user_cmd);
		err = -EINVAL;
		break;
	}

	mutex_unlock(&touch_dev->mutex);

unlock_srcu:
	srcu_read_unlock(&xmi_srcu, idx);

	if (!err) {
		if (copy_to_user(argp, buf, sizeof(buf)))
			err = -EFAULT;
	} else {
		pr_err("%s: driver operation failed: %d\n", __func__, err);
	}

	return err;
}

static int xiaomi_touch_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations xiaomitouch_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= xiaomi_touch_dev_open,
	.read		= xiaomi_touch_dev_read,
	.write		= xiaomi_touch_dev_write,
	.poll		= xiaomi_touch_dev_poll,
	.unlocked_ioctl	= xiaomi_touch_dev_ioctl,
	.compat_ioctl	= xiaomi_touch_dev_ioctl,
	.release	= xiaomi_touch_dev_release,
	.llseek		= no_llseek,
};

int xiaomitouch_register_modedata(struct xiaomi_touch_interface *data)
{
	struct xiaomi_touch_pdata *pdata;
	struct xiaomi_touch_interface *touch_data;
	int idx;

	idx = srcu_read_lock(&xmi_srcu);
	pdata = srcu_dereference(g_xmi_pdata, &xmi_srcu);
	if (!pdata) {
		pr_err("%s: device not ready\n", __func__);
		srcu_read_unlock(&xmi_srcu, idx);
		return -ENODEV;
	}

	touch_data = pdata->touch_data;
	if (!touch_data) {
		pr_err("%s: touch_data not allocated\n", __func__);
		srcu_read_unlock(&xmi_srcu, idx);
		return -ENOMEM;
	}

	mutex_lock(&pdata->touch_dev.mutex);
	memcpy(touch_data->touch_mode, data->touch_mode,
	       sizeof(touch_data->touch_mode));
	touch_data->set_mode_value = data->set_mode_value;
	touch_data->get_mode_value = data->get_mode_value;
	touch_data->reset_mode = data->reset_mode;
	touch_data->get_mode_all = data->get_mode_all;
	touch_data->palm_sensor_read = data->palm_sensor_read;
	touch_data->palm_sensor_write = data->palm_sensor_write;
	touch_data->priv = data->priv;
	mutex_unlock(&pdata->touch_dev.mutex);

	srcu_read_unlock(&xmi_srcu, idx);
	return 0;
}
EXPORT_SYMBOL_GPL(xiaomitouch_register_modedata);

void xiaomitouch_unregister_modedata(void)
{
	struct xiaomi_touch_pdata *pdata;
	int idx;

	idx = srcu_read_lock(&xmi_srcu);
	pdata = srcu_dereference(g_xmi_pdata, &xmi_srcu);
	if (!pdata) {
		srcu_read_unlock(&xmi_srcu, idx);
		return;
	}

	mutex_lock(&pdata->touch_dev.mutex);
	memset(pdata->touch_data, 0,
	       sizeof(struct xiaomi_touch_interface));
	mutex_unlock(&pdata->touch_dev.mutex);

	srcu_read_unlock(&xmi_srcu, idx);
}
EXPORT_SYMBOL_GPL(xiaomitouch_unregister_modedata);

int update_palm_sensor_value(int value)
{
	struct xiaomi_touch_pdata *pdata;
	int idx;

	idx = srcu_read_lock(&xmi_srcu);
	pdata = srcu_dereference(g_xmi_pdata, &xmi_srcu);
	if (!pdata) {
		srcu_read_unlock(&xmi_srcu, idx);
		return -ENODEV;
	}

	mutex_lock(&pdata->touch_dev.palm_mutex);
	if (value != pdata->palm_value) {
		pr_info("%s: value: %d\n", __func__, value);
		pdata->palm_value = value;
		pdata->palm_changed = true;
		sysfs_notify(&pdata->touch_dev.dev->kobj,
			     NULL, "palm_sensor");
	}
	mutex_unlock(&pdata->touch_dev.palm_mutex);

	srcu_read_unlock(&xmi_srcu, idx);
	return 0;
}
EXPORT_SYMBOL_GPL(update_palm_sensor_value);

static ssize_t palm_sensor_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct xiaomi_touch_pdata *pdata = dev_get_drvdata(dev);

	pdata->palm_changed = false;
	return sysfs_emit(buf, "%d\n", pdata->palm_value);
}

static ssize_t palm_sensor_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct xiaomi_touch_pdata *pdata = dev_get_drvdata(dev);
	unsigned int input;

	if (kstrtouint(buf, 0, &input) < 0)
		return -EINVAL;

	mutex_lock(&pdata->touch_dev.mutex);
	if (pdata->touch_data && pdata->touch_data->palm_sensor_write)
		pdata->touch_data->palm_sensor_write(!!input);
	else
		pr_err("%s: not implemented\n", __func__);
	mutex_unlock(&pdata->touch_dev.mutex);

	pr_info("%s: value: %d\n", __func__, !!input);
	return count;
}
static DEVICE_ATTR_RW(palm_sensor);

static struct attribute *touch_attr_group[] = {
	&dev_attr_palm_sensor.attr,
	NULL,
};

static int xiaomi_touch_parse_dt(struct device *dev,
				 struct xiaomi_touch_pdata *pdata)
{
	struct device_node *np = dev->of_node;
	int ret;

	if (!np)
		return -ENODEV;

	ret = of_property_read_string(np, "touch,name", &pdata->name);
	if (ret)
		return ret;

	pr_info("%s: touch,name: %s\n", __func__, pdata->name);
	return 0;
}

static int xiaomi_touch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xiaomi_touch_pdata *pdata;
	struct xiaomi_touch *touch_dev;
	int ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	touch_dev = &pdata->touch_dev;
	mutex_init(&touch_dev->mutex);
	mutex_init(&touch_dev->palm_mutex);
	mutex_init(&touch_dev->psensor_mutex);
	init_waitqueue_head(&touch_dev->wait_queue);

	ret = xiaomi_touch_parse_dt(dev, pdata);
	if (ret < 0) {
		pr_err("%s: parse dt error: %d\n", __func__, ret);
		goto err_mutex;
	}

	touch_dev->class = class_create(THIS_MODULE, "touch");
	if (!touch_dev->class) {
		pr_err("%s: create device class err\n", __func__);
		ret = -ENOMEM;
		goto err_mutex;
	}

	touch_dev->dev = device_create(touch_dev->class, NULL, 'T',
				       NULL, "touch_dev");
	if (!touch_dev->dev) {
		pr_err("%s: create device dev err\n", __func__);
		ret = -ENOMEM;
		goto err_class;
	}

	pdata->touch_data = kzalloc(sizeof(struct xiaomi_touch_interface),
				    GFP_KERNEL);
	if (!pdata->touch_data) {
		ret = -ENOMEM;
		pr_err("%s: alloc mem for touch_data\n", __func__);
		goto err_device;
	}

	dev_set_drvdata(touch_dev->dev, pdata);
	platform_set_drvdata(pdev, pdata);

	touch_dev->attrs.attrs = touch_attr_group;
	ret = sysfs_create_group(&touch_dev->dev->kobj, &touch_dev->attrs);
	if (ret) {
		pr_err("%s: cannot create sysfs: %d\n", __func__, ret);
		goto err_sysfs;
	}

	mutex_lock(&xmi_pdata_lock);
	rcu_assign_pointer(g_xmi_pdata, pdata);
	mutex_unlock(&xmi_pdata_lock);

	touch_dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	touch_dev->misc_dev.name = "xiaomi-touch";
	touch_dev->misc_dev.fops = &xiaomitouch_dev_fops;
	touch_dev->misc_dev.parent = NULL;

	ret = misc_register(&touch_dev->misc_dev);
	if (ret) {
		pr_err("%s: create misc device err: %d\n", __func__, ret);
		goto err_misc;
	}

	pr_info("%s: over\n", __func__);
	return 0;

err_misc:
	mutex_lock(&xmi_pdata_lock);
	rcu_assign_pointer(g_xmi_pdata, NULL);
	mutex_unlock(&xmi_pdata_lock);
	synchronize_srcu(&xmi_srcu);
	sysfs_remove_group(&touch_dev->dev->kobj, &touch_dev->attrs);
err_sysfs:
	kfree(pdata->touch_data);
err_device:
	device_destroy(touch_dev->class, touch_dev->dev->devt);
err_class:
	class_destroy(touch_dev->class);
err_mutex:
	mutex_destroy(&touch_dev->mutex);
	mutex_destroy(&touch_dev->palm_mutex);
	mutex_destroy(&touch_dev->psensor_mutex);
	pr_err("%s: fail!\n", __func__);
	return ret;
}

static int xiaomi_touch_remove(struct platform_device *pdev)
{
	struct xiaomi_touch_pdata *pdata = platform_get_drvdata(pdev);
	struct xiaomi_touch *touch_dev;

	if (!pdata)
		return 0;

	touch_dev = &pdata->touch_dev;

	misc_deregister(&touch_dev->misc_dev);

	mutex_lock(&xmi_pdata_lock);
	rcu_assign_pointer(g_xmi_pdata, NULL);
	mutex_unlock(&xmi_pdata_lock);

	synchronize_srcu(&xmi_srcu);

	if (touch_dev->dev && touch_dev->attrs.attrs)
		sysfs_remove_group(&touch_dev->dev->kobj, &touch_dev->attrs);

	if (touch_dev->dev)
		device_destroy(touch_dev->class, touch_dev->dev->devt);

	if (touch_dev->class)
		class_destroy(touch_dev->class);

	kfree(pdata->touch_data);

	mutex_destroy(&touch_dev->mutex);
	mutex_destroy(&touch_dev->palm_mutex);
	mutex_destroy(&touch_dev->psensor_mutex);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id xiaomi_touch_of_match[] = {
	{ .compatible = "xiaomi-touch", },
	{ },
};
MODULE_DEVICE_TABLE(of, xiaomi_touch_of_match);

static struct platform_driver xiaomi_touch_device_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "xiaomi-touch",
		.of_match_table	= of_match_ptr(xiaomi_touch_of_match),
	},
	.probe	= xiaomi_touch_probe,
	.remove	= xiaomi_touch_remove,
};

static int __init xiaomi_touch_init(void)
{
	return platform_driver_register(&xiaomi_touch_device_driver);
}

static void __exit xiaomi_touch_exit(void)
{
	platform_driver_unregister(&xiaomi_touch_device_driver);
}

subsys_initcall(xiaomi_touch_init);
module_exit(xiaomi_touch_exit);

MODULE_DESCRIPTION("Xiaomi Touch Feature Driver");
MODULE_LICENSE("GPL");
