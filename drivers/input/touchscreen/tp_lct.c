// SPDX-License-Identifier: GPL-2.0-only
/*
 * Centralized Longcheer/Xiaomi touchpanel procfs core.
 *
 * Follows the ops + device_register pattern (similar to tcpc): touch drivers
 * register a const struct tp_lct_ops with their drv_data, and the procfs
 * nodes are created for every operation, unsupported ones return -EOPNOTSUPP.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/input/tp_lct.h>

#define TP_LCT_SELFTEST_RETRIES		3
#define TP_LCT_SELFTEST_UNKNOWN		"0\n"
#define TP_LCT_SELFTEST_FAIL		"1\n"
#define TP_LCT_SELFTEST_PASS		"2\n"

enum tp_lct_toggle {
	TP_LCT_GESTURE = 0,
	TP_LCT_AOD,
	TP_LCT_PALM,
	TP_LCT_WORK,
	TP_LCT_TOGGLE_MAX,
};

struct tp_lct_toggle_node {
	struct tp_lct_device *lct;
	const char *name;
	struct proc_dir_entry *proc;
	bool enabled;
};

struct tp_lct_device {
	struct device *dev;
	const struct tp_lct_ops *ops;
	void *drv_data;
	struct mutex lock; /* leaf lock: never held across ops callbacks */

	struct tp_lct_toggle_node toggles[TP_LCT_TOGGLE_MAX];
	struct proc_dir_entry *grip_proc;
	struct proc_dir_entry *info_proc;
	struct proc_dir_entry *lockdown_proc;
	struct proc_dir_entry *selftest_proc;

	int screen_angle;
	char selftest_status[16];
	bool in_selftest;
};

static const char * const tp_lct_toggle_names[TP_LCT_TOGGLE_MAX] = {
	[TP_LCT_GESTURE] = "tp_gesture",
	[TP_LCT_AOD]     = "tp_aod",
	[TP_LCT_PALM]    = "tp_palm",
	[TP_LCT_WORK]    = "tp_work",
};

static int (*tp_lct_get_toggle_set_cb(const struct tp_lct_ops *ops,
				      enum tp_lct_toggle id))(struct tp_lct_device *, bool)
{
	switch (id) {
	case TP_LCT_GESTURE:
		return ops->set_gesture;
	case TP_LCT_AOD:
		return ops->set_aod;
	case TP_LCT_PALM:
		return ops->set_palm;
	case TP_LCT_WORK:
		return ops->set_work;
	default:
		return NULL;
	}
}

static int (*tp_lct_get_toggle_get_cb(const struct tp_lct_ops *ops,
				      enum tp_lct_toggle id))(struct tp_lct_device *)
{
	switch (id) {
	case TP_LCT_GESTURE:
		return ops->get_gesture;
	case TP_LCT_AOD:
		return ops->get_aod;
	case TP_LCT_PALM:
		return ops->get_palm;
	case TP_LCT_WORK:
		return ops->get_work;
	default:
		return NULL;
	}
}

void *tp_lct_get_drvdata(struct tp_lct_device *lct)
{
	return lct ? lct->drv_data : NULL;
}
EXPORT_SYMBOL_GPL(tp_lct_get_drvdata);

/* --- Boolean toggle proc ops (gesture, aod, palm, work) --- */

static ssize_t tp_lct_toggle_read(struct file *file, char __user *buf,
				  size_t size, loff_t *ppos)
{
	struct tp_lct_toggle_node *node = PDE_DATA(file_inode(file));
	struct tp_lct_device *lct;
	int (*get_cb)(struct tp_lct_device *lct);
	enum tp_lct_toggle id;
	char out[4];
	bool en;
	int ret;

	if (!node || !node->lct)
		return -ENODEV;

	lct = node->lct;
	id = (enum tp_lct_toggle)(node - lct->toggles);
	get_cb = tp_lct_get_toggle_get_cb(lct->ops, id);

	if (get_cb) {
		ret = get_cb(lct);
		if (ret < 0)
			return ret;
		en = ret > 0;
		mutex_lock(&lct->lock);
		node->enabled = en;
		mutex_unlock(&lct->lock);
	} else if (tp_lct_get_toggle_set_cb(lct->ops, id)) {
		mutex_lock(&lct->lock);
		en = node->enabled;
		mutex_unlock(&lct->lock);
	} else {
		return -EOPNOTSUPP;
	}

	out[0] = en ? '1' : '0';
	out[1] = '\n';
	return simple_read_from_buffer(buf, size, ppos, out, 2);
}

static ssize_t tp_lct_toggle_write(struct file *file, const char __user *buf,
				   size_t size, loff_t *ppos)
{
	struct tp_lct_toggle_node *node = PDE_DATA(file_inode(file));
	struct tp_lct_device *lct;
	int (*cb)(struct tp_lct_device *lct, bool enable);
	enum tp_lct_toggle id;
	char kbuf[16];
	unsigned int val;
	bool enable;
	int ret;

	if (!node || !node->lct)
		return -ENODEV;

	lct = node->lct;
	id = (enum tp_lct_toggle)(node - lct->toggles);
	cb = tp_lct_get_toggle_set_cb(lct->ops, id);
	if (!cb)
		return -EOPNOTSUPP;

	if (size == 0 || size >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, buf, size))
		return -EFAULT;

	kbuf[size] = '\0';
	ret = kstrtouint(strim(kbuf), 10, &val);
	if (ret)
		return ret;

	enable = val > 0;
	ret = cb(lct, enable);
	if (ret)
		return ret;

	mutex_lock(&lct->lock);
	node->enabled = enable;
	mutex_unlock(&lct->lock);

	return size;
}

static const struct file_operations tp_lct_toggle_fops = {
	.read	= tp_lct_toggle_read,
	.write	= tp_lct_toggle_write,
	.llseek	= default_llseek,
};

/* --- Grip area (/proc/tp_grip_area) --- */

static ssize_t tp_lct_grip_read(struct file *file, char __user *buf,
				size_t size, loff_t *ppos)
{
	struct tp_lct_device *lct = PDE_DATA(file_inode(file));
	char out[16];
	int angle, len;

	if (!lct)
		return -ENODEV;

	if (lct->ops->get_grip_angle) {
		angle = lct->ops->get_grip_angle(lct);
		if (angle < 0)
			return angle;
		mutex_lock(&lct->lock);
		lct->screen_angle = angle;
		mutex_unlock(&lct->lock);
	} else if (lct->ops->set_grip_angle) {
		mutex_lock(&lct->lock);
		angle = lct->screen_angle;
		mutex_unlock(&lct->lock);
	} else {
		return -EOPNOTSUPP;
	}

	len = scnprintf(out, sizeof(out), "%d\n", angle);
	return simple_read_from_buffer(buf, size, ppos, out, len);
}

static ssize_t tp_lct_grip_write(struct file *file, const char __user *buf,
				 size_t size, loff_t *ppos)
{
	struct tp_lct_device *lct = PDE_DATA(file_inode(file));
	char kbuf[16];
	int angle, ret;

	if (!lct || !lct->ops->set_grip_angle)
		return -EOPNOTSUPP;

	if (size == 0 || size >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, buf, size))
		return -EFAULT;

	kbuf[size] = '\0';
	ret = kstrtoint(strim(kbuf), 10, &angle);
	if (ret)
		return ret;

	ret = lct->ops->set_grip_angle(lct, angle);
	if (ret < 0)
		return ret;

	mutex_lock(&lct->lock);
	lct->screen_angle = angle;
	mutex_unlock(&lct->lock);

	return size;
}

static const struct file_operations tp_lct_grip_fops = {
	.read	= tp_lct_grip_read,
	.write	= tp_lct_grip_write,
	.llseek	= default_llseek,
};

/* --- Info & lockdown (/proc/tp_info, /proc/tp_lockdown_info) --- */

static ssize_t tp_lct_info_read(struct file *file, char __user *buf,
				size_t size, loff_t *ppos)
{
	struct tp_lct_device *lct = PDE_DATA(file_inode(file));
	char out[TP_LCT_INFO_MAX_LEN];
	int ret;

	if (!lct)
		return -ENODEV;

	if (!lct->ops->get_info)
		return -EOPNOTSUPP;

	ret = lct->ops->get_info(lct, out, sizeof(out));
	if (ret < 0)
		return ret;

	return simple_read_from_buffer(buf, size, ppos, out,
				       strnlen(out, sizeof(out)));
}

static ssize_t tp_lct_lockdown_read(struct file *file, char __user *buf,
				    size_t size, loff_t *ppos)
{
	struct tp_lct_device *lct = PDE_DATA(file_inode(file));
	char out[TP_LCT_INFO_MAX_LEN];
	int ret;

	if (!lct)
		return -ENODEV;

	if (!lct->ops->get_lockdown_info)
		return -EOPNOTSUPP;

	ret = lct->ops->get_lockdown_info(lct, out, sizeof(out));
	if (ret < 0)
		return ret;

	return simple_read_from_buffer(buf, size, ppos, out,
				       strnlen(out, sizeof(out)));
}

static const struct file_operations tp_lct_info_fops = {
	.read	= tp_lct_info_read,
	.llseek	= default_llseek,
};

static const struct file_operations tp_lct_lockdown_fops = {
	.read	= tp_lct_lockdown_read,
	.llseek	= default_llseek,
};

/* --- Selftest (/proc/tp_selftest) --- */

static void tp_lct_run_selftest(struct tp_lct_device *lct, u8 cmd)
{
	int i, res;

	for (i = 0; i < TP_LCT_SELFTEST_RETRIES; i++) {
		const char *status;

		res = lct->ops->selftest(lct, cmd);
		if (res == 2)
			status = TP_LCT_SELFTEST_PASS;
		else if (res == 1)
			status = TP_LCT_SELFTEST_FAIL;
		else
			status = TP_LCT_SELFTEST_UNKNOWN;

		mutex_lock(&lct->lock);
		strscpy(lct->selftest_status, status,
			sizeof(lct->selftest_status));
		mutex_unlock(&lct->lock);

		if (res == 2)
			return;
	}
}

static ssize_t tp_lct_selftest_read(struct file *file, char __user *buf,
				    size_t size, loff_t *ppos)
{
	struct tp_lct_device *lct = PDE_DATA(file_inode(file));
	char out[16];
	int len;

	if (!lct)
		return -ENODEV;

	if (!lct->ops->selftest)
		return -EOPNOTSUPP;

	mutex_lock(&lct->lock);
	len = scnprintf(out, sizeof(out), "%s", lct->selftest_status);
	mutex_unlock(&lct->lock);

	return simple_read_from_buffer(buf, size, ppos, out, len);
}

static ssize_t tp_lct_selftest_write(struct file *file, const char __user *buf,
				     size_t size, loff_t *ppos)
{
	struct tp_lct_device *lct = PDE_DATA(file_inode(file));
	char kbuf[32];
	const char *cmd_str;
	u8 cmd;

	if (!lct)
		return -ENODEV;

	if (!lct->ops->selftest)
		return -EOPNOTSUPP;

	if (size == 0 || size >= sizeof(kbuf))
		return -EINVAL;

	if (copy_from_user(kbuf, buf, size))
		return -EFAULT;

	kbuf[size] = '\0';
	cmd_str = strim(kbuf);

	if (!strcmp(cmd_str, "mmi"))
		cmd = TP_LCT_SELFTEST_MMI;
	else if (!strcmp(cmd_str, "i2c"))
		cmd = TP_LCT_SELFTEST_I2C;
	else if (!strcmp(cmd_str, "open"))
		cmd = TP_LCT_SELFTEST_OPEN;
	else if (!strcmp(cmd_str, "short"))
		cmd = TP_LCT_SELFTEST_SHORT;
	else
		return -EINVAL;

	mutex_lock(&lct->lock);
	if (lct->in_selftest) {
		mutex_unlock(&lct->lock);
		return -EBUSY;
	}
	lct->in_selftest = true;
	mutex_unlock(&lct->lock);

	tp_lct_run_selftest(lct, cmd);

	mutex_lock(&lct->lock);
	lct->in_selftest = false;
	mutex_unlock(&lct->lock);

	return size;
}

static const struct file_operations tp_lct_selftest_fops = {
	.read	= tp_lct_selftest_read,
	.write	= tp_lct_selftest_write,
	.llseek	= default_llseek,
};

/* --- Registration / lifecycle --- */

static struct proc_dir_entry *tp_lct_create_proc(struct tp_lct_device *lct,
						 const char *name, umode_t mode,
						 const struct file_operations *fops,
						 void *data)
{
	struct proc_dir_entry *proc;

	proc = proc_create_data(name, mode, NULL, fops, data);
	if (!proc && lct->dev)
		dev_warn(lct->dev, "failed to create /proc/%s\n", name);

	return proc;
}

struct tp_lct_device *tp_lct_device_register(struct device *parent,
					     const struct tp_lct_ops *ops,
					     void *drv_data)
{
	struct tp_lct_device *lct;
	int i;

	if (!ops)
		return ERR_PTR(-EINVAL);

	lct = kzalloc(sizeof(*lct), GFP_KERNEL);
	if (!lct)
		return ERR_PTR(-ENOMEM);

	lct->dev = parent;
	lct->ops = ops;
	lct->drv_data = drv_data;
	mutex_init(&lct->lock);
	strscpy(lct->selftest_status, TP_LCT_SELFTEST_UNKNOWN,
		sizeof(lct->selftest_status));

	for (i = 0; i < TP_LCT_TOGGLE_MAX; i++) {
		lct->toggles[i].lct = lct;
		lct->toggles[i].name = tp_lct_toggle_names[i];
		lct->toggles[i].proc = tp_lct_create_proc(lct,
							  tp_lct_toggle_names[i],
							  0644,
							  &tp_lct_toggle_fops,
							  &lct->toggles[i]);
		if (!lct->toggles[i].proc)
			goto err_remove_toggles;
	}

	lct->grip_proc = tp_lct_create_proc(lct, "tp_grip_area", 0644,
					    &tp_lct_grip_fops, lct);
	if (!lct->grip_proc)
		goto err_remove_toggles;

	lct->info_proc = tp_lct_create_proc(lct, "tp_info", 0444,
					    &tp_lct_info_fops, lct);
	if (!lct->info_proc)
		goto err_remove_grip;

	lct->lockdown_proc = tp_lct_create_proc(lct, "tp_lockdown_info", 0444,
						&tp_lct_lockdown_fops, lct);
	if (!lct->lockdown_proc)
		goto err_remove_info;

	lct->selftest_proc = tp_lct_create_proc(lct, "tp_selftest", 0644,
						&tp_lct_selftest_fops, lct);
	if (!lct->selftest_proc)
		goto err_remove_lockdown;

	return lct;

err_remove_lockdown:
	remove_proc_entry("tp_lockdown_info", NULL);
err_remove_info:
	remove_proc_entry("tp_info", NULL);
err_remove_grip:
	remove_proc_entry("tp_grip_area", NULL);
err_remove_toggles:
	while (--i >= 0)
		remove_proc_entry(lct->toggles[i].name, NULL);
	mutex_destroy(&lct->lock);
	kfree(lct);
	/*
	 * proc_create_data() returns NULL both on allocation failure and when
	 * the name is already taken; a clash with another panel is the
	 * realistic case for a global /proc namespace.
	 */
	return ERR_PTR(-EEXIST);
}
EXPORT_SYMBOL_GPL(tp_lct_device_register);

void tp_lct_device_unregister(struct tp_lct_device *lct)
{
	int i;

	if (IS_ERR_OR_NULL(lct))
		return;

	if (lct->selftest_proc)
		remove_proc_entry("tp_selftest", NULL);
	if (lct->lockdown_proc)
		remove_proc_entry("tp_lockdown_info", NULL);
	if (lct->info_proc)
		remove_proc_entry("tp_info", NULL);
	if (lct->grip_proc)
		remove_proc_entry("tp_grip_area", NULL);

	for (i = 0; i < TP_LCT_TOGGLE_MAX; i++) {
		if (lct->toggles[i].proc)
			remove_proc_entry(lct->toggles[i].name, NULL);
	}

	mutex_destroy(&lct->lock);
	kfree(lct);
}
EXPORT_SYMBOL_GPL(tp_lct_device_unregister);

MODULE_DESCRIPTION("Centralized Touchpanel LCT Procfs Core");
MODULE_LICENSE("GPL");
