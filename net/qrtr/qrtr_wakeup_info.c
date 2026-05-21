/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/qrtr.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/qrtr_wakeup_info.h>
#include "qrtr.h"
#include <asm/arch_timer.h>

#define MAX_WAKEUP_ENTRY_ALLOWED 256
#define QSOCKET_MIN_SVC_ID 4096
#define QSOCKET_MAX_SVC_ID 8192
#define QRTR_WAKEUP_INFO_MINOR 0
#define QRTR_WAKEUP_DEV_MAX 1

struct qmi_header {
	u8 msg_type;
	u16 txn_id;
	u8 msg_id;
} __packed;

struct qrtr_wakeup_driver {
	struct cdev *cdev;
	struct device *dev;
	dev_t qrtr_wake_major;
	struct class *qrtr_wake_class;
	wait_queue_head_t readq;
	bool read_data;
	spinlock_t read_data_lock;      /* lock to protect read_data */
	struct qrtr_wakeup_info *info;
} wake_drv;

static unsigned int max_wakeup_entry = 1;
static spinlock_t max_entry_lock;	/* lock to protect max_wakeup_entry */

static ssize_t qrtr_max_wk_entry_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", max_wakeup_entry);
}

static ssize_t qrtr_max_wk_entry_store(struct device *dev, struct device_attribute *attr,
				       const char *buf, size_t count)
{
	unsigned int max_wk, max_wakeup_entry_old;
	struct qrtr_wakeup_info *info;
	unsigned long flags;
	int ret, i;

	ret = kstrtoint(buf, 10, &max_wk);
	if (ret < 0) {
		pr_err("failed to write input, error %d\n", ret);
		return 0;
	}
	if (max_wk > MAX_WAKEUP_ENTRY_ALLOWED || !max_wk)
		return -EINVAL;

	spin_lock_irqsave(&max_entry_lock, flags);
	if (max_wk > max_wakeup_entry) {
		info = krealloc(wake_drv.info, sizeof(struct qrtr_wakeup_info) * max_wk,
				GFP_KERNEL);
		if (!info) {
			spin_unlock_irqrestore(&max_entry_lock, flags);
			return -ENOMEM;
		}
		wake_drv.info = info;
		max_wakeup_entry_old = max_wakeup_entry;
		for (i = max_wakeup_entry_old; i < max_wk; i++) {
			memset(&wake_drv.info[i], 0, sizeof(struct qrtr_wakeup_info));
			wake_drv.info[i].version = INFO_VERSION_1;
			wake_drv.info[i].svc_id = -1;
			wake_drv.info[i].msg_id = -1;
			wake_drv.info[i].msg_type = -1;
			wake_drv.info[i].client_pid = -1;
		}
	}
	max_wakeup_entry = max_wk;
	spin_unlock_irqrestore(&max_entry_lock, flags);

	return count;
}

static DEVICE_ATTR_RW(qrtr_max_wk_entry);

static ssize_t qrtr_wakeup_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	size_t to_copy, data_size = sizeof(struct qrtr_wakeup_info) * max_wakeup_entry;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;
	spin_lock_irq(&wake_drv.read_data_lock);
	wake_drv.read_data = false;
	spin_unlock_irq(&wake_drv.read_data_lock);
	to_copy = min_t(size_t, count, data_size);
	if (copy_to_user(buf, wake_drv.info, to_copy))
		return -EFAULT;

	return to_copy;
}

static __poll_t qrtr_wakeup_poll(struct file *file, poll_table *wait)
{
	unsigned long flags;
	__poll_t mask = 0;

	poll_wait(file, &wake_drv.readq, wait);
	spin_lock_irqsave(&wake_drv.read_data_lock, flags);
	if (wake_drv.read_data)
		mask |= POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&wake_drv.read_data_lock, flags);

	return mask;
}

static const struct file_operations qrtr_wakeup_fops = {
	.owner = THIS_MODULE,
	.read = qrtr_wakeup_read,
	.poll = qrtr_wakeup_poll,
};

void qrtr_save_wakeup_reason(u64 pl, struct qrtr_cb cb, int pid, char *name, int service_id)
{
	struct qrtr_wakeup_info *info;
	u32 buf = (unsigned int)pl;
	struct qmi_header *hdr;
	unsigned long flags;
	static int index;

	spin_lock_irqsave(&max_entry_lock, flags);
	if (index >= max_wakeup_entry)
		index = 0;
	spin_unlock_irqrestore(&max_entry_lock, flags);

	info = &wake_drv.info[index];

	info->version = INFO_VERSION_1;
	info->svc_id = service_id < 0 ? -1 : service_id;

	if ((info->svc_id >= QSOCKET_MIN_SVC_ID && info->svc_id <= QSOCKET_MAX_SVC_ID) ||
	    cb.type != QRTR_TYPE_DATA) {
		info->msg_type = -1;
		info->msg_id = -1;
	} else {
		hdr = (struct qmi_header *)&buf;
		info->msg_type = hdr->msg_type;
		info->msg_id = hdr->msg_id;
	}

	info->client_pid = pid;
	memcpy(info->client_name, name, TASK_COMM_LEN);

	info->src_node = cb.src_node;
	info->dst_node = cb.dst_node;
	info->timestamp = ktime_to_us(ktime_get());
	info->qtimer = arch_counter_get_cntvct();

	index++;

	spin_lock_irqsave(&wake_drv.read_data_lock, flags);
	wake_drv.read_data = true;
	spin_unlock_irqrestore(&wake_drv.read_data_lock, flags);
	wake_up_interruptible(&wake_drv.readq);
}
EXPORT_SYMBOL_GPL(qrtr_save_wakeup_reason);

int qrtr_wakeup_info_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&wake_drv.qrtr_wake_major, QRTR_WAKEUP_INFO_MINOR,
				  QRTR_WAKEUP_DEV_MAX, "qrtr-wakeup-info");
	if (ret < 0) {
		pr_err("%s: alloc_chrdev_region failed ret:%d\n", __func__, ret);
		return -ENOMEM;
	}
	wake_drv.cdev = cdev_alloc();
	wake_drv.cdev->ops = &qrtr_wakeup_fops;
	cdev_add(wake_drv.cdev, wake_drv.qrtr_wake_major, 1);
	wake_drv.qrtr_wake_class = class_create(THIS_MODULE,"qrtr_wakeup_class");
	if (IS_ERR(wake_drv.qrtr_wake_class)) {
		ret = PTR_ERR(wake_drv.qrtr_wake_class);
		goto fail_class;
	}
	wake_drv.dev = device_create(wake_drv.qrtr_wake_class, NULL, wake_drv.qrtr_wake_major, NULL,
				     "qrtr-wakeup-info");
	if (IS_ERR(wake_drv.dev)) {
		ret = PTR_ERR(wake_drv.dev);
		goto fail_dev_create;
	}

	init_waitqueue_head(&wake_drv.readq);
	spin_lock_init(&wake_drv.read_data_lock);
	spin_lock_init(&max_entry_lock);

	ret = device_create_file(wake_drv.dev, &dev_attr_qrtr_max_wk_entry);
	if (ret < 0) {
		pr_err("%s: Failed to create sysfs entry %d\n", __func__, ret);
		device_destroy(wake_drv.qrtr_wake_class, wake_drv.qrtr_wake_major);
		goto fail_dev_create;
	}

	wake_drv.info = kzalloc(sizeof(*wake_drv.info), GFP_KERNEL);
	if (!wake_drv.info) {
		device_destroy(wake_drv.qrtr_wake_class, wake_drv.qrtr_wake_major);
		goto fail_dev_create;
	}
	wake_drv.info[0].version = INFO_VERSION_1;
	wake_drv.info[0].svc_id = -1;
	wake_drv.info[0].msg_id = -1;
	wake_drv.info[0].msg_type = -1;
	wake_drv.info[0].client_pid = -1;

	pr_info("QRTR wakeup info driver init done\n");
	return 0;

fail_dev_create:
	class_destroy(wake_drv.qrtr_wake_class);
fail_class:
	cdev_del(wake_drv.cdev);
	unregister_chrdev_region(wake_drv.qrtr_wake_major, 1);
	return ret;
}

void qrtr_wakeup_info_exit(void)
{
	kfree(wake_drv.info);
	device_destroy(wake_drv.qrtr_wake_class, wake_drv.qrtr_wake_major);
	class_destroy(wake_drv.qrtr_wake_class);
	cdev_del(wake_drv.cdev);
	unregister_chrdev_region(wake_drv.qrtr_wake_major, 1);
}

MODULE_DESCRIPTION("QRTR wakeup info driver");
MODULE_LICENSE("GPL");
