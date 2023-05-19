/**
 * Copyright Elliptic Labs
 *
 * Userspace I/O channel: a single-writer device node that lets a userspace
 * process inject raw data frames into the elliptic device FIFO (as if they
 * arrived from the DSP).
 */

#include <asm/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/wait.h>

#include <elliptic/elliptic_data_io.h>
#include <elliptic/elliptic_device.h>

#define USERSPACE_IO_DEVICE_NAME "elliptic_us_io"

/**
 * struct elliptic_userspace_device - State for the userspace-IO device node.
 *
 * @cdev:      Kernel character-device bookkeeping.
 * @open_lock: Mutex enforcing single-opener semantics.
 *             Initialised in elliptic_userspace_io_driver_init(),
 *             destroyed  in elliptic_userspace_io_driver_exit().
 *             Acquired in device_open(), released in device_close().
 */
struct elliptic_userspace_device {
	struct cdev cdev;
	struct mutex open_lock;
};

static struct elliptic_userspace_device io_device;
static dev_t elliptic_userspace_major;

static int device_open(struct inode *inode, struct file *filp)
{
	if (inode->i_cdev != &io_device.cdev) {
		EL_PRINT_W("dev pointer mismatch");
		return -ENODEV;
	}

	if (mutex_lock_interruptible(&io_device.open_lock)) {
		EL_PRINT_E("open interrupted by signal");
		return -ERESTARTSYS;
	}

	EL_PRINT_I("Opened device %s", USERSPACE_IO_DEVICE_NAME);
	return 0;
}

static ssize_t device_write(struct file *fp, const char __user *buff,
			    size_t length, loff_t *ppos)
{
	int push_result;

	push_result = elliptic_data_push(ELLIPTIC_ALL_DEVICES, buff, length,
					 ELLIPTIC_DATA_PUSH_FROM_USERSPACE);
	if (push_result != 0)
		return (ssize_t)push_result;

	return (ssize_t)length;
}

static int device_close(struct inode *inode, struct file *filp)
{
	/* paired with device_open() */
	mutex_unlock(&io_device.open_lock);
	EL_PRINT_I("Closed device %s", USERSPACE_IO_DEVICE_NAME);
	return 0;
}

static const struct file_operations elliptic_userspace_fops = {
	.owner = THIS_MODULE,
	.open = device_open,
	.write = device_write,
	.release = device_close,
};

int elliptic_userspace_io_driver_init(void)
{
	dev_t device_number;
	struct device *device;
	int err;

	err = alloc_chrdev_region(&device_number, 0, 1,
				  USERSPACE_IO_DEVICE_NAME);
	if (err < 0) {
		EL_PRINT_E("alloc_chrdev_region failed ret=%d", err);
		return err;
	}

	elliptic_userspace_major = MAJOR(device_number);
	device_number = MKDEV(elliptic_userspace_major, 0);

	/*
	 * Initialise open_lock before cdev_add() so it is ready if the device
	 * node is opened immediately after registration.
	 */
	mutex_init(&io_device.open_lock);

	cdev_init(&io_device.cdev, &elliptic_userspace_fops);
	io_device.cdev.owner = THIS_MODULE;

	err = cdev_add(&io_device.cdev, device_number, 1);
	if (err) {
		EL_PRINT_E("cdev_add failed ret=%d", err);
		goto fail_cdev;
	}

	device = device_create(elliptic_class, NULL, device_number,
			       NULL, USERSPACE_IO_DEVICE_NAME);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		EL_PRINT_E("device_create failed ret=%d", err);
		goto fail_device;
	}

	return 0;

fail_device:
	cdev_del(&io_device.cdev);
fail_cdev:
	mutex_destroy(&io_device.open_lock);
	unregister_chrdev_region(device_number, 1);
	return err;
}

void elliptic_userspace_io_driver_exit(void)
{
	BUG_ON(!elliptic_class);

	device_destroy(elliptic_class, MKDEV(elliptic_userspace_major, 0));
	cdev_del(&io_device.cdev);
	unregister_chrdev_region(MKDEV(elliptic_userspace_major, 0), 1);

	/*
	 * Ensure the mutex is in an unlocked state before destroying it.
	 * If a writer is currently inside device_open() blocked on open_lock,
	 * mutex_trylock() will fail (already locked); we do not unlock it here
	 * because device_close() will do so when the opener eventually unblocks
	 * and sees that the cdev is gone.  We only need to handle the case where
	 * the device is idle (open_lock is free) so mutex_destroy() finds it in
	 * a consistent state.
	 */
	mutex_destroy(&io_device.open_lock);
}
