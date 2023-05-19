/**
 * Copyright Elliptic Labs
 *
 * Userspace control channel: a single-reader device node that delivers
 * AP→DSP parameter writes to a userspace daemon for inspection/logging.
 * A ping-pong buffer avoids holding the lock across copy_to_user().
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

#define USERSPACE_CTRL_IO_DEVICE_NAME "elliptic_us_ctrl_io"

/**
 * struct elliptic_userspace_ctrl_device - State for the ctrl device node.
 *
 * @cdev:             Kernel character-device bookkeeping.
 * @open_lock:        Mutex enforcing single-opener semantics.
 *                    Initialised in elliptic_userspace_ctrl_driver_init(),
 *                    destroyed  in elliptic_userspace_ctrl_driver_exit().
 *                    Acquired in device_open(), released in device_close().
 * @ping_pong_idx:    Index of the buffer currently being read (ping).
 *                    The writer always writes to the opposite buffer (pong).
 * @ping_pong_buffer_size: Byte count of valid data in each half.
 * @ping_pong_buffer: Double-buffer storage; each half is ELLIPTIC_MSG_BUF_SIZE.
 * @data_available:   Wait queue signalled when new data lands in the pong buffer.
 * @data_lock:        Mutex protecting ping_pong_idx / sizes / pong buffer.
 *                    Initialised and destroyed alongside @open_lock.
 * @data_state:       0 = no new data; >0 = data ready; <0 = cancelled.
 */
struct elliptic_userspace_ctrl_device {
	struct cdev cdev;
	struct mutex open_lock;

	int ping_pong_idx;
	size_t ping_pong_buffer_size[2];
	uint8_t ping_pong_buffer[2][ELLIPTIC_MSG_BUF_SIZE];

	wait_queue_head_t data_available;
	struct mutex data_lock;
	atomic_t data_state;
};

static struct elliptic_userspace_ctrl_device ctrl_device;
static dev_t elliptic_userspace_ctrl_major;

/* Return the "read" (ping) buffer.  Caller must hold data_lock. */
static uint8_t *get_ping_buffer(struct elliptic_userspace_ctrl_device *dev,
				size_t *data_size)
{
	if (data_size)
		*data_size = dev->ping_pong_buffer_size[dev->ping_pong_idx];
	return dev->ping_pong_buffer[dev->ping_pong_idx];
}

/* Return the "write" (pong) buffer.  Caller must hold data_lock. */
static uint8_t *get_pong_buffer(struct elliptic_userspace_ctrl_device *dev,
				size_t *data_size)
{
	int pong = 1 - dev->ping_pong_idx;

	if (data_size)
		*data_size = dev->ping_pong_buffer_size[pong];
	return dev->ping_pong_buffer[pong];
}

/* Record how many bytes were written to the pong buffer. Caller holds data_lock. */
static void set_pong_buffer_size(struct elliptic_userspace_ctrl_device *dev,
				 size_t data_size)
{
	dev->ping_pong_buffer_size[1 - dev->ping_pong_idx] = data_size;
}

/* Swap ping and pong so the reader sees the freshly written data. Caller holds data_lock. */
static void swap_ping_pong(struct elliptic_userspace_ctrl_device *dev)
{
	dev->ping_pong_idx = 1 - dev->ping_pong_idx;
}

static int device_open(struct inode *inode, struct file *filp)
{
	if (inode->i_cdev != &ctrl_device.cdev) {
		EL_PRINT_E("dev pointer mismatch");
		return -ENODEV;
	}

	if (mutex_lock_interruptible(&ctrl_device.open_lock)) {
		EL_PRINT_E("open interrupted by signal");
		return -ERESTARTSYS;
	}

	EL_PRINT_I("Opened device %s", USERSPACE_CTRL_IO_DEVICE_NAME);
	return 0;
}

static ssize_t device_read(struct file *fp, char __user *buff,
			   size_t user_buf_length, loff_t *ppos)
{
	size_t bytes_read = 0;
	uint8_t *ping_buffer;
	int result;

	if (user_buf_length < ELLIPTIC_MSG_BUF_SIZE) {
		EL_PRINT_E("user buffer too small: %zu < %zu",
			   user_buf_length, (size_t)ELLIPTIC_MSG_BUF_SIZE);
		return -EINVAL;
	}

	result = wait_event_interruptible(ctrl_device.data_available,
					  atomic_read(&ctrl_device.data_state) != 0);
	if (result != 0) {
		if (result == -ERESTARTSYS)
			EL_PRINT_E("read interrupted");
		else
			EL_PRINT_E("wait_event error=%d", result);
		return result;
	}

	if (atomic_read(&ctrl_device.data_state) <= 0) {
		EL_PRINT_W("data_state=%d on wake-up, nothing to read",
			   atomic_read(&ctrl_device.data_state));
		atomic_set(&ctrl_device.data_state, 0);
		return 0;
	}

	result = mutex_lock_interruptible(&ctrl_device.data_lock);
	if (result) {
		EL_PRINT_E("data_lock interrupted ret=%d", result);
		return result;
	}

	swap_ping_pong(&ctrl_device);
	ping_buffer = get_ping_buffer(&ctrl_device, &bytes_read);

	if (bytes_read > user_buf_length) {
		EL_PRINT_E("ping buffer %zu > user buffer %zu",
			   bytes_read, user_buf_length);
		bytes_read = 0;
		goto out_unlock;
	}

	if (copy_to_user(buff, ping_buffer, bytes_read)) {
		EL_PRINT_E("copy_to_user failed");
		bytes_read = 0;
		goto out_unlock;
	}

	atomic_set(&ctrl_device.data_state, 0);

out_unlock:
	mutex_unlock(&ctrl_device.data_lock);
	return (ssize_t)bytes_read;
}

static int device_close(struct inode *inode, struct file *filp)
{
	/* paired with device_open() */
	mutex_unlock(&ctrl_device.open_lock);
	EL_PRINT_I("Closed device %s", USERSPACE_CTRL_IO_DEVICE_NAME);
	return 0;
}

static const struct file_operations elliptic_userspace_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = device_open,
	.read = device_read,
	.release = device_close,
};

int elliptic_userspace_ctrl_driver_init(void)
{
	dev_t device_number;
	struct device *device;
	int err;

	err = alloc_chrdev_region(&device_number, 0, 1,
				  USERSPACE_CTRL_IO_DEVICE_NAME);
	if (err < 0) {
		EL_PRINT_E("alloc_chrdev_region failed ret=%d", err);
		return err;
	}

	elliptic_userspace_ctrl_major = MAJOR(device_number);
	device_number = MKDEV(elliptic_userspace_ctrl_major, 0);

	/*
	 * Initialise both mutexes and the wait queue before cdev_add() so that
	 * they are ready if the device is opened immediately after registration.
	 */
	mutex_init(&ctrl_device.open_lock);
	mutex_init(&ctrl_device.data_lock);
	init_waitqueue_head(&ctrl_device.data_available);
	atomic_set(&ctrl_device.data_state, 0);

	cdev_init(&ctrl_device.cdev, &elliptic_userspace_ctrl_fops);
	ctrl_device.cdev.owner = THIS_MODULE;

	err = cdev_add(&ctrl_device.cdev, device_number, 1);
	if (err) {
		EL_PRINT_E("cdev_add failed ret=%d", err);
		goto fail_cdev;
	}

	device = device_create(elliptic_class, NULL, device_number,
			       NULL, USERSPACE_CTRL_IO_DEVICE_NAME);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		EL_PRINT_E("device_create failed ret=%d", err);
		goto fail_device;
	}

	return 0;

fail_device:
	cdev_del(&ctrl_device.cdev);
fail_cdev:
	mutex_destroy(&ctrl_device.data_lock);
	mutex_destroy(&ctrl_device.open_lock);
	unregister_chrdev_region(device_number, 1);
	return err;
}

void elliptic_userspace_ctrl_driver_exit(void)
{
	BUG_ON(!elliptic_class);

	device_destroy(elliptic_class,
		       MKDEV(elliptic_userspace_ctrl_major, 0));
	cdev_del(&ctrl_device.cdev);
	unregister_chrdev_region(MKDEV(elliptic_userspace_ctrl_major, 0), 1);

	/*
	 * If a reader is blocked in device_open() waiting for open_lock, signal
	 * the wait queue so it can wake up and see that the cdev is gone, then
	 * destroy the mutexes.
	 */
	wake_up_all(&ctrl_device.data_available);
	atomic_set(&ctrl_device.data_state, -1);

	/*
	 * Unlock open_lock only if a reader currently holds it (i.e. device is
	 * open during rmmod), so any pending device_close() can proceed.
	 */
	mutex_trylock(&ctrl_device.open_lock);
	mutex_unlock(&ctrl_device.open_lock);

	mutex_destroy(&ctrl_device.data_lock);
	mutex_destroy(&ctrl_device.open_lock);
}

int32_t elliptic_userspace_ctrl_write(uint32_t message_id,
				      const char *data, size_t data_size)
{
	uint8_t *pong_buffer;

	if (data_size > ELLIPTIC_MSG_BUF_SIZE) {
		EL_PRINT_E("data_size %zu > buf_size %zu",
			   data_size, (size_t)ELLIPTIC_MSG_BUF_SIZE);
		return -EINVAL;
	}

	mutex_lock(&ctrl_device.data_lock);

	pong_buffer = get_pong_buffer(&ctrl_device, NULL);
	set_pong_buffer_size(&ctrl_device, data_size);
	memcpy(pong_buffer, data, data_size);
	atomic_set(&ctrl_device.data_state, 1);

	mutex_unlock(&ctrl_device.data_lock);

	wake_up_interruptible(&ctrl_device.data_available);
	return 0;
}
