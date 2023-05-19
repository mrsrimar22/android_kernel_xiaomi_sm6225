/**
 * Copyright Elliptic Labs
 */
/* #define DEBUG */
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <dsp/apr_elliptic.h>
#include <elliptic/elliptic_data_io.h>
#include <elliptic/elliptic_device.h>
#include <elliptic/elliptic_mixer_controls.h>
#include <elliptic/elliptic_sysfs.h>

/*
 * Alternative calibration-load path: read calibration from the filesystem
 * during driver init and push it to the DSP immediately.
 */
#define ELLIPTIC_LOAD_CALIBRATION_DATA_FROM_FILESYSTEM 1

#ifdef ELLIPTIC_LOAD_CALIBRATION_DATA_FROM_FILESYSTEM
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#endif

static struct elliptic_device *elliptic_devices;
struct class *elliptic_class;
static dev_t elliptic_major;
static struct wakeup_source *wake_source;

void elliptic_data_cancel(struct elliptic_data *elliptic_data)
{
	atomic_set(&elliptic_data->abort_io, 1);
	wake_up_interruptible(&elliptic_data->fifo_isr_not_empty);
}

void elliptic_data_reset_debug_counters(struct elliptic_data *elliptic_data)
{
	elliptic_data->isr_fifo_discard = 0;
}

void elliptic_data_print_debug_counters(struct elliptic_data *elliptic_data)
{
	if (elliptic_data->isr_fifo_discard > 0)
		EL_PRINT_E("isr fifo discarded %u frames",
			   elliptic_data->isr_fifo_discard);

	if (elliptic_data->userspace_read_total != elliptic_data->isr_write_total)
		EL_PRINT_I("userspace reads / isr writes: %u / %u",
			   elliptic_data->userspace_read_total,
			   elliptic_data->isr_write_total);

	EL_PRINT_I("total isr fifo discarded frames: %u",
		   elliptic_data->isr_fifo_discard_total);
}

void elliptic_data_update_debug_counters(struct elliptic_data *elliptic_data)
{
	elliptic_data->isr_fifo_discard_total += elliptic_data->isr_fifo_discard;
}

/* Caller must hold fifo_isr_spinlock. */
static void elliptic_data_flush_isr_fifo(struct elliptic_data *elliptic_data)
{
	kfifo_reset(&elliptic_data->fifo_isr);
}

/* Caller must hold fifo_isr_spinlock. */
static void elliptic_data_isr_fifo_pop(struct elliptic_data *elliptic_data,
					size_t size)
{
	uint8_t temp_buffer[ELLIPTIC_MSG_BUF_SIZE];
	unsigned int fifo_result;

	if (size > ELLIPTIC_MSG_BUF_SIZE) {
		EL_PRINT_E("pop size %zu too large", size);
		return;
	}

	fifo_result = kfifo_out(&elliptic_data->fifo_isr, temp_buffer, size);
	if (fifo_result != size)
		EL_PRINT_E("failed to pop element (got %u, wanted %zu)",
			   fifo_result, size);
}

int elliptic_notify_gain_change_msg(int component_id, int gaindb)
{
	int32_t msg[3] = {ESCPT_COMPONENT_GAIN_CHANGE, component_id, gaindb};

	return elliptic_data_write(ELLIPTIC_ULTRASOUND_SET_PARAMS,
				   (const char *)msg, sizeof(msg));
}

static int device_open(struct inode *inode, struct file *filp)
{
	unsigned int major = imajor(inode);
	unsigned int minor = iminor(inode);
	struct elliptic_device *dev;
	struct elliptic_data *elliptic_data;

	if (major != elliptic_major || minor >= ELLIPTIC_NUM_DEVICES) {
		EL_PRINT_W("no device found with minor=%u major=%u",
			   minor, major);
		return -ENODEV;
	}

	dev = &elliptic_devices[minor];

	if (inode->i_cdev != &dev->cdev) {
		EL_PRINT_W("dev pointer mismatch");
		return -ENODEV;
	}

	/*
	 * Enforce single-opener semantics.  mutex_lock_interruptible() returns
	 * -EINTR if a signal arrives while waiting, which propagates correctly
	 * to userspace as EINTR rather than blocking forever.
	 */
	if (mutex_lock_interruptible(&dev->open_lock)) {
		EL_PRINT_E("open interrupted by signal");
		return -ERESTARTSYS;
	}

	elliptic_data = &dev->el_data;

	spin_lock(&elliptic_data->fifo_isr_spinlock);
	elliptic_data_flush_isr_fifo(elliptic_data);
	spin_unlock(&elliptic_data->fifo_isr_spinlock);

	atomic_set(&elliptic_data->abort_io, 0);
	elliptic_data_reset_debug_counters(elliptic_data);

	filp->private_data = dev;
	atomic_set(&dev->opened, 1);

	EL_PRINT_I("Opened device elliptic%u", minor);
	return 0;
}

static int device_close(struct inode *inode, struct file *filp)
{
	struct elliptic_device *device = filp->private_data;
	struct elliptic_data *elliptic_data;

	if (!device) {
		EL_PRINT_E("device not found");
		return -ENODEV;
	}

	elliptic_data = &device->el_data;

	atomic_set(&device->opened, 0);
	elliptic_data_update_debug_counters(elliptic_data);
	elliptic_data_print_debug_counters(elliptic_data);
	elliptic_data_cancel(elliptic_data);

	/* paired with mutex_lock_interruptible() in device_open() */
	mutex_unlock(&device->open_lock);

	EL_PRINT_I("Closed device elliptic%u", iminor(inode));
	return 0;
}

static ssize_t device_read(struct file *fp, char __user *buff,
			   size_t length, loff_t *ppos)
{
	struct elliptic_device *elliptic_device = fp->private_data;

	return elliptic_data_pop(&elliptic_device->el_data, buff, length);
}

static ssize_t device_write(struct file *fp, const char *buff,
			    size_t length, loff_t *ppos)
{
	ssize_t ret;

	if (!buff || length == 0)
		return 0;

	ret = elliptic_data_io_write(ELLIPTIC_ULTRASOUND_SET_PARAMS,
				     buff, length);
	return ret >= 0 ? (ssize_t)length : 0;
}

static long device_ioctl(struct file *fp, unsigned int number,
			 unsigned long param)
{
	struct elliptic_device *device = fp->private_data;
	struct elliptic_data *elliptic_data = &device->el_data;
	int err;
	int32_t enable;
	int32_t msg_act[4]  = {0, 0, 0, 0};
	int32_t msg_ramp[4] = {-1, 0, 0, 0};
	struct elliptic_system_configuration_parameter sys_param;
	unsigned char header[8];
	unsigned char *payload;
	unsigned int mirror_tag, mirror_payload_size;

	switch (number) {
	case IOCTL_ELLIPTIC_DATA_IO_CANCEL:
		EL_PRINT_D("IOCTL_ELLIPTIC_CANCEL_READ %ld", param);
		elliptic_data_cancel(elliptic_data);
		break;

	case IOCTL_ELLIPTIC_ACTIVATE_ENGINE:
		if (copy_from_user(&enable, (void __user *)param, sizeof(enable))) {
			EL_PRINT_E("copy_from_user failed for ACTIVATE_ENGINE");
			return -EFAULT;
		}
		msg_act[0] = enable ? 1 : 0;
		EL_PRINT_D("IOCTL_ACTIVATE_ENGINE enable=%d", enable);
		err = elliptic_data_write(ELLIPTIC_ULTRASOUND_SET_PARAMS,
					  (const char *)msg_act, sizeof(msg_act));
		if (err)
			return err;
		break;

	case IOCTL_ELLIPTIC_SET_RAMP_DOWN:
		EL_PRINT_D("IOCTL_SET_RAMP_DOWN");
		err = elliptic_data_write(ELLIPTIC_ULTRASOUND_SET_PARAMS,
					  (const char *)msg_ramp, sizeof(msg_ramp));
		if (err)
			return err;
		break;

	case IOCTL_ELLIPTIC_SYSTEM_CONFIGURATION:
		if (copy_from_user(&sys_param, (void __user *)param, sizeof(sys_param))) {
			EL_PRINT_E("copy_from_user failed for SYSTEM_CONFIGURATION");
			return -EFAULT;
		}
		EL_PRINT_D("IOCTL_SYSTEM_CONFIGURATION type=%d", sys_param.type);
		err = elliptic_data_write(ELLIPTIC_ULTRASOUND_SET_PARAMS,
					  (const char *)&sys_param, sizeof(sys_param));
		if (err)
			return err;
		break;

	case IOCTL_ELLIPTIC_DATA_IO_MIRROR:
		if (copy_from_user(header, (void __user *)param, sizeof(header))) {
			EL_PRINT_E("copy_from_user failed for mirror header");
			return -EFAULT;
		}
		mirror_tag = *(unsigned int *)header;
		mirror_payload_size = *((unsigned int *)header + 1);

		if (mirror_tag != MIRROR_TAG || mirror_payload_size == 0 ||
		    mirror_payload_size > (ELLIPTIC_SET_PARAMS_SIZE * 4)) {
			EL_PRINT_E("mirror: invalid tag or length");
			break;
		}

		payload = kzalloc(mirror_payload_size, GFP_KERNEL);
		if (!payload)
			return -ENOMEM;

		if (copy_from_user(payload, (void __user *)(param + 8),
				   mirror_payload_size)) {
			EL_PRINT_E("copy_from_user failed for mirror payload");
			kfree(payload);
			return -EFAULT;
		}

		err = elliptic_data_io_write(ELLIPTIC_ULTRASOUND_SET_PARAMS,
					     payload, mirror_payload_size);
		kfree(payload);

		if (err) {
			EL_PRINT_E("mirror: elliptic_data_io_write failed ret=%d", err);
			return err;
		}
		break;

	default:
		EL_PRINT_W("unknown IOCTL number=%u", number);
		break;
	}

	return 0;
}

static unsigned int device_poll(struct file *file,
				struct poll_table_struct *poll_table)
{
	struct elliptic_device *device = file->private_data;
	struct elliptic_data *elliptic_data = &device->el_data;

	poll_wait(file, &elliptic_data->fifo_isr_not_empty, poll_table);

	return kfifo_is_empty(&elliptic_data->fifo_isr) ? 0 : (POLLIN | POLLRDNORM);
}

static const struct file_operations elliptic_fops = {
	.owner = THIS_MODULE,
	.open = device_open,
	.release = device_close,
	.read = device_read,
	.write = device_write,
	.poll = device_poll,
	.unlocked_ioctl = device_ioctl,
};

int elliptic_data_initialize(struct elliptic_data *elliptic_data,
			     size_t queue_size, unsigned int wakeup_timeout,
			     int id)
{
	/* kfifo requires power-of-two sizes */
	if (!queue_size || (queue_size & (queue_size - 1))) {
		EL_PRINT_E("queue_size %zu is not a power of two", queue_size);
		return -EINVAL;
	}

	atomic_set(&elliptic_data->abort_io, 0);
	spin_lock_init(&elliptic_data->fifo_isr_spinlock);
	elliptic_data->wakeup_timeout = wakeup_timeout;
	mutex_init(&elliptic_data->user_buffer_lock);
	init_waitqueue_head(&elliptic_data->fifo_isr_not_empty);

	if (kfifo_alloc(&elliptic_data->fifo_isr, queue_size, GFP_KERNEL)) {
		EL_PRINT_E("failed to allocate fifo isr");
		mutex_destroy(&elliptic_data->user_buffer_lock);
		return -ENOMEM;
	}

	return 0;
}

int elliptic_data_cleanup(struct elliptic_data *elliptic_data)
{
	kfifo_free(&elliptic_data->fifo_isr);
	mutex_destroy(&elliptic_data->user_buffer_lock);
	return 0;
}

size_t elliptic_data_pop(struct elliptic_data *elliptic_data,
			 char __user *user_buffer, size_t buffer_size)
{
	int result;
	unsigned long num_copied;
	unsigned int fifo_result;
	unsigned long flags;

	if (buffer_size < ELLIPTIC_MSG_BUF_SIZE) {
		EL_PRINT_E("buffer_size %zu < required %zu",
			   buffer_size, (size_t)ELLIPTIC_MSG_BUF_SIZE);
		return 0;
	}

	result = wait_event_interruptible(elliptic_data->fifo_isr_not_empty,
		!kfifo_is_empty(&elliptic_data->fifo_isr) ||
		atomic_read(&elliptic_data->abort_io));

	if (atomic_read(&elliptic_data->abort_io)) {
		atomic_set(&elliptic_data->abort_io, 0);
		EL_PRINT_D("pop cancelled");
		return 0;
	}

	if (result != 0) {
		if (result == -ERESTARTSYS)
			EL_PRINT_I("wait interrupted");
		else
			EL_PRINT_E("wait error=%d", result);
		return 0;
	}

	spin_lock_irqsave(&elliptic_data->fifo_isr_spinlock, flags);
	fifo_result = kfifo_out(&elliptic_data->fifo_isr,
				elliptic_data->isr_swap_buffer,
				ELLIPTIC_MSG_BUF_SIZE);
	spin_unlock_irqrestore(&elliptic_data->fifo_isr_spinlock, flags);

	if (fifo_result == 0) {
		EL_PRINT_E("fifo_out returned 0");
		return 0;
	}

	mutex_lock(&elliptic_data->user_buffer_lock);
	num_copied = copy_to_user(user_buffer, elliptic_data->isr_swap_buffer,
				  ELLIPTIC_MSG_BUF_SIZE);
	mutex_unlock(&elliptic_data->user_buffer_lock);

	if (num_copied) {
		EL_PRINT_E("copy_to_user failed (%lu bytes not copied)", num_copied);
		return 0;
	}

	++elliptic_data->userspace_read_total;
	return ELLIPTIC_MSG_BUF_SIZE;
}

int elliptic_data_push(int deviceid, const char *buffer,
		       size_t buffer_size, elliptic_data_push_t data_source)
{
	size_t zeros_to_pad;
	unsigned int copied_from_user = 0;
	int copy_from_user_result;
	int i = 0;
	int i_max = ELLIPTIC_NUM_DEVICES;
	unsigned long flags;
	uint8_t zero_pad_buffer[ELLIPTIC_MSG_BUF_SIZE] = {0};

	if (buffer_size > ELLIPTIC_MSG_BUF_SIZE)
		return -EINVAL;

	zeros_to_pad = ELLIPTIC_MSG_BUF_SIZE - buffer_size;

	if (deviceid != ELLIPTIC_ALL_DEVICES) {
		i = deviceid;
		i_max = i + 1;
	}

	for (; i < i_max; ++i) {
		struct elliptic_device *device = &elliptic_devices[i];
		struct elliptic_data *el_data = &device->el_data;
		unsigned int fifo_result;

		if (!atomic_read(&device->opened))
			continue;

		spin_lock_irqsave(&el_data->fifo_isr_spinlock, flags);

		/* Make room if needed by evicting the oldest frame */
		if (kfifo_avail(&el_data->fifo_isr) < ELLIPTIC_MSG_BUF_SIZE) {
			++el_data->isr_fifo_discard;
			elliptic_data_isr_fifo_pop(el_data, ELLIPTIC_MSG_BUF_SIZE);
		}

		if (data_source == ELLIPTIC_DATA_PUSH_FROM_KERNEL) {
			fifo_result = kfifo_in(&el_data->fifo_isr, buffer, buffer_size);
			if (fifo_result == 0) {
				spin_unlock_irqrestore(&el_data->fifo_isr_spinlock, flags);
				continue;
			}
		} else {
			copy_from_user_result = kfifo_from_user(&el_data->fifo_isr,
							buffer, buffer_size,
							&copied_from_user);
			if (copy_from_user_result == -EFAULT) {
				spin_unlock_irqrestore(&el_data->fifo_isr_spinlock, flags);
				continue;
			}
		}

		if (zeros_to_pad > 0) {
			fifo_result = kfifo_in(&el_data->fifo_isr,
					       zero_pad_buffer, zeros_to_pad);
			if (fifo_result == 0) {
				/* Roll back the partial write */
				elliptic_data_isr_fifo_pop(el_data, buffer_size);
				++el_data->isr_fifo_discard;
				spin_unlock_irqrestore(&el_data->fifo_isr_spinlock, flags);
				continue;
			}
		}

		++el_data->isr_write_total;
		spin_unlock_irqrestore(&el_data->fifo_isr_spinlock, flags);

		wake_up_interruptible(&el_data->fifo_isr_not_empty);
		__pm_wakeup_event(wake_source, el_data->wakeup_timeout);
	}

	return 0;
}

int elliptic_open_port(int portid)
{
	return elliptic_io_open_port(portid);
}

int elliptic_close_port(int portid)
{
	return elliptic_io_close_port(portid);
}

int32_t elliptic_data_write(uint32_t message_id,
			    const char *data, size_t data_size)
{
	int32_t err_dsp, err_us;

	err_dsp = elliptic_data_io_write(message_id, data, data_size);
	if (err_dsp)
		EL_PRINT_E("write to DSP failed ret=%d", err_dsp);

	err_us = elliptic_userspace_ctrl_write(message_id, data, data_size);
	if (err_us)
		EL_PRINT_E("write to userspace ctrl failed ret=%d", err_us);

	return err_dsp | err_us;
}

/**
 * elliptic_device_initialize() - Set up one /dev/ellipticN node.
 *
 * Calls mutex_init() for open_lock here; the paired mutex_destroy() is in
 * elliptic_device_cleanup().
 */
static int elliptic_device_initialize(struct elliptic_device *elliptic_device,
				      int minor, struct class *class)
{
	dev_t device_number = MKDEV(elliptic_major, minor);
	struct device *device;
	int err;

	BUG_ON(!elliptic_device || !class);

	/*
	 * Initialise the single-opener mutex.  This must come before cdev_add()
	 * because the device may be opened immediately after that call returns.
	 */
	mutex_init(&elliptic_device->open_lock);

	cdev_init(&elliptic_device->cdev, &elliptic_fops);
	elliptic_device->cdev.owner = THIS_MODULE;

	err = cdev_add(&elliptic_device->cdev, device_number, 1);
	if (err) {
		EL_PRINT_E("cdev_add failed for %s%d ret=%d",
			   ELLIPTIC_DEVICENAME, minor, err);
		mutex_destroy(&elliptic_device->open_lock);
		return err;
	}

	device = device_create(class, NULL, device_number,
			       NULL, ELLIPTIC_DEVICENAME "%d", minor);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		EL_PRINT_E("device_create failed for %s%d ret=%d",
			   ELLIPTIC_DEVICENAME, minor, err);
		cdev_del(&elliptic_device->cdev);
		mutex_destroy(&elliptic_device->open_lock);
		return err;
	}

	return 0;
}

/**
 * elliptic_device_cleanup() - Tear down one /dev/ellipticN node.
 *
 * If the device is still open when the driver is removed (e.g. rmmod while
 * a process holds it), we release open_lock so any blocked opener can
 * unblock, then destroy the mutex.
 */
static void elliptic_device_cleanup(struct elliptic_device *dev, int minor,
				    struct class *class)
{
	BUG_ON(!dev || !class);

	device_destroy(class, MKDEV(elliptic_major, minor));
	cdev_del(&dev->cdev);

	/*
	 * If the device is held open, open_lock is currently locked.  Unlock it
	 * so any thread waiting in mutex_lock_interruptible() can observe the
	 * unregistered cdev and return -ENODEV, then destroy the mutex.
	 */
	if (atomic_read(&dev->opened))
		mutex_unlock(&dev->open_lock);

	mutex_destroy(&dev->open_lock);
}

static void elliptic_driver_cleanup(int devices_to_destroy)
{
	int i;

	if (elliptic_devices) {
		elliptic_data_io_cleanup();

		for (i = 0; i < devices_to_destroy; ++i) {
			elliptic_data_cleanup(&elliptic_devices[i].el_data);
			elliptic_device_cleanup(&elliptic_devices[i], i,
						elliptic_class);
		}

		kfree(elliptic_devices);
		elliptic_devices = NULL;
	}

	if (elliptic_class) {
		class_destroy(elliptic_class);
		elliptic_class = NULL;
	}

	unregister_chrdev_region(MKDEV(elliptic_major, 0), ELLIPTIC_NUM_DEVICES);
}

#ifdef ELLIPTIC_LOAD_CALIBRATION_DATA_FROM_FILESYSTEM
#define ELLIPTIC_CALIBRATION_MAX_DATA_SIZE \
	(ELLIPTIC_CALIBRATION_V2_DATA_SIZE + ELLIPTIC_CALIBRATION_DATA_SIZE)

static unsigned char calibration_data[ELLIPTIC_CALIBRATION_MAX_DATA_SIZE];
static const char *calibration_filename =
	"/mnt/vendor/persist/audio/elliptic_calibration";

static size_t load_calibration_data(const char *filename)
{
	int fd;
	size_t bytes_read;

	fd = ksys_open(filename, O_RDONLY, 0);
	if (fd < 0)
		return 0;

	bytes_read = ksys_read(fd, calibration_data,
			       ELLIPTIC_CALIBRATION_MAX_DATA_SIZE);
	ksys_close(fd);

	/* Accept only exact-size matches */
	if (bytes_read == ELLIPTIC_CALIBRATION_DATA_SIZE ||
	    bytes_read == ELLIPTIC_CALIBRATION_V2_DATA_SIZE)
		return bytes_read;

	return 0;
}

static int32_t elliptic_send_calibration_to_engine(size_t calib_data_size)
{
	elliptic_set_calibration_data(calibration_data, calib_data_size);
	return elliptic_data_write(ELLIPTIC_ULTRASOUND_SET_PARAMS,
				   (const char *)calibration_data,
				   calib_data_size);
}
#endif /* ELLIPTIC_LOAD_CALIBRATION_DATA_FROM_FILESYSTEM */

int elliptic_driver_init(void)
{
	dev_t device_number;
	int err;
	int i;
	int devices_to_destroy = 0;
#ifdef ELLIPTIC_LOAD_CALIBRATION_DATA_FROM_FILESYSTEM
	size_t calib_data_size;
#endif

	err = alloc_chrdev_region(&device_number, 0, ELLIPTIC_NUM_DEVICES,
				  ELLIPTIC_DEVICENAME);
	if (err < 0) {
		EL_PRINT_E("alloc_chrdev_region failed ret=%d", err);
		return err;
	}
	elliptic_major = MAJOR(device_number);

	elliptic_class = class_create(THIS_MODULE, "chardev");
	if (!elliptic_class) {
		EL_PRINT_E("class_create failed");
		goto fail;
	}

	err = elliptic_initialize_sysfs();
	if (err)
		goto fail;

	elliptic_devices = kcalloc(ELLIPTIC_NUM_DEVICES,
				   sizeof(*elliptic_devices), GFP_KERNEL);
	if (!elliptic_devices) {
		err = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < ELLIPTIC_NUM_DEVICES; ++i) {
		err = elliptic_device_initialize(&elliptic_devices[i], i,
						 elliptic_class);
		if (err) {
			devices_to_destroy = i;
			goto fail;
		}
		devices_to_destroy = i + 1;

		err = elliptic_data_initialize(&elliptic_devices[i].el_data,
					       ELLIPTIC_DATA_FIFO_SIZE,
					       ELLIPTIC_WAKEUP_TIMEOUT, i);
		if (err)
			goto fail;
	}

	if (elliptic_data_io_initialize())
		goto fail;

	if (elliptic_userspace_io_driver_init())
		goto fail;

	if (elliptic_userspace_ctrl_driver_init())
		goto fail_with_io;

	wake_source = wakeup_source_register(NULL, "elliptic_wake_source");
	if (!wake_source) {
		EL_PRINT_E("wakeup_source_register failed");
		err = -ENOMEM;
		goto fail_with_io_and_ctrl;
	}

#ifdef ELLIPTIC_LOAD_CALIBRATION_DATA_FROM_FILESYSTEM
	calib_data_size = load_calibration_data(calibration_filename);
	if (calib_data_size > 0)
		elliptic_send_calibration_to_engine(calib_data_size);
#endif

	return 0;

fail_with_io_and_ctrl:
	elliptic_userspace_ctrl_driver_exit();
fail_with_io:
	elliptic_userspace_io_driver_exit();
fail:
	elliptic_driver_cleanup(devices_to_destroy);
	return err;
}

void elliptic_driver_exit(void)
{
	wakeup_source_unregister(wake_source);
	elliptic_cleanup_sysfs();
	elliptic_driver_cleanup(ELLIPTIC_NUM_DEVICES);
	elliptic_userspace_ctrl_driver_exit();
	elliptic_userspace_io_driver_exit();
}

MODULE_AUTHOR("Elliptic Labs");
MODULE_DESCRIPTION("Providing Interface to UPS data");
MODULE_LICENSE("GPL");
