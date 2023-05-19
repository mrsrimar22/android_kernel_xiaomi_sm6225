/**
 * Copyright Elliptic Labs
 */

#pragma once

#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <elliptic/elliptic_data_io.h>

#define ELLIPTIC_DEVICENAME  "elliptic"
#define ELLIPTIC_NUM_DEVICES 2

#define IOCTL_ELLIPTIC_APP  197
#define MIRROR_TAG          0x3D0A4842

#define IOCTL_ELLIPTIC_DATA_IO_CANCEL \
	_IO(IOCTL_ELLIPTIC_APP, 2)

#define IOCTL_ELLIPTIC_ACTIVATE_ENGINE \
	_IOW(IOCTL_ELLIPTIC_APP, 3, int)

#define IOCTL_ELLIPTIC_SET_RAMP_DOWN \
	_IO(IOCTL_ELLIPTIC_APP, 4)

#define IOCTL_ELLIPTIC_SYSTEM_CONFIGURATION \
	_IOW(IOCTL_ELLIPTIC_APP, 5, int)

#define IOCTL_ELLIPTIC_DATA_IO_MIRROR \
	_IOW(IOCTL_ELLIPTIC_APP, 117, unsigned char *)

/**
 * struct elliptic_device - Per-minor-node driver state.
 *
 * @opened:    Atomic flag; 1 while the device file is held open.
 * @cdev:      Kernel character-device bookkeeping.
 * @open_lock: Mutex that enforces single-opener semantics.
 *             Acquired in device_open(), released in device_close().
 *             Initialised by elliptic_device_initialize(),
 *             destroyed  by elliptic_device_cleanup().
 * @el_data:   FIFO and wait-queue state for the data path.
 */
struct elliptic_device {
	atomic_t opened;
	struct cdev cdev;
	struct mutex open_lock;
	struct elliptic_data el_data;
};

extern struct class *elliptic_class;

#define EL_PRINT_E(string, arg...) \
	pr_err("[ELUS]: %s: " string "\n", __func__, ##arg)

#define EL_PRINT_W(string, arg...) \
	pr_warn("[ELUS]: %s: " string "\n", __func__, ##arg)

#define EL_PRINT_I(string, arg...) \
	pr_info("[ELUS]: %s: " string "\n", __func__, ##arg)

#define EL_PRINT_D(string, arg...) \
	pr_debug("[ELUS]: %s: " string "\n", __func__, ##arg)
