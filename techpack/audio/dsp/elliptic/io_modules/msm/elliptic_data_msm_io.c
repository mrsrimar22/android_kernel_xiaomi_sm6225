/**
 * Copyright Elliptic Labs
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/errno.h>

#include <elliptic/elliptic_data_io.h>
#include <elliptic/elliptic_device.h>

#include <dsp/apr_elliptic.h>
#include <dsp/q6afe-v2.h>

#define IO_PING_PONG_BUFFER_SIZE 512
#define AFE_MSM_RX_PSEUDOPORT_ID 0x8001
#define AFE_MSM_TX_PSEUDOPORT_ID 0x8002

struct elliptic_msm_io_device {
};

int elliptic_data_io_initialize(void)
{
	return 0;
}

int elliptic_data_io_cleanup(void)
{
	return 0;
}

int elliptic_io_open_port(int portid)
{
	if (portid == ULTRASOUND_RX_PORT_ID)
		return afe_start_pseudo_port(AFE_MSM_RX_PSEUDOPORT_ID);
	else if (portid == ULTRASOUND_TX_PORT_ID)
		return afe_start_pseudo_port(AFE_MSM_TX_PSEUDOPORT_ID);

	pr_err("%s: Invalid port ID: %d\n", __func__, portid);
	return -EINVAL;
}

int elliptic_io_close_port(int portid)
{
	if (portid == ULTRASOUND_RX_PORT_ID)
		return afe_stop_pseudo_port(AFE_MSM_RX_PSEUDOPORT_ID);
	else if (portid == ULTRASOUND_TX_PORT_ID)
		return afe_stop_pseudo_port(AFE_MSM_TX_PSEUDOPORT_ID);

	pr_err("%s: Invalid port ID: %d\n", __func__, portid);
	return -EINVAL;
}

int32_t elliptic_data_io_write(uint32_t message_id, const char *data,
	size_t data_size)
{
	int32_t result = 0;
	u8 *safe_data;

	if (!data || data_size == 0)
		return -EINVAL;

	safe_data = kmemdup(data, data_size, GFP_KERNEL);
	if (!safe_data) {
		pr_err("%s: Failed to allocate memory for APR payload\n", __func__);
		return -ENOMEM;
	}

	result = ultrasound_apr_set_parameter(ELLIPTIC_PORT_ID,
		message_id, safe_data, (int32_t)data_size);

	kfree(safe_data);

	return result;
}

int32_t elliptic_data_io_transact(uint32_t message_id, const char *data,
	size_t data_size, char *output_data, size_t output_data_size)
{
	pr_err("%s: unimplemented\n", __func__);
	return -ENOSYS; 
}
