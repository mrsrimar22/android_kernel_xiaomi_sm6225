/**
 * Copyright Elliptic Labs
 *
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <elliptic/elliptic_data_io.h>
#include <elliptic/elliptic_device.h>

#define USE_IRQ 11

static struct task_struct *simulating_task;
static atomic_t cancel;

struct elliptic_data_io_test_state {
};
#define BUFFER_SIZE 128

static int32_t output_buffer[BUFFER_SIZE];
static DEFINE_SPINLOCK(buffer_lock);

irqreturn_t irq_handler(int irq, void *dev_id)
{
	int result;
	unsigned long flags;

	spin_lock_irqsave(&buffer_lock, flags);
	result = elliptic_data_push(ELLIPTIC_ALL_DEVICES,
				(const char *)output_buffer, BUFFER_SIZE * sizeof(int32_t));
	spin_unlock_irqrestore(&buffer_lock, flags);

	return IRQ_HANDLED;
}

static void fill_buffer(int32_t *buffer, size_t len, int32_t value)
{
	size_t i;

	for (i = 0; i < len; ++i)
		buffer[i] = value;
}

int simulating_thread(void *context)
{
	static int32_t count = 0;
	unsigned long flags;

	pr_debug("%s\n", __func__);

	while (!kthread_should_stop()) {
		spin_lock_irqsave(&buffer_lock, flags);
		fill_buffer(output_buffer, BUFFER_SIZE, count);
		spin_unlock_irqrestore(&buffer_lock, flags);
		++count;
		asm("int $0x3B");
		msleep(10);
	}
	return 0;
}

int32_t elliptic_data_io_test_write(uint32_t message_id, const char *data,
	size_t data_size)
{
		return 0;
}

int32_t elliptic_data_io_test_transact(uint32_t message_id, const char *data,
	size_t data_size, char *output_data, size_t output_data_size)
{
	return 0;
}

void elliptic_data_io_test_cancel(struct elliptic_data *elliptic_data)
{
	atomic_set(&elliptic_data->abort_io, 1);
	wake_up_interruptible(&elliptic_data->fifo_isr_not_empty);
}

int elliptic_data_io_test_initialize(void)
{
	int ret;

	pr_debug("%s\n", __func__);

	atomic_set(&cancel, 0);
	ret = request_irq(USE_IRQ, irq_handler, IRQF_SHARED, "my_device", (void *)(irq_handler));
	if (ret) {
		pr_debug("my_device: cannot register IRQ\n");
		return -EPERM;
	}

	simulating_task = kthread_run(&simulating_thread, NULL, "el_simulating_thread");
	if (IS_ERR(simulating_task)) {
		free_irq(USE_IRQ, (void *)(irq_handler));
		return PTR_ERR(simulating_task);
	}

	return 0;
}

int elliptic_data_io_test_cleanup(void)
{
	if (simulating_task) {
		kthread_stop(simulating_task);
		simulating_task = NULL;
	}

	free_irq(USE_IRQ, (void *)(irq_handler));
	return 0;
}
