// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) "[wl2866d]: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/srcu.h>
#include "wl2866d.h"

#define WL2866D_IO_REG_LIMIT	20
#define WL2866D_IO_BUFFER_LIMIT	128
#define WL2866D_MISC_MAJOR	250

/*
 * Default output voltage register map.
 * Array indices correspond to the OUT_* / VOL_* / DISCHARGE_* enum values.
 */
static const struct {
	u8 reg;
	int value;
} wl2866d_on_config[] = {
	[OUT_DVDD1] = { 0x03, 0x64 }, /* 1.2 V */
	[OUT_DVDD2] = { 0x04, 0x4B }, /* 1.05 V */
	[OUT_AVDD1] = { 0x05, 0x80 }, /* 2.8 V */
	[OUT_AVDD2] = { 0x06, 0x80 }, /* 2.8 V */
	[VOL_ENABLE] = { 0x0E, 0x0F },
	[VOL_DISABLE] = { 0x0E, 0x00 },
	[DISCHARGE_ENABLE] = { 0x02, 0x8F },
	[DISCHARGE_DISABLE] = { 0x02, 0x00 },
};

/*
 * SRCU domain + single active instance.
 * wl2866d_active is written only under probe/remove serialisation
 * (guaranteed by the I2C core). Readers hold an SRCU read-side lock
 * obtained via wl2866d_lock().
 */
DEFINE_STATIC_SRCU(wl2866d_srcu);
static struct wl2866d_device *wl2866d_active;

int wl2866d_lock(struct wl2866d_lock_ctx *ctx)
{
	ctx->srcu_idx = srcu_read_lock(&wl2866d_srcu);

	if (!READ_ONCE(wl2866d_active)) {
		srcu_read_unlock(&wl2866d_srcu, ctx->srcu_idx);
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(wl2866d_lock);

void wl2866d_unlock(struct wl2866d_lock_ctx *ctx)
{
	srcu_read_unlock(&wl2866d_srcu, ctx->srcu_idx);
}
EXPORT_SYMBOL_GPL(wl2866d_unlock);

static s32 wl2866d_write_reg(struct wl2866d_device *wdev, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };

	if (i2c_master_send(wdev->i2c_client, buf, sizeof(buf)) < 0) {
		pr_err("write error: reg=0x%02x val=0x%02x\n", reg, val);
		return -EIO;
	}

	return 0;
}

static int wl2866d_read_reg(struct wl2866d_device *wdev, u8 reg, u8 *val)
{
	u8 rdbuf = 0;

	if (i2c_master_send(wdev->i2c_client, &reg, 1) != 1) {
		pr_err("write (addr phase) error: reg=0x%02x\n", reg);
		return -EIO;
	}

	if (i2c_master_recv(wdev->i2c_client, &rdbuf, 1) != 1) {
		pr_err("read error: reg=0x%02x\n", reg);
		return -EIO;
	}

	*val = rdbuf;
	return 0;
}

static int __wl2866d_camera_power_control(struct wl2866d_device *wdev,
			  unsigned int out_iotype,
			  int is_power_on)
{
	unsigned char reg_val = 0, reg_read;
	int ret;

	if (!wdev || !READ_ONCE(wdev->on))
		return -ENODEV;

	if (out_iotype > OUT_AVDD2) {
		pr_err("invalid out_iotype %u\n", out_iotype);
		return -EINVAL;
	}

	ret = wl2866d_read_reg(wdev,
			wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
	if (ret < 0) {
		pr_err("read VOL_ENABLE reg failed\n");
		return ret;
	}

	ret = wl2866d_write_reg(wdev,
			wl2866d_on_config[DISCHARGE_ENABLE].reg,
			wl2866d_on_config[DISCHARGE_ENABLE].value);
	if (ret < 0) {
		pr_err("set DISCHARGE_ENABLE failed\n");
		return ret;
	}

	if (is_power_on) {
		ret = wl2866d_read_reg(wdev,
				wl2866d_on_config[out_iotype].reg, &reg_read);
		if (ret < 0 ||
		    reg_read != (u8)wl2866d_on_config[out_iotype].value) {
			ret = wl2866d_write_reg(wdev,
					wl2866d_on_config[out_iotype].reg,
					wl2866d_on_config[out_iotype].value);
			if (ret < 0) {
				pr_err("set voltage failed for out_iotype=%u\n",
					out_iotype);
				return ret;
			}
		}

		if (out_iotype == OUT_DVDD2) {
			u8 vset = 0;

			if (is_power_on == 1050000)
				vset = 0x4B;
			else if (is_power_on == 1200000)
				vset = 0x64;

			if (vset) {
				ret = wl2866d_write_reg(wdev,
						wl2866d_on_config[out_iotype].reg,
						vset);
				if (ret < 0) {
					pr_err("DVDD2 voltage override failed\n");
					return ret;
				}
			}
		}

		reg_val |= (u8)(1u << out_iotype);
	} else {
		reg_val &= (u8)~(1u << out_iotype);
	}

	ret = wl2866d_write_reg(wdev,
			wl2866d_on_config[VOL_ENABLE].reg, reg_val);
	if (ret < 0)
		pr_err("write VOL_ENABLE failed for out_iotype=%u\n",
			out_iotype);

	return ret;
}

int wl2866d_camera_power_control(struct wl2866d_lock_ctx *ctx,
				 unsigned int out_iotype,
				 int is_power_on)
{
	struct wl2866d_device *wdev;
	int ret;

	wdev = READ_ONCE(wl2866d_active);
	if (WARN_ON(!wdev))
		return -ENODEV;

	mutex_lock(&wdev->lock);
	ret = __wl2866d_camera_power_control(wdev, out_iotype, is_power_on);
	mutex_unlock(&wdev->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(wl2866d_camera_power_control);

static int wl2866d_camera_init_power_control(struct wl2866d_device *wdev,
			  unsigned int out_iotype,
			  int is_power_on)
{
	int ret;

	if (!wdev)
		return -ENODEV;

	mutex_lock(&wdev->lock);
	ret = __wl2866d_camera_power_control(wdev, out_iotype, is_power_on);
	mutex_unlock(&wdev->lock);

	return ret;
}

static char hex_to_char(u8 val)
{
	val &= 0x0F;
	return (val >= 10) ? (val - 10 + 'A') : (val + '0');
}

static u8 char_to_hex(char ch)
{
	if (ch >= 'a' && ch <= 'f')
		return ch - 'a' + 10;
	if (ch >= 'A' && ch <= 'F')
		return ch - 'A' + 10;
	if (ch >= '0' && ch <= '9')
		return ch - '0';
	return 0;
}

static int wl2866d_open(struct inode *inode, struct file *file)
{
	struct wl2866d_device *wdev =
		container_of(file->private_data, struct wl2866d_device,
			    misc_dev);

	mutex_lock(&wdev->lock);
	if (!READ_ONCE(wdev->on)) {
		mutex_unlock(&wdev->lock);
		return -ENODEV;
	}
	wdev->offset = 0;
	mutex_unlock(&wdev->lock);

	file->private_data = wdev;
	return 0;
}

static int wl2866d_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static ssize_t wl2866d_read(struct file *file, char __user *buf,
			    size_t count, loff_t *offset)
{
	struct wl2866d_device *wdev = file->private_data;
	int ret = 0, num = 0, i;
	u8 start, u8val;

	if (!wdev)
		return -ENODEV;
	if (count == 0 || count > WL2866D_IO_REG_LIMIT)
		return -ERANGE;
	if (count > wdev->io_buf_size / 6)
		return -ERANGE;

	mutex_lock(&wdev->lock);

	if (!READ_ONCE(wdev->on)) {
		mutex_unlock(&wdev->lock);
		return -ENODEV;
	}

	if (wdev->offset > 0xFF) {
		mutex_unlock(&wdev->lock);
		return -EINVAL;
	}

	start = (u8)wdev->offset;
	memset(wdev->io_buf, 0, count * 6);

	for (i = 0; i < (int)count; i++) {
		u8 addr = start + (u8)i;

		ret = wl2866d_read_reg(wdev, addr, &u8val);
		if (ret < 0) {
			mutex_unlock(&wdev->lock);
			return ret;
		}
		wdev->io_buf[num++] = hex_to_char(addr >> 4);
		wdev->io_buf[num++] = hex_to_char(addr);
		wdev->io_buf[num++] = ' ';
		wdev->io_buf[num++] = hex_to_char(u8val >> 4);
		wdev->io_buf[num++] = hex_to_char(u8val);
		wdev->io_buf[num++] = ' ';
	}
	mutex_unlock(&wdev->lock);

	if (copy_to_user(buf, wdev->io_buf, num))
		return -EFAULT;

	return (ssize_t)count;
}

static ssize_t wl2866d_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *offset)
{
	struct wl2866d_device *wdev = file->private_data;
	char *kbuf;
	int ret = 0, i;

	if (!wdev)
		return -ENODEV;
	if (count == 0 || count > WL2866D_IO_BUFFER_LIMIT)
		return -ERANGE;
	if (count % 6 != 0)
		return -EINVAL;

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	mutex_lock(&wdev->lock);

	if (!READ_ONCE(wdev->on)) {
		mutex_unlock(&wdev->lock);
		kfree(kbuf);
		return -ENODEV;
	}

	for (i = 0; i < (int)count; i += 6) {
		u8 addr = (char_to_hex(kbuf[i]) << 4) |
			    char_to_hex(kbuf[i + 1]);
		u8 val = (char_to_hex(kbuf[i + 3]) << 4) |
			    char_to_hex(kbuf[i + 4]);

		ret = wl2866d_write_reg(wdev, addr, val);
		if (ret < 0) {
			mutex_unlock(&wdev->lock);
			kfree(kbuf);
			return ret;
		}
	}
	mutex_unlock(&wdev->lock);

	kfree(kbuf);
	return count;
}

static loff_t wl2866d_llseek(struct file *file, loff_t offset, int whence)
{
	struct wl2866d_device *wdev = file->private_data;
	u32 new_offset;

	if (!wdev)
		return -ENODEV;

	mutex_lock(&wdev->lock);
	switch (whence) {
	case SEEK_CUR:
		new_offset = wdev->offset + (u32)offset;
		break;
	default:
		new_offset = 0;
		break;
	}
	wdev->offset = min(new_offset, (u32)0xFF);
	mutex_unlock(&wdev->lock);

	return file->f_pos;
}

static const struct file_operations wl2866d_fops = {
	.owner = THIS_MODULE,
	.open = wl2866d_open,
	.release = wl2866d_release,
	.llseek = wl2866d_llseek,
	.read = wl2866d_read,
	.write = wl2866d_write,
};

static int wl2866d_power_on(struct wl2866d_device *wdev)
{
	wdev->en_gpiod = devm_gpiod_get(wdev->dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(wdev->en_gpiod)) {
		pr_err("failed to get en gpio: %ld\n", PTR_ERR(wdev->en_gpiod));
		return PTR_ERR(wdev->en_gpiod);
	}

	gpiod_set_value_cansleep(wdev->en_gpiod, 0);
	return 0;
}

static int wl2866d_init_register(struct wl2866d_device *wdev)
{
	static const u8 regs[] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	};
	static const u8 vals[] = {
		0x00, 0x00, 0x8f, 0x64, 0x4b, 0x80, 0x80, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	int i, j, rc;

	for (i = 0; i < (int)ARRAY_SIZE(regs); i++) {
		for (j = 0; j < 3; j++) {
			rc = wl2866d_write_reg(wdev, regs[i], vals[i]);
			if (!rc)
				break;
			pr_err("write 0x%02x=0x%02x failed, attempt %d\n",
					regs[i], vals[i], j + 1);
			if (j == 2)
				return -EIO;
			usleep_range(3000, 3010);
		}
	}

	return 0;
}

static int wl2866d_match_id(struct wl2866d_device *wdev)
{
	struct device_node *np = wdev->dev->of_node;
	int i, ret;

	ret = of_property_read_u8(np, "id_reg", &wdev->id_reg);
	if (ret) {
		pr_err("id_reg missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u8(np, "id_val", &wdev->id_val);
	if (ret) {
		pr_err("id_val missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u8(np, "id_val1", &wdev->id_val1);
	if (ret) {
		pr_err("id_val1 missing or invalid\n");
		return ret;
	}

	for (i = 0; i < 3; i++) {
		ret = wl2866d_read_reg(wdev, wdev->id_reg, &wdev->chip_id);
		if (!ret && (wdev->chip_id == wdev->id_val ||
			    wdev->chip_id == wdev->id_val1))
			break;
		pr_err("id mismatch (ret=%d chip_id=0x%02x), attempt %d\n",
				ret, wdev->chip_id, i + 1);
		if (i == 2)
			return -ENODEV;
		usleep_range(3000, 3010);
	}

	pr_info("chip_id=0x%02x\n", wdev->chip_id);
	return 0;
}

static int wl2866d_init_module_dev(struct wl2866d_device *wdev)
{
	struct device_node *np = wdev->dev->of_node;
	u32 inits[WL2866D_MAX_CONFIG_NUM * 2];
	u32 num = 0;
	int ret, i;

	ret = of_property_read_u32(np, "init_num", &num);
	if (ret) {
		pr_err("init_num missing or invalid\n");
		return ret;
	}

	if (num == 0 || num > WL2866D_MAX_CONFIG_NUM) {
		pr_err("init_num %u out of range [1, %d]\n",
				num, WL2866D_MAX_CONFIG_NUM);
		return -EINVAL;
	}
	wdev->init_num = (u8)num;

	ret = of_property_read_u32_array(np, "inits", inits, num * 2);
	if (ret) {
		pr_err("inits array missing or invalid\n");
		return ret;
	}

	for (i = 0; i < (int)num; i++) {
		wdev->inits[i].u8Add = (u8)inits[i * 2];
		wdev->inits[i].u8Val = (u8)inits[i * 2 + 1];
		ret = wl2866d_write_reg(wdev,
					wdev->inits[i].u8Add,
					wdev->inits[i].u8Val);
		if (ret < 0) {
			pr_err("init write failed at index %d\n", i);
			return ret;
		}
	}

	return 0;
}

static int wl2866d_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct wl2866d_device *wdev;
	int ret;

	pr_info("entry\n");

	wdev = devm_kzalloc(&client->dev, sizeof(*wdev), GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	wdev->i2c_client = client;
	wdev->dev = &client->dev;
	mutex_init(&wdev->lock);
	i2c_set_clientdata(client, wdev);

	wdev->io_buf_size = WL2866D_IO_BUFFER_LIMIT;
	wdev->io_buf = devm_kcalloc(wdev->dev, 1,
				wdev->io_buf_size, GFP_KERNEL);
	if (!wdev->io_buf) {
		ret = -ENOMEM;
		goto err_destroy_mutex;
	}

	ret = wl2866d_power_on(wdev);
	if (ret) {
		pr_err("power_on failed, ret=%d\n", ret);
		goto err_destroy_mutex;
	}

	ret = wl2866d_init_register(wdev);
	if (ret) {
		pr_err("init_register failed, ret=%d\n", ret);
		goto err_destroy_mutex;
	}

	ret = wl2866d_match_id(wdev);
	if (ret) {
		pr_err("match_id failed, ret=%d\n", ret);
		goto err_destroy_mutex;
	}

	ret = wl2866d_init_module_dev(wdev);
	if (ret) {
		pr_err("init_module failed, ret=%d\n", ret);
		goto err_destroy_mutex;
	}

	snprintf(wdev->misc_name, sizeof(wdev->misc_name),
			"wl2866d-%d-%04x", client->adapter->nr, client->addr);
	wdev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	wdev->misc_dev.name = wdev->misc_name;
	wdev->misc_dev.fops = &wl2866d_fops;

	ret = misc_register(&wdev->misc_dev);
	if (ret) {
		pr_err("misc_register failed (%d)\n", ret);
		goto err_destroy_mutex;
	}

	/*
	 * Mark device live and publish it for SRCU borrowers.
	 * Order: on = true must be visible before wl2866d_active is set,
	 * so that a concurrent wl2866d_lock() that wins the race sees a
	 * fully initialised device.
	 */
	WRITE_ONCE(wdev->on, true);
	smp_wmb();
	WRITE_ONCE(wl2866d_active, wdev);

	wl2866d_camera_init_power_control(wdev, OUT_DVDD1, 0);
	wl2866d_camera_init_power_control(wdev, OUT_DVDD2, 0);
	wl2866d_camera_init_power_control(wdev, OUT_AVDD1, 0);
	wl2866d_camera_init_power_control(wdev, OUT_AVDD2, 0);

	pr_info("probe succeeded, dev=%s\n", wdev->misc_name);
	return 0;

err_destroy_mutex:
	mutex_destroy(&wdev->lock);
	i2c_set_clientdata(client, NULL);
	pr_err("probe failed, ret=%d\n", ret);
	return ret;
}

static int wl2866d_remove(struct i2c_client *client)
{
	struct wl2866d_device *wdev = i2c_get_clientdata(client);

	if (!wdev)
		return 0;

	WRITE_ONCE(wl2866d_active, NULL);
	WRITE_ONCE(wdev->on, false);
	synchronize_srcu(&wl2866d_srcu);

	misc_deregister(&wdev->misc_dev);
	mutex_destroy(&wdev->lock);
	i2c_set_clientdata(client, NULL);

	pr_info("remove succeeded\n");
	return 0;
}

static const struct of_device_id wl2866d_of_match[] = {
	{ .compatible = "ovti,wl2866d-i2c", },
	{ },
};
MODULE_DEVICE_TABLE(of, wl2866d_of_match);

static const struct i2c_device_id wl2866d_id[] = {
	{ "ovti,wl2866d-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl2866d_id);

static struct i2c_driver wl2866d_i2c_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "ovti,wl2866d-i2c",
		.of_match_table	= of_match_ptr(wl2866d_of_match),
	},
	.probe		= wl2866d_probe,
	.remove		= wl2866d_remove,
	.id_table	= wl2866d_id,
};

static int __init cam_wl2866_init_module(void)
{
	int ret;

	ret = i2c_add_driver(&wl2866d_i2c_driver);
	if (ret) {
		pr_err("i2c_add_driver failed (%d)\n", ret);
		return ret;
	}

	pr_info("OK\n");
	return 0;
}

static void __exit cam_wl2866_exit_module(void)
{
	i2c_del_driver(&wl2866d_i2c_driver);
	pr_info("exit\n");
}

subsys_initcall(cam_wl2866_init_module);
module_exit(cam_wl2866_exit_module);

MODULE_DESCRIPTION("WL2866D Power IC Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
