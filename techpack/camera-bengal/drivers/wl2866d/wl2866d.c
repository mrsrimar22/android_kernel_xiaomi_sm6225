#define pr_fmt(fmt)	"[wl2866d]: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include "wl2866d.h"

#define WL2866D_MAX_CONFIG_NUM 16
#define WL2866D_IO_REG_LIMIT 20
#define WL2866D_IO_BUFFER_LIMIT 128
#define WL2866D_MISC_MAJOR 250

#define CONTROL_GPIO54_ENABLE 0

#define WL2866D_PWR_NUMBER 4
static const char *wl2866d_pwr_name[WL2866D_PWR_NUMBER] = {
	"WL2866D_DVDD1",
	"WL2866D_DVDD2",
	"WL2866D_AVDD1",
	"WL2866D_AVDD2",
};

static int wl2866d_pwr_value[WL2866D_PWR_NUMBER] = {
	1200, 1050, 2800, 2800
};

struct wl2866d_map {
	u8 reg;
	int value;
};

static const struct wl2866d_map wl2866d_on_config[] = {
	{0x03, 0x64},	//DVDD1	1.2 V
	{0x04, 0x4B},	//DVDD2	1.05 V
	{0x05, 0x80},	//AVDD1	2.8 V
	{0x06, 0x80},	//AVDD2	2.8 v
	{0x0E, 0x0F},
	{0x0E, 0x00},
	{0x02, 0x8F},
	{0x02, 0x00},
};

/*!
 * reg_value struct
 */
struct reg_value {
	u8 u8Add;
	u8 u8Val;
};

/*!
 * wl2866d_data_t struct
 */
struct wl2866d_data_t {
	struct i2c_client *i2c_client;
	struct regulator *vin1_regulator;
	u32 vin1_vol;
	struct regulator *vin2_regulator;
	u32 vin2_vol;
	int en_gpio;
#if CONTROL_GPIO54_ENABLE
	int iov_gpio; //gpio54, i2c pull control
#endif
	u8 chip_id;
	u8 id_reg;
	u8 id_val;
	u8 id_val1;
	u8 init_num;
	struct reg_value inits[WL2866D_MAX_CONFIG_NUM];
	u32 offset;
	bool on;

	struct gpio_desc *en_gpiod;
#if CONTROL_GPIO54_ENABLE
	struct gpio_desc *iov_gpiod;
#endif
};

/*!
 * wl2866d_data
 */
static struct wl2866d_data_t wl2866d_data;
DEFINE_MUTEX(wl2866d_mutex);

/*!
 * wl2866d write reg function
 *
 * @param reg u8
 * @param val u8
 * @return  Error code indicating success or failure
 */
static s32 wl2866d_write_reg(u8 reg, u8 val)
{
	u8 au8Buf[2] = {0};
	au8Buf[0] = reg;
	au8Buf[1] = val;

	/* i2c_master_send returns number of bytes written or negative error */
	if (i2c_master_send(wl2866d_data.i2c_client, au8Buf, sizeof(au8Buf)) < 0) {
		pr_err("write reg error: reg=%x, val=%x\n", reg, val);
		return -EINVAL;
	}

	return 0;
}

/*!
 * wl2866d read reg function
 *
 * @param reg u8
 * @param val u8 *
 * @return  Error code indicating success or failure
 */
static int wl2866d_read_reg(u8 reg, u8 *val)
{
	u8 au8RegBuf[1] = {0};
	u8 u8RdVal = 0;
	au8RegBuf[0] = reg;

	/* write register address */
	if (i2c_master_send(wl2866d_data.i2c_client, au8RegBuf, sizeof(au8RegBuf)) != (int)sizeof(au8RegBuf)) {
		pr_err("write reg error: reg=%x\n", reg);
		return -EINVAL;
	}

	/* read one byte */
	if (i2c_master_recv(wl2866d_data.i2c_client, &u8RdVal, sizeof(u8RdVal)) != (int)sizeof(u8RdVal)) {
		pr_err("read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -EINVAL;
	}

	*val = u8RdVal;
	pr_debug("wl2866d_read_reg %02x@%02x\n", u8RdVal, reg);
	return 0;
}

#if CONTROL_GPIO54_ENABLE
/*static void enable_i2c_pullup()
{
	int ret = 0;

	pr_debug("try control gpio 54\n");
	if (!gpio_is_valid(iov_gpio)) {
		pr_err("no iov pin available--no return\n");
		//return -EINVAL;
	} else {
		ret = devm_gpio_request_one(&wl2866d_data.i2c_client->dev, iov_gpio,
				GPIOF_OUT_INIT_HIGH, "wl2866d_iov");
		if (ret < 0) {
			pr_err("wl2866d_iov request failed (%d)\n", ret);
			//return ret;
		} else {
			pr_debug("iov request ok\n");
			gpio_direction_output(iov_gpio, 1);
			pr_debug("iov_gpio set high, free it.\n");
			devm_gpio_free(&wl2866d_data.i2c_client->dev, iov_gpio);
		}
	}
}*/

static void iov_one_shot_edge_pulse(int gpio)
{
	int ret, val;

	if (!gpio_is_valid(gpio)) {
		pr_debug("invalid gpio %d\n", gpio);
		return;
	}

	pr_debug("requesting gpio %d\n", gpio);
	ret = gpio_request(gpio, "wl2866d_iov_oneshot");
	if (ret) {
		pr_err("gpio_request %d failed (%d)\n", gpio, ret);
		return;
	}

	gpio_direction_output(gpio, 1);
	val = gpio_get_value_cansleep(gpio);
	pr_debug("after set HIGH, read=%d\n", val);

	gpio_direction_output(gpio, 0);
	val = gpio_get_value_cansleep(gpio);
	pr_debug("after set LOW, read=%d\n", val);

	gpio_free(gpio);
	pr_debug("gpio %d freed\n", gpio);
}
#endif

/*!
 * wl2866d power on function
 *
 * @param dev struct device *
 * @return  Error code indicating success or failure
 */
/*static int wl2866d_power_on(struct device *dev)
{
	int ret = 0;
	wl2866d_data.on = false;

	//mutex_init(&wl2866d_mutex);
	wl2866d_data.en_gpio = of_get_named_gpio(dev->of_node, "en-gpio", 0);
	if (!gpio_is_valid(wl2866d_data.en_gpio)) {
		pr_err("no en pin available\n");
		return -EINVAL;
	}

	ret = devm_gpio_request_one(dev, wl2866d_data.en_gpio,
			GPIOF_OUT_INIT_LOW, "wl2866d_en");
	if (ret < 0) {
		pr_err("en_gpio request failed (%d)\n", ret);
		return ret;
	} else {
		gpio_direction_output(wl2866d_data.en_gpio, 0);
		pr_debug("en_gpio set low\n");
	}

#if CONTROL_GPIO54_ENABLE
	iov_gpio = of_get_named_gpio(dev->of_node, "iov-gpio", 0);
	enable_i2c_pullup();
#endif
	return 0;
}*/

static int wl2866d_power_on(struct device *dev)
{
	wl2866d_data.on = false;

	wl2866d_data.en_gpiod = devm_gpiod_get(dev, "en", GPIOD_OUT_LOW);
	if (IS_ERR(wl2866d_data.en_gpiod)) {
		pr_err("failed to get en gpiod: (%ld)\n", PTR_ERR(wl2866d_data.en_gpiod));
		return PTR_ERR(wl2866d_data.en_gpiod);
	}

	gpiod_set_value_cansleep(wl2866d_data.en_gpiod, 0);
	pr_info("en_gpio set LOW\n");

#if CONTROL_GPIO54_ENABLE
	wl2866d_data.iov_gpio = of_get_named_gpio(dev->of_node, "iov-gpios", 0);
	if (!gpio_is_valid(wl2866d_data.iov_gpio)) {
		pr_debug("iov gpio not provided or invalid (%d)\n", wl2866d_data.iov_gpio);
	} else {
		pr_debug("iov gpio %d saved for one-shot\n", wl2866d_data.iov_gpio);
	}
#endif

	return 0;
}

static int wl2866d_init_register(void)
{
	int i = 0;
	int j = 0;
	int rc = -1;
	u8 reg[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
	u8 val[16] = {0x00, 0x00, 0x8f, 0x64, 0x4b, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	for (i = 0; i < 16; i++) {
		for (j = 0; j < 3; j++) {
			rc = wl2866d_write_reg(reg[i], val[i]);
			if (rc) {
				pr_err("write 0x%x 0x%x failed, retry %d\n", reg[i], val[i], j);
				if (2 == j) {
					return -EINVAL;
				}
				usleep_range(3000, 3010);
			} else {
				break;
			}
		}
	}

	return 0;
}

/*!
 * wl2866d match id function
 *
 * @param dev struct device *
 * @return  Error code indicating success or failure
 */
static int wl2866d_match_id(struct device *dev)
{
	int ret = 0;
	int i = 0;

	ret = of_property_read_u32(dev->of_node, "id_reg",
			(u32 *) &(wl2866d_data.id_reg));
	if (ret) {
		pr_err("id_reg missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "id_val",
			(u32 *) &(wl2866d_data.id_val));
	if (ret) {
		pr_err("id_val missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "id_val1",
			(u32 *) &(wl2866d_data.id_val1));
	if (ret) {
		pr_err("id_val1 missing or invalid\n");
		return ret;
	}

	for (i = 0; i < 3; i++) {
		ret = wl2866d_read_reg(wl2866d_data.id_reg, &(wl2866d_data.chip_id));
		if (ret < 0 ||
			(wl2866d_data.chip_id != wl2866d_data.id_val && wl2866d_data.chip_id != wl2866d_data.id_val1)) {
			pr_err("%d -> %x not match, retry %d\n", ret, wl2866d_data.chip_id, i);
			if (2 == i) {
				return -ENODEV;
			}
			usleep_range(3000, 3010);
		} else {
			break;
		}
	}

	pr_info("chip_id: %d\n", wl2866d_data.chip_id);
	return 0;
}

/*!
 * wl2866d init dev function
 *
 * @param dev struct device *
 * @return  Error code indicating success or failure
 */
static int cam_wl2866_init_module_dev(struct device *dev)
{
	int ret = 0, i = 0;
	u32 inits[32];

	ret = of_property_read_u32(dev->of_node, "init_num",
			(u32 *) &(wl2866d_data.init_num));
	if (ret) {
		pr_err("init_num missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32_array(dev->of_node, "inits",
			inits, wl2866d_data.init_num * 2);
	if (ret) {
		pr_err("inits missing or invalid\n");
		return ret;
	}

	for (i = 0; i < wl2866d_data.init_num; i++) {
		wl2866d_data.inits[i].u8Add = inits[i * 2 + 0];
		wl2866d_data.inits[i].u8Val = inits[i * 2 + 1];
		ret = wl2866d_write_reg(wl2866d_data.inits[i].u8Add, wl2866d_data.inits[i].u8Val);
		if (ret < 0 ) {
			pr_err("update failed (%d)\n", ret);
			return ret;
		}
	}

	pr_debug("init done\n");
	return 0;
}

//bit0:DVDD1, bit1:DVDD2, bit2:AVDD1, bit3:AVDD2
int wl2866d_camera_power_control(unsigned int out_iotype, int is_power_on)
{
	int ret = -1;
	unsigned char reg_val = 0, reg_read;

	if (!wl2866d_data.on) {
		pr_err("function is not available\n");
		return ret;
	}

	if (out_iotype > OUT_AVDD2) {
		pr_err("out_iotype > OUT_AVDD2, param err\n");
		return ret;
	}

	mutex_lock(&wl2866d_mutex);
	ret = wl2866d_read_reg(wl2866d_on_config[VOL_ENABLE].reg, &reg_val);
	if (ret < 0) {
		pr_err("read power out control reg failed\n");
		goto fail_return;
	}
	pr_debug("pwr_ctrl: %s\n", wl2866d_pwr_name[out_iotype]);

	/* Enable DISCHARGE mode to avoid leakage */
	ret = wl2866d_write_reg(wl2866d_on_config[DISCHARGE_ENABLE].reg, wl2866d_on_config[DISCHARGE_ENABLE].value);
	if (ret < 0) {
		pr_err("set discharge mode failed\n");
	}

	if (is_power_on) {
		if (reg_val == 0) {
#if CONTROL_GPIO54_ENABLE
			iov_one_shot_edge_pulse(wl2866d_data.iov_gpio);
#endif
		}
		pr_debug("power on, set voltage %d\n", wl2866d_pwr_value[out_iotype]);
		ret = wl2866d_read_reg(wl2866d_on_config[out_iotype].reg, &reg_read);
		if (ret < 0 || reg_read != wl2866d_on_config[out_iotype].value) {
			ret = wl2866d_write_reg(wl2866d_on_config[out_iotype].reg,
					wl2866d_on_config[out_iotype].value);
			if (ret < 0 ) {
				pr_err("set voltage fail for %d\n", out_iotype);
				goto fail_return;
			}
		} else {
			pr_debug("voltage is right, no need write reg\n");
		}

		reg_val |= 1 << out_iotype;
		pr_debug("power on, set reg %02x\n", reg_val);

		if ((1050000 == is_power_on) && (OUT_DVDD2 == out_iotype)) {
			ret = wl2866d_write_reg(wl2866d_on_config[out_iotype].reg, 0x4B);
			if (ret < 0)
				pr_err("set DVDD2 voltage failed\n");
			else
				pr_debug("set DVDD2 to 1.05V\n");
		} else if ((1200000 == is_power_on) && (OUT_DVDD2 == out_iotype)) {
			ret = wl2866d_write_reg(wl2866d_on_config[out_iotype].reg, 0x64);
			if (ret < 0)
				pr_err("set DVDD2 voltage failed\n");
			else
				pr_debug("set DVDD2 to 1.2V\n");
		}
	} else {
		reg_val &= ~(1 << out_iotype);
		pr_debug("power off, set reg %02x\n", reg_val);
	}

	ret = wl2866d_write_reg(wl2866d_on_config[VOL_ENABLE].reg, reg_val);
	if (ret < 0) {
		pr_err("enable %d failed\n", out_iotype);
		goto fail_return;
	}

fail_return:
	mutex_unlock(&wl2866d_mutex);
	return ret;
}
EXPORT_SYMBOL(wl2866d_camera_power_control);

/*!
 * wl2866d GetHexCh function
 *
 * @param value u8
 * @param shift int
 * @return char value
 */
static char GetHexCh(u8 value, int shift)
{
	u8 data = (value >> shift) & 0x0F;
	char ch = 0;

	if (data >= 10) {
		ch = data - 10 + 'A';
	} else if (data >= 0) {
		ch = data + '0';
	}

	return ch;
}

/*!
 * wl2866d read function
 *
 * @param file struct file *
 * @param buf char __user *
 * @param count size_t
 * @param offset loff_t *
 * @return  read count
 */
static ssize_t wl2866d_read(struct file *file, char __user *buf,
		size_t count, loff_t *offset)
{
	char *buffer = NULL;
	int ret = 0, num = 0, i = 0;
	u8 u8add = wl2866d_data.offset, u8val = 0;

	buffer = kzalloc(WL2866D_IO_BUFFER_LIMIT, GFP_KERNEL);
	if (buffer == NULL) {
		pr_err("memalloc failed\n");
		return -ENOMEM;
	}

	if (count > WL2866D_IO_REG_LIMIT) {
		pr_err("out of range, read count %zu > %d\n", count, WL2866D_IO_REG_LIMIT);
		kfree(buffer);
		return -ERANGE;
	}

	pr_debug("read %zu registers from %02zx to %02zx\n",
			count, (size_t)u8add, (size_t)(u8add + count - 1));
	for (i = 0; i < count; i++, u8add++) {
		ret = wl2866d_read_reg(u8add, &u8val);
		if (ret < 0) {
			pr_err("read %02x failed (%d)\n", (unsigned int)u8add, ret);
			kfree(buffer);
			return ret;
		}

		if (num + 6 > WL2866D_IO_BUFFER_LIMIT) {
			pr_err("internal buffer overflow\n");
			kfree(buffer);
			return -EINVAL;
		}

		buffer[num++] = GetHexCh(u8add, 4);
		buffer[num++] = GetHexCh(u8add, 0);
		buffer[num++] = ' ';
		buffer[num++] = GetHexCh(u8val, 4);
		buffer[num++] = GetHexCh(u8val, 0);
		buffer[num++] = ' ';
		pr_debug("read REG[%02x@%02x]\n", (unsigned int)u8add, (unsigned int)u8val);
	}

	if (copy_to_user(buf, buffer, num))
		pr_err("copy_to_user failed\n");

	kfree(buffer);
	return count;
}

/*!
 * wl2866d GetHex function
 *
 * @param ch char
 * @return hex value
 */
static u8 GetHex(char ch)
{
	u8 value = 0;

	if (ch >= 'a') {
		value = ch - 'a' + 10;
	} else if (ch >= 'A') {
		value = ch - 'A' + 10;
	} else if (ch >= '0') {
		value = ch - '0';
	}

	return value;
}

/*!
 * wl2866d write function
 *
 * @param file struct file *
 * @param buf char __user *
 * @param count size_t
 * @param offset loff_t *
 * @return  write count
 */
static ssize_t wl2866d_write(struct file *file, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret = 0, i = 0;
	char *buffer = NULL;

	if (count > WL2866D_IO_BUFFER_LIMIT) {
		pr_err("out of range, write size %zu > %d\n", count, WL2866D_IO_BUFFER_LIMIT);
		return -ERANGE;
	}

	buffer = memdup_user(buf, count);
	if (IS_ERR(buffer)) {
		pr_err("can't get user data\n");
		return PTR_ERR(buffer);
	}

	pr_debug("write %zu bytes\n", count);
	for (i = 0; i < count; i += 6) {
		u8 u8add = (GetHex(buffer[i + 0]) << 4) | GetHex(buffer[i + 1]);
		u8 u8val = (GetHex(buffer[i + 3]) << 4) | GetHex(buffer[i + 4]);
		ret = wl2866d_write_reg(u8add, u8val);
		if (ret < 0 ) {
			pr_err("write failed (%d)\n", ret);
			kfree(buffer);
			return -ENODEV;
		}
		pr_debug("write REG[%02x@%02x]\n", (unsigned int)u8add, (unsigned int)u8val);
	}

	kfree(buffer);
	return count;
}

/*!
 * wl2866d seek function
 *
 * @param file struct file *
 * @param offset loff_t
 * @param whence int
 * @return file pos
 */
loff_t wl2866d_llseek(struct file *file, loff_t offset, int whence)
{
	switch (whence) {
	case SEEK_CUR:
		wl2866d_data.offset += offset;
		break;
	default:
		wl2866d_data.offset = 0;
		break;
	}

	pr_debug("update read pos to %02x\n", wl2866d_data.offset);
	return file->f_pos;
}

/*!
 * wl2866d open function
 *
 * @param inode struct inode *
 * @param file struct file *
 * @return Error code indicating success or failure
 */
static int wl2866d_open(struct inode *inode, struct file *file)
{
	if (!wl2866d_data.on) {
		pr_err("open failed\n");
		return -ENODEV;
	}

	wl2866d_data.offset = 0;
	return 0;
}

/*!
 * file_operations struct
 */
static const struct file_operations wl2866d_fops = {
	.owner	= THIS_MODULE,
	.open	= wl2866d_open,
	.llseek	= wl2866d_llseek,
	.read	= wl2866d_read,
	.write	= wl2866d_write,
};

/*!
 * miscdevice struct
 */
static struct miscdevice wl2866d_miscdev = {
	.minor	= WL2866D_MISC_MAJOR,
	.name	= "wl2866d",
	.fops	= &wl2866d_fops,
};

/*!
 * wl2866d I2C probe function
 *
 * @param client struct i2c_client *
 * @param id struct i2c_device_id *
 * @return  Error code indicating success or failure
 */
static int wl2866d_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	pr_info("entry\n");
	memset(&wl2866d_data, 0, sizeof(struct wl2866d_data_t));
	wl2866d_data.i2c_client = client;

	ret = wl2866d_power_on(&client->dev);
	if (ret) {
		pr_err("power_on failed (%d)\n", ret);
		return ret;
	}

	ret = wl2866d_init_register();
	if (ret) {
		pr_err("init_register failed (%d)\n", ret);
		return ret;
	}

	ret = wl2866d_match_id(&client->dev);
	if (ret) {
		pr_err("match_id failed (%d)\n", ret);
		return ret;
	}

	ret = cam_wl2866_init_module_dev(&client->dev);
	if (ret) {
		pr_err("init_module_dev failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = misc_register(&wl2866d_miscdev);
	if (ret < 0) {
		pr_err("misc_register failed (%d)\n", ret);
		return ret;
	}

	wl2866d_data.on = true;
	wl2866d_camera_power_control(OUT_DVDD1, 0);
	wl2866d_camera_power_control(OUT_DVDD2, 0);
	wl2866d_camera_power_control(OUT_AVDD1, 0);
	wl2866d_camera_power_control(OUT_AVDD2, 0);

	pr_info("successed!\n");
	return 0;
}

/*!
 * wl2866d I2C remove function
 *
 * @param client struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int wl2866d_remove(struct i2c_client *client)
{
	pr_info("entry\n");
	misc_deregister(&wl2866d_miscdev);
	wl2866d_data.on = false;
	return 0;
}

/*!
 * i2c_device_id struct
 */
static const struct i2c_device_id wl2866d_id[] = {
	{"ovti,wl2866d-i2c", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, wl2866d_id);

#ifdef CONFIG_OF
static const struct of_device_id wl2866d_i2c_of_match_table[] = {
	{ .compatible = "ovti,wl2866d-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, wl2866d_i2c_of_match_table);
#endif

/*!
 * i2c_driver struct
 */
static struct i2c_driver wl2866d_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ovti,wl2866d-i2c",
		.of_match_table = of_match_ptr(wl2866d_i2c_of_match_table),
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
	.probe = wl2866d_probe,
	.remove = wl2866d_remove,
	.id_table = wl2866d_id,
};

/*!
 * wl2866d init function
 *
 * @return  Error code indicating success or failure
 */
static int __init cam_wl2866_init_module(void)
{
	int ret = 0;
	int num_retry = 3;

	do {
		ret = i2c_add_driver(&wl2866d_i2c_driver);
		if (ret == 0) {
			pr_info("add wl2866d driver success\n");
			return ret;
		} else {
			pr_err("add wl2866d driver failed, error=%d, retry times=%d\n",
					ret, num_retry);
			num_retry--;
		}
	} while ((!wl2866d_data.on) && (num_retry > 0));

	return ret;
}

/*!
 * WL2866D cleanup function
 */
static void __exit cam_wl2866_exit_module(void)
{
	i2c_del_driver(&wl2866d_i2c_driver);
	pr_info("delete wl2866d driver success\n");
}

//allow work as standalone module
module_init(cam_wl2866_init_module);
module_exit(cam_wl2866_exit_module);

MODULE_DESCRIPTION("WL2866D Power IC Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
