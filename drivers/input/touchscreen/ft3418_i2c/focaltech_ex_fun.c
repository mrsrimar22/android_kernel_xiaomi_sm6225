// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#include "focaltech_ex_fun.h"

/*****************************************************************************
 * Apk procfs interface
 *****************************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff,
			       size_t count, loff_t *ppos)
{
	u8 *writebuf = NULL;
	u8 tmpbuf[PROC_BUF_SIZE] = {0};
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	char tmp[PROC_BUF_SIZE];
	struct fts_ts_data *ts_data = PDE_DATA(file_inode(filp));
	struct ftxxxx_proc *proc;

	if (!ts_data)
		return -EINVAL;

	proc = &ts_data->proc;

	if (buflen <= 1) {
		FTS_ERROR("apk proc write count(%d) fail", buflen);
		return -EINVAL;
	}

	if (buflen > PROC_BUF_SIZE)
		buflen = PROC_BUF_SIZE;

	writebuf = tmpbuf;

	if (copy_from_user(writebuf, buff, buflen)) {
		FTS_ERROR("[APK]: copy from user error!!");
		ret = -EFAULT;
		goto proc_write_err;
	}

	proc->opmode = writebuf[0];
	switch (proc->opmode) {
	case PROC_SET_TEST_FLAG:
		FTS_DEBUG("[APK]: PROC_SET_TEST_FLAG = %x", writebuf[1]);
#if FTS_ESDCHECK_EN
		if (ts_data->esdcheck) {
			if (writebuf[1] == 0)
				fts_esdcheck_switch(ts_data->esdcheck, ENABLE);
			else
				fts_esdcheck_switch(ts_data->esdcheck, DISABLE);
		}
#endif
		break;
	case PROC_READ_REGISTER:
		proc->cmd[0] = writebuf[1];
		break;
	case PROC_WRITE_REGISTER:
		ret = fts_write_reg(ts_data, writebuf[1], writebuf[2]);
		if (ret < 0) {
			FTS_ERROR("PROC_WRITE_REGISTER write error");
			goto proc_write_err;
		}
		break;
	case PROC_READ_DATA:
		writelen = buflen - 1;
		if (writelen >= FTX_MAX_COMMMAND_LENGTH) {
			FTS_ERROR("cmd(PROC_READ_DATA) length(%d) fail", writelen);
			goto proc_write_err;
		}
		memcpy(proc->cmd, writebuf + 1, writelen);
		proc->cmd_len = writelen;
		ret = fts_write(ts_data, writebuf + 1, writelen);
		if (ret < 0) {
			FTS_ERROR("PROC_READ_DATA write error");
			goto proc_write_err;
		}
		break;
	case PROC_WRITE_DATA:
		writelen = buflen - 1;
		ret = fts_write(ts_data, writebuf + 1, writelen);
		if (ret < 0) {
			FTS_ERROR("PROC_WRITE_DATA write error");
			goto proc_write_err;
		}
		break;
	case PROC_HW_RESET:
		snprintf(tmp, PROC_BUF_SIZE, "%s", writebuf + 1);
		tmp[buflen - 1] = '\0';
		if (strncmp(tmp, "focal_driver", 12) == 0) {
			FTS_DEBUG("APK execute HW Reset");
			fts_reset_proc(ts_data, 0);
		}
		break;
	case PROC_SET_BOOT_MODE:
		FTS_DEBUG("[APK]: PROC_SET_BOOT_MODE = %x", writebuf[1]);
		if (writebuf[1] == 0)
			ts_data->fw_is_running = true;
		else
			ts_data->fw_is_running = false;
		break;
	case PROC_ENTER_TEST_ENVIRONMENT:
		FTS_DEBUG("[APK]: PROC_ENTER_TEST_ENVIRONMENT = %x", writebuf[1]);
		if (writebuf[1] == 0)
			fts_enter_test_environment(ts_data, false);
		else
			fts_enter_test_environment(ts_data, true);
		break;
	default:
		break;
	}
	ret = buflen;

proc_write_err:
	return ret;
}

static ssize_t fts_debug_read(struct file *filp, char __user *buff,
			      size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int buflen = count;
	u8 *readbuf = NULL;
	u8 tmpbuf[PROC_BUF_SIZE] = {0};
	struct fts_ts_data *ts_data = PDE_DATA(file_inode(filp));
	struct ftxxxx_proc *proc;

	if (!ts_data)
		return -EINVAL;

	proc = &ts_data->proc;

	if (buflen <= 0) {
		FTS_ERROR("apk proc read count(%d) fail", buflen);
		return -EINVAL;
	}

	if (buflen > PROC_BUF_SIZE)
		buflen = PROC_BUF_SIZE;

	readbuf = tmpbuf;

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 1);
#endif

	switch (proc->opmode) {
	case PROC_READ_REGISTER:
		num_read_chars = 1;
		ret = fts_read_reg(ts_data, proc->cmd[0], &readbuf[0]);
		if (ret < 0) {
			FTS_ERROR("PROC_READ_REGISTER read error");
			goto proc_read_err;
		}
		break;
	case PROC_READ_DATA:
		num_read_chars = buflen;
		ret = fts_read(ts_data, NULL, 0, readbuf, num_read_chars);
		if (ret < 0) {
			FTS_ERROR("PROC_READ_DATA read error");
			goto proc_read_err;
		}
		break;
	default:
		break;
	}

	ret = num_read_chars;

	if (copy_to_user(buff, readbuf, num_read_chars)) {
		FTS_ERROR("copy to user error");
		ret = -EFAULT;
	}

proc_read_err:
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 0);
#endif

	return ret;
}

static const struct file_operations fts_proc_fops = {
	.owner	= THIS_MODULE,
	.read	= fts_debug_read,
	.write	= fts_debug_write,
};

int fts_create_apk_debug_channel(struct fts_ts_data *ts_data)
{
	struct ftxxxx_proc *proc;
	struct fts_ex_fun_data *ex_fun;

	if (!ts_data)
		return -EINVAL;

	proc = &ts_data->proc;
	ex_fun = kzalloc(sizeof(*ex_fun), GFP_KERNEL);
	if (!ex_fun)
		return -ENOMEM;

	ex_fun->ts_data = ts_data;
	ts_data->ex_fun = ex_fun;

	proc->proc_entry = proc_create_data(PROC_NAME, 0600, NULL,
			&fts_proc_fops, ts_data);
	if (!proc->proc_entry) {
		FTS_ERROR("create proc entry fail");
		kfree(ts_data->ex_fun);
		return -ENOMEM;
	}

	FTS_DEBUG("Create proc entry success!");
	return 0;
}

void fts_release_apk_debug_channel(struct fts_ts_data *ts_data)
{
	if (ts_data) {
		if (ts_data->proc.proc_entry)
			proc_remove(ts_data->proc.proc_entry);

		kfree(ts_data->ex_fun);
	}
}

/************************************************************************
 * sysfs interface
 ***********************************************************************/
static ssize_t fts_hw_reset_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	ssize_t count = 0;

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);
	fts_reset_proc(ts_data, 0);
	count = snprintf(buf, PAGE_SIZE, "hw reset executed\n");
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_irq_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct irq_desc *desc;

	if (!ts_data)
		return -EINVAL;

	desc = irq_to_desc(ts_data->irq);
	if (!desc)
		return -EINVAL;

	return snprintf(buf, PAGE_SIZE, "irq_depth:%d\n", desc->depth);
}

static ssize_t fts_irq_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_DEBUG("enable irq");
		fts_irq_enable(ts_data);
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_DEBUG("disable irq");
		fts_irq_disable(ts_data);
	}
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_boot_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);
	if (FTS_SYSFS_ECHO_ON(buf)) {
		FTS_DEBUG("[EX-FUN]set to boot mode");
		ts_data->fw_is_running = false;
	} else if (FTS_SYSFS_ECHO_OFF(buf)) {
		FTS_DEBUG("[EX-FUN]set to fw mode");
		ts_data->fw_is_running = true;
	}
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_boot_mode_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	ssize_t count = 0;

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);
	if (ts_data->fw_is_running)
		count = snprintf(buf, PAGE_SIZE, "tp is in fw mode\n");
	else
		count = snprintf(buf, PAGE_SIZE, "tp is in boot mode\n");
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_fw_version_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int ret = 0;
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	ssize_t num_read_chars = 0;
	u8 fwver = 0;

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 1);
#endif

	ret = fts_read_reg(ts_data, FTS_REG_FW_VER, &fwver);

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 0);
#endif

	if (ret < 0 || fwver == 0xFF || fwver == 0x00)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02x\n", fwver);

	mutex_unlock(&ts_data->state_lock);
	return num_read_chars;
}

static ssize_t fts_rw_reg_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct fts_ex_fun_data *ex_fun;
	int count = 0;
	int i;

	if (!ts_data || !ts_data->ex_fun)
		return -EINVAL;

	ex_fun = ts_data->ex_fun;

	mutex_lock(&ts_data->state_lock);

	if (ex_fun->len < 0) {
		count = snprintf(buf, PAGE_SIZE, "Invalid cmd line\n");
	} else if (ex_fun->len == 1) {
		if (ex_fun->type == RWREG_OP_READ) {
			if (ex_fun->res == 0)
				count = snprintf(buf, PAGE_SIZE,
						 "Read %02X: %02X\n",
						 ex_fun->reg, ex_fun->val);
			else
				count = snprintf(buf, PAGE_SIZE,
						 "Read %02X failed, ret: %d\n",
						 ex_fun->reg, ex_fun->res);
		} else {
			if (ex_fun->res == 0)
				count = snprintf(buf, PAGE_SIZE,
						 "Write %02X, %02X success\n",
						 ex_fun->reg, ex_fun->val);
			else
				count = snprintf(buf, PAGE_SIZE,
						 "Write %02X failed, ret: %d\n",
						 ex_fun->reg, ex_fun->res);
		}
	} else {
		if (ex_fun->type == RWREG_OP_READ) {
			count = snprintf(buf, PAGE_SIZE,
					 "Read Reg: [%02X]-[%02X]\n",
					 ex_fun->reg, ex_fun->reg + ex_fun->len);
			count += scnprintf(buf + count, PAGE_SIZE - count, "Result: ");
			if (ex_fun->res) {
				count += scnprintf(buf + count, PAGE_SIZE - count,
						   "failed, ret: %d\n", ex_fun->res);
			} else {
				if (ex_fun->opbuf && ex_fun->dirty) {
					for (i = 0; i < ex_fun->len; i++)
						count += scnprintf(buf + count,
								   PAGE_SIZE - count, "%02X ",
								   ex_fun->opbuf[i]);
					count += scnprintf(buf + count,
							   PAGE_SIZE - count, "\n");
				}
			}
		} else {
			count = snprintf(buf, PAGE_SIZE,
					 "Write Reg: [%02X]-[%02X]\n",
					 ex_fun->reg, ex_fun->reg + ex_fun->len - 1);
			count += scnprintf(buf + count, PAGE_SIZE - count, "Write Data: ");
			if (ex_fun->opbuf) {
				for (i = 1; i < ex_fun->len; i++)
					count += scnprintf(buf + count, PAGE_SIZE - count,
							   "%02X ", ex_fun->opbuf[i]);
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
			}
			if (ex_fun->res)
				count += scnprintf(buf + count, PAGE_SIZE - count,
						   "Result: failed, ret: %d\n",
						   ex_fun->res);
			else
				count += scnprintf(buf + count, PAGE_SIZE - count,
						   "Result: success\n");
		}
	}

	mutex_unlock(&ts_data->state_lock);

	return count;
}

static int shex_to_int(const char *hex_buf, int size)
{
	int i;
	int base = 1;
	int value = 0;
	char single;

	for (i = size - 1; i >= 0; i--) {
		single = hex_buf[i];

		if ((single >= '0') && (single <= '9'))
			value += (single - '0') * base;
		else if ((single >= 'a') && (single <= 'z'))
			value += (single - 'a' + 10) * base;
		else if ((single >= 'A') && (single <= 'Z'))
			value += (single - 'A' + 10) * base;
		else
			return -EINVAL;

		base *= 16;
	}

	return value;
}

static u8 shex_to_u8(const char *hex_buf, int size)
{
	return (u8)shex_to_int(hex_buf, size);
}

static int fts_parse_buf(struct fts_ex_fun_data *ex_fun,
			 const char *buf,
			 size_t cmd_len)
{
	int length;
	int i;
	char *tmpbuf;

	ex_fun->reg = shex_to_u8(buf + 1, 2);
	length = shex_to_int(buf + 3, 2);

	if (buf[0] == '1') {
		ex_fun->len = length;
		ex_fun->type = RWREG_OP_READ;
	} else {
		if (cmd_len < (length * 2 + 5)) {
			FTS_ERROR("data invalided!");
			return -EINVAL;
		}

		ex_fun->type = RWREG_OP_WRITE;
		ex_fun->len = length + 1;
	}

	if (ex_fun->len > 0) {
		tmpbuf = kzalloc(ex_fun->len, GFP_KERNEL);
		if (!tmpbuf)
			return -ENOMEM;

		if (ex_fun->type == RWREG_OP_WRITE) {
			tmpbuf[0] = ex_fun->reg & 0xFF;
			for (i = 1; i < ex_fun->len; i++)
				tmpbuf[i] = shex_to_u8(buf + 5 + i * 2 - 2, 2);
		}
		ex_fun->opbuf = tmpbuf;
	}

	return ex_fun->len;
}

static void fts_rw_op_cleanup(struct fts_ex_fun_data *ex_fun)
{
	if (!ex_fun)
		return;

	kfree(ex_fun->opbuf);
	ex_fun->opbuf = NULL;
	ex_fun->dirty = false;
}

static ssize_t fts_rw_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct fts_ex_fun_data *ex_fun;
	ssize_t cmd_length = 0;
	int ret = 0;

	if (!ts_data || !ts_data->ex_fun)
		return -EINVAL;

	ex_fun = ts_data->ex_fun;

	mutex_lock(&ts_data->state_lock);
	cmd_length = count - 1;

	fts_rw_op_cleanup(ex_fun);
	ex_fun->len = fts_parse_buf(ex_fun, buf, cmd_length);

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 1);
#endif

	if (ex_fun->len < 0) {
		ret = ex_fun->len;
		goto tprwreg_store_done;
	}

	if (ex_fun->type == RWREG_OP_READ) {
		if (ex_fun->len == 1) {
			u8 reg = ex_fun->reg & 0xFF;
			u8 val = 0;

			ex_fun->res = fts_read_reg(ts_data, reg, &val);
			ex_fun->val = val;
			if (ex_fun->res == 0)
				ex_fun->res = 0;
		} else {
			u8 reg = ex_fun->reg & 0xFF;

			ex_fun->res = fts_read(ts_data, &reg, 1, ex_fun->opbuf, ex_fun->len);
			if (ex_fun->res >= 0) {
				ex_fun->res = 0;
				ex_fun->dirty = true;
			}
		}
	} else if (ex_fun->type == RWREG_OP_WRITE) {
		if (ex_fun->len == 1) {
			u8 reg = ex_fun->reg & 0xFF;
			u8 val = ex_fun->val & 0xFF;

			ex_fun->res = fts_write_reg(ts_data, reg, val);
		} else {
			ex_fun->res = fts_write(ts_data, ex_fun->opbuf, ex_fun->len);
		}

		if (ex_fun->res >= 0)
			ex_fun->res = 0;
	}

tprwreg_store_done:
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 0);
#endif
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_upgrade_bin_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	char fwname[FILE_NAME_LENGTH] = { 0 };

	if (!ts_data)
		return -EINVAL;

	if (count <= 1 || count >= FILE_NAME_LENGTH - 32)
		return -EINVAL;

	snprintf(fwname, FILE_NAME_LENGTH, "%s", buf);
	fwname[count - 1] = '\0';

	mutex_lock(&ts_data->state_lock);
	fts_upgrade_bin(ts_data, fwname, 0);
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_force_upgrade_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	char fwname[FILE_NAME_LENGTH] = { 0 };

	if (!ts_data)
		return -EINVAL;

	if (count <= 1 || count >= FILE_NAME_LENGTH - 32)
		return -EINVAL;

	snprintf(fwname, FILE_NAME_LENGTH, "%s", buf);
	fwname[count - 1] = '\0';

	mutex_lock(&ts_data->state_lock);
	fts_upgrade_bin(ts_data, fwname, 1);
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_driver_info_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	struct fts_ts_platform_data *pdata;
	int count = 0;

	if (!ts_data || !ts_data->pdata)
		return -EINVAL;

	pdata = ts_data->pdata;

	mutex_lock(&ts_data->state_lock);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Driver Ver:%s\n", FTS_DRIVER_VERSION);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Resolution:(%d,%d)~(%d,%d)\n",
			   pdata->x_min, pdata->y_min,
			   pdata->x_max, pdata->y_max);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Max Touchs:%d\n", pdata->max_touch_number);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "reset gpio:%d,int gpio:%d,irq:%d\n",
			   desc_to_gpio(pdata->reset_gpio),
			   desc_to_gpio(pdata->irq_gpio),
			   ts_data->irq);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "IC ID:0x%02x%02x\n",
			   ts_data->ic_info.ids.chip_idh,
			   ts_data->ic_info.ids.chip_idl);

	if (ts_data->bus_type == BUS_TYPE_I2C)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "BUS:%s,addr:0x%x\n",
				   "I2C", ts_data->client->addr);
	else
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "BUS:%s,mode:%d,max_freq:%d\n",
				   "SPI", ts_data->spi->mode,
				   ts_data->spi->max_speed_hz);

	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_dump_reg_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int count = 0;
	u8 val = 0;

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);

#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 1);
#endif
	fts_read_reg(ts_data, FTS_REG_POWER_MODE, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Power Mode:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_FW_VER, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "FW Ver:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_LIC_VER, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "LCD Initcode Ver:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_IDE_PARA_VER_ID, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Param Ver:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_IDE_PARA_STATUS, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Param status:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_VENDOR_ID, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Vendor ID:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_GESTURE_EN, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "Gesture Mode:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_CHARGER_MODE_EN, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "charge stat:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_INT_CNT, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "INT count:0x%02x\n", val);

	fts_read_reg(ts_data, FTS_REG_FLOW_WORK_CNT, &val);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "ESD count:0x%02x\n", val);
#if FTS_ESDCHECK_EN
	if (ts_data->esdcheck)
		fts_esdcheck_proc_busy(ts_data->esdcheck, 0);
#endif

	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_touch_point_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int count = 0;
	int i = 0;

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "touch point buffer:\n");
	for (i = 0; i < ts_data->pnt_buf_size; i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "%02x ", ts_data->point_buf[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_log_level_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int count = 0;

	if (!ts_data)
		return -EINVAL;

	mutex_lock(&ts_data->state_lock);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "log level:%d\n", ts_data->log_level);
	mutex_unlock(&ts_data->state_lock);

	return count;
}

static ssize_t fts_log_level_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);
	int ret;
	int value = 0;

	if (!ts_data)
		return -EINVAL;

	FTS_FUNC_ENTER();
	ret = kstrtoint(buf, 10, &value);
	if (ret)
		return ret;

	mutex_lock(&ts_data->state_lock);
	ts_data->log_level = value;
	mutex_unlock(&ts_data->state_lock);
	FTS_FUNC_EXIT();

	return count;
}

static DEVICE_ATTR_RO(fts_fw_version);
static DEVICE_ATTR_RW(fts_rw_reg);
static DEVICE_ATTR_WO(fts_upgrade_bin);
static DEVICE_ATTR_WO(fts_force_upgrade);
static DEVICE_ATTR_RO(fts_driver_info);
static DEVICE_ATTR_RO(fts_dump_reg);
static DEVICE_ATTR_RO(fts_hw_reset);
static DEVICE_ATTR_RW(fts_irq);
static DEVICE_ATTR_RW(fts_boot_mode);
static DEVICE_ATTR_RO(fts_touch_point);
static DEVICE_ATTR_RW(fts_log_level);

static struct attribute *fts_attributes[] = {
	&dev_attr_fts_fw_version.attr,
	&dev_attr_fts_rw_reg.attr,
	&dev_attr_fts_dump_reg.attr,
	&dev_attr_fts_upgrade_bin.attr,
	&dev_attr_fts_force_upgrade.attr,
	&dev_attr_fts_driver_info.attr,
	&dev_attr_fts_hw_reset.attr,
	&dev_attr_fts_irq.attr,
	&dev_attr_fts_boot_mode.attr,
	&dev_attr_fts_touch_point.attr,
	&dev_attr_fts_log_level.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

int fts_create_sysfs(struct fts_ts_data *ts_data)
{
	if (!ts_data || !ts_data->dev)
		return -EINVAL;

	if (sysfs_create_group(&ts_data->dev->kobj, &fts_attribute_group)) {
		FTS_ERROR("[EX]: sysfs_create_group() failed!!");
		return -ENOMEM;
	}

	return 0;
}

int fts_remove_sysfs(struct fts_ts_data *ts_data)
{
	if (!ts_data || !ts_data->dev)
		return -EINVAL;

	sysfs_remove_group(&ts_data->dev->kobj, &fts_attribute_group);

	if (ts_data->ex_fun)
		fts_rw_op_cleanup(ts_data->ex_fun);

	return 0;
}
