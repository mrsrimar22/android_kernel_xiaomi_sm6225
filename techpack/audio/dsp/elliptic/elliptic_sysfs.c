/**
 * Copyright Elliptic Labs
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <elliptic/elliptic_device.h>
#include <elliptic/elliptic_sysfs.h>
#include <elliptic/elliptic_mixer_controls.h>
#include "elliptic_version.h"

#define ELLIPTIC_DIAGNOSTICS_DATA_SECTION_COUNT 16
#define ELLIPTIC_CALIBRATION_MAX_DISPLAY_COUNT  96
#define ELLIPTIC_ML_DISPLAY_COUNT               16

extern struct elliptic_system_configuration_parameters_cache
	elliptic_system_configuration_cache;

static struct kobject *elliptic_sysfs_kobj;

static ssize_t calibration_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct elliptic_shared_data_block *calibration_obj;

	calibration_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_CALIBRATION_DATA);
	if (!calibration_obj) {
		EL_PRINT_E("calibration_obj is NULL");
		return -EINVAL;
	}

	if (count > calibration_obj->size) {
		EL_PRINT_E("write length %zu larger than buffer", count);
		return -EINVAL; /* Perbaikan: Cegah infinite loop user-space */
	}

	memcpy(calibration_obj->buffer, buf, count);
	return count;
}

static ssize_t calibration_v2_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct elliptic_shared_data_block *calibration_obj;

	calibration_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_CALIBRATION_V2_DATA);
	if (!calibration_obj) {
		EL_PRINT_E("calibration_obj is NULL");
		return -EINVAL;
	}

	if (count > calibration_obj->size) {
		EL_PRINT_E("write length %zu larger than buffer", count);
		return -EINVAL;
	}

	memcpy(calibration_obj->buffer, buf, count);
	return count;
}

static ssize_t diagnostics_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct elliptic_shared_data_block *diagnostics_obj;

	diagnostics_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_DIAGNOSTICS_DATA);
	if (!diagnostics_obj) {
		EL_PRINT_E("diagnostics_obj is NULL");
		return -EINVAL;
	}

	if (count > diagnostics_obj->size) {
		EL_PRINT_E("write length %zu larger than buffer", count);
		return -EINVAL;
	}

	memcpy(diagnostics_obj->buffer, buf, count);
	return count;
}

static ssize_t ml_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct elliptic_shared_data_block *ml_obj;

	ml_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_ML_DATA);
	if (!ml_obj) {
		EL_PRINT_E("ml_obj is NULL");
		return -EINVAL;
	}

	if (count > ml_obj->size) {
		EL_PRINT_E("write length %zu larger than buffer", count);
		return -EINVAL;
	}

	memcpy(ml_obj->buffer, buf, count);
	return count;
}

static ssize_t calibration_show_core(char *buf, int at, int pretty)
{
	int written = 0;
	int i;
	uint8_t *caldata;
	struct elliptic_shared_data_block *calibration_obj;

	calibration_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_CALIBRATION_DATA);
	if (!calibration_obj) {
		EL_PRINT_E("calibration_obj is NULL");
		return -EINVAL;
	}

	caldata = (uint8_t *)calibration_obj->buffer;

	if (pretty) {
		if (caldata[0] == 0xDE && caldata[1] == 0xAD) {
			written += sysfs_emit_at(buf, at + written,
						 "Calibration Data: not loaded");
		} else {
			written += sysfs_emit_at(buf, at + written,
						 "Calibration Data: ");
			for (i = 0; i < calibration_obj->size; ++i)
				written += sysfs_emit_at(buf, at + written,
							 "0x%02x ", caldata[i]);
		}
	} else {
		for (i = 0; i < calibration_obj->size; ++i)
			written += sysfs_emit_at(buf, at + written,
						 "0x%02x ", caldata[i]);
	}
	written += sysfs_emit_at(buf, at + written, "\n\n");
	return written;
}

static ssize_t calibration_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return calibration_show_core(buf, 0, 0);
}

static ssize_t calibration_v2_show_core(char *buf, int at, int pretty)
{
	int written = 0;
	int i;
	uint8_t *caldata;
	struct elliptic_shared_data_block *calibration_obj;

	calibration_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_CALIBRATION_V2_DATA);
	if (!calibration_obj) {
		EL_PRINT_E("calibration_obj is NULL");
		return -EINVAL;
	}

	caldata = (uint8_t *)calibration_obj->buffer;

	if (pretty) {
		if (caldata[0] == 0xDE && caldata[1] == 0xAD) {
			written += sysfs_emit_at(buf, at + written,
						 "Calibration Ext Data: not loaded");
		} else {
			int j = (ELLIPTIC_CALIBRATION_V2_DATA_SIZE >> 2) - 1;

			written += sysfs_emit_at(buf, at + written,
						 "Calibration Ext Data: ");
			for (i = 0; i < ELLIPTIC_CALIBRATION_MAX_DISPLAY_COUNT; ++i)
				written += sysfs_emit_at(buf, at + written,
							 "0x%02x ", caldata[i]);

			written += sysfs_emit_at(buf, at + written,
						 "\nTruncated at %d",
						 ELLIPTIC_CALIBRATION_MAX_DISPLAY_COUNT);

			if (j >= 7) {
				written += sysfs_emit_at(buf, at + written,
							 "\nmisc: %u %u %u %u %u %u %u %u\n",
							 caldata[j - 7], caldata[j - 6],
							 caldata[j - 5], caldata[j - 4],
							 caldata[j - 3], caldata[j - 2],
							 caldata[j - 1], caldata[j]);
			}
		}
	} else {
		for (i = 0; i < calibration_obj->size; ++i)
			written += sysfs_emit_at(buf, at + written,
						 "0x%02x ", caldata[i]);
	}
	written += sysfs_emit_at(buf, at + written, "\n\n");
	return written;
}

static ssize_t calibration_v2_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return calibration_v2_show_core(buf, 0, 0);
}

static ssize_t diagnostics_show_core(char *buf, int at, int pretty)
{
	int written = 0;
	uint32_t *data32;
	int i;
	struct elliptic_shared_data_block *diagnostics_obj;

	diagnostics_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_DIAGNOSTICS_DATA);
	if (!diagnostics_obj) {
		EL_PRINT_E("diagnostics_obj is NULL");
		return -EINVAL;
	}

	data32 = (uint32_t *)diagnostics_obj->buffer;

	if (pretty) {
		written += sysfs_emit_at(buf, at + written, "Diagnostics:\n  counters:\n");
		for (i = 0; i < ELLIPTIC_DIAGNOSTICS_DATA_SECTION_COUNT; ++i) {
			written += sysfs_emit_at(buf, at + written,
						 "   %u %u %u %u\n",
						 data32[4 * i], data32[4 * i + 1],
						 data32[4 * i + 2], data32[4 * i + 3]);
		}
	} else {
		for (i = 0; i < (diagnostics_obj->size >> 4); ++i) {
			written += sysfs_emit_at(buf, at + written,
						 "   %u %u %u %u\n",
						 data32[4 * i], data32[4 * i + 1],
						 data32[4 * i + 2], data32[4 * i + 3]);
		}
	}
	written += sysfs_emit_at(buf, at + written, "\n\n");
	return written;
}

static ssize_t diagnostics_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return diagnostics_show_core(buf, 0, 0);
}

static ssize_t ml_show_core(char *buf, int at, int pretty)
{
	int written = 0;
	int i, values;
	uint32_t *mldata;
	struct elliptic_shared_data_block *ml_obj;

	ml_obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_ML_DATA);
	if (!ml_obj) {
		EL_PRINT_E("ml_obj is NULL");
		return -EINVAL;
	}

	mldata = (uint32_t *)ml_obj->buffer;

	if (pretty) {
		if (mldata[0] == 0x0 && mldata[1] == 0x0) {
			written += sysfs_emit_at(buf, at + written,
						 "ML Data: not loaded");
		} else {
			written += sysfs_emit_at(buf, at + written, "ML Data: ");
			for (i = 0; i < ELLIPTIC_ML_DISPLAY_COUNT; ++i)
				written += sysfs_emit_at(buf, at + written,
							 "0x%08x ", mldata[i]);
			written += sysfs_emit_at(buf, at + written,
						 "\nTruncated at %d",
						 ELLIPTIC_ML_DISPLAY_COUNT);
		}
	} else {
		values = ml_obj->size >> 2;
		for (i = 0; i < values; ++i)
			written += sysfs_emit_at(buf, at + written,
						 "0x%08x ", mldata[i]);
	}
	written += sysfs_emit_at(buf, at + written, "\n\n");
	return written;
}

static ssize_t ml_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	return ml_show_core(buf, 0, 0);
}

static ssize_t version_show_core(char *buf, int at, int pretty)
{
	struct elliptic_engine_version_info *ver;
	struct elliptic_shared_data_block *obj;

	obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_VERSION_INFO);
	if (!obj) {
		EL_PRINT_E("version_obj is NULL");
		return -EINVAL;
	}

	ver = (struct elliptic_engine_version_info *)obj->buffer;

	if (pretty) {
		if (ver->major == 0xDE && ver->minor == 0xAD)
			return sysfs_emit_at(buf, at, "Version: unknown\n");
		return sysfs_emit_at(buf, at, "Version: %d.%d.%d.%d\n",
				     ver->major, ver->minor, ver->build, ver->revision);
	}
	return sysfs_emit_at(buf, at, "%d.%d.%d.%d\n",
			     ver->major, ver->minor, ver->build, ver->revision);
}

static ssize_t version_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return version_show_core(buf, 0, 0);
}

static ssize_t branch_show_core(char *buf, int at, int pretty)
{
	struct elliptic_shared_data_block *obj;

	obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_BRANCH_INFO);
	if (!obj) {
		EL_PRINT_E("branch_obj not found");
		return 0;
	}

	if (pretty)
		return sysfs_emit_at(buf, at, "Branch: %s\n", (const char *)obj->buffer);

	return sysfs_emit_at(buf, at, "%s\n", (const char *)obj->buffer);
}

static ssize_t branch_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return branch_show_core(buf, 0, 0);
}

static ssize_t tag_show_core(char *buf, int at, int pretty)
{
	struct elliptic_shared_data_block *obj;

	obj = elliptic_get_shared_obj(ELLIPTIC_OBJ_ID_TAG_INFO);
	if (!obj) {
		EL_PRINT_E("tag_obj not found");
		return 0;
	}

	if (pretty)
		return sysfs_emit_at(buf, at, "Tag: %s\n", (const char *)obj->buffer);

	return sysfs_emit_at(buf, at, "%s\n", (const char *)obj->buffer);
}

static ssize_t tag_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return tag_show_core(buf, 0, 0);
}

static ssize_t cache_show(char *buf, int at)
{
	struct elliptic_system_configuration_parameters_cache *cache =
		&elliptic_system_configuration_cache;
	int written = 0;

	written += sysfs_emit_at(buf, at + written, "Cache:\n");
	written += sysfs_emit_at(buf, at + written, "    mi:%d\n", cache->microphone_index);
	written += sysfs_emit_at(buf, at + written, "    om:%d\n", cache->operation_mode);
	written += sysfs_emit_at(buf, at + written, "    omf:%d\n", cache->operation_mode_flags);
	written += sysfs_emit_at(buf, at + written, "    cs:%d\n", cache->calibration_state);
	written += sysfs_emit_at(buf, at + written, "    cp:%d\n", cache->calibration_profile);
	written += sysfs_emit_at(buf, at + written, "    ug:%d\n", cache->ultrasound_gain);
	written += sysfs_emit_at(buf, at + written, "    ll:%d\n", cache->log_level);
	written += sysfs_emit_at(buf, at + written, "    es:%d\n", cache->engine_suspend);

	return written;
}

static ssize_t opmode_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", elliptic_system_configuration_cache.operation_mode);
}

static ssize_t opmode_flags_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", elliptic_system_configuration_cache.operation_mode_flags);
}

static ssize_t driver_version_show(char *buf, int at)
{
	return sysfs_emit_at(buf, at, "Driver version: %s-%s (%s)\n",
			     build_name, build_number, build_source_version);
}

static ssize_t state_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += driver_version_show(buf, len);
	len += version_show_core(buf, len, 1);
	len += branch_show_core(buf, len, 1);
	len += tag_show_core(buf, len, 1);
	len += calibration_show_core(buf, len, 1);
	len += calibration_v2_show_core(buf, len, 1);
	len += diagnostics_show_core(buf, len, 1);
	len += ml_show_core(buf, len, 1);
	len += cache_show(buf, len);

	return len;
}

static DEVICE_ATTR_RW(calibration);
static DEVICE_ATTR_RO(version);
static DEVICE_ATTR_RO(branch);
static DEVICE_ATTR_RW(calibration_v2);
static DEVICE_ATTR_RW(diagnostics);
static DEVICE_ATTR_RO(state);
static DEVICE_ATTR_RO(tag);
static DEVICE_ATTR_RW(ml);
static DEVICE_ATTR_RO(opmode);
static DEVICE_ATTR_RO(opmode_flags);

static struct attribute *elliptic_attrs[] = {
	&dev_attr_calibration.attr,
	&dev_attr_version.attr,
	&dev_attr_branch.attr,
	&dev_attr_calibration_v2.attr,
	&dev_attr_diagnostics.attr,
	&dev_attr_state.attr,
	&dev_attr_tag.attr,
	&dev_attr_ml.attr,
	&dev_attr_opmode.attr,
	&dev_attr_opmode_flags.attr,
	NULL,
};

static const struct attribute_group elliptic_attr_group = {
	.name = ELLIPTIC_SYSFS_ENGINE_FOLDER,
	.attrs = elliptic_attrs,
};

int elliptic_initialize_sysfs(void)
{
	int err;

	elliptic_sysfs_kobj = kobject_create_and_add(ELLIPTIC_SYSFS_ROOT_FOLDER,
						     kernel_kobj->parent);

	if (!elliptic_sysfs_kobj) {
		EL_PRINT_E("failed to create kobj");
		return -ENOMEM;
	}

	err = sysfs_create_group(elliptic_sysfs_kobj, &elliptic_attr_group);
	if (err) {
		EL_PRINT_E("failed to create sysfs group: %d", err);
		kobject_put(elliptic_sysfs_kobj);
		elliptic_sysfs_kobj = NULL;
		return err;
	}

	return 0;
}

void elliptic_cleanup_sysfs(void)
{
	if (!elliptic_sysfs_kobj)
		return;

	sysfs_remove_group(elliptic_sysfs_kobj, &elliptic_attr_group);
	kobject_put(elliptic_sysfs_kobj);
	elliptic_sysfs_kobj = NULL;
}
