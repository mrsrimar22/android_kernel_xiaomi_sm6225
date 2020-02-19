/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __TP_COMMON_H__
#define __TP_COMMON_H__

#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>

enum tp_feature {
	TP_FEATURE_CAPACITIVE_KEYS,
	TP_FEATURE_DOUBLE_TAP,
	TP_FEATURE_REVERSED_KEYS,
	TP_FEATURE_MAX,
};

struct tp_feature_entry;

struct tp_feature_ops {
	ssize_t (*show)(struct tp_feature_entry *entry, char *buf);
	ssize_t (*store)(struct tp_feature_entry *entry, const char *buf, size_t count);
};

/**
 * struct tp_feature_entry - Objek fitur yang di-embed ke struct consumer
 * @feature: ID fitur (menentukan nama node sysfs)
 * @ops: Callback show/store milik consumer
 * @kattr: Internal sysfs attribute
 */
struct tp_feature_entry {
	enum tp_feature feature;
	const struct tp_feature_ops *ops;
	struct kobj_attribute kattr;
};

int tp_common_register_feature(struct tp_feature_entry *entry);
void tp_common_unregister_feature(struct tp_feature_entry *entry);

#endif /* __TP_COMMON_H__ */
