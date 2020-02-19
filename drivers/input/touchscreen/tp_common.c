// SPDX-License-Identifier: GPL-2.0-only
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/input/tp_common.h>

static struct kobject *touchpanel_kobj;

static const char * const tp_feature_names[TP_FEATURE_MAX] = {
	[TP_FEATURE_CAPACITIVE_KEYS] = "capacitive_keys",
	[TP_FEATURE_DOUBLE_TAP]      = "double_tap",
	[TP_FEATURE_REVERSED_KEYS]   = "reversed_keys",
};

static ssize_t tp_feature_sysfs_show(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     char *buf)
{
	struct tp_feature_entry *entry = container_of(attr,
						      struct tp_feature_entry,
						      kattr);

	if (entry->ops && entry->ops->show)
		return entry->ops->show(entry, buf);

	return -EOPNOTSUPP;
}

static ssize_t tp_feature_sysfs_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	struct tp_feature_entry *entry = container_of(attr,
						      struct tp_feature_entry,
						      kattr);

	if (entry->ops && entry->ops->store)
		return entry->ops->store(entry, buf, count);

	return -EOPNOTSUPP;
}

int tp_common_register_feature(struct tp_feature_entry *entry)
{
	if (!entry || entry->feature >= TP_FEATURE_MAX || !entry->ops)
		return -EINVAL;

	if (!touchpanel_kobj)
		return -ENODEV;

	sysfs_attr_init(&entry->kattr.attr);
	entry->kattr.attr.name = tp_feature_names[entry->feature];
	entry->kattr.attr.mode = 0644; /* S_IRUGO | S_IWUSR */
	entry->kattr.show = entry->ops->show ? tp_feature_sysfs_show : NULL;
	entry->kattr.store = entry->ops->store ? tp_feature_sysfs_store : NULL;

	return sysfs_create_file(touchpanel_kobj, &entry->kattr.attr);
}
EXPORT_SYMBOL_GPL(tp_common_register_feature);

void tp_common_unregister_feature(struct tp_feature_entry *entry)
{
	if (!entry || !touchpanel_kobj)
		return;

	sysfs_remove_file(touchpanel_kobj, &entry->kattr.attr);
}
EXPORT_SYMBOL_GPL(tp_common_unregister_feature);

static int __init tp_common_init(void)
{
	touchpanel_kobj = kobject_create_and_add("touchpanel", NULL);
	if (!touchpanel_kobj)
		return -ENOMEM;

	return 0;
}

static void __exit tp_common_exit(void)
{
	if (touchpanel_kobj) {
		kobject_del(touchpanel_kobj);
		kobject_put(touchpanel_kobj);
		touchpanel_kobj = NULL;
	}
}

core_initcall(tp_common_init);
module_exit(tp_common_exit);

MODULE_DESCRIPTION("Touchpanel Common Sysfs Core");
MODULE_LICENSE("GPL");
