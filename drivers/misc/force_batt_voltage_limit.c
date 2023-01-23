/*
 * Possible values for "force_batt_voltage_limit" are :
 *
 *   0 - Disabled (default)
 *   1 - Limit battery to 4.2mV
*/

#include <linux/module.h>

#include <linux/fastchg.h>

int force_batt_voltage_limit = 1;

static int __init get_batt_voltage_limit_opt(char *ffc)
{
	if (!strcmp(ffc, "1"))
		force_batt_voltage_limit = 1;
	else
		force_batt_voltage_limit = 0;

	return 1;
}

__setup("ffc=", get_batt_voltage_limit_opt);

static ssize_t force_batt_voltage_limit_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", force_batt_voltage_limit);
	return count;
}

static ssize_t force_batt_voltage_limit_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int temp;
	sscanf(buf, "%d ", &temp);
	force_batt_voltage_limit = !!temp;
	return count;
}

static struct kobj_attribute force_batt_voltage_limit_attribute =
	__ATTR(force_batt_voltage_limit, 0664, force_batt_voltage_limit_show, force_batt_voltage_limit_store);

static struct attribute *force_batt_voltage_limit_attrs[] = {
	&force_batt_voltage_limit_attribute.attr,
	NULL
};

static struct attribute_group force_batt_voltage_limit_attr_group = {
	.attrs = force_batt_voltage_limit_attrs
};

static int force_batt_voltage_limit_init(void)
{
	struct kobject *kobj;
	int ret = 0;

	kobj = kobject_create_and_add("force_batt_voltage_limit", kernel_kobj);
	if (!kobj)
		return -ENOMEM;

	ret = sysfs_create_group(kobj, &force_batt_voltage_limit_attr_group);
	if (ret)
		kobject_put(kobj);

	return ret;
}

module_init(force_batt_voltage_limit_init);
