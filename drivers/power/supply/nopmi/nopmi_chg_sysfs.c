// SPDX-License-Identifier: GPL-2.0-only
/*
 * nopmi_chg_sysfs - qcom-battery sysfs class sub-module
 */
#define pr_fmt(fmt) "[nopmi_chg_sysfs]: %s: " fmt, __func__

#include <linux/slab.h>
#ifdef CONFIG_TCPC_CLASS
#include "inc/tcpci.h"
#endif

#include "nopmi_chg.h"
#include "nopmi_chg_sysfs.h"

static const char * const nopmi_usb_type_text[] = {
	[POWER_SUPPLY_TYPE_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_TYPE_BATTERY]		= "Battery",
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	[POWER_SUPPLY_TYPE_BATTERY_VERIFY]	= "Battery_Verify",
#endif
	[POWER_SUPPLY_TYPE_UPS]			= "UPS",
	[POWER_SUPPLY_TYPE_MAINS]		= "Mains",
	[POWER_SUPPLY_TYPE_USB]			= "USB",
	[POWER_SUPPLY_TYPE_USB_DCP]		= "USB_DCP",
	[POWER_SUPPLY_TYPE_USB_CDP]		= "USB_CDP",
	[POWER_SUPPLY_TYPE_USB_ACA]		= "USB_ACA",
	[POWER_SUPPLY_TYPE_USB_TYPE_C]		= "USB_C",
	[POWER_SUPPLY_TYPE_USB_PD]		= "USB_PD",
	[POWER_SUPPLY_TYPE_USB_PD_DRP]		= "USB_PD_DRP",
	[POWER_SUPPLY_TYPE_APPLE_BRICK_ID]	= "BrickID",
	[POWER_SUPPLY_TYPE_USB_HVDCP]		= "USB_HVDCP",
	[POWER_SUPPLY_TYPE_USB_HVDCP_3]		= "USB_HVDCP_3",
	[POWER_SUPPLY_TYPE_USB_HVDCP_3P5]	= "USB_HVDCP_3P5",
	[POWER_SUPPLY_TYPE_WIRELESS]		= "Wireless",
	[POWER_SUPPLY_TYPE_USB_FLOAT]		= "USB_FLOAT",
	[POWER_SUPPLY_TYPE_BMS]			= "BMS",
	[POWER_SUPPLY_TYPE_PARALLEL]		= "Parallel",
	[POWER_SUPPLY_TYPE_MAIN]		= "Main",
	[POWER_SUPPLY_TYPE_WIPOWER]		= "Wipower",
	[POWER_SUPPLY_TYPE_UFP]			= "USB_C_UFP",
	[POWER_SUPPLY_TYPE_DFP]			= "USB_C_DFP",
	[POWER_SUPPLY_TYPE_CHARGE_PUMP]		= "Charge_Pump",
};

static const char * const nopmi_usbc_text[] = {
	[POWER_SUPPLY_TYPEC_NONE]			= "Nothing attached",
	[POWER_SUPPLY_TYPEC_SINK]			= "Sink attached",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]		= "Powered cable w/ sink",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY]	= "Debug Accessory",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]		= "Audio Adapter",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]		= "Powered cable w/o sink",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]		= "Source attached (default current)",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]		= "Source attached (medium current)",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]		= "Source attached (high current)",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]		= "Non compliant",
};

static inline struct nopmi_chg *class_to_nopmi_chg(struct class *c)
{
	struct nopmi_chg_sysfs *sysfs =
		container_of(c, struct nopmi_chg_sysfs, battery_class);

	return sysfs->parent;
}

static const char *nopmi_get_usb_type_name(u32 type)
{
	if (type < ARRAY_SIZE(nopmi_usb_type_text) &&
	    nopmi_usb_type_text[type])
		return nopmi_usb_type_text[type];

	return "Unknown";
}

static const char *nopmi_get_usbc_text_name(u32 mode)
{
	if (mode < ARRAY_SIZE(nopmi_usbc_text) &&
	    nopmi_usbc_text[mode])
		return nopmi_usbc_text[mode];

	return "Unknown";
}

static int nopmi_read_psy_int(struct power_supply *psy,
			      enum power_supply_property psp,
			      int *out)
{
	union power_supply_propval pval = {0, };
	int ret;

	if (!psy)
		return -ENODEV;

	ret = power_supply_get_property(psy, psp, &pval);
	if (ret < 0)
		return ret;

	*out = pval.intval;
	return 0;
}

static int nopmi_write_psy_int(struct power_supply *psy,
			       enum power_supply_property psp,
			       int val)
{
	union power_supply_propval pval = { .intval = val };

	if (!psy)
		return -ENODEV;

	return power_supply_set_property(psy, psp, &pval);
}

/*
 * Class attributes: /sys/class/qcom-battery/<attr>
 */

enum nopmi_psy_id {
	NOPMI_PSY_BMS,
	NOPMI_PSY_BATT,
	NOPMI_PSY_USB,
};

struct nopmi_psy_class_attr {
	struct class_attribute attr;
	enum nopmi_psy_id psy_id;
	enum power_supply_property psp;
};

static struct power_supply *nopmi_get_psy(struct nopmi_chg *chg,
					  enum nopmi_psy_id psy_id)
{
	switch (psy_id) {
	case NOPMI_PSY_BMS:
		return chg->bms_psy;
	case NOPMI_PSY_BATT:
		return chg->batt_psy;
	case NOPMI_PSY_USB:
		return chg->usb_psy;
	default:
		return NULL;
	}
}

static ssize_t nopmi_psy_int_show(struct class *c,
				  struct class_attribute *attr,
				  char *buf)
{
	struct nopmi_psy_class_attr *nattr =
		container_of(attr, struct nopmi_psy_class_attr, attr);
	struct nopmi_chg *chg = class_to_nopmi_chg(c);
	int val;

	if (!chg)
		return -ENODEV;

	if (nopmi_read_psy_int(nopmi_get_psy(chg, nattr->psy_id),
			       nattr->psp, &val) < 0)
		return -ENODATA;

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t nopmi_psy_int_store(struct class *c,
				   struct class_attribute *attr,
				   const char *buf, size_t len)
{
	struct nopmi_psy_class_attr *nattr =
		container_of(attr, struct nopmi_psy_class_attr, attr);
	struct nopmi_chg *chg = class_to_nopmi_chg(c);
	int val, ret;

	if (!chg)
		return -ENODEV;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	ret = nopmi_write_psy_int(nopmi_get_psy(chg, nattr->psy_id),
				  nattr->psp, val);
	return ret < 0 ? ret : len;
}

#define NOPMI_CLASS_ATTR_PSY_INT_RO(_name, _psy_id, _psp)		\
static struct nopmi_psy_class_attr nopmi_attr_##_name = {		\
	.attr = __ATTR(_name, 0444, nopmi_psy_int_show, NULL),		\
	.psy_id = (_psy_id),						\
	.psp = (_psp),							\
}

#define NOPMI_CLASS_ATTR_PSY_INT_RW(_name, _psy_id, _psp)		\
static struct nopmi_psy_class_attr nopmi_attr_##_name = {		\
	.attr = __ATTR(_name, 0644, nopmi_psy_int_show,			\
		       nopmi_psy_int_store),				\
	.psy_id = (_psy_id),						\
	.psp = (_psp),							\
}

NOPMI_CLASS_ATTR_PSY_INT_RO(shutdown_delay, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_SHUTDOWN_DELAY);
NOPMI_CLASS_ATTR_PSY_INT_RO(cycle_count, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_CYCLE_COUNT);
NOPMI_CLASS_ATTR_PSY_INT_RO(resistance_id, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_RESISTANCE_ID);
NOPMI_CLASS_ATTR_PSY_INT_RO(fg_batt_id, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_RESISTANCE_ID);
NOPMI_CLASS_ATTR_PSY_INT_RO(soc, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_CAPACITY);
NOPMI_CLASS_ATTR_PSY_INT_RO(soc_decimal, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_SOC_DECIMAL);
NOPMI_CLASS_ATTR_PSY_INT_RO(soc_decimal_rate, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_SOC_DECIMAL_RATE);
NOPMI_CLASS_ATTR_PSY_INT_RO(authentic, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_AUTHENTIC);
NOPMI_CLASS_ATTR_PSY_INT_RO(chip_ok, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_CHIP_OK);
NOPMI_CLASS_ATTR_PSY_INT_RO(fastcharge_mode, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_FASTCHARGE_MODE);
NOPMI_CLASS_ATTR_PSY_INT_RO(soh, NOPMI_PSY_BMS,
			    POWER_SUPPLY_PROP_SOH);
NOPMI_CLASS_ATTR_PSY_INT_RO(typec_cc_orientation, NOPMI_PSY_USB,
			    POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION);

static ssize_t usb_real_type_show(struct class *c,
				  struct class_attribute *attr,
				  char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);
	int val;

	if (!chg)
		return -ENODEV;

	if (nopmi_read_psy_int(chg->usb_psy,
			       POWER_SUPPLY_PROP_REAL_TYPE, &val) < 0)
		return -ENODATA;

	return scnprintf(buf, PAGE_SIZE, "%s\n", nopmi_get_usb_type_name(val));
}
static CLASS_ATTR_RO(usb_real_type);

static ssize_t real_type_show(struct class *c,
			      struct class_attribute *attr,
			      char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);
	int val;

	if (!chg)
		return -ENODEV;

	if (nopmi_read_psy_int(chg->usb_psy,
			       POWER_SUPPLY_PROP_REAL_TYPE, &val) < 0)
		return -ENODATA;

	return scnprintf(buf, PAGE_SIZE, "%s\n", nopmi_get_usb_type_name(val));
}
static CLASS_ATTR_RO(real_type);

static ssize_t typec_mode_show(struct class *c,
			       struct class_attribute *attr,
			       char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);
	int val;

	if (!chg)
		return -ENODEV;

	if (nopmi_read_psy_int(chg->usb_psy,
			       POWER_SUPPLY_PROP_TYPEC_MODE, &val) < 0)
		return -ENODATA;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 nopmi_get_usbc_text_name(val));
}
static CLASS_ATTR_RO(typec_mode);

static ssize_t quick_charge_type_show(struct class *c,
				      struct class_attribute *attr,
				      char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);

	if (!chg)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 nopmi_get_quick_charge_type(chg));
}
static CLASS_ATTR_RO(quick_charge_type);

static ssize_t usbpd_verified_show(struct class *c,
				   struct class_attribute *attr,
				   char *buf)
{
	int verified = 1;

	switch (nopmi_get_charger_ic_type()) {
	case NOPMI_CHARGER_IC_MAXIM:
		verified = max77729_usbc_is_pd_verified();
		break;
	case NOPMI_CHARGER_IC_SYV:
		verified = adapter_dev_get_pd_verified();
		break;
	default:
		break;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", verified);
}
static CLASS_ATTR_RO(usbpd_verified);

NOPMI_CLASS_ATTR_PSY_INT_RW(mtbf_current, NOPMI_PSY_USB,
			    POWER_SUPPLY_PROP_MTBF_CURRENT);
NOPMI_CLASS_ATTR_PSY_INT_RW(input_suspend, NOPMI_PSY_BATT,
			    POWER_SUPPLY_PROP_INPUT_SUSPEND);
NOPMI_CLASS_ATTR_PSY_INT_RW(thermal_level, NOPMI_PSY_BATT,
			    POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL);

#define CP_CHIP_VENDOR_SC8551		8551

static int nopmi_get_cp_vendor(struct nopmi_chg *chg)
{
	int val = 0;

	if (nopmi_read_psy_int(chg->cp_psy,
			       POWER_SUPPLY_PROP_SC_CHIP_VENDOR, &val) < 0)
		return 0;

	return val;
}

static ssize_t cp_model_name_show(struct class *c,
				  struct class_attribute *attr,
				  char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);

	if (!chg)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 nopmi_get_cp_vendor(chg) == CP_CHIP_VENDOR_SC8551 ?
			 "sc8551" : "Unknown");
}
static CLASS_ATTR_RO(cp_model_name);

static ssize_t cp_manufacturer_show(struct class *c,
				    struct class_attribute *attr,
				    char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);

	if (!chg)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 nopmi_get_cp_vendor(chg) == CP_CHIP_VENDOR_SC8551 ?
			 "Southchip" : "Unknown");
}
static CLASS_ATTR_RO(cp_manufacturer);

static bool nopmi_is_ds28e16(void)
{
#ifdef CONFIG_BATT_VERIFY_BY_DS28E16
	struct power_supply *verify_psy;
	union power_supply_propval pval = {0, };
	bool matched = false;

	verify_psy = power_supply_get_by_name("batt_verify");
	if (!verify_psy)
		return false;

	if (power_supply_get_property(verify_psy,
				      POWER_SUPPLY_PROP_VERIFY_MODEL_NAME,
				      &pval) == 0 && pval.strval)
		matched = !strcmp(pval.strval, "ds28e16");

	power_supply_put(verify_psy);
	return matched;
#else
	return false;
#endif
}

static ssize_t ds_model_name_show(struct class *c,
				  struct class_attribute *attr,
				  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 nopmi_is_ds28e16() ? "ds28e16" : "Unknown");
}
static CLASS_ATTR_RO(ds_model_name);

static ssize_t ds_manufacturer_show(struct class *c,
				    struct class_attribute *attr,
				    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 nopmi_is_ds28e16() ? "Maxim" : "Unknown");
}
static CLASS_ATTR_RO(ds_manufacturer);

#define TCPC_VID_RICHTEK		0x29cf
#define TCPC_PID_RT1711			0x1711
#define TCPC_DID_RT1711H_A		0x2170
#define TCPC_DID_RT1711H_B		0x2171
#define TCPC_DID_RT1711H_C		0x2172
#define TCPC_DID_RT1715_D		0x2173
#define TCPC_VID_HUSB311		0x2e99
#define TCPC_PID_HUSB311		0x0311
#define TCPC_DID_HUSB311		0x0000

static void nopmi_get_cc_info(const char **model, const char **mfr)
{
	*model = "Unknown";
	*mfr = "Unknown";

	if (nopmi_is_syv_ic()) {
#ifdef CONFIG_TCPC_CLASS
		struct tcpc_device *tcpc;
		u16 vid, pid, did;

		tcpc = tcpc_dev_get_by_name("type_c_port0");
		if (!tcpc)
			return;

		vid = tcpc->desc.vid;
		pid = tcpc->desc.pid;
		did = tcpc->desc.did;
		put_device(&tcpc->dev);

		if (vid == TCPC_VID_RICHTEK && pid == TCPC_PID_RT1711) {
			switch (did) {
			case TCPC_DID_RT1711H_A:
			case TCPC_DID_RT1711H_B:
			case TCPC_DID_RT1711H_C:
				*model = "rt1711h";
				*mfr = "Richtek";
				break;
			case TCPC_DID_RT1715_D:
				*model = "rt1715";
				*mfr = "Richtek";
				break;
			default:
				break;
			}
		} else if (vid == TCPC_VID_HUSB311 &&
			   pid == TCPC_PID_HUSB311 &&
			   did == TCPC_DID_HUSB311) {
			*model = "husb311";
			*mfr = "Hynetek";
		}
#endif
	} else if (nopmi_is_maxim_ic()) {
		if (max77729_usbc_is_valid() < 0)
			return;

		*model = "max77729";
		*mfr = "Maxim";
	}
}

static ssize_t cc_model_name_show(struct class *c,
				  struct class_attribute *attr,
				  char *buf)
{
	const char *model, *mfr;

	nopmi_get_cc_info(&model, &mfr);
	return scnprintf(buf, PAGE_SIZE, "%s\n", model);
}
static CLASS_ATTR_RO(cc_model_name);

static ssize_t cc_manufacturer_show(struct class *c,
				    struct class_attribute *attr,
				    char *buf)
{
	const char *model, *mfr;

	nopmi_get_cc_info(&model, &mfr);
	return scnprintf(buf, PAGE_SIZE, "%s\n", mfr);
}
static CLASS_ATTR_RO(cc_manufacturer);

static void nopmi_get_batt_info(struct nopmi_chg *chg,
				const char **model,
				const char **mfr)
{
	union power_supply_propval pval = {0, };

	*model = "M376-unknown-5000mAh";
	*mfr = "Unknown";

	if (!chg || !chg->bms_psy)
		return;

	if (power_supply_get_property(chg->bms_psy,
				      POWER_SUPPLY_PROP_MODEL_NAME,
				      &pval) < 0 || !pval.strval)
		return;

	*model = pval.strval;
	if (!strcmp(pval.strval, "M376-NVT-5000mAh"))
		*mfr = "Novatek";
	else if (!strcmp(pval.strval, "M376-GuanYu-5000mAh"))
		*mfr = "GuanYu";
	else if (!strcmp(pval.strval, "M376-Sunwoda-5000mAh"))
		*mfr = "Sunwoda";
}

static ssize_t batt_model_name_show(struct class *c,
				    struct class_attribute *attr,
				    char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);
	const char *model, *mfr;

	if (!chg)
		return -ENODEV;

	nopmi_get_batt_info(chg, &model, &mfr);
	return scnprintf(buf, PAGE_SIZE, "%s\n", model);
}
static CLASS_ATTR_RO(batt_model_name);

static ssize_t batt_manufacturer_show(struct class *c,
				      struct class_attribute *attr,
				      char *buf)
{
	struct nopmi_chg *chg = class_to_nopmi_chg(c);
	const char *model, *mfr;

	if (!chg)
		return -ENODEV;

	nopmi_get_batt_info(chg, &model, &mfr);
	return scnprintf(buf, PAGE_SIZE, "%s\n", mfr);
}
static CLASS_ATTR_RO(batt_manufacturer);

static struct attribute *battery_class_attrs[] = {
	/* RO (0444) */
	&class_attr_usb_real_type.attr,
	&class_attr_real_type.attr,
	&nopmi_attr_shutdown_delay.attr.attr,
	&nopmi_attr_cycle_count.attr.attr,
	&nopmi_attr_resistance_id.attr.attr,
	&nopmi_attr_fg_batt_id.attr.attr,
	&nopmi_attr_soc.attr.attr,
	&nopmi_attr_soc_decimal.attr.attr,
	&nopmi_attr_soc_decimal_rate.attr.attr,
	&nopmi_attr_typec_cc_orientation.attr.attr,
	&class_attr_typec_mode.attr,
	&class_attr_quick_charge_type.attr,
	&nopmi_attr_authentic.attr.attr,
	&nopmi_attr_chip_ok.attr.attr,
	&class_attr_usbpd_verified.attr,
	&nopmi_attr_fastcharge_mode.attr.attr,
	&nopmi_attr_soh.attr.attr,
	&class_attr_cp_model_name.attr,
	&class_attr_cp_manufacturer.attr,
	&class_attr_cc_model_name.attr,
	&class_attr_cc_manufacturer.attr,
	&class_attr_ds_model_name.attr,
	&class_attr_ds_manufacturer.attr,
	&class_attr_batt_model_name.attr,
	&class_attr_batt_manufacturer.attr,
	/* RW (0644) */
	&nopmi_attr_mtbf_current.attr.attr,
	&nopmi_attr_input_suspend.attr.attr,
	&nopmi_attr_thermal_level.attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(battery_class);

struct nopmi_chg_sysfs *nopmi_chg_sysfs_init(struct nopmi_chg *chg)
{
	struct nopmi_chg_sysfs *sysfs;
	int ret;

	if (!chg || !chg->dev)
		return ERR_PTR(-EINVAL);

	sysfs = kzalloc(sizeof(*sysfs), GFP_KERNEL);
	if (!sysfs)
		return ERR_PTR(-ENOMEM);

	sysfs->parent = chg;
	sysfs->battery_class.name = "qcom-battery";
	sysfs->battery_class.owner = THIS_MODULE;
	sysfs->battery_class.class_groups = battery_class_groups;

	ret = class_register(&sysfs->battery_class);
	if (ret < 0) {
		pr_err("failed to register qcom-battery class, ret=%d\n", ret);
		kfree(sysfs);
		return ERR_PTR(ret);
	}

	return sysfs;
}

void nopmi_chg_sysfs_deinit(struct nopmi_chg_sysfs *sysfs)
{
	if (IS_ERR_OR_NULL(sysfs))
		return;

	class_unregister(&sysfs->battery_class);
	kfree(sysfs);
}
