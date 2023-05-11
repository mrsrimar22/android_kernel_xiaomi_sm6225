#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <linux/power_supply.h>
#include <linux/stat.h>
#include <linux/ctype.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/vmalloc.h>
#include <linux/preempt.h>
#include "inc/xm_adapter_class.h"
#include "inc/tcpm.h"

#define PROBE_CNT_MAX	50
static int log_level = 0;
int get_apdo_regain;

#define adapter_err(fmt, ...)						\
do {									\
	printk(KERN_ERR "[xm_pd_adapter]: " fmt, ##__VA_ARGS__);	\
} while (0)

#define adapter_info(fmt, ...)							\
do {										\
	if (log_level >= 1)							\
		printk(KERN_ERR "[xm_pd_adapter]: " fmt, ##__VA_ARGS__);	\
	else									\
		printk(KERN_INFO "[xm_pd_adapter]: " fmt, ##__VA_ARGS__);	\
} while (0)

#define adapter_dbg(fmt, ...)							\
do {										\
	if (log_level >= 2)							\
		printk(KERN_ERR "[xm_pd_adapter]: " fmt, ##__VA_ARGS__);	\
	else									\
		printk(KERN_DEBUG "[xm_pd_adapter]: " fmt, ##__VA_ARGS__);	\
} while (0)

struct xm_pd_adapter_info {
	struct tcpc_device *tcpc;
	struct notifier_block pd_nb;
	struct adapter_device *adapter_dev;
	struct task_struct *adapter_task;
	const char *adapter_dev_name;
	bool enable_kpoc_shdn;
	struct tcpm_svid_list *adapter_svid_list;
	struct adapter_device *pd_adapter;
};

static void usbpd_mi_vdm_received(struct xm_pd_adapter_info *pinfo, struct tcp_ny_uvdm uvdm)
{
	int i, cmd;

	if (uvdm.uvdm_svid != USB_PD_MI_SVID)
		return;

	cmd = UVDM_HDR_CMD(uvdm.uvdm_data[0]);
	adapter_info("%s: cmd = %d\n", __func__, cmd);

	adapter_info("%s: uvdm.ack: %d, uvdm.uvdm_cnt: %d, uvdm.uvdm_svid: 0x%04x\n",
			__func__, uvdm.ack, uvdm.uvdm_cnt, uvdm.uvdm_svid);

	switch (cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
		pinfo->pd_adapter->vdm_data.ta_version = uvdm.uvdm_data[1];
		adapter_info("%s: ta_version: %x\n", __func__, pinfo->pd_adapter->vdm_data.ta_version);
		break;
	case USBPD_UVDM_CHARGER_TEMP:
		pinfo->pd_adapter->vdm_data.ta_temp = (uvdm.uvdm_data[1] & 0xFFFF) * 10;
		adapter_info("%s: pinfo->pd_adapter->vdm_data.ta_temp: %d\n", __func__, pinfo->pd_adapter->vdm_data.ta_temp);
		break;
	case USBPD_UVDM_CHARGER_VOLTAGE:
		pinfo->pd_adapter->vdm_data.ta_voltage = (uvdm.uvdm_data[1] & 0xFFFF) * 10;
		pinfo->pd_adapter->vdm_data.ta_voltage *= 1000; /*V->mV*/
		adapter_info("%s: ta_voltage: %d\n", __func__, pinfo->pd_adapter->vdm_data.ta_voltage);
		break;
	case USBPD_UVDM_SESSION_SEED:
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++) {
			pinfo->pd_adapter->vdm_data.s_secert[i] = uvdm.uvdm_data[i+1];
			adapter_info("%s: usbpd s_secert uvdm.uvdm_data[%d]=0x%x\n", __func__, i+1, uvdm.uvdm_data[i+1]);
		}
		break;
	case USBPD_UVDM_AUTHENTICATION:
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++) {
			pinfo->pd_adapter->vdm_data.digest[i] = uvdm.uvdm_data[i+1];
			adapter_info("%s: usbpd digest[%d]=0x%x\n", __func__, i+1, uvdm.uvdm_data[i+1]);
		}
		break;
	case USBPD_UVDM_REVERSE_AUTHEN:
		pinfo->pd_adapter->vdm_data.reauth = (uvdm.uvdm_data[1] & 0xFFFF);
		break;
	default:
		break;
	}
	pinfo->pd_adapter->uvdm_state = cmd;
}

static int pd_tcp_notifier_call(struct notifier_block *pnb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct xm_pd_adapter_info *pinfo;

	pinfo = container_of(pnb, struct xm_pd_adapter_info, pd_nb);

	adapter_dbg("%s: PD charger event: %d %d\n", __func__,
		(int)event, (int)noti->pd_state.connected);
	switch (event) {
	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case  PD_CONNECT_NONE:
			pinfo->adapter_dev->adapter_id = 0;
			pinfo->adapter_dev->adapter_svid = 0;
			pinfo->adapter_dev->uvdm_state = USBPD_UVDM_DISCONNECT;
			pinfo->adapter_dev->verifed = 0;
			pinfo->adapter_dev->verify_process = 0;
			break;
		case PD_CONNECT_PE_READY_SNK_PD30:
			pinfo->adapter_dev->uvdm_state = USBPD_UVDM_CONNECT;
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			pinfo->adapter_dev->uvdm_state = USBPD_UVDM_CONNECT;
			get_apdo_regain = 1;
			break;
		};
		break;
	case TCP_NOTIFY_UVDM:
		adapter_info("%s: tcpc received uvdm message.\n", __func__);
		usbpd_mi_vdm_received(pinfo, noti->uvdm_msg);
		break;
	}
	return NOTIFY_OK;
}

static int pd_get_svid(struct adapter_device *dev)
{
	struct xm_pd_adapter_info *info;
	struct pd_source_cap_ext cap_ext;
	int ret;
	int i = 0;
	uint32_t pd_vdos[8];

	info = (struct xm_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL)
		return -EINVAL;

	adapter_info("%s: entry\n", __func__);
	if (info->adapter_dev->adapter_svid != 0)
		return 0;

	if (info->adapter_svid_list == NULL) {
		if (in_interrupt())
			info->adapter_svid_list = kmalloc(sizeof(struct tcpm_svid_list), GFP_ATOMIC);
		else
			info->adapter_svid_list = kmalloc(sizeof(struct tcpm_svid_list), GFP_KERNEL);

		if (info->adapter_svid_list == NULL) {
			adapter_err("%s: adapter_svid_list is still NULL!\n", __func__);
			return -ENOMEM;
		}
	}

	ret = tcpm_inquire_pd_partner_inform(info->tcpc, pd_vdos);
	if (ret == TCPM_SUCCESS) {
		adapter_info("%s: find adapter id success.\n", __func__);
		for (i = 0; i < 8; i++)
			adapter_info("%s: VDO[%d]:%08x\n", __func__, i, pd_vdos[i]);

		info->adapter_dev->adapter_svid = pd_vdos[0] & 0x0000FFFF;
		info->adapter_dev->adapter_id = pd_vdos[2] & 0x0000FFFF;
		adapter_info("%s: adapter_svid = %04x\n", __func__, info->adapter_dev->adapter_svid);
		adapter_info("%s: adapter_id = %08x\n", __func__, info->adapter_dev->adapter_id);

		ret = tcpm_inquire_pd_partner_svids(info->tcpc, info->adapter_svid_list);
		adapter_info("%s: tcpm_inquire_pd_partner_svids, ret=%d!\n", __func__, ret);
		if (ret == TCPM_SUCCESS) {
			adapter_info("%s: discover svid number is %d\n", __func__, info->adapter_svid_list->cnt);
			for (i = 0; i < info->adapter_svid_list->cnt; i++) {
				adapter_info("%s: SVID[%d]:%04x\n", __func__, i, info->adapter_svid_list->svids[i]);
				if (info->adapter_svid_list->svids[i] == USB_PD_MI_SVID)
					info->adapter_dev->adapter_svid = USB_PD_MI_SVID;
			}
		}
	} else {
		ret = tcpm_dpm_pd_get_source_cap_ext(info->tcpc,
			NULL, &cap_ext);
		if (ret == TCPM_SUCCESS) {
			info->adapter_dev->adapter_svid = cap_ext.vid & 0x0000FFFF;
			info->adapter_dev->adapter_id = cap_ext.pid & 0x0000FFFF;
			info->adapter_dev->adapter_fw_ver = cap_ext.fw_ver & 0x0000FFFF;
			info->adapter_dev->adapter_hw_ver = cap_ext.hw_ver & 0x0000FFFF;
			adapter_info("%s: adapter_svid = %04x\n", __func__, info->adapter_dev->adapter_svid);
			adapter_info("%s: adapter_id = %08x\n", __func__, info->adapter_dev->adapter_id);
			adapter_info("%s: adapter_fw_ver = %08x\n", __func__, info->adapter_dev->adapter_fw_ver);
			adapter_info("%s: adapter_hw_ver = %08x\n", __func__, info->adapter_dev->adapter_hw_ver);
		} else {
			adapter_err("%s: get adapter message failed!\n", __func__);
			return ret;
		}
	}

	return 0;
}

#define BSWAP_32(x) \
	(u32)((((u32)(x) & 0xff000000) >> 24) | \
			(((u32)(x) & 0x00ff0000) >> 8) | \
			(((u32)(x) & 0x0000ff00) << 8) | \
			(((u32)(x) & 0x000000ff) << 24))

static void usbpd_sha256_bitswap32(unsigned int *array, int len)
{
	int i;

	for (i = 0; i < len; i++)
		array[i] = BSWAP_32(array[i]);
}

void charToint(char *str, int input_len, unsigned int *out, unsigned int *outlen)
{
	int i;

	if (outlen != NULL)
		*outlen = 0;
	for (i = 0; i < (input_len / 4 + 1); i++) {
		out[i] = ((str[i*4 + 3] * 0x1000000) |
				(str[i*4 + 2] * 0x10000) |
				(str[i*4 + 1] * 0x100) |
				str[i*4]);
		*outlen = *outlen + 1;
	}

	adapter_info("%s: outlen = %d\n", __func__, *outlen);
	for (i = 0; i < *outlen; i++)
		adapter_info("%s: out[%d] = %08x\n", __func__, i, out[i]);
	adapter_info("%s: char to int done.\n", __func__);
}

static int tcp_dpm_event_cb_uvdm(struct tcpc_device *tcpc, int ret,
				 struct tcp_dpm_event *event)
{
	int i;
	struct tcp_dpm_custom_vdm_data vdm_data = event->tcp_dpm_data.vdm_data;

	adapter_info("%s: vdm_data.cnt = %d\n", __func__, vdm_data.cnt);
	for (i = 0; i < vdm_data.cnt; i++)
		adapter_info("%s: vdm_data.vdos[%d] = 0x%08x\n", __func__, i,
			vdm_data.vdos[i]);
	return 0;
}

const struct tcp_dpm_event_cb_data cb_data = {
	.event_cb = tcp_dpm_event_cb_uvdm,
};

static int pd_request_vdm_cmd(struct adapter_device *dev,
	enum uvdm_state cmd,
	unsigned char *data,
	unsigned int data_len)
{
	u32 vdm_hdr = 0;
	int rc = 0;
	struct tcp_dpm_custom_vdm_data *vdm_data;
	struct xm_pd_adapter_info *info;
	unsigned int *int_data;
	unsigned int outlen;
	int i;

	if (in_interrupt()) {
		int_data = kmalloc(40, GFP_ATOMIC);
		vdm_data = kmalloc(sizeof(*vdm_data), GFP_ATOMIC);
		adapter_info("%s: kmalloc atomic ok.\n", __func__);
	} else {
		int_data = kmalloc(40, GFP_KERNEL);
		vdm_data = kmalloc(sizeof(*vdm_data), GFP_KERNEL);
		adapter_info("%s: kmalloc kernel ok.\n", __func__);
	}
	memset(int_data, 0, 40);

	charToint(data, data_len, int_data, &outlen);

	info = (struct xm_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL) {
		rc = -EINVAL;
		goto release_req;
	}

	vdm_hdr = VDM_HDR(info->adapter_dev->adapter_svid, USBPD_VDM_REQUEST, cmd);
	vdm_data->wait_resp = true;
	vdm_data->vdos[0] = vdm_hdr;

	switch (cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
	case USBPD_UVDM_CHARGER_TEMP:
	case USBPD_UVDM_CHARGER_VOLTAGE:
		vdm_data->cnt = 1;
		rc = tcpm_dpm_send_custom_vdm(info->tcpc, vdm_data, &cb_data);//&tcp_dpm_evt_cb_null
		if (rc < 0) {
			adapter_err("%s: failed to send %d\n", __func__, cmd);
			goto release_req;
		}
		break;
	case USBPD_UVDM_VERIFIED:
	case USBPD_UVDM_REMOVE_COMPENSATION:
		vdm_data->cnt = 1 + USBPD_UVDM_VERIFIED_LEN;

		for (i = 0; i < USBPD_UVDM_VERIFIED_LEN; i++)
			vdm_data->vdos[i + 1] = int_data[i];
		adapter_info("%s: verify-0: %08x\n", __func__, vdm_data->vdos[1]);

		rc = tcpm_dpm_send_custom_vdm(info->tcpc, vdm_data, &cb_data);//&tcp_dpm_evt_cb_null
		if (rc < 0) {
			adapter_err("%s: failed to send %d\n", __func__, cmd);
			goto release_req;
		}
		break;
	case USBPD_UVDM_SESSION_SEED:
	case USBPD_UVDM_AUTHENTICATION:
	case USBPD_UVDM_REVERSE_AUTHEN:
		usbpd_sha256_bitswap32(int_data, USBPD_UVDM_SS_LEN);
		vdm_data->cnt = 1 + USBPD_UVDM_SS_LEN;
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++)
			vdm_data->vdos[i + 1] = int_data[i];

		for (i = 0; i < USBPD_UVDM_SS_LEN; i++)
			adapter_info("%s: %08x\n", __func__, vdm_data->vdos[i+1]);

		rc = tcpm_dpm_send_custom_vdm(info->tcpc, vdm_data, &cb_data);//&tcp_dpm_evt_cb_null
		if (rc < 0) {
			adapter_err("%s: failed to send %d\n", __func__, cmd);
			goto release_req;
		}
		break;
	default:
		adapter_err("%s: cmd:%d is not support\n", __func__, cmd);
		break;
	}

release_req:
	if (int_data != NULL)
		kfree(int_data);
	if (vdm_data != NULL)
		kfree(vdm_data);
	return rc;
}

static int pd_get_power_role(struct adapter_device *dev)
{
	struct xm_pd_adapter_info *info;

	info = (struct xm_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL)
		return -EINVAL;

	info->adapter_dev->role = tcpm_inquire_pd_power_role(info->tcpc);
	adapter_info("%s: power role is %d\n", __func__, info->adapter_dev->role);
	return 0;
}

static int pd_get_current_state(struct adapter_device *dev)
{
	struct xm_pd_adapter_info *info;

	info = (struct xm_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL)
		return -EINVAL;

	info->adapter_dev->current_state = tcpm_inquire_pd_state_curr(info->tcpc);
	adapter_info("%s: current state is %d\n", __func__, info->adapter_dev->current_state);
	return 0;
}

static int pd_get_pdos(struct adapter_device *dev)
{
	struct xm_pd_adapter_info *info;
	struct tcpm_power_cap cap;
	int ret, i;

	info = (struct xm_pd_adapter_info *)adapter_dev_get_drvdata(dev);
	if (info == NULL || info->tcpc == NULL)
		return -EINVAL;

	ret = tcpm_inquire_pd_source_cap(info->tcpc, &cap);
	adapter_info("%s: tcpm_inquire_pd_source_cap is %d.\n", __func__, ret);
	if (ret)
		return ret;
	for (i = 0; i < 7; i++) {
		info->adapter_dev->received_pdos[i] = cap.pdos[i];
		adapter_info("%s: pdo[%d] { received_pdos is %08x, cap.pdos is %08x }\n",
			__func__, i, info->adapter_dev->received_pdos[i], cap.pdos[i]);
	}

	return 0;
}

static int pd_set_pd_verify_process(struct adapter_device *dev, int verify_in_process)
{
	int ret = 0;
	//union power_supply_propval val = {0,};
	//struct power_supply *usb_psy = NULL;

	adapter_info("%s: pd verify in process: %d\n", __func__, verify_in_process);
/*
	usb_psy = power_supply_get_by_name("usb");

	if (usb_psy) {
		val.intval = verify_in_process;
		ret = power_supply_set_property(usb_psy,
			POWER_SUPPLY_PROP_PD_VERIFY_IN_PROCESS, &val);
	} else {
		adapter_err("[%s] usb psy not found!\n", __func__);
	}
*/
	return ret;
}

static int pd_get_cap(struct adapter_device *dev,
	enum adapter_cap_type type,
	struct adapter_power_cap *tacap)
{
	int ret;
	int i;
	int timeout = 0;
	struct xm_pd_adapter_info *info;
	struct tcpm_remote_power_cap pd_cap;

	info = (struct xm_pd_adapter_info *)adapter_dev_get_drvdata(dev);

	if (info == NULL || info->tcpc == NULL)
		return -EINVAL;

	if (info->adapter_dev->verify_process)
		return -1;

	if (type == XM_PD) {
APDO_REGAIN:
		pd_cap.nr = 0;
		pd_cap.selected_cap_idx = 0;
		tcpm_get_remote_power_cap(info->tcpc, &pd_cap);

		tacap->nr = pd_cap.nr;
		tacap->selected_cap_idx = pd_cap.selected_cap_idx - 1;
		adapter_info("%s: nr:%d idx:%d\n",
			__func__, pd_cap.nr, pd_cap.selected_cap_idx - 1);
		for (i = 0; i < pd_cap.nr; i++) {
			tacap->ma[i] = pd_cap.ma[i];
			tacap->max_mv[i] = pd_cap.max_mv[i];
			tacap->min_mv[i] = pd_cap.min_mv[i];
			tacap->maxwatt[i] = tacap->max_mv[i] * tacap->ma[i];
			tacap->type[i] = pd_cap.type[i];
			/*adapter_info("%s: [%d] mv:[%d,%d] ma:[%d] watt:[%d,%d] type:[%d,%d]\n",
				__func__, i, tacap->min_mv[i],
				tacap->max_mv[i], tacap->ma[i],
				tacap->minwatt[i], tacap->maxwatt[i],
				tacap->type[i], pd_cap.type[i]);*/
		}
	}  else if (type == XM_PD_APDO_REGAIN) {
		get_apdo_regain = 0;
		ret = tcpm_dpm_pd_get_source_cap(info->tcpc, NULL);
		if (ret == TCPM_SUCCESS) {
			while (timeout < 10) {
				if (get_apdo_regain) {
					adapter_info("%s: ready to get pps info!\n", __func__);
					goto APDO_REGAIN;
				} else {
					msleep(100);
					timeout++;
				}
			}
			adapter_info("%s: ready to get pps info - for test!\n", __func__);
			goto APDO_REGAIN;
		} else {
			adapter_err("%s: tcpm_dpm_pd_get_source_cap failed!\n", __func__);
			return -EINVAL;
		}
	}
	adapter_info("%s: tacap->nr is %d\n", __func__, tacap->nr);

	return 0;
}

static struct adapter_ops adapter_ops = {
	.get_cap = pd_get_cap,
	.get_svid = pd_get_svid,
	.request_vdm_cmd = pd_request_vdm_cmd,
	.get_power_role = pd_get_power_role,
	.get_current_state = pd_get_current_state,
	.get_pdos = pd_get_pdos,
	.set_pd_verify_process = pd_set_pd_verify_process,
};

static int adapter_parse_dt(struct xm_pd_adapter_info *info,
	struct device *dev)
{
	struct device_node *np = dev->of_node;

	adapter_err("%s: entry\n", __func__);

	if (!np) {
		adapter_err("%s: no device node\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_string(np, "adapter_name",
		&info->adapter_dev_name) < 0)
		adapter_err("%s: no adapter name\n", __func__);

	return 0;
}

static int xm_pd_adapter_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct xm_pd_adapter_info *info = NULL;
	static int probe_cnt = 0;

	adapter_err("%s: entry, probe_cnt = %d\n", __func__, ++probe_cnt);

	info = devm_kzalloc(&pdev->dev, sizeof(struct xm_pd_adapter_info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	platform_set_drvdata(pdev, info);

	info->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!info->tcpc) {
		adapter_err("%s: tcpc device not ready, defer\n", __func__);
		ret = -EPROBE_DEFER;
		if (probe_cnt >= PROBE_CNT_MAX) {
			adapter_err("%s: failed to get tcpc device\n", __func__);
			ret = -ENODEV;
		}
		goto err_get_tcpc;
	}

	adapter_class_init();
	adapter_parse_dt(info, &pdev->dev);

	info->adapter_dev = adapter_device_register(info->adapter_dev_name,
			    &pdev->dev, info, &adapter_ops, NULL);
	if (IS_ERR(info->adapter_dev)) {
		ret = PTR_ERR(info->adapter_dev);
		info->adapter_dev = NULL;
		adapter_err("%s: adapter_device_register failed: %d\n", __func__, ret);
		goto err_adapter_register;
	} else if (!info->adapter_dev) {
		ret = -ENOMEM;
		adapter_err("%s: adapter_device_register returned NULL\n", __func__);
		goto err_adapter_register;
	}

	adapter_dev_set_drvdata(info->adapter_dev, info);

	info->pd_adapter = get_adapter_by_name("pd_adapter");
	if (info->pd_adapter)
		adapter_info("%s: Found PD adapter [%s]\n", __func__,
			     info->pd_adapter->props.alias_name);
	else
		adapter_err("%s: Error: can't find PD adapter\n", __func__);

	info->pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(info->tcpc, &info->pd_nb,
			TCP_NOTIFY_TYPE_USB | TCP_NOTIFY_TYPE_MISC | TCP_NOTIFY_TYPE_MODE);
	if (ret < 0) {
		adapter_err("%s: register tcpc notifier fail: %d\n", __func__, ret);
		ret = -EINVAL;
		goto err_register_notifier;
	}

	adapter_info("%s: success\n", __func__);
	return 0;

err_register_notifier:
	if (!IS_ERR_OR_NULL(info->adapter_dev))
		adapter_device_unregister(info->adapter_dev);
err_adapter_register:
	if (info->adapter_svid_list)
		kfree(info->adapter_svid_list);
	adapter_class_exit();
err_get_tcpc:
	platform_set_drvdata(pdev, NULL);
	adapter_err("%s: fail!\n", __func__);
	return ret;
}

static int xm_pd_adapter_remove(struct platform_device *pdev)
{
	struct xm_pd_adapter_info *info = platform_get_drvdata(pdev);

	if (!info)
		return 0;

	unregister_tcp_dev_notifier(info->tcpc, &info->pd_nb,
			TCP_NOTIFY_TYPE_USB | TCP_NOTIFY_TYPE_MISC | TCP_NOTIFY_TYPE_MODE);

	if (!IS_ERR_OR_NULL(info->adapter_dev))
		adapter_device_unregister(info->adapter_dev);

	if (info->adapter_svid_list)
		kfree(info->adapter_svid_list);

	adapter_class_exit();
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id xm_pd_adapter_of_match[] = {
	{.compatible = "xiaomi,pd_adapter",},
	{},
};
MODULE_DEVICE_TABLE(of, xm_pd_adapter_of_match);

static struct platform_driver xm_pd_adapter_driver = {
	.probe = xm_pd_adapter_probe,
	.remove = xm_pd_adapter_remove,
	.driver = {
		   .name = "xm_pd_adapter",
		   .owner = THIS_MODULE,
		   .of_match_table = xm_pd_adapter_of_match,
	},
};

static int __init xm_pd_adapter_init(void)
{
	return platform_driver_register(&xm_pd_adapter_driver);
}
module_init(xm_pd_adapter_init);

static void __exit xm_pd_adapter_exit(void)
{
	platform_driver_unregister(&xm_pd_adapter_driver);
}
module_exit(xm_pd_adapter_exit);

MODULE_DESCRIPTION("Xiaomi PD Adapter Driver");
MODULE_AUTHOR("getian@xiaomi.com");
MODULE_LICENSE("GPL");
