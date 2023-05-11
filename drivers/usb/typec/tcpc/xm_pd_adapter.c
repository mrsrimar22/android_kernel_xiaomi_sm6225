// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include "inc/xm_adapter_class.h"
#include "inc/tcpci.h"

#define PROBE_CNT_MAX	50

struct xm_pd_adapter_info {
	struct tcpc_device *tcpc;
	struct notifier_block pd_nb;
	struct adapter_device *adapter_dev;
	struct task_struct *adapter_task;
	const char *adapter_dev_name;
	const char *alias_name;
	bool enable_kpoc_shdn;
	struct tcpm_svid_list adapter_svid_list;
	struct adapter_device *pd_adapter;
	struct mutex vdm_lock;
	unsigned int vdm_int_data[10];
	struct tcp_dpm_custom_vdm_data vdm_data;
	atomic_t apdo_regain;
};

#define ADAPTER_PREFIX "[xm_pd_adapter]: "
#define adapter_err(fmt, ...) pr_err(ADAPTER_PREFIX fmt, ##__VA_ARGS__)
#define adapter_info(fmt, ...) pr_info(ADAPTER_PREFIX fmt, ##__VA_ARGS__)
#define adapter_dbg(fmt, ...) pr_debug(ADAPTER_PREFIX fmt, ##__VA_ARGS__)

static void usbpd_mi_vdm_received(struct xm_pd_adapter_info *pinfo,
				  struct tcp_ny_uvdm uvdm)
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
		adapter_info("%s: ta_version: %x\n",
			     __func__, pinfo->pd_adapter->vdm_data.ta_version);
		break;
	case USBPD_UVDM_CHARGER_TEMP:
		pinfo->pd_adapter->vdm_data.ta_temp = (uvdm.uvdm_data[1] & 0xFFFF) * 10;
		adapter_info("%s: pinfo->pd_adapter->vdm_data.ta_temp: %d\n",
			     __func__, pinfo->pd_adapter->vdm_data.ta_temp);
		break;
	case USBPD_UVDM_CHARGER_VOLTAGE:
		pinfo->pd_adapter->vdm_data.ta_voltage = (uvdm.uvdm_data[1] & 0xFFFF) * 10;
		pinfo->pd_adapter->vdm_data.ta_voltage *= 1000; /*V->mV*/
		adapter_info("%s: ta_voltage: %d\n",
			     __func__, pinfo->pd_adapter->vdm_data.ta_voltage);
		break;
	case USBPD_UVDM_SESSION_SEED:
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++) {
			pinfo->pd_adapter->vdm_data.s_secret[i] = uvdm.uvdm_data[i + 1];
			adapter_info("%s: usbpd s_secret uvdm.uvdm_data[%d]=0x%x\n",
				     __func__, i + 1, uvdm.uvdm_data[i + 1]);
		}
		break;
	case USBPD_UVDM_AUTHENTICATION:
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++) {
			pinfo->pd_adapter->vdm_data.digest[i] = uvdm.uvdm_data[i + 1];
			adapter_info("%s: usbpd digest[%d]=0x%x\n",
				     __func__, i + 1, uvdm.uvdm_data[i + 1]);
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
	struct xm_pd_adapter_info *pinfo = container_of(pnb, struct xm_pd_adapter_info, pd_nb);

	if (!pinfo) {
		adapter_err("%s: pinfo is NULL\n", __func__);
		return NOTIFY_OK;
	}

	adapter_dbg("%s: PD charger event: %d %d\n", __func__,
		    (int)event, (int)noti->pd_state.connected);
	switch (event) {
	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			atomic_set(&pinfo->apdo_regain, 0);
		case PD_CONNECT_HARD_RESET:
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
			atomic_set(&pinfo->apdo_regain, 1);
			break;
		}
		break;
	case TCP_NOTIFY_UVDM:
		adapter_info("%s: tcpc received uvdm message\n", __func__);
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
	u32 pd_vdos[8];

	info = adapter_dev_get_drvdata(dev);
	if (!info)
		return -EINVAL;

	if (info->adapter_dev->adapter_svid != 0)
		return 0;

	memset(&info->adapter_svid_list, 0, sizeof(info->adapter_svid_list));

	ret = tcpm_inquire_pd_partner_inform(info->tcpc, pd_vdos);
	if (ret == TCPM_SUCCESS) {
		adapter_info("%s: find adapter id success.\n", __func__);
		for (i = 0; i < 8; i++)
			adapter_info("%s: VDO[%d]:%08x\n", __func__, i, pd_vdos[i]);

		info->adapter_dev->adapter_svid = pd_vdos[0] & 0x0000FFFF;
		info->adapter_dev->adapter_id = pd_vdos[2] & 0x0000FFFF;
		adapter_info("%s: adapter_svid = %04x\n", __func__,
			     info->adapter_dev->adapter_svid);
		adapter_info("%s: adapter_id = %08x\n", __func__,
			     info->adapter_dev->adapter_id);

		ret = tcpm_inquire_pd_partner_svids(info->tcpc, &info->adapter_svid_list);
		adapter_info("%s: tcpm_inquire_pd_partner_svids, ret=%d!\n", __func__, ret);
		if (ret == TCPM_SUCCESS) {
			adapter_info("%s: discover svid number is %d\n", __func__,
				     info->adapter_svid_list.cnt);
			for (i = 0; i < info->adapter_svid_list.cnt; i++) {
				adapter_info("%s: SVID[%d]:%04x\n", __func__, i,
					     info->adapter_svid_list.svids[i]);
				if (info->adapter_svid_list.svids[i] == USB_PD_MI_SVID)
					info->adapter_dev->adapter_svid = USB_PD_MI_SVID;
			}
		}
	} else {
		ret = tcpm_dpm_pd_get_source_cap_ext(info->tcpc, NULL, &cap_ext);
		if (ret == TCPM_SUCCESS) {
			info->adapter_dev->adapter_svid = cap_ext.vid & 0x0000FFFF;
			info->adapter_dev->adapter_id = cap_ext.pid & 0x0000FFFF;
			info->adapter_dev->adapter_fw_ver = cap_ext.fw_ver & 0x0000FFFF;
			info->adapter_dev->adapter_hw_ver = cap_ext.hw_ver & 0x0000FFFF;
			adapter_info("%s: adapter_svid = %04x\n", __func__,
				     info->adapter_dev->adapter_svid);
			adapter_info("%s: adapter_id = %08x\n", __func__,
				     info->adapter_dev->adapter_id);
			adapter_info("%s: adapter_fw_ver = %08x\n", __func__,
				     info->adapter_dev->adapter_fw_ver);
			adapter_info("%s: adapter_hw_ver = %08x\n", __func__,
				     info->adapter_dev->adapter_hw_ver);
		} else {
			adapter_err("%s: get adapter message failed!\n", __func__);
			return ret;
		}
	}

	return 0;
}

static void usbpd_sha256_bitswap32(unsigned int *array, int len)
{
	int i;

	for (i = 0; i < len; i++)
		array[i] = swab32(array[i]);
}

void char_to_int(char *str, int input_len, unsigned int *out, unsigned int *outlen)
{
	int i, count;
	size_t chunk;
	u8 buf[4];

	count = (input_len + 3) / 4;
	for (i = 0; i < count; i++) {
		memset(buf, 0, sizeof(buf));
		chunk = min_t(size_t, 4, input_len - i * 4);
		memcpy(buf, str + i * 4, chunk);
		out[i] = get_unaligned_le32(buf);
	}

	if (outlen)
		*outlen = count;
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

static int pd_request_vdm_cmd(struct adapter_device *dev,
			      enum uvdm_state cmd,
			      unsigned char *data,
			      unsigned int data_len)
{
	struct tcp_dpm_event_cb_data cb_data = {
		.event_cb = tcp_dpm_event_cb_uvdm,
	};
	u32 vdm_hdr = 0;
	int rc = 0;
	struct xm_pd_adapter_info *info;
	unsigned int outlen;
	int i;

	info = adapter_dev_get_drvdata(dev);
	if (!info || !info->tcpc) {
		adapter_info("%s: adapter info is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&info->vdm_lock);

	if ((data_len + 3) / 4 > ARRAY_SIZE(info->vdm_int_data)) {
		adapter_err("%s: data_len %u too large\n", __func__, data_len);
		mutex_unlock(&info->vdm_lock);
		return -EINVAL;
	}

	memset(info->vdm_int_data, 0, sizeof(info->vdm_int_data));
	memset(&info->vdm_data, 0, sizeof(info->vdm_data));

	char_to_int(data, data_len, info->vdm_int_data, &outlen);

	vdm_hdr = VDM_HDR(info->adapter_dev->adapter_svid, USBPD_VDM_REQUEST, cmd);
	info->vdm_data.wait_resp = true;
	info->vdm_data.vdos[0] = vdm_hdr;

	switch (cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
	case USBPD_UVDM_CHARGER_TEMP:
	case USBPD_UVDM_CHARGER_VOLTAGE:
		info->vdm_data.cnt = 1;
		rc = tcpm_dpm_send_custom_vdm(info->tcpc, &info->vdm_data, &cb_data);
		if (rc < 0) {
			adapter_err("%s: failed to send %d\n", __func__, cmd);
			goto unlock;
		}
		break;
	case USBPD_UVDM_VERIFIED:
	case USBPD_UVDM_REMOVE_COMPENSATION:
		info->vdm_data.cnt = 1 + USBPD_UVDM_VERIFIED_LEN;
		for (i = 0; i < USBPD_UVDM_VERIFIED_LEN; i++)
			info->vdm_data.vdos[i + 1] = info->vdm_int_data[i];
		adapter_info("%s: verify-0: %08x\n", __func__, info->vdm_data.vdos[1]);
		rc = tcpm_dpm_send_custom_vdm(info->tcpc, &info->vdm_data, &cb_data);
		if (rc < 0) {
			adapter_err("%s: failed to send %d\n", __func__, cmd);
			goto unlock;
		}
		break;
	case USBPD_UVDM_SESSION_SEED:
	case USBPD_UVDM_AUTHENTICATION:
	case USBPD_UVDM_REVERSE_AUTHEN:
		usbpd_sha256_bitswap32(info->vdm_int_data, USBPD_UVDM_SS_LEN);
		info->vdm_data.cnt = 1 + USBPD_UVDM_SS_LEN;
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++)
			info->vdm_data.vdos[i + 1] = info->vdm_int_data[i];
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++)
			adapter_info("%s: %08x\n", __func__, info->vdm_data.vdos[i + 1]);
		rc = tcpm_dpm_send_custom_vdm(info->tcpc, &info->vdm_data, &cb_data);
		if (rc < 0) {
			adapter_err("%s: failed to send %d\n", __func__, cmd);
			goto unlock;
		}
		break;
	default:
		adapter_err("%s: cmd:%d is not support\n", __func__, cmd);
		break;
	}

unlock:
	mutex_unlock(&info->vdm_lock);
	return rc;
}

static int pd_get_power_role(struct adapter_device *dev)
{
	struct xm_pd_adapter_info *info = adapter_dev_get_drvdata(dev);

	if (!info || !info->tcpc)
		return -EINVAL;

	info->adapter_dev->role = tcpm_inquire_pd_power_role(info->tcpc);
	adapter_info("%s: power role is %d\n", __func__, info->adapter_dev->role);
	return 0;
}

static int pd_get_current_state(struct adapter_device *dev)
{
	struct xm_pd_adapter_info *info = adapter_dev_get_drvdata(dev);

	if (!info || !info->tcpc)
		return -EINVAL;

	info->adapter_dev->current_state = tcpm_inquire_pd_state_curr(info->tcpc);
	adapter_info("%s: current state is %d\n", __func__,
		     info->adapter_dev->current_state);
	return 0;
}

static int pd_get_pdos(struct adapter_device *dev)
{
	struct xm_pd_adapter_info *info = adapter_dev_get_drvdata(dev);
	struct tcpm_power_cap cap;
	int ret, i;

	if (!info || !info->tcpc)
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

	adapter_info("%s: pd verify in process: %d\n", __func__, verify_in_process);

	return ret;
}

static void pd_adapter_fill_power_cap(struct xm_pd_adapter_info *info,
				      struct adapter_power_cap *tacap)
{
	struct tcpm_remote_power_cap pd_cap = { 0 };
	int i;

	tcpm_get_remote_power_cap(info->tcpc, &pd_cap);

	tacap->nr = pd_cap.nr;
	tacap->selected_cap_idx = pd_cap.selected_cap_idx - 1;
	adapter_info("%s: nr:%d idx:%d\n", __func__,
		     pd_cap.nr, pd_cap.selected_cap_idx - 1);

	for (i = 0; i < pd_cap.nr; i++) {
		tacap->ma[i] = pd_cap.ma[i];
		tacap->max_mv[i] = pd_cap.max_mv[i];
		tacap->min_mv[i] = pd_cap.min_mv[i];
		tacap->maxwatt[i] = tacap->max_mv[i] * tacap->ma[i];
		tacap->type[i] = pd_cap.type[i];
	}
}

static bool pd_adapter_wait_apdo_regain(struct xm_pd_adapter_info *info)
{
	const int max_retry = 10;
	int ret, i;

	atomic_set(&info->apdo_regain, 0);

	ret = tcpm_dpm_pd_get_source_cap(info->tcpc, NULL);
	if (ret != TCPM_SUCCESS) {
		adapter_err("%s: tcpm_dpm_pd_get_source_cap failed!\n", __func__);
		return false;
	}

	for (i = 0; i < max_retry; i++) {
		if (atomic_read(&info->apdo_regain)) {
			adapter_info("%s: ready to get pps info!\n", __func__);
			return true;
		}
		msleep(100);
	}

	adapter_info("%s: ready to get pps info - for test!\n", __func__);
	return true;
}

static int pd_get_cap(struct adapter_device *dev,
		      enum adapter_cap_type type,
		      struct adapter_power_cap *tacap)
{
	struct xm_pd_adapter_info *info;

	info = (struct xm_pd_adapter_info *)adapter_dev_get_drvdata(dev);

	if (!info || !info->tcpc)
		return -EINVAL;

	if (info->adapter_dev->verify_process)
		return -1;

	if (type == XM_PD) {
		pd_adapter_fill_power_cap(info, tacap);
	} else if (type == XM_PD_APDO_REGAIN) {
		if (!pd_adapter_wait_apdo_regain(info))
			return -EINVAL;
		pd_adapter_fill_power_cap(info, tacap);
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

static int xm_pd_adapter_parse_dt(struct platform_device *pdev,
				  struct xm_pd_adapter_info *info)
{
	struct device_node *np = pdev->dev.of_node;
	const char *alias_name;
	int ret;

	if (!np)
		return -ENODEV;

	ret = of_property_read_string(np, "adapter_name",
				      &info->adapter_dev_name);
	if (ret < 0) {
		adapter_err("%s: missing adapter_name\n", __func__);
		return ret;
	}

	/* alias_name is cosmetic only; fall back to adapter_name */
	info->alias_name = info->adapter_dev_name;
	if (!of_property_read_string(np, "alias_name", &alias_name))
		info->alias_name = alias_name;

	return 0;
}

static int xm_pd_adapter_probe(struct platform_device *pdev)
{
	struct tcpc_device *tcpc = NULL;
	struct xm_pd_adapter_info *info = NULL;
	static int probe_cnt;
	int ret = 0;

	adapter_info("%s: enter, probe_cnt: %d\n", __func__, ++probe_cnt);
	if (probe_cnt >= PROBE_CNT_MAX) {
		adapter_err("%s: probe count exceeded: %d >= %d\n",
			    __func__, probe_cnt, PROBE_CNT_MAX);
		return -ENODEV;
	}

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		adapter_err("%s: tcpc device not ready, defer\n", __func__);
		return -EPROBE_DEFER;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto err_put_tcpc;
	}

	platform_set_drvdata(pdev, info);

	info->tcpc = tcpc;
	mutex_init(&info->vdm_lock);
	atomic_set(&info->apdo_regain, 0);

	ret = xm_pd_adapter_parse_dt(pdev, info);
	if (ret < 0)
		goto err_free;

	info->adapter_dev = adapter_device_register(info->adapter_dev_name,
						    &pdev->dev, info, &adapter_ops,
						    info->alias_name);
	if (IS_ERR(info->adapter_dev)) {
		ret = PTR_ERR(info->adapter_dev);
		info->adapter_dev = NULL;
		adapter_err("%s: adapter_device_register failed: %d\n",
			    __func__, ret);
		goto err_free;
	}

	adapter_dev_set_drvdata(info->adapter_dev, info);

	info->pd_adapter = get_adapter_by_name("pd_adapter");
	if (!info->pd_adapter) {
		adapter_err("%s: can't find pd_adapter device\n", __func__);
		ret = -ENODEV;
		goto err_free_adapter;
	}
	adapter_info("%s: Found PD adapter [%s]\n", __func__,
		     info->adapter_dev_name);

	info->pd_nb.notifier_call = pd_tcp_notifier_call;
	info->pd_nb.priority = TCP_NOTIFY_PRIO_XM_PD_ADAPTER;
	ret = register_tcp_dev_notifier(info->tcpc, &info->pd_nb,
					TCP_NOTIFY_TYPE_USB |
					TCP_NOTIFY_TYPE_MISC |
					TCP_NOTIFY_TYPE_MODE);
	if (ret < 0) {
		adapter_err("%s: register tcpc notifier fail: %d\n",
			    __func__, ret);
		ret = -EINVAL;
		goto err_put_adapter;
	}

	adapter_info("%s: success\n", __func__);
	return 0;

err_put_adapter:
	adapter_put(info->pd_adapter);
err_free_adapter:
	adapter_dev_set_drvdata(info->adapter_dev, NULL);
	adapter_device_unregister(info->adapter_dev);
err_free:
	mutex_destroy(&info->vdm_lock);
	platform_set_drvdata(pdev, NULL);
err_put_tcpc:
	tcpc_dev_put(tcpc);

	adapter_err("%s: fail!\n", __func__);
	return ret;
}

static int xm_pd_adapter_remove(struct platform_device *pdev)
{
	struct xm_pd_adapter_info *info = platform_get_drvdata(pdev);

	if (!info)
		return 0;

	if (info->tcpc)
		unregister_tcp_dev_notifier(info->tcpc, &info->pd_nb,
					    TCP_NOTIFY_TYPE_USB |
					    TCP_NOTIFY_TYPE_MISC |
					    TCP_NOTIFY_TYPE_MODE);

	if (info->pd_adapter)
		adapter_put(info->pd_adapter);

	if (!IS_ERR_OR_NULL(info->adapter_dev)) {
		adapter_dev_set_drvdata(info->adapter_dev, NULL);
		adapter_device_unregister(info->adapter_dev);
	}

	if (info->tcpc)
		tcpc_dev_put(info->tcpc);

	mutex_destroy(&info->vdm_lock);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id xm_pd_adapter_of_match[] = {
	{ .compatible = "xiaomi,pd_adapter", },
	{ },
};
MODULE_DEVICE_TABLE(of, xm_pd_adapter_of_match);

static struct platform_driver xm_pd_adapter_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "xm_pd_adapter",
		.of_match_table	= of_match_ptr(xm_pd_adapter_of_match),
	},
	.probe		= xm_pd_adapter_probe,
	.remove	= xm_pd_adapter_remove,
};

static int __init xm_pd_adapter_init(void)
{
	return platform_driver_register(&xm_pd_adapter_driver);
}
fs_initcall_sync(xm_pd_adapter_init);

static void __exit xm_pd_adapter_exit(void)
{
	platform_driver_unregister(&xm_pd_adapter_driver);
}
module_exit(xm_pd_adapter_exit);

MODULE_DESCRIPTION("Xiaomi PD Adapter Driver");
MODULE_AUTHOR("getian@xiaomi.com");
MODULE_LICENSE("GPL");
