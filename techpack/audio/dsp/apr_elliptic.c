// SPDX-License-Identifier: GPL-2.0
/**
 * Elliptic Labs - APR interface implementation.
 *
 */

#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <sound/asound.h>
#include <sound/control.h>
#include <sound/soc.h>

#include "../asoc/msm-pcm-routing-v2.h"
#include <dsp/apr_audio-v2.h>
#include <dsp/apr_elliptic.h>
#include <dsp/q6audio-v2.h>
#include <dsp/q6common.h>
#include <elliptic/elliptic_data_io.h>
#include <elliptic/elliptic_mixer_controls.h>

enum hall_event {
	HALL_SLIDER_UP = 4,
	HALL_SLIDER_DOWN = 5,
	HALL_SLIDING = 6,
};

enum driver_sensor_type {
	DRIVER_SENSOR_HALL = 35,
};

struct driver_sensor_event {
	enum driver_sensor_type type;
	union {
		int32_t event;
		int32_t reserved[2];
	};
};

static int32_t process_version_msg(uint32_t *payload, uint32_t payload_size);
static int32_t process_branch_msg(uint32_t *payload, uint32_t payload_size);
static int32_t process_tag_msg(uint32_t *payload, uint32_t payload_size);
static int32_t process_calibration_msg(uint32_t *payload, uint32_t payload_size);
static int32_t process_calibration_v2_msg(uint32_t *payload, uint32_t payload_size);
static int32_t process_ml_msg(uint32_t *payload, uint32_t payload_size);
static int32_t process_diagnostics_msg(uint32_t *payload, uint32_t payload_size);
static int32_t process_sensorhub_msg(uint32_t *payload, uint32_t payload_size);

/**
 * build_and_send_apr_pkt() - Allocate, fill, and dispatch an APR SET_PARAM
 *   packet using either the v2 or v3 command format.
 *
 * The caller has already packed @packed_param_data; this helper wraps it in
 * the correct APR envelope, acquires the AFE lock, sends the packet, and
 * releases the lock.
 *
 * @port_id:           AFE port ID (16-bit).
 * @index:             AFE port index used as the APR token and as the index
 *                     into elus_afe.ptr_wait[].
 * @packed_param_data: Pre-packed parameter header + payload bytes.
 * @packed_data_size:  Size of @packed_param_data in bytes.
 * @use_v3:            true -> AFE_PORT_CMD_SET_PARAM_V3,
 *                     false -> AFE_PORT_CMD_SET_PARAM_V2.
 *
 * Return: Return value of apr_send_pkt() (bytes sent ≥ 0) or -ENOMEM.
 */
static int build_and_send_apr_pkt(u16 port_id, int index,
				  const u8 *packed_param_data,
				  int packed_data_size, bool use_v3)
{
	int ret;

	if (use_v3) {
		uint32_t pkt_size = sizeof(struct afe_port_cmd_set_param_v3)
				    + packed_data_size;
		struct afe_port_cmd_set_param_v3 *pkt = kzalloc(pkt_size, GFP_KERNEL);

		if (!pkt)
			return -ENOMEM;

		pkt->apr_hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
		pkt->apr_hdr.pkt_size = pkt_size;
		pkt->apr_hdr.src_port = 0;
		pkt->apr_hdr.dest_port = 0;
		pkt->apr_hdr.token = index;
		pkt->apr_hdr.opcode = AFE_PORT_CMD_SET_PARAM_V3;
		pkt->port_id = port_id;
		pkt->payload_size = packed_data_size;
		memcpy(&pkt->param_data, packed_param_data, packed_data_size);

		mutex_lock(elus_afe.ptr_afe_apr_lock);
		atomic_set(elus_afe.ptr_state, 1);
		atomic_set(elus_afe.ptr_status, 0);
		ret = apr_send_pkt(*elus_afe.ptr_apr, (uint32_t *)pkt);
		mutex_unlock(elus_afe.ptr_afe_apr_lock);

		kfree(pkt);
	} else {
		uint32_t pkt_size = sizeof(struct afe_port_cmd_set_param_v2)
				    + packed_data_size;
		struct afe_port_cmd_set_param_v2 *pkt = kzalloc(pkt_size, GFP_KERNEL);

		if (!pkt)
			return -ENOMEM;

		pkt->apr_hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
		pkt->apr_hdr.pkt_size = pkt_size;
		pkt->apr_hdr.src_port = 0;
		pkt->apr_hdr.dest_port = 0;
		pkt->apr_hdr.token = index;
		pkt->apr_hdr.opcode = AFE_PORT_CMD_SET_PARAM_V2;
		pkt->port_id = port_id;
		pkt->payload_size = packed_data_size;
		memcpy(&pkt->param_data, packed_param_data, packed_data_size);

		mutex_lock(elus_afe.ptr_afe_apr_lock);
		atomic_set(elus_afe.ptr_state, 1);
		atomic_set(elus_afe.ptr_status, 0);
		ret = apr_send_pkt(*elus_afe.ptr_apr, (uint32_t *)pkt);
		mutex_unlock(elus_afe.ptr_afe_apr_lock);

		kfree(pkt);
	}

	return ret;
}

/**
 * afe_set_parameter() - Pack and send a parameter to the DSP, then wait for
 *   the DSP acknowledgement.
 *
 * @port:        Logical port number (passed to q6audio helpers).
 * @param_id:    DSP parameter identifier.
 * @module_id:   DSP module identifier (TX or RX).
 * @prot_config: Payload to send.  Must not be NULL.
 * @length:      Payload length in bytes.  Must be > 0.
 *
 * Return: 0 on success, negative errno on failure.
 */
static int afe_set_parameter(int port, int param_id, int module_id,
			     struct afe_ultrasound_set_params_t *prot_config,
			     uint32_t length)
{
	struct param_hdr_v3 param_hdr = {0};
	u16 port_id;
	int index;
	u8 *packed_param_data = NULL;
	int packed_data_size;
	int ret;

	pr_debug("[ELUS]: %s\n", __func__);

	if (!prot_config || length == 0) {
		pr_err("%s: invalid args: prot_config=%p length=%u\n",
		       __func__, prot_config, length);
		return -EINVAL;
	}

	if (length > (UINT_MAX - sizeof(union param_hdrs))) {
		pr_err("%s: length %u would overflow packed_data_size\n",
		       __func__, length);
		return -EINVAL;
	}

	port_id = q6audio_get_port_id(port);
	ret = q6audio_validate_port(port_id);
	if (ret < 0) {
		pr_err("%s: invalid port id=0x%x ret=%d\n",
		       __func__, port_id, ret);
		return -EINVAL;
	}

	index = q6audio_get_port_index(port);
	if (index < 0) {
		pr_err("%s: invalid port index=%d for port=0x%x\n",
		       __func__, index, port);
		return -EINVAL;
	}

	packed_data_size = sizeof(union param_hdrs) + length;

	param_hdr.module_id = module_id;
	param_hdr.instance_id = INSTANCE_ID_0;
	param_hdr.param_id = param_id;
	param_hdr.param_size = length;

	pr_debug("[ELUS]: param_size=%u\n", length);

	packed_param_data = kzalloc(packed_data_size, GFP_KERNEL);
	if (!packed_param_data)
		return -ENOMEM;

	ret = q6common_pack_pp_params(packed_param_data, &param_hdr,
				      (u8 *)prot_config, &packed_data_size);
	if (ret) {
		pr_err("%s: failed to pack param header+data, ret=%d\n",
		       __func__, ret);
		goto out_free;
	}

	ret = build_and_send_apr_pkt(port_id, index, packed_param_data,
				     packed_data_size,
				     q6common_is_instance_id_supported());
	if (ret < 0) {
		pr_err("%s: send failed for port=%d param_id=0x%x, ret=%d\n",
		       __func__, port, param_id, ret);
		goto out_free;
	}

	ret = wait_event_timeout(elus_afe.ptr_wait[index],
				 atomic_read(elus_afe.ptr_state) == 0,
				 msecs_to_jiffies(elus_afe.timeout_ms));
	if (ret == 0) {
		pr_err("%s: DSP ACK timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto out_free;
	}

	if (atomic_read(elus_afe.ptr_status) != 0) {
		pr_err("%s: DSP returned error status=%d\n",
		       __func__, atomic_read(elus_afe.ptr_status));
		ret = -EINVAL;
		goto out_free;
	}

	ret = 0;

out_free:
	pr_info("%s: param_id=0x%x status=%d\n", __func__, param_id, ret);
	kfree(packed_param_data);
	return ret;
}

int32_t ultrasound_apr_set_parameter(int32_t port_id, uint32_t param_id,
				     u8 *user_params, int32_t length)
{
	uint32_t module_id = (port_id == ELLIPTIC_PORT_ID)
			   ? ELLIPTIC_ULTRASOUND_MODULE_TX
			   : ELLIPTIC_ULTRASOUND_MODULE_RX;

	return afe_set_parameter(port_id, param_id, module_id,
				 (struct afe_ultrasound_set_params_t *)user_params,
				 length);
}

/**
 * copy_payload_to_shared_obj() - Common pattern: validate size, look up the
 *   shared data block, and copy payload[3..] into it.
 *
 * @obj_id:       Shared-object identifier (ELLIPTIC_OBJ_ID_*).
 * @payload:      APR payload array.
 * @payload_size: Bytes available in payload (from payload[2]).
 * @min_size:     Minimum payload size required before copying.
 * @copy_limit:   Maximum bytes to copy (typically == min_size or max_size).
 * @out_size:     Receives the number of bytes actually copied.
 *
 * Return: bytes copied on success, -EINVAL if the shared object is not found
 *         or the payload is too small.
 */
static int32_t copy_payload_to_shared_obj(uint32_t obj_id,
					  uint32_t *payload, uint32_t payload_size,
					  size_t min_size, size_t copy_limit, size_t *out_size)
{
	struct elliptic_shared_data_block *data_block;
	size_t copy_size;

	if (payload_size < min_size)
		return -EINVAL;

	data_block = elliptic_get_shared_obj(obj_id);
	if (!data_block) {
		pr_err("[ELUS]: %s: shared obj %u not found\n", __func__, obj_id);
		return -EINVAL;
	}

	copy_size = min_t(size_t, data_block->size, copy_limit);
	memcpy((u8 *)data_block->buffer, &payload[3], copy_size);

	if (out_size)
		*out_size = copy_size;

	return (int32_t)copy_size;
}

static int32_t process_version_msg(uint32_t *payload, uint32_t payload_size)
{
	size_t copy_size;
	int32_t ret;

	pr_debug("[ELUS]: %s: size=%u\n", __func__, payload_size);

	ret = copy_payload_to_shared_obj(ELLIPTIC_OBJ_ID_VERSION_INFO,
					 payload, payload_size,
					 ELLIPTIC_VERSION_INFO_SIZE,
					 ELLIPTIC_VERSION_INFO_SIZE,
					 &copy_size);
	if (ret >= 0)
		pr_debug("[ELUS]: %s: version copied to AP cache\n", __func__);

	return ret;
}

static int32_t process_branch_msg(uint32_t *payload, uint32_t payload_size)
{
	size_t copy_size;
	int32_t ret;

	pr_debug("[ELUS]: %s: size=%u\n", __func__, payload_size);

	ret = copy_payload_to_shared_obj(ELLIPTIC_OBJ_ID_BRANCH_INFO,
					 payload, payload_size,
					 ELLIPTIC_BRANCH_INFO_SIZE,
					 ELLIPTIC_BRANCH_INFO_MAX_SIZE,
					 &copy_size);
	if (ret >= 0)
		pr_debug("[ELUS]: %s: branch copied to AP cache\n", __func__);

	return ret;
}

static int32_t process_tag_msg(uint32_t *payload, uint32_t payload_size)
{
	size_t copy_size;
	int32_t ret;

	pr_debug("[ELUS]: %s: size=%u\n", __func__, payload_size);

	ret = copy_payload_to_shared_obj(ELLIPTIC_OBJ_ID_TAG_INFO,
					 payload, payload_size,
					 ELLIPTIC_TAG_INFO_SIZE,
					 ELLIPTIC_TAG_INFO_SIZE,
					 &copy_size);
	if (ret >= 0)
		pr_debug("[ELUS]: %s: tag copied to AP cache\n", __func__);

	return ret;
}

static int32_t process_calibration_msg(uint32_t *payload, uint32_t payload_size)
{
	size_t copy_size;
	int32_t ret;

	pr_debug("[ELUS]: %s: size=%u\n", __func__, payload_size);

	ret = copy_payload_to_shared_obj(ELLIPTIC_OBJ_ID_CALIBRATION_DATA,
					 payload, payload_size,
					 ELLIPTIC_CALIBRATION_DATA_SIZE,
					 ELLIPTIC_CALIBRATION_DATA_SIZE,
					 &copy_size);
	if (ret >= 0) {
		pr_debug("[ELUS]: %s: calibration data copied to AP cache\n", __func__);
		elliptic_set_calibration_data((u8 *)&payload[3], copy_size);
	}

	return ret;
}

static int32_t process_calibration_v2_msg(uint32_t *payload, uint32_t payload_size)
{
	size_t copy_size;
	int32_t ret;

	pr_debug("[ELUS]: %s: size=%u\n", __func__, payload_size);

	ret = copy_payload_to_shared_obj(ELLIPTIC_OBJ_ID_CALIBRATION_V2_DATA,
					 payload, payload_size,
					 ELLIPTIC_CALIBRATION_V2_DATA_SIZE,
					 ELLIPTIC_CALIBRATION_V2_DATA_SIZE,
					 &copy_size);
	if (ret >= 0) {
		pr_debug("[ELUS]: %s: calibration v2 data copied to AP cache\n", __func__);
		elliptic_set_calibration_data((u8 *)&payload[3], copy_size);
	}

	return ret;
}

static int32_t process_ml_msg(uint32_t *payload, uint32_t payload_size)
{
	size_t copy_size;
	int32_t ret;

	pr_debug("[ELUS]: %s: size=%u\n", __func__, payload_size);

	ret = copy_payload_to_shared_obj(ELLIPTIC_OBJ_ID_ML_DATA,
					 payload, payload_size,
					 ELLIPTIC_ML_DATA_SIZE,
					 ELLIPTIC_ML_DATA_SIZE,
					 &copy_size);
	if (ret >= 0)
		pr_debug("[ELUS]: %s: ML data copied to AP cache\n", __func__);

	return ret;
}

static int32_t process_diagnostics_msg(uint32_t *payload, uint32_t payload_size)
{
	size_t copy_size;
	int32_t ret;

	pr_debug("[ELUS]: %s: size=%u\n", __func__, payload_size);

	ret = copy_payload_to_shared_obj(ELLIPTIC_OBJ_ID_DIAGNOSTICS_DATA,
					 payload, payload_size,
					 ELLIPTIC_DIAGNOSTICS_DATA_SIZE,
					 ELLIPTIC_DIAGNOSTICS_DATA_SIZE,
					 &copy_size);
	if (ret >= 0)
		pr_debug("[ELUS]: %s: diagnostics data copied to AP cache\n", __func__);

	return ret;
}

static int32_t process_sensorhub_msg(uint32_t *payload, uint32_t payload_size)
{
	pr_debug("[ELUS]: %s: param_id=%u size=%u\n",
		 __func__, payload[1], payload_size);
	/* No data to cache; placeholder for future sensorhub integration. */
	return 0;
}

int32_t elliptic_process_apr_payload(uint32_t *payload)
{
	/*
	 * Payload layout (all indices are uint32_t words):
	 *   [0] Module ID
	 *   [1] Param  ID
	 *   [2] Payload size in bytes (lower 16 bits); upper 16 bits reserved
	 *   [3+] Data
	 */
	uint32_t payload_size;
	int32_t ret = -1;

	if (payload[0] != ELLIPTIC_ULTRASOUND_MODULE_TX) {
		pr_err("[ELUS]: %s: unrecognised module ID %u\n", __func__, payload[0]);
		return ret;
	}

	payload_size = payload[2] & 0xFFFF;

	switch (payload[1]) {
	case ELLIPTIC_ULTRASOUND_PARAM_ID_ENGINE_VERSION:
		ret = process_version_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_BUILD_BRANCH:
		ret = process_branch_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_TAG:
		ret = process_tag_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_CALIBRATION_DATA:
		ret = process_calibration_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_CALIBRATION_V2_DATA:
		ret = process_calibration_v2_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_ML_DATA:
		ret = process_ml_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_DIAGNOSTICS_DATA:
		ret = process_diagnostics_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_SENSORHUB:
		ret = process_sensorhub_msg(payload, payload_size);
		break;
	case ELLIPTIC_ULTRASOUND_PARAM_ID_ENGINE_DATA:
		ret = elliptic_data_push(ELLIPTIC_ALL_DEVICES,
					 (const char *)&payload[3],
					 (size_t)payload_size,
					 ELLIPTIC_DATA_PUSH_FROM_KERNEL);
		if (ret != 0) {
			pr_err("[ELUS]: %s: failed to push APR payload ret=%d\n", __func__, ret);
			return ret;
		}
		ret = payload_size;
		break;
	default:
		pr_err("[ELUS]: %s: unrecognised param_id=%u\n",
		       __func__, payload[1]);
		break;
	}

	return ret;
}

/**
 * elliptic_set_hall_state() - Translate a hall-slider state integer into a
 *   DSP sensor event and send it via AFE.
 *
 * @state: 0 = slider up, 1 = slider down, 2 = sliding.
 *
 * Return: 0 on success, negative errno on failure.
 */
int elliptic_set_hall_state(int state)
{
	struct driver_sensor_event dse = { .type = DRIVER_SENSOR_HALL };
	int ret;

	switch (state) {
	case 0:
		dse.event = HALL_SLIDER_UP;
		break;
	case 1:
		dse.event = HALL_SLIDER_DOWN;
		break;
	case 2:
		dse.event = HALL_SLIDING;
		break;
	default:
		pr_err("%s: invalid HALL state=%d\n", __func__, state);
		return -EINVAL;
	}

	ret = afe_set_parameter(ELLIPTIC_PORT_ID,
				2, ELLIPTIC_ULTRASOUND_MODULE_TX,
				(struct afe_ultrasound_set_params_t *)&dse,
				sizeof(dse));
	return ret;
}
EXPORT_SYMBOL(elliptic_set_hall_state);
