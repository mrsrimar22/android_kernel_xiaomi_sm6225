/* SPDX-License-Identifier: GPL-2.0 */
/**
 * Elliptic Labs - APR (Audio Packet Router) interface to the DSP ultrasound
 * module.  All DSP communication goes through afe_set_parameter() which is
 * driven by the state kept in struct afe_ultrasound_state.
 */

#pragma once

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <dsp/apr_audio-v2.h>
#include <elliptic/elliptic_data_io.h>

#define ELLIPTIC_SET_PARAMS_SIZE        114
#define ELLIPTIC_ULTRASOUND_MODULE_TX   0x0F010201
#define ELLIPTIC_ULTRASOUND_MODULE_RX   0x0FF10202
#define ULTRASOUND_OPCODE               0x0FF10204
#define ELLIPTIC_PORT_ID    AFE_PORT_ID_PSEUDOPORT_01

/**
 * struct afe_ultrasound_set_params_t - Raw payload sent to the DSP via APR.
 *
 * The layout matches the DSP-side expectation: a flat array of 32-bit words
 * whose interpretation depends on the param_id in the APR header.
 */
struct afe_ultrasound_set_params_t {
	uint32_t payload[ELLIPTIC_SET_PARAMS_SIZE];
} __packed;

/**
 * struct afe_ultrasound_state - Shared state between this driver and the AFE
 * layer used when sending parameters to / receiving responses from the DSP.
 *
 * The AFE layer owns the underlying lock, wait-queue array, and APR handle.
 * This driver holds pointers into those objects so it can participate in the
 * same synchronisation protocol without duplicating infrastructure.
 *
 * Locking order: always acquire ptr_afe_apr_lock before touching ptr_state /
 * ptr_status.  ptr_state is then cleared by the AFE callback which wakes
 * ptr_wait, allowing afe_set_parameter() to detect completion.
 *
 * @ptr_apr:          Pointer to the AFE APR handle (void **).
 * @ptr_status:       DSP return status written by the AFE callback.
 * @ptr_state:        Set to 1 before sending a packet; cleared to 0 by the
 *                    callback when the DSP has acknowledged the command.
 * @ptr_wait:         Array of per-port wait queues used with ptr_state.
 * @ptr_afe_apr_lock: Pointer to the mutex that protects the APR send path.
 *                    This mutex is owned, initialised, and destroyed entirely
 *                    by the AFE layer (q6afe.c).  This driver only borrows the
 *                    pointer — it must never call mutex_init() or
 *                    mutex_destroy() on it.  The pointer is set by the AFE
 *                    layer when it populates elus_afe, before any DSP call.
 * @timeout_ms:       Maximum time to wait for DSP acknowledgement.
 */
struct afe_ultrasound_state {
	void **ptr_apr;
	atomic_t *ptr_status;
	atomic_t *ptr_state;
	atomic_t us_apr_state;
	wait_queue_head_t *ptr_wait;
	struct mutex *ptr_afe_apr_lock;  /* owned by AFE layer, not this driver */
	int timeout_ms;
};

typedef struct afe_ultrasound_state afe_ultrasound_state_t;
extern afe_ultrasound_state_t elus_afe;
extern atomic_t elliptic_rx_active;
extern atomic_t elliptic_tx_active;

/**
 * ultrasound_apr_set_parameter() - Send a parameter to the DSP ultrasound
 *   module and wait for acknowledgement.
 *
 * Selects the correct module ID (TX vs RX) based on @port_id and delegates
 * to the internal afe_set_parameter() helper.
 *
 * @port_id:     AFE port identifier (ELLIPTIC_PORT_ID for the TX path).
 * @param_id:    Parameter identifier understood by the DSP module.
 * @user_params: Pointer to the raw parameter bytes.
 * @length:      Number of bytes in @user_params.
 *
 * Return: 0 on success, negative errno on failure.
 */
int32_t ultrasound_apr_set_parameter(int32_t port_id, uint32_t param_id,
				     u8 *user_params, int32_t length);

/**
 * elliptic_process_apr_payload() - Dispatch an unsolicited DSP → AP payload.
 *
 * Called from the AFE callback when the DSP sends data upstream (engine
 * output, calibration results, version strings, etc.).  Routes the payload
 * to the appropriate handler based on the param_id embedded in @payload.
 *
 * @payload: Pointer to the raw APR payload words.
 *           payload[0] = Module ID
 *           payload[1] = Param ID
 *           payload[2] = payload size (lower 16 bits)
 *           payload[3..] = data
 *
 * Return: Bytes processed on success, negative errno on failure, -1 if the
 *         module ID is unrecognised.
 */
int32_t elliptic_process_apr_payload(uint32_t *payload);

/**
 * elliptic_notify_gain_change_msg() - Notify the DSP of a speaker gain change.
 *
 * @component_id: Audio component identifier.
 * @gaindb:       New gain value in dB.
 *
 * Return: 0 on success, negative errno on failure.
 */
int elliptic_notify_gain_change_msg(int component_id, int gaindb);
