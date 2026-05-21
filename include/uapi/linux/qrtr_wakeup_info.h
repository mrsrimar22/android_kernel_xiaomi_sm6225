/*
 * SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 */

#ifndef _LINUX_QRTR_WAKEUP_INFO_H
#define _LINUX_QRTR_WAKEUP_INFO_H

#define TASK_COMM_LEN 16
#define INFO_VERSION_1 1

struct qrtr_wakeup_info {
	__u8 version;
	__s32 svc_id;
	__s32 msg_id;
	__s32 msg_type;
	__s32 client_pid;
	char client_name[TASK_COMM_LEN];
	__u32 src_node;
	__u32 dst_node;
	__u64 timestamp;
	__u64 qtimer;
} __packed;
#endif /* _LINUX_QRTR_WAKEUP_INFO_H */
