/* SPDX-License-Identifier: GPL-2.0 */
/*
 * aw87xxx_log.h
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 */

#ifndef __AW87XXX_LOG_H__
#define __AW87XXX_LOG_H__

#include <linux/kernel.h>

/*
 * print information control
 */
#define AW_LOGI(fmt, ...) \
	pr_info("[Awinic] %s: " fmt "\n", __func__, ##__VA_ARGS__)

#define AW_LOGD(fmt, ...) \
	pr_debug("[Awinic] %s: " fmt "\n", __func__, ##__VA_ARGS__)

#define AW_LOGE(fmt, ...) \
	pr_err("[Awinic] %s: " fmt "\n", __func__, ##__VA_ARGS__)

#define AW_DEV_LOGI(dev, fmt, ...) \
	pr_info("[Awinic] [%s] %s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#define AW_DEV_LOGD(dev, fmt, ...) \
	pr_debug("[Awinic] [%s] %s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#define AW_DEV_LOGE(dev, fmt, ...) \
	pr_err("[Awinic] [%s] %s: " fmt "\n", dev_name(dev), __func__, ##__VA_ARGS__)

#define AW_BIN_LOGD(fmt, ...) \
	pr_debug("[Awinic_BIN] %s, line=%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

#define AW_BIN_LOGE(fmt, ...) \
	pr_err("[Awinic_BIN_ERR] %s, line=%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__)

#endif
