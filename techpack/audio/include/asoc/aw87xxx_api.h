// SPDX-License-Identifier: GPL-2.0
/*
 * aw87xxx_api.h  aw87xxx pa module
 *
 * Copyright (c) 2024 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 */

#ifndef __AW87XXX_API_H__
#define __AW87XXX_API_H__

#include <linux/types.h>

/*
 * Split-phase power / profile API for codec DAPM event handlers.
 *
 * Use these four functions when the codec driver controls power
 * sequencing via separate PRE_PMU / POST_PMU /
 * PRE_PMD / POST_PMD DAPM events:
 *
 *   PRE_PMU  -> aw87xxx_power_on_by_index()
 *                 Asserts RESET HIGH only.  No I²C.  Lets supply / GPIO
 *                 rail settle before the POST_PMU register write.
 *
 *   POST_PMU -> aw87xxx_set_profile_by_index()
 *                 Pushes register config (mute → regs → unmute) to the
 *                 already-powered chip.  Falls back to a full cold-start
 *                 if the chip is unexpectedly still off.
 *
 *   PRE_PMD  -> aw87xxx_soft_off_by_index()
 *                 Sends the soft-off register sequence via I²C while the
 *                 chip is still powered.  RESET GPIO stays HIGH so that
 *                 POST_PMD can assert it after its own settling delay.
 *
 *   POST_PMD -> aw87xxx_power_off_by_index()
 *                 Asserts RESET LOW after a hardware settling delay.
 *                 hwen_status is still ON after PRE_PMD so this is never
 *                 a no-op under normal sequencing.
 */
int aw87xxx_power_on_by_index(int dev_index);
int aw87xxx_set_profile_by_index(int dev_index, char *profile);
int aw87xxx_soft_off_by_index(int dev_index);
int aw87xxx_power_off_by_index(int dev_index);

#endif /* __AW87XXX_API_H__ */