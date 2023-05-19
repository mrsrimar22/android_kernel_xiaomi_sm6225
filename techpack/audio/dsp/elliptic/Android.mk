# Android makefile for audio kernel modules

ifeq ($(call is-board-platform,$(MSMSTEPPE) $(TRINKET)),true)
AUDIO_SELECT  := CONFIG_SND_SOC_SM6150=m
endif

ifeq ($(call is-board-platform,bengal),true)
AUDIO_SELECT  := CONFIG_SND_SOC_BENGAL=m
endif

ifeq ($(call is-board-platform,lito),true)
AUDIO_SELECT  := CONFIG_SND_SOC_LITO=m
endif

AUDIO_CHIPSET := audio
ifeq ($(call is-board-platform-in-list,$(MSMSTEPPE) $(TRINKET) bengal lito),true)

LOCAL_PATH := $(call my-dir)

ifneq ($(findstring vendor,$(LOCAL_PATH)),)

ifneq ($(findstring opensource,$(LOCAL_PATH)),)
	AUDIO_BLD_DIR := $(shell pwd)/vendor/qcom/opensource/audio-kernel
endif # opensource

DLKM_DIR := $(TOP)/device/qcom/common/dlkm

KBUILD_OPTIONS := AUDIO_ROOT=$(AUDIO_BLD_DIR)
KBUILD_OPTIONS += MODNAME=elliptic_dlkm
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
KBUILD_OPTIONS += $(AUDIO_SELECT)

###########################################################
include $(CLEAR_VARS)
LOCAL_MODULE              := $(AUDIO_CHIPSET)_elliptic.ko
LOCAL_MODULE_KBUILD_NAME  := elliptic_dlkm.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk
###########################################################

endif # DLKM check
endif # supported target check
