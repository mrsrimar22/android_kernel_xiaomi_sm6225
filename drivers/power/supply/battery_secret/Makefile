ifneq ($(CONFIG_FRAME_WARN),0)
frame_warn_flag := -Wframe-larger-than=3072
endif

CFLAGS_sha384_software.o := $(frame_warn_flag)
obj-$(CONFIG_BATT_VERIFY_BY_DS28E16) += onewire_gpio.o ucl_sha3.o sha384_software.o ds28e16.o
obj-$(CONFIG_ONEWIRE_GPIO) += onewire_gpio.o
