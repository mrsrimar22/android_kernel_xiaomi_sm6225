/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * onewire_gpio - 1-Wire bus provider/consumer interface
 *
 * Copyright (c) 2016 Xiaomi Inc.
 *
 */
#ifndef __ONEWIRE_GPIO_H__
#define __ONEWIRE_GPIO_H__

#include <linux/types.h>

struct onewire_gpio_data;

/**
 * onewire_bus_get - acquire a reference to a named 1-Wire bus device.
 *
 * @name: bus label string as specified in device tree.
 *
 * Returns pointer to onewire_gpio_data on success, NULL if not found.
 * Acquired reference must be released with onewire_bus_put().
 */
struct onewire_gpio_data *onewire_bus_get(const char *name);

/**
 * onewire_bus_put - release a reference acquired with onewire_bus_get().
 */
void onewire_bus_put(struct onewire_gpio_data *od);

u8 onewire_reset(struct onewire_gpio_data *od);
u8 onewire_read_byte(struct onewire_gpio_data *od);
void onewire_write_byte(struct onewire_gpio_data *od, u8 val);
void onewire_software_reset(struct onewire_gpio_data *od);

#endif /* __ONEWIRE_GPIO_H__ */
