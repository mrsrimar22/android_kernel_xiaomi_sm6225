/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __BOARD_ID_H__
#define __BOARD_ID_H__

/*
 * Maximum size, including the trailing NUL, of the hwname string.
 * board_id_get_hwname() requires a buffer of at least this size.
 */
#define BOARD_ID_HWNAME_MAX 32

/* hwlevel value reported for "MP" hardware. */
#define BOARD_ID_HWLEVEL_MP 9

/*
 * Copy the hwname supplied on the kernel command line
 * (androidboot.hwname=) into @str.
 *
 * @str must point to a buffer of at least BOARD_ID_HWNAME_MAX bytes.
 * The string is empty when the parameter was not supplied.
 */
void board_id_get_hwname(char *str);

int board_id_get_hwlevel(void);
int board_id_get_hwversion_product_num(void);
int board_id_get_hwversion_major_num(void);
int board_id_get_hwversion_minor_num(void);

#if defined(CONFIG_TARGET_PROJECT_K7B)
int get_board_new_version(void);
#endif

#endif /* __BOARD_ID_H__ */
