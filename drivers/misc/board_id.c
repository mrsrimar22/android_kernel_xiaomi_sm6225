// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 wanghan <wanghan@longcheer.com>
 */

#define pr_fmt(fmt) "board_id: " fmt

#include <linux/cache.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>

#include "board_id.h"

/*
 * The board identifiers are parsed once from the kernel command line during
 * start_kernel() and never change afterwards.  They are stored in
 * __ro_after_init memory so an accidental late write faults instead of
 * silently changing what the board reports.
 *
 * Parsing is all-or-nothing: a value is only published once the whole
 * parameter has been validated, so a malformed command line leaves the
 * zero-initialised default in place instead of a partially parsed result.
 * The setup handlers always return 1: the parameter is known to this driver,
 * even when its value is rejected, so the kernel must not report it as an
 * unknown boot option.
 */

/*
 * String: 'androidboot.board.new_version='
 * Format: 'androidboot.board.new_version=<number>'
 * Example:
 *     androidboot.board.new_version=0
 *     androidboot.board.new_version=1
 */
#if defined(CONFIG_TARGET_PROJECT_K7B)
static int board_new_version __ro_after_init;

int get_board_new_version(void)
{
	return board_new_version;
}
EXPORT_SYMBOL(get_board_new_version);

static int __init setup_board_new_version(char *str)
{
	int val;

	if (kstrtoint(str, 10, &val) || val < 0) {
		pr_err("invalid board.new_version '%s'\n", str);
		return 1;
	}

	board_new_version = val;
	pr_info("board_new_version: %d\n", board_new_version);
	return 1;
}
__setup("androidboot.board.new_version=", setup_board_new_version);
#endif

/*
 * String: 'androidboot.hwname='
 * Format: 'androidboot.hwname=<string>'
 * Example:
 *     androidboot.hwname=secret
 *     androidboot.hwname=rosemary
 *     androidboot.hwname=maltose
 */
static char board_id_hwname[BOARD_ID_HWNAME_MAX] __ro_after_init;

void board_id_get_hwname(char *str)
{
	if (!str)
		return;

	strscpy(str, board_id_hwname, BOARD_ID_HWNAME_MAX);
}
EXPORT_SYMBOL(board_id_get_hwname);

static int __init setup_board_id_hwname(char *str)
{
	if (strscpy(board_id_hwname, str, sizeof(board_id_hwname)) < 0)
		pr_warn("hwname '%s' truncated to %zu bytes\n",
			str, sizeof(board_id_hwname) - 1);

	pr_info("board_id_hwname: %s\n", board_id_hwname);
	return 1;
}
__setup("androidboot.hwname=", setup_board_id_hwname);

/*
 * String: 'androidboot.hwlevel='
 * Format: 'androidboot.hwlevel=<number/string>'
 * Example:
 *     androidboot.hwlevel=0
 *     androidboot.hwlevel=1
 *     androidboot.hwlevel=2
 *     androidboot.hwlevel=MP (9)
 */
static int board_id_hwlevel __ro_after_init;

int board_id_get_hwlevel(void)
{
	return board_id_hwlevel;
}
EXPORT_SYMBOL(board_id_get_hwlevel);

static int __init setup_board_id_hwlevel(char *str)
{
	int val;

	if (!strcmp("MP", str)) {
		board_id_hwlevel = BOARD_ID_HWLEVEL_MP;
		pr_info("board_id_hwlevel: %d (MP)\n", board_id_hwlevel);
		return 1;
	}

	if (kstrtoint(str, 10, &val) || val < 0) {
		pr_err("invalid hwlevel '%s'\n", str);
		return 1;
	}

	board_id_hwlevel = val;
	pr_info("board_id_hwlevel: %d\n", board_id_hwlevel);
	return 1;
}
__setup("androidboot.hwlevel=", setup_board_id_hwlevel);

/*
 * String: 'androidboot.hwversion='
 * Format: xx.xx.xx < product_number . major_number . minor_number >
 * Example:
 *     androidboot.hwversion=xx.xx.xx // product  , major, minor
 *     androidboot.hwversion=1.10.0   // secret   , 13   , 0
 *     androidboot.hwversion=1.20.0   // secret   , 0    , 0
 *     androidboot.hwversion=1.30.0   // secret   , 10   , 0
 *     androidboot.hwversion=2.10.0   // rosemary , 20   , 0
 *     androidboot.hwversion=3.10.0   // maltose  , 90   , 0
 */
static int board_id_hwversion_product_num __ro_after_init;
static int board_id_hwversion_major_num __ro_after_init;
static int board_id_hwversion_minor_num __ro_after_init;

int board_id_get_hwversion_product_num(void)
{
	return board_id_hwversion_product_num;
}
EXPORT_SYMBOL(board_id_get_hwversion_product_num);

int board_id_get_hwversion_major_num(void)
{
	return board_id_hwversion_major_num;
}
EXPORT_SYMBOL(board_id_get_hwversion_major_num);

int board_id_get_hwversion_minor_num(void)
{
	return board_id_hwversion_minor_num;
}
EXPORT_SYMBOL(board_id_get_hwversion_minor_num);

static int __init setup_board_id_hwversion(char *str)
{
	char buf[32];
	char *cursor = buf;
	char *token;
	int product;
	int major;
	int minor;

	/*
	 * Parse a private copy: strsep() modifies its input and the original
	 * string is still needed for the error message.
	 */
	if (strscpy(buf, str, sizeof(buf)) < 0)
		goto invalid;

	token = strsep(&cursor, ".");
	if (!token || kstrtoint(token, 10, &product) || product < 0)
		goto invalid;

	token = strsep(&cursor, ".");
	if (!token || kstrtoint(token, 10, &major) || major < 0)
		goto invalid;

	token = strsep(&cursor, ".");
	if (!token || kstrtoint(token, 10, &minor) || minor < 0)
		goto invalid;

	/* Reject trailing components such as '1.2.3.4'. */
	if (cursor)
		goto invalid;

	board_id_hwversion_product_num = product;
	board_id_hwversion_major_num = major;
	board_id_hwversion_minor_num = minor;

	pr_info("board_id_hwversion_product_num: %d\n",
		board_id_hwversion_product_num);
	pr_info("board_id_hwversion_major_num: %d\n",
		board_id_hwversion_major_num);
	pr_info("board_id_hwversion_minor_num: %d\n",
		board_id_hwversion_minor_num);
	return 1;

invalid:
	pr_err("invalid hwversion '%s', expected <product>.<major>.<minor>\n",
	       str);
	return 1;
}
__setup("androidboot.hwversion=", setup_board_id_hwversion);
