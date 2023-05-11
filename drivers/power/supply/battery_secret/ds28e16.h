/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ds28e16 - DeepCover Secure Authenticator driver
 *
 * Constants, error codes, and command definitions for the DS28E16
 * 1-Wire secure authenticator IC. All implementation details and
 * internal state are private to ds28e16.c.
 */
#ifndef __DS28E16_H__
#define __DS28E16_H__

#define DS_TRUE				1
#define DS_FALSE			0

#define ERROR_NO_DEVICE			-1
#define ERROR_R_STATUS			-2
#define ERROR_R_ROMID			-3
#define ERROR_R_PAGEDATA		-4
#define ERROR_COMPUTE_MAC		-5
#define ERROR_S_SECRET			-6
#define ERROR_UNMATCH_MAC		-7

#define CMD_RELEASE_BYTE		0xAA
#define CMD_READ_ROM			0x33
#define CMD_SKIP_ROM			0xCC

#define CMD_START			0x66
#define CMD_WRITE_MEM			0x96
#define CMD_READ_MEM			0x44
#define CMD_READ_STATUS			0xAA
#define CMD_SET_PAGE_PROT		0xC3
#define CMD_COMP_READ_AUTH		0xA5
#define CMD_COMP_S_SECRET		0x3C
#define CMD_DECREMENT_CNT		0xC9
#define CMD_DISABLE_DEVICE		0x33

#define RESULT_SUCCESS			0xAA
#define RESULT_FAIL_NONE		0xFF

#define DELAY_DS28E16_EE_WRITE		100
#define DELAY_DS28E16_EE_READ		50

#define PAGE0				0x00
#define PAGE1				0x01
#define DC_PAGE				0x02
#define SECRET_PAGE			0x03
#define MAX_PAGENUM			0x04

#define ANONYMOUS			1

#define GET_USER_MEMORY_RETRY		8
#define GET_ROM_ID_RETRY		8
#define GET_BLOCK_STATUS_RETRY		8
#define GET_MAC_RETRY			8
#define GET_S_SECRET_RETRY		4

#define FAMILY_CODE			0x9F
#define CUSTOM_ID_MSB			0x04
#define CUSTOM_ID_LSB			0xF0

#endif /* __DS28E16_H__ */
