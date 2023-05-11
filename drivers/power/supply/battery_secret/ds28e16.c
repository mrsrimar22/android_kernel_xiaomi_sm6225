// SPDX-License-Identifier: GPL-2.0-only
/*
 * ds28e16 - DeepCover Secure Authenticator driver
 *
 * Copyright (C) 2015 Maxim Integrated Products, Inc.
 * Copyright (C) 2022 Xiaomi Inc.
 *
 * Communicates with the DS28E16 over a bit-bang 1-Wire bus (onewire_gpio).
 * Authentication state is cached per-device instance for the lifetime of the
 * driver binding.
 *
 */
#define pr_fmt(fmt) "[ds28e16]: %s: " fmt, __func__

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "ds28e16.h"
#include "onewire_gpio.h"
#include <linux/crc16.h>

/**
 * struct ds28e16_data - all runtime state for one DS28E16 instance.
 */
struct ds28e16_data {
	struct platform_device *pdev;
	struct device *dev;

	struct power_supply *verify_psy;
	struct power_supply_desc verify_psy_d;

	struct delayed_work authentic_work;

	bool batt_verified;
	bool romid_verified;
	int mi_auth_result;

	int page0_val;
	u8 page0_data[16];

	struct mutex lock;	/* protects cached/config fields */

	u8 last_result_byte;
	u8 manid[2];

	u8 mi_romid[8];

	u8 session_seed[32];
	u8 s_secret[32];
	u8 challenge[32];

	bool auth_anon;		/* true = anonymous mode */
	bool auth_bdconst;	/* true = use binding data constant */
	int pagenumber;		/* page queried during authentication */

	u32 attr_trytimes;

	int auth_retry_count;

	char ow_bus_label[32];
	struct onewire_gpio_data *ow_bus;
};

/*
 * crc_low_first - Dallas/Maxim 1-Wire CRC-8 (polynomial 0x8C, LSB first).
 * Used to verify the 8-byte ROM ID.
 */
static u8 crc_low_first(const u8 *ptr, u8 len)
{
	u8 i, crc = 0x00;

	while (len--) {
		crc ^= *ptr++;
		for (i = 0; i < 8; i++)
			crc = (crc & 0x01) ? (crc >> 1) ^ 0x8c : (crc >> 1);
	}
	return crc;
}

#define ROL64(x, y) (((x) << (y)) | ((x) >> (64 - (y))))

static const u64 keccakf_rndc[24] = {
	0x0000000000000001ULL, 0x0000000000008082ULL, 0x800000000000808aULL,
	0x8000000080008000ULL, 0x000000000000808bULL, 0x0000000080000001ULL,
	0x8000000080008081ULL, 0x8000000000008009ULL, 0x000000000000008aULL,
	0x0000000000000088ULL, 0x0000000080008009ULL, 0x000000008000000aULL,
	0x000000008000808bULL, 0x800000000000008bULL, 0x8000000000008089ULL,
	0x8000000000008003ULL, 0x8000000000008002ULL, 0x8000000000000080ULL,
	0x000000000000800aULL, 0x800000008000000aULL, 0x8000000080008081ULL,
	0x8000000000008080ULL, 0x0000000080000001ULL, 0x8000000080008008ULL
};

static const u8 keccakf_rho[24] = {
	1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 2, 14,
	27, 41, 56, 8, 25, 43, 62, 18, 39, 61, 20, 44
};

static const u8 keccakf_pi[24] = {
	10, 7, 11, 17, 18, 3, 5, 16, 8, 21, 24, 4,
	15, 23, 19, 13, 12, 2, 20, 14, 22, 9, 6, 1
};

static void keccakf(u64 st[25])
{
	int round, i, j;
	u64 t, bc[5];

	for (round = 0; round < 24; round++) {
		/* Theta */
		for (i = 0; i < 5; i++)
			bc[i] = st[i] ^ st[i + 5] ^ st[i + 10] ^ st[i + 15] ^ st[i + 20];

		for (i = 0; i < 5; i++) {
			t = bc[(i + 4) % 5] ^ ROL64(bc[(i + 1) % 5], 1);
			for (j = 0; j < 25; j += 5)
				st[j + i] ^= t;
		}

		/* Rho & Pi */
		t = st[1];
		for (i = 0; i < 24; i++) {
			j = keccakf_pi[i];
			bc[0] = st[j];
			st[j] = ROL64(t, keccakf_rho[i]);
			t = bc[0];
		}

		/* Chi */
		for (j = 0; j < 25; j += 5) {
			for (i = 0; i < 5; i++)
				bc[i] = st[j + i];
			for (i = 0; i < 5; i++)
				st[j + i] ^= (~bc[(i + 1) % 5]) & bc[(i + 2) % 5];
		}

		/* Iota */
		st[0] ^= keccakf_rndc[round];
	}
}

struct ds_sha3_256_ctx {
	u64 st[25];
	u32 pt;
};

static inline void ds_sha3_256_init(struct ds_sha3_256_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
}

static void ds_sha3_256_update(struct ds_sha3_256_ctx *ctx, const u8 *data, size_t len)
{
	u8 *st_bytes = (u8 *)ctx->st;

	while (len--) {
		st_bytes[ctx->pt++] ^= *data++;
		if (ctx->pt == 136) {
			keccakf(ctx->st);
			ctx->pt = 0;
		}
	}
}

static void ds_sha3_256_final(struct ds_sha3_256_ctx *ctx, u8 *out)
{
	u8 *st_bytes = (u8 *)ctx->st;

	st_bytes[ctx->pt] ^= 0x06;
	st_bytes[135] ^= 0x80;
	keccakf(ctx->st);

	memcpy(out, ctx->st, 32);
}

static int sha3_256_hmac(const u8 *key, u32 key_len,
			 const u8 *message, u32 msg_len, u8 *mac)
{
	struct ds_sha3_256_ctx ctx;
	u8 k_pad[136];
	u8 thash[32];
	u32 i;

	if (key_len > 136 || msg_len > 512)
		return 0;

	/* Phase 1: Inner hash = SHA3-256(ipad || message) */
	memset(k_pad, 0x36, sizeof(k_pad));
	for (i = 0; i < key_len; i++)
		k_pad[i] ^= key[i];

	ds_sha3_256_init(&ctx);
	ds_sha3_256_update(&ctx, k_pad, sizeof(k_pad));
	ds_sha3_256_update(&ctx, message, msg_len);
	ds_sha3_256_final(&ctx, thash);

	/* Phase 2: Outer hash = SHA3-256(opad || thash) */
	memset(k_pad, 0x5C, sizeof(k_pad));
	for (i = 0; i < key_len; i++)
		k_pad[i] ^= key[i];

	ds_sha3_256_init(&ctx);
	ds_sha3_256_update(&ctx, k_pad, sizeof(k_pad));
	ds_sha3_256_update(&ctx, thash, sizeof(thash));
	ds_sha3_256_final(&ctx, mac);

	return 1;
}

/*
 * Thin inline helpers that forward calls through the acquired
 * struct onewire_bus pointer.
 *
 * All callers must hold ds->lock.
 */
static inline u8 ds_ow_reset(struct ds28e16_data *ds)
{
	return onewire_reset(ds->ow_bus);
}

static inline u8 ds_ow_read_byte(struct ds28e16_data *ds)
{
	return onewire_read_byte(ds->ow_bus);
}

static inline void ds_ow_write_byte(struct ds28e16_data *ds, u8 val)
{
	onewire_write_byte(ds->ow_bus, val);
}

static inline void ds_ow_software_reset(struct ds28e16_data *ds)
{
	onewire_software_reset(ds->ow_bus);
}

/**
 * ds28e16_standard_cmd_flow - execute one DS28E16 function command.
 *
 * @ds:        device instance
 * @write_buf: command frame: [length_byte, cmd, params...]
 * @write_len: total bytes in write_buf
 * @delay_ms:  strong pull-up hold time in ms; 0 = no pull-up needed
 * @read_buf:  output buffer (caller must size to *read_len bytes)
 * @read_len:  in: expected response length; out: actual response length
 *
 * Acquires and releases ds->lock internally.  Callers must NOT hold
 * ds->lock when calling this function.
 *
 * On any bus or CRC error the bus is reset before the lock is released,
 * leaving the line in a known idle state for the next transaction.
 *
 * Returns DS_TRUE on success, DS_FALSE / ERROR_NO_DEVICE on failure.
 *
 * Wire sequence:
 *    <Reset> <SKIP ROM> <START(0x66)> <write_buf> <CRC16-check>
 *    [RELEASE + delay] <dummy> <len> <result+data> <CRC16-check>
 */
static int ds28e16_standard_cmd_flow(struct ds28e16_data *ds,
				     u8 *write_buf, int write_len,
				     int delay_ms,
				     u8 *read_buf, int *read_len)
{
	u8 buf[128];
	u16 crc16;
	int i, buf_len = 0;
	int expected_read_len = *read_len;

	mutex_lock(&ds->lock);

	if (ds_ow_reset(ds) != 0) {
		ds_ow_software_reset(ds);
		mutex_unlock(&ds->lock);
		pr_err("reset: no device\n");
		return ERROR_NO_DEVICE;
	}

	ds_ow_write_byte(ds, CMD_SKIP_ROM);

	buf[buf_len++] = CMD_START;
	memcpy(&buf[buf_len], write_buf, write_len);
	buf_len += write_len;

	for (i = 0; i < buf_len; i++)
		ds_ow_write_byte(ds, buf[i]);

	buf[buf_len++] = ds_ow_read_byte(ds);
	buf[buf_len++] = ds_ow_read_byte(ds);

	crc16 = 0;
	for (i = 0; i < buf_len; i++)
		crc16 = crc16_byte(crc16, buf[i]);

	if (crc16 != 0xB001) {
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		pr_err("TX CRC error\n");
		return DS_FALSE;
	}

	if (delay_ms > 0) {
		ds_ow_write_byte(ds, CMD_RELEASE_BYTE);
		msleep(delay_ms);
	}

	ds_ow_read_byte(ds);
	*read_len = ds_ow_read_byte(ds);

	if (*read_len != expected_read_len) {
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		pr_err("length mismatch: got %d expected %d\n",
		       *read_len, expected_read_len);
		return DS_FALSE;
	}

	buf_len = *read_len + 2;
	for (i = 0; i < buf_len; i++)
		buf[i] = ds_ow_read_byte(ds);

	crc16 = 0;
	crc16 = crc16_byte(crc16, (u8)*read_len);
	for (i = 0; i < buf_len; i++)
		crc16 = crc16_byte(crc16, buf[i]);

	if (crc16 != 0xB001) {
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		pr_err("RX CRC error\n");
		return DS_FALSE;
	}

	memcpy(read_buf, buf, *read_len);
	mutex_unlock(&ds->lock);
	return DS_TRUE;
}

/**
 * ds28e16_read_romid - read the 8-byte 1-Wire ROM ID from the device.
 *
 * Serves the cached value if a valid read has been performed previously.
 * On a fresh read the CRC-8 is verified and the family + custom-ID bytes
 * are checked to confirm a DS28E16.
 *
 * The cache check, bus transaction, and cache update all run under ds->lock
 * to prevent a concurrent reader from observing a partially-updated mi_romid.
 */
static int ds28e16_read_romid(struct ds28e16_data *ds, u8 *romid)
{
	u8 i, crc;

	mutex_lock(&ds->lock);

	if (ds->romid_verified && ds->mi_romid[0] != 0x1f) {
		memcpy(romid, ds->mi_romid, 8);
		mutex_unlock(&ds->lock);
		return DS_TRUE;
	}

	if (ds_ow_reset(ds) != 0) {
		ds_ow_software_reset(ds);
		mutex_unlock(&ds->lock);
		pr_err("reset: no device\n");
		return ERROR_NO_DEVICE;
	}

	ds_ow_write_byte(ds, CMD_READ_ROM);
	usleep_range(10, 20);
	for (i = 0; i < 8; i++)
		romid[i] = ds_ow_read_byte(ds);

	crc = crc_low_first(romid, 7);
	if (crc != romid[7]) {
		mutex_unlock(&ds->lock);
		pr_err("ROM ID CRC mismatch\n");
		return DS_FALSE;
	}

	memcpy(ds->mi_romid, romid, 8);
	ds->romid_verified = (ds->mi_romid[0] == FAMILY_CODE &&
			      ds->mi_romid[6] == CUSTOM_ID_MSB &&
			      (ds->mi_romid[5] & 0xf0) == CUSTOM_ID_LSB);

	pr_debug("RomID: %8phC\n", romid);

	mutex_unlock(&ds->lock);
	return DS_TRUE;
}

/**
 * ds28e16_virtual_cmd_read_status - raw 1-Wire sequence used to wake the
 * device during ROM ID retry recovery.
 */
static void ds28e16_virtual_cmd_read_status(struct ds28e16_data *ds)
{
	int i;

	mutex_lock(&ds->lock);
	ds_ow_reset(ds);
	ds_ow_write_byte(ds, CMD_SKIP_ROM);
	ds_ow_write_byte(ds, CMD_START);
	ds_ow_write_byte(ds, 0x01);
	ds_ow_write_byte(ds, CMD_READ_STATUS);
	ds_ow_read_byte(ds);
	ds_ow_read_byte(ds);
	ds_ow_write_byte(ds, CMD_RELEASE_BYTE);
	msleep(50);
	for (i = 0; i < 11; i++)
		ds_ow_read_byte(ds);
	ds_ow_reset(ds);
	mutex_unlock(&ds->lock);
}

/**
 * ds28e16_cmd_read_status - read the 7-byte status / protection register.
 *
 * Byte layout returned (0-indexed from data[0]):
 *    [0..1]  page protection for page 0 and page 1
 *    [2]     counter page protection
 *    [3]     reserved
 *    [4]     MANID byte 0
 *    [5]     MANID byte 1
 *    [6]     chip version
 *
 * Callers must NOT hold ds->lock.
 */
static int ds28e16_cmd_read_status(struct ds28e16_data *ds, u8 *data)
{
	u8 write_buf[4];
	u8 read_buf[16];
	int write_len = 0;
	int read_len = 7;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	write_buf[write_len++] = 1;
	write_buf[write_len++] = CMD_READ_STATUS;

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_READ,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_buf[0] != RESULT_SUCCESS) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	memcpy(data, &read_buf[1], 7);

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	ds->manid[0] = data[4];
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

/**
 * ds28e16_cmd_read_memory - read one user-memory or counter page.
 *
 * @pg:   0 = page 0, 1 = page 1, 2 = decrement counter page
 * @data: output buffer; must be at least 16 bytes
 *
 * The device returns 33 bytes: 1 result byte + 16 data bytes +
 * 16 zero-padding bytes. Only the first 16 data bytes are copied
 * to the caller's buffer.
 *
 * Callers must NOT hold ds->lock.
 */
static int ds28e16_cmd_read_memory(struct ds28e16_data *ds,
				   int pg, u8 *data)
{
	u8 write_buf[4];
	u8 read_buf[64];
	int write_len = 0;
	int read_len = 33;
	u8 pagenum = (u8)pg & 0x03;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	write_buf[write_len++] = 2;
	write_buf[write_len++] = CMD_READ_MEM;
	write_buf[write_len++] = pagenum;

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_READ,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_len != 33) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	if (read_buf[0] != RESULT_SUCCESS &&
	    !(read_buf[0] == 0x55 && pagenum == 0x02)) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	memcpy(data, &read_buf[1], 16);

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

/**
 * ds28e16_cmd_write_memory - write 16 bytes to a user-memory page.
 *
 * Invalidates the corresponding cache on success.
 */
static int ds28e16_cmd_write_memory(struct ds28e16_data *ds,
				    int pg, u8 *data)
{
	u8 write_buf[32];
	u8 read_buf[4];
	int write_len = 0;
	int read_len = 1;
	u8 pagenum = (u8)pg & 0x03;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	write_buf[write_len++] = 18;
	write_buf[write_len++] = CMD_WRITE_MEM;
	write_buf[write_len++] = pagenum;
	memcpy(&write_buf[write_len], data, 16);
	write_len += 16;

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_len != 1 || read_buf[0] != RESULT_SUCCESS) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	if (pagenum == 0x00) {
		ds->page0_val = 0;
		memset(ds->page0_data, 0x00, 16);
	}
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

/**
 * ds28e16_cmd_decrement_counter - decrement the 17-bit monotonic counter.
 */
static __maybe_unused int ds28e16_cmd_decrement_counter(struct ds28e16_data *ds)
{
	u8 write_buf[4];
	u8 read_buf[4];
	int write_len = 0;
	int read_len = 1;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	write_buf[write_len++] = 1;
	write_buf[write_len++] = CMD_DECREMENT_CNT;

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_READ,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_len != 1 || read_buf[0] != RESULT_SUCCESS) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

/**
 * ds28e16_cmd_set_page_protection - set read/write protection on a page.
 */
static __maybe_unused int ds28e16_cmd_set_page_protection(struct ds28e16_data *ds,
							  int page, u8 prot)
{
	u8 write_buf[8];
	u8 read_buf[4];
	int write_len = 0;
	int read_len = 1;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	write_buf[write_len++] = 3;
	write_buf[write_len++] = CMD_SET_PAGE_PROT;
	write_buf[write_len++] = page & 0x03;
	write_buf[write_len++] = prot & 0x03;

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_len != 1 || read_buf[0] != RESULT_SUCCESS) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

/**
 * ds28e16_cmd_device_disable - permanently disable the device.
 *
 * @op:       operation code (0x0F = disable)
 * @password: 8-byte authorisation password
 *
 * WARNING: irreversible.
 */
static __maybe_unused int ds28e16_cmd_device_disable(struct ds28e16_data *ds,
						     int op, u8 *password)
{
	u8 write_buf[16];
	u8 read_buf[4];
	int write_len = 0;
	int read_len = 1;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	write_buf[write_len++] = 10;
	write_buf[write_len++] = CMD_DISABLE_DEVICE;
	write_buf[write_len++] = op & 0x0F;
	memcpy(&write_buf[write_len], password, 8);
	write_len += 8;

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_len != 1 || read_buf[0] != RESULT_SUCCESS) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

/**
 * ds28e16_cmd_compute_read_page_auth - request a device MAC.
 *
 * @anon:      if non-zero, substitute 0xFF for the ROM ID in the hash input
 * @pg:        page number whose data is included in the MAC
 * @challenge: 32-byte nonce
 * @hmac:      output - 32-byte HMAC-SHA3-256 computed by the device
 */
static int ds28e16_cmd_compute_read_page_auth(struct ds28e16_data *ds,
					      int anon, int pg,
					      u8 *challenge, u8 *hmac)
{
	u8 write_buf[40];
	u8 read_buf[40];
	int write_len = 0;
	int read_len = 33;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	write_buf[write_len++] = 35;
	write_buf[write_len++] = CMD_COMP_READ_AUTH;
	write_buf[write_len] = pg & 0x03;
	if (anon)
		write_buf[write_len] |= 0xE0;
	write_len++;
	write_buf[write_len++] = 0x02;
	memcpy(&write_buf[write_len], challenge, 32);
	write_len += 32;

	pr_debug("write_buf: %35ph\n", write_buf);

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_buf[0] != RESULT_SUCCESS) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	mutex_unlock(&ds->lock);

	memcpy(hmac, &read_buf[1], 32);

	pr_debug("device HMAC: %32ph\n", hmac);
	return DS_TRUE;
}

/**
 * ds28e16_cmd_compute_s_secret - instruct the device to compute its
 * session secret from the partial secret (session seed) provided by
 * the host.
 *
 * @anon:     anonymous mode flag
 * @bdconst:  use binding data constant flag
 * @pg:       page number to bind the secret to
 * @partial:  32-byte partial secret (session seed)
 */
static int ds28e16_cmd_compute_s_secret(struct ds28e16_data *ds,
					int anon, int bdconst,
					int pg, u8 *partial)
{
	u8 write_buf[40];
	u8 read_buf[4];
	int write_len = 0;
	int read_len = 1;
	int param = pg & 0x03;

	mutex_lock(&ds->lock);
	ds->last_result_byte = RESULT_FAIL_NONE;
	mutex_unlock(&ds->lock);

	if (bdconst)
		param |= 0x04;
	if (anon)
		param |= 0xE0;

	write_buf[write_len++] = 35;
	write_buf[write_len++] = CMD_COMP_S_SECRET;
	write_buf[write_len++] = (u8)param;
	write_buf[write_len++] = 0x08;
	memcpy(&write_buf[write_len], partial, 32);
	write_len += 32;

	pr_debug("write_buf: %35ph\n", write_buf);

	if (ds28e16_standard_cmd_flow(ds, write_buf, write_len,
				      DELAY_DS28E16_EE_WRITE,
				      read_buf, &read_len) != DS_TRUE)
		return DS_FALSE;

	if (read_buf[0] != RESULT_SUCCESS) {
		mutex_lock(&ds->lock);
		ds_ow_reset(ds);
		mutex_unlock(&ds->lock);
		return DS_FALSE;
	}

	mutex_lock(&ds->lock);
	ds->last_result_byte = read_buf[0];
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

static int ds28e16_read_romid_retry(struct ds28e16_data *ds, u8 *romid)
{
	int i;
	u8 first_byte;

	for (i = 0; i < 5; i++) {
		mutex_lock(&ds->lock);
		first_byte = ds->mi_romid[0];
		mutex_unlock(&ds->lock);

		if (first_byte != 0x1f && first_byte != 0x00)
			break;

		pr_info("ROM ID recovery attempt %d\n", i);
		ds28e16_virtual_cmd_read_status(ds);
		ds28e16_read_romid(ds, romid);
		usleep_range(100, 200);
	}

	for (i = 0; i < GET_ROM_ID_RETRY; i++) {
		if (ds28e16_read_romid(ds, romid) == DS_TRUE)
			return DS_TRUE;
	}
	return DS_FALSE;
}

/**
 * ds28e16_authenticate - full HMAC-SHA3-256 challenge-response authentication.
 *
 * Snapshots all volatile configuration (auth params, session_seed, s_secret,
 * challenge) under ds->lock at the start, then releases the lock before
 * performing any bus I/O. ds->lock is only ever held for short, in-memory
 * critical sections; it is never held across a 1-Wire transaction (which can
 * block for 50-100+ ms on strong pull-up delays), so other threads reading
 * or writing device state via sysfs are never stalled behind a bus op. This
 * snapshot still gives a consistent view of the parameters for one full
 * authentication round.
 *
 * Returns DS_TRUE on success, or an ERROR_* code on failure.
 */
static int ds28e16_authenticate(struct ds28e16_data *ds)
{
	int auth_anon, auth_bdconst, pagenumber;
	u8 session_seed[32];
	u8 s_secret[32];
	u8 challenge[32];
	u8 page_data[16];
	u8 mac_from_device[32];
	u8 mac_computed[32];
	u8 status_chip[16];
	u8 msg[128];
	int msg_len = 0;
	int i;

	mutex_lock(&ds->lock);
	auth_anon = ds->auth_anon;
	auth_bdconst = ds->auth_bdconst;
	pagenumber = ds->pagenumber;
	memcpy(session_seed, ds->session_seed, 32);
	memcpy(s_secret, ds->s_secret, 32);
	memcpy(challenge, ds->challenge, 32);
	mutex_unlock(&ds->lock);

	if (ds28e16_read_romid_retry(ds, ds->mi_romid) != DS_TRUE) {
		pr_err("read_romid_retry failed\n");
		return ERROR_R_ROMID;
	}

	for (i = 0; i < GET_BLOCK_STATUS_RETRY; i++) {
		if (ds28e16_cmd_read_status(ds, status_chip) == DS_TRUE)
			break;
	}
	if (i < GET_BLOCK_STATUS_RETRY) {
		mutex_lock(&ds->lock);
		ds->manid[0] = status_chip[4];
		mutex_unlock(&ds->lock);
	} else {
		pr_err("read_status failed\n");
		return ERROR_R_STATUS;
	}

	for (i = 0; i < GET_S_SECRET_RETRY; i++) {
		if (ds28e16_cmd_compute_s_secret(ds, auth_anon, auth_bdconst,
						 0, session_seed) == DS_TRUE)
			break;
	}
	if (i == GET_S_SECRET_RETRY) {
		pr_err("compute_s_secret failed\n");
		return ERROR_S_SECRET;
	}

	for (i = 0; i < GET_MAC_RETRY; i++) {
		if (ds28e16_cmd_compute_read_page_auth(ds, auth_anon, pagenumber,
						       challenge, mac_from_device) == DS_TRUE)
			break;
	}
	if (i == GET_MAC_RETRY) {
		pr_err("compute_read_page_auth failed\n");
		return ERROR_COMPUTE_MAC;
	}

	pr_debug("session_seed: %32ph\n", session_seed);
	pr_debug("s_secret: %32ph\n", s_secret);
	pr_debug("challenge: %32ph\n", challenge);
	pr_debug("device MAC: %32ph\n", mac_from_device);

	for (i = 0; i < GET_USER_MEMORY_RETRY; i++) {
		if (ds28e16_cmd_read_memory(ds, pagenumber, page_data) == DS_TRUE)
			break;
	}
	if (i == GET_USER_MEMORY_RETRY) {
		pr_err("read_memory failed\n");
		return ERROR_R_PAGEDATA;
	}

	if (auth_anon != ANONYMOUS) {
		mutex_lock(&ds->lock);
		memcpy(&msg[msg_len], ds->mi_romid, 8);
		mutex_unlock(&ds->lock);
	} else {
		memset(&msg[msg_len], 0xff, 8);
	}
	msg_len += 8;

	memcpy(&msg[msg_len], page_data, 16);
	msg_len += 16;
	memset(&msg[msg_len], 0x00, 16);
	msg_len += 16;

	memcpy(&msg[msg_len], challenge, 32);
	msg_len += 32;

	msg[msg_len++] = pagenumber & 0x03;

	mutex_lock(&ds->lock);
	memcpy(&msg[msg_len], ds->manid, 2);
	mutex_unlock(&ds->lock);
	msg_len += 2;

	pr_debug("host MAC input: %80ph\n", msg);

	sha3_256_hmac(s_secret, 32, msg, msg_len, mac_computed);

	pr_debug("host MAC: %32ph\n", mac_computed);

	for (i = 0; i < 32; i++) {
		if (mac_computed[i] != mac_from_device[i])
			break;
	}

	if (i != 32) {
		pr_err("MAC mismatch at byte %d\n", i);
		WRITE_ONCE(ds->batt_verified, false);
		WRITE_ONCE(ds->mi_auth_result, ERROR_UNMATCH_MAC);
		return ERROR_UNMATCH_MAC;
	}

	pr_info("authentication successful\n");
	WRITE_ONCE(ds->batt_verified, true);
	WRITE_ONCE(ds->mi_auth_result, DS_TRUE);
	return DS_TRUE;
}

static int ds28e16_get_chip_ok(struct ds28e16_data *ds, int *val)
{
	int ret;

	*val = 0;

	if (READ_ONCE(ds->romid_verified)) {
		*val = 1;
		return DS_TRUE;
	}

	ret = ds28e16_read_romid_retry(ds, ds->mi_romid);
	if (ret != DS_TRUE) {
		pr_err("read_romid_retry failed\n");
		return -EAGAIN;
	}

	mutex_lock(&ds->lock);
	if (ds->mi_romid[0] == FAMILY_CODE &&
	    ds->mi_romid[6] == CUSTOM_ID_MSB &&
	    (ds->mi_romid[5] & 0xf0) == CUSTOM_ID_LSB) {
		*val = 1;
		WRITE_ONCE(ds->romid_verified, true);
	}
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

static int ds28e16_get_page0_data(struct ds28e16_data *ds,
				  u8 *buf, int buf_len)
{
	int ret;

	if (buf_len < 16)
		return -EINVAL;

	mutex_lock(&ds->lock);
	if (ds->page0_val != 0) {
		memcpy(buf, ds->page0_data, 16);
		mutex_unlock(&ds->lock);
		return DS_TRUE;
	}
	mutex_unlock(&ds->lock);

	for (ret = 0; ret < GET_USER_MEMORY_RETRY; ret++) {
		if (ds28e16_cmd_read_memory(ds, 0, buf) == DS_TRUE)
			break;
	}
	if (ret == GET_USER_MEMORY_RETRY) {
		pr_err("get_page_data_retry(0) failed\n");
		return -EAGAIN;
	}

	mutex_lock(&ds->lock);
	ds->page0_val = buf[0];
	memcpy(ds->page0_data, buf, 16);
	mutex_unlock(&ds->lock);

	return DS_TRUE;
}

static enum power_supply_property verify_props[] = {
	POWER_SUPPLY_PROP_ROMID,
	POWER_SUPPLY_PROP_DS_STATUS,
	POWER_SUPPLY_PROP_PAGENUMBER,
	POWER_SUPPLY_PROP_PAGEDATA,
	POWER_SUPPLY_PROP_AUTHEN_RESULT,
	POWER_SUPPLY_PROP_SESSION_SEED,
	POWER_SUPPLY_PROP_S_SECRET,
	POWER_SUPPLY_PROP_CHALLENGE,
	POWER_SUPPLY_PROP_AUTH_ANON,
	POWER_SUPPLY_PROP_AUTH_BDCONST,
	POWER_SUPPLY_PROP_PAGE0_DATA,
	POWER_SUPPLY_PROP_PAGE1_DATA,
	POWER_SUPPLY_PROP_VERIFY_MODEL_NAME,
	POWER_SUPPLY_PROP_CHIP_OK,
};

static int verify_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct ds28e16_data *ds = power_supply_get_drvdata(psy);
	u8 buf[50];
	int tmp, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VERIFY_MODEL_NAME:
		ret = ds28e16_read_romid_retry(ds, ds->mi_romid);
		val->strval = (ret == DS_TRUE) ? "ds28e16" : "unknown";
		break;

	case POWER_SUPPLY_PROP_AUTHEN_RESULT:
		val->intval = READ_ONCE(ds->batt_verified) ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_SESSION_SEED:
		mutex_lock(&ds->lock);
		memcpy(val->arrayval, ds->session_seed, 32);
		mutex_unlock(&ds->lock);
		break;

	case POWER_SUPPLY_PROP_S_SECRET:
		mutex_lock(&ds->lock);
		memcpy(val->arrayval, ds->s_secret, 32);
		mutex_unlock(&ds->lock);
		break;

	case POWER_SUPPLY_PROP_CHALLENGE:
		mutex_lock(&ds->lock);
		memcpy(val->arrayval, ds->challenge, 32);
		mutex_unlock(&ds->lock);
		break;

	case POWER_SUPPLY_PROP_PAGENUMBER:
		mutex_lock(&ds->lock);
		val->intval = ds->pagenumber;
		mutex_unlock(&ds->lock);
		break;

	case POWER_SUPPLY_PROP_ROMID:
		ret = ds28e16_read_romid_retry(ds, ds->mi_romid);
		if (ret != DS_TRUE)
			return -EAGAIN;
		mutex_lock(&ds->lock);
		memcpy(val->arrayval, ds->mi_romid, 8);
		mutex_unlock(&ds->lock);
		break;

	case POWER_SUPPLY_PROP_CHIP_OK:
		if (READ_ONCE(ds->romid_verified)) {
			val->intval = 1;
			break;
		}
		ret = ds28e16_get_chip_ok(ds, &tmp);
		if (ret != DS_TRUE)
			return -EAGAIN;
		val->intval = tmp ? 1 : 0;
		break;

	case POWER_SUPPLY_PROP_DS_STATUS:
		for (ret = 0; ret < GET_BLOCK_STATUS_RETRY; ret++) {
			if (ds28e16_cmd_read_status(ds, buf) == DS_TRUE)
				break;
		}
		if (ret == GET_BLOCK_STATUS_RETRY)
			return -EAGAIN;
		memcpy(val->arrayval, buf, 7);
		break;

	case POWER_SUPPLY_PROP_PAGEDATA:
		mutex_lock(&ds->lock);
		tmp = ds->pagenumber;
		mutex_unlock(&ds->lock);
		for (ret = 0; ret < GET_USER_MEMORY_RETRY; ret++) {
			if (ds28e16_cmd_read_memory(ds, tmp, buf) == DS_TRUE)
				break;
		}
		if (ret == GET_USER_MEMORY_RETRY)
			return -EAGAIN;
		memcpy(val->arrayval, buf, 16);
		break;

	case POWER_SUPPLY_PROP_PAGE0_DATA:
		ret = ds28e16_get_page0_data(ds, buf, sizeof(buf));
		if (ret != DS_TRUE)
			return -EAGAIN;
		memcpy(val->arrayval, buf, 16);
		break;

	case POWER_SUPPLY_PROP_PAGE1_DATA:
		for (ret = 0; ret < GET_USER_MEMORY_RETRY; ret++) {
			if (ds28e16_cmd_read_memory(ds, 1, buf) == DS_TRUE)
				break;
		}
		if (ret == GET_USER_MEMORY_RETRY)
			return -EAGAIN;
		memcpy(val->arrayval, buf, 16);
		break;

	default:
		pr_debug("unsupported property %d\n", psp);
		return -ENODATA;
	}

	return 0;
}

static int verify_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct ds28e16_data *ds = power_supply_get_drvdata(psy);

	mutex_lock(&ds->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_PAGENUMBER:
		ds->pagenumber = val->intval;
		break;
	case POWER_SUPPLY_PROP_AUTH_ANON:
		ds->auth_anon = !!val->intval;
		break;
	case POWER_SUPPLY_PROP_AUTH_BDCONST:
		ds->auth_bdconst = !!val->intval;
		break;
	default:
		mutex_unlock(&ds->lock);
		pr_debug("unsupported property %d\n", prop);
		return -ENODATA;
	}
	mutex_unlock(&ds->lock);

	return 0;
}

static int verify_prop_is_writeable(struct power_supply *psy,
				    enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_PAGENUMBER:
	case POWER_SUPPLY_PROP_AUTH_ANON:
	case POWER_SUPPLY_PROP_AUTH_BDCONST:
		return 1;
	default:
		return 0;
	}
}

static int verify_psy_register(struct ds28e16_data *ds)
{
	struct power_supply_config cfg = {};

	ds->verify_psy_d.name = "batt_verify";
	ds->verify_psy_d.type = POWER_SUPPLY_TYPE_BATTERY_VERIFY;
	ds->verify_psy_d.properties = verify_props;
	ds->verify_psy_d.num_properties = ARRAY_SIZE(verify_props);
	ds->verify_psy_d.get_property = verify_get_property;
	ds->verify_psy_d.set_property = verify_set_property;
	ds->verify_psy_d.property_is_writeable = verify_prop_is_writeable;

	cfg.drv_data = ds;
	cfg.of_node = ds->dev->of_node;

	ds->verify_psy = devm_power_supply_register(ds->dev,
						    &ds->verify_psy_d, &cfg);
	if (IS_ERR(ds->verify_psy)) {
		pr_err("devm_power_supply_register failed\n");
		return PTR_ERR(ds->verify_psy);
	}

	pr_info("%s registered\n", ds->verify_psy_d.name);
	return 0;
}

static ssize_t ds_auth_result_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	int result = ds28e16_authenticate(ds);

	switch (result) {
	case DS_TRUE:
		return scnprintf(buf, PAGE_SIZE, "Authenticate success\n");
	case ERROR_R_STATUS:
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed: ERROR_R_STATUS\n");
	case ERROR_UNMATCH_MAC:
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed: MAC mismatch\n");
	case ERROR_R_ROMID:
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed: ERROR_R_ROMID\n");
	case ERROR_COMPUTE_MAC:
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed: ERROR_COMPUTE_MAC\n");
	case ERROR_S_SECRET:
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed: ERROR_S_SECRET\n");
	default:
		return scnprintf(buf, PAGE_SIZE,
				 "Authenticate failed: unknown (%d)\n", result);
	}
}

static ssize_t ds_romid_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 romid[8] = {};
	u32 tries;
	int i, count = 0;
	int status;

	mutex_lock(&ds->lock);
	tries = ds->attr_trytimes;
	mutex_unlock(&ds->lock);

	for (i = 0; i < tries; i++) {
		status = ds28e16_read_romid_retry(ds, romid);
		if (status == DS_TRUE)
			count++;
		usleep_range(1000, 1200);
	}

	return scnprintf(buf, PAGE_SIZE,
			 "Success=%d RomID=%8phC\n", count, romid);
}

static ssize_t ds_pagenumber_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	int val;

	mutex_lock(&ds->lock);
	val = ds->pagenumber;
	mutex_unlock(&ds->lock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t ds_pagenumber_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	int val, ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	if (val >= 0 && val <= 3) {
		mutex_lock(&ds->lock);
		ds->pagenumber = val;
		mutex_unlock(&ds->lock);
	}

	return count;
}

static ssize_t ds_pagedata_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 pagedata[16] = {};
	u32 tries;
	int pagenum, i, j, count = 0;

	mutex_lock(&ds->lock);
	tries = ds->attr_trytimes;
	pagenum = ds->pagenumber;
	mutex_unlock(&ds->lock);

	for (i = 0; i < tries; i++) {
		for (j = 0; j < GET_USER_MEMORY_RETRY; j++) {
			if (ds28e16_cmd_read_memory(ds, pagenum, pagedata) == DS_TRUE) {
				count++;
				break;
			}
		}
		usleep_range(1000, 1200);
	}

	return scnprintf(buf, PAGE_SIZE,
			 "Success=%d data=%16ph\n", count, pagedata);
}

/*
 * hex_parse - parse a comma-separated hex-byte string of length @len
 * into an output buffer using kernel standard hex2bin().
 *
 * Returns 0 on success, -EINVAL if invalid characters are encountered.
 */
static int hex_parse(const char *buf, u8 *out, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (hex2bin(&out[i], buf, 1) < 0)
			return -EINVAL;
		buf += 2;
		if (i < len - 1) {
			if (*buf != ',')
				return -EINVAL;
			buf++;
		}
	}
	return 0;
}

static ssize_t ds_pagedata_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 tmp[16];
	int ret, pagenum;

	ret = hex_parse(buf, tmp, 16);
	if (ret)
		return ret;

	mutex_lock(&ds->lock);
	pagenum = ds->pagenumber;
	mutex_unlock(&ds->lock);

	pr_debug("write page %d: %16ph\n", pagenum, tmp);

	if (ds28e16_cmd_write_memory(ds, pagenum, tmp) != DS_TRUE)
		pr_err("write_memory failed\n");

	return count;
}

static ssize_t ds_time_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u32 val;

	mutex_lock(&ds->lock);
	val = ds->attr_trytimes;
	mutex_unlock(&ds->lock);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t ds_time_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	if (val > 0) {
		mutex_lock(&ds->lock);
		ds->attr_trytimes = val;
		mutex_unlock(&ds->lock);
	}

	return count;
}

static ssize_t ds_session_seed_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 tmp[32];

	mutex_lock(&ds->lock);
	memcpy(tmp, ds->session_seed, 32);
	mutex_unlock(&ds->lock);

	return scnprintf(buf, PAGE_SIZE, "%32ph\n", tmp);
}

static ssize_t ds_session_seed_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 tmp[32];
	int ret;

	ret = hex_parse(buf, tmp, 32);
	if (ret)
		return ret;

	mutex_lock(&ds->lock);
	memcpy(ds->session_seed, tmp, 32);
	mutex_unlock(&ds->lock);

	return count;
}

static ssize_t ds_challenge_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 tmp[32];

	mutex_lock(&ds->lock);
	memcpy(tmp, ds->challenge, 32);
	mutex_unlock(&ds->lock);

	return scnprintf(buf, PAGE_SIZE, "%32ph\n", tmp);
}

static ssize_t ds_challenge_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 tmp[32];
	int ret;

	ret = hex_parse(buf, tmp, 32);
	if (ret)
		return ret;

	mutex_lock(&ds->lock);
	memcpy(ds->challenge, tmp, 32);
	mutex_unlock(&ds->lock);

	return count;
}

static ssize_t ds_s_secret_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 tmp[32];

	mutex_lock(&ds->lock);
	memcpy(tmp, ds->s_secret, 32);
	mutex_unlock(&ds->lock);

	return scnprintf(buf, PAGE_SIZE, "%32ph\n", tmp);
}

static ssize_t ds_s_secret_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 tmp[32];
	int ret;

	ret = hex_parse(buf, tmp, 32);
	if (ret)
		return ret;

	mutex_lock(&ds->lock);
	memcpy(ds->s_secret, tmp, 32);
	mutex_unlock(&ds->lock);

	return count;
}

static ssize_t ds_auth_anon_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	int val;

	mutex_lock(&ds->lock);
	val = ds->auth_anon;
	mutex_unlock(&ds->lock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t ds_auth_anon_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	int val, ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&ds->lock);
	ds->auth_anon = !!val;
	mutex_unlock(&ds->lock);

	return count;
}

static ssize_t ds_auth_bdconst_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	int val;

	mutex_lock(&ds->lock);
	val = ds->auth_bdconst;
	mutex_unlock(&ds->lock);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t ds_auth_bdconst_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	int val, ret;

	ret = kstrtoint(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&ds->lock);
	ds->auth_bdconst = !!val;
	mutex_unlock(&ds->lock);

	return count;
}

static ssize_t ds_readstatus_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct ds28e16_data *ds = dev_get_drvdata(dev);
	u8 status[16] = {};
	u32 tries;
	int i, j, count = 0;

	mutex_lock(&ds->lock);
	tries = ds->attr_trytimes;
	mutex_unlock(&ds->lock);

	for (i = 0; i < tries; i++) {
		for (j = 0; j < GET_BLOCK_STATUS_RETRY; j++) {
			if (ds28e16_cmd_read_status(ds, status) == DS_TRUE) {
				count++;
				break;
			}
		}
		usleep_range(1000, 1200);
	}

	return scnprintf(buf, PAGE_SIZE,
			 "Success=%d status=%16ph\n", count, status);
}

static DEVICE_ATTR_RO(ds_readstatus);
static DEVICE_ATTR_RO(ds_romid);
static DEVICE_ATTR(ds_pagenumber, 0660, ds_pagenumber_show, ds_pagenumber_store);
static DEVICE_ATTR(ds_pagedata, 0660, ds_pagedata_show, ds_pagedata_store);
static DEVICE_ATTR(ds_time, 0660, ds_time_show, ds_time_store);
static DEVICE_ATTR(ds_session_seed, 0600, ds_session_seed_show, ds_session_seed_store);
static DEVICE_ATTR(ds_challenge, 0600, ds_challenge_show, ds_challenge_store);
static DEVICE_ATTR(ds_s_secret, 0600, ds_s_secret_show, ds_s_secret_store);
static DEVICE_ATTR(ds_auth_anon, 0660, ds_auth_anon_show, ds_auth_anon_store);
static DEVICE_ATTR(ds_auth_bdconst, 0660, ds_auth_bdconst_show, ds_auth_bdconst_store);
static DEVICE_ATTR_RO(ds_auth_result);

static struct attribute *ds_attrs[] = {
	&dev_attr_ds_readstatus.attr,
	&dev_attr_ds_romid.attr,
	&dev_attr_ds_pagenumber.attr,
	&dev_attr_ds_pagedata.attr,
	&dev_attr_ds_time.attr,
	&dev_attr_ds_session_seed.attr,
	&dev_attr_ds_challenge.attr,
	&dev_attr_ds_s_secret.attr,
	&dev_attr_ds_auth_anon.attr,
	&dev_attr_ds_auth_bdconst.attr,
	&dev_attr_ds_auth_result.attr,
	NULL,
};

static const struct attribute_group ds_attr_group = {
	.attrs = ds_attrs,
};

#define AUTHENTIC_PERIOD_MS	500
#define AUTHENTIC_COUNT_MAX	10

static void authentic_work(struct work_struct *work)
{
	struct ds28e16_data *ds = container_of(work, struct ds28e16_data,
					       authentic_work.work);
	int i, result = 0;

	for (i = 0; i < 3; i++) {
		result = ds28e16_authenticate(ds);
		if (result == DS_TRUE)
			break;
		usleep_range(100, 200);
	}

	if (result == DS_TRUE) {
		WRITE_ONCE(ds->batt_verified, true);
		ds->auth_retry_count = 0;
		power_supply_changed(ds->verify_psy);
		pr_info("battery verified\n");
	} else {
		ds->auth_retry_count++;
		if (ds->auth_retry_count < AUTHENTIC_COUNT_MAX) {
			schedule_delayed_work(&ds->authentic_work,
					      msecs_to_jiffies(AUTHENTIC_PERIOD_MS));
			pr_info("verification retry %d/%d\n",
				ds->auth_retry_count, AUTHENTIC_COUNT_MAX);
		} else {
			WRITE_ONCE(ds->batt_verified, false);
			ds->auth_retry_count = 0;
			power_supply_changed(ds->verify_psy);
			pr_err("battery verification failed after %d attempts: %d\n",
			       AUTHENTIC_COUNT_MAX, result);
		}
	}
}

/**
 * ds28e16_init_defaults - populate the per-instance auth parameters with
 * the platform-default values.
 *
 * These values must match what is provisioned into the chip's Secret Page
 * at manufacturing time. They can be overridden via sysfs if needed.
 *
 * Called before any sysfs or workqueue context can access the fields,
 * so no locking is needed here.
 */
static void ds28e16_init_defaults(struct ds28e16_data *ds)
{
	static const u8 default_session_seed[32] = {
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
		0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
	};
	static const u8 default_s_secret[32] = {
		0x0C, 0x99, 0x2B, 0xD3, 0x95, 0xDB, 0xA0, 0xB4,
		0xEF, 0x07, 0xB3, 0xD8, 0x75, 0xF3, 0xC7, 0xAE,
		0xDA, 0xC4, 0x41, 0x2F, 0x48, 0x93, 0xB5, 0xD9,
		0xE1, 0xE5, 0x4B, 0x20, 0x9B, 0xF3, 0x77, 0x39,
	};

	memcpy(ds->session_seed, default_session_seed, 32);
	memcpy(ds->s_secret, default_s_secret, 32);
	memset(ds->challenge, 0x00, 32);

	ds->auth_anon = true;
	ds->auth_bdconst = true;
	ds->pagenumber = 1;
	ds->attr_trytimes = 1;
	ds->mi_auth_result = 0;
}

static int ds28e16_parse_dt(struct device *dev, struct ds28e16_data *ds)
{
	struct device_node *np = dev->of_node;
	const char *label;
	int ret;

	ret = of_property_read_string(np, "xiaomi,onewire-bus", &label);
	if (ret) {
		pr_err("missing 'xiaomi,onewire-bus' property: %d\n", ret);
		return ret;
	}
	strscpy(ds->ow_bus_label, label, sizeof(ds->ow_bus_label));
	pr_debug("onewire bus: '%s'\n", ds->ow_bus_label);

	return 0;
}

static int ds28e16_probe(struct platform_device *pdev)
{
	struct ds28e16_data *ds;
	struct onewire_gpio_data *ow_bus;
	int ret;

	pr_info("entry\n");

	if (strcmp(pdev->name, "soc:maxim_ds28e16") != 0)
		return -ENODEV;

	if (!pdev->dev.of_node || !of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	ds = devm_kzalloc(&pdev->dev, sizeof(*ds), GFP_KERNEL);
	if (!ds)
		return -ENOMEM;

	ret = ds28e16_parse_dt(&pdev->dev, ds);
	if (ret) {
		pr_err("parse_dt failed: %d\n", ret);
		return ret;
	}

	ow_bus = onewire_bus_get(ds->ow_bus_label);
	if (!ow_bus) {
		pr_info("onewire bus '%s' not ready, deferring\n",
			ds->ow_bus_label);
		return -EPROBE_DEFER;
	}

	ds->dev = &pdev->dev;
	ds->pdev = pdev;
	ds->ow_bus = ow_bus;
	platform_set_drvdata(pdev, ds);

	mutex_init(&ds->lock);
	ds28e16_init_defaults(ds);

	INIT_DELAYED_WORK(&ds->authentic_work, authentic_work);

	ret = sysfs_create_group(&ds->dev->kobj, &ds_attr_group);
	if (ret) {
		pr_err("sysfs_create_group failed: %d\n", ret);
		goto err_out;
	}

	ret = verify_psy_register(ds);
	if (ret) {
		pr_err("verify_psy_register failed: %d\n", ret);
		goto err_sysfs;
	}

	if (ds28e16_authenticate(ds) == DS_TRUE) {
		WRITE_ONCE(ds->batt_verified, true);
		power_supply_changed(ds->verify_psy);
		pr_info("battery verified on first attempt\n");
	} else {
		schedule_delayed_work(&ds->authentic_work, msecs_to_jiffies(100));
	}

	pr_info("success\n");
	return 0;

err_sysfs:
	sysfs_remove_group(&ds->dev->kobj, &ds_attr_group);
err_out:
	mutex_destroy(&ds->lock);
	platform_set_drvdata(pdev, NULL);
	onewire_bus_put(ow_bus);
	pr_err("probe failed: %d\n", ret);
	return ret;
}

static int ds28e16_remove(struct platform_device *pdev)
{
	struct ds28e16_data *ds = platform_get_drvdata(pdev);

	if (!ds)
		return 0;

	cancel_delayed_work_sync(&ds->authentic_work);
	sysfs_remove_group(&ds->dev->kobj, &ds_attr_group);
	mutex_destroy(&ds->lock);
	platform_set_drvdata(pdev, NULL);
	onewire_bus_put(ds->ow_bus);
	return 0;
}

static const struct of_device_id ds28e16_dt_match[] = {
	{ .compatible = "maxim,ds28e16", },
	{ },
};
MODULE_DEVICE_TABLE(of, ds28e16_dt_match);

static struct platform_driver ds28e16_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "maxim_ds28e16",
		.of_match_table	= of_match_ptr(ds28e16_dt_match),
	},
	.probe	= ds28e16_probe,
	.remove	= ds28e16_remove,
};

static int __init ds28e16_init(void)
{
	return platform_driver_register(&ds28e16_driver);
}

static void __exit ds28e16_exit(void)
{
	platform_driver_unregister(&ds28e16_driver);
}

subsys_initcall_sync(ds28e16_init);
module_exit(ds28e16_exit);

MODULE_AUTHOR("Xiaomi Inc.");
MODULE_DESCRIPTION("DS28E16 DeepCover Secure Authenticator driver");
MODULE_LICENSE("GPL");
