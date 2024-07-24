/* SPDX-License-Identifier: Apache-2.0 OR GPL-2.0-or-later */

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef MCTP_HAVE_FILEIO
#include <unistd.h>
#include <fcntl.h>
#endif

#define pr_fmt(x) "i2c: " x

#if __linux__
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#endif

#if defined __ZEPHYR__
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>
#include <zephyr/sys/crc.h>
#else
#include "crc8.h"
#endif

#include "libmctp.h"
#include "libmctp-alloc.h"
#include "libmctp-log.h"
#include "libmctp-i2c.h"
#include "container_of.h"
#define binding_to_i2c(b)     container_of(b, struct mctp_binding_i2c, binding)
#define BMC_ADDR	      0x12
#define MCTP_I2C_COMMAND_CODE 0x0f
#define SMBUS_PEC_SIZE	      1
#define MCTP_I2C_MTU                                                           \
	sizeof(struct mctp_i2c_hdr) + MCTP_PACKET_SIZE(MCTP_BTU) +             \
		SMBUS_PEC_SIZE
#define LRU_OLDEST_AGE 255
#define EID_CACHE_SIZE 16

struct eid_lru_entry {
	uint8_t eid;
	uint8_t addr;
	uint8_t age;
};
struct mctp_binding_i2c {
	struct mctp_binding binding;
	unsigned long bus_id;
	uint8_t src_addr;
	struct eid_lru_entry eid_cache[EID_CACHE_SIZE];
	uint8_t (*mctp_to_i2c_addr)(struct mctp_binding_i2c *i2c, uint8_t eid);
	mctp_i2c_tx_fn tx_fn;
	const void *tx_fn_data;
#ifdef MCTP_HAVE_FILEIO
	int in_fd;
	int out_fd;
#endif
};

struct mctp_i2c_hdr {
	uint8_t dest;
	uint8_t command;
	uint8_t byte_count;
	uint8_t src;
} __attribute__((packed));

static void cache_addr(struct mctp_binding_i2c *i2c, uint8_t eid, uint8_t addr)
{
	int oldest = 0;
	for (int i = 0; i < EID_CACHE_SIZE; i++) {
		if (i2c->eid_cache[i].eid == eid) {
			i2c->eid_cache[i].age = 0;
			i2c->eid_cache[i].addr = addr;
		}
		if (i2c->eid_cache[i].age > i2c->eid_cache[oldest].age) {
			oldest = i;
		}
	}

	i2c->eid_cache[oldest].eid = eid;
	i2c->eid_cache[oldest].addr = addr;
	i2c->eid_cache[oldest].age = 0;
}

static uint8_t lookup_addr(struct mctp_binding_i2c *i2c, uint8_t eid)
{
	uint8_t hit = 0xff;
	for (int i = 0; i < EID_CACHE_SIZE; i++) {
		if (i2c->eid_cache[i].age < 255) {
			i2c->eid_cache[i].age++;
		}
		if (i2c->eid_cache[i].eid == eid) {
			i2c->eid_cache[i].age = 0;
			hit = i2c->eid_cache[i].addr;
		}
	}
	return hit;
}

//charles copy below code from kernel to ensure pec result
#define POLY    (0x1070U << 3)
static uint8_t crc8_kernel(uint16_t data)
{
	int i;

	for (i = 0; i < 8; i++) {
		if (data & 0x8000)
			data = data ^ POLY;
		data = data << 1;
	}
	return (uint8_t)(data >> 8);
}

/**
 * i2c_smbus_pec - Incremental CRC8 over the given input data array
 * @crc: previous return crc8 value
 * @p: pointer to data buffer.
 * @count: number of bytes in data buffer.
 *
 * Incremental CRC8 over count bytes in the array pointed to by p
 */
uint8_t i2c_smbus_pec(uint8_t crc, uint8_t *p, size_t count)
{
	int i;

	for (i = 0; i < count; i++)
		crc = crc8_kernel((crc ^ p[i]) << 8);
	return crc;
}
static uint8_t cal_pec(uint8_t *data, uint8_t len)
{
	// return crc8(data, len, 0x07, 0x00, false);
	uint8_t crc=0;
	return i2c_smbus_pec(crc, data, len);
}

static int mctp_binding_i2c_tx(struct mctp_binding *b, struct mctp_pktbuf *pkt)
{
	struct mctp_binding_i2c *i2c = binding_to_i2c(b);
	struct mctp_i2c_hdr *i2c_hdr;
	struct mctp_hdr *mctp_hdr = mctp_pktbuf_hdr(pkt);
	uint8_t len = mctp_pktbuf_size(pkt);
	uint8_t *pec;
	int dest;

	dest = lookup_addr(i2c, mctp_hdr->dest);
	if (dest == 0xff)
		return -1; //unknown EID

	// mctp_prdebug("mctp_binding_i2c_tx dest eid=0x%02x i2c=%d\n",
	// 	     mctp_hdr->dest, dest);

	i2c_hdr = (struct mctp_i2c_hdr *)pkt->data;
	i2c_hdr->dest = (uint8_t)(dest & 0xfe);
	i2c_hdr->command = MCTP_I2C_COMMAND_CODE;
	i2c_hdr->byte_count = len + sizeof(i2c_hdr->src);
	i2c_hdr->src = i2c->src_addr;

	len += sizeof(struct mctp_i2c_hdr);
	pec = pkt->data + len; // find the offset of PEC

	*pec = cal_pec(pkt->data, len);
	len++;

	if (!i2c->tx_fn) {
		mctp_prdebug("%s tx_fn is null\n", __func__);
		return 0;
	}
	return i2c->tx_fn(i2c->tx_fn_data, pkt->data, len);
}

static int mctp_binding_i2c_start(struct mctp_binding *binding)
{
	struct mctp_binding_i2c *i2c = binding_to_i2c(binding);

	/*
	to-do: imp dicovery EP's physical addr on the bus
	  send getEndPoint ID to each possible i2c addr
	  if got response, assigned a new EID
	note:
	  mctp-demux will be replaced when the in-kernel MCTP patch got merged
	  so keep it simple for now
	*/
	cache_addr(i2c, 0x12, 0x80);
	cache_addr(i2c, 0x13, 0x81);
	cache_addr(i2c, 0x14, 0x82);
	cache_addr(i2c, 0x15, 0x83);

	mctp_binding_set_tx_enabled(binding, true);

	return 0;
}

struct mctp_binding *mctp_binding_i2c_core(struct mctp_binding_i2c *b)
{
	return &b->binding;
}

void mctp_i2c_set_tx_fn(struct mctp_binding_i2c *i2c, mctp_i2c_tx_fn fn,
			const void *data)
{
	i2c->tx_fn = fn;
	i2c->tx_fn_data = data;
}

int mctp_i2c_rx(struct mctp_binding_i2c *i2c, uint8_t *buf, uint32_t len)
{
	uint8_t expected;
	struct mctp_i2c_hdr *i2c_hdr;
	struct mctp_pktbuf *pkt;
	struct mctp_hdr *hdr;

	if (len < sizeof(struct mctp_i2c_hdr) + SMBUS_PEC_SIZE) {
		mctp_prdebug("%s received %d bytes is not enough\n", __func__, len);
		return 0;
	}
	mctp_prdebug("%s received %d bytes\n", __func__, len);

	expected = cal_pec(buf, len - 1);
	mctp_prdebug("%s PEC(expected=0x%02x, pec=0x%02x )\n",
			     __func__, expected, buf[len - 1]);
	if (expected != buf[len - 1]) {
		mctp_prerr("%s invalid PEC(expected=0x%02x, pec=0x%02x )\n",
			     __func__, expected, buf[len - 1]);
		return -3;
	}

	i2c_hdr = (struct mctp_i2c_hdr *)buf;
	// subtract 4 bytes that not include in byte_count(byte0~3 and last byte)
	expected = len - 4;
	if (expected != i2c_hdr->byte_count) {
		mctp_prdebug("%s wrong mctp message length\n", __func__);
		return -4;
	}

	if (i2c_hdr->command != MCTP_I2C_COMMAND_CODE)
		return -5;

	pkt = mctp_pktbuf_alloc(&i2c->binding, i2c_hdr->byte_count - 1);
	if (!pkt) {
		mctp_prdebug("%s out of mmeory\n", __func__);
		return -5;
	}
	memcpy(pkt->data, buf, len);
	hdr = mctp_pktbuf_hdr(pkt);
	cache_addr(i2c, hdr->src, i2c_hdr->src);
	mctp_bus_rx(&i2c->binding, pkt);

	return 0;
}

struct mctp_binding_i2c *mctp_i2c_init(uint8_t addr)
{
	struct mctp_binding_i2c *i2c;

	i2c = __mctp_alloc(sizeof(*i2c));
	memset(i2c, 0, sizeof(*i2c));

	i2c->binding.name = "i2c";
	i2c->binding.version = 1;
	i2c->binding.pkt_size = MCTP_PACKET_SIZE(MCTP_BTU);
	i2c->binding.pkt_header = 4; // dest, cmd,byte count, src
	i2c->binding.pkt_trailer = 1; //pec

	i2c->binding.start = mctp_binding_i2c_start;
	i2c->binding.tx = mctp_binding_i2c_tx;

	i2c->src_addr =
		addr << 1 | 0x01; //per the desc. of DSP0237 Table-1, byte4

	for (int i = 0; i < EID_CACHE_SIZE; i++) {
		i2c->eid_cache[i].age = LRU_OLDEST_AGE;
	}
	mctp_prdebug("set log level to debug\n");
	mctp_set_log_stdio(MCTP_LOG_DEBUG);

	return i2c;
}

void mctp_i2c_destroy(struct mctp_binding_i2c *i2c)
{
	__mctp_free(i2c);
}

#ifdef MCTP_HAVE_FILEIO
int mctp_i2c_read(struct mctp_binding_i2c *i2c)
{
	uint8_t buf[256];
	int ret;
	ssize_t len;

	ret = lseek(i2c->in_fd, 0, SEEK_SET);
	if (ret < 0) {
		mctp_prerr("Failed to seek");
		return -1;
	}

	len = read(i2c->in_fd, buf, sizeof(buf));
	if (len < 0) {
		mctp_prerr("Failed to read");
		return -1;
	}

	return mctp_i2c_rx(i2c, buf, len);
}

static int mctp_i2c_write(const void *fn_data, uint8_t *buf, uint32_t len)
{
	uint8_t *tx_buf = (uint8_t *)buf;
	struct mctp_binding_i2c *i2c = (struct mctp_binding_i2c *)fn_data;
	struct i2c_msg msg;
	struct i2c_rdwr_ioctl_data data = { &msg, 1 };
	int ret;

#if 0
	mctp_prdebug("%s xfer %d bytes to addr 0x%02x\n data:", __func__, len, tx_buf[0]);
	for(int i=1; i<len;i++) {
		mctp_prdebug("0x%02x ", tx_buf[i]);
	}
#endif

	msg.addr = tx_buf[0] >> 1;
	msg.flags = 0;
	msg.len = len - 1;
	msg.buf = tx_buf + 1;

	ret = ioctl(i2c->out_fd, I2C_RDWR, &data);
	if (ret < 0) {
		mctp_prdebug("%s: ioctl ret = %d", __func__, ret);
		return ret;
	}
	return 0;
}

int mctp_i2c_open_fd(struct mctp_binding_i2c *i2c, uint8_t bus)
{
	uint8_t path[256];

	snprintf(path, sizeof(path),
		 "/sys/bus/i2c/devices/%d-10%02x/slave-mqueue", bus,
		 i2c->src_addr >> 1);
	mctp_prdebug("open %s\n", path);
	i2c->in_fd = open(path, O_RDONLY);
	if (i2c->in_fd < 0) {
		mctp_prdebug("cannot open %s\n", path);
		return -1;
	}

	snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
	mctp_prdebug("open %s\n", path);
	i2c->out_fd = open(path, O_RDWR);
	if (i2c->out_fd < 0) {
		mctp_prdebug("cannot open %s\n", path);
		return -1;
	}

	mctp_i2c_set_tx_fn(i2c, mctp_i2c_write, i2c);

	return 0;
}

int mctp_i2c_get_fd(struct mctp_binding_i2c *i2c)
{
	return i2c->in_fd;
}

#endif