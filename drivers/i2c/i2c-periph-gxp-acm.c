// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>


#define ACM_ADDR 0x20
#define GXP_ADDR   0x10
#define ACM_CMD	0x46
#define ACM_VER	1
#define ACM_PACKET_SIZE 72
#define ACM_CHECKSUM_SIZE 1
#define ACM_BLOCK_SIZE PAGE_SIZE
#define IN_BLOCKS_NUM 5

enum ACM_ERR {
	ACM_OK = 0,
	ACM_ERROR = 1,
	ACM_INVALID_REQUEST = 2,
	ACM_BUSY = 3,
	ACM_PROTOCOL_ERR = 4,
	ACM_BUFF_SIZE_TOO_LARGE = 5,
	ACM_INPROGRESS = 6,
	ACM_UNSUPPORTED = 7,
	ACM_FLASHING = 8,
	ACM_FLASHING_APM = 9,
	ACM_CRC_ERROR = 10,
	ACM_VASCOMM_ERR = 11,
	ACM_BUF_TO_SMALL = 12,
	ACM_PWR_NOT_RDY = 13,
	ACM_QUEUE_FULL = 14,
	ACM_FLASH_SEEK_ERR = 15,
	ACM_FLASH_RD_ERR = 16,
	ACM_CFG_FILE_ORW_ERR = 17
};

enum ACM_INST {
	ACM_BLOCK_RD = 1,
	ACM_BLOCK_WR = 2
};

struct drvdata_gxp_acm {
	struct gpio_desc *irq_n;
	struct bin_attribute bin_in;
	struct kernfs_node *kn_in;
	struct bin_attribute bin_out;
	struct kernfs_node *kn_out;
	u8 seq_num;
	u8 buf_idx;
	u8 buf_len;
	u8 buf_wr_resp;
	u8 buf[ACM_PACKET_SIZE];
	u8 out_block_valid;
	u8 out_block[ACM_BLOCK_SIZE];	// from user to gxp_acm
	int out_block_sent_len;
	u8 in_blocks[IN_BLOCKS_NUM][ACM_BLOCK_SIZE];	// from gxp_acm to user
	u8 in_blocks_head;
	u8 in_blocks_tail;
	u8 in_blocks_len;
	int in_block_recv_len;
	int in_block_drop_cnt; // for debug
	int in_block_recv_cnt; // for debug
};

struct acm_packet_header {
	u8 command;
	u8 src_addr;
	u8 payload_cnt;
	u8 instruction;
	u8 version;
	struct {
		u8 dest:4;
		u8 src:4;
	} lun;
	struct {
		u8 seq:4;
		u8 resp_not_req:1;
		u8 resp:1;
		u8 end:1;
		u8 start:1;
	} ctrl_bits;
} __packed;

struct acm_packet {
	struct acm_packet_header header;
	u8 payload[ACM_PACKET_SIZE - sizeof(struct acm_packet_header)];
} __packed;

struct  acm_block_header {
	u16 length;
	u8 instruction;
} __packed;

struct acm_block {
	struct acm_block_header header;
	u8 payload[ACM_BLOCK_SIZE - sizeof(struct acm_block_header)];
} __packed;

static u8 PEC_CRC8[] = {
//x0    x1    x2    x3    x4    x5    x6    x7    x8    x9    xa    xb    xc    xd    xe    xf
0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,// 00
0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,// 10
0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,// 20
0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,// 30
0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,// 40
0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,// 50
0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,// 60
0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,// 70
0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,// 80
0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,// 90
0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,// A0
0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,// B0
0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,// C0
0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,// D0
0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,// E0
0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,// F0
};

int gxp_acm_crc_check(u8 *buf, u8 buf_len, u8 i2c_addr)
{
	u8 crc_calc;
	int i;

	crc_calc = 0;
	crc_calc = PEC_CRC8[(i2c_addr ^ crc_calc)]; //pre load dest i2c addr to start the calc.  Should always be 0x10

	// The last byte has the CRC
	for (i = 0; i < (buf_len-1); i++)
		crc_calc = PEC_CRC8[(buf[i] ^ crc_calc)];

	return (crc_calc != buf[buf_len]) ? -1:0;
}

u8 gxp_acm_crc_calc(u8 *buf, u8 buf_len, u8 i2c_addr)
{
	u8 crc;
	int i;

	crc = 0;
	crc = PEC_CRC8[(i2c_addr ^ crc)];  //preload dest addr

	// Calculate the CRC
	for (i = 0; i < buf_len; i++)
		crc = PEC_CRC8[(buf[i] ^ crc)];

	return crc;
}

static int gxp_acm_write_resp(struct drvdata_gxp_acm *drvdata, int block_number, u8 instruction, u8 rc)
{
	struct acm_packet *packet = (struct acm_packet *)drvdata->buf;

	packet->header.command = ACM_CMD;
	packet->header.src_addr = GXP_ADDR;
	packet->header.payload_cnt = 3;
	packet->header.instruction = instruction;
	packet->header.version = ACM_VER;
	packet->header.lun.src = 0x0;
	packet->header.lun.dest = 0x0;
	packet->header.ctrl_bits.start = 1;
	packet->header.ctrl_bits.end = 1;
	packet->header.ctrl_bits.resp = 1;
	packet->header.ctrl_bits.resp_not_req = 1;
	packet->header.ctrl_bits.seq = drvdata->seq_num;
	packet->payload[0] = (block_number >> 8) & 0xff;
	packet->payload[1] = block_number & 0xff;
	packet->payload[2] = rc;
	drvdata->buf_len = sizeof(struct acm_packet_header) + packet->header.payload_cnt;
	drvdata->buf[drvdata->buf_len] = gxp_acm_crc_calc(drvdata->buf, drvdata->buf_len, GXP_ADDR);
	drvdata->buf_len++; //inc 1 for crc
	drvdata->buf_idx = 0;
	drvdata->buf_wr_resp = 1;
	gpiod_set_value(drvdata->irq_n, 0); // raise ACM IRQ

	return 0;
}

static int gxp_acm_write_packet_done(struct drvdata_gxp_acm *drvdata)
{
	struct acm_packet *packet = (struct acm_packet *)drvdata->buf;
	struct acm_block *in_block = (struct acm_block *)drvdata->in_blocks[drvdata->in_blocks_tail];
	u16 in_block_length;

	if (packet->header.command != ACM_CMD)
		return -1;	// has to be 'F'(0x46)

	if (packet->header.src_addr != ACM_ADDR)
		return -2;	// has to be 0x20 for packet from ACM

	if (packet->header.ctrl_bits.resp != 0)
		return -4;	// has to be 0x0

	drvdata->seq_num = packet->header.ctrl_bits.seq;

	if (packet->header.ctrl_bits.start && packet->header.ctrl_bits.end) {
		//single packet block
		u8 rc = ACM_OK;
		u16 block_number;

		//todo: verify CRC(the last byte of packet)
		memcpy(in_block->payload, packet->payload, packet->header.payload_cnt);
		block_number = in_block->payload[0] << 8 |  in_block->payload[1]; //payload is big endian
		in_block_length = packet->header.payload_cnt;

		in_block->header.length = cpu_to_be16(in_block_length);
		in_block->header.instruction = packet->header.instruction;
		drvdata->in_block_recv_len = packet->header.payload_cnt;

		drvdata->in_blocks_len++;
		drvdata->in_blocks_tail++;
		if (drvdata->in_blocks_tail >= IN_BLOCKS_NUM)
			drvdata->in_blocks_tail = 0;

		kernfs_notify(drvdata->kn_in);
		if ((packet->header.ctrl_bits.resp_not_req == 0) && (packet->header.instruction == ACM_BLOCK_WR)) {
			//response for write block instruction only, the read block instruction is responded by user space.
			gxp_acm_write_resp(drvdata, block_number, packet->header.instruction, rc);
		}
	} else if (packet->header.ctrl_bits.start && !packet->header.ctrl_bits.end) {
		//first packet of multi packet block
		//todo: verify CRC(the last byte of packet)
		in_block->payload[0] = packet->payload[0];
		in_block->payload[1] = packet->payload[1];
		in_block_length = (packet->payload[2] << 8) | packet->payload[3]; //payload is big endian

		in_block->header.length = cpu_to_be16(in_block_length);
		in_block->header.instruction = packet->header.instruction;
		memcpy(&in_block->payload[2], &packet->payload[4], packet->header.payload_cnt - 2);
		drvdata->in_block_recv_len = packet->header.payload_cnt - 2; //excuse block length bytes(2)
	} else if (!packet->header.ctrl_bits.start && !packet->header.ctrl_bits.end) {
		//middle packets of multi packet block
		//todo: verify CRC(the last byte of packet)
		memcpy(&in_block->payload[drvdata->in_block_recv_len], packet->payload, packet->header.payload_cnt);
		drvdata->in_block_recv_len += packet->header.payload_cnt;
	} else {
		//last packets of multi packet block
		u8 rc = ACM_OK;
		u16 block_number = in_block->payload[0] << 8 |  in_block->payload[1]; //payload is big endian
		//todo: verify CRC(the last byte of packet)
		memcpy(&in_block->payload[drvdata->in_block_recv_len], packet->payload, packet->header.payload_cnt);
		drvdata->in_block_recv_len += packet->header.payload_cnt;
		in_block_length = be16_to_cpu(in_block->header.length);
		if (in_block_length != drvdata->in_block_recv_len) {
			in_block->header.length = cpu_to_be16(0);
			rc = ACM_ERROR;
		}

		drvdata->in_blocks_len++;
		drvdata->in_blocks_tail++;
		if (drvdata->in_blocks_tail >= IN_BLOCKS_NUM)
			drvdata->in_blocks_tail = 0;

		kernfs_notify(drvdata->kn_in);
		if (packet->header.ctrl_bits.resp_not_req == 0)
			gxp_acm_write_resp(drvdata, block_number, packet->header.instruction, rc);
	}

	return 0;
}

static int gxp_acm_out_start(struct drvdata_gxp_acm *drvdata)
{
	// user layer do dummy read "bin_out" to notify driver that data in out_block is ready for ACM
	struct acm_packet *packet = (struct acm_packet *)drvdata->buf;
	struct acm_block *out_block = (struct acm_block *)drvdata->out_block;
	int out_block_length = be16_to_cpu(out_block->header.length);
	int block_number = (out_block->payload[0]<<8) |  out_block->payload[1];

	if (out_block_length == 0)
		return -EIO;

	if (out_block_length <= 64) {
		//single packet block
		memcpy(packet->payload, out_block->payload, out_block_length);
		packet->header.payload_cnt = out_block_length;
		packet->header.ctrl_bits.start = 1;
		packet->header.ctrl_bits.end = 1;
	} else {
		//first packet of multi packet block
		packet->payload[0] = out_block->payload[0]; // block number
		packet->payload[1] = out_block->payload[1];
		packet->payload[2] = (out_block_length >> 8) & 0xff; // packet is in big endian order
		packet->payload[3] = out_block_length & 0xff;
		packet->payload[4] = out_block->payload[2]; // return code
		memcpy(&packet->payload[4], &out_block->payload[3], 59); //block data
		packet->header.payload_cnt = 64; //block#(2) + block_len(2) + RC(1) + block data(59)

		packet->header.ctrl_bits.start = 1;
		packet->header.ctrl_bits.end = 0;
	}
	packet->header.command = ACM_CMD;
	packet->header.instruction = out_block->header.instruction;
	packet->header.src_addr = GXP_ADDR;
	packet->header.version = ACM_VER;
	packet->header.lun.src = 0x0;
	packet->header.lun.dest = 0x0;
	packet->header.ctrl_bits.resp = (packet->header.instruction == ACM_BLOCK_RD)?1:0;
	packet->header.ctrl_bits.resp_not_req = 1;
	packet->header.ctrl_bits.seq = block_number % 0x0f;
	drvdata->buf_len = sizeof(struct acm_packet_header) + packet->header.payload_cnt;

	drvdata->buf[drvdata->buf_len] = gxp_acm_crc_calc(drvdata->buf, drvdata->buf_len, ACM_ADDR);
	drvdata->buf_len++; //inc 1 for crc
	drvdata->buf_wr_resp = 0;
	drvdata->buf_idx = 0;
	drvdata->out_block_sent_len = 0;
	drvdata->out_block_valid = 1;

	gpiod_set_value(drvdata->irq_n, 0); // raise ACM IRQ
	return 0;
}

static int gxp_acm_read_packet_done(struct drvdata_gxp_acm *drvdata)
{
	struct acm_packet *packet = (struct acm_packet *)drvdata->buf;
	struct acm_block *out_block = (struct acm_block *)drvdata->out_block;
	int remaining_length;
	int out_block_length = be16_to_cpu(out_block->header.length);
	int block_number = (out_block->payload[0]<<8) |  out_block->payload[1];

	if (drvdata->buf_wr_resp) {
		//ACM done the write block response, deassert irq
		drvdata->out_block_valid = 0;
		gpiod_set_value(drvdata->irq_n, 1);
		return 0;
	}

	drvdata->out_block_sent_len += drvdata->buf_len - sizeof(struct acm_packet_header) - ACM_CHECKSUM_SIZE;
	if (out_block_length > 64) {
		// multi packet block send 2 extra bytes for block length in first packet
		out_block_length += 2;
	}
	remaining_length = out_block_length - drvdata->out_block_sent_len;

	if (remaining_length == 0) {
		// all packet send out, lower ACM IRQ
		drvdata->out_block_valid = 0;
		gpiod_set_value(drvdata->irq_n, 1);
	} else {
		if (remaining_length > 64) {
			//middle packet of multi packet block
			memcpy(packet->payload, out_block->payload, 64);
			packet->header.payload_cnt = 64;
			packet->header.ctrl_bits.start = 0;
			packet->header.ctrl_bits.end = 0;
			packet->header.ctrl_bits.resp = 0;
		} else {
			//last packet of multi packet block
			memcpy(packet->payload, out_block->payload, remaining_length);
			packet->header.payload_cnt = remaining_length;
			packet->header.ctrl_bits.start = 0;
			packet->header.ctrl_bits.end = 1;
			packet->header.ctrl_bits.resp = 0;
		}
		packet->header.command = ACM_CMD;
		packet->header.instruction = out_block->header.instruction;
		packet->header.src_addr = GXP_ADDR;
		packet->header.version = ACM_VER;
		packet->header.lun.src = 0x0;
		packet->header.lun.dest = 0x0;
		packet->header.ctrl_bits.resp = (packet->header.instruction == ACM_BLOCK_RD)?1:0;
		packet->header.ctrl_bits.resp_not_req = 1;
		packet->header.ctrl_bits.seq = block_number % 0x0f;

		drvdata->buf_len = sizeof(struct acm_packet_header) + packet->header.payload_cnt;
		drvdata->buf[drvdata->buf_len] = gxp_acm_crc_calc(drvdata->buf, drvdata->buf_len, ACM_ADDR);
		drvdata->buf_len++; //inc 1 for crc
		drvdata->buf_idx = 0;
		drvdata->buf_wr_resp = 0;
	}

	return 0;
}

static int gxp_acm_cb(struct i2c_client *client,
				     enum i2c_slave_event event, u8 *val)
{
	struct drvdata_gxp_acm *drvdata = i2c_get_clientdata(client);
	struct acm_packet *packet = (struct acm_packet *)drvdata->buf;
	int ret;

	switch (event) {
	case I2C_SLAVE_WRITE_REQUESTED:
		if (drvdata->in_blocks_len > 0 && drvdata->in_blocks_head == drvdata->in_blocks_tail) {
			drvdata->in_block_drop_cnt++;
			return -ENOMEM;
		}

		drvdata->in_block_recv_len = 0;
		drvdata->buf_len = 0;
		drvdata->buf_wr_resp = 0;

		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		if (drvdata->in_blocks_len > 0 && drvdata->in_blocks_head == drvdata->in_blocks_tail) {
			drvdata->in_block_drop_cnt++;
			return -ENOMEM;
		}

		if (drvdata->buf_len >= ACM_PACKET_SIZE) {
			drvdata->in_block_drop_cnt++;
			return -ENOMEM;
		}

		drvdata->buf[drvdata->buf_len++] = *val;
		//if (drvdata->buf_len <= 7) {
			//first 7 bytes are header
		//}
		break;

	case I2C_SLAVE_READ_PROCESSED:
		drvdata->buf_idx++;
		if (drvdata->out_block_valid && drvdata->buf_idx < drvdata->buf_len)
			*val = drvdata->buf[drvdata->buf_idx];
		else
			*val = 0;

		break;

	case I2C_SLAVE_READ_REQUESTED:
		if (drvdata->out_block_valid && drvdata->buf_idx < drvdata->buf_len)
			*val = drvdata->buf[drvdata->buf_idx];
		else
			*val = 0;

		break;

	case I2C_SLAVE_STOP:
		if (packet->header.src_addr == ACM_ADDR) {
			if (drvdata->in_blocks_len > 0 && drvdata->in_blocks_head == drvdata->in_blocks_tail) {
				drvdata->in_block_drop_cnt++;
			} else {
				ret = gxp_acm_write_packet_done(drvdata);
				drvdata->in_block_recv_cnt++;
			}
		} else if (packet->header.src_addr == GXP_ADDR) {
			ret = gxp_acm_read_packet_done(drvdata);
		}

		break;

	default:
		break;
	}

	return 0;
}

static ssize_t gxp_acm_bin_in_read(struct file *flip, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	// user layer read "bin_in" for received block data from ACM
	struct drvdata_gxp_acm *drvdata = dev_get_drvdata(kobj_to_dev(kobj));
	struct acm_block *in_block = (struct acm_block *)drvdata->in_blocks[drvdata->in_blocks_head];
	u16 in_block_length = be16_to_cpu(in_block->header.length);

	if (drvdata->in_blocks_len <= 0)
		return 0;

	if (off >= (in_block_length + sizeof(struct acm_block_header))) {
		drvdata->in_blocks_len--;
		drvdata->in_blocks_head++;
		if (drvdata->in_blocks_head >= IN_BLOCKS_NUM)
			drvdata->in_blocks_head = 0;

		return 0;
	}

	if ((off + count) >= (in_block_length + sizeof(struct acm_block_header)))
		count = (in_block_length + sizeof(struct acm_block_header) - off);

	memcpy(buf, ((u8 *)in_block) + off, count);
	return count;
}

static ssize_t gxp_acm_bin_in_write(struct file *flip, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	// user layer do dummy write "bin_in" to in_block is ready to receive new block from ACM
	struct drvdata_gxp_acm *drvdata = dev_get_drvdata(kobj_to_dev(kobj));

	if (drvdata->in_blocks_len > 0) {
		drvdata->in_blocks_len--;
		drvdata->in_blocks_head++;
		if (drvdata->in_blocks_head >= IN_BLOCKS_NUM)
			drvdata->in_blocks_head = 0;
	}

	return count;
}

static ssize_t gxp_acm_bin_out_read(struct file *flip, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct drvdata_gxp_acm *drvdata = dev_get_drvdata(kobj_to_dev(kobj));
	struct acm_block *out_block = (struct acm_block *)drvdata->out_block;
	u16 out_block_length = be16_to_cpu(out_block->header.length);

	if (off >= (out_block_length + sizeof(struct acm_block_header)))
		return 0;

	if ((off + count) >= (out_block_length + sizeof(struct acm_block_header)))
		count = (out_block_length + sizeof(struct acm_block_header) - off);

	memcpy(buf, &drvdata->out_block[off], count);
	return count;
}

static ssize_t gxp_acm_bin_out_write(struct file *flip, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct drvdata_gxp_acm *drvdata = dev_get_drvdata(kobj_to_dev(kobj));
	struct acm_block *out_block = (struct acm_block *)drvdata->out_block;
	u16 out_block_length;


	if (drvdata->out_block_valid || drvdata->buf_wr_resp)
		return -EAGAIN;

	memcpy(&drvdata->out_block[off], buf, count);
	out_block_length = be16_to_cpu(out_block->header.length);
	if ((off + count) >= (out_block_length + sizeof(struct acm_block_header)))
		gxp_acm_out_start(drvdata);

	return count;
}

#ifdef DEBUG
static ssize_t gxp_acm_debug_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct drvdata_gxp_acm *drvdata = dev_get_drvdata(dev);
	ssize_t ret;

	ret = sprintf(buf, "last out_block_sent_len=%d\n", drvdata->out_block_sent_len);
	ret += sprintf(buf + ret, "out_block_valid=%d\n", drvdata->out_block_valid);
	ret += sprintf(buf + ret, "last in_block_recv_len=%d\n", drvdata->in_block_recv_len);
	ret += sprintf(buf + ret, "in_block_drop_cnt=%d\n", drvdata->in_block_drop_cnt);
	ret += sprintf(buf + ret, "in_block_recv_cnt=%d\n", drvdata->in_block_recv_cnt);
	ret += sprintf(buf + ret, "in_blocks_len=%d\n", drvdata->in_blocks_len);
	return ret;
}

static ssize_t gxp_acm_debug_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct drvdata_gxp_acm *drvdata = dev_get_drvdata(dev);

	if (!strncmp(buf, "clearIRQ", 8)) {
		gpiod_set_value(drvdata->irq_n, 1);
		drvdata->buf_wr_resp = 0;
		drvdata->out_block_valid = 0;
	} else if (!strncmp(buf, "clearCnt", 8)) {
		drvdata->in_block_drop_cnt = 0;
		drvdata->in_block_recv_cnt = 0;
	} else {
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(gxp_acm_debug);

static struct attribute *gxp_acm_attrs[] = {
	&dev_attr_gxp_acm_debug.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gxp_acm);
#endif

static int gxp_acm_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct drvdata_gxp_acm *drvdata;
	int ret;

	drvdata = devm_kzalloc(&client->dev, sizeof(struct drvdata_gxp_acm), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->irq_n = devm_gpiod_get_optional(dev, "irq_n", GPIOD_OUT_HIGH);
	if (IS_ERR(drvdata->irq_n)) {
		dev_err(dev, "failed to get IRQ_N GPIO\n");
		return PTR_ERR(drvdata->irq_n);
	}

	i2c_set_clientdata(client, drvdata);

	sysfs_bin_attr_init(&drvdata->bin_in);
	drvdata->bin_in.attr.name = "gxp_acm_in";
	drvdata->bin_in.attr.mode = 0600;
	drvdata->bin_in.read = gxp_acm_bin_in_read;
	drvdata->bin_in.write = gxp_acm_bin_in_write;
	drvdata->bin_in.size = ACM_BLOCK_SIZE;

	ret = sysfs_create_bin_file(&dev->kobj, &drvdata->bin_in);
	if (ret)
		goto out;

	drvdata->kn_in = kernfs_find_and_get(dev->kobj.sd, drvdata->bin_in.attr.name);
	if (!drvdata->kn_in)
		goto out_free_bin_in;

	sysfs_bin_attr_init(&drvdata->bin_out);
	drvdata->bin_out.attr.name = "gxp_acm_out";
	drvdata->bin_out.attr.mode = 0600;
	drvdata->bin_out.read = gxp_acm_bin_out_read;
	drvdata->bin_out.write = gxp_acm_bin_out_write;
	drvdata->bin_out.size = ACM_BLOCK_SIZE;

	ret = sysfs_create_bin_file(&dev->kobj, &drvdata->bin_out);
	if (ret)
		goto out_free_kn_in;

	drvdata->kn_out = kernfs_find_and_get(dev->kobj.sd, drvdata->bin_out.attr.name);
	if (!drvdata->kn_out) {
		ret = -EFAULT;
		goto out_free_bin_out;
	}

#ifdef DEBUG
	ret = sysfs_create_group(&dev->kobj, &gxp_acm_group);
	if (ret)
		dev_err(dev, "could not create sysfs device attrs\n");
#endif

	ret = i2c_slave_register(client, gxp_acm_cb);
	if (!ret)
		goto out;

	kernfs_put(drvdata->kn_out);
out_free_bin_out:
	sysfs_remove_bin_file(&dev->kobj, &drvdata->bin_out);
out_free_kn_in:
	kernfs_put(drvdata->kn_in);
out_free_bin_in:
	sysfs_remove_bin_file(&dev->kobj, &drvdata->bin_in);
out:
	return ret;
};

static int gxp_acm_remove(struct i2c_client *client)
{
	struct drvdata_gxp_acm *drvdata = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	i2c_slave_unregister(client);
	kernfs_put(drvdata->kn_in);
	sysfs_remove_bin_file(&dev->kobj, &drvdata->bin_in);
	kernfs_put(drvdata->kn_out);
	sysfs_remove_bin_file(&dev->kobj, &drvdata->bin_out);

#ifdef DEBUG
	sysfs_remove_group(&dev->kobj, &gxp_acm_group);
#endif

	return 0;
}

static const struct of_device_id gxp_acm_of_match[] = {
	{
		.compatible = "hpe,gxp-acm",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gxp_acm_of_match);

static struct i2c_driver gxp_acm_driver = {
	.driver = {
		.name = "gxp-acm",
		.of_match_table = of_match_ptr(gxp_acm_of_match),
	},
	.probe = gxp_acm_probe,
	.remove = gxp_acm_remove,
};
module_i2c_driver(gxp_acm_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP Apollo Chassis Manager driver");
MODULE_LICENSE("GPL v2");
