// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2020 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/sched/signal.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>

#include "gxp-soclib.h"

#define CHIF_DEBUG 0

#define MAX_CHANNELS 32
#define CHANNEL_SHAREMEM 24

#define DEVICE_NAME "chif"
#define DEVICE_COUNT MAX_CHANNELS

#define SHARED_MEM_CHIF_DESCRIPTOR_NUM 2
#define SHARED_MEM_CHIF_DESCRIPTOR_SIZE 4096
#define SHARED_MEM_CHIF_QUEUE_SIZE 16

#define SHARED_MEM_SIZE 0x100000 //1MB

#define REG_SMEMCFG 0x50
#define REG_INDOOR 0xd0
#define REG_OUTDOOR 0xd4
#define REG_FN2ISTAT 0xb0
#define FN2IMASK_DBELL 0x01

#define CCB_CTRL_GO_MASK 0x0000000080000000LL
#define CCB_CTRL_ACTIVE_MASK 0x0000000040000000LL

struct chif_pkthdr {
	uint16_t pkt_size;
	uint16_t sequence;
	uint16_t command;
	uint8_t service_id;
	uint8_t version;
} __packed;

struct CCB {
	uint64_t sqFifo;
	uint64_t sqDesc;
	uint64_t sqCtrl;

	uint64_t rqFifo;
	uint64_t rqDesc;
	uint64_t rqCtrl;

	uint64_t doorbellBase;
	uint64_t channelNumber;

	uint8_t context[128 - (sizeof(uint64_t) * 8)];
} __packed;

struct chif_queue {
	int hw_index;
	int head;
	int tail;
	int size;
	int max_size;
	uint8_t pkt[SHARED_MEM_CHIF_QUEUE_SIZE][SHARED_MEM_CHIF_DESCRIPTOR_SIZE];
	uint32_t pkt_len[SHARED_MEM_CHIF_QUEUE_SIZE];
	uint32_t max_pkt_len;
};

struct ShmemChifHeader {
	uint32_t Signature;
	uint32_t MajorVersion;
	uint32_t MinorVersion;
	uint8_t Reserved[48];
	uint32_t Checksum;
} __packed;

struct chif_ccb_ctrl {
	uint64_t L2sz : 4;
	uint64_t FifoIndexMask : 14;
	uint64_t DescLimit : 12;
	uint64_t Active : 1;
	uint64_t Go : 1;
	uint64_t Doorbell : 1;
	uint64_t Reserved : 31;
} __packed;

struct chif_fifo_entry {
	uint32_t QWords : 10; // Bit[9:0]   Qwords
	uint32_t DescriptorNum : 12; // Bit[21:10] DescriptorNum
	uint32_t Consumed : 1; // Bit[22]    Data Consumed
	uint32_t Occupied : 1; // Bit[23]    Occupied (with data)
	uint32_t SequenceNum : 8; // Bit[31:24] SequenceNum
	uint32_t SequenceNum2; // Bit[63:32] SequenceNum
} __packed;

struct chif_desc_entry {
	uint8_t data[SHARED_MEM_CHIF_DESCRIPTOR_SIZE];
} __packed;

struct chif_drvdata {
	struct device *dev;
	struct cdev chif_cdev;
	struct regmap *fn2_map;
	dev_t devid;
	int irq;
	dma_addr_t alloc_buf_dma_addr;
	void *alloc_buf;
	void *shmem;
	dma_addr_t shmem_phys;
	void *nvram;

	struct CCB *ccbTable; //point to ccb table[]
	struct ShmemChifHeader *
		shmemChifHeader; //point to shmem + 0x4000 where the shared memory chif channel header.
	wait_queue_head_t wait[MAX_CHANNELS];
	bool waiting[MAX_CHANNELS];
};

struct chif_queue chif24_queue;
#define QUEUE_RC_FULL 1
#define QUEUE_RC_PKT_OVER_SIZE 2
#define QUEUE_RC_EMPTY 3
#define QUEUE_RC_BUF_TOO_SMALL 4

static int chif_queue_push(struct chif_queue *q, uint8_t *pkt, uint32_t len)
{
	if (q->size == q->max_size)
		return -QUEUE_RC_FULL;

	if (len > q->max_pkt_len)
		return -QUEUE_RC_PKT_OVER_SIZE;


	//dev_info(drvdata->dev, "push: at %d! len = 0x%08x\n", q->tail, len);
	memcpy(q->pkt[q->tail], pkt, len);
	q->pkt_len[q->tail] = len;
	q->size++;

	//move tail to next slot
	q->tail++;
	if (q->tail >= q->max_size) {
		//wrap the point
		q->tail = 0;
	}

	return 0;
}

static int chif_queue_pop(struct chif_queue *q, uint8_t **p_pkt_buf,
			  uint32_t len)
{
	int pkt_len = 0;

	if (q->size == 0)
		return -QUEUE_RC_EMPTY;

	if (len < q->pkt_len[q->head])
		return -QUEUE_RC_BUF_TOO_SMALL;

	//dev_info(drvdata->dev, "pop: at %d!\n", q->head);
	*p_pkt_buf = q->pkt[q->head];
	pkt_len = q->pkt_len[q->head];
	q->size--;

	//move head to next slot
	q->head++;
	if (q->head >= q->max_size) {
		//wrap the point
		q->head = 0;
	}

	return pkt_len;
}

static int ccb_table_init(struct CCB *ccbTable)
{
	//ccb's fifobar, descbar is initialized by host driver

	//todo: what else need to init?

	return 0;
}

static int chif_doorbell26(struct chif_drvdata *drvdata)
{
	int reg;

	regmap_write(drvdata->fn2_map, REG_OUTDOOR, 0x04000000);

	regmap_read(drvdata->fn2_map, REG_FN2ISTAT, &reg);
	reg = (reg >> 16) & 0xffff;
	if (reg & FN2IMASK_DBELL)
		regmap_write(drvdata->fn2_map, REG_OUTDOOR, 0x01);

	return 0;
}

static struct chif_fifo_entry *GetSqFifo(struct chif_drvdata *drvdata,
					 int channel)
{
	struct chif_fifo_entry *send_fifo =
		drvdata->shmem + drvdata->ccbTable[channel].sqFifo;

	return send_fifo;
}

static struct chif_fifo_entry *GetRqFifo(struct chif_drvdata *drvdata,
					 int channel)
{
	struct chif_fifo_entry *recv_fifo =
		drvdata->shmem + drvdata->ccbTable[channel].rqFifo;

	return recv_fifo;
}

static struct chif_desc_entry *GetSqDesc(struct chif_drvdata *drvdata,
					 int channel)
{
	struct chif_desc_entry *send_desc =
		drvdata->shmem + drvdata->ccbTable[channel].sqDesc;

	return send_desc;
}

static struct chif_desc_entry *GetRqDesc(struct chif_drvdata *drvdata,
					 int channel)
{
	struct chif_desc_entry *recv_desc =
		drvdata->shmem + drvdata->ccbTable[channel].rqDesc;

	return recv_desc;
}

static int chif_open(struct inode *inode, struct file *file)
{
	int channel = iminor(inode);
	struct chif_drvdata *drvdata;
	struct CCB *p_ccb;

	drvdata = container_of(inode->i_cdev, struct chif_drvdata, chif_cdev);

	if (channel >= DEVICE_COUNT)
		return -EINVAL;

	file->private_data = drvdata;
	p_ccb = &drvdata->ccbTable[channel];

	//dev_info(drvdata->dev,  "chif open channel:%d\n", channel);

	if (!(p_ccb->sqCtrl & CCB_CTRL_GO_MASK)) {
		if (channel == 24) {
			memset(&chif24_queue, 0, sizeof(struct chif_queue));
			chif24_queue.max_size = SHARED_MEM_CHIF_QUEUE_SIZE;
			chif24_queue.max_pkt_len =
				SHARED_MEM_CHIF_DESCRIPTOR_SIZE;
		}
	}

	return 0;
}

static ssize_t chif_read(struct file *file, char __user *buf, size_t count,
			 loff_t *f_pos)
{
	struct chif_drvdata *drvdata =
		(struct chif_drvdata *)file->private_data;
	unsigned int channel = iminor(file_inode(file));

	uint8_t *pkt_buf;
	int pkt_len;
	DECLARE_WAITQUEUE(wait, current);

	if (channel != CHANNEL_SHAREMEM) {
		dev_err(drvdata->dev, "channel %d does not support now\n", channel);
		return -EIO;
	}

	if (count < SHARED_MEM_CHIF_DESCRIPTOR_SIZE) {
		dev_err(drvdata->dev, "read length is too small(%d < %d)\n", count,
		       SHARED_MEM_CHIF_DESCRIPTOR_SIZE);
		return -EINVAL;
	}

	//steps to read chif packet from fifo
	//1.check queue.
	//	if queue is non empty
	//		goto step2
	//	if queue is empty, set user space process to sleep state
	//		chif irq handler would copy entry to queue and waked up the user space process
	//		user space process is waked up by IRQ handler, goto step 1.
	//2.copy the entry to user space

	//IRQ handler
	//	get interrupt due to host's door bell
	//		test each entry in the fifo
	//	if o bit is asserted then process the fifo with smaller seq# first
	//		get packet id and len
	//		get desc for packet addr
	//		copy to user space, set c bit
	//		wake up user space process

	//find the entry in fifo to process

	pkt_len = chif_queue_pop(&chif24_queue, &pkt_buf, count);
	if (pkt_len < 0) {
		//fifo is empty
		if (file->f_flags & O_NONBLOCK) {
			//return if it is nonblock read
			dev_err(drvdata->dev, "%s nonlock\n", __func__);
			return -EAGAIN;
		}

		// block process to wait event from irq_handle
		add_wait_queue(&drvdata->wait[channel], &wait);
		drvdata->waiting[channel] = true;
		while (pkt_len < 0) {
			//dev_info(drvdata->dev, "read: sleep\n");
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();

			//dev_info(drvdata->dev, "read: waked up!\n");
			if (signal_pending(current)) {
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&drvdata->wait[channel],
							&wait);
				drvdata->waiting[channel] = false;
				return -ERESTARTSYS;
			}
			pkt_len = chif_queue_pop(&chif24_queue, &pkt_buf, count);
		}
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&drvdata->wait[channel], &wait);
		drvdata->waiting[channel] = false;
	}

	if (pkt_len > 0) {
		if (copy_to_user(buf, pkt_buf, pkt_len)) {
			dev_err(drvdata->dev, "error when copy_to_user( length=%d)\n", pkt_len);
			return -EFAULT;
		}
		//dev_info(drvdata->dev, "copy_to_user( length=%d  f_pos=%d)\n", pkt_len, *f_pos);
	}

	return pkt_len;
}

static ssize_t chif_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *f_pos)
{
	struct chif_drvdata *drvdata =
		(struct chif_drvdata *)file->private_data;
	struct chif_fifo_entry *recv_fifo;
	struct chif_desc_entry *recv_desc;
	struct CCB *p_ccb;
	unsigned int channel = iminor(file_inode(file));

	int round_len;
	int fifo_idx;
	uint32_t desc_idx;
	uint32_t desc_len;
	int timeout;

	uint8_t dummy[4096];

	if (channel != CHANNEL_SHAREMEM) {
		dev_err(drvdata->dev, "channel %d is not supported\n", channel);
		return -EIO;
	}

	p_ccb = &drvdata->ccbTable[channel];
	recv_fifo = GetRqFifo(drvdata, channel);
	recv_desc = GetRqDesc(drvdata, channel);

	if (!(p_ccb->rqCtrl & CCB_CTRL_GO_MASK)) {
		dev_err(drvdata->dev, "go bit of rqCtrl is not set yet or cleared by host. drop response write\n");
		p_ccb->rqCtrl &= ~CCB_CTRL_ACTIVE_MASK;
		return -EIO;
	}

	if (!(p_ccb->rqCtrl & CCB_CTRL_ACTIVE_MASK)) {
		//dev_info(drvdata->dev, "set active bit of rqCtrl\n");
		p_ccb->rqCtrl |= CCB_CTRL_ACTIVE_MASK;
	}

	fifo_idx = 0; //always use fifo0 to response
	desc_idx = recv_fifo[fifo_idx].DescriptorNum;

	if (count % 8)
		round_len = count + 8 - (count % 8);
	else
		round_len = count;

	recv_fifo[fifo_idx].QWords = round_len >> 3;

	if (!recv_fifo[fifo_idx].Occupied) {
		//recv fifo is empty
		dev_err(drvdata->dev, "error: recv fifo is empty\n");

		//wait host
		timeout = 100;
		while (timeout) {
			timeout--;
			msleep(100);
			if (recv_fifo[fifo_idx].Occupied)
				break;
		}

		if (timeout == 0) {
			dev_err(drvdata->dev, "error: wait ecv fifo for empty but timeout\n");
			return -EIO;
		}
		//dev_info(drvdata->dev, "info: wait ecv fifo empty for %d00ms\n", 100 - timeout);
	}

	if (count > SHARED_MEM_CHIF_DESCRIPTOR_SIZE) {
		//write data is too big
		dev_err(drvdata->dev, "error: write len (%d) > recv packet len (%d)\n", count,
		       desc_len);
		return -EIO;
	}

	if (recv_fifo[fifo_idx].Consumed) {
		dev_err(drvdata->dev, "error: not free fifo, host not dequeue last msg from fifo\n");
		return -EIO;
	}

	memset(&recv_desc[desc_idx], 0, round_len);
	//dev_info(drvdata->dev, "write: count=%d round=%d\n", count, round_len);

	if (copy_from_user(&recv_desc[desc_idx], buf, count)) {
		dev_err(drvdata->dev, "write: copy_from_user() failed\n");
		return -EIO;
	}
	recv_fifo[fifo_idx].Consumed = 1;

	//dummy read to ensure value to write to shared memory.
	memcpy(dummy, &recv_fifo[fifo_idx], sizeof(struct chif_fifo_entry));
	memcpy(dummy, &recv_desc[desc_idx], round_len);

	if (p_ccb->doorbellBase) {
		//set doorbell
		regmap_write(drvdata->fn2_map, REG_OUTDOOR, 1 << channel);
	}

	return count;
}

static int chif_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations chif_fops = {
	.open = chif_open,
	.read = chif_read,
	.write = chif_write,
	.release = chif_release,
};

static int CompareSeq(uint32_t a, uint32_t b)
{
	//todo: handle the case when seq_num got wrapped.
	if (a > b)
		return 1;
	else if (a < b)
		return -1;
	else
		return 0;
}

static int ActiveCCB(struct chif_drvdata *drvdata, int channel)
{
	struct CCB *p_ccb = &drvdata->ccbTable[channel];

	if (!(p_ccb->sqCtrl & CCB_CTRL_GO_MASK)) {
		dev_err(drvdata->dev, "go bit of sqCtrl is not set yet or cleared by host\n");
		p_ccb->sqCtrl &= ~CCB_CTRL_ACTIVE_MASK;
		return -1;
	}

	if (!(p_ccb->sqCtrl & CCB_CTRL_ACTIVE_MASK)) {
		//dev_info(drvdata->dev, "set active bit of sqCtrl\n");
		p_ccb->sqCtrl |= CCB_CTRL_ACTIVE_MASK;
	}

	return 1;
}

//the function return the entry index with smallest seq_num in fifo
static int GetOccupiedEntry(struct chif_fifo_entry *fifo)
{
	int i;
	int fifo_idx = -1;
	uint32_t fifo_seq = 0;

	for (i = 0; i < SHARED_MEM_CHIF_DESCRIPTOR_NUM; i++) {
		if ((fifo[i].Occupied == 1) && (fifo[i].Consumed == 0)) {
			//todo: handle the case when seq_num got wrapped.
			if (CompareSeq(fifo[i].SequenceNum, fifo_seq) <= 0) {
				fifo_seq = fifo[i].SequenceNum;
				fifo_idx = i;
			}
		}
	}
	//dev_info(drvdata->dev, "Get:fifo_idx=%d\n", fifo_idx);
	return fifo_idx;
}

static int ConsumePacket(struct chif_drvdata *drvdata, int channel)
{
	//mark consumed flag packet with no data in fifo
	//return number of packet with data.
	int i;
	int ret;
	int nonEmptyPackets = 0;
	int desc_idx;
	int desc_len;
	struct chif_fifo_entry *send_fifo;
	struct chif_desc_entry *send_desc;

	send_fifo = GetSqFifo(drvdata, channel);
	send_desc = GetSqDesc(drvdata, channel);

	//Consume empty pkt first
	for (i = 0; i < SHARED_MEM_CHIF_DESCRIPTOR_NUM; i++) {
		if ((send_fifo[i].Occupied == 1) &&
		    (send_fifo[i].Consumed == 0)) {
			if (send_fifo[i].QWords == 0)
				send_fifo[i].Consumed = 1;
			else
				nonEmptyPackets++;
		}
	}

	if (nonEmptyPackets > 0) {
		//push pkt to chif24_queue
		i = GetOccupiedEntry(send_fifo);
		while (i >= 0) {
			desc_idx = send_fifo[i].DescriptorNum;
			desc_len = send_fifo[i].QWords << 3;
			//dev_info(drvdata->dev, "Consume fifo idx:%d,Desc:%d,Len:0x%08x,QWords:0x%08x\n", i, desc_idx, desc_len, send_fifo[i].QWords);
			ret = chif_queue_push(&chif24_queue, send_desc[desc_idx].data,
					desc_len);
			switch (ret) {
			case -QUEUE_RC_FULL:
				dev_err(drvdata->dev, "push:queue is full\n");
				break;
			case -QUEUE_RC_PKT_OVER_SIZE:
				dev_err(drvdata->dev, "push:len %d > max_pkt_len %d\n", desc_len, chif24_queue.max_pkt_len);
				break;
			default:
				//dev_err(drvdata->dev, "chif_queue_push:ret=%d\n", ret);
				break;
			}

			send_fifo[i].Consumed = 1;
			i = GetOccupiedEntry(send_fifo);
		}
	}

	return nonEmptyPackets;
}

static irqreturn_t chif_irq_handle(int irq, void *_drvdata)
{
	struct chif_drvdata *drvdata = (struct chif_drvdata *)_drvdata;

	unsigned int doorbell;
	int i;

	regmap_read(drvdata->fn2_map, REG_INDOOR, &doorbell);

	//clear the inbound doorbell bits
	regmap_write(drvdata->fn2_map, REG_INDOOR, doorbell);

	for (i = (MAX_CHANNELS - 1); i >= 0; i--) {
		if ((1 << i) & doorbell) {
			if (ActiveCCB(drvdata, i) < 0) {
				//skip CCB without Go bit asserted
				continue;
			}

			//dev_info(drvdata->dev, "chif_irq_handle: channel%d doorbell=0x%08x\n", i, doorbell);
			if (ConsumePacket(drvdata, i) > 0) {
				//there is packet with data need to process, wait user space daemon
				if (drvdata->waiting[i]) {
					//dev_info(drvdata->dev, "channel%d wake up!\n", i);
					wake_up_interruptible(
						&drvdata->wait[i]);
				}
			}
		}
	}

	return IRQ_HANDLED;
}

static ssize_t header_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct chif_drvdata *drvdata = dev_get_drvdata(dev);
	struct CCB *p_ccb;
	ssize_t ret;

	ret = sprintf(buf, "Signature:0x%08x\n",
		      drvdata->shmemChifHeader->Signature);
	ret += sprintf(buf + ret, "MajorVersion:0x%08x\n",
		       drvdata->shmemChifHeader->MajorVersion);
	ret += sprintf(buf + ret, "MinorVersion:0x%08x\n",
		       drvdata->shmemChifHeader->MinorVersion);
	ret += sprintf(buf + ret, "Checksum:0x%08x\n",
		       drvdata->shmemChifHeader->Checksum);

	//dump ccb of chif24
	p_ccb = &drvdata->ccbTable[24];
	ret += sprintf(buf + ret, "ccb[24] at %p sizeof(ccb)=%d\n", p_ccb,
		       sizeof(struct CCB));
	ret += sprintf(buf + ret, "sqfifo:0x%016llx\n", p_ccb->sqFifo);
	ret += sprintf(buf + ret, "sqdesc:0x%016llx\n", p_ccb->sqDesc);
	ret += sprintf(buf + ret, "sqCtrl:0x%016llx\n", p_ccb->sqCtrl);
	ret += sprintf(buf + ret, "rqfifo:0x%016llx\n", p_ccb->rqFifo);
	ret += sprintf(buf + ret, "rqdesc:0x%016llx\n", p_ccb->rqDesc);
	ret += sprintf(buf + ret, "rqCtrl:0x%016llx\n", p_ccb->rqCtrl);
	ret += sprintf(buf + ret, "doorbellbase:0x%16llx\n", p_ccb->doorbellBase);
	ret += sprintf(buf + ret, "channelNumber:%lld\n", p_ccb->channelNumber);

	return ret;
}
static DEVICE_ATTR_RO(header);

static struct attribute *chif_attrs[] = {
	&dev_attr_header.attr,
	NULL,
};
ATTRIBUTE_GROUPS(chif);

static int chif_probe(struct platform_device *pdev)
{
	int rc;
	int major, minor;
	int i;
	struct chif_drvdata *drvdata;
	struct device *dev;
	uint32_t reg;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct chif_drvdata),
			       GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);
	drvdata->dev = &pdev->dev;

	drvdata->fn2_map = syscon_node_to_regmap(drvdata->dev->parent->of_node);
	if (IS_ERR(drvdata->fn2_map)) {
		dev_err(&pdev->dev, "Unable to find fn2 regmap\n");
		return PTR_ERR(drvdata->fn2_map);
	}

	if (dma_set_mask_and_coherent(drvdata->dev, DMA_BIT_MASK(32))) {
		dev_warn(drvdata->dev, "No suitable DMA available\n");
		return -ENOMEM;
	}

	drvdata->alloc_buf = dma_alloc_coherent(drvdata->dev,
						SHARED_MEM_SIZE * 2, &drvdata->alloc_buf_dma_addr,
						GFP_KERNEL | GFP_DMA);
	if (!drvdata->alloc_buf) {
		dev_err(drvdata->dev,
			"fail to alloc memory for shared mem area\n");
		return -ENOMEM;
	}

	drvdata->shmem_phys = drvdata->alloc_buf_dma_addr;
	drvdata->shmem = drvdata->alloc_buf;
	if (drvdata->alloc_buf_dma_addr & 0xFFFFF) {
		drvdata->shmem_phys = drvdata->alloc_buf_dma_addr + 0x100000;
		drvdata->shmem = drvdata->alloc_buf + 0x100000;
		drvdata->shmem_phys = (drvdata->shmem_phys & ~0xFFFFF);
		drvdata->shmem =
			(void *)((unsigned int)drvdata->alloc_buf & ~0xFFFFF);
	}

	dev_info(drvdata->dev, "chif: alloc_buf_dma_addr=0x%08x\n",
	       drvdata->alloc_buf_dma_addr);
	dev_info(drvdata->dev, "chif: alloc_buf=%p\n", drvdata->alloc_buf);
	dev_info(drvdata->dev, "chif: shmem virt=%p phys=0x%08x\n", drvdata->shmem,
	       drvdata->shmem_phys);

	drvdata->ccbTable = (struct CCB *)drvdata->shmem;
	drvdata->shmemChifHeader = drvdata->shmem + 0x4000;

	ccb_table_init(drvdata->ccbTable);

	//init shared memory setting
	reg = drvdata->shmem_phys;
	reg |= 0x09; //size code for 1MB
	reg &= 0x3fffffff; //bit31:30 are reserved
	regmap_write(drvdata->fn2_map, REG_SMEMCFG, reg);

	rc = platform_get_irq(pdev, 0);
	if (rc < 0) {
		dev_err(drvdata->dev, "unable to obtain IRQ number\n");
		goto dma_free;
	}
	drvdata->irq = rc;

	rc = devm_request_irq(drvdata->dev, drvdata->irq, chif_irq_handle,
			      IRQF_SHARED, DEVICE_NAME, drvdata);
	if (rc < 0) {
		dev_err(drvdata->dev, "irq request failed\n");
		goto dma_free;
	}

	rc = alloc_chrdev_region(&drvdata->devid, 0, DEVICE_COUNT, DEVICE_NAME);
	if (rc < 0) {
		dev_err(drvdata->dev, "unable to alloc char device\n");
		rc = -ENODEV;
		goto dma_free;
	}

	//create device node for chif24
	major = MAJOR(drvdata->devid);
	minor = 24;
	dev_info(drvdata->dev, "chif major=%d minor=%d\n", major, minor);

	cdev_init(&drvdata->chif_cdev, &chif_fops);
	drvdata->chif_cdev.owner = THIS_MODULE;

	rc = cdev_add(&drvdata->chif_cdev, MKDEV(major, minor), 1);
	if (rc < 0) {
		dev_err(drvdata->dev, "unable to add char device\n");
		goto unregister_chrdev;
	}

	dev = device_create_with_groups(soc_class, drvdata->dev, MKDEV(major, minor),
					drvdata, chif_groups, DEVICE_NAME "%d", 24);
	if (IS_ERR(dev)) {
		dev_err(drvdata->dev, "unable to create sysfs\n");
		goto unregister_chrdev;
	}

	for (i = 0; i < MAX_CHANNELS; i++) {
		init_waitqueue_head(&drvdata->wait[i]);
		drvdata->waiting[i] = false;
	}

	//clear packet queue which is for buffering data from host to user space daemon
	memset(&chif24_queue, 0, sizeof(struct chif_queue));
	chif24_queue.max_size = SHARED_MEM_CHIF_QUEUE_SIZE;
	chif24_queue.max_pkt_len = SHARED_MEM_CHIF_DESCRIPTOR_SIZE;

	chif_doorbell26(drvdata);
	return rc;

unregister_chrdev:
	unregister_chrdev_region(drvdata->devid, DEVICE_COUNT);
dma_free:
	dma_free_coherent(drvdata->dev, SHARED_MEM_SIZE * 2, drvdata->alloc_buf, drvdata->alloc_buf_dma_addr);
fail:
	return rc;
}

static int chif_remove(struct platform_device *pdev)
{
	struct chif_drvdata *drvdata;

	drvdata = (struct chif_drvdata *)platform_get_drvdata(pdev);
	cdev_del(&drvdata->chif_cdev);
	unregister_chrdev_region(drvdata->devid, DEVICE_COUNT);
	dma_free_coherent(drvdata->dev, SHARED_MEM_SIZE * 2, drvdata->alloc_buf, drvdata->alloc_buf_dma_addr);
	return 0;
}

static const struct of_device_id chif_of_match[] = {
	{ .compatible = "hpe,gxp-chif" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_xreg_of_match);

static struct platform_driver chif_driver = {
	.probe = chif_probe,
	.remove = chif_remove,
		.driver = {
		.name = "gxp-chif",
		.of_match_table = of_match_ptr(chif_of_match),
	},
};
module_platform_driver(chif_driver);

MODULE_AUTHOR("Gilbert Chen<gilbert.chen@hpe.com");
MODULE_DESCRIPTION("CHIF Driver");
