// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2021 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/mutex.h>

#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/poll.h>

#include "gxp-soclib.h"
#include "gxp-dbg.h"

#define DBG_POST_PORTDATA		0x4
#define DBG_POST_CSR			0x1E

struct gxp_dbg_drvdata {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *base;
	struct mutex mutex;
	int irq;
	unsigned int state;
	unsigned short int postcode;
	unsigned short int previouspostcode;
	unsigned short int initialvalue;
	unsigned short int count;
	dev_t postcodedev;
	struct cdev postcode_c_dev;
	struct class *postcode_cl;
	struct task_struct *powergood_thread;
};

struct gxp_dbg_drvdata *drvdata;

static DECLARE_WAIT_QUEUE_HEAD(wq);

static int post_open(struct inode *inode, struct file *file)
{
	unsigned short int value;

	value = readw(drvdata->base + DBG_POST_CSR);
	// We need to wait for the interrupt to be launched if state
	// is null. or let it go if postcode value is not null after reading it
	mutex_lock(&drvdata->mutex);
	if (drvdata->count > 0) {
		mutex_unlock(&drvdata->mutex);
				return -EBUSY;
	}
	drvdata->postcode = readl(drvdata->base + DBG_POST_PORTDATA);
	drvdata->previouspostcode = 0xFF;
	drvdata->initialvalue = 0x00;
	drvdata->count++;
	mutex_unlock(&drvdata->mutex);
	return 0;
}

static int post_release(struct inode *inode, struct file *flip)
{
	mutex_lock(&drvdata->mutex);
	drvdata->count--;
	mutex_unlock(&drvdata->mutex);
	return 0;
}

static int post_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	// whatever happens we need to return an initial value even if postcode == 0 and previouspostcode == 0
	// this case is happening when the system is offline
	wait_event_interruptible(wq, drvdata->postcode != drvdata->previouspostcode);
	drvdata->previouspostcode = drvdata->postcode;
	if (copy_to_user(buf, &drvdata->postcode, 2))
		return -EFAULT;
	return 1;
}

static __poll_t post_poll(struct file *file,
					struct poll_table_struct *pt)
{
	__poll_t mask = 0;
	// very basic poll event management
	wait_event_interruptible(wq, drvdata->postcode != drvdata->previouspostcode);
	mask |= EPOLLIN;
	return mask;
}

static const struct file_operations post_fops = {
	.owner		= THIS_MODULE,
	.open		= post_open,
	.read		= post_read,
	.poll		= post_poll,
	.release	= post_release,
};


static int wait_power_transition(void *pv)
{
	unsigned short int value;

	while (1) {
		wait_event_interruptible(gxp_fn2, gxp_pgood_trigger != 0);
		msleep(200);
		{
			if ((drvdata->state == 0)  && (gxp_pgood_trigger == 1)) {
				value = readw(drvdata->base + DBG_POST_CSR);
				value = value | 0xf;
				writew(value, drvdata->base + DBG_POST_CSR);

				drvdata->state = 1;
				gxp_pgood_trigger = 0;
			}
			if (gxp_pgood_trigger == 2) {
				// we reset the power state, as a transition to off happened
				drvdata->state = 0;
				gxp_pgood_trigger = 0;
			}
		}
	}

	return 0;
}




static ssize_t postcode_enable_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	ret = sprintf(buf, "%d", drvdata->state);

	return ret;
}

static ssize_t postcode_enable_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int input;
	unsigned short int value;
	int rc;

	rc = kstrtouint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;
	if (input != 0 && input != 1)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	drvdata->state = input;
	if (drvdata->state == 1) {
		value = readw(drvdata->base + DBG_POST_CSR);
		value = value | 0xf;
		writew(value, drvdata->base + DBG_POST_CSR);
	}

	mutex_unlock(&drvdata->mutex);
	return count;
}

static DEVICE_ATTR_RW(postcode_enable);

static struct attribute *dbg_attrs[] = {
	&dev_attr_postcode_enable.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dbg);

static int sysfs_register(struct device *parent)
{
	struct device *dev;

	dev = device_create_with_groups(soc_class, parent, 0,
					drvdata, dbg_groups, "dbg");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	drvdata->dev = dev;
	return 0;
}

static irqreturn_t gxp_dbg_post_irq(int irq, void *_drvdata)
{
	unsigned short int value;

	value = readl(drvdata->base + DBG_POST_PORTDATA);
	if (drvdata->postcode != value) {
		mutex_lock(&drvdata->mutex);
		drvdata->previouspostcode = drvdata->postcode;
		drvdata->postcode = value;
		mutex_unlock(&drvdata->mutex);
	}
	// update CSR
		value = readw(drvdata->base + DBG_POST_CSR);
	writew(value | 0xc, drvdata->base + DBG_POST_CSR);
	wake_up_interruptible(&wq);
	return IRQ_HANDLED;
}

static int gxp_dbg_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	struct device *dev_ret;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_dbg_drvdata),
				GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	drvdata->state = 0;
	drvdata->postcode =  0x00;
	drvdata->previouspostcode = 0xFF;
	drvdata->initialvalue = 0x00;
	drvdata->count = 0x00;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);
	// let's retrieve the driver irq
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to obtain IRQ number\n");
		return ret;
			}
	drvdata->irq = ret;
	// Let's attach to the irq
	ret = devm_request_irq(&pdev->dev,
				drvdata->irq,
				gxp_dbg_post_irq,
				IRQF_SHARED,
				"gxp-dbg-post",
				drvdata);

	mutex_init(&drvdata->mutex);


	// Let's create the character device for the output
	ret = alloc_chrdev_region(&drvdata->postcodedev, 0, 1, "gxp-dbg-post");
	if (ret < 0)
		return ret;

	drvdata->postcode_cl = class_create(THIS_MODULE, "chardrv");
	if (IS_ERR(drvdata->postcode_cl)) {
		unregister_chrdev_region(drvdata->postcodedev, 1);
		return PTR_ERR(drvdata->postcode_cl);
	}
	dev_ret = device_create(drvdata->postcode_cl, NULL, drvdata->postcodedev, NULL, "postcode");
	if (IS_ERR(dev_ret)) {
		class_destroy(drvdata->postcode_cl);
		unregister_chrdev_region(drvdata->postcodedev, 1);
		return PTR_ERR(dev_ret);
	}

	cdev_init(&drvdata->postcode_c_dev, &post_fops);
	ret = cdev_add(&drvdata->postcode_c_dev, drvdata->postcodedev, 1);
	if (ret < 0) {
		device_destroy(drvdata->postcode_cl, drvdata->postcodedev);
				class_destroy(drvdata->postcode_cl);
		unregister_chrdev_region(drvdata->postcodedev, 1);
		return ret;
	}

	// let's start the power transition thread
	drvdata->powergood_thread = kthread_run(wait_power_transition, NULL, "Power Management Thread");
	if (drvdata->powergood_thread) {
		pr_info("Power Management Thread Created Successfully...\n");
	} else {
		pr_err("Cannot create Power Management kthread\n");
		return -1;
	}

	// register driver control through sysfs

	return sysfs_register(&pdev->dev);
}

static const struct of_device_id gxp_dbg_of_match[] = {
	{ .compatible = "hpe,gxp-dbg" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_dbg_of_match);

static struct platform_driver gxp_dbg_driver = {
	.probe = gxp_dbg_probe,
	.driver = {
		.name = "gxp-dbg",
		.of_match_table = of_match_ptr(gxp_dbg_of_match),
	},
};
module_platform_driver(gxp_dbg_driver);

MODULE_AUTHOR("Jean-Marie Verdun <jean-marie.verdun@hpe.com>");
MODULE_DESCRIPTION("HPE GXP DBG Driver");
