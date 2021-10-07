// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
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
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "gxp-soclib.h"

#define GXP_CSM_CELLS	1
#define ASRSTAT		0x5c
#define ASRESTAT	0x5d
#define AFUNEN2		0xde
#define AFUNEN2_EHCIEN	0x40
#define SHUTDOWN	0xe7

struct gxp_csm_drvdata {
	void __iomem *base;
	struct mutex mutex;
	struct mfd_cell cells[GXP_CSM_CELLS];
};

static ssize_t vehci_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_csm_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned char value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readb(drvdata->base + AFUNEN2);
	ret = sprintf(buf, "%d", (value&AFUNEN2_EHCIEN)?1:0);

	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t vehci_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gxp_csm_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int input;
	unsigned char value;
	int rc;

	rc = kstrtouint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	value = readb(drvdata->base + AFUNEN2);
	if (input)
		value |= AFUNEN2_EHCIEN;
	else
		value &= ~AFUNEN2_EHCIEN;
	writeb(value, drvdata->base + AFUNEN2);

	mutex_unlock(&drvdata->mutex);
	return count;
}
static DEVICE_ATTR_RW(vehci_enable);

static ssize_t sw_reset_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_csm_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readb(drvdata->base + ASRESTAT);
	ret = sprintf(buf, "%d", (value&0x80)?1:0);

	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t sw_reset_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_csm_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	int rc;

	rc = kstrtouint(buf, 0, &value);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	if (value) {
		//generate a sw reset
		writeb(0x00, drvdata->base + ASRSTAT);
		writeb(0x80, drvdata->base + ASRESTAT);
	} else {
		//clear sw_reset event
		writeb(0x00, drvdata->base + ASRESTAT);
	}
	mutex_unlock(&drvdata->mutex);
	return count;
}
static DEVICE_ATTR_RW(sw_reset);

static ssize_t sw_shutdown_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct gxp_csm_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	int rc;

	rc = kstrtouint(buf, 0, &value);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	if (value)
		writeb(0xb2, drvdata->base + SHUTDOWN);
	mutex_unlock(&drvdata->mutex);
	return count;
}
static DEVICE_ATTR_WO(sw_shutdown);


static struct attribute *csm_attrs[] = {
	&dev_attr_sw_reset.attr,
	&dev_attr_sw_shutdown.attr,
	&dev_attr_vehci_enable.attr,
	NULL,
};
ATTRIBUTE_GROUPS(csm);

static int sysfs_register(struct device *parent,
			struct gxp_csm_drvdata *drvdata)
{
	struct device *dev;

	dev = device_create_with_groups(soc_class, parent, 0,
					drvdata, csm_groups, "csm");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	return 0;
}

static int gxp_csm_probe(struct platform_device *pdev)
{
	struct gxp_csm_drvdata *drvdata;
	struct resource *res;
	unsigned int i;

	drvdata = devm_kzalloc(&pdev->dev,
				sizeof(struct gxp_csm_drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	platform_set_drvdata(pdev, drvdata);
	for (i = 0 ; i < pdev->num_resources ; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (i == 0) {
			drvdata->base = devm_ioremap_resource(&pdev->dev, res);
			if (IS_ERR(drvdata->base))
				return PTR_ERR(drvdata->base);
		} else {
			void __iomem *tmp;

			tmp = devm_ioremap_resource(&pdev->dev, res);
			if (IS_ERR(tmp))
				return PTR_ERR(tmp);
		}
	}
	mutex_init(&drvdata->mutex);
	sysfs_register(&pdev->dev, drvdata);

	return 0;
}

static const struct of_device_id gxp_csm_of_match[] = {
	{ .compatible = "hpe,gxp-csm" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_csm_of_match);

static struct platform_driver gxp_csm_driver = {
	.probe = gxp_csm_probe,
	.driver = {
		.name = "gxp-csm",
		.of_match_table = of_match_ptr(gxp_csm_of_match),
	},
};
module_platform_driver(gxp_csm_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP CSM Driver");
