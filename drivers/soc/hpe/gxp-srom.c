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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "gxp-soclib.h"

#define SROM_LOC		0xf4

struct gxp_srom_drvdata {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *base;
	struct mutex mutex;
};

static ssize_t vromoff_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_srom_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	ssize_t ret;

	mutex_lock(&drvdata->mutex);

	value = readl(drvdata->base + SROM_LOC);
	ret = sprintf(buf, "0x%08x", value);

	mutex_unlock(&drvdata->mutex);
	return ret;
}

static ssize_t vromoff_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_srom_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned int value;
	int rc;

	rc = kstrtouint(buf, 0, &value);
	if (rc < 0)
		return -EINVAL;

	mutex_lock(&drvdata->mutex);

	writel(value, drvdata->base + SROM_LOC);

	mutex_unlock(&drvdata->mutex);
	return count;
}
static DEVICE_ATTR_RW(vromoff);

static struct attribute *srom_attrs[] = {
	&dev_attr_vromoff.attr,
	NULL,
};
ATTRIBUTE_GROUPS(srom);

static int sysfs_register(struct device *parent,
			struct gxp_srom_drvdata *drvdata)
{
	struct device *dev;

	dev = device_create_with_groups(soc_class, parent, 0,
					drvdata, srom_groups, "srom");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	drvdata->dev = dev;
	return 0;
}

static int gxp_srom_probe(struct platform_device *pdev)
{
	struct gxp_srom_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_srom_drvdata),
				GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	mutex_init(&drvdata->mutex);

	return sysfs_register(&pdev->dev, drvdata);
}

static const struct of_device_id gxp_srom_of_match[] = {
	{ .compatible = "hpe,gxp-srom" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_srom_of_match);

static struct platform_driver gxp_srom_driver = {
	.probe = gxp_srom_probe,
	.driver = {
		.name = "gxp-srom",
		.of_match_table = of_match_ptr(gxp_srom_of_match),
	},
};
module_platform_driver(gxp_srom_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP SROM Driver");
