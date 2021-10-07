// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#define OFFSET_TSENCMD	0x00
#define OFFSET_TSENCFG	0x02
#define OFFSET_TSENDAT	0x04
#define OFFSET_TSENSTAT	0x06

struct gxp_coretemp_drvdata {
	struct device	*dev;
	struct device	*hwmon_dev;
	void __iomem	*base;
	struct mutex update_lock;
};

static ssize_t show_temp_input(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_coretemp_drvdata *drvdata = dev_get_drvdata(dev);
	int val;

	if (readw(drvdata->base + OFFSET_TSENSTAT) & 0x0001) {
		val = readw(drvdata->base + OFFSET_TSENDAT);
		val = ((val * 3874)/1000 - 2821)/10;
	} else {
		val = 0;
	}
	return sprintf(buf, "%d000\n", val);
}

static SENSOR_DEVICE_ATTR(temp1_input, 0444, show_temp_input, NULL, 0);

static struct attribute *gxp_coretemp_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gxp_coretemp);

static int gxp_coretemp_init(struct gxp_coretemp_drvdata *drvdata)
{
	writew(0xc0a0, drvdata->base + OFFSET_TSENCMD);
	writew(0xc080, drvdata->base + OFFSET_TSENCMD);
	writew(0xc081, drvdata->base + OFFSET_TSENCMD);

	udelay(64);

	writew(0xc083, drvdata->base + OFFSET_TSENCMD);

	return 0;
}

static int gxp_coretemp_probe(struct platform_device *pdev)
{
	struct gxp_coretemp_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_coretemp_drvdata),
			GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	mutex_init(&drvdata->update_lock);

	drvdata->hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
			"coretemp", drvdata, gxp_coretemp_groups);
	if (IS_ERR(drvdata->hwmon_dev))
		return PTR_ERR(drvdata->hwmon_dev);

	return gxp_coretemp_init(drvdata);
}

static const struct of_device_id gxp_coretemp_of_match[] = {
	{ .compatible = "hpe,gxp-coretemp" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_fan_ctrl_of_match);

static struct platform_driver gxp_coretemp_driver = {
	.probe		= gxp_coretemp_probe,
	.driver = {
		.name	= "gxp-coretemp",
		.of_match_table = gxp_coretemp_of_match,
	},
};
module_platform_driver(gxp_coretemp_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP core temp driver");

