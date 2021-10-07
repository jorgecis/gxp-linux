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

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define OFFSET_PWM0DUTY	0x10
#define OFFSET_PWM1DUTY	0x11
#define OFFSET_PWM2DUTY	0x12
#define OFFSET_PWM3DUTY	0x13
#define OFFSET_PWM4DUTY	0x14
#define OFFSET_PWM5DUTY	0x15
#define OFFSET_PWM6DUTY	0x16
#define OFFSET_PWM7DUTY	0x17


struct gxp_fan_ctrl_drvdata {
	struct device	*dev;
	struct device	*hwmon_dev;
	struct regmap	*xreg_map;
	struct regmap	*fn2_map;
	void __iomem	*base;
	struct mutex update_lock;
};

static ssize_t show_fault(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = (to_sensor_dev_attr(attr))->index;
	struct gxp_fan_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned char val;
	unsigned int reg;
	unsigned int mask = 0x1;

	// Check Fan present.
	regmap_read(drvdata->xreg_map, 0x28, &reg);
	reg = reg >> 8;
	mask = mask << (nr * 2); // Shift 2 bits per Fan

	val = (reg & mask) ? 1 : 0;

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_in(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int nr = (to_sensor_dev_attr(attr))->index;
	struct gxp_fan_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned char val;
	unsigned int reg;

	// Check Power Status
	regmap_read(drvdata->fn2_map, 0x70, &reg);
	if (reg & BIT(24)) {
		// Check Fan present
		regmap_read(drvdata->xreg_map, 0x24, &reg);
		reg = reg >> 24;

		// If Fan presents, then read it.
		val = (reg & BIT(nr)) ? readb(drvdata->base +
						OFFSET_PWM0DUTY + nr) : 0;
	} else {
		// Power Off
		val = 0;
	}

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_pwm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int nr = (to_sensor_dev_attr(attr))->index;
	struct gxp_fan_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned char val;

	val = readb(drvdata->base + OFFSET_PWM0DUTY + nr);

	return sprintf(buf, "%d\n", val);
}

static ssize_t store_pwm(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int nr = (to_sensor_dev_attr(attr))->index;
	struct gxp_fan_ctrl_drvdata *drvdata = dev_get_drvdata(dev);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	if (val > 255)
		return -1; // out of range

	mutex_lock(&drvdata->update_lock);

	writeb(val, drvdata->base + OFFSET_PWM0DUTY + nr);

	mutex_unlock(&drvdata->update_lock);
	return count;
}

static SENSOR_DEVICE_ATTR(pwm0, 0200 | 0444, show_pwm, store_pwm, 0);
static SENSOR_DEVICE_ATTR(pwm1, 0200 | 0444, show_pwm, store_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm2, 0200 | 0444, show_pwm, store_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm3, 0200 | 0444, show_pwm, store_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm4, 0200 | 0444, show_pwm, store_pwm, 4);
static SENSOR_DEVICE_ATTR(pwm5, 0200 | 0444, show_pwm, store_pwm, 5);
static SENSOR_DEVICE_ATTR(pwm6, 0200 | 0444, show_pwm, store_pwm, 6);
static SENSOR_DEVICE_ATTR(pwm7, 0200 | 0444, show_pwm, store_pwm, 7);

static struct sensor_device_attribute sda_in_input[] = {
	SENSOR_ATTR(fan0_input, 0444, show_in, NULL, 0),
	SENSOR_ATTR(fan1_input, 0444, show_in, NULL, 1),
	SENSOR_ATTR(fan2_input, 0444, show_in, NULL, 2),
	SENSOR_ATTR(fan3_input, 0444, show_in, NULL, 3),
	SENSOR_ATTR(fan4_input, 0444, show_in, NULL, 4),
	SENSOR_ATTR(fan5_input, 0444, show_in, NULL, 5),
	SENSOR_ATTR(fan6_input, 0444, show_in, NULL, 6),
	SENSOR_ATTR(fan7_input, 0444, show_in, NULL, 7),
};

static SENSOR_DEVICE_ATTR(fan0_fault, 0444, show_fault, NULL, 0);
static SENSOR_DEVICE_ATTR(fan1_fault, 0444, show_fault, NULL, 1);
static SENSOR_DEVICE_ATTR(fan2_fault, 0444, show_fault, NULL, 2);
static SENSOR_DEVICE_ATTR(fan3_fault, 0444, show_fault, NULL, 3);
static SENSOR_DEVICE_ATTR(fan4_fault, 0444, show_fault, NULL, 4);
static SENSOR_DEVICE_ATTR(fan5_fault, 0444, show_fault, NULL, 5);
static SENSOR_DEVICE_ATTR(fan6_fault, 0444, show_fault, NULL, 6);
static SENSOR_DEVICE_ATTR(fan7_fault, 0444, show_fault, NULL, 7);


#define IN_UNIT_ATTRS(X) (&sda_in_input[X].dev_attr.attr)

static struct attribute *gxp_fan_ctrl_attrs[] = {
	&sensor_dev_attr_fan0_fault.dev_attr.attr,
	&sensor_dev_attr_fan1_fault.dev_attr.attr,
	&sensor_dev_attr_fan2_fault.dev_attr.attr,
	&sensor_dev_attr_fan3_fault.dev_attr.attr,
	&sensor_dev_attr_fan4_fault.dev_attr.attr,
	&sensor_dev_attr_fan5_fault.dev_attr.attr,
	&sensor_dev_attr_fan6_fault.dev_attr.attr,
	&sensor_dev_attr_fan7_fault.dev_attr.attr,
	IN_UNIT_ATTRS(0),
	IN_UNIT_ATTRS(1),
	IN_UNIT_ATTRS(2),
	IN_UNIT_ATTRS(3),
	IN_UNIT_ATTRS(4),
	IN_UNIT_ATTRS(5),
	IN_UNIT_ATTRS(6),
	IN_UNIT_ATTRS(7),
	&sensor_dev_attr_pwm0.dev_attr.attr,
	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm4.dev_attr.attr,
	&sensor_dev_attr_pwm5.dev_attr.attr,
	&sensor_dev_attr_pwm6.dev_attr.attr,
	&sensor_dev_attr_pwm7.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(gxp_fan_ctrl);

static int gxp_fan_ctrl_probe(struct platform_device *pdev)
{
	struct gxp_fan_ctrl_drvdata *drvdata;
	struct device *dev = &pdev->dev;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_fan_ctrl_drvdata),
			GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	drvdata->xreg_map = syscon_regmap_lookup_by_phandle(dev->of_node,
			"xreg_handle");
	if (IS_ERR(drvdata->xreg_map)) {
		dev_err(dev, "failed to map xreg_handle\n");
		return -ENODEV;
	}

	drvdata->fn2_map = syscon_regmap_lookup_by_phandle(dev->of_node,
			"fn2_handle");
	if (IS_ERR(drvdata->fn2_map)) {
		dev_err(dev, "failed to map fn2_handle\n");
		return -ENODEV;
	}

	mutex_init(&drvdata->update_lock);

	drvdata->hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
			"fan_ctrl", drvdata, gxp_fan_ctrl_groups);
	return PTR_ERR_OR_ZERO(drvdata->hwmon_dev);
}

static const struct of_device_id gxp_fan_ctrl_of_match[] = {
	{ .compatible = "hpe,gxp-fan-ctrl" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_fan_ctrl_of_match);

static struct platform_driver gxp_fan_ctrl_driver = {
	.probe		= gxp_fan_ctrl_probe,
	.driver = {
		.name	= "gxp-fan-crrl",
		.of_match_table = gxp_fan_ctrl_of_match,
	},
};
module_platform_driver(gxp_fan_ctrl_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP Fan Ctrl driver");
