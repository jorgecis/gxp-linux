// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>

#define PSU_MAX 4
struct gxp_power_drvdata {
	struct device	*dev;
	struct device	*hwmon_dev;
	u16 total_power;
	int psu_number;
	struct i2c_client *psu_client[PSU_MAX];
};

struct gxp_psu_drvdata {
	struct i2c_client *client;
	u16 input_power;
	u16 input_voltage;
	u16 input_current;
	u16 output_power;
	u16 output_voltage;
	u16 output_current;
	s16 inlet_temp;
	u16 fan_speed;
	struct mutex update_lock;
};

static void find_psu_device(struct device *dev)
{
	struct gxp_power_drvdata *drvdata = dev_get_drvdata(dev);
	// struct gxp_psu_drvdata *psu_data;
	struct device_node *np;
	int i = 0;

	drvdata->total_power = 0;
	drvdata->psu_number = 0;
	while (i < PSU_MAX) {
		np = of_parse_phandle(dev->of_node, "psu_phandle", i);
		if (!np)
			break;

		drvdata->psu_number++;

		drvdata->psu_client[i] = of_find_i2c_device_by_node(np);

		dev_info(dev, "Found PSU--addr:%x num:%d\n",
			drvdata->psu_client[i]->addr, drvdata->psu_number);
		of_node_put(np);
		np = NULL;
		i++;
	}
}

static ssize_t show_power_input(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_power_drvdata *drvdata = dev_get_drvdata(dev);
	struct gxp_psu_drvdata *psu_data;
	u16 total_power = 0;
	int i = 0;

	while (i < drvdata->psu_number) {
		if (drvdata->psu_client[i] != NULL) {
			psu_data = i2c_get_clientdata(drvdata->psu_client[i]);
			total_power += psu_data->input_power;
			dev_dbg(dev, "addr%x num:%d i:%d pwr:%u tpwr:%u\n",
					drvdata->psu_client[i]->addr,
					drvdata->psu_number, i,
					psu_data->input_power, total_power);
			i++;
		}
	}

	return sprintf(buf, "%u\n", total_power);
}

static SENSOR_DEVICE_ATTR(power1_input, 0444, show_power_input, NULL, 0);

static struct attribute *gxp_power_attrs[] = {
	&sensor_dev_attr_power1_input.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(gxp_power);

static int gxp_power_probe(struct platform_device *pdev)
{
	struct gxp_power_drvdata *drvdata;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_power_drvdata),
			GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);
	find_psu_device(&pdev->dev);

	drvdata->hwmon_dev = devm_hwmon_device_register_with_groups(&pdev->dev,
			"power", drvdata, gxp_power_groups);
	return PTR_ERR_OR_ZERO(drvdata->hwmon_dev);
}

static const struct of_device_id gxp_power_of_match[] = {
	{ .compatible = "hpe,gxp-power" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_power_of_match);

static struct platform_driver gxp_power_driver = {
	.probe		= gxp_power_probe,
	.driver = {
		.name	= "gxp-power",
		.of_match_table = gxp_power_of_match,
	},
};

module_platform_driver(gxp_power_driver);

MODULE_AUTHOR("Louis Hsu <kai-hsiang.hsu@hpe.com>");
MODULE_DESCRIPTION("HPE GXP POWER driver");
