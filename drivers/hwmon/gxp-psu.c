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
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>

#define READ_REG_CMD		0x00
#define REG_IN_VOL		0x10
#define REG_IN_CUR		0x11
#define REG_IN_PWR		0x12
#define REG_OUT_VOL		0x20
#define REG_OUT_CUR		0x21
#define REG_OUT_PWR		0x22
#define REG_FAN_SPEED		0x40
#define REG_INLET_TEMP		0x42

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

static unsigned char cal_checksum(unsigned char *buf, unsigned long size)
{
	unsigned char sum = 0;

	while (size > 0) {
		sum += (*(buf++));
		size--;
	}
	return ((~sum)+1);
}

static unsigned char valid_checksum(unsigned char *buf, unsigned long size)
{
	unsigned char sum = 0;

	while (size > 0) {
		sum += (*(buf++));
		size--;
	}
	return sum;
}

static int psu_read_reg_word(struct gxp_psu_drvdata *drvdata,
		u8 reg, u16 *value)
{
	struct i2c_client *client = drvdata->client;
	unsigned char buf_tx[3] = {(client->addr << 1), READ_REG_CMD, reg};
	unsigned char buf_rx[3] = {0};
	unsigned char tx[3] = {0};
	unsigned char rx[3] = {0};
	unsigned char chksum = cal_checksum(buf_tx, 3);
	struct i2c_msg msgs[2] = {0};
	int ret = 0;

	tx[0] = 0;
	tx[1] = reg;
	tx[2] = chksum;
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].buf = tx;
	msgs[0].len = 3;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = rx;
	msgs[1].len = 3;
	mutex_lock(&drvdata->update_lock);
	ret = i2c_transfer(client->adapter, msgs, 2);
	mutex_unlock(&drvdata->update_lock);
	if (ret < 0) {
		dev_dbg(&client->dev,
		"gxppsu_i2c_tx_fail addr:0x%x reg:0x%x chk:0x%x ret:0x%x\n",
		client->addr, reg, chksum, ret);
		return ret;
	}

	buf_rx[0] = rx[0];
	buf_rx[1] = rx[1];
	buf_rx[2] = rx[2];
	if (valid_checksum(buf_rx, 3) != 0) {
		dev_dbg(&client->dev,
		"gxppsu_checksum_fail addr:0x%x reg:0x%x, data:%x %x %x\n",
		client->addr, reg, rx[0], rx[1], rx[2]);
		return -EAGAIN;
	}

	*value = rx[0] + (rx[1] << 8);
	dev_dbg(&client->dev, "chk:%x val:%x, %x %d\n", chksum, *value,
			client->addr << 1, reg);
	return ret;
}

static int gxppsu_update_client(struct device *dev, u8 reg)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;

	switch (reg) {
	case REG_IN_PWR:
		ret = psu_read_reg_word(drvdata, REG_IN_PWR,
				&drvdata->input_power);
		break;
	case REG_IN_VOL:
		ret = psu_read_reg_word(drvdata, REG_IN_VOL,
				&drvdata->input_voltage);
		break;
	case REG_IN_CUR:
		ret = psu_read_reg_word(drvdata, REG_IN_CUR,
				&drvdata->input_current);
		break;
	case REG_OUT_PWR:
		ret = psu_read_reg_word(drvdata, REG_OUT_PWR,
				&drvdata->output_power);
		break;
	case REG_OUT_VOL:
		ret = psu_read_reg_word(drvdata, REG_OUT_VOL,
				&drvdata->output_voltage);
		break;
	case REG_OUT_CUR:
		ret = psu_read_reg_word(drvdata, REG_OUT_CUR,
				&drvdata->output_current);
		break;
	case REG_INLET_TEMP:
		ret = psu_read_reg_word(drvdata, REG_INLET_TEMP,
				&drvdata->inlet_temp);
		break;
	case REG_FAN_SPEED:
		ret = psu_read_reg_word(drvdata, REG_FAN_SPEED,
				&drvdata->fan_speed);
		break;
	default:
		dev_err(&drvdata->client->dev, "gxppsu_error_reg 0x%x\n", reg);
		return -EOPNOTSUPP;
	}

	return ret;
}

static ssize_t show_power_input(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_IN_PWR;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", drvdata->input_power);
}

static ssize_t show_in_input(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_IN_VOL;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", drvdata->input_voltage);
}

static ssize_t show_curr_input(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_IN_CUR;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%u\n", drvdata->input_current);
}

static ssize_t show_power_output(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_OUT_PWR;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", drvdata->output_power);
}

static ssize_t show_in_output(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_OUT_VOL;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", drvdata->output_voltage);
}

static ssize_t show_curr_output(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_OUT_CUR;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", drvdata->output_current);
}

static ssize_t show_temp_input(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_INLET_TEMP;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", drvdata->inlet_temp);
}

static ssize_t show_fan_input(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct gxp_psu_drvdata *drvdata = dev_get_drvdata(dev);
	int ret = 0;
	u8 reg = REG_FAN_SPEED;

	ret = gxppsu_update_client(dev, reg);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%u\n", drvdata->fan_speed);
}

static SENSOR_DEVICE_ATTR(power1_input, 0444, show_power_input, NULL, 0);
static SENSOR_DEVICE_ATTR(in0_input, 0444, show_in_input, NULL, 1);
static SENSOR_DEVICE_ATTR(curr1_input, 0444, show_curr_input, NULL, 2);
static SENSOR_DEVICE_ATTR(power2_input, 0444, show_power_output, NULL, 3);
static SENSOR_DEVICE_ATTR(in1_input, 0444, show_in_output, NULL, 4);
static SENSOR_DEVICE_ATTR(curr2_input, 0444, show_curr_output, NULL, 5);
static SENSOR_DEVICE_ATTR(temp1_input, 0444, show_temp_input, NULL, 6);
static SENSOR_DEVICE_ATTR(fan1_input, 0444, show_fan_input, NULL, 7);

static struct attribute *gxp_psu_attrs[] = {
	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_power2_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_curr2_input.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(gxp_psu);

static int gxp_psu_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct gxp_psu_drvdata *drvdata;
	struct device *hwmon_dev;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL)) {
		return -EIO;
	}

	drvdata = devm_kzalloc(&client->dev, sizeof(struct gxp_psu_drvdata),
			GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->client = client;
	i2c_set_clientdata(client, drvdata);

	mutex_init(&drvdata->update_lock);

	hwmon_dev = devm_hwmon_device_register_with_groups(&client->dev, "psu",
			drvdata, gxp_psu_groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(&client->dev, "%s:'%s' addr:0x%x\n", dev_name(hwmon_dev),
			client->name, client->addr);

	return 0;
}

static const struct of_device_id gxp_psu_of_match[] = {
	{ .compatible = "hpe,gxp-psu" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_psu_of_match);

static struct i2c_driver gxp_psu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= gxp_psu_probe,
	.driver = {
		.name	= "gxp-psu",
		.of_match_table = gxp_psu_of_match,
	},
};
module_i2c_driver(gxp_psu_driver);

MODULE_AUTHOR("Louis Hsu <kai-hsiang.hsu@hpe.com>");
MODULE_DESCRIPTION("HPE GXP PSU driver");

