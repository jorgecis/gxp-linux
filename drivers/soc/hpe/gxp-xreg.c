// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sysfs.h>

#include "gxp-soclib.h"

#define GPIO_DIR_OUT 0
#define GPIO_DIR_IN  1

#define XREG_SIDEBAND_SEL	0x40

#define XREG_INT_GRP_STAT_MASK	0x88
#define XREG_INT_HI_PRI_EN	0x8C
#define XREG_INT_GRP5_BASE	0xB0
#define XREG_INT_GRP5_FLAG	0xB0
#define XREG_INT_GRP5_MASK	0xB1
#define XREG_INT_GRP5_STAT	0xB2
#define XREG_INT_GRP5_PIN_BASE	59

enum xreg_gpio_pn {
	IOP_LED1 = 0,
	IOP_LED2,
	IOP_LED3,
	IOP_LED4,
	IOP_LED5,
	IOP_LED6,
	IOP_LED7,
	IOP_LED8,
	FAN1_INST = 8,
	FAN2_INST,
	FAN3_INST,
	FAN4_INST,
	FAN5_INST,
	FAN6_INST,
	FAN7_INST,
	FAN8_INST,
	FAN9_INST,
	FAN10_INST,
	FAN11_INST,
	FAN12_INST,
	FAN13_INST,
	FAN14_INST,
	FAN15_INST,
	FAN16_INST,
	FAN1_FAIL = 24,
	FAN2_FAIL,
	FAN3_FAIL,
	FAN4_FAIL,
	FAN5_FAIL,
	FAN6_FAIL,
	FAN7_FAIL,
	FAN8_FAIL,
	FAN9_FAIL,
	FAN10_FAIL,
	FAN11_FAIL,
	FAN12_FAIL,
	FAN13_FAIL,
	FAN14_FAIL,
	FAN15_FAIL,
	FAN16_FAIL,
	FAN1_ID = 40,
	FAN2_ID,
	FAN3_ID,
	FAN4_ID,
	FAN5_ID,
	FAN6_ID,
	FAN7_ID,
	FAN8_ID,
	FAN9_ID,
	FAN10_ID,
	FAN11_ID,
	FAN12_ID,
	FAN13_ID,
	FAN14_ID,
	FAN15_ID,
	FAN16_ID,
	LED_IDENTIFY = 56,
	LED_HEALTH_RED,
	LED_HEALTH_AMBER,
	PWR_BTN_INT = 59,
	UID_PRESS_INT,
	SLP_INT,
	ACM_FORCE_OFF = 70,
	ACM_REMOVED,
	ACM_REQ_N
};

struct gxp_xreg_drvdata {
	void __iomem *base;
	struct regmap *xreg_map;
	struct gpio_chip gpio_chip;
	int irq;
};

static ssize_t server_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *drvdata = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	regmap_read(drvdata->xreg_map, 0x0, &value);
	ret = sprintf(buf, "0x%04x", (value&0xffff00)>>8);

	return ret;
}
static DEVICE_ATTR_RO(server_id);

static ssize_t sideband_sel_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct gxp_xreg_drvdata *drvdata = dev_get_drvdata(dev);
	int value;
	ssize_t ret;

	regmap_read(drvdata->xreg_map, 0x40, &value);
	ret = sprintf(buf, "0x%02x", value&0x03);

	return ret;
}

static ssize_t sideband_sel_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct gxp_xreg_drvdata *drvdata = dev_get_drvdata(dev);
	int input;
	int rc;

	rc = kstrtoint(buf, 0, &input);
	if (rc < 0)
		return -EINVAL;

	if (input & 0x03)
		return -EINVAL;

	regmap_update_bits(drvdata->xreg_map, 0x40, 0x03, input);

	return count;
}
static DEVICE_ATTR_RW(sideband_sel);

static struct attribute *xreg_attrs[] = {
	&dev_attr_server_id.attr,
	&dev_attr_sideband_sel.attr,
	NULL,
};
ATTRIBUTE_GROUPS(xreg);

static int sysfs_register(struct device *parent, struct gxp_xreg_drvdata *xreg)
{
	struct device *dev;

	dev = device_create_with_groups(soc_class, parent, 0,
					xreg, xreg_groups, "xreg");
	if (IS_ERR(dev))
		return PTR_ERR(dev);

	return 0;
}

static int gxp_gpio_xreg_get(struct gpio_chip *chip, unsigned int offset)
{
	struct gxp_xreg_drvdata *drvdata = dev_get_drvdata(chip->parent);
	unsigned int val;
	int ret = 0;

	switch (offset) {
	case IOP_LED1 ... IOP_LED8:
		//offset 0x40 bit 0~7
		regmap_read(drvdata->xreg_map, 0x04, &val);
		ret = (val&BIT(offset))?1:0;
		break;
	case FAN1_INST ...FAN8_INST:
		//0x26 fan[8:1]_inst
		regmap_read(drvdata->xreg_map, 0x24, &val);
		ret = (val&BIT((offset - FAN1_INST) + 24))?1:0;
		break;
	case FAN9_INST ... FAN16_INST:
		//0x27 fan[16:9]_inst
		regmap_read(drvdata->xreg_map, 0x28, &val);
		ret = (val&BIT(offset - FAN9_INST))?1:0;
		break;
	case FAN1_FAIL ... FAN16_FAIL:
		//0x29 fan[8:1]_fail
		//0x2A fan[16:9]_fail
		regmap_read(drvdata->xreg_map, 0x28, &val);
		ret = (val&BIT((offset - FAN1_FAIL) + 8))?1:0;
		break;
	case FAN1_ID ... FAN8_ID:
		//0x2B fan[8:1]_id
		regmap_read(drvdata->xreg_map, 0x28, &val);
		ret = (val&BIT((offset - FAN1_ID) + 24))?1:0;
		break;
	case FAN9_ID ... FAN16_ID:
		//0x2C fan[16:9]_id
		regmap_read(drvdata->xreg_map, 0x2c, &val);
		ret = (val&BIT(offset - FAN9_ID))?1:0;
		break;
	case PWR_BTN_INT ... SLP_INT:
		regmap_read(drvdata->xreg_map, XREG_INT_GRP5_FLAG, &val);
		ret = (val&BIT((offset - PWR_BTN_INT) + 16))?0:1;  // Active_low for default
		break;
	case 62 ... 65:
		// Placehold for NMI_BUTTON/RESET_BUTTON/SIO_S5/SIO_ONCONTROL
		break;
	case ACM_FORCE_OFF ... ACM_REQ_N:
		//0x0a bit0 ... bit2
		regmap_read(drvdata->xreg_map, 0x08, &val);
		ret = (val&BIT((offset - ACM_FORCE_OFF) + 16))?0:1;
		break;
	default:
		break;
	}

	return ret;
}

static void gxp_gpio_xreg_set(struct gpio_chip *chip,
			unsigned int offset, int value)
{
	struct gxp_xreg_drvdata *drvdata = dev_get_drvdata(chip->parent);

	switch (offset) {
	case IOP_LED1 ... IOP_LED8:
		//offset 0x04 bit 0~7
		regmap_update_bits(drvdata->xreg_map, 0x04, BIT(offset),
				value == 0?0:BIT(offset));
		break;
	case LED_IDENTIFY:
		//offset 0x05 bit 7:6
		regmap_update_bits(drvdata->xreg_map, 0x04, BIT(15)|BIT(14),
				value == 0?BIT(15):BIT(15)|BIT(14));
		break;
	case LED_HEALTH_RED:
		//offset 0x0d bit 7
		regmap_update_bits(drvdata->xreg_map, 0x0c, BIT(15),
				value == 0?0:BIT(15));
		break;
	case LED_HEALTH_AMBER:
		//offset 0x0d bit 6
		regmap_update_bits(drvdata->xreg_map, 0x0c, BIT(14),
				value == 0?0:BIT(14));
		break;
	case ACM_FORCE_OFF:
		//offset 0x0a bit0 = 0x08 bit16
		regmap_update_bits(drvdata->xreg_map, 0x08, BIT(16),
				value == 0?0:BIT(16));
		break;
	case ACM_REQ_N:
		//offset 0x0a bit2 = 0x08 bit18
		regmap_update_bits(drvdata->xreg_map, 0x08, BIT(18),
				value == 0?0:BIT(18));
		break;
	default:
		break;
	}
}

static int gxp_gpio_xreg_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	int ret = GPIO_DIR_IN;

	switch (offset) {
	case IOP_LED1 ... IOP_LED8:
	case LED_IDENTIFY ... LED_HEALTH_AMBER:
	case ACM_FORCE_OFF:
	case ACM_REQ_N:
		ret = GPIO_DIR_OUT;
		break;
	default:
		break;
	}

	return ret;
}

static int gxp_gpio_xreg_direction_input(struct gpio_chip *chip,
				unsigned int offset)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	case 8 ... 55:
		ret = 0;
		break;
	case 59 ... 65:
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int gxp_gpio_xreg_direction_output(struct gpio_chip *chip,
				unsigned int offset, int value)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	case IOP_LED1 ... IOP_LED8:
	case LED_IDENTIFY ... LED_HEALTH_AMBER:
	case ACM_FORCE_OFF:
	case ACM_REQ_N:
		gxp_gpio_xreg_set(chip, offset, value);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static void gxp_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct gxp_xreg_drvdata *drvdata = dev_get_drvdata(chip->parent);
	unsigned int val;

	// Read latched interrupt
	regmap_read(drvdata->xreg_map, XREG_INT_GRP5_BASE, &val);

	//Clear latched interrupt
	regmap_update_bits(drvdata->xreg_map, XREG_INT_GRP5_BASE,
			0xFF, 0xFF);
}

static void gxp_gpio_irq_set_mask(struct irq_data *d, bool set)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct gxp_xreg_drvdata *drvdata = dev_get_drvdata(chip->parent);

	regmap_update_bits(drvdata->xreg_map, XREG_INT_GRP5_BASE,
			BIT(8)|BIT(10), set == true ? 0:BIT(8)|BIT(10));
	regmap_update_bits(drvdata->xreg_map, XREG_INT_GRP5_BASE,
			0xFF, 0xFF);
}

static void gxp_gpio_irq_mask(struct irq_data *d)
{
	gxp_gpio_irq_set_mask(d, false);
}

static void gxp_gpio_irq_unmask(struct irq_data *d)
{
	gxp_gpio_irq_set_mask(d, true);
}

static int gxp_gpio_set_type(struct irq_data *d, unsigned int type)
{
	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	return 0;
}

static irqreturn_t gxp_xreg_irq_handle(int irq, void *_drvdata)
{
	struct gxp_xreg_drvdata *drvdata = (struct gxp_xreg_drvdata *) _drvdata;
	unsigned int val, girq, i;

	//handle xreg interrupt group5
	val = readb(drvdata->base + XREG_INT_GRP5_FLAG);

	for_each_set_bit(i, (unsigned long *)&val, 3) {
		girq = irq_find_mapping(drvdata->gpio_chip.irq.domain,
											i + XREG_INT_GRP5_PIN_BASE);
		generic_handle_irq(girq);
	}

	return IRQ_HANDLED;
}

const static struct gpio_chip xreg_chip = {
	.label			= "gxp-xreg",
	.owner			= THIS_MODULE,
	.get			= gxp_gpio_xreg_get,
	.set			= gxp_gpio_xreg_set,
	.get_direction = gxp_gpio_xreg_get_direction,
	.direction_input = gxp_gpio_xreg_direction_input,
	.direction_output = gxp_gpio_xreg_direction_output,
	.base = -1,
	//.can_sleep		= true,
};

static struct irq_chip gxp_gpio_irqchip = {
	.name		= "gxp-xreg",
	.irq_ack	= gxp_gpio_irq_ack,
	.irq_mask	= gxp_gpio_irq_mask,
	.irq_unmask	= gxp_gpio_irq_unmask,
	.irq_set_type	= gxp_gpio_set_type,
};

static const struct of_device_id gxp_xreg_of_match[] = {
	{ .compatible = "hpe,gxp-xreg" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_xreg_of_match);

static int gxp_xreg_probe(struct platform_device *pdev)
{
	int ret;
	struct gxp_xreg_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev,
				sizeof(struct gxp_xreg_drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	drvdata->xreg_map = syscon_regmap_lookup_by_compatible("hpe,gxp-xreg");
	if (IS_ERR(drvdata->xreg_map)) {
		dev_err(&pdev->dev, "Unable to find xreg regmap\n");
		return PTR_ERR(drvdata->xreg_map);
	}

	drvdata->gpio_chip = xreg_chip;
	drvdata->gpio_chip.ngpio = 100;
	drvdata->gpio_chip.parent = &pdev->dev;

	ret = devm_gpiochip_add_data(&pdev->dev, &drvdata->gpio_chip, NULL);
	if (ret < 0)
		dev_err(&pdev->dev, "Could not register gpiochip for xreg, %d\n", ret);

	ret = gpiochip_irqchip_add(&drvdata->gpio_chip,
	&gxp_gpio_irqchip, 0, handle_edge_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_info(&pdev->dev, "Could not add irqchip - %d\n", ret);
		gpiochip_remove(&drvdata->gpio_chip);
		return ret;
	}

	// Set up interrupt from XReg
	// Group5 Mask
	regmap_update_bits(drvdata->xreg_map, XREG_INT_HI_PRI_EN, BIT(4), 0x10);
	regmap_update_bits(drvdata->xreg_map, XREG_INT_GRP_STAT_MASK, BIT(4), 0x00);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Get irq from platform fail - %d\n", ret);
		return ret;
	}
	drvdata->irq = ret;

	ret = devm_request_irq(&pdev->dev, drvdata->irq, gxp_xreg_irq_handle,
							IRQF_SHARED, "gxp-xreg", drvdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ handler failed - %d\n", ret);
		return ret;
	}

	ret = sysfs_register(&pdev->dev, drvdata);
	if (ret < 0) {
		dev_warn(&pdev->dev, "Unable to register sysfs\n");
		return ret;
	}

	return 0;
}

static struct platform_driver gxp_xreg_driver = {
	.probe = gxp_xreg_probe,
	.driver = {
		.name = "gxp-xreg",
		.of_match_table = of_match_ptr(gxp_xreg_of_match),
	},
};
module_platform_driver(gxp_xreg_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP Xreg Driver");
