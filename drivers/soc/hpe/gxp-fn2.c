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
#include <linux/wait.h>
#include <linux/sched.h>

#include "gxp-soclib.h"

#define GPIO_DIR_OUT 0
#define GPIO_DIR_IN  1

#define FN2_VPBTN	0x46
#define FN2_SEVSTAT	0x70
#define PGOOD_MASK 0x01
#define PERST_MASK 0x02
#define FN2_SEVMASK	0x74

DECLARE_WAIT_QUEUE_HEAD(gxp_fn2);
unsigned int gxp_pgood_trigger;

enum xreg_gpio_pn {
	VPBTN = 0,			//out
	PGOOD,					//in
	PERST,					//in
	POST_COMPLETE,	//in
};

struct gxp_fn2_drvdata {
	void __iomem *base;
	struct regmap *fn2_map;
	struct regmap *xreg_map;
	struct gpio_chip gpio_chip;
	int irq;
};

static int gxp_fn2_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(chip->parent);
	unsigned int val;
	int ret = 0;

	switch (offset) {
	case PGOOD:
		//offset 0x70 bit 24
		regmap_read(drvdata->fn2_map, FN2_SEVSTAT, &val);
		ret = (val&BIT(24))?1:0;
		if (ret) {
			// We signal the transition to power good
			gxp_pgood_trigger = 1;
			wake_up_interruptible(&gxp_fn2);
		} else {
			// We signal the transition to power down
			gxp_pgood_trigger = 2;
			wake_up_interruptible(&gxp_fn2);
		}
		break;
	case PERST:
		//offset 0x70 bit 25
		regmap_read(drvdata->fn2_map, FN2_SEVSTAT, &val);
		ret = (val&BIT(25))?1:0;
		break;
	case POST_COMPLETE:
		//todo: read from sram
	default:
		break;
	}

	return ret;
}

static void gxp_fn2_gpio_set(struct gpio_chip *chip, unsigned int offset,
		int value)
{
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(chip->parent);

	switch (offset) {
	case VPBTN:
		//offset 0xF=0x04
		regmap_update_bits(drvdata->xreg_map, 0x0c,
				0xFF000000, 0x04000000);
		//offset 0x44 bit 16
		regmap_update_bits(drvdata->fn2_map, 0x44, BIT(16),
				value == 0?0:BIT(16));
		break;
	default:
		break;
	}
}

static int gxp_fn2_gpio_get_direction(struct gpio_chip *chip,
		unsigned int offset)
{
	int ret = GPIO_DIR_IN;

	switch (offset) {
	case VPBTN:
		ret = GPIO_DIR_OUT;
		break;
	default:
		break;
	}

	return ret;
}

static int gxp_fn2_gpio_direction_input(struct gpio_chip *chip,
		unsigned int offset)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	case PGOOD:
	case PERST:
	case POST_COMPLETE:
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int gxp_fn2_gpio_direction_output(struct gpio_chip *chip,
		unsigned int offset, int value)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	case VPBTN:
		gxp_fn2_gpio_set(chip, offset, value);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static void gxp_fn2_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(chip->parent);
	unsigned int val;

	// Read latched interrupt
	regmap_read(drvdata->fn2_map, FN2_SEVSTAT, &val);

	//Clear latched interrupt
	regmap_update_bits(drvdata->fn2_map, FN2_SEVSTAT,
			0xFFFF, 0xFFFF);
}

static void gxp_fn2_gpio_irq_set_mask(struct irq_data *d, bool set)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	struct gxp_fn2_drvdata *drvdata = dev_get_drvdata(chip->parent);

	regmap_update_bits(drvdata->fn2_map, FN2_SEVMASK,
			BIT(0), set == true ? BIT(0):0);
}

static void gxp_fn2_gpio_irq_mask(struct irq_data *d)
{
	gxp_fn2_gpio_irq_set_mask(d, false);
}

static void gxp_fn2_gpio_irq_unmask(struct irq_data *d)
{
	gxp_fn2_gpio_irq_set_mask(d, true);
}

static int gxp_fn2_gpio_set_type(struct irq_data *d, unsigned int type)
{
	if (type & IRQ_TYPE_LEVEL_MASK)
		irq_set_handler_locked(d, handle_level_irq);
	else
		irq_set_handler_locked(d, handle_edge_irq);

	return 0;
}

static irqreturn_t gxp_fn2_irq_handle(int irq, void *_drvdata)
{
	struct gxp_fn2_drvdata *drvdata = (struct gxp_fn2_drvdata *)_drvdata;
	unsigned int val, girq;

	//handle system event
	val = readb(drvdata->base + FN2_SEVSTAT);

	if (val & PGOOD_MASK) {
		girq = irq_find_mapping(drvdata->gpio_chip.irq.domain, PGOOD);
		generic_handle_irq(girq);
	}
/*
 *	if (val & PERST_MASK) {
 *		girq = irq_find_mapping(drvdata->gpio_chip.irq.domain, PERST);
 *		generic_handle_irq(girq);
 *	}
 */
	return IRQ_HANDLED;
}

const static struct gpio_chip fn2_chip = {
	.label			= "gxp-fn2",
	.owner			= THIS_MODULE,
	.get			= gxp_fn2_gpio_get,
	.set			= gxp_fn2_gpio_set,
	.get_direction = gxp_fn2_gpio_get_direction,
	.direction_input = gxp_fn2_gpio_direction_input,
	.direction_output = gxp_fn2_gpio_direction_output,
	.base = -1,
	//.can_sleep		= true,
};

static struct irq_chip gxp_gpio_irqchip = {
	.name		= "gxp-fn2",
	.irq_ack	= gxp_fn2_gpio_irq_ack,
	.irq_mask	= gxp_fn2_gpio_irq_mask,
	.irq_unmask	= gxp_fn2_gpio_irq_unmask,
	.irq_set_type	= gxp_fn2_gpio_set_type,
};

static const struct of_device_id gxp_fn2_of_match[] = {
	{ .compatible = "hpe,gxp-fn2" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_fn2_of_match);

static int gxp_fn2_probe(struct platform_device *pdev)
{
	int ret;
	struct gxp_fn2_drvdata *drvdata;
	struct resource *res;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_fn2_drvdata),
				GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	drvdata->fn2_map = syscon_regmap_lookup_by_compatible("hpe,gxp-fn2");
	if (IS_ERR(drvdata->fn2_map)) {
		dev_err(&pdev->dev, "Unable to find fn2 regmap\n");
		return PTR_ERR(drvdata->fn2_map);
	}

	drvdata->xreg_map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"xreg_handle");
	if (IS_ERR(drvdata->xreg_map)) {
		dev_err(&pdev->dev, "failed to map xreg_handle\n");
		return -ENODEV;
	}

	drvdata->gpio_chip = fn2_chip;
	drvdata->gpio_chip.ngpio = 100;
	drvdata->gpio_chip.parent = &pdev->dev;

	ret = devm_gpiochip_add_data(&pdev->dev, &drvdata->gpio_chip, NULL);
	if (ret < 0)
		dev_err(&pdev->dev, "Could not register gpiochip for fn2, %d\n", ret);

	ret = gpiochip_irqchip_add(&drvdata->gpio_chip,
	&gxp_gpio_irqchip, 0, handle_edge_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_info(&pdev->dev, "Could not add irqchip - %d\n", ret);
		gpiochip_remove(&drvdata->gpio_chip);
		return ret;
	}

	// Set up interrupt from fn2 system event reg

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Get irq from platform fail - %d\n", ret);
		return ret;
	}
	drvdata->irq = ret;

	ret = devm_request_irq(&pdev->dev, drvdata->irq, gxp_fn2_irq_handle,
							IRQF_SHARED, "gxp-fn2", drvdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ handler failed - %d\n", ret);
		return ret;
	}

	return 0;
}

static struct platform_driver gxp_fn2_driver = {
	.probe = gxp_fn2_probe,
	.driver = {
		.name = "gxp-fn2",
		.of_match_table = of_match_ptr(gxp_fn2_of_match),
	},
};
module_platform_driver(gxp_fn2_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP FN2 Driver");
