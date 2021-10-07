// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/interrupt.h>

#define GPIDATL	0x40
#define GPIDATH 0x60
#define GPODATL	0xb0
#define GPODATH 0xb4
#define GPODAT2L	0xf8
#define GPODAT2H	0xfc
#define GPOOWNL	0x110
#define GPOOWNH 0x114
#define GPOOWN2L	0x118
#define GPOOWN2H	0x11c

#define GPIO_DIR_OUT 0
#define GPIO_DIR_IN  1

#define INTERRUPT_BUTTON_ADDR           0xb0
#define INTERRUPT_SYSEVT_ADDR           0x70
#define INTERRUPT_SYSEVT_MASK_ADDR      0x74

#define GPIO_XREG_INTERRUPT_BASE  59
#define GPIO_SYSEVT_INTERRUPT_BASE   201

#define GPIO_CHIP_XREG    0
#define GPIO_CHIP_COMMON  1
#define GPIO_CHIP_NUM     2

struct gxp_gpio_drvdata {
	struct regmap *csm_map;
	struct regmap *vuhc0_map;
	struct gpio_chip chip;
	int irq;
};

static int csm_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct gxp_gpio_drvdata *drvdata = dev_get_drvdata(chip->parent);
	int ret = 0;

	switch (offset) {
	case 0 ... 31:
		// GPI LOW
		regmap_read(drvdata->csm_map, GPIDATL,	&ret);
		ret = (ret & BIT(offset))?1:0;
		break;
	case 32 ... 63:
		//GPI HIGH
		regmap_read(drvdata->csm_map, GPIDATH,	&ret);
		ret = (ret & BIT(offset - 32))?1:0;
		break;
	case 64 ... 95:
		//GPO chain 1 LOW
		regmap_read(drvdata->csm_map, GPODATL,	&ret);
		ret = (ret & BIT(offset - 64))?1:0;
		break;
	case 96 ... 127:
		//GPO chain 1 HIGH
		regmap_read(drvdata->csm_map, GPODATH,	&ret);
		ret = (ret & BIT(offset - 96))?1:0;
		break;
	case 128 ...  159:
		//GPO chain 2 LOW
		regmap_read(drvdata->csm_map, GPODAT2L,	&ret);
		ret = (ret & BIT(offset - 128))?1:0;
		break;
	case 160 ... 191:
		//GPO chain 2 HIGH
		regmap_read(drvdata->csm_map, GPODAT2H,	&ret);
		ret = (ret & BIT(offset - 160))?1:0;
		break;
	case 192:
		//SW_RESET
		regmap_read(drvdata->csm_map, 0x5C,	&ret);
		ret = (ret & BIT(15))?1:0;
		break;
	case 193:
		// Placehold for NMI_OUT
		break;
	default:
		break;
	}
	return ret;
}

static void csm_gpio_set(struct gpio_chip *chip, unsigned int offset,
		int value)
{
	struct gxp_gpio_drvdata *drvdata = dev_get_drvdata(chip->parent);
	uint32_t tmp;

	switch (offset) {
	case 64 ... 95:
		//GPO chain 1 LOW
		//keep onwership setting
		regmap_read(drvdata->csm_map, GPOOWNL, &tmp);
		tmp = (tmp&BIT(offset - 64))?1:0;

		//output value
		regmap_update_bits(drvdata->csm_map, GPOOWNL,
				BIT(offset - 64), BIT(offset - 64));
		regmap_update_bits(drvdata->csm_map, GPODATL,
				BIT(offset - 64), value?BIT(offset - 64):0);

		//restore ownership setting
		regmap_update_bits(drvdata->csm_map, GPOOWNL,
				BIT(offset - 64), tmp?BIT(offset - 64):0);
		break;
	case 96 ... 127:
		//GPO chain 1 HIGH
		//keep onwership setting
		regmap_read(drvdata->csm_map, GPOOWNH, &tmp);
		tmp = (tmp&BIT(offset - 96))?1:0;

		//output value
		regmap_update_bits(drvdata->csm_map, GPOOWNH,
				BIT(offset - 96),	BIT(offset - 96));
		regmap_update_bits(drvdata->csm_map, GPODATH,
				BIT(offset - 96), value?BIT(offset - 96):0);

		//restore ownership setting
		regmap_update_bits(drvdata->csm_map, GPOOWNH,
				BIT(offset - 96), tmp?BIT(offset - 96):0);
		break;
	case 128 ... 159:
		//GPO chain 2 LOW
		//keep onwership setting
		regmap_read(drvdata->csm_map, GPOOWN2L, &tmp);
		tmp = (tmp&BIT(offset - 128))?1:0;

		//output value
		regmap_update_bits(drvdata->csm_map, GPOOWN2L,
				BIT(offset - 128), BIT(offset - 128));
		regmap_update_bits(drvdata->csm_map, GPODAT2L,
				BIT(offset - 128), value?BIT(offset - 128):0);

		//restore ownership setting
		regmap_update_bits(drvdata->csm_map, GPOOWN2L,
				BIT(offset - 128), tmp?BIT(offset - 128):0);
		break;
	case 160 ... 191:
		//GPO chain 2 HIGH
		//keep onwership setting
		regmap_read(drvdata->csm_map, GPOOWN2H,	&tmp);
		tmp = (tmp&BIT(offset - 160))?1:0;

		//output value
		regmap_update_bits(drvdata->csm_map, GPOOWN2H,
				BIT(offset - 160), BIT(offset - 160));
		regmap_update_bits(drvdata->csm_map, GPODAT2H,
				BIT(offset - 160), value?BIT(offset - 160):0);

		//restore ownership setting
		regmap_update_bits(drvdata->csm_map, GPOOWN2H,
				BIT(offset - 160), tmp?BIT(offset - 160):0);
		break;
	case 192:
		//SW_RESET
		if (value) {
			regmap_update_bits(drvdata->csm_map, 0x5C,
					BIT(0), BIT(0)); //unmask
			regmap_update_bits(drvdata->csm_map, 0x5C,
					BIT(15), BIT(15));
		} else {
			regmap_update_bits(drvdata->csm_map, 0x5C,
					BIT(15), 0);
		}
		break;
	case 193:
		// Placehold for NMI_OUT
		break;
	default:
		break;
	}
}

static int csm_gpio_get_direction(struct gpio_chip *chip,
		unsigned int offset)
{
	int ret = 0;

	switch (offset) {
	case 0 ... 63:
		ret = GPIO_DIR_IN;
		break;
	case 64 ... 191:
		ret = GPIO_DIR_OUT;
		break;
	case 192 ... 193:
		ret = GPIO_DIR_OUT;
		break;
	case 194:
		ret = GPIO_DIR_IN;
		break;
	default:
		break;
	}

	return ret;
}

static int csm_gpio_direction_input(struct gpio_chip *chip,
		unsigned int offset)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	case 0 ... 63:
		ret = 0;
		break;
	case 194:
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int csm_gpio_direction_output(struct gpio_chip *chip,
		unsigned int offset, int value)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	case 64 ... 191:
	case 192 ... 193:
		csm_gpio_set(chip, offset, value);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int vuhc_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct gxp_gpio_drvdata *drvdata = dev_get_drvdata(chip->parent);
	unsigned int val;
	int ret = 0;

	if (offset < 8) {
		regmap_read(drvdata->vuhc0_map, 0x64 + 4 * offset,   &val);
		ret = (val&BIT(13))?1:0;
	}

	return ret;
}

static void vuhc_gpio_set(struct gpio_chip *chip, unsigned int offset,
		int value)
{
	//struct gxp_gpio_drvdata *drvdata = dev_get_drvdata(chip->parent);

	switch (offset) {
	default:
		break;
	}
}

static int vuhc_gpio_get_direction(struct gpio_chip *chip,
		unsigned int offset)
{
	int ret = 0;

	switch (offset) {
	case 0:
	case 1:
	case 2:
		ret = GPIO_DIR_IN;
		break;
	default:
		break;
	}

	return ret;
}

static int vuhc_gpio_direction_input(struct gpio_chip *chip,
		unsigned int offset)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	case 0:
	case 1:
	case 2:
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int vuhc_gpio_direction_output(struct gpio_chip *chip,
		unsigned int offset, int value)
{
	int ret = -ENOTSUPP;

	switch (offset) {
	default:
		break;
	}

	return ret;
}

static int gxp_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret = 0;

	if (offset < 200)
		ret = csm_gpio_get(chip, offset);
	else if (offset >= 250 && offset < 300)
		ret = vuhc_gpio_get(chip, offset - 250);
	return ret;
}

static void gxp_gpio_set(struct gpio_chip *chip,
			unsigned int offset, int value)
{
	if (offset < 200)
		csm_gpio_set(chip, offset, value);
	else if (offset >= 250 && offset < 300)
		vuhc_gpio_set(chip, offset - 250, value);
}

static int gxp_gpio_get_direction(struct gpio_chip *chip,
		unsigned int offset)
{
	int ret = 0;

	if (offset < 200)
		ret = csm_gpio_get_direction(chip, offset);
	else if (offset >= 250 && offset < 300)
		ret = vuhc_gpio_get_direction(chip, offset - 250);
	return ret;
}

static int gxp_gpio_direction_input(struct gpio_chip *chip,
				unsigned int offset)
{
	int ret = 0;

	if (offset < 200)
		ret = csm_gpio_direction_input(chip, offset);
	else if (offset >= 250 && offset < 300)
		ret = vuhc_gpio_direction_input(chip, offset - 250);
	return ret;
}

static int gxp_gpio_direction_output(struct gpio_chip *chip,
				unsigned int offset, int value)
{
	int ret = 0;

	if (offset < 200)
		ret = csm_gpio_direction_output(chip, offset, value);
	else if (offset >= 250 && offset < 300)
		ret = vuhc_gpio_direction_output(chip, offset - 250, value);
	return ret;
}

const static struct gpio_chip common_chip = {
	.label			= "gxp_gpio",
	.owner			= THIS_MODULE,
	.get			= gxp_gpio_get,
	.set			= gxp_gpio_set,
	.get_direction = gxp_gpio_get_direction,
	.direction_input = gxp_gpio_direction_input,
	.direction_output = gxp_gpio_direction_output,
	.base = -1,
	//.can_sleep		= true,
};

static int gxp_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct gxp_gpio_drvdata *drvdata;
	struct device *dev = &pdev->dev;
	struct device *parent;

	parent = dev->parent;
	if (!parent) {
		dev_err(dev, "no parent\n");
		return -ENODEV;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_gpio_drvdata),
				GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);

	drvdata->csm_map = syscon_regmap_lookup_by_phandle(dev->of_node,
							"csm_handle");
	if (IS_ERR(drvdata->csm_map)) {
		dev_err(dev, "failed to map csm_handle\n");
		return -ENODEV;
	}

	drvdata->vuhc0_map = syscon_regmap_lookup_by_phandle(dev->of_node,
							"vuhc0_handle");
	if (IS_ERR(drvdata->vuhc0_map)) {
		dev_err(dev, "failed to map vuhc0_handle\n");
		return -ENODEV;
	}

	drvdata->chip = common_chip;
	drvdata->chip.ngpio = 300;
					// 0~63: csm GPI
					// 64~127: csm GPO chain1
					// 128~191: csm GPO chain2
					// 192~199: csm misc.
					// 200~249: fn2 misc.
					// 250~299: vuhc misc.
	drvdata->chip.parent = &pdev->dev;

	ret = devm_gpiochip_add_data(&pdev->dev, &drvdata->chip, NULL);
	if (ret < 0)
		dev_err(&pdev->dev, "could not register gpiochip, %d\n", ret);

	return 0;
}

static const struct of_device_id gxp_gpio_of_match[] = {
	{ .compatible = "hpe,gxp-gpio"},
	{}
};
MODULE_DEVICE_TABLE(of, gxp_gpio_of_match);

static struct platform_driver gxp_gpio_driver = {
	.driver = {
		.name	= "gxp-gpio",
		.of_match_table = gxp_gpio_of_match,
	},
	.probe		= gxp_gpio_probe,
};
module_platform_driver(gxp_gpio_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("GPIO interface for GXP");
