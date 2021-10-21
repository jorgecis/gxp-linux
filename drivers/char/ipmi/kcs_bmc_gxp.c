// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include "kcs_bmc_device.h"


#define DEVICE_NAME     "gxp-kcs-bmc"

#define KCS_CHANNEL_MAX       1

#define GXP_LPC_IDR           0x04
#define GXP_LPC_ODR           0x04
#define GXP_LPC_STR           0x05
#define GXP_LPC_INT           0x06

#define GXP_KCS_AUTOIRQ       0x04
#define GXP_KCS_MSKOBFINT     0x01

#define GXP_KCS_LPC_ACTIVATE  0x01

struct gxp_kcs_bmc {

	struct kcs_bmc_device kcs_bmc;

	struct resource *kcsResource;
	void __iomem *kcsRegMap;
	struct regmap *kcsCfgMap;
};

static inline struct gxp_kcs_bmc *to_gxp_kcs_bmc(struct kcs_bmc_device *kcs_bmc)
{
	return container_of(kcs_bmc, struct gxp_kcs_bmc, kcs_bmc);
}

static u8 hpe_kcs_inb(struct kcs_bmc_device *kcs_bmc, u32 reg)
{
	struct gxp_kcs_bmc *priv = to_gxp_kcs_bmc(kcs_bmc);
	u32 val = 0;

	val = readb(priv->kcsRegMap + reg);
	return (u8) val;
}

static void hpe_kcs_outb(struct kcs_bmc_device *kcs_bmc, u32 reg, u8 data)
{
	struct gxp_kcs_bmc *priv = to_gxp_kcs_bmc(kcs_bmc);

	writeb(data, priv->kcsRegMap + reg);
}

static void hpe_kcs_updateb(struct kcs_bmc_device *kcs_bmc, u32 reg, u8 mask, u8 val)
{
        struct gxp_kcs_bmc *priv = to_gxp_kcs_bmc(kcs_bmc);
        int rc;

        rc = regmap_update_bits(priv->kcsRegMap, reg, mask, val);
        WARN(rc != 0, "regmap_update_bits() failed: %d\n", rc);
}


static void hpe_kcs_turn_on_auto_irq(struct kcs_bmc_device *kcs_bmc)
{
	struct gxp_kcs_bmc *priv = to_gxp_kcs_bmc(kcs_bmc);

	writeb(GXP_KCS_AUTOIRQ | GXP_KCS_MSKOBFINT,
		priv->kcsRegMap + GXP_LPC_INT);
}

static void hpe_kcs_activate(struct kcs_bmc_device *kcs_bmc)
{
	struct gxp_kcs_bmc *priv = to_gxp_kcs_bmc(kcs_bmc);

	regmap_write(priv->kcsCfgMap, 0x00, GXP_KCS_LPC_ACTIVATE);
}

static void hpe_kcs_irq_mask_update(struct kcs_bmc_device *kcs_bmc, u8 mask, u8 state)
{
	pr_debug("%s Update mask %x state %x\n", __func__, mask, state);
}

static irqreturn_t hpe_kcs_irq(int irq, void *arg)
{
	struct kcs_bmc_device *kcs_bmc = arg;

	if (!kcs_bmc_handle_event(kcs_bmc))
		return IRQ_HANDLED;
	return IRQ_NONE;
}

static int hpe_kcs_config_irq(struct kcs_bmc_device *kcs_bmc,
	    struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Cannot get platform irq - %d\n", irq);
		return irq;
	}

	return devm_request_irq(dev, irq, hpe_kcs_irq, IRQF_SHARED,
	dev_name(dev), kcs_bmc);
}

static const struct kcs_ioreg gxp_kcs_bmc_ioregs[KCS_CHANNEL_MAX] = {
	{ .idr = GXP_LPC_IDR, .odr = GXP_LPC_ODR, .str = GXP_LPC_STR },
};

static const struct kcs_bmc_device_ops hpe_kcs_ops = {
	.irq_mask_update = hpe_kcs_irq_mask_update,
	.io_inputb = hpe_kcs_inb,
	.io_outputb = hpe_kcs_outb,
	.io_updateb = hpe_kcs_updateb,
};


static int hpe_kcs_probe(struct platform_device *pdev)
{
	struct gxp_kcs_bmc *priv;
	struct kcs_bmc_device *kcs_bmc;
	u32 chan;
	int rc;

	// kcs_chan => kcs channel
	rc = of_property_read_u32(pdev->dev.of_node, "kcs_chan", &chan);
	if ((rc != 0) || (chan == 0 || chan > KCS_CHANNEL_MAX)) {
		dev_err(&pdev->dev, "no valid 'kcs_chan' configured\n");
		return -ENODEV;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;


	kcs_bmc = &priv->kcs_bmc;
	kcs_bmc->dev = &pdev->dev;
	kcs_bmc->channel = chan;
	kcs_bmc->ioreg = gxp_kcs_bmc_ioregs[chan - 1];
	kcs_bmc->ops = &hpe_kcs_ops;

	spin_lock_init(&kcs_bmc->lock);

	priv->kcsResource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->kcsRegMap = devm_ioremap_resource(&pdev->dev, priv->kcsResource);
	if (IS_ERR(priv->kcsRegMap))
		return PTR_ERR(priv->kcsRegMap);

	priv->kcsCfgMap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"kcs-bmc-cfg");
	if (IS_ERR(priv->kcsCfgMap)) {
		dev_err(&pdev->dev, "Couldn't get KCS configuration regmap\n");
		return -ENODEV;
	}


	platform_set_drvdata(pdev, priv);

	rc = hpe_kcs_config_irq(kcs_bmc, pdev);
	if (rc) {
		dev_err(&pdev->dev, "Fail to register IRQ - %d\n", rc);
		return rc;
	}

	rc = kcs_bmc_add_device(&priv->kcs_bmc);
	if (rc) {
		dev_err(&pdev->dev, "Unable to register device\n");
		return rc;
	}

	// Turn on auto-IRQ to host and only interrupt IOP when IBF is set.
	dev_info(&pdev->dev, "Turn on Auto IRQ\n");
	hpe_kcs_turn_on_auto_irq(kcs_bmc);

	// Turn on KCS0
	dev_info(&pdev->dev, "Activate KCS Device\n");
	hpe_kcs_activate(kcs_bmc);

	pr_info("channel=%u idr=0x%x odr=0x%x str=0x%x\n",
		chan, kcs_bmc->ioreg.idr, kcs_bmc->ioreg.odr,
		kcs_bmc->ioreg.str);

	return 0;
}

static int hpe_kcs_remove(struct platform_device *pdev)
{
	struct gxp_kcs_bmc *priv = platform_get_drvdata(pdev);
	struct kcs_bmc_device *kcs_bmc = &priv->kcs_bmc;

	kcs_bmc_remove_device(kcs_bmc);

	return 0;
}

static const struct of_device_id gxp_kcs_bmc_match[] = {
	{ .compatible = "hpe,gxp-kcs-bmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, gxp_kcs_bmc_match);

static struct platform_driver gxp_kcs_bmc_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = gxp_kcs_bmc_match,
	},
	.probe  = hpe_kcs_probe,
	.remove = hpe_kcs_remove,
};
module_platform_driver(gxp_kcs_bmc_driver);

MODULE_AUTHOR("John Chung <john.chung@hpe.com>");
MODULE_AUTHOR("Jorge Cisneros <jorge.cisneros@hpe.com>");
MODULE_DESCRIPTION("HPE device interface to the KCS BMC device");
