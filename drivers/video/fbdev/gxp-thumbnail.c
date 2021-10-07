// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2019 Hewlett-Packard Development Company, L.P.
 *
 * Frame Buffer Device for GXP Thumbnail
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#define THUMBNAIL_CFG                0x00
#define THUMBNAIL_HORIZ_SIZE         0x08
#define THUMBNAIL_VERT_SIZE          0x0C
#define THUMBNAIL_TNAXISTS           0x14
#define THUMBNAIL_DEST_BAR           0x18
#define THUMBNAIL_DEST_PITCH         0x1C

#define TNAXISTS_AXID                0x10000
#define TNAXISTS_AXIR                0x20000

static int gWidth;
static int gHeight;
static int gBpp;

/*
 *  Frame buffer operations
 */
static struct fb_ops gxpfb_ops = {
	.owner        = THIS_MODULE,
};

static struct fb_fix_screeninfo gxpfb_fix = {
	.id = "Thumbnail",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo gxpfb_var = {
	.height = -1,
	.width = -1,
	.activate = FB_ACTIVATE_NOW,
	.vmode = FB_VMODE_NONINTERLACED,
};

struct gxp_tn_par {
	void __iomem *base;
	void *virt_mem;
	void *phys_mem;
	uint32_t framesize;
	u32 palette[16];
};

/* static int thumbnail_clear(struct gxp_tn_par *info)
 * {
 *   void __iomem *vMem = info->virt_mem;
 *   void __iomem *vMemPos;
 *   uint32_t h, yPos;
 *
 *   yPos = gBpp * gWidth;
 *   for (h = 0; h < gHeight; h++) {
 *     vMemPos = vMem + (yPos * h);
 *     memset((void *)vMemPos, 0, yPos);
 *   }
 *
 *   return 0;
 * }
 * static int thumbnail_status(struct gxp_tn_par *info)
 * {
 *    return (readl(info->base + THUMBNAIL_CFG) & 0x01);
 * }
 */

static void thumbnail_wait_for_axi(struct gxp_tn_par *gxp_par)
{
	uint32_t busy = true;
	uint32_t time = 5000;
	uint32_t reg = 0;

	while (time--) {
		reg = readl(gxp_par->base + THUMBNAIL_TNAXISTS) &
			(TNAXISTS_AXID|TNAXISTS_AXIR);

		if (reg == 0x0) {
			busy = false;
			break;
		}
		udelay(100);
	}

	if (busy)
		pr_err("<TN_FB> AXI BUS is busy ...\n");
}


static void gxpfb_enable(struct fb_info *info)
{
	// void __iomem *base = info->screen_base;
	struct gxp_tn_par *gxp_par = (struct gxp_tn_par *)info->par;
	void __iomem *reg_base = gxp_par->base;
	uint32_t reg = 0;

	writel(0x00000000, reg_base + THUMBNAIL_CFG);
	thumbnail_wait_for_axi(gxp_par);

	if (gBpp == 4)
		writel(0x00000100, reg_base + THUMBNAIL_CFG);

	writel((gWidth - 1), reg_base + THUMBNAIL_HORIZ_SIZE);
	writel((gHeight - 1), reg_base + THUMBNAIL_VERT_SIZE);

	writel(gxp_par->phys_mem, reg_base + THUMBNAIL_DEST_BAR);
	writel((gWidth * gBpp), reg_base + THUMBNAIL_DEST_PITCH);

	thumbnail_wait_for_axi(gxp_par);

	reg = readl(reg_base + THUMBNAIL_CFG);
	writel(0x00000001 | reg, reg_base + THUMBNAIL_CFG);
	udelay(100);
}

/* static void gxpfb_disable(struct gxp_tn_par *info)
 * {
 *   writel(0x00000000, (info->base + THUMBNAIL_CFG) );
 *
 *   return;
 * }
 */

int gxpfb_probe(struct platform_device *pdev)
{
	struct gxp_tn_par *gxp_par;
	struct fb_info *info;
	struct resource *res;
	dma_addr_t physical_dma = 0;
	int err;
	int ret;
	int bits_per_pixel = 0;

	// Allocate frame buffer
	info = framebuffer_alloc(sizeof(struct gxp_tn_par), &pdev->dev);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);

	// Assigned frame buffer private data
	gxp_par = info->par;

	// Get register based memory
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gxp_par->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gxp_par->base)) {
		pr_err("<TN_FB> io-map memory allocate fail ... ");
		err = PTR_ERR(gxp_par->base);
		goto out;
	}

	// Get thumbnail configuration

	// Default bits-per-pixel = 16 bits
	of_property_read_u32(pdev->dev.of_node, "bits-per-pixel",
				&bits_per_pixel);
	switch (bits_per_pixel) {
	case 32:
		gBpp = 4;
		break;
	default:
		gBpp = 2;
		break;
	}

	// Default resolution 800x600 , max resolution 1024x768
	ret = of_property_read_u32(pdev->dev.of_node, "width", &gWidth);
	if (ret || gWidth > 1024 || gWidth < 0)
		gWidth = 800;

	ret = of_property_read_u32(pdev->dev.of_node, "height", &gHeight);
	if (ret || gHeight > 768 || gHeight < 0)
		gHeight = 600;

	pr_info("<TN_FB> gBpp = %d, gWidth = %d, gHeight = %d\n",
		gBpp, gWidth, gHeight);

	// Allocate frame buffer memory
	gxp_par->framesize = gWidth * gHeight * gBpp;
	gxp_par->virt_mem = dma_alloc_coherent(&pdev->dev, gxp_par->framesize,
						&physical_dma, GFP_KERNEL);
	if (!gxp_par->virt_mem) {
		err = -ENOMEM;
		goto out;
	}
	info->screen_base = gxp_par->virt_mem;
	gxp_par->phys_mem = (void *) physical_dma;

	info->fix = gxpfb_fix;
	info->fix.smem_start = physical_dma; //gxp_par->virt_mem;
	info->fix.smem_len = gxp_par->framesize;
	info->fix.line_length = gWidth * gBpp;

	info->var = gxpfb_var;
	info->var.xres = gWidth;
	info->var.yres = gHeight;
	info->var.bits_per_pixel = gBpp * 8;
	// Missing *line_length* from barebox
	// var->line_length ....
	info->var.blue.length = gBpp == 4 ? 8 : 5;
	info->var.blue.offset = gBpp == 4 ? 16 : 10;
	info->var.green.length = gBpp == 4 ? 8 : 5;
	info->var.green.offset = gBpp == 4 ? 8 : 5;
	info->var.red.length = gBpp == 4 ? 8 : 5;
	info->var.red.offset = 0;

	info->fbops = &gxpfb_ops;
	info->flags = FBINFO_DEFAULT;

	info->apertures = alloc_apertures(1);
	if (!info->apertures) {
		pr_err("<TN_FB> Cannot allocate memory for apertures\n");
		err = -ENOMEM;
		goto out;
	}
	info->apertures->ranges[0].base = (resource_size_t) gxp_par->virt_mem;
	info->apertures->ranges[0].size = gxp_par->framesize;

	info->pseudo_palette = gxp_par->palette;

	dev_info(&pdev->dev,
		"framebuffer at 0x%lx, 0x%x bytes, mapped to 0x%p\n",
		info->fix.smem_start, info->fix.smem_len,
		info->screen_base);
	dev_info(&pdev->dev, "format=rgb%s, mode=%dx%dx%d, linelength=%d\n",
		gBpp == 4 ? "888" : "555",
		info->var.xres, info->var.yres,
		info->var.bits_per_pixel, info->fix.line_length);

	if (register_framebuffer(info) < 0) {
		err = -EINVAL;
		goto out;
	}

	// Enable thumbnail
	gxpfb_enable(info);

	return 0;

out:
	if (gxp_par->virt_mem)
		dma_free_coherent(&pdev->dev, gxp_par->framesize,
				gxp_par->virt_mem, physical_dma);

	if (gxp_par->base)
		devm_iounmap(&pdev->dev, gxp_par->base);

	if (info)
		framebuffer_release(info);

	return err;
}


static int gxpfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct gxp_tn_par *par = info->par;

	// Unregister framebuffer device
	unregister_framebuffer(info);

	// Free io memory map
	devm_iounmap(info->dev, par->base);

	// Free DMA memory
	dma_free_coherent(info->dev, par->framesize, par->virt_mem,
			(dma_addr_t)par->phys_mem);
	info->screen_base = NULL;
	info->screen_size = 0;

	// TODO: Need to free info->aperture ?

	framebuffer_release(info);
	return 0;
}

static const struct of_device_id gxp_thumbnail_match[] = {
	{.compatible = "hpe,gxp-thumbnail"},
	{ /* null */ },
}
MODULE_DEVICE_TABLE(of, gxp_thumbnail_match);

static struct platform_driver gxpfb_driver = {
	.probe		= gxpfb_probe,
	.remove		= gxpfb_remove,
	.driver		= {
		.name		= "gxp-thumbnail",
		.of_match_table = gxp_thumbnail_match,
	}
};

module_platform_driver(gxpfb_driver);

MODULE_DESCRIPTION("GXP Thumbnail framebuffer driver");
MODULE_LICENSE("GPL");
