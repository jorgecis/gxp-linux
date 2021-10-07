// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "8250.h"

#define GXP_16550A_BASE     0x18

#define GXP_VUART_CMD       0x08

#define GXP_VUART_VSR       0x10
#define GXP_VUART_VCR       0x14
#define GXP_VUART_VBRD      0x16
#define GXP_VUART_VDATA     0x18

#define GXP_VUART_INT_EVENT   0x7D

#define GXP_VUART_VCR_TFIFO_DRAIN (0 << 12) /* bits[15:12] */
#define GXP_VUART_VCR_RFIFO_DRAIN (8 << 8)  /* bits[11:8] */
#define GXP_VUART_ENABLE    0x01

#define GXP_VUART_VSR_OVERRUN_MASK  0x02
#define GXP_VUART_VSR_RX_DATAVAIL   0x01
#define GXP_VUART_VSR_DTR           0x04
#define GXP_VUART_VSR_RTS           0x08
#define GXP_VUART_TX_DRAIN          0x10
#define GXP_VUART_MSR               0x20
#define GXP_VUART_LSR               0x40

#define GXP_VUART_CFG_INT_SELECT    0x40
#define GXP_VUART_CFG_DES_HIGH      0x30
#define GXP_VUART_CFG_DES_LOW       0x31

#define GXP_VUART_CFG_INT_MASK      0x0F
#define UART_LCR_DLAB_MASK          0x7F

struct gxp_vuart {
	struct device   *dev;
	void __iomem    *regs;
	struct regmap   *cfgReg;
	struct clk      *clk;
	int             line;
	struct uart_8250_port *port;
};

static ssize_t lpc_address_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gxp_vuart *vuart = dev_get_drvdata(dev);
	u32 val = 0;
	u32 tmp = 0;
	int rc;

	// Read descriptor high address
	rc = regmap_read(vuart->cfgReg, GXP_VUART_CFG_DES_HIGH, &tmp);
	WARN(rc != 0, "regmap_read() failed: %d\n", rc);
	val = tmp << 8;

	// Read descriptor low address
	rc = regmap_read(vuart->cfgReg, GXP_VUART_CFG_DES_LOW, &tmp);
	WARN(rc != 0, "regmap_read() failed: %d\n", rc);
	val |= tmp;

	return snprintf(buf, PAGE_SIZE - 1, "0x%x\n", (u16) val);
}

static ssize_t lpc_address_store(struct device *dev,
	 struct device_attribute *attr,
	 const char *buf, size_t count)
{
	struct gxp_vuart *vuart = dev_get_drvdata(dev);
	unsigned long val;
	u32 tmp = 0;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	// Write descriptor high address
	tmp = (val >> 8) & 0xFF;
	err = regmap_write(vuart->cfgReg, GXP_VUART_CFG_DES_HIGH, tmp);
	WARN(err != 0, "regmap_write() failed: %d\n", err);

	// Write descriptor low address
	tmp = (val >> 0) & 0xFF;
	err = regmap_write(vuart->cfgReg, GXP_VUART_CFG_DES_LOW, tmp);
	WARN(err != 0, "regmap_write() failed: %d\n", err);

	return count;
}

static DEVICE_ATTR_RW(lpc_address);

static ssize_t sirq_show(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	struct gxp_vuart *vuart = dev_get_drvdata(dev);
	u32 val = 0;
	int rc;

	rc = regmap_read(vuart->cfgReg, GXP_VUART_CFG_INT_SELECT, &val);
	WARN(rc != 0, "regmap_read() failed: %d\n", rc);
	val &= GXP_VUART_CFG_INT_MASK;

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", rc == 0 ? val : 0);
}

static ssize_t sirq_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct gxp_vuart *vuart = dev_get_drvdata(dev);
	unsigned long val = 0;
	u32 tmp = 0;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	// Read the interrupt select register
	err = regmap_read(vuart->cfgReg, GXP_VUART_CFG_INT_SELECT, &tmp);
	WARN(err != 0, "regmap_read() failed: %d\n", err);
	tmp &= ~GXP_VUART_CFG_INT_MASK;

	// Modify the irq-select and write it back
	tmp |= val;
	err = regmap_write(vuart->cfgReg, GXP_VUART_CFG_INT_SELECT, tmp);
	WARN(err != 0, "regmap_write() failed: %d\n", err);

	return count;
}

static DEVICE_ATTR_RW(sirq);

static ssize_t vuart_enable_show(struct device *dev,
	     struct device_attribute *attr, char *buf)
{
	struct gxp_vuart *vuart = dev_get_drvdata(dev);
	u16 reg = readw(vuart->regs + GXP_VUART_VCR);

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", reg & GXP_VUART_ENABLE);
}

static ssize_t vuart_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gxp_vuart *vuart = dev_get_drvdata(dev);
	u16 reg = readw(vuart->regs + GXP_VUART_VCR);
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	reg &= ~GXP_VUART_ENABLE;
	reg |= (val & GXP_VUART_ENABLE);

	writew(reg, vuart->regs + GXP_VUART_VCR);
	return count;
}

static DEVICE_ATTR_RW(vuart_enable);

static struct attribute *gxp_vuart_attrs[] = {
	&dev_attr_sirq.attr,
	&dev_attr_lpc_address.attr,
	&dev_attr_vuart_enable.attr,
	NULL,
};

static const struct attribute_group gxp_vuart_attr_group = {
	.attrs = gxp_vuart_attrs,
};

static void gxp_vuart_set_enabled(struct gxp_vuart *vuart, bool enabled)
{
	u16 reg = readw(vuart->regs + GXP_VUART_VCR);
	u32 idx = 0;

	if (enabled)
		reg |= (GXP_VUART_ENABLE | GXP_VUART_VCR_TFIFO_DRAIN |
			GXP_VUART_VCR_RFIFO_DRAIN);
	else
		reg &= ~(GXP_VUART_ENABLE | GXP_VUART_VCR_TFIFO_DRAIN |
			 GXP_VUART_VCR_RFIFO_DRAIN);

	writew(reg, vuart->regs + GXP_VUART_VCR);

	// To clear up FIFO buffer before starting VUART
	for (idx = 0; idx < 32; idx++)
		reg = readb(vuart->regs + GXP_VUART_VDATA);
}

static int gxp_vuart_handle_irq(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned long flags;
	uint32_t reg = 0;
	uint32_t handled = 0;
	uint32_t nRxFifo = 0;
	uint8_t  ch = 0;

	void __iomem *regBase = (up->port.membase - GXP_16550A_BASE);

	spin_lock_irqsave(&port->lock, flags);

	// Handle GXP Virtual Serial Command
	reg = readl(regBase + GXP_VUART_VSR);

	// Check interrupt event
	if (!(reg & GXP_VUART_INT_EVENT)) {
		// Nothing happen
		return 0;
	}

	if (reg & GXP_VUART_VSR_RX_DATAVAIL) {
		nRxFifo = reg >> 20;
		if (nRxFifo) {
			do {
				ch = serial_in(up, UART_RX);
				tty_insert_flip_char(&port->state->port,
						     ch, TTY_NORMAL);
			} while (--nRxFifo);

			// clear interrupt flag
			writel(GXP_VUART_VSR_RX_DATAVAIL, regBase +
			       GXP_VUART_VSR);
			handled = 1;
		}

		if (handled)
			tty_flip_buffer_push(&port->state->port);
	}

	if (reg & GXP_VUART_TX_DRAIN) {
		serial8250_modem_status(up);

		serial8250_tx_chars(up);

		// clear interrupt flag
		writel(GXP_VUART_TX_DRAIN, regBase + GXP_VUART_VSR);
		handled = 1;
	}

	if (reg & GXP_VUART_VSR_OVERRUN_MASK) {
		writel(GXP_VUART_VSR_OVERRUN_MASK, regBase + GXP_VUART_VSR);
		handled = 1;
	}

	if (reg & GXP_VUART_VSR_DTR) {
		writel(GXP_VUART_VSR_DTR, regBase + GXP_VUART_VSR);
		handled = 1;
	}

	if (reg & GXP_VUART_VSR_RTS) {
		writel(GXP_VUART_VSR_RTS, regBase + GXP_VUART_VSR);
		handled = 1;
	}

	if (reg & GXP_VUART_MSR) {
		writel(GXP_VUART_MSR, regBase + GXP_VUART_VSR);
		handled = 1;
	}

	if (reg & GXP_VUART_LSR) {
		writel(GXP_VUART_LSR, regBase + GXP_VUART_VSR);
		handled = 1;
	}

	spin_unlock_irqrestore(&port->lock, flags);

	if (handled == 0)
		pr_info("VUART IRQ NO HANDLED ...\n");
	return 1;
}

static unsigned int gxp_serial_in(struct uart_port *p, int offset)
{

	unsigned int value = readb(p->membase + (offset << p->regshift));
	return value;
}


static void gxp_serial_out(struct uart_port *p, int offset, int value)
{
	writeb(value, p->membase + (offset << p->regshift));

	// RX_PROMPT
	writeb(1, p->membase + (GXP_VUART_CMD << p->regshift));
}

static void gxp_8250_set_termios(struct uart_port *port,
				struct ktermios *termios,
				struct ktermios *old)
{
	struct uart_8250_port *uart_8250_port = up_to_u8250p(port);

	uart_8250_port->fcr |= UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10 |
	UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT;

	serial8250_do_set_termios(port, termios, old);
}

static void gxp_8250_set_divisor(struct uart_port *port, unsigned int baud,
				  unsigned int quot, unsigned int quot_frac)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	struct gxp_vuart *vuart = port->private_data;

	gxp_serial_out(port, UART_LCR, up->lcr | UART_LCR_DLAB);

	writew(0x01, vuart->regs + GXP_VUART_VBRD);
	gxp_serial_out(port, UART_LCR, up->lcr & UART_LCR_DLAB_MASK);
}

static int gxp_handle_irq(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	int ret;

	serial8250_rpm_get(up);

	ret = gxp_vuart_handle_irq(port);

	serial8250_rpm_put(up);
	return ret;
}

static int gxp_vuart_probe(struct platform_device *pdev)
{

	struct uart_8250_port port;
	struct gxp_vuart *vuart;
	struct device_node *np;
	struct resource *res;
	u32 clk, prop;
	int rc;

	np = pdev->dev.of_node;

	vuart = devm_kzalloc(&pdev->dev, sizeof(*vuart), GFP_KERNEL);
	if (!vuart)
		return -ENOMEM;

	vuart->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vuart->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vuart->regs))
		return PTR_ERR(vuart->regs);

	// Get regmap from syscon
	vuart->cfgReg = syscon_regmap_lookup_by_phandle(np, "vuart_cfg");
	if (IS_ERR(vuart->cfgReg)) {
		dev_err(vuart->dev, "Couldn't get VUART A configuration regmap\n");
		return -ENODEV;
	}

	memset(&port, 0, sizeof(port));
	port.port.private_data = vuart;
	port.port.membase = vuart->regs + GXP_16550A_BASE;
	port.port.mapbase = res->start + GXP_16550A_BASE;
	port.port.mapsize = resource_size(res);
	port.port.serial_in = gxp_serial_in;
	port.port.serial_out = gxp_serial_out;
	port.port.set_termios = gxp_8250_set_termios;
	port.port.set_divisor = gxp_8250_set_divisor;
	port.port.dev = &pdev->dev;

	rc = sysfs_create_group(&vuart->dev->kobj, &gxp_vuart_attr_group);
	if (rc < 0)
		return rc;

	if (of_property_read_u32(np, "clock-frequency", &clk)) {
		vuart->clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(vuart->clk)) {
			dev_warn(&pdev->dev,
				"clk or clock-frequency not defined\n");
			rc = PTR_ERR(vuart->clk);
			goto err_sysfs_remove;
		}

		rc = clk_prepare_enable(vuart->clk);
		if (rc < 0)
			goto err_sysfs_remove;

		clk = clk_get_rate(vuart->clk);
	}

	/* If current-speed was set, then try not to change it. */
	if (of_property_read_u32(np, "current-speed", &prop) == 0)
		port.port.custom_divisor = clk / (16 * prop);

	/* Check for shifted address mapping */
	if (of_property_read_u32(np, "reg-offset", &prop) == 0)
		port.port.mapbase += prop;

	/* Check for registers offset within the devices address range */
	if (of_property_read_u32(np, "reg-shift", &prop) == 0)
		port.port.regshift = prop;

	/* Check for fifo size */
	if (of_property_read_u32(np, "fifo-size", &prop) == 0)
		port.port.fifosize = prop;

	/* Check for a fixed line number */
	if (of_property_read_u32(np, "serial-line", &prop) == 0)
		port.port.line = prop;

	port.port.irq = irq_of_parse_and_map(np, 0);
	port.port.irqflags = IRQF_SHARED;
	port.port.handle_irq = gxp_handle_irq;
	port.port.iotype = UPIO_MEM;
	port.port.type = PORT_16550A;
	port.port.uartclk = clk;
	port.port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF
	  | UPF_FIXED_PORT | UPF_FIXED_TYPE;

	if (of_property_read_bool(np, "no-loopback-test"))
		port.port.flags |= UPF_SKIP_TEST;

	if (port.port.fifosize)
		port.capabilities = UART_CAP_FIFO;

	if (of_property_read_bool(np, "auto-flow-control"))
		port.capabilities |= UART_CAP_AFE;

	rc = serial8250_register_8250_port(&port);
	if (rc < 0) {
		pr_info("register 8250 port - %d\n", rc);
		goto err_clk_disable;
	}

	vuart->line = rc;
	gxp_vuart_set_enabled(vuart, true);
	platform_set_drvdata(pdev, vuart);
	return 0;

err_clk_disable:
	clk_disable_unprepare(vuart->clk);
	irq_dispose_mapping(port.port.irq);
err_sysfs_remove:
	sysfs_remove_group(&vuart->dev->kobj, &gxp_vuart_attr_group);
	return rc;
}

static int gxp_vuart_remove(struct platform_device *pdev)
{
	struct gxp_vuart *vuart = platform_get_drvdata(pdev);

	gxp_vuart_set_enabled(vuart, false);
	serial8250_unregister_port(vuart->line);
	sysfs_remove_group(&vuart->dev->kobj, &gxp_vuart_attr_group);
	clk_disable_unprepare(vuart->clk);

	return 0;
}

static const struct of_device_id gxp_vuart_table[] = {
	{ .compatible = "hpe,gxp-vuart" },
	{ },
};

static struct platform_driver gxp_vuart_driver = {
	.driver = {
	  .name = "gxp-vuart",
	  .of_match_table = gxp_vuart_table,
	},
	.probe = gxp_vuart_probe,
	.remove = gxp_vuart_remove,
};

module_platform_driver(gxp_vuart_driver);

MODULE_AUTHOR("John Chung <john.chung@hpe.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for HPE VUART device");
