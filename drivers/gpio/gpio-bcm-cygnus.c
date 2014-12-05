/*
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/ioport.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>

#define CYGNUS_GPIO_DATA_IN_OFFSET   0x00
#define CYGNUS_GPIO_DATA_OUT_OFFSET  0x04
#define CYGNUS_GPIO_OUT_EN_OFFSET    0x08
#define CYGNUS_GPIO_IN_TYPE_OFFSET   0x0c
#define CYGNUS_GPIO_INT_DE_OFFSET    0x10
#define CYGNUS_GPIO_INT_EDGE_OFFSET  0x14
#define CYGNUS_GPIO_INT_MSK_OFFSET   0x18
#define CYGNUS_GPIO_INT_STAT_OFFSET  0x1c
#define CYGNUS_GPIO_INT_MSTAT_OFFSET 0x20
#define CYGNUS_GPIO_INT_CLR_OFFSET   0x24
#define CYGNUS_GPIO_PAD_RES_OFFSET   0x34
#define CYGNUS_GPIO_RES_EN_OFFSET    0x38

/* drive strength control for ASIU GPIO */
#define CYGNUS_GPIO_ASIU_DRV0_CTRL_OFFSET 0x58

/* drive strength control for CCM GPIO */
#define CYGNUS_GPIO_CCM_DRV0_CTRL_OFFSET  0x00

#define GPIO_BANK_SIZE 0x200
#define NGPIOS_PER_BANK 32
#define GPIO_BIT(pin) ((pin) % NGPIOS_PER_BANK)
#define GPIO_BANK(pin) ((pin) / NGPIOS_PER_BANK)

#define GPIO_FLAG_BIT_MASK           0xffff
#define GPIO_PULL_BIT_SHIFT          16
#define GPIO_PULL_BIT_MASK           0x3

#define GPIO_DRV_STRENGTH_BIT_SHIFT  20
#define GPIO_DRV_STRENGTH_BITS       3
#define GPIO_DRV_STRENGTH_BIT_MASK   ((1 << GPIO_DRV_STRENGTH_BITS) - 1)

/*
 * For GPIO internal pull up/down registers
 */
enum gpio_pull {
	GPIO_PULL_NONE = 0,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN,
	GPIO_PULL_INVALID,
};

/*
 * GPIO drive strength
 */
enum gpio_drv_strength {
	GPIO_DRV_STRENGTH_2MA = 0,
	GPIO_DRV_STRENGTH_4MA,
	GPIO_DRV_STRENGTH_6MA,
	GPIO_DRV_STRENGTH_8MA,
	GPIO_DRV_STRENGTH_10MA,
	GPIO_DRV_STRENGTH_12MA,
	GPIO_DRV_STRENGTH_14MA,
	GPIO_DRV_STRENGTH_16MA,
	GPIO_DRV_STRENGTH_INVALID,
};

struct bcm_cygnus_gpio {
	struct device *dev;
	void __iomem *base;
	void __iomem *io_ctrl;
	spinlock_t lock;
	struct gpio_chip gc;
	unsigned num_banks;
	int irq;
	struct irq_domain *irq_domain;
};

static unsigned int gpio_base_index;

static struct bcm_cygnus_gpio *to_bcm_cygnus_gpio(struct gpio_chip *gc)
{
	return container_of(gc, struct bcm_cygnus_gpio, gc);
}

static int bcm_cygnus_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct bcm_cygnus_gpio *cygnus_gpio = to_bcm_cygnus_gpio(gc);

	return irq_find_mapping(cygnus_gpio->irq_domain, offset);
}

static unsigned int __gpio_reg_offset(struct bcm_cygnus_gpio *cygnus_gpio,
		unsigned gpio)
{
	return GPIO_BANK(gpio) * GPIO_BANK_SIZE;
}

static unsigned int __gpio_bitpos(struct bcm_cygnus_gpio *cygnus_gpio,
		unsigned gpio)
{
	return GPIO_BIT(gpio);
}

static void bcm_cygnus_gpio_irq_handler(unsigned int irq,
		struct irq_desc *desc)
{
	struct bcm_cygnus_gpio *cygnus_gpio;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int i, bit;

	chained_irq_enter(chip, desc);

	cygnus_gpio = irq_get_handler_data(irq);

	/* go through the entire GPIO banks and handle all interrupts */
	for (i = 0; i < cygnus_gpio->num_banks; i++) {
		unsigned long val = readl(cygnus_gpio->base +
				(i * GPIO_BANK_SIZE) +
				CYGNUS_GPIO_INT_MSTAT_OFFSET);

		for_each_set_bit(bit, &val, NGPIOS_PER_BANK) {
			unsigned pin = NGPIOS_PER_BANK * i + bit;
			int child_irq =
				bcm_cygnus_gpio_to_irq(&cygnus_gpio->gc, pin);

			/*
			 * Clear the interrupt before invoking the
			 * handler, so we do not leave any window
			 */
			writel(1 << bit,
				cygnus_gpio->base + (i * GPIO_BANK_SIZE) +
				CYGNUS_GPIO_INT_CLR_OFFSET);

			generic_handle_irq(child_irq);
		}
	}

	chained_irq_exit(chip, desc);
}

static void bcm_cygnus_gpio_irq_ack(struct irq_data *d)
{
	struct bcm_cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	unsigned int offset, shift;
	u32 val;

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_INT_CLR_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	val = 1 << shift;
	writel(val, cygnus_gpio->base + offset);

	dev_dbg(cygnus_gpio->dev, "gpio:%u offset:0x%04x shift:%u\n", gpio,
			offset, shift);
}

static void bcm_cygnus_gpio_irq_mask(struct irq_data *d)
{
	struct bcm_cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	unsigned int offset, shift;
	u32 val;
	unsigned long flags;

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_INT_MSK_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	val = readl(cygnus_gpio->base + offset);
	val &= ~(1 << shift);
	writel(val, cygnus_gpio->base + offset);

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev, "gpio:%u offset:0x%04x shift:%u\n", gpio,
			offset, shift);
}

static void bcm_cygnus_gpio_irq_unmask(struct irq_data *d)
{
	struct bcm_cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	unsigned int offset, shift;
	u32 val;
	unsigned long flags;

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_INT_MSK_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	val = readl(cygnus_gpio->base + offset);
	val |= 1 << shift;
	writel(val, cygnus_gpio->base + offset);

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev, "gpio:%u offset:0x%04x shift:%u\n", gpio,
			offset, shift);
}

static int bcm_cygnus_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct bcm_cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	unsigned int int_type, dual_edge, edge_lvl;
	unsigned int offset, shift;
	u32 val;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		int_type = 0;
		dual_edge = 0;
		edge_lvl = 1;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		int_type = 0;
		dual_edge = 0;
		edge_lvl = 0;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		int_type = 0;
		dual_edge = 1;
		edge_lvl = 0;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		int_type = 1;
		dual_edge = 0;
		edge_lvl = 1;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		int_type = 1;
		dual_edge = 0;
		edge_lvl = 0;
		break;

	default:
		dev_err(cygnus_gpio->dev, "invalid GPIO irq type 0x%x\n", type);
		return -EINVAL;
	}

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	shift = __gpio_bitpos(cygnus_gpio, gpio);
	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_IN_TYPE_OFFSET;
	val = readl(cygnus_gpio->base + offset);
	val &= ~(1 << shift);
	val |= int_type << shift;
	writel(val, cygnus_gpio->base + offset);

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_INT_DE_OFFSET;
	val = readl(cygnus_gpio->base + offset);
	val &= ~(1 << shift);
	val |= dual_edge << shift;
	writel(val, cygnus_gpio->base + offset);

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_INT_EDGE_OFFSET;
	val = readl(cygnus_gpio->base + offset);
	val &= ~(1 << shift);
	val |= edge_lvl << shift;
	writel(val, cygnus_gpio->base + offset);

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	return 0;
}

static struct irq_chip bcm_cygnus_gpio_irq_chip = {
	.name = "bcm-cygnus-gpio",
	.irq_ack = bcm_cygnus_gpio_irq_ack,
	.irq_mask = bcm_cygnus_gpio_irq_mask,
	.irq_unmask = bcm_cygnus_gpio_irq_unmask,
	.irq_set_type = bcm_cygnus_gpio_irq_set_type,
};

static int bcm_cygnus_gpio_direction_input(struct gpio_chip *gc,
		unsigned gpio)
{
	struct bcm_cygnus_gpio *cygnus_gpio = to_bcm_cygnus_gpio(gc);
	unsigned int offset, shift;
	u32 val;
	unsigned long flags;

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_OUT_EN_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	val = readl(cygnus_gpio->base + offset);
	val &= ~(1 << shift);
	writel(val, cygnus_gpio->base + offset);

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev, "gpio:%u offset:0x%04x shift:%u\n", gpio,
			offset, shift);

	return 0;
}

static int bcm_cygnus_gpio_direction_output(struct gpio_chip *gc,
		unsigned gpio, int value)
{
	struct bcm_cygnus_gpio *cygnus_gpio = to_bcm_cygnus_gpio(gc);
	unsigned int offset, shift;
	u32 val;
	unsigned long flags;

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_OUT_EN_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	val = readl(cygnus_gpio->base + offset);
	val |= 1 << shift;
	writel(val, cygnus_gpio->base + offset);

	dev_dbg(cygnus_gpio->dev, "gpio:%u offset:0x%04x shift:%u\n", gpio,
			offset, shift);

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_DATA_OUT_OFFSET;

	val = readl(cygnus_gpio->base + offset);
	if (value)
		val |= 1 << shift;
	else
		val &= ~(1 << shift);
	writel(val, cygnus_gpio->base + offset);

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev,
		"gpio:%u offset:0x%04x shift:%u val:0x%08x\n",
		gpio, offset, shift, val);

	return 0;
}

static void bcm_cygnus_gpio_set(struct gpio_chip *gc, unsigned gpio,
		int value)
{
	struct bcm_cygnus_gpio *cygnus_gpio = to_bcm_cygnus_gpio(gc);
	unsigned int offset, shift;
	u32 val;
	unsigned long flags;

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_DATA_OUT_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	val = readl(cygnus_gpio->base + offset);
	if (value)
		val |= 1 << shift;
	else
		val &= ~(1 << shift);
	writel(val, cygnus_gpio->base + offset);

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev,
		"gpio:%u offset:0x%04x shift:%u val:0x%08x\n",
		gpio, offset, shift, val);
}

static int bcm_cygnus_gpio_get(struct gpio_chip *gc, unsigned gpio)
{
	struct bcm_cygnus_gpio *cygnus_gpio = to_bcm_cygnus_gpio(gc);
	unsigned int offset, shift;
	u32 val;

	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_DATA_IN_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	val = readl(cygnus_gpio->base + offset);
	val = (val >> shift) & 1;

	dev_dbg(cygnus_gpio->dev, "gpio:%u offset:0x%04x shift:%u val:%u\n",
			gpio, offset, shift, val);

	return val;
}

static struct lock_class_key gpio_lock_class;

static int bcm_cygnus_gpio_irq_map(struct irq_domain *d, unsigned int irq,
				 irq_hw_number_t hwirq)
{
	int ret;

	ret = irq_set_chip_data(irq, d->host_data);
	if (ret < 0)
		return ret;
	irq_set_lockdep_class(irq, &gpio_lock_class);
	irq_set_chip_and_handler(irq, &bcm_cygnus_gpio_irq_chip,
			handle_simple_irq);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static void bcm_cygnus_gpio_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static struct irq_domain_ops bcm_cygnus_irq_ops = {
	.map = bcm_cygnus_gpio_irq_map,
	.unmap = bcm_cygnus_gpio_irq_unmap,
	.xlate = irq_domain_xlate_twocell,
};

#ifdef CONFIG_OF_GPIO
static void bcm_cygnus_gpio_set_pull(struct bcm_cygnus_gpio *cygnus_gpio,
		unsigned gpio, enum gpio_pull pull)
{
	unsigned int offset, shift;
	u32 val, pullup;
	unsigned long flags;

	switch (pull) {
	case GPIO_PULL_UP:
		pullup = 1;
		break;
	case GPIO_PULL_DOWN:
		pullup = 0;
		break;
	case GPIO_PULL_NONE:
	case GPIO_PULL_INVALID:
	default:
		return;
	}

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	/* set pull up/down */
	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_PAD_RES_OFFSET;
	shift = __gpio_bitpos(cygnus_gpio, gpio);

	val = readl(cygnus_gpio->base + offset);
	val &= ~(1 << shift);
	if (pullup)
		val |= 1 << shift;
	writel(val, cygnus_gpio->base + offset);

	/* enable pad */
	offset = __gpio_reg_offset(cygnus_gpio, gpio) +
		CYGNUS_GPIO_RES_EN_OFFSET;
	val = readl(cygnus_gpio->base + offset);
	val |= 1 << shift;
	writel(val, cygnus_gpio->base + offset);

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);
}

static void bcm_cygnus_gpio_set_strength(struct bcm_cygnus_gpio *cygnus_gpio,
		unsigned gpio, enum gpio_drv_strength strength)
{
	struct device *dev = cygnus_gpio->dev;
	void __iomem *base;
	unsigned int i, offset, shift;
	u32 val;
	unsigned long flags;

	/* some GPIO controllers do not support drive strength configuration */
	if (of_find_property(dev->of_node, "no-drv-strength", NULL))
		return;

	if (cygnus_gpio->io_ctrl) {
		base = cygnus_gpio->io_ctrl;
		offset = CYGNUS_GPIO_CCM_DRV0_CTRL_OFFSET;
	} else {
		base = cygnus_gpio->base;
		offset = __gpio_reg_offset(cygnus_gpio, gpio) +
			CYGNUS_GPIO_ASIU_DRV0_CTRL_OFFSET;
	}

	shift = __gpio_bitpos(cygnus_gpio, gpio);

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	for (i = 0; i < GPIO_DRV_STRENGTH_BITS; i++) {
		val = readl(base + offset);
		val &= ~(1 << shift);
		val |= ((strength >> i) & 0x1) << shift;
		writel(val, base + offset);
		offset += 4;
	}

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);
}

static int bcm_cygnus_gpio_of_xlate(struct gpio_chip *gc,
		const struct of_phandle_args *gpiospec, u32 *flags)
{
	struct bcm_cygnus_gpio *cygnus_gpio = to_bcm_cygnus_gpio(gc);
	enum gpio_pull pull;
	enum gpio_drv_strength strength;

	if (gc->of_gpio_n_cells < 2)
		return -EINVAL;

	if (WARN_ON(gpiospec->args_count < gc->of_gpio_n_cells))
		return -EINVAL;

	if (gpiospec->args[0] >= gc->ngpio)
		return -EINVAL;

	pull = (gpiospec->args[1] >> GPIO_PULL_BIT_SHIFT) & GPIO_PULL_BIT_MASK;
	if (WARN_ON(pull >= GPIO_PULL_INVALID))
		return -EINVAL;

	strength = (gpiospec->args[1] >> GPIO_DRV_STRENGTH_BIT_SHIFT) &
		GPIO_DRV_STRENGTH_BIT_MASK;

	if (flags)
		*flags = gpiospec->args[1] & GPIO_FLAG_BIT_MASK;

	bcm_cygnus_gpio_set_pull(cygnus_gpio, gpiospec->args[0], pull);
	bcm_cygnus_gpio_set_strength(cygnus_gpio, gpiospec->args[0], strength);

	return gpiospec->args[0];
}
#endif

static const struct of_device_id bcm_cygnus_gpio_of_match[] = {
	{ .compatible = "brcm,cygnus-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_cygnus_gpio_of_match);

static int bcm_cygnus_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct resource *res;
	struct bcm_cygnus_gpio *cygnus_gpio;
	struct gpio_chip *gc;
	u32 i, ngpios;
	int ret;

	match = of_match_device(bcm_cygnus_gpio_of_match, dev);
	if (!match) {
		dev_err(&pdev->dev, "failed to find GPIO controller\n");
		return -ENODEV;
	}

	cygnus_gpio = devm_kzalloc(dev, sizeof(*cygnus_gpio), GFP_KERNEL);
	if (!cygnus_gpio)
		return -ENOMEM;

	cygnus_gpio->dev = dev;
	platform_set_drvdata(pdev, cygnus_gpio);

	if (of_property_read_u32(dev->of_node, "ngpios", &ngpios)) {
		dev_err(&pdev->dev, "missing ngpios device tree property\n");
		return -ENODEV;
	}
	cygnus_gpio->num_banks = (ngpios + NGPIOS_PER_BANK - 1) /
		NGPIOS_PER_BANK;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "unable to get I/O resource\n");
		return -ENODEV;
	}

	cygnus_gpio->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cygnus_gpio->base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		return PTR_ERR(cygnus_gpio->base);
	}

	/*
	 * Only certain types of Cygnus GPIO interfaces have I/O control
	 * registers
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		cygnus_gpio->io_ctrl = devm_ioremap_resource(dev, res);
		if (IS_ERR(cygnus_gpio->io_ctrl)) {
			dev_err(&pdev->dev, "unable to map I/O memory\n");
			return PTR_ERR(cygnus_gpio->io_ctrl);
		}
	}

	spin_lock_init(&cygnus_gpio->lock);

	gc = &cygnus_gpio->gc;
	gc->base = gpio_base_index;
	gpio_base_index += ngpios;
	gc->ngpio = ngpios;
	gc->label = dev_name(dev);
	gc->dev = dev;
#ifdef CONFIG_OF_GPIO
	gc->of_node = dev->of_node;
	gc->of_gpio_n_cells = 2;
	gc->of_xlate = bcm_cygnus_gpio_of_xlate;
#endif
	gc->direction_input = bcm_cygnus_gpio_direction_input;
	gc->direction_output = bcm_cygnus_gpio_direction_output;
	gc->set = bcm_cygnus_gpio_set;
	gc->get = bcm_cygnus_gpio_get;
	gc->to_irq = bcm_cygnus_gpio_to_irq;

	ret = gpiochip_add(gc);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to add GPIO chip\n");
		goto err_dec_gpio_base;
	}

	/*
	 * Some of the GPIO interfaces do not have interrupt wired to the main
	 * processor
	 */
	cygnus_gpio->irq = platform_get_irq(pdev, 0);
	if (cygnus_gpio->irq < 0) {
		ret = cygnus_gpio->irq;
		if (ret == -EPROBE_DEFER)
			goto err_rm_gpiochip;

		dev_info(&pdev->dev, "no interrupt hook\n");
	}

	cygnus_gpio->irq_domain = irq_domain_add_linear(dev->of_node,
			gc->ngpio, &bcm_cygnus_irq_ops, cygnus_gpio);
	if (!cygnus_gpio->irq_domain) {
		dev_err(&pdev->dev, "unable to allocate IRQ domain\n");
		ret = -ENXIO;
		goto err_rm_gpiochip;
	}

	for (i = 0; i < gc->ngpio; i++) {
		int irq = irq_create_mapping(cygnus_gpio->irq_domain, i);

		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_data(irq, cygnus_gpio);
		irq_set_chip_and_handler(irq, &bcm_cygnus_gpio_irq_chip,
				handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	irq_set_chained_handler(cygnus_gpio->irq, bcm_cygnus_gpio_irq_handler);
	irq_set_handler_data(cygnus_gpio->irq, cygnus_gpio);

	return 0;

err_rm_gpiochip:
	gpiochip_remove(gc);

err_dec_gpio_base:
	gpio_base_index -= ngpios;
	return ret;
}

static struct platform_driver bcm_cygnus_gpio_driver = {
	.driver = {
		.name = "bcm-cygnus-gpio",
		.owner = THIS_MODULE,
		.of_match_table = bcm_cygnus_gpio_of_match,
	},
	.probe = bcm_cygnus_gpio_probe,
};

module_platform_driver(bcm_cygnus_gpio_driver);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom Cygnus GPIO Driver");
MODULE_LICENSE("GPL v2");
