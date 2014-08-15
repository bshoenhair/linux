/*
 * Copyright 2014 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>

#define IPROC_CLK_POLICY_FREQ_OFFSET    0x008
#define IPROC_CLK_POLICY0_MSK_OFFSET    0x010
#define IPROC_CLK_APB_SW_DIV_OFFSET     0xA10
#define IPROC_CLK_PLL_ARMA_OFFSET       0xC00
#define IPROC_CLK_PLL_ARMB_OFFSET       0xC04
#define IPROC_CLK_PLL_ARMC_OFFSET       0xC08
#define IPROC_CLK_PLL_ARMCTL5_OFFSET    0xC20
#define IPROC_CLK_PLL_ARM_OFFSET_OFFSET 0xC24
#define IPROC_CLK_ARM_DIV_OFFSET        0xE00
#define IPROC_CLK_POLICY_DBG_OFFSET     0xEC0

#define IPROC_CLK_ARM_DIV_ARM_PLL_SELECT_OVERRIDE_SHIFT        4
#define IPROC_CLK_ARM_DIV_ARM_PLL_SELECT_MASK                  0xf
#define IPROC_CLK_POLICY_FREQ_OFFSET_POLICY_FREQ_MASK          0xf
#define IPROC_CLK_POLICY_FREQ_OFFSET_POLICY_FREQ_SHIFT         8
#define IPROC_CLK_POLICY_DBG_OFFSET_ACT_FREQ_SHIFT             12
#define IPROC_CLK_POLICY_DBG_OFFSET_ACT_FREQ_MASK              7
#define IPROC_CLK_PLL_ARM_OFFSET_PLLARM_OFFSET_SW_CTL_SHIFT    29
#define CLK_PLL_ARM_OFFSET_PLLARM_NDIV_INT_OFFSET              20
#define CLK_PLL_ARM_OFFSET_PLLARM_NDIV_INT_MASK                0xff
#define CLK_PLL_ARM_OFFSET_PLLARM_NDIV_FRAC_OFFSET             0xfffff
#define CLK_PLL_ARMA_PLLARM_NDIV_INT_SHIFT                     8
#define CLK_PLL_ARMA_PLLARM_NDIV_INT_MASK                      0x3ff
#define CLK_PLL_ARMB_PLLARM_NDIV_FRAC_MASK                     0xfffff
#define CLK_PLL_ARMC_PLLARM_MDIV_MASK                          0xff
#define CLK_PLL_ARMCTL5_PLLARM_H_MDIV_MASK                     0xff
#define CLK_PLL_ARMC_PLLARM_BYPCLK_EN_SHIFT                    8
#define CLK_PLL_ARMA_PLLARM_PDIV_SHIFT                         24
#define CLK_PLL_ARMA_PLLARM_PDIV_MASK                          0xf
#define CLK_PLL_ARMA_PLLARM_LOCK_SHIFT                         28
#define CLK_ARM_DIV_APB0_FREE_DIV_SHIFT                        8
#define CLK_ARM_DIV_APB0_FREE_DIV_MASK                         0x7
#define CLK_ARM_DIV_ARM_SWITCH_DIV_SHIFT                       8
#define CLK_ARM_DIV_ARM_SWITCH_DIV_MASK                        0x3
#define CLK_APB_SW_DIV_APB_CLK_DIV_MASK                        0x3

struct brcm_clk {
	struct clk_hw   hw;
	void __iomem    *regs_base;
	int             chan;
	unsigned long   rate;
};

/* Identifies derived clocks from ARM PLL. */
enum {
	ARMPLL_APB0_FREE_CLK   = 0,
	ARMPLL_ARM_SWITCH_CLK  = 1,
	ARMPLL_ARM_APB_CLK     = 2,
	ARMPLL_ARM_PERIPH_CLK  = 3
};

/* Frequency id's from policy0_freq field of POLICY_FREQ register. */
enum a9pll_policy_freq {
	PLL_CRYSTAL_CLK   = 0,
	PLL_SYS_CLK       = 2,
	PLL_CH0_SLOW_CLK  = 6,
	PLL_CH1_FAST_CLK  = 7
};

#define to_brcm_clk(p) container_of(p, struct brcm_clk, hw)

static int iproc_cru_arm_freq_id(void __iomem *regs_base)
{
	u32 reg_f, reg;
	unsigned policy = 0;
	unsigned fid;
	unsigned active_freq;

	/* Read policy frequency. */
	reg_f = readl(regs_base + IPROC_CLK_POLICY_FREQ_OFFSET);

	/* Check for PLL policy software override. */
	reg = readl(regs_base + IPROC_CLK_ARM_DIV_OFFSET);
	if (reg & (1 << IPROC_CLK_ARM_DIV_ARM_PLL_SELECT_OVERRIDE_SHIFT))
		policy = reg & IPROC_CLK_ARM_DIV_ARM_PLL_SELECT_MASK;

	/* Get frequency ID based on policy. */
	fid = (reg_f >>
		(IPROC_CLK_POLICY_FREQ_OFFSET_POLICY_FREQ_SHIFT * policy)) &
		IPROC_CLK_POLICY_FREQ_OFFSET_POLICY_FREQ_MASK;

	/* Verify freq id from debug register. */
	reg = readl(regs_base + IPROC_CLK_POLICY_DBG_OFFSET);
	/* Read current active frequency id. */
	active_freq = IPROC_CLK_POLICY_DBG_OFFSET_ACT_FREQ_MASK &
		(reg >> IPROC_CLK_POLICY_DBG_OFFSET_ACT_FREQ_SHIFT);

	if (fid != active_freq) {
		pr_debug("IPROC CRU clock frequency id override %d->%d\n",
			fid, active_freq);

		fid = active_freq;
	}

	pr_debug("Active frequency ID %d\n", fid);

	return fid;
}

/*
 * Get ndiv integer and combine with fractional part to create 64 bit
 * value.
 */
static u64 a9pll_get_ndiv(struct brcm_clk *clk)
{
	u32 arm_offset_reg;
	u32 pllarma_reg;
	u32 pllarmb_reg;
	u32 ndiv_int;
	u32 ndiv_frac;
	u64 ndiv;

	arm_offset_reg = readl(clk->regs_base +
		IPROC_CLK_PLL_ARM_OFFSET_OFFSET);

    /*
	 * Check if offset mode is active to determine which register to
	 * get ndiv from.
	 */
	if (arm_offset_reg &
		(1 << IPROC_CLK_PLL_ARM_OFFSET_PLLARM_OFFSET_SW_CTL_SHIFT)) {
		/* Offset mode active. Get integer divide from offset reg. */
		ndiv_int = (arm_offset_reg >>
			CLK_PLL_ARM_OFFSET_PLLARM_NDIV_INT_OFFSET) &
			CLK_PLL_ARM_OFFSET_PLLARM_NDIV_INT_MASK;

		if (ndiv_int == 0)
			ndiv_int = 256;

		/* Get ndiv fractional divider. */
		ndiv_frac = arm_offset_reg &
			CLK_PLL_ARM_OFFSET_PLLARM_NDIV_FRAC_OFFSET;
	} else {
		/* Offset mode not active so read PLL ndiv from PLLARMA. */
		pllarma_reg = readl(clk->regs_base + IPROC_CLK_PLL_ARMA_OFFSET);
		ndiv_int = (pllarma_reg >> CLK_PLL_ARMA_PLLARM_NDIV_INT_SHIFT) &
			CLK_PLL_ARMA_PLLARM_NDIV_INT_MASK;

		if (ndiv_int == 0)
			ndiv_int = 1024;

		/* Get ndiv fractional divider. */
		pllarmb_reg = readl(clk->regs_base + IPROC_CLK_PLL_ARMB_OFFSET);
		ndiv_frac = pllarmb_reg & CLK_PLL_ARMB_PLLARM_NDIV_FRAC_MASK;
	}

	ndiv = ((u64) ndiv_int << 20) | ndiv_frac;

	return ndiv;
}

/*
 * Determine mdiv (post divider) based on the frequency id being used.
 * There are 4 clocks that can be used to derive the output clock rate:
 *    - 25 MHz crystal
 *    - sys_clk
 *    - channel 0 (slow clock)
 *    - channel 1 (fast clock)
 *
 * If the slow clock is being used then mdiv is read from PLLARMC. If
 * the fast clock is being used then the channel 1 mdiv is used.
 * Otherwise there is no post divider.
 *
 * @return The mdiv value. -EIO if an error occurred.
 */
static int a9pll_get_mdiv(struct brcm_clk *clk)
{
	u32 mdiv;
	u32 pllarmc_reg;
	u32 armctl5_reg;
	u32 freq_id;

	/* Get the policy frequency. */
	freq_id = iproc_cru_arm_freq_id(clk->regs_base);

	switch (freq_id) {
	/* There is no divider for these frequency id's. */
	case PLL_CRYSTAL_CLK:
	case PLL_SYS_CLK:
		mdiv = 1;
		break;

	case PLL_CH0_SLOW_CLK: {
	    /* Read mdiv (post-divider) from PLLARMC bits 0:7 */
	    pllarmc_reg = readl(clk->regs_base + IPROC_CLK_PLL_ARMC_OFFSET);
	    mdiv = pllarmc_reg & CLK_PLL_ARMC_PLLARM_MDIV_MASK;
	    if (mdiv == 0)
			mdiv = 256;
		break;
	}

	case PLL_CH1_FAST_CLK: {
		/* Post divider for channel 1 is in CTL5 (pllarm_h_mdiv). */
		armctl5_reg = readl(clk->regs_base +
			IPROC_CLK_PLL_ARMCTL5_OFFSET);
	    mdiv = armctl5_reg & CLK_PLL_ARMCTL5_PLLARM_H_MDIV_MASK;
	    if (mdiv == 0)
			mdiv = 256;
		break;
	}

	default:
		return -EIO;
	}

	return mdiv;
}

/*
 * Calculate the output frequency of the ARM PLL. The main output clock
 * is 'arm_clk'.
 *
 * The frequency is calculated based on the ARM PLL divider values:
 *	 pdiv = ARM PLL input pre-divider
 *   ndiv = ARM PLL feedback divider
 *   mdiv = ARM PLL post divider
 *
 * The frequency is calculated by:
 *   ((ndiv * parent clock rate) / pdiv) / mdiv
 */
static int a9pll_status(struct brcm_clk *clk, unsigned long parent_rate)
{
	u32 pllarma_reg;
	u32 pllarmc_reg;
	u32 pdiv;
	u32 mdiv;
	u64 ndiv;
	u32 arm_clk_freq;

	pr_debug("a9pll_status: clk 0x%x\n", (unsigned int)clk);

	BUG_ON(!clk->regs_base);

	pllarma_reg = readl(clk->regs_base + IPROC_CLK_PLL_ARMA_OFFSET);
	pllarmc_reg = readl(clk->regs_base + IPROC_CLK_PLL_ARMC_OFFSET);

	/* Check if PLL is in bypass mode - input frequency to output */
	if (pllarmc_reg & (1 << CLK_PLL_ARMC_PLLARM_BYPCLK_EN_SHIFT)) {
		clk->rate = parent_rate;
		return 0;
	}

	/* Check if PLL is locked. It must be unlocked. */
	if ((pllarma_reg &
		(1 << CLK_PLL_ARMA_PLLARM_LOCK_SHIFT)) == 0) {
		clk->rate = 0;
		return -EIO;
	}

	/* Read pdiv from PLLARMA. */
	pdiv = (pllarma_reg >> CLK_PLL_ARMA_PLLARM_PDIV_SHIFT) &
		CLK_PLL_ARMA_PLLARM_PDIV_MASK;
	if (pdiv == 0)
		pdiv = 16;

	/* Determine ndiv. */
	ndiv = a9pll_get_ndiv(clk);

	/* Determine mdiv (post divider). */
	mdiv = a9pll_get_mdiv(clk);
	if (mdiv == -EIO) {
		clk->rate = 0;
		return -EIO;
	}

	/* Calculate clock frequency. */
	arm_clk_freq = (ndiv * parent_rate) >> 20;
	arm_clk_freq = (arm_clk_freq / pdiv) / mdiv;

	clk->rate = arm_clk_freq;

	pr_debug("ARM PLL (arm_clk) rate %lu. parent rate = %lu, ",
		clk->rate, parent_rate);
	pr_debug("ndiv_int = %d, pdiv = %d, mdiv = %d\n",
		 (u32)ndiv >> 20, pdiv, mdiv);

	return clk->rate;
}

static unsigned long clk_a9pll_recalc_rate(struct clk_hw *hwclk,
	unsigned long parent_rate)
{
	struct brcm_clk *bcm_clk = to_brcm_clk(hwclk);

	return a9pll_status(bcm_clk, parent_rate);
}

static const struct clk_ops a9pll_ops = {
	.recalc_rate = clk_a9pll_recalc_rate,
};

/*
 * Get status of any of the ARMPLL output channels
 */
static int a9pll_chan_status(struct brcm_clk *clk, unsigned long parent_rate)
{
	u32 reg;
	unsigned div;

	BUG_ON(!clk->regs_base);

	reg = readl(clk->regs_base + IPROC_CLK_ARM_DIV_OFFSET);
	pr_debug("Clock Div = %#x\n", reg);

	switch (clk->chan) {
	case ARMPLL_APB0_FREE_CLK:
		/* apb0_free_div bits 10:8 */
		div = (reg >> CLK_ARM_DIV_APB0_FREE_DIV_SHIFT) &
			CLK_ARM_DIV_APB0_FREE_DIV_MASK;
		div++;
		break;

	case ARMPLL_ARM_SWITCH_CLK:
		/* arm_switch_div bits 6:5 */
		div = (reg >> CLK_ARM_DIV_ARM_SWITCH_DIV_SHIFT) &
			CLK_ARM_DIV_ARM_SWITCH_DIV_MASK;
		div++;
		break;

	case ARMPLL_ARM_APB_CLK:
		/* IPROC_CLK_APB_SW_DIV_REG apb_clk_div bits 1:0 */
		reg = readl(clk->regs_base + IPROC_CLK_APB_SW_DIV_OFFSET);
		div = reg & CLK_APB_SW_DIV_APB_CLK_DIV_MASK;
		div++;
		break;

	case ARMPLL_ARM_PERIPH_CLK:      /* periph_clk */
		div = 2;
		break;

	default:
		return -EINVAL;
	}

	clk->rate = parent_rate / div;
	pr_debug("Clock rate A9PLL chan 0x%x: %lu, div: %d\n",
		clk->chan, clk->rate, div);

	return clk->rate;
}

static unsigned long clk_a9pll_chan_recalc_rate(struct clk_hw *hwclk,
	unsigned long parent_rate)
{
	struct brcm_clk *bcm_clk = to_brcm_clk(hwclk);

	return a9pll_chan_status(bcm_clk, parent_rate);
}

static const struct clk_ops a9pll_chan_ops = {
	.recalc_rate = clk_a9pll_chan_recalc_rate,
};

static __init struct clk *iproc_clock_init(struct device_node *node,
	const struct clk_ops *ops)
{
	u32 channel = 0;
	struct clk *clk;
	struct brcm_clk *brcm_clk;
	const char *clk_name = node->name;
	const char *parent_name;
	struct clk_init_data init;
	int rc;

	pr_debug("Clock name %s\n", node->name);

	rc = of_property_read_u32(node, "channel", &channel);
	brcm_clk = kzalloc(sizeof(*brcm_clk), GFP_KERNEL);
	if (WARN_ON(!brcm_clk))
		return NULL;

	/* Read base address from device tree and map to virtual address. */
	brcm_clk->regs_base = of_iomap(node, 0);
	if (WARN_ON(!brcm_clk->regs_base))
		goto err_alloc;

	brcm_clk->chan = channel;
	of_property_read_string(node, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = ops;
	init.flags = 0;
	parent_name = of_clk_get_parent_name(node, 0);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	brcm_clk->hw.init = &init;

	clk = clk_register(NULL, &brcm_clk->hw);
	if (WARN_ON(IS_ERR(clk)))
		goto err_unmap;

	rc = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (WARN_ON(IS_ERR_VALUE(rc)))
		goto err_unregister;

	rc = clk_register_clkdev(clk, clk_name, NULL);
	if (WARN_ON(IS_ERR_VALUE(rc)))
		goto err_provider;

	return clk;

err_provider:
	of_clk_del_provider(node);

err_unregister:
	clk_unregister(clk);

err_unmap:
	iounmap(brcm_clk->regs_base);

err_alloc:
	kfree(brcm_clk);

	return NULL;
}

static void __init iproc_armpll_init(struct device_node *node)
{
	iproc_clock_init(node, &a9pll_ops);
}
CLK_OF_DECLARE(iproc_armpllx, "brcm,iproc-arm-a9pll", iproc_armpll_init);

static void __init iproc_arm_ch_init(struct device_node *node)
{
	iproc_clock_init(node, &a9pll_chan_ops);
}
CLK_OF_DECLARE(iproc_arm_ch, "brcm,iproc-arm-ch", iproc_arm_ch_init);
