/* rk3288_lcdc.c
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:
 *      hjc <hjc@rock-chips.com>
 *      mark yao <mark.yao@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <drm/rockchip_drm.h>
#include <drm/drm_crtc.h>
#include <drm/drm_modes.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include "../rockchip_drm_lcdc.h"
#include "rk3288_lcdc.h"

static struct lcdc_win_data lcdc_win[] = {
	[0] = {
		.id = 0,
		.zpos = ZPOS_DEFAULT_WIN,
		.fmt_10 = 0,
		.win_lb_mode = 0x4,
		.swap_rb = 0,
	},
	[1] = {
		.id = 1,
		.zpos = ZPOS_UNUSED_WIN,
		.fmt_10 = 0,
		.win_lb_mode = 0x4,
		.swap_rb = 0,
	},
	[2] = {
		.id = 2,
		.zpos = ZPOS_CURSOR_WIN,
		.fmt_10 = 0,
		.win_lb_mode = 0x4,
		.swap_rb = 0,
	},
	[3] = {
		.id = 3,
		.zpos = ZPOS_UNUSED_WIN,
	},
};

static int rk3288_lcdc_get_id(u32 phy_base)
{
	/*vop big*/
	if (phy_base == 0xff930000)
		return 0;
	/*vop lit*/
	else if (phy_base == 0xff940000)
		return 1;
	else
		return -EINVAL;
}

static int rk3288_lcdc_clk_enable(struct lcdc_device *lcdc_dev)
{
	int ret = 0;

	if (!lcdc_dev->clk_on) {
		ret = clk_prepare_enable(lcdc_dev->hclk);
		if (ret < 0) {
			dev_err(lcdc_dev->dev, "failed to enable hclk\n");
			return ret;
		}

		ret = clk_prepare_enable(lcdc_dev->dclk);
		if (ret < 0) {
			dev_err(lcdc_dev->dev, "failed to enable dclk\n");
			goto err_dclk;
		}

		clk_prepare_enable(lcdc_dev->aclk);
		if (ret < 0) {
			dev_err(lcdc_dev->dev, "failed to enable aclk\n");
			goto err_aclk;
		}

		spin_lock(&lcdc_dev->reg_lock);
		lcdc_dev->clk_on = 1;
		spin_unlock(&lcdc_dev->reg_lock);
	}

	return ret;
err_aclk:
	clk_disable_unprepare(lcdc_dev->aclk);
err_dclk:
	clk_disable_unprepare(lcdc_dev->hclk);
	return ret;
}

static void rk3288_lcdc_clk_disable(struct lcdc_device *lcdc_dev)
{
	if (lcdc_dev->clk_on) {
		spin_lock(&lcdc_dev->reg_lock);
		lcdc_dev->clk_on = 0;
		spin_unlock(&lcdc_dev->reg_lock);
		mdelay(25);
		clk_disable_unprepare(lcdc_dev->dclk);
		clk_disable_unprepare(lcdc_dev->hclk);
		clk_disable_unprepare(lcdc_dev->aclk);
	}
}

static void rk3288_lcdc_disable_irq(struct lcdc_device *lcdc_dev)
{
	u32 mask, val;

	if (likely(lcdc_dev->clk_on)) {
		spin_lock(&lcdc_dev->reg_lock);
		mask = m_DSP_HOLD_VALID_INTR_EN | m_FS_INTR_EN |
			m_LINE_FLAG_INTR_EN | m_BUS_ERROR_INTR_EN;
		val = v_DSP_HOLD_VALID_INTR_EN(0) | v_FS_INTR_EN(0) |
			v_LINE_FLAG_INTR_EN(0) | v_BUS_ERROR_INTR_EN(0);
		lcdc_msk_reg(lcdc_dev, INTR_CTRL0, mask, val);

		mask = m_DSP_HOLD_VALID_INTR_CLR | m_FS_INTR_CLR |
			m_LINE_FLAG_INTR_CLR | m_LINE_FLAG_INTR_CLR;
		val = v_DSP_HOLD_VALID_INTR_CLR(0) | v_FS_INTR_CLR(0) |
			v_LINE_FLAG_INTR_CLR(0) | v_BUS_ERROR_INTR_CLR(0);
		lcdc_msk_reg(lcdc_dev, INTR_CTRL0, mask, val);

		mask = m_WIN0_EMPTY_INTR_EN | m_WIN1_EMPTY_INTR_EN |
			m_WIN2_EMPTY_INTR_EN | m_WIN3_EMPTY_INTR_EN |
			m_HWC_EMPTY_INTR_EN | m_POST_BUF_EMPTY_INTR_EN |
			m_POST_BUF_EMPTY_INTR_EN;
		val = v_WIN0_EMPTY_INTR_EN(0) | v_WIN1_EMPTY_INTR_EN(0) |
			v_WIN2_EMPTY_INTR_EN(0) | v_WIN3_EMPTY_INTR_EN(0) |
			v_HWC_EMPTY_INTR_EN(0) | v_POST_BUF_EMPTY_INTR_EN(0) |
			v_PWM_GEN_INTR_EN(0);
		lcdc_msk_reg(lcdc_dev, INTR_CTRL1, mask, val);

		mask = m_WIN0_EMPTY_INTR_CLR | m_WIN1_EMPTY_INTR_CLR |
			m_WIN2_EMPTY_INTR_CLR | m_WIN3_EMPTY_INTR_CLR |
			m_HWC_EMPTY_INTR_CLR | m_POST_BUF_EMPTY_INTR_CLR |
			m_POST_BUF_EMPTY_INTR_CLR;
		val = v_WIN0_EMPTY_INTR_CLR(0) | v_WIN1_EMPTY_INTR_CLR(0) |
			v_WIN2_EMPTY_INTR_CLR(0) | v_WIN3_EMPTY_INTR_CLR(0) |
			v_HWC_EMPTY_INTR_CLR(0) |
			v_POST_BUF_EMPTY_INTR_CLR(0) |
			v_PWM_GEN_INTR_CLR(0);
		lcdc_msk_reg(lcdc_dev, INTR_CTRL1, mask, val);
		lcdc_cfg_done(lcdc_dev);
		spin_unlock(&lcdc_dev->reg_lock);
	}
	mdelay(1);
}

static void rk3288_lcdc_enable_irq(struct lcdc_device *lcdc_dev)
{
	u32 mask, val;

	if (likely(lcdc_dev->clk_on)) {
		spin_lock(&lcdc_dev->reg_lock);
		mask = m_FS_INTR_CLR | m_FS_INTR_EN;
		val = v_FS_INTR_CLR(1) | v_FS_INTR_EN(1);
		lcdc_msk_reg(lcdc_dev, INTR_CTRL0, mask, val);
		spin_unlock(&lcdc_dev->reg_lock);
	}
}

static irqreturn_t rk3288_lcdc_isr(int irq, void *dev_id)
{
	struct lcdc_device *lcdc_dev =
			(struct lcdc_device *)dev_id;
	u32 intr0_reg;

	intr0_reg = lcdc_readl(lcdc_dev, INTR_CTRL0);
	if (intr0_reg & m_FS_INTR_STS) {
		lcdc_msk_reg(lcdc_dev, INTR_CTRL0, m_FS_INTR_CLR,
			     v_FS_INTR_CLR(1));
		lcdc_vsync_event_handler(lcdc_dev->dev);
	}

	return IRQ_HANDLED;
}

static struct pwr_ctr *
	rk3288_lcdc_parse_pwr(struct device *dev, struct device_node *node)
{
	enum of_gpio_flags flags;
	struct pwr_ctr *pwr_ctr;
	u32 val;

	pwr_ctr = kmalloc(sizeof(*pwr_ctr), GFP_KERNEL);
	if (!pwr_ctr) {
		dev_err(dev, "can't malloc for pwr_ctr, fail\n");
		return NULL;
	}
	strcpy(pwr_ctr->name, node->name);
	if (!of_property_read_u32(node, "rockchip,power_type", &val)) {
		if (val == GPIO) {
			pwr_ctr->type = GPIO;
			pwr_ctr->gpio = of_get_gpio_flags(node, 0, &flags);

			if (!gpio_is_valid(pwr_ctr->gpio)) {
				dev_err(dev, "%s[%d] is invalid gpio\n",
					node->name, pwr_ctr->gpio);
				goto err_free_pwr_ctr;
			}

			pwr_ctr->atv_val = !(flags & OF_GPIO_ACTIVE_LOW);
			if (gpio_request(pwr_ctr->gpio, node->name)) {
				dev_err(dev, "%s[%d] requeset fail\n",
					node->name, pwr_ctr->gpio);
				goto err_free_pwr_ctr;
			}
		} else {
			pwr_ctr->type = REGULATOR;
		}
	}
	if (!of_property_read_u32(node, "rockchip,delay", &val))
		pwr_ctr->delay = val;
	else
		pwr_ctr->delay = 0;

	return pwr_ctr;

err_free_pwr_ctr:
	kfree(pwr_ctr);
	return NULL;
}

static void rk3288_lcdc_parse_dt(struct lcdc_device *lcdc_dev)
{
	struct device_node *np = lcdc_dev->dev->of_node;
	struct device_node *root = of_get_child_by_name(lcdc_dev->dev->of_node,
							"power_ctr");
	struct device *dev = lcdc_dev->dev;
	struct device_node *child;
	struct list_head *pos;
	struct pwr_ctr *pwr_ctr;
	u32 val;

	if (of_property_read_u32(np, "rockchip,prop", &val))
		/*default set it as primary */
		lcdc_dev->prop = 0;
	else
		lcdc_dev->prop = val;

	if (of_property_read_u32(np, "rockchip,pwr18", &val))
		/*default set it as 3.xv power supply */
		lcdc_dev->pwr18 = false;
	else
		lcdc_dev->pwr18 = (val ? true : false);


	if (!root) {
		dev_info(lcdc_dev->dev, "can't find power_ctr for lcdc%d\n",
			 lcdc_dev->id);
		return;
	}

	INIT_LIST_HEAD(&lcdc_dev->pwrlist_head);

	for_each_child_of_node(root, child) {
		pwr_ctr = rk3288_lcdc_parse_pwr(dev, child);
		if (pwr_ctr)
			list_add_tail(&pwr_ctr->list, &lcdc_dev->pwrlist_head);
	}

	if (!list_empty(&lcdc_dev->pwrlist_head)) {
		list_for_each(pos, &lcdc_dev->pwrlist_head) {
			pwr_ctr = list_entry(pos, struct pwr_ctr, list);
			dev_info(dev, "pwr_ctr_name:%s\n"
				 "pwr_type:%s\n"
				 "gpio:%d\n"
				 "atv_val:%d\n"
				 "delay:%d\n\n",
				 pwr_ctr->name,
				 (pwr_ctr->type == GPIO) ?
				 "gpio" : "regulator",
				 pwr_ctr->gpio,
				 pwr_ctr->atv_val,
				 pwr_ctr->delay);
		}
	}
}

static void rk3288_pwr_enable(struct lcdc_device *lcdc_dev)
{
	struct list_head *pos;
	struct pwr_ctr *pwr_ctr;

	if (list_empty(&lcdc_dev->pwrlist_head))
		return;

	list_for_each(pos, &lcdc_dev->pwrlist_head) {
		pwr_ctr = list_entry(pos, struct pwr_ctr, list);
		if (pwr_ctr->type == GPIO) {
			gpio_direction_output(pwr_ctr->gpio,
					      pwr_ctr->atv_val);
			mdelay(pwr_ctr->delay);
		}
	}
}

static void rk3288_pwr_disable(struct lcdc_device *lcdc_dev)
{
	struct list_head *pos;
	struct pwr_ctr *pwr_ctr;

	if (list_empty(&lcdc_dev->pwrlist_head))
		return;

	list_for_each(pos, &lcdc_dev->pwrlist_head) {
		pwr_ctr = list_entry(pos, struct pwr_ctr, list);
		if (pwr_ctr->type == GPIO)
			gpio_set_value(pwr_ctr->gpio, pwr_ctr->atv_val);
	}
}

static void rk3288_lcdc_alpha_cfg(struct lcdc_device *lcdc_dev,
				  struct lcdc_win_data *layer)
{
	struct alpha_config alpha_config;

	u32 mask, val;
	int ppixel_alpha, global_alpha;
	u32 src_alpha_ctl, dst_alpha_ctl;

	ppixel_alpha = ((layer->format == ARGB888) ||
			(layer->format == ABGR888)) ? 1 : 0;
	global_alpha = (layer->g_alpha_val == 0) ? 0 : 1;
	alpha_config.src_global_alpha_val = layer->g_alpha_val;
	layer->alpha_mode = AB_SRC_OVER;
	switch (layer->alpha_mode) {
	case AB_USER_DEFINE:
		break;
	case AB_CLEAR:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_SRC:
		alpha_config.src_factor_mode = AA_ONE;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_DST:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_ONE;
		break;
	case AB_SRC_OVER:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		if (global_alpha)
			alpha_config.src_factor_mode = AA_SRC_GLOBAL;
		else
			alpha_config.src_factor_mode = AA_ONE;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_DST_OVER:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_ONE;
		break;
	case AB_SRC_IN:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_DST_IN:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_SRC;
		break;
	case AB_SRC_OUT:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_ZERO;
		break;
	case AB_DST_OUT:
		alpha_config.src_factor_mode = AA_ZERO;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_SRC_ATOP:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_DST_ATOP:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_SRC;
		break;
	case XOR:
		alpha_config.src_color_mode = AA_SRC_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_INVERSE;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	case AB_SRC_OVER_GLOBAL:
		alpha_config.src_global_alpha_mode = AA_PER_PIX_GLOBAL;
		alpha_config.src_color_mode = AA_SRC_NO_PRE_MUL;
		alpha_config.src_factor_mode = AA_SRC_GLOBAL;
		alpha_config.dst_factor_mode = AA_SRC_INVERSE;
		break;
	default:
		pr_err("alpha mode error\n");
		break;
	}
	if ((ppixel_alpha == 1) && (global_alpha == 1))
		alpha_config.src_global_alpha_mode = AA_PER_PIX_GLOBAL;
	else if (ppixel_alpha == 1)
		alpha_config.src_global_alpha_mode = AA_PER_PIX;
	else if (global_alpha == 1)
		alpha_config.src_global_alpha_mode = AA_GLOBAL;
	else
		dev_warn(lcdc_dev->dev, "alpha_en should be 0\n");
	alpha_config.src_alpha_mode = AA_STRAIGHT;
	alpha_config.src_alpha_cal_m0 = AA_NO_SAT;

	switch (layer->id) {
	case 0:
		src_alpha_ctl = 0x60;
		dst_alpha_ctl = 0x64;
		break;
	case 1:
		src_alpha_ctl = 0xa0;
		dst_alpha_ctl = 0xa4;
		break;
	case 2:
		src_alpha_ctl = 0xdc;
		dst_alpha_ctl = 0xec;
		break;
	case 3:
		src_alpha_ctl = 0x12c;
		dst_alpha_ctl = 0x13c;
		break;
	}
	mask = m_WIN0_DST_FACTOR_M0;
	val = v_WIN0_DST_FACTOR_M0(alpha_config.dst_factor_mode);
	lcdc_msk_reg(lcdc_dev, dst_alpha_ctl, mask, val);
	mask = m_WIN0_SRC_ALPHA_EN | m_WIN0_SRC_COLOR_M0 |
		m_WIN0_SRC_ALPHA_M0 | m_WIN0_SRC_BLEND_M0 |
		m_WIN0_SRC_ALPHA_CAL_M0 | m_WIN0_SRC_FACTOR_M0|
		m_WIN0_SRC_GLOBAL_ALPHA;
	val = v_WIN0_SRC_ALPHA_EN(1) |
		v_WIN0_SRC_COLOR_M0(alpha_config.src_color_mode) |
		v_WIN0_SRC_ALPHA_M0(alpha_config.src_alpha_mode) |
		v_WIN0_SRC_BLEND_M0(alpha_config.src_global_alpha_mode) |
		v_WIN0_SRC_ALPHA_CAL_M0(alpha_config.src_alpha_cal_m0) |
		v_WIN0_SRC_FACTOR_M0(alpha_config.src_factor_mode) |
		v_WIN0_SRC_GLOBAL_ALPHA(alpha_config.src_global_alpha_val);
	lcdc_msk_reg(lcdc_dev, src_alpha_ctl, mask, val);
}

static void rk3288_lcdc_win01_set(struct lcdc_device *lcdc_dev,
				  struct lcdc_win_data *win)
{
	unsigned int mask, val;
	unsigned int off = win->id * 0x40;
	struct drm_display_mode *mode = lcdc_dev->mode;
	u32 dsp_stx = win->xpos + mode->htotal - mode->hsync_start;
	u32 dsp_sty = win->ypos + mode->vtotal - mode->vsync_start;

	spin_lock(&lcdc_dev->reg_lock);

	if (win->enabled) {
		mask = m_WIN0_EN | m_WIN0_DATA_FMT | m_WIN0_FMT_10 |
			m_WIN0_LB_MODE | m_WIN0_RB_SWAP;
		val = v_WIN0_EN(1) | v_WIN0_DATA_FMT(win->format) |
			v_WIN0_FMT_10(win->fmt_10) |
			v_WIN0_LB_MODE(win->win_lb_mode) |
			v_WIN0_RB_SWAP(win->swap_rb);
		lcdc_msk_reg(lcdc_dev, WIN0_CTRL0+off, mask, val);

		mask = m_WIN0_BIC_COE_SEL |
			m_WIN0_VSD_YRGB_GT4 | m_WIN0_VSD_YRGB_GT2 |
			m_WIN0_VSD_CBR_GT4 | m_WIN0_VSD_CBR_GT2 |
			m_WIN0_YRGB_HOR_SCL_MODE | m_WIN0_YRGB_VER_SCL_MODE |
			m_WIN0_YRGB_HSD_MODE | m_WIN0_YRGB_VSU_MODE |
			m_WIN0_YRGB_VSD_MODE | m_WIN0_CBR_HOR_SCL_MODE |
			m_WIN0_CBR_VER_SCL_MODE | m_WIN0_CBR_HSD_MODE |
			m_WIN0_CBR_VSU_MODE | m_WIN0_CBR_VSD_MODE;
		val = v_WIN0_BIC_COE_SEL(win->bic_coe_el) |
			v_WIN0_VSD_YRGB_GT4(win->vsd_yrgb_gt4) |
			v_WIN0_VSD_YRGB_GT2(win->vsd_yrgb_gt2) |
			v_WIN0_VSD_CBR_GT4(win->vsd_cbr_gt4) |
			v_WIN0_VSD_CBR_GT2(win->vsd_cbr_gt2) |
			v_WIN0_YRGB_HOR_SCL_MODE(win->yrgb_hor_scl_mode) |
			v_WIN0_YRGB_VER_SCL_MODE(win->yrgb_ver_scl_mode) |
			v_WIN0_YRGB_HSD_MODE(win->yrgb_hsd_mode) |
			v_WIN0_YRGB_VSU_MODE(win->yrgb_vsu_mode) |
			v_WIN0_YRGB_VSD_MODE(win->yrgb_vsd_mode) |
			v_WIN0_CBR_HOR_SCL_MODE(win->cbr_hor_scl_mode) |
			v_WIN0_CBR_VER_SCL_MODE(win->cbr_ver_scl_mode) |
			v_WIN0_CBR_HSD_MODE(win->cbr_hsd_mode) |
			v_WIN0_CBR_VSU_MODE(win->cbr_vsu_mode) |
			v_WIN0_CBR_VSD_MODE(win->cbr_vsd_mode);
		lcdc_msk_reg(lcdc_dev, WIN0_CTRL1+off, mask, val);

		val = v_WIN0_VIR_STRIDE(win->y_vir_stride) |
			v_WIN0_VIR_STRIDE_UV(win->uv_vir_stride);
		lcdc_writel(lcdc_dev, WIN0_VIR+off, val);
		lcdc_writel(lcdc_dev, WIN0_YRGB_MST+off, win->yrgb_addr);
		lcdc_writel(lcdc_dev, WIN0_CBR_MST+off, win->uv_addr);
		val = v_WIN0_ACT_WIDTH(win->xact) |
			v_WIN0_ACT_HEIGHT(win->yact);
		lcdc_writel(lcdc_dev, WIN0_ACT_INFO+off, val);

		val = v_WIN0_DSP_WIDTH(win->xsize) |
			v_WIN0_DSP_HEIGHT(win->ysize);
		lcdc_writel(lcdc_dev, WIN0_DSP_INFO+off, val);

		val = v_WIN0_DSP_XST(dsp_stx) |
			v_WIN0_DSP_YST(dsp_sty);
		lcdc_writel(lcdc_dev, WIN0_DSP_ST+off, val);

		val = v_WIN0_HS_FACTOR_YRGB(0x1000) |
			v_WIN0_VS_FACTOR_YRGB(0x1000);
		lcdc_writel(lcdc_dev, WIN0_SCL_FACTOR_YRGB+off, val);

		val = v_WIN0_HS_FACTOR_CBR(0x1000) |
			v_WIN0_VS_FACTOR_CBR(0x1000);
		lcdc_writel(lcdc_dev, WIN0_SCL_FACTOR_CBR+off, val);

		lcdc_writel(lcdc_dev, WIN0_COLOR_KEY, 0x80000000);
		if (win->alpha_en == 1) {
			rk3288_lcdc_alpha_cfg(lcdc_dev, win);
		} else {
			mask = m_WIN0_SRC_ALPHA_EN;
			val = v_WIN0_SRC_ALPHA_EN(0);
			lcdc_msk_reg(lcdc_dev,
				     WIN0_SRC_ALPHA_CTRL+off, mask, val);
		}
	} else {
		mask = m_WIN0_EN;
		val = v_WIN0_EN(0);
		lcdc_msk_reg(lcdc_dev, WIN0_CTRL0+off, mask, val);
	}

	spin_unlock(&lcdc_dev->reg_lock);
}

static void rk3288_lcdc_win23_set(struct lcdc_device *lcdc_dev,
				  struct lcdc_win_data *win)
{
	unsigned int mask, val;
	unsigned int off = (win->id-2) * 0x50;
	struct drm_display_mode *mode = lcdc_dev->mode;
	u32 dsp_stx;
	u32 dsp_sty;

	dsp_stx = win->xpos + mode->htotal - mode->hdisplay;
	dsp_sty = win->ypos + mode->vtotal - mode->vdisplay;

	if (win->enabled) {
		mask = m_WIN2_EN | m_WIN2_DATA_FMT | m_WIN2_RB_SWAP;
		val = v_WIN2_EN(1) | v_WIN2_DATA_FMT(win->fmt_cfg) |
			v_WIN2_RB_SWAP(win->swap_rb);
		lcdc_msk_reg(lcdc_dev, WIN2_CTRL0+off, mask, val);
		/*area 0*/
		mask = m_WIN2_MST0_EN;
		val = v_WIN2_MST0_EN(1);
		lcdc_msk_reg(lcdc_dev, WIN2_CTRL0+off, mask, val);

		mask = m_WIN2_VIR_STRIDE0;
		val = v_WIN2_VIR_STRIDE0(win->y_vir_stride);
		lcdc_msk_reg(lcdc_dev, WIN2_VIR0_1+off, mask, val);

		lcdc_writel(lcdc_dev, WIN2_MST0+off, win->yrgb_addr);
		val = v_WIN2_DSP_WIDTH0(win->xsize) |
			v_WIN2_DSP_HEIGHT0(win->ysize);
		lcdc_writel(lcdc_dev, WIN2_DSP_INFO0+off, val);
		val = v_WIN2_DSP_XST0(dsp_stx) |
			v_WIN2_DSP_YST0(dsp_sty);
		lcdc_writel(lcdc_dev, WIN2_DSP_ST0+off, val);
		if (win->alpha_en == 1) {
			rk3288_lcdc_alpha_cfg(lcdc_dev, win);
		} else {
			mask = m_WIN2_SRC_ALPHA_EN;
			val = v_WIN2_SRC_ALPHA_EN(0);
			lcdc_msk_reg(lcdc_dev,
				     WIN2_SRC_ALPHA_CTRL+off, mask, val);
		}
	} else {
		mask = m_WIN2_EN | m_WIN2_MST0_EN |
			m_WIN2_MST0_EN | m_WIN2_MST2_EN |
			m_WIN2_MST3_EN;
		val = v_WIN2_EN(0) | v_WIN2_MST0_EN(0) |
			v_WIN2_MST1_EN(0) | v_WIN2_MST2_EN(0) |
			v_WIN2_MST3_EN(0);
		lcdc_msk_reg(lcdc_dev, WIN2_CTRL0+off, mask, val);
	}
}

static int rk3288_lcdc_initial(struct lcdc_device *lcdc_dev)
{
	lcdc_dev->hclk = devm_clk_get(lcdc_dev->dev, "hclk_lcdc");
	lcdc_dev->aclk = devm_clk_get(lcdc_dev->dev, "aclk_lcdc");
	lcdc_dev->dclk = devm_clk_get(lcdc_dev->dev, "dclk_lcdc");

	if ((IS_ERR(lcdc_dev->aclk)) || (IS_ERR(lcdc_dev->dclk)) ||
	    (IS_ERR(lcdc_dev->hclk))) {
		dev_err(lcdc_dev->dev, "failed to get lcdc%d clk source\n",
			lcdc_dev->id);
		return -ENODEV;
	}

	if (rk3288_lcdc_clk_enable(lcdc_dev) < 0) {
		dev_err(lcdc_dev->dev, "failed to enable lcdc%d clks\n",
			lcdc_dev->id);
		return -ENODEV;
	}

	memcpy(lcdc_dev->regsbak, lcdc_dev->regs, lcdc_dev->len);
	lcdc_writel(lcdc_dev, CABC_GAUSS_LINE0_0, 0x15110903);
	lcdc_writel(lcdc_dev, CABC_GAUSS_LINE0_1, 0x00030911);
	lcdc_writel(lcdc_dev, CABC_GAUSS_LINE1_0, 0x1a150b04);
	lcdc_writel(lcdc_dev, CABC_GAUSS_LINE1_1, 0x00040b15);
	lcdc_writel(lcdc_dev, CABC_GAUSS_LINE2_0, 0x15110903);
	lcdc_writel(lcdc_dev, CABC_GAUSS_LINE2_1, 0x00030911);

	lcdc_writel(lcdc_dev, FRC_LOWER01_0, 0x12844821);
	lcdc_writel(lcdc_dev, FRC_LOWER01_1, 0x21488412);
	lcdc_writel(lcdc_dev, FRC_LOWER10_0, 0xa55a9696);
	lcdc_writel(lcdc_dev, FRC_LOWER10_1, 0x5aa56969);
	lcdc_writel(lcdc_dev, FRC_LOWER11_0, 0xdeb77deb);
	lcdc_writel(lcdc_dev, FRC_LOWER11_1, 0xed7bb7de);

	lcdc_set_bit(lcdc_dev, SYS_CTRL, m_AUTO_GATING_EN);
	lcdc_cfg_done(lcdc_dev);

	lcdc_dev->standby = DRM_MODE_DPMS_OFF;

	return 0;
}

static struct lcdc_driver *rk3288_lcdc_init(struct platform_device *pdev)
{
	struct lcdc_device *lcdc_dev;
	struct lcdc_driver *lcdc_drv;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct device_node *np;
	int ret = 0;

	if (!dev->of_node)
		return NULL;

	np = pdev->dev.of_node;

	/* if the primary lcdc has not registered, the extend
	 * lcdc register later
	*/
	lcdc_dev = devm_kzalloc(dev, sizeof(struct lcdc_device), GFP_KERNEL);
	if (!lcdc_dev) {
		dev_err(&pdev->dev, "rk3288 lcdc device kmalloc fail!");
		return NULL;
	}
	lcdc_dev->dev = dev;
	lcdc_drv = &lcdc_dev->lcdc_drv;

	rk3288_lcdc_parse_dt(lcdc_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lcdc_dev->reg_phy_base = res->start;
	lcdc_dev->len = resource_size(res);
	lcdc_dev->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(lcdc_dev->regs)) {
		dev_err(&pdev->dev, "ioremap lcdc devices fail\n");
		return NULL;
	}

	lcdc_dev->regsbak = devm_kzalloc(dev, lcdc_dev->len, GFP_KERNEL);
	if (IS_ERR(lcdc_dev->regsbak)) {
		dev_err(&pdev->dev, "lcdc devices kzalloc reg backup fail\n");
		return NULL;
	}

	lcdc_dev->id = rk3288_lcdc_get_id(lcdc_dev->reg_phy_base);
	if (lcdc_dev->id < 0) {
		dev_err(&pdev->dev, "no such lcdc device id[%d]!\n",
			lcdc_dev->id);
		return NULL;
	}

	dev_set_name(lcdc_dev->dev, "lcdc%d", lcdc_dev->id);

	lcdc_dev->irq = platform_get_irq(pdev, 0);
	if (lcdc_dev->irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ for lcdc%d\n",
			lcdc_dev->id);
		return NULL;
	}

	ret = devm_request_irq(dev, lcdc_dev->irq, rk3288_lcdc_isr,
			       IRQF_DISABLED | IRQF_SHARED,
			       dev_name(dev), lcdc_dev);
	if (ret) {
		dev_err(&pdev->dev, "cannot requeset irq %d - err %d\n",
			lcdc_dev->irq, ret);
		return NULL;
	}

	spin_lock_init(&lcdc_dev->reg_lock);

	ret = rk3288_lcdc_initial(lcdc_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot initial lcdc dev - err %d\n", ret);
		return NULL;
	}

	dev_info(dev, "lcdc%d probe ok\n", lcdc_dev->id);

	return lcdc_drv;
}

static void rk3288_lcdc_deinit(struct lcdc_driver *lcdc_drv)
{
}

static void rk3288_lcdc_dpms(struct lcdc_driver *lcdc_drv, int mode)
{
	struct lcdc_device *lcdc_dev =
			container_of(lcdc_drv, struct lcdc_device, lcdc_drv);

	if (lcdc_dev->standby == mode)
		return;

	if (mode == DRM_MODE_DPMS_ON) {
		if (rk3288_lcdc_clk_enable(lcdc_dev) < 0) {
			dev_err(lcdc_dev->dev, "failed to enable lcdc%d clks\n",
				lcdc_dev->id);
			return;
		}

		spin_lock(&lcdc_dev->reg_lock);
		lcdc_msk_reg(lcdc_dev, SYS_CTRL, m_STANDBY_EN, v_STANDBY_EN(0));
		spin_unlock(&lcdc_dev->reg_lock);
		rk3288_lcdc_enable_irq(lcdc_dev);
		rk3288_pwr_enable(lcdc_dev);
	} else {
		rk3288_pwr_disable(lcdc_dev);
		if (lcdc_dev->clk_on) {
			rk3288_lcdc_disable_irq(lcdc_dev);
			spin_lock(&lcdc_dev->reg_lock);
			lcdc_msk_reg(lcdc_dev, SYS_CTRL,
				     m_STANDBY_EN, v_STANDBY_EN(1));
			spin_unlock(&lcdc_dev->reg_lock);
			rk3288_lcdc_clk_disable(lcdc_dev);
		}
	}

	lcdc_dev->standby = mode;
}

static void rk3288_lcdc_mode_set(struct lcdc_driver *lcdc_drv,
				 struct drm_display_mode *mode)
{
	struct lcdc_device *lcdc_dev =
			container_of(lcdc_drv, struct lcdc_device, lcdc_drv);
	u32 mask, val;
	u32 v = 0;
	u16 face = OUT_D888_P666;
	u16 x_res = mode->hdisplay;
	u16 y_res = mode->vdisplay;
	u16 h_total = mode->htotal;
	u16 v_total = mode->vtotal;
	u16 hsync_len = mode->hsync_end - mode->hsync_start;
	u16 left_margin = mode->htotal - mode->hsync_end;
	u16 vsync_len = mode->vsync_end - mode->vsync_start;
	u16 upper_margin = mode->vtotal - mode->vsync_end;

	u8 pin_vsync = (mode->flags & DRM_MODE_FLAG_NVSYNC) ? 1 : 0;
	u8 pin_hsync = (mode->flags & DRM_MODE_FLAG_NVSYNC) ? 1 : 0;
	u8 pin_den = 0;
	u8 pin_dclk = 1;
	u8 swap_gb = 0;
	u8 swap_rb = 0;
	u8 swap_rg = 0;
	u8 swap_dumy = 0;
	u8 swap_delta = 0;
	u8 type = SCREEN_LVDS;

	spin_lock(&lcdc_dev->reg_lock);
	if (likely(lcdc_dev->clk_on)) {
		val = 0;
		switch (face) {
		case OUT_P565:
			face = OUT_P565;
			val = v_DITHER_DOWN_EN(1) | v_DITHER_DOWN_MODE(0) |
				v_DITHER_DOWN_SEL(1);
			break;
		case OUT_P666:
			face = OUT_P666;
			val = v_DITHER_DOWN_EN(1) | v_DITHER_DOWN_MODE(1) |
				v_DITHER_DOWN_SEL(1);
			break;
		case OUT_D888_P666:
			val = v_DITHER_DOWN_MODE(1);
		case OUT_D888_P565:
			val |= v_DITHER_DOWN_EN(1) | v_DITHER_DOWN_SEL(1);
		case OUT_P888:
			face = OUT_P888;
			break;
		default:
			dev_err(lcdc_dev->dev, "un supported interface!\n");
			break;
		}
		mask = m_DITHER_DOWN_EN | m_DITHER_DOWN_MODE |
			m_DITHER_DOWN_SEL;
		lcdc_msk_reg(lcdc_dev, DSP_CTRL1, mask, val);
		switch (type) {
		case SCREEN_RGB:
		case SCREEN_LVDS:
		case SCREEN_DUAL_LVDS:
			mask = m_RGB_OUT_EN;
			val = v_RGB_OUT_EN(1);
			v = 1 << (3+16);
			v |= (lcdc_dev->id << 3);
			break;
		case SCREEN_HDMI:
			face = OUT_RGB_AAA;
			mask = m_HDMI_OUT_EN;
			val = v_HDMI_OUT_EN(1);
			break;
		case SCREEN_MIPI:
			mask = m_MIPI_OUT_EN;
			val = v_MIPI_OUT_EN(1);
			break;
		case SCREEN_DUAL_MIPI:
			mask = m_MIPI_OUT_EN | m_DOUB_CHANNEL_EN;
			val = v_MIPI_OUT_EN(1) | v_DOUB_CHANNEL_EN(1);
			break;
		case SCREEN_EDP:
			face = OUT_RGB_AAA;
			mask = m_DITHER_DOWN_EN | m_DITHER_UP_EN;
			val = v_DITHER_DOWN_EN(0) | v_DITHER_UP_EN(0);
			lcdc_msk_reg(lcdc_dev, DSP_CTRL1, mask, val);
			mask = m_EDP_OUT_EN;
			val = v_EDP_OUT_EN(1);
			break;
		}
		lcdc_msk_reg(lcdc_dev, SYS_CTRL, mask, val);

		mask = m_DSP_OUT_MODE | m_DSP_HSYNC_POL | m_DSP_VSYNC_POL |
			m_DSP_DEN_POL | m_DSP_DCLK_POL | m_DSP_BG_SWAP |
			m_DSP_RB_SWAP | m_DSP_RG_SWAP | m_DSP_DELTA_SWAP |
			m_DSP_DUMMY_SWAP | m_DSP_OUT_ZERO | m_DSP_BLANK_EN |
			m_DSP_BLACK_EN | m_DSP_X_MIR_EN | m_DSP_Y_MIR_EN;
		val = v_DSP_OUT_MODE(face) | v_DSP_HSYNC_POL(pin_hsync) |
			v_DSP_VSYNC_POL(pin_vsync) |
			v_DSP_DEN_POL(pin_den) | v_DSP_DCLK_POL(pin_dclk) |
			v_DSP_BG_SWAP(swap_gb) | v_DSP_RB_SWAP(swap_rb) |
			v_DSP_RG_SWAP(swap_rg) |
			v_DSP_DELTA_SWAP(swap_delta) |
			v_DSP_DUMMY_SWAP(swap_dumy) | v_DSP_OUT_ZERO(0) |
			v_DSP_BLANK_EN(0) | v_DSP_BLACK_EN(0);
		lcdc_msk_reg(lcdc_dev, DSP_CTRL0, mask, val);

		mask = m_DSP_BG_BLUE | m_DSP_BG_GREEN | m_DSP_BG_RED;
		val = v_DSP_BG_BLUE(0) | v_DSP_BG_GREEN(0) | v_DSP_BG_RED(0);
		lcdc_msk_reg(lcdc_dev, DSP_BG, mask, val);

		mask = m_DSP_HS_PW | m_DSP_HTOTAL;
		val = v_DSP_HS_PW(hsync_len) | v_DSP_HTOTAL(h_total);
		lcdc_msk_reg(lcdc_dev, DSP_HTOTAL_HS_END, mask, val);

		mask = m_DSP_HACT_END | m_DSP_HACT_ST;
		val = v_DSP_HACT_END(hsync_len + left_margin + x_res) |
			v_DSP_HACT_ST(hsync_len + left_margin);
		lcdc_msk_reg(lcdc_dev, DSP_HACT_ST_END, mask, val);

		mask = m_DSP_VS_PW | m_DSP_VTOTAL;
		val = v_DSP_VS_PW(vsync_len) | v_DSP_VTOTAL(v_total);
		lcdc_msk_reg(lcdc_dev, DSP_VTOTAL_VS_END, mask, val);

		mask = m_DSP_VACT_END | m_DSP_VACT_ST;
		val = v_DSP_VACT_END(vsync_len + upper_margin + y_res) |
			v_DSP_VACT_ST(vsync_len + upper_margin);
		lcdc_msk_reg(lcdc_dev, DSP_VACT_ST_END, mask, val);
	}
	spin_unlock(&lcdc_dev->reg_lock);
	clk_set_rate(lcdc_dev->dclk, mode->clock * 1000);
	lcdc_dev->mode = mode;
}

static void rk3288_lcdc_enable_vblank(struct lcdc_driver *lcdc_drv)
{
	rk3288_lcdc_enable_irq(container_of(lcdc_drv,
					    struct lcdc_device, lcdc_drv));
}

static void rk3288_lcdc_disable_vblank(struct lcdc_driver *lcdc_drv)
{
	rk3288_lcdc_disable_irq(container_of(lcdc_drv,
					     struct lcdc_device, lcdc_drv));
}

static struct lcdc_win_data *
	rk3288_lcdc_get_win(struct lcdc_driver *lcdc_drv, int zpos)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lcdc_win); i++) {
		if (lcdc_win[i].zpos == zpos)
			return &lcdc_win[i];
	}

	return NULL;
}

static void rk3288_lcdc_win_commit(struct lcdc_driver *lcdc_drv,
				   struct lcdc_win_data *win)
{
	struct lcdc_device *lcdc_dev =
			container_of(lcdc_drv, struct lcdc_device, lcdc_drv);
	struct device *dev = lcdc_dev->dev;

	switch (win->id) {
	case 0:
	case 1:
		rk3288_lcdc_win01_set(lcdc_dev, win);
		break;
	case 2:
	case 3:
		rk3288_lcdc_win23_set(lcdc_dev, win);
		break;
	default:
		dev_info(dev, "not support win%d\n", win->id);
	}

	spin_lock(&lcdc_dev->reg_lock);
	lcdc_cfg_done(lcdc_dev);
	spin_unlock(&lcdc_dev->reg_lock);
}

static struct lcdc_driver_ops rk3288_lcdc_driver_ops = {
	.init = rk3288_lcdc_init,
	.deinit = rk3288_lcdc_deinit,
	.dpms = rk3288_lcdc_dpms,
	.mode_set = rk3288_lcdc_mode_set,
	.enable_vblank = rk3288_lcdc_enable_vblank,
	.disable_vblank = rk3288_lcdc_disable_vblank,
	.get_win = rk3288_lcdc_get_win,
	.win_commit = rk3288_lcdc_win_commit,
};

struct lcdc_driver_data rockchip_rk3288_lcdc = {
	.num_win = ARRAY_SIZE(lcdc_win),
	.ops = &rk3288_lcdc_driver_ops,
};
