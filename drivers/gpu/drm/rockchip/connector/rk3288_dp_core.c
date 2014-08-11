/*
* Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
* Author:
*      yxj <yxj@rock-chips.com>
*      cym <cym@rock-chips.com>
*
* based on exynos_dp_core.c
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/of.h>

#include "rk3288_dp_core.h"

static int rk3288_edp_clk_enable(struct rk3288_edp *edp)
{
	int ret = 0;

	if (!edp->clk_on) {
		ret = clk_prepare_enable(edp->pclk);
		if (ret < 0) {
			dev_err(edp->dev, "cannot enable edp pclk %d\n", ret);
			goto err_pclk;
		}

		ret = clk_prepare_enable(edp->clk_edp);
		if (ret < 0) {
			dev_err(edp->dev, "cannot enable clk_edp %d\n", ret);
			goto err_clk_edp;
		}

		ret = clk_set_rate(edp->clk_24m, 24000000);
		if (ret < 0) {
			dev_err(edp->dev, "cannot set edp clk_24m rate %d\n",
				ret);
			goto err_clk_24m;
		}

		ret = clk_prepare_enable(edp->clk_24m);
		if (ret < 0) {
			dev_err(edp->dev, "cannot enable edp clk_24m %d\n",
				ret);
			goto err_clk_24m;
		}

		edp->clk_on = true;
	}

	return 0;

err_clk_24m:
	clk_disable_unprepare(edp->clk_edp);
err_clk_edp:
	clk_disable_unprepare(edp->pclk);
err_pclk:
	edp->clk_on = false;

	return ret;
}

static int rk3288_edp_clk_disable(struct rk3288_edp *edp)
{
	if (edp->clk_on) {
		clk_disable_unprepare(edp->pclk);
		clk_disable_unprepare(edp->clk_edp);
		clk_disable_unprepare(edp->clk_24m);
		edp->clk_on = false;
	}

	return 0;
}

static int rk3288_edp_pre_init(struct rk3288_edp *edp)
{
	u32 val;
	int ret = 0;

	val = GRF_EDP_REF_CLK_SEL_INTER | (GRF_EDP_REF_CLK_SEL_INTER << 16);
	ret = regmap_write(edp->grf, edp->soc_data->grf_soc_con12, val);
	if (ret != 0) {
		dev_err(edp->dev, "Could not write to GRF: %d\n", ret);
		return ret;
	}

	reset_control_assert(edp->rst);
	usleep_range(10, 20);
	reset_control_deassert(edp->rst);

	return 0;
}

static int rk3288_edp_init_edp(struct rk3288_edp *edp)
{
	int lcdc_id = 1;

	u32 val = 0;
	int ret = 0;

	/*select lcdc*/
	if (lcdc_id == 1)
		val = EDP_SEL_VOP_LIT | (EDP_SEL_VOP_LIT << 16);
	else
		val = EDP_SEL_VOP_LIT << 16;
	ret = regmap_write(edp->grf, edp->soc_data->grf_soc_con6, val);
	if (ret != 0) {
		dev_err(edp->dev, "Could not write to GRF: %d\n", ret);
		return ret;
	}

	rk3288_edp_reset(edp);
	rk3288_edp_init_refclk(edp);
	rk3288_edp_init_interrupt(edp);
	rk3288_edp_enable_sw_function(edp);
	rk3288_edp_init_analog_func(edp);
	rk3288_edp_init_hpd(edp);
	rk3288_edp_init_aux(edp);

	return 0;
}

static int rk3288_edp_get_max_rx_bandwidth(
					struct rk3288_edp *edp,
					u8 *bandwidth)
{
	u8 data;
	int retval = 0;

	/*
	 * For DP rev.1.1, Maximum link rate of Main Link lanes
	 * 0x06 = 1.62 Gbps, 0x0a = 2.7 Gbps
	 */
	retval = rk3288_edp_read_byte_from_dpcd(
			edp, DPCD_ADDR_MAX_LINK_RATE, &data);
	if (retval < 0)
		*bandwidth = 0;
	else
		*bandwidth = data;

	return retval;
}

static int rk3288_edp_get_max_rx_lane_count(struct rk3288_edp *edp,
					    u8 *lane_count)
{
	u8 data;
	int retval;

	/*
	 * For DP rev.1.1, Maximum number of Main Link lanes
	 * 0x01 = 1 lane, 0x02 = 2 lanes, 0x04 = 4 lanes
	 */
	retval = rk3288_edp_read_byte_from_dpcd(
			edp, DPCD_ADDR_MAX_LANE_COUNT, &data);
	if (retval < 0)
		*lane_count = 0;
	else
		*lane_count = DPCD_MAX_LANE_COUNT(data);

	return retval;
}

static int rk3288_edp_init_training(struct rk3288_edp *edp)
{
	int retval;

	/*
	 * MACRO_RST must be applied after the PLL_LOCK to avoid
	 * the DP inter pair skew issue for at least 10 us
	 */
	rk3288_edp_reset_macro(edp);

	retval = rk3288_edp_get_max_rx_bandwidth(
				edp, &edp->link_train.link_rate);
	retval = rk3288_edp_get_max_rx_lane_count(
				edp, &edp->link_train.lane_count);
	dev_dbg(edp->dev, "max link rate:%d.%dGps max number of lanes:%d\n",
		edp->link_train.link_rate * 27/100,
		edp->link_train.link_rate*27%100,
		edp->link_train.lane_count);

	if ((edp->link_train.link_rate != LINK_RATE_1_62GBPS) &&
	    (edp->link_train.link_rate != LINK_RATE_2_70GBPS)) {
		dev_warn(edp->dev, "Rx Max Link Rate is abnormal :%x !\n"
			 "use default link rate:%d.%dGps\n",
			 edp->link_train.link_rate,
			 edp->video_info.link_rate*27/100,
			 edp->video_info.link_rate*27%100);
			 edp->link_train.link_rate = edp->video_info.link_rate;
	}

	if (edp->link_train.lane_count == 0) {
		dev_err(edp->dev, "Rx Max Lane count is abnormal :%x !\n"
			"use default lanes:%d\n",
			edp->link_train.lane_count,
			edp->video_info.lane_count);
		edp->link_train.lane_count = edp->video_info.lane_count;
	}

	rk3288_edp_analog_power_ctr(edp, 1);

	return 0;
}

static int rk3288_edp_hw_link_training(struct rk3288_edp *edp)
{
	u32 cnt = 50;
	u32 val;

	/* Set link rate and count as you want to establish*/
	rk3288_edp_set_link_bandwidth(edp, edp->link_train.link_rate);
	rk3288_edp_set_lane_count(edp, edp->link_train.lane_count);
	rk3288_edp_hw_link_training_en(edp);
	val = rk3288_edp_wait_hw_lt_done(edp);
	while (val) {
		if (cnt-- <= 0) {
			dev_err(edp->dev, "hw lt timeout");
			return -ETIMEDOUT;
		}
		mdelay(1);
		val = rk3288_edp_wait_hw_lt_done(edp);
	}

	val = rk3288_edp_get_hw_lt_status(edp);
	if (val)
		dev_err(edp->dev, "hw lt err:%d\n", val);

	return val;
}

static int rk3288_edp_set_link_train(struct rk3288_edp *edp)
{
	int retval;

	rk3288_edp_init_training(edp);

	retval = rk3288_edp_hw_link_training(edp);
	if (retval < 0)
		dev_err(edp->dev, "DP hw LT failed!\n");

	return retval;
}

static int rk3288_edp_config_video(struct rk3288_edp *edp,
				   struct video_info *video_info)
{
	int retval = 0;
	int timeout_loop = 0;
	int done_count = 0;

	rk3288_edp_config_video_slave_mode(edp, video_info);

	rk3288_edp_set_video_color_format(edp, video_info->color_depth,
					  video_info->color_space,
					  video_info->dynamic_range,
					  video_info->ycbcr_coeff);

	if (rk3288_edp_get_pll_lock_status(edp) == DP_PLL_UNLOCKED) {
		dev_err(edp->dev, "PLL is not locked yet.\n");
		return -EINVAL;
	}

	for (;;) {
		timeout_loop++;
		if (rk3288_edp_is_slave_video_stream_clock_on(edp) == 0)
			break;

		if (DP_TIMEOUT_LOOP_CNT < timeout_loop) {
			dev_err(edp->dev, "Timeout of video streamclk ok\n");
			return -ETIMEDOUT;
		}

		udelay(1);
	}

	/* Set to use the register calculated M/N video */
	rk3288_edp_set_video_cr_mn(edp, CALCULATED_M, 0, 0);

	/* Disable video mute */
	rk3288_edp_enable_video_mute(edp, 0);

	/* Configure video slave mode */
	rk3288_edp_enable_video_master(edp, 0);

	/* Enable video */
	rk3288_edp_start_video(edp);

	timeout_loop = 0;

	for (;;) {
		timeout_loop++;
		if (rk3288_edp_is_video_stream_on(edp) == 0) {
			done_count++;
			if (done_count > 10)
				break;
		} else if (done_count) {
			done_count = 0;
		}
		if (DP_TIMEOUT_LOOP_CNT < timeout_loop) {
			dev_err(edp->dev, "Timeout of video streamclk ok\n");
			return -ETIMEDOUT;
		}

		mdelay(1);
	}

	if (retval != 0)
		dev_err(edp->dev, "Video stream is not detected!\n");

	return retval;
}

static irqreturn_t rk3288_edp_isr(int irq, void *arg)
{
	struct rk3288_edp *edp = arg;
	enum dp_irq_type irq_type;

	irq_type = rk3288_edp_get_irq_type(edp);
	switch (irq_type) {
	case DP_IRQ_TYPE_HP_CABLE_IN:
		dev_dbg(edp->dev, "Received irq - cable in\n");
		rk3288_edp_clear_hotplug_interrupts(edp);
		break;
	case DP_IRQ_TYPE_HP_CABLE_OUT:
		dev_dbg(edp->dev, "Received irq - cable out\n");
		rk3288_edp_clear_hotplug_interrupts(edp);
		break;
	case DP_IRQ_TYPE_HP_CHANGE:
		/*
		 * We get these change notifications once in a while, but there
		 * is nothing we can do with them. Just ignore it for now and
		 * only handle cable changes.
		 */
		dev_dbg(edp->dev, "Received irq - hotplug change; ignoring.\n");
		rk3288_edp_clear_hotplug_interrupts(edp);
		break;
	default:
		dev_err(edp->dev, "Received irq - unknown type[%x]!\n",
			irq_type);
		rk3288_edp_clear_hotplug_interrupts(edp);
		break;
	}

	return IRQ_HANDLED;
}

static void rk3288_edp_enable(struct rockchip_connector *conn)
{
	struct rk3288_edp *edp = conn->priv;
	int ret = 0;

	ret = rk3288_edp_clk_enable(edp);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp clk %d\n", ret);
		return;
	}

	ret = rk3288_edp_pre_init(edp);
	if (ret < 0) {
		dev_err(edp->dev, "edp pre init fail %d\n", ret);
		return;
	}

	ret = rk3288_edp_init_edp(edp);
	if (ret < 0) {
		dev_err(edp->dev, "edp init fail %d\n", ret);
		return;
	}

	enable_irq(edp->irq);

	ret = rk3288_edp_set_link_train(edp);
	if (ret)
		dev_err(edp->dev, "link train failed!\n");
	else
		dev_dbg(edp->dev, "link training success.\n");

	rk3288_edp_set_lane_count(edp, edp->link_train.lane_count);
	rk3288_edp_set_link_bandwidth(edp, edp->link_train.link_rate);
	rk3288_edp_init_video(edp);

	ret = rk3288_edp_config_video(edp, &edp->video_info);
	if (ret)
		dev_err(edp->dev, "unable to config video\n");
}

static void  rk3288_edp_disable(struct rockchip_connector *conn)
{
	struct rk3288_edp *edp = conn->priv;

	disable_irq(edp->irq);
	rk3288_edp_reset(edp);
	rk3288_edp_analog_power_ctr(edp, 0);
	rk3288_edp_clk_disable(edp);
}

static int rk3288_edp_setmode(struct rockchip_connector *conn,
			      struct drm_display_mode *mode)
{
	struct rk3288_edp *edp = conn->priv;

	memcpy(&edp->mode, mode, sizeof(*mode));

	return 0;
}

static struct rockchip_connector edp_conn = {
	.enable = rk3288_edp_enable,
	.disable = rk3288_edp_disable,
	.setmode = rk3288_edp_setmode,
};

static struct rk3288_edp_soc_data soc_data[2] = {
	{.grf_soc_con6 = 0x025c,
	 .grf_soc_con12 = 0x0274},/*RK3288*/
	{.grf_soc_con6 = -1,
	 .grf_soc_con12 = -1},/* no lvds switching needed */
};

static const struct of_device_id rk3288_edp_dt_ids[] = {
	{.compatible = "rockchip,rk3288-edp",
	 .data = (void *)&soc_data[0] },
	{}
};
MODULE_DEVICE_TABLE(of, rk3288_edp_dt_ids);

static int rk3288_edp_probe(struct platform_device *pdev)
{
	struct rk3288_edp *edp;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	int ret = 0;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

	edp = devm_kzalloc(&pdev->dev, sizeof(struct rk3288_edp), GFP_KERNEL);
	if (!edp) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	match = of_match_node(rk3288_edp_dt_ids, np);
	edp->soc_data = (struct rk3288_edp_soc_data *)match->data;
	/*
	 * The control bit is located in the GRF register space.
	 */
	if (edp->soc_data->grf_soc_con6 >= 0) {
		edp->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
		if (IS_ERR(edp->grf)) {
			dev_err(&pdev->dev,
				"rk3288-edp needs rockchip,grf property\n");
			return PTR_ERR(edp->grf);
		}
	}

	edp->dev = &pdev->dev;
	edp->video_info.h_sync_polarity	= 0;
	edp->video_info.v_sync_polarity	= 0;
	edp->video_info.interlaced	= 0;
	edp->video_info.color_space	= CS_RGB;
	edp->video_info.dynamic_range	= VESA;
	edp->video_info.ycbcr_coeff	= COLOR_YCBCR601;
	edp->video_info.color_depth	= COLOR_8;

	edp->video_info.link_rate	= LINK_RATE_1_62GBPS;
	edp->video_info.lane_count	= LANE_CNT4;
	edp_conn.type = ROCKCHIP_DISPLAY_TYPE_EDP;
	edp_conn.priv = edp;
	edp_conn.dev = &pdev->dev;

	edp->base = rockchip_connector_register(&edp_conn);
	if (!edp->base) {
		dev_err(&pdev->dev, "connector register fail\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	edp->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(edp->regs)) {
		dev_err(&pdev->dev, "ioremap reg failed\n");
		return PTR_ERR(edp->regs);
	}

	edp->clk_edp = devm_clk_get(&pdev->dev, "clk_edp");
	if (IS_ERR(edp->clk_edp)) {
		dev_err(&pdev->dev, "cannot get clk_edp\n");
		return PTR_ERR(edp->clk_edp);
	}

	edp->clk_24m = devm_clk_get(&pdev->dev, "clk_edp_24m");
	if (IS_ERR(edp->clk_24m)) {
		dev_err(&pdev->dev, "cannot get clk_edp_24m\n");
		return PTR_ERR(edp->clk_24m);
	}

	edp->pclk = devm_clk_get(&pdev->dev, "pclk_edp");
	if (IS_ERR(edp->pclk)) {
		dev_err(&pdev->dev, "cannot get pclk\n");
		return PTR_ERR(edp->pclk);
	}

	edp->rst = devm_reset_control_get(&pdev->dev, "edp");
	if (IS_ERR(edp->rst)) {
		dev_err(&pdev->dev, "failed to get reset\n");
		return PTR_ERR(edp->rst);
	}

	ret = rk3288_edp_clk_enable(edp);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp clk %d\n", ret);
		return ret;
	}

	ret = rk3288_edp_pre_init(edp);
	if (ret < 0) {
		dev_err(edp->dev, "failed to pre init %d\n", ret);
		return ret;
	}

	edp->irq = platform_get_irq(pdev, 0);
	if (edp->irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return edp->irq;
	}

	ret = devm_request_irq(&pdev->dev, edp->irq, rk3288_edp_isr, 0,
			       dev_name(&pdev->dev), edp);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", edp->irq);
		return ret;
	}

	disable_irq_nosync(edp->irq);

	edp->standby = true;

	platform_set_drvdata(pdev, edp);
	dev_set_name(edp->dev, "rk3288-edp");

	dev_info(&pdev->dev, "rk3288 edp driver probe success\n");

	return 0;
}

static int rk3288_edp_remove(struct platform_device *pdev)
{
	rk3288_edp_disable(&edp_conn);

	return 0;
}

struct platform_driver rk3288_edp_driver = {
	.probe = rk3288_edp_probe,
	.remove = rk3288_edp_remove,
	.driver = {
		   .name = "rk3288-edp",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rk3288_edp_dt_ids),
	},
};
