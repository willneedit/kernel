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

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>

#include <linux/component.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include "rockchip_edp_core.h"

#define connector_to_edp(c) \
		container_of(c, struct rockchip_edp_device, connector)

#define encoder_to_edp(c) \
		container_of(c, struct rockchip_edp_device, encoder)

static struct rockchip_edp_soc_data soc_data[2] = {
	/* rk3288 */
	{.grf_soc_con6 = 0x025c,
	 .grf_soc_con12 = 0x0274},
	/* no edp switching needed */
	{.grf_soc_con6 = -1,
	 .grf_soc_con12 = -1},
};

static const struct of_device_id rockchip_edp_dt_ids[] = {
	{.compatible = "rockchip,rk3288-edp",
	 .data = (void *)&soc_data[0] },
	{}
};

MODULE_DEVICE_TABLE(of, rockchip_edp_dt_ids);

static int rockchip_edp_clk_enable(struct rockchip_edp_device *edp)
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
			dev_err(edp->dev, "cannot set edp clk_24m parent %d\n",
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

static int rockchip_edp_clk_disable(struct rockchip_edp_device *edp)
{
	if (edp->clk_on) {
		clk_disable_unprepare(edp->pclk);
		clk_disable_unprepare(edp->clk_edp);
		clk_disable_unprepare(edp->clk_24m);
		edp->clk_on = false;
	}

	return 0;
}

static int rockchip_edp_pre_init(struct rockchip_edp_device *edp)
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
	usleep_range(1000, 2000);
	reset_control_deassert(edp->rst);

	return 0;
}

static int rockchip_edp_init_edp(struct rockchip_edp_device *edp)
{
	rockchip_edp_reset(edp);
	rockchip_edp_init_refclk(edp);
	rockchip_edp_init_interrupt(edp);
	rockchip_edp_enable_sw_function(edp);
	rockchip_edp_init_analog_func(edp);
	rockchip_edp_init_hpd(edp);
	rockchip_edp_init_aux(edp);

	return 0;
}

static int rockchip_edp_get_max_rx_bandwidth(
					struct rockchip_edp_device *edp,
					u8 *bandwidth)
{
	u8 data;
	int retval = 0;

	/*
	 * For DP rev.1.1, Maximum link rate of Main Link lanes
	 * 0x06 = 1.62 Gbps, 0x0a = 2.7 Gbps
	 */
	retval = rockchip_edp_read_byte_from_dpcd(
			edp, DPCD_ADDR_MAX_LINK_RATE, &data);
	if (retval < 0)
		*bandwidth = 0;
	else
		*bandwidth = data;

	return retval;
}

static int rockchip_edp_get_max_rx_lane_count(struct rockchip_edp_device *edp,
					    u8 *lane_count)
{
	u8 data;
	int retval;

	/*
	 * For DP rev.1.1, Maximum number of Main Link lanes
	 * 0x01 = 1 lane, 0x02 = 2 lanes, 0x04 = 4 lanes
	 */
	retval = rockchip_edp_read_byte_from_dpcd(
			edp, DPCD_ADDR_MAX_LANE_COUNT, &data);
	if (retval < 0)
		*lane_count = 0;
	else
		*lane_count = DPCD_MAX_LANE_COUNT(data);

	return retval;
}

static int rockchip_edp_init_training(struct rockchip_edp_device *edp)
{
	int retval;

	/*
	 * MACRO_RST must be applied after the PLL_LOCK to avoid
	 * the DP inter pair skew issue for at least 10 us
	 */
	rockchip_edp_reset_macro(edp);

	retval = rockchip_edp_get_max_rx_bandwidth(
				edp, &edp->link_train.link_rate);
	retval = rockchip_edp_get_max_rx_lane_count(
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

	rockchip_edp_analog_power_ctr(edp, 1);

	return 0;
}

static int rockchip_edp_hw_link_training(struct rockchip_edp_device *edp)
{
	u32 cnt = 50;
	u32 val;

	/* Set link rate and count as you want to establish*/
	rockchip_edp_set_link_bandwidth(edp, edp->link_train.link_rate);
	rockchip_edp_set_lane_count(edp, edp->link_train.lane_count);
	rockchip_edp_hw_link_training_en(edp);
	val = rockchip_edp_wait_hw_lt_done(edp);
	while (val) {
		if (cnt-- <= 0) {
			dev_err(edp->dev, "hw lt timeout");
			return -ETIMEDOUT;
		}
		mdelay(100);
		val = rockchip_edp_wait_hw_lt_done(edp);
	}

	val = rockchip_edp_get_hw_lt_status(edp);
	if (val)
		dev_err(edp->dev, "hw lt err:%d\n", val);

	return val;
}

static int rockchip_edp_set_link_train(struct rockchip_edp_device *edp)
{
	int retval;

	rockchip_edp_init_training(edp);

	retval = rockchip_edp_hw_link_training(edp);
	if (retval < 0)
		dev_err(edp->dev, "DP hw LT failed!\n");

	return retval;
}

static int rockchip_edp_config_video(struct rockchip_edp_device *edp,
				   struct video_info *video_info)
{
	int retval = 0;
	int timeout_loop = 0;
	int done_count = 0;

	rockchip_edp_config_video_slave_mode(edp, video_info);

	rockchip_edp_set_video_color_format(edp, video_info->color_depth,
					    video_info->color_space,
					    video_info->dynamic_range,
					    video_info->ycbcr_coeff);

	if (rockchip_edp_get_pll_lock_status(edp) == DP_PLL_UNLOCKED) {
		dev_err(edp->dev, "PLL is not locked yet.\n");
		return -EINVAL;
	}

	for (;;) {
		timeout_loop++;
		if (rockchip_edp_is_slave_video_stream_clock_on(edp) == 0)
			break;

		if (DP_TIMEOUT_LOOP_CNT < timeout_loop) {
			dev_err(edp->dev, "Timeout of video streamclk ok\n");
			return -ETIMEDOUT;
		}

		udelay(100);
	}

	/* Set to use the register calculated M/N video */
	rockchip_edp_set_video_cr_mn(edp, CALCULATED_M, 0, 0);

	/* Disable video mute */
	rockchip_edp_enable_video_mute(edp, 0);

	/* Configure video slave mode */
	rockchip_edp_enable_video_master(edp, 0);

	/* Enable video */
	rockchip_edp_start_video(edp);

	timeout_loop = 0;

	for (;;) {
		timeout_loop++;
		if (rockchip_edp_is_video_stream_on(edp) == 0) {
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

		mdelay(100);
	}

	if (retval != 0)
		dev_err(edp->dev, "Video stream is not detected!\n");

	return retval;
}

static irqreturn_t rockchip_edp_isr(int irq, void *arg)
{
	struct rockchip_edp_device *edp = arg;
	enum dp_irq_type irq_type;

	irq_type = rockchip_edp_get_irq_type(edp);
	switch (irq_type) {
	case DP_IRQ_TYPE_HP_CABLE_IN:
		dev_dbg(edp->dev, "Received irq - cable in\n");
		rockchip_edp_clear_hotplug_interrupts(edp);
		break;
	case DP_IRQ_TYPE_HP_CABLE_OUT:
		dev_dbg(edp->dev, "Received irq - cable out\n");
		rockchip_edp_clear_hotplug_interrupts(edp);
		break;
	case DP_IRQ_TYPE_HP_CHANGE:
		/*
		 * We get these change notifications once in a while, but there
		 * is nothing we can do with them. Just ignore it for now and
		 * only handle cable changes.
		 */
		dev_dbg(edp->dev, "Received irq - hotplug change; ignoring.\n");
		rockchip_edp_clear_hotplug_interrupts(edp);
		break;
	default:
		dev_err(edp->dev, "Received irq - unknown type[%x]!\n",
			irq_type);
		rockchip_edp_clear_hotplug_interrupts(edp);
		break;
	}

	return IRQ_HANDLED;
}

static void rockchip_edp_commit(struct drm_encoder *encoder)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);
	int ret = 0;

	ret = rockchip_edp_set_link_train(edp);
	if (ret)
		dev_err(edp->dev, "link train failed!\n");
	else
		dev_dbg(edp->dev, "link training success.\n");

	rockchip_edp_set_lane_count(edp, edp->link_train.lane_count);
	rockchip_edp_set_link_bandwidth(edp, edp->link_train.link_rate);
	rockchip_edp_init_video(edp);

	ret = rockchip_edp_config_video(edp, &edp->video_info);
	if (ret)
		dev_err(edp->dev, "unable to config video\n");
}

static void rockchip_edp_poweron(struct drm_encoder *encoder)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);
	int ret = 0;

	if (edp->dpms_mode == DRM_MODE_DPMS_ON)
		return;

	if (edp->panel)
		edp->panel->funcs->enable(edp->panel);

	ret = rockchip_edp_clk_enable(edp);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp clk %d\n", ret);
		return;
	}

	ret = rockchip_edp_pre_init(edp);
	if (ret < 0) {
		dev_err(edp->dev, "edp pre init fail %d\n", ret);
		return;
	}

	ret = rockchip_edp_init_edp(edp);
	if (ret < 0) {
		dev_err(edp->dev, "edp init fail %d\n", ret);
		return;
	}

	enable_irq(edp->irq);
	rockchip_edp_commit(encoder);
}

static void rockchip_edp_poweroff(struct drm_encoder *encoder)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);

	if (edp->dpms_mode == DRM_MODE_DPMS_OFF)
		return;

	disable_irq(edp->irq);
	rockchip_edp_reset(edp);
	rockchip_edp_analog_power_ctr(edp, 0);
	rockchip_edp_clk_disable(edp);
	if (edp->panel)
		edp->panel->funcs->disable(edp->panel);
}

static enum drm_connector_status
rockchip_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void rockchip_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs rockchip_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rockchip_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = rockchip_connector_destroy,
};

static int rockchip_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_edp_device *edp = connector_to_edp(connector);
	struct drm_panel *panel = edp->panel;

	return panel->funcs->get_modes(panel);
}

static struct drm_encoder *
	rockchip_connector_best_encoder(struct drm_connector *connector)
{
	struct rockchip_edp_device *edp = connector_to_edp(connector);

	return &edp->encoder;
}

static enum drm_mode_status rockchip_connector_mode_valid(
		struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	/* TODO(rk): verify that the mode is really valid */
	return MODE_OK;
}

static struct drm_connector_helper_funcs rockchip_connector_helper_funcs = {
	.get_modes = rockchip_connector_get_modes,
	.mode_valid = rockchip_connector_mode_valid,
	.best_encoder = rockchip_connector_best_encoder,
};

static void rockchip_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);

	if (edp->dpms_mode == mode)
		return;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		rockchip_edp_poweron(encoder);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		rockchip_edp_poweroff(encoder);
		break;
	default:
		break;
	}

	edp->dpms_mode = mode;
}

static bool
rockchip_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	if (!adjusted_mode->private) {
		struct rockchip_display_mode *priv_mode;

		priv_mode = kzalloc(sizeof(*priv_mode), GFP_KERNEL);
		priv_mode->out_type = ROCKCHIP_DISPLAY_TYPE_EDP;
		adjusted_mode->private = (int *)priv_mode;
	}

	return true;
}

static void rockchip_drm_encoder_mode_set(struct drm_encoder *encoder,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adjusted)
{
	struct rockchip_edp_device *edp = encoder_to_edp(encoder);
	struct rockchip_drm_private *private = edp->drm_dev->dev_private;
	struct rockchip_drm_crtc *rk_crtc;
	u32 val;
	int ret;
	int index;

	index = drm_crtc_index(encoder->crtc);

	rk_crtc = &private->rk_crtc[index];

	if (rk_crtc->id == ROCKCHIP_CRTC_VOPL)
		val = EDP_SEL_VOP_LIT | (EDP_SEL_VOP_LIT << 16);
	else
		val = EDP_SEL_VOP_LIT << 16;

	ret = regmap_write(edp->grf, edp->soc_data->grf_soc_con6, val);
	if (ret != 0)
		dev_err(edp->dev, "Could not write to GRF: %d\n", ret);

	memcpy(&edp->mode, adjusted, sizeof(*mode));
}

static void rockchip_drm_encoder_prepare(struct drm_encoder *encoder)
{
}

static void rockchip_drm_encoder_commit(struct drm_encoder *encoder)
{
	/*rockchip_edp_commit(encoder);*/
}

static void rockchip_drm_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_plane *plane;
	struct drm_device *dev = encoder->dev;

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);

	/* all planes connected to this encoder should be also disabled. */
	list_for_each_entry(plane, &dev->mode_config.plane_list, head) {
		if (plane->crtc && (plane->crtc == encoder->crtc))
			plane->funcs->disable_plane(plane);
	}
}


static struct drm_encoder_helper_funcs rockchip_encoder_helper_funcs = {
	.dpms = rockchip_drm_encoder_dpms,
	.mode_fixup = rockchip_drm_encoder_mode_fixup,
	.mode_set = rockchip_drm_encoder_mode_set,
	.prepare = rockchip_drm_encoder_prepare,
	.commit = rockchip_drm_encoder_commit,
	.disable = rockchip_drm_encoder_disable,
};
static void rockchip_drm_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static struct drm_encoder_funcs rockchip_encoder_funcs = {
	.destroy = rockchip_drm_encoder_destroy,
};

static int rockchip_edp_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct rockchip_edp_device *edp = dev_get_drvdata(dev);
	struct device_node *panel_node;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct drm_device *drm_dev = data;
	int ret;

	edp->drm_dev = drm_dev;

	panel_node = of_parse_phandle(edp->dev->of_node,
				      "rockchip,panel", 0);
	if (!panel_node) {
		DRM_ERROR("failed to find diaplay panel\n");
		return -ENODEV;
	}

	edp->panel = of_drm_find_panel(panel_node);
	if (!edp->panel) {
		DRM_ERROR("failed to find diaplay panel\n");
		of_node_put(panel_node);
		return -EPROBE_DEFER;
	}

	of_node_put(panel_node);

	encoder = &edp->encoder;
	encoder->possible_crtcs = 1 << 0;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_encoder_funcs,
			       DRM_MODE_ENCODER_LVDS);
	if (ret) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &rockchip_encoder_helper_funcs);

	connector = &edp->connector;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm_dev, connector,
				 &rockchip_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err_free_encoder;
	}

	drm_connector_helper_add(connector,
				 &rockchip_connector_helper_funcs);

	ret = drm_sysfs_connector_add(connector);
	if (ret) {
		DRM_ERROR("failed to add drm_sysfs\n");
		goto err_free_connector;
	}

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector_sysfs;
	}

	ret = drm_panel_attach(edp->panel, connector);
	if (ret) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector_sysfs;
	}

	return 0;

err_free_connector_sysfs:
	drm_sysfs_connector_remove(connector);
err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void rockchip_edp_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct rockchip_edp_device *edp = dev_get_drvdata(dev);
	struct drm_encoder *encoder;

	encoder = &edp->encoder;

	if (edp->panel)
		drm_panel_detach(edp->panel);

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	encoder->funcs->destroy(encoder);
	drm_sysfs_connector_remove(&edp->connector);
	drm_connector_cleanup(&edp->connector);
	drm_encoder_cleanup(encoder);
}

static const struct component_ops rockchip_edp_component_ops = {
	.bind = rockchip_edp_bind,
	.unbind = rockchip_edp_unbind,
};

static int rockchip_edp_probe(struct platform_device *pdev)
{
	struct rockchip_edp_device *edp;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	int ret = 0;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

	edp = devm_kzalloc(&pdev->dev, sizeof(*edp), GFP_KERNEL);
	if (!edp) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	match = of_match_node(rockchip_edp_dt_ids, np);
	edp->soc_data = (struct rockchip_edp_soc_data *)match->data;
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

	ret = rockchip_edp_clk_enable(edp);
	if (ret < 0) {
		dev_err(edp->dev, "cannot enable edp clk %d\n", ret);
		return ret;
	}

	ret = rockchip_edp_pre_init(edp);
	if (ret < 0) {
		dev_err(edp->dev, "failed to pre init %d\n", ret);
		return ret;
	}

	edp->irq = platform_get_irq(pdev, 0);
	if (edp->irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		return edp->irq;
	}

	ret = devm_request_irq(&pdev->dev, edp->irq, rockchip_edp_isr, 0,
			       dev_name(&pdev->dev), edp);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", edp->irq);
		return ret;
	}

	disable_irq_nosync(edp->irq);

	edp->dpms_mode = DRM_MODE_DPMS_OFF;

	platform_set_drvdata(pdev, edp);

	ret = rockchip_drm_component_add(edp->dev, &rockchip_edp_component_ops);
	if (ret)
		return -EINVAL;

	dev_set_name(edp->dev, "rockchip-edp");
	dev_info(&pdev->dev, "rockchip edp driver probe success\n");

	return 0;
}

static int rockchip_edp_remove(struct platform_device *pdev)
{
	struct rockchip_edp_device *edp = platform_get_drvdata(pdev);
	struct drm_encoder *encoder = &edp->encoder;

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);

	rockchip_drm_component_del(edp->dev);
	return 0;
}

struct platform_driver rockchip_edp_driver = {
	.probe = rockchip_edp_probe,
	.remove = rockchip_edp_remove,
	.driver = {
		   .name = "rockchip-edp",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_edp_dt_ids),
	},
};
