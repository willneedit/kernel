/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>

#include <linux/component.h>
#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include "rockchip_drm_drv.h"

#define LVDS_CH0_REG_0			0x00
#define LVDS_CH0_REG_1			0x04
#define LVDS_CH0_REG_2			0x08
#define LVDS_CH0_REG_3			0x0c
#define LVDS_CH0_REG_4			0x10
#define LVDS_CH0_REG_5			0x14
#define LVDS_CH0_REG_9			0x24
#define LVDS_CFG_REG_C			0x30
#define LVDS_CH0_REG_D			0x34
#define LVDS_CH0_REG_F			0x3c
#define LVDS_CH0_REG_20			0x80
#define LVDS_CFG_REG_21			0x84

#define LVDS_SEL_VOP_LIT		(1 << 3)

#define LVDS_FMT_MASK			(0x07 << 16)
#define LVDS_MSB			(0x01 << 3)
#define LVDS_DUAL			(0x01 << 4)
#define LVDS_FMT_1			(0x01 << 5)
#define LVDS_TTL_EN			(0x01 << 6)
#define LVDS_START_PHASE_RST_1		(0x01 << 7)
#define LVDS_DCLK_INV			(0x01 << 8)
#define LVDS_CH0_EN			(0x01 << 11)
#define LVDS_CH1_EN			(0x01 << 12)
#define LVDS_PWRDN			(0x01 << 15)

#define LVDS_24BIT		(0 << 1)
#define LVDS_18BIT		(1 << 1)
#define LVDS_FORMAT_VESA	(0 << 0)
#define LVDS_FORMAT_JEIDA	(1 << 0)

#define connector_to_ctx(c) \
		container_of(c, struct lvds_context, connector)

#define encoder_to_ctx(c) \
		container_of(c, struct lvds_context, encoder)

/*
 * @grf_offset: offset inside the grf regmap for setting the rockchip lvds
 */
struct rockchip_lvds_soc_data {
	int grf_gpio_iomux;
	int grf_vop_sel;
	int grf_lvds_ctrl;
};

static struct rockchip_lvds_soc_data soc_data[2] = {
	{.grf_gpio_iomux = 0x000c,
	 .grf_vop_sel = 0x025c,
	 .grf_lvds_ctrl = 0x0260},
};

static const struct of_device_id rockchip_lvds_dt_ids[] = {
	{.compatible = "rockchip,rk3288-lvds",
	 .data = (void *)&soc_data[0] },
	{}
};

static int lvds_name_to_format(const char *s)
{
	if (!s)
		return 0;

	if (strncmp(s, "jeida", 6) == 0)
		return LVDS_FORMAT_JEIDA;
	else if (strncmp(s, "vesa", 6) == 0)
		return LVDS_FORMAT_VESA;

	return 0;
}

struct lvds_context {
	struct device *dev;
	struct drm_device *drm_dev;

	struct drm_panel *panel;
	struct drm_connector connector;
	struct drm_encoder encoder;

	int format;
	struct drm_display_mode mode;

	struct rockchip_lvds_soc_data *soc_data;
	void __iomem *regs;
	struct regmap *grf;
	struct clk *pclk;

	int dpms_mode;
};

static inline void lvds_writel(struct lvds_context *lvds, u32 offset, u32 val)
{
	writel_relaxed(val, lvds->regs + offset);
	writel_relaxed(val, lvds->regs + offset + 0x100);
}

static void rockchip_lvds_disable(struct lvds_context *ctx)
{
	int ret = 0;

	ret = regmap_write(ctx->grf, ctx->soc_data->grf_lvds_ctrl, 0xffff8000);
	if (ret != 0)
		dev_err(ctx->dev, "Could not write to GRF: %d\n", ret);

	/* disable tx */
	writel_relaxed(0x00, ctx->regs + LVDS_CFG_REG_21);
	/* disable pll */
	writel_relaxed(0xff, ctx->regs + LVDS_CFG_REG_C);

	clk_disable_unprepare(ctx->pclk);
}

static void rockchip_lvds_enable(struct lvds_context *ctx)
{
	int ret;

	ret = clk_prepare_enable(ctx->pclk);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to enable lvds pclk %d\n", ret);
		return;
	}

	lvds_writel(ctx, LVDS_CH0_REG_0, 0xbf);
	lvds_writel(ctx, LVDS_CH0_REG_1, 0x3f);
	lvds_writel(ctx, LVDS_CH0_REG_2, 0xfe);
	lvds_writel(ctx, LVDS_CH0_REG_3, 0x46);
	lvds_writel(ctx, LVDS_CH0_REG_4, 0x00);
	lvds_writel(ctx, LVDS_CH0_REG_D, 0x0a);
	lvds_writel(ctx, LVDS_CH0_REG_20, 0x44);
	writel_relaxed(0x00, ctx->regs + LVDS_CFG_REG_C);
	writel_relaxed(0x92, ctx->regs + LVDS_CFG_REG_21);
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
	struct lvds_context *ctx = connector_to_ctx(connector);
	struct drm_panel *panel = ctx->panel;

	return panel->funcs->get_modes(panel);
}

static struct drm_encoder *
	rockchip_connector_best_encoder(struct drm_connector *connector)
{
	struct lvds_context *ctx = connector_to_ctx(connector);

	return &ctx->encoder;
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
	struct lvds_context *ctx = encoder_to_ctx(encoder);

	if (ctx->dpms_mode == mode)
		return;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (ctx->panel)
			ctx->panel->funcs->enable(ctx->panel);
		rockchip_lvds_enable(ctx);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		rockchip_lvds_disable(ctx);
		if (ctx->panel)
			ctx->panel->funcs->disable(ctx->panel);
		break;
	default:
		break;
	}

	ctx->dpms_mode = mode;
}

static bool
rockchip_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	if (!adjusted_mode->private) {
		struct rockchip_display_mode *priv_mode;

		priv_mode = kzalloc(sizeof(*priv_mode), GFP_KERNEL);
		priv_mode->out_type = ROCKCHIP_DISPLAY_TYPE_LVDS;
		adjusted_mode->private = (int *)priv_mode;
	}

	return true;
}

static void rockchip_drm_encoder_mode_set(struct drm_encoder *encoder,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adjusted)
{
	struct lvds_context *ctx = encoder_to_ctx(encoder);
	struct rockchip_drm_private *private = ctx->drm_dev->dev_private;
	struct rockchip_drm_crtc *rk_crtc;
	u32 val;
	int ret;
	int index;

	index = drm_crtc_index(encoder->crtc);
	rk_crtc = &private->rk_crtc[index];

	if (rk_crtc->id == ROCKCHIP_CRTC_VOPL)
		val = LVDS_SEL_VOP_LIT | (LVDS_SEL_VOP_LIT << 16);
	else
		val = LVDS_SEL_VOP_LIT << 16;

	ret = regmap_write(ctx->grf, ctx->soc_data->grf_vop_sel, val);
	if (ret != 0) {
		dev_err(ctx->dev, "Could not write to GRF: %d\n", ret);
		return;
	}

	val = ctx->format;
	val |= LVDS_CH0_EN;
	val |= (1 << 8);
	val |= (0xffff << 16);
	ret = regmap_write(ctx->grf, ctx->soc_data->grf_lvds_ctrl, val);
	if (ret != 0) {
		dev_err(ctx->dev, "Could not write to GRF: %d\n", ret);
		return;
	}
}

static void rockchip_drm_encoder_prepare(struct drm_encoder *encoder)
{
	/* drm framework doesn't check NULL. */
}

static void rockchip_drm_encoder_commit(struct drm_encoder *encoder)
{
	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
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

static int rockchip_lvds_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct lvds_context *ctx = dev_get_drvdata(dev);
	struct device_node *panel_node;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct drm_device *drm_dev = data;
	int ret;

	ctx->drm_dev = drm_dev;

	panel_node = of_parse_phandle(ctx->dev->of_node,
			"rockchip,panel", 0);
	if (!panel_node) {
		DRM_ERROR("failed to find diaplay panel\n");
		return -ENODEV;
	}

	ctx->panel = of_drm_find_panel(panel_node);
	if (!ctx->panel) {
		DRM_ERROR("failed to find diaplay panel\n");
		of_node_put(panel_node);
		return -EPROBE_DEFER;
	}

	of_node_put(panel_node);

	encoder = &ctx->encoder;
	encoder->possible_crtcs = 1 << 0;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_encoder_funcs,
			       DRM_MODE_ENCODER_LVDS);
	if (ret) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &rockchip_encoder_helper_funcs);

	connector = &ctx->connector;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm_dev, connector,
				 &rockchip_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
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

	ret = drm_panel_attach(ctx->panel, connector);
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

static void rockchip_lvds_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct lvds_context *ctx = dev_get_drvdata(dev);
	struct drm_encoder *encoder;

	encoder = &ctx->encoder;

	if (ctx->panel)
		drm_panel_detach(ctx->panel);

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	encoder->funcs->destroy(encoder);
	drm_sysfs_connector_remove(&ctx->connector);
	drm_connector_cleanup(&ctx->connector);
	drm_encoder_cleanup(encoder);
}

static const struct component_ops rockchip_lvds_component_ops = {
	.bind = rockchip_lvds_bind,
	.unbind = rockchip_lvds_unbind,
};

static int rockchip_lvds_probe(struct platform_device *pdev)
{
	struct lvds_context *ctx;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;
	const char *name;
	const struct of_device_id *match;
	int ret;
	u32 i;

	if (!np) {
		dev_err(&pdev->dev, "Missing device tree node.\n");
		return -EINVAL;
	}

	ctx = devm_kzalloc(&pdev->dev, sizeof(struct lvds_context), GFP_KERNEL);
	if (!ctx) {
		dev_err(&pdev->dev, "no memory for state\n");
		return -ENOMEM;
	}

	match = of_match_node(rockchip_lvds_dt_ids, np);
	ctx->soc_data = (struct rockchip_lvds_soc_data *)match->data;
	ctx->dev = &pdev->dev;
	/*
	 * The control bit is located in the GRF register space.
	 */
	if (ctx->soc_data->grf_gpio_iomux >= 0 ||
	    ctx->soc_data->grf_vop_sel >= 0 ||
	    ctx->soc_data->grf_lvds_ctrl >= 0) {
		ctx->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
		if (IS_ERR(ctx->grf)) {
			dev_err(&pdev->dev,
				"rockchip-lvds needs rockchip,grf property\n");
			return PTR_ERR(ctx->grf);
		}
	}

	if (of_property_read_string(np, "rockchip,data-mapping", &name))
		/* default set it as format jeida */
		ctx->format = LVDS_FORMAT_JEIDA;
	else
		ctx->format = lvds_name_to_format(name);

	if (of_property_read_u32(np, "rockchip,data-width", &i)) {
		ctx->format |= LVDS_24BIT;
	} else {
		if (i == 24) {
			ctx->format |= LVDS_24BIT;
		} else if (i == 18) {
			ctx->format |= LVDS_18BIT;
		} else {
			dev_err(&pdev->dev,
				"rockchip-lvds unsupport data-width[%d]\n", i);
			return -EINVAL;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->regs)) {
		dev_err(&pdev->dev, "ioremap reg failed\n");
		return PTR_ERR(ctx->regs);
	}

	ctx->pclk = devm_clk_get(&pdev->dev, "pclk_lvds");
	if (IS_ERR(ctx->pclk)) {
		dev_err(&pdev->dev, "get clk failed\n");
		return PTR_ERR(ctx->pclk);
	}

	platform_set_drvdata(pdev, ctx);

	ctx->dpms_mode = DRM_MODE_DPMS_OFF;

	ret = rockchip_drm_component_add(ctx->dev,
					 &rockchip_lvds_component_ops);
	if (ret)
		return -EINVAL;

	dev_info(&pdev->dev, "rockchip lvds driver probe success\n");

	return 0;
}

static int rockchip_lvds_remove(struct platform_device *pdev)
{
	struct lvds_context *ctx = platform_get_drvdata(pdev);

	rockchip_lvds_disable(ctx);

	rockchip_drm_component_del(ctx->dev);
	return 0;
}

struct platform_driver rockchip_lvds_driver = {
	.probe = rockchip_lvds_probe,
	.remove = rockchip_lvds_remove,
	.driver = {
		   .name = "rockchip-lvds",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rockchip_lvds_dt_ids),
	},
};
