/* rockchip_drm_dp.c
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_dpi.c
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

#include <linux/regulator/consumer.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_dp.h"
#include "rockchip_drm_lcdc.h"

struct rockchip_dp_context {
	struct device *dev;

	struct rockchip_dp *dp;
	struct drm_panel *panel;
	struct drm_connector connector;
	struct drm_encoder *encoder;

	struct drm_display_mode mode;

	int dpms_mode;
};

#define connector_to_dp(c) \
		container_of(c, struct rockchip_dp_context, connector)

static enum drm_connector_status
rockchip_dp_detect(struct drm_connector *connector, bool force)
{
	return true;
}

static void rockchip_dp_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs rockchip_dp_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rockchip_dp_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = rockchip_dp_connector_destroy,
};

static int rockchip_dp_get_modes(struct drm_connector *connector)
{
	struct rockchip_dp_context *ctx = connector_to_dp(connector);
	struct drm_display_mode *mode;
	const struct drm_display_mode *priv_mode = &ctx->mode;

	mode = drm_mode_duplicate(connector->dev, priv_mode);

	if (mode) {
		drm_mode_probed_add(connector, mode);
		return 1;
	}

	return 0;
}

static struct drm_encoder *
	rockchip_dp_best_encoder(struct drm_connector *connector)
{
	struct rockchip_dp_context *ctx = connector_to_dp(connector);

	return ctx->encoder;
}

static struct drm_connector_helper_funcs rockchip_dp_connector_helper_funcs = {
	.get_modes = rockchip_dp_get_modes,
	.best_encoder = rockchip_dp_best_encoder,
};

static int rockchip_dp_create_connector(struct rockchip_drm_display *display,
					struct drm_encoder *encoder)
{
	struct rockchip_dp_context *ctx = display->ctx;
	struct drm_connector *connector = &ctx->connector;
	int ret;

	ctx->encoder = encoder;

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(encoder->dev, connector,
				 &rockchip_dp_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector,
				 &rockchip_dp_connector_helper_funcs);
	drm_sysfs_connector_add(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

	return 0;
}

static void rockchip_dp_poweron(struct rockchip_dp_context *ctx)
{
	if (ctx->dp)
		ctx->dp->enable(ctx->dp);
}

static void rockchip_dp_poweroff(struct rockchip_dp_context *ctx)
{
	if (ctx->dp)
		ctx->dp->disable(ctx->dp);
}

static void rockchip_dp_dpms(struct rockchip_drm_display *display, int mode)
{
	struct rockchip_dp_context *ctx = display->ctx;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (ctx->dpms_mode != DRM_MODE_DPMS_ON)
				rockchip_dp_poweron(ctx);
			break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (ctx->dpms_mode == DRM_MODE_DPMS_ON)
			rockchip_dp_poweroff(ctx);
		break;
	default:
		break;
	}
	ctx->dpms_mode = mode;
}

static int rockchip_check_mode(struct rockchip_drm_display *display,
			       struct drm_display_mode *mode)
{
	return MODE_OK;
}
static struct rockchip_drm_display_ops rockchip_dp_display_ops = {
	.create_connector = rockchip_dp_create_connector,
	.check_mode = rockchip_check_mode,
	.dpms = rockchip_dp_dpms,
};

static struct rockchip_drm_display rockchip_dp_display = {
	.type = ROCKCHIP_DISPLAY_TYPE_LCD,
	.ops = &rockchip_dp_display_ops,
};

static int rockchip_dp_parse_dt(struct rockchip_dp_context *ctx)
{
	struct device *dev = ctx->dev;
	struct device_node *dn = dev->of_node;
	struct device_node *np;
	struct videomode vm;
	struct rockchip_mode_priv *priv;
	u32 val;
	int ret;
	np = of_get_child_by_name(dn, "display-timings");
	if (!np) {
		DRM_ERROR("can't find display timings\n");
		return 0;
	}

	of_node_put(np);

	memset(&vm, 0, sizeof(vm));

	ret = of_get_videomode(dn, &vm, 0);
	if (ret < 0)
		return ret;
	drm_display_mode_from_videomode(&vm, &ctx->mode);

	ctx->mode.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	priv = devm_kzalloc(dev,sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		DRM_ERROR("failed to create private mode data\n");
		return -ENOMEM;
	}

	if (of_property_read_u32(dn, "rockchip,type", &val))
		priv->type = 0;
	else
		priv->type = val;

	if (of_property_read_u32(dn, "rockchip,face", &val))
		priv->face = 0;
	else
		priv->face = val;

	if (of_property_read_u32(dn, "rockchip,lvds-format", &val))
		priv->lvds_format = 0;
	else
		priv->lvds_format = val;
	if (of_property_read_u32(dn, "rockchip,color-swap", &val))
		priv->color_swap = 0;
	else
		priv->color_swap = val;
	priv->lcdc_id = 1;
	priv->flags = vm.flags;
	ctx->mode.private = (void *)priv;

	return 0;
}

struct rockchip_drm_display *rockchip_dp_probe(struct device *dev)
{
	struct rockchip_dp_context *ctx;
	int ret;

	ret = rockchip_drm_component_add(dev,
					 ROCKCHIP_DEVICE_TYPE_CONNECTOR,
					 rockchip_dp_display.type);
	if (ret)
		return ERR_PTR(ret);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		goto err_del_component;

	ctx->dev = dev;
	rockchip_dp_display.ctx = ctx;
	ctx->dpms_mode = DRM_MODE_DPMS_OFF;

	ret = rockchip_dp_parse_dt(ctx);
	if (ret < 0) {
		dev_err(dev, "parse dp devices tree failed %d\n", ret);
		goto err_del_component;
	}

	return &rockchip_dp_display;

err_del_component:
	rockchip_drm_component_del(dev, ROCKCHIP_DEVICE_TYPE_CONNECTOR);

	return NULL;
}

int rockchip_dp_remove(struct device *dev)
{
	struct drm_encoder *encoder = rockchip_dp_display.encoder;
	struct rockchip_dp_context *ctx = rockchip_dp_display.ctx;

	rockchip_dp_dpms(&rockchip_dp_display, DRM_MODE_DPMS_OFF);
	encoder->funcs->destroy(encoder);
	drm_connector_cleanup(&ctx->connector);

	rockchip_drm_component_del(dev, ROCKCHIP_DEVICE_TYPE_CONNECTOR);

	return 0;
}

int rockchip_dp_register(struct rockchip_dp *dp)
{
	struct rockchip_dp_context *ctx = rockchip_dp_display.ctx;
	struct rockchip_mode_priv *priv;

	if (!(ctx)) {
		DRM_ERROR("dp controller have not init, register later\n");
		return -1;
	}

	priv = (void *)ctx->mode.private;

	if (priv->type != dp->type)
		return -1;

	if (ctx->dp)
		DRM_ERROR("dp type(%d) already registered\n", dp->type);
	ctx->dp = dp;
	if (ctx->dp->setmode)
		ctx->dp->setmode(dp, &ctx->mode);
	DRM_DEBUG_KMS("succes register dp type=%d\n", dp->type);

	return 0;
}
