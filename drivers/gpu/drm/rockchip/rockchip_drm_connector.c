/*
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
#include <drm/drm_edid.h>

#include <linux/component.h>

#include <video/of_videomode.h>
#include <video/videomode.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_connector.h"
#include "rockchip_drm_lcdc.h"

struct rockchip_conn_context {
	struct device *dev;
	struct drm_device *drm_dev;

	struct drm_panel *panel;
	struct rockchip_connector *conn;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct rockchip_panel_special priv_mode;

	u32 flags;
	int type;

	int dpms_mode;
};

#define connector_to_ctx(c) \
		container_of(c, struct rockchip_conn_context, connector)

#define encoder_to_ctx(c) \
		container_of(c, struct rockchip_conn_context, encoder)

static inline int rockchip_convert_conn_type(int type)
{
	switch (type) {
	case ROCKCHIP_DISPLAY_TYPE_RGB:
	case ROCKCHIP_DISPLAY_TYPE_LVDS:
		return DRM_MODE_CONNECTOR_LVDS;
	case ROCKCHIP_DISPLAY_TYPE_EDP:
		return DRM_MODE_CONNECTOR_eDP;
	case ROCKCHIP_DISPLAY_TYPE_HDMI:
		return DRM_MODE_CONNECTOR_HDMIA;
	}

	return  DRM_MODE_CONNECTOR_Unknown;
}

static inline int rockchip_convert_encoder_type(int type)
{
	switch (type) {
	case ROCKCHIP_DISPLAY_TYPE_RGB:
	case ROCKCHIP_DISPLAY_TYPE_LVDS:
	case ROCKCHIP_DISPLAY_TYPE_EDP:
		return DRM_MODE_ENCODER_LVDS;
	}

	return DRM_MODE_ENCODER_NONE;
}

static enum drm_connector_status
rockchip_conn_detect(struct drm_connector *connector, bool force)
{
	struct rockchip_conn_context *ctx = connector_to_ctx(connector);
	struct rockchip_connector *conn = ctx->conn;

	if (conn->is_detect)
		return conn->is_detect(conn);
	else
		return true;
}

static void rockchip_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs rockchip_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rockchip_conn_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = rockchip_connector_destroy,
};

static u8 *rockchip_do_get_edid(struct drm_connector *connector)
{
	struct rockchip_conn_context *ctx = connector_to_ctx(connector);
	struct rockchip_connector *conn = ctx->conn;
	int i, j = 0, valid_ext = 0;
	u8 *block, *new;
	bool print_bad_edid = !connector->bad_edid_counter;

	if ((block = kmalloc(EDID_LENGTH, GFP_KERNEL)) == NULL)
		return NULL;

	/* base block fetch */
	for (i = 0; i < 4; i++) {
		if(conn->getedid(conn, 0, block))
			goto out;
		if (drm_edid_block_valid(block, 0, print_bad_edid))
			break;
		if (i == 0 && memchr_inv(block, 0, EDID_LENGTH)) {
			connector->null_edid_counter++;
			goto carp;
		}
	}
	if (i == 4)
		goto carp;

	/* if there's no extensions, we're done */
	if (block[0x7e] == 0)
		return block;

	new = krealloc(block, (block[0x7e] + 1) * EDID_LENGTH, GFP_KERNEL);
	if (!new)
		goto out;
	block = new;

	for (j = 1; j <= block[0x7e]; j++) {
		for (i = 0; i < 4; i++) {
			if(conn->getedid(conn, j, block + (valid_ext + 1) *
					  EDID_LENGTH))
				goto out;
			if (drm_edid_block_valid(block + (valid_ext + 1) *
						 EDID_LENGTH, j,
						 print_bad_edid)) {
				valid_ext++;
				break;
			}
		}

		if (i == 4 && print_bad_edid) {
			dev_warn(connector->dev->dev,
				 "%s: Ignoring invalid EDID block %d.\n",
				 drm_get_connector_name(connector), j);
			connector->bad_edid_counter++;
		}
	}

	if (valid_ext != block[0x7e]) {
		block[EDID_LENGTH-1] += block[0x7e] - valid_ext;
		block[0x7e] = valid_ext;
		new = krealloc(block, (valid_ext + 1) * EDID_LENGTH,
			       GFP_KERNEL);
		if (!new)
			goto out;
		block = new;
	}

	return block;

carp:
	if (print_bad_edid) {
		dev_warn(connector->dev->dev, "%s: EDID block %d invalid.\n",
			 drm_get_connector_name(connector), j);
	}
	connector->bad_edid_counter++;

out:
	kfree(block);
	return NULL;
}

static int rockchip_conn_get_modes(struct drm_connector *connector)
{
	struct rockchip_conn_context *ctx = connector_to_ctx(connector);
	struct rockchip_connector *conn = ctx->conn;
	struct drm_panel *panel = ctx->panel;
	struct edid *edid;
	int count = 0;

	if (conn->getedid) {
		edid = (struct edid *)rockchip_do_get_edid(connector);
		if (IS_ERR_OR_NULL(edid)) {
			DRM_ERROR("operation get_edid failed\n");
			return 0;
		}

		count = drm_add_edid_modes(connector, edid);
		if (!count) {
			DRM_ERROR("Add edid modes failed %d\n", count);
			kfree(edid);
			return 0;
		}

		drm_mode_connector_update_edid_property(connector, edid);
	} else if (panel) {
		count = panel->funcs->get_modes(panel);
	} else {
		DRM_ERROR("get connector modes fail\n");
	}

	return count;
}

static enum drm_mode_status
	rockchip_conn_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct rockchip_conn_context *ctx = connector_to_ctx(connector);
	struct rockchip_connector *conn = ctx->conn;

	if (conn->checkmode)
		return conn->checkmode(conn, mode);
	else
		return MODE_OK;	
}

static struct drm_encoder *
	rockchip_conn_best_encoder(struct drm_connector *connector)
{
	struct rockchip_conn_context *ctx = connector_to_ctx(connector);

	return &ctx->encoder;
}

static struct drm_connector_helper_funcs rockchip_connector_helper_funcs = {
	.get_modes = rockchip_conn_get_modes,
	.mode_valid = rockchip_conn_mode_valid,
	.best_encoder = rockchip_conn_best_encoder,
};

static void rockchip_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct rockchip_conn_context *ctx = encoder_to_ctx(encoder);
	struct rockchip_connector *conn = ctx->conn;

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		if (ctx->dpms_mode != DRM_MODE_DPMS_ON) {
			if (ctx->panel)
				ctx->panel->funcs->enable(ctx->panel);
			conn->enable(conn);
		}
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (ctx->dpms_mode == DRM_MODE_DPMS_ON) {
			conn->disable(conn);
			if (ctx->panel)
				ctx->panel->funcs->disable(ctx->panel);
		}
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
	struct rockchip_panel_special *priv_mode =
					(void *)adjusted_mode->private;
	struct rockchip_conn_context *ctx = encoder_to_ctx(encoder);

	if (!priv_mode) {
		/*
		 * if special panel info no define, set it as default;
		 */
		priv_mode = &ctx->priv_mode;
		adjusted_mode->private = (void *)priv_mode;
	}
	
	priv_mode->out_type = ctx->conn->type;

	return true;
}

static void rockchip_drm_encoder_mode_set(struct drm_encoder *encoder,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adjusted)
{
	/* just set dummy now */
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

static unsigned int rockchip_drm_encoder_clones(struct drm_encoder *encoder)
{
	struct rockchip_conn_context *ctx = encoder_to_ctx(encoder);
	struct drm_encoder *clone;
	struct drm_device *dev = encoder->dev;
	unsigned int clone_mask = 0;
	int cnt = 0;

	list_for_each_entry(clone, &dev->mode_config.encoder_list, head) {
		switch (ctx->type) {
		case ROCKCHIP_DISPLAY_TYPE_RGB:
		case ROCKCHIP_DISPLAY_TYPE_LVDS:
		case ROCKCHIP_DISPLAY_TYPE_EDP:
		case ROCKCHIP_DISPLAY_TYPE_HDMI:
			clone_mask |= (1 << (cnt++));
			break;
		default:
			continue;
		}
	}

	return clone_mask;
}

void rockchip_connector_hotplug_handler(void *data)
{
	struct rockchip_conn_context *ctx = data;
	
	if (ctx->drm_dev)
		drm_helper_hpd_irq_event(ctx->drm_dev);
}

static int rockchip_conn_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct rockchip_conn_context *ctx;
	unsigned long possible_crtcs = 0;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct device_node *panel_node;
	struct drm_device *drm_dev = data;
	int ret;

	ctx = rockchip_drm_component_data_get(dev,
					      ROCKCHIP_DEVICE_TYPE_CONNECTOR);
	if (!ctx) {
		DRM_ERROR("can't find dp content form component\n");
		return -EINVAL;
	}
	
	ctx->drm_dev = drm_dev;
	ret = rockchip_drm_pipe_get(dev);
	if (ret < 0) {
		DRM_ERROR("failed to bind display port\n");
		return ret;
	}

	possible_crtcs |= 1 << ret;

	encoder = &ctx->encoder;
	encoder->possible_crtcs = possible_crtcs;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	ret = drm_encoder_init(drm_dev, encoder, &rockchip_encoder_funcs,
			       rockchip_convert_encoder_type(ctx->type));
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
				 rockchip_convert_conn_type(ctx->type));
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

	if (ctx->type & ROCKCHIP_DISPLAY_TYPE_LCD) {
		panel_node = of_parse_phandle(dev->of_node,
					      "rockchip,panel", 0);
		if (!panel_node) {
			DRM_ERROR("failed to find diaplay panel\n");
			goto err_detach_connector;
		}
		ctx->panel = of_drm_find_panel(panel_node);
		if (!ctx->panel) {
			DRM_ERROR("failed to find diaplay panel\n");
			ret = -ENODEV;
			goto err_detach_connector;
		}

		of_node_put(panel_node);

		ret = drm_panel_attach(ctx->panel, connector);
		if (ret) {
			DRM_ERROR("failed to attach connector and encoder\n");
			goto err_detach_connector;
		}
	} else {
		struct rockchip_panel_special *priv_mode = &ctx->priv_mode;

		priv_mode->out_face = ROCKCHIP_OUTFACE_P888;
		priv_mode->color_swap = 0;
		priv_mode->pwr18 = false;	
		priv_mode->dither = false;
		priv_mode->flags = DISPLAY_FLAGS_PIXDATA_NEGEDGE;
	}

	return 0;

err_detach_connector:
	drm_mode_connector_detach_encoder(connector, encoder);
err_free_connector_sysfs:
	drm_sysfs_connector_remove(connector);
err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void rockchip_conn_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct rockchip_conn_context *ctx;
	struct drm_encoder *encoder;

	ctx = rockchip_drm_component_data_get(dev,
					      ROCKCHIP_DEVICE_TYPE_CONNECTOR);
	encoder = &ctx->encoder;

	if (ctx->panel)
		drm_panel_detach(ctx->panel);

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	encoder->funcs->destroy(encoder);
	drm_sysfs_connector_remove(&ctx->connector);
	drm_connector_cleanup(&ctx->connector);
	drm_encoder_cleanup(encoder);

	rockchip_drm_component_del(dev, ROCKCHIP_DEVICE_TYPE_CONNECTOR);
}

static const struct component_ops rockchip_conn_component_ops = {
	.bind = rockchip_conn_bind,
	.unbind = rockchip_conn_unbind,
};

void *rockchip_connector_register(struct rockchip_connector *conn)
{
	struct rockchip_conn_context *ctx;
	struct device *dev = conn->dev;
	int ret;

	if (!dev) {
		DRM_ERROR("please provide a device at dp register\n");
		return NULL;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	ret = rockchip_drm_component_add(dev, ROCKCHIP_DEVICE_TYPE_CONNECTOR,
					 conn->type, ctx);
	if (ret < 0) {
		DRM_ERROR("register connector component fail ret =%d\n", ret);
		return NULL;
	}

	ctx->conn = conn;
	ctx->dev = dev;
	ctx->type = conn->type;
	ctx->dpms_mode = DRM_MODE_DPMS_OFF;

	ret = component_add(dev, &rockchip_conn_component_ops);
	if (ret)
		goto err_del_component;

	DRM_DEBUG_KMS("succes register connector type=%d\n", conn->type);

	return ctx;

err_del_component:
	rockchip_drm_component_del(dev, ROCKCHIP_DEVICE_TYPE_CONNECTOR);
	return NULL;
}

void rockchip_connector_unregister(void *data)
{
	struct rockchip_conn_context *ctx = data;

	if (!ctx)
		return;
	rockchip_drm_component_del(ctx->dev, ROCKCHIP_DEVICE_TYPE_CONNECTOR);
	component_del(ctx->dev, &rockchip_conn_component_ops);
}

void rockchip_drm_encoder_setup(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head)
		encoder->possible_clones =
				rockchip_drm_encoder_clones(encoder);
}
