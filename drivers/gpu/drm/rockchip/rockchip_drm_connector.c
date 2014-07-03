/* rockchip_drm_connector.c
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_connector.c
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

#include <drm/rockchip_drm.h>
#include "rockchip_drm_drv.h"
#include "rockchip_drm_encoder.h"
#include "rockchip_drm_connector.h"

#define to_rockchip_connector(x) \
		container_of(x, struct rockchip_drm_connector, drm_connector)

struct rockchip_drm_connector {
	struct drm_connector drm_connector;
	struct rockchip_drm_display *display;
	uint32_t encoder_id;
};

static int rockchip_drm_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_drm_connector *rockchip_connector =
					to_rockchip_connector(connector);
	struct rockchip_drm_display *display = rockchip_connector->display;
	unsigned int count = 0;
	int ret;

	/*
	 * if get_edid() exists then get_edid() callback of hdmi side
	 * is called to get edid data through i2c interface else
	 * get timing from the lcdc driver(display controller).
	 *
	 * P.S. in case of lcd panel, count is always 1 if success
	 * because lcd panel has only one mode.
	 */
	if (display->ops->get_edid) {
		struct edid *edid;

		edid = display->ops->get_edid(display, connector);
		if (IS_ERR_OR_NULL(edid)) {
			ret = PTR_ERR(edid);
			DRM_ERROR("Panel operation get_edid failed %d\n", ret);
			goto out;
		}

		count = drm_add_edid_modes(connector, edid);
		if (!count) {
			DRM_ERROR("Add edid modes failed %d\n", count);
			kfree(edid);
			return 0;
		}

		drm_mode_connector_update_edid_property(connector, edid);
	} else {
		struct rockchip_drm_panel_info *panel;
		struct drm_display_mode *mode =
					drm_mode_create(connector->dev);
		if (!mode) {
			DRM_ERROR("failed to create a new display mode.\n");
			return 0;
		}

		if (display->ops->get_panel) {
			panel = display->ops->get_panel(display);
		} else {
			drm_mode_destroy(connector->dev, mode);
			return 0;
		}

		drm_display_mode_from_videomode(&panel->vm, mode);
		mode->width_mm = panel->width_mm;
		mode->height_mm = panel->height_mm;
		connector->display_info.width_mm = mode->width_mm;
		connector->display_info.height_mm = mode->height_mm;

		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);

		count = 1;
	}
	return count;
}

static int rockchip_drm_connector_mode_valid(struct drm_connector *connector,
					     struct drm_display_mode *mode)
{
	struct rockchip_drm_connector *rockchip_connector =
					to_rockchip_connector(connector);
	struct rockchip_drm_display *display = rockchip_connector->display;
	int ret = MODE_BAD;

	if (display->ops->check_mode)
		if (!display->ops->check_mode(display, mode))
			ret = MODE_OK;

	return ret;
}

static struct drm_encoder *
	rockchip_drm_best_encoder(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct rockchip_drm_connector *rockchip_connector =
					to_rockchip_connector(connector);
	struct drm_mode_object *obj;
	struct drm_encoder *encoder;

	obj = drm_mode_object_find(dev, rockchip_connector->encoder_id,
				   DRM_MODE_OBJECT_ENCODER);
	if (!obj) {
		DRM_DEBUG_KMS("Unknown ENCODER ID %d\n",
			      rockchip_connector->encoder_id);
		return NULL;
	}

	encoder = obj_to_encoder(obj);

	return encoder;
}

static struct drm_connector_helper_funcs rockchip_connector_helper_funcs = {
	.get_modes = rockchip_drm_connector_get_modes,
	.mode_valid = rockchip_drm_connector_mode_valid,
	.best_encoder = rockchip_drm_best_encoder,
};

static int rockchip_drm_connector_fill_modes(struct drm_connector *connector,
					     unsigned int max_width,
					     unsigned int max_height)
{
	struct rockchip_drm_connector *rockchip_connector =
					to_rockchip_connector(connector);
	struct rockchip_drm_display *display = rockchip_connector->display;
	unsigned int width, height;

	width = max_width;
	height = max_height;

	/*
	 * if specific driver want to find desired_mode using maxmum
	 * resolution then get max width and height from that driver.
	 */
	if (display->ops->get_max_resol)
		display->ops->get_max_resol(display, &width, &height);

	return drm_helper_probe_single_connector_modes(connector, width,
						       height);
}

/* get detection status of display device. */
static enum drm_connector_status
rockchip_drm_connector_detect(struct drm_connector *connector, bool force)
{
	struct rockchip_drm_connector *rockchip_connector =
					to_rockchip_connector(connector);
	struct rockchip_drm_display *display = rockchip_connector->display;
	enum drm_connector_status status = connector_status_disconnected;

	if (display->ops->is_connected) {
		if (display->ops->is_connected(display))
			status = connector_status_connected;
		else
			status = connector_status_disconnected;
	}

	return status;
}

static void rockchip_drm_connector_destroy(struct drm_connector *connector)
{
	struct rockchip_drm_connector *rockchip_connector =
		to_rockchip_connector(connector);

	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(rockchip_connector);
}

static struct drm_connector_funcs rockchip_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = rockchip_drm_connector_fill_modes,
	.detect = rockchip_drm_connector_detect,
	.destroy = rockchip_drm_connector_destroy,
};

struct drm_connector *rockchip_drm_connector_create(struct drm_device *dev,
						    struct drm_encoder *encoder)
{
	struct rockchip_drm_connector *rockchip_connector;
	struct rockchip_drm_display *display =
				rockchip_drm_get_display(encoder);
	struct drm_connector *connector;
	int type;
	int err;

	rockchip_connector = kzalloc(sizeof(*rockchip_connector), GFP_KERNEL);
	if (!rockchip_connector)
		return NULL;

	connector = &rockchip_connector->drm_connector;

	switch (display->type) {
	case ROCKCHIP_DISPLAY_TYPE_HDMI:
		type = DRM_MODE_CONNECTOR_HDMIA;
		connector->interlace_allowed = true;
		connector->polled = DRM_CONNECTOR_POLL_HPD;
		break;
	case ROCKCHIP_DISPLAY_TYPE_LCD:
		type = DRM_MODE_CONNECTOR_LVDS;
		break;
	default:
		type = DRM_MODE_CONNECTOR_Unknown;
		break;
	}

	drm_connector_init(dev, connector, &rockchip_connector_funcs, type);
	drm_connector_helper_add(connector, &rockchip_connector_helper_funcs);

	err = drm_sysfs_connector_add(connector);
	if (err)
		goto err_connector;

	rockchip_connector->encoder_id = encoder->base.id;
	rockchip_connector->display = display;
	connector->dpms = DRM_MODE_DPMS_OFF;
	connector->encoder = encoder;

	err = drm_mode_connector_attach_encoder(connector, encoder);
	if (err) {
		DRM_ERROR("failed to attach a connector to a encoder\n");
		goto err_sysfs;
	}

	DRM_DEBUG_KMS("connector has been created\n");

	return connector;

err_sysfs:
	drm_sysfs_connector_remove(connector);
err_connector:
	drm_connector_cleanup(connector);
	kfree(rockchip_connector);
	return NULL;
}
