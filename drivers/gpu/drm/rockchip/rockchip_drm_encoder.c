/* rockchip_drm_encoder.c
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_encoder.c
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

#include "rockchip_drm_drv.h"
#include "rockchip_drm_encoder.h"

#define to_rockchip_encoder(x)	container_of(x, struct rockchip_drm_encoder,\
					     drm_encoder)

/*
 * rockchip specific encoder structure.
 *
 * @drm_encoder: encoder object.
 * @display: the display structure that maps to this encoder
 */
struct rockchip_drm_encoder {
	struct drm_encoder drm_encoder;
	struct rockchip_drm_display *display;
};

static void rockchip_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct rockchip_drm_encoder *rockchip_encoder =
						to_rockchip_encoder(encoder);
	struct rockchip_drm_display *display = rockchip_encoder->display;

	DRM_DEBUG_KMS("encoder dpms: %d\n", mode);

	if (display->ops->dpms)
		display->ops->dpms(display, mode);
}

static bool
rockchip_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct rockchip_drm_encoder *rockchip_encoder =
						to_rockchip_encoder(encoder);
	struct rockchip_drm_display *display = rockchip_encoder->display;
	struct drm_connector *connector;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->encoder != encoder)
			continue;

		if (display->ops->mode_fixup)
			display->ops->mode_fixup(display, connector, mode,
					adjusted_mode);
	}

	return true;
}

static void rockchip_drm_encoder_mode_set(struct drm_encoder *encoder,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adjusted)
{
	struct rockchip_drm_encoder *rockchip_encoder =
						to_rockchip_encoder(encoder);
	struct rockchip_drm_display *display = rockchip_encoder->display;

	if (display->ops->mode_set)
		display->ops->mode_set(display, adjusted);
}

static void rockchip_drm_encoder_prepare(struct drm_encoder *encoder)
{
	/* drm framework doesn't check NULL. */
}

static void rockchip_drm_encoder_commit(struct drm_encoder *encoder)
{
	struct rockchip_drm_encoder *rockchip_encoder =
						to_rockchip_encoder(encoder);
	struct rockchip_drm_display *display = rockchip_encoder->display;

	if (display->ops->dpms)
		display->ops->dpms(display, DRM_MODE_DPMS_ON);

	if (display->ops->commit)
		display->ops->commit(display);
}

static void rockchip_drm_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_plane *plane;
	struct drm_device *dev = encoder->dev;

	rockchip_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);

	/* all planes connected to this encoder should be also disabled. */
	drm_for_each_legacy_plane(plane, &dev->mode_config.plane_list) {
		if (plane->crtc == encoder->crtc)
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
	struct rockchip_drm_encoder *rockchip_encoder =
						to_rockchip_encoder(encoder);

	drm_encoder_cleanup(encoder);
	kfree(rockchip_encoder);
}

static struct drm_encoder_funcs rockchip_encoder_funcs = {
	.destroy = rockchip_drm_encoder_destroy,
};

static unsigned int rockchip_drm_encoder_clones(struct drm_encoder *encoder)
{
	struct drm_encoder *clone;
	struct drm_device *dev = encoder->dev;
	struct rockchip_drm_encoder *rockchip_encoder =
						to_rockchip_encoder(encoder);
	struct rockchip_drm_display *display = rockchip_encoder->display;
	unsigned int clone_mask = 0;
	int cnt = 0;

	list_for_each_entry(clone, &dev->mode_config.encoder_list, head) {
		switch (display->type) {
		case ROCKCHIP_DISPLAY_TYPE_LCD:
		case ROCKCHIP_DISPLAY_TYPE_HDMI:
			clone_mask |= (1 << (cnt++));
			break;
		default:
			continue;
		}
	}

	return clone_mask;
}

void rockchip_drm_encoder_setup(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head)
		encoder->possible_clones =
				rockchip_drm_encoder_clones(encoder);
}

struct drm_encoder *
	rockchip_drm_encoder_create(struct drm_device *dev,
				    struct rockchip_drm_display *display,
				    unsigned long possible_crtcs)
{
	struct drm_encoder *encoder;
	struct rockchip_drm_encoder *rockchip_encoder;

	if (!possible_crtcs)
		return NULL;

	rockchip_encoder = kzalloc(sizeof(*rockchip_encoder), GFP_KERNEL);
	if (!rockchip_encoder)
		return NULL;

	rockchip_encoder->display = display;
	encoder = &rockchip_encoder->drm_encoder;
	encoder->possible_crtcs = possible_crtcs;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	drm_encoder_init(dev, encoder, &rockchip_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS);

	drm_encoder_helper_add(encoder, &rockchip_encoder_helper_funcs);

	DRM_DEBUG_KMS("encoder has been created\n");

	return encoder;
}

struct rockchip_drm_display *
	rockchip_drm_get_display(struct drm_encoder *encoder)
{
	return to_rockchip_encoder(encoder)->display;
}
