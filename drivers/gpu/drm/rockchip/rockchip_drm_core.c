/* rockchip_drm_core.c
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_core.c
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
#include "rockchip_drm_drv.h"
#include "rockchip_drm_crtc.h"
#include "rockchip_drm_encoder.h"
#include "rockchip_drm_fbdev.h"

int rockchip_drm_create_enc_conn(struct drm_device *dev,
				 struct rockchip_drm_display *display)
{
	struct drm_encoder *encoder;
	int ret;
	unsigned long possible_crtcs = 0;

	ret = rockchip_drm_crtc_get_pipe_from_type(dev, display->type);
	if (ret < 0)
		return ret;

	possible_crtcs |= 1 << ret;

	/* create and initialize a encoder for this sub driver. */
	encoder = rockchip_drm_encoder_create(dev, display, possible_crtcs);
	if (!encoder) {
		DRM_ERROR("failed to create encoder\n");
		return -EFAULT;
	}

	display->encoder = encoder;

	ret = display->ops->create_connector(display, encoder);
	if (ret) {
		DRM_ERROR("failed to create connector ret = %d\n", ret);
		goto err_destroy_encoder;
	}

	return 0;

err_destroy_encoder:
	encoder->funcs->destroy(encoder);
	return ret;
}
