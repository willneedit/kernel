/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_fbdev.c
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
#include <drm/drm_fb_cma_helper.h>

#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"

#define MAX_CONNECTOR		4
#define PREFERRED_BPP		32

int rockchip_drm_fbdev_init(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct drm_fbdev_cma *fbdev_cma;
	unsigned int num_crtc;

	if (!dev->mode_config.num_crtc || !dev->mode_config.num_connector)
		return 0;

	if (private->fbdev_cma) {
		DRM_ERROR("no allow to reinit cma fbdev\n");
		return -EINVAL;
	}

	num_crtc = dev->mode_config.num_crtc;

	fbdev_cma = drm_fbdev_cma_init(dev, PREFERRED_BPP, num_crtc,
				       MAX_CONNECTOR);
	if (!fbdev_cma) {
		DRM_ERROR("failed to init cma fbdev\n");
		return -ENOMEM;
	}

	private->fbdev_cma = fbdev_cma;

	return 0;
}

void rockchip_drm_fbdev_fini(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;

	if (!private || !private->fbdev_cma)
		return;

	drm_fbdev_cma_fini(private->fbdev_cma);
}
