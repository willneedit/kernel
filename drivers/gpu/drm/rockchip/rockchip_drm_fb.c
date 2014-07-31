/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_fb.c
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
#include <drm/drm_fb_helper.h>

#include <uapi/drm/rockchip_drm.h>

static struct drm_framebuffer *
rockchip_user_fb_create(struct drm_device *dev, struct drm_file *file_priv,
			struct drm_mode_fb_cmd2 *mode_cmd)
{
	return drm_fb_cma_create(dev, file_priv, mode_cmd);
}

static const struct drm_mode_config_funcs rockchip_drm_mode_config_funcs = {
	.fb_create = rockchip_user_fb_create,
};

void rockchip_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	/*
	 * set max width and height as default value(4096x4096).
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	dev->mode_config.max_width = 4096;
	dev->mode_config.max_height = 4096;

	dev->mode_config.funcs = &rockchip_drm_mode_config_funcs;
}
