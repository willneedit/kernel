/* rockchip_drm_encoder.h
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_encoder.h
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

#ifndef _ROCKCHIP_DRM_ENCODER_H_
#define _ROCKCHIP_DRM_ENCODER_H_

struct rockchip_drm_manager;

void rockchip_drm_encoder_setup(struct drm_device *dev);
struct drm_encoder *
	rockchip_drm_encoder_create(struct drm_device *dev,
				    struct rockchip_drm_display *mgr,
				    unsigned long possible_crtcs);
struct rockchip_drm_display *
	rockchip_drm_get_display(struct drm_encoder *encoder);

#endif /* _ROCKCHIP_DRM_ENCODER_H_ */
