/* rockchip_drm_crtc.h
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_crtc.h
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

#ifndef _ROCKCHIP_DRM_CRTC_H_
#define _ROCKCHIP_DRM_CRTC_H_

struct drm_device;
struct drm_crtc;
struct rockchip_drm_manager;
struct rockchip_drm_overlay;

int rockchip_drm_crtc_create(struct rockchip_drm_manager *manager);
int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe);
void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe);
void rockchip_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe);
void rockchip_drm_crtc_complete_scanout(struct drm_framebuffer *fb);

void rockchip_drm_crtc_plane_mode_set(struct drm_crtc *crtc,
				      struct rockchip_drm_overlay *overlay);
void rockchip_drm_crtc_plane_commit(struct drm_crtc *crtc, int zpos);
void rockchip_drm_crtc_plane_enable(struct drm_crtc *crtc, int zpos);
void rockchip_drm_crtc_plane_disable(struct drm_crtc *crtc, int zpos);

/* This function gets pipe value to crtc device matched with out_type. */
int rockchip_drm_crtc_get_pipe_from_type(struct drm_device *drm_dev,
					 unsigned int out_type);

void rockchip_drm_crtc_cancel_pending_flip(struct drm_device *dev);
#endif /* _ROCKCHIP_DRM_CRTC_H_ */
