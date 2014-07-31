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

#ifndef _ROCKCHIP_DRM_GEM_H_
#define _ROCKCHIP_DRM_GEM_H_

/*
 * request gem object creation and buffer allocation as the size
 * that it is calculated with framebuffer information such as width,
 * height and bpp.
 */
int rockchip_drm_gem_create_ioctl(struct drm_device *dev, void *data,
				  struct drm_file *file_priv);

/* get buffer offset to map to user space. */
int rockchip_drm_gem_map_offset_ioctl(struct drm_device *dev, void *data,
				      struct drm_file *file_priv);

/*
 * mmap the physically continuous memory that a gem object contains
 * to user space.
 */
int rockchip_drm_gem_mmap_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv);

/* get buffer information to memory region allocated by gem. */
int rockchip_drm_gem_get_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv);
#endif
