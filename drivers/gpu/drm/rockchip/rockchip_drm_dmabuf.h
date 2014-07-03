/* rockchip_drm_dmabuf.h
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_dmabuf.h
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

#ifndef _ROCKCHIP_DRM_DMABUF_H_
#define _ROCKCHIP_DRM_DMABUF_H_
struct dma_buf *rockchip_dmabuf_prime_export(struct drm_device *drm_dev,
					     struct drm_gem_object *obj,
					     int flags);

struct drm_gem_object *rockchip_dmabuf_prime_import(struct drm_device *drm_dev,
						    struct dma_buf *dma_buf);
#endif /* _ROCKCHIP_DRM_DMABUF_H_ */
