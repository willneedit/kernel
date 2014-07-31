/*
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_gem.c
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
#include <drm/drm_vma_manager.h>
#include <drm/drm_gem_cma_helper.h>

#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"

int rockchip_drm_gem_create_ioctl(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	struct drm_rockchip_gem_create *args = data;
	struct drm_gem_cma_object *cma_obj;
	struct drm_gem_object *gem_obj;
	int ret;

	cma_obj = drm_gem_cma_create(dev, args->size);
	if (IS_ERR(cma_obj))
		return PTR_ERR_OR_ZERO(cma_obj);

	gem_obj = &cma_obj->base;

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, gem_obj, &args->handle);
	if (ret)
		goto err_handle_create;

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(gem_obj);

	return PTR_ERR_OR_ZERO(cma_obj);

err_handle_create:
	drm_gem_cma_free_object(gem_obj);
	return ret;
}

int rockchip_drm_gem_map_offset_ioctl(struct drm_device *dev, void *data,
				      struct drm_file *file_priv)
{
	struct drm_rockchip_gem_map_off *args = data;

	DRM_DEBUG_KMS("handle = 0x%x, offset = 0x%lx\n",
		      args->handle, (unsigned long)args->offset);

	return drm_gem_cma_dumb_map_offset(file_priv, dev, args->handle,
					   &args->offset);
}

int rockchip_drm_gem_mmap_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct drm_rockchip_gem_mmap *args = data;
	struct drm_gem_object *obj;
	unsigned long addr;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	addr = vm_mmap(obj->filp, 0, args->size, PROT_READ | PROT_WRITE,
		       MAP_SHARED, 0);

	drm_gem_object_unreference(obj);

	if (IS_ERR_VALUE(addr)) {
		mutex_unlock(&dev->struct_mutex);
		return (int)addr;
	}

	mutex_unlock(&dev->struct_mutex);

	args->mapped = addr;

	DRM_DEBUG_KMS("mapped = 0x%lx\n", (unsigned long)args->mapped);

	return 0;
}

int rockchip_drm_gem_get_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct drm_rockchip_gem_info *args = data;
	struct drm_gem_object *obj;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}

	args->size = obj->size;

	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

int rockchip_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				     struct drm_device *dev, uint32_t handle,
				     uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	mutex_lock(&dev->struct_mutex);

	/*
	 * get offset of memory allocated for drm framebuffer.
	 * - this callback would be called by user application
	 * with DRM_IOCTL_MODE_MAP_DUMB command.
	 */

	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		ret = -EINVAL;
		goto unlock;
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		goto out;

	*offset = drm_vma_node_offset_addr(&obj->vma_node);
	DRM_DEBUG_KMS("offset = 0x%lx\n", (unsigned long)*offset);

out:
	drm_gem_object_unreference(obj);
unlock:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}
