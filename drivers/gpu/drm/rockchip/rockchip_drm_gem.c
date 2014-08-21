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

#include <drm/drmP.h>
#include <drm/drm_vma_manager.h>
#include <drm/rockchip_drm.h>

#include <linux/iommu.h>
#include <linux/rockchip-iovmm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"

static int rockchip_gem_alloc_buf(struct rockchip_gem_object *rk_obj)
{
	struct drm_gem_object *obj = &rk_obj->base;
	struct drm_device *drm = obj->dev;
	dma_addr_t paddr;
	struct page **p;
	int npages = obj->size >> PAGE_SHIFT;
	int ret;
	int i;

	BUG_ON(rk_obj->pages);
	
	if (iommu_present(&platform_bus_type)) {
		p = drm_gem_get_pages(obj, GFP_KERNEL);
		if (IS_ERR(p)) {
			dev_err(drm->dev, "could not get pages: %ld\n",
					PTR_ERR(p));
			return -ENOMEM;
		}
	} else {
		p = drm_calloc_large(npages, sizeof(struct page *));
		if (!p) {
			DRM_ERROR("failed to allocate pages.\n");
			return -ENOMEM;
		}

		rk_obj->vaddr = dma_alloc_writecombine(drm->dev, obj->size,
				&rk_obj->paddr, GFP_KERNEL | __GFP_NOWARN);
		if (!rk_obj->vaddr) {
			dev_err(drm->dev, "failed to allocate buffer with size %d\n",
					obj->size);
			drm_free_large(rk_obj->pages);
			return -ENOMEM;
		}

		paddr = rk_obj->paddr;
		for (i = 0; i < npages; i++) {
			p[i] = phys_to_page(paddr);
			paddr += PAGE_SIZE;
		}
	}
	
	rk_obj->pages = p;
	rk_obj->sgt = drm_prime_pages_to_sg(p, npages);
	if (IS_ERR(rk_obj->sgt)) {
		dev_err(drm->dev, "failed to allocate sgt\n");
		ret = -ENOMEM;
		goto err_free_pages;
	}

	return 0;

err_free_pages:
	if (iommu_present(&platform_bus_type)) {
		drm_gem_put_pages(obj, rk_obj->pages, true, false);
	} else {
		dma_free_writecombine(drm->dev, obj->size, rk_obj->vaddr,
				      rk_obj->paddr);
		drm_free_large(rk_obj->pages);
	}
	rk_obj->pages = NULL;

	return ret;
}

static void rockchip_gem_free_buf(struct rockchip_gem_object *rk_obj)
{
	struct drm_gem_object *obj = &rk_obj->base;
	struct drm_device *drm = obj->dev;

	/* For non-cached buffers, ensure the new pages are clean
	 * because display controller, GPU, etc. are not coherent:
	 */
	if (rk_obj->flags & (ROCKCHIP_BO_WC | ROCKCHIP_BO_NONCACHABLE))
		dma_unmap_sg(drm->dev, rk_obj->sgt->sgl,
				rk_obj->sgt->nents, DMA_BIDIRECTIONAL);
	if (iommu_present(&platform_bus_type)) {
		if (rk_obj->vaddr)
			vunmap(rk_obj->vaddr);
		if (rk_obj->paddr)
			rockchip_iommu_unmap(rk_obj);
	}
	sg_free_table(rk_obj->sgt);
	kfree(rk_obj->sgt);

	if (iommu_present(&platform_bus_type)) {
		drm_gem_put_pages(obj, rk_obj->pages, true, false);
	} else {
		dma_free_writecombine(drm->dev, obj->size, rk_obj->vaddr,
				      rk_obj->paddr);
		drm_free_large(rk_obj->pages);
	}

	rk_obj->pages = NULL;
}

int rockchip_iommu_mmap(struct device *dev,struct rockchip_gem_object *rk_obj)
{
	rk_obj->paddr = rockchip_iovmm_map(dev,	rk_obj->sgt->sgl, 0,
					   rk_obj->base.size);
	if (!rk_obj->paddr)
		return -ENXIO;

	rk_obj->mmu_dev = dev;

	return 0;
}

void rockchip_iommu_unmap(struct rockchip_gem_object *rk_obj)
{
	if (rk_obj->mmu_dev)
		rockchip_iovmm_unmap(rk_obj->mmu_dev, rk_obj->paddr);

	rk_obj->mmu_dev = NULL;
}

struct rockchip_gem_object *
	rockchip_gem_create_object(struct drm_device *drm, unsigned int size)
{
	struct rockchip_gem_object *rk_obj;
	struct drm_gem_object *obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	rk_obj = kzalloc(sizeof(*rk_obj), GFP_KERNEL);
	if (!rk_obj)
		return ERR_PTR(-ENOMEM);

	obj = &rk_obj->base;

	ret = drm_gem_object_init(drm, obj, size);
	if (ret)
		goto err_free_rk_obj;

	ret = drm_gem_create_mmap_offset(obj);
	if (ret) {
		goto err_free_obj;
	}

	ret = rockchip_gem_alloc_buf(rk_obj);
	if (ret)
		goto err_free_mmap_offset;

	return rk_obj;

err_free_mmap_offset:
	drm_gem_free_mmap_offset(obj);
err_free_obj:
	drm_gem_object_release(obj);
err_free_rk_obj:
	kfree(rk_obj);
	return ERR_PTR(ret);
}

/*
 * rockchip_drm_free_object - (struct drm_driver)->gem_free_object callback
 * function
 */
void rockchip_drm_free_object(struct drm_gem_object *obj)
{
	struct rockchip_gem_object *rk_obj;

	drm_gem_free_mmap_offset(obj);

	rk_obj = to_rockchip_obj(obj);

	rockchip_gem_free_buf(rk_obj);
	drm_gem_free_mmap_offset(obj);

	drm_gem_object_release(obj);

	kfree(rk_obj);
}

/*
 * rockchip_gem_create_with_handle - allocate an object with the given
 * size and create a gem handle on it
 *
 * returns a struct rockchip_gem_object* on success or ERR_PTR values
 * on failure.
 */
static struct rockchip_gem_object *rockchip_gem_create_with_handle(
		struct drm_file *file_priv,
		struct drm_device *drm, unsigned int size,
		unsigned int *handle)
{
	struct rockchip_gem_object *rk_obj;
	struct drm_gem_object *obj;
	int ret;

	rk_obj = rockchip_gem_create_object(drm, size);
	if (IS_ERR(rk_obj))
		return NULL;

	obj = &rk_obj->base;

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, obj, handle);
	if (ret)
		goto err_handle_create;

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(obj);

	return rk_obj;

err_handle_create:
	rockchip_drm_free_object(obj);

	return ERR_PTR(ret);
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

/* low-level interface prime helpers */
struct sg_table *rockchip_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct rockchip_gem_object *rk_obj = to_rockchip_obj(obj);
	struct sg_table *sgt;
	int ret;

	sgt = drm_prime_pages_to_sg(rk_obj->pages, obj->size / PAGE_SIZE);
	if (IS_ERR(sgt)) {
		dev_err(obj->dev->dev, "failed to allocate sgt\n");
		goto out;
	}

	return sgt;

out:
	kfree(sgt);
	return NULL;

}

struct drm_gem_object *
rockchip_gem_prime_import_sg_table(struct drm_device *dev, size_t size,
				  struct sg_table *sgt)
{
	struct rockchip_gem_object *rk_obj;

	if (sgt->nents != 1)
		return ERR_PTR(-EINVAL);

	rk_obj = rockchip_gem_create_object(dev, size);
	if (IS_ERR(rk_obj))
		return ERR_PTR(-ENOMEM);

	DRM_DEBUG_PRIME("dma_addr = 0x%x, size = %zu\n", rk_obj->paddr, size);

	return &rk_obj->base;
}

void *rockchip_gem_prime_vmap(struct drm_gem_object *obj)
{
	struct rockchip_gem_object *rk_obj = to_rockchip_obj(obj);

	return rk_obj->vaddr;
}

void rockchip_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr)
{
	/* Nothing to do */
}

int rockchip_gem_prime_mmap(struct drm_gem_object *obj,
			   struct vm_area_struct *vma)
{
	struct drm_device *dev = obj->dev;
	int ret;

	mutex_lock(&dev->struct_mutex);
	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

/*
 * rockchip_gem_dumb_create - (struct drm_driver)->dumb_create callback
 * function
 *
 * This aligns the pitch and size arguments to the minimum required. wrap
 * this into your own function if you need bigger alignment.
 */
int rockchip_gem_dumb_create(struct drm_file *file_priv,
		struct drm_device *dev, struct drm_mode_create_dumb *args)
{
	struct rockchip_gem_object* rk_obj;
	int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	if (args->pitch < min_pitch)
		args->pitch = min_pitch;

	if (args->size < args->pitch * args->height)
		args->size = args->pitch * args->height;

	rk_obj = rockchip_gem_create_with_handle(file_priv, dev,
			args->size, &args->handle);
	return PTR_ERR_OR_ZERO(rk_obj);
}

int rockchip_drm_gem_get_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv)
{
	struct drm_rockchip_gem_info *args = data;
	struct rockchip_gem_object *rk_obj;
	struct drm_gem_object *obj;

	mutex_lock(&dev->struct_mutex);

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}
	
	rk_obj = to_rockchip_obj(obj);

	args->flags = rk_obj->flags;
	args->size = obj->size;

	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);

	return 0;
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

int rockchip_drm_gem_map_offset_ioctl(struct drm_device *drm, void *data,
				      struct drm_file *file_priv)
{
	struct drm_rockchip_gem_map_off *args = data;

	return rockchip_drm_gem_dumb_map_offset(file_priv, drm, args->handle,
					 &args->offset);
}

int rockchip_drm_gem_create_ioctl(struct drm_device *dev, void *data,
				  struct drm_file *file_priv)
{
	struct drm_rockchip_gem_create *args = data;
	struct rockchip_gem_object* rk_obj;

	rk_obj = rockchip_gem_create_with_handle(file_priv, dev,
			args->size, &args->handle);
	return PTR_ERR_OR_ZERO(rk_obj);
}
