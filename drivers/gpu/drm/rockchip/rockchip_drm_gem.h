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

#define to_rockchip_obj(x) container_of(x, struct rockchip_gem_object, base)

struct rockchip_gem_object {
	struct drm_gem_object base;
	
	struct page **pages;
	struct sg_table *sgt;
	dma_addr_t paddr;
	void *vaddr;
	unsigned int flags;

	struct device *mmu_dev;
};

struct rockchip_gem_object *
	rockchip_gem_create_object(struct drm_device *drm, unsigned int size);
void rockchip_drm_free_object(struct drm_gem_object *obj);

struct sg_table *rockchip_gem_prime_get_sg_table(struct drm_gem_object *obj);
struct drm_gem_object *
rockchip_gem_prime_import_sg_table(struct drm_device *dev, size_t size,
				  struct sg_table *sgt);
int rockchip_drm_gem_map_offset_ioctl(struct drm_device *drm, void *data,
				      struct drm_file *file_priv);
void *rockchip_gem_prime_vmap(struct drm_gem_object *obj);
void rockchip_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr);
int rockchip_gem_prime_mmap(struct drm_gem_object *obj,
			   struct vm_area_struct *vma);
int rockchip_gem_dumb_create(struct drm_file *file_priv,
		struct drm_device *dev, struct drm_mode_create_dumb *args);
int rockchip_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				     struct drm_device *dev, uint32_t handle,
				     uint64_t *offset);
int rockchip_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma);

int rockchip_iommu_mmap(struct device *dev,struct rockchip_gem_object *rk_obj);
void rockchip_iommu_unmap(struct rockchip_gem_object *rk_obj);
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
