/* rockchip_drm_gem.h
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

#ifndef _ROCKCHIP_DRM_GEM_H_
#define _ROCKCHIP_DRM_GEM_H_

#define to_rockchip_gem_obj(x)	container_of(x,\
					     struct rockchip_drm_gem_obj, base)

#define IS_NONCONTIG_BUFFER(f)		(f & ROCKCHIP_BO_NONCONTIG)

/*
 * rockchip drm gem buffer structure.
 *
 * @kvaddr: kernel virtual address to allocated memory region.
 * *userptr: user space address.
 * @dma_addr: bus address(accessed by dma) to allocated memory region.
 *        - this address could be physical address without IOMMU and
 *        device address with IOMMU.
 * @write: whether pages will be written to by the caller.
 * @pages: Array of backing pages.
 * @sgt: sg table to transfer page data.
 * @size: size of allocated memory region.
 * @pfnmap: indicate whether memory region from userptr is mmaped with
 * VM_PFNMAP or not.
 */
struct rockchip_drm_gem_buf {
	void __iomem *kvaddr;
	unsigned long userptr;
	dma_addr_t dma_addr;
	struct dma_attrs dma_attrs;
	unsigned int write;
	struct page **pages;
	struct sg_table *sgt;
	unsigned long size;
	bool pfnmap;
};

/*
 * rockchip drm buffer structure.
 *
 * @base: a gem object.
 *      - a new handle to this gem object would be created
 *      by drm_gem_handle_create().
 * @buffer: a pointer to rockchip_drm_gem_buffer object.
 *      - contain the information to memory region allocated
 *      by user request or at framebuffer creation.
 *      continuous memory region allocated by user request
 *      or at framebuffer creation.
 * @size: size requested from user, in bytes and this size is aligned
 *      in page unit.
 * @vma: a pointer to vm_area.
 * @flags: indicate memory type to allocated buffer and cache attruibute.
 *
 * P.S. this object would be transferred to user as kms_bo.handle so
 *      user can access the buffer through kms_bo.handle.
 */
struct rockchip_drm_gem_obj {
	struct drm_gem_object base;
	struct rockchip_drm_gem_buf *buffer;
	unsigned long size;
	struct vm_area_struct *vma;
	unsigned int flags;
};

/* destroy a buffer with gem object */
void rockchip_drm_gem_destroy(struct rockchip_drm_gem_obj *rockchip_gem_obj);

/* create a private gem object and initialize it. */
struct rockchip_drm_gem_obj *rockchip_drm_gem_init(struct drm_device *dev,
						   unsigned long size);

/* create a new buffer with gem object */
struct rockchip_drm_gem_obj *rockchip_drm_gem_create(struct drm_device *dev,
						     unsigned int flags,
						     unsigned long size);

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

int rockchip_drm_gem_mmap_buffer(struct file *filp,
				 struct vm_area_struct *vma);

/* get buffer information to memory region allocated by gem. */
int rockchip_drm_gem_get_ioctl(struct drm_device *dev, void *data,
			       struct drm_file *file_priv);

/* free gem object. */
void rockchip_drm_gem_free_object(struct drm_gem_object *gem_obj);

/* create memory region for drm framebuffer. */
int rockchip_drm_gem_dumb_create(struct drm_file *file_priv,
				 struct drm_device *dev,
				 struct drm_mode_create_dumb *args);

/* map memory region for drm framebuffer to user space. */
int rockchip_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				     struct drm_device *dev, uint32_t handle,
				     uint64_t *offset);

/* page fault handler and mmap fault address(virtual) to physical memory. */
int rockchip_drm_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);

/* set vm_flags and we can change the vm attribute to other one at here. */
int rockchip_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma);
#endif
