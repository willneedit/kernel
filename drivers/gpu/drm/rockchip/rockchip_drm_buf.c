/* rockchip_drm_buf.c
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_buf.c
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
#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_buf.h"

static int lowlevel_buffer_allocate(struct drm_device *dev,
				    unsigned int flags,
				    struct rockchip_drm_gem_buf *buf)
{
	dma_addr_t start_addr;
	unsigned int i = 0;
	int ret = 0;
	enum dma_attr attr;
	unsigned int nr_pages;

	if (buf->dma_addr) {
		DRM_DEBUG_KMS("already allocated.\n");
		return 0;
	}

	init_dma_attrs(&buf->dma_attrs);

	/*
	 * if ROCKCHIP_BO_CONTIG, fully physically contiguous memory
	 * region will be allocated else physically contiguous
	 * as possible.
	 */
	if (!(flags & ROCKCHIP_BO_NONCONTIG))
		dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &buf->dma_attrs);

	/*
	 * if ROCKCHIP_BO_WC or ROCKCHIP_BO_NONCACHABLE, writecombine mapping
	 * else cachable mapping.
	 */
	if (flags & ROCKCHIP_BO_WC || !(flags & ROCKCHIP_BO_CACHABLE))
		attr = DMA_ATTR_WRITE_COMBINE;
	else
		attr = DMA_ATTR_NON_CONSISTENT;

	dma_set_attr(attr, &buf->dma_attrs);
	dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &buf->dma_attrs);

	nr_pages = buf->size >> PAGE_SHIFT;

	buf->pages = drm_calloc_large(nr_pages, sizeof(struct page *));
	if (!buf->pages) {
		DRM_ERROR("failed to allocate pages.\n");
		return -ENOMEM;
	}

	buf->kvaddr = dma_alloc_attrs(dev->dev, buf->size, &buf->dma_addr,
				      GFP_KERNEL, &buf->dma_attrs);
	if (!buf->kvaddr) {
		DRM_ERROR("failed to allocate buffer.\n");
		ret = -ENOMEM;
		goto err_free;
	}

	start_addr = buf->dma_addr;
	while (i < nr_pages) {
		buf->pages[i] = phys_to_page(start_addr);
		start_addr += PAGE_SIZE;
		i++;
	}

	buf->sgt = drm_prime_pages_to_sg(buf->pages, nr_pages);
	if (IS_ERR(buf->sgt)) {
		DRM_ERROR("failed to get sg table.\n");
		ret = PTR_ERR(buf->sgt);
		goto err_free_attrs;
	}

	DRM_DEBUG_KMS("dma_addr(0x%lx), size(0x%lx)\n",
		      (unsigned long)buf->dma_addr,
		      buf->size);

	return ret;

err_free_attrs:
	dma_free_attrs(dev->dev, buf->size, buf->pages,
		       (dma_addr_t)buf->dma_addr, &buf->dma_attrs);
	buf->dma_addr = (dma_addr_t)NULL;
err_free:
	return ret;
}

static void lowlevel_buffer_deallocate(struct drm_device *dev,
				       unsigned int flags,
				       struct rockchip_drm_gem_buf *buf)
{
	if (!buf->dma_addr) {
		DRM_DEBUG_KMS("dma_addr is invalid.\n");
		return;
	}

	DRM_DEBUG_KMS("dma_addr(0x%lx), size(0x%lx)\n",
		      (unsigned long)buf->dma_addr,
		      buf->size);

	sg_free_table(buf->sgt);

	kfree(buf->sgt);
	buf->sgt = NULL;

	dma_free_attrs(dev->dev, buf->size, buf->kvaddr,
		       (dma_addr_t)buf->dma_addr, &buf->dma_attrs);
	drm_free_large(buf->pages);

	buf->dma_addr = (dma_addr_t)NULL;
}

struct rockchip_drm_gem_buf *rockchip_drm_init_buf(struct drm_device *dev,
						   unsigned int size)
{
	struct rockchip_drm_gem_buf *buffer;

	DRM_DEBUG_KMS("desired size = 0x%x\n", size);

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return NULL;

	buffer->size = size;

	return buffer;
}

void rockchip_drm_fini_buf(struct drm_device *dev,
			   struct rockchip_drm_gem_buf *buffer)
{
	kfree(buffer);
}

int rockchip_drm_alloc_buf(struct drm_device *dev,
			   struct rockchip_drm_gem_buf *buf,
			   unsigned int flags)
{
	/*
	 * allocate memory region and set the memory information
	 * to vaddr and dma_addr of a buffer object.
	 */
	if (lowlevel_buffer_allocate(dev, flags, buf) < 0)
		return -ENOMEM;

	return 0;
}

void rockchip_drm_free_buf(struct drm_device *dev,
			   unsigned int flags,
			   struct rockchip_drm_gem_buf *buffer)
{
	lowlevel_buffer_deallocate(dev, flags, buffer);
}
