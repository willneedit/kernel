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
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>

#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_fb.h"

#define MAX_CONNECTOR		4
#define PREFERRED_BPP		32
#define to_rockchip_fbdev(x) container_of(x, struct rockchip_fbdev, helper)

struct rockchip_fbdev {
	struct drm_fb_helper helper;
	struct drm_gem_object *bo;
};

static int rockchip_drm_fb_mmap(struct fb_info *info,
			struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;

	if (offset + size > info->fix.smem_len)
		return -EINVAL;

	pos = (unsigned long)info->fix.smem_start + offset;

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;
}

static struct fb_ops rockchip_drm_fbdev_ops = {
	.owner		= THIS_MODULE,
	.fb_mmap        = rockchip_drm_fb_mmap,
	.fb_fillrect    = cfb_fillrect,
	.fb_copyarea    = cfb_copyarea,
	.fb_imageblit   = cfb_imageblit,
	.fb_check_var	= drm_fb_helper_check_var,
	.fb_set_par	= drm_fb_helper_set_par,
	.fb_blank	= drm_fb_helper_blank,
	.fb_pan_display	= drm_fb_helper_pan_display,
	.fb_setcmap	= drm_fb_helper_setcmap,
};

static int rockchip_drm_fbdev_create(struct drm_fb_helper *helper,
	struct drm_fb_helper_surface_size *sizes)
{
	struct rockchip_fbdev *fbdev = to_rockchip_fbdev(helper);
	struct drm_mode_fb_cmd2 mode_cmd = { 0 };
	struct drm_device *dev = helper->dev;
	struct rockchip_gem_object *rk_obj;
	struct drm_framebuffer *fb;
	unsigned int bytes_per_pixel;
	unsigned long offset;
	struct fb_info *fbi;
	size_t size;
	int ret;

	bytes_per_pixel = DIV_ROUND_UP(sizes->surface_bpp, 8);

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;
	mode_cmd.pitches[0] = sizes->surface_width * bytes_per_pixel;
	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);

	size = mode_cmd.pitches[0] * mode_cmd.height;

	rk_obj = rockchip_gem_create_object(dev, size);
	if (IS_ERR(rk_obj))
		return -ENOMEM;

	fbdev->bo = &rk_obj->base;

	fbi = framebuffer_alloc(0, dev->dev);
	if (!fbi) {
		dev_err(dev->dev, "Failed to allocate framebuffer info.\n");
		ret = -ENOMEM;
		goto err_rockchip_gem_free_object;
	}

	helper->fb = rockchip_drm_framebuffer_init(dev, &mode_cmd, fbdev->bo);
	if (IS_ERR(helper->fb)) {
		dev_err(dev->dev, "Failed to allocate DRM framebuffer.\n");
		ret = PTR_ERR(helper->fb);
		goto err_framebuffer_release;
	}

	helper->fbdev = fbi;

	fbi->par = helper;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->fbops = &rockchip_drm_fbdev_ops;

	ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
	if (ret) {
		dev_err(dev->dev, "Failed to allocate color map.\n");
		goto err_rockchip_drm_framebuffer_fini;
	}

	fb = helper->fb;
	drm_fb_helper_fill_fix(fbi, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(fbi, helper, fb->width, fb->height);

	offset = fbi->var.xoffset * bytes_per_pixel;
	offset += fbi->var.yoffset * fb->pitches[0];

	if (!rk_obj->vaddr)
		rk_obj->vaddr = vmap(rk_obj->pages,
				     rk_obj->base.size >> PAGE_SHIFT,
				     VM_MAP, pgprot_writecombine(PAGE_KERNEL));

	dev->mode_config.fb_base = 0;
	fbi->screen_base = rk_obj->vaddr + offset;
	fbi->screen_size = rk_obj->base.size;
	fbi->fix.smem_start = (unsigned long)rk_obj->vaddr;
	fbi->fix.smem_len = rk_obj->base.size;

	DRM_DEBUG_KMS("FB [%dx%d]-%d vaddr=%p paddr=%x offset=%ld size=%d\n",
		      fb->width, fb->height, fb->depth, rk_obj->vaddr,
		      rk_obj->paddr, offset, size);
	return 0;

err_rockchip_drm_framebuffer_fini:
	rockchip_drm_framebuffer_fini(helper->fb);
err_framebuffer_release:
	framebuffer_release(fbi);
err_rockchip_gem_free_object:
	rockchip_drm_free_object(&rk_obj->base);
	return ret;
}

static struct drm_fb_helper_funcs rockchip_drm_fb_helper_funcs = {
	.fb_probe = rockchip_drm_fbdev_create,
};

int rockchip_drm_fbdev_init(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct rockchip_fbdev *fbdev;
	struct drm_fb_helper *helper;
	unsigned int num_crtc;
	int ret;

	if (!dev->mode_config.num_crtc || !dev->mode_config.num_connector)
		return -EINVAL;

	if (private->fb_helper) {
		DRM_ERROR("no allow to reinit fbdev\n");
		return -EINVAL;
	}

	num_crtc = dev->mode_config.num_crtc;

	fbdev = kzalloc(sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev) {
		dev_err(dev->dev, "Failed to allocate drm fbdev.\n");
		return -ENOMEM;
	}

	fbdev->helper.funcs = &rockchip_drm_fb_helper_funcs;
	helper = &fbdev->helper;

	ret = drm_fb_helper_init(dev, helper, num_crtc, MAX_CONNECTOR);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to initialize drm fb helper.\n");
		goto err_free;
	}

	ret = drm_fb_helper_single_add_all_connectors(helper);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to add connectors.\n");
		goto err_drm_fb_helper_fini;
	}

	/* disable all the possible outputs/crtcs before entering KMS mode */
	drm_helper_disable_unused_functions(dev);

	ret = drm_fb_helper_initial_config(helper, PREFERRED_BPP);
	if (ret < 0) {
		dev_err(dev->dev, "Failed to set initial hw configuration.\n");
		goto err_drm_fb_helper_fini;
	}

	private->fb_helper = helper;

	return 0;

err_drm_fb_helper_fini:
	drm_fb_helper_fini(helper);
err_free:
	kfree(fbdev);
	return ret;
}

void rockchip_drm_fbdev_fini(struct drm_device *dev)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct drm_fb_helper *helper;
	struct rockchip_fbdev *fbdev;

	if (!private || !private->fb_helper)
		return;

	helper = private->fb_helper;
	fbdev = to_rockchip_fbdev(helper);

	if (helper->fbdev) {
		struct fb_info *info;
		int ret;

		info = helper->fbdev;
		ret = unregister_framebuffer(info);
		if (ret < 0)
			DRM_DEBUG_KMS("failed unregister_framebuffer()\n");

		if (info->cmap.len)
			fb_dealloc_cmap(&info->cmap);

		framebuffer_release(info);
	}

	if (helper->fb)
		rockchip_drm_framebuffer_fini(helper->fb);

	drm_fb_helper_fini(helper);
	kfree(fbdev);
	private->fb_helper = NULL;
}
