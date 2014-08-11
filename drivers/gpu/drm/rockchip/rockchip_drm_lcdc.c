/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_fimd.c
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
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>

#include <linux/iommu.h>
#include <linux/rockchip-iovmm.h>
#include <drm/rockchip_drm.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_lcdc.h"

#define LCDC_DEFAULT_FRAMERATE 60

static const uint32_t formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

struct rockchip_plane {
	int zpos;
	struct drm_plane base;
};

struct lcdc_context {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_crtc crtc;
	struct drm_pending_vblank_event *event;
	struct drm_display_mode mode;
	struct drm_plane *plane;
	struct lcdc_driver *drv;
	unsigned int default_win;
	unsigned int dpms;
	int pipe;
	wait_queue_head_t wait_vsync_queue;
	atomic_t wait_vsync_event;
};

#define to_lcdc_data(x) ((x)->drv->data)
#define to_lcdc_ctx(x) container_of(x, struct lcdc_context, crtc)
#define to_rockchip_plane(x) container_of(x, struct rockchip_plane, base)

const struct of_device_id lcdc_driver_dt_match[] = {
#ifdef CONFIG_LCDC_RK3288
	{ .compatible = "rockchip,rk3288-lcdc",
	  .data = (void *)&rockchip_rk3288_lcdc },
#endif
	{},
};

/****
1,success : pointer to the device inside of platform device 
2,fail       : NULL
****/
struct device *rockchip_get_sysmmu_device_by_compatible(const char *compt)
{
	struct device_node *dn = NULL;
	struct platform_device *pd = NULL;
	struct device *ret = NULL ;

	dn = of_find_compatible_node(NULL,NULL,compt);
	if(!dn)
	{
		pr_err("can't find device node %s \r\n",compt);
		return NULL;
	}
	
	pd = of_find_device_by_node(dn);
	if(!pd)
	{	
		pr_err("can't find platform device in device node %s \r\n",compt);
		return  NULL;
	}
	ret = &pd->dev;
	
	return ret;

}

int lcdc_sysmmu_fault_handler(struct device *dev,
		enum rk_iommu_inttype itype, unsigned long pgtable_base,
		unsigned long fault_addr,unsigned int status)
{
	DRM_ERROR("Generating Kernel OOPS... because it is unrecoverable.\n");

	return 0;
}
static inline void platform_set_sysmmu(struct device *sysmmu, struct device *dev)
{
	dev->archdata.iommu = sysmmu;
}
static inline struct lcdc_driver_data *drm_lcdc_get_driver_data(
	struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(lcdc_driver_dt_match, &pdev->dev);

	return (struct lcdc_driver_data *)of_id->data;
}

static int rockchip_plane_get_size(int start, unsigned length, unsigned last)
{
	int end = start + length;
	int size = 0;

	if (start <= 0) {
		if (end > 0)
			size = min_t(unsigned, end, last);
	} else if (start <= last) {
		size = min_t(unsigned, last - start, length);
	}

	return size;
}

static int rockchip_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
				 struct drm_framebuffer *fb, int crtc_x,
				 int crtc_y, unsigned int crtc_w,
				 unsigned int crtc_h, uint32_t src_x,
				 uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	struct lcdc_context *ctx = to_lcdc_ctx(crtc);
	struct lcdc_driver_data *lcdc_data = to_lcdc_data(ctx);
	struct drm_gem_object *obj;
	struct rockchip_gem_object *rk_obj;
	struct lcdc_win_data *win_data;
	unsigned long offset;
	unsigned int actual_w;
	unsigned int actual_h;
	int win;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	obj = rockchip_fb_get_gem_obj(fb, 0);
	if (!obj) {
		DRM_ERROR("fail to get cma object from framebuffer\n");
		return -EINVAL;
	}

	rk_obj = to_rockchip_obj(obj);
	actual_w = rockchip_plane_get_size(crtc_x,
					   crtc_w, crtc->mode.hdisplay);
	actual_h = rockchip_plane_get_size(crtc_y,
					   crtc_h, crtc->mode.vdisplay);
	if (crtc_x < 0) {
		if (actual_w)
			src_x -= crtc_x;
		crtc_x = 0;
	}

	if (crtc_y < 0) {
		if (actual_h)
			src_y -= crtc_y;
		crtc_y = 0;
	}

	win = rockchip_plane->zpos;
	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= ZPOS_MAX_NUM)
		return -EINVAL;

	offset = (src_x >> 16) * (fb->bits_per_pixel >> 3);
	offset += (src_y >> 16) * fb->pitches[0];

	DRM_DEBUG_KMS("offset = 0x%lx, pitch = %x\n", offset, fb->pitches[0]);

	win_data = lcdc_data->get_win(ctx->drv, win);

	win_data->xpos = crtc_x;
	win_data->ypos = crtc_y;
	win_data->xsize = actual_w;
	win_data->ysize = actual_h;
	win_data->xact = fb->width;
	win_data->yact = fb->height;
	win_data->y_vir_stride = fb->pitches[0] / (fb->bits_per_pixel >> 3);
	win_data->yrgb_addr = rk_obj->paddr + offset;
	win_data->uv_addr = 0;
	win_data->alpha_en = false;

	switch (fb->pixel_format) {
	case DRM_FORMAT_ARGB8888:
		win_data->alpha_en = true;
	case DRM_FORMAT_XRGB8888:
		win_data->format = ARGB888;
		break;
	case DRM_FORMAT_RGB565:
		win_data->format = RGB565;
		win_data->y_vir_stride =
			((win_data->y_vir_stride * 3) >> 2)
			+ win_data->y_vir_stride % 3;
		break;
	default:
		DRM_DEBUG_KMS("invalid pixel size so using unpacked 24bpp.\n");
		win_data->alpha_en = false;
		win_data->format = ARGB888;
		break;
	}
	win_data->enabled = true;
	DRM_DEBUG_KMS("offset_x = %d, offset_y = %d\n",
		      win_data->xpos, win_data->ypos);
	DRM_DEBUG_KMS("ovl_width = %d, ovl_height = %d\n",
		      win_data->xsize, win_data->ysize);
	DRM_DEBUG_KMS("paddr = 0x%lx\n", (unsigned long)win_data->yrgb_addr);
	DRM_DEBUG_KMS("fb_width = %d, actual_w = %d\n",
		      fb->width, actual_w);

	lcdc_data->win_commit(ctx->drv, win_data);
	return 0;
}

static int rockchip_disable_plane(struct drm_plane *plane)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	struct lcdc_context *ctx = to_lcdc_ctx(plane->crtc);
	struct lcdc_driver_data *lcdc_data = to_lcdc_data(ctx);
	struct lcdc_win_data *win_data;
	int win = rockchip_plane->zpos;

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= ZPOS_MAX_NUM)
		return -EINVAL;

	win_data = lcdc_data->get_win(ctx->drv, win);

	win_data->enabled = false;
	lcdc_data->win_commit(ctx->drv, win_data);

	return 0;
}

static void rockchip_plane_destroy(struct drm_plane *plane)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	rockchip_disable_plane(plane);
	drm_plane_cleanup(plane);
	kfree(rockchip_plane);
}

static int rockchip_plane_set_property(struct drm_plane *plane,
				       struct drm_property *property,
				       uint64_t val)
{
	struct drm_device *dev = plane->dev;
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	struct rockchip_drm_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	if (property == dev_priv->plane_zpos_property) {
		rockchip_plane->zpos = val;
		return 0;
	}

	return -EINVAL;
}

static struct drm_plane_funcs rockchip_plane_funcs = {
	.update_plane = rockchip_update_plane,
	.disable_plane = rockchip_disable_plane,
	.destroy = rockchip_plane_destroy,
	.set_property = rockchip_plane_set_property,
};

static void rockchip_plane_attach_zpos_property(struct drm_plane *plane)
{
	struct drm_device *dev = plane->dev;
	struct rockchip_drm_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	prop = dev_priv->plane_zpos_property;
	if (!prop) {
		prop = drm_property_create_range(dev, 0, "zpos", 0,
						 MAX_PLANE - 1);
		if (!prop)
			return;

		dev_priv->plane_zpos_property = prop;
	}

	drm_object_attach_property(&plane->base, prop, 0);
}

struct drm_plane *rockchip_plane_init(struct drm_device *dev,
				      unsigned long possible_crtcs, bool priv)
{
	struct rockchip_plane *rockchip_plane;
	struct rockchip_drm_private *private = dev->dev_private;
	int err;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	rockchip_plane = kzalloc(sizeof(*rockchip_plane), GFP_KERNEL);
	if (!rockchip_plane)
		return NULL;

	err = drm_plane_init(dev, &rockchip_plane->base, possible_crtcs,
			     &rockchip_plane_funcs, formats,
			     ARRAY_SIZE(formats), priv);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(rockchip_plane);
		return NULL;
	}

	if (priv) {
		rockchip_plane->base.crtc = private->crtc[0];
		rockchip_plane->zpos = DEFAULT_ZPOS;
	} else {
		rockchip_plane_attach_zpos_property(&rockchip_plane->base);
	}

	return &rockchip_plane->base;
}

int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct lcdc_context *ctx = to_lcdc_ctx(private->crtc[pipe]);
	struct lcdc_driver_data *lcdc_data = to_lcdc_data(ctx);

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	if (ctx->dpms != DRM_MODE_DPMS_ON)
		return -EPERM;

	lcdc_data->enable_vblank(ctx->drv);

	return 0;
}

void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct lcdc_context *ctx = to_lcdc_ctx(private->crtc[pipe]);
	struct lcdc_driver_data *lcdc_data = to_lcdc_data(ctx);

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	if (ctx->dpms != DRM_MODE_DPMS_ON)
		return;

	lcdc_data->disable_vblank(ctx->drv);
}

static void rockchip_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct lcdc_context *ctx = to_lcdc_ctx(crtc);
	struct lcdc_driver_data *lcdc_data = to_lcdc_data(ctx);

	DRM_DEBUG_KMS("crtc[%d] mode[%d]\n", crtc->base.id, mode);

	if (ctx->dpms == mode) {
		DRM_DEBUG_KMS("desired dpms mode is same as previous one.\n");
		return;
	}

	if (mode > DRM_MODE_DPMS_ON) {
		/* wait for the completion of page flip. */
		if (!wait_event_timeout(ctx->wait_vsync_queue,
					!atomic_read(&ctx->wait_vsync_event),
					HZ/20))
			DRM_DEBUG_KMS("vblank wait timed out.\n");
		drm_vblank_off(crtc->dev, ctx->pipe);
	}

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		lcdc_data->dpms(ctx->drv, DRM_MODE_DPMS_ON);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		lcdc_data->dpms(ctx->drv, DRM_MODE_DPMS_OFF);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}

	ctx->dpms = mode;
}

static void rockchip_drm_crtc_prepare(struct drm_crtc *crtc)
{
	/* drm framework doesn't check NULL. */
}

static bool rockchip_drm_crtc_mode_fixup(struct drm_crtc *crtc,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	/* just do dummy now */

	return true;
}

static int rockchip_drm_crtc_mode_set(struct drm_crtc *crtc,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode,
				      int x, int y,
				      struct drm_framebuffer *fb)
{
	struct lcdc_context *ctx = to_lcdc_ctx(crtc);
	struct lcdc_driver_data *lcdc_data = to_lcdc_data(ctx);

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	/* nothing to do if we haven't set the mode yet */
	if (adjusted_mode->htotal == 0 || adjusted_mode->vtotal == 0)
		return -EINVAL;

	drm_mode_copy(&ctx->mode, adjusted_mode);
	lcdc_data->mode_set(ctx->drv, &ctx->mode);

	return 0;
}
static int rockchip_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
			     struct drm_framebuffer *old_fb)
{
	struct lcdc_context *ctx = to_lcdc_ctx(crtc);
	unsigned int crtc_w;
	unsigned int crtc_h;
	int ret;

	crtc_w = crtc->fb->width - crtc->x;
	crtc_h = crtc->fb->height - crtc->y;

	ret = rockchip_update_plane(ctx->plane, crtc, crtc->fb, 0, 0, crtc_w,
				    crtc_h, crtc->x, crtc->y, crtc_w, crtc_h);
	if (ret < 0) {
		DRM_ERROR("fail to update plane\n");
		return -EINVAL;
	}

	return 0;
}

static void rockchip_drm_crtc_commit(struct drm_crtc *crtc)
{
	rockchip_drm_crtc_mode_set_base(crtc, crtc->x, crtc->y, crtc->fb);
}

static struct drm_crtc_helper_funcs rockchip_crtc_helper_funcs = {
	.dpms = rockchip_drm_crtc_dpms,
	.prepare = rockchip_drm_crtc_prepare,
	.mode_fixup = rockchip_drm_crtc_mode_fixup,
	.mode_set = rockchip_drm_crtc_mode_set,
	.mode_set_base = rockchip_drm_crtc_mode_set_base,
	.commit = rockchip_drm_crtc_commit,
};

static int rockchip_drm_crtc_page_flip(struct drm_crtc *crtc,
				       struct drm_framebuffer *fb,
				       struct drm_pending_vblank_event *event,
				       uint32_t page_flip_flags)
{
	struct drm_device *dev = crtc->dev;
	struct lcdc_context *ctx = to_lcdc_ctx(crtc);
	struct drm_framebuffer *old_fb = crtc->fb;
	unsigned int crtc_w;
	unsigned int crtc_h;
	int ret = -EINVAL;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	/* when the page flip is requested, crtc's dpms should be on */
	if (ctx->dpms > DRM_MODE_DPMS_ON) {
		DRM_ERROR("failed page flip request.\n");
		return -EINVAL;
	}

	mutex_lock(&dev->struct_mutex);

	/*
	 * the pipe from user always is 0 so we can set pipe number
	 * of current owner to event.
	 */
	ret = drm_vblank_get(dev, ctx->pipe);
	if (ret) {
		DRM_DEBUG("failed to acquire vblank counter\n");
		goto out;
	}

	spin_lock_irq(&dev->event_lock);
	if (ctx->event) {
		spin_unlock_irq(&dev->event_lock);
		DRM_ERROR("already pending flip!\n");
		ret = -EBUSY;
		goto out;
	}
	ctx->event = event;
	atomic_set(&ctx->wait_vsync_event, 1);
	spin_unlock_irq(&dev->event_lock);

	crtc->fb = fb;
	crtc_w = crtc->fb->width - crtc->x;
	crtc_h = crtc->fb->height - crtc->y;

	ret = rockchip_update_plane(ctx->plane, crtc, fb, 0, 0, crtc_w, crtc_h,
					crtc->x, crtc->y, crtc_w, crtc_h);
	if (ret) {
		crtc->fb = old_fb;

		spin_lock_irq(&dev->event_lock);
		drm_vblank_put(dev, ctx->pipe);
		atomic_set(&ctx->wait_vsync_event, 0);
		ctx->event = NULL;
		spin_unlock_irq(&dev->event_lock);

		goto out;
	}
out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

void rockchip_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *dev_priv = dev->dev_private;
	struct drm_crtc *drm_crtc = dev_priv->crtc[pipe];
	struct lcdc_context *ctx;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	if (!drm_crtc)
		return;

	ctx = to_lcdc_ctx(drm_crtc);
	event = ctx->event;

	spin_lock_irqsave(&dev->event_lock, flags);

	if (event) {
		ctx->event = NULL;
		drm_send_vblank_event(dev, -1, event);
		drm_vblank_put(dev, pipe);
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
	}

	spin_unlock_irqrestore(&dev->event_lock, flags);
}

void rockchip_drm_crtc_cancel_pending_flip(struct drm_device *dev)
{
	int i;
	DRM_DEBUG_KMS("cancle pending flip\n");

	for (i = 0; i < dev->num_crtcs; i++)
		rockchip_drm_crtc_finish_pageflip(dev, i);
}

static void rockchip_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct lcdc_context *ctx = to_lcdc_ctx(crtc);
	struct rockchip_drm_private *private = crtc->dev->dev_private;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	private->crtc[ctx->pipe] = NULL;

	drm_crtc_cleanup(crtc);
}

static struct drm_crtc_funcs rockchip_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = rockchip_drm_crtc_page_flip,
	.destroy = rockchip_drm_crtc_destroy,
};

void lcdc_vsync_event_handler(struct device *dev)
{
	struct drm_pending_vblank_event *event;
	struct lcdc_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev;
	unsigned long flags;

	DRM_DEBUG_KMS("LINE[%d]\n", __LINE__);
	/* check the crtc is detached already from encoder */
	if (ctx && (ctx->pipe < 0 || !ctx->drm_dev))
		return;

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);

	event = ctx->event;
	drm_dev = ctx->drm_dev;

	spin_lock_irqsave(&drm_dev->event_lock, flags);

	if (event) {
		ctx->event = NULL;
		drm_send_vblank_event(drm_dev, -1, event);
		drm_vblank_put(drm_dev, ctx->pipe);
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
	}

	spin_unlock_irqrestore(&drm_dev->event_lock, flags);
}

static int lcdc_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm_dev = data;
	struct rockchip_drm_private *private = drm_dev->dev_private;
	struct lcdc_context *ctx = dev_get_drvdata(dev);
	struct drm_crtc *crtc;

	ctx->drm_dev = drm_dev;

	ctx->pipe = rockchip_drm_pipe_get(dev);
	ctx->dpms = DRM_MODE_DPMS_OFF;
	crtc = &ctx->crtc;

	private->crtc[ctx->pipe] = crtc;
	ctx->plane = rockchip_plane_init(drm_dev, 1 << ctx->pipe, true);
	drm_crtc_init(drm_dev, crtc, &rockchip_crtc_funcs);
	drm_crtc_helper_add(crtc, &rockchip_crtc_helper_funcs);

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = true, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *      just specific driver own one instead because
	 *      drm framework supports only one irq handler.
	 */
	drm_dev->irq_enabled = true;

	/*
	 * with vblank_disable_allowed = true, vblank interrupt will be disabled
	 * by drm timer once a current process gives up ownership of
	 * vblank event.(after drm_vblank_put function is called)
	 */
	drm_dev->vblank_disable_allowed = true;
#ifdef CONFIG_ROCKCHIP_IOVMM
	if (iommu_present(&platform_bus_type)) {
		struct device *mmu_dev = rockchip_get_sysmmu_device_by_compatible("rockchip,vopl_mmu");
		if (mmu_dev) {
			platform_set_sysmmu(mmu_dev, ctx->drm_dev->dev);
			rockchip_iovmm_set_fault_handler(ctx->drm_dev->dev, lcdc_sysmmu_fault_handler);
			rockchip_iovmm_activate(ctx->drm_dev->dev);
		}
	}
#endif

	return 0;
}

static void lcdc_unbind(struct device *dev, struct device *master,
			void *data)
{	struct drm_device *drm_dev = data;
	struct rockchip_drm_private *private = drm_dev->dev_private;
	struct lcdc_context *ctx = dev_get_drvdata(dev);
	struct drm_crtc *crtc = &ctx->crtc;

	drm_crtc_cleanup(crtc);
	private->crtc[ctx->pipe] = NULL;
}

static const struct component_ops lcdc_component_ops = {
	.bind = lcdc_bind,
	.unbind = lcdc_unbind,
};

static int lcdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcdc_context *ctx;
	struct lcdc_driver *lcdc_drv;
	struct lcdc_driver_data *lcdc_data = drm_lcdc_get_driver_data(pdev);
	int type;
	int ret = -EINVAL;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (!(lcdc_data->num_win && lcdc_data->init &&
	      lcdc_data->deinit && lcdc_data->dpms &&
	      lcdc_data->mode_set && lcdc_data->enable_vblank &&
	      lcdc_data->disable_vblank &&
	      lcdc_data->win_commit)) {
		DRM_ERROR("lcdc driver ops is Incomplete\n");
		return -EINVAL;
	}
	lcdc_drv = lcdc_data->init(pdev);
	if (!lcdc_drv)
		return -EINVAL;

	lcdc_drv->data = lcdc_data;

	type = lcdc_drv->id ? ROCKCHIP_DISPLAY_TYPE_LCD :
				ROCKCHIP_DISPLAY_TYPE_HDMI;
	ret = rockchip_drm_component_add(&pdev->dev, ROCKCHIP_DEVICE_TYPE_CRTC,
					 type, ctx);
	if (ret)
		goto err_deinit_lcdc;

	ctx->dev = dev;
	ctx->default_win = ZPOS_DEFAULT_WIN;

	ctx->drv = lcdc_drv;

	init_waitqueue_head(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	platform_set_drvdata(pdev, ctx);

	pm_runtime_enable(&pdev->dev);

	ret = component_add(&pdev->dev, &lcdc_component_ops);
	if (ret)
		goto err_disable_pm_runtime;

	return ret;

err_disable_pm_runtime:
	pm_runtime_disable(dev);
	rockchip_drm_component_del(dev, ROCKCHIP_DEVICE_TYPE_CRTC);
err_deinit_lcdc:
	lcdc_data->deinit(ctx->drv);
	return ret;
}

static int lcdc_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	component_del(&pdev->dev, &lcdc_component_ops);
	rockchip_drm_component_del(&pdev->dev, ROCKCHIP_DEVICE_TYPE_CRTC);

	return 0;
}

struct platform_driver rockchip_lcdc_platform_driver = {
	.probe = lcdc_probe,
	.remove = lcdc_remove,
	.driver = {
		.name = "rockchip-lcdc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lcdc_driver_dt_match),
	},
};
