/* rockchip_drm_lcdc.c
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_crtc.h"
#include "rockchip_drm_lcdc.h"
#include "rockchip_drm_dp.h"

#define LCDC_DEFAULT_FRAMERATE 60

struct lcdc_context {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_display_mode mode;
	struct lcdc_driver *drv;
	unsigned int default_win;
	bool suspended;
	int pipe;
	wait_queue_head_t wait_vsync_queue;
	atomic_t wait_vsync_event;
	struct rockchip_drm_display *display;
};

#define to_lcdc_ops(x) ((x)->drv->data->ops)

static const struct of_device_id lcdc_driver_dt_match[] = {
#ifdef CONFIG_LCDC_RK3288
	{ .compatible = "rockchip,rk3288-lcdc",
	  .data = (void *)&rockchip_rk3288_lcdc },
#endif
	{},
};

static inline struct lcdc_driver_data *drm_lcdc_get_driver_data(
	struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(lcdc_driver_dt_match, &pdev->dev);

	return (struct lcdc_driver_data *)of_id->data;
}

static void lcdc_wait_for_vblank(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;

	if (ctx->suspended)
		return;

	atomic_set(&ctx->wait_vsync_event, 1);

	/*
	 * wait for FIMD to signal VSYNC interrupt or return after
	 * timeout which is set to 50ms (refresh rate of 20).
	 */
	if (!wait_event_timeout(ctx->wait_vsync_queue,
				!atomic_read(&ctx->wait_vsync_event),
				HZ/20))
		DRM_DEBUG_KMS("vblank wait timed out.\n");
}

static int lcdc_mgr_initialize(struct rockchip_drm_manager *mgr,
			       struct drm_device *drm_dev)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct rockchip_drm_private *priv;

	priv = drm_dev->dev_private;

	ctx->drm_dev = drm_dev;
	mgr->drm_dev = drm_dev;
	ctx->pipe = priv->pipe++;
	mgr->pipe = ctx->pipe;

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

	return 0;
}

static void lcdc_mgr_remove(struct rockchip_drm_manager *mgr)
{
}

static bool lcdc_mode_fixup(struct rockchip_drm_manager *mgr,
			    const struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	if (adjusted_mode->vrefresh == 0)
		adjusted_mode->vrefresh = LCDC_DEFAULT_FRAMERATE;

	return true;
}

static void lcdc_mode_set(struct rockchip_drm_manager *mgr,
			  const struct drm_display_mode *in_mode)
{
	struct lcdc_context *ctx = mgr->ctx;

	drm_mode_copy(&ctx->mode, in_mode);
}

static void lcdc_commit(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct drm_display_mode *mode = &ctx->mode;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (ctx->suspended)
		return;

	/* nothing to do if we haven't set the mode yet */
	if (mode->htotal == 0 || mode->vtotal == 0)
		return;

	lcdc_ops->mode_set(ctx->drv, mode);
}

static int lcdc_enable_vblank(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (ctx->suspended)
		return -EPERM;

	lcdc_ops->enable_vblank(ctx->drv);

	return 0;
}

static void lcdc_disable_vblank(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (ctx->suspended)
		return;

	lcdc_ops->disable_vblank(ctx->drv);
}

static void lcdc_win_mode_set(struct rockchip_drm_manager *mgr,
			      struct rockchip_drm_overlay *overlay)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_win_data *win_data;
	int win;
	unsigned long offset;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (!overlay) {
		DRM_ERROR("overlay is NULL\n");
		return;
	}

	win = overlay->zpos;
	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= ZPOS_MAX_NUM)
		return;

	offset = overlay->fb_x * (overlay->bpp >> 3);
	offset += overlay->fb_y * overlay->pitch;

	DRM_DEBUG_KMS("offset = 0x%lx, pitch = %x\n", offset, overlay->pitch);

	win_data = lcdc_ops->get_win(ctx->drv, win);

	win_data->xpos = overlay->crtc_x;
	win_data->ypos = overlay->crtc_y;
	win_data->xsize = overlay->crtc_width;
	win_data->ysize = overlay->crtc_height;
	win_data->xact = overlay->fb_width;
	win_data->yact = overlay->fb_height;
	win_data->y_vir_stride = overlay->pitch / (overlay->bpp >> 3);
	win_data->yrgb_addr = overlay->dma_addr[0] + offset;
	win_data->uv_addr = 0;
	win_data->alpha_en = false;

	switch (overlay->pixel_format) {
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
	DRM_DEBUG_KMS("fb_width = %d, crtc_width = %d\n",
		      overlay->fb_width, overlay->crtc_width);
}

static void lcdc_win_commit(struct rockchip_drm_manager *mgr, int zpos)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_win_data *win_data;
	int win = zpos;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (ctx->suspended)
		return;

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= ZPOS_MAX_NUM)
		return;

	win_data = lcdc_ops->get_win(ctx->drv, win);

	/* If suspended, enable this on resume */
	if (ctx->suspended) {
		win_data->resume = true;
		return;
	}

	lcdc_ops->win_commit(ctx->drv, win_data);
}

static void lcdc_win_disable(struct rockchip_drm_manager *mgr, int zpos)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_win_data *win_data;
	int win = zpos;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win >= ZPOS_MAX_NUM)
		return;

	win_data = lcdc_ops->get_win(ctx->drv, win);

	if (ctx->suspended) {
		/* do not resume this window*/
		win_data->resume = false;
		return;
	}

	win_data->enabled = false;
	lcdc_ops->win_commit(ctx->drv, win_data);
}

static void lcdc_window_suspend(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_win_data *win_data;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);
	int i;

	for (i = 0; i < ZPOS_MAX_NUM; i++) {
		win_data = lcdc_ops->get_win(ctx->drv, i);
		win_data->resume = win_data->enabled;
		if (win_data->enabled)
			lcdc_win_disable(mgr, i);
	}
	lcdc_wait_for_vblank(mgr);
}

static void lcdc_window_resume(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_win_data *win_data;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);
	int i;

	for (i = 0; i < ZPOS_MAX_NUM; i++) {
		win_data = lcdc_ops->get_win(ctx->drv, i);
		win_data->enabled = win_data->resume;
		win_data->resume = false;
	}
}

static void lcdc_apply(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_win_data *win_data;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);
	int i;

	for (i = 0; i < ZPOS_MAX_NUM; i++) {
		win_data = lcdc_ops->get_win(ctx->drv, i);
		if (win_data->enabled)
			lcdc_win_commit(mgr, i);
	}

	lcdc_commit(mgr);
}

static int lcdc_poweron(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (!ctx->suspended)
		return 0;

	ctx->suspended = false;

	pm_runtime_get_sync(ctx->dev);

	lcdc_ops->dpms(ctx->drv, DRM_MODE_DPMS_ON);

	lcdc_window_resume(mgr);

	lcdc_apply(mgr);

	return 0;
}

static int lcdc_poweroff(struct rockchip_drm_manager *mgr)
{
	struct lcdc_context *ctx = mgr->ctx;
	struct lcdc_driver_ops *lcdc_ops = to_lcdc_ops(ctx);

	if (ctx->suspended)
		return 0;

	/*
	 * We need to make sure that all windows are disabled before we
	 * suspend that connector. Otherwise we might try to scan from
	 * a destroyed buffer later.
	 */
	lcdc_window_suspend(mgr);

	lcdc_ops->dpms(ctx->drv, DRM_MODE_DPMS_OFF);

	pm_runtime_put_sync(ctx->dev);

	ctx->suspended = true;

	return 0;
}

static void lcdc_dpms(struct rockchip_drm_manager *mgr, int mode)
{
	DRM_DEBUG_KMS("%s, %d\n", __FILE__, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		lcdc_poweron(mgr);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		lcdc_poweroff(mgr);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}
}

static struct rockchip_drm_manager_ops lcdc_manager_ops = {
	.dpms = lcdc_dpms,
	.mode_fixup = lcdc_mode_fixup,
	.mode_set = lcdc_mode_set,
	.commit = lcdc_commit,
	.enable_vblank = lcdc_enable_vblank,
	.disable_vblank = lcdc_disable_vblank,
	.wait_for_vblank = lcdc_wait_for_vblank,
	.win_mode_set = lcdc_win_mode_set,
	.win_commit = lcdc_win_commit,
	.win_disable = lcdc_win_disable,
};

void lcdc_vsync_event_handler(struct device *dev)
{
	struct rockchip_drm_manager *mgr = dev_get_drvdata(dev);
	struct lcdc_context *ctx = mgr->ctx;
	/* check the crtc is detached already from encoder */
	if (ctx->pipe < 0 || !ctx->drm_dev)
		return;

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	rockchip_drm_crtc_finish_pageflip(ctx->drm_dev, ctx->pipe);

	/* set wait vsync event to zero and wake up queue. */
	if (atomic_read(&ctx->wait_vsync_event)) {
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
	}
}

static int lcdc_bind(struct device *dev, struct device *master, void *data)
{
	struct rockchip_drm_manager *mgr = dev_get_drvdata(dev);
	struct lcdc_context *ctx = mgr->ctx;
	struct drm_device *drm_dev = data;

	lcdc_mgr_initialize(mgr, drm_dev);
	rockchip_drm_crtc_create(mgr);

	if (ctx->display)
		rockchip_drm_create_enc_conn(drm_dev, ctx->display);

	return 0;
}

static void lcdc_unbind(struct device *dev, struct device *master,
			void *data)
{
	struct rockchip_drm_manager *mgr = dev_get_drvdata(dev);
	struct lcdc_context *ctx = mgr->ctx;
	struct drm_crtc *crtc = mgr->crtc;

	lcdc_dpms(mgr, DRM_MODE_DPMS_OFF);

	if (ctx->display)
		rockchip_dp_remove(dev);

	lcdc_mgr_remove(mgr);

	crtc->funcs->destroy(crtc);
}

static const struct component_ops lcdc_component_ops = {
	.bind = lcdc_bind,
	.unbind = lcdc_unbind,
};

static int lcdc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcdc_context *ctx;
	struct rockchip_drm_manager *lcdc_manager;
	struct lcdc_driver *lcdc_drv;
	struct lcdc_driver_data *lcdc_data = drm_lcdc_get_driver_data(pdev);
	int ret = -EINVAL;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (!lcdc_data->num_win) {
		DRM_ERROR("can not find display win\n");
		return -EINVAL;
	}

	if (!(lcdc_data->ops && lcdc_data->ops->init &&
	      lcdc_data->ops->deinit && lcdc_data->ops->dpms &&
	      lcdc_data->ops->mode_set && lcdc_data->ops->enable_vblank &&
	      lcdc_data->ops->disable_vblank &&
	      lcdc_data->ops->win_commit)) {
		DRM_ERROR("lcdc driver ops is Incomplete\n");
		return -EINVAL;
	}

	lcdc_drv = lcdc_data->ops->init(pdev);
	if (!lcdc_drv)
		return -EINVAL;

	lcdc_drv->data = lcdc_data;

	lcdc_manager = devm_kzalloc(dev, sizeof(*lcdc_manager), GFP_KERNEL);
	if (!lcdc_manager) {
		ret = -ENOMEM;
		goto err_deinit_lcdc;
	}

	lcdc_manager->type = (lcdc_drv->prop == 0) ? ROCKCHIP_DISPLAY_TYPE_LCD
						: ROCKCHIP_DISPLAY_TYPE_HDMI;
	lcdc_manager->ops = &lcdc_manager_ops;

	ret = rockchip_drm_component_add(&pdev->dev, ROCKCHIP_DEVICE_TYPE_CRTC,
					 lcdc_manager->type);
	if (ret)
		goto err_deinit_lcdc;

	ctx->dev = dev;
	ctx->suspended = true;
	ctx->default_win = ZPOS_DEFAULT_WIN;

	ctx->drv = lcdc_drv;

	init_waitqueue_head(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	platform_set_drvdata(pdev, lcdc_manager);

	lcdc_manager->ctx = ctx;

	pm_runtime_enable(&pdev->dev);

	ret = component_add(&pdev->dev, &lcdc_component_ops);
	if (ret)
		goto err_disable_pm_runtime;

	if (lcdc_manager->type == ROCKCHIP_DISPLAY_TYPE_LCD)
		ctx->display = rockchip_dp_probe(&pdev->dev);

	return ret;

err_disable_pm_runtime:
	pm_runtime_disable(dev);
	rockchip_drm_component_del(dev, ROCKCHIP_DEVICE_TYPE_CRTC);
err_deinit_lcdc:
	lcdc_data->ops->deinit(ctx->drv);
	return ret;
}

static int lcdc_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	component_del(&pdev->dev, &lcdc_component_ops);
	rockchip_drm_component_del(&pdev->dev, ROCKCHIP_DEVICE_TYPE_CRTC);

	return 0;
}

struct platform_driver lcdc_platform_driver = {
	.probe = lcdc_probe,
	.remove = lcdc_remove,
	.driver = {
		.name = "rockchip-lcdc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(lcdc_driver_dt_match),
	},
};

module_platform_driver(lcdc_platform_driver);
