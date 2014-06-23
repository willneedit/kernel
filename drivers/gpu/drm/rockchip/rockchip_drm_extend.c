/*
 * rockchip_drm_extend.c
 *
 * Copyright (C) ROCKCHIP, Inc.
 * Author:yzq<yzq@rock-chips.com>
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
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>

#include <video/of_display_timing.h>
#include <drm/rockchip_drm.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/iomap.h>
#include <linux/rk_fb.h>
#include <video/display_timing.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/iomap.h>
#include <linux/rockchip/grf.h>
#include "rockchip_drm_drv.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_crtc.h"
#include "rockchip_drm_iommu.h"
#include "rockchip_drm_extend.h"
static struct device *g_dev = NULL;
static int extend_activate(struct extend_context *ctx, bool enable);
static const char generic_edid[] = {
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x49, 0x63, 0x48, 0x44, 0x01, 0x00, 0x00, 0x00,
	0x14, 0x0E, 0x01, 0x03, 0x80, 0x50, 0x2D, 0x78,
	0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
	0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
	0x00, 0xFA, 0x00, 0x1C, 0x16, 0x20, 0x58, 0x2C,
	0x25, 0x00, 0xC4, 0x8E, 0x21, 0x00, 0x00, 0x0A,
	0x00, 0x00, 0x00, 0xFA, 0x00, 0xE0, 0x2D, 0x10,
	0x10, 0x3E, 0x02, 0x00, 0x13, 0x8E, 0x21, 0x00,
	0x00, 0x0A, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0,
	0x1E, 0x20, 0x00, 0x28, 0x45, 0x00, 0x0A, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC,
	0x00, 0x47, 0x45, 0x4E, 0x45, 0x52, 0x49, 0x43,
	0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x50,
	0x02, 0x03, 0x22, 0x71, 0x46, 0x85, 0x01, 0x02,
	0x01, 0x06, 0x10, 0x23, 0x09, 0x7F, 0x07, 0x23,
	0x0F, 0x3F, 0x07, 0x23, 0x15, 0x07, 0x38, 0x83,
	0x2F, 0x00, 0x00, 0x66, 0x03, 0x0C, 0x00, 0x10,
	0x00, 0x80, 0x01, 0x1D, 0x00, 0x72, 0x51, 0xD0,
	0x1E, 0x20, 0x6E, 0x28, 0x55, 0x00, 0xC4, 0x8E,
	0x21, 0x00, 0x00, 0x1E, 0xD6, 0x09, 0x80, 0xA0,
	0x20, 0xE0, 0x2D, 0x10, 0x08, 0x60, 0x22, 0x00,
	0x12, 0x8E, 0x21, 0x08, 0x08, 0x18, 0x8C, 0x0A,
	0xA0, 0x14, 0x51, 0xF0, 0x16, 0x00, 0x26, 0x7C,
	0x43, 0x00, 0x13, 0x8E, 0x21, 0x00, 0x00, 0x98,
	0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
	0x58, 0x2C, 0x45, 0x00, 0xC4, 0x8E, 0x21, 0x00,
	0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xED,
};

static bool extend_display_is_connected(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	DRM_DEBUG_KMS("%s\n", __FILE__);

	return drm_disp->is_connected ? true : false;

	/* TODO. */
}

static void *extend_get_panel(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	struct list_head *pos;
	struct fb_modelist *modelist;
	struct fb_videomode *mode;

	DRM_DEBUG_KMS("%s\n", __FILE__);
	if (!drm_disp->is_connected)
		return NULL;
	list_for_each(pos, drm_disp->modelist) {
		modelist = list_entry(pos, struct fb_modelist, list);
		mode = &modelist->mode;
		if (mode->flag == HDMI_VIDEO_DEFAULT_MODE)
			break;
	}

	memcpy(&ctx->panel->timing, mode, sizeof(struct fb_videomode));

	return ctx->panel;
}

static void *extend_get_modelist(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;

	return drm_disp->modelist;
}

static int extend_check_timing(struct device *dev, void *timing_t)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	struct list_head *pos;
	struct fb_modelist *modelist;
	struct fb_videomode *mode;
	struct fb_videomode *timing = timing_t;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* TODO. */
	list_for_each(pos, drm_disp->modelist) {
		modelist = list_entry(pos, struct fb_modelist, list);
		mode = &modelist->mode;
		if (mode->xres == timing->xres
		    && mode->yres == timing->yres
		    && mode->pixclock == timing->pixclock
		    && mode->refresh == timing->refresh)
			return 0;
	}
	return -1;
}

static int extend_display_power_on(struct device *dev, int mode)
{
	struct extend_context *ctx = get_extend_context(dev);
	DRM_DEBUG_KMS("%s\n", __FILE__);
	/* TODO */
	extend_activate(ctx, mode == DRM_MODE_DPMS_ON ? true : false);

	return 0;
}

struct edid *extend_get_edid(struct device *dev,
			     struct drm_connector *connector)
{
	int i, j = 0, valid_extensions = 0;
	u8 *block, *new;
	struct edid *edid = NULL;
	struct edid *raw_edid = NULL;
	bool print_bad_edid = !connector->bad_edid_counter;
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;

	if ((block = kmalloc(EDID_LENGTH, GFP_KERNEL)) == NULL)
		return NULL;
	/* base block fetch */
	for (i = 0; i < 4; i++) {
		if(drm_disp->screen_ops.getedid(drm_disp, 0, block))
			goto out;
		if (drm_edid_block_valid(block, 0, print_bad_edid))
			break;
	}
	if (i == 4)
		goto carp;

	/* if there's no extensions, we're done */
	if (block[0x7e] == 0)
		return block;

	new = krealloc(block, (block[0x7e] + 1) * EDID_LENGTH, GFP_KERNEL);
	if (!new)
		goto out;
	block = new;

	for (j = 1; j <= block[0x7e]; j++) {
		for (i = 0; i < 4; i++) {
			if (drm_disp->screen_ops.getedid(drm_disp, 0, block))
				goto out;
			if (drm_edid_block_valid
			    (block + (valid_extensions + 1) * EDID_LENGTH, j,
			     print_bad_edid)) {
				valid_extensions++;
				break;
			}
		}

		if (i == 4 && print_bad_edid) {
			dev_warn(connector->dev->dev,
				 "%s: Ignoring invalid EDID block %d.\n",
				 drm_get_connector_name(connector), j);

			connector->bad_edid_counter++;
		}
	}

	if (valid_extensions != block[0x7e]) {
		block[EDID_LENGTH - 1] += block[0x7e] - valid_extensions;
		block[0x7e] = valid_extensions;
		new =
		    krealloc(block, (valid_extensions + 1) * EDID_LENGTH,
			     GFP_KERNEL);
		if (!new)
			goto out;
		block = new;
	}
	edid = (struct edid *)block;
	return edid;

carp:
	if (print_bad_edid) {
		dev_warn(connector->dev->dev, "%s: EDID block %d invalid.\n",
			 drm_get_connector_name(connector), j);
	}
	connector->bad_edid_counter++;

out:
	kfree(block);
	raw_edid = (struct edid *)generic_edid;
	edid =
	    kmemdup(raw_edid, (1 + raw_edid->extensions) * EDID_LENGTH,
		    GFP_KERNEL);
	if (!edid) {
		DRM_DEBUG_KMS("failed to allocate edid\n");
		return ERR_PTR(-ENOMEM);
	}
	return edid;
}

static struct rockchip_drm_display_ops extend_display_ops = {
	.type = ROCKCHIP_DISPLAY_TYPE_HDMI,
	.is_connected = extend_display_is_connected,
	.get_panel = extend_get_panel,
	.get_modelist = extend_get_modelist,
	.check_timing = extend_check_timing,
	.power_on = extend_display_power_on,
	.get_edid = extend_get_edid,
};

static void extend_dpms(struct device *subdrv_dev, int mode)
{
	struct extend_context *ctx = get_extend_context(subdrv_dev);

	DRM_DEBUG_KMS("%s, %d\n", __FILE__, mode);

	mutex_lock(&ctx->lock);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		/*
		 * enable primary hardware only if suspended status.
		 *
		 * P.S. extend_dpms function would be called at booting time so
		 * clk_enable could be called double time.
		 */

		if (ctx->suspended)
			pm_runtime_get_sync(subdrv_dev);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		if (!ctx->suspended)
			pm_runtime_put_sync(subdrv_dev);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}

	mutex_unlock(&ctx->lock);
}

static void extend_apply(struct device *subdrv_dev)
{
	struct extend_context *ctx = get_extend_context(subdrv_dev);
	struct rockchip_drm_manager *mgr = ctx->subdrv.manager;
	struct rockchip_drm_manager_ops *mgr_ops = mgr->ops;
	struct rockchip_drm_overlay_ops *ovl_ops = mgr->overlay_ops;
	struct extend_win_data *win_data;
	int i;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	for (i = 0; i < WINDOWS_NR; i++) {
		win_data = &ctx->win_data[i];
		if (win_data->enabled && (ovl_ops && ovl_ops->commit))
			ovl_ops->commit(subdrv_dev, i);
	}

	if (mgr_ops && mgr_ops->commit)
		mgr_ops->commit(subdrv_dev);
}

static void extend_commit(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	struct rockchip_drm_panel_info *panel =
	    (struct rockchip_drm_panel_info *)extend_get_panel(dev);

	if (ctx->suspended)
		return;

	drm_disp->mode = &panel->timing;

	drm_disp->enable = true;
	rk_drm_disp_handle(drm_disp, RK_DRM_SCREEN_SET, 0);
}

static int extend_enable_vblank(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return -EPERM;

	ctx->vblank_en = true;
	return 0;
}

static void extend_disable_vblank(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (ctx->suspended)
		return;

	ctx->vblank_en = false;
}

static void extend_wait_for_vblank(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);

	if (ctx->suspended)
		return;

	atomic_set(&ctx->wait_vsync_event, 1);

	if (!wait_event_timeout(ctx->wait_vsync_queue,
				!atomic_read(&ctx->wait_vsync_event),
				DRM_HZ / 20))
		DRM_DEBUG_KMS("vblank wait timed out.\n");
}

static void extend_event_call_back_handle(struct rk_drm_display *drm_disp,
					  int win_id, int event)
{
	struct extend_context *ctx = get_extend_context(g_dev);
	struct rockchip_drm_subdrv *subdrv = &ctx->subdrv;
	struct rockchip_drm_manager *manager = subdrv->manager;
	struct drm_device *drm_dev = subdrv->drm_dev;
	switch (event) {
	case RK_DRM_CALLBACK_VSYNC:
		/* check the crtc is detached already from encoder */
		if (manager->pipe < 0)
			return;

		drm_handle_vblank(drm_dev, manager->pipe);
		rockchip_drm_crtc_finish_pageflip(drm_dev, manager->pipe);

		/* set wait vsync event to zero and wake up queue. */
		if (atomic_read(&ctx->wait_vsync_event)) {
			atomic_set(&ctx->wait_vsync_event, 0);
			DRM_WAKEUP(&ctx->wait_vsync_queue);
		}
		break;
	case RK_DRM_CALLBACK_HOTPLUG:
		DRM_DEBUG_KMS("-->%s hotplug event %d\n", __func__, event);
		drm_helper_hpd_irq_event(drm_dev);
		break;

	default:
		DRM_ERROR("-->%s unhandle event %d\n", __func__, event);
		break;
	}
}

static void extend_get_max_resol(void *ctx, unsigned int *width,
				 unsigned int *height)
{
	DRM_DEBUG_KMS("[%d] %s\n", __LINE__, __func__);

	*width = MAX_HDMI_WIDTH;
	*height = MAX_HDMI_HEIGHT;
}

static struct rockchip_drm_manager_ops extend_manager_ops = {
	.dpms 		= extend_dpms,
	.apply 		= extend_apply,
	.commit 	= extend_commit,
	.enable_vblank 	= extend_enable_vblank,
	.disable_vblank = extend_disable_vblank,
	.wait_for_vblank = extend_wait_for_vblank,
	.get_max_resol 	= extend_get_max_resol,
};

static void extend_win_mode_set(struct device *dev,
				struct rockchip_drm_overlay *overlay)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	struct extend_win_data *win_data;
	int win;
	unsigned long offset;
	struct list_head *pos, *head;
	struct fb_modelist *modelist;
	struct fb_videomode *mode;
	struct drm_display_mode *disp_mode = NULL;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (!overlay) {
		dev_err(dev, "overlay is NULL\n");
		return;
	}

	win = overlay->zpos;
	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win > WINDOWS_NR)
		return;

	offset = overlay->fb_x * (overlay->bpp >> 3);
	offset += overlay->fb_y * overlay->pitch;

	win_data = &ctx->win_data[win];

	win_data->offset_x = overlay->crtc_x;
	win_data->offset_y = overlay->crtc_y;
	win_data->ovl_width = overlay->crtc_width;
	win_data->ovl_height = overlay->crtc_height;
	win_data->fb_width = overlay->fb_width;
	win_data->fb_height = overlay->fb_height;
	win_data->dma_addr = overlay->dma_addr[0] + offset;
	win_data->bpp = overlay->bpp;
	win_data->buf_offsize = (overlay->fb_width - overlay->crtc_width) *
	    (overlay->bpp >> 3);
	win_data->line_size = overlay->pitch;
	head = drm_disp->modelist;
	list_for_each(pos, head) {
		modelist = list_entry(pos, struct fb_modelist, list);
		mode = &modelist->mode;
		if (mode->xres == overlay->mode_width
		    && mode->yres == overlay->mode_height
		    && mode->pixclock == overlay->pixclock)
			break;
	}
	if (drm_disp->mode != mode) {
		drm_disp->mode = mode;
		drm_disp->enable = true;
		rk_drm_disp_handle(drm_disp, RK_DRM_SCREEN_SET, 0);
	}

}

static void extend_win_set_pixfmt(struct device *dev, unsigned int win)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct extend_win_data *win_data = &ctx->win_data[win];
}

static void extend_win_set_colkey(struct device *dev, unsigned int win)
{
	//      struct extend_context *ctx = get_extend_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

}

void extend_overlay_commit(struct rk_overlay_api *ovl)
{
	struct extend_context *ctx = get_extend_context(g_dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	struct rk_win_data *rk_win = NULL;
	struct extend_win_data *win_data;
	int win;

	if (ctx->suspended)
		return;
	win = LAYER_VIDEO_WIN;
	rk_win = &drm_disp->win[win];
	win_data = &ctx->win_data[win];

	if (ovl->xact || ovl->yact) {
		rk_win->format = YUV420;
		rk_win->xact = ovl->xact;
		rk_win->yact = ovl->yact;
		rk_win->xvir = ovl->xvir;
	} else {
		rk_win->xpos = ovl->xpos;
		rk_win->ypos = ovl->ypos;
		rk_win->xsize = ovl->xsize;
		rk_win->ysize = ovl->ysize;
	}

	if (ovl->y_addr && rk_win->xact) {
		rk_win->yrgb_addr = ovl->y_addr;
		rk_win->uv_addr = ovl->uv_addr;
		rk_win->enabled = true;

		rk_drm_disp_handle(drm_disp,
				   RK_DRM_WIN_COMMIT | RK_DRM_DISPLAY_COMMIT,
				   1 << win);

		win_data->enabled = true;

	}

}

static void extend_win_commit(struct device *dev, int zpos)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	struct rk_win_data *rk_win = NULL;
	struct extend_win_data *win_data;
	int win = zpos;
	unsigned long val, size;
	u32 xpos, ypos;
	int rk_win_id = win;

	if (ctx->suspended)
		return;

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win > WINDOWS_NR)
		return;
	win_data = &ctx->win_data[win];
	switch (win) {
	case 0:
		rk_win_id = LAYER_NORMAL_WIN;
		break;
	case 1:
		rk_win_id = LAYER_CURSOR_WIN;
		break;
	default:
		DRM_ERROR("un support win id %d\n", win);
	}
	rk_win = &drm_disp->win[rk_win_id];

	switch (win_data->bpp) {
	case 32:
		rk_win->format = ARGB888;
		rk_win->alpha_en = true;
		break;
	case 24:
		rk_win->format = RGB888;
		break;
	case 16:
		rk_win->format = RGB565;
		break;
	default:
		DRM_ERROR("not support format %d\n", win_data->bpp);
		break;
	}

	rk_win->xpos = win_data->offset_x;
	rk_win->ypos = win_data->offset_y;
	rk_win->xact = win_data->ovl_width;
	rk_win->yact = win_data->ovl_height;
	rk_win->xsize = win_data->ovl_width;
	rk_win->ysize = win_data->ovl_height;
	rk_win->xvir = win_data->line_size / (win_data->bpp >> 3);
	rk_win->yrgb_addr = win_data->dma_addr;
	rk_win->enabled = true;

	rk_drm_disp_handle(drm_disp, RK_DRM_WIN_COMMIT | RK_DRM_DISPLAY_COMMIT,
			   1 << rk_win_id);

	win_data->enabled = true;
}

static void extend_win_disable(struct device *dev, int zpos)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct rk_drm_display *drm_disp = ctx->drm_disp;
	struct extend_win_data *win_data;
	int win = zpos;
	int rk_win_id = win;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	if (win == DEFAULT_ZPOS)
		win = ctx->default_win;

	if (win < 0 || win > WINDOWS_NR)
		return;

	win_data = &ctx->win_data[win];

	if (ctx->suspended) {
		/* do not resume this window */
		win_data->resume = false;
		return;
	}
	switch (win) {
	case 0:
		rk_win_id = LAYER_NORMAL_WIN;
		break;
	case 1:
		rk_win_id = LAYER_CURSOR_WIN;
		break;
	default:
		DRM_ERROR("un support win id\n");
	}
	drm_disp->win[rk_win_id].enabled = false;
	rk_drm_disp_handle(drm_disp, RK_DRM_WIN_COMMIT | RK_DRM_DISPLAY_COMMIT,
			   1 << rk_win_id);

	win_data->enabled = false;
}

static struct rockchip_drm_overlay_ops extend_overlay_ops = {
	.mode_set = extend_win_mode_set,
	.commit = extend_win_commit,
	.disable = extend_win_disable,
};

static struct rockchip_drm_manager extend_manager = {
	.pipe 		= -1,
	.ops 		= &extend_manager_ops,
	.overlay_ops 	= &extend_overlay_ops,
	.display_ops 	= &extend_display_ops,
};

static int extend_subdrv_probe(struct drm_device *drm_dev, struct device *dev)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = 1, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *      just specific driver own one instead because
	 *      drm framework supports only one irq handler.
	 */
	drm_dev->irq_enabled = 1;

	/*
	 * with vblank_disable_allowed = 1, vblank interrupt will be disabled
	 * by drm timer once a current process gives up ownership of
	 * vblank event.(after drm_vblank_put function is called)
	 */
	drm_dev->vblank_disable_allowed = 1;

	/* attach this sub driver to iommu mapping if supported. */
	if (is_drm_iommu_supported(drm_dev))
		drm_iommu_attach_device(drm_dev, dev);

	return 0;
}

static void extend_subdrv_remove(struct drm_device *drm_dev, struct device *dev)
{
	DRM_DEBUG_KMS("%s\n", __FILE__);

	/* detach this sub driver from iommu mapping if supported. */
	if (is_drm_iommu_supported(drm_dev))
		drm_iommu_detach_device(drm_dev, dev);
}

static void extend_clear_win(struct extend_context *ctx, int win)
{
	u32 val;

	DRM_DEBUG_KMS("%s\n", __FILE__);

}

static void extend_window_suspend(struct device *dev)
{
	struct extend_win_data *win_data = NULL;
	struct extend_context *ctx = get_extend_context(dev);
	int i;

	for (i = 0; i < WINDOWS_NR; i++) {
		win_data = &ctx->win_data[i];
		win_data->resume = win_data->enabled;
		extend_win_disable(dev, i);
	}
	extend_wait_for_vblank(dev);
}

static void extend_window_resume(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);
	struct extend_win_data *win_data;
	int i;

	for (i = 0; i < WINDOWS_NR; i++) {
		win_data = &ctx->win_data[i];
		win_data->enabled = win_data->resume;
		win_data->resume = false;
	}
}

static int extend_activate(struct extend_context *ctx, bool enable)
{
	struct device *dev = ctx->subdrv.dev;
	struct rk_drm_display *drm_disp = ctx->drm_disp;

	if (enable) {
		int ret;

		ctx->suspended = false;

		drm_disp->enable = true;

		rk_drm_disp_handle(drm_disp, 0, RK_DRM_SCREEN_BLANK);

		/* if vblank was enabled status, enable it again. */
		if (ctx->vblank_en)
			extend_enable_vblank(dev);

		extend_window_resume(dev);
	} else {
		extend_window_suspend(dev);

		drm_disp->enable = false;

		rk_drm_disp_handle(drm_disp, 0, RK_DRM_SCREEN_BLANK);

		ctx->suspended = true;
	}

	return 0;
}

static int extend_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct extend_context *ctx;
	struct rockchip_drm_subdrv *subdrv;
	struct rockchip_drm_panel_info *panel;
	struct rk_drm_display *drm_display = NULL;
	int ret = -EINVAL;

	DRM_DEBUG_KMS("%s\n", __FILE__);

	g_dev = dev;
	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	panel =
	    devm_kzalloc(dev, sizeof(struct rockchip_drm_panel_info),
			 GFP_KERNEL);
	ctx->panel = panel;

	drm_display = rk_drm_get_diplay(RK_DRM_EXTEND_SCREEN);
	ctx->drm_disp = drm_display;
	ctx->default_win = 0;

	drm_display->event_call_back = extend_event_call_back_handle;

	DRM_INIT_WAITQUEUE(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	subdrv = &ctx->subdrv;

	subdrv->dev = dev;
	subdrv->manager = &extend_manager;
	subdrv->probe = extend_subdrv_probe;
	subdrv->remove = extend_subdrv_remove;

	mutex_init(&ctx->lock);

	platform_set_drvdata(pdev, ctx);

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	//extend_commit(dev);
	extend_activate(ctx, true);

	rockchip_drm_subdrv_register(subdrv);

	return 0;
}

static int extend_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct extend_context *ctx = platform_get_drvdata(pdev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	rockchip_drm_subdrv_unregister(&ctx->subdrv);

	if (ctx->suspended)
		goto out;

	pm_runtime_set_suspended(dev);
	pm_runtime_put_sync(dev);

out:
	pm_runtime_disable(dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int extend_suspend(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);

	/*
	 * do not use pm_runtime_suspend(). if pm_runtime_suspend() is
	 * called here, an error would be returned by that interface
	 * because the usage_count of pm runtime is more than 1.
	 */
	if (!pm_runtime_suspended(dev))
		return extend_activate(ctx, false);

	return 0;
}

static int extend_resume(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);

	/*
	 * if entered to sleep when lcd panel was on, the usage_count
	 * of pm runtime would still be 1 so in this case, fimd driver
	 * should be on directly not drawing on pm runtime interface.
	 */
	if (!pm_runtime_suspended(dev)) {
		int ret;

		ret = extend_activate(ctx, true);
		if (ret < 0)
			return ret;

		/*
		 * in case of dpms on(standby), extend_apply function will
		 * be called by encoder's dpms callback to update fimd's
		 * registers but in case of sleep wakeup, it's not.
		 * so extend_apply function should be called at here.
		 */
		extend_apply(dev);
	}

	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int extend_runtime_suspend(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	return extend_activate(ctx, false);
}

static int extend_runtime_resume(struct device *dev)
{
	struct extend_context *ctx = get_extend_context(dev);

	DRM_DEBUG_KMS("%s\n", __FILE__);

	return extend_activate(ctx, true);
}
#endif

static const struct dev_pm_ops extend_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(extend_suspend, extend_resume)
	    SET_RUNTIME_PM_OPS(extend_runtime_suspend, extend_runtime_resume,
			       NULL)
};

struct platform_driver extend_platform_driver = {
	.probe = extend_probe,
	.remove = extend_remove,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "extend-display",
		   .pm = &extend_pm_ops,
		   },
};
