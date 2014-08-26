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
#include <linux/delay.h>
#include <drm/rockchip_drm.h>

#include <video/of_display_timing.h>
#include <video/of_videomode.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_vop.h"

#define VOP_DEFAULT_FRAMERATE 60
#define VOP_MAX_WIN_SUPPORT 5
#define VOP_REG(off, msk, s) \
		{.offset = off, \
		 .mask = msk, \
		 .shift = s,}

#define __REG_SET(x, off, msk, shift, v) \
		vop_msk_write(x, off, (msk) << shift, (v) << shift)

#define REG_SET(x, base, reg, v) \
		__REG_SET(x, base + reg.offset, reg.mask, reg.shift, v)

#define VOP_WIN_SET(x, win, name, v) \
		REG_SET(x, win->base, win->phy->name, v)
#define VOP_CTRL_SET(x, name, v) \
		REG_SET(x, 0, (x)->data->ctrl->name, v)

#define to_vop_ctx(x) container_of(x, struct vop_context, crtc)
#define to_rockchip_plane(x) container_of(x, struct rockchip_plane, base)

struct rockchip_plane {
	int id;
	struct drm_plane base;
	const struct vop_win *win;
	struct vop_context *ctx;
};

struct vop_context {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_crtc crtc;
	struct drm_pending_vblank_event *event;
	struct drm_plane *plane;
	struct vop_driver *drv;
	unsigned int dpms;
	unsigned int win_msk;
	wait_queue_head_t wait_vsync_queue;
	atomic_t wait_vsync_event;

	struct vop_driver_data *data;

	void *regsbak;
	void __iomem *regs;

	/* physical map length of vop register */
	uint32_t len;

	/* one time only one process allowed to config the register */
	spinlock_t reg_lock;
	/* lock vop irq reg */
	spinlock_t irq_lock;

	unsigned int irq;

	/* vop AHP clk */
	struct clk *hclk;
	/* vop dclk */
	struct clk *dclk;
	/* vop share memory frequency */
	struct clk *aclk;
	uint32_t pixclock;

	int pipe;
	bool clk_on;
};

enum vop_data_format {
	ARGB888 = 0,
	RGB888,
	RGB565,
	YUV420SP = 4,
	YUV422SP,
	YUV444SP,
};

struct vop_reg_data {
	uint32_t offset;
	uint32_t value;
};

struct vop_reg {
	uint32_t offset;
	uint32_t shift;
	uint32_t mask;
};

struct vop_ctrl {
	struct vop_reg standby;
	struct vop_reg gate_en;
	struct vop_reg mmu_en;
	struct vop_reg rgb_en;
	struct vop_reg edp_en;
	struct vop_reg hdmi_en;
	struct vop_reg mipi_en;
	struct vop_reg out_mode;
	struct vop_reg dither_down;
	struct vop_reg dither_up;
	struct vop_reg pin_sync;

	struct vop_reg htotal_pw;
	struct vop_reg hact_st_end;
	struct vop_reg vtotal_pw;
	struct vop_reg vact_st_end;
	struct vop_reg hpost_st_end;
	struct vop_reg vpost_st_end;
};

struct vop_win_phy {
	const uint32_t *data_formats;
	uint32_t nformats;

	struct vop_reg enable;
	struct vop_reg format;
	struct vop_reg act_info;
	struct vop_reg dsp_info;
	struct vop_reg dsp_st;
	struct vop_reg yrgb_addr;
	struct vop_reg uv_addr;
	struct vop_reg yrgb_vir;
	struct vop_reg uv_vir;

	struct vop_reg dst_alpha_ctl;
	struct vop_reg src_alpha_ctl;
};

struct vop_win {
	uint32_t base;
	const struct vop_win_phy *phy;
};

struct vop_driver_data {
	int id;
	const void *init_table;
	int table_size;
	const struct vop_ctrl *ctrl;
	const struct vop_win *win[VOP_MAX_WIN_SUPPORT];
};

static const uint32_t formats_01[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
};

static const uint32_t formats_234[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static const struct vop_win_phy win01_data = {
	.data_formats = formats_01,
	.nformats = ARRAY_SIZE(formats_01),
	.enable = VOP_REG(WIN0_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN0_CTRL0, 0x7, 1),
	.act_info = VOP_REG(WIN0_ACT_INFO, 0x1fff1fff, 0),
	.dsp_info = VOP_REG(WIN0_DSP_INFO, 0x1fff1fff, 0),
	.dsp_st = VOP_REG(WIN0_DSP_ST, 0x1fff1fff, 0),
	.yrgb_addr = VOP_REG(WIN0_YRGB_MST, 0xffffffff, 0),
	.uv_addr = VOP_REG(WIN0_CBR_MST, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN0_VIR, 0x3fff, 0),
	.uv_vir = VOP_REG(WIN0_VIR, 0x3fff, 16),
	.src_alpha_ctl = VOP_REG(WIN0_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN0_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy win23_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(WIN2_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN2_CTRL0, 0x7, 1),
	.dsp_info = VOP_REG(WIN2_DSP_INFO0, 0x0fff0fff, 0),
	.dsp_st = VOP_REG(WIN2_DSP_ST0, 0x1fff1fff, 0),
	.yrgb_addr = VOP_REG(WIN2_MST0, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN2_VIR0_1, 0x1fff, 0),
	.src_alpha_ctl = VOP_REG(WIN2_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN2_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy cursor_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(HWC_CTRL0, 0x1, 0),
	.format = VOP_REG(HWC_CTRL0, 0x7, 1),
	.dsp_st = VOP_REG(HWC_DSP_ST, 0x1fff1fff, 0),
	.yrgb_addr = VOP_REG(HWC_MST, 0xffffffff, 0),
};

static const struct vop_win win0 = {
	.base = 0,
	.phy = &win01_data,
};

static const struct vop_win win1 = {
	.base = 0x40,
	.phy = &win01_data,
};

static const struct vop_win win2 = {
	.base = 0,
	.phy = &win23_data,
};

static const struct vop_win win3 = {
	.base = 0x50,
	.phy = &win23_data,
};

static const struct vop_win win_cursor = {
	.base = 0,
	.phy = &cursor_data,
};

static const struct vop_ctrl ctrl_data = {
	.standby = VOP_REG(SYS_CTRL, 0x1, 22),
	.gate_en = VOP_REG(SYS_CTRL, 0x1, 23),
	.mmu_en = VOP_REG(SYS_CTRL, 0x1, 20),
	.rgb_en = VOP_REG(SYS_CTRL, 0x1, 12),
	.hdmi_en = VOP_REG(SYS_CTRL, 0x1, 13),
	.edp_en = VOP_REG(SYS_CTRL, 0x1, 14),
	.mipi_en = VOP_REG(SYS_CTRL, 0x1, 15),
	.dither_down = VOP_REG(DSP_CTRL1, 0xf, 1),
	.dither_up = VOP_REG(DSP_CTRL1, 0x1, 6),
	.out_mode = VOP_REG(DSP_CTRL0, 0xf, 0),
	.pin_sync = VOP_REG(DSP_CTRL0, 0xf, 4),
	.htotal_pw = VOP_REG(DSP_HTOTAL_HS_END, 0x1fff1fff, 0),
	.hact_st_end = VOP_REG(DSP_HACT_ST_END, 0x1fff1fff, 0),
	.vtotal_pw = VOP_REG(DSP_VTOTAL_VS_END, 0x1fff1fff, 0),
	.vact_st_end = VOP_REG(DSP_VACT_ST_END, 0x1fff1fff, 0),
	.hpost_st_end = VOP_REG(POST_DSP_HACT_INFO, 0x1fff1fff, 0),
	.vpost_st_end = VOP_REG(POST_DSP_VACT_INFO, 0x1fff1fff, 0),
};

static const struct vop_reg_data vop_init_reg_table[] = {
	{SYS_CTRL, 0x00801000},
	{WIN0_CTRL0, 0x00000080},
	{WIN1_CTRL0, 0x00000080},
};

static const struct vop_driver_data rockchip_rk3288_vopl = {
	.id = ROCKCHIP_CRTC_VOPL,
	.init_table = vop_init_reg_table,
	.table_size = ARRAY_SIZE(vop_init_reg_table),
	.ctrl = &ctrl_data,
	.win[0] = &win0,
	.win[1] = &win1,
	.win[2] = &win2,
	.win[3] = &win3,
	.win[4] = &win_cursor,
};

static const struct vop_driver_data rockchip_rk3288_vopb = {
	.id = ROCKCHIP_CRTC_VOPB,
	.init_table = vop_init_reg_table,
	.table_size = ARRAY_SIZE(vop_init_reg_table),
	.ctrl = &ctrl_data,
	.win[0] = &win0,
	.win[1] = &win1,
	.win[2] = &win2,
	.win[3] = &win3,
	.win[4] = &win_cursor,
};

static const struct of_device_id vop_driver_dt_match[] = {
	{ .compatible = "rockchip,rk3288-vopl",
	  .data = (void *)&rockchip_rk3288_vopl },
	{ .compatible = "rockchip,rk3288-vopb",
	  .data = (void *)&rockchip_rk3288_vopb },
	{},
};

static inline void vop_writel(struct vop_context *ctx,
			      uint32_t offset, uint32_t v)
{
	uint32_t *pv = ctx->regsbak + offset;

	*pv = v;
	writel(v, ctx->regs + offset);
}

static inline uint32_t vop_readl(struct vop_context *ctx, uint32_t offset)
{
	return readl(ctx->regs + offset);
}

static inline void vop_cfg_done(struct vop_context *ctx)
{
	writel(0x01, ctx->regs + REG_CFG_DONE);
}

static inline void vop_msk_write(struct vop_context *ctx,
				 uint32_t offset, uint32_t msk, uint32_t v)
{
	if (msk) {
		uint32_t *pv = ctx->regsbak + offset;

		*pv &= (~msk);
		*pv |= v;

		writel(*pv, ctx->regs + offset);
	}
}

static inline struct vop_driver_data *vop_get_driver_data(struct device *dev)
{
	const struct of_device_id *of_id =
			of_match_device(vop_driver_dt_match, dev);

	return (struct vop_driver_data *)of_id->data;
}

static enum vop_data_format vop_convert_format(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return ARGB888;
	case DRM_FORMAT_RGB888:
		return RGB888;
	case DRM_FORMAT_RGB565:
		return RGB565;
	case DRM_FORMAT_NV12:
		return YUV420SP;
	case DRM_FORMAT_NV16:
		return YUV422SP;
	case DRM_FORMAT_NV24:
		return YUV444SP;
	default:
		DRM_ERROR("unsupport format[%08x] so using unpacked 24bpp.\n",
			  format);
		return ARGB888;
	}
}

static bool is_alpha_support(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_ARGB8888:
		return true;
	default:
		return false;
	}
}

#ifdef CONFIG_ROCKCHIP_IOVMM
struct device *vop_get_sysmmu_dev(struct vop_context *ctx)
{
	struct device_node *mmu_node;
	struct platform_device *pd;

	mmu_node = of_parse_phandle(ctx->dev->of_node,
				    "rockchip,iommu", 0);
	if (!mmu_node) {
		DRM_ERROR("failed to find mmu_node\n");
		return NULL;
	}

	pd = of_find_device_by_node(mmu_node);
	if (!pd) {
		pr_err("can't find platform device in device node\n");
		of_node_put(mmu_node);
		return  NULL;
	}

	of_node_put(mmu_node);

	return &pd->dev;
}

int vop_sysmmu_fault_handler(struct device *dev, enum rk_iommu_inttype itype,
			     unsigned long pgtable_base,
			     unsigned long fault_addr, unsigned int status)
{
	DRM_ERROR("Generating Kernel OOPS... because it is unrecoverable.\n");

	return 0;
}

static inline void platform_set_sysmmu(struct device *sysmmu,
				       struct device *dev)
{
	dev->archdata.iommu = sysmmu;
}
#endif

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

static int vop_clk_enable(struct vop_context *ctx)
{
	int ret = 0;

	if (!ctx->clk_on) {
		ret = clk_prepare_enable(ctx->hclk);
		if (ret < 0) {
			dev_err(ctx->dev, "failed to enable hclk\n");
			return ret;
		}

		ret = clk_prepare_enable(ctx->dclk);
		if (ret < 0) {
			dev_err(ctx->dev, "failed to enable dclk\n");
			goto err_dclk;
		}

		ret = clk_prepare_enable(ctx->aclk);
		if (ret < 0) {
			dev_err(ctx->dev, "failed to enable aclk\n");
			goto err_aclk;
		}
		ctx->clk_on = true;
	}

	return ret;
err_aclk:
	clk_disable_unprepare(ctx->aclk);
err_dclk:
	clk_disable_unprepare(ctx->hclk);
	return ret;
}

static void vop_clk_disable(struct vop_context *ctx)
{
	if (ctx->clk_on) {
		clk_disable_unprepare(ctx->dclk);
		clk_disable_unprepare(ctx->hclk);
		clk_disable_unprepare(ctx->aclk);
		ctx->clk_on = false;
	}
}

static void vop_power_on(struct vop_context *ctx)
{
	if (vop_clk_enable(ctx) < 0) {
		dev_err(ctx->dev, "failed to enable ctx%d clks\n",
			ctx->pipe);
		return;
	}

	spin_lock(&ctx->reg_lock);

	VOP_CTRL_SET(ctx, standby, 0);

	spin_unlock(&ctx->reg_lock);
}

static void vop_power_off(struct vop_context *ctx)
{
	spin_lock(&ctx->reg_lock);

	VOP_CTRL_SET(ctx, standby, 1);

	spin_unlock(&ctx->reg_lock);

	vop_clk_disable(ctx);
}

static int rockchip_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
				 struct drm_framebuffer *fb, int crtc_x,
				 int crtc_y, unsigned int crtc_w,
				 unsigned int crtc_h, uint32_t src_x,
				 uint32_t src_y, uint32_t src_w, uint32_t src_h)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	const struct vop_win *win = rockchip_plane->win;
	struct vop_context *ctx = to_vop_ctx(crtc);
	struct drm_gem_object *obj;
	struct rockchip_gem_object *rk_obj;
	unsigned long offset;
	unsigned int actual_w;
	unsigned int actual_h;
	unsigned int dsp_stx;
	unsigned int dsp_sty;
	unsigned int y_vir_stride;
	enum vop_data_format format;
	bool is_alpha;
	int ret;

	if (!win) {
		DRM_ERROR("can't find win data for vop, failed\n");
		return -EINVAL;
	}

	obj = rockchip_fb_get_gem_obj(fb, 0);
	if (!obj) {
		DRM_ERROR("fail to get rockchip gem object from framebuffer\n");
		return -EINVAL;
	}

	rk_obj = to_rockchip_obj(obj);

	if (!rk_obj->paddr) {
		ret = rockchip_iommu_mmap(ctx->dev, rk_obj);
		if (ret < 0)
			return ret;
	}

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

	dsp_stx = crtc_x + crtc->mode.htotal - crtc->mode.hsync_start;
	dsp_sty = crtc_y + crtc->mode.vtotal - crtc->mode.vsync_start;

	offset = src_x * (fb->bits_per_pixel >> 3);
	offset += src_y * fb->pitches[0];

	y_vir_stride = fb->pitches[0] / (fb->bits_per_pixel >> 3);
	is_alpha = is_alpha_support(fb->pixel_format);
	format = vop_convert_format(fb->pixel_format);

	spin_lock(&ctx->reg_lock);

	VOP_WIN_SET(ctx, win, format, format);
	VOP_WIN_SET(ctx, win, yrgb_vir, y_vir_stride);
	VOP_WIN_SET(ctx, win, yrgb_addr, rk_obj->paddr + offset);
	VOP_WIN_SET(ctx, win, act_info,
		    ((actual_h - 1) << 16) | (actual_w - 1));
	VOP_WIN_SET(ctx, win, dsp_info,
		    ((actual_h - 1) << 16) | (actual_w - 1));
	VOP_WIN_SET(ctx, win, dsp_st, (dsp_sty << 16) | dsp_stx);
	if (is_alpha) {
		VOP_WIN_SET(ctx, win, dst_alpha_ctl, 0xc0);
		VOP_WIN_SET(ctx, win, src_alpha_ctl, 0x69);
	} else {
		VOP_WIN_SET(ctx, win, src_alpha_ctl, 0);
	}

	VOP_WIN_SET(ctx, win, enable, 1);

	vop_cfg_done(ctx);

	spin_unlock(&ctx->reg_lock);

	return 0;
}

static int rockchip_disable_plane(struct drm_plane *plane)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	struct vop_context *ctx = rockchip_plane->ctx;
	const struct vop_win *win = rockchip_plane->win;

	spin_lock(&ctx->reg_lock);

	VOP_WIN_SET(ctx, win, enable, 0);

	vop_cfg_done(ctx);

	spin_unlock(&ctx->reg_lock);

	return 0;
}

static void rockchip_plane_destroy(struct drm_plane *plane)
{
	struct rockchip_plane *rockchip_plane = to_rockchip_plane(plane);
	struct vop_context *ctx = rockchip_plane->ctx;

	rockchip_disable_plane(plane);
	drm_plane_cleanup(plane);
	ctx->win_msk &= ~(1 << rockchip_plane->id);
	kfree(rockchip_plane);
}

static const struct drm_plane_funcs rockchip_plane_funcs = {
	.update_plane = rockchip_update_plane,
	.disable_plane = rockchip_disable_plane,
	.destroy = rockchip_plane_destroy,
};

struct drm_plane *rockchip_plane_init(struct vop_context *ctx,
				      unsigned long possible_crtcs, bool priv)
{
	struct rockchip_plane *rockchip_plane;
	struct vop_driver_data *vop_data = ctx->data;
	const struct vop_win *win;
	int i;
	int err;

	rockchip_plane = kzalloc(sizeof(*rockchip_plane), GFP_KERNEL);
	if (!rockchip_plane)
		return NULL;

	for (i = 0; i < VOP_MAX_WIN_SUPPORT; i++) {
		if (!(ctx->win_msk & (1 << i))) {
			win = vop_data->win[i];
			break;
		}
	}

	if (VOP_MAX_WIN_SUPPORT == i) {
		DRM_ERROR("failed to find win\n");
		kfree(rockchip_plane);
		return NULL;
	}

	ctx->win_msk |= (1 << i);
	rockchip_plane->id = i;
	rockchip_plane->win = win;
	rockchip_plane->ctx = ctx;

	err = drm_plane_init(ctx->drm_dev, &rockchip_plane->base,
			     possible_crtcs, &rockchip_plane_funcs,
			     win->phy->data_formats,
			     win->phy->nformats, priv);
	if (err) {
		DRM_ERROR("failed to initialize plane\n");
		kfree(rockchip_plane);
		return NULL;
	}

	return &rockchip_plane->base;
}

int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct vop_context *ctx = to_vop_ctx(private->rk_crtc[pipe].crtc);
	unsigned long flags;

	if (ctx->dpms != DRM_MODE_DPMS_ON)
		return -EPERM;

	spin_lock_irqsave(&ctx->irq_lock, flags);

	vop_msk_write(ctx, INTR_CTRL0, LINE_FLAG_INTR_MSK,
		      LINE_FLAG_INTR_EN(1));

	spin_unlock_irqrestore(&ctx->irq_lock, flags);

	return 0;
}

void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct vop_context *ctx = to_vop_ctx(private->rk_crtc[pipe].crtc);
	unsigned long flags;

	if (ctx->dpms != DRM_MODE_DPMS_ON)
		return;

	spin_lock_irqsave(&ctx->irq_lock, flags);

	vop_msk_write(ctx, INTR_CTRL0, LINE_FLAG_INTR_MSK,
		      LINE_FLAG_INTR_EN(0));

	spin_unlock_irqrestore(&ctx->irq_lock, flags);
}

static void rockchip_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct vop_context *ctx = to_vop_ctx(crtc);

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
		vop_power_on(ctx);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		vop_power_off(ctx);
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
	struct vop_context *ctx = to_vop_ctx(crtc);
	u16 hsync_len = adjusted_mode->hsync_end - adjusted_mode->hsync_start;
	u16 left_margin = adjusted_mode->htotal - adjusted_mode->hsync_end;
	u16 vsync_len = adjusted_mode->vsync_end - adjusted_mode->vsync_start;
	u16 upper_margin = adjusted_mode->vtotal - adjusted_mode->vsync_end;
	u16 hdisplay = adjusted_mode->hdisplay;
	u16 vdisplay = adjusted_mode->vdisplay;
	u16 htotal = adjusted_mode->htotal;
	u16 vtotal = adjusted_mode->vtotal;
	struct rockchip_display_mode *priv_mode =
					(void *)adjusted_mode->private;
	unsigned long flags;
	uint32_t val;

	/* nothing to do if we haven't set the mode yet */
	if (adjusted_mode->htotal == 0 || adjusted_mode->vtotal == 0)
		return -EINVAL;

	if (!priv_mode) {
		DRM_ERROR("fail to found display output type[%d]\n",
			  priv_mode->out_type);
		return -EINVAL;
	}

	switch (priv_mode->out_type) {
	case ROCKCHIP_DISPLAY_TYPE_RGB:
	case ROCKCHIP_DISPLAY_TYPE_LVDS:
		VOP_CTRL_SET(ctx, rgb_en, 1);
		VOP_CTRL_SET(ctx, out_mode, ROCKCHIP_OUTFACE_P888);
		break;
	case ROCKCHIP_DISPLAY_TYPE_EDP:
		VOP_CTRL_SET(ctx, edp_en, 1);
		VOP_CTRL_SET(ctx, out_mode, ROCKCHIP_OUTFACE_AAAA);
		break;
	case ROCKCHIP_DISPLAY_TYPE_HDMI:
		VOP_CTRL_SET(ctx, out_mode, ROCKCHIP_OUTFACE_AAAA);
		VOP_CTRL_SET(ctx, hdmi_en, 1);
		break;
	default:
		DRM_ERROR("unsupport out type[%d]\n", priv_mode->out_type);
		return -EINVAL;
	};

	VOP_CTRL_SET(ctx, pin_sync, 0x8);

	VOP_CTRL_SET(ctx, htotal_pw, (htotal << 16) | hsync_len);
	val = (hsync_len + left_margin) << 16;
	val |= hsync_len + left_margin + hdisplay;
	VOP_CTRL_SET(ctx, hact_st_end, val);
	VOP_CTRL_SET(ctx, hpost_st_end, val);

	VOP_CTRL_SET(ctx, vtotal_pw, (vtotal << 16) | vsync_len);
	val = (vsync_len + upper_margin) << 16;
	val |= vsync_len + upper_margin + vdisplay;
	VOP_CTRL_SET(ctx, vact_st_end, val);
	VOP_CTRL_SET(ctx, vpost_st_end, val);

	spin_lock_irqsave(&ctx->irq_lock, flags);

	vop_msk_write(ctx, INTR_CTRL0, DSP_LINE_NUM_MSK,
		      DSP_LINE_NUM(vsync_len + upper_margin + vdisplay));

	spin_unlock_irqrestore(&ctx->irq_lock, flags);

	clk_set_rate(ctx->dclk, adjusted_mode->clock * 1000);

	return 0;
}

static int rockchip_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
			     struct drm_framebuffer *old_fb)
{
	struct vop_context *ctx = to_vop_ctx(crtc);
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

static const struct drm_crtc_helper_funcs rockchip_crtc_helper_funcs = {
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
	struct vop_context *ctx = to_vop_ctx(crtc);
	struct drm_framebuffer *old_fb = crtc->fb;
	unsigned int crtc_w;
	unsigned int crtc_h;
	int ret;

	/* when the page flip is requested, crtc's dpms should be on */
	if (ctx->dpms > DRM_MODE_DPMS_ON) {
		DRM_DEBUG("failed page flip request at dpms[%d].\n", ctx->dpms);
		return 0;
	}

	ret = drm_vblank_get(dev, ctx->pipe);
	if (ret) {
		DRM_DEBUG("failed to acquire vblank counter\n");
		return ret;
	}

	spin_lock_irq(&dev->event_lock);
	if (ctx->event) {
		spin_unlock_irq(&dev->event_lock);
		DRM_ERROR("already pending flip!\n");
		return -EBUSY;
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
	}

	return ret;
}

void rockchip_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *dev_priv = dev->dev_private;
	struct drm_crtc *drm_crtc = dev_priv->rk_crtc[pipe].crtc;
	struct vop_context *ctx;
	unsigned long flags;

	if (!drm_crtc)
		return;

	ctx = to_vop_ctx(drm_crtc);

	spin_lock_irqsave(&dev->event_lock, flags);

	if (ctx->event) {
		drm_send_vblank_event(dev, -1, ctx->event);
		drm_vblank_put(dev, pipe);
		atomic_set(&ctx->wait_vsync_event, 0);
		wake_up(&ctx->wait_vsync_queue);
		ctx->event = NULL;
	}

	spin_unlock_irqrestore(&dev->event_lock, flags);
}

void rockchip_drm_crtc_cancel_pending_flip(struct drm_device *dev)
{
	int i;

	for (i = 0; i < dev->num_crtcs; i++)
		rockchip_drm_crtc_finish_pageflip(dev, i);
}

static void rockchip_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct vop_context *ctx = to_vop_ctx(crtc);
	struct rockchip_drm_private *private = crtc->dev->dev_private;

	private->rk_crtc[ctx->pipe].crtc = NULL;

	drm_crtc_cleanup(crtc);
}

static const struct drm_crtc_funcs rockchip_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = rockchip_drm_crtc_page_flip,
	.destroy = rockchip_drm_crtc_destroy,
};

static int vop_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm_dev = data;
	struct device *mmu_dev;
	struct rockchip_drm_private *private = drm_dev->dev_private;
	struct vop_context *ctx = dev_get_drvdata(dev);
	struct vop_driver_data *vop_data = ctx->data;
	unsigned long possible_crtcs;
	struct drm_crtc *crtc;
	int nr;

	ctx->drm_dev = drm_dev;

	ctx->win_msk = 0;
	ctx->pipe = private->num_pipe++;
	crtc = &ctx->crtc;

	private->rk_crtc[ctx->pipe].crtc = crtc;
	private->rk_crtc[ctx->pipe].id = vop_data->id;

	possible_crtcs = (1 << ctx->pipe);

	ctx->plane = rockchip_plane_init(ctx, 1 << ctx->pipe, true);
	if (!ctx->plane) {
		DRM_ERROR("fail to init primary plane\n");
		return -EINVAL;
	}

	for (nr = 1; nr < ROCKCHIP_MAX_PLANE; nr++) {
		struct drm_plane *plane;

		plane = rockchip_plane_init(ctx, 1 << ctx->pipe, false);
		if (!plane)
			return -EINVAL;
	}

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

	mmu_dev = vop_get_sysmmu_dev(ctx);
	if (!mmu_dev) {
		DRM_ERROR("Do not find mmu devices\n");
		return -EPROBE_DEFER;
	}
	platform_set_sysmmu(mmu_dev, ctx->dev);
	rockchip_iovmm_set_fault_handler(ctx->dev,
			vop_sysmmu_fault_handler);
	rockchip_iovmm_activate(ctx->dev);

	return 0;
}

static void vop_unbind(struct device *dev, struct device *master,
			void *data)
{
	struct drm_device *drm_dev = data;
	struct rockchip_drm_private *private = drm_dev->dev_private;
	struct vop_context *ctx = dev_get_drvdata(dev);
	struct drm_crtc *crtc = &ctx->crtc;

	rockchip_plane_destroy(ctx->plane);

	drm_crtc_cleanup(crtc);
	private->rk_crtc[ctx->pipe].crtc = NULL;
}

static const struct component_ops vop_component_ops = {
	.bind = vop_bind,
	.unbind = vop_unbind,
};

static int rockchip_vop_initial(struct vop_context *ctx)
{
	int i;
	struct vop_driver_data *vop_data = ctx->data;
	const struct vop_reg_data *init_table = vop_data->init_table;

	ctx->hclk = devm_clk_get(ctx->dev, "hclk_vop");
	ctx->aclk = devm_clk_get(ctx->dev, "aclk_vop");
	ctx->dclk = devm_clk_get(ctx->dev, "dclk_vop");

	if ((IS_ERR(ctx->aclk)) || (IS_ERR(ctx->dclk)) ||
	    (IS_ERR(ctx->hclk))) {
		dev_err(ctx->dev, "failed to get vop%d clk source\n",
			ctx->pipe);
		return -ENODEV;
	}

	if (vop_clk_enable(ctx) < 0) {
		dev_err(ctx->dev, "failed to enable ctx%d clks\n",
			ctx->pipe);
		return -EIO;
	}

	memcpy(ctx->regsbak, ctx->regs, ctx->len);

	for (i = 0; i < vop_data->table_size; i++)
		vop_writel(ctx, init_table[i].offset, init_table[i].value);

	for (i = 0; i < VOP_MAX_WIN_SUPPORT; i++)
		VOP_WIN_SET(ctx, vop_data->win[i], enable, 0);

	vop_cfg_done(ctx);

	ctx->dpms = DRM_MODE_DPMS_OFF;

	return 0;
}

static irqreturn_t rockchip_vop_isr(int irq, void *data)
{
	struct vop_context *ctx = data;
	uint32_t intr0_reg;
	unsigned long flags;

	if (ctx && (ctx->pipe < 0 || !ctx->drm_dev) &&
	    ctx->dpms != DRM_MODE_DPMS_ON)
		return IRQ_HANDLED;

	spin_lock_irqsave(&ctx->irq_lock, flags);

	intr0_reg = vop_readl(ctx, INTR_CTRL0);
	if (intr0_reg & LINE_FLAG_INTR)
		vop_writel(ctx, INTR_CTRL0, intr0_reg | LINE_FLAG_INTR_CLR);

	spin_unlock_irqrestore(&ctx->irq_lock, flags);

	drm_handle_vblank(ctx->drm_dev, ctx->pipe);
	rockchip_drm_crtc_finish_pageflip(ctx->drm_dev, ctx->pipe);

	return IRQ_HANDLED;
}

static int vop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vop_context *ctx;
	struct device_node *np;
	struct resource *res;
	struct vop_driver_data *vop_data = vop_get_driver_data(dev);
	int ret = -EINVAL;

	if (!dev->of_node)
		return -ENODEV;

	np = pdev->dev.of_node;

	if (!vop_data)
		return -ENODEV;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->data = vop_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->len = resource_size(res);
	ctx->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->regs)) {
		dev_err(&pdev->dev, "ioremap vop devices fail\n");
		return -ENODEV;
	}

	ctx->regsbak = devm_kzalloc(dev, ctx->len, GFP_KERNEL);
	if (IS_ERR(ctx->regsbak)) {
		dev_err(&pdev->dev, "lcdc devices kzalloc reg backup fail\n");
		return -ENOMEM;
	}

	ctx->irq = platform_get_irq(pdev, 0);
	if (ctx->irq < 0) {
		dev_err(&pdev->dev, "cannot find IRQ for vop%d\n",
			ctx->pipe);
		return -ENODEV;
	}

	ret = devm_request_irq(dev, ctx->irq, rockchip_vop_isr,
			       IRQF_DISABLED | IRQF_SHARED,
			       dev_name(dev), ctx);
	if (ret) {
		dev_err(&pdev->dev, "cannot requeset irq %d - err %d\n",
			ctx->irq, ret);
		return ret;
	}

	spin_lock_init(&ctx->reg_lock);
	spin_lock_init(&ctx->irq_lock);

	ret = rockchip_vop_initial(ctx);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot initial vop dev - err %d\n", ret);
		return ret;
	}

	ctx->dev = dev;

	init_waitqueue_head(&ctx->wait_vsync_queue);
	atomic_set(&ctx->wait_vsync_event, 0);

	platform_set_drvdata(pdev, ctx);

	pm_runtime_enable(&pdev->dev);

	ret = rockchip_drm_component_add(ctx->dev, &vop_component_ops);
	if (ret < 0)
		goto err_disable_pm_runtime;

	dev_info(dev, "vop[%s] probe ok\n", dev_name(ctx->dev));

	return 0;

err_disable_pm_runtime:
	pm_runtime_disable(dev);
	return ret;
}

static int vop_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	rockchip_drm_component_del(&pdev->dev);

	return 0;
}

struct platform_driver rockchip_vop_platform_driver = {
	.probe = vop_probe,
	.remove = vop_remove,
	.driver = {
		.name = "rockchip-vop",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vop_driver_dt_match),
	},
};
