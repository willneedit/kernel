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

#ifndef _ROCKCHIP_DRM_LCDC_H_
#define _ROCKCHIP_DRM_LCDC_H_
#include <linux/platform_device.h>
#include <drm/drm_crtc.h>

#include "rockchip_drm_drv.h"

enum {
	ZPOS_DEFAULT_WIN = 0,
	ZPOS_CURSOR_WIN,
	ZPOS_MAX_NUM,
	ZPOS_UNUSED_WIN
};

enum data_format {
	ARGB888 = 0,
	RGB888,
	RGB565,
	YUV420 = 4,
	YUV422,
	YUV444,
	XRGB888,
	XBGR888,
	ABGR888,
	YUV420_A = 10,
	YUV422_A,
	YUV444_A
};

struct lcdc_win_data {
	int zpos;
	int id;
	enum data_format format;
	u32 xact;
	u32 yact;
	u32 xsize;
	u32 ysize;
	u32 xpos;
	u32 ypos;
	u32 y_vir_stride;
	u32 uv_vir_stride;
	bool alpha_en;
	bool enabled;
	bool resume;

	dma_addr_t yrgb_addr;
	dma_addr_t uv_addr;

	int dsp_stx;
	int dsp_sty;
	/*win sel layer*/
	int z_order;
	u8 fmt_cfg;
	u8 fmt_10;
	u8 swap_rb;
	u32 reserved;
	u32 area_num;
	u32 scale_yrgb_x;
	u32 scale_yrgb_y;
	u32 scale_cbcr_x;
	u32 scale_cbcr_y;
	bool support_3d;

	u8 win_lb_mode;

	u8 bic_coe_el;
	/* h 01:scale up ;10:down */
	u8 yrgb_hor_scl_mode;
	/* v 01:scale up ;10:down */
	u8 yrgb_ver_scl_mode;
	/* h scale down mode */
	u8 yrgb_hsd_mode;
	/* v scale up mode */
	u8 yrgb_vsu_mode;
	/* v scale down mode */
	u8 yrgb_vsd_mode;
	u8 cbr_hor_scl_mode;
	u8 cbr_ver_scl_mode;
	u8 cbr_hsd_mode;
	u8 cbr_vsu_mode;
	u8 cbr_vsd_mode;
	u8 vsd_yrgb_gt4;
	u8 vsd_yrgb_gt2;
	u8 vsd_cbr_gt4;
	u8 vsd_cbr_gt2;

	u32 alpha_mode;
	u32 g_alpha_val;
	u32 color_key_val;
};

struct lcdc_driver_data {
	int num_win;
	struct lcdc_driver * (*init)(struct platform_device *pdev);
	void (*deinit)(struct lcdc_driver *drv);
	void (*dpms)(struct lcdc_driver *drv, int mode);
	void (*mode_set)(struct lcdc_driver *drv,
			 struct drm_display_mode *mode);
	void (*enable_vblank)(struct lcdc_driver *drv);
	void (*disable_vblank)(struct lcdc_driver *drv);
	struct lcdc_win_data * (*get_win)(struct lcdc_driver *drv, int zpos);
	void (*win_commit)(struct lcdc_driver *drv,
			   struct lcdc_win_data *win);
};

struct lcdc_driver {
	int id;

	struct lcdc_driver_data *data;
};


void lcdc_vsync_event_handler(struct device *dev);
#ifdef CONFIG_LCDC_RK3288
extern struct lcdc_driver_data rockchip_rk3288_lcdc;
#endif
#endif /* _ROCKCHIP_DRM_LCDC_H_ */
