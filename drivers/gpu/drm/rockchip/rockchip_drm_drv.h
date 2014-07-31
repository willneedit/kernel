/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_drv.h
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

#ifndef _ROCKCHIP_DRM_DRV_H_
#define _ROCKCHIP_DRM_DRV_H_

#include <linux/module.h>

#define MAX_CRTC	3
#define MAX_PLANE	5
#define MAX_FB_BUFFER	4
#define DEFAULT_ZPOS	-1

struct drm_device;
struct drm_connector;

/*
 * display output interface supported by rockchip lcdc
 */
#define ROCKCHIP_OUTFACE_P888	0
#define ROCKCHIP_OUTFACE_P666	1
#define ROCKCHIP_OUTFACE_P565	2
/* for use special outface */
#define ROCKCHIP_OUTFACE_AAAA	15

#define ROCKCHIP_COLOR_SWAP_RG	0x1
#define ROCKCHIP_COLOR_SWAP_RB	0x2
#define ROCKCHIP_COLOR_SWAP_GB	0x4
/*
 * Special panel info for rockchip
 *
 * @out_type: lcd controller need to know the sceen type.
 * @out_face: the output pin interface.
 * @color_swap: if want to swap color at output, use this.
 * @pwr18: choice the power supply 1.8 or 3.3 mode for lcdc
 * @dither: use dither func at lcd output
 * @flags: the display flags, now just for pin sync level.
 */
struct rockchip_panel_special {
	int out_type;
	int out_face;
	u32 color_swap;
	bool pwr18;
	bool dither;
	u32 flags;
};

/* This enumerates device type. */
enum rockchip_drm_device_type {
	ROCKCHIP_DEVICE_TYPE_NONE,
	ROCKCHIP_DEVICE_TYPE_CRTC,
	ROCKCHIP_DEVICE_TYPE_CONNECTOR,
};

/* this enumerates display type. */
enum rockchip_drm_output_type {
	ROCKCHIP_DISPLAY_TYPE_NONE = 0,
	/* RGB Interface. */
	ROCKCHIP_DISPLAY_TYPE_RGB = (1 << 0),
	/* LVDS Interface. */
	ROCKCHIP_DISPLAY_TYPE_LVDS = (1 << 1),
	/* DUAL LVDS Interface. */
	ROCKCHIP_DISPLAY_TYPE_DUAL_LVDS = (1 << 2),
	/* EDP Interface. */
	ROCKCHIP_DISPLAY_TYPE_EDP = (1 << 3),
	/* MIPI Interface. */
	ROCKCHIP_DISPLAY_TYPE_MIPI = (1 << 4),
	/* HDMI Interface. */
	ROCKCHIP_DISPLAY_TYPE_HDMI = (1 << 5),
};

/*
 * Rockchip drm private structure.
 *
 * @pipe: the pipe number for this crtc/manager.
 */
struct rockchip_drm_private {
	struct drm_fbdev_cma *fbdev_cma;
	/*
	 * created crtc object would be contained at this array and
	 * this array is used to be aware of which crtc did it request vblank.
	 */
	struct drm_crtc *crtc[MAX_CRTC];
	struct drm_property *plane_zpos_property;
	struct drm_property *crtc_mode_property;

	unsigned int pipe;
};


void rockchip_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe);
void rockchip_drm_crtc_cancel_pending_flip(struct drm_device *dev);
int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe);
void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe);

struct drm_plane *rockchip_plane_init(struct drm_device *dev,
				      unsigned long possible_crtcs, bool priv);

void rockchip_drm_encoder_setup(struct drm_device *dev);

void *rockchip_drm_component_data_get(struct device *dev,
				      enum rockchip_drm_device_type dev_type);
int rockchip_drm_pipe_get(struct device *dev);

int rockchip_drm_component_add(struct device *dev,
			       enum rockchip_drm_device_type dev_type,
			       int out_type, void *data);
void rockchip_drm_component_del(struct device *dev,
				enum rockchip_drm_device_type dev_type);

extern struct platform_driver rockchip_panel_platform_driver;
#ifdef CONFIG_DRM_ROCKCHIP_LCDC
extern struct platform_driver rockchip_lcdc_platform_driver;
#endif
#ifdef CONFIG_RK3288_LVDS
extern struct platform_driver rk3288_lvds_driver;
#endif
#ifdef CONFIG_RK3288_DP
extern struct platform_driver rk3288_edp_driver;
#endif
#endif /* _ROCKCHIP_DRM_DRV_H_ */
