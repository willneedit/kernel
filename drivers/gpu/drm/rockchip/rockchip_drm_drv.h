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

#ifndef _ROCKCHIP_DRM_DRV_H
#define _ROCKCHIP_DRM_DRV_H

#include <linux/module.h>
#include <linux/component.h>

#define ROCKCHIP_MAX_FB_BUFFER	4
#define ROCKCHIP_MAX_CONNECTOR	2

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
 * Special mode info for rockchip
 *
 * @out_type: lcd controller need to know the sceen type.
 */
struct rockchip_display_mode {
	int out_type;
};

#define ROCKCHIP_EVENT_HOTPLUG	1

enum rockchip_plane_type {
	ROCKCHIP_WIN0,
	ROCKCHIP_WIN1,
	ROCKCHIP_WIN2,
	ROCKCHIP_WIN3,
	ROCKCHIP_CURSOR,
	ROCKCHIP_MAX_PLANE,
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
	ROCKCHIP_DISPLAY_TYPE_RGB,
	/* LVDS Interface. */
	ROCKCHIP_DISPLAY_TYPE_LVDS,
	/* EDP Interface. */
	ROCKCHIP_DISPLAY_TYPE_EDP,
	/* MIPI Interface. */
	ROCKCHIP_DISPLAY_TYPE_MIPI,
	/* HDMI Interface. */
	ROCKCHIP_DISPLAY_TYPE_HDMI,
};

enum rockchip_crtc_type {
	ROCKCHIP_CRTC_VOPL,
	ROCKCHIP_CRTC_VOPB,
	ROCKCHIP_MAX_CRTC,
};

/* Rockchip drm crtc
 * id@ identify the crtc, so the connector know which crtc connected;
 */
struct rockchip_drm_crtc {
	int id;
	struct drm_crtc *crtc;
};

/*
 * Rockchip drm private structure.
 *
 * @num_pipe: the pipe number for this crtc.
 */
struct rockchip_drm_private {
	struct drm_fb_helper *fb_helper;
	/*
	 * created crtc object would be contained at this array and
	 * this array is used to be aware of which crtc did it request vblank.
	 */
	struct rockchip_drm_crtc rk_crtc[ROCKCHIP_MAX_CRTC];

	unsigned int num_pipe;
};

void rockchip_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe);
void rockchip_drm_crtc_cancel_pending_flip(struct drm_device *dev);
int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe);
void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe);

int rockchip_drm_component_add(struct device *dev,
			       const struct component_ops *ops);
void rockchip_drm_component_del(struct device *dev);

extern struct platform_driver rockchip_vop_platform_driver;
#ifdef CONFIG_ROCKCHIP_LVDS
extern struct platform_driver rockchip_lvds_driver;
#endif
#ifdef CONFIG_ROCKCHIP_EDP
extern struct platform_driver rockchip_edp_driver;
#endif
#endif /* _ROCKCHIP_DRM_DRV_H */
