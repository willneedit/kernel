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

#ifndef _ROCKCHIP_DRM_CONNECTOR_H_
#define _ROCKCHIP_DRM_CONNECTOR_H_

#include <drm/drm_crtc.h>

#include "rockchip_drm_drv.h"

struct rockchip_connector {
	struct device *dev;
	int type;
	void *priv;
	u32 flags;

	void (*enable)(struct rockchip_connector *conn);
	void (*disable)(struct rockchip_connector *conn);
	int (*setmode)(struct rockchip_connector *conn,
		       struct drm_display_mode *mode);
	int (*checkmode)(struct rockchip_connector *conn,
		       struct drm_display_mode *mode);
	int (*getedid)(struct rockchip_connector *conn,
		       int block, unsigned char *buff);
	int (*is_detect)(struct rockchip_connector *conn);
};

void *rockchip_connector_register(struct rockchip_connector *conn);
void rockchip_connector_unregister(void *data);
void rockchip_connector_hotplug_handler(void *data);
#endif
