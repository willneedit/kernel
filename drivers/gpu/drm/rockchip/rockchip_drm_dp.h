/* rockchip_drm_dp.h
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

#ifndef _ROCKCHIP_DRM_DP_H_
#define _ROCKCHIP_DRM_DP_H_
struct rockchip_dp {
	int type;
	void *priv;

	void (*enable)(struct rockchip_dp *dp);
	void (*disable)(struct rockchip_dp *dp);
	int (*setmode)(struct rockchip_dp *dp, struct drm_display_mode *mode);
};

int rockchip_dp_register(struct rockchip_dp *dp);

struct rockchip_drm_display *rockchip_dp_probe(struct device *dev);
int rockchip_dp_remove(struct device *dev);
#endif
