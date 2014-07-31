/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on panel-simple.c
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

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>

#include "rockchip_drm_drv.h"

/* TODO: convert to gpiod_*() API once it's been merged */
#define GPIO_ACTIVE_LOW	(1 << 0)

struct rockchip_panel {
	struct drm_panel base;
	bool enabled;

	struct drm_display_mode mode;
	struct rockchip_panel_special priv;

	unsigned long enable_gpio_flags;
	int enable_gpio;
};

static inline struct rockchip_panel *to_rockchip_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rockchip_panel, base);
}

static int rockchip_panel_disable(struct drm_panel *panel)
{
	struct rockchip_panel *p = to_rockchip_panel(panel);

	if (!p->enabled)
		return 0;

	if (gpio_is_valid(p->enable_gpio)) {
		if (p->enable_gpio_flags & GPIO_ACTIVE_LOW)
			gpio_set_value(p->enable_gpio, 1);
		else
			gpio_set_value(p->enable_gpio, 0);
	}

	p->enabled = false;

	return 0;
}

static int rockchip_panel_enable(struct drm_panel *panel)
{
	struct rockchip_panel *p = to_rockchip_panel(panel);

	if (p->enabled)
		return 0;

	if (gpio_is_valid(p->enable_gpio)) {
		if (p->enable_gpio_flags & GPIO_ACTIVE_LOW)
			gpio_set_value(p->enable_gpio, 0);
		else
			gpio_set_value(p->enable_gpio, 1);
	}

	p->enabled = true;

	return 0;
}

static int rockchip_panel_get_modes(struct drm_panel *panel)
{
	struct rockchip_panel *p = to_rockchip_panel(panel);
	struct drm_device *drm = panel->drm;
	struct drm_connector *connector = panel->connector;
	const struct drm_display_mode *m = &p->mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(drm, m);
	if (!mode) {
		dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
			m->hdisplay, m->vdisplay, m->vrefresh);
		return 0;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs rockchip_panel_funcs = {
	.disable = rockchip_panel_disable,
	.enable = rockchip_panel_enable,
	.get_modes = rockchip_panel_get_modes,
};

static int rockchip_name_to_face(const char *s)
{
	if (!s)
		return 0;

	if (strncmp(s, "r8g8b8", 6) == 0)
		return ROCKCHIP_OUTFACE_P888;
	else if (strncmp(s, "r6g6b6", 6) == 0)
		return ROCKCHIP_OUTFACE_P666;
	else if (strncmp(s, "r5g6b5", 6) == 0)
		return ROCKCHIP_OUTFACE_P565;

	DRM_ERROR("unsupport display output face[%s]\n", s);

	return 0;
}

static int rockchip_panel_probe(struct platform_device *pdev)
{
	struct rockchip_panel *panel;
	struct device *dev = &pdev->dev;
	struct rockchip_panel_special *priv;
	struct device_node *dn = dev->of_node;
	struct device_node *np;
	enum of_gpio_flags flags;
	struct videomode vm;
	const char *name;
	int err;

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	priv = &panel->priv;
	panel->enabled = false;

	panel->enable_gpio = of_get_named_gpio_flags(dev->of_node,
						     "enable-gpios", 0,
						     &flags);
	if (gpio_is_valid(panel->enable_gpio)) {
		unsigned int value;

		if (flags & OF_GPIO_ACTIVE_LOW)
			panel->enable_gpio_flags |= GPIO_ACTIVE_LOW;

		err = gpio_request(panel->enable_gpio, "enable");
		if (err < 0) {
			dev_err(dev, "failed to request GPIO#%u: %d\n",
				panel->enable_gpio, err);
			return err;
		}

		value = (panel->enable_gpio_flags & GPIO_ACTIVE_LOW) != 0;

		err = gpio_direction_output(panel->enable_gpio, value);
		if (err < 0) {
			dev_err(dev, "failed to setup GPIO%u: %d\n",
				panel->enable_gpio, err);
			goto free_gpio;
		}
	}

	if (of_property_read_bool(dn, "color-swap-rb"))
		priv->color_swap = ROCKCHIP_COLOR_SWAP_RB;

	if (of_property_read_bool(dn, "color-swap-rg"))
		priv->color_swap |= ROCKCHIP_COLOR_SWAP_RG;

	if (of_property_read_bool(dn, "color-swap-gb"))
		priv->color_swap |= ROCKCHIP_COLOR_SWAP_GB;

	if (of_property_read_string(dn, "rockchip,output-face", &name))
		/* default set it as RGB screen */
		priv->out_face = ROCKCHIP_OUTFACE_P666;
	else
		priv->out_face = rockchip_name_to_face(name);

	priv->pwr18 = of_property_read_bool(dn, "lcd-vcc18");
	priv->dither = of_property_read_bool(dn, "output-dither");

	np = of_get_child_by_name(dn, "display-timings");
	if (!np) {
		DRM_ERROR("can't find display timings\n");
		return 0;
	}

	of_node_put(np);
	memset(&vm, 0, sizeof(vm));

	err = of_get_videomode(dn, &vm, 0);
	if (err < 0)
		return err;

	drm_display_mode_from_videomode(&vm, &panel->mode);
	panel->mode.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	priv->flags = vm.flags;
	panel->mode.private = (void *)priv;

	drm_panel_init(&panel->base);
	panel->base.dev = dev;
	panel->base.funcs = &rockchip_panel_funcs;

	err = drm_panel_add(&panel->base);
	if (err < 0)
		goto free_gpio;

	dev_set_drvdata(dev, panel);

	return 0;

free_gpio:
	if (gpio_is_valid(panel->enable_gpio))
		gpio_free(panel->enable_gpio);

	return err;
}

static const struct of_device_id platform_of_match[] = {
	{
		.compatible = "rockchip,panel",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, platform_of_match);

static int rockchip_panel_remove(struct platform_device *pdev)
{
	struct rockchip_panel *panel = dev_get_drvdata(&pdev->dev);

	drm_panel_detach(&panel->base);
	drm_panel_remove(&panel->base);

	if (gpio_is_valid(panel->enable_gpio))
		gpio_free(panel->enable_gpio);

	return 0;
}

struct platform_driver rockchip_panel_platform_driver = {
	.driver = {
		.name = "rockchip,panel",
		.owner = THIS_MODULE,
		.of_match_table = platform_of_match,
	},
	.probe = rockchip_panel_probe,
	.remove = rockchip_panel_remove,
};
