/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_drv.c
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
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>

#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_gem.h"

#define DRIVER_NAME	"rockchip"
#define DRIVER_DESC	"RockChip Soc DRM"
#define DRIVER_DATE	"20140818"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static DEFINE_MUTEX(drm_component_lock);
static LIST_HEAD(drm_component_list);

struct component_dev {
	struct list_head list;
	struct device *dev;
	const struct component_ops *ops;
};

static struct platform_device *rockchip_drm_pdev;

static int rockchip_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct rockchip_drm_private *private;
	int ret;

	private = devm_kzalloc(dev->dev, sizeof(*private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	dev_set_drvdata(dev->dev, dev);
	dev->dev_private = private;

	drm_mode_config_init(dev);

	rockchip_drm_mode_config_init(dev);

	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev->dev, dev);
	if (ret)
		goto err_cleanup_vblank;

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	ret = drm_vblank_init(dev, ROCKCHIP_MAX_CRTC);
	if (ret)
		goto err_mode_config_cleanup;

	platform_set_drvdata(dev->platformdev, dev);
	rockchip_drm_fbdev_init(dev);

	/* force connectors detection */
	drm_helper_hpd_irq_event(dev);

	return 0;

err_cleanup_vblank:
	drm_vblank_cleanup(dev);
err_mode_config_cleanup:
	drm_mode_config_cleanup(dev);
	kfree(private);
	return ret;
}

static int rockchip_drm_unload(struct drm_device *dev)
{
	rockchip_drm_fbdev_fini(dev);
	drm_vblank_cleanup(dev);
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);

	kfree(dev->dev_private);

	component_unbind_all(dev->dev, dev);
	dev->dev_private = NULL;

	return 0;
}

static int rockchip_drm_suspend(struct drm_device *dev, pm_message_t state)
{
	struct drm_connector *connector;

	drm_modeset_lock_all(dev);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		int old_dpms = connector->dpms;

		if (connector->funcs->dpms)
			connector->funcs->dpms(connector, DRM_MODE_DPMS_OFF);

		/* Set the old mode back to the connector for resume */
		connector->dpms = old_dpms;
	}
	drm_modeset_unlock_all(dev);

	return 0;
}

static int rockchip_drm_resume(struct drm_device *dev)
{
	struct drm_connector *connector;

	drm_modeset_lock_all(dev);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->funcs->dpms)
			connector->funcs->dpms(connector, connector->dpms);
	}
	drm_modeset_unlock_all(dev);

	drm_helper_resume_force_mode(dev);

	return 0;
}

void rockchip_drm_lastclose(struct drm_device *dev)
{
	struct rockchip_drm_private *priv = dev->dev_private;

	drm_modeset_lock_all(dev);
	if (priv->fb_helper)
		drm_fb_helper_restore_fbdev_mode(priv->fb_helper);
	drm_modeset_unlock_all(dev);
}

static const struct drm_ioctl_desc rockchip_ioctls[] = {
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_CREATE, rockchip_gem_create_ioctl,
			  DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_GET, rockchip_gem_get_ioctl,
			  DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_MAP_OFFSET,
			  rockchip_gem_map_offset_ioctl, DRM_UNLOCKED |
			  DRM_AUTH),
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_MMAP, rockchip_gem_mmap_ioctl,
			  DRM_UNLOCKED | DRM_AUTH),
};

static const struct file_operations rockchip_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.mmap = drm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.release = drm_release,
};

const struct vm_operations_struct rockchip_drm_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static struct drm_driver rockchip_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME,
	.load			= rockchip_drm_load,
	.unload			= rockchip_drm_unload,
	.lastclose		= rockchip_drm_lastclose,
	.suspend		= rockchip_drm_suspend,
	.resume			= rockchip_drm_resume,
	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= rockchip_drm_crtc_enable_vblank,
	.disable_vblank		= rockchip_drm_crtc_disable_vblank,
	.gem_vm_ops		= &rockchip_drm_vm_ops,
	.gem_free_object	= rockchip_gem_free_object,
	.dumb_create		= rockchip_gem_dumb_create,
	.dumb_map_offset	= rockchip_gem_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= rockchip_gem_prime_get_sg_table,
	.gem_prime_import_sg_table	= rockchip_gem_prime_import_sg_table,
	.gem_prime_vmap		= rockchip_gem_prime_vmap,
	.gem_prime_vunmap	= rockchip_gem_prime_vunmap,
	.gem_prime_mmap		= rockchip_gem_prime_mmap,
	.ioctls			= rockchip_ioctls,
	.num_ioctls		= ARRAY_SIZE(rockchip_ioctls),
	.fops			= &rockchip_drm_driver_fops,
	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.date	= DRIVER_DATE,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,
};

#ifdef CONFIG_PM_SLEEP
static int rockchip_drm_sys_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);
	pm_message_t message;

	if (pm_runtime_suspended(dev))
		return 0;

	message.event = PM_EVENT_SUSPEND;

	return rockchip_drm_suspend(drm_dev, message);
}

static int rockchip_drm_sys_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	if (pm_runtime_suspended(dev))
		return 0;

	return rockchip_drm_resume(drm_dev);
}
#endif

static const struct dev_pm_ops rockchip_drm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rockchip_drm_sys_suspend,
				rockchip_drm_sys_resume)
};

int rockchip_drm_component_add(struct device *dev,
			       const struct component_ops *ops)
{
	struct component_dev *cdev;
	int ret;

	ret = component_add(dev, ops);
	if (ret)
		return ret;

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev) {
		ret = -ENOMEM;
		goto del_component;
	}

	cdev->dev = dev;
	cdev->ops = ops;

	mutex_lock(&drm_component_lock);
	list_add_tail(&cdev->list, &drm_component_list);
	mutex_unlock(&drm_component_lock);

	return 0;
del_component:
	component_del(dev, ops);
	return ret;
}

void rockchip_drm_component_del(struct device *dev)
{
	struct component_dev *cdev, *next;

	mutex_lock(&drm_component_lock);

	list_for_each_entry_safe(cdev, next, &drm_component_list, list) {
		if (cdev->dev == dev) {
			list_del(&cdev->list);

			component_del(cdev->dev, cdev->ops);
			kfree(cdev);
			break;
		}
	}

	mutex_unlock(&drm_component_lock);
}

static int compare_of(struct device *dev, void *data)
{
	return dev == data;
}

static int rockchip_drm_add_components(struct device *dev, struct master *m)
{
	struct component_dev *cdev;
	unsigned int attach_cnt = 0;

	mutex_lock(&drm_component_lock);

	list_for_each_entry(cdev, &drm_component_list, list) {
		int ret;

		if (!cdev->dev)
			continue;

		attach_cnt++;

		mutex_unlock(&drm_component_lock);

		ret = component_master_add_child(m, compare_of, cdev->dev);
		if (ret < 0)
			return ret;

		mutex_lock(&drm_component_lock);
	}

	mutex_unlock(&drm_component_lock);

	return attach_cnt ? 0 : -ENODEV;
}

static int rockchip_drm_bind(struct device *dev)
{
	return drm_platform_init(&rockchip_drm_driver, to_platform_device(dev));
}

static void rockchip_drm_unbind(struct device *dev)
{
	drm_put_dev(dev_get_drvdata(dev));
}

static const struct component_master_ops rockchip_drm_ops = {
	.add_components = rockchip_drm_add_components,
	.bind = rockchip_drm_bind,
	.unbind = rockchip_drm_unbind,
};

static int rockchip_drm_platform_probe(struct platform_device *pdev)
{
	int ret;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	rockchip_drm_driver.num_ioctls = ARRAY_SIZE(rockchip_ioctls);

	ret = component_master_add(&pdev->dev, &rockchip_drm_ops);
	if (ret < 0)
		DRM_DEBUG_KMS("re-tried by last sub driver probed later.\n");

	return ret;
}

static int rockchip_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &rockchip_drm_ops);
	return 0;
}

static struct platform_driver rockchip_drm_platform_driver = {
	.probe = rockchip_drm_platform_probe,
	.remove = rockchip_drm_platform_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "rockchip-drm",
		.pm = &rockchip_drm_pm_ops,
	},
};

static int rockchip_drm_init(void)
{
	int ret;

	ret = platform_driver_register(&rockchip_vop_platform_driver);
	if (ret < 0)
		goto out_vop;

#ifdef CONFIG_ROCKCHIP_LVDS
	ret = platform_driver_register(&rockchip_lvds_driver);
	if (ret)
		goto out_lvds;
#endif

#ifdef CONFIG_ROCKCHIP_EDP
	ret = platform_driver_register(&rockchip_edp_driver);
	if (ret)
		goto out_edp;
#endif

#ifdef CONFIG_RK32_HDMI
	 ret = platform_driver_register(&rk32_hdmi_driver);
	 if (ret)
	 	goto out_hdmi;
#endif
	rockchip_drm_pdev = platform_device_register_simple("rockchip-drm", -1,
							    NULL, 0);
	if (IS_ERR(rockchip_drm_pdev)) {
		ret = PTR_ERR(rockchip_drm_pdev);
		goto out_drm_pdev;
	}

	ret = platform_driver_register(&rockchip_drm_platform_driver);
	if (ret)
		goto out_drm_driver;

	return 0;

out_drm_driver:
	platform_device_unregister(rockchip_drm_pdev);
out_drm_pdev:
#ifdef CONFIG_ROCKCHIP_EDP
	platform_driver_unregister(&rockchip_edp_driver);
#ifdef CONFIG_RK32_HDMI
        platform_driver_unregister(&rk32_hdmi_driver);
out_hdmi:
#endif
out_edp:
#endif
#ifdef CONFIG_ROCKCHIP_LVDS
	platform_driver_unregister(&rockchip_lvds_driver);
out_lvds:
#endif
	platform_driver_unregister(&rockchip_vop_platform_driver);
out_vop:

	return ret;
}

static void rockchip_drm_exit(void)
{
	platform_device_unregister(rockchip_drm_pdev);
	platform_driver_unregister(&rockchip_drm_platform_driver);
#ifdef CONFIG_ROCKCHIP_EDP
	platform_driver_unregister(&rockchip_edp_driver);
#endif
#ifdef CONFIG_ROCKCHIP_LVDS
	platform_driver_unregister(&rockchip_lvds_driver);
#endif
	platform_driver_unregister(&rockchip_vop_platform_driver);
}

/*
 * use device_initcall_sync because we should wait panel-simple init
 */
device_initcall_sync(rockchip_drm_init);
module_exit(rockchip_drm_exit);

MODULE_AUTHOR("mark yao <mark.yao@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP DRM Driver");
MODULE_LICENSE("GPL v2");
