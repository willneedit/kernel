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

#include <linux/pm_runtime.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include <linux/anon_inodes.h>
#include <linux/component.h>

#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_gem.h"

#define DRIVER_NAME	"rockchip-drm"
#define DRIVER_DESC	"RockChip Soc DRM"
#define DRIVER_DATE	"20140725"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

#define VBLANK_OFF_DELAY	50000

static struct platform_device *rockchip_drm_pdev;

static DEFINE_MUTEX(drm_component_lock);
static LIST_HEAD(drm_component_list);

struct component_dev {
	struct list_head list;
	struct device *crtc_dev;
	struct device *conn_dev;
	unsigned int out_type;
	int pipe;
	void *crtc_data;
	void *conn_data;
};

static int rockchip_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct rockchip_drm_private *private;
	int ret;
	int nr;

	private = kzalloc(sizeof(*private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	dev_set_drvdata(dev->dev, dev);
	dev->dev_private = (void *)private;

	drm_mode_config_init(dev);

	rockchip_drm_mode_config_init(dev);


	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev->dev, dev);
	if (ret)
		goto err_cleanup_vblank;

	for (nr = 0; nr < MAX_PLANE; nr++) {
		struct drm_plane *plane;
		unsigned long possible_crtcs = (1 << MAX_CRTC) - 1;

		plane = rockchip_plane_init(dev, possible_crtcs, false);
		if (!plane)
			goto err_mode_config_cleanup;
	}

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(dev);

	ret = drm_vblank_init(dev, MAX_CRTC);
	if (ret)
		goto err_mode_config_cleanup;

	/* setup possible_clones. */
	rockchip_drm_encoder_setup(dev);

	drm_vblank_offdelay = VBLANK_OFF_DELAY;

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

static int rockchip_drm_open(struct drm_device *dev, struct drm_file *file)
{
	return 0;
}

static void rockchip_drm_postclose(struct drm_device *dev,
				   struct drm_file *file)
{
	struct drm_pending_event *e, *et;
	unsigned long flags;

	if (!file->driver_priv)
		return;

	/* Release all events not unhandled by page flip handler. */
	rockchip_drm_crtc_cancel_pending_flip(dev);

	spin_lock_irqsave(&dev->event_lock, flags);

	/* Release all events handled by page flip handler but not freed. */
	list_for_each_entry_safe(e, et, &file->event_list, link) {
		list_del(&e->link);
		e->destroy(e);
	}

	spin_unlock_irqrestore(&dev->event_lock, flags);

	kfree(file->driver_priv);
	file->driver_priv = NULL;
}

static const struct drm_ioctl_desc rockchip_ioctls[] = {
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_CREATE, rockchip_drm_gem_create_ioctl,
			  DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_MAP_OFFSET,
			  rockchip_drm_gem_map_offset_ioctl, DRM_UNLOCKED |
			  DRM_AUTH),
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_MMAP, rockchip_drm_gem_mmap_ioctl,
			  DRM_UNLOCKED | DRM_AUTH),
	DRM_IOCTL_DEF_DRV(ROCKCHIP_GEM_GET, rockchip_drm_gem_get_ioctl,
			  DRM_UNLOCKED),
};

static const struct file_operations rockchip_drm_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.mmap = drm_gem_cma_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.release = drm_release,
};

static struct drm_driver rockchip_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME,
	.load			= rockchip_drm_load,
	.unload			= rockchip_drm_unload,
	.suspend		= rockchip_drm_suspend,
	.resume			= rockchip_drm_resume,
	.open			= rockchip_drm_open,
	.postclose		= rockchip_drm_postclose,
	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= rockchip_drm_crtc_enable_vblank,
	.disable_vblank		= rockchip_drm_crtc_disable_vblank,
	.gem_free_object        = drm_gem_cma_free_object,
	.gem_vm_ops             = &drm_gem_cma_vm_ops,
	.dumb_create            = drm_gem_cma_dumb_create,
	.dumb_map_offset        = drm_gem_cma_dumb_map_offset,
	.dumb_destroy           = drm_gem_dumb_destroy,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import       = drm_gem_prime_import,
	.gem_prime_export       = drm_gem_prime_export,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap         = drm_gem_cma_prime_vmap,
	.gem_prime_vunmap       = drm_gem_cma_prime_vunmap,
	.gem_prime_mmap         = drm_gem_cma_prime_mmap,
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

int rockchip_drm_pipe_get(struct device *dev)
{
	struct component_dev *cdev, *next;
	int pipe = -1;

	mutex_lock(&drm_component_lock);

	list_for_each_entry_safe(cdev, next, &drm_component_list, list) {
		if ((cdev->crtc_dev == dev) || (cdev->conn_dev == dev)) {
			pipe = cdev->pipe;
			break;
		}
	}

	mutex_unlock(&drm_component_lock);

	return pipe;
}

int rockchip_drm_out_type_get(struct device *dev)
{
	struct component_dev *cdev, *next;
	int type = -1;

	mutex_lock(&drm_component_lock);

	list_for_each_entry_safe(cdev, next, &drm_component_list, list) {
		if ((cdev->crtc_dev == dev) || (cdev->conn_dev == dev)) {
			type = cdev->out_type;
			break;
		}
	}

	mutex_unlock(&drm_component_lock);

	return type;
}

void *rockchip_drm_component_data_get(struct device *dev,
				  enum rockchip_drm_device_type dev_type)
{
	struct component_dev *cdev, *next;
	void *data = NULL;

	mutex_lock(&drm_component_lock);

	list_for_each_entry_safe(cdev, next, &drm_component_list, list) {
		if ((cdev->crtc_dev == dev) || (cdev->conn_dev == dev)) {
			if (dev_type == ROCKCHIP_DEVICE_TYPE_CRTC)
				data = cdev->crtc_data;
			else if (dev_type == ROCKCHIP_DEVICE_TYPE_CONNECTOR)
				data = cdev->conn_data;
			break;
		}
	}

	mutex_unlock(&drm_component_lock);

	return data;
}

int rockchip_drm_component_add(struct device *dev,
			       enum rockchip_drm_device_type dev_type,
			       int out_type, void *data)
{
	struct component_dev *cdev;
	int pipe = -1;

	if (dev_type != ROCKCHIP_DEVICE_TYPE_CRTC &&
	    dev_type != ROCKCHIP_DEVICE_TYPE_CONNECTOR) {
		DRM_ERROR("invalid device type.\n");
		return -EINVAL;
	}

	mutex_lock(&drm_component_lock);

	/*
	 * Make sure to check if there is a component which has two device
	 * objects, for connector and for encoder/connector.
	 * It should make sure that crtc and encoder/connector drivers are
	 * ready before rockchip drm core binds them.
	 */
	list_for_each_entry(cdev, &drm_component_list, list) {
		pipe++;
		/*
		 * out_type from crtc and display port, crtc set possible
		 * out_type maskbit at out_type, and display posr set out_type
		 * directly. and if crtc and display port all register, set
		 * out_type not maskbit;
		 */
		if (cdev->out_type & out_type) {
			if (cdev->crtc_dev && cdev->conn_dev) {
				DRM_ERROR("already register, not allow");
				return -EINVAL;
			}

			if (dev_type == ROCKCHIP_DEVICE_TYPE_CRTC) {
				cdev->pipe = pipe;
				cdev->crtc_dev = dev;
				cdev->crtc_data = data;
			} else if (dev_type == ROCKCHIP_DEVICE_TYPE_CONNECTOR) {
				cdev->conn_dev = dev;
				cdev->conn_data = data;
				cdev->out_type = out_type;
			}

			mutex_unlock(&drm_component_lock);
			return 0;
		}
	}

	mutex_unlock(&drm_component_lock);

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	if (dev_type == ROCKCHIP_DEVICE_TYPE_CRTC) {
		cdev->crtc_dev = dev;
		cdev->crtc_data = data;
	} else if (dev_type == ROCKCHIP_DEVICE_TYPE_CONNECTOR) {
		cdev->conn_dev = dev;
		cdev->conn_data = data;
	}

	cdev->out_type = out_type;

	mutex_lock(&drm_component_lock);
	list_add_tail(&cdev->list, &drm_component_list);
	mutex_unlock(&drm_component_lock);

	return 0;
}

void rockchip_drm_component_del(struct device *dev,
				enum rockchip_drm_device_type dev_type)
{
	struct component_dev *cdev, *next;

	mutex_lock(&drm_component_lock);

	list_for_each_entry_safe(cdev, next, &drm_component_list, list) {
		if (dev_type == ROCKCHIP_DEVICE_TYPE_CRTC) {
			if (cdev->crtc_dev == dev)
				cdev->crtc_dev = NULL;
		}

		if (dev_type == ROCKCHIP_DEVICE_TYPE_CONNECTOR) {
			if (cdev->conn_dev == dev)
				cdev->conn_dev = NULL;
		}

		/*
		 * Release cdev object only in case that both of crtc and
		 * encoder/connector device objects are NULL.
		 */
		if (!cdev->crtc_dev && !cdev->conn_dev) {
			list_del(&cdev->list);
			kfree(cdev);
		}

		break;
	}

	mutex_unlock(&drm_component_lock);
}

static int compare_of(struct device *dev, void *data)
{
	return dev == (struct device *)data;
}

static int rockchip_drm_add_components(struct device *dev, struct master *m)
{
	struct component_dev *cdev;
	unsigned int attach_cnt = 0;

	mutex_lock(&drm_component_lock);

	list_for_each_entry(cdev, &drm_component_list, list) {
		int ret;

		/*
		 * Add components to master only in case that crtc and
		 * encoder/connector device objects exist.
		 */
		if (!cdev->crtc_dev || !cdev->conn_dev)
			continue;

		attach_cnt++;

		mutex_unlock(&drm_component_lock);

		/*
		 * Do not chage below call order.
		 * crtc device first should be added to master because
		 * connector/encoder need pipe number of crtc when they
		 * are created.
		 */
		ret = component_master_add_child(m, compare_of, cdev->crtc_dev);
		ret |= component_master_add_child(m, compare_of,
						  cdev->conn_dev);
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

	return 0;
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

	ret = platform_driver_register(&rockchip_panel_platform_driver);
	if (ret < 0)
		return -ENOMEM;

#ifdef CONFIG_DRM_ROCKCHIP_LCDC
	ret = platform_driver_register(&rockchip_lcdc_platform_driver);
	if (ret < 0)
		goto out_lcdc;
#endif

#ifdef CONFIG_RK3288_LVDS
	ret = platform_driver_register(&rk3288_lvds_driver);
	if (ret)
		goto out_lvds;
#endif

#ifdef CONFIG_RK3288_DP
	ret = platform_driver_register(&rk3288_edp_driver);
	if (ret)
		goto out_edp;
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
#ifdef CONFIG_RK3288_DP
	platform_driver_unregister(&rk3288_edp_driver);
out_edp:
#endif
#ifdef CONFIG_RK3288_LVDS
	platform_driver_unregister(&rk3288_lvds_driver);
out_lvds:
#endif
#ifdef CONFIG_DRM_ROCKCHIP_LCDC
	platform_driver_unregister(&rockchip_lcdc_platform_driver);
out_lcdc:
#endif
	platform_driver_unregister(&rockchip_panel_platform_driver);
	return ret;
}

static void rockchip_drm_exit(void)
{
	platform_device_unregister(rockchip_drm_pdev);
	platform_driver_unregister(&rockchip_drm_platform_driver);
#ifdef CONFIG_RK3288_DP
	platform_driver_unregister(&rk3288_edp_driver);
#endif
#ifdef CONFIG_RK3288_LVDS
	platform_driver_unregister(&rk3288_lvds_driver);
#endif
#ifdef CONFIG_DRM_ROCKCHIP_LCDC
	platform_driver_unregister(&rockchip_lcdc_platform_driver);
#endif
	platform_driver_unregister(&rockchip_panel_platform_driver);
}

module_init(rockchip_drm_init);
module_exit(rockchip_drm_exit);

MODULE_AUTHOR("mark yao <mark.yao@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP DRM Driver");
MODULE_LICENSE("GPL v2");
