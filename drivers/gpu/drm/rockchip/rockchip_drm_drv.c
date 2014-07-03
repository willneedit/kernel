/* rockchip_drm_drv.c
 *
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

#include <linux/anon_inodes.h>
#include <linux/component.h>

#include <drm/rockchip_drm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_crtc.h"
#include "rockchip_drm_encoder.h"
#include "rockchip_drm_fbdev.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_plane.h"
#include "rockchip_drm_dmabuf.h"

#define DRIVER_NAME	"rockchip"
#define DRIVER_DESC	"rockchip Soc DRM"
#define DRIVER_DATE	"20140623"
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
	enum rockchip_drm_output_type out_type;
	unsigned int dev_type_flag;
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

	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev->dev, dev);
	if (ret)
		goto err_cleanup_vblank;

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

static const struct file_operations rockchip_drm_gem_fops = {
	.mmap = rockchip_drm_gem_mmap_buffer,
};

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
	struct drm_rockchip_file_private *file_priv;
	struct file *anon_filp;
	int ret;

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;

	file->driver_priv = file_priv;

	anon_filp = anon_inode_getfile("rockchip_gem", &rockchip_drm_gem_fops,
				       NULL, 0);
	if (IS_ERR(anon_filp)) {
		ret = PTR_ERR(anon_filp);
		goto err_file_priv_free;
	}

	anon_filp->f_mode = FMODE_READ | FMODE_WRITE;
	file_priv->anon_filp = anon_filp;

	return ret;

err_file_priv_free:
	kfree(file_priv);
	file->driver_priv = NULL;
	return ret;
}

static void rockchip_drm_preclose(struct drm_device *dev,
				  struct drm_file *file)
{
}

static void rockchip_drm_postclose(struct drm_device *dev,
				   struct drm_file *file)
{
	struct drm_rockchip_file_private *file_priv;
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

	file_priv = file->driver_priv;
	if (file_priv->anon_filp)
		fput(file_priv->anon_filp);

	kfree(file->driver_priv);
	file->driver_priv = NULL;
}

static void rockchip_drm_lastclose(struct drm_device *dev)
{
	rockchip_drm_fbdev_restore_mode(dev);
}

static const struct vm_operations_struct rockchip_drm_gem_vm_ops = {
	.fault = rockchip_drm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

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
	.mmap = rockchip_drm_gem_mmap,
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
	.preclose		= rockchip_drm_preclose,
	.lastclose		= rockchip_drm_lastclose,
	.postclose		= rockchip_drm_postclose,
	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= rockchip_drm_crtc_enable_vblank,
	.disable_vblank		= rockchip_drm_crtc_disable_vblank,
	.gem_free_object	= rockchip_drm_gem_free_object,
	.gem_vm_ops		= &rockchip_drm_gem_vm_ops,
	.dumb_create		= rockchip_drm_gem_dumb_create,
	.dumb_map_offset	= rockchip_drm_gem_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= rockchip_dmabuf_prime_export,
	.gem_prime_import	= rockchip_dmabuf_prime_import,
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
			       enum rockchip_drm_device_type dev_type,
			       enum rockchip_drm_output_type out_type)
{
	struct component_dev *cdev;

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
		if (cdev->out_type == out_type) {
			/*
			 * If crtc and encoder/connector device objects are
			 * added already just return.
			 */
			if (cdev->dev_type_flag == (ROCKCHIP_DEVICE_TYPE_CRTC |
					ROCKCHIP_DEVICE_TYPE_CONNECTOR)) {
				mutex_unlock(&drm_component_lock);
				return 0;
			}

			if (dev_type == ROCKCHIP_DEVICE_TYPE_CRTC) {
				cdev->crtc_dev = dev;
				cdev->dev_type_flag |= dev_type;
			}

			if (dev_type == ROCKCHIP_DEVICE_TYPE_CONNECTOR) {
				cdev->conn_dev = dev;
				cdev->dev_type_flag |= dev_type;
			}

			mutex_unlock(&drm_component_lock);
			return 0;
		}
	}

	mutex_unlock(&drm_component_lock);

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	if (dev_type == ROCKCHIP_DEVICE_TYPE_CRTC)
		cdev->crtc_dev = dev;
	if (dev_type == ROCKCHIP_DEVICE_TYPE_CONNECTOR)
		cdev->conn_dev = dev;

	cdev->out_type = out_type;
	cdev->dev_type_flag = dev_type;

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
			if (cdev->crtc_dev == dev) {
				cdev->crtc_dev = NULL;
				cdev->dev_type_flag &= ~dev_type;
			}
		}

		if (dev_type == ROCKCHIP_DEVICE_TYPE_CONNECTOR) {
			if (cdev->conn_dev == dev) {
				cdev->conn_dev = NULL;
				cdev->dev_type_flag &= ~dev_type;
			}
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
		 * lcdc and dp modules have same device object so add
		 * only crtc device object in this case.
		 *
		 * TODO. if dp module follows driver-model driver then
		 * below codes can be removed.
		 */
		if (cdev->crtc_dev == cdev->conn_dev) {
			ret = component_master_add_child(m, compare_of,
							 cdev->crtc_dev);
			if (ret < 0)
				return ret;

			goto out_lock;
		}

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

out_lock:
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

	rockchip_drm_pdev = platform_device_register_simple("rockchip-drm", -1,
							    NULL, 0);
	if (IS_ERR(rockchip_drm_pdev))
		return PTR_ERR(rockchip_drm_pdev);

	ret = platform_driver_register(&rockchip_drm_platform_driver);
	if (ret)
		goto err_unregister_pd;

	return 0;

err_unregister_pd:
	platform_device_unregister(rockchip_drm_pdev);

	return ret;
}

static void rockchip_drm_exit(void)
{
	platform_device_unregister(rockchip_drm_pdev);
	platform_driver_unregister(&rockchip_drm_platform_driver);
}

module_init(rockchip_drm_init);
module_exit(rockchip_drm_exit);
