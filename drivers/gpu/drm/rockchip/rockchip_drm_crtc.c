/* rockchip_drm_crtc.c
 *
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:mark yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_crtc.c
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

#include "rockchip_drm_crtc.h"
#include "rockchip_drm_drv.h"
#include "rockchip_drm_encoder.h"
#include "rockchip_drm_plane.h"

#define to_rockchip_crtc(x)	container_of(x, struct rockchip_drm_crtc,\
					     drm_crtc)

enum rockchip_crtc_mode {
	/* normal mode */
	CRTC_MODE_NORMAL,
	/* The private plane of crtc is blank */
	CRTC_MODE_BLANK,
};

/*
 * Rockchip specific crtc structure.
 *
 * @drm_crtc: crtc object.
 * @drm_plane: pointer of private plane object for this crtc
 * @manager: the manager associated with this crtc
 * @pipe: a crtc index created at load() with a new crtc object creation
 *      and the crtc object would be set to private->crtc array
 *      to get a crtc object corresponding to this pipe from private->crtc
 *      array when irq interrupt occurred. the reason of using this pipe is that
 *      drm framework doesn't support multiple irq yet.
 *      we can refer to the crtc to current hardware interrupt occurred through
 *      this pipe value.
 * @dpms: store the crtc dpms value
 * @mode: store the crtc mode value
 */
struct rockchip_drm_crtc {
	struct drm_crtc drm_crtc;
	struct drm_plane *plane;
	struct rockchip_drm_manager *manager;
	wait_queue_head_t pending_flip_queue;
	enum rockchip_crtc_mode mode;
	struct drm_pending_vblank_event *event;
	atomic_t pending_flip;
	unsigned int pipe;
	unsigned int dpms;
};

static void rockchip_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);
	struct rockchip_drm_manager *manager = rockchip_crtc->manager;

	DRM_DEBUG_KMS("crtc[%d] mode[%d]\n", crtc->base.id, mode);

	if (rockchip_crtc->dpms == mode) {
		DRM_DEBUG_KMS("desired dpms mode is same as previous one.\n");
		return;
	}

	if (mode > DRM_MODE_DPMS_ON) {
		/* wait for the completion of page flip. */
		wait_event(rockchip_crtc->pending_flip_queue,
			   atomic_read(&rockchip_crtc->pending_flip) == 0);
		drm_vblank_off(crtc->dev, rockchip_crtc->pipe);
	}

	if (manager->ops->dpms)
		manager->ops->dpms(manager, mode);

	rockchip_crtc->dpms = mode;
}

static void rockchip_drm_crtc_prepare(struct drm_crtc *crtc)
{
	/* drm framework doesn't check NULL. */
}

static void rockchip_drm_crtc_commit(struct drm_crtc *crtc)
{
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);
	struct rockchip_drm_manager *manager = rockchip_crtc->manager;

	rockchip_drm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);

	rockchip_plane_commit(rockchip_crtc->plane);

	if (manager->ops->commit)
		manager->ops->commit(manager);

	rockchip_plane_dpms(rockchip_crtc->plane, DRM_MODE_DPMS_ON);
}

static bool rockchip_drm_crtc_mode_fixup(struct drm_crtc *crtc,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);
	struct rockchip_drm_manager *manager = rockchip_crtc->manager;

	if (manager->ops->mode_fixup)
		return manager->ops->mode_fixup(manager, mode, adjusted_mode);

	return true;
}

static int rockchip_drm_crtc_mode_set(struct drm_crtc *crtc,
				      struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode,
				      int x, int y,
				      struct drm_framebuffer *old_fb)
{
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);
	struct rockchip_drm_manager *manager = rockchip_crtc->manager;
	struct drm_plane *plane = rockchip_crtc->plane;
	unsigned int crtc_w;
	unsigned int crtc_h;
	int ret;

	/*
	 * copy the mode data adjusted by mode_fixup() into crtc->mode
	 * so that hardware can be seet to proper mode.
	 */
	memcpy(&crtc->mode, adjusted_mode, sizeof(*adjusted_mode));

	crtc_w = crtc->fb->width - x;
	crtc_h = crtc->fb->height - y;

	if (manager->ops->mode_set)
		manager->ops->mode_set(manager, &crtc->mode);

	ret = rockchip_plane_mode_set(plane, crtc, crtc->fb,
				      0, 0, crtc_w, crtc_h,
				      x, y, crtc_w, crtc_h);
	if (ret)
		return ret;

	plane->crtc = crtc;
	plane->fb = crtc->fb;
	drm_framebuffer_reference(plane->fb);

	return 0;
}

static int rockchip_drm_crtc_mode_set_commit(struct drm_crtc *crtc,
					     int x, int y,
					     struct drm_framebuffer *old_fb)
{
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);
	struct drm_plane *plane = rockchip_crtc->plane;
	unsigned int crtc_w;
	unsigned int crtc_h;
	int ret;

	/* when framebuffer changing is requested, crtc's dpms should be on */
	if (rockchip_crtc->dpms > DRM_MODE_DPMS_ON) {
		DRM_ERROR("failed framebuffer changing request.\n");
		return -EPERM;
	}

	crtc_w = crtc->fb->width - x;
	crtc_h = crtc->fb->height - y;

	ret = rockchip_plane_mode_set(plane, crtc, crtc->fb,
				      0, 0, crtc_w, crtc_h, x, y,
				      crtc_w, crtc_h);
	if (ret)
		return ret;

	rockchip_drm_crtc_commit(crtc);

	return 0;
}

static int rockchip_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					   struct drm_framebuffer *old_fb)
{
	return rockchip_drm_crtc_mode_set_commit(crtc, x, y, old_fb);
}

static void rockchip_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct drm_plane *plane;
	int ret;

	rockchip_drm_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);

	list_for_each_entry(plane, &crtc->dev->mode_config.plane_list, head) {
		if (plane->crtc != crtc)
			continue;

		ret = plane->funcs->disable_plane(plane);
		if (ret)
			DRM_ERROR("Failed to disable plane %d\n", ret);
	}
}

static struct drm_crtc_helper_funcs rockchip_crtc_helper_funcs = {
	.dpms = rockchip_drm_crtc_dpms,
	.prepare = rockchip_drm_crtc_prepare,
	.commit = rockchip_drm_crtc_commit,
	.mode_fixup = rockchip_drm_crtc_mode_fixup,
	.mode_set = rockchip_drm_crtc_mode_set,
	.mode_set_base = rockchip_drm_crtc_mode_set_base,
	.disable = rockchip_drm_crtc_disable,
};

static int rockchip_drm_crtc_page_flip(struct drm_crtc *crtc,
				       struct drm_framebuffer *fb,
				       struct drm_pending_vblank_event *event,
				       uint32_t page_flip_flags)
{
	struct drm_device *dev = crtc->dev;
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);
	struct drm_framebuffer *old_fb = crtc->fb;
	int ret = -EINVAL;

	/* when the page flip is requested, crtc's dpms should be on */
	if (rockchip_crtc->dpms > DRM_MODE_DPMS_ON) {
		DRM_ERROR("failed page flip request.\n");
		return -EINVAL;
	}

	if (rockchip_crtc->event) {
		DRM_ERROR("already pending flip!\n");
		return -EBUSY;
	}

	mutex_lock(&dev->struct_mutex);

	/*
	 * the pipe from user always is 0 so we can set pipe number
	 * of current owner to event.
	 */
	ret = drm_vblank_get(dev, rockchip_crtc->pipe);
	if (ret) {
		DRM_DEBUG("failed to acquire vblank counter\n");

		goto out;
	}

	spin_lock_irq(&dev->event_lock);
	rockchip_crtc->event = event;
	atomic_set(&rockchip_crtc->pending_flip, 1);
	spin_unlock_irq(&dev->event_lock);

	crtc->fb = fb;
	ret = rockchip_drm_crtc_mode_set_commit(crtc, crtc->x, crtc->y, NULL);
	if (ret) {
		crtc->fb = old_fb;

		spin_lock_irq(&dev->event_lock);
		drm_vblank_put(dev, rockchip_crtc->pipe);
		atomic_set(&rockchip_crtc->pending_flip, 0);
		rockchip_crtc->event = NULL;
		spin_unlock_irq(&dev->event_lock);

		goto out;
	}
out:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static void rockchip_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);
	struct rockchip_drm_private *private = crtc->dev->dev_private;

	private->crtc[rockchip_crtc->pipe] = NULL;

	drm_crtc_cleanup(crtc);
	kfree(rockchip_crtc);
}

static int rockchip_drm_crtc_set_property(struct drm_crtc *crtc,
					  struct drm_property *property,
					  uint64_t val)
{
	struct drm_device *dev = crtc->dev;
	struct rockchip_drm_private *dev_priv = dev->dev_private;
	struct rockchip_drm_crtc *rockchip_crtc = to_rockchip_crtc(crtc);

	if (property == dev_priv->crtc_mode_property) {
		enum rockchip_crtc_mode mode = val;

		if (mode == rockchip_crtc->mode)
			return 0;

		rockchip_crtc->mode = mode;

		switch (mode) {
		case CRTC_MODE_NORMAL:
			rockchip_drm_crtc_commit(crtc);
			break;
		case CRTC_MODE_BLANK:
			rockchip_plane_dpms(rockchip_crtc->plane,
					    DRM_MODE_DPMS_OFF);
			break;
		default:
			break;
		}

		return 0;
	}

	return -EINVAL;
}

static struct drm_crtc_funcs rockchip_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = rockchip_drm_crtc_page_flip,
	.destroy = rockchip_drm_crtc_destroy,
	.set_property = rockchip_drm_crtc_set_property,
};

static const struct drm_prop_enum_list mode_names[] = {
	{ CRTC_MODE_NORMAL, "normal" },
	{ CRTC_MODE_BLANK, "blank" },
};

static void rockchip_drm_crtc_attach_mode_property(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct rockchip_drm_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->crtc_mode_property;
	if (!prop) {
		prop = drm_property_create_enum(dev, 0, "mode", mode_names,
						ARRAY_SIZE(mode_names));
		if (!prop)
			return;

		dev_priv->crtc_mode_property = prop;
	}

	drm_object_attach_property(&crtc->base, prop, 0);
}

int rockchip_drm_crtc_create(struct rockchip_drm_manager *manager)
{
	struct rockchip_drm_crtc *rockchip_crtc;
	struct rockchip_drm_private *private = manager->drm_dev->dev_private;
	struct drm_crtc *crtc;

	rockchip_crtc = kzalloc(sizeof(*rockchip_crtc), GFP_KERNEL);
	if (!rockchip_crtc)
		return -ENOMEM;

	init_waitqueue_head(&rockchip_crtc->pending_flip_queue);
	atomic_set(&rockchip_crtc->pending_flip, 0);

	rockchip_crtc->dpms = DRM_MODE_DPMS_OFF;
	rockchip_crtc->manager = manager;
	rockchip_crtc->pipe = manager->pipe;
	rockchip_crtc->plane = rockchip_plane_init(manager->drm_dev,
						   1 << manager->pipe, true);
	if (!rockchip_crtc->plane) {
		kfree(rockchip_crtc);
		return -ENOMEM;
	}

	manager->crtc = &rockchip_crtc->drm_crtc;
	crtc = &rockchip_crtc->drm_crtc;

	private->crtc[manager->pipe] = crtc;

	drm_crtc_init(manager->drm_dev, crtc, &rockchip_crtc_funcs);
	drm_crtc_helper_add(crtc, &rockchip_crtc_helper_funcs);

	rockchip_drm_crtc_attach_mode_property(crtc);

	return 0;
}

int rockchip_drm_crtc_enable_vblank(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct rockchip_drm_crtc *rockchip_crtc =
		to_rockchip_crtc(private->crtc[pipe]);
	struct rockchip_drm_manager *manager = rockchip_crtc->manager;

	if (rockchip_crtc->dpms != DRM_MODE_DPMS_ON)
		return -EPERM;

	if (manager->ops->enable_vblank)
		manager->ops->enable_vblank(manager);

	return 0;
}

void rockchip_drm_crtc_disable_vblank(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *private = dev->dev_private;
	struct rockchip_drm_crtc *rockchip_crtc =
		to_rockchip_crtc(private->crtc[pipe]);
	struct rockchip_drm_manager *manager = rockchip_crtc->manager;

	if (rockchip_crtc->dpms != DRM_MODE_DPMS_ON)
		return;

	if (manager->ops->disable_vblank)
		manager->ops->disable_vblank(manager);
}

void rockchip_drm_crtc_finish_pageflip(struct drm_device *dev, int pipe)
{
	struct rockchip_drm_private *dev_priv = dev->dev_private;
	struct drm_crtc *drm_crtc = dev_priv->crtc[pipe];
	struct rockchip_drm_crtc *rockchip_crtc;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	if (!drm_crtc)
		return;

	rockchip_crtc = to_rockchip_crtc(drm_crtc);
	event = rockchip_crtc->event;

	spin_lock_irqsave(&dev->event_lock, flags);
	if (event) {
		rockchip_crtc->event = NULL;
		drm_send_vblank_event(dev, -1, event);
		drm_vblank_put(dev, pipe);
		atomic_set(&rockchip_crtc->pending_flip, 0);
		wake_up(&rockchip_crtc->pending_flip_queue);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

void rockchip_drm_crtc_cancel_pending_flip(struct drm_device *dev)
{
	int i;

	DRM_DEBUG_KMS("cancle pending flip\n");

	for (i = 0; i < dev->num_crtcs; i++)
		rockchip_drm_crtc_finish_pageflip(dev, i);
}

void rockchip_drm_crtc_plane_mode_set(struct drm_crtc *crtc,
				      struct rockchip_drm_overlay *overlay)
{
	struct rockchip_drm_manager *manager = to_rockchip_crtc(crtc)->manager;

	if (manager->ops->win_mode_set)
		manager->ops->win_mode_set(manager, overlay);
}

void rockchip_drm_crtc_plane_commit(struct drm_crtc *crtc, int zpos)
{
	struct rockchip_drm_manager *manager = to_rockchip_crtc(crtc)->manager;

	if (manager->ops->win_commit)
		manager->ops->win_commit(manager, zpos);
}

void rockchip_drm_crtc_plane_enable(struct drm_crtc *crtc, int zpos)
{
	struct rockchip_drm_manager *manager = to_rockchip_crtc(crtc)->manager;

	if (manager->ops->win_enable)
		manager->ops->win_enable(manager, zpos);
}

void rockchip_drm_crtc_plane_disable(struct drm_crtc *crtc, int zpos)
{
	struct rockchip_drm_manager *manager = to_rockchip_crtc(crtc)->manager;

	if (manager->ops->win_disable)
		manager->ops->win_disable(manager, zpos);
}

void rockchip_drm_crtc_complete_scanout(struct drm_framebuffer *fb)
{
	struct rockchip_drm_manager *manager;
	struct drm_device *dev = fb->dev;
	struct drm_crtc *crtc;

	/*
	 * make sure that overlay data are updated to real hardware
	 * for all encoders.
	 */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		manager = to_rockchip_crtc(crtc)->manager;

		/*
		 * wait for vblank interrupt
		 * - this makes sure that overlay data are updated to
		 *     real hardware.
		 */
		if (manager->ops->wait_for_vblank)
			manager->ops->wait_for_vblank(manager);
	}
}

int rockchip_drm_crtc_get_pipe_from_type(struct drm_device *drm_dev,
					 unsigned int out_type)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &drm_dev->mode_config.crtc_list, head) {
		struct rockchip_drm_crtc *rockchip_crtc;

		rockchip_crtc = to_rockchip_crtc(crtc);
		if (rockchip_crtc->manager->type == out_type)
			return rockchip_crtc->manager->pipe;
	}

	return -EPERM;
}
