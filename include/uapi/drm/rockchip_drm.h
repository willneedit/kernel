/*
 *
 * Copyright (c) Fuzhou Rockchip Electronics Co.Ltd
 * Authors:
 *       mark yao <yzq@rock-chips.com>
 *
 * base on exynos_drm.h
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _UAPI_ROCKCHIP_DRM_H
#define _UAPI_ROCKCHIP_DRM_H

#include <drm/drm.h>

/**
 * User-desired buffer creation information structure.
 *
 * @size: user-desired memory allocation size.
 * @flags: user request for setting memory type or cache attributes.
 * @handle: returned a handle to created gem object.
 *     - this handle will be set by gem module of kernel side.
 */
struct drm_rockchip_gem_create {
	uint64_t size;
	uint32_t flags;
	uint32_t handle;
};

/**
 * A structure for getting buffer offset.
 *
 * @handle: a pointer to gem object created.
 * @pad: just padding to be 64-bit aligned.
 * @offset: relatived offset value of the memory region allocated.
 *     - this value should be set by user.
 */
struct drm_rockchip_gem_map_off {
	uint32_t handle;
	uint32_t pad;
	uint64_t offset;
};

/**
 * A structure for mapping buffer.
 *
 * @handle: a handle to gem object created.
 * @pad: just padding to be 64-bit aligned.
 * @size: memory size to be mapped.
 * @mapped: having user virtual address mmaped.
 *      - this variable would be filled by rockchip gem module
 *      of kernel side with user virtual address which is allocated
 *      by do_mmap().
 */
struct drm_rockchip_gem_mmap {
	uint32_t handle;
	uint32_t pad;
	uint64_t size;
	uint64_t mapped;
};

/**
 * A structure to gem information.
 *
 * @handle: a handle to gem object created.
 * @flags: flag value including memory type and cache attribute and
 *      this value would be set by driver.
 * @size: size to memory region allocated by gem and this size would
 *      be set by driver.
 */
struct drm_rockchip_gem_info {
	uint32_t handle;
	uint32_t flags;
	uint64_t size;
};

#define DRM_ROCKCHIP_GEM_CREATE		0x00
#define DRM_ROCKCHIP_GEM_MAP_OFFSET	0x01
#define DRM_ROCKCHIP_GEM_MMAP		0x02
#define DRM_ROCKCHIP_GEM_GET		0x04

#define DRM_IOCTL_ROCKCHIP_GEM_CREATE	DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_ROCKCHIP_GEM_CREATE, struct drm_rockchip_gem_create)

#define DRM_IOCTL_ROCKCHIP_GEM_MAP_OFFSET	DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_ROCKCHIP_GEM_MAP_OFFSET, struct drm_rockchip_gem_map_off)

#define DRM_IOCTL_ROCKCHIP_GEM_MMAP	DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_ROCKCHIP_GEM_MMAP, struct drm_rockchip_gem_mmap)

#define DRM_IOCTL_ROCKCHIP_GEM_GET	DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_ROCKCHIP_GEM_GET, struct drm_rockchip_gem_info)
#endif /* _UAPI_ROCKCHIP_DRM_H */
