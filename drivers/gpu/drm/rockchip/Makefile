#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm -Idrivers/gpu/drm/rockchip

obj-$(CONFIG_DRM_ROCKCHIP_LCDC) += rockchip_drm_lcdc.o lcdc/
obj-$(CONFIG_DRM_ROCKCHIP_DP) += rockchip_drm_dp.o dp/

rockchipdrm-y := rockchip_drm_drv.o rockchip_drm_encoder.o \
		rockchip_drm_crtc.o rockchip_drm_fbdev.o rockchip_drm_fb.o \
		rockchip_drm_buf.o rockchip_drm_gem.o rockchip_drm_core.o \
		rockchip_drm_plane.o rockchip_drm_dmabuf.o

obj-$(CONFIG_DRM_ROCKCHIP) += rockchipdrm.o
