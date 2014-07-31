#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm -Idrivers/gpu/drm/rockchip

rockchipdrm-y := rockchip_drm_drv.o rockchip_drm_gem.o \
		rockchip_drm_fb.o rockchip_drm_fbdev.o \
		rockchip_panel.o

obj-$(CONFIG_DRM_ROCKCHIP_CONNECTOR) += rockchip_drm_connector.o connector/
obj-$(CONFIG_DRM_ROCKCHIP_LCDC) += rockchip_drm_lcdc.o lcdc/
obj-$(CONFIG_DRM_ROCKCHIP) += rockchipdrm.o
