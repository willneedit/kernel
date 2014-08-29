#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm -Idrivers/gpu/drm/rockchip

rockchipdrm-y := rockchip_drm_drv.o rockchip_drm_fb.o rockchip_drm_fbdev.o \
		rockchip_drm_gem.o rockchip_drm_vop.o

rockchipdrm-$(CONFIG_ROCKCHIP_LVDS) += rockchip_lvds.o
rockchipdrm-$(CONFIG_ROCKCHIP_EDP) += rockchip_edp_core.o rockchip_edp_reg.o
rockchipdrm-$(CONFIG_RK32_HDMI) += rk32_hdmi.o rk32_hdmi_hw.o rk32_hdmi_ddc.o 
obj-$(CONFIG_DRM_ROCKCHIP) += rockchipdrm.o
