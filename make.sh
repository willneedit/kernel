#!/bin/bash

DTB=rk3288-veyron.dtb
if [ "$1" = "evb" ];then
	DTB=rk3288-evb-rk808.dtb
fi
export ARCH=arm
export CROSS_COMPILE=armv7a-cros-linux-gnueabi-
./chromeos/scripts/prepareconfig chromiumos-rockchip && \
echo "CONFIG_VLAN_8021Q=m\nCONFIG_USB_NET_CDC_MBIM=m\nCONFIG_USB_WDM=m" >> .config && \
make oldnoconfig && \
make zImage -j`grep -r processor /proc/cpuinfo |wc -l` && \
make $DTB && \
cp arch/arm/boot/dts/$DTB rk3288.dtb && \
./build_gpt_kernel.sh
#cat arch/arm/boot/zImage arch/arm/boot/dts/rk3288-tmp.dtb > arch/arm/boot/kernel
#./mkkrnlimg arch/arm/boot/kernel kernel.img
