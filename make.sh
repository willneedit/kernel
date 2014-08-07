#!/bin/bash

DTB=rk3288-veyron.dtb
if [ "$1" = "evb" ];then
	DTB=rk3288-evb-rk808.dtb
fi
export ARCH=arm
export CROSS_COMPILE=armv7a-cros-linux-gnueabi-
./chromeos/scripts/prepareconfig chromiumos-rockchip && \
make oldnoconfig && \
make zImage -j4 && \
make $DTB && \
cp arch/arm/boot/dts/$DTB rk3288.dtb && \
./build_gpt_kernel.sh
#cat arch/arm/boot/zImage arch/arm/boot/dts/rk3288-tmp.dtb > arch/arm/boot/kernel
#./mkkrnlimg arch/arm/boot/kernel kernel.img
