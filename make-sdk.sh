#!/bin/bash
./chromeos/scripts/prepareconfig chromiumos-rockchip
export ARCH=arm
export CROSS_COMPILE=armv7a-cros-linux-gnueabi-
make oldnoconfig
make zImage -j8
make rk3288-tmp.dtb
#cat arch/arm/boot/zImage arch/arm/boot/dts/rk3288-tmp.dtb > arch/arm/boot/kernel
#./mkkrnlimg arch/arm/boot/kernel kernel.img
./build_gpt_kernel-sdk.sh
