#!/bin/bash
#./chromeos/scripts/prepareconfig chromiumos-rockchip
#make oldnoconfig
make zImage -j48
make rk3188-odysnoon.dtb
cat arch/arm/boot/zImage arch/arm/boot/dts/rk3188-odysnoon.dtb > Kernel
#./mkkrnlimg arch/arm/boot/kernel kernel.img
#./build_gpt_kernel.sh


