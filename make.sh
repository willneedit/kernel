#!/bin/bash
export ARCH=arm

#dtb
DTB=rk3288-veyron.dtb
if [ "$1" = "evb" ];then
	DTB=rk3288-evb-rk808.dtb
fi
 
#toolchain
export CROSS_COMPILE=armv7a-cros-linux-gnueabi-
#export CROSS_COMPILE=arm-eabi-

#prepare .config
./chromeos/scripts/prepareconfig chromiumos-rockchip
#make rockchip_chromium_defconfig

echo "CONFIG_VLAN_8021Q=m\nCONFIG_USB_NET_CDC_MBIM=m\nCONFIG_USB_WDM=m" >> .config && \
make oldnoconfig && \
make zImage -j`grep -E "siblings|processor" /proc/cpuinfo |wc -l` && \
make $DTB && \
cp arch/arm/boot/dts/$DTB rk3288.dtb || exit

#cat arch/arm/boot/zImage rk3288.dtb > arch/arm/boot/kernel
#./chrome/mkkrnlimg arch/arm/boot/kernel kernel.img
./chrome/build_gpt_kernel.sh


