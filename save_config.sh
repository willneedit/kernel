#!/bin/sh
cp .config .config_bak
sed -i "/CONFIG_VLAN_8021Q/d" .config
sed -i "/CONFIG_USB_NET_CDC_MBIM/d" .config
sed -i "/CONFIG_USB_WDM/d" .config
ARCH=arm make oldnoconfig
rm chromeos/config/armel/chromiumos-rockchip.flavour.config
mv .config chromeos/config/armel/rk3288.config
sed -i "s/silentoldconfig/oldnoconfig/" ./chromeos/scripts/kernelconfig
./chromeos/scripts/kernelconfig oldconfig
git checkout ./chromeos/scripts/kernelconfig
mv chromeos/config/armel/rk3288.config chromeos/config/armel/chromiumos-rockchip.flavour.config
mv .config_bak .config
git show 1b3f7a0f16215acef99743bc595ef5b29dac4f65 | patch -p1
git diff
