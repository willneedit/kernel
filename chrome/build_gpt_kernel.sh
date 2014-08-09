#!/bin/sh
PATH=$PATH:chrome
mkimage -D "-I dts -O dtb -p 1024" -f chrome/its_script kernel.tmp || exit
./chrome/scripts/build_kernel_image.sh --board=veyron --enable_serial ttyS2 --noenable_rootfs_verification --use_dev_keys true --arch arm --keys_dir chrome/devkeys/ --vmlinuz kernel.tmp --to kernel.img
