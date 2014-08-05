mkimage -D "-I dts -O dtb -p 1024" -f its_script-sdk kernel.tmp
~/trunk/src/scripts/build_kernel_image.sh --board veyron --enable_serial ttyS2 --noenable_rootfs_verification false --use_dev_keys true --arch arm --keys_dir ~/trunk/src/platform/vboot_reference/tests/devkeys/ --vmlinuz kernel.tmp --to kernel.img
