#!/bin/sh
./upgrade_tool db loader.bin
./upgrade_tool wl 20480 kernel.img
ls >/dev/null
