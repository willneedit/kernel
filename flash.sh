#!/bin/sh
sudo ./upgrade_tool db loader.bin
sudo ./upgrade_tool wl 20480 kernel.img
sudo ./upgrade_tool rd
ls >/dev/null
