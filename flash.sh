#!/bin/sh
sudo ./chrome/upgrade_tool db ./chrome/loader.bin
sudo ./chrome/upgrade_tool wl 20480 kernel.img
sudo ./chrome/upgrade_tool rd
ls --color
