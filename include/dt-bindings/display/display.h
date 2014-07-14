/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __DT_BINDINGS_DISPLAY_H
#define __DT_BINDINGS_DISPLAY_H
/********************************************************************
**          display output interface supported by rockchip lcdc                       *
********************************************************************/
/* */
#define OUT_P888            0	//24bit screen,connect to lcdc D0~D23
#define OUT_P666            1	//18bit screen,connect to lcdc D0~D17
#define OUT_P565            2
#define OUT_S888x           4
#define OUT_CCIR656         6
#define OUT_S888            8
#define OUT_S888DUMY        12
#define OUT_RGB_AAA	    15
#define OUT_P16BPP4         24
#define OUT_D888_P666       0x21	//18bit screen,connect to lcdc D2~D7, D10~D15, D18~D23
#define OUT_D888_P565       0x22

#define SCREEN_NULL        0
#define SCREEN_RGB	   1
#define SCREEN_LVDS	   2
#define SCREEN_DUAL_LVDS   3
#define SCREEN_MCU         4
#define SCREEN_TVOUT       5
#define SCREEN_HDMI        6
#define SCREEN_MIPI	   7
#define SCREEN_DUAL_MIPI   8
#define SCREEN_EDP         9

#define LVDS_8BIT_1     0
#define LVDS_8BIT_2     1
#define LVDS_8BIT_3     2
#define LVDS_6BIT       3

#endif
