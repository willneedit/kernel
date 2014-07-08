/* rk3288_lcdc.h
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:
 *      hjc <hjc@rock-chips.com>
 *      mark yao <mark.yao@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#ifndef _RK3288_LCDC_H_
#define _RK3288_LCDC_H_

#define SET_BIT(x, bit) ((x) << (bit))
#define SET_BIT_MASK(x, bit, mask) SET_BIT((x) & (mask), bit)

#define GPIO		0
#define REGULATOR	1

/* register definition */
#define REG_CFG_DONE			(0x0000)
#define VERSION_INFO			(0x0004)
#define m_RTL_VERSION			SET_BIT(0xffff, 0)
#define m_FPGA_VERSION			SET_BIT(0xffff, 16)
#define SYS_CTRL			(0x0008)
#define v_DIRECT_PATH_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_DIRECT_PATCH_SEL(x)		SET_BIT_MASK(x, 1, 3)
#define v_DOUB_CHANNEL_EN(x)		SET_BIT_MASK(x, 3, 1)
#define v_DOUB_CH_OVERLAP_NUM(x)	SET_BIT_MASK(x, 4, 0xf)
#define v_EDPI_HALT_EN(x)		SET_BIT_MASK(x, 8, 1)
#define v_EDPI_WMS_MODE(x)		SET_BIT_MASK(x, 9, 1)
#define v_EDPI_WMS_FS(x)		SET_BIT_MASK(x, 10, 1)
#define v_RGB_OUT_EN(x)			SET_BIT_MASK(x, 12, 1)
#define v_HDMI_OUT_EN(x)		SET_BIT_MASK(x, 13, 1)
#define v_EDP_OUT_EN(x)			SET_BIT_MASK(x, 14, 1)
#define v_MIPI_OUT_EN(x)		SET_BIT_MASK(x, 15, 1)
#define v_DMA_BURST_LENGTH(x)		SET_BIT_MASK(x, 18, 3)
#define v_MMU_EN(x)			SET_BIT_MASK(x, 20, 1)
#define v_DMA_STOP(x)			SET_BIT_MASK(x, 21, 1)
#define v_STANDBY_EN(x)			SET_BIT_MASK(x, 22, 1)
#define v_AUTO_GATING_EN(x)		SET_BIT_MASK(x, 23, 1)

#define m_DIRECT_PATH_EN		SET_BIT(1, 0)
#define m_DIRECT_PATCH_SEL		SET_BIT(3, 1)
#define m_DOUB_CHANNEL_EN		SET_BIT(1, 3)
#define m_DOUB_CH_OVERLAP_NUM		SET_BIT(0xf, 4)
#define m_EDPI_HALT_EN			SET_BIT(1, 8)
#define m_EDPI_WMS_MODE			SET_BIT(1, 9)
#define m_EDPI_WMS_FS			SET_BIT(1, 10)
#define m_RGB_OUT_EN			SET_BIT(1, 12)
#define m_HDMI_OUT_EN			SET_BIT(1, 13)
#define m_EDP_OUT_EN			SET_BIT(1, 14)
#define m_MIPI_OUT_EN			SET_BIT(1, 15)
#define m_DMA_BURST_LENGTH		SET_BIT(3, 18)
#define m_MMU_EN			SET_BIT(1, 20)
#define m_DMA_STOP			SET_BIT(1, 21)
#define m_STANDBY_EN			SET_BIT(1, 22)
#define m_AUTO_GATING_EN		SET_BIT(1, 23)
#define SYS_CTRL1			(0x000c)
#define v_NOC_HURRY_EN(x)		SET_BIT_MASK(x, 0, 0x1)
#define v_NOC_HURRY_VALUE(x)		SET_BIT_MASK(x, 1, 0x3)
#define v_NOC_HURRY_THRESHOLD(x)	SET_BIT_MASK(x, 3, 0x3f)
#define v_NOC_QOS_EN(x)			SET_BIT_MASK(x, 9, 0x1)
#define v_NOC_WIN_QOS(x)		SET_BIT_MASK(x, 10, 0x3)
#define v_AXI_MAX_OUTSTANDING_EN(x)	SET_BIT_MASK(x, 12, 0x1)
#define v_AXI_OUTSTANDING_MAX_NUM(x)	SET_BIT_MASK(x, 13, 0x1f)

#define m_NOC_HURRY_EN			SET_BIT(0x1, 0)
#define m_NOC_HURRY_VALUE		SET_BIT(0x3, 1)
#define m_NOC_HURRY_THRESHOLD		SET_BIT(0x3f, 3)
#define m_NOC_QOS_EN			SET_BIT(0x1, 9)
#define m_NOC_WIN_QOS			SET_BIT(0x3, 10)
#define m_AXI_MAX_OUTSTANDING_EN	SET_BIT(0x1, 12)
#define m_AXI_OUTSTANDING_MAX_NUM	SET_BIT(0x1f, 13)

#define DSP_CTRL0		(0x0010)
#define v_DSP_OUT_MODE(x)	SET_BIT_MASK(x, 0, 0x0f)
#define v_DSP_HSYNC_POL(x)	SET_BIT_MASK(x, 4, 1)
#define v_DSP_VSYNC_POL(x)	SET_BIT_MASK(x, 5, 1)
#define v_DSP_DEN_POL(x)	SET_BIT_MASK(x, 6, 1)
#define v_DSP_DCLK_POL(x)	SET_BIT_MASK(x, 7, 1)
#define v_DSP_DCLK_DDR(x)	SET_BIT_MASK(x, 8, 1)
#define v_DSP_DDR_PHASE(x)	SET_BIT_MASK(x, 9, 1)
#define v_DSP_INTERLACE(x)	SET_BIT_MASK(x, 10, 1)
#define v_DSP_FIELD_POL(x)	SET_BIT_MASK(x, 11, 1)
#define v_DSP_BG_SWAP(x)	SET_BIT_MASK(x, 12, 1)
#define v_DSP_RB_SWAP(x)	SET_BIT_MASK(x, 13, 1)
#define v_DSP_RG_SWAP(x)	SET_BIT_MASK(x, 14, 1)
#define v_DSP_DELTA_SWAP(x)	SET_BIT_MASK(x, 15, 1)
#define v_DSP_DUMMY_SWAP(x)	SET_BIT_MASK(x, 16, 1)
#define v_DSP_OUT_ZERO(x)	SET_BIT_MASK(x, 17, 1)
#define v_DSP_BLANK_EN(x)	SET_BIT_MASK(x, 18, 1)
#define v_DSP_BLACK_EN(x)	SET_BIT_MASK(x, 19, 1)
#define v_DSP_CCIR656_AVG(x)	SET_BIT_MASK(x, 20, 1)
#define v_DSP_YUV_CLIP(x)	SET_BIT_MASK(x, 21, 1)
#define v_DSP_X_MIR_EN(x)	SET_BIT_MASK(x, 22, 1)
#define v_DSP_Y_MIR_EN(x)	SET_BIT_MASK(x, 23, 1)
#define m_DSP_OUT_MODE		SET_BIT(0x0f, 0)
#define m_DSP_HSYNC_POL		SET_BIT(1, 4)
#define m_DSP_VSYNC_POL		SET_BIT(1, 5)
#define m_DSP_DEN_POL		SET_BIT(1, 6)
#define m_DSP_DCLK_POL		SET_BIT(1, 7)
#define m_DSP_DCLK_DDR		SET_BIT(1, 8)
#define m_DSP_DDR_PHASE		SET_BIT(1, 9)
#define m_DSP_INTERLACE		SET_BIT(1, 10)
#define m_DSP_FIELD_POL		SET_BIT(1, 11)
#define m_DSP_BG_SWAP		SET_BIT(1, 12)
#define m_DSP_RB_SWAP		SET_BIT(1, 13)
#define m_DSP_RG_SWAP		SET_BIT(1, 14)
#define m_DSP_DELTA_SWAP	SET_BIT(1, 15)
#define m_DSP_DUMMY_SWAP	SET_BIT(1, 16)
#define m_DSP_OUT_ZERO		SET_BIT(1, 17)
#define m_DSP_BLANK_EN		SET_BIT(1, 18)
#define m_DSP_BLACK_EN		SET_BIT(1, 19)
#define m_DSP_CCIR656_AVG	SET_BIT(1, 20)
#define m_DSP_YUV_CLIP		SET_BIT(1, 21)
#define m_DSP_X_MIR_EN		SET_BIT(1, 22)
#define m_DSP_Y_MIR_EN		SET_BIT(1, 23)

#define DSP_CTRL1		(0x0014)
#define v_DSP_LUT_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_PRE_DITHER_DOWN_EN(x)	SET_BIT_MASK(x, 1, 1)
#define v_DITHER_DOWN_EN(x)	SET_BIT_MASK(x, 2, 1)
#define v_DITHER_DOWN_MODE(x)	SET_BIT_MASK(x, 3, 1)
#define v_DITHER_DOWN_SEL(x)	SET_BIT_MASK(x, 4, 1)
#define v_DITHER_UP_EN(x)	SET_BIT_MASK(x, 6, 1)
#define v_DSP_LAYER0_SEL(x)	SET_BIT_MASK(x, 8, 3)
#define v_DSP_LAYER1_SEL(x)	SET_BIT_MASK(x, 10, 3)
#define v_DSP_LAYER2_SEL(x)	SET_BIT_MASK(x, 12, 3)
#define v_DSP_LAYER3_SEL(x)	SET_BIT_MASK(x, 14, 3)
#define m_DSP_LUT_EN		SET_BIT(1, 0)
#define m_PRE_DITHER_DOWN_EN	SET_BIT(1, 1)
#define m_DITHER_DOWN_EN	SET_BIT(1, 2)
#define m_DITHER_DOWN_MODE	SET_BIT(1, 3)
#define m_DITHER_DOWN_SEL	SET_BIT(1, 4)
#define m_DITHER_UP_EN		SET_BIT(1, 6)
#define m_DSP_LAYER0_SEL	SET_BIT(3, 8)
#define m_DSP_LAYER1_SEL	SET_BIT(3, 10)
#define m_DSP_LAYER2_SEL	SET_BIT(3, 12)
#define m_DSP_LAYER3_SEL	SET_BIT(3, 14)

#define DSP_BG			(0x0018)
#define v_DSP_BG_BLUE(x)	SET_BIT_MASK(x << 2, 0, 0x3ff)
#define v_DSP_BG_GREEN(x)	SET_BIT_MASK(x << 2, 10, 0x3ff)
#define v_DSP_BG_RED(x)		SET_BIT_MASK(x << 2, 20, 0x3ff)
#define m_DSP_BG_BLUE		SET_BIT(0x3ff, 0)
#define m_DSP_BG_GREEN		SET_BIT(0x3ff, 10)
#define m_DSP_BG_RED		SET_BIT(0x3ff, 20)

#define MCU_CTRL		(0x001c)
#define v_MCU_PIX_TOTAL(x)	SET_BIT_MASK(x, 0, 0x3f)
#define v_MCU_CS_PST(x)		SET_BIT_MASK(x, 6, 0xf)
#define v_MCU_CS_PEND(x)	SET_BIT_MASK(x, 10, 0x3f)
#define v_MCU_RW_PST(x)		SET_BIT_MASK(x, 16, 0xf)
#define v_MCU_RW_PEND(x)	SET_BIT_MASK(x, 20, 0x3f)
#define v_MCU_CLK_SEL(x)	SET_BIT_MASK(x, 26, 1)
#define v_MCU_HOLD_MODE(x)	SET_BIT_MASK(x, 27, 1)
#define v_MCU_FRAME_ST(x)	SET_BIT_MASK(x, 28, 1)
#define v_MCU_RS(x)		SET_BIT_MASK(x, 29, 1)
#define v_MCU_BYPASS(x)		SET_BIT_MASK(x, 30, 1)
#define v_MCU_TYPE(x)		SET_BIT_MASK(x, 31, 1)
#define m_MCU_PIX_TOTAL		SET_BIT(0x3f, 0)
#define m_MCU_CS_PST		SET_BIT(0xf, 6)
#define m_MCU_CS_PEND		SET_BIT(0x3f, 10)
#define m_MCU_RW_PST		SET_BIT(0xf, 16)
#define m_MCU_RW_PEND		SET_BIT(0x3f, 20)
#define m_MCU_CLK_SEL		SET_BIT(1, 26)
#define m_MCU_HOLD_MODE		SET_BIT(1, 27)
#define m_MCU_FRAME_ST		SET_BIT(1, 28)
#define m_MCU_RS		SET_BIT(1, 29)
#define m_MCU_BYPASS		SET_BIT(1, 30)
#define m_MCU_TYPE		SET_BIT((u32)1, 31)

#define INTR_CTRL0			(0x0020)
#define v_DSP_HOLD_VALID_INTR_STS(x)	SET_BIT_MASK(x, 0, 1)
#define v_FS_INTR_STS(x)		SET_BIT_MASK(x, 1, 1)
#define v_LINE_FLAG_INTR_STS(x)		SET_BIT_MASK(x, 2, 1)
#define v_BUS_ERROR_INTR_STS(x)		SET_BIT_MASK(x, 3, 1)
#define v_DSP_HOLD_VALID_INTR_EN(x)	SET_BIT_MASK(x, 4, 1)
#define v_FS_INTR_EN(x)			SET_BIT_MASK(x, 5, 1)
#define v_LINE_FLAG_INTR_EN(x)		SET_BIT_MASK(x, 6, 1)
#define v_BUS_ERROR_INTR_EN(x)		SET_BIT_MASK(x, 7, 1)
#define v_DSP_HOLD_VALID_INTR_CLR(x)	SET_BIT_MASK(x, 8, 1)
#define v_FS_INTR_CLR(x)		SET_BIT_MASK(x, 9, 1)
#define v_LINE_FLAG_INTR_CLR(x)		SET_BIT_MASK(x, 10, 1)
#define v_BUS_ERROR_INTR_CLR(x)		SET_BIT_MASK(x, 11, 1)
#define v_DSP_LINE_FLAG_NUM(x)		SET_BIT_MASK(x, 12, 0xfff)

#define m_DSP_HOLD_VALID_INTR_STS	SET_BIT(1, 0)
#define m_FS_INTR_STS			SET_BIT(1, 1)
#define m_LINE_FLAG_INTR_STS		SET_BIT(1, 2)
#define m_BUS_ERROR_INTR_STS		SET_BIT(1, 3)
#define m_DSP_HOLD_VALID_INTR_EN	SET_BIT(1, 4)
#define m_FS_INTR_EN			SET_BIT(1, 5)
#define m_LINE_FLAG_INTR_EN		SET_BIT(1, 6)
#define m_BUS_ERROR_INTR_EN		SET_BIT(1, 7)
#define m_DSP_HOLD_VALID_INTR_CLR	SET_BIT(1, 8)
#define m_FS_INTR_CLR			SET_BIT(1, 9)
#define m_LINE_FLAG_INTR_CLR		SET_BIT(1, 10)
#define m_BUS_ERROR_INTR_CLR		SET_BIT(1, 11)
#define m_DSP_LINE_FLAG_NUM		SET_BIT(0xfff, 12)

#define INTR_CTRL1			(0x0024)
#define v_WIN0_EMPTY_INTR_STS(x)	SET_BIT_MASK(x, 0, 1)
#define v_WIN1_EMPTY_INTR_STS(x)	SET_BIT_MASK(x, 1, 1)
#define v_WIN2_EMPTY_INTR_STS(x)	SET_BIT_MASK(x, 2, 1)
#define v_WIN3_EMPTY_INTR_STS(x)	SET_BIT_MASK(x, 3, 1)
#define v_HWC_EMPTY_INTR_STS(x)		SET_BIT_MASK(x, 4, 1)
#define v_POST_BUF_EMPTY_INTR_STS(x)	SET_BIT_MASK(x, 5, 1)
#define v_PWM_GEN_INTR_STS(x)		SET_BIT_MASK(x, 6, 1)
#define v_WIN0_EMPTY_INTR_EN(x)		SET_BIT_MASK(x, 8, 1)
#define v_WIN1_EMPTY_INTR_EN(x)		SET_BIT_MASK(x, 9, 1)
#define v_WIN2_EMPTY_INTR_EN(x)		SET_BIT_MASK(x, 10, 1)
#define v_WIN3_EMPTY_INTR_EN(x)		SET_BIT_MASK(x, 11, 1)
#define v_HWC_EMPTY_INTR_EN(x)		SET_BIT_MASK(x, 12, 1)
#define v_POST_BUF_EMPTY_INTR_EN(x)	SET_BIT_MASK(x, 13, 1)
#define v_PWM_GEN_INTR_EN(x)		SET_BIT_MASK(x, 14, 1)
#define v_WIN0_EMPTY_INTR_CLR(x)	SET_BIT_MASK(x, 16, 1)
#define v_WIN1_EMPTY_INTR_CLR(x)	SET_BIT_MASK(x, 17, 1)
#define v_WIN2_EMPTY_INTR_CLR(x)	SET_BIT_MASK(x, 18, 1)
#define v_WIN3_EMPTY_INTR_CLR(x)	SET_BIT_MASK(x, 19, 1)
#define v_HWC_EMPTY_INTR_CLR(x)		SET_BIT_MASK(x, 20, 1)
#define v_POST_BUF_EMPTY_INTR_CLR(x)	SET_BIT_MASK(x, 21, 1)
#define v_PWM_GEN_INTR_CLR(x)		SET_BIT_MASK(x, 22, 1)

#define m_WIN0_EMPTY_INTR_STS		SET_BIT(1, 0)
#define m_WIN1_EMPTY_INTR_STS		SET_BIT(1, 1)
#define m_WIN2_EMPTY_INTR_STS		SET_BIT(1, 2)
#define m_WIN3_EMPTY_INTR_STS		SET_BIT(1, 3)
#define m_HWC_EMPTY_INTR_STS		SET_BIT(1, 4)
#define m_POST_BUF_EMPTY_INTR_STS	SET_BIT(1, 5)
#define m_PWM_GEN_INTR_STS		SET_BIT(1, 6)
#define m_WIN0_EMPTY_INTR_EN		SET_BIT(1, 8)
#define m_WIN1_EMPTY_INTR_EN		SET_BIT(1, 9)
#define m_WIN2_EMPTY_INTR_EN		SET_BIT(1, 10)
#define m_WIN3_EMPTY_INTR_EN		SET_BIT(1, 11)
#define m_HWC_EMPTY_INTR_EN		SET_BIT(1, 12)
#define m_POST_BUF_EMPTY_INTR_EN	SET_BIT(1, 13)
#define m_PWM_GEN_INTR_EN		SET_BIT(1, 14)
#define m_WIN0_EMPTY_INTR_CLR		SET_BIT(1, 16)
#define m_WIN1_EMPTY_INTR_CLR		SET_BIT(1, 17)
#define m_WIN2_EMPTY_INTR_CLR		SET_BIT(1, 18)
#define m_WIN3_EMPTY_INTR_CLR		SET_BIT(1, 19)
#define m_HWC_EMPTY_INTR_CLR		SET_BIT(1, 20)
#define m_POST_BUF_EMPTY_INTR_CLR	SET_BIT(1, 21)
#define m_PWM_GEN_INTR_CLR		SET_BIT(1, 22)

/*win0 register*/
#define WIN0_CTRL0			(0x0030)
#define v_WIN0_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_WIN0_DATA_FMT(x)		SET_BIT_MASK(x, 1, 7)
#define v_WIN0_FMT_10(x)		SET_BIT_MASK(x, 4, 1)
#define v_WIN0_LB_MODE(x)		SET_BIT_MASK(x, 5, 7)
#define v_WIN0_INTERLACE_READ(x)	SET_BIT_MASK(x, 8, 1)
#define v_WIN0_NO_OUTSTANDING(x)	SET_BIT_MASK(x, 9, 1)
#define v_WIN0_CSC_MODE(x)		SET_BIT_MASK(x, 10, 3)
#define v_WIN0_RB_SWAP(x)		SET_BIT_MASK(x, 12, 1)
#define v_WIN0_ALPHA_SWAP(x)		SET_BIT_MASK(x, 13, 1)
#define v_WIN0_MID_SWAP(x)		SET_BIT_MASK(x, 14, 1)
#define v_WIN0_UV_SWAP(x)		SET_BIT_MASK(x, 15, 1)
#define v_WIN0_PPAS_ZERO_EN(x)		SET_BIT_MASK(x, 16, 1)
#define v_WIN0_YRGB_DEFLICK(x)		SET_BIT_MASK(x, 18, 1)
#define v_WIN0_CBR_DEFLICK(x)		SET_BIT_MASK(x, 19, 1)
#define v_WIN0_YUV_CLIP(x)		SET_BIT_MASK(x, 20, 1)

#define m_WIN0_EN			SET_BIT(1, 0)
#define m_WIN0_DATA_FMT			SET_BIT(7, 1)
#define m_WIN0_FMT_10			SET_BIT(1, 4)
#define m_WIN0_LB_MODE			SET_BIT(7, 5)
#define m_WIN0_INTERLACE_READ		SET_BIT(1, 8)
#define m_WIN0_NO_OUTSTANDING		SET_BIT(1, 9)
#define m_WIN0_CSC_MODE			SET_BIT(3, 10)
#define m_WIN0_RB_SWAP			SET_BIT(1, 12)
#define m_WIN0_ALPHA_SWAP		SET_BIT(1, 13)
#define m_WIN0_MID_SWAP			SET_BIT(1, 14)
#define m_WIN0_UV_SWAP			SET_BIT(1, 15)
#define m_WIN0_PPAS_ZERO_EN		SET_BIT(1, 16)
#define m_WIN0_YRGB_DEFLICK		SET_BIT(1, 18)
#define m_WIN0_CBR_DEFLICK		SET_BIT(1, 19)
#define m_WIN0_YUV_CLIP			SET_BIT(1, 20)

#define WIN0_CTRL1			(0x0034)
#define v_WIN0_YRGB_AXI_GATHER_EN(x)	SET_BIT_MASK(x, 0, 1)
#define v_WIN0_CBR_AXI_GATHER_EN(x)	SET_BIT_MASK(x, 1, 1)
#define v_WIN0_BIC_COE_SEL(x)		SET_BIT_MASK(x, 2, 3)
#define v_WIN0_VSD_YRGB_GT4(x)		SET_BIT_MASK(x, 4, 1)
#define v_WIN0_VSD_YRGB_GT2(x)		SET_BIT_MASK(x, 5, 1)
#define v_WIN0_VSD_CBR_GT4(x)		SET_BIT_MASK(x, 6, 1)
#define v_WIN0_VSD_CBR_GT2(x)		SET_BIT_MASK(x, 7, 1)
#define v_WIN0_YRGB_AXI_GATHER_NUM(x)	SET_BIT_MASK(x, 8, 0xf)
#define v_WIN0_CBR_AXI_GATHER_NUM(x)	SET_BIT_MASK(x, 12, 7)
#define v_WIN0_LINE_LOAD_MODE(x)	SET_BIT_MASK(x, 15, 1)
#define v_WIN0_YRGB_HOR_SCL_MODE(x)	SET_BIT_MASK(x, 16, 3)
#define v_WIN0_YRGB_VER_SCL_MODE(x)	SET_BIT_MASK(x, 18, 3)
#define v_WIN0_YRGB_HSD_MODE(x)		SET_BIT_MASK(x, 20, 3)
#define v_WIN0_YRGB_VSU_MODE(x)		SET_BIT_MASK(x, 22, 1)
#define v_WIN0_YRGB_VSD_MODE(x)		SET_BIT_MASK(x, 23, 1)
#define v_WIN0_CBR_HOR_SCL_MODE(x)	SET_BIT_MASK(x, 24, 3)
#define v_WIN0_CBR_VER_SCL_MODE(x)	SET_BIT_MASK(x, 26, 3)
#define v_WIN0_CBR_HSD_MODE(x)		SET_BIT_MASK(x, 28, 3)
#define v_WIN0_CBR_VSU_MODE(x)		SET_BIT_MASK(x, 30, 1)
#define v_WIN0_CBR_VSD_MODE(x)		SET_BIT_MASK(x, 31, 1)

#define m_WIN0_YRGB_AXI_GATHER_EN	SET_BIT(1, 0)
#define m_WIN0_CBR_AXI_GATHER_EN	SET_BIT(1, 1)
#define m_WIN0_BIC_COE_SEL		SET_BIT(3, 2)
#define m_WIN0_VSD_YRGB_GT4		SET_BIT(1, 4)
#define m_WIN0_VSD_YRGB_GT2		SET_BIT(1, 5)
#define m_WIN0_VSD_CBR_GT4		SET_BIT(1, 6)
#define m_WIN0_VSD_CBR_GT2		SET_BIT(1, 7)
#define m_WIN0_YRGB_AXI_GATHER_NUM	SET_BIT(0xf, 8)
#define m_WIN0_CBR_AXI_GATHER_NUM	SET_BIT(7, 12)
#define m_WIN0_LINE_LOAD_MODE		SET_BIT(1, 15)
#define m_WIN0_YRGB_HOR_SCL_MODE	SET_BIT(3, 16)
#define m_WIN0_YRGB_VER_SCL_MODE	SET_BIT(3, 18)
#define m_WIN0_YRGB_HSD_MODE		SET_BIT(3, 20)
#define m_WIN0_YRGB_VSU_MODE		SET_BIT(1, 22)
#define m_WIN0_YRGB_VSD_MODE		SET_BIT(1, 23)
#define m_WIN0_CBR_HOR_SCL_MODE		SET_BIT(3, 24)
#define m_WIN0_CBR_VER_SCL_MODE		SET_BIT(3, 26)
#define m_WIN0_CBR_HSD_MODE		SET_BIT(3, 28)
#define m_WIN0_CBR_VSU_MODE		SET_BIT((u32)1, 30)
#define m_WIN0_CBR_VSD_MODE		SET_BIT((u32)1, 31)

#define WIN0_COLOR_KEY			(0x0038)
#define v_WIN0_COLOR_KEY(x)		SET_BIT_MASK(x, 0, 0x3fffffff)
#define v_WIN0_COLOR_KEY_EN(x)		SET_BIT_MASK(x, 31, 1)
#define m_WIN0_COLOR_KEY		SET_BIT(0x3fffffff, 0)
#define m_WIN0_COLOR_KEY_EN		SET_BIT((u32)1, 31)

#define WIN0_VIR			(0x003c)
#define v_WIN0_VIR_STRIDE(x)		SET_BIT_MASK(x, 0, 0x3fff)
#define v_WIN0_VIR_STRIDE_UV(x)		SET_BIT_MASK(x, 16, 0x3fff)
#define m_WIN0_VIR_STRIDE		SET_BIT(0x3fff, 0)
#define m_WIN0_VIR_STRIDE_UV		SET_BIT(0x3fff, 16)

#define WIN0_YRGB_MST			(0x0040)
#define WIN0_CBR_MST			(0x0044)
#define WIN0_ACT_INFO			(0x0048)
#define v_WIN0_ACT_WIDTH(x)		SET_BIT_MASK(x-1, 0, 0x1fff)
#define v_WIN0_ACT_HEIGHT(x)		SET_BIT_MASK(x-1, 16, 0x1fff)
#define m_WIN0_ACT_WIDTH		SET_BIT(0x1fff, 0)
#define m_WIN0_ACT_HEIGHT		SET_BIT(0x1fff, 16)

#define WIN0_DSP_INFO			(0x004c)
#define v_WIN0_DSP_WIDTH(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN0_DSP_HEIGHT(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN0_DSP_WIDTH		SET_BIT(0xfff, 0)
#define m_WIN0_DSP_HEIGHT		SET_BIT(0xfff, 16)

#define WIN0_DSP_ST			(0x0050)
#define v_WIN0_DSP_XST(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN0_DSP_YST(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN0_DSP_XST			SET_BIT(0x1fff, 0)
#define m_WIN0_DSP_YST			SET_BIT(0x1fff, 16)

#define WIN0_SCL_FACTOR_YRGB		(0x0054)
#define v_WIN0_HS_FACTOR_YRGB(x)	SET_BIT_MASK(x, 0, 0xffff)
#define v_WIN0_VS_FACTOR_YRGB(x)	SET_BIT_MASK(x, 16, 0xffff)
#define m_WIN0_HS_FACTOR_YRGB		SET_BIT(0xffff, 0)
#define m_WIN0_VS_FACTOR_YRGB		SET_BIT((u32)0xffff, 16)

#define WIN0_SCL_FACTOR_CBR		(0x0058)
#define v_WIN0_HS_FACTOR_CBR(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_WIN0_VS_FACTOR_CBR(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_WIN0_HS_FACTOR_CBR		SET_BIT(0xffff, 0)
#define m_WIN0_VS_FACTOR_CBR		SET_BIT((u32)0xffff, 16)

#define WIN0_SCL_OFFSET			(0x005c)
#define v_WIN0_HS_OFFSET_YRGB(x)	SET_BIT_MASK(x, 0, 0xff)
#define v_WIN0_HS_OFFSET_CBR(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_WIN0_VS_OFFSET_YRGB(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN0_VS_OFFSET_CBR(x)		SET_BIT_MASK(x, 24, 0xff)

#define m_WIN0_HS_OFFSET_YRGB		SET_BIT(0xff, 0)
#define m_WIN0_HS_OFFSET_CBR		SET_BIT(0xff, 8)
#define m_WIN0_VS_OFFSET_YRGB		SET_BIT(0xff, 16)
#define m_WIN0_VS_OFFSET_CBR		SET_BIT((u32)0xff, 24)

#define WIN0_SRC_ALPHA_CTRL		(0x0060)
#define v_WIN0_SRC_ALPHA_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_WIN0_SRC_COLOR_M0(x)		SET_BIT_MASK(x, 1, 1)
#define v_WIN0_SRC_ALPHA_M0(x)		SET_BIT_MASK(x, 2, 1)
#define v_WIN0_SRC_BLEND_M0(x)		SET_BIT_MASK(x, 3, 3)
#define v_WIN0_SRC_ALPHA_CAL_M0(x)	SET_BIT_MASK(x, 5, 1)
#define v_WIN0_SRC_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define v_WIN0_SRC_GLOBAL_ALPHA(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN0_FADING_VALUE(x)		SET_BIT_MASK(x, 24, 0xff)

#define m_WIN0_SRC_ALPHA_EN		SET_BIT(1, 0)
#define m_WIN0_SRC_COLOR_M0		SET_BIT(1, 1)
#define m_WIN0_SRC_ALPHA_M0		SET_BIT(1, 2)
#define m_WIN0_SRC_BLEND_M0		SET_BIT(3, 3)
#define m_WIN0_SRC_ALPHA_CAL_M0		SET_BIT(1, 5)
#define m_WIN0_SRC_FACTOR_M0		SET_BIT(7, 6)
#define m_WIN0_SRC_GLOBAL_ALPHA		SET_BIT(0xff, 16)
#define m_WIN0_FADING_VALUE		SET_BIT(0xff, 24)

#define WIN0_DST_ALPHA_CTRL		(0x0064)
#define v_WIN0_DST_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define m_WIN0_DST_FACTOR_M0		SET_BIT(7, 6)

#define WIN0_FADING_CTRL		(0x0068)
#define v_WIN0_FADING_OFFSET_R(x)	SET_BIT_MASK(x, 0, 0xff)
#define v_WIN0_FADING_OFFSET_G(x)	SET_BIT_MASK(x, 8, 0xff)
#define v_WIN0_FADING_OFFSET_B(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN0_FADING_EN(x)		SET_BIT_MASK(x, 24, 1)

#define m_WIN0_FADING_OFFSET_R		SET_BIT(0xff, 0)
#define m_WIN0_FADING_OFFSET_G		SET_BIT(0xff, 8)
#define m_WIN0_FADING_OFFSET_B		SET_BIT(0xff, 16)
#define m_WIN0_FADING_EN		SET_BIT(1, 24)

/*win1 register*/
#define WIN1_CTRL0			(0x0070)
#define v_WIN1_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_WIN1_DATA_FMT(x)		SET_BIT_MASK(x, 1, 7)
#define v_WIN1_FMT_10(x)		SET_BIT_MASK(x, 4, 1)
#define v_WIN1_LB_MODE(x)		SET_BIT_MASK(x, 5, 7)
#define v_WIN1_INTERLACE_READ_MODE(x)	SET_BIT_MASK(x, 8, 1)
#define v_WIN1_NO_OUTSTANDING(x)	SET_BIT_MASK(x, 9, 1)
#define v_WIN1_CSC_MODE(x)		SET_BIT_MASK(x, 10, 3)
#define v_WIN1_RB_SWAP(x)		SET_BIT_MASK(x, 12, 1)
#define v_WIN1_ALPHA_SWAP(x)		SET_BIT_MASK(x, 13, 1)
#define v_WIN1_MID_SWAP(x)		SET_BIT_MASK(x, 14, 1)
#define v_WIN1_UV_SWAP(x)		SET_BIT_MASK(x, 15, 1)
#define v_WIN1_PPAS_ZERO_EN(x)		SET_BIT_MASK(x, 16, 1)
#define v_WIN1_YRGB_DEFLICK(x)		SET_BIT_MASK(x, 18, 1)
#define v_WIN1_CBR_DEFLICK(x)		SET_BIT_MASK(x, 19, 1)
#define v_WIN1_YUV_CLIP(x)		SET_BIT_MASK(x, 20, 1)

#define m_WIN1_EN			SET_BIT(1, 0)
#define m_WIN1_DATA_FMT			SET_BIT(7, 1)
#define m_WIN1_FMT_10			SET_BIT(1, 4)
#define m_WIN1_LB_MODE			SET_BIT(7, 5)
#define m_WIN1_INTERLACE_READ_MODE	SET_BIT(1, 8)
#define m_WIN1_NO_OUTSTANDING		SET_BIT(1, 9)
#define m_WIN1_CSC_MODE			SET_BIT(3, 10)
#define m_WIN1_RB_SWAP			SET_BIT(1, 12)
#define m_WIN1_ALPHA_SWAP		SET_BIT(1, 13)
#define m_WIN1_MID_SWAP			SET_BIT(1, 14)
#define m_WIN1_UV_SWAP			SET_BIT(1, 15)
#define m_WIN1_PPAS_ZERO_EN		SET_BIT(1, 16)
#define m_WIN1_YRGB_DEFLICK		SET_BIT(1, 18)
#define m_WIN1_CBR_DEFLICK		SET_BIT(1, 19)
#define m_WIN1_YUV_CLIP			SET_BIT(1, 20)

#define WIN1_CTRL1			(0x0074)
#define v_WIN1_YRGB_AXI_GATHER_EN(x)	SET_BIT_MASK(x, 0, 1)
#define v_WIN1_CBR_AXI_GATHER_EN(x)	SET_BIT_MASK(x, 1, 1)
#define v_WIN1_BIC_COE_SEL(x)		SET_BIT_MASK(x, 2, 3)
#define v_WIN1_VSD_YRGB_GT4(x)		SET_BIT_MASK(x, 4, 1)
#define v_WIN1_VSD_YRGB_GT2(x)		SET_BIT_MASK(x, 5, 1)
#define v_WIN1_VSD_CBR_GT4(x)		SET_BIT_MASK(x, 6, 1)
#define v_WIN1_VSD_CBR_GT2(x)		SET_BIT_MASK(x, 7, 1)
#define v_WIN1_YRGB_AXI_GATHER_NUM(x)	SET_BIT_MASK(x, 8, 0xf)
#define v_WIN1_CBR_AXI_GATHER_NUM(x)	SET_BIT_MASK(x, 12, 7)
#define v_WIN1_LINE_LOAD_MODE(x)	SET_BIT_MASK(x, 15, 1)
#define v_WIN1_YRGB_HOR_SCL_MODE(x)	SET_BIT_MASK(x, 16, 3)
#define v_WIN1_YRGB_VER_SCL_MODE(x)	SET_BIT_MASK(x, 18, 3)
#define v_WIN1_YRGB_HSD_MODE(x)		SET_BIT_MASK(x, 20, 3)
#define v_WIN1_YRGB_VSU_MODE(x)		SET_BIT_MASK(x, 22, 1)
#define v_WIN1_YRGB_VSD_MODE(x)		SET_BIT_MASK(x, 23, 1)
#define v_WIN1_CBR_HOR_SCL_MODE(x)	SET_BIT_MASK(x, 24, 3)
#define v_WIN1_CBR_VER_SCL_MODE(x)	SET_BIT_MASK(x, 26, 3)
#define v_WIN1_CBR_HSD_MODE(x)		SET_BIT_MASK(x, 28, 3)
#define v_WIN1_CBR_VSU_MODE(x)		SET_BIT_MASK(x, 30, 1)
#define v_WIN1_CBR_VSD_MODE(x)		SET_BIT_MASK(x, 31, 1)

#define m_WIN1_YRGB_AXI_GATHER_EN	SET_BIT(1, 0)
#define m_WIN1_CBR_AXI_GATHER_EN	SET_BIT(1, 1)
#define m_WIN1_BIC_COE_SEL		SET_BIT(3, 2)
#define m_WIN1_VSD_YRGB_GT4		SET_BIT(1, 4)
#define m_WIN1_VSD_YRGB_GT2		SET_BIT(1, 5)
#define m_WIN1_VSD_CBR_GT4		SET_BIT(1, 6)
#define m_WIN1_VSD_CBR_GT2		SET_BIT(1, 7)
#define m_WIN1_YRGB_AXI_GATHER_NUM	SET_BIT(0xf, 8)
#define m_WIN1_CBR_AXI_GATHER_NUM	SET_BIT(7, 12)
#define m_WIN1_LINE_LOAD_MODE		SET_BIT(1, 15)
#define m_WIN1_YRGB_HOR_SCL_MODE	SET_BIT(3, 16)
#define m_WIN1_YRGB_VER_SCL_MODE	SET_BIT(3, 18)
#define m_WIN1_YRGB_HSD_MODE		SET_BIT(3, 20)
#define m_WIN1_YRGB_VSU_MODE		SET_BIT(1, 22)
#define m_WIN1_YRGB_VSD_MODE		SET_BIT(1, 23)
#define m_WIN1_CBR_HOR_SCL_MODE		SET_BIT(3, 24)
#define m_WIN1_CBR_VER_SCL_MODE		SET_BIT(3, 26)
#define m_WIN1_CBR_HSD_MODE		SET_BIT(3, 28)
#define m_WIN1_CBR_VSU_MODE		SET_BIT(1, 30)
#define m_WIN1_CBR_VSD_MODE		SET_BIT((u32)1, 31)

#define WIN1_COLOR_KEY			(0x0078)
#define v_WIN1_COLOR_KEY(x)		SET_BIT_MASK(x, 0, 0x3fffffff)
#define v_WIN1_COLOR_KEY_EN(x)		SET_BIT_MASK(x, 31, 1)
#define m_WIN1_COLOR_KEY		SET_BIT(0x3fffffff, 0)
#define m_WIN1_COLOR_KEY_EN		SET_BIT((u32)1, 31)

#define WIN1_VIR			(0x007c)
#define v_WIN1_VIR_STRIDE(x)		SET_BIT_MASK(x, 0, 0x3fff)
#define v_WIN1_VIR_STRIDE_UV(x)		SET_BIT_MASK(x, 16, 0x3fff)
#define m_WIN1_VIR_STRIDE		SET_BIT(0x3fff, 0)
#define m_WIN1_VIR_STRIDE_UV		SET_BIT(0x3fff, 16)

#define WIN1_YRGB_MST			(0x0080)
#define WIN1_CBR_MST			(0x0084)
#define WIN1_ACT_INFO			(0x0088)
#define v_WIN1_ACT_WIDTH(x)		SET_BIT_MASK(x-1, 0, 0x1fff)
#define v_WIN1_ACT_HEIGHT(x)		SET_BIT_MASK(x-1, 16, 0x1fff)
#define m_WIN1_ACT_WIDTH		SET_BIT(0x1fff, 0)
#define m_WIN1_ACT_HEIGHT		SET_BIT(0x1fff, 16)

#define WIN1_DSP_INFO			(0x008c)
#define v_WIN1_DSP_WIDTH(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN1_DSP_HEIGHT(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN1_DSP_WIDTH		SET_BIT(0xfff, 0)
#define m_WIN1_DSP_HEIGHT		SET_BIT(0xfff, 16)

#define WIN1_DSP_ST			(0x0090)
#define v_WIN1_DSP_XST(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN1_DSP_YST(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN1_DSP_XST			SET_BIT(0x1fff, 0)
#define m_WIN1_DSP_YST			SET_BIT(0x1fff, 16)

#define WIN1_SCL_FACTOR_YRGB		(0x0094)
#define v_WIN1_HS_FACTOR_YRGB(x)	SET_BIT_MASK(x, 0, 0xffff)
#define v_WIN1_VS_FACTOR_YRGB(x)	SET_BIT_MASK(x, 16, 0xffff)
#define m_WIN1_HS_FACTOR_YRGB		SET_BIT(0xffff, 0)
#define m_WIN1_VS_FACTOR_YRGB		SET_BIT((u32)0xffff, 16)

#define WIN1_SCL_FACTOR_CBR		(0x0098)
#define v_WIN1_HS_FACTOR_CBR(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_WIN1_VS_FACTOR_CBR(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_WIN1_HS_FACTOR_CBR		SET_BIT(0xffff, 0)
#define m_WIN1_VS_FACTOR_CBR		SET_BIT((u32)0xffff, 16)

#define WIN1_SCL_OFFSET			(0x009c)
#define v_WIN1_HS_OFFSET_YRGB(x)	SET_BIT_MASK(x, 0, 0xff)
#define v_WIN1_HS_OFFSET_CBR(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_WIN1_VS_OFFSET_YRGB(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN1_VS_OFFSET_CBR(x)		SET_BIT_MASK(x, 24, 0xff)

#define m_WIN1_HS_OFFSET_YRGB		SET_BIT(0xff, 0)
#define m_WIN1_HS_OFFSET_CBR		SET_BIT(0xff, 8)
#define m_WIN1_VS_OFFSET_YRGB		SET_BIT(0xff, 16)
#define m_WIN1_VS_OFFSET_CBR		SET_BIT((u32)0xff, 24)

#define WIN1_SRC_ALPHA_CTRL		(0x00a0)
#define v_WIN1_SRC_ALPHA_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_WIN1_SRC_COLOR_M0(x)		SET_BIT_MASK(x, 1, 1)
#define v_WIN1_SRC_ALPHA_M0(x)		SET_BIT_MASK(x, 2, 1)
#define v_WIN1_SRC_BLEND_M0(x)		SET_BIT_MASK(x, 3, 3)
#define v_WIN1_SRC_ALPHA_CAL_M0(x)	SET_BIT_MASK(x, 5, 1)
#define v_WIN1_SRC_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define v_WIN1_SRC_GLOBAL_ALPHA(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN1_FADING_VALUE(x)		SET_BIT_MASK(x, 24, 0xff)

#define m_WIN1_SRC_ALPHA_EN		SET_BIT(1, 0)
#define m_WIN1_SRC_COLOR_M0		SET_BIT(1, 1)
#define m_WIN1_SRC_ALPHA_M0		SET_BIT(1, 2)
#define m_WIN1_SRC_BLEND_M0		SET_BIT(3, 3)
#define m_WIN1_SRC_ALPHA_CAL_M0		SET_BIT(1, 5)
#define m_WIN1_SRC_FACTOR_M0		SET_BIT(7, 6)
#define m_WIN1_SRC_GLOBAL_ALPHA		SET_BIT(0xff, 16)
#define m_WIN1_FADING_VALUE		SET_BIT(0xff, 24)

#define WIN1_DST_ALPHA_CTRL		(0x00a4)
#define v_WIN1_DST_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define m_WIN1_DST_FACTOR_M0		SET_BIT(7, 6)

#define WIN1_FADING_CTRL		(0x00a8)
#define v_WIN1_FADING_OFFSET_R(x)	SET_BIT_MASK(x, 0, 0xff)
#define v_WIN1_FADING_OFFSET_G(x)	SET_BIT_MASK(x, 8, 0xff)
#define v_WIN1_FADING_OFFSET_B(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN1_FADING_EN(x)		SET_BIT_MASK(x, 24, 1)

#define m_WIN1_FADING_OFFSET_R		SET_BIT(0xff, 0)
#define m_WIN1_FADING_OFFSET_G		SET_BIT(0xff, 8)
#define m_WIN1_FADING_OFFSET_B		SET_BIT(0xff, 16)
#define m_WIN1_FADING_EN		SET_BIT(1, 24)

/*win2 register*/
#define WIN2_CTRL0			(0x00b0)
#define v_WIN2_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_WIN2_DATA_FMT(x)		SET_BIT_MASK(x, 1, 7)
#define v_WIN2_MST0_EN(x)		SET_BIT_MASK(x, 4, 1)
#define v_WIN2_MST1_EN(x)		SET_BIT_MASK(x, 5, 1)
#define v_WIN2_MST2_EN(x)		SET_BIT_MASK(x, 6, 1)
#define v_WIN2_MST3_EN(x)		SET_BIT_MASK(x, 7, 1)
#define v_WIN2_INTERLACE_READ(x)	SET_BIT_MASK(x, 8, 1)
#define v_WIN2_NO_OUTSTANDING(x)	SET_BIT_MASK(x, 9, 1)
#define v_WIN2_CSC_MODE(x)		SET_BIT_MASK(x, 10, 1)
#define v_WIN2_RB_SWAP(x)		SET_BIT_MASK(x, 12, 1)
#define v_WIN2_ALPHA_SWAP(x)		SET_BIT_MASK(x, 13, 1)
#define v_WIN2_ENDIAN_MODE(x)		SET_BIT_MASK(x, 14, 1)
#define v_WIN2_LUT_EN(x)		SET_BIT_MASK(x, 18, 1)

#define m_WIN2_EN			SET_BIT(1, 0)
#define m_WIN2_DATA_FMT			SET_BIT(7, 1)
#define m_WIN2_MST0_EN			SET_BIT(1, 4)
#define m_WIN2_MST1_EN			SET_BIT(1, 5)
#define m_WIN2_MST2_EN			SET_BIT(1, 6)
#define m_WIN2_MST3_EN			SET_BIT(1, 7)
#define m_WIN2_INTERLACE_READ		SET_BIT(1, 8)
#define m_WIN2_NO_OUTSTANDING		SET_BIT(1, 9)
#define m_WIN2_CSC_MODE			SET_BIT(1, 10)
#define m_WIN2_RB_SWAP			SET_BIT(1, 12)
#define m_WIN2_ALPHA_SWAP		SET_BIT(1, 13)
#define m_WIN2_ENDIAN_MODE		SET_BIT(1, 14)
#define m_WIN2_LUT_EN			SET_BIT(1, 18)

#define WIN2_CTRL1			(0x00b4)
#define v_WIN2_AXI_GATHER_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_WIN2_AXI_GATHER_NUM(x)	SET_BIT_MASK(x, 4, 0xf)
#define m_WIN2_AXI_GATHER_EN		SET_BIT(1, 0)
#define m_WIN2_AXI_GATHER_NUM		SET_BIT(0xf, 4)

#define WIN2_VIR0_1			(0x00b8)
#define v_WIN2_VIR_STRIDE0(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN2_VIR_STRIDE1(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN2_VIR_STRIDE0		SET_BIT(0x1fff, 0)
#define m_WIN2_VIR_STRIDE1		SET_BIT(0x1fff, 16)

#define WIN2_VIR2_3			(0x00bc)
#define v_WIN2_VIR_STRIDE2(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN2_VIR_STRIDE3(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN2_VIR_STRIDE2		SET_BIT(0x1fff, 0)
#define m_WIN2_VIR_STRIDE3		SET_BIT(0x1fff, 16)

#define WIN2_MST0			(0x00c0)
#define WIN2_DSP_INFO0			(0x00c4)
#define v_WIN2_DSP_WIDTH0(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN2_DSP_HEIGHT0(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN2_DSP_WIDTH0		SET_BIT(0xfff, 0)
#define m_WIN2_DSP_HEIGHT0		SET_BIT(0xfff, 16)

#define WIN2_DSP_ST0			(0x00c8)
#define v_WIN2_DSP_XST0(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN2_DSP_YST0(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN2_DSP_XST0			SET_BIT(0x1fff, 0)
#define m_WIN2_DSP_YST0			SET_BIT(0x1fff, 16)

#define WIN2_COLOR_KEY			(0x00cc)
#define v_WIN2_COLOR_KEY(x)		SET_BIT_MASK(x, 0, 0xffffff)
#define v_WIN2_KEY_EN(x)		SET_BIT_MASK(x, 24, 1)
#define m_WIN2_COLOR_KEY		SET_BIT(0xffffff, 0)
#define m_WIN2_KEY_EN			SET_BIT((u32)1, 24)

#define WIN2_MST1			(0x00d0)
#define WIN2_DSP_INFO1			(0x00d4)
#define v_WIN2_DSP_WIDTH1(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN2_DSP_HEIGHT1(x)		SET_BIT_MASK(x-1, 16, 0xfff)

#define m_WIN2_DSP_WIDTH1		SET_BIT(0xfff, 0)
#define m_WIN2_DSP_HEIGHT1		SET_BIT(0xfff, 16)

#define WIN2_DSP_ST1			(0x00d8)
#define v_WIN2_DSP_XST1(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN2_DSP_YST1(x)		SET_BIT_MASK(x, 16, 0x1fff)

#define m_WIN2_DSP_XST1			SET_BIT(0x1fff, 0)
#define m_WIN2_DSP_YST1			SET_BIT(0x1fff, 16)

#define WIN2_SRC_ALPHA_CTRL		(0x00dc)
#define v_WIN2_SRC_ALPHA_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_WIN2_SRC_COLOR_M0(x)		SET_BIT_MASK(x, 1, 1)
#define v_WIN2_SRC_ALPHA_M0(x)		SET_BIT_MASK(x, 2, 1)
#define v_WIN2_SRC_BLEND_M0(x)		SET_BIT_MASK(x, 3, 3)
#define v_WIN2_SRC_ALPHA_CAL_M0(x)	SET_BIT_MASK(x, 5, 1)
#define v_WIN2_SRC_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define v_WIN2_SRC_GLOBAL_ALPHA(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN2_FADING_VALUE(x)		SET_BIT_MASK(x, 24, 0xff)

#define m_WIN2_SRC_ALPHA_EN		SET_BIT(1, 0)
#define m_WIN2_SRC_COLOR_M0		SET_BIT(1, 1)
#define m_WIN2_SRC_ALPHA_M0		SET_BIT(1, 2)
#define m_WIN2_SRC_BLEND_M0		SET_BIT(3, 3)
#define m_WIN2_SRC_ALPHA_CAL_M0		SET_BIT(1, 5)
#define m_WIN2_SRC_FACTOR_M0		SET_BIT(7, 6)
#define m_WIN2_SRC_GLOBAL_ALPHA		SET_BIT(0xff, 16)
#define m_WIN2_FADING_VALUE		SET_BIT(0xff, 24)

#define WIN2_MST2			(0x00e0)
#define WIN2_DSP_INFO2			(0x00e4)
#define v_WIN2_DSP_WIDTH2(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN2_DSP_HEIGHT2(x)		SET_BIT_MASK(x-1, 16, 0xfff)

#define m_WIN2_DSP_WIDTH2		SET_BIT(0xfff, 0)
#define m_WIN2_DSP_HEIGHT2		SET_BIT(0xfff, 16)

#define WIN2_DSP_ST2			(0x00e8)
#define v_WIN2_DSP_XST2(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN2_DSP_YST2(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN2_DSP_XST2			SET_BIT(0x1fff, 0)
#define m_WIN2_DSP_YST2			SET_BIT(0x1fff, 16)

#define WIN2_DST_ALPHA_CTRL		(0x00ec)
#define v_WIN2_DST_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define m_WIN2_DST_FACTOR_M0		SET_BIT(7, 6)

#define WIN2_MST3			(0x00f0)
#define WIN2_DSP_INFO3			(0x00f4)
#define v_WIN2_DSP_WIDTH3(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN2_DSP_HEIGHT3(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN2_DSP_WIDTH3		SET_BIT(0xfff, 0)
#define m_WIN2_DSP_HEIGHT3		SET_BIT(0xfff, 16)

#define WIN2_DSP_ST3			(0x00f8)
#define v_WIN2_DSP_XST3(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN2_DSP_YST3(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN2_DSP_XST3			SET_BIT(0x1fff, 0)
#define m_WIN2_DSP_YST3			SET_BIT(0x1fff, 16)

#define WIN2_FADING_CTRL		(0x00fc)
#define v_WIN2_FADING_OFFSET_R(x)	SET_BIT_MASK(x, 0, 0xff)
#define v_WIN2_FADING_OFFSET_G(x)	SET_BIT_MASK(x, 8, 0xff)
#define v_WIN2_FADING_OFFSET_B(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN2_FADING_EN(x)		SET_BIT_MASK(x, 24, 1)

#define m_WIN2_FADING_OFFSET_R		SET_BIT(0xff, 0)
#define m_WIN2_FADING_OFFSET_G		SET_BIT(0xff, 8)
#define m_WIN2_FADING_OFFSET_B		SET_BIT(0xff, 16)
#define m_WIN2_FADING_EN		SET_BIT(1, 24)

/*win3 register*/
#define WIN3_CTRL0			(0x0100)
#define v_WIN3_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_WIN3_DATA_FMT(x)		SET_BIT_MASK(x, 1, 7)
#define v_WIN3_MST0_EN(x)		SET_BIT_MASK(x, 4, 1)
#define v_WIN3_MST1_EN(x)		SET_BIT_MASK(x, 5, 1)
#define v_WIN3_MST2_EN(x)		SET_BIT_MASK(x, 6, 1)
#define v_WIN3_MST3_EN(x)		SET_BIT_MASK(x, 7, 1)
#define v_WIN3_INTERLACE_READ(x)	SET_BIT_MASK(x, 8, 1)
#define v_WIN3_NO_OUTSTANDING(x)	SET_BIT_MASK(x, 9, 1)
#define v_WIN3_CSC_MODE(x)		SET_BIT_MASK(x, 10, 1)
#define v_WIN3_RB_SWAP(x)		SET_BIT_MASK(x, 12, 1)
#define v_WIN3_ALPHA_SWAP(x)		SET_BIT_MASK(x, 13, 1)
#define v_WIN3_ENDIAN_MODE(x)		SET_BIT_MASK(x, 14, 1)
#define v_WIN3_LUT_EN(x)		SET_BIT_MASK(x, 18, 1)

#define m_WIN3_EN			SET_BIT(1, 0)
#define m_WIN3_DATA_FMT			SET_BIT(7, 1)
#define m_WIN3_MST0_EN			SET_BIT(1, 4)
#define m_WIN3_MST1_EN			SET_BIT(1, 5)
#define m_WIN3_MST2_EN			SET_BIT(1, 6)
#define m_WIN3_MST3_EN			SET_BIT(1, 7)
#define m_WIN3_INTERLACE_READ		SET_BIT(1, 8)
#define m_WIN3_NO_OUTSTANDING		SET_BIT(1, 9)
#define m_WIN3_CSC_MODE			SET_BIT(1, 10)
#define m_WIN3_RB_SWAP			SET_BIT(1, 12)
#define m_WIN3_ALPHA_SWAP		SET_BIT(1, 13)
#define m_WIN3_ENDIAN_MODE		SET_BIT(1, 14)
#define m_WIN3_LUT_EN			SET_BIT(1, 18)

#define WIN3_CTRL1			(0x0104)
#define v_WIN3_AXI_GATHER_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_WIN3_AXI_GATHER_NUM(x)	SET_BIT_MASK(x, 4, 0xf)
#define m_WIN3_AXI_GATHER_EN		SET_BIT(1, 0)
#define m_WIN3_AXI_GATHER_NUM		SET_BIT(0xf, 4)

#define WIN3_VIR0_1			(0x0108)
#define v_WIN3_VIR_STRIDE0(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN3_VIR_STRIDE1(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN3_VIR_STRIDE0		SET_BIT(0x1fff, 0)
#define m_WIN3_VIR_STRIDE1		SET_BIT(0x1fff, 16)

#define WIN3_VIR2_3			(0x010c)
#define v_WIN3_VIR_STRIDE2(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN3_VIR_STRIDE3(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN3_VIR_STRIDE2		SET_BIT(0x1fff, 0)
#define m_WIN3_VIR_STRIDE3		SET_BIT(0x1fff, 16)

#define WIN3_MST0			(0x0110)
#define WIN3_DSP_INFO0			(0x0114)
#define v_WIN3_DSP_WIDTH0(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN3_DSP_HEIGHT0(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN3_DSP_WIDTH0		SET_BIT(0xfff, 0)
#define m_WIN3_DSP_HEIGHT0		SET_BIT(0xfff, 16)

#define WIN3_DSP_ST0			(0x0118)
#define v_WIN3_DSP_XST0(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN3_DSP_YST0(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN3_DSP_XST0			SET_BIT(0x1fff, 0)
#define m_WIN3_DSP_YST0			SET_BIT(0x1fff, 16)

#define WIN3_COLOR_KEY			(0x011c)
#define v_WIN3_COLOR_KEY(x)		SET_BIT_MASK(x, 0, 0xffffff)
#define v_WIN3_KEY_EN(x)		SET_BIT_MASK(x, 24, 1)
#define m_WIN3_COLOR_KEY		SET_BIT(0xffffff, 0)
#define m_WIN3_KEY_EN			SET_BIT((u32)1, 24)

#define WIN3_MST1			(0x0120)
#define WIN3_DSP_INFO1			(0x0124)
#define v_WIN3_DSP_WIDTH1(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN3_DSP_HEIGHT1(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN3_DSP_WIDTH1		SET_BIT(0xfff, 0)
#define m_WIN3_DSP_HEIGHT1		SET_BIT(0xfff, 16)

#define WIN3_DSP_ST1			(0x0128)
#define v_WIN3_DSP_XST1(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN3_DSP_YST1(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN3_DSP_XST1			SET_BIT(0x1fff, 0)
#define m_WIN3_DSP_YST1			SET_BIT(0x1fff, 16)

#define WIN3_SRC_ALPHA_CTRL		(0x012c)
#define v_WIN3_SRC_ALPHA_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_WIN3_SRC_COLOR_M0(x)		SET_BIT_MASK(x, 1, 1)
#define v_WIN3_SRC_ALPHA_M0(x)		SET_BIT_MASK(x, 2, 1)
#define v_WIN3_SRC_BLEND_M0(x)		SET_BIT_MASK(x, 3, 3)
#define v_WIN3_SRC_ALPHA_CAL_M0(x)	SET_BIT_MASK(x, 5, 1)
#define v_WIN3_SRC_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define v_WIN3_SRC_GLOBAL_ALPHA(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN3_FADING_VALUE(x)		SET_BIT_MASK(x, 24, 0xff)

#define m_WIN3_SRC_ALPHA_EN		SET_BIT(1, 0)
#define m_WIN3_SRC_COLOR_M0		SET_BIT(1, 1)
#define m_WIN3_SRC_ALPHA_M0		SET_BIT(1, 2)
#define m_WIN3_SRC_BLEND_M0		SET_BIT(3, 3)
#define m_WIN3_SRC_ALPHA_CAL_M0		SET_BIT(1, 5)
#define m_WIN3_SRC_FACTOR_M0		SET_BIT(7, 6)
#define m_WIN3_SRC_GLOBAL_ALPHA		SET_BIT(0xff, 16)
#define m_WIN3_FADING_VALUE		SET_BIT(0xff, 24)

#define WIN3_MST2			(0x0130)
#define WIN3_DSP_INFO2			(0x0134)
#define v_WIN3_DSP_WIDTH2(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN3_DSP_HEIGHT2(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN3_DSP_WIDTH2		SET_BIT(0xfff, 0)
#define m_WIN3_DSP_HEIGHT2		SET_BIT(0xfff, 16)

#define WIN3_DSP_ST2			(0x0138)
#define v_WIN3_DSP_XST2(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN3_DSP_YST2(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN3_DSP_XST2			SET_BIT(0x1fff, 0)
#define m_WIN3_DSP_YST2			SET_BIT(0x1fff, 16)

#define WIN3_DST_ALPHA_CTRL		(0x013c)
#define v_WIN3_DST_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define m_WIN3_DST_FACTOR_M0		SET_BIT(7, 6)

#define WIN3_MST3			(0x0140)
#define WIN3_DSP_INFO3			(0x0144)
#define v_WIN3_DSP_WIDTH3(x)		SET_BIT_MASK(x-1, 0, 0xfff)
#define v_WIN3_DSP_HEIGHT3(x)		SET_BIT_MASK(x-1, 16, 0xfff)
#define m_WIN3_DSP_WIDTH3		SET_BIT(0xfff, 0)
#define m_WIN3_DSP_HEIGHT3		SET_BIT(0xfff, 16)

#define WIN3_DSP_ST3			(0x0148)
#define v_WIN3_DSP_XST3(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_WIN3_DSP_YST3(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_WIN3_DSP_XST3			SET_BIT(0x1fff, 0)
#define m_WIN3_DSP_YST3			SET_BIT(0x1fff, 16)

#define WIN3_FADING_CTRL		(0x014c)
#define v_WIN3_FADING_OFFSET_R(x)	SET_BIT_MASK(x, 0, 0xff)
#define v_WIN3_FADING_OFFSET_G(x)	SET_BIT_MASK(x, 8, 0xff)
#define v_WIN3_FADING_OFFSET_B(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_WIN3_FADING_EN(x)		SET_BIT_MASK(x, 24, 1)

#define m_WIN3_FADING_OFFSET_R		SET_BIT(0xff, 0)
#define m_WIN3_FADING_OFFSET_G		SET_BIT(0xff, 8)
#define m_WIN3_FADING_OFFSET_B		SET_BIT(0xff, 16)
#define m_WIN3_FADING_EN		SET_BIT(1, 24)

/*hwc register*/
#define HWC_CTRL0			(0x0150)
#define v_HWC_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_HWC_DATA_FMT(x)		SET_BIT_MASK(x, 1, 7)
#define v_HWC_MODE(x)			SET_BIT_MASK(x, 4, 1)
#define v_HWC_SIZE(x)			SET_BIT_MASK(x, 5, 3)
#define v_HWC_INTERLACE_READ(x)		SET_BIT_MASK(x, 8, 1)
#define v_HWC_NO_OUTSTANDING(x)		SET_BIT_MASK(x, 9, 1)
#define v_HWC_CSC_MODE(x)		SET_BIT_MASK(x, 10, 1)
#define v_HWC_RB_SWAP(x)		SET_BIT_MASK(x, 12, 1)
#define v_HWC_ALPHA_SWAP(x)		SET_BIT_MASK(x, 13, 1)
#define v_HWC_ENDIAN_MODE(x)		SET_BIT_MASK(x, 14, 1)
#define v_HWC_LUT_EN(x)			SET_BIT_MASK(x, 18, 1)

#define m_HWC_EN			SET_BIT(1, 0)
#define m_HWC_DATA_FMT			SET_BIT(7, 1)
#define m_HWC_MODE			SET_BIT(1, 4)
#define m_HWC_SIZE			SET_BIT(3, 5)
#define m_HWC_INTERLACE_READ		SET_BIT(1, 8)
#define m_HWC_NO_OUTSTANDING		SET_BIT(1, 9)
#define m_HWC_CSC_MODE			SET_BIT(1, 10)
#define m_HWC_RB_SWAP			SET_BIT(1, 12)
#define m_HWC_ALPHA_SWAP		SET_BIT(1, 13)
#define m_HWC_ENDIAN_MODE		SET_BIT(1, 14)
#define m_HWC_LUT_EN			SET_BIT(1, 18)

#define HWC_CTRL1			(0x0154)
#define v_HWC_AXI_GATHER_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_HWC_AXI_GATHER_NUM(x)		SET_BIT_MASK(x, 4, 7)
#define m_HWC_AXI_GATHER_EN		SET_BIT(1, 0)
#define m_HWC_AXI_GATHER_NUM		SET_BIT(7, 4)

#define HWC_MST				(0x0158)
#define HWC_DSP_ST			(0x015c)
#define v_HWC_DSP_XST3(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_HWC_DSP_YST3(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_HWC_DSP_XST3			SET_BIT(0x1fff, 0)
#define m_HWC_DSP_YST3			SET_BIT(0x1fff, 16)

#define HWC_SRC_ALPHA_CTRL		(0x0160)
#define v_HWC_SRC_ALPHA_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_HWC_SRC_COLOR_M0(x)		SET_BIT_MASK(x, 1, 1)
#define v_HWC_SRC_ALPHA_M0(x)		SET_BIT_MASK(x, 2, 1)
#define v_HWC_SRC_BLEND_M0(x)		SET_BIT_MASK(x, 3, 3)
#define v_HWC_SRC_ALPHA_CAL_M0(x)	SET_BIT_MASK(x, 5, 1)
#define v_HWC_SRC_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define v_HWC_SRC_GLOBAL_ALPHA(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_HWC_FADING_VALUE(x)		SET_BIT_MASK(x, 24, 0xff)

#define m_HWC_SRC_ALPHA_EN		SET_BIT(1, 0)
#define m_HWC_SRC_COLOR_M0		SET_BIT(1, 1)
#define m_HWC_SRC_ALPHA_M0		SET_BIT(1, 2)
#define m_HWC_SRC_BLEND_M0		SET_BIT(3, 3)
#define m_HWC_SRC_ALPHA_CAL_M0		SET_BIT(1, 5)
#define m_HWC_SRC_FACTOR_M0		SET_BIT(7, 6)
#define m_HWC_SRC_GLOBAL_ALPHA		SET_BIT(0xff, 16)
#define m_HWC_FADING_VALUE		SET_BIT(0xff, 24)

#define HWC_DST_ALPHA_CTRL		(0x0164)
#define v_HWC_DST_FACTOR_M0(x)		SET_BIT_MASK(x, 6, 7)
#define m_HWC_DST_FACTOR_M0		SET_BIT(7, 6)

#define HWC_FADING_CTRL			(0x0168)
#define v_HWC_FADING_OFFSET_R(x)	SET_BIT_MASK(x, 0, 0xff)
#define v_HWC_FADING_OFFSET_G(x)	SET_BIT_MASK(x, 8, 0xff)
#define v_HWC_FADING_OFFSET_B(x)	SET_BIT_MASK(x, 16, 0xff)
#define v_HWC_FADING_EN(x)		SET_BIT_MASK(x, 24, 1)

#define m_HWC_FADING_OFFSET_R		SET_BIT(0xff, 0)
#define m_HWC_FADING_OFFSET_G		SET_BIT(0xff, 8)
#define m_HWC_FADING_OFFSET_B		SET_BIT(0xff, 16)
#define m_HWC_FADING_EN			SET_BIT(1, 24)

/*post process register*/
#define POST_DSP_HACT_INFO		(0x0170)
#define v_DSP_HACT_END_POST(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_HACT_ST_POST(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_HACT_END_POST		SET_BIT(0x1fff, 0)
#define m_DSP_HACT_ST_POST		SET_BIT(0x1fff, 16)

#define POST_DSP_VACT_INFO		(0x0174)
#define v_DSP_VACT_END_POST(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_VACT_ST_POST(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_VACT_END_POST		SET_BIT(0x1fff, 0)
#define m_DSP_VACT_ST_POST		SET_BIT(0x1fff, 16)

#define POST_SCL_FACTOR_YRGB		(0x0178)
#define v_POST_HS_FACTOR_YRGB(x)	SET_BIT_MASK(x, 0, 0xffff)
#define v_POST_VS_FACTOR_YRGB(x)	SET_BIT_MASK(x, 16, 0xffff)
#define m_POST_HS_FACTOR_YRGB		SET_BIT(0xffff, 0)
#define m_POST_VS_FACTOR_YRGB		SET_BIT(0xffff, 16)

#define POST_SCL_CTRL			(0x0180)
#define v_POST_HOR_SD_EN(x)		SET_BIT_MASK(x, 0, 1)
#define v_POST_VER_SD_EN(x)		SET_BIT_MASK(x, 1, 1)

#define m_POST_HOR_SD_EN		SET_BIT(0x1, 0)
#define m_POST_VER_SD_EN		SET_BIT(0x1, 1)

#define POST_DSP_VACT_INFO_F1		(0x0184)
#define v_DSP_VACT_END_POST_F1(x)	SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_VACT_ST_POST_F1(x)	SET_BIT_MASK(x, 16, 0x1fff)

#define m_DSP_VACT_END_POST_F1		SET_BIT(0x1fff, 0)
#define m_DSP_VACT_ST_POST_F1		SET_BIT(0x1fff, 16)

#define DSP_HTOTAL_HS_END		(0x0188)
#define v_DSP_HS_PW(x)			SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_HTOTAL(x)			SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_HS_PW			SET_BIT(0x1fff, 0)
#define m_DSP_HTOTAL			SET_BIT(0x1fff, 16)

#define DSP_HACT_ST_END			(0x018c)
#define v_DSP_HACT_END(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_HACT_ST(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_HACT_END			SET_BIT(0x1fff, 0)
#define m_DSP_HACT_ST			SET_BIT(0x1fff, 16)

#define DSP_VTOTAL_VS_END		(0x0190)
#define v_DSP_VS_PW(x)			SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_VTOTAL(x)			SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_VS_PW			SET_BIT(0x1fff, 0)
#define m_DSP_VTOTAL			SET_BIT(0x1fff, 16)

#define DSP_VACT_ST_END			(0x0194)
#define v_DSP_VACT_END(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_VACT_ST(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_VACT_END			SET_BIT(0x1fff, 0)
#define m_DSP_VACT_ST			SET_BIT(0x1fff, 16)

#define DSP_VS_ST_END_F1		(0x0198)
#define v_DSP_VS_END_F1(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_VS_ST_F1(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_VS_END_F1			SET_BIT(0x1fff, 0)
#define m_DSP_VS_ST_F1			SET_BIT(0x1fff, 16)

#define DSP_VACT_ST_END_F1		(0x019c)
#define v_DSP_VACT_END_F1(x)		SET_BIT_MASK(x, 0, 0x1fff)
#define v_DSP_VAC_ST_F1(x)		SET_BIT_MASK(x, 16, 0x1fff)
#define m_DSP_VACT_END_F1		SET_BIT(0x1fff, 0)
#define m_DSP_VAC_ST_F1			SET_BIT(0x1fff, 16)

/*pwm register*/
#define PWM_CTRL			(0x01a0)
#define v_PWM_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_PWM_MODE(x)			SET_BIT_MASK(x, 1, 3)

#define v_DUTY_POL(x)			SET_BIT_MASK(x, 3, 1)
#define v_INACTIVE_POL(x)		SET_BIT_MASK(x, 4, 1)
#define v_OUTPUT_MODE(x)		SET_BIT_MASK(x, 5, 1)
#define v_BL_EN(x)			SET_BIT_MASK(x, 8, 1)
#define v_CLK_SEL(x)			SET_BIT_MASK(x, 9, 1)
#define v_PRESCALE(x)			SET_BIT_MASK(x, 12, 7)
#define v_SCALE(x)			SET_BIT_MASK(x, 16, 0xff)
#define v_RPT(x)			SET_BIT_MASK(x, 24, 0xff)

#define m_PWM_EN			SET_BIT(1, 0)
#define m_PWM_MODE			SET_BIT(3, 1)

#define m_DUTY_POL			SET_BIT(1, 3)
#define m_INACTIVE_POL			SET_BIT(1, 4)
#define m_OUTPUT_MODE			SET_BIT(1, 5)
#define m_BL_EN				SET_BIT(1, 8)
#define m_CLK_SEL			SET_BIT(1, 9)
#define m_PRESCALE			SET_BIT(7, 12)
#define m_SCALE				SET_BIT(0xff, 16)
#define m_RPT				SET_BIT((u32)0xff, 24)

#define PWM_PERIOD_HPR			(0x01a4)
#define PWM_DUTY_LPR			(0x01a8)
#define PWM_CNT				(0x01ac)

/*BCSH register*/
#define BCSH_COLOR_BAR			(0x01b0)
#define v_BCSH_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_BCSH_COLOR_BAR_Y(x)		SET_BIT_MASK(x, 2, 0x3ff)
#define v_BCSH_COLOR_BAR_U(x)		SET_BIT_MASK(x, 12, 0x3ff)
#define v_BCSH_COLOR_BAR_V(x)		SET_BIT_MASK(x, 22, 0x3ff)

#define m_BCSH_EN			SET_BIT(1, 0)
#define m_BCSH_COLOR_BAR_Y		SET_BIT(0x3ff, 2)
#define m_BCSH_COLOR_BAR_U		SET_BIT(0x3ff, 12)
#define m_BCSH_COLOR_BAR_V		SET_BIT((u32)0x3ff, 22)

#define BCSH_BCS			(0x01b4)
#define v_BCSH_BRIGHTNESS(x)		SET_BIT_MASK(x, 0, 0xff)
#define v_BCSH_CONTRAST(x)		SET_BIT_MASK(x, 8, 0x1ff)
#define v_BCSH_SAT_CON(x)		SET_BIT_MASK(x, 20, 0x3ff)
#define v_BCSH_OUT_MODE(x)		SET_BIT_MASK(x, 30, 0x3)

#define m_BCSH_BRIGHTNESS		SET_BIT(0xff, 0)
#define m_BCSH_CONTRAST			SET_BIT(0x1ff, 8)
#define m_BCSH_SAT_CON			SET_BIT(0x3ff, 20)
#define m_BCSH_OUT_MODE			SET_BIT((u32)0x3, 30)

#define BCSH_H				(0x01b8)
#define v_BCSH_SIN_HUE(x)		SET_BIT_MASK(x, 0, 0x1ff)
#define v_BCSH_COS_HUE(x)		SET_BIT_MASK(x, 16, 0x1ff)

#define m_BCSH_SIN_HUE			SET_BIT(0x1ff, 0)
#define m_BCSH_COS_HUE			SET_BIT(0x1ff, 16)

#define CABC_CTRL0			(0x01c0)
#define v_CABC_EN(x)			SET_BIT_MASK(x, 0, 1)
#define v_CABC_CALC_PIXEL_NUM(x)	SET_BIT_MASK(x, 1, 0x7fffff)
#define v_CABC_STAGE_UP(x)		SET_BIT_MASK(x, 24, 0xff)
#define m_CABC_EN			SET_BIT(1, 0)
#define m_CABC_CALC_PIXEL_NUM		SET_BIT(0x7fffff, 1)
#define m_CABC_STAGE_UP			SET_BIT(0xff, 24)

#define CABC_CTRL1			(0x01c4)
#define v_CABC_TOTAL_NUM(x)		SET_BIT_MASK(x, 1, 0x7fffff)
#define v_CABC_STAGE_DOWN(x)		SET_BIT_MASK(x, 24, 0xff)
#define m_CABC_TOTAL_NUM		SET_BIT(0x7fffff, 1)
#define m_CABC_STAGE_DOWN		SET_BIT(0xff, 24)

#define CABC_GAUSS_LINE0_0		(0x01c8)
#define v_CABC_T_LINE0_0(x)		SET_BIT_MASK(x, 0, 0xff)
#define v_CABC_T_LINE0_1(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_CABC_T_LINE0_2(x)		SET_BIT_MASK(x, 16, 0xff)
#define v_CABC_T_LINE0_3(x)		SET_BIT_MASK(x, 24, 0xff)
#define m_CABC_T_LINE0_0		SET_BIT(0xff, 0)
#define m_CABC_T_LINE0_1		SET_BIT(0xff, 8)
#define m_CABC_T_LINE0_2		SET_BIT(0xff, 16)
#define m_CABC_T_LINE0_3		SET_BIT((u32)0xff, 24)

#define CABC_GAUSS_LINE0_1		(0x01cc)
#define v_CABC_T_LINE0_4(x)		SET_BIT_MASK(x, 0, 0xff)
#define v_CABC_T_LINE0_5(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_CABC_T_LINE0_6(x)		SET_BIT_MASK(x, 16, 0xff)
#define m_CABC_T_LINE0_4		SET_BIT(0xff, 0)
#define m_CABC_T_LINE0_5		SET_BIT(0xff, 8)
#define m_CABC_T_LINE0_6		SET_BIT(0xff, 16)

#define CABC_GAUSS_LINE1_0		(0x01d0)
#define v_CABC_T_LINE1_0(x)		SET_BIT_MASK(x, 0, 0xff)
#define v_CABC_T_LINE1_1(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_CABC_T_LINE1_2(x)		SET_BIT_MASK(x, 16, 0xff)
#define v_CABC_T_LINE1_3(x)		SET_BIT_MASK(x, 24, 0xff)
#define m_CABC_T_LINE1_0		SET_BIT(0xff, 0)
#define m_CABC_T_LINE1_1		SET_BIT(0xff, 8)
#define m_CABC_T_LINE1_2		SET_BIT(0xff, 16)
#define m_CABC_T_LINE1_3		SET_BIT((u32)0xff, 24)

#define CABC_GAUSS_LINE1_1		(0x01d4)
#define v_CABC_T_LINE1_4(x)		SET_BIT_MASK(x, 0, 0xff)
#define v_CABC_T_LINE1_5(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_CABC_T_LINE1_6(x)		SET_BIT_MASK(x, 16, 0xff)
#define m_CABC_T_LINE1_4		SET_BIT(0xff, 0)
#define m_CABC_T_LINE1_5		SET_BIT(0xff, 8)
#define m_CABC_T_LINE1_6		SET_BIT(0xff, 16)

#define CABC_GAUSS_LINE2_0		(0x01d8)
#define v_CABC_T_LINE2_0(x)		SET_BIT_MASK(x, 0, 0xff)
#define v_CABC_T_LINE2_1(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_CABC_T_LINE2_2(x)		SET_BIT_MASK(x, 16, 0xff)
#define v_CABC_T_LINE2_3(x)		SET_BIT_MASK(x, 24, 0xff)
#define m_CABC_T_LINE2_0		SET_BIT(0xff, 0)
#define m_CABC_T_LINE2_1		SET_BIT(0xff, 8)
#define m_CABC_T_LINE2_2		SET_BIT(0xff, 16)
#define m_CABC_T_LINE2_3		SET_BIT((u32)0xff, 24)

#define CABC_GAUSS_LINE2_1		(0x01dc)
#define v_CABC_T_LINE2_4(x)		SET_BIT_MASK(x, 0, 0xff)
#define v_CABC_T_LINE2_5(x)		SET_BIT_MASK(x, 8, 0xff)
#define v_CABC_T_LINE2_6(x)		SET_BIT_MASK(x, 16, 0xff)
#define m_CABC_T_LINE2_4		SET_BIT(0xff, 0)
#define m_CABC_T_LINE2_5		SET_BIT(0xff, 8)
#define m_CABC_T_LINE2_6		SET_BIT(0xff, 16)

/*FRC register*/
#define FRC_LOWER01_0			(0x01e0)
#define v_FRC_LOWER01_FRM0(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_FRC_LOWER01_FRM1(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_FRC_LOWER01_FRM0		SET_BIT(0xffff, 0)
#define m_FRC_LOWER01_FRM1		SET_BIT((u32)0xffff, 16)

#define FRC_LOWER01_1			(0x01e4)
#define v_FRC_LOWER01_FRM2(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_FRC_LOWER01_FRM3(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_FRC_LOWER01_FRM2		SET_BIT(0xffff, 0)
#define m_FRC_LOWER01_FRM3		SET_BIT((u32)0xffff, 16)

#define FRC_LOWER10_0			(0x01e8)
#define v_FRC_LOWER10_FRM0(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_FRC_LOWER10_FRM1(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_FRC_LOWER10_FRM0		SET_BIT(0xffff, 0)
#define m_FRC_LOWER10_FRM1		SET_BIT((u32)0xffff, 16)

#define FRC_LOWER10_1			(0x01ec)
#define v_FRC_LOWER10_FRM2(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_FRC_LOWER10_FRM3(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_FRC_LOWER10_FRM2		SET_BIT(0xffff, 0)
#define m_FRC_LOWER10_FRM3		SET_BIT((u32)0xffff, 16)

#define FRC_LOWER11_0			(0x01f0)
#define v_FRC_LOWER11_FRM0(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_FRC_LOWER11_FRM1(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_FRC_LOWER11_FRM0		SET_BIT(0xffff, 0)
#define m_FRC_LOWER11_FRM1		SET_BIT((u32)0xffff, 16)

#define FRC_LOWER11_1			(0x01f4)
#define v_FRC_LOWER11_FRM2(x)		SET_BIT_MASK(x, 0, 0xffff)
#define v_FRC_LOWER11_FRM3(x)		SET_BIT_MASK(x, 16, 0xffff)
#define m_FRC_LOWER11_FRM2		SET_BIT(0xffff, 0)
#define m_FRC_LOWER11_FRM3		SET_BIT((u32)0xffff, 16)

#define MMU_DTE_ADDR			(0x0300)
#define v_MMU_DTE_ADDR(x)		SET_BIT_MASK(x, 0, 0xffffffff)
#define m_MMU_DTE_ADDR			SET_BIT(0xffffffff, 0)

#define MMU_STATUS			(0x0304)
#define v_PAGING_ENABLED(x)		SET_BIT_MASK(x, 0, 1)
#define v_PAGE_FAULT_ACTIVE(x)		SET_BIT_MASK(x, 1, 1)
#define v_STAIL_ACTIVE(x)		SET_BIT_MASK(x, 2, 1)
#define v_MMU_IDLE(x)			SET_BIT_MASK(x, 3, 1)
#define v_REPLAY_BUFFER_EMPTY(x)	SET_BIT_MASK(x, 4, 1)
#define v_PAGE_FAULT_IS_WRITE(x)	SET_BIT_MASK(x, 5, 1)
#define v_PAGE_FAULT_BUS_ID(x)		SET_BIT_MASK(x, 6, 0x1f)
#define m_PAGING_ENABLED		SET_BIT(1, 0)
#define m_PAGE_FAULT_ACTIVE		SET_BIT(1, 1)
#define m_STAIL_ACTIVE			SET_BIT(1, 2)
#define m_MMU_IDLE			SET_BIT(1, 3)
#define m_REPLAY_BUFFER_EMPTY		SET_BIT(1, 4)
#define m_PAGE_FAULT_IS_WRITE		SET_BIT(1, 5)
#define m_PAGE_FAULT_BUS_ID		SET_BIT(0x1f, 6)

#define MMU_COMMAND			(0x0308)
#define v_MMU_CMD(x)			SET_BIT_MASK(x, 0, 0x3)
#define m_MMU_CMD			SET_BIT(0x3, 0)

#define MMU_PAGE_FAULT_ADDR		(0x030c)
#define v_PAGE_FAULT_ADDR(x)		SET_BIT_MASK(x, 0, 0xffffffff)
#define m_PAGE_FAULT_ADDR		SET_BIT(0xffffffff, 0)

#define MMU_ZAP_ONE_LINE		(0x0310)
#define v_MMU_ZAP_ONE_LINE(x)		SET_BIT_MASK(x, 0, 0xffffffff)
#define m_MMU_ZAP_ONE_LINE		SET_BIT(0xffffffff, 0)

#define MMU_INT_RAWSTAT			(0x0314)
#define v_PAGE_FAULT_RAWSTAT(x)		SET_BIT_MASK(x, 0, 1)
#define v_READ_BUS_ERROR_RAWSTAT(x)	SET_BIT_MASK(x, 1, 1)
#define m_PAGE_FAULT_RAWSTAT		SET_BIT(1, 0)
#define m_READ_BUS_ERROR_RAWSTAT	SET_BIT(1, 1)

#define MMU_INT_CLEAR			(0x0318)
#define v_PAGE_FAULT_CLEAR(x)		SET_BIT_MASK(x, 0, 1)
#define v_READ_BUS_ERROR_CLEAR(x)	SET_BIT_MASK(x, 1, 1)
#define m_PAGE_FAULT_CLEAR		SET_BIT(1, 0)
#define m_READ_BUS_ERROR_CLEAR		SET_BIT(1, 1)

#define MMU_INT_MASK			(0x031c)
#define v_PAGE_FAULT_MASK(x)		SET_BIT_MASK(x, 0, 1)
#define v_READ_BUS_ERROR_MASK(x)	SET_BIT_MASK(x, 1, 1)
#define m_PAGE_FAULT_MASK		SET_BIT(1, 0)
#define m_READ_BUS_ERROR_MASK		SET_BIT(1, 1)

#define MMU_INT_STATUS			(0x0320)
#define v_PAGE_FAULT_STATUS(x)		SET_BIT_MASK(x, 0, 1)
#define v_READ_BUS_ERROR_STATUS(x)	SET_BIT_MASK(x, 1, 1)
#define m_PAGE_FAULT_STATUS		SET_BIT(1, 0)
#define m_READ_BUS_ERROR_STATUS		SET_BIT(1, 1)

#define MMU_AUTO_GATING			(0x0324)
#define v_MMU_AUTO_GATING(x)		SET_BIT_MASK(x, 0, 1)
#define m_MMU_AUTO_GATING		SET_BIT(1, 0)

#define WIN2_LUT_ADDR			(0x0400)
#define WIN3_LUT_ADDR			(0x0800)
#define HWC_LUT_ADDR			(0x0c00)
#define GAMMA_LUT_ADDR			(0x1000)
#define MCU_BYPASS_WPORT		(0x2200)
#define MCU_BYPASS_RPORT		(0x2300)

#define PWM_MODE_ONE_SHOT		(0x0)
#define PWM_MODE_CONTINUOUS		(0x1)
#define PWM_MODE_CAPTURE		(0x2)
enum lb_mode {
	LB_YUV_3840X5 = 0x0,
	LB_YUV_2560X8,
	LB_RGB_3840X2,
	LB_RGB_2560X4,
	LB_RGB_1920X5,
	LB_RGB_1280X8
};

enum sacle_up_mode {
	SCALE_UP_BIL = 0x0,
	SCALE_UP_BIC
};

enum scale_down_mode {
	SCALE_DOWN_BIL = 0x0,
	SCALE_DOWN_AVG
};

/*ALPHA BLENDING MODE*/
enum alpha_mode {
	AB_USER_DEFINE = 0x0,
	AB_CLEAR,
	AB_SRC,
	AB_DST,
	AB_SRC_OVER,
	AB_DST_OVER,
	AB_SRC_IN,
	AB_DST_IN,
	AB_SRC_OUT,
	AB_DST_OUT,
	AB_SRC_ATOP,
	AB_DST_ATOP,
	XOR,
	AB_SRC_OVER_GLOBAL
};

enum src_alpha_mode {
	AA_STRAIGHT = 0x0,
	AA_INVERSE
};

enum global_alpha_mode {
	AA_GLOBAL = 0x0,
	AA_PER_PIX,
	AA_PER_PIX_GLOBAL
};

enum src_alpha_sel {
	AA_SAT = 0x0,
	AA_NO_SAT
};

enum src_color_mode {
	AA_SRC_PRE_MUL = 0x0,
	AA_SRC_NO_PRE_MUL
};

enum factor_mode {
	AA_ZERO = 0x0,
	AA_ONE,
	AA_SRC,
	AA_SRC_INVERSE,
	AA_SRC_GLOBAL
};

struct pwr_ctr {
	struct list_head list;

	char name[32];
	int type;
	int is_rst;
	int gpio;
	int atv_val;
	char rgl_name[32];
	int volt;
	int delay;
};

struct lcdc_device {
	int id;
	struct device *dev;
	struct lcdc_driver lcdc_drv;

	struct drm_display_mode *mode;
	struct list_head pwrlist_head;

	void __iomem *regs;
	/*back up reg*/
	void *regsbak;
	/* physical basic address of lcdc register*/
	u32 reg_phy_base;
	/* physical map length of lcdc register*/
	u32 len;
	/*one time only one process allowed to config the register*/
	spinlock_t reg_lock;

	int __iomem *dsp_lut_addr_base;

	/*used for primary or extended display device*/
	int prop;
	bool pre_init;
	/*if lcdc use 1.8v power supply*/
	bool pwr18;
	/*if aclk or hclk is closed , cess to register is not allowed*/
	bool clk_on;
	/*active layer counter, hen atv_layer_cnt = 0, isable lcdc*/
	u8 atv_layer_cnt;

	unsigned int irq;

	/*lcdc AHP clk*/
	struct clk *hclk;
	/*lcdc dclk*/
	struct clk *dclk;
	/*lcdc share memory frequency*/
	struct clk *aclk;
	u32 pixclock;
	/*1:standby, :wrok*/
	u32 standby;
};

struct alpha_config {
	/*win0_src_alpha_m0*/
	enum src_alpha_mode src_alpha_mode;
	/*win0_src_global_alpha*/
	u32 src_global_alpha_val;
	/*win0_src_blend_m0*/
	enum global_alpha_mode src_global_alpha_mode;
	/*win0_src_alpha_cal_m0*/
	enum src_alpha_sel src_alpha_cal_m0;
	/*win0_src_color_m0*/
	enum src_color_mode src_color_mode;
	/*win0_src_factor_m0*/
	enum factor_mode src_factor_mode;
	/*win0_dst_factor_m0*/
	enum factor_mode dst_factor_mode;
};

static inline void lcdc_writel(struct lcdc_device *lcdc_dev, u32 offset, u32 v)
{
	u32 *_pv = (u32 *)lcdc_dev->regsbak;

	_pv += (offset >> 2);
	*_pv = v;
	writel_relaxed(v, lcdc_dev->regs + offset);
}

static inline u32 lcdc_readl(struct lcdc_device *lcdc_dev, u32 offset)
{
	u32 v;
	u32 *_pv = (u32 *)lcdc_dev->regsbak;

	_pv += (offset >> 2);
	v = readl_relaxed(lcdc_dev->regs + offset);
	*_pv = v;

	return v;
}

static inline u32 lcdc_read_bit(struct lcdc_device *lcdc_dev,
				u32 offset, u32 msk)
{
	u32 _v = readl_relaxed(lcdc_dev->regs + offset);

	_v &= msk;

	return (_v >> msk);
}

static inline void lcdc_set_bit(struct lcdc_device *lcdc_dev,
				u32 offset, u32 msk)
{
	u32 *_pv = (u32 *)lcdc_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) |= msk;
	writel_relaxed(*_pv, lcdc_dev->regs + offset);
}

static inline void lcdc_clr_bit(struct lcdc_device *lcdc_dev,
				u32 offset, u32 msk)
{
	u32 *_pv = (u32 *)lcdc_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) &= (~msk);
	writel_relaxed(*_pv, lcdc_dev->regs + offset);
}

static inline void lcdc_msk_reg(struct lcdc_device *lcdc_dev,
				u32 offset, u32 msk, u32 v)
{
	u32 *_pv = (u32 *)lcdc_dev->regsbak;

	_pv += (offset >> 2);
	(*_pv) &= (~msk);
	(*_pv) |= v;
	writel_relaxed(*_pv, lcdc_dev->regs + offset);
}

static inline void lcdc_cfg_done(struct lcdc_device *lcdc_dev)
{
	writel_relaxed(0x01, lcdc_dev->regs + REG_CFG_DONE);
	dsb();
}
#endif /* _RK3288_LCDC_H_ */
