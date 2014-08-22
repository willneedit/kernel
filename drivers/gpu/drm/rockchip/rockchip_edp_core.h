/* rk3288_dp_core.h
*
* based on exynos_dp_core.h
* Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
* Author:
*      yxj <yxj@rock-chips.com>
*      cym <cym@rock-chips.com>
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the
* Free Software Foundation; either version 2 of the License, or (at your
* option) any later version.
*/

#ifndef __ROCKCHIP_DP_H
#define __ROCKCHIP_DP_H

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include "rockchip_drm_drv.h"

#define DP_TIMEOUT_LOOP_CNT 100
#define MAX_CR_LOOP 5
#define MAX_EQ_LOOP 5

#define GRF_EDP_REF_CLK_SEL_INTER		(1 << 4)
#define GRF_EDP_HDCP_EN				(1 << 15)
#define GRF_EDP_BIST_EN				(1 << 14)
#define GRF_EDP_MEM_CTL_BY_EDP			(1 << 13)
#define GRF_EDP_SECURE_EN			(1 << 3)
#define EDP_SEL_VOP_LIT				(1 << 5)

enum dp_irq_type {
	DP_IRQ_TYPE_HP_CABLE_IN,
	DP_IRQ_TYPE_HP_CABLE_OUT,
	DP_IRQ_TYPE_HP_CHANGE,
	DP_IRQ_TYPE_UNKNOWN,
};

enum color_coefficient {
	COLOR_YCBCR601,
	COLOR_YCBCR709
};

enum dynamic_range {
	VESA,
	CEA
};

enum pll_status {
	DP_PLL_UNLOCKED,
	DP_PLL_LOCKED
};

enum clock_recovery_m_value_type {
	CALCULATED_M,
	REGISTER_M
};

enum video_timing_recognition_type {
	VIDEO_TIMING_FROM_CAPTURE,
	VIDEO_TIMING_FROM_REGISTER
};

enum pattern_set {
	PRBS7,
	D10_2,
	TRAINING_PTN1,
	TRAINING_PTN2,
	DP_NONE
};

enum color_space {
	CS_RGB,
	CS_YCBCR422,
	CS_YCBCR444
};

enum color_depth {
	COLOR_6,
	COLOR_8,
	COLOR_10,
	COLOR_12
};

enum link_rate_type {
	LINK_RATE_1_62GBPS = 0x06,
	LINK_RATE_2_70GBPS = 0x0a
};

enum link_lane_count_type {
	LANE_CNT1 = 1,
	LANE_CNT2 = 2,
	LANE_CNT4 = 4
};

enum link_training_state {
	LT_START,
	LT_CLK_RECOVERY,
	LT_EQ_TRAINING,
	FINISHED,
	FAILED
};

enum voltage_swing_level {
	VOLTAGE_LEVEL_0,
	VOLTAGE_LEVEL_1,
	VOLTAGE_LEVEL_2,
	VOLTAGE_LEVEL_3,
};

enum pre_emphasis_level {
	PRE_EMPHASIS_LEVEL_0,
	PRE_EMPHASIS_LEVEL_1,
	PRE_EMPHASIS_LEVEL_2,
	PRE_EMPHASIS_LEVEL_3,
};

enum analog_power_block {
	AUX_BLOCK,
	CH0_BLOCK,
	CH1_BLOCK,
	CH2_BLOCK,
	CH3_BLOCK,
	ANALOG_TOTAL,
	POWER_ALL
};

struct video_info {
	char *name;

	bool h_sync_polarity;
	bool v_sync_polarity;
	bool interlaced;

	enum color_space color_space;
	enum dynamic_range dynamic_range;
	enum color_coefficient ycbcr_coeff;
	enum color_depth color_depth;

	enum link_rate_type link_rate;
	enum link_lane_count_type lane_count;
};

struct link_train {
	int eq_loop;
	int cr_loop[4];

	u8 link_rate;
	u8 lane_count;
	u8 training_lane[4];

	enum link_training_state lt_state;
};

/*
 * @grf_offset: offset inside the grf regmap for setting the rk3288 lvds
 */
struct rockchip_edp_soc_data {
	int grf_soc_con6;
	int grf_soc_con12;
};

struct rockchip_edp_device {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_panel *panel;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_display_mode mode;

	struct rockchip_edp_soc_data *soc_data;

	void __iomem *regs;
	struct regmap *grf;
	unsigned int irq;
	struct clk *clk_edp;
	struct clk *clk_24m_parent;
	struct clk *clk_24m;
	struct clk *pclk;
	struct reset_control *rst;
	struct link_train link_train;
	struct video_info video_info;
	bool clk_on;

	int dpms_mode;
};

void rockchip_edp_enable_video_mute(struct rockchip_edp_device *edp,
				    bool enable);
void rockchip_edp_stop_video(struct rockchip_edp_device *edp);
void rockchip_edp_lane_swap(struct rockchip_edp_device *edp, bool enable);
void rockchip_edp_init_refclk(struct rockchip_edp_device *edp);
void rockchip_edp_init_interrupt(struct rockchip_edp_device *edp);
void rockchip_edp_reset(struct rockchip_edp_device *edp);
void rockchip_edp_config_interrupt(struct rockchip_edp_device *edp);
u32 rockchip_edp_get_pll_lock_status(struct rockchip_edp_device *edp);
void rockchip_edp_analog_power_ctr(struct rockchip_edp_device *edp,
				   bool enable);
void rockchip_edp_init_analog_func(struct rockchip_edp_device *edp);
void rockchip_edp_init_hpd(struct rockchip_edp_device *edp);
void rockchip_edp_reset_aux(struct rockchip_edp_device *edp);
void rockchip_edp_init_aux(struct rockchip_edp_device *edp);
int rockchip_edp_get_plug_in_status(struct rockchip_edp_device *edp);
void rockchip_edp_enable_sw_function(struct rockchip_edp_device *edp);
int rockchip_edp_start_aux_transaction(struct rockchip_edp_device *edp);
int rockchip_edp_write_byte_to_dpcd(struct rockchip_edp_device *edp,
				    unsigned int reg_addr,
				    unsigned char data);
int rockchip_edp_read_byte_from_dpcd(struct rockchip_edp_device *edp,
				     unsigned int reg_addr,
				     unsigned char *data);
int rockchip_edp_write_bytes_to_dpcd(struct rockchip_edp_device *edp,
				     unsigned int reg_addr,
				     unsigned int count,
				     unsigned char data[]);
int rockchip_edp_read_bytes_from_dpcd(struct rockchip_edp_device *edp,
				      unsigned int reg_addr,
				      unsigned int count,
				      unsigned char data[]);
int rockchip_edp_select_i2c_device(struct rockchip_edp_device *edp,
				   unsigned int device_addr,
				   unsigned int reg_addr);
int rockchip_edp_read_byte_from_i2c(struct rockchip_edp_device *edp,
				   unsigned int device_addr,
				   unsigned int reg_addr,
				   unsigned int *data);
int rockchip_edp_read_bytes_from_i2c(struct rockchip_edp_device *edp,
				     unsigned int device_addr,
				     unsigned int reg_addr,
				     unsigned int count,
				     unsigned char edid[]);
void rockchip_edp_set_link_bandwidth(struct rockchip_edp_device *edp,
				     u32 bwtype);
void rockchip_edp_get_link_bandwidth(struct rockchip_edp_device *edp,
				     u32 *bwtype);
void rockchip_edp_set_lane_count(struct rockchip_edp_device *edp,
				 u32 count);
void rockchip_edp_get_lane_count(struct rockchip_edp_device *edp,
				 u32 *count);
void rockchip_edp_enable_enhanced_mode(struct rockchip_edp_device *edp,
				       bool enable);
void rockchip_edp_set_training_pattern(struct rockchip_edp_device *edp,
				       enum pattern_set pattern);
void rockchip_edp_set_lane0_pre_emphasis(struct rockchip_edp_device *edp,
					 u32 level);
void rockchip_edp_set_lane1_pre_emphasis(struct rockchip_edp_device *edp,
					 u32 level);
void rockchip_edp_set_lane2_pre_emphasis(struct rockchip_edp_device *edp,
					 u32 level);
void rockchip_edp_set_lane3_pre_emphasis(struct rockchip_edp_device *edp,
					 u32 level);
void rockchip_edp_set_lane0_link_training(struct rockchip_edp_device *edp,
					u32 training_lane);
void rockchip_edp_set_lane1_link_training(struct rockchip_edp_device *edp,
					u32 training_lane);
void rockchip_edp_set_lane2_link_training(struct rockchip_edp_device *edp,
					u32 training_lane);
void rockchip_edp_set_lane3_link_training(struct rockchip_edp_device *edp,
					u32 training_lane);
u32 rockchip_edp_get_lane0_link_training(struct rockchip_edp_device *edp);
u32 rockchip_edp_get_lane1_link_training(struct rockchip_edp_device *edp);
u32 rockchip_edp_get_lane2_link_training(struct rockchip_edp_device *edp);
u32 rockchip_edp_get_lane3_link_training(struct rockchip_edp_device *edp);
void rockchip_edp_reset_macro(struct rockchip_edp_device *edp);
int rockchip_edp_init_video(struct rockchip_edp_device *edp);

void rockchip_edp_set_video_color_format(struct rockchip_edp_device *edp,
				       u32 color_depth,
				       u32 color_space,
				       u32 dynamic_range,
				       u32 coeff);
int
rockchip_edp_is_slave_video_stream_clock_on(struct rockchip_edp_device *edp);
void rockchip_edp_set_video_cr_mn(struct rockchip_edp_device *edp,
				  enum clock_recovery_m_value_type type,
				  u32 m_value,
				  u32 n_value);
void rockchip_edp_set_video_timing_mode(struct rockchip_edp_device *edp,
					u32 type);
void rockchip_edp_enable_video_master(struct rockchip_edp_device *edp,
				      bool enable);
void rockchip_edp_start_video(struct rockchip_edp_device *edp);
int rockchip_edp_is_video_stream_on(struct rockchip_edp_device *edp);
void rockchip_edp_config_video_slave_mode(struct rockchip_edp_device *edp,
					struct video_info *video_info);
void rockchip_edp_enable_scrambling(struct rockchip_edp_device *edp);
void rockchip_edp_disable_scrambling(struct rockchip_edp_device *edp);
void rockchip_edp_rx_control(struct rockchip_edp_device *edp, bool enable);
int rockchip_edp_bist_cfg(struct rockchip_edp_device *edp);
void rockchip_edp_hw_link_training_en(struct rockchip_edp_device *edp);
int rockchip_edp_get_hw_lt_status(struct rockchip_edp_device *edp);
int rockchip_edp_wait_hw_lt_done(struct rockchip_edp_device *edp);
enum dp_irq_type rockchip_edp_get_irq_type(struct rockchip_edp_device *edp);
void rockchip_edp_clear_hotplug_interrupts(struct rockchip_edp_device *edp);

/* I2C EDID Chip ID, Slave Address */
#define I2C_EDID_DEVICE_ADDR			0x50
#define I2C_E_EDID_DEVICE_ADDR			0x30

#define EDID_BLOCK_LENGTH			0x80
#define EDID_HEADER_PATTERN			0x00
#define EDID_EXTENSION_FLAG			0x7e
#define EDID_CHECKSUM				0x7f

/* Definition for DPCD Register */
#define DPCD_ADDR_DPCD_REV			0x0000
#define DPCD_ADDR_MAX_LINK_RATE			0x0001
#define DPCD_ADDR_MAX_LANE_COUNT		0x0002
#define DPCD_ADDR_LINK_BW_SET			0x0100
#define DPCD_ADDR_LANE_COUNT_SET		0x0101
#define DPCD_ADDR_TRAINING_PATTERN_SET		0x0102
#define DPCD_ADDR_TRAINING_LANE0_SET		0x0103
#define DPCD_ADDR_LANE0_1_STATUS		0x0202
#define DPCD_ADDR_LANE_ALIGN_STATUS_UPDATED	0x0204
#define DPCD_ADDR_ADJUST_REQUEST_LANE0_1	0x0206
#define DPCD_ADDR_ADJUST_REQUEST_LANE2_3	0x0207
#define DPCD_ADDR_TEST_REQUEST			0x0218
#define DPCD_ADDR_TEST_RESPONSE			0x0260
#define DPCD_ADDR_TEST_EDID_CHECKSUM		0x0261
#define DPCD_ADDR_SINK_POWER_STATE		0x0600

/* DPCD_ADDR_MAX_LANE_COUNT */
#define DPCD_ENHANCED_FRAME_CAP(x)		(((x) >> 7) & 0x1)
#define DPCD_MAX_LANE_COUNT(x)			((x) & 0x1f)

/* DPCD_ADDR_LANE_COUNT_SET */
#define DPCD_ENHANCED_FRAME_EN			(0x1 << 7)
#define DPCD_LANE_COUNT_SET(x)			((x) & 0x1f)

/* DPCD_ADDR_TRAINING_PATTERN_SET */
#define DPCD_SCRAMBLING_DISABLED		(0x1 << 5)
#define DPCD_SCRAMBLING_ENABLED			(0x0 << 5)
#define DPCD_TRAINING_PATTERN_2			(0x2 << 0)
#define DPCD_TRAINING_PATTERN_1			(0x1 << 0)
#define DPCD_TRAINING_PATTERN_DISABLED		(0x0 << 0)

/* DPCD_ADDR_TRAINING_LANE0_SET */
#define DPCD_MAX_PRE_EMPHASIS_REACHED		(0x1 << 5)
#define DPCD_PRE_EMPHASIS_SET(x)		(((x) & 0x3) << 3)
#define DPCD_PRE_EMPHASIS_GET(x)		(((x) >> 3) & 0x3)
#define DPCD_PRE_EMPHASIS_PATTERN2_LEVEL0	(0x0 << 3)
#define DPCD_MAX_SWING_REACHED			(0x1 << 2)
#define DPCD_VOLTAGE_SWING_SET(x)		(((x) & 0x3) << 0)
#define DPCD_VOLTAGE_SWING_GET(x)		(((x) >> 0) & 0x3)
#define DPCD_VOLTAGE_SWING_PATTERN1_LEVEL0	(0x0 << 0)

/* DPCD_ADDR_LANE0_1_STATUS */
#define DPCD_LANE_SYMBOL_LOCKED			(0x1 << 2)
#define DPCD_LANE_CHANNEL_EQ_DONE		(0x1 << 1)
#define DPCD_LANE_CR_DONE			(0x1 << 0)
#define DPCD_CHANNEL_EQ_BITS			(DPCD_LANE_CR_DONE|	\
						 DPCD_LANE_CHANNEL_EQ_DONE|\
						 DPCD_LANE_SYMBOL_LOCKED)

/* DPCD_ADDR_LANE_ALIGN__STATUS_UPDATED */
#define DPCD_LINK_STATUS_UPDATED		(0x1 << 7)
#define DPCD_DOWNSTREAM_PORT_STATUS_CHANGED	(0x1 << 6)
#define DPCD_INTERLANE_ALIGN_DONE		(0x1 << 0)

/* DPCD_ADDR_TEST_REQUEST */
#define DPCD_TEST_EDID_READ			(0x1 << 2)

/* DPCD_ADDR_TEST_RESPONSE */
#define DPCD_TEST_EDID_CHECKSUM_WRITE		(0x1 << 2)

/* DPCD_ADDR_SINK_POWER_STATE */
#define DPCD_SET_POWER_STATE_D0			(0x1 << 0)
#define DPCD_SET_POWER_STATE_D4			(0x2 << 0)

#define DPCD_SYMBOL_ERR_CONUT_LANE0		0x210

#endif
