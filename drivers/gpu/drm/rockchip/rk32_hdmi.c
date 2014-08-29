/*
 *  Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 *Authors:
 *      zwl<zwl@rock-chips.com>
 *      Andy yan <andy.yan@rock-chips.com>
 * Based on drivers/video/rockchip/hdmi/chips/rk32/rk3188_hdmi.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/pinctrl/consumer.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>

#if defined(CONFIG_OF)
#include <linux/of.h>
#include <linux/of_device.h>
#endif

#if defined(CONFIG_DEBUG_FS)
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include "rk32_hdmi.h"
#include "rockchip_drm_drv.h"

#define GRF_SOC_CON6             0x025c
#define HDMI_SEL_VOP_LIT	(1 << 4)

#if defined(CONFIG_DEBUG_FS)
static const struct rk32_hdmi_reg_table hdmi_reg_table[] = {
	{IDENTIFICATION_BASE, CONFIG3_ID},
	{INTERRUPT_BASE, IH_MUTE},
	{VIDEO_SAMPLER_BASE, TX_BCBDATA1},
	{VIDEO_PACKETIZER_BASE, VP_MASK},
	{FRAME_COMPOSER_BASE, FC_DBGTMDS2},
	{HDMI_SOURCE_PHY_BASE, PHY_PLLCFGFREQ2},
	{I2C_MASTER_PHY_BASE, PHY_I2CM_SDA_HOLD},
	{AUDIO_SAMPLER_BASE, AHB_DMA_STPADDR_SET1_0},
	{MAIN_CONTROLLER_BASE, MC_SWRSTZREQ_2},
	{COLOR_SPACE_CONVERTER_BASE, CSC_SPARE_2},
	{HDCP_ENCRYPTION_ENGINE_BASE, HDCP_REVOC_LIST},
	{HDCP_BKSV_BASE, HDCPREG_BKSV4},
	{HDCP_AN_BASE, HDCPREG_AN7},
	{ENCRYPTED_DPK_EMBEDDED_BASE, HDCPREG_DPK6},
	{CEC_ENGINE_BASE, CEC_WKUPCTRL},
	{I2C_MASTER_BASE, I2CM_SCDC_UPDATE1},
};

static int rk32_hdmi_reg_show(struct seq_file *s, void *v)
{
	int i = 0, j = 0;
	u32 val = 0;
	struct rk32_hdmi *hdmi = s->private;
	seq_puts(s, "\n>>>hdmi_ctl reg");
	for (i = 0; i < 16; i++)
		seq_printf(s, " %2x", i);

	seq_puts(s,
		 "\n-----------------------------------------------------------------");

	for (i = 0; i < ARRAY_SIZE(hdmi_reg_table); i++) {
		for (j = hdmi_reg_table[i].reg_base;
		     j <= hdmi_reg_table[i].reg_end; j++) {
			val = hdmi_readl(hdmi, j);
			if ((j - hdmi_reg_table[i].reg_base) % 16 == 0)
				seq_printf(s, "\n>>>hdmi_ctl %2x:", j);
			seq_printf(s, " %02x", val);

		}
	}
	seq_puts(s,
		 "\n-----------------------------------------------------------------\n");

	return 0;
}

static ssize_t rk32_hdmi_reg_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	u32 reg;
	u32 val;
	char kbuf[25];
	struct rk32_hdmi *hdmi =
	    ((struct seq_file *)file->private_data)->private;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	if (sscanf(kbuf, "%x%x", &reg, &val) != 1)
		return -EINVAL;

	if ((reg < 0) || (reg > I2CM_SCDC_UPDATE1)) {
		dev_info(hdmi->dev, "it is no hdmi reg\n");
		return count;
	}
	dev_info(hdmi->dev, "/**********rk32 hdmi reg config******/");
	dev_info(hdmi->dev, "\n reg=%x val=%x\n", reg, val);
	hdmi_writel(hdmi, reg, val);

	return count;
}

static int rk32_hdmi_reg_open(struct inode *inode, struct file *file)
{
	struct rk32_hdmi *hdmi = inode->i_private;

	return single_open(file, rk32_hdmi_reg_show, hdmi);
}

static const struct file_operations rk32_hdmi_reg_fops = {
	.owner = THIS_MODULE,
	.open = rk32_hdmi_reg_open,
	.read = seq_read,
	.write = rk32_hdmi_reg_write,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

int hdmi_init_video_para(struct rockchip_hdmi_driver *hdmi_drv,
			 struct hdmi_video_para *video)
{
	video->vic = hdmi_drv->vic;
	video->input_mode = VIDEO_INPUT_RGB_YCBCR_444;
	video->input_color = VIDEO_INPUT_COLOR_RGB;
	video->output_mode = hdmi_drv->prop.sink_hdmi;
	video->format_3d = 0;	/* TODO modify according to EDID if need */
	video->pixel_repet = 0;
	/* 0:IT Video Format  1:CE Video Format
	 * TODO modify according to EDID
	 */
	video->color_limit_range = 1;

#ifdef SOURCE_ABOVE_10BIT
	if (hdmi_drv->prop.deepcolor & HDMI_COLOR_DEPTH_16BIT)
		video->color_depth = HDMI_COLOR_DEPTH_16BIT;
	else if (hdmi_drv->prop.deepcolor & HDMI_COLOR_DEPTH_12BIT)
		video->color_depth = HDMI_COLOR_DEPTH_12BIT;
	else
#endif
	if (hdmi_drv->prop.deepcolor & HDMI_COLOR_DEPTH_10BIT)
		video->color_depth = HDMI_COLOR_DEPTH_10BIT;
	else
		video->color_depth = HDMI_COLOR_DEPTH_8BIT;

	if (hdmi_drv->prop.ycbcr444)
		video->output_color = VIDEO_OUTPUT_YCBCR444;
	else if (hdmi_drv->prop.ycbcr422)
		video->output_color = VIDEO_OUTPUT_YCBCR422;
	else
		video->output_color = VIDEO_OUTPUT_RGB444;

	/*For DVI, output RGB */
	if (hdmi_drv->prop.sink_hdmi == 0)
		video->output_color = VIDEO_OUTPUT_RGB444;

	return 0;
}

#ifdef HDMI_INT_USE_POLL
#define HDMI_POLL_MDELAY	100
static void rk32_hdmi_delay_work(struct work_struct *work)
{

	struct delayed_work *delayed_work = to_delayed_work(work);
	struct rk32_hdmi *hdmi =
	    container_of(delayed_work, struct rk32_hdmi, delay_work);
	struct rockchip_hdmi_driver *hdmi_drv = &hdmi->drv;
	int phy_int = 0, i2cm_int = 0, phy_i2cm_int = 0, cec_int = 0;
	int aud_dma_int = 0;

	/* read interrupt */
	phy_int = hdmi_readl(hdmi, IH_PHY_STAT0);
	i2cm_int = hdmi_readl(hdmi, IH_I2CM_STAT0);
	phy_i2cm_int = hdmi_readl(hdmi, IH_I2CMPHY_STAT0);
	cec_int = hdmi_readl(hdmi, IH_CEC_STAT0);
	aud_dma_int = hdmi_readl(hdmi, IH_AHBDMAAUD_STAT0);
	/*
	   hdcp_int = hdmi_readl(hdmi, A_APIINTSTAT);
	 */

	/* clear interrupt */
	hdmi_writel(hdmi, IH_PHY_STAT0, phy_int);
	hdmi_writel(hdmi, IH_I2CM_STAT0, i2cm_int);
	hdmi_writel(hdmi, IH_I2CMPHY_STAT0, phy_i2cm_int);
	hdmi_writel(hdmi, IH_CEC_STAT0, cec_int);
	hdmi_writel(hdmi, IH_AHBDMAAUD_STAT0, aud_dma_int);
	/*
	   hdmi_writel(hdmi, A_APIINTCLR, hdcp_int);
	 */
	/* HPD or RX_SENSE */
	if ((phy_int & m_HPD) || ((phy_int & 0x3c) == 0x3c)) {
		if ((hdmi_drv->state != SYSTEM_CONFIG) && hdmi_drv->drm_dev)
			drm_helper_hpd_irq_event(hdmi_drv->drm_dev);
	}

	queue_delayed_work(hdmi->workqueue, &hdmi->delay_work,
			   msecs_to_jiffies(HDMI_POLL_MDELAY));
}
#endif

static int rk32_hdmi_clk_enable(struct rk32_hdmi *hdmi)
{
	clk_prepare_enable(hdmi->pclk);
	clk_prepare_enable(hdmi->hdcp_clk);
	return 0;
}

static int rk32_hdmi_clk_disable(struct rk32_hdmi *hdmi)
{
	clk_disable_unprepare(hdmi->pclk);
	clk_disable_unprepare(hdmi->hdcp_clk);
	return 0;
}

static int rk32_hdmi_init(struct rk32_hdmi *hdmi)
{
	int ret = 0;
	u32 val = 0;

	hdmi->lcdc_id = 1;
	/* lcdc source select */
	val = (0x01 << 20) | (hdmi->lcdc_id << 4);
	ret += regmap_write(hdmi->grf, GRF_SOC_CON6, val);
	ret += regmap_write(hdmi->grf, 0x74, 0x30002000);
	ret += regmap_write(hdmi->grf, 0x78, 0x00030002);
	if (ret != 0) {
		dev_err(hdmi->dev, "Could not write to GRF: %d\n", ret);
		return ret;
	}

	rk3288_hdmi_reset(hdmi);

	return ret;
}

static void rk32_hdmi_early_suspend(struct rk32_hdmi *hdmi)
{
	return;
}

static void rk32_hdmi_early_resume(struct rk32_hdmi *hdmi)
{
	return;
}

#if 0
static int rk32_hdmi_fb_event_notify(struct notifier_block *self,
				     unsigned long action, void *data)
{
	struct fb_event *event = data;
	int blank_mode = *((int *)event->data);

	if (action == FB_EARLY_EVENT_BLANK) {
		switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			break;
		default:
			rk32_hdmi_early_suspend(hdmi);
			break;
		}
	} else if (action == FB_EVENT_BLANK) {
		switch (blank_mode) {
		case FB_BLANK_UNBLANK:
			rk32_hdmi_early_resume(hdmi);
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block rk32_hdmi_fb_notifier = {
	.notifier_call = rk32_hdmi_fb_event_notify,
};
#endif

static int rk32_hdmi_connector_fill_modes(struct drm_connector *connector,
					  unsigned int max_width,
					  unsigned int max_height)
{
	max_width = 1920;
	max_height = 1080;
	return drm_helper_probe_single_connector_modes(connector,
						max_width, max_height);
}

static enum drm_connector_status rk32_hdmi_connector_detect(struct drm_connector
							    *connector,
							    bool force)
{
	struct rockchip_hdmi_driver *hdmi_drv =
	    container_of(connector, struct rockchip_hdmi_driver, connector);
	struct rk32_hdmi *hdmi = container_of(hdmi_drv, struct rk32_hdmi, drv);
	int hotplug = rk3288_hdmi_detect_hotplug(hdmi);
	enum drm_connector_status status =
	    (hotplug ==
	     HDMI_HPD_ACTIVED) ? connector_status_connected :
	    connector_status_disconnected;
	if (status != connector->status)
		dev_info(hdmi->dev, "[CONNECTOR:%d:%s] status updated from %s to %s\n",
		       connector->base.id, drm_get_connector_name(connector),
		       drm_get_connector_status_name(connector->status),
		       drm_get_connector_status_name(status));
	if (status == connector_status_disconnected) {
		hdmi_drv->state = HDMI_SLEEP;
		hdmi_drv->edid = NULL;
	}
	return status;
}

static void rk32_hdmi_connector_destroy(struct drm_connector *connector)
{
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs rk32_hdmi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = rk32_hdmi_connector_detect,
	.fill_modes = rk32_hdmi_connector_fill_modes,
	.destroy = rk32_hdmi_connector_destroy,
};


static int rk32_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_hdmi_driver *hdmi_drv = container_of(connector,
					struct rockchip_hdmi_driver, connector);
	struct edid *edid;
	/*the system will get edid more than one time,
	 * if we aready have get the edid success befor,
	 * we reused it*/
	if (hdmi_drv->edid)
		edid = hdmi_drv->edid;
	else
		edid = drm_get_edid(connector, &hdmi_drv->i2c);
	if (!edid)
		return -ENODEV;
	drm_mode_connector_update_edid_property(connector, edid);
	hdmi_drv->state = SYSTEM_CONFIG;
	hdmi_drv->edid = edid;
	return drm_add_edid_modes(connector, edid);
}

static struct drm_encoder *rk32_hdmi_connector_best_encoder(struct drm_connector
							    *connector)
{
	struct rockchip_hdmi_driver *hdmi_drv =
	    container_of(connector, struct rockchip_hdmi_driver, connector);
	return &hdmi_drv->encoder;
}

static enum drm_mode_status rk32_hdmi_connector_mode_valid(struct drm_connector
							   *connector,
							   struct
							   drm_display_mode
							   *mode)
{
	struct rockchip_hdmi_driver *hdmi_drv = container_of(connector,
					struct rockchip_hdmi_driver, connector);
	struct rk32_hdmi *hdmi = container_of(hdmi_drv, struct rk32_hdmi, drv);
	bool valid = false;

	valid = ((mode->vdisplay == 1080) && (mode->hdisplay == 1920)) ||
	    ((mode->vdisplay == 768) && (mode->hdisplay == 1024)) ||
	    ((mode->vdisplay == 720) && (mode->hdisplay == 1280)) ||
	    ((mode->vdisplay == 576) && (mode->hdisplay == 720)) ||
	    ((mode->vdisplay == 480) && (mode->hdisplay == 720));
	if (valid)
		dev_info(hdmi->dev, "%s valid mode xres:%d yres:%d \n",
			__func__, mode->hdisplay, mode->vdisplay);
	else
		return MODE_BAD;

	return MODE_OK;
}

static struct drm_connector_helper_funcs rk32_hdmi_connector_helper_funcs = {
	.get_modes = rk32_hdmi_connector_get_modes,
	.best_encoder = rk32_hdmi_connector_best_encoder,
	.mode_valid = rk32_hdmi_connector_mode_valid,
};

static void rk32_hdmi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct rockchip_hdmi_driver *hdmi_drv =
	    container_of(encoder, struct rockchip_hdmi_driver, encoder);
	struct rk32_hdmi *hdmi = container_of(hdmi_drv, struct rk32_hdmi, drv);
	dev_info(hdmi->dev, "%s mode:%d\n", __func__, mode);
	return;
	if (hdmi->drv.dpms_mode == mode)
		return;
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		mdelay(100);
		rk32_hdmi_early_resume(hdmi);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		rk32_hdmi_early_suspend(hdmi);
		break;
	default:
		break;
	}
	hdmi->drv.dpms_mode = mode;
}

static void rk32_hdmi_mode_fixup(struct drm_connector *connector,
				 const struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	struct drm_display_mode *m;
	enum drm_mode_status mode_ok;
	struct rockchip_hdmi_driver *hdmi_drv = container_of(connector,
					struct rockchip_hdmi_driver, connector);
	struct rk32_hdmi *hdmi = container_of(hdmi_drv,
					struct rk32_hdmi, drv);
	dev_info(hdmi->dev, "%s:%s\n",
		__func__, drm_get_connector_name(connector));

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	mode_ok = rk32_hdmi_connector_mode_valid(connector, adjusted_mode);

	/* just return if user desired mode exists. */
	if (mode_ok == MODE_OK)
		return;

	/*
	 * otherwise, find the most suitable mode among modes and change it
	 * to adjusted_mode.
	 */
	list_for_each_entry(m, &connector->modes, head) {
		mode_ok = rk32_hdmi_connector_mode_valid(connector, m);

		if (mode_ok == MODE_OK) {
			struct drm_mode_object base;
			struct list_head head;

			dev_info(hdmi->dev, "desired mode doesn't exist so\n"
			       "use the most suitable mode among modes:.\n"
			       "Adjusted Mode: [%d]x[%d] [%d]Hz\n",
			       m->hdisplay, m->vdisplay, m->vrefresh);

			/* preserve display mode header while copying. */
			head = adjusted_mode->head;
			base = adjusted_mode->base;
			memcpy(adjusted_mode, m, sizeof(*m));
			adjusted_mode->head = head;
			adjusted_mode->base = base;
			break;
		}
	}
}

static bool rk32_hdmi_encoder_mode_fixup(struct drm_encoder *encoder,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_connector *connector;
	if (!adjusted_mode->private) {
		struct rockchip_display_mode *priv_mode;
		priv_mode = kzalloc(sizeof(*priv_mode), GFP_KERNEL);
		priv_mode->out_type = ROCKCHIP_DISPLAY_TYPE_HDMI;
		adjusted_mode->private = (int *)priv_mode;
	}

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		if (connector->encoder == encoder)
			rk32_hdmi_mode_fixup(connector, mode, adjusted_mode);
	}

	return true;
}

static int drm_display_mode_to_vic(struct drm_display_mode *m)
{
	return 0;
}

static void rk32_hdmi_encoder_mode_set(struct drm_encoder *encoder,
				       struct drm_display_mode *m,
				       struct drm_display_mode *adjusted)
{
	struct rockchip_hdmi_driver *hdmi_drv =
	    container_of(encoder, struct rockchip_hdmi_driver, encoder);
	struct rk32_hdmi *hdmi = container_of(hdmi_drv, struct rk32_hdmi, drv);
	struct rockchip_drm_private *private = hdmi_drv->drm_dev->dev_private;
	struct rockchip_drm_crtc *rk_crtc;
	u32 val;
	int ret;
	int index;
	struct hdmi_video_para *video =
	    kzalloc(sizeof(struct hdmi_video_para), GFP_KERNEL);

	index = drm_crtc_index(encoder->crtc);

	rk_crtc = &private->rk_crtc[index];

	if (rk_crtc->id == ROCKCHIP_CRTC_VOPL)
		val = HDMI_SEL_VOP_LIT | (HDMI_SEL_VOP_LIT << 16);
	else
		val = HDMI_SEL_VOP_LIT << 16;
	ret = regmap_write(hdmi->grf, GRF_SOC_CON6, val);
	dev_info(hdmi->dev, "vop %s output to hdmi\n",
	       (rk_crtc->id == ROCKCHIP_CRTC_VOPL) ? "LIT" : "BIG");
	video->private = m;
	hdmi_drv->vic = HDMI_1280x720p_60Hz;
	hdmi_init_video_para(hdmi_drv, video);
	ret = rk3288_hdmi_config_video(hdmi, video);
	rk3288_hdmi_control_output(hdmi, HDMI_ENABLE);

}

static void rk32_hdmi_encoder_prepare(struct drm_encoder *encoder)
{
	printk(KERN_INFO "%s>>>>>>>>>>>\n", __func__);
}

static void rk32_hdmi_encoder_commit(struct drm_encoder *encoder)
{
	printk(KERN_INFO "%s>>>>>>>>>>>\n", __func__);
}

static void rk32_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	printk(KERN_INFO "%s>>>>>>>>>>>\n", __func__);
}

static struct drm_encoder_helper_funcs rk32_hdmi_encoder_helper_funcs = {
	.dpms = rk32_hdmi_encoder_dpms,
	.mode_fixup = rk32_hdmi_encoder_mode_fixup,
	.mode_set = rk32_hdmi_encoder_mode_set,
	.prepare = rk32_hdmi_encoder_prepare,
	.commit = rk32_hdmi_encoder_commit,
	.disable = rk32_hdmi_encoder_disable,
};

static void rk32_hdmi_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static struct drm_encoder_funcs rk32_hdmi_encoder_funcs = {
	.destroy = rk32_hdmi_encoder_destroy,
};

static int rk32_hdmi_bind(struct device *dev, struct device *master, void *data)
{
	struct rk32_hdmi *hdmi = dev_get_drvdata(dev);
	struct rockchip_hdmi_driver *hdmi_drv = &hdmi->drv;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct drm_device *drm_dev = data;
	int ret;

	hdmi_drv->drm_dev = drm_dev;

	encoder = &hdmi_drv->encoder;
	encoder->possible_crtcs = 1 << 0 | 1 << 1;

	ret = drm_encoder_init(drm_dev, encoder, &rk32_hdmi_encoder_funcs,
			       DRM_MODE_ENCODER_TMDS);
	if (ret) {
		DRM_ERROR("failed to initialize encoder with drm\n");
		return ret;
	}
	drm_encoder_helper_add(encoder, &rk32_hdmi_encoder_helper_funcs);

	connector = &hdmi_drv->connector;
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->dpms = DRM_MODE_DPMS_OFF;

	ret = drm_connector_init(drm_dev, connector,
				 &rk32_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		DRM_ERROR("failed to initialize connector with drm\n");
		goto err_free_encoder;
	}

	drm_connector_helper_add(connector, &rk32_hdmi_connector_helper_funcs);

	ret = drm_sysfs_connector_add(connector);
	if (ret) {
		DRM_ERROR("failed to add drm_sysfs\n");
		goto err_free_connector;
	}

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("failed to attach connector and encoder\n");
		goto err_free_connector_sysfs;
	}

	return 0;
err_free_connector_sysfs:
	drm_sysfs_connector_remove(connector);
err_free_connector:
	drm_connector_cleanup(connector);
err_free_encoder:
	drm_encoder_cleanup(encoder);
	return ret;
}

static void rk32_hdmi_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct rk32_hdmi *hdmi = dev_get_drvdata(dev);
	struct drm_encoder *encoder;

	encoder = &hdmi->drv.encoder;

	rk32_hdmi_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	encoder->funcs->destroy(encoder);
	drm_sysfs_connector_remove(&hdmi->drv.connector);
	drm_connector_cleanup(&hdmi->drv.connector);
	drm_encoder_cleanup(encoder);
}

static const struct component_ops rk32_hdmi_component_ops = {
	.bind = rk32_hdmi_bind,
	.unbind = rk32_hdmi_unbind,
};

#if defined(CONFIG_OF)
static int rk32_hdmi_parse_dt(struct rk32_hdmi *hdmi)
{
	int val = 0;
	struct device_node *np = hdmi->dev->of_node;

	if (!of_property_read_u32(np, "rockchips,hdmi_audio_source", &val))
		hdmi->drv.audio.type = val;

	hdmi->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(hdmi->grf)) {
		dev_err(hdmi->dev, "read rockchip,grf node failed\n");
		return PTR_ERR(hdmi->grf);
	}

	hdmi->pclk = devm_clk_get(hdmi->dev, "pclk_hdmi");
	if (IS_ERR(hdmi->pclk)) {
		dev_err(hdmi->dev, "Unable to get hdmi pclk\n");
		return PTR_ERR(hdmi->pclk);
	}

	hdmi->hdcp_clk = devm_clk_get(hdmi->dev, "hdcp_clk_hdmi");
	if (IS_ERR(hdmi->hdcp_clk)) {
		dev_err(hdmi->dev, "Unable to get hdmi hdcp clk\n");
		return PTR_ERR(hdmi->hdcp_clk);
	}

	return 0;
}

static const struct of_device_id rk32_hdmi_dt_ids[] = {
	{.compatible = "rockchip,rk32-hdmi",},
	{}
};
#endif

static int rk32_hdmi_probe(struct platform_device *pdev)
{
	struct rk32_hdmi *hdmi = NULL;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi) {
		dev_err(dev, "failed to allocate rk32 hdmi!\n");
		return -ENOMEM;
	}

	hdmi->dev = dev;
	platform_set_drvdata(pdev, hdmi);

	if (dev->of_node) {
		ret = rk32_hdmi_parse_dt(hdmi);
		if (ret < 0)
			return ret;
	} else {
		dev_err(dev, "no dts node found for  rk32 hdmi!\n");
		return -EINVAL;
	}

	/* request and remap iomem */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(hdmi->dev, "Unable to get register resource\n");
		return -EINVAL;
	}
	hdmi->regbase = devm_ioremap_resource(hdmi->dev, res);
	if (IS_ERR(hdmi->regbase)) {
		dev_err(hdmi->dev, "cannot ioremap registers\n");
		return PTR_ERR(hdmi->regbase);
	}

	ret = rockchip_drm_component_add(hdmi->dev, &rk32_hdmi_component_ops);
	if (ret) {
		dev_err(hdmi->dev, "add rockchip drm component failed.\n");
		return ret;
	}
	ret = rk32_hdmi_ddc_i2c_register(&hdmi->drv);
	if (ret) {
               dev_err(hdmi->dev, "failed to create i2c adapter for ddc\n");
	       return ret;
	}
	
	/* enable pd and pclk and hdcp_clk */
	rk32_hdmi_clk_enable(hdmi);
	rk32_hdmi_init(hdmi);

#ifndef HDMI_INT_USE_POLL
	/* get and request the IRQ */
	hdmi->irq = platform_get_irq(pdev, 0);
	if (hdmi->irq <= 0) {
		dev_err(hdmi->dev, "failed to get hdmi irq resource (%d).\n",
			hdmi->irq);
		return -ENXIO;
	}

	ret = devm_request_irq(hdmi->dev, hdmi->irq, hdmi_irq, 0,
			       dev_name(hdmi->dev), hdmi);
	if (ret) {
		dev_err(hdmi->dev, "hdmi request_irq failed (%d).\n", ret);
		return ret;
	}
#else

	hdmi->irq = 0;
	hdmi->workqueue = create_singlethread_workqueue("hdmi");
	INIT_DELAYED_WORK(&hdmi->delay_work, rk32_hdmi_delay_work);
	queue_delayed_work(hdmi->workqueue, &hdmi->delay_work,
			   msecs_to_jiffies(10));
#endif

#if defined(CONFIG_DEBUG_FS)
	hdmi->drv.debugfs_dir = debugfs_create_dir("rk32-hdmi", NULL);
	if (IS_ERR(hdmi->drv.debugfs_dir)) {
		dev_err(hdmi->dev, "failed to create debugfs dir!\n");
	} else {
		debugfs_create_file("hdmi", S_IRUSR, hdmi->drv.debugfs_dir,
				    hdmi, &rk32_hdmi_reg_fops);
	}
#endif
	dev_info(hdmi->dev, "probe success.\n");
	return 0;

}

static int rk32_hdmi_remove(struct platform_device *pdev)
{
	return 0;
}

static void rk32_hdmi_shutdown(struct platform_device *pdev)
{

}

struct platform_driver rk32_hdmi_driver = {
	.probe = rk32_hdmi_probe,
	.remove = rk32_hdmi_remove,
	.driver = {
		   .name = "rk32-hdmi",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(rk32_hdmi_dt_ids),
		   },
	.shutdown = rk32_hdmi_shutdown,
};
