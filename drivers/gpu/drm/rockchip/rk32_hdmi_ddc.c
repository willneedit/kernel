/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Authors: Andy yan <andy.yan@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rk32_hdmi.h"


static int hdmi_i2c_xfer(struct i2c_adapter *i2c,
			struct i2c_msg *msgs, int num)
{
	struct rockchip_hdmi_driver *hdmi_drv =
		container_of(i2c, struct rockchip_hdmi_driver, i2c);
	struct rk32_hdmi *hdmi = container_of(hdmi_drv, struct rk32_hdmi, drv);
	struct i2c_msg *p;
	int block ;
	char *buf;
	int ret;
	if (num == 2)  {
		p = &msgs[0];
		block = (p->buf[0]) / EDID_LENGTH;
		p = &msgs[1];
		buf = p->buf;
	} else if ( num == 3) {
		p = &msgs[1];
                block = (p->buf[0]) / EDID_LENGTH;
                p = &msgs[2];
                buf = p->buf;	
	}

	if (p->len < EDID_LENGTH)
		return num;
	ret = rk3288_hdmi_read_edid(hdmi, block, buf);
	if (ret)
		return 0;
	if(!hdmi_drv->specs)
		hdmi_drv->specs = kzalloc(sizeof(struct fb_monspecs), GFP_KERNEL);
	if (!hdmi_drv->specs)
		return -ENOMEM;
	fb_edid_to_monspecs(buf, hdmi_drv->specs);
	printk("%s block :%d  num %d \n", __func__, block,num);

	return num;
}

static u32 hdmi_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm hdmi_i2c_algorithm = {
	.master_xfer	= hdmi_i2c_xfer,
	.functionality	= hdmi_i2c_func,
};


void rk32_hdmi_ddc_i2c_destroy(struct i2c_adapter *i2c)
{
	i2c_del_adapter(i2c);
}

int rk32_hdmi_ddc_i2c_register(struct rockchip_hdmi_driver *hdmi_drv)
{
	struct rk32_hdmi *hdmi = container_of(hdmi_drv,
                                        struct rk32_hdmi, drv);
	struct i2c_adapter *i2c = NULL;
	int ret;


	i2c = &hdmi_drv->i2c;
	i2c->owner = THIS_MODULE;
	i2c->class = I2C_CLASS_DDC;
	snprintf(i2c->name, sizeof(i2c->name), "rk32-hdmi-ddc");
	i2c->dev.parent = hdmi->dev;
	i2c->algo = &hdmi_i2c_algorithm;

	ret = i2c_add_adapter(i2c);

	return ret;

}
