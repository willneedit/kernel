/*
 * Regulator driver for rk808 PMIC chip for rk31xx
 *
 * Based on rk808.c that is work by zhangqing<zhangqing@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/rk808.h>
#include <linux/mfd/core.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>


struct rk808 *g_rk808;

static struct mfd_cell rk808s[] = {
	{
		.name = "rk808-regulator",
	},
	{
		.name = "rk808-rtc",
	},
};

int rk808_i2c_read(struct i2c_client *client, u8 reg, u8 *valuep)
{
	struct i2c_msg msgs[2];
	u8 value = 0;
	int err;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(reg);
	msgs[0].buf = &reg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = sizeof(value);
	msgs[1].buf = &value;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err < 0)
		return err;

	*valuep = value;

	return 0;
}

int rk808_i2c_write(struct i2c_client *client, u8 reg, u8 value)
{
	u8 buffer[2] = { reg, value };
	struct i2c_msg msg;
	int err;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(buffer);
	msg.buf = buffer;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err < 0)
		return err;

	return 0;
}

u8 rk808_reg_read(struct rk808 *rk808, u8 reg)
{
	u8 val = 0;
	int ret;

	mutex_lock(&rk808->io_lock);

	ret = rk808_i2c_read(rk808->i2c, reg, &val);
/*	printk("reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)val&0xff);*/
	if (ret < 0) {
		mutex_unlock(&rk808->io_lock);
		return ret;
	}
	mutex_unlock(&rk808->io_lock);
	return (val & 0xff);
}
EXPORT_SYMBOL_GPL(rk808_reg_read);

int rk808_reg_write(struct rk808 *rk808, u8 reg, u8 val)
{
	int err = 0;

	mutex_lock(&rk808->io_lock);

	err = rk808_i2c_write(rk808->i2c, reg, val);
	if (err < 0)
		dev_err(rk808->dev, "Write for reg 0x%x failed\n", reg);

	mutex_unlock(&rk808->io_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rk808_reg_write);

int rk808_set_bits(struct rk808 *rk808, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int ret;

	mutex_lock(&rk808->io_lock);

	ret = rk808_i2c_read(rk808->i2c, reg, &tmp);
/*	printk("1 reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)tmp&0xff);*/
	if (ret < 0) {
		mutex_unlock(&rk808->io_lock);
		return ret;
	}
	tmp = (tmp & ~mask) | val;
	ret = rk808_i2c_write(rk808->i2c, reg, tmp);
/*	printk("reg write 0x%02x -> 0x%02x\n", (int)reg, (unsigned)val&0xff);*/
	if (ret < 0) {
		mutex_unlock(&rk808->io_lock);
		return ret;
	}
	ret = rk808_i2c_read(rk808->i2c, reg, &tmp);
	if (ret < 0) {
		mutex_unlock(&rk808->io_lock);
		return ret;
	}
/*	printk("2 reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)tmp&0xff);*/
	mutex_unlock(&rk808->io_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(rk808_set_bits);

int rk808_clear_bits(struct rk808 *rk808, u8 reg, u8 mask)
{
	u8 data;
	int err;

	mutex_lock(&rk808->io_lock);
	err = rk808_i2c_read(rk808->i2c, reg, &data);
	if (err < 0) {
		dev_err(rk808->dev, "read from reg %x failed\n", reg);
		goto out;
	}
	data &= ~mask;
	err = rk808_i2c_write(rk808->i2c, reg, data);
	if (err < 0)
		dev_err(rk808->dev, "write to reg %x failed\n", reg);
out:
	mutex_unlock(&rk808->io_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rk808_clear_bits);

#ifdef CONFIG_OF
static struct of_device_id rk808_of_match[] = {
	{ .compatible = "rockchip,rk808"},
	{ },
};
MODULE_DEVICE_TABLE(of, rk808_of_match);

static struct rk808_board *rk808_parse_dt(struct rk808 *rk808)
{
	struct rk808_board *pdata;
	struct device_node *np;

	np = of_node_get(rk808->dev->of_node);
	if (!np) {
		dev_err(rk808->dev, "could not find pmic sub-node\n");
		return NULL;
	}
	pdata = devm_kzalloc(rk808->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;
	pdata->irq = rk808->chip_irq;
	pdata->irq_base = -1;

	pdata->irq_gpio = of_get_named_gpio(np, "gpios", 0);
		if (!gpio_is_valid(pdata->irq_gpio)) {
			dev_err(rk808->dev, "invalid gpio: %d\n",
				pdata->irq_gpio);
			return NULL;
		}
	pdata->pm_off = of_property_read_bool(np,
	"rk808,system-power-controller");

	return pdata;
}

#else
static struct rk808_board *rk808_parse_dt(struct i2c_client *i2c)
{
	return NULL;
}
#endif


static void rk808_device_shutdown(void)
{
	int ret;
	struct rk808 *rk808 = g_rk808;

	dev_info(rk808->dev, "%s\n", __func__);

	ret = regmap_update_bits(rk808->regmap,
		RK808_INT_STS_MSK_REG1, (0x3 << 5), (0x3 << 5));
	/*close rtc int when power off*/
	ret = regmap_update_bits(rk808->regmap,
		RK808_RTC_INT_REG, (0x3 << 2), 0);
	/*close rtc int when power off*/
	ret = regmap_update_bits(rk808->regmap,
		RK808_DEVCTRL_REG, (0x1 << 3), (0x1 << 3));
	if (ret < 0)
		dev_err(rk808->dev, "rk808 power off error!\n");

	while (1)
		wfi();
}
EXPORT_SYMBOL_GPL(rk808_device_shutdown);

static int rk808_pre_init(struct rk808 *rk808)
{
	int ret, val;

	dev_info(rk808->dev, "%s,line=%d\n", __func__, __LINE__);
	 /***********set ILIM ************/
	val = rk808_reg_read(rk808, RK808_BUCK3_CONFIG_REG);
	val &= (~(0x7 << 0));
	val |= (0x2 << 0);
	ret = rk808_reg_write(rk808, RK808_BUCK3_CONFIG_REG, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_BUCK3_CONFIG_REG reg\n");
		return ret;
		}

	val = rk808_reg_read(rk808, RK808_BUCK4_CONFIG_REG);
	val &= (~(0x7 << 0));
	val |= (0x3 << 0);
	ret = rk808_reg_write(rk808, RK808_BUCK4_CONFIG_REG, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_BUCK4_CONFIG_REG reg\n");
		return ret;
		}

	val = rk808_reg_read(rk808, RK808_BOOST_CONFIG_REG);
	val &= (~(0x7 << 0));
	val |= (0x1 << 0);
	ret = rk808_reg_write(rk808, RK808_BOOST_CONFIG_REG, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_BOOST_CONFIG_REG reg\n");
		return ret;
		}
	/*****************************************/
	/***********set buck OTP function************/
	ret = rk808_reg_write(rk808, 0x6f, 0x5a);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write 0x6f reg\n");
		return ret;
		}

	ret = rk808_reg_write(rk808, 0x91, 0x80);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write 0x91 reg\n");
		return ret;
		}
	ret = rk808_reg_write(rk808, 0x92, 0x55);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write 0x92 reg\n");
		return ret;
		}
	/*****************************************/
	/***********set buck 12.5mv/us ************/
	val = rk808_reg_read(rk808, RK808_BUCK1_CONFIG_REG);
	val &= (~(0x3 << 3));
	val |= (0x3 << 0);
	ret = rk808_reg_write(rk808, RK808_BUCK1_CONFIG_REG, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_BUCK1_CONFIG_REG reg\n");
		return ret;
		}

	val = rk808_reg_read(rk808, RK808_BUCK2_CONFIG_REG);
	val &= (~(0x3 << 3));
	val |= (0x3 << 0);
	ret = rk808_reg_write(rk808, RK808_BUCK2_CONFIG_REG, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_BUCK2_CONFIG_REG reg\n");
		return ret;
		}
	/*****************************************/
	/*******enable switch and boost***********/
	val = rk808_reg_read(rk808, RK808_DCDC_EN_REG);
	val |= (0x3 << 5);/*enable switch1/2*/
	val |= (0x1 << 4);/*enable boost*/
	ret = rk808_reg_write(rk808, RK808_DCDC_EN_REG, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_DCDC_EN_REG reg\n");
		return ret;
	}
	/****************************************/
	/****************set vbat low **********/
	val = rk808_reg_read(rk808, RK808_VB_MON_REG);
	val &= (~(VBAT_LOW_VOL_MASK | VBAT_LOW_ACT_MASK));
	val |= (RK808_VBAT_LOW_3V5 | EN_VBAT_LOW_IRQ);
	ret = rk808_reg_write(rk808, RK808_VB_MON_REG, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK818_VB_MON_REG reg\n");
		return ret;
		}
	/**************************************/
	/**********mask int****************/
	val = rk808_reg_read(rk808, RK808_INT_STS_MSK_REG1);
	val |= (0x1 << 0); /*mask vout_lo_int*/
	ret = rk808_reg_write(rk808, RK808_INT_STS_MSK_REG1, val);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_INT_STS_MSK_REG1 reg\n");
		return ret;
		}
	/**********************************/
	/**********enable clkout2****************/
	ret = rk808_reg_write(rk808, RK808_CLK32OUT_REG, 0x01);
	if (ret < 0) {
		dev_err(rk808->dev, "Unable to write RK808_CLK32OUT_REG reg\n");
		return ret;
		}
	/**********************************/
	return 0;
}

static int rk808_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct rk808 *rk808;
	struct rk808_board *pdev;
	const struct of_device_id *match;
	int ret;
	dev_info(&client->dev, "%s,line=%d\n", __func__, __LINE__);

	if (client->dev.of_node) {
		match = of_match_device(rk808_of_match, &client->dev);
		if (!match) {
			dev_err(&client->dev, "Failed to find matching dt id\n");
			return -EINVAL;
		}
	}

	rk808 = devm_kzalloc(&client->dev, sizeof(struct rk808), GFP_KERNEL);
	if (rk808 == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	rk808->i2c = client;
	rk808->dev = &client->dev;
	i2c_set_clientdata(client, rk808);

	rk808->regmap = devm_regmap_init_i2c(client, &rk808_regmap_config);
	if (IS_ERR(rk808->regmap)) {
		dev_err(rk808->dev, "regmap initialization failed\n");
		return ret;
	}

	mutex_init(&rk808->io_lock);

	ret = rk808_reg_read(rk808, 0x2f);
	if ((ret < 0) || (ret == 0xff)) {
		dev_err(rk808->dev, "The device is not rk808 %d\n", ret);
		goto err;
	}

	ret = rk808_pre_init(rk808);
	if (ret < 0) {
		dev_err(rk808->dev, "The rk808_pre_init failed %d\n", ret);
		goto err;
	}

	if (rk808->dev->of_node)
		pdev = rk808_parse_dt(rk808);
	ret = rk808_irq_init(rk808, client->irq, pdev);
	if (ret < 0)
		goto err;

	ret = mfd_add_devices(rk808->dev, -1,
			      rk808s, ARRAY_SIZE(rk808s),
			      NULL, 0, NULL);

	g_rk808 = rk808;
	if (pdev->pm_off && !pm_power_off)
		pm_power_off = rk808_device_shutdown;

	return 0;

err:
	mfd_remove_devices(rk808->dev);
	return ret;

}

static int rk808_remove(struct i2c_client *i2c)
{
	struct rk808 *rk808 = i2c_get_clientdata(i2c);
	int i;

	for (i = 0; i < rk808->num_regulators; i++)
		if (rk808->rdev[i])
			regulator_unregister(rk808->rdev[i]);
	kfree(rk808->rdev);
	i2c_set_clientdata(i2c, NULL);
	kfree(rk808);

	return 0;
}

static const struct i2c_device_id rk808_ids[] = {
	 { "rk808", 0},
	 { }
};

//MODULE_DEVICE_TABLE(i2c, rk808_i2c_id);
MODULE_DEVICE_TABLE(i2c, rk808_ids);

static struct i2c_driver rk808_i2c_driver = {
	.driver = {
		.name = "rk808",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rk808_of_match),
	},
	.probe    = rk808_probe,
	.remove   = rk808_remove,
	.id_table = rk808_ids,
};

module_i2c_driver(rk808_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhangqing <zhangqing@rock-chips.com>");
MODULE_DESCRIPTION("rk808 PMIC driver");
