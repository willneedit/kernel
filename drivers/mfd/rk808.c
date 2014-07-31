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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mfd/rk808.h>
#include <linux/mfd/core.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/slab.h>

struct rk808_reg_data {
	int addr;
	int mask;
	int value;
};

static struct rk808 *g_rk808;

static struct resource rtc_resources[] = {
	{
		.start  = RK808_IRQ_RTC_ALARM,
		.end    = RK808_IRQ_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct mfd_cell rk808s[] = {
	{
		.name = "rk808-regulator",
	},
	{
		.name = "rk808-rtc",
		.num_resources = ARRAY_SIZE(rtc_resources),
		.resources = &rtc_resources[0],
	},
};

static struct rk808_reg_data pre_init_reg[] = {
	{RK808_BUCK3_CONFIG_REG,	~(0x7 << 0),	(0x2 << 0)},
	{RK808_BUCK4_CONFIG_REG,	~(0x7 << 0),	(0x3 << 0)},
	{RK808_BOOST_CONFIG_REG,	~(0x7 << 0),	(0x1 << 0)},
	{RK808_BUCK1_CONFIG_REG,	~(0x3 << 3),	(0x3 << 0)},
	{RK808_BUCK2_CONFIG_REG,	~(0x3 << 3),	(0x3 << 0)},
	{RK808_DCDC_EN_REG,		0xff,		(0x7 << 4)},
	{RK808_VB_MON_REG,		~0x17,		0x17},
	{RK808_INT_STS_MSK_REG1,	0,		1},
	{RK808_CLK32OUT_REG,		0,		1},
	{RK808_INT_STS_REG1,		0xff,		0},
	{RK808_INT_STS_REG2,		0xff,		0},
	{RK808_RTC_STATUS_REG,		0xff,		0},
};

static const struct regmap_irq rk808_irqs[] = {
	/* INT_STS */
	[RK808_IRQ_VOUT_LO] = {
		.mask = RK808_IRQ_VOUT_LO_MSK,
		.reg_offset = 0,
	},
	[RK808_IRQ_VB_LO] = {
		.mask = RK808_IRQ_VB_LO_MSK,
		.reg_offset = 0,
	},
	[RK808_IRQ_HOTDIE] = {
		.mask = RK808_IRQ_HOTDIE_MSK,
		.reg_offset = 0,
	},
	[RK808_IRQ_RTC_ALARM] = {
		.mask = RK808_IRQ_RTC_ALARM_MSK,
		.reg_offset = 0,
	},
	[RK808_IRQ_RTC_PERIOD] = {
		.mask = RK808_IRQ_RTC_PERIOD_MSK,
		.reg_offset = 0,
	},

	/* INT_STS2 */
	[RK808_IRQ_PLUG_IN_INT] = {
		.mask = RK808_IRQ_PLUG_IN_INT_MSK,
		.reg_offset = 1,
	},
	[RK808_IRQ_PLUG_OUT_INT] = {
		.mask = RK808_IRQ_PLUG_OUT_INT_MSK,
		.reg_offset = 1,
	},
};

static struct regmap_irq_chip rk808_irq_chip = {
	.name = "rk808",
	.irqs = rk808_irqs,
	.num_irqs = ARRAY_SIZE(rk808_irqs),
	.num_regs = 2,
	.irq_reg_stride = 2,
	.status_base = RK808_INT_STS_REG1,
	.mask_base = RK808_INT_STS_MSK_REG1,
	.ack_base = RK808_INT_STS_REG1,
};

static int rk808_irq_init(struct rk808 *rk808)
{
	int ret = 0;
	struct rk808_board *pdata = rk808->pdata;

	if (!pdata) {
		dev_err(rk808->dev, "No interrupt support, no pdata\n");
		return -EINVAL;
	}

	if (!pdata->irq) {
		dev_err(rk808->dev, "No interrupt support, no core IRQ\n");
		return -EINVAL;
	}

	ret = regmap_add_irq_chip(rk808->regmap, pdata->irq,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, pdata->irq_base,
		&rk808_irq_chip, &rk808->irq_data);
	if (ret < 0)
		dev_err(rk808->dev, "Failed to add irq_chip %d\n", ret);
	return ret;
}

static int rk808_irq_exit(struct rk808 *rk808)
{
	struct rk808_board *pdata = rk808->pdata;
	if (pdata->irq > 0)
		regmap_del_irq_chip(pdata->irq, rk808->irq_data);
	return 0;
}

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

	if (!rk808) {
		dev_err(rk808->dev, "%s have no g_rk808\n", __func__);
		return;
	}

	ret = regmap_update_bits(rk808->regmap,
		RK808_INT_STS_MSK_REG1, (0x3 << 5), (0x3 << 5));
	/* close rtc int when power off */
	ret = regmap_update_bits(rk808->regmap,
		RK808_RTC_INT_REG, (0x3 << 2), 0);
	/* close rtc int when power off */
	ret = regmap_update_bits(rk808->regmap,
		RK808_DEVCTRL_REG, (0x1 << 3), (0x1 << 3));
	if (ret < 0)
		dev_err(rk808->dev, "rk808 power off error!\n");

	while (1)
		wfi();
}

static int rk808_pre_init(struct rk808 *rk808)
{
	int i;
	int ret = 0;
	uint32_t val;
	int table_size = sizeof(pre_init_reg)/sizeof(struct rk808_reg_data);

	for (i = 0; i < table_size; i++) {
		ret = regmap_read(rk808->regmap, pre_init_reg[i].addr, &val);
		if (ret < 0) {
			dev_err(rk808->dev,
				"0x%x read err\n", pre_init_reg[i].addr);
			return ret;
		}
		val &= pre_init_reg[i].mask;
		val |= pre_init_reg[i].value;
		ret = regmap_write(rk808->regmap, pre_init_reg[i].addr, val);
		if (ret < 0) {
			dev_err(rk808->dev,
				"0x%x write err\n", pre_init_reg[i].addr);
			return ret;
		}
	}

	return 0;
}

static int rk808_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	uint32_t val;
	struct rk808 *rk808;
	struct rk808_board *pdata = dev_get_platdata(&client->dev);
	rk808 = devm_kzalloc(&client->dev, sizeof(struct rk808), GFP_KERNEL);
	if (rk808 == NULL) {
		return -ENOMEM;
	}
	rk808->i2c = client;
	rk808->dev = &client->dev;
	i2c_set_clientdata(client, rk808);

	rk808->regmap = devm_regmap_init_i2c(client, &rk808_regmap_config);
	if (IS_ERR(rk808->regmap)) {
		ret = PTR_ERR(rk808->regmap);
		dev_err(rk808->dev, "regmap initialization failed\n");
		return ret;
	}

	ret = regmap_read(rk808->regmap, 0x2f, &val);
	if (ret < 0) {
		dev_err(rk808->dev, "The device is not rk808 %d\n", ret);
		goto err_regmap;
	}

	ret = rk808_pre_init(rk808);
	if (ret < 0) {
		dev_err(rk808->dev, "The rk808_pre_init failed %d\n", ret);
		goto err_regmap;
	}

	if (IS_ENABLED(CONFIG_OF) && rk808->dev->of_node) {
		pdata = rk808_parse_dt(rk808);
		if (IS_ERR(pdata)) {
			ret = IS_ERR(pdata);
			goto err_regmap;
		}
	}

	if (!pdata)
		goto err_regmap;

	pdata->irq = client->irq;
	pdata->irq_base = -1;

	rk808->pdata = pdata;

	ret = rk808_irq_init(rk808);
	if (ret < 0)
		goto err_irq;


	ret = mfd_add_devices(rk808->dev, -1,
			      rk808s, ARRAY_SIZE(rk808s),
			      NULL, 0, NULL);
	if (ret < 0) {
		dev_err(rk808->dev, "failed to add MFD devices %d\n", ret);
		goto err_mfd;
	}

	g_rk808 = rk808;
	if (pdata->pm_off && !pm_power_off)
		pm_power_off = rk808_device_shutdown;

	return 0;
err_mfd:
	mfd_remove_devices(rk808->dev);
err_irq:
	if (pdata)
		devm_kfree(&client->dev, pdata);
	pdata = NULL;
err_regmap:
	if (rk808)
		devm_kfree(&client->dev, rk808);
	rk808 = NULL;
	return ret;
}

static int rk808_remove(struct i2c_client *i2c)
{
	struct rk808 *rk808 = i2c_get_clientdata(i2c);

	rk808_irq_exit(rk808);
	mfd_remove_devices(rk808->dev);
	return 0;
}

static const struct i2c_device_id rk808_ids[] = {
	 { "rk808", 0},
	 { }
};

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
