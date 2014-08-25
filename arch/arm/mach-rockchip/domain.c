/*
 * Rockchip Generic power domain support.
 *
 * Copyright (c) 2014 ROCKCHIP, Co. Ltd.
 * Author: dkl <dkl@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pm_domain.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <dt-bindings/domain/rockchip.h>

#include "domain.h"

struct rockchip_pm_domain rk3288_pm_domains[] = {
	DOMAIN_RK3288(RK3288_PD_GPU, 9, 9, 2),
};

struct rockchip_pmu rk3288_pmu = {
	.pwr_offset = 0x08,
	.status_offset = 0x0c,
	.req_offset = 0x10,
	.idle_offset = 0x14,
	.ack_offset = 0x14,
	.num_pds = ARRAY_SIZE(rk3288_pm_domains),
	.pds = rk3288_pm_domains,
};

static struct rockchip_pm_domain *rockchip_pd_id_to_pd(struct rockchip_pmu *pmu,
						       u32 id)
{
	int i;

	for (i = 0; i < pmu->num_pds; i++) {
		if (pmu->pds[i].id == id)
			return &(pmu->pds[i]);
	}

	return NULL;
}

static void rockchip_add_device_to_domain(struct rockchip_pm_domain *pd,
					  struct device *dev)
{
	int ret;

	dev_dbg(dev, "adding to power domain %s\n", pd->pd.name);

	while (1) {
		ret = pm_genpd_add_device(&pd->pd, dev);
		if (ret != -EAGAIN)
			break;
		cond_resched();
	}

	pm_genpd_dev_need_restore(dev, true);
}

static void rockchip_remove_device_from_domain(struct device *dev)
{
	struct generic_pm_domain *genpd = dev_to_genpd(dev);
	int ret;

	dev_dbg(dev, "removing from power domain %s\n", genpd->name);

	while (1) {
		ret = pm_genpd_remove_device(genpd, dev);
		if (ret != -EAGAIN)
			break;
		cond_resched();
	}
}

static void rockchip_read_domain_from_dt(struct device *dev)
{
	struct of_phandle_args pd_args;
	struct platform_device *pd_pdev;
	struct rockchip_pmu *pmu;
	struct rockchip_pm_domain *pd;
	int id, ret;

	ret = of_parse_phandle_with_args(dev->of_node, "rockchip,power-domain",
					 "#rockchip,power-domain-cells", 0,
					 &pd_args);
	if (ret < 0)
		return;

	pd_pdev = of_find_device_by_node(pd_args.np);
	if (!pd_pdev)
		return;

	pmu = (struct rockchip_pmu *)platform_get_drvdata(pd_pdev);
	id = pd_args.args[0];
	pd = rockchip_pd_id_to_pd(pmu, id);
	if (!pd) {
		dev_err(dev, "%s: id %d is unvalid\n", __func__, id);
		return;
	}

	rockchip_add_device_to_domain(pd, dev);
}

static int rockchip_pm_notifier_call(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct device *dev = data;

	switch (event) {
	case BUS_NOTIFY_BIND_DRIVER:
		if (dev->of_node)
			rockchip_read_domain_from_dt(dev);
		break;
	case BUS_NOTIFY_UNBOUND_DRIVER:
		rockchip_remove_device_from_domain(dev);
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block platform_nb = {
	.notifier_call = rockchip_pm_notifier_call,
};

static DEFINE_SPINLOCK(pmu_idle_lock);

static int rockchip_pmu_set_idle_request(struct generic_pm_domain *domain,
					 bool idle)
{
	struct rockchip_pm_domain *pd = to_rockchip_pd(domain);

	u32 idle_mask = BIT(pd->idle_shift);
	u32 idle_target = idle << (pd->idle_shift);
	u32 ack_mask = BIT(pd->ack_shift);
	u32 ack_target = idle << (pd->ack_shift);
	u32 mask = BIT(pd->req_shift);
	unsigned int val;
	unsigned long flags;

	spin_lock_irqsave(&pmu_idle_lock, flags);

	regmap_read(pd->pmu->regmap_pmu, pd->pmu->req_offset, &val);
	if (idle)
		val |=  mask;
	else
		val &= ~mask;

	regmap_write(pd->pmu->regmap_pmu, pd->pmu->req_offset, val);
	dsb();

	do {
		regmap_read(pd->pmu->regmap_pmu, pd->pmu->ack_offset, &val);
	} while ((val & ack_mask) != ack_target);

	do {
		regmap_read(pd->pmu->regmap_pmu, pd->pmu->idle_offset, &val);
	} while ((val & idle_mask) != idle_target);

	spin_unlock_irqrestore(&pmu_idle_lock, flags);

	return 0;
}

static bool rockchip_pmu_power_domain_is_on(struct generic_pm_domain *domain)
{
	struct rockchip_pm_domain *pd;
	unsigned int val;

	pd = to_rockchip_pd(domain);

	regmap_read(pd->pmu->regmap_pmu, pd->pmu->status_offset, &val);

	/* 1'b0: power on, 1'b1: power off */
	return !(val & BIT(pd->status_shift));
}

static noinline void rockchip_do_pmu_set_power_domain(
		struct generic_pm_domain *domain, bool on)
{
	struct rockchip_pm_domain *pd = to_rockchip_pd(domain);
	u8 bit = pd->pwr_shift;
	unsigned int val;

	regmap_read(pd->pmu->regmap_pmu, pd->pmu->pwr_offset, &val);
	if (on)
		val &= ~BIT(bit);
	else
		val |=  BIT(bit);
	regmap_write(pd->pmu->regmap_pmu, pd->pmu->pwr_offset, val);
	dsb();

	do {
		regmap_read(pd->pmu->regmap_pmu, pd->pmu->status_offset, &val);
	} while ((val & BIT(pd->status_shift)) == on);
}

static DEFINE_SPINLOCK(pmu_pd_lock);

static int rockchip_pmu_set_power_domain(struct generic_pm_domain *domain,
					 bool on)
{
	unsigned long flags;
	/*
	   struct rockchip_pm_domain *pd = to_rockchip_pd(domain);
	   struct rockchip_domain_qos qos;
	   int i;
	 */

	spin_lock_irqsave(&pmu_pd_lock, flags);

	if (rockchip_pmu_power_domain_is_on(domain) == on)
		goto out;

	if (!on) {
		/* if power down, idle request to NIU first */
		rockchip_pmu_set_idle_request(domain, true);
	}

	rockchip_do_pmu_set_power_domain(domain, on);

	if (on) {
		/* if power up, idle request release to NIU */
		rockchip_pmu_set_idle_request(domain, false);
	}

out:
	spin_unlock_irqrestore(&pmu_pd_lock, flags);
	return 0;
}

static int rockchip_pd_power(struct generic_pm_domain *domain, bool power_on)
{
	struct rockchip_pm_domain *pd = to_rockchip_pd(domain);
	int i, ret;

	for (i = 0 ; i < pd->num_clks; i++)
		if (pd->clks[i])
			clk_prepare_enable(pd->clks[i]);

	ret = rockchip_pmu_set_power_domain(domain, power_on);

	for (i = 0 ; i < pd->num_clks; i++)
		if (pd->clks[i])
			clk_disable_unprepare(pd->clks[i]);

	return ret;
}

static int rockchip_pd_power_on(struct generic_pm_domain *domain)
{
	return rockchip_pd_power(domain, true);
}

static int rockchip_pd_power_off(struct generic_pm_domain *domain)
{
	return rockchip_pd_power(domain, false);
}

static int rockchip_init_power_domain(struct device_node *np,
				      struct rockchip_pmu *pmu)
{
	struct platform_device *pdev;
	struct rockchip_pm_domain *pd;
	int on, cnt, i, ret;
	struct clk *clk;
	struct regmap *regmap_pmu;
	struct device_node *node;
	u32 id;

	node = of_parse_phandle(np, "rockchip,pmu", 0);
	regmap_pmu = syscon_node_to_regmap(node);
	if (IS_ERR(regmap_pmu)) {
		pr_err("%s: failed to get regmap_pmu", __func__);
		return PTR_ERR(regmap_pmu);
	}
	pmu->regmap_pmu = regmap_pmu;

	for_each_available_child_of_node(np, node) {
		ret = of_property_read_u32(node, "rockchip,pd-id", &id);
		if (ret != 0) {
			pr_err("%s: failed to get id\n", __func__);
			continue;
		}

		pd = rockchip_pd_id_to_pd(pmu, id);
		if (!pd) {
			pr_err("%s: id %d is unvalid\n", __func__, id);
			continue;
		}

		pd->pmu = pmu;

		cnt = of_count_phandle_with_args(node, "clocks",
						 "#clock-cells");
		if (cnt > 0) {
			pd->clks = kcalloc(cnt, sizeof(struct clk *),
					   GFP_KERNEL);
			if (!pd->clks) {
				pr_err("%s: failed to allocate memory for clks\n",
				       __func__);
				continue;
			}
			pd->num_clks = cnt;
			for (i = 0; i < cnt; i++) {
				clk = of_clk_get(np, i);
				if (IS_ERR(clk))
					pr_err("%s: failed to get clk(index %d)\n",
					       __func__, i);
				else
					pd->clks[i] = clk;
			}
		}

		pd->pd.name = kstrdup(node->name, GFP_KERNEL);
		pd->pd.power_off = rockchip_pd_power_off;
		pd->pd.power_on = rockchip_pd_power_on;
		pd->pd.of_node = np;

		/*FIXME*/
		on = true;

		pm_genpd_init(&pd->pd, NULL, !on);
	}

	pdev = of_find_device_by_node(np);
	platform_set_drvdata(pdev, pmu);

	return 0;
}

static const struct of_device_id rk3288_power_domain_dt_ids[] = {
	{ .compatible = "rockchip,rk3288-power-domain", },
	{ /* sentinel */ },
};

static __init int rk3288_pm_init_power_domain(struct device_node *np)
{
	return rockchip_init_power_domain(np, &rk3288_pmu);
}

static __init int rockchip_pm_init_power_domain(void)
{
	struct device_node *np;

	for_each_matching_node(np, rk3288_power_domain_dt_ids)
		rk3288_pm_init_power_domain(np);

	bus_register_notifier(&platform_bus_type, &platform_nb);

	return 0;
}

arch_initcall(rockchip_pm_init_power_domain);
