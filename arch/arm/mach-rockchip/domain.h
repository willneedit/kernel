#ifndef __MACH_ROCKCHIP_DAMAIN_H
#define __MACH_ROCKCHIP_DAMAIN_H

#define CPU_AXI_QOS_PRIORITY    0x08
#define CPU_AXI_QOS_MODE        0x0c
#define CPU_AXI_QOS_BANDWIDTH   0x10
#define CPU_AXI_QOS_SATURATION  0x14
#define CPU_AXI_QOS_EXTCONTROL  0x18

#define QOS_NUM_REGS 5

struct rockchip_domain_qos {
	struct regmap   *regmap_qos;
	u32		qos_saves[QOS_NUM_REGS];
};

struct rockchip_pm_domain {
	u32				id;
	int				pwr_shift;
	int				status_shift;
	int				req_shift;
	int				idle_shift;
	int				ack_shift;
	u8				num_clks;
	struct clk			**clks;
	u8				num_qos;
	struct rockchip_domain_qos	*qos;
	struct generic_pm_domain	pd;
	struct rockchip_pmu		*pmu;
};

struct rockchip_pmu {
	struct regmap			*regmap_pmu;
	u32				pwr_offset;
	u32				status_offset;
	u32				req_offset;
	u32				idle_offset;
	u32				ack_offset;
	u8				num_pds;
	struct rockchip_pm_domain	*pds;
};

#define to_rockchip_pd(_gpd) container_of(_gpd, struct rockchip_pm_domain, pd)

#define DOMAIN(_id, _pwr_s, _status_s, _req_s, _idle_s, _ack_s)	\
{								\
	.id = _id,						\
	.pwr_shift = _pwr_s,					\
	.status_shift = _status_s,				\
	.req_shift = _req_s,					\
	.idle_shift = _idle_s,					\
	.ack_shift = _ack_s,					\
}

#define DOMAIN_RK3288(_id, _pwr_s, _status_s, _req_s) \
	DOMAIN(_id, _pwr_s, _status_s, _req_s, _req_s, (_req_s+16))

#endif
