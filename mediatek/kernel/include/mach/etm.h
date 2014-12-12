#ifndef __ETM_H
#define __ETM_H

struct etm_driver_data
{
	void __iomem *etm_regs;
	int is_ptm;
};

struct etb_driver_data
{
	void __iomem *etb_regs;
	void __iomem *funnel_regs;
	void __iomem *tpiu_regs;
	void __iomem *dem_regs;
	int use_etr;
	u32 etr_len;
	u32 etr_virt;
	dma_addr_t etr_phys;
};

#endif
