/*
    TurboSight TBS PCIE DVB driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 www.tbsdtv.com
*/

#ifndef _TBS_PCIE_H_
#define _TBS_PCIE_H_

#include <linux/pci.h>
#include <linux/interrupt.h>

#include "dvb_demux.h"
#include "dmxdev.h"
#include "dvb_frontend.h"
#include "dvb_net.h"

#define TBS_PCIE_WRITE(__addr, __offst, __data)	writel((__data), (dev->mmio + (__addr + __offst)))
#define TBS_PCIE_READ(__addr, __offst)		readl((dev->mmio + (__addr + __offst)))

struct tbs_pcie_dev;
struct tbs_adapter;

struct tbs_adap_config {
        u32 ts_in;
};

struct tbs_card_config {
	char *model_name;
	char *dev_type;

	int adapters;

	int (*frontend_attach)(struct tbs_adapter *adapter, int type);

	irqreturn_t (*irq_handler)(int irq, void *dev_id);

	struct tbs_adap_config adap_config[8];
};

struct tbs_i2c {
	struct tbs_pcie_dev	*dev;
	u8			i2c_dev;
	struct i2c_adapter	i2c_adap;
	u32			base;
	int                    	ready;
	wait_queue_head_t	wq;
	struct mutex		i2c_lock;
};

struct tbs_adapter {
	struct tbs_pcie_dev	*dev;
	struct tbs_i2c		*i2c;

	struct tasklet_struct	tasklet;
	spinlock_t		adap_lock;
	int			active;

	u32			buffer_size;
	u32			buffer;
	u8			sync_offset;

	struct dvb_adapter	dvb_adapter;
	struct dvb_frontend	*fe;
	struct dvb_demux	demux;
	struct dmxdev		dmxdev;
	struct dvb_net		dvbnet;
	struct dmx_frontend	fe_hw;
	struct dmx_frontend	fe_mem;

	int			feeds;
	int			count;
	int			tsin;

	void			*adap_priv;
};

struct tbs_pcie_dev {
	struct pci_dev		*pdev;
	void __iomem		*mmio;

	dma_addr_t 		mem_addr_phys;
	__le32 *		mem_addr_virt;

	struct tbs_adapter	tbs_pcie_adap[8];
	struct tbs_i2c		i2c_bus[4];

	struct tbs_card_config	*card_config;
	u8			int_type;
};

#endif
