/*
    Copyright (C) 2014 tbsdtv.com
*/

#ifndef TBS6704FE_H
#define TBS6704FE_H

#include <linux/dvb/frontend.h>
#include "tbs_pcie.h"

struct tbs6704fe_config {
	u8 tbs6704fe_address;

	int (*tbs6704_ctl1)(struct tbs_pcie_dev *dev, int a);
	int (*tbs6704_ctl2)(struct tbs_pcie_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6704FE) || \
	(defined(CONFIG_DVB_TBS6704FE_MODULE) && defined(MODULE))
extern struct dvb_frontend* tbs6704fe_attach(
	const struct tbs6704fe_config* config,
	struct i2c_adapter* i2c);
#else
static inline struct dvb_frontend* tbs6704fe_attach(
	const struct tbs6704fe_config* config,
	struct i2c_adapter* i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6704FE_H */
