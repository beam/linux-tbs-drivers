/*
    Copyright (C) 2014 tbsdtv.com
*/

#ifndef TBS6814FE_H
#define TBS6814FE_H

#include <linux/dvb/frontend.h>
#include "tbs_pcie.h"

struct tbs6814fe_config {
	u8 tbs6814fe_address;

	int (*tbs6814_ctl1)(struct tbs_pcie_dev *dev, int a);
	int (*tbs6814_ctl2)(struct tbs_pcie_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6814FE) || \
	(defined(CONFIG_DVB_TBS6814FE_MODULE) && defined(MODULE))
extern struct dvb_frontend* tbs6814fe_attach(
	const struct tbs6814fe_config* config,
	struct i2c_adapter* i2c);
#else
static inline struct dvb_frontend* tbs6814fe_attach(
	const struct tbs6814fe_config* config,
	struct i2c_adapter* i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6814FE_H */
