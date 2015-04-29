/*
    TurboSight TBS 6205 DVBT/T2/C frontend driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 www.tbsdtv.com
*/

#ifndef TBS6205FE_H
#define TBS6205FE_H

#include <linux/dvb/frontend.h>
#include "tbs_pcie.h"

struct tbs6205fe_config {
	u8 tbs6205fe_address;

	int (*tbs6205_ctl1)(struct tbs_pcie_dev *dev, int a);
	int (*tbs6205_ctl2)(struct tbs_pcie_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6205FE) || \
	(defined(CONFIG_DVB_TBS6205FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs6205fe_attach(
	const struct tbs6205fe_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *tbs6205fe_attach(
	const struct tbs6205fe_config *config,
	struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6205FE_H */
