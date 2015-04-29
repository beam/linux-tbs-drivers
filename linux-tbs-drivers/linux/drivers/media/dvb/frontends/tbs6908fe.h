/*
    TurboSight TBS 6908 DVBS/S2 frontend driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 tbsdtv.com
*/

#ifndef TBS6908FE_H
#define TBS6908FE_H

#include <linux/dvb/frontend.h>
#include "tbs_pcie.h"

struct tbs6908fe_config {
	u8 tbs6908fe_address;

	int (*tbs6908_ctl1)(struct tbs_pcie_dev *dev, int a);
	int (*tbs6908_ctl2)(struct tbs_pcie_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6908FE) || \
	(defined(CONFIG_DVB_TBS6908FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs6908fe_attach(
	const struct tbs6908fe_config *config,
	struct i2c_adapter *i2c, int demod);
#else
static inline struct dvb_frontend *tbs6908fe_attach(
	const struct tbs6908fe_config *config,
	struct i2c_adapter *i2c, int demod)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6908FE_H */
