/*
    TurboSight TBS 6909 DVBS/S2 frontend driver
    Copyright (C) 2015 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2015 tbsdtv.com
*/

#ifndef TBS6909FE_H
#define TBS6909FE_H

#include <linux/dvb/frontend.h>
#include "tbs_pcie.h"

struct tbs6909fe_config {
	u8 tbs6909fe_address;

	int (*tbs6909_ctl1)(struct tbs_pcie_dev *dev, int a);
	int (*tbs6909_ctl2)(struct tbs_pcie_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6909FE) || \
	(defined(CONFIG_DVB_TBS6909FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs6909fe_attach(
	const struct tbs6909fe_config *config,
	struct i2c_adapter *i2c, int demod, int mode);
#else
static inline struct dvb_frontend *tbs6909fe_attach(
	const struct tbs6909fe_config *config,
	struct i2c_adapter *i2c, int demod, int mode)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6909FE_H */
