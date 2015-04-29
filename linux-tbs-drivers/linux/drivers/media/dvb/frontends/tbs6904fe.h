/*
    TurboSight TBS 6904 DVBS/S2 frontend driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 www.tbsdtv.com
*/

#ifndef TBS6904FE_H
#define TBS6904FE_H

#include <linux/dvb/frontend.h>
#include "tbs_pcie.h"

struct tbs6904fe_config {
	u8 tbs6904fe_address;

	int (*tbs6904_ctl1)(struct tbs_pcie_dev *dev, int a);
	int (*tbs6904_ctl2)(struct tbs_pcie_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6904FE) || \
	(defined(CONFIG_DVB_TBS6904FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs6904fe_attach(
	const struct tbs6904fe_config *config,
	struct i2c_adapter *i2c, int demod);
#else
static inline struct dvb_frontend *tbs6904fe_attach(
	const struct tbs6904fe_config *config,
	struct i2c_adapter *i2c, int demod)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6904FE_H */
