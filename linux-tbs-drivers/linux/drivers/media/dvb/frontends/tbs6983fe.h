/*
    TurboSight TBS 6983 DVBS/S2 frontend driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 TurboSight.com
*/

#ifndef TBS6983FE_H
#define TBS6983FE_H

#include <linux/dvb/frontend.h>
#include "saa716x_priv.h"

struct tbs6983fe_config {
	u8 tbs6983fe_address;

	int (*tbs6983_ctrl1)(struct saa716x_dev *dev, int a);
	int (*tbs6983_ctrl2)(struct saa716x_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6983FE) || \
	(defined(CONFIG_DVB_TBS6983FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs6983fe_attach(
	const struct tbs6983fe_config *config,
	struct i2c_adapter *i2c, int demod);
#else
static inline struct dvb_frontend *tbs6983fe_attach(
	const struct tbs6983fe_config *config,
	struct i2c_adapter *i2c, int demod)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6983FE_H */
