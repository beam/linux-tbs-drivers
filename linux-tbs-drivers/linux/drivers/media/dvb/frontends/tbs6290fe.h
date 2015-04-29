/*
    TurboSight TBS 6290 DVBT/T2/C frontend driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 TurboSight.com
*/

#ifndef TBS6290FE_H
#define TBS6290FE_H

#include <linux/dvb/frontend.h>
#include "saa716x_priv.h"

struct tbs6290fe_config {
	u8 tbs6290fe_address;

	int (*tbs6290_ctrl1)(struct saa716x_dev *dev, int a);
	int (*tbs6290_ctrl2)(struct saa716x_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6290FE) || \
	(defined(CONFIG_DVB_TBS6290FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs6290fe_attach(
	const struct tbs6290fe_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *tbs6290fe_attach(
	const struct tbs6290fe_config *config,
	struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS6290FE_H */
