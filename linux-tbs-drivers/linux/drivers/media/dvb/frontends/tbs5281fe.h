/*
    TurboSight TBS 5281 DVBT/T2/C frontend driver
    Copyright (C) 2013 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2013 TurboSight.com
*/

#ifndef TBS5281FE_H
#define TBS5281FE_H

#include <linux/dvb/frontend.h>
#include "cx231xx.h"

struct tbs5281fe_config {
	u8 tbs5281fe_address;

	int (*tbs5281_ctrl1)(struct cx231xx *dev, int a);
	u32 (*tbs5281_ctrl2)(struct cx231xx *dev, int a);
	void (*tbs5281_ctrl3)(struct cx231xx *dev, int a, u32 b);
};

#if defined(CONFIG_DVB_TBS5281FE) || \
	(defined(CONFIG_DVB_TBS5281FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs5281fe_attach(
	const struct tbs5281fe_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *tbs5281fe_attach(
	const struct tbs5281fe_config *config,
	struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS5281FE_H */
