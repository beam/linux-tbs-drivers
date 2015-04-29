/*
    TurboSight TBS 5990 DVBT/T2/C frontend driver
    Copyright (C) 2013 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2013 TurboSight.com
*/

#ifndef TBS5990FE_H
#define TBS5990FE_H

#include <linux/dvb/frontend.h>
#include "cx231xx.h"

struct tbs5990fe_config {
	u8 tbs5990fe_address;

	int (*tbs5990_ctrl1)(struct cx231xx *dev, int a);
	u32 (*tbs5990_ctrl2)(struct cx231xx *dev, int a);
	void (*tbs5990_ctrl3)(struct cx231xx *dev, int a, u32 b);
};

#if defined(CONFIG_DVB_TBS5990FE) || \
	(defined(CONFIG_DVB_TBS5990FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs5990fe_attach(
	const struct tbs5990fe_config *config,
	struct i2c_adapter *i2c, int demod);
#else
static inline struct dvb_frontend *tbs5990fe_attach(
	const struct tbs5990fe_config *config,
	struct i2c_adapter *i2c, int demod)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS5990FE_H */
