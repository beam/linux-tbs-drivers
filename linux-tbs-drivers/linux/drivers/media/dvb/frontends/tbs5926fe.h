/*
    TurboSight TBS 5926 DVBS/S2 frontend driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 TurboSight.com
*/

#ifndef TBS5926FE_H
#define TBS5926FE_H

#include <linux/dvb/frontend.h>
#include "cx231xx.h"

struct tbs5926fe_config {
	u8 tbs5926fe_address;

        int (*tbs5926_ctrl1)(struct cx231xx *dev, int a);
        u32 (*tbs5926_ctrl2)(struct cx231xx *dev, int a);
        void (*tbs5926_ctrl3)(struct cx231xx *dev, int a, u32 b);
};

#if defined(CONFIG_DVB_TBS5926FE) || \
	(defined(CONFIG_DVB_TBS5926FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs5926fe_attach(
	const struct tbs5926fe_config *config,
	struct i2c_adapter *i2c, int demod);
#else
static inline struct dvb_frontend *tbs5926fe_attach(
	const struct tbs5926fe_config *config,
	struct i2c_adapter *i2c, int demod)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS5926FE_H */
