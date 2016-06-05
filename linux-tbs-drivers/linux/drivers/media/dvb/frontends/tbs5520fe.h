/*
    TurboSight TBS 5520 frontend driver

    Copyright (C) 2015 www.tbsdtv.com
*/

#ifndef TBS5520FE_H
#define TBS5520FE_H

#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"

struct tbs5520fe_config {
	u8 tbs5520fe_address;
};

#if defined(CONFIG_DVB_TBS5520FE) || \
	(defined(CONFIG_DVB_TBS5520FE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs5520fe_attach(
	const struct tbs5520fe_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *tbs5520fe_attach(
	const struct tbs5520fe_config *config,
	struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS5520FE_H */
