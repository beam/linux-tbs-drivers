/*
    TurboSight TBS 5922SE DVBS/S2 frontend driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 TurboSight.com
*/

#ifndef TBS5922SE_H
#define TBS5922SE_H

#include <linux/dvb/frontend.h>
#include "dvb-usb.h"

struct tbs5922se_config {
	u8 tbs5922se_address;

	int (*tbs5922se_ctrl)(struct usb_device *dev, u8 *a);
};

#if defined(CONFIG_DVB_TBS5922SE) || \
	(defined(CONFIG_DVB_TBS5922SE_MODULE) && defined(MODULE))
extern struct dvb_frontend *tbs5922se_attach(
	const struct tbs5922se_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *tbs5922se_attach(
	const struct tbs5922se_config *config,
	struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* TBS5922SE_H */
