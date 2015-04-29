/*
    TurboSight TBS 6925C DVBS/S2 controls
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 TurboSight.com
*/

#ifndef TBS6925CCTRL_H
#define TBS6925CCTRL_H

#include <linux/dvb/frontend.h>
#include "cx23885.h"

struct tbs6925cctrl_config {
	u8 tbs6925cctrl_address;

	int (*tbs6925c_ctrl1)(struct cx23885_dev *dev, int a);
	int (*tbs6925c_ctrl2)(struct cx23885_dev *dev, int a, int b);
};

#if defined(CONFIG_DVB_TBS6925CCTRL) || \
	(defined(CONFIG_DVB_TBS6925CCTRL_MODULE) && defined(MODULE))

extern struct dvb_frontend *tbs6925cctrl_attach(
	struct dvb_frontend *fe,
	struct i2c_adapter *i2c,
	const struct tbs6925cctrl_config *config);

#else
static inline struct dvb_frontend *tbs6925cctrl_attach(
	struct dvb_frontend *fe,
	struct i2c_adapter *i2c,
	const struct tbs6925cctrl_config *config)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif /* CONFIG_DVB_TBS6925CCTRL */

#endif /* TBS6925CCTRL_H */
