/*
    TurboSight (TBS) DVB Controls
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 www.tbsdtv.com
*/

#ifndef TBSDVBCTL_H
#define TBSDVBCTL_H

extern int tbsdvbctl1(struct tbs_pcie_dev *dev, int a);
extern int tbsdvbctl2(struct tbs_pcie_dev *dev, int a, int b);

#endif
