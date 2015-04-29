/*
    TurboSight TBS PCIE DVB driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 www.tbsdtv.com
*/

#ifndef _TBS_PCIE_REG_H
#define _TBS_PCIE_REG_H

#define TBS_GPIO_BASE		0x0000

#define TBS_GPIO_DATA(i)	((3 - (i)) * 0x04)
#define TBS_GPIO_DATA_3		0x00	/* adapter 0 */
#define TBS_GPIO_DATA_2		0x04	/* adapter 1 */
#define TBS_GPIO_DATA_1		0x08	/* adapter 2 */
#define TBS_GPIO_DATA_0		0x0c	/* adapter 3 */

#define TBS_I2C_BASE_3		0x4000
#define TBS_I2C_BASE_2		0x5000
#define TBS_I2C_BASE_1		0x6000
#define TBS_I2C_BASE_0		0x7000

#define TBS_I2C_CTRL		0x00
#define TBS_I2C_DATA		0x04

#define TBS_I2C_START_BIT	(0x00000001 <<  7)
#define TBS_I2C_STOP_BIT	(0x00000001 <<  6)

#define TBS_I2C_SADDR_2BYTE	(0x00000001 <<  5)
#define TBS_I2C_SADDR_1BYTE	(0x00000001 <<  4)

#define TBS_I2C_READ_BIT	(0x00000001 <<  8)

#define TBS_INT_BASE		0xc000

#define TBS_INT_STATUS		0x00
#define TBS_INT_ENABLE		0x04
#define TBS_I2C_MASK_3		0x08
#define TBS_I2C_MASK_2		0x0c
#define TBS_I2C_MASK_1		0x10
#define TBS_I2C_MASK_0		0x14

#define TBS_DMA_MASK(i)		(0x18 + (3 - (i)) * 0x04)
#define TBS_DMA_MASK_3		0x18
#define TBS_DMA_MASK_2		0x1C
#define TBS_DMA_MASK_1		0x20
#define TBS_DMA_MASK_0		0x24

#define TBS_DMA_BASE(i)		(0x8000 + (3 - (i)) * 0x1000)
#define TBS_DMA_BASE_3		0x8000
#define TBS_DMA_BASE_2		0x9000
#define TBS_DMA_BASE_1		0xa000
#define TBS_DMA_BASE_0		0xb000

#define TBS_DMA_START		0x00
#define TBS_DMA_STATUS		0x00
#define TBS_DMA_SIZE		0x04
#define TBS_DMA_ADDR_HIGH	0x08
#define TBS_DMA_ADDR_LOW	0x0c
#define TBS_DMA_CELL_SIZE	0x10

#define TBS_PCIE_PAGE_SIZE	4194304
#define TBS_PCIE_DMA_TOTAL	770048
#define TBS_PCIE_CELL_SIZE	96256

#endif
