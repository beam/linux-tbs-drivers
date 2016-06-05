/*
    TurboSight TBS PCIE DVB driver
    Copyright (C) 2014 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    Copyright (C) 2014 www.tbsdtv.com
*/

#include "tbs_pcie.h"
#include "tbs_pcie-reg.h"

#include "tbsdvbctl.h"

#include "tbsfe.h"

#include "tbs6904fe.h"
#include "tbs6908fe.h"
#include "tbs6205fe.h"
#include "tbs6814fe.h"
#include "tbs6704fe.h"
#include "tbs6909fe.h"

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

unsigned int tbs_int_type;
module_param(tbs_int_type, int, 0644);
MODULE_PARM_DESC(tbs_int_type, "force Interrupt Handler type: 0=INT-A, 1=MSI. default INT-A mode");

unsigned int tbs_mode_single;
module_param(tbs_mode_single, int, 0644);
MODULE_PARM_DESC(tbs_mode_single, "default 0, 1: Single mode for 6908/03");

unsigned int tbs6909_mode = 1;
module_param(tbs6909_mode, int, 0644);
MODULE_PARM_DESC(tbs6909_mode, "default 0, 1: Multi-switch");

extern int tbs_ci_init(struct tbs_adapter *adap, int nr);
extern void tbs_ci_release(struct tbs_adapter *adap);

/* simple gpio */
static void tbs_pcie_gpio_write(struct tbs_pcie_dev *dev, int gpio, int pin, int set)
{
	uint32_t value;

	value = TBS_PCIE_READ(TBS_GPIO_BASE, TBS_GPIO_DATA(gpio));
	if (set)
		value |= 1 << pin;
	else
		value &= ~(1 << pin);
	TBS_PCIE_WRITE(TBS_GPIO_BASE, TBS_GPIO_DATA(gpio), value);
}

static int tbs_i2c_xfer(struct i2c_adapter *adapter,
			       struct i2c_msg msg[], int num)
{
	struct tbs_i2c *i2c = i2c_get_adapdata(adapter);
	struct tbs_pcie_dev *dev = i2c->dev;

	u8 loop = 0;
	u8 *buf_ptr = NULL;
	u16 size = 0, buf_size;

	u32 data0 = 0;
	u32 data1 = 0;
	int timeout;

	mutex_lock(&i2c->i2c_lock);

	if ((num == 2) && (msg[1].flags & I2C_M_RD) && (msg[0].len == 6)) {
		
		data0 |= (msg[0].addr << 9);
		data0 |= TBS_I2C_START_BIT;
		data0 |= (msg[0].buf[0] << 16);
		data0 |= (msg[0].buf[1] << 24);

		memcpy(&data1, &msg[0].buf[2], 4);

		data0 |= TBS_I2C_STOP_BIT;
		data0 |= 6;

		if (TBS_PCIE_READ(i2c->base, TBS_I2C_CTRL) == 1)
			i2c->ready = 0;

		/* send i2c data */
		TBS_PCIE_WRITE(i2c->base, TBS_I2C_DATA, data1);
		TBS_PCIE_WRITE(i2c->base, TBS_I2C_CTRL, data0);

		/* timeout of 1 sec */
		timeout = wait_event_timeout(i2c->wq, i2c->ready == 1, HZ);
		if (timeout <= 0) {
			printk(KERN_ERR "TBS PCIE I2C%d timeout\n", i2c->i2c_dev);
			mutex_unlock(&i2c->i2c_lock);
			return -EIO;
		}

		msleep(100);
		data0 = data1 = 0;

		data0 |= (msg[0].addr << 9) | TBS_I2C_READ_BIT;
		data0 |= TBS_I2C_START_BIT;
		data0 |= TBS_I2C_STOP_BIT;
		data0 |= 4;

		if (TBS_PCIE_READ(i2c->base, TBS_I2C_CTRL) == 1)
			i2c->ready = 0;

		TBS_PCIE_WRITE(i2c->base, TBS_I2C_CTRL, data0);

		/* timeout of 1 sec */
		timeout = wait_event_timeout(i2c->wq, i2c->ready == 1, HZ);
		if (timeout <= 0) {
			printk(KERN_ERR "TBS PCIE I2C%d timeout\n", i2c->i2c_dev);
			mutex_unlock(&i2c->i2c_lock);
			return -EIO;
		}

		data0 = TBS_PCIE_READ(i2c->base, TBS_I2C_DATA);
		memcpy(msg[1].buf, &data0, 4);

		mutex_unlock(&i2c->i2c_lock);
		return num;
	}

	if ((num == 2 &&
		 msg[1].flags & I2C_M_RD && !(msg[0].flags & I2C_M_RD)) ||
		(num == 1 && (msg[0].flags & I2C_M_RD))) {	

	size = (num == 2) ? msg[1].len : msg[0].len;

	do {
		data0 = 0;

		data0 |= (msg[0].addr << 9) | TBS_I2C_READ_BIT;

		if (loop == 0) {
			data0 |= TBS_I2C_START_BIT;
	
			/* sub-address is 1 or 2 byte */
			if ((msg[0].len == 1) && (num == 2)) {
				data0 |= TBS_I2C_SADDR_1BYTE;
				data0 |= (msg[0].buf[0] << 16);
			}

			if ((msg[0].len == 2) && (num == 2)) {
				data0 |= TBS_I2C_SADDR_2BYTE;
				data0 |= (msg[0].buf[0] << 16);
				data0 |= (msg[0].buf[1] << 24);
			}
		}

		buf_size = size > 4 ? 4 : size;

		if (size <= 4)
			data0 |= TBS_I2C_STOP_BIT;

		data0 |= buf_size;
		
		if (TBS_PCIE_READ(i2c->base, TBS_I2C_CTRL) == 1)
			i2c->ready = 0;
	
		TBS_PCIE_WRITE(i2c->base, TBS_I2C_CTRL, data0);

		/* timeout of 1 sec */
		timeout = wait_event_timeout(i2c->wq, i2c->ready == 1, HZ);
		if (timeout <= 0) {
			printk(KERN_ERR "TBS PCIE I2C%d timeout\n", i2c->i2c_dev);
			mutex_unlock(&i2c->i2c_lock);
			return -EIO;
		}

		data0 = TBS_PCIE_READ(i2c->base, TBS_I2C_DATA);
		memcpy((num == 2) ? (msg[1].buf + 4*loop) : (msg[0].buf + 4*loop), &data0, buf_size);

		loop++;

		if (size >= 4)
			size -= 4;
		else
			size = 0;
	} while (size > 0);
	
	mutex_unlock(&i2c->i2c_lock);
	return num;
	}

	if (num == 1 && !(msg[0].flags & I2C_M_RD)) {

	size = msg[0].len;
	buf_ptr = (u8 *) msg[0].buf;

	do {
		data0 = data1 = 0;

		data0 |= (msg[0].addr << 9);

		if (loop == 0)
			data0 |= TBS_I2C_START_BIT;

		buf_size = size > 6 ? 6 : size;

		if (buf_size >= 1)
			data0 |= (buf_ptr[loop * 6] << 16);

		if (buf_size >= 2)
			data0 |= (buf_ptr[loop * 6 + 1] << 24);

		if (buf_size >= 3) {
			memcpy(&data1, &buf_ptr[loop * 6 + 2], (buf_size - 2));
		}

		if (size <= 6)
			data0 |= TBS_I2C_STOP_BIT;

		data0 |= buf_size;

		if (TBS_PCIE_READ(i2c->base, TBS_I2C_CTRL) == 1)
			i2c->ready = 0;

		/* send i2c data */
		TBS_PCIE_WRITE(i2c->base, TBS_I2C_DATA, data1);
		TBS_PCIE_WRITE(i2c->base, TBS_I2C_CTRL, data0);

		/* timeout of 1 sec */
		timeout = wait_event_timeout(i2c->wq, i2c->ready == 1, HZ);
		if (timeout <= 0) {
			printk(KERN_ERR "TBS PCIE I2C%d timeout\n", i2c->i2c_dev);
			mutex_unlock(&i2c->i2c_lock);
			return -EIO;
		}

		loop++;

		if (size >= 6)
			size -= 6;
		else
			size = 0;

	} while (size > 0);

	mutex_unlock(&i2c->i2c_lock);
	return num;
	}

	printk(KERN_INFO "TBS PCIE I2C%d request not implemented\n", i2c->i2c_dev);

	return -EIO;
}

static u32 tbs_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL;
}

struct i2c_algorithm tbs_i2c_algo = {
	.master_xfer   = tbs_i2c_xfer,
	.functionality = tbs_i2c_func,
};

int tbs_pcie_mac(struct i2c_adapter *i2c_adap, u8 count, u8 *mac)
{
	u8 b[6];
	int ret;

	struct i2c_msg msg[] = {
		{ .addr = 0x50, .flags = 0,
			.buf = b, .len = 1 },
		{ .addr = 0x50, .flags = I2C_M_RD,
			.buf = b, .len = 6 }
	};

	b[0] = 0xa0 + 0x10*count;

	ret = i2c_transfer(i2c_adap, msg, 2);

	if (ret != 2) {
		printk("TBS PCIE read MAC failed\n");
		return -1;
	}

	memcpy(mac, b, 6);

	return 0;
};

static int tbs_i2c_init(struct tbs_pcie_dev *dev, u32 board)
{
	struct tbs_i2c *i2c;
	struct i2c_adapter *adap;
	int i, j, i2c_cnt, err = 0;

	/* i2c init base addr */
	switch (board) {
	case 0x6205:
		dev->i2c_bus[0].base = TBS_I2C_BASE_0;
		dev->i2c_bus[1].base = TBS_I2C_BASE_1;
		dev->i2c_bus[2].base = TBS_I2C_BASE_2;
		dev->i2c_bus[3].base = TBS_I2C_BASE_3;

#if 1
		TBS_PCIE_WRITE(dev->i2c_bus[0].base, 0x08, 39);
		TBS_PCIE_WRITE(dev->i2c_bus[1].base, 0x08, 39);
		TBS_PCIE_WRITE(dev->i2c_bus[2].base, 0x08, 39);
		TBS_PCIE_WRITE(dev->i2c_bus[3].base, 0x08, 39);
#endif
		break;
	case 0x6704:
	case 0x6814:
	case 0x6904:
		dev->i2c_bus[0].base = TBS_I2C_BASE_0;
		dev->i2c_bus[1].base = TBS_I2C_BASE_1;
		dev->i2c_bus[2].base = TBS_I2C_BASE_2;
		dev->i2c_bus[3].base = TBS_I2C_BASE_3;
		break;
	case 0x6281:
		dev->i2c_bus[0].base = TBS_I2C_BASE_2;
		dev->i2c_bus[1].base = TBS_I2C_BASE_1;
		break;
	case 0x6290:
	case 0x6910:
	case 0x6903:
	case 0x6902:
		dev->i2c_bus[0].base = TBS_I2C_BASE_2;
		dev->i2c_bus[1].base = TBS_I2C_BASE_3;
#if 1
		TBS_PCIE_WRITE(dev->i2c_bus[0].base, 0x08, 39);
		TBS_PCIE_WRITE(dev->i2c_bus[1].base, 0x08, 39);
#endif

		break;
	case 0x6901:
		dev->i2c_bus[0].base = TBS_I2C_BASE_3;
		break;
	case 0x6905:
	case 0x6908:
	case 0x6909:
		dev->i2c_bus[0].base = TBS_I2C_BASE_1;
		dev->i2c_bus[1].base = TBS_I2C_BASE_1;
		dev->i2c_bus[2].base = TBS_I2C_BASE_3;
		dev->i2c_bus[3].base = TBS_I2C_BASE_3;
#if 1
		TBS_PCIE_WRITE(dev->i2c_bus[0].base, 0x08, 39);
		TBS_PCIE_WRITE(dev->i2c_bus[1].base, 0x08, 39);
		TBS_PCIE_WRITE(dev->i2c_bus[2].base, 0x08, 39);
		TBS_PCIE_WRITE(dev->i2c_bus[3].base, 0x08, 39);
#endif
		break;
	default:
		printk("TBS PCIE Unsupported board detected\n");	
	}

	/* enable i2c interrupts */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_I2C_MASK_0, 0x00000001);
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_I2C_MASK_1, 0x00000001);
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_I2C_MASK_2, 0x00000001);
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_I2C_MASK_3, 0x00000001);

	i2c_cnt = (dev->card_config->adapters > 4) ? 4 :
			dev->card_config->adapters;
	for (i = 0; i < i2c_cnt; i++) {
		i2c = &dev->i2c_bus[i];
		i2c->dev = dev;
		i2c->i2c_dev = i;
		i2c->base = dev->i2c_bus[i].base;
		/* i2c->ready = 1; */

		init_waitqueue_head(&i2c->wq);
		mutex_init(&i2c->i2c_lock);

		adap = &i2c->i2c_adap;
		i2c_set_adapdata(adap, i2c);

		/* TODO: replace X by I2C adapter number */
		strlcpy(adap->name, "TBS PCIE I2C Adapter X", sizeof(adap->name));

		adap->algo = &tbs_i2c_algo;
		adap->algo_data = dev;
		adap->dev.parent = &dev->pdev->dev;

		err = i2c_add_adapter(adap);
		if (err)
			goto fail;
	}

	return 0;

fail:
	for (j = 0; j < i; j++) {
		i2c = &dev->i2c_bus[j];
		adap = &i2c->i2c_adap;
		i2c_del_adapter(adap);
	}
	return err;
}

static void tbs_i2c_exit(struct tbs_pcie_dev *dev)
{
	struct tbs_i2c *i2c;
	struct i2c_adapter *adap;
	int i, i2c_cnt;

	i2c_cnt = (dev->card_config->adapters > 4) ?
		4 : dev->card_config->adapters;
	for (i = 0; i < i2c_cnt; i++) {
		i2c = &dev->i2c_bus[i];
		adap = &i2c->i2c_adap;
		i2c_del_adapter(adap);
	}
}

static int tbs_pcie_dma_init(struct tbs_pcie_dev *dev)
{
	dma_addr_t dma_addr;
	u32 dma_addr_offset;

	dev->mem_addr_virt = 
		pci_alloc_consistent(dev->pdev, TBS_PCIE_PAGE_SIZE, &dma_addr);

	if (!dev->mem_addr_virt) {
		printk("TBS PCIE allocate memory failed\n");
		return -ENOMEM;
	}

	dev->mem_addr_phys = dma_addr;

	/* clear memory */
	memset(dev->mem_addr_virt, 0, TBS_PCIE_PAGE_SIZE);

	/* TODO: better use loop to init */
	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*0;
	TBS_PCIE_WRITE(TBS_DMA_BASE_0, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA_BASE_0, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA_BASE_0, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA_BASE_0, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE);

	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*1;
	TBS_PCIE_WRITE(TBS_DMA_BASE_1, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA_BASE_1, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA_BASE_1, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA_BASE_1, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE);			

	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*2;
	TBS_PCIE_WRITE(TBS_DMA_BASE_2, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA_BASE_2, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA_BASE_2, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA_BASE_2, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE); 

	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*3;
	TBS_PCIE_WRITE(TBS_DMA_BASE_3, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA_BASE_3, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA_BASE_3, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA_BASE_3, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE); 	

	/* TODO: check number of adapters */
	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*4;
	TBS_PCIE_WRITE(TBS_DMA2_BASE_0, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_0, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_0, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_0, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE);

	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*5;
	TBS_PCIE_WRITE(TBS_DMA2_BASE_1, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_1, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_1, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_1, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE);

	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*6;
	TBS_PCIE_WRITE(TBS_DMA2_BASE_2, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_2, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_2, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_2, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE);

	dma_addr_offset = dev->mem_addr_phys + (TBS_PCIE_DMA_TOTAL + 256)*7;
	TBS_PCIE_WRITE(TBS_DMA2_BASE_3, TBS_DMA_ADDR_HIGH, 0);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_3, TBS_DMA_ADDR_LOW, dma_addr_offset);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_3, TBS_DMA_SIZE, TBS_PCIE_DMA_TOTAL);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_3, TBS_DMA_CELL_SIZE, TBS_PCIE_CELL_SIZE);

	return 0;
}

static void tbs_pcie_dma_free(struct tbs_pcie_dev *dev)
{
	if (dev->mem_addr_virt != NULL) {
		pci_free_consistent(dev->pdev, TBS_PCIE_PAGE_SIZE,
				dev->mem_addr_virt, dev->mem_addr_phys);

		dev->mem_addr_virt = NULL;
	}
}

static void tbs_pcie_dma_start(struct tbs_adapter *adapter)
{
	struct tbs_pcie_dev *dev = adapter->dev;

	spin_lock_irq(&adapter->adap_lock);

	adapter->buffer = 0;
	adapter->sync_offset = 0;

	if (adapter->tsin >= 4)
	{
	TBS_PCIE_READ(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_DMA2_MASK(adapter->tsin), 0x00000001);
	TBS_PCIE_WRITE(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_START, 0x00000001);
	} else {
	TBS_PCIE_READ(TBS_DMA_BASE(adapter->tsin), TBS_DMA_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_DMA_MASK(adapter->tsin), 0x00000001); 
	TBS_PCIE_WRITE(TBS_DMA_BASE(adapter->tsin), TBS_DMA_START, 0x00000001);
	}

	adapter->active = 1;

	spin_unlock_irq(&adapter->adap_lock);
}

static void tbs_pcie_dma_stop(struct tbs_adapter *adapter)
{
	struct tbs_pcie_dev *dev = adapter->dev;

	spin_lock_irq(&adapter->adap_lock);

	if (adapter->tsin >= 4)
	{
	TBS_PCIE_READ(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_DMA2_MASK(adapter->tsin), 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_START, 0x00000000);
	} else {
	TBS_PCIE_READ(TBS_DMA_BASE(adapter->tsin), TBS_DMA_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_DMA_MASK(adapter->tsin), 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA_BASE(adapter->tsin), TBS_DMA_START, 0x00000000);
	}

	adapter->active = 0;

	spin_unlock_irq(&adapter->adap_lock);
}

static void adapter_tasklet(unsigned long adap)
{
	struct tbs_adapter *adapter = (struct tbs_adapter *) adap;
	struct tbs_pcie_dev *dev = adapter->dev;
	u8* data;
	u32 active_buffer;
	int i = 0;

	if (adapter->tsin >= 4)
	adapter->buffer = TBS_PCIE_READ(TBS_DMA2_BASE(adapter->tsin), TBS_DMA_STATUS);
	else
	adapter->buffer = TBS_PCIE_READ(TBS_DMA_BASE(adapter->tsin), TBS_DMA_STATUS);
	adapter->buffer += 6;

	active_buffer = adapter->buffer;

	spin_lock(&adapter->adap_lock);

	data = (u8*)dev->mem_addr_virt + (adapter->tsin)*(TBS_PCIE_DMA_TOTAL + 256) +
						TBS_PCIE_CELL_SIZE*(active_buffer & 0x07);

	if ((adapter->sync_offset == 0) || (*(data+adapter->sync_offset) != 0x47)) {
		for(i = 0; i < 256; i++)
			if ((*(data + i) == 0x47) && (*(data + i + 188) == 0x47) &&
		 					(*(data + i + 2*188) == 0x47)) {
				adapter->sync_offset = i;
				break;
			}
	}

	data += adapter->sync_offset;

	/* copy from cell0 sync byte offset to cell7 */
	if ((active_buffer & 0x07) == 0x07)
	memcpy((u8*)dev->mem_addr_virt + (adapter->tsin)*(TBS_PCIE_DMA_TOTAL + 256) + TBS_PCIE_DMA_TOTAL,
	(u8*)dev->mem_addr_virt + (adapter->tsin)*(TBS_PCIE_DMA_TOTAL + 256), adapter->sync_offset);
		
	if ((dev->mem_addr_virt) && (adapter->active))
		dvb_dmx_swfilter_packets(&adapter->demux, data, adapter->buffer_size / 188); 

	spin_unlock(&adapter->adap_lock);

}

static irqreturn_t tbs6904_pcie_irq(int irq, void *dev_id)
{
	struct tbs_pcie_dev *dev = (struct tbs_pcie_dev *) dev_id;
	struct tbs_i2c *i2c;
	u32 stat;

	stat = TBS_PCIE_READ(TBS_INT_BASE, TBS_INT_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_STATUS, stat);

	if (!(stat & 0x000000ff))
	{
		TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001); 
		return IRQ_HANDLED;
	}

	if (stat & 0x00000080)
		tasklet_schedule(&dev->tbs_pcie_adap[0].tasklet);

	if (stat & 0x00000040)
		tasklet_schedule(&dev->tbs_pcie_adap[1].tasklet);

	if (stat & 0x00000020)
		tasklet_schedule(&dev->tbs_pcie_adap[2].tasklet);

	if (stat & 0x00000010)
		tasklet_schedule(&dev->tbs_pcie_adap[3].tasklet);

 	if (stat & 0x00000008) {
		i2c = &dev->i2c_bus[0];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
 	
	if (stat & 0x00000004) {
		i2c = &dev->i2c_bus[1];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
	
	if (stat & 0x00000002) {
		i2c = &dev->i2c_bus[2];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
	
	if (stat & 0x00000001) {
		i2c = &dev->i2c_bus[3];
		i2c->ready = 1;
		wake_up(&i2c->wq);	
	}

	/* enable interrupt */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);

	return IRQ_HANDLED;
}

static irqreturn_t tbs6901_pcie_irq(int irq, void *dev_id)
{
	struct tbs_pcie_dev *dev = (struct tbs_pcie_dev *) dev_id;
	struct tbs_i2c *i2c;
	u32 stat;

	stat = TBS_PCIE_READ(TBS_INT_BASE, TBS_INT_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_STATUS, stat);

	if (!(stat & 0x000000ff))
	{
		TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001); 
		return IRQ_HANDLED;
	}

	if (stat & 0x00000010)
		tasklet_schedule(&dev->tbs_pcie_adap[0].tasklet);

	if (stat & 0x00000001) {
		i2c = &dev->i2c_bus[0];
		i2c->ready = 1;
		wake_up(&i2c->wq);	
	}

	/* enable interrupt */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);

	return IRQ_HANDLED;
}

static irqreturn_t tbs6902_pcie_irq(int irq, void *dev_id)
{
	struct tbs_pcie_dev *dev = (struct tbs_pcie_dev *) dev_id;
	struct tbs_i2c *i2c;
	u32 stat;

	stat = TBS_PCIE_READ(TBS_INT_BASE, TBS_INT_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_STATUS, stat);

	if (!(stat & 0x000000ff))
	{
		TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001); 
		return IRQ_HANDLED;
	}

	if (stat & 0x00000020)
		tasklet_schedule(&dev->tbs_pcie_adap[0].tasklet);

	if (stat & 0x00000010)
		tasklet_schedule(&dev->tbs_pcie_adap[1].tasklet);

	if (stat & 0x00000002) {
		i2c = &dev->i2c_bus[0];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
	
	if (stat & 0x00000001) {
		i2c = &dev->i2c_bus[1];
		i2c->ready = 1;
		wake_up(&i2c->wq);	
	}

	/* enable interrupt */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);

	return IRQ_HANDLED;
}

static irqreturn_t tbs_dual_pcie_irq(int irq, void *dev_id)
{
	struct tbs_pcie_dev *dev = (struct tbs_pcie_dev *) dev_id;
	struct tbs_i2c *i2c;
	u32 stat;

	stat = TBS_PCIE_READ(TBS_INT_BASE, TBS_INT_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_STATUS, stat);

	if (!(stat & 0x000000ff))
	{
		TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001); 
		return IRQ_HANDLED;
	}

	if (stat & 0x00000020)
		tasklet_schedule(&dev->tbs_pcie_adap[1].tasklet);

	if (stat & 0x00000010)
		tasklet_schedule(&dev->tbs_pcie_adap[0].tasklet);

	if (stat & 0x00000002) {
		i2c = &dev->i2c_bus[0];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
	
	if (stat & 0x00000001) {
		i2c = &dev->i2c_bus[1];
		i2c->ready = 1;
		wake_up(&i2c->wq);	
	}

	/* enable interrupt */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);

	return IRQ_HANDLED;
}

static irqreturn_t tbs_tbs6281se_pcie_irq(int irq, void *dev_id)
{
	struct tbs_pcie_dev *dev = (struct tbs_pcie_dev *) dev_id;
	struct tbs_i2c *i2c;
	u32 stat;

	stat = TBS_PCIE_READ(TBS_INT_BASE, TBS_INT_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_STATUS, stat);

	if (!(stat & 0x000000ff))
	{
		TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001); 
		return IRQ_HANDLED;
	}

	if (stat & 0x00000020)
		tasklet_schedule(&dev->tbs_pcie_adap[1].tasklet);

	if (stat & 0x00000040)
		tasklet_schedule(&dev->tbs_pcie_adap[0].tasklet);

	if (stat & 0x00000002) {
		i2c = &dev->i2c_bus[0];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
	
	if (stat & 0x00000004) {
		i2c = &dev->i2c_bus[1];
		i2c->ready = 1;
		wake_up(&i2c->wq);	
	}

	/* enable interrupt */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);

	return IRQ_HANDLED;
}

static irqreturn_t tbs6908_pcie_irq(int irq, void *dev_id)
{
	struct tbs_pcie_dev *dev = (struct tbs_pcie_dev *) dev_id;
	struct tbs_i2c *i2c;
	u32 stat;

	stat = TBS_PCIE_READ(TBS_INT_BASE, TBS_INT_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_STATUS, stat);

	if (!(stat & 0x000000ff))
	{
		TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001); 
		return IRQ_HANDLED;
	}

	if (stat & 0x00000080)
		tasklet_schedule(&dev->tbs_pcie_adap[1].tasklet);

	if (stat & 0x00000040)
		tasklet_schedule(&dev->tbs_pcie_adap[0].tasklet);

	if (stat & 0x00000020)
		tasklet_schedule(&dev->tbs_pcie_adap[3].tasklet);

	if (stat & 0x00000010)
		tasklet_schedule(&dev->tbs_pcie_adap[2].tasklet);

 	if (stat & 0x00000008) {
		i2c = &dev->i2c_bus[0];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
 	
	if (stat & 0x00000004) {
		i2c = &dev->i2c_bus[1];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);
		}
		i2c = &dev->i2c_bus[0];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);
		}
	}
	
	if(stat & 0x00000002) {
		i2c = &dev->i2c_bus[2];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
	
	if(stat & 0x00000001) {
		i2c = &dev->i2c_bus[3];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);	
		}
		i2c = &dev->i2c_bus[2];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);
		}
	}

	/* enable interrupt */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);

	return IRQ_HANDLED;
}

static irqreturn_t tbs6909_pcie_irq(int irq, void *dev_id)
{
	struct tbs_pcie_dev *dev = (struct tbs_pcie_dev *) dev_id;
	struct tbs_i2c *i2c;
	u32 stat;

	stat = TBS_PCIE_READ(TBS_INT_BASE, TBS_INT_STATUS);

	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_STATUS, stat);

	if (!(stat & 0x00000fff))
	{
		TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001); 
		return IRQ_HANDLED;
	}

	if (stat & 0x00000080)
		tasklet_schedule(&dev->tbs_pcie_adap[0].tasklet);

	if (stat & 0x00000040)
		tasklet_schedule(&dev->tbs_pcie_adap[2].tasklet);

	if (stat & 0x00000020)
		tasklet_schedule(&dev->tbs_pcie_adap[4].tasklet);

	if (stat & 0x00000010)
		tasklet_schedule(&dev->tbs_pcie_adap[6].tasklet);

	if (stat & 0x00000800)
		tasklet_schedule(&dev->tbs_pcie_adap[1].tasklet);

	if (stat & 0x00000400)
		tasklet_schedule(&dev->tbs_pcie_adap[3].tasklet);

	if (stat & 0x00000200)
		tasklet_schedule(&dev->tbs_pcie_adap[5].tasklet);

	if (stat & 0x00000100)
		tasklet_schedule(&dev->tbs_pcie_adap[7].tasklet);

 	if (stat & 0x00000008) {
		i2c = &dev->i2c_bus[0];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
 	
	if (stat & 0x00000004) {
		i2c = &dev->i2c_bus[1];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);
		}
		i2c = &dev->i2c_bus[0];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);
		}
	}
	
	if(stat & 0x00000002) {
		i2c = &dev->i2c_bus[2];
		i2c->ready = 1;
		wake_up(&i2c->wq);
	}
	
	if(stat & 0x00000001) {
		i2c = &dev->i2c_bus[3];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);	
		}
		i2c = &dev->i2c_bus[2];
		if (i2c->ready == 0) {
			i2c->ready = 1;
			wake_up(&i2c->wq);
		}
	}

	/* enable interrupt */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000001);

	return IRQ_HANDLED;
}

static int start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct tbs_adapter *adapter = dvbdmx->priv;

	if (!adapter->feeds)
		tbs_pcie_dma_start(adapter);

	return ++adapter->feeds;
}

static int stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct tbs_adapter *adapter = dvbdmx->priv;

	if (--adapter->feeds)
		return adapter->feeds;

	tbs_pcie_dma_stop(adapter);

	return 0;
}

static void tbs_dvb_exit(struct tbs_adapter *adapter)
{
	struct dvb_adapter *adap = &adapter->dvb_adapter;
	struct dvb_demux *dvbdemux = &adapter->demux;

	if (adapter->fe) {
		dvb_unregister_frontend(adapter->fe);
		dvb_frontend_detach(adapter->fe);
		adapter->fe = NULL;
	}

	dvb_net_release(&adapter->dvbnet);

	dvbdemux->dmx.close(&dvbdemux->dmx);

	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
					&adapter->fe_mem);
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, 
					&adapter->fe_hw);

	dvb_dmxdev_release(&adapter->dmxdev);

	dvb_dmx_release(&adapter->demux);

	dvb_unregister_adapter(adap);
}

static int tbs_dvb_init(struct tbs_adapter *adapter)
{
	struct dvb_adapter *adap = &adapter->dvb_adapter;
	struct dvb_demux *dvbdemux = &adapter->demux;
	struct dmxdev *dmxdev;
	struct dmx_frontend *fe_hw;
	struct dmx_frontend *fe_mem;
	int ret;

	ret = dvb_register_adapter(adap, "TBS PCIE DVB Adapter",
					THIS_MODULE,
					&adapter->dev->pdev->dev,
					adapter_nr);
	if (ret < 0) {
		printk(KERN_ERR "error registering adapter\n");
		return -ENODEV;
	}

	dvbdemux->priv = adapter;
	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	dvbdemux->write_to_decoder = NULL;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING);

	ret = dvb_dmx_init(dvbdemux);
	if (ret < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err0;
	}

	dmxdev = &adapter->dmxdev;

	dmxdev->filternum = 256;
	dmxdev->demux = &dvbdemux->dmx;
	dmxdev->capabilities = 0;

	ret = dvb_dmxdev_init(dmxdev, adap);
	if (ret < 0) {
		printk("dvb_dmxdev_init failed, ERROR=%d", ret);
		goto err1;
	}

	fe_hw = &adapter->fe_hw;
	fe_mem = &adapter->fe_mem;

	fe_hw->source = DMX_FRONTEND_0;

	ret = dvbdemux->dmx.add_frontend(&dvbdemux->dmx, fe_hw);
	if ( ret < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err2;
	}

	fe_mem->source = DMX_MEMORY_FE;

	ret = dvbdemux->dmx.add_frontend(&dvbdemux->dmx, fe_mem);
	if (ret  < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err3;
	}

	ret = dvbdemux->dmx.connect_frontend(&dvbdemux->dmx, fe_hw);
	if (ret < 0) {
		printk("dvb_dmx_init failed, ERROR=%d", ret);
		goto err4;
	}

	ret = dvb_net_init(adap, &adapter->dvbnet, adapter->dmxdev.demux);
	if (ret < 0) {
		printk("dvb_net_init failed, ERROR=%d", ret);
		goto err5;
	}

	/* adapter->fe = NULL; */

	if (adapter->dev->card_config->frontend_attach) {
		ret = adapter->dev->card_config->frontend_attach(adapter, 0);
		if (ret < 0)
			printk("TBS frontend attach failed\n");

		if (adapter->fe != NULL) {
			ret = dvb_register_frontend(adap, adapter->fe);
			if (ret < 0) {
				printk("TBS PCIE register frontend failed\n");
				goto err6;
			}
		}
	}

	return 0;

err6:
	dvb_frontend_detach(adapter->fe);
err5:
err4:
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, fe_mem);
err3:
	dvbdemux->dmx.remove_frontend(&dvbdemux->dmx, fe_hw);
err2:
	dvb_dmxdev_release(dmxdev);
err1:
	dvb_dmx_release(dvbdemux);
err0:
	dvb_unregister_adapter(adap);

	return ret;
}

static int tbs_adapters_attach(struct tbs_pcie_dev *dev)
{
	struct tbs_adapter *adapter;
	int i, ret = 0;

	for (i = 0; i < dev->card_config->adapters; i++) {
		adapter = &dev->tbs_pcie_adap[i];

		ret = tbs_dvb_init(adapter);

		if (ret < 0)
			printk(KERN_ERR "TBS PCIE Adapter%d \
				attach failed\n", adapter->count);
	}

	return ret;
}

static void tbs_adapters_detach(struct tbs_pcie_dev *dev)
{
	struct tbs_adapter *adapter;
	int i;

	for (i = 0; i < dev->card_config->adapters; i++) {
		adapter = &dev->tbs_pcie_adap[i];
		tbs_dvb_exit(adapter);
	}
}

static void tbs_adapters_init(struct tbs_pcie_dev *dev)
{
	struct tbs_adapter *tbs_adap;
	int i;

	/* disable all interrupts */
	TBS_PCIE_WRITE(TBS_INT_BASE, TBS_INT_ENABLE, 0x00000000); 

	/* disable dma */
	TBS_PCIE_WRITE(TBS_DMA_BASE_0, TBS_DMA_START, 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA_BASE_1, TBS_DMA_START, 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA_BASE_2, TBS_DMA_START, 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA_BASE_3, TBS_DMA_START, 0x00000000);
	/* TODO: check adapter numbers */
	TBS_PCIE_WRITE(TBS_DMA2_BASE_0, TBS_DMA_START, 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_1, TBS_DMA_START, 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_2, TBS_DMA_START, 0x00000000);
	TBS_PCIE_WRITE(TBS_DMA2_BASE_3, TBS_DMA_START, 0x00000000);

	for (i = 0; i < dev->card_config->adapters; i++) {
		tbs_adap = &dev->tbs_pcie_adap[i];
		tbs_adap->dev = dev;
		tbs_adap->count = i;
		tbs_adap->tsin = dev->card_config->adap_config[i].ts_in;
		tbs_adap->i2c = &dev->i2c_bus[i];

		tbs_adap->buffer_size = TBS_PCIE_CELL_SIZE;

		tasklet_init(&tbs_adap->tasklet, adapter_tasklet, (unsigned long) tbs_adap);
		spin_lock_init(&tbs_adap->adap_lock);
	}
}

static void tbs_adapters_release(struct tbs_pcie_dev *dev)
{
	struct tbs_adapter *tbs_adap;
	int i;

	for (i = 0; i < dev->card_config->adapters; i++) {
		tbs_adap = &dev->tbs_pcie_adap[i];
		tbs_adap->dev = dev;
		tasklet_kill(&tbs_adap->tasklet);
	}
}

static struct tbs6904fe_config tbs6904_fe_config0 = {
	.tbs6904fe_address = 0x60,

	.tbs6904_ctl1 = tbsdvbctl1,
	.tbs6904_ctl2 = tbsdvbctl2,
};

static struct tbs6904fe_config tbs6904_fe_config1 = {
	.tbs6904fe_address = 0x68,

	.tbs6904_ctl1 = tbsdvbctl1,
	.tbs6904_ctl2 = tbsdvbctl2,
};

static int tbs6904fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap2 = &dev->tbs_pcie_adap[2];
	struct i2c_adapter *i2c2 = &adap2->i2c->i2c_adap;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct tbs_adapter *adap3 = &dev->tbs_pcie_adap[3];

	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {
		tbs_pcie_gpio_write(dev, adapter->count ? 1 : 0, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 1 : 0, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6904fe_attach,
			adapter->count ? &tbs6904_fe_config1 : &tbs6904_fe_config0,
								i2c, adapter->count);

		if (!adapter->fe) 
			goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);
	}

	if (adapter->count == 2 || adapter->count == 3) {
		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 2, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 2, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6904fe_attach,
			(adapter->count-2) ? &tbs6904_fe_config1 : &tbs6904_fe_config0,
								i2c, adapter->count);
		if (!adapter->fe)
			 goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);
	}

	if (adapter->count == 3) {
		tbs_pcie_mac(i2c2, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 3;
		printk(KERN_INFO "TurboSight TBS6904 MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap3->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap2->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
} 

static int tbs6910fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	//struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;

	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct i2c_adapter *i2c1 = &adap1->i2c->i2c_adap;

	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {
		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 3, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 3, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6904fe_attach,
			adapter->count ? &tbs6904_fe_config0 : &tbs6904_fe_config1,
			adapter->count ? i2c0 : i2c1, 3 - adapter->count);
		if (!adapter->fe) 
			goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);

		tbs_ci_init(adapter, 1 - adapter->count);
	}

	if (adapter->count == 1) {
		tbs_pcie_mac(i2c0, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 1;
		printk(KERN_INFO "TurboSight TBS6910 MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

static int tbs6901fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	u8 mac[6];

	if (adapter->count == 0) {
		tbs_pcie_gpio_write(dev, 3, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, 3, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6904fe_attach,
						&tbs6904_fe_config1, i2c, 3);
		if (!adapter->fe) 
			goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);

		tbs_pcie_mac(i2c, adapter->count, mac);
		memcpy(adapter->dvb_adapter.proposed_mac, mac, 6);
		printk(KERN_INFO "TurboSight TBS6901 MAC Address: %pM\n",
			adapter->dvb_adapter.proposed_mac);
	}

	return 0;
exit:
	return -ENODEV;
}

static int tbs6902fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	//struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;

	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct i2c_adapter *i2c1 = &adap1->i2c->i2c_adap;

	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {
		tbs_pcie_gpio_write(dev, adapter->count ? 3 : 2, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 3 : 2, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6904fe_attach,
			adapter->count ? &tbs6904_fe_config1 : &tbs6904_fe_config0,
						adapter->count ? i2c1 : i2c0, 2 + adapter->count);
		if (!adapter->fe) 
			goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);
	}

	if (adapter->count == 1) {
		tbs_pcie_mac(i2c1, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 1;
		printk(KERN_INFO "TurboSight TBS6902 MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

static struct tbs6908fe_config tbs6908_fe_config = {
	.tbs6908fe_address = 0x68,

	.tbs6908_ctl1 = tbsdvbctl1,
	.tbs6908_ctl2 = tbsdvbctl2,
};

static int tbs6908fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	//struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;
	
	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;

	struct tbs_adapter *adap2 = &dev->tbs_pcie_adap[2];
	struct i2c_adapter *i2c2 = &adap2->i2c->i2c_adap;

	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct tbs_adapter *adap3 = &dev->tbs_pcie_adap[3];

	u8 mac[6];

	if (adapter->count == 0) {
		tbs_pcie_gpio_write(dev, 1, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, 1, 0, 1);
		msleep(100);
	}

	if (adapter->count == 2) {
		tbs_pcie_gpio_write(dev, 3, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, 3, 0, 1);
		msleep(100);
	}

	if (adapter->count == 0 || adapter->count == 1) {

		adapter->fe = dvb_attach(tbs6908fe_attach, &tbs6908_fe_config,
								i2c0, adapter->count, tbs_mode_single);

		if (!adapter->fe)
			goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);
	}

	if (adapter->count == 2 || adapter->count == 3) {


		adapter->fe = dvb_attach(tbs6908fe_attach, &tbs6908_fe_config,
								i2c2, adapter->count, tbs_mode_single);
		if (!adapter->fe)
			 goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);
	}

	if (adapter->count == 3) {
		tbs_pcie_mac(i2c0, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 3;
		printk(KERN_INFO "TurboSight TBS6905/8 MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap3->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap2->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

static int tbs6903fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	//struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;

	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct i2c_adapter *i2c1 = &adap1->i2c->i2c_adap;

	u8 mac[6];

	if (adapter->count == 0) {
                tbs_pcie_gpio_write(dev, 3, 0, 0);
                msleep(50);
                tbs_pcie_gpio_write(dev, 3, 0, 1);
                msleep(100);
	}

	if (adapter->count == 0 || adapter->count == 1) {

		adapter->fe = dvb_attach(tbs6908fe_attach, &tbs6908_fe_config,
								i2c1, 2 + adapter->count, tbs_mode_single);

		if (!adapter->fe) 
			goto exit;

		dvb_attach(tbsfe_attach, adapter->fe);
	}

	if (adapter->count == 1) {
		tbs_pcie_mac(i2c0, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 1;
		printk(KERN_INFO "TurboSight TBS6903 MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

static struct tbs6205fe_config tbs6205_fe_config = {
	.tbs6205fe_address = 0x64,

	.tbs6205_ctl1 = tbsdvbctl1,
	.tbs6205_ctl2 = tbsdvbctl2,
};

static int tbs6205fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap2 = &dev->tbs_pcie_adap[2];
	struct i2c_adapter *i2c2 = &adap2->i2c->i2c_adap;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct tbs_adapter *adap3 = &dev->tbs_pcie_adap[3];

	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {

		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 0, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 0, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6205fe_attach, &tbs6205_fe_config, i2c);

		if (!adapter->fe) 
			goto exit;
	}

	if (adapter->count == 2 || adapter->count == 3) {

		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 1, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 1, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6205fe_attach, &tbs6205_fe_config, i2c);

		if (!adapter->fe)
			goto exit;
	}

	if (adapter->count == 3) {
		tbs_pcie_mac(i2c2, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 3;
		printk(KERN_INFO "TurboSight TBS6205 MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap3->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap2->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

static int tbs6290se_frontend_attach(struct tbs_adapter *adapter, int type)
{
	//struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

#if 1
	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;
#endif

	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct i2c_adapter *i2c1 = &adap1->i2c->i2c_adap;

	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {

		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 0, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 0, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6205fe_attach, &tbs6205_fe_config, adapter->count ? i2c0 : i2c1);

		if (!adapter->fe)
			goto exit;

		tbs_ci_init(adapter, 1 - adapter->count);
	}

	if (adapter->count == 1) {
		tbs_pcie_mac(i2c1, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 1;
		printk(KERN_INFO "TurboSight TBS6290SE MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

static int tbs6281se_frontend_attach(struct tbs_adapter *adapter, int type)
{
	//struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;

	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct i2c_adapter *i2c1 = &adap1->i2c->i2c_adap;

	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {

		tbs_pcie_gpio_write(dev, adapter->count ? 1 : 2, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 1 : 2, 0, 1);
		msleep(100);

		adapter->fe = dvb_attach(tbs6205fe_attach, &tbs6205_fe_config, adapter->count ? i2c0 : i2c1);

		if (!adapter->fe)
			goto exit;

	}

	if (adapter->count == 1) {
		tbs_pcie_mac(i2c0, 0, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 1;
		printk(KERN_INFO "TurboSight TBS6281SE MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

static struct tbs6814fe_config tbs6814_fe_config0 = {
	.tbs6814fe_address = 0x44,

	.tbs6814_ctl1 = tbsdvbctl1,
	.tbs6814_ctl2 = tbsdvbctl2,
};

static struct tbs6814fe_config tbs6814_fe_config1 = {
	.tbs6814fe_address = 0x43,

	.tbs6814_ctl1 = tbsdvbctl1,
	.tbs6814_ctl2 = tbsdvbctl2,
};

static int tbs6814fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;


	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {
#if 0
		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 0, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 2 : 0, 0, 1);
		msleep(100);
#endif
		adapter->fe = dvb_attach(tbs6814fe_attach, 
				adapter->count ? &tbs6814_fe_config1 : &tbs6814_fe_config0, i2c);

		if (!adapter->fe)
			goto exit;

		tbs_pcie_mac(i2c0, adapter->count, mac);
		memcpy(adapter->dvb_adapter.proposed_mac, mac, 6);
		printk(KERN_INFO "TurboSight TBS6814 ISDB-T card adapter%d MAC=%pM\n",
			adapter->count, adapter->dvb_adapter.proposed_mac);
	}

	if (adapter->count == 2 || adapter->count == 3) {
#if 0
		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 1, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 1, 0, 1);
		msleep(100);
#endif
		adapter->fe = dvb_attach(tbs6814fe_attach,
				(adapter->count - 2) ? &tbs6814_fe_config1 : &tbs6814_fe_config0, i2c);

		if (!adapter->fe) 
			goto exit;

		tbs_pcie_mac(i2c0, adapter->count, mac);
		memcpy(adapter->dvb_adapter.proposed_mac, mac, 6);
		printk(KERN_INFO "TurboSight TBS6814 ISDB-T card adapter%d MAC=%pM\n",
			adapter->count, adapter->dvb_adapter.proposed_mac);
	}

	return 0;
exit:
	return -ENODEV;
}

static struct tbs6704fe_config tbs6704_fe_config = {
	.tbs6704fe_address = 0x60,

	.tbs6704_ctl1 = tbsdvbctl1,
	.tbs6704_ctl2 = tbsdvbctl2,
};

static int tbs6704fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct i2c_adapter *i2c0 = &adap0->i2c->i2c_adap;

	u8 mac[6];

	if (adapter->count == 0 || adapter->count == 1) {
#if 1
		tbs_pcie_gpio_write(dev, adapter->count ? 1 : 0, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, adapter->count ? 1 : 0, 0, 1);
		msleep(100);
#endif
		adapter->fe = dvb_attach(tbs6704fe_attach, &tbs6704_fe_config, i2c);

		if (!adapter->fe) 
			goto exit;

		tbs_pcie_mac(i2c0, adapter->count, mac);
		memcpy(adapter->dvb_adapter.proposed_mac, mac, 6);
		printk(KERN_INFO "TurboSight TBS6704 ATSC card adapter%d MAC=%pM\n",
			adapter->count, adapter->dvb_adapter.proposed_mac);
	}

	if (adapter->count == 2 || adapter->count == 3) {
#if 1
		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 2, 0, 0);
		msleep(50);
		tbs_pcie_gpio_write(dev, (adapter->count-2) ? 3 : 2, 0, 1);
		msleep(100);
#endif
		adapter->fe = dvb_attach(tbs6704fe_attach, &tbs6704_fe_config, i2c);

		if (!adapter->fe)
			goto exit;

		tbs_pcie_mac(i2c0, adapter->count, mac);
		memcpy(adapter->dvb_adapter.proposed_mac, mac, 6);
		printk(KERN_INFO "TurboSight TBS6704 ATSC card adapter%d MAC=%pM\n",
			adapter->count, adapter->dvb_adapter.proposed_mac);
	}

	return 0;
exit:
	return -ENODEV;
}

static struct tbs6909fe_config tbs6909_fe_config = {
	.tbs6909fe_address = 0x60,

	.tbs6909_ctl1 = tbsdvbctl1,
	.tbs6909_ctl2 = tbsdvbctl2,
};

static int tbs6909fe_frontend_attach(struct tbs_adapter *adapter, int type)
{
	//struct i2c_adapter *i2c = &adapter->i2c->i2c_adap;
	struct tbs_pcie_dev *dev = adapter->dev;
	
	struct tbs_adapter *adap2 = &dev->tbs_pcie_adap[2];
	struct i2c_adapter *i2c2 = &adap2->i2c->i2c_adap;

	struct tbs_adapter *adap0 = &dev->tbs_pcie_adap[0];
	struct tbs_adapter *adap1 = &dev->tbs_pcie_adap[1];
	struct tbs_adapter *adap3 = &dev->tbs_pcie_adap[3];
	struct tbs_adapter *adap4 = &dev->tbs_pcie_adap[4];
	struct tbs_adapter *adap5 = &dev->tbs_pcie_adap[5];
	struct tbs_adapter *adap6 = &dev->tbs_pcie_adap[6];
	struct tbs_adapter *adap7 = &dev->tbs_pcie_adap[7];

	u8 mac[6];

	adapter->fe = dvb_attach(tbs6909fe_attach, &tbs6909_fe_config,
					i2c2, adapter->count, tbs6909_mode);

	if (!adapter->fe) 
		goto exit;

	dvb_attach(tbsfe_attach, adapter->fe);

	if (adapter->count == 7) {
		tbs_pcie_mac(i2c2, adapter->count, mac);
		memcpy(adap0->dvb_adapter.proposed_mac, mac, 6);
		mac[5] += 7;
		printk(KERN_INFO "TurboSight TBS6909 MAC Addresses: %pM - %pM\n",
			adap0->dvb_adapter.proposed_mac, mac);
		memcpy(adap7->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap6->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap5->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap4->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap3->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap2->dvb_adapter.proposed_mac, mac, 6);
		mac[5]--;
		memcpy(adap1->dvb_adapter.proposed_mac, mac, 6);
	}

	return 0;
exit:
	return -ENODEV;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
static void __devexit tbs_remove(struct pci_dev *pdev)
#else
static void tbs_remove(struct pci_dev *pdev)
#endif
{
	struct tbs_pcie_dev *dev =
		(struct tbs_pcie_dev*) pci_get_drvdata(pdev);
	struct tbs_adapter *tbs_adap;
	int i;

	for (i = 0; i < dev->card_config->adapters; i++) {
		tbs_adap = &dev->tbs_pcie_adap[i];
		if (tbs_adap->adap_priv)
			tbs_ci_release(tbs_adap);
	}

	tbs_adapters_detach(dev);
	tbs_i2c_exit(dev);

	/* disable interrupts */
	free_irq(dev->pdev->irq, dev);
	if (dev->int_type)
		pci_disable_msi(dev->pdev);

	tbs_adapters_release(dev);
	tbs_pcie_dma_free(dev);

	if (dev->mmio)
		iounmap(dev->mmio);

	kfree(dev);

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
static int __devinit tbs_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
#else
static int tbs_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
#endif
{
	struct tbs_pcie_dev *dev;
	int err = 0, ret = -ENODEV;
	u32 revision;

	dev  = kzalloc(sizeof (struct tbs_pcie_dev), GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_ERR "pcie_tbs_probe ERROR: out of memory\n");
		ret = -ENOMEM;
		goto fail0;
	}

	dev->int_type = tbs_int_type;
	dev->pdev = pdev;
	dev->card_config = (struct tbs_card_config *) pci_id->driver_data;

	err = pci_enable_device(pdev);
	if (err != 0) {
		ret = -ENODEV;
		printk(KERN_ERR "pcie_tbs_probe ERROR: PCI enable failed (%i)\n", err);
		goto fail1;
	}

	dev->mmio = ioremap(pci_resource_start(dev->pdev, 0),
			pci_resource_len(dev->pdev, 0));

  	if (!dev->mmio) {
		printk(KERN_ERR "pcie_tbs_probe ERROR: Mem 0 remap failed\n");
		ret = -ENODEV; /* -ENOMEM better?! */ 
		goto fail2;
	}

	/* msi is enabled */
	if (dev->int_type == 1) {
		if (pci_msi_enabled())
			err = pci_enable_msi(dev->pdev);
		if (err) {
			printk(KERN_INFO "pcie_tbs_probe INFO: MSI enable failed <%d>", err);
			goto fail0;
		}
	}

	ret = request_irq(dev->pdev->irq,
			dev->card_config->irq_handler,
			dev->int_type ? IRQF_DISABLED : IRQF_SHARED,
			"TBS PCIE",
			(void *) dev);

	if (ret < 0) {
		printk(KERN_ERR "pcie_tbs_probe ERROR: IRQ registration failed <%d>\n", ret);
		ret = -ENODEV;
		goto fail3;
	}

	revision = TBS_PCIE_READ(0, 0x20);
	printk("TBS PCIE Detected TBS%04x Board Rev %d", dev->pdev->subsystem_vendor,
		(revision & 0xff000000) >> 24);

	pci_set_drvdata(pdev, dev);

	if ((dev->pdev->subsystem_vendor == 0x6902) &&
		(dev->pdev->subsystem_device == 0x0001)) {
		if (tbs_i2c_init(dev, 0x6901) < 0)
			goto fail1;
	} else {
		if (tbs_i2c_init(dev, dev->pdev->subsystem_vendor) < 0)
			goto fail1;
	}

	/* dvb init */
	tbs_adapters_init(dev);

	if (tbs_pcie_dma_init(dev) < 0)
		goto fail4;

	if (tbs_adapters_attach(dev) < 0)
		goto fail5;

	return 0;

fail5:
	printk(KERN_ERR "pcie_tbs_probe ERROR: fail5\n");
	tbs_adapters_detach(dev);
	tbs_adapters_release(dev);
fail4:
	printk(KERN_ERR "pcie_tbs_probe ERROR: fail4\n");
	tbs_pcie_dma_free(dev);
fail3:
	if (dev->int_type) {
		printk(KERN_ERR "pcie_tbs_probe ERROR: MSI registration failed\n");
		pci_disable_msi(dev->pdev);
	}
	free_irq(dev->pdev->irq, dev);
	if (dev->mmio)
		iounmap(dev->mmio);
fail2:
	pci_disable_device(pdev);
fail1:
	pci_set_drvdata(pdev, NULL);
	kfree(dev);
fail0:
	return ret;
}

#define PCIE_MODEL_TURBOSIGHT_TBS6904	"TurboSight TBS 6904"
#define PCIE_DEV_TURBOSIGHT_TBS6904	"DVB-S/S2"

static struct tbs_card_config pcie_tbs6904_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6904,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6904,
	.adapters		= 4,
	.frontend_attach	= tbs6904fe_frontend_attach,
	.irq_handler	= tbs6904_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 0
			}, 
			{
				/* adapter 1 */
				.ts_in = 1
			},
			{
				/* adapter 2 */
				.ts_in = 2
			},
			{
				/* adapter 3 */
				.ts_in = 3
			}
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6910	"TurboSight TBS 6910"
#define PCIE_DEV_TURBOSIGHT_TBS6910	"DVB-S/S2"

static struct tbs_card_config pcie_tbs6910_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6910,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6910,
	.adapters		= 2,
	.frontend_attach	= tbs6910fe_frontend_attach,
	.irq_handler	= tbs_dual_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 3
			}, 
			{
				/* adapter 1 */
				.ts_in = 2
			},
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6901	"TurboSight TBS 6901"
#define PCIE_DEV_TURBOSIGHT_TBS6901	"DVB-S/S2"

static struct tbs_card_config pcie_tbs6901_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6901,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6901,
	.adapters		= 1,
	.frontend_attach	= tbs6901fe_frontend_attach,
	.irq_handler	= tbs6901_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 3
			}, 
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6902	"TurboSight TBS 6902"
#define PCIE_DEV_TURBOSIGHT_TBS6902	"DVB-S/S2"

static struct tbs_card_config pcie_tbs6902_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6902,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6902,
	.adapters		= 2,
	.frontend_attach	= tbs6902fe_frontend_attach,
	.irq_handler	= tbs6902_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 2
			}, 
			{
				/* adapter 1 */
				.ts_in = 3
			},
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6908	"TurboSight TBS 6908"
#define PCIE_DEV_TURBOSIGHT_TBS6908	"DVB-S/S2"

static struct tbs_card_config pcie_tbs6908_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6908,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6908,
	.adapters		= 4,
	.frontend_attach	= tbs6908fe_frontend_attach,
	.irq_handler	= tbs6908_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 1
			}, 
			{
				/* adapter 1 */
				.ts_in = 0
			},
			{
				/* adapter 2 */
				.ts_in = 3
			},
			{
				/* adapter 3 */
				.ts_in = 2
			}
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6903	"TurboSight TBS 6903"
#define PCIE_DEV_TURBOSIGHT_TBS6903	"DVB-S/S2"

static struct tbs_card_config pcie_tbs6903_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6903,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6903,
	.adapters		= 2,
	.frontend_attach	= tbs6903fe_frontend_attach,
	.irq_handler	= tbs_dual_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 3
			}, 
			{
				/* adapter 1 */
				.ts_in = 2
			},
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6205	"TurboSight TBS 6205"
#define PCIE_DEV_TURBOSIGHT_TBS6205	"DVB-T/T2/C"

static struct tbs_card_config pcie_tbs6205_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6205,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6205,
	.adapters		= 4,
	.frontend_attach	= tbs6205fe_frontend_attach,
	.irq_handler	= tbs6904_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 0
			}, 
			{
				/* adapter 1 */
				.ts_in = 1
			},
			{
				/* adapter 2 */
				.ts_in = 2
			},
			{
				/* adapter 3 */
				.ts_in = 3
			}
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6290   "TurboSight TBS 6290 SE"
#define PCIE_DEV_TURBOSIGHT_TBS6290     "DVB-T/T2/C"

static struct tbs_card_config pcie_tbs6290_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6290,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6290,
	.adapters		= 2,
	.frontend_attach	= tbs6290se_frontend_attach,
	.irq_handler	= tbs_dual_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 3
			},
			{
				/* adapter 1 */
				.ts_in = 2
			},
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6281   "TurboSight TBS 6281 SE"
#define PCIE_DEV_TURBOSIGHT_TBS6281     "DVB-T/T2/C"

static struct tbs_card_config pcie_tbs6281_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6281,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6281,
	.adapters		= 2,
	.frontend_attach	= tbs6281se_frontend_attach,
	.irq_handler	= tbs_tbs6281se_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 1
			},
			{
				/* adapter 1 */
				.ts_in = 2
			},
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6814	"TurboSight TBS 6814"
#define PCIE_DEV_TURBOSIGHT_TBS6814	"ISDB-T"

static struct tbs_card_config pcie_tbs6814_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6814,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6814,
	.adapters		= 4,
	.frontend_attach	= tbs6814fe_frontend_attach,
	.irq_handler	= tbs6904_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 0
			}, 
			{
				/* adapter 1 */
				.ts_in = 1
			},
			{
				/* adapter 2 */
				.ts_in = 2
			},
			{
				/* adapter 3 */
				.ts_in = 3
			}
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6704	"TurboSight TBS 6704"
#define PCIE_DEV_TURBOSIGHT_TBS6704	"ATSC"

static struct tbs_card_config pcie_tbs6704_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6704,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6704,
	.adapters		= 4,
	.frontend_attach	= tbs6704fe_frontend_attach,
	.irq_handler	= tbs6904_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 0
			}, 
			{
				/* adapter 1 */
				.ts_in = 1
			},
			{
				/* adapter 2 */
				.ts_in = 2
			},
			{
				/* adapter 3 */
				.ts_in = 3
			}
		}
};

#define PCIE_MODEL_TURBOSIGHT_TBS6909	"TurboSight TBS 6909"
#define PCIE_DEV_TURBOSIGHT_TBS6909	"DVB-S/S2"

static struct tbs_card_config pcie_tbs6909_config = {
	.model_name		= PCIE_MODEL_TURBOSIGHT_TBS6909,
	.dev_type		= PCIE_DEV_TURBOSIGHT_TBS6909,
	.adapters		= 8,
	.frontend_attach	= tbs6909fe_frontend_attach,
	.irq_handler	= tbs6909_pcie_irq,
	.adap_config	= {
			{
				/* adapter 0 */
				.ts_in = 0
			}, 
			{
				/* adapter 1 */
				.ts_in = 4
			},
			{
				/* adapter 2 */
				.ts_in = 1
			},
			{
				/* adapter 3 */
				.ts_in = 5
			},
			{
				/* adapter 4 */
				.ts_in = 2
			},
			{
				/* adapter 5 */
				.ts_in = 6
			},
			{
				/* adapter 6 */
				.ts_in = 3
			},
			{
				/* adapter 7 */
				.ts_in = 7
			}
		}
};

#define MAKE_ENTRY( __vend, __chip, __subven, __subdev, __configptr) {	\
	.vendor		= (__vend),					\
	.device		= (__chip),					\
	.subvendor	= (__subven),					\
	.subdevice	= (__subdev),					\
	.driver_data	= (unsigned long) (__configptr)			\
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
static const struct pci_device_id tbs_pci_table[] __devinitdata = {
#else
static const struct pci_device_id tbs_pci_table[] = {
#endif
	MAKE_ENTRY(0x544d, 0x6178, 0x6904, 0x1131, &pcie_tbs6904_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6908, 0x1131, &pcie_tbs6908_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6908, 0x0001, &pcie_tbs6908_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6908, 0x1132, &pcie_tbs6908_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6905, 0x0001, &pcie_tbs6908_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6205, 0x1131, &pcie_tbs6205_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6205, 0x0001, &pcie_tbs6205_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6290, 0x0002, &pcie_tbs6290_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6281, 0x0002, &pcie_tbs6281_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6814, 0x1131, &pcie_tbs6814_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6814, 0x0001, &pcie_tbs6814_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6704, 0x0001, &pcie_tbs6704_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6903, 0x0001, &pcie_tbs6903_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6910, 0x0001, &pcie_tbs6910_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6902, 0x0002, &pcie_tbs6902_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6902, 0x0001, &pcie_tbs6901_config),
	MAKE_ENTRY(0x544d, 0x6178, 0x6909, 0x0001, &pcie_tbs6909_config),
	{ }
};
MODULE_DEVICE_TABLE(pci, tbs_pci_table);

static struct pci_driver tbs_pci_driver = {
	.name        = "TBS PCIE",
	.id_table    = tbs_pci_table,
	.probe       = tbs_probe,
	.remove      = tbs_remove,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
static int __devinit pcie_tbs_init(void)
#else
static __init int pcie_tbs_init(void)
#endif
{
	return pci_register_driver(&tbs_pci_driver);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
static void __devexit pcie_tbs_exit(void)
#else
static __exit void pcie_tbs_exit(void)
#endif
{
	pci_unregister_driver(&tbs_pci_driver);
}

module_init(pcie_tbs_init);
module_exit(pcie_tbs_exit);

MODULE_DESCRIPTION("TBS PCIE driver");
MODULE_AUTHOR("Konstantin Dimitrov <kosio.dimitrov@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
