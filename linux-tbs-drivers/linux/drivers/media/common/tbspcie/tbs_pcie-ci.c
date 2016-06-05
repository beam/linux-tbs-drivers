/*
	TurboSight TBS PCIE CI driver
    	Copyright (C) 2015 Konstantin Dimitrov <kosio.dimitrov@gmail.com>

    	Copyright (C) 2015 www.tbsdtv.com
*/

#include "tbs_pcie.h"

#include "dvb_ca_en50221.h"

#define TBS_CI_BASE(i)	(0x6000 + (1 - (i)) * 0x1000)
#define TBS_CI_BASE_1	0x6000
#define TBS_CI_BASE_0	0x7000

struct tbs_ci_state {
	struct dvb_ca_en50221 ca;
	struct mutex ca_mutex;
	int nr;
	void *priv; /* struct tbs_adapter *priv; */
	int status;
};

int tbs_ci_read_cam_control(struct dvb_ca_en50221 *ca, 
	int slot, u8 address)
{
	struct tbs_ci_state *state = ca->data;
	struct tbs_adapter *adapter =
			(struct tbs_adapter *) state->priv;
	struct tbs_pcie_dev *dev = adapter->dev;
	u32 data = 0;

	if (slot != 0)
		return -EINVAL;

	mutex_lock(&state->ca_mutex);

	data |= (address & 3) << 8;
	data |= 0x02 << 16;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x00, data);
	udelay(150);
	
	data = TBS_PCIE_READ(TBS_CI_BASE(state->nr), 0x08);

	mutex_unlock(&state->ca_mutex);

	return (data & 0xff);
}

int tbs_ci_write_cam_control(struct dvb_ca_en50221 *ca, int slot,
	u8 address, u8 value)
{
	struct tbs_ci_state *state = ca->data;
	struct tbs_adapter *adapter =
			(struct tbs_adapter *) state->priv;
	struct tbs_pcie_dev *dev = adapter->dev;
	u32 data = 0;

	if (slot != 0)
		return -EINVAL;

	mutex_lock(&state->ca_mutex);

	data |= (address & 3) << 8;
	data |= 0x03 << 16;
	data |= value << 24;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x00, data);
	udelay(150);

	mutex_unlock(&state->ca_mutex);

	return 0;
}

int tbs_ci_read_attribute_mem(struct dvb_ca_en50221 *ca,
	int slot, int address)
{
	struct tbs_ci_state *state = ca->data;
	struct tbs_adapter *adapter = 
			(struct tbs_adapter *) state->priv;
	struct tbs_pcie_dev *dev = adapter->dev;
	u32 data = 0;

	if (slot != 0)
		return -EINVAL;

	mutex_lock(&state->ca_mutex);

	data |= (address >> 8) & 0x7f;
	data |= (address & 0xff) << 8;
	data |= 0x00 << 16;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x00, data);
	udelay(150);

	data = TBS_PCIE_READ(TBS_CI_BASE(state->nr), 0x04);

	mutex_unlock(&state->ca_mutex);

	return (data & 0xff);
}

int tbs_ci_write_attribute_mem(struct dvb_ca_en50221 *ca,
	int slot, int address, u8 value)
{
	struct tbs_ci_state *state = ca->data;
	struct tbs_adapter *adapter =
			(struct tbs_adapter *) state->priv;
	struct tbs_pcie_dev *dev = adapter->dev;
	u32 data = 0;

	if (slot != 0)
		return -EINVAL;

	mutex_lock(&state->ca_mutex);

	data |= (address >> 8) & 0x7f;
	data |= (address & 0xff) << 8;
	data |= 0x01 << 16;
	data |= value << 24;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x00, data);
	udelay(150);

	mutex_unlock(&state->ca_mutex);

	return 0;
}

static int tbs_ci_set_video_port(struct dvb_ca_en50221 *ca,
	int slot, int enable)
{
	struct tbs_ci_state *state = ca->data;
	struct tbs_adapter *adapter = 
			(struct tbs_adapter *) state->priv;
	struct tbs_pcie_dev *dev = adapter->dev;
	u32 data;

	if (slot != 0)
		return -EINVAL;

	mutex_lock(&state->ca_mutex);

	data = enable & 1;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x0c, data);

	mutex_unlock(&state->ca_mutex);

	printk("tbs_pcie_ci: Adapter %d CI slot %sabled\n",
		adapter->fe->dvb->num,
		enable ? "en" : "dis");

	return 0;
}

int tbs_ci_slot_shutdown(struct dvb_ca_en50221 *ca, int slot)
{
	return tbs_ci_set_video_port(ca, slot, /* enable */ 0);
}

int tbs_ci_slot_ts_enable(struct dvb_ca_en50221 *ca, int slot)
{
	return tbs_ci_set_video_port(ca, slot, /* enable */ 1);
}

int tbs_ci_slot_reset(struct dvb_ca_en50221 *ca, int slot)
{
	struct tbs_ci_state *state = ca->data;
	struct tbs_adapter *adapter =
			(struct tbs_adapter *) state->priv;
	struct tbs_pcie_dev *dev = adapter->dev;
	u32 data;

	if (slot != 0)
		return -EINVAL;
	
	mutex_lock (&state->ca_mutex);

	data = 1;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x04, data);
	msleep (5);

	data = 0;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x04, data);
	msleep (1400);

	mutex_unlock (&state->ca_mutex);

	return 0;
}

int tbs_ci_poll_slot_status(struct dvb_ca_en50221 *ca, 
	int slot, int open)
{
	struct tbs_ci_state *state = ca->data;
	struct tbs_adapter *adapter =
			(struct tbs_adapter *) state->priv;
	struct tbs_pcie_dev *dev = adapter->dev;
	u32 data;

	if (slot != 0)
		return -EINVAL;

	mutex_lock(&state->ca_mutex);

	data = TBS_PCIE_READ(TBS_CI_BASE(state->nr), 0x0c);
	data &= 1;

	if (state->status != data){
		TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x08, !data);
		msleep(300);
		state->status = data;
	}
	
	mutex_unlock(&state->ca_mutex);

	if (data & 1) {
		return (DVB_CA_EN50221_POLL_CAM_PRESENT |
			DVB_CA_EN50221_POLL_CAM_READY);
	} else {
		return 0;
	}

	return 0;
}

int tbs_ci_init(struct tbs_adapter *adap, int nr)
{
	struct tbs_ci_state *state;
//	struct tbs_pcie_dev *dev = adap->dev;
//	int data;
	int ret;

	/* allocate memory for the internal state */
	state = kzalloc(sizeof(struct tbs_ci_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto error1;
	}

	adap->adap_priv = state;
	
	state->nr = nr;
	state->status = 0;

	mutex_init(&state->ca_mutex);
	
	state->ca.owner = THIS_MODULE;
	state->ca.read_attribute_mem = tbs_ci_read_attribute_mem;
	state->ca.write_attribute_mem = tbs_ci_write_attribute_mem;
	state->ca.read_cam_control = tbs_ci_read_cam_control;
	state->ca.write_cam_control = tbs_ci_write_cam_control;
	state->ca.slot_reset = tbs_ci_slot_reset;
	state->ca.slot_shutdown = tbs_ci_slot_shutdown;
	state->ca.slot_ts_enable = tbs_ci_slot_ts_enable;
	state->ca.poll_slot_status = tbs_ci_poll_slot_status;
	state->ca.data = state;
	state->priv = adap;

	printk("tbs_pcie_ci: Initializing TBS PCIE CI#%d slot\n", nr);

	ret = dvb_ca_en50221_init(&adap->dvb_adapter, &state->ca,
		/* flags */ 0, /* n_slots */ 1);
	if (ret != 0) goto error2;

#if 0
	data = 0x00000005;
	TBS_PCIE_WRITE(TBS_CI_BASE(state->nr), 0x10, data);
#endif

	printk("tbs_pcie_ci: Adapter %d CI slot initialized\n", 
		adap->dvb_adapter.num);

	return 0;
	
error2: 
	//memset (&state->ca, 0, sizeof (state->ca)); 
	kfree(state);
error1:
	printk("tbs_pcie_ci: Adapter %d CI slot initialization failed\n",
		adap->dvb_adapter.num);
	return ret;
}

void tbs_ci_release(struct tbs_adapter *adap)
{
	struct tbs_ci_state *state;

	if (NULL == adap) return;

	state = (struct tbs_ci_state *)adap->adap_priv;

	if (NULL == state) return;

	if (NULL == state->ca.data) return;

	dvb_ca_en50221_release(&state->ca);
	//memset(&state->ca, 0, sizeof(state->ca));
	kfree(state);
}
