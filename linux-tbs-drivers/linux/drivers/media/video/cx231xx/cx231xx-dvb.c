/*
 DVB device driver for cx231xx

 Copyright (C) 2008 <srinivasa.deevi at conexant dot com>
		Based on em28xx driver

 This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include "cx231xx.h"
#include <media/v4l2-common.h>
#include <media/videobuf-vmalloc.h>

#include "xc5000.h"
#include "s5h1432.h"
#include "tda18271.h"
#include "s5h1411.h"
#include "lgdt3305.h"
#include "mb86a20s.h"
#include "tbs5280fe.h"
#include "tbs5281fe.h"
#include "tbs5990fe.h"
#include "tbs5926fe.h"
#include "tbscxci.h"
#include "tbsfe.h"

MODULE_DESCRIPTION("driver for cx231xx based DVB cards");
MODULE_AUTHOR("Srinivasa Deevi <srinivasa.deevi@conexant.com>");
MODULE_LICENSE("GPL");

static unsigned int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "enable debug messages [dvb]");

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

#define dprintk(level, fmt, arg...) do {			\
if (debug >= level) 						\
	printk(KERN_DEBUG "%s/2-dvb: " fmt, dev->name, ## arg);	\
} while (0)

#define CX231XX_DVB_NUM_BUFS 5
#define CX231XX_DVB_MAX_PACKETSIZE 564
#define CX231XX_DVB_MAX_PACKETS 64

#if 0
struct cx231xx_dvb {
	struct dvb_frontend *frontend;

	/* feed count management */
	struct mutex lock;
	int nfeeds;
	u8 count;

	/* general boilerplate stuff */
	struct dvb_adapter adapter;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dmx_frontend fe_hw;
	struct dmx_frontend fe_mem;
	struct dvb_net net;
};
#endif

static struct s5h1432_config dvico_s5h1432_config = {
	.output_mode   = S5H1432_SERIAL_OUTPUT,
	.gpio          = S5H1432_GPIO_ON,
	.qam_if        = S5H1432_IF_4000,
	.vsb_if        = S5H1432_IF_4000,
	.inversion     = S5H1432_INVERSION_OFF,
	.status_mode   = S5H1432_DEMODLOCKING,
	.mpeg_timing   = S5H1432_MPEGTIMING_CONTINOUS_NONINVERTING_CLOCK,
};

static struct tda18271_std_map cnxt_rde253s_tda18271_std_map = {
	.dvbt_6   = { .if_freq = 4000, .agc_mode = 3, .std = 4,
		      .if_lvl = 1, .rfagc_top = 0x37, },
	.dvbt_7   = { .if_freq = 4000, .agc_mode = 3, .std = 5,
		      .if_lvl = 1, .rfagc_top = 0x37, },
	.dvbt_8   = { .if_freq = 4000, .agc_mode = 3, .std = 6,
		      .if_lvl = 1, .rfagc_top = 0x37, },
};

static struct tda18271_std_map mb86a20s_tda18271_config = {
	.dvbt_6   = { .if_freq = 3300, .agc_mode = 3, .std = 4,
		      .if_lvl = 7, .rfagc_top = 0x37, },
};

static struct tda18271_config cnxt_rde253s_tunerconfig = {
	.std_map = &cnxt_rde253s_tda18271_std_map,
	.gate    = TDA18271_GATE_ANALOG,
};

static struct s5h1411_config tda18271_s5h1411_config = {
	.output_mode   = S5H1411_SERIAL_OUTPUT,
	.gpio          = S5H1411_GPIO_OFF,
	.vsb_if        = S5H1411_IF_3250,
	.qam_if        = S5H1411_IF_4000,
	.inversion     = S5H1411_INVERSION_ON,
	.status_mode   = S5H1411_DEMODLOCKING,
	.mpeg_timing   = S5H1411_MPEGTIMING_CONTINOUS_NONINVERTING_CLOCK,
};
static struct s5h1411_config xc5000_s5h1411_config = {
	.output_mode   = S5H1411_SERIAL_OUTPUT,
	.gpio          = S5H1411_GPIO_OFF,
	.vsb_if        = S5H1411_IF_3250,
	.qam_if        = S5H1411_IF_3250,
	.inversion     = S5H1411_INVERSION_OFF,
	.status_mode   = S5H1411_DEMODLOCKING,
	.mpeg_timing   = S5H1411_MPEGTIMING_CONTINOUS_NONINVERTING_CLOCK,
};

static struct lgdt3305_config hcw_lgdt3305_config = {
	.i2c_addr           = 0x0e,
	.mpeg_mode          = LGDT3305_MPEG_SERIAL,
	.tpclk_edge         = LGDT3305_TPCLK_FALLING_EDGE,
	.tpvalid_polarity   = LGDT3305_TP_VALID_HIGH,
	.deny_i2c_rptr      = 1,
	.spectral_inversion = 1,
	.qam_if_khz         = 4000,
	.vsb_if_khz         = 3250,
};

static struct tda18271_std_map hauppauge_tda18271_std_map = {
	.atsc_6   = { .if_freq = 3250, .agc_mode = 3, .std = 4,
		      .if_lvl = 1, .rfagc_top = 0x58, },
	.qam_6    = { .if_freq = 4000, .agc_mode = 3, .std = 5,
		      .if_lvl = 1, .rfagc_top = 0x58, },
};

static struct tda18271_config hcw_tda18271_config = {
	.std_map = &hauppauge_tda18271_std_map,
	.gate    = TDA18271_GATE_DIGITAL,
};

static const struct mb86a20s_config pv_mb86a20s_config = {
	.demod_address = 0x10,
	.is_serial = true,
};

static struct tda18271_config pv_tda18271_config = {
	.std_map = &mb86a20s_tda18271_config,
	.gate    = TDA18271_GATE_DIGITAL,
	.small_i2c = TDA18271_03_BYTE_CHUNK_INIT,
};

int cx231xx_tbsctrl1(struct cx231xx *dev, int type)
{
	if (type == 1)
		return cx231xx_set_gpio_bit(dev, dev->gpio_dir, 
					(u8 *)&dev->gpio_val);
	else
		return cx231xx_get_gpio_bit(dev, dev->gpio_dir, 
					(u8 *)&dev->gpio_val);
}

u32 cx231xx_tbsctrl2(struct cx231xx *dev, int type)
{
	if (type == 1)
		return dev->gpio_dir;
	else
		return dev->gpio_val;
}

void cx231xx_tbsctrl3(struct cx231xx *dev, int type, u32 val)
{
	if (type == 1)
		dev->gpio_dir = val;
	else
		dev->gpio_val = val;
}

static struct tbs5280fe_config tbs5280fe_config0 = {
	.tbs5280fe_address = 0x6c,

	.tbs5280_ctrl1 = cx231xx_tbsctrl1,
	.tbs5280_ctrl2 = cx231xx_tbsctrl2,
	.tbs5280_ctrl3 = cx231xx_tbsctrl3,
};

static struct tbs5280fe_config tbs5280fe_config1 = {
	.tbs5280fe_address = 0x6d,

	.tbs5280_ctrl1 = cx231xx_tbsctrl1,
	.tbs5280_ctrl2 = cx231xx_tbsctrl2,
	.tbs5280_ctrl3 = cx231xx_tbsctrl3,
};

static struct tbs5281fe_config tbs5281fe_config = {
	.tbs5281fe_address = 0x64,

	.tbs5281_ctrl1 = cx231xx_tbsctrl1,
	.tbs5281_ctrl2 = cx231xx_tbsctrl2,
	.tbs5281_ctrl3 = cx231xx_tbsctrl3,
};

static struct tbs5990fe_config tbs5990fe_config0 = {
	.tbs5990fe_address = 0x60,

	.tbs5990_ctrl1 = cx231xx_tbsctrl1,
	.tbs5990_ctrl2 = cx231xx_tbsctrl2,
	.tbs5990_ctrl3 = cx231xx_tbsctrl3,
};

static struct tbs5990fe_config tbs5990fe_config1 = {
	.tbs5990fe_address = 0x68,
	
	.tbs5990_ctrl1 = cx231xx_tbsctrl1,
	.tbs5990_ctrl2 = cx231xx_tbsctrl2,
	.tbs5990_ctrl3 = cx231xx_tbsctrl3,
};

static struct tbs5926fe_config tbs5926fe_config = {
	.tbs5926fe_address = 0x68,

	.tbs5926_ctrl1 = cx231xx_tbsctrl1,
	.tbs5926_ctrl2 = cx231xx_tbsctrl2,
	.tbs5926_ctrl3 = cx231xx_tbsctrl3,
};

static inline void print_err_status(struct cx231xx *dev, int packet, int status)
{
	char *errmsg = "Unknown";

	switch (status) {
	case -ENOENT:
		errmsg = "unlinked synchronuously";
		break;
	case -ECONNRESET:
		errmsg = "unlinked asynchronuously";
		break;
	case -ENOSR:
		errmsg = "Buffer error (overrun)";
		break;
	case -EPIPE:
		errmsg = "Stalled (device not responding)";
		break;
	case -EOVERFLOW:
		errmsg = "Babble (bad cable?)";
		break;
	case -EPROTO:
		errmsg = "Bit-stuff error (bad cable?)";
		break;
	case -EILSEQ:
		errmsg = "CRC/Timeout (could be anything)";
		break;
	case -ETIME:
		errmsg = "Device does not respond";
		break;
	}
	if (packet < 0) {
		dprintk(1, "URB status %d [%s].\n", status, errmsg);
	} else {
		dprintk(1, "URB packet %d, status %d [%s].\n",
			packet, status, errmsg);
	}
}

static inline int dvb_isoc_copy(struct cx231xx *dev, struct urb *urb)
{
	int i;

	if (!dev)
		return 0;

	if (dev->state & DEV_DISCONNECTED)
		return 0;

	if (urb->status < 0) {
		print_err_status(dev, -1, urb->status);
		if (urb->status == -ENOENT)
			return 0;
	}

	for (i = 0; i < urb->number_of_packets; i++) {
		int status = urb->iso_frame_desc[i].status;

		if (status < 0) {
			print_err_status(dev, i, status);
			if (urb->iso_frame_desc[i].status != -EPROTO)
				continue;
		}

		dvb_dmx_swfilter(&dev->dvb[0]->demux,
				 urb->transfer_buffer +
				urb->iso_frame_desc[i].offset,
				urb->iso_frame_desc[i].actual_length);
	}

	return 0;
}

static inline int dvb_isoc_copy_ts2(struct cx231xx *dev, struct urb *urb)
{
	int i;

	if (!dev)
		return 0;

	if (dev->state & DEV_DISCONNECTED)
		return 0;

	if (urb->status < 0) {
		print_err_status(dev, -1, urb->status);
		if (urb->status == -ENOENT)
			return 0;
	}

	for (i = 0; i < urb->number_of_packets; i++) {
		int status = urb->iso_frame_desc[i].status;

		if (status < 0) {
			print_err_status(dev, i, status);
			if (urb->iso_frame_desc[i].status != -EPROTO)
				continue;
		}

		dvb_dmx_swfilter(&dev->dvb[1]->demux,
				 urb->transfer_buffer +
				urb->iso_frame_desc[i].offset,
				urb->iso_frame_desc[i].actual_length);
	}

	return 0;
}

static inline int dvb_bulk_copy(struct cx231xx *dev, struct urb *urb)
{
	if (!dev)
		return 0;

	if (dev->state & DEV_DISCONNECTED)
		return 0;

	if (urb->status < 0) {
		print_err_status(dev, -1, urb->status);
		if (urb->status == -ENOENT)
			return 0;
	}

	/* Feed the transport payload into the kernel demux */
	dvb_dmx_swfilter(&dev->dvb[0]->demux,
		urb->transfer_buffer, urb->actual_length);

	return 0;
}

static inline int dvb_bulk_copy_ts2(struct cx231xx *dev, struct urb *urb)
{
	if (!dev)
		return 0;

	if (dev->state & DEV_DISCONNECTED)
		return 0;

	if (urb->status < 0) {
		print_err_status(dev, -1, urb->status);
		if (urb->status == -ENOENT)
			return 0;
	}

	/* Feed the transport payload into the kernel demux */
	dvb_dmx_swfilter(&dev->dvb[1]->demux,
		urb->transfer_buffer, urb->actual_length);

	return 0;
}

static int start_streaming(struct cx231xx_dvb *dvb)
{
	int rc;
	struct cx231xx *dev = dvb->adapter.priv;

	if (dev->USE_ISO) {
		/* cx231xx_info("DVB transfer mode is ISO.\n"); */
		mutex_lock(&dev->i2c_lock);
		cx231xx_enable_i2c_port_3(dev, false);
		if (dvb->count == 0)
			cx231xx_set_alt_setting(dev, INDEX_TS1, 5);
		if (dvb->count == 1)
			cx231xx_set_alt_setting(dev, INDEX_TS2, 5);
		cx231xx_enable_i2c_port_3(dev, true);
		mutex_unlock(&dev->i2c_lock);
		rc = cx231xx_set_mode(dev, CX231XX_DIGITAL_MODE);
		if (rc < 0)
			return rc;
		dev->mode_tv = 1;
		if (dvb->count == 1)
		return cx231xx_init_isoc_ts2(dev, CX231XX_DVB_MAX_PACKETS,
				CX231XX_DVB_NUM_BUFS,
				dev->ts2_mode.max_pkt_size,
				dvb_isoc_copy_ts2);
		else
		return cx231xx_init_isoc(dev, CX231XX_DVB_MAX_PACKETS,
				CX231XX_DVB_NUM_BUFS,
				dev->ts1_mode.max_pkt_size,
				dvb_isoc_copy);
		
	} else {
		/* cx231xx_info("DVB transfer mode is BULK.\n"); */
		if (dvb->count == 0)
			cx231xx_set_alt_setting(dev, INDEX_TS1, 0);
		if (dvb->count == 1)
			cx231xx_set_alt_setting(dev, INDEX_TS2, 0);
		rc = cx231xx_set_mode(dev, CX231XX_DIGITAL_MODE);
		if (rc < 0)
			return rc;
		dev->mode_tv = 1;
		if (dvb->count == 1)
		return cx231xx_init_bulk_ts2(dev, CX231XX_DVB_MAX_PACKETS,
				CX231XX_DVB_NUM_BUFS,
				dev->ts2_mode.max_pkt_size,
				dvb_bulk_copy_ts2);
		else
		return cx231xx_init_bulk(dev, CX231XX_DVB_MAX_PACKETS,
				CX231XX_DVB_NUM_BUFS,
				dev->ts1_mode.max_pkt_size,
				dvb_bulk_copy);
	}

}

static int stop_streaming(struct cx231xx_dvb *dvb)
{
	struct cx231xx *dev = dvb->adapter.priv;

	if (dev->USE_ISO)
		if (dvb->count == 0)
			cx231xx_uninit_isoc(dev);
		if (dvb->count == 1)
			cx231xx_uninit_isoc_ts2(dev);
	else
		if (dvb->count == 0)
			cx231xx_uninit_bulk(dev);
		if (dvb->count == 1)
			cx231xx_uninit_bulk_ts2(dev);

	cx231xx_set_mode(dev, CX231XX_SUSPEND);

	return 0;
}

static int start_feed(struct dvb_demux_feed *feed)
{
	struct dvb_demux *demux = feed->demux;
	struct cx231xx_dvb *dvb = demux->priv;
	int rc, ret;

	if (!demux->dmx.frontend)
		return -EINVAL;

	mutex_lock(&dvb->lock);
	dvb->nfeeds++;
	rc = dvb->nfeeds;

	if (dvb->nfeeds == 1) {
		ret = start_streaming(dvb);
		if (ret < 0)
			rc = ret;
	}

	mutex_unlock(&dvb->lock);
	return rc;
}

static int stop_feed(struct dvb_demux_feed *feed)
{
	struct dvb_demux *demux = feed->demux;
	struct cx231xx_dvb *dvb = demux->priv;
	int err = 0;

	mutex_lock(&dvb->lock);
	dvb->nfeeds--;

	if (0 == dvb->nfeeds)
		err = stop_streaming(dvb);

	mutex_unlock(&dvb->lock);
	return err;
}

/* ------------------------------------------------------------------ */
static int cx231xx_dvb_bus_ctrl(struct dvb_frontend *fe, int acquire)
{
	struct cx231xx *dev = fe->dvb->priv;

	if (acquire)
		return cx231xx_set_mode(dev, CX231XX_DIGITAL_MODE);
	else
		return cx231xx_set_mode(dev, CX231XX_SUSPEND);
}

/* ------------------------------------------------------------------ */

static struct xc5000_config cnxt_rde250_tunerconfig = {
	.i2c_address = 0x61,
	.if_khz = 4000,
};
static struct xc5000_config cnxt_rdu250_tunerconfig = {
	.i2c_address = 0x61,
	.if_khz = 3250,
};

/* ------------------------------------------------------------------ */
#if 0
static int attach_xc5000(u8 addr, struct cx231xx *dev)
{

	struct dvb_frontend *fe;
	struct xc5000_config cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.i2c_adap = &dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap;
	cfg.i2c_addr = addr;

	if (!dev->dvb[i]->frontend) {
		printk(KERN_ERR "%s/2: dvb frontend not attached. "
		       "Can't attach xc5000\n", dev->name);
		return -EINVAL;
	}

	fe = dvb_attach(xc5000_attach, dev->dvb[i]->frontend, &cfg);
	if (!fe) {
		printk(KERN_ERR "%s/2: xc5000 attach failed\n", dev->name);
		dvb_frontend_detach(dev->dvb[i]->frontend);
		dev->dvb[i]->frontend = NULL;
		return -EINVAL;
	}

	printk(KERN_INFO "%s/2: xc5000 attached\n", dev->name);

	return 0;
}
#endif

int cx231xx_set_analog_freq(struct cx231xx *dev, u32 freq)
{
	int status = 0;

	if ((dev->dvb[0] != NULL) && (dev->dvb[0]->frontend != NULL)) {

		struct dvb_tuner_ops *dops = &dev->dvb[0]->frontend->ops.tuner_ops;

		if (dops->set_analog_params != NULL) {
			struct analog_parameters params;

			params.frequency = freq;
			params.std = dev->norm;
			params.mode = 0;	/* 0- Air; 1 - cable */
			/*params.audmode = ;       */

			/* Set the analog parameters to set the frequency */
			dops->set_analog_params(dev->dvb[0]->frontend, &params);
		}

	}

	return status;
}

int cx231xx_reset_analog_tuner(struct cx231xx *dev)
{
	int status = 0;

	if ((dev->dvb[0] != NULL) && (dev->dvb[0]->frontend != NULL)) {

		struct dvb_tuner_ops *dops = &dev->dvb[0]->frontend->ops.tuner_ops;

		if (dops->init != NULL && !dev->xc_fw_load_done) {

			cx231xx_info("Reloading firmware for XC5000\n");
			status = dops->init(dev->dvb[0]->frontend);
			if (status == 0) {
				dev->xc_fw_load_done = 1;
				cx231xx_info
				    ("XC5000 firmware download completed\n");
			} else {
				dev->xc_fw_load_done = 0;
				cx231xx_info
				    ("XC5000 firmware download failed !!!\n");
			}
		}

	}

	return status;
}

/* ------------------------------------------------------------------ */

static int register_dvb(struct cx231xx_dvb *dvb,
			struct module *module,
			struct cx231xx *dev, struct device *device)
{
	int result;

	mutex_init(&dvb->lock);

	/* register adapter */
	result = dvb_register_adapter(&dvb->adapter, dev->name, module, device,
				      adapter_nr);
	if (result < 0) {
		printk(KERN_WARNING
		       "%s: dvb_register_adapter failed (errno = %d)\n",
		       dev->name, result);
		goto fail_adapter;
	}

	dvb->adapter.priv = dev;

#if 0

	/* register frontend */
	result = dvb_register_frontend(&dvb->adapter, dvb->frontend);
	if (result < 0) {
		printk(KERN_WARNING
		       "%s: dvb_register_frontend failed (errno = %d)\n",
		       dev->name, result);
		goto fail_frontend;
	}
#endif

	/* register demux stuff */
	dvb->demux.dmx.capabilities =
	    DMX_TS_FILTERING | DMX_SECTION_FILTERING |
	    DMX_MEMORY_BASED_FILTERING;
	dvb->demux.priv = dvb;
	dvb->demux.filternum = 256;
	dvb->demux.feednum = 256;
	dvb->demux.start_feed = start_feed;
	dvb->demux.stop_feed = stop_feed;

	result = dvb_dmx_init(&dvb->demux);
	if (result < 0) {
		printk(KERN_WARNING "%s: dvb_dmx_init failed (errno = %d)\n",
		       dev->name, result);
		goto fail_dmx;
	}

	dvb->dmxdev.filternum = 256;
	dvb->dmxdev.demux = &dvb->demux.dmx;
	dvb->dmxdev.capabilities = 0;
	result = dvb_dmxdev_init(&dvb->dmxdev, &dvb->adapter);
	if (result < 0) {
		printk(KERN_WARNING "%s: dvb_dmxdev_init failed (errno = %d)\n",
		       dev->name, result);
		goto fail_dmxdev;
	}

	dvb->fe_hw.source = DMX_FRONTEND_0;
	result = dvb->demux.dmx.add_frontend(&dvb->demux.dmx, &dvb->fe_hw);
	if (result < 0) {
		printk(KERN_WARNING
		       "%s: add_frontend failed (DMX_FRONTEND_0, errno = %d)\n",
		       dev->name, result);
		goto fail_fe_hw;
	}

	dvb->fe_mem.source = DMX_MEMORY_FE;
	result = dvb->demux.dmx.add_frontend(&dvb->demux.dmx, &dvb->fe_mem);
	if (result < 0) {
		printk(KERN_WARNING
		       "%s: add_frontend failed (DMX_MEMORY_FE, errno = %d)\n",
		       dev->name, result);
		goto fail_fe_mem;
	}

	result = dvb->demux.dmx.connect_frontend(&dvb->demux.dmx, &dvb->fe_hw);
	if (result < 0) {
		printk(KERN_WARNING
		       "%s: connect_frontend failed (errno = %d)\n", dev->name,
		       result);
		goto fail_fe_conn;
	}

	/* register network adapter */
	dvb_net_init(&dvb->adapter, &dvb->net, &dvb->demux.dmx);

#if 0
        /* register frontend */
        result = dvb_register_frontend(&dvb->adapter, dvb->frontend);
        if (result < 0) {
                printk(KERN_WARNING
                       "%s: dvb_register_frontend failed (errno = %d)\n",
                       dev->name, result);
                goto fail_frontend;
        }
#endif
	return 0;

fail_fe_conn:
	dvb->demux.dmx.remove_frontend(&dvb->demux.dmx, &dvb->fe_mem);
fail_fe_mem:
	dvb->demux.dmx.remove_frontend(&dvb->demux.dmx, &dvb->fe_hw);
fail_fe_hw:
	dvb_dmxdev_release(&dvb->dmxdev);
fail_dmxdev:
	dvb_dmx_release(&dvb->demux);
fail_dmx:
	dvb_unregister_frontend(dvb->frontend);
#if 0
fail_frontend:
	dvb_frontend_detach(dvb->frontend);
	dvb_unregister_adapter(&dvb->adapter);
#endif
fail_adapter:
	return result;
}

static void unregister_dvb(struct cx231xx_dvb *dvb)
{
	dvb_net_release(&dvb->net);
	dvb->demux.dmx.remove_frontend(&dvb->demux.dmx, &dvb->fe_mem);
	dvb->demux.dmx.remove_frontend(&dvb->demux.dmx, &dvb->fe_hw);
	dvb_dmxdev_release(&dvb->dmxdev);
	dvb_dmx_release(&dvb->demux);
	dvb_unregister_frontend(dvb->frontend);
	dvb_frontend_detach(dvb->frontend);
	dvb_unregister_adapter(&dvb->adapter);
}

static int tbs_cx_mac(struct i2c_adapter *i2c_adap, u8 count, u8 *mac)
{
	u8 b[64], e[256];
	int ret, i;

	struct i2c_msg msg[] = {
		{ .addr = 0x50, .flags = 0,
			.buf = b, .len = 1 },
		{ .addr = 0x50, .flags = I2C_M_RD,
			.buf = b, .len = 64 }
	};

	for (i = 0; i < 4; i++) {
		b[0] = 0x40 * i;

		ret = i2c_transfer(i2c_adap, msg, 2);

		if (ret != 2) {
			printk("TBS CX read MAC failed\n");
			return -1;
		}

		memcpy(&e[0x40 * i], b , 64);
	}
	
	memcpy(mac, &e[0x58 + 6 + 0x10*count], 6);
	
	return 0;
};

static int dvb_init(struct cx231xx *dev)
{
	int i, result = 0;
	struct cx231xx_dvb *dvb;
	u8 mac[6] = {0, 0, 0, 0, 0, 0};

	if (!dev->board.has_dvb) {
		/* This device does not support the extension */
		return 0;
	}

	for (i = 0; i < dev->board.adap_cnt; i++) {
	dvb = kzalloc(sizeof(struct cx231xx_dvb), GFP_KERNEL);

	if (dvb == NULL) {
		printk(KERN_INFO "cx231xx_dvb: memory allocation failed\n");
		return -ENOMEM;
	}

	dvb->count = i;

	dev->dvb[i] = dvb;
	dev->cx231xx_set_analog_freq = cx231xx_set_analog_freq;
	dev->cx231xx_reset_analog_tuner = cx231xx_reset_analog_tuner;

	mutex_lock(&dev->lock);

	/* register everything */
	result = register_dvb(dev->dvb[i], THIS_MODULE, dev, &dev->udev->dev);

	cx231xx_set_mode(dev, CX231XX_DIGITAL_MODE);
	cx231xx_demod_reset(dev);
	/* init frontend */
	switch (dev->model) {
	case CX231XX_BOARD_CNXT_CARRAERA:
	case CX231XX_BOARD_CNXT_RDE_250:

		dev->dvb[i]->frontend = dvb_attach(s5h1432_attach,
					&dvico_s5h1432_config,
					&dev->i2c_bus[dev->board.demod_i2c_master].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
			       ": Failed to attach s5h1432 front end\n");
			result = -EINVAL;
			goto out_free;
		}

		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		if (!dvb_attach(xc5000_attach, dev->dvb[i]->frontend,
			       &dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap,
			       &cnxt_rde250_tunerconfig)) {
			result = -EINVAL;
			goto out_free;
		}

		break;
	case CX231XX_BOARD_CNXT_SHELBY:
	case CX231XX_BOARD_CNXT_RDU_250:

		dev->dvb[i]->frontend = dvb_attach(s5h1411_attach,
					       &xc5000_s5h1411_config,
					       &dev->i2c_bus[dev->board.demod_i2c_master].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
			       ": Failed to attach s5h1411 front end\n");
			result = -EINVAL;
			goto out_free;
		}

		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		if (!dvb_attach(xc5000_attach, dev->dvb[i]->frontend,
			       &dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap,
			       &cnxt_rdu250_tunerconfig)) {
			result = -EINVAL;
			goto out_free;
		}
		break;
	case CX231XX_BOARD_CNXT_RDE_253S:

		dev->dvb[i]->frontend = dvb_attach(s5h1432_attach,
					&dvico_s5h1432_config,
					&dev->i2c_bus[dev->board.demod_i2c_master].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
			       ": Failed to attach s5h1432 front end\n");
			result = -EINVAL;
			goto out_free;
		}

		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		if (!dvb_attach(tda18271_attach, dev->dvb[i]->frontend,
			       0x60, &dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap,
			       &cnxt_rde253s_tunerconfig)) {
			result = -EINVAL;
			goto out_free;
		}
		break;
	case CX231XX_BOARD_CNXT_RDU_253S:

		dev->dvb[i]->frontend = dvb_attach(s5h1411_attach,
					       &tda18271_s5h1411_config,
					       &dev->i2c_bus[dev->board.demod_i2c_master].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
			       ": Failed to attach s5h1411 front end\n");
			result = -EINVAL;
			goto out_free;
		}

		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		if (!dvb_attach(tda18271_attach, dev->dvb[i]->frontend,
			       0x60, &dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap,
			       &cnxt_rde253s_tunerconfig)) {
			result = -EINVAL;
			goto out_free;
		}
		break;
	case CX231XX_BOARD_HAUPPAUGE_EXETER:

		printk(KERN_INFO "%s: looking for tuner / demod on i2c bus: %d\n",
		       __func__, i2c_adapter_id(&dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap));

		dev->dvb[i]->frontend = dvb_attach(lgdt3305_attach,
						&hcw_lgdt3305_config,
						&dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
			       ": Failed to attach LG3305 front end\n");
			result = -EINVAL;
			goto out_free;
		}

		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		dvb_attach(tda18271_attach, dev->dvb[i]->frontend,
			   0x60, &dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap,
			   &hcw_tda18271_config);
		break;

	case CX231XX_BOARD_PV_PLAYTV_USB_HYBRID:
	case CX231XX_BOARD_KWORLD_UB430_USB_HYBRID:

		printk(KERN_INFO "%s: looking for demod on i2c bus: %d\n",
		       __func__, i2c_adapter_id(&dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap));

		dev->dvb[i]->frontend = dvb_attach(mb86a20s_attach,
						&pv_mb86a20s_config,
						&dev->i2c_bus[dev->board.demod_i2c_master].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
			       ": Failed to attach mb86a20s demod\n");
			result = -EINVAL;
			goto out_free;
		}

		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		dvb_attach(tda18271_attach, dev->dvb[i]->frontend,
			   0x60, &dev->i2c_bus[dev->board.tuner_i2c_master].i2c_adap,
			   &pv_tda18271_config);
		break;
	case CX231XX_BOARD_TBS_5280:

		dev->dvb[i]->frontend = dvb_attach(tbs5280fe_attach,
						i ? &tbs5280fe_config1 : &tbs5280fe_config0,
						&dev->i2c_bus[dev->board.demod_i2c_master].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
				": Failed to attach demod %d\n", i);
			result = -EINVAL;
			goto out_free;
		}
		
		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		break;
	case CX231XX_BOARD_TBS_5281:

		dev->dvb[i]->frontend = dvb_attach(tbs5281fe_attach,
						&tbs5281fe_config,
						&dev->i2c_bus[dev->board.demod_i2c_master + i].i2c_adap);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
				": Failed to attach demod %d\n", i);
			result = -EINVAL;
			goto out_free;
		}
		
		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		break;
	case CX231XX_BOARD_TBS_5990:

		dev->dvb[i]->frontend = dvb_attach(tbs5990fe_attach,
						i ? &tbs5990fe_config1 : &tbs5990fe_config0,
						&dev->i2c_bus[dev->board.demod_i2c_master + i].i2c_adap, i);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
				": Failed to attach demod %d\n", i);
			result = -EINVAL;
			goto out_free;
		}

		dvb_attach(tbsfe_attach, dev->dvb[i]->frontend);

		if (i == 0) { 
			tbs_cx_mac(&dev->i2c_bus[1].i2c_adap, 0, mac);
		}

		if (i == 1) {
			memcpy(dev->dvb[0]->adapter.proposed_mac, mac, 6);
			printk(KERN_INFO "TurboSight TBS5990 MAC Addresse bas: %pM\n", mac);
			mac[5] +=1;
			memcpy(dev->dvb[1]->adapter.proposed_mac, mac, 6);
		}
		
		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		break;
	case CX231XX_BOARD_TBS_5926:

			dev->dvb[i]->frontend = dvb_attach(tbs5926fe_attach,
							&tbs5926fe_config,
							&dev->i2c_bus[dev->board.demod_i2c_master].i2c_adap, i ? 1:0);

		if (dev->dvb[i]->frontend == NULL) {
			printk(DRIVER_NAME
				": Failed to attach demod %d\n", i);
				result = -EINVAL;
				goto out_free;
		}

		/* define general-purpose callback pointer */
		dvb->frontend->callback = cx231xx_tuner_callback;

		break;

	default:
		printk(KERN_ERR "%s/2: The frontend of your DVB/ATSC card"
		       " isn't supported yet\n", dev->name);
		break;
	}
	if (NULL == dvb->frontend) {
		printk(KERN_ERR
		       "%s/2: frontend initialization failed\n", dev->name);
		result = -EINVAL;
		goto out_free;
	}

	/* Ensure all frontends negotiate bus access */
	dev->dvb[i]->frontend->ops.ts_bus_ctrl = cx231xx_dvb_bus_ctrl;
	result = dvb_register_frontend(&dev->dvb[i]->adapter, dev->dvb[i]->frontend);
//	if (result < 0)
//		goto fail_frontend;

	/* post init frontend */
	switch (dev->model) {
	case CX231XX_BOARD_TBS_5990:
		tbscxci_init(dev->dvb[i], i);
		break;
	}

	mutex_unlock(&dev->lock);

	if (result < 0)
		goto out_free;
	}

	printk(KERN_INFO "Successfully loaded cx231xx-dvb\n");

//fail_frontend:
//	dvb_frontend_detach(dev->dvb[i]->frontend);
//	dvb_unregister_adapter(&dev->dvb[i]->adapter);
ret:
	cx231xx_set_mode(dev, CX231XX_SUSPEND);
	return result;

out_free:
	kfree(dvb);
	dev->dvb[i] = NULL;
	goto ret;
}

static int dvb_fini(struct cx231xx *dev)
{
	int i;

	if (!dev->board.has_dvb) {
		/* This device does not support the extension */
		return 0;
	}

	for (i = 0; i < dev->board.adap_cnt; i++) {
	if (dev->dvb[i]) {
		tbscxci_release(dev->dvb[i]);
		unregister_dvb(dev->dvb[i]);
		dev->dvb[i] = NULL;
	}
	}

	return 0;
}

static struct cx231xx_ops dvb_ops = {
	.id = CX231XX_DVB,
	.name = "Cx231xx dvb Extension",
	.init = dvb_init,
	.fini = dvb_fini,
};

static int __init cx231xx_dvb_register(void)
{
	return cx231xx_register_extension(&dvb_ops);
}

static void __exit cx231xx_dvb_unregister(void)
{
	cx231xx_unregister_extension(&dvb_ops);
}

module_init(cx231xx_dvb_register);
module_exit(cx231xx_dvb_unregister);
