/*
 *  usbvision-cards.c
 *  usbvision cards definition file
 *
 * Copyright (c) 1999-2005 Joerg Heckenbach <joerg@heckenbach-aw.de>
 *
 * This module is part of usbvision driver project.
 * Updates to driver completed by Dwaine P. Garden
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


#include <linux/module.h>
#include <linux/list.h>
#include <media/v4l2-dev.h>
#include <media/tuner.h>
#include "usbvision.h"
#include "usbvision-cards.h"

/* Supported Devices: A table for usbvision.c*/
struct usbvision_device_data_st  usbvision_device_data[] = {
	[XANBOO] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 4,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Xanboo",
	},
	[BELKIN_VIDEOBUS_II] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Belkin USB VideoBus II Adapter",
	},
	[BELKIN_VIDEOBUS] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Belkin Components USB VideoBus",
	},
	[BELKIN_USB_VIDEOBUS_II] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Belkin USB VideoBus II",
	},
	[ECHOFX_INTERVIEW_LITE] = {
		.interface      = 0,
		.codec          = CODEC_SAA7111,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = -1,
		.y_offset       = -1,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "echoFX InterView Lite",
	},
	[USBGEAR_USBG_V1] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "USBGear USBG-V1 resp. HAMA USB",
	},
	[D_LINK_V100] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 4,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "D-Link V100",
	},
	[X10_USB_CAMERA] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "X10 USB Camera",
	},
	[HPG_WINTV_LIVE_PAL_BG] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = -1,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Live (PAL B/G)",
	},
	[HPG_WINTV_LIVE_PRO_NTSC_MN] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Live Pro (NTSC M/N)",
	},
	[ZORAN_PMD_NOGATECH] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 2,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Zoran Co. PMD (Nogatech) AV-grabber Manhattan",
	},
	[NOGATECH_USB_TV_NTSC_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = -1,
		.y_offset       = 20,
		.model_string   = "Nogatech USB-TV (NTSC) FM",
	},
	[PNY_USB_TV_NTSC_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = -1,
		.y_offset       = 20,
		.model_string   = "PNY USB-TV (NTSC) FM",
	},
	[PV_PLAYTV_USB_PRO_PAL_FM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "PixelView PlayTv-USB PRO (PAL) FM",
	},
	[ZT_721] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "ZTV ZT-721 2.4GHz USB A/V Receiver",
	},
	[HPG_WINTV_NTSC_MN] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = -1,
		.y_offset       = 20,
		.model_string   = "Hauppauge WinTV USB (NTSC M/N)",
	},
	[HPG_WINTV_PAL_BG] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTV USB (PAL B/G)",
	},
	[HPG_WINTV_PAL_I] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTV USB (PAL I)",
	},
	[HPG_WINTV_PAL_SECAM_L] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_SECAM,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_SECAM,
		.x_offset       = 0x80,
		.y_offset       = 0x16,
		.model_string   = "Hauppauge WinTV USB (PAL/SECAM L)",
	},
	[HPG_WINTV_PAL_D_K] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTV USB (PAL D/K)",
	},
	[HPG_WINTV_NTSC_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTV USB (NTSC FM)",
	},
	[HPG_WINTV_PAL_BG_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTV USB (PAL B/G FM)",
	},
	[HPG_WINTV_PAL_I_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTV USB (PAL I FM)",
	},
	[HPG_WINTV_PAL_D_K_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTV USB (PAL D/K FM)",
	},
	[HPG_WINTV_PRO_NTSC_MN] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_MICROTUNE_4049FM5,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (NTSC M/N)",
	},
	[HPG_WINTV_PRO_NTSC_MN_V2] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_MICROTUNE_4049FM5,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (NTSC M/N) V2",
	},
	[HPG_WINTV_PRO_PAL] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_FM1216ME_MK3,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL/SECAM B/G/I/D/K/L)",
	},
	[HPG_WINTV_PRO_NTSC_MN_V3] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (NTSC M/N) V3",
	},
	[HPG_WINTV_PRO_PAL_BG] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL B/G)",
	},
	[HPG_WINTV_PRO_PAL_I] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL I)",
	},
	[HPG_WINTV_PRO_PAL_SECAM_L] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_SECAM,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_SECAM,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL/SECAM L)",
	},
	[HPG_WINTV_PRO_PAL_D_K] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL D/K)",
	},
	[HPG_WINTV_PRO_PAL_SECAM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_SECAM,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_SECAM,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL/SECAM BGDK/I/L)",
	},
	[HPG_WINTV_PRO_PAL_SECAM_V2] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_SECAM,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_SECAM,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL/SECAM BGDK/I/L) V2",
	},
	[HPG_WINTV_PRO_PAL_BG_V2] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_ALPS_TSBE1_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL B/G) V2",
	},
	[HPG_WINTV_PRO_PAL_BG_D_K] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_ALPS_TSBE1_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL B/G,D/K)",
	},
	[HPG_WINTV_PRO_PAL_I_D_K] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_LG_PAL_NEW_TAPC,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL I,D/K)",
	},
	[HPG_WINTV_PRO_NTSC_MN_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (NTSC M/N FM)",
	},
	[HPG_WINTV_PRO_PAL_BG_FM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL B/G FM)",
	},
	[HPG_WINTV_PRO_PAL_I_FM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL I FM)",
	},
	[HPG_WINTV_PRO_PAL_D_K_FM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL D/K FM)",
	},
	[HPG_WINTV_PRO_TEMIC_PAL_FM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_MICROTUNE_4049FM5,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (Temic PAL/SECAM B/G/I/D/K/L FM)",
	},
	[HPG_WINTV_PRO_TEMIC_PAL_BG_FM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_MICROTUNE_4049FM5,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (Temic PAL B/G FM)",
	},
	[HPG_WINTV_PRO_PAL_FM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_FM1216ME_MK3,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (PAL/SECAM B/G/I/D/K/L FM)",
	},
	[HPG_WINTV_PRO_NTSC_MN_FM_V2] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Hauppauge WinTV USB Pro (NTSC M/N FM) V2",
	},
	[CAMTEL_TVB330] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = 5,
		.y_offset       = 5,
		.model_string   = "Camtel Technology USB TV Genie Pro FM Model TVB330",
	},
	[DIGITAL_VIDEO_CREATOR_I] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Digital Video Creator I",
	},
	[GLOBAL_VILLAGE_GV_007_NTSC] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 82,
		.y_offset       = 20,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Global Village GV-007 (NTSC)",
	},
	[DAZZLE_DVC_50_REV_1_NTSC] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Dazzle Fusion Model DVC-50 Rev 1 (NTSC)",
	},
	[DAZZLE_DVC_80_REV_1_PAL] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Dazzle Fusion Model DVC-80 Rev 1 (PAL)",
	},
	[DAZZLE_DVC_90_REV_1_SECAM] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_SECAM,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Dazzle Fusion Model DVC-90 Rev 1 (SECAM)",
	},
	[ESKAPE_LABS_MYTV2GO] = {
		.interface      = 0,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_FM1216ME_MK3,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Eskape Labs MyTV2Go",
	},
	[PINNA_PCTV_USB_PAL] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 0,
		.tuner          = 1,
		.tuner_type     = TUNER_TEMIC_4066FY5_PAL_I,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Pinnacle Studio PCTV USB (PAL)",
	},
	[PINNA_PCTV_USB_SECAM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_SECAM,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_SECAM,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Pinnacle Studio PCTV USB (SECAM)",
	},
	[PINNA_PCTV_USB_PAL_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = 128,
		.y_offset       = 23,
		.model_string   = "Pinnacle Studio PCTV USB (PAL) FM",
	},
	[MIRO_PCTV_USB] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_PAL,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Miro PCTV USB",
	},
	[PINNA_PCTV_USB_NTSC_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Pinnacle Studio PCTV USB (NTSC) FM",
	},
	[PINNA_PCTV_USB_NTSC_FM_V3] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Pinnacle Studio PCTV USB (NTSC) FM V3",
	},
	[PINNA_PCTV_USB_PAL_FM_V2] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_TEMIC_4009FR5_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Pinnacle Studio PCTV USB (PAL) FM V2",
	},
	[PINNA_PCTV_USB_NTSC_FM_V2] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_TEMIC_4039FR5_NTSC,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Pinnacle Studio PCTV USB (NTSC) FM V2",
	},
	[PINNA_PCTV_USB_PAL_FM_V3] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_TEMIC_4009FR5_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Pinnacle Studio PCTV USB (PAL) FM V3",
	},
	[PINNA_LINX_VD_IN_CAB_NTSC] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Pinnacle Studio Linx Video input cable (NTSC)",
	},
	[PINNA_LINX_VD_IN_CAB_PAL] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 2,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Pinnacle Studio Linx Video input cable (PAL)",
	},
	[PINNA_PCTV_BUNGEE_PAL_FM] = {
		.interface      = -1,
		.codec          = CODEC_SAA7113,
		.video_channels = 3,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 1,
		.radio          = 1,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_TEMIC_4009FR5_PAL,
		.x_offset       = 0,
		.y_offset       = 3,
		.dvi_yuv_override = 1,
		.dvi_yuv        = 7,
		.model_string   = "Pinnacle PCTV Bungee USB (PAL) FM",
	},
	[HPG_WINTV] = {
		.interface      = -1,
		.codec          = CODEC_SAA7111,
		.video_channels = 3,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 1,
		.radio          = 0,
		.vbi            = 1,
		.tuner          = 1,
		.tuner_type     = TUNER_PHILIPS_NTSC_M,
		.x_offset       = -1,
		.y_offset       = -1,
		.model_string   = "Hauppauge WinTv-USB",
	},
	[MICROCAM_NTSC] = {
		.interface      = -1,
		.codec          = CODEC_WEBCAM,
		.video_channels = 1,
		.video_norm     = V4L2_STD_NTSC,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 0,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 71,
		.y_offset       = 15,
		.model_string   = "Nogatech USB MicroCam NTSC (NV3000N)",
	},
	[MICROCAM_PAL] = {
		.interface      = -1,
		.codec          = CODEC_WEBCAM,
		.video_channels = 1,
		.video_norm     = V4L2_STD_PAL,
		.audio_channels = 0,
		.radio          = 0,
		.vbi            = 0,
		.tuner          = 0,
		.tuner_type     = 0,
		.x_offset       = 71,
		.y_offset       = 18,
		.model_string   = "Nogatech USB MicroCam PAL (NV3001P)",
	},
};
const int usbvision_device_data_size = ARRAY_SIZE(usbvision_device_data);

/* Supported Devices */

struct usb_device_id usbvision_table[] = {
	{ USB_DEVICE(0x0a6f, 0x0400), .driver_info = XANBOO },
	{ USB_DEVICE(0x050d, 0x0106), .driver_info = BELKIN_VIDEOBUS_II },
	{ USB_DEVICE(0x050d, 0x0207), .driver_info = BELKIN_VIDEOBUS },
	{ USB_DEVICE(0x050d, 0x0208), .driver_info = BELKIN_USB_VIDEOBUS_II },
	{ USB_DEVICE(0x0571, 0x0002), .driver_info = ECHOFX_INTERVIEW_LITE },
	{ USB_DEVICE(0x0573, 0x0003), .driver_info = USBGEAR_USBG_V1 },
	{ USB_DEVICE(0x0573, 0x0400), .driver_info = D_LINK_V100 },
	{ USB_DEVICE(0x0573, 0x2000), .driver_info = X10_USB_CAMERA },
	{ USB_DEVICE(0x0573, 0x2d00), .driver_info = HPG_WINTV_LIVE_PAL_BG },
	{ USB_DEVICE(0x0573, 0x2d01), .driver_info = HPG_WINTV_LIVE_PRO_NTSC_MN },
	{ USB_DEVICE(0x0573, 0x2101), .driver_info = ZORAN_PMD_NOGATECH },
	{ USB_DEVICE(0x0573, 0x3000), .driver_info = MICROCAM_NTSC },
	{ USB_DEVICE(0x0573, 0x3001), .driver_info = MICROCAM_PAL },
	{ USB_DEVICE(0x0573, 0x4100), .driver_info = NOGATECH_USB_TV_NTSC_FM },
	{ USB_DEVICE(0x0573, 0x4110), .driver_info = PNY_USB_TV_NTSC_FM },
	{ USB_DEVICE(0x0573, 0x4450), .driver_info = PV_PLAYTV_USB_PRO_PAL_FM },
	{ USB_DEVICE(0x0573, 0x4550), .driver_info = ZT_721 },
	{ USB_DEVICE(0x0573, 0x4d00), .driver_info = HPG_WINTV_NTSC_MN },
	{ USB_DEVICE(0x0573, 0x4d01), .driver_info = HPG_WINTV_PAL_BG },
	{ USB_DEVICE(0x0573, 0x4d02), .driver_info = HPG_WINTV_PAL_I },
	{ USB_DEVICE(0x0573, 0x4d03), .driver_info = HPG_WINTV_PAL_SECAM_L },
	{ USB_DEVICE(0x0573, 0x4d04), .driver_info = HPG_WINTV_PAL_D_K },
	{ USB_DEVICE(0x0573, 0x4d10), .driver_info = HPG_WINTV_NTSC_FM },
	{ USB_DEVICE(0x0573, 0x4d11), .driver_info = HPG_WINTV_PAL_BG_FM },
	{ USB_DEVICE(0x0573, 0x4d12), .driver_info = HPG_WINTV_PAL_I_FM },
	{ USB_DEVICE(0x0573, 0x4d14), .driver_info = HPG_WINTV_PAL_D_K_FM },
	{ USB_DEVICE(0x0573, 0x4d2a), .driver_info = HPG_WINTV_PRO_NTSC_MN },
	{ USB_DEVICE(0x0573, 0x4d2b), .driver_info = HPG_WINTV_PRO_NTSC_MN_V2 },
	{ USB_DEVICE(0x0573, 0x4d2c), .driver_info = HPG_WINTV_PRO_PAL },
	{ USB_DEVICE(0x0573, 0x4d20), .driver_info = HPG_WINTV_PRO_NTSC_MN_V3 },
	{ USB_DEVICE(0x0573, 0x4d21), .driver_info = HPG_WINTV_PRO_PAL_BG },
	{ USB_DEVICE(0x0573, 0x4d22), .driver_info = HPG_WINTV_PRO_PAL_I },
	{ USB_DEVICE(0x0573, 0x4d23), .driver_info = HPG_WINTV_PRO_PAL_SECAM_L },
	{ USB_DEVICE(0x0573, 0x4d24), .driver_info = HPG_WINTV_PRO_PAL_D_K },
	{ USB_DEVICE(0x0573, 0x4d25), .driver_info = HPG_WINTV_PRO_PAL_SECAM },
	{ USB_DEVICE(0x0573, 0x4d26), .driver_info = HPG_WINTV_PRO_PAL_SECAM_V2 },
	{ USB_DEVICE(0x0573, 0x4d27), .driver_info = HPG_WINTV_PRO_PAL_BG_V2 },
	{ USB_DEVICE(0x0573, 0x4d28), .driver_info = HPG_WINTV_PRO_PAL_BG_D_K },
	{ USB_DEVICE(0x0573, 0x4d29), .driver_info = HPG_WINTV_PRO_PAL_I_D_K },
	{ USB_DEVICE(0x0573, 0x4d30), .driver_info = HPG_WINTV_PRO_NTSC_MN_FM },
	{ USB_DEVICE(0x0573, 0x4d31), .driver_info = HPG_WINTV_PRO_PAL_BG_FM },
	{ USB_DEVICE(0x0573, 0x4d32), .driver_info = HPG_WINTV_PRO_PAL_I_FM },
	{ USB_DEVICE(0x0573, 0x4d34), .driver_info = HPG_WINTV_PRO_PAL_D_K_FM },
	{ USB_DEVICE(0x0573, 0x4d35), .driver_info = HPG_WINTV_PRO_TEMIC_PAL_FM },
	{ USB_DEVICE(0x0573, 0x4d36), .driver_info = HPG_WINTV_PRO_TEMIC_PAL_BG_FM },
	{ USB_DEVICE(0x0573, 0x4d37), .driver_info = HPG_WINTV_PRO_PAL_FM },
	{ USB_DEVICE(0x0573, 0x4d38), .driver_info = HPG_WINTV_PRO_NTSC_MN_FM_V2 },
	{ USB_DEVICE(0x0768, 0x0006), .driver_info = CAMTEL_TVB330 },
	{ USB_DEVICE(0x07d0, 0x0001), .driver_info = DIGITAL_VIDEO_CREATOR_I },
	{ USB_DEVICE(0x07d0, 0x0002), .driver_info = GLOBAL_VILLAGE_GV_007_NTSC },
	{ USB_DEVICE(0x07d0, 0x0003), .driver_info = DAZZLE_DVC_50_REV_1_NTSC },
	{ USB_DEVICE(0x07d0, 0x0004), .driver_info = DAZZLE_DVC_80_REV_1_PAL },
	{ USB_DEVICE(0x07d0, 0x0005), .driver_info = DAZZLE_DVC_90_REV_1_SECAM },
	{ USB_DEVICE(0x07f8, 0x9104), .driver_info = ESKAPE_LABS_MYTV2GO },
	{ USB_DEVICE(0x2304, 0x010d), .driver_info = PINNA_PCTV_USB_PAL },
	{ USB_DEVICE(0x2304, 0x0109), .driver_info = PINNA_PCTV_USB_SECAM },
	{ USB_DEVICE(0x2304, 0x0110), .driver_info = PINNA_PCTV_USB_PAL_FM },
	{ USB_DEVICE(0x2304, 0x0111), .driver_info = MIRO_PCTV_USB },
	{ USB_DEVICE(0x2304, 0x0112), .driver_info = PINNA_PCTV_USB_NTSC_FM },
	{ USB_DEVICE(0x2304, 0x0113), .driver_info = PINNA_PCTV_USB_NTSC_FM_V3 },
	{ USB_DEVICE(0x2304, 0x0210), .driver_info = PINNA_PCTV_USB_PAL_FM_V2 },
	{ USB_DEVICE(0x2304, 0x0212), .driver_info = PINNA_PCTV_USB_NTSC_FM_V2 },
	{ USB_DEVICE(0x2304, 0x0214), .driver_info = PINNA_PCTV_USB_PAL_FM_V3 },
	{ USB_DEVICE(0x2304, 0x0300), .driver_info = PINNA_LINX_VD_IN_CAB_NTSC },
	{ USB_DEVICE(0x2304, 0x0301), .driver_info = PINNA_LINX_VD_IN_CAB_PAL },
	{ USB_DEVICE(0x2304, 0x0419), .driver_info = PINNA_PCTV_BUNGEE_PAL_FM },
	{ USB_DEVICE(0x2400, 0x4200), .driver_info = HPG_WINTV },
	{ },    /* terminate list */
};

MODULE_DEVICE_TABLE(usb, usbvision_table);