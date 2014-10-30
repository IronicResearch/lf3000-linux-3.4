/*
 * (C) Copyright 2009
 * jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef __DEVICES_H__
#define __DEVICES_H__

#include <linux/platform_device.h>
#include <linux/i2c-gpio.h>

#include "clkdev.h"
#include "display.h"
#include "dmadev.h"

/* cpu */
#define DEV_NAME_UART 			"nxp-uart"		// pl0115 (amba-pl011.c)
#define	DEV_NAME_FB				"nxp-fb"
#define	DEV_NAME_DISP			"nxp-disp"
#define	DEV_NAME_LCD			"nxp-lcd"
#define	DEV_NAME_LVDS			"nxp-lvds"
#define	DEV_NAME_HDMI			"nxp-hdmi"
#define	DEV_NAME_RESCONV		"nxp-resconv"
#define	DEV_NAME_MIPI			"nxp-mipi"
#define	DEV_NAME_PCM			"nxp-pcm"
#define	DEV_NAME_I2S			"nxp-i2s"
#define	DEV_NAME_SPDIF_TX		"nxp-spdif-tx"
#define	DEV_NAME_SPDIF_RX		"nxp-spdif-rx"
#define	DEV_NAME_I2C			"nxp-i2c"
#define	DEV_NAME_NAND			"lf2000-nand"
#define	DEV_NAME_KEYPAD			"nxp-keypad"
#define	DEV_NAME_SDHC			"nxp-sdhc"
#define	DEV_NAME_PWM			"nxp-pwm"
#define	DEV_NAME_TIMER			"nxp-timer"
#define	DEV_NAME_SOC_PWM		"nxp-soc-pwm"
#define DEV_NAME_GPIO           "nxp-gpio"
#define DEV_NAME_RTC            "nxp-rtc"
#define	DEV_NAME_GMAC			"nxp-gmac"
#define	DEV_NAME_MPEGTSI		"nxp-mpegtsi"
#define	DEV_NAME_MALI			"nxp-mali"
#define	DEV_NAME_DIT			"nxp-deinterlace"
#define	DEV_NAME_PPM			"nxp-ppm"
#define	DEV_NAME_VIP			"nxp-vip"
#define	DEV_NAME_CODA			"nxp-coda"
#define	DEV_NAME_USB2HOST		"nxp-usb2h"
#define	DEV_NAME_CRYPTO			"nxp-crypto"
#define	DEV_NAME_SCALER			"nxp-scaler"
#define	DEV_NAME_PDM			"nxp-pdm"
#define	DEV_NAME_SPI			"nxp-spi"
#define	DEV_NAME_CPUFREQ		"nxp-cpufreq"
#define ADC_DEV_NAME                    "nx-adc"

/*
 *	Frame buffer platform data and display controller
 */
struct nxp_fb_plat_data {
	int 	module;				/* 0: primary, 1: secondary */
	int 	layer;				/* RGB 0, 1, 2 */
	unsigned int format;		/* RGB format */
	unsigned int bgcolor;
	int		bitperpixel;		/* bit per pixel */
	int		x_resol_max;		/* x resolution for change resolution */
	int		y_resol_max;		/* y resolution for change resolution  */
	int		x_resol;			/* x resolution */
	int		y_resol;			/* y resolution */
	int		buffers;			/* set 2 when support pan, default 1 */
	/* for direct fb region */
	unsigned int fb_mem_base;
	unsigned int fb_mem_end;
	/* for lcd dpi (default 0) */
	long	lcd_with_mm;		/* with (mm), default 0 */
	long	lcd_height_mm;		/* height (mm), default 0 */
	int		skip_pan_vsync;
	struct 	disp_vsync_info	*vsync;
};

struct nxp_syncgen_plat_data {
	struct disp_vsync_info *vsync;
	struct disp_syncgen_param *par;
};

struct nxp_lcd_plat_data {
	int display_in;
	struct disp_vsync_info *vsync;
	struct disp_lcd_param *par;
};

struct nxp_lvds_plat_data {
	int display_in;
	struct disp_vsync_info *vsync;
	struct disp_lvds_param *par;
};

struct nxp_hdmi_plat_data {
    int preset;			/* 0 = 1280 * 720p, 1=1920 * 1080p */
	int display_in;
};

struct nxp_resconv_plat_data {
	int display_in;
	struct disp_vsync_info *vsync;
	struct disp_resconv_param *par;
};


/*
 *	Sound platform data
 */
#include <sound/pcm.h>

/* I2S */
struct nxp_i2s_plat_data {
	int		master_mode;
	int 	trans_mode;					/* 0:I2S, 1:Left 2:Right justified */
	int 	sample_rate;
	int 	sample_bit;					/* support only 8, 16, 24 */
	int 	frame_bit;					/* support only 32, 48 */
	int 	LR_pol_inv;
	int		pre_supply_mclk;			/* codec require mclk out, before codec initialize */
    bool 	(*dma_filter)(struct dma_chan *chan, void *filter_param);
	const char *dma_play_ch;
	const char *dma_capt_ch;
};

/* SPDIF */
struct nxp_spdif_plat_data {
	int sample_rate;
	int hdmi_out;
    bool (*dma_filter)(struct dma_chan *chan, void *filter_param);
	const char *dma_ch;
};

/* sound DAI (I2S/SPDIF and codec interface) */
struct nxp_snd_jack_pin {
	int	support;
	int	detect_level;
	int detect_io;
	int debounce_time;
};

struct nxp_snd_dai_plat_data {
	int		snd_device;					/* 0=i2s, 1=spdif */
	int 	snd_dev_ch;
	int 	codec_master;
	unsigned int sample_rate;
	unsigned int pcm_format;		/* SNDRV_PCM_FMTBIT_S16_LE, SNDRV_PCM_FMTBIT_S24_LE, .. (include/sound/pcm.h) */
	struct nxp_snd_jack_pin hp_jack;
	struct nxp_snd_jack_pin mic_jack;
};

/*
 *	I2C platform data
 */
struct nxp_i2c_plat_data {
	int  port;
	int  irq;
	struct i2c_gpio_platform_data *gpio;
	unsigned int base_addr;
	long rate;
	int	 io_mode;
};

/*
 *	Touch calibration platform data
 */
struct nxp_ts_cali_plat_data {
	int		touch_points;		/* support touch points num when multi touch */
	int 	x_resol;
	int 	y_resol;
	int		rotate;				/* 0, 90, 180, 270 */
	long	pointercal[10]; 	/* calibration value (tslib) */
};

/*
 *	CPU Freq platform data
 */
struct nxp_cpufreq_plat_data {
	int pll_dev;					/* core pll : 0, 1, 2, 3 */
	unsigned long (*freq_table)[2];	/* [freq KHz].[u volt] */
	int	table_size;
	long max_cpufreq;		/* unit Khz */
	long max_retention; 	/* unit msec */
	long rest_cpufreq;		/* unit Khz */
	long rest_retention;  	/* unit msec */
};

/*
 *	SDHC platform data
 */
#include <linux/mmc/dw_mmc.h>
#include <linux/mmc/host.h>

extern void __init nxp_dwmci_platform_device_register(struct dw_mci_board *mci,
						int id, unsigned long rate);

/*
 *	Keypad platform data
 */
struct nxp_key_plat_data {
	int				   bt_count;
	unsigned int  	 * bt_io;
	unsigned int  	 * bt_code;
	unsigned int  	 * bt_long;			/* long press action */
	unsigned int  	 * bt_code_type;	/* short key type 0=short key, 1 = long key */
	unsigned int  	 * bt_long_type;	/* long  key type 0=short key, 1 = long key */
	unsigned int  	 bt_rep;			/* key repeat 1 = on , 0 = off */

	int				   bt_delay;		/* short key delay */
	int				   bt_long_delay;	/* long  key delay */
	struct input_dev * bt_device;
};


/*
 * NAND platform data
 */
struct nxp_nand_plat_data {
	struct mtd_partition * parts;
	int nr_parts;
	int chip_delay;
};

/*
 *	VMem platform data
 */
struct nxp_vmem_plat_data {
	int		resv_task_num;
	char **	resv_task_name;
};

/*
 *  MPEGTSIF platform data
 */
struct nxp_mp2ts_plat_data {
	unsigned char cap_ch_num;
    unsigned char op_mode;      // NX_MPEGTSIF_OPMODE_MASTER,       NX_MPEGTSIF_OPMODE_SLAVE
	unsigned char clock_pol;	// NX_MPEGTSIF_CLOCKPOL_FALLING,    NX_MPEGTSIF_CLOCKPOL_RISING
	unsigned char data_pol;	    // NX_MPEGTSIF_DATAPOL_LOW,         NX_MPEGTSIF_DATAPOL_HIGH
	unsigned char sync_pol;	    // NX_MPEGTSIF_SYNCPOL_FALLING,     NX_MPEGTSIF_SYNCPOL_RISING
	unsigned char error_pol;	// NX_MPEGTSIF_ERRORPOL_FALLING,    NX_MPEGTSIF_ERRORPOL_RISING
	unsigned char data_width;	// NX_MPEGTSIF_DATAWIDTH_8BIT,      NX_MPEGTSIF_DATAWIDTH_1BIT
//	unsigned int  word_cnt;		// 1 ~ 64
};

#endif	/* __DEVICES_H__ */

