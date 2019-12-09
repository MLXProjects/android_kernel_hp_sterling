/*
 * hpipaq214-wm97xx.c  --  hpipaq214 Continuous Touch screen driver
 *
 * Oliver Ford - copied from zylonite-wm97xx.c:
 * Copyright 2004, 2007, 2008 Wolfson Microelectronics PLC.
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 * Parts Copyright : Ian Molton <spyro@f2s.com>
 *                   Andrew Zabolotny <zap@homelink.ru>
 *                   Alvin Wong <alvinhochun@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * Notes:
 *     This is a wm97xx extended touch driver supporting interrupt driven
 *     and continuous operation on hpipaq214 (which has a WM9713 on board).
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/wm97xx.h>

#include <mach/hardware.h>
#include <mach/mfp.h>
#include <mach/regs-ac97.h>

#define IPAQ214_PENDOWN_IRQ_GPIO	89

struct continuous {
	u16 id;    /* codec id */
	u8 code;   /* continuous code */
	u8 reads;  /* number of coord reads per read cycle */
	u32 speed; /* number of coords per second */
};

#define WM_READS(sp) ((sp / HZ) + 1)

static const struct continuous cinfo[] = {
	{ WM9713_ID2, 0, WM_READS(94),  94  },
	{ WM9713_ID2, 1, WM_READS(120), 120 },
	{ WM9713_ID2, 2, WM_READS(154), 154 },
	{ WM9713_ID2, 3, WM_READS(188), 188 },
};

/* continuous speed index */
static int sp_idx;

/*
 * Pen sampling frequency (Hz) in continuous mode.
 */
static int cont_rate = 200;
module_param(cont_rate, int, 0);
MODULE_PARM_DESC(cont_rate, "Sampling rate in continuous mode (Hz)");

/*
 * Pressure readback.
 *
 * Set to 1 to read back pen down pressure
 */
static int pressure;
module_param(pressure, int, 0);
MODULE_PARM_DESC(pressure, "Pressure readback (1 = pressure, 0 = no pressure)");

/*
 * AC97 touch data slot.
 *
 * Touch screen readback data ac97 slot
 */
static int ac97_touch_slot = 5;
module_param(ac97_touch_slot, int, 0);
MODULE_PARM_DESC(ac97_touch_slot, "Touch screen data slot AC97 number");


static int penup=1;

/* flush AC97 slot 5 FIFO machines */
static void wm97xx_acc_pen_up(struct wm97xx *wm)
{
	int i;
	//dev_info(wm->dev,"pen_up\n");
	penup=1;

	msleep(1);

	for (i = 0; i < 16; i++)
		MODR;
}

static int wm97xx_acc_pen_down(struct wm97xx *wm)
{
	const int DATA_X=0x1, DATA_Y=0x2, DATA_P=0x4;
	u16 x, y, p = 0x100;
	u16 in;
	int status;
	int reads = 0;
	static u16 lastin, tries;
	int i;

	status=pressure?0:DATA_P;

	/* When the AC97 queue has been drained we need to allow time
	 * to buffer up samples otherwise we end up spinning polling
	 * for samples.  The controller can't have a suitably low
	 * threashold set to use the notifications it gives.
	 */
	msleep(1);

	/* getting duplicated data continuously is assumed to be outdated */
	in = MODR;
	if (in == lastin) {
		tries++;
		if (tries > 2) {
			tries = 0;
			//dev_info(wm->dev,"tell you pen_up\n",tries);
			penup=1;
			return RC_PENUP;
		}
		//dev_info(wm->dev,"same? tries=%d\n",tries);
		return RC_AGAIN;
	}

	/* When I was testing the accelerated touchscreen with debug
	 * information, I found that somehow the first few coordinates
	 * could be something similar to previous touch. I didn't
	 * investigate the issue, but by experimenting I get at most
	 * 6 wrong coordinates at first, so I decided to drop the 6
	 * possibly wrong coordinates at start. My later testing shows
	 * that there are no wrong coordinates in 100 tries to draw on
	 * the touchscreen.
	 */
	if(penup){
		penup=0;
		for (i = 0; i < 6; i++){
			/* some sort of debug things
			switch(in&WM97XX_ADCSRC_MASK){
				case WM97XX_ADCSEL_X:
					x=in&0xfff;
					dev_info(wm->dev,"1dropped X = %d\n",x);
					break;
				case WM97XX_ADCSEL_Y:
					y=in&0xfff;
					dev_info(wm->dev,"1dropped Y = %d\n",y);
					break;
				case WM97XX_ADCSEL_PRES:
					p=in&0xfff;
					break;
			}
			*/
			in=MODR;
		}
	}

	do {
		if (in == lastin) {
			in = MODR;
			/* getting duplicated data, so no more touches */
			if (in == lastin)
				goto up;
		}
		lastin = in;

		/* what data had I just received? */
		switch(in&WM97XX_ADCSRC_MASK){
			case WM97XX_ADCSEL_X:
				x=in&0xfff;
				status|=DATA_X;
				//dev_info(wm->dev,"X = %d\n",x);
				break;
			case WM97XX_ADCSEL_Y:
				y=in&0xfff;
				/* take Y only when X was taken */
				if(status&DATA_X){
					status|=DATA_Y;
					//dev_info(wm->dev,"Y = %d\n",y);
				//}else{ 
					//dev_info(wm->dev,"dropped Y = %d\n",y);
				}
				break;
			case WM97XX_ADCSEL_PRES:
				p=in&0xfff;
				status|=DATA_P;
				break;
		}

		/* do we have all data? */
		if(status==(DATA_X|DATA_Y|DATA_P)){
			tries=0;
			input_report_abs(wm->input_dev, ABS_X, x);
			input_report_abs(wm->input_dev, ABS_Y, y);
			input_report_abs(wm->input_dev, ABS_PRESSURE, p);
			input_report_key(wm->input_dev, BTN_TOUCH, (p != 0));
			input_sync(wm->input_dev);
			//dev_info(wm->dev,"sent X = %d Y = %d\n",x,y);
			status=pressure?0:DATA_P;
			reads++; /* not sure how this works in original code */
		}
		in=MODR;
	} while (reads < cinfo[sp_idx].reads);
up:
	//dev_info(wm->dev,"return\n");
	return RC_PENDOWN | RC_AGAIN;
}

static int wm97xx_acc_startup(struct wm97xx *wm)
{
	int idx;

	/* check we have a codec */
	if (wm->ac97 == NULL)
		return -ENODEV;

	/* Go you big red fire engine */
	for (idx = 0; idx < ARRAY_SIZE(cinfo); idx++) {
		if (wm->id != cinfo[idx].id)
			continue;
		sp_idx = idx;
		if (cont_rate <= cinfo[idx].speed)
			break;
	}
	wm->acc_rate = cinfo[sp_idx].code;
	wm->acc_slot = ac97_touch_slot;
	dev_info(wm->dev,
		 "hpipaq214 accelerated touchscreen driver, %d samples/sec\n",
		 cinfo[sp_idx].speed);

	return 0;
}

static void wm97xx_irq_enable(struct wm97xx *wm, int enable)
{
	if (enable)
		enable_irq(wm->pen_irq);
	else
		disable_irq_nosync(wm->pen_irq);
}

static struct wm97xx_mach_ops hpipaq214_mach_ops = {
	.acc_enabled	= 1,
	.acc_pen_up	= wm97xx_acc_pen_up,
	.acc_pen_down	= wm97xx_acc_pen_down,
	.acc_startup	= wm97xx_acc_startup,
	.irq_enable	= wm97xx_irq_enable,
	.irq_gpio	= WM97XX_GPIO_2,
};

static int hpipaq214_wm97xx_probe(struct platform_device *pdev)
{
	struct wm97xx *wm = platform_get_drvdata(pdev);
	int gpio_touch_irq = IPAQ214_PENDOWN_IRQ_GPIO;

	/*if (cpu_is_pxa320())
		gpio_touch_irq = mfp_to_gpio(MFP_PIN_GPIO15);
	else
		gpio_touch_irq = mfp_to_gpio(MFP_PIN_GPIO26);
	*/

	wm->pen_irq = IRQ_GPIO(gpio_touch_irq);
	set_irq_type(IRQ_GPIO(gpio_touch_irq), IRQ_TYPE_EDGE_BOTH);

	wm97xx_config_gpio(wm, WM97XX_GPIO_13, WM97XX_GPIO_IN,
			   WM97XX_GPIO_POL_HIGH,
			   WM97XX_GPIO_STICKY,
			   WM97XX_GPIO_WAKE);
	wm97xx_config_gpio(wm, WM97XX_GPIO_2, WM97XX_GPIO_OUT,
			   WM97XX_GPIO_POL_HIGH,
			   WM97XX_GPIO_NOTSTICKY,
			   WM97XX_GPIO_NOWAKE);

	return wm97xx_register_mach_ops(wm, &hpipaq214_mach_ops);
}

static int hpipaq214_wm97xx_remove(struct platform_device *pdev)
{
	struct wm97xx *wm = platform_get_drvdata(pdev);

	wm97xx_unregister_mach_ops(wm);

	return 0;
}

static struct platform_driver hpipaq214_wm97xx_driver = {
	.probe	= hpipaq214_wm97xx_probe,
	.remove	= hpipaq214_wm97xx_remove,
	.driver	= {
		.name	= "wm97xx-touch",
	},
};

static int __init hpipaq214_wm97xx_init(void)
{
	return platform_driver_register(&hpipaq214_wm97xx_driver);
}

static void __exit hpipaq214_wm97xx_exit(void)
{
	platform_driver_unregister(&hpipaq214_wm97xx_driver);
}

module_init(hpipaq214_wm97xx_init);
module_exit(hpipaq214_wm97xx_exit);

/* Module information */
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_DESCRIPTION("wm97xx continuous touch driver for hpipaq214");
MODULE_LICENSE("GPL");
