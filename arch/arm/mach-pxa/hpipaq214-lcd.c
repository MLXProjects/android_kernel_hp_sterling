/*
 * linux/arch/arm/mach-pxa/hpipaq214-lcd.c
 *
 * Liquid Crystal Display Support for the HP iPAQ 21x.
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * LCD and backlight support including use of SSP to enable LCD.
 * There are some settings that arn't quite correct and cause the
 * image to occasionally stick ~3/4 of the screen to the left/right.
 *
 * Also needs some work to reduce the flashing caused by turning
 * the backlight on before the LCD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/pwm_backlight.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/pxafb.h>
#include <mach/ssp.h>
#include <mach/regs-lcd.h>

#include "devices.h"

#if defined(CONFIG_FB_PXA) || defined(CONFIG_FB_PXA_MODULES)

int gpio_lcd_setup = 112;

// This must be sent down SSP1 before the LCD is enabled.
static uint32_t ssp_data_on[] = {
	0x00007003, 0x00007200, 0x00007000, 0x00007210, 
	0x00007010, 0x000072af, 0x00007011, 0x00007200, 
	0x00007012, 0x00007200, 0x00007013, 0x00007206, 
	0x00007014, 0x00007206, 0x00007015, 0x00007270, 
	0x00007016, 0x0000721f, 0x00007017, 0x00007207, 
	0x00007018, 0x00007207, 0x00007019, 0x00007233, 
	0x00007002, 0x00007200, 0x00007001, 0x00007210, 
	0x00007005, 0x00007224, 0x00007006, 0x00007245, 
	0x0000701a, 0x00007202, 0x0000701b, 0x00007251, 
	0x00007020, 0x00007224, 0x00007021, 0x0000720b, 
	0x00007022, 0x00007219, 0x00007023, 0x0000720c, 
	0x00007024, 0x00007207, 0x00007025, 0x00007219, 
	0x00007026, 0x00007213, 0x00007027, 0x00007210, 
	0x00007028, 0x00007202, 0x00007029, 0x00007202, 
	0x0000702a, 0x00007202, 0x0000702b, 0x00007204, 
	0x0000702c, 0x0000720b, 0x0000702d, 0x00007219, 
	0x0000702e, 0x0000720c, 0x0000702f, 0x00007207, 
	0x00007030, 0x00007219, 0x00007031, 0x00007213, 
	0x00007032, 0x00007210, 0x00007033, 0x00007202, 
	0x00007034, 0x00007202, 0x00007035, 0x00007202, 
	0x00007050, 0x00007202, 0x00007053, 0x00007200, 
	0x00007054, 0x00007244, 0x00007055, 0x00007244, 
	0x00007056, 0x0000720a, 0x00007059, 0x0000720b, 
	0x0000705a, 0x00007204, 0x0000705b, 0x000072ca, 
	0x0000705c, 0x00007204, 0x0000705d, 0x0000720a, 
	0x0000705e, 0x00007210, 0x0000705f, 0x0000721d, 
	0x00007062, 0x00007230, 0x00007063, 0x0000721d, 
	0x00007066, 0x00007250, 0x00007067, 0x0000721d, 
	0x00007070, 0x0000726d, 0x00007071, 0x00007214, 
	0x00007076, 0x000072ff, 0x00007079, 0x00007202, 
	0x00007000, 0x00007200
};

static uint32_t ssp_data_off[] = {
	0x00007002, 0x00007201
};

static void ssp_send_sequence(uint32_t *data, int len){
	int i, ret;
	int lcd_ssp_port = 1; //1 based
	static struct ssp_dev ssp;

	ret = ssp_init(&ssp, lcd_ssp_port, SSP_NO_IRQ);
	if(ret){
		printk("Error initialising the SSP port.\n");
		return;
	}

	//setup: 
	//  CTRL 0: disabled, no interrupts, data size = 16bit, 
	//  CTRL 1: Recv Threshold = 4, Trans Threshold = 8, SPH=1, SPO=1
	//  SSPSP = ? = 0
	//  speed = 0x081,  ( or'ed with CTRL1)
	ssp_config(&ssp, 0x00c0000f, 0x00001218, 0x00000000, 0x00008100); 
	ssp_enable(&ssp);

	for(i=0;i<len;i++){
		ret = ssp_write_word(&ssp, data[i]);
		if(ret){
			printk("ssp_write_word: Error (timeout) writing to SSP.\n");
			return;
		}
		udelay(50); //delay 50us -probly not neccesary with the slightly better SSP comms
		ssp_flush(&ssp);
		if(ret){
			printk("ssp_flush: Error (timeout) writing to SSP.\n");
			return;
		}
	}

	ssp_exit(&ssp);	
}

static void hpipaq214_lcd_onoff(int on, struct fb_var_screeninfo *var)
{	//presumably power, but who knows

	if(gpio_lcd_setup < 0) //didn't get the GPIO, can't do anything
		return;

	if (on) {
		//gpio twiddle
		gpio_set_value(gpio_lcd_setup, 1);
		mdelay(2);
		gpio_set_value(gpio_lcd_setup, 0);
		mdelay(20);
		gpio_set_value(gpio_lcd_setup, 1);
		mdelay(20);

		ssp_send_sequence(ssp_data_on, sizeof(ssp_data_on) / 4);
		mdelay(250);

	}else{
		ssp_send_sequence(ssp_data_off, sizeof(ssp_data_off) / 4);
		mdelay(50);
	}
}

static struct platform_pwm_backlight_data hpipaq214_backlight_data = {
	.pwm_id		= 2,
	.max_brightness	= 255,
	.dft_brightness	= 96,
	.pwm_period_ns	= 10000,
};

static struct platform_device hpipaq214_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent = &pxa27x_device_pwm0.dev,
		.platform_data	= &hpipaq214_backlight_data,
	},
};

static struct pxafb_mode_info hpipaq214_mode = {
	.xres			= 480,
	.yres			= 640,
	.vsync_len		= 7,
	.hsync_len		= 7,
	.upper_margin		= 4,
	.lower_margin		= 3,
	.left_margin		= 2,
	.right_margin		= 6,
	.sync			= 0,
	.bpp			= 16,
	.pixclock		= 50000,
};


static struct pxafb_mach_info hpipaq214_lcd_info = {
	.modes			= &hpipaq214_mode,
	.num_modes		= 1,
	.lcd_conn		= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
	.lccr4			= LCCR4_PCDDIV, /* set PCDDIV to get more accurate pixclock */
	.pxafb_lcd_power	= hpipaq214_lcd_onoff,
	.video_mem_size		= 2 * 1024 * 1024, /* 2 MB, (we have 128MB in total) */
	//.pxafb_backlight_power	= hpipaq214_backlight_power,
};

void __init hpipaq214_init_lcd(void)
{
	int err;

	platform_device_register(&hpipaq214_backlight_device);

	err = gpio_request(gpio_lcd_setup, "LCD setup gpio");
	if(err){
		printk("Unable to register LCD setup gpio (%i)\n",gpio_lcd_setup);
		gpio_lcd_setup = -1;
	}else{		
		gpio_direction_output(gpio_lcd_setup, 1); //keep it on for now!
	}

	set_pxa_fb_info(&hpipaq214_lcd_info);
}
#else
void  __init hpipaq214_init_lcd(void) {};
#endif /* CONFIG_FB_PXA || CONFIG_FB_PXA_MODULES */
