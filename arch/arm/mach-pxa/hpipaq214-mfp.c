/*
 * linux/arch/arm/mach-pxa/hpipaq214-mfp.c
 *
 * Multi-Function Pin Support for the Hewlett-Packard iPAQ 21x.
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * These are all configured as I've found them elsewhere to ensure 
 * the suspend/resume works properly.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <mach/mfp-pxa300.h>

#define MAX_SLOTS	3
#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)


/********************************************************
* Multi Function Pin                                    *
********************************************************/

static mfp_cfg_t hpipaq214_mfp_cfg[] __initdata = {
	/* FLASH */
	MFP_CFG_X(DF_INT_RnB, AF0, DS10X, FLOAT),
	MFP_CFG_X(DF_nWE, AF1, DS10X, PULL_HIGH),
	MFP_CFG_X(DF_nRE_nOE, AF1, DS10X, PULL_HIGH),
	MFP_CFG_DRV(nBE0, AF0, DS01X),
	MFP_CFG_DRV(nBE1, AF0, DS01X),
	MFP_CFG_X(DF_ALE_nWE, AF1, DS10X, PULL_LOW),
	MFP_CFG_DRV(DF_ADDR0, AF0, DS01X),
	MFP_CFG_DRV(DF_ADDR1, AF0, DS01X),
	MFP_CFG_DRV(DF_ADDR2, AF0, DS01X),
	MFP_CFG_DRV(DF_ADDR3, AF0, DS01X),
	MFP_CFG_X(DF_IO0, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO8, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO1, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO9, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO2, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO10, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO3, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO11, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_CLE_nOE, AF0, DS10X, PULL_LOW),
	MFP_CFG_DRV(nLUA, AF0, DS01X),
	MFP_CFG_X(DF_nCS0, AF1, DS10X, PULL_HIGH),
	MFP_CFG_DRV(nLLA, AF0, DS01X),
	MFP_CFG_X(DF_IO4, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO12, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO5, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO13, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO6, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO14, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO7, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_IO15, AF1, DS08X, PULL_HIGH),
	MFP_CFG_X(DF_nCS1, AF1, DS06X, PULL_HIGH),

	/* LCD */	
	MFP_CFG_DRV(GPIO54, AF1, DS03X),	//GPIO54_LCD_LDD_0,
	MFP_CFG_DRV(GPIO55, AF1, DS03X),	//GPIO55_LCD_LDD_1,
	MFP_CFG_DRV(GPIO56, AF1, DS03X),	//GPIO56_LCD_LDD_2,
	MFP_CFG_DRV(GPIO57, AF1, DS03X),	//GPIO57_LCD_LDD_3,
	MFP_CFG_DRV(GPIO58, AF1, DS03X),	//GPIO58_LCD_LDD_4,
	MFP_CFG_DRV(GPIO59, AF1, DS03X),	//GPIO59_LCD_LDD_5,
	MFP_CFG_DRV(GPIO60, AF1, DS03X),	//GPIO60_LCD_LDD_6,
	MFP_CFG_DRV(GPIO61, AF1, DS03X),	//GPIO61_LCD_LDD_7,
	MFP_CFG_DRV(GPIO62, AF1, DS03X),	//GPIO62_LCD_LDD_8,
	MFP_CFG_DRV(GPIO63, AF1, DS03X),	//GPIO63_LCD_LDD_9,
	MFP_CFG_DRV(GPIO64, AF1, DS03X),	//GPIO64_LCD_LDD_10,
	MFP_CFG_DRV(GPIO65, AF1, DS03X),	//GPIO65_LCD_LDD_11,
	MFP_CFG_DRV(GPIO66, AF1, DS03X),	//GPIO66_LCD_LDD_12,
	MFP_CFG_DRV(GPIO67, AF1, DS03X),	//GPIO67_LCD_LDD_13,
	MFP_CFG_DRV(GPIO68, AF1, DS03X),	//GPIO68_LCD_LDD_14,
	MFP_CFG_DRV(GPIO69, AF1, DS03X),	//GPIO69_LCD_LDD_15,
	MFP_CFG_DRV(GPIO72, AF1, DS03X),	//GPIO72_LCD_FCLK,
	GPIO73_LCD_LCLK,
	GPIO74_LCD_PCLK,
	GPIO75_LCD_BIAS,
	GPIO76_LCD_VSYNC,
	GPIO127_LCD_CS_N,

	// KEYPAD
	// right config for wake up source?
 	GPIO121_KP_MKOUT_0,	//MFP_CFG_LPM(GPIO121, AF1, DRIVE_HIGH)	Y
	GPIO122_KP_MKOUT_1,	//MFP_CFG_LPM(GPIO122, AF1, DRIVE_HIGH)	Y
	GPIO123_KP_MKOUT_2,	//MFP_CFG_LPM(GPIO123, AF1, DRIVE_HIGH)	Y
	GPIO115_KP_MKIN_0 | MFP_LPM_EDGE_BOTH,	//MFP_CFG_LPM(GPIO115, AF1, FLOAT)	Y
	GPIO116_KP_MKIN_1 | MFP_LPM_EDGE_BOTH,	//MFP_CFG_LPM(GPIO116, AF1, FLOAT)	Y
	GPIO117_KP_MKIN_2 | MFP_LPM_EDGE_BOTH,	//MFP_CFG_LPM(GPIO117, AF1, FLOAT)	Y
	GPIO118_KP_MKIN_3 | MFP_LPM_EDGE_BOTH,	//MFP_CFG_LPM(GPIO118, AF1, FLOAT)	Y 


	//. AC97
	GPIO23_AC97_nACRESET,
//	GPIO24_AC97_SYSCLK,	//doesn't appear to be a sysclk
	GPIO29_AC97_BITCLK,
	GPIO25_AC97_SDATA_IN_0,
	GPIO27_AC97_SDATA_OUT,
	GPIO28_AC97_SYNC,

	// MMC1 - config DRIVE_LOW for D3
	MFP_CFG_X(GPIO3, AF4, DS03X, DRIVE_LOW),	// GPIO3_MMC1_DAT0 
	MFP_CFG_X(GPIO4, AF4, DS03X, DRIVE_LOW),	// GPIO3_MMC1_DAT1
	MFP_CFG_X(GPIO5, AF4, DS03X, DRIVE_LOW),	// GPIO3_MMC1_DAT2
	MFP_CFG_X(GPIO6, AF4, DS03X, DRIVE_LOW),	// GPIO3_MMC1_DAT3
	MFP_CFG_X(GPIO7, AF4, DS03X, DRIVE_LOW),	// GPIO7_MMC1_CLK
	MFP_CFG_X(GPIO8, AF4, DS03X, DRIVE_LOW),	// GPIO8_MMC1_CMD
 	
	// MMC2: SDIO Marvell 8686 Wifi (libertas) - force pull high in D0, drive low in D3
	MFP_CFG_X(GPIO9, AF4, DS13X, DRIVE_LOW) | MFP_PULL_HIGH,		// GPIO9_MMC2_DAT0
	MFP_CFG_X(GPIO10, AF4, DS13X, DRIVE_LOW) | MFP_PULL_HIGH,		// GPIO10_MMC2_DAT1
	MFP_CFG_X(GPIO11, AF4, DS13X, DRIVE_LOW) | MFP_PULL_HIGH,		// GPIO11_MMC2_DAT2
	MFP_CFG_X(GPIO12, AF4, DS13X, DRIVE_LOW) | MFP_PULL_HIGH,		// GPIO12_MMC2_DAT3
	MFP_CFG_X(GPIO13, AF4, DS13X, DRIVE_LOW) | MFP_PULL_HIGH,		// GPIO13_MMC2_CLK
	MFP_CFG_X(GPIO14, AF4, DS13X, DRIVE_LOW) | MFP_PULL_HIGH,		// GPIO14_MMC2_CMD

	//PWM
	GPIO19_PWM2_OUT,	// Backlight drive - PWM2 	

	//Unknown GPIOs used to turn the BCM2045 BlueTooth chip on
	MFP_CFG_DRV(GPIO53, AF0, DS03X),
	MFP_CFG_DRV(GPIO71, AF0, DS03X),
	MFP_CFG_DRV(GPIO81, AF0, DS03X),
	MFP_CFG_DRV(GPIO114, AF0, DS01X),
	MFP_CFG_DRV(GPIO124, AF0, DS03X),

	//Unknown GPIOs used to turn the Marvell SD8686 WiFi chip on
	MFP_CFG_DRV(GPIO4_2, AF0, DS01X),
	MFP_CFG_DRV(GPIO97, AF0, DS04X),
	MFP_CFG_DRV(GPIO99, AF0, DS01X),
	MFP_CFG_DRV(GPIO100, AF0, DS04X),
	MFP_CFG_DRV(GPIO103, AF0, DS04X),

	// Misc GPIOs
	MFP_CFG_DRV(GPIO3_2, AF0, DS04X),	// The BLUE LED (Bluetooth/wifi on light) 
	MFP_CFG_DRV(GPIO5_2, AF0, DS04X),	// The green LED in same place as blue one
	MFP_CFG_DRV(GPIO89, AF0, DS03X),	// Pen-down IRQ for the touchscreen device
	GPIO15_GPIO,				// MMC1 Card Detect	
	GPIO102_GPIO,				// MMC1 Write Protect
	MFP_CFG_DRV(GPIO104, AF0, DS13X),	// WM9713 Power?
	MFP_CFG_DRV(GPIO96, AF0, DS13X),	// Speaker Enable
	MFP_CFG_DRV(GPIO98, AF0, DS13X),	// Headphone Enable (Jack and 24-pin)
	MFP_CFG_DRV(GPIO7_2, AF0, DS13X),	// Headphone Jack/24Pin Select
	GPIO95_GPIO | MFP_PULL_HIGH,		// Headphone Jack Detect.
	/* GPIO95 Should be set to "Override AF pull setting, but turn off both pulls"/
	 *  Unfortunately, linux can't set this from here at present but making sure 
	 *  the pull-down is off seems to work so we jsut force the pull-up on.
	 *    ... groan */

	MFP_CFG_DRV(GPIO8_2, AF0, DS01X),	// USB Host VBus enable
	GPIO10_2_GPIO,				// USB Host Related
 	GPIO9_2_GPIO,				// 24-pin jack detect
 	GPIO82_GPIO,				// mini-usb / 24-pin client select
	GPIO90_GPIO,				// USB Host Reset

 	GPIO125_GPIO,				// Quicklogic power/chip enable?
 	GPIO111_GPIO,				// Quicklogic power/chip enable?
 	GPIO20_GPIO,				// Quicklogic unknown ('low-power' related)

 	GPIO101_GPIO,				// Compact Flash Detect 1
 	GPIO70_GPIO,				// Compact Flash Detect 2

	//Charging GPIOs (MAX8677)
	GPIO91_GPIO,				// ! DC-In OK
	GPIO120_GPIO | MFP_LPM_DRIVE_LOW,	// ! Charge Enable - Ensure enabled in LPM
	GPIO105_GPIO,				// Charging
	GPIO106_GPIO,				// Prequal
	GPIO107_GPIO,				// Battery Connected

 	//Other set to gpios as wince start parition
 	MFP_CFG(GPIO6_2, AF0),
 	GPIO113_GPIO,
 	GPIO17_GPIO,
 	GPIO93_GPIO,
 	GPIO18_GPIO,

/*	//Quick capture, unused but should set for LPM config
	GPIO46_CI_DD_7,	
	GPIO49_CI_MCLK,
	GPIO50_CI_PCLK,
	GPIO51_CI_VSYNC,
	GPIO52_CI_HSYNC,
*/
	GPIO126_OW_DQ | MFP_LPM_FLOAT,	//One-wire bus - GPIO is connected to middle battery pin
	MFP_CFG_X(GPIO109, AF1, DS01X, DRIVE_LOW), // GPIO109_UART3_TXD but with drive low in D3
	MFP_CFG_X(GPIO110, AF1, DS01X, DRIVE_LOW), // GPIO110_UART3_RXD but with drive low in D3

	//SSP and GPIO112 used for LCD up/down code
	GPIO85_SSP1_SCLK,
	GPIO86_SSP1_FRM,
	GPIO87_SSP1_TXD,
	GPIO88_SSP1_RXD,
	GPIO112_GPIO,	//use in LCD power up/down

	// FFUART used for bluetooth
	GPIO77_UART1_RXD,
	GPIO78_UART1_TXD,
	GPIO79_UART1_CTS, 
	GPIO84_UART1_RTS, 
//*/

};


void  __init hpipaq214_init_mfp(void){

	pxa3xx_mfp_config(ARRAY_AND_SIZE(hpipaq214_mfp_cfg));
}
