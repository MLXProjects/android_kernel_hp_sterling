/*
 * linux/arch/arm/mach-pxa/hpipaq214.c
 *
 * Support for the Hewlett-Packard iPAQ 21x series.
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/pda_power.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ds1wm.h>
#include <linux/gpio_keys.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/system.h>
#include <mach/gpio.h>
#include <mach/audio.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxa3xx-regs.h>

#include "generic.h"
#include "devices.h"
#include "clock.h"

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

extern void hpipaq214_init_mfp(void);		// defined in hpipaq214-mfp.c
extern void hpipaq214_init_usb(void);		// defined in hpipaq214-usb.c
extern void hpipaq214_init_lcd(void);		// defined in hpipaq214-lcd.c
extern void hpipaq214_init_mmc(void);		// defined in hpipaq214-mmc.c
extern void hpipaq214_init_nand(void);		// defined in hpipaq214-nand.c

/********************************************************
* DS1WM (1Wire Bus Master)                              *
********************************************************/
#ifdef CONFIG_W1_MASTER_DS1WM

extern struct platform_device hpipaq214_device_ds1wm;

struct clk clk_hpipaq214_ds1wm = {
	.ops	= &clk_pxa3xx_cken_ops,
	.rate	= 28000000,
	.cken	= CKEN_1WIRE,
	.delay	= 0,
};

static struct clk_lookup hpipaq214_clkregs[] = {
	{
		.clk		= &clk_hpipaq214_ds1wm,
		.dev_id		= "ds1wm",
		.con_id		= NULL,
	}
};

static int hpipaq214_ds1wm_enable(struct platform_device *pdev)
{
	printk("Enabling DS1WM (CLK on)\n");

	clk_cken_enable(&clk_hpipaq214_ds1wm);

	return 0;
}

static int hpipaq214_ds1wm_disable(struct platform_device *pdev)
{
	printk("Disabling DS1WM (CLK off)\n");

	clk_cken_disable(&clk_hpipaq214_ds1wm);

	return 0;
}


static struct ds1wm_driver_data hpipaq214_ds1wm_info = {
	.clock_rate 	= 28000000, 
	.active_high	= 1,
};

static struct resource hpipaq214_resource_ds1wm[] = {
	[0] = {
		.start	= 0x41b00000,
		.end	= 0x41b00014,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_1WIRE,
		.end	= IRQ_1WIRE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct mfd_cell hpipaq214_ds1wm_cell = {
	.name          = "ds1wm",
	.enable        = hpipaq214_ds1wm_enable,
	.disable       = hpipaq214_ds1wm_disable,
//	.suspend)(struct platform_device *dev);
//	.resume)(struct platform_device *dev);
	.driver_data   = &hpipaq214_ds1wm_info,
	.resources	= hpipaq214_resource_ds1wm,
	.num_resources	= ARRAY_SIZE(hpipaq214_resource_ds1wm),
};

struct platform_device hpipaq214_device_ds1wm = {
	.name		= "ds1wm",
	.id		= -1,
	.resource	= hpipaq214_resource_ds1wm,
	.num_resources	= ARRAY_SIZE(hpipaq214_resource_ds1wm),
	.dev		=  {
		.platform_data  = &hpipaq214_ds1wm_cell,
	}
};

static void __init hpipaq214_init_ds1wm(void)
{
	clks_register(hpipaq214_clkregs, ARRAY_SIZE(hpipaq214_clkregs));

	platform_device_register(&hpipaq214_device_ds1wm);
}
#else
static inline void hpipaq214_init_ds1wm(void) {}
#endif /* CONFIG_W1_MASTER_DS1WM */

/********************************************************
* Platform devices                                      *
********************************************************/

static struct platform_device hpipaq214_bluetooth = {
	.name		= "hpipaq214-bluetooth",
	.id		= -1,
};

static struct platform_device hpipaq214_wifi = {
	.name		= "hpipaq214-wifi",
	.id		= -1,
};

static struct platform_device hpipaq214_powerkey = {
	.name		= "hpipaq214-powerkey",
	.id		= -1,
};

//Other devices are added by arch/arm/mach-pxa/devices.c because we have a PXA3xx
static struct platform_device *hpipaq214_devices[] __initdata = {
	&hpipaq214_bluetooth,
	&hpipaq214_wifi,
	&hpipaq214_powerkey,
};


/****************************
* AC Power Supply           *
****************************/
#if defined(CONFIG_PDA_POWER) || defined(CONFIG_PDA_POWER_MODULES)

#define GPIO_DOK 	91	/* ! DC-In OK (MAX8677) */
#define GPIO_CEN 	120	/* ! Charge Enable (MAX8677) */
#define GPIO_CHG 	105	/* Charging (MAX8677) */
#define GPIO_PREQ 	106	/* Prequal (MAX8677) */
#define GPIO_BATT_CONN 	107	/* Battery Connected (??) */
//NOTE: CEN pulled low in LPM by MFP CFG.

static int hpipaq214_power_init(struct device *dev)
{
	int ret;

	ret = gpio_request(GPIO_DOK, "!DC-in OK");
	ret += gpio_request(GPIO_CEN, "!Charge Enable");
/*	ret += gpio_request(GPIO_CHG, "Charging");
	ret += gpio_request(GPIO_PREQ, "Prequal charge");
	ret += gpio_request(GPIO_BATT_CONN, "Battery Connected"); */
	
	ret += gpio_direction_input(GPIO_DOK);
	ret += gpio_direction_output(GPIO_CEN, 0); //leave ther charger always on
/*	ret += gpio_direction_input(GPIO_CHG);
	ret += gpio_direction_input(GPIO_PREQ);
	ret += gpio_direction_input(GPIO_BATT_CONN);*/
	
	return ret;
}

static void hpipaq214_power_exit(struct device *dev)
{
	gpio_free(GPIO_DOK);
	gpio_free(GPIO_CEN);
/*	gpio_free(GPIO_CHG);
	gpio_free(GPIO_PREQ);
	gpio_free(GPIO_BATT_CONN);	*/
}

static int hpipaq214_power_ac_online(void)
{
	return (gpio_get_value(GPIO_DOK) == 0);
}

static char *hpipaq214_ac_supplied_to[] = {
	"ds2760-battery.0",
};

static struct pda_power_pdata hpipaq214_power_data = {
	.init			= hpipaq214_power_init,
	.is_ac_online		= hpipaq214_power_ac_online,
	.exit			= hpipaq214_power_exit,
	.supplied_to		= hpipaq214_ac_supplied_to,
	.num_supplicants	= ARRAY_SIZE(hpipaq214_ac_supplied_to),
};

static struct resource hpipaq214_power_resource[] = {
	{
		.name		= "ac",
		.start		= gpio_to_irq(GPIO_DOK),
		.end		= gpio_to_irq(GPIO_DOK),
		.flags		= IORESOURCE_IRQ |
				  IORESOURCE_IRQ_HIGHEDGE |
				  IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device hpipaq214_power_device = {
	.name			= "pda-power",
	.id			= -1,
	.dev.platform_data	= &hpipaq214_power_data,
	.resource		= hpipaq214_power_resource,
	.num_resources		= ARRAY_SIZE(hpipaq214_power_resource),
};
static void __init hpipaq214_init_power(void)
{
	int ret;
	ret = platform_device_register(&hpipaq214_power_device);	
	if (ret)
		printk(KERN_ERR "unable to register pda_power device\n");
}
#else
static inline void hpipaq214_init_power(void) {}
#endif /* CONFIG_PDA_POWER || CONFIG_PDA_POWER_MODULES */


/****************************
* Keypad                    *
****************************/
#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULES)
static unsigned int hpipaq214_matrix_key_map[] = {

	/* KEY(row, col, key_code) */
	KEY(2, 0, KEY_0),	//Func key A - left most
	KEY(3, 0, KEY_1),	//Func key B
	KEY(1, 0, KEY_2),	//Func key C
	KEY(1, 1, KEY_3),	//Func key D - right most
	KEY(0, 2, KEY_4),	//The record button
	
	KEY(0, 0, KEY_5),	//Unknown or N/C 1
	KEY(0, 1, KEY_6),	//Unknown or N/C 2
		
	KEY(1, 2, KEY_LEFT),
	KEY(2, 1, KEY_UP),
	KEY(2, 2, KEY_RIGHT),
	KEY(3, 1, KEY_DOWN),
	KEY(3, 2, KEY_ENTER),	//Nav centre key
	
};

static struct pxa27x_keypad_platform_data hpipaq214_keypad_info = {
	.matrix_key_rows	= 4,
	.matrix_key_cols	= 3,
	.matrix_key_map		= hpipaq214_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(hpipaq214_matrix_key_map),

	.enable_rotary0		= 0,

	.debounce_interval	= 30,
};

static void __init hpipaq214_init_keypad(void)
{
	pxa_set_keypad_info(&hpipaq214_keypad_info);
}
#else
static inline void hpipaq214_init_keypad(void) {}
#endif


/********************************************************
* AC97                                                  *
********************************************************/
#if defined(CONFIG_SND_PXA3XX_SOC_HPIPAQ214)
int wm9713_power_gpio = 104;

void hpipaq214_audio_suspend(void *priv){
	if(wm9713_power_gpio >= 0){ //didn't get the GPIO, can't do anything
		printk(KERN_INFO "hpipaq214_audio_suspend(), gpio%i -> off",wm9713_power_gpio);
		gpio_set_value(wm9713_power_gpio, 0);	
	}
}

void hpipaq214_audio_resume(void *priv){
	if(wm9713_power_gpio >= 0){ //didn't get the GPIO, can't do anything
		printk(KERN_INFO "hpipaq214_audio_resume, gpio%i -> on",wm9713_power_gpio);
		gpio_set_value(wm9713_power_gpio, 1);
	}
}

static pxa2xx_audio_ops_t hpipaq214_audio_ops = {
	.suspend = hpipaq214_audio_suspend,
	.resume = hpipaq214_audio_resume,
};

static void __init hpipaq214_init_audio(void)
{
	int ret;

	ret = gpio_request(wm9713_power_gpio, "WM9713 power?");	
	if(ret){
		printk(KERN_ERR "Unable to register WM9713/audio gpio (%i)\n",wm9713_power_gpio);
		wm9713_power_gpio = -1;
	}else
		gpio_direction_output(wm9713_power_gpio, 1); //keep it on for now!
	
	printk("hpipaq214_init_audio: %x\n",(unsigned int)&hpipaq214_audio_ops);

 	pxa_set_ac97_info(&hpipaq214_audio_ops);

}
#else
static inline void hpipaq214_init_audio(void) {}
#endif /* CONFIG_SND_PXA3XX_SOC_HPIPAQ214 */

/********************************************************
* GPIO SWITCH (Headphone detect)                        *
********************************************************/
#if defined(CONFIG_SWITCH_GPIO) || defined(CONFIG_SWITCH_GPIO_MODULE)
static struct gpio_switch_platform_data hpipaq214_hpswitch_info = {
	.name		= "h2w",
	.gpio		= 95,
	.state_on	= "1",
	.state_off	= "0",
};

static struct platform_device hpipaq214_device_hpswitch = {
	.name		= "switch-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &hpipaq214_hpswitch_info,
	}
};

static void __init hpipaq214_init_hpswitch(void)
{
	platform_device_register(&hpipaq214_device_hpswitch);
}
#else
static inline void hpipaq214_init_hpswitch(void) {}
#endif

/********************************************************
* GPIO LEDs                                             *
********************************************************/
#if defined(CONFIG_LEDS_GPIO) || defined(CONFIG_LEDS_GPIO_MODULE)
static struct gpio_led hpipaq214_leds[] = {
	[0] = {
		.name			= "hpipaq214:green:1",
		.default_trigger	= "default-on",
		.gpio			= 5,
	},
	[1] = {
		.name			= "hpipaq214:blue:2",
		.default_trigger	= "mmc0",
		.gpio			= 3,
	},
};

static struct gpio_led_platform_data hpipaq214_leds_info = {
	.leds		= hpipaq214_leds,
	.num_leds	= ARRAY_SIZE(hpipaq214_leds),
};

static struct platform_device hpipaq214_device_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &hpipaq214_leds_info,
	}
};

static void __init hpipaq214_init_leds(void)
{
	platform_device_register(&hpipaq214_device_leds);
}
#else
static inline void hpipaq214_init_leds(void) {}
#endif

//initlisation of stuff I don't actually know where to put yet
static void __init hpipaq214_init_misc(void){

	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);
	//hpipaq214-bt does the ffuart


	//if(!gpio_request(3, "Blue LED (left)"))
	//	gpio_direction_output(3, 0);
	//if(!gpio_request(5, "Green LED (left)"))
	//	gpio_direction_output(5, 1); //turn it on to say we're alive!		

	//Make sure QuickLogic chip enables are off to save power since we don't use it
	// saves ~50%!!! Though, this seems to drain power in D3 overnight
	// TODO: investigate....
	if(!gpio_request(111, "Quicklogic Chip Enable 1?"))
		gpio_direction_output(111, 0);
	if(!gpio_request(125, "Quicklogic Chip Enable 2?"))
		gpio_direction_output(125, 0);
	if(!gpio_request(20, "Quicklogic Unknown GPIO 20"))
		gpio_direction_output(20, 1);
	if(!gpio_request(17, "Unknown GPIO 17"))
		gpio_direction_output(17, 1);	//set high on this system
	if(!gpio_request(17, "Unknown GPIO 6"))
		gpio_direction_output(6, 1);	//set high on this system

	
}

#define IPAQ214_BOOTLOADER_NORMAL	0x00000000
#define IPAQ214_BOOTLOADER_RECOVERY	0x0000F600

static void hpipaq214_restart(char str, const char *cmd){
	// tell U-boot to reboot into recovery
	// HACK: as RTC registers are preserved, we use them.
	if(cmd && strncmp(cmd,"recovery",8)==0){
		// (RYAR1 & 0x001FFE00) == 0x0000F600
		__REG(0x4090001C)=0x0000F7FF; // magic number for recovery
		__REG(0x40900018)=0xFFFFFFFF;
	}else{
		__REG(0x4090001C)=0x00000000; // set it to anything else
		__REG(0x40900018)=0x00000000; // to prevent unintended recovery
	}

	arm_machine_restart('g', cmd);
}

static void hpipaq214_power_off(void){

	// clear certain registers to prevent booting recovery
	__REG(0x4090001C)=0x00000000; // set it to anything else
	__REG(0x40900018)=0x00000000; // to prevent unintended recovery

	// let's get into deep sleep...
	__asm__(
		"mov	r0, #0x07;"
		"mcr	p14, 0, r0, c7, c0, 0;"
	);
	for(;;);
}

static void __init hpipaq214_init(void)
{	
	hpipaq214_init_mfp();
	hpipaq214_init_leds();
	hpipaq214_init_misc();
	hpipaq214_init_lcd();
        hpipaq214_init_mmc();
	hpipaq214_init_keypad();
	hpipaq214_init_nand();
	hpipaq214_init_usb();
	hpipaq214_init_hpswitch();
	hpipaq214_init_audio();
	hpipaq214_init_power();
	hpipaq214_init_ds1wm();

	//add the various platform devices
	platform_add_devices(hpipaq214_devices, ARRAY_SIZE(hpipaq214_devices));

	// set reset mode and power off pointer
	pm_power_off = hpipaq214_power_off;
	arm_pm_restart = hpipaq214_restart;
}

MACHINE_START(HPIPAQ214, "Hewlett-Packard iPAQ 214")
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq	= pxa3xx_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= hpipaq214_init,
MACHINE_END
