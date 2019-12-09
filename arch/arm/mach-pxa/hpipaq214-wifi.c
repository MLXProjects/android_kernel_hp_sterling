/*
 * linux/arch/arm/mach-pxa/hpipaq214-wifi.c
 *
 * Driver for Wireless power and MMC interfacing for the HP iPAQ 21x
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/mmc.h>
#include <mach/gpio.h>
#include <mach/uart.h>

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

/* The following three should be in platform device data or something,
 *   not global to driver, but we can only have 1 of this device anyway */
static int state;					/* Current State */
static irq_handler_t mmc2_detect_int = NULL;	/* Stored fake IRQ handler called */
static void * mmc2_detect_int_data = NULL;		/*  to force rescan of MMC2 */

#ifdef DEBUG
static void dumpAll(void){ //debugging
	int i;
	int gpio[] = { 4, 97, 99, 100, 103 };	
	printk(KERN_DEBUG "WIFI GPIOS: ");
	for(i=0;i<7;i++){		
		printk(KERN_DEBUG "%i=%s ",gpio[i], gpio_get_value(gpio[i]) ? "on" : "off");
	}
	printk(KERN_DEBUG "\n");
}
#else
static void dumpAll(void){}
#endif

void hpipaq214_wifi_force_rescan(void){ //struct device *dev){

	if (mmc2_detect_int != NULL){
		//dev_info(dev, "Forcing rescan of MMC2 for wifi...");
		printk("<1> Forcing rescan of MMC2 for wifi...\n");
		mmc2_detect_int(-1, mmc2_detect_int_data);
	}else{
		//dev_err(dev, "MMC2 detect IRQ handler pointer not set,"
		//		" cannot force scan of MMC2 for wifi\n");
		printk("<1> MMC2 detect IRQ handler pointer not set, cannot force scan of MMC2 for wifi\n");
	}
}
EXPORT_SYMBOL(hpipaq214_wifi_force_rescan);

void hpipaq214_wifi_power_off(void){ //struct device *dev){
	gpio_set_value(100, 0);
	gpio_set_value(97, 0);
	gpio_set_value(4, 0);
	gpio_set_value(99, 0);
	//dev_info(dev, "Wifi now off.\n");
	printk("<1> Wifi now off.\n");

	state = 0;
	dumpAll();
}
EXPORT_SYMBOL(hpipaq214_wifi_power_off);

void hpipaq214_wifi_power_on(void){ //struct device *dev){
	gpio_set_value(4, 1);
	gpio_set_value(103, 0);
	gpio_set_value(103, 1);
	gpio_set_value(99, 1);
	mdelay(30);
	gpio_set_value(103, 0);
	gpio_set_value(100, 1);
	gpio_set_value(97, 1);
	mdelay(10);
	gpio_set_value(103, 1);
	//dev_info(dev, "Wifi now hopefully on.\n");
	printk("<1> Wifi now hopefully on.\n");

	state = 1;
	dumpAll();
}
EXPORT_SYMBOL(hpipaq214_wifi_power_on);

static ssize_t hpipaq214_wifi_show_wifistate(struct device *dev,
				struct device_attribute *attr, char *buf){	
	strcpy(buf, (state < 0) ? "error" : (state > 0 ? "on" : "off"));
	return strlen(buf) + 1;	
}

static ssize_t hpipaq214_wifi_store_wifistate(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count){

	if(buf[0] == 'o'){
		if(buf[1] == 'n') {
			hpipaq214_wifi_power_on(); //dev);
			hpipaq214_wifi_force_rescan(); //dev);
		}else if(buf[1] == 'f' && buf[2] == 'f') {
			hpipaq214_wifi_power_off(); //dev);
			hpipaq214_wifi_force_rescan(); //dev);
		}
	}
	
	return count;
}

DEVICE_ATTR(wifistate, (S_IRUGO | S_IWUGO), hpipaq214_wifi_show_wifistate, hpipaq214_wifi_store_wifistate);

static int hpipaq214_wifi_probe(struct platform_device *pdev)
{	
	int wifi_gpios[] = { 4, 97, 99, 100, 103 };
	int i;

	if(device_create_file(&pdev->dev, &dev_attr_wifistate))
		dev_err(&pdev->dev, "Unable to create sysfs entries\n");
	state = 0;

	for(i=0;i<5;i++)
		if(!gpio_request(wifi_gpios[i], "Wireless Control GPIO"))
			gpio_direction_output(wifi_gpios[i], 0);

	/* Start with it off */
	hpipaq214_wifi_power_off(); //&pdev->dev);

	return 0;
}

static int hpipaq214_wifi_mci2_init(struct device *dev,
			     irq_handler_t hpipaq214_detect_int,
			     void *data)
{
	mmc2_detect_int = hpipaq214_detect_int;
	mmc2_detect_int_data = data;

	return 0;
}

static void hpipaq214_wifi_mci2_exit(struct device *dev, void *data)
{
	/* anything to do?? */
}

static void hpipaq214_wifi_mci2_setpower(struct device *dev, unsigned int vdd){
	/* printk("mci2(wifi) setpower %i\n",vdd); */
	/* anything to do?? */
}

static struct pxamci_platform_data hpipaq214_wifi_mci2_platform_data = {
	.detect_delay_ms	= 20,
	.ocr_mask	= ~0, //report them all, we'll see what it asks for
	.init 		= hpipaq214_wifi_mci2_init,
	.exit		= hpipaq214_wifi_mci2_exit,	
	.setpower 	= hpipaq214_wifi_mci2_setpower,
	.gpio_card_detect = -1,
	.gpio_card_ro	= -1,
	.gpio_power	= -1,

};


static int hpipaq214_wifi_remove(struct platform_device *pdev)
{
	hpipaq214_wifi_power_off(); //&pdev->dev);	
	return 0;
}

static int hpipaq214_wifi_suspend(struct platform_device *pdev, pm_message_t pmstate)
{
	if(state > 0)
		dev_info(&pdev->dev, "Forcing WiFi off for suspend");

	hpipaq214_wifi_power_off(); //&pdev->dev);
	
	return 0;
}

static struct platform_driver wifi_driver = {
	.driver		= {	
		.name	= "hpipaq214-wifi",
	},
	.probe		= hpipaq214_wifi_probe,
	.suspend	= hpipaq214_wifi_suspend,
	.remove		= hpipaq214_wifi_remove,
	
};

static int __init hpipaq214_wifi_init(void)
{
	printk(KERN_INFO "HP iPAQ 21x Wireless Control Driver\n");
	platform_driver_register(&wifi_driver);

	/* Register info the MMC2 */
	pxa3xx_set_mci2_info(&hpipaq214_wifi_mci2_platform_data);	
	
	return 0;
}

static void __exit hpipaq214_wifi_exit(void)
{
        platform_driver_unregister(&wifi_driver);
}

module_init(hpipaq214_wifi_init);
module_exit(hpipaq214_wifi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oliver Ford  <ipaqcode-at-oliford-co-uk>");
MODULE_DESCRIPTION("HP iPAQ 21x Wireless Control Driver");
