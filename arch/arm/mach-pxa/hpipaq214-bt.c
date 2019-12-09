/*
 * linux/arch/arm/mach-pxa/hpipaq214-bt.c
 *
 * Driver for bluetooth power for HP iPAQ 21x
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

#include <mach/gpio.h>
#include <mach/uart.h>
#include <mach/mfp-pxa300.h>
#include "generic.h"

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

static int state; //should be part in platform device data, not global to driver, but we can only have 1 anyway

#ifdef DEBUG
static void dumpAll(void){ //debugging
	int i;
	int gpio[] = { 53, 71, 79, 81, 84, 114, 124 };	
	printk(KERN_DEBUG "BT GPIOS: ");
	for(i=0;i<7;i++){		
		printk(KERN_DEBUG "%i=%s ",gpio[i], gpio_get_value(gpio[i]) ? "on" : "off");
	}
	printk(KERN_DEBUG "\n");
}
#else
static void dumpAll(void){}
#endif

static mfp_cfg_t rts_cts_as_uart[] = {
	GPIO79_UART1_CTS, 
	GPIO84_UART1_RTS,
};

static mfp_cfg_t rts_cts_as_gpio[] = {
	GPIO79_GPIO,
	GPIO84_GPIO,
};


static void hpipaq214_bt_power_off(struct device *dev){	
	gpio_set_value(81,0);
	gpio_set_value(3,0);
	gpio_set_value(53,0);
	gpio_set_value(71,0);
	gpio_set_value(124,0);
	gpio_set_value(114,0);
	state = 0;
	dumpAll();

	dev_info(dev, "Bluetooth off\n");
}

static void hpipaq214_bt_power_on(struct device *dev){
	if(gpio_request(79, "FFUART CTS (BlueTooth)") ||
	   gpio_request(84, "FFUART RTS (BlueTooth)")){
		dev_err(dev, "Unable to get FFUART CTS/RTS control gpios");
		state = -2;
		return;
	}
	dumpAll();
	gpio_direction_input(79);
	gpio_direction_output(84, 1); 
	
	//reconfig RTS and CTS as GPIOs for this bit
	pxa3xx_mfp_config(ARRAY_AND_SIZE(rts_cts_as_gpio));
	dumpAll();
	
	hpipaq214_bt_power_off(dev);	//everything off
	mdelay(10); dumpAll();
	
	gpio_set_value(53, 1);
	gpio_set_value(71, 1);
	gpio_set_value(124, 1);
	mdelay(1); dumpAll();
		
	gpio_set_value(81, 0);
	gpio_set_value(84, 1);		// RTS	off

	gpio_set_value(114, 1);
	mdelay(2); dumpAll();

	gpio_set_value(114, 0);
	mdelay(2); dumpAll();

	gpio_set_value(114, 1);
	gpio_set_value(81, 1);
	mdelay(500); dumpAll();

	gpio_set_value(84, 0);		// RTS on
	mdelay(5); dumpAll();

	if( !gpio_get_value(79) ){	//check CTS status (active low)
		dev_info(dev,"Bluetooth on and ready\n");	
		state = 1;
	}else{
		dev_err(dev, "ERROR: Bluetooth not stating clear to send\n");	
		hpipaq214_bt_power_off(dev);
		state = -2;
	}

	dumpAll();
	gpio_free(79);
	gpio_free(84);
	
	pxa3xx_mfp_config(ARRAY_AND_SIZE(rts_cts_as_uart));
	dumpAll();
}

static ssize_t hpipaq214_bt_show_btstate(struct device *dev,
				struct device_attribute *attr, char *buf){	
	strcpy(buf, (state < 0) ? "error" : (state > 0 ? "1" : "0"));
	return strlen(buf) + 1;	
}

static ssize_t hpipaq214_bt_store_btstate(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count){
	
	if(buf[0] == '1'){
		hpipaq214_bt_power_on(dev);
	}else if(buf[0] == '0'){
		hpipaq214_bt_power_off(dev);
	}
	
	return count;
}

DEVICE_ATTR(btstate, (S_IRUGO | S_IWUGO), hpipaq214_bt_show_btstate, hpipaq214_bt_store_btstate);

/* UART open/close callbacks */
static int hpipaq214_bt_uart_open(struct device *dev, void *data)
{
	struct device *btdev = data;
	dev_info(btdev, "Detected FFUART open, switching on BT\n");	
	hpipaq214_bt_power_on(btdev);
	if(state != 1){
		dev_err(btdev, "Bluetooth switchon failed, failing port open\n");
		return -EIO;
	}
	return 0;
	
}

static void hpipaq214_bt_uart_close(struct device *dev, void *data)
{
	struct device *btdev = data;
	dev_info(btdev, "Detected FFUART close, switching off BT\n");
	hpipaq214_bt_power_off(btdev);
}

struct pxauart_platform_data hpipaq214_bt_ffuart_info = {
	.open = hpipaq214_bt_uart_open,
	.close = hpipaq214_bt_uart_close,
};
/***********************************/

static int hpipaq214_bt_probe(struct platform_device *pdev)
{	
	int bt_gpios[] = { 53, 71, 81, 114, 124 };
	int i;

	if(device_create_file(&pdev->dev, &dev_attr_btstate))
		dev_err(&pdev->dev, "Unable to create sysfs entries\n");
	state = 0;

	for(i=0;i<5;i++)
		if(!gpio_request(bt_gpios[i], "Bluetooth Control GPIO"))
			gpio_direction_output(bt_gpios[i], 0);

	/* Start with it off */
	hpipaq214_bt_power_off(&pdev->dev);

	hpipaq214_bt_ffuart_info.data = &pdev->dev;
	pxa_set_ffuart_info(&hpipaq214_bt_ffuart_info);


	return 0;
}

static int hpipaq214_bt_remove(struct platform_device *pdev)
{
	hpipaq214_bt_power_off(&pdev->dev);	
	return 0;
}

static int hpipaq214_bt_suspend(struct platform_device *pdev, pm_message_t pmstate)
{
	if(state > 0)
		dev_info(&pdev->dev, "Forcing BT off for suspend");

	hpipaq214_bt_power_off(&pdev->dev);
	
	return 0;
}

static struct platform_driver bluetooth_driver = {
	.driver		= {	
		.name	= "hpipaq214-bluetooth",
	},
	.probe		= hpipaq214_bt_probe,
	.suspend	= hpipaq214_bt_suspend,
	.remove		= hpipaq214_bt_remove,
	
};

static int __init hpipaq214_bt_init(void)
{
	printk(KERN_INFO "HP iPAQ 21x Bluetooth Control Driver\n");
	platform_driver_register(&bluetooth_driver);
	
	return 0;
}

static void __exit hpipaq214_bt_exit(void)
{
        platform_driver_unregister(&bluetooth_driver);
}

module_init(hpipaq214_bt_init);
module_exit(hpipaq214_bt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oliver Ford  <ipaqcode-at-oliford-co-uk>");
MODULE_DESCRIPTION("HP iPAQ 21x Bluetooth Control Driver");
