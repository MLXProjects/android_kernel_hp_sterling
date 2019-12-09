/*
 * linux/arch/arm/mach-pxa/hpipaq214-wifi-mod.c
 *
 * Driver for Wireless power and MMC interfacing for the HP iPAQ 21x
 * in Android (module)
 *
 * Copyright (C) 2011 Alvin Wong <alvinhochun-at-gmail-com>
 *
 */

#include <linux/module.h>

extern void hpipaq214_wifi_force_rescan(void);
extern void hpipaq214_wifi_power_off(void);
extern void hpipaq214_wifi_power_on(void);

static int __init hpipaq214_wifi_mod_init(void)
{
	printk(KERN_INFO "HP iPAQ 21x Wireless Control Driver (Android module)\n");
	/* In order to make wifi on when module init */
	hpipaq214_wifi_power_on(); //dev);
	hpipaq214_wifi_force_rescan(); //dev);

	return 0;
}

static void __exit hpipaq214_wifi_mod_exit(void)
{
	/* In order to make wifi off when module exit */
	hpipaq214_wifi_power_off(); //dev);
	hpipaq214_wifi_force_rescan(); //dev);
}

module_init(hpipaq214_wifi_mod_init);
module_exit(hpipaq214_wifi_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alvin Wong  <alvinhochun-at-gmail-com>");
MODULE_DESCRIPTION("HP iPAQ 21x Wireless Control Driver (Android module)");
