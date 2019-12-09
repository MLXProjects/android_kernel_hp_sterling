/*
 * linux/arch/arm/mach-pxa/hpipaq214-mmc.c
 *
 * Multimedia Card and SDIO Support for the HP iPAQ 214
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>

#include <asm/gpio.h>
#include <mach/mmc.h>

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

/********************************************************
* Multi Media Card Reader                               *
********************************************************/
int gpio_mmc1_wp = 102; //write protect
int gpio_mmc1_cd = 15; //card detect

#if defined(CONFIG_MMC)
static int hpipaq214_mci_ro(struct device *dev)
{
	//struct platform_device *pdev = to_platform_device(dev);

	return gpio_get_value(gpio_mmc1_wp);
}

static int hpipaq214_mci_init(struct device *dev,
			     irq_handler_t hpipaq214_detect_int,
			     void *data)
{
	//struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq;

#ifdef CONFIG_MMC_DEBUG
	printk("MMC: hpipaq214_mci_init complete\n");
#endif
	return 0; //none of this needs doing now

	cd_irq = gpio_to_irq(gpio_mmc1_cd);	

	/*
	 * setup GPIO for MMC controller
	 */
	err = gpio_request(gpio_mmc1_cd, "MMC card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_mmc1_cd);

	err = gpio_request(gpio_mmc1_wp, "MMC write protect");
	if (err)
		goto err_request_wp;
	gpio_direction_input(gpio_mmc1_wp);

	err = request_irq(cd_irq, hpipaq214_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}


err_request_irq:
	gpio_free(gpio_mmc1_wp);
err_request_wp:
	gpio_free(gpio_mmc1_cd);
err_request_cd:
	return err;
}

static void hpipaq214_mci_exit(struct device *dev, void *data)
{
	//struct platform_device *pdev = to_platform_device(dev);
	int cd_irq;

	cd_irq = gpio_to_irq(gpio_mmc1_cd);
	free_irq(cd_irq, data);
	gpio_free(gpio_mmc1_cd);
	gpio_free(gpio_mmc1_wp);
} 

/* This stuff is the power I2C to the MAX8660.
 * It's requried for powering up/down the MMC1 card.
 * TODO: This works as it is but is nasty code.
 *         As support for the power I2C turns up in the 
 *         mainline kernel, it should be changed to use that.
 */

#define PWR_I2C_4	__REG(0x40f500c4)
#define PWR_I2C_8	__REG(0x40f500c8)	
#define PWR_I2C_C	__REG(0x40f500cc)

static int hpipaq214_pw_c0_wait_on_cc(void){
	int i;
	unsigned long l;
	for(i=0;i<2500;i++){
		l = PWR_I2C_C;
		
		if( (l & 0x40) == 0x40 ){
			PWR_I2C_C = l | 0x40;
			return 0; //done
		}
		mdelay(1);
	}
	return 1; //timeout
}

static int hpipaq214_pw_c0_send_byte(unsigned char byte1, unsigned char byte2){
	unsigned long l;

	printk("pwr_c0 sending %02x %02x... ",byte1,byte2);
	PWR_I2C_4 = 0x68;	
	PWR_I2C_8 = (PWR_I2C_8 & ~0x0f) | 0x09;
	if(hpipaq214_pw_c0_wait_on_cc())
		goto timeout;

	PWR_I2C_4 = byte1;	
	PWR_I2C_8 = (PWR_I2C_8 & ~0x03) | 0x08;
	if(hpipaq214_pw_c0_wait_on_cc())
		goto timeout;


	PWR_I2C_4 = byte2;	
	l = (PWR_I2C_8 & ~0x01) | 0x08 | 0x02;
	/*if( Zero flag?? )
		l |= 0x02;
	else
		l &= ~0x02;*/
	PWR_I2C_8 = l;
	if(hpipaq214_pw_c0_wait_on_cc())
		goto timeout;
	
	PWR_I2C_8 = PWR_I2C_8 & ~0x02;	

	printk("OK\n");
	return 0; //ok
timeout:
	printk("Timed out\n");
	return 1;	
}

static void hpipaq214_mci_setpower(struct device *dev, unsigned int vdd){
	int ret;
	int v1,v2,setting;
	
	printk("+hpipaq214_mci_setpower(%i)\n",vdd);

	if(vdd == 0){
		printk("%s: MMC Power off\n", __func__);
		hpipaq214_pw_c0_send_byte(0x12,0x05); // VCC_CARD1 off
		printk("-hpipaq214_mci_setpower OK\n");
		return;
	}
	
	switch(1 << vdd){
		case MMC_VDD_165_195: v1=1;v2=8; setting = 0x00; break; /* VDD1.65 - 1.95 */
		case MMC_VDD_20_21: v1=2;v2=0; setting = 0x02; break; /* VDD2.0 ~ 2.1 */
		case MMC_VDD_21_22: v1=2;v2=1; setting = 0x03; break; /* VDD2.1 ~ 2.2 */
		case MMC_VDD_22_23: v1=2;v2=2; setting = 0x04; break; /* VDD2.2 ~ 2.3 */
		case MMC_VDD_23_24: v1=2;v2=3; setting = 0x05; break; /* VDD2.3 ~ 2.4 */
		case MMC_VDD_24_25: v1=2;v2=4; setting = 0x06; break; /* VDD2.4 ~ 2.5 */
		case MMC_VDD_25_26: v1=2;v2=5; setting = 0x07; break; /* VDD2.5 ~ 2.6 */
		case MMC_VDD_26_27: v1=2;v2=6; setting = 0x08; break; /* VDD2.6 ~ 2.7 */
		case MMC_VDD_27_28: v1=2;v2=7; setting = 0x09; break; /* VDD2.7 ~ 2.8 */
		case MMC_VDD_28_29: v1=2;v2=8; setting = 0x0a; break; /* VDD2.8 ~ 2.9 */
		case MMC_VDD_29_30: v1=2;v2=9; setting = 0x0b; break; /* VDD2.9 ~ 3.0 */
		case MMC_VDD_30_31: v1=3;v2=0; setting = 0x0c; break; /* VDD3.0 ~ 3.1 */
		case MMC_VDD_31_32: v1=3;v2=1; setting = 0x0d; break; /* VDD3.1 ~ 3.2 */
		case MMC_VDD_32_33: v1=3;v2=2; setting = 0x0e; break; /* VDD3.2 ~ 3.3 */
		case MMC_VDD_33_34: v1=3;v2=3; setting = 0x0f; break; /* VDD3.3 ~ 3.4 */
		case MMC_VDD_34_35: v1=3;v2=4; setting = -1; break; /* VDD3.4 ~ 3.5 */
		case MMC_VDD_35_36: v1=3;v2=5; setting = -1; break; /* VDD3.5 ~ 3.6 */
		default:
			printk("%s: Unknown voltage request '%i'\n", __func__,vdd);
			hpipaq214_pw_c0_send_byte(0x12,0x05); // VCC_CARD1 off
			printk("-hpipaq214_mci_setpower ERROR\n");
			return;
	}

	if(setting < 0){
		printk("%s: Unable to suppy requested %i.%iV power to MMC.\n",__func__,v1,v2);
		hpipaq214_pw_c0_send_byte(0x12,0x07); // VCC_CARD1 off
		printk("-hpipaq214_mci_setpower ERROR\n");
		return;
	}

	printk("%s: Setting MMC power to %i.%iV.\n",__func__,v1,v2);

	ret = hpipaq214_pw_c0_send_byte(0x39,setting);  // VCC_CARD1/2 voltage
	if(!ret)
		ret =  hpipaq214_pw_c0_send_byte(0x12,0x07);  // VCC_CARD1 on
	
	if(ret){
		hpipaq214_pw_c0_send_byte(0x12,0x05); // VCC_CARD1 off
		printk("%s: Power I2C byte send failed. No MMC Power!\n", __func__);
	}

	printk("-hpipaq214_mci_setpower OK\n");
}

static struct pxamci_platform_data hpipaq214_mci_platform_data = {
	.detect_delay_ms	= 20,
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21 | MMC_VDD_21_22 |
				MMC_VDD_22_23 | MMC_VDD_23_24 | MMC_VDD_24_25 | 
				MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | 
				MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 | 
				MMC_VDD_31_32 | MMC_VDD_32_33 | MMC_VDD_33_34,
				 //can do 1.8 <= V <= 3.3
	.init 		= hpipaq214_mci_init,
	.exit		= hpipaq214_mci_exit,
//	.get_ro		= hpipaq214_mci_ro,
	.setpower 	= hpipaq214_mci_setpower,
	.gpio_card_detect = 15,
	.gpio_card_ro	= 102,
	.gpio_card_ro_invert = 0,
	.gpio_power	= -1,
};


void __init hpipaq214_init_mmc(void)
{
#ifdef CONFIG_MMC_DEBUG
	printk("MMC: hpipaq214_init_mmc, setting mci info\n");
#endif
	pxa_set_mci_info(&hpipaq214_mci_platform_data);
	/* MMC2 is dealt with in hpipaq214-wifi.c */
	/* MMC3 isn't connected or configured to MFP on the iPAQ 21x */
	
}
#else
void inline  __init hpipaq214_init_mmc(void) {}
#endif
