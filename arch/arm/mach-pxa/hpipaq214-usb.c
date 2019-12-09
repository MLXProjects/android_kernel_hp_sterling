/*
 * linux/arch/arm/mach-pxa/hpipaq214-usb.c
 *
 * USB Host and Client Support for the Hewlett-Packard iPAQ 21x.
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * Client driver not actually present yet!!
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
  */

#include <linux/kernel.h>

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/mfp-pxa300.h>
#include <mach/ohci.h>
#include <mach/pxa3xx-regs.h>

#include "generic.h"

int gpio_24pin_detect = 9;
int irq_24pin_detect = -1;
int gpio_usb_reset = 90;

// GPIO for 'USB Reset' might be common host/client
static void hpipaq214_usb_reset(void){
	gpio_set_value(gpio_usb_reset,0);
	mdelay(200);

	gpio_set_value(gpio_usb_reset,1);
	mdelay(200);
}

static irqreturn_t detect_24pin_irq(int irq, void *priv)
{
	printk("24-pin connector change detected. Is now: %i \n",
			gpio_get_value(irq_24pin_detect));
	return IRQ_HANDLED;
}

/****************************
* USB 2 Host (OHCI)        *
****************************/
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
#if defined(CONFIG_USB_MHN_U2D)
	#error Cant compile both USB host and client yet.
#endif

static mfp_cfg_t hpipaq214_mfp_usb_host[] __initdata = {
	//Config USB pins for host
	GPIO30_USB_P2_2 | MFP_LPM_PULL_LOW,
	GPIO31_USB_P2_6  | MFP_LPM_PULL_LOW,
	GPIO32_USB_P2_4 | MFP_LPM_PULL_LOW,
	GPIO33_ULPI_OTG_INTR | MFP_LPM_PULL_LOW,
	GPIO34_USB_P2_5 | MFP_LPM_PULL_LOW,
	GPIO35_USB_P2_3 | MFP_LPM_PULL_LOW,
	GPIO36_UTM_PHYDATA_6 | MFP_LPM_PULL_LOW,
	GPIO37_UTM_PHYDATA_7 | MFP_LPM_PULL_LOW,
	GPIO38_UTM_CLK  | MFP_LPM_DRIVE_LOW,
};


int gpio_usb_host_vbus = 8;
int gpio_usb_host_10 = 10;

/********************************************************************
 *  PXA310 USB Host Port Control
 *
 * Port control for the host seems to be through the USB2 
 * Client (U2D) registers at 0x20 through 0x34. The PXA27x doesn't
 * have the U2D and the PXA300 USB/U2D Drivers don't use/define
 * them either. So this is all just copying and guesswork...
 *
 * At least some of the following is neccessary to make it work,
 * though probably not all. It is most, though not all, of what wince
 * does so when you have the docs, it might not actually make sense.
 *
 * Update: As before but not with proper names/comments from docs.
 *
 ********************************************************************/

/* Some of the PXA310 U2D Registers and some of their bits */
#define U2DBASE			0x54100000
#define	U2DLEN			0x10000

#define U2DCR			0x00	/* U2D Control Register */
#define U2DOTGCR		0x20	/* U2D OTG Control Register */
#define U2DOTGICR		0x24	/* U2D OTG Interrupt Enable Register */
#define U2DOTGISR		0x28	/* U2D OTG Interrupt Status Register */
#define U2DOTGUSR		0x2c	/* U2D OTG ULPI Status Register */
#define U2DOTGUCR		0x30	/* U2D OTG ULPI Control Register */
#define U2DP3CR			0x34	/* U2D Host Port 3 Control Register */

#define U2DCR_UDE		(1 << 0)	/* U2D Enable */

#define U2DOTGUSR_MODE_MASK	(0xf << 28)	/* ULPI Modes Mask */
#define U2DOTGUSR_LPA		(1 << 31)	/* ULPI Low Power Mode Active */
#define U2DOTGUSR_S6A		(1 << 30)	/* ULPI Serial 6-pin Mode Active */
#define U2DOTGUSR_S3A		(1 << 29)	/* ULPI Serial 3-pin Mode Active */
#define U2DOTGUSR_CKA		(1 << 28)	/* ULPI Car Kit Mode Active */

#define U2DOTGCR_OTGEN		(1 << 31)	/* OTG Enable */
#define U2DOTGCR_CKAF		(1 << 5)	/* Carkit Mode Alternate Function Select */
#define U2DOTGCR_UTMID		(1 << 4)	/* UTMI Interface Disable */
#define U2DOTGCR_ULAF		(1 << 3)	/* ULPI Mode Alternate Function Select */
#define U2DOTGCR_SMAF		(1 << 2)	/* Serial Mode Alternate Function Select */
#define U2DOTGCR_RTSM		(1 << 1)	/* ULPI Retrun to Syncronous Mode */
#define U2DOTGCR_ULE		(1 << 0)	/* ULPI Wrapper Enable */

#define U2DOTGINT_SF		(1 << 17)       /* OTG Set Feature Command Received */
#define U2DOTGINT_SI		(1 << 16)       /* OTG Interrupt */
#define U2DOTGINT_RLS1		(1 << 14)       /* RXCMD Linestate[1] Change Interrupt Rise */
#define U2DOTGINT_RLS0		(1 << 13)       /* RXCMD Linestate[0] Change Interrupt Rise */
#define U2DOTGINT_RID		(1 << 12)       /* RXCMD OTG ID Change Interrupt Rise */
#define U2DOTGINT_RSE		(1 << 11)       /* RXCMD OTG Session End Interrupt Rise */
#define U2DOTGINT_RSV		(1 << 10)       /* RXCMD OTG Session Valid Interrupt Rise */
#define U2DOTGINT_RVV		(1 << 9)        /* RXCMD OTG Vbus Valid Interrupt Rise */
#define U2DOTGINT_RCK		(1 << 8)        /* RXCMD Carkit Interrupt Rise */
#define U2DOTGINT_FLS1		(1 << 6)        /* RXCMD Linestate[1] Change Interrupt Fall */
#define U2DOTGINT_FLS0		(1 << 5)        /* RXCMD Linestate[0] Change Interrupt Fall */
#define U2DOTGINT_FID		(1 << 4)        /* RXCMD OTG ID Change Interrupt Fall */
#define U2DOTGINT_FSE		(1 << 3)        /* RXCMD OTG Session End Interrupt Fall */
#define U2DOTGINT_FSV		(1 << 2)        /* RXCMD OTG Session Valid Interrupt Fall */
#define U2DOTGINT_FVV		(1 << 1)        /* RXCMD OTG Vbus Valid Interrupt Fall */
#define U2DOTGINT_FCK		(1 << 0)        /* RXCMD Carkit Interrupt Fall */ 

#define U2DOTGUCR_RUN		(1 << 25)	/* Run loaded operation */

/* ULPI PHY Registers from ULPI Specification */
#define ULPI_FUNC_CTRL_WRITE		0x04	/* Function Control Write */
#define ULPI_IFACE_CTRL_WRITE		0x07	/* Interface Control Write */
#define ULPI_IFACE_CTRL_SET		0x08	/* Interface Control Set */
#define ULPI_IFACE_CTRL_CLEAR		0x09	/* Interface Control Clear */
#define ULPI_OTG_CTRL_SET		0x0b	/* OTG Control set */
#define ULPI_USB_INT_RISE_SET		0x0e	/* USB Interrupt enable rising set */
#define ULPI_USB_INT_RISE_CLEAR		0x0f	/* USB Interrupt enable rising clear */
#define ULPI_USB_INT_FALL_SET		0x11	/* USB Interrupt enable falling set */
#define ULPI_USB_INT_FALL_CLEAR		0x12	/* USB Interrupt enable falling clear  */

/* Internally used current ULPI mode return */
#define ULPI_MODE_LOW_POWER	1
#define ULPI_MODE_SERIAL_6PIN	4
#define ULPI_MODE_SERIAL_3PIN	3
#define ULPI_MODE_CAR_KIT	5
#define ULPI_MODE_NONE		2


#undef SUPERDEBUG

void __iomem *u2dregs;

#ifdef SUPERDEBUG
static u32 u2d_readl(u32 reg){
	u32 val = __raw_readl(u2dregs + reg);
	printk("U2D Read %02x = %08x\n",reg,val);
	return val;
}

static void u2d_writel(u32 reg, u32 val){
	printk("U2D %02x = %08x, write %08x",reg,__raw_readl(u2dregs + reg),val);
	__raw_writel(val,u2dregs + reg);
	printk(", readback = %08x\n",__raw_readl(u2dregs + reg));
}

static void u2d_orl(u32 reg, u32 val){
	u32 old = __raw_readl(u2dregs + reg);
	u32 new = old | val;
	printk("U2D %02x = %08x, or %08x = %08x",reg,old,val,new);
	__raw_writel(new,u2dregs + reg);
	printk(", readback = %08x\n",__raw_readl(u2dregs + reg));
}

static void u2d_andl(u32 reg, u32 val){
	u32 old = __raw_readl(u2dregs + reg);
	u32 new = old & val;
	printk("U2D %02x = %08x, and %08x = %08x",reg,old,val,new);
	__raw_writel(new,u2dregs + reg);
	printk(", readback = %08x\n",__raw_readl(u2dregs + reg));
}
#else
#define u2d_readl(reg)	__raw_readl(u2dregs + (reg))
#define u2d_writel(reg, value)	__raw_writel((value), u2dregs + (reg))
#define u2d_andl(reg, value) \
	__raw_writel( __raw_readl(u2dregs + (reg)) & value, u2dregs + (reg));
#define u2d_orl(reg, value) \
	__raw_writel( __raw_readl(u2dregs + (reg)) | value, u2dregs + (reg));
#endif

static void waitForULPISynchronousMode(void)
{
	int timeoutCount = 0;

	/* Wait for all ULPI mode flags to go off */ 
	while( (u2d_readl(U2DOTGUSR) & U2DOTGUSR_MODE_MASK) ){

		timeoutCount++;
		mdelay(2);

		if(timeoutCount > 500){	// 1second!
			printk("Waiting for U2DOTGUSR timeout reached, resetting USB\n");
			hpipaq214_usb_reset();
			break;
		}
	}
}

static void setULPISynchronousMode(void)
{
	u2d_orl(U2DOTGCR, U2DOTGCR_UTMID); /* Disable UTMI Interface */
	u2d_orl(U2DOTGCR, U2DOTGCR_ULE); /* Enable ULPI Wrapper */
	u2d_orl(U2DOTGCR, U2DOTGCR_RTSM); /* Return ULPI to Syncronous Mode */

	/* Disable carkit Mode Path, 
	 * Disable serial mode path
	 * Enable ULPI mode path */ 
	u2d_writel(U2DOTGCR, (u2d_readl(U2DOTGCR) & ~(U2DOTGCR_CKAF | U2DOTGCR_SMAF))
		 | U2DOTGCR_ULAF );

	waitForULPISynchronousMode();
}

static void writePHY(u32 addr, u32 wdata)
{
	u32 timeoutCount;

	u2d_writel(U2DOTGUCR, (( wdata | (addr << 8) ) << 8) 
				| U2DOTGUCR_RUN );

	timeoutCount = 0;
	/* Wait for operation to complete */
	while( u2d_readl(U2DOTGUCR) & U2DOTGUCR_RUN ){
		timeoutCount++;
		if( timeoutCount > 0x00989680){
			printk("U2DOTGUCR timeout reached while writing to PHY, resetting USB\n");
			hpipaq214_usb_reset();
			break;
		}
	};
}
static void hpipaq214_ohci_exit(struct device *dev)
{
	//just turn the Vbus power off
	gpio_set_value(gpio_usb_host_vbus, 0);
	iounmap(u2dregs);
}

static int hpipaq214_ohci_init(struct device *dev)
{
	u2dregs = ioremap(U2DBASE, U2DLEN);
	if (!u2dregs) {
		printk("ioremap failed");
		return -ENOMEM;
	}
	//make sure both the USB2 clocks are on since we'll be using their regs
	CKENA |= CKEN_USBH;
	CKENA |= CKEN_USB2;

	gpio_set_value(gpio_usb_host_vbus, 0); /* Turn VBus off for the moment */
	gpio_set_value(gpio_usb_host_10, 1); //make sure it's on

	mdelay(100);

	hpipaq214_usb_reset();			/* reset the ULPI PHY */

	/* Booting directly from winCE hangs here on any read/write to the OTG regs */

	u2d_andl(U2DCR, ~U2DCR_UDE); 		/* Disable U2D, we won't be using it */

	u2d_writel(U2DOTGICR, 0x00); 		/* Turn off all OTG interrupts */

	u2d_orl(U2DOTGCR, U2DOTGCR_OTGEN); 	/* Enable the OTG System */

	setULPISynchronousMode(); 		/* Switch the PXA's USB system to ULPI Synchronous mode so we can talk to the PHY */

	writePHY(ULPI_OTG_CTRL_SET, 0x06); 	/* Signal PHY to turn D+ and D- 15kOhm pull downs on */

	u2d_andl(U2DP3CR, ~0x100); 		/* Select port3 to 6-pin serial mode */
	u2d_andl(U2DP3CR, ~0x200);
	writePHY(ULPI_IFACE_CTRL_CLEAR, 0x02); 	/* Disable ULPI 3-pin serial mode */
	writePHY(ULPI_IFACE_CTRL_SET, 0x01); 	/* EnableULPI  6-pin serial mode */

	u2d_andl(U2DCR, ~U2DCR_UDE);		/* Make sure the U2D is still disabled */
	u2d_andl(U2DOTGCR, ~U2DOTGCR_ULAF); 	/* Disable ULPI Mode */
	u2d_andl(U2DOTGCR, ~U2DOTGCR_CKAF); 	/* Disable Carkit mode */
	u2d_orl(U2DOTGCR, U2DOTGCR_UTMID); 	/* Disable UTMI interface */

	u2d_orl(U2DOTGCR, U2DOTGCR_SMAF); 	/* Enable serial mode (Host) */

	/* We don't set up any sort of IRQ handler, and it appears to work anyway, so... meh
	u2d_writel(U2DOTGICR, (U2DOTGINT_FLS1 | U2DOTGINT_FLS0 | U2DOTGINT_RLS1 | U2DOTGINT_RLS0 | U2DOTGINT_SF | U2DOTGINT_SI)); */
		/* RXCMD Linestate[1] Change Interrupt Fall */
		/* RXCMD Linestate[0] Change Interrupt Fall */
		/* RXCMD Linestate[1] Change Interrupt Rise */
		/* RXCMD Linestate[0] Change Interrupt Rise */
		/* OTG Set Feature Command Received */
		/* OTG Interrupt */ 

	gpio_set_value(gpio_usb_host_vbus, 1); /* now we can turn VBus on for any devices */

	return 0;
}

static struct pxaohci_platform_data hpipaq214_ohci_platform_data = {
	.port_mode	= PMM_NPS_MODE,		//we leave the power on for the moment
	.init		= hpipaq214_ohci_init,
	.exit		= hpipaq214_ohci_exit,
};

static void __init hpipaq214_init_ohci(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(hpipaq214_mfp_usb_host));

	if(!gpio_request(gpio_usb_host_vbus, "USB Host VBus enable"))
		gpio_direction_output(gpio_usb_host_vbus, 0);

	if(!gpio_request(gpio_usb_host_10, "USB Host Unknown"))
		gpio_direction_output(gpio_usb_host_10, 0);

	pxa_set_ohci_info(&hpipaq214_ohci_platform_data);
}
#else
static inline __init void hpipaq214_init_ohci(void) {}
#endif

/********************************
* PXA's USB 2 Client  (U2D)     *
********************************/
#if defined(CONFIG_USB_MHN_U2D)

#error Not fully implemented.
// The PXA's ULPI interface has a USB3319 ULPI transceiver connected but which
// doesn't seem to be connected to the 24-pin connector. The winCE bootloader
// and winCE itself use the quicklogic chi&p for USB client comms, though there seems
// to be the option of using the PXA's USB 2 Client for KITL.
// I have converted the U2D driver from the PXA300 from 2.4.16 to the this kernel
// but can't seem to find where its connected.

static mfp_cfg_t hpipaq214_mfp_usb_client[] __initdata = {
	GPIO30_ULPI_DATA_OUT_0,
	GPIO31_ULPI_DATA_OUT_1,
	GPIO32_ULPI_DATA_OUT_2,
	GPIO33_ULPI_DATA_OUT_3,
	GPIO34_ULPI_DATA_OUT_4,
	GPIO35_ULPI_DATA_OUT_5,
	GPIO36_ULPI_DATA_OUT_6,
	GPIO37_ULPI_DATA_OUT_7,
	GPIO38_ULPI_CLK,
	//ULPI_DIR, //presently, defs are wrong, but these should be correct from boot 
	//ULPI_NXT,
	//ULPI_STP,
};

/********************************************************************/
int gpio_u2d_vbus = 18;
int gpio_u2d_jack_select = 82;
int u2d_irq = -1;

static int hpipaq214_u2d_init(struct device *dev, irq_handler_t hpipaq214_u2d_detect_int, void *data)
{
	int err;
	hpipaq214_usb_reset();

	printk("hpipaq214_u2d_init");

	err = gpio_request(gpio_u2d_vbus, "U2D VBUS detect");
	err += gpio_request(gpio_u2d_jack_select, "U2D 24-pin / miniUSB select");
	if(err){
		printk("Unable to register U2D gpios\n");
	}else{
		gpio_direction_input(gpio_u2d_vbus);
		gpio_direction_output(gpio_u2d_jack_select, 0); //select 24-pin
	}

	u2d_irq = gpio_to_irq(gpio_u2d_vbus);

	err = request_irq(u2d_irq, hpipaq214_u2d_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "U2D vbus detect", data);
	if (err) {
		printk(KERN_ERR "%s: U2D: "
				"can't request cable detect IRQ\n", __func__);
		return 0;
	}

	return 0;

}

static int hpipaq214_u2d_is_connected(void)
{
	int isconn = (gpio_get_value(gpio_u2d_vbus) == 0);
	printk("hpipaq214_u2d_is_connected reporting %i\n",isconn);
	return isconn;
}

static void hpipaq214_u2d_command(int cmd)
{
	printk("hpipaq214_u2d_command: ");
	switch(cmd){
		case MHN_U2D_CMD_DISCONNECT:
			printk("MHN_U2D_CMD_DISCONNECT\n");
			//gpio_set_value(gpio_u2d_dp_pullup, 0);
			break;
		case MHN_U2D_CMD_CONNECT:
			printk("MHN_U2D_CMD_CONNECT\n");
			//gpio_set_value(gpio_u2d_dp_pullup, 1);
			break;
		default:
			printk("unknown\n");
			break;
	}
}

static void hpipaq214_u2d_exit(struct device *dev, void *data)
{
	gpio_free(gpio_u2d_jack_select);
	gpio_free(gpio_u2d_vbus);
	printk("hpipaq214_u2d_exit\n");
}

static struct mhn_u2d_mach_info u2d_info = {
	.u2d_init		= hpipaq214_u2d_init,
	.u2d_is_connected	= hpipaq214_u2d_is_connected,
	.u2d_command		= hpipaq214_u2d_command,
	.u2d_exit		= hpipaq214_u2d_exit,
};

static struct platform_device *hpipaq214_devices_2[] __initdata = {
	&pxa3xx_device_u2d,
};

static void __init hpipaq214_init_u2d(void){
	pxa3xx_mfp_config(ARRAY_AND_SIZE(hpipaq214_mfp_usb_client));
	pxa3xx_set_u2d_info(&u2d_info);

	printk("hpipaq214_u2d_init\n");
	//need specific addition since it's not part of mainline kernel
	//platform_device_add(&pxa3xx_device_u2d);
	platform_add_devices(hpipaq214_devices_2, ARRAY_SIZE(hpipaq214_devices_2));
}
#else
static inline  __init void hpipaq214_init_u2d(void) {}
#endif

void  __init hpipaq214_init_usb(void) {

	if(!gpio_request(gpio_24pin_detect, "24-pin jack detect")){
		gpio_direction_input(gpio_24pin_detect);
	
		irq_24pin_detect = gpio_to_irq(gpio_24pin_detect);
		if (request_irq(irq_24pin_detect, detect_24pin_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"24-pin jack detect", NULL)) {
			printk(KERN_ERR "%s: USB: "
					"can't request 24pin detect IRQ\n", __func__);
			return;
		}
	}

	if(!gpio_request(gpio_usb_reset, "USB Reset"))
		gpio_direction_output(gpio_usb_reset, 1);	//off for now

	hpipaq214_init_ohci();
	//hpipaq214_init_u2d();	

	//	gpio_free(gpio_24pin_detect); <-- put somwhere, with a free_irq aswell??	
}
