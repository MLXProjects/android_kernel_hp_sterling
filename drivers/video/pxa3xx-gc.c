/* Basic driver for the PXA3xx 2D Graphics Controller 
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Driver creates and handles the instruction ringbuffer.
 *
 * Main interface is with IOCTLs through the character device:
 * GCIO_ADD_COMMAND		Adds a command to the ring buffer.
 * GCIO_WAIT_ALL_COMPLETE	Wait for the GC to execute
 *					all the commands in the ring buffer.
 * GCIO_GET_RINGBUFF_FREE	Returns no. of free bytes in the ring buffer.
 * GCIO_RESET			Resets and tests the GC.
 *
 * Instructions added with GCIO_ADD_COMAND are NOT checked for anything
 * (validity, clipping, correct memory areas etc.)
 *
 */

#ifdef CONFIG_FB_PXA3XX_GFX

//define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/completion.h>
#include <linux/fb.h>
#include <linux/miscdevice.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/fb.h>
#include <asm/irq.h>

#include "pxa3xx-gc.h"

#define TIMEOUT_ALL_COMPLETE	5000

/* Something is wrong with dev_dbg in here
 * and I can't figure out what. */
#define gc_debug(gci, format, arg...)		\
	if(gci->debug)				\
		printk(format , ## arg)

static inline unsigned long
gc_readl(struct pxa3xx_gc_info *gci, unsigned int off) {
	return __raw_readl(gci->mmio_base + off);
}

static inline void
gc_writel(struct pxa3xx_gc_info *gci, unsigned int off, unsigned long val) {
	__raw_writel(val, gci->mmio_base + off);
}

/**** Character device interface ******/

/* Can only have 1 GC device for now */
static struct pxa3xx_gc_info *infoForCharDev = NULL;

#define GC_DEV_NAME	"pxa3xx-gcu"
#define GC_MAJOR 241	/* local/experimental */

#define GCIO_ADD_COMMAND 	_IOWR(GC_MAJOR, 2, unsigned long *)
#define GCIO_WAIT_ALL_COMPLETE	_IOR(GC_MAJOR, 3, int)
#define GCIO_GET_RINGBUFF_FREE	_IOR(GC_MAJOR, 4, int)
#define GCIO_RESET		_IOR(GC_MAJOR, 5, int)

static int pxa3xx_gc_cdev_open(struct inode *inode, struct file *file) {
	struct pxa3xx_gc_info *gci = infoForCharDev;

	/* allow only one user at a time */
	if (atomic_inc_and_test(&gci->cdevOpenCount)) {
		gc_debug(gci, "GC: Graphic controller already in use.\n");
		return -EBUSY;
	}

	return 0;
}

static int pxa3xx_gc_cdev_release(struct inode *inode, struct file *file) {
	struct pxa3xx_gc_info *gci = infoForCharDev;
	atomic_dec(&gci->cdevOpenCount);

	return 0;
}

static ssize_t pxa3xx_gc_cdev_read(struct file *file, char __user *buffer,
		size_t length, loff_t *offset) {
	struct pxa3xx_gc_info *gci = infoForCharDev;
	dev_err(gci->dev, "GC: read %i\n", length);
	return length;
}

static ssize_t pxa3xx_gc_cdev_write(struct file *file,
		const char  __user *buffer, size_t length, loff_t *offset) {
	struct pxa3xx_gc_info *gci = infoForCharDev;
	dev_err(gci->dev, "GC: write %i\n", length);
	return length;
}

static int pxa3xx_gc_cdev_ioctl(struct inode *inode, struct file *file,
		unsigned int ioctl_num, unsigned long ioctl_param) { 

	struct pxa3xx_gc_info *gci = infoForCharDev;
	unsigned long datalen;
	unsigned long *data;
	void __user *argp = (void __user *)ioctl_param;
	int ret;

	switch (ioctl_num) {
	case GCIO_ADD_COMMAND:

		/* The lowest nibble of the first ulong
		 * gives the number of extra ulongs */
		if (copy_from_user(&datalen, argp, sizeof(unsigned long))){
			dev_err(gci->dev, "GC: copy_from_user failed\n");
			return -EFAULT;
		}
		datalen = ((datalen & 0x0F) + 1) * 4;

		data = kmalloc(datalen, GFP_KERNEL);
		if (!data)
			return -ENOMEM;
		if (copy_from_user(data, argp, datalen)){
			dev_err(gci->dev, "GC: copy_from_user failed\n");
			kfree(data);
			return -EFAULT;
		}

		ret = pxa3xx_gc_ringbuff_add(gci, data, datalen);

		kfree(data);
		return ret;
	case GCIO_WAIT_ALL_COMPLETE:
		return pxa3xx_gc_wait_all_complete(gci);
	case GCIO_GET_RINGBUFF_FREE:
		return pxa3xx_gc_ringbuff_free(gci);
	case GCIO_RESET:
		return pxa3xx_gc_full_reset(gci);
	default:
		dev_err(gci->dev, 
			"GC: Unsupported IOCTL %u with param %08lx\n",
			ioctl_num, ioctl_param);
		return -EINVAL;
	}
}

static const struct file_operations pxa3xx_gc_ops = {
	.owner =	THIS_MODULE,
	.read =		pxa3xx_gc_cdev_read, 	/* read */
	.write = 	pxa3xx_gc_cdev_write,	/* write */
	.ioctl = 	pxa3xx_gc_cdev_ioctl,   /* ioctl */
	.open = 	pxa3xx_gc_cdev_open,	/* open */
	.release = 	pxa3xx_gc_cdev_release  /* close */
};

static struct miscdevice pxa3xx_gc_dev = {
	.minor = 	MISC_DYNAMIC_MINOR,
	.name = 	"pxa3xx-gcu",
	.fops = 	&pxa3xx_gc_ops,
};


static int pxa3xx_gc_init_chardev(struct pxa3xx_gc_info *gci) {
	int ret;
	atomic_set(&gci->cdevOpenCount, 0);

	if (infoForCharDev) {
		dev_err(gci->dev,
			"Cannot register more than 1 pxa3xx-gcu device.\n");
		return -EBUSY;
	}
	infoForCharDev = gci;

	ret = misc_register(&pxa3xx_gc_dev);	
	if (ret < 0) {
		dev_err(gci->dev,
			"GC: Register misc device failed: %d\n", ret);
		return -ENODEV;
	}

	return 0;
}

/****** sysfs interface *******/

static ssize_t pxa3xx_gc_show_debug(struct device *dev,
				struct device_attribute *attr, char *buf){

	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev);
	strcpy(buf, gci->debug ? "on\n" : "off\n");
	return strlen(buf) + 1;	
}

static ssize_t pxa3xx_gc_store_debug(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count){

	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev);

	if( (count == 1 && buf[0] == '0') || (count >= 3 && 
			buf[0] == 'o' && buf[1] == 'f' && buf[2] == 'f') )
		gci->debug = 0;
	else
		gci->debug = -1;

	return count;
}

static ssize_t pxa3xx_gc_show_state(struct device *dev,
				struct device_attribute *attr, char *buf){
	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev);
	int ret;

	ret = pxa3xx_gc_test_ready(gci);	
	if(!ret){
		strcat(buf, "ready\n");
		return 6;
	}else{
		sprintf(buf, "error %i", ret);
		return strlen(buf) + 1;	
	}	
}

static ssize_t pxa3xx_gc_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count){

	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev);

	if (strncasecmp(buf, "reset", count) == 0)
		pxa3xx_gc_full_reset(gci);
	else if (strncasecmp(buf, "test", count) == 0)
		pxa3xx_gc_do_tests(gci);

	return count;
}

static ssize_t pxa3xx_gc_show_nop(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev);
	snprintf(buf, PAGE_SIZE, "%lu\n", gc_readl(gci, GCNOPID));

	return strlen(buf) + 1;	
}

static ssize_t pxa3xx_gc_store_nop(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev);

	pxa3xx_gc_cmd_nop(gci, simple_strtoul(buf, NULL, 0));

	return count;
}

#define DBGREG(x)	{"(x)", x}

static ssize_t pxa3xx_gc_show_regs(struct device *dev,
			struct device_attribute *attr, char *buf){
	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev);
	int i, pos = 0;

	struct debug_reg {
		const char *name;
		unsigned int offset;
	};

	struct debug_reg regs[] = {
		DBGREG(GCCR), DBGREG(GCISCR), DBGREG(GCIECR), DBGREG(GCNOPID),
		DBGREG(GCALPHASET), DBGREG(GCTSET),
		DBGREG(GCRBBR), DBGREG(GCRBLR), DBGREG(GCRBHR), DBGREG(GCRBTR),
		DBGREG(GCRBEXHR), 
		DBGREG(GCBBBR),DBGREG(GCBBHR),DBGREG(GCBBEXHR),
		DBGREG(GCD0BR),DBGREG(GCD0STP),DBGREG(GCD0STR),DBGREG(GCD0PF),
		DBGREG(GCD1BR),DBGREG(GCD1STP),DBGREG(GCD1STR),DBGREG(GCD1PF),
		DBGREG(GCD2BR),DBGREG(GCD2STP),DBGREG(GCD2STR),DBGREG(GCD2PF),
		DBGREG(GCS0BR),DBGREG(GCS0STP),DBGREG(GCS0STR),DBGREG(GCS0PF),
		DBGREG(GCS1BR),DBGREG(GCS1STP),DBGREG(GCS1STR),DBGREG(GCS1PF),
		DBGREG(GCCABADDR), DBGREG(GCTABADDR), DBGREG(GCMABADDR),
	};

	for(i=0; i < ARRAY_SIZE(regs); i++){
		pos += snprintf(buf + pos, PAGE_SIZE - pos, "%02X %s=%08lx\n",
				regs[i].offset, regs[i].name,
				gc_readl(gci, regs[i].offset));
	}

	return pos + 1;	
}

static ssize_t pxa3xx_gc_store_regs(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
/*	struct pxa3xx_gc_info *gci = dev_get_drvdata(dev); */

	return count;
}

DEVICE_ATTR(debug, (S_IRUGO | S_IWUGO), pxa3xx_gc_show_debug, pxa3xx_gc_store_debug);
DEVICE_ATTR(regs, (S_IRUGO | S_IWUGO), pxa3xx_gc_show_regs, pxa3xx_gc_store_regs);
DEVICE_ATTR(nop, (S_IRUGO | S_IWUGO), pxa3xx_gc_show_nop, pxa3xx_gc_store_nop);
DEVICE_ATTR(state, (S_IRUGO | S_IWUGO), pxa3xx_gc_show_state, pxa3xx_gc_store_state);

int pxa3xx_gc_init_sysdev(struct pxa3xx_gc_info *gci) {
	if(device_create_file(gci->dev, &dev_attr_debug)){
		dev_err(gci->dev, "Unable to create debug sysfs entries\n");
		return -ENODEV;
	}
	if(device_create_file(gci->dev, &dev_attr_regs)){
		dev_err(gci->dev, "Unable to create regs sysfs entries\n");
		return -ENODEV;
	}
	if(device_create_file(gci->dev, &dev_attr_nop)){
		dev_err(gci->dev, "Unable to create nop sysfs entries\n");
		return -ENODEV;
	}
	if(device_create_file(gci->dev, &dev_attr_state)){
		dev_err(gci->dev, "Unable to create state sysfs entries\n");
		return -ENODEV;
	}
	return 0;
}

/***********************************/

/** Returns the number of free bytes in the ringbuffer */
static int pxa3xx_gc_ringbuff_free(struct pxa3xx_gc_info *gci){
	unsigned long execHead = gc_readl(gci, GCRBEXHR);
	unsigned long tail = gc_readl(gci, GCRBTR);

	/* there is always 4 less bytes then you'd think as we
	 * can't move the tail pointer past the last entry without 
	 * it thinking the buffer is empty again. */
	if(execHead > tail) //buffer wrapped
		return execHead - tail - 4; 
	else
		return gci->ringbuff_size - tail + execHead - 4;
}

/** Add data to the ring buffer and signal to execute */
static int pxa3xx_gc_ringbuff_add(struct pxa3xx_gc_info *gci,
			unsigned long *data, int len) {
	unsigned long tail, *ptr;
	int free, i;
//	int lenToEnd;

	if(len & 0x3){
		/* Die noisily and completely so it gets noticed */
		dev_err(gci->dev,
			"GC: Cmd %08lx with len %i - not multiple of 4\n",
			*data, len);
		gc_writel(gci, GCCR, gc_readl(gci, GCCR) | GCCR_ABORT); 
		return -EINVAL;
	}

	free = pxa3xx_gc_ringbuff_free(gci);	
	if(len > free)
		return -ENOMEM;

	tail = gc_readl(gci, GCRBTR) - (unsigned long)gci->ringbuff_dma;
	ptr = (unsigned long *)(gci->ringbuff_cpu + tail);
	for(i=0; i < (len/4); i++){
		*ptr++ = data[i];
		if( ptr >= (unsigned long *)(gci->ringbuff_cpu + gci->ringbuff_size) )
			ptr = (unsigned long *)gci->ringbuff_cpu;	
	}
	tail += len;
	if(tail >= gci->ringbuff_size)
		tail -= gci->ringbuff_size;

/*	// This doesn't seem to completely work
	lenToEnd = gci->ringbuff_size - tail; // room to end of buffer 
	if( len >= lenToEnd ){
		memcpy(gci->ringbuff_cpu + tail, data, lenToEnd);
		memcpy(gci->ringbuff_cpu, data, len - lenToEnd);
		tail = len - lenToEnd;
	}else{
		memcpy(gci->ringbuff_cpu + tail, data, len);
		tail += len;
	}*/

/*	printk("GC: Data= ");
	for(i=0; i<(len/4); i++)
		printk("%08lx ", *(data + i));
	printk("\n");
	gc_debug(gci, "GC: moving tail to %lu\n", tail);
*/
	udelay(5); /* Need to wait for the data to get to the memory??? */
	gc_writel(gci, GCRBTR, gci->ringbuff_dma + tail);

	return 0;
}	

static int pxa3xx_gc_wait_all_complete(struct pxa3xx_gc_info *gci){
	int ret;
	unsigned long gciscr;

	gc_writel(gci, GCISCR, 0xFF); //clear any previous conditions

	init_completion(&gci->allDoneOrError); //clear the completion state
	gc_writel(gci, GCIECR, GCI_STOP_INT | GCI_PF_INT | GCI_EEOB_INT |
				GCI_IIN_INT |	GCI_IOP_INT | GCI_BF_INT | 
				GCI_IN_INT | GCI_EOB_INT);

	ret = wait_for_completion_timeout(&gci->allDoneOrError,
				msecs_to_jiffies(TIMEOUT_ALL_COMPLETE));
	gciscr = gc_readl(gci, GCISCR) & 0xFF;

	if( ret == 0 ){
		dev_err(gci->dev, "GC: Timed out waiting for commands done.");	
		return -EFAULT;
	}else if( gciscr & (GCI_STOP_INT | GCI_PF_INT |
				GCI_IIN_INT | GCI_IOP_INT | GCI_IN_INT) ){
		dev_err(gci->dev, "GC: Error, interrupt or stop while waiting"
			" for commands to complete. GCISCR=0x%02lx", gciscr);
		return gciscr;
	}else if( gciscr & GCI_EEOB_INT ) //all ok 
		return 0;

	dev_err(gci->dev, "GC: Cannot work out how completion completed!\n");
	return -EFAULT;
}

/** Executes a nop command */
static int pxa3xx_gc_cmd_nop(struct pxa3xx_gc_info *gci, unsigned long id) {
	unsigned long data = GC_NOP | ((id & 0x000FFFFF) << 4);
	gc_debug(gci, "GC: Adding NOP command with nop_id %08lx.\n", id);

	return pxa3xx_gc_ringbuff_add(gci, &data, 4);
}

/** Executes a set buffer command */
static int pxa3xx_gc_cmd_set_buff(struct pxa3xx_gc_info *gci, unsigned int buff, 
				unsigned long base, unsigned char step,
				unsigned int stride, unsigned char pixelFormat){
	unsigned long data[3];

	gc_debug(gci, "GC: Adding set buffer command: buff=%02x, base=%08lx,"
				"step=%i, stride=%i, pxfmt=%01x.\n",
				buff, base, step, stride, pixelFormat);	

	data[0] = GC_BUFFI | (buff << GC_BUFFI_ADDR_BIT) | 0x02;
	data[1] = base;
	data[2] = ((pixelFormat & 0x0F) << 19) | 
			((stride & 0x3FFF) << 5) | (step & 0x0F);

	return pxa3xx_gc_ringbuff_add(gci, data, 12);
}

/** Executes a copy blt */
static inline int pxa3xx_gc_cmd_copy_blt(struct pxa3xx_gc_info *gci, 
			int s0x, int s0y, int dx, int dy, int w, int h){

	return pxa3xx_gc_cmd_raster_blt(gci, s0x, s0y, 0, 0, dx, dy, w, h, 0xCC);
}

/** Executes a raster blt */
static int pxa3xx_gc_cmd_raster_blt(struct pxa3xx_gc_info *gci, 
				int s0x, int s0y, int s1x, int s1y,
				int dx, int dy, int w, int h, unsigned char op) {
		unsigned long data[8];

	data[0] = GC_RAST | ((op & 0xFF) << 16) | 7;
	data[1] = dx & 0x07FF;
	data[2] = dy & 0x07FF;
	data[3] = s0x & 0x07FF;
	data[4] = s0y & 0x07FF;
	data[5] = s1x & 0x07FF;
	data[6] = s1y & 0x07FF;
	data[7] = ((h & 0x07FF) << 16) | (w & 0x07FF);

	return pxa3xx_gc_ringbuff_add(gci, data, 32);	
}

/** Executes a color fill command */
static int pxa3xx_gc_cmd_color_fill(struct pxa3xx_gc_info *gci, int x0, int y0, int w, int h, 
				unsigned long colorrgba){
	unsigned long data[5];
//	gc_debug(gci, "GC: Color fill at (%i,%i) by (%ix%i) in colour %08lx\n",
//			x0, y0, w, h, colorrgba);

	data[0] = GC_CFILL | (0 << 20) | (PF_32B_RGBA8888 << 8) | GC_CFILL_IMM | 4;
	data[1] = x0 & 0x07FF;
	data[2] = y0 & 0x07FF;
	data[3] = ((h & 0x07FF) << 16) | (w & 0x07FF);
	data[4] = colorrgba;

	return pxa3xx_gc_ringbuff_add(gci, data, 20);	
}

/** Executes a stretch BLT */
static int pxa3xx_gc_cmd_stretch_blt(struct pxa3xx_gc_info *gci, 
				int dx0, int dy0, int dw, int dh,
				int sx0, int sy0, int sw, int sh ) {

	unsigned long data[9];
	unsigned int x_str_int, x_str_frac;
	unsigned int y_str_int, y_str_frac;

	gc_debug(gci, "GC: Stretch BLT (%i,%i)(%ix%i) -> (%i,%i)(%ix%i)\n",
			 sx0, sy0, sw, sh, dx0, dy0, dw, dh );

	x_str_int = 0;
	if(sw > dw){
		dev_err(gci->dev, "GC: Stretch BLT with SW > DW\n");
		return -1;
	}else if(sw == dw)
		x_str_frac = 0x3FF;
	else
		x_str_frac = (((sw - 1) * 0x400) / (dw - 1)) & 0x3FF;
	

	y_str_int = 0;
	if(sh > dh){
		dev_err(gci->dev, "GC: Stretch BLT with SH > DH\n");
		return -1;
	}else if(sh == dh)
		y_str_frac = 0x3FF;
	else
		y_str_frac = (((sh - 1) * 0x400) / (dh - 1)) & 0x3FF;

	gc_debug(gci, "GC: xstr=%04x.%04x ystr=%04x.%04x\n",
			x_str_int, x_str_frac, y_str_int, y_str_frac);

	data[0] = GC_STRBLT | 8;
	data[1] = dx0 & 0x07FF;
	data[2] = dy0 & 0x07FF;
	data[3] = sx0 & 0x07FF;
	data[4] = sy0 & 0x07FF;
	data[5] = ((sh & 0x07FF) << 16) | (sw & 0x07FF);
	data[6] = ((dh & 0x07FF) << 16) | (dw & 0x07FF);
	data[7] = ((x_str_int & 0x03FF) << 16) | (x_str_frac & 0x03FF);
	data[8] = ((y_str_int & 0x03FF) << 16) | (y_str_frac & 0x03FF);

	return pxa3xx_gc_ringbuff_add(gci, data, 36);	
}

/*** Clears and resets the ring buffer */
static int pxa3xx_gc_full_reset(struct pxa3xx_gc_info *gci){
	unsigned long gccr;
	int ret;

	gc_debug(gci, "GC: resetting grahics controller...\n");

	gccr = (0 << GCCR_CURR_DEST) | (1 << GCCR_DEST);

	/* Assert the abort control bit and wait for it to do so */
	gc_writel(gci, GCCR, gccr | GCCR_ABORT );
	mdelay(1);
	gc_writel(gci, GCCR, gccr);

	pxa3xx_gc_setup_ringbuff(gci);

	ret = pxa3xx_gc_test_ready(gci);
	if(!ret)
		gc_debug(gci, "GC: Reset ok and GC ready\n");

	return ret;
}

/** Test the GC by executing a set NOP command and
 * checking the NOPID register */
static int pxa3xx_gc_test_ready(struct pxa3xx_gc_info *gci) {
	int ret;
	const unsigned long testID = 0x12345;
	unsigned long readback;

	/* clear the nop_id register */
	gc_writel(gci, GCNOPID, 0x00);

	/* queue a set nop_id command */
	ret = pxa3xx_gc_cmd_nop(gci, testID);
	if(ret){
		dev_err(gci->dev, "GC: Error %i adding NOP command to ring buffer.\n", ret);
		return ret;
	}

	/* wait for it to complete */
	ret = pxa3xx_gc_wait_all_complete(gci);
	if(ret){
		dev_err(gci->dev, "GC: Error %i waiting for NOP command to complete.\n", ret);
		return ret;
	}

	/* Read it back to make sure it wrote the correct thing */
	readback = gc_readl(gci, GCNOPID); 
	gc_debug(gci, "GC: nop_id read back as %08lx.\n", readback);
		
	if ( readback != testID ){
		dev_err(gci->dev, "GC: NOP ID setting test failed."
				"Wrote %08lx but read back %08lx",
				testID, readback);
		return -EFAULT;
	}

	return 0;
}

/** Clears and resets the ring buffer */
static void pxa3xx_gc_setup_ringbuff(struct pxa3xx_gc_info *gci){
	/* Set Base Addr - This will also set the head and execution head */
	gc_writel(gci, GCRBBR, gci->ringbuff_dma); 

	/* Tail - Set the tail to the same place so it */
	/* doesn't start running when we set the length */
	gc_writel(gci, GCRBTR, gci->ringbuff_dma); 

	/* Set the buffer length */
	gc_writel(gci, GCRBLR, gci->ringbuff_size); 

	gc_debug(gci, "GC: Set ringbuffer regs, Base=%08lx,"
		"Length=%08lx, Head=%08lx, Tail=%08lx, ExecHead=%08lx\n",
		gc_readl(gci, GCRBBR), gc_readl(gci, GCRBLR), gc_readl(gci, GCRBHR),
		gc_readl(gci, GCRBTR), gc_readl(gci, GCRBEXHR) );
}

/** Handler for all GC IRQs */
static irqreturn_t pxa3xx_gc_handle_irq(int irq, void *dev_id) {
	struct pxa3xx_gc_info *gci = dev_id;

	unsigned int gciscr = gc_readl(gci, GCISCR);
	unsigned long *l;
	int n;

	if (gciscr & GCI_STOP_INT)
		dev_err(gci->dev, "GC: IRQ: Caught STOP\n");

	if (gciscr & GCI_PF_INT)
		dev_err(gci->dev, "GC: IRQ: Caught Invalid Pixel Format\n");

	if (gciscr & GCI_IIN_INT)
		dev_err(gci->dev, "GC: IRQ: Caught Illegal Instruction\n");

	if (gciscr & GCI_IOP_INT)
		dev_err(gci->dev, "GC: IRQ: Caught Illegal Operation\n");

	if (gciscr & GCI_BF_INT)
		gc_debug(gci, "GC: IRQ: Caught Display Buffer Flip\n");

	if (gciscr & GCI_IN_INT)
		gc_debug(gci, "GC: IRQ: Caught Interrupt Instruction\n");

	if ( !(gciscr & 0xFF) )
	 	dev_err(gci->dev, "GC: IRQ with none set\n");

/*	if (gciscr & GCI_EEOB_INT)
		gc_debug(gci, "GC: IRQ: Caught Execution End of Buffer\n");

	if (gciscr & GCI_EOB_INT)
		gc_debug(gci, "GC: IRQ: Caught End of Buffer\n");
*/

	/* if error or end of execution, signal complete */
	if ( gciscr & (GCI_STOP_INT | GCI_PF_INT | GCI_EEOB_INT | 
			GCI_IIN_INT | GCI_IOP_INT | GCI_IN_INT ) )
		complete(&gci->allDoneOrError);
	

	/* On error, dump the command currently at exechead */
	if( gciscr & (GCI_PF_INT | GCI_IIN_INT | GCI_IOP_INT) ) {
		gc_debug(gci, "GC: in IRQ tail=%08lx, exechead=%08lx, "
				"head=%08lx.\n", gc_readl(gci, GCRBTR),
			gc_readl(gci, GCRBEXHR), gc_readl(gci, GCRBHR));

		gc_debug(gci, "GC: gccabaddr=%08lx, gctabaddr=%08lx,"
				" gcmabaddr=%08lx\n", gc_readl(gci, GCCABADDR),
			gc_readl(gci, GCTABADDR), gc_readl(gci, GCMABADDR));

		l = (unsigned long *)( ((unsigned long)gci->ringbuff_cpu) +
			(gc_readl(gci, GCRBEXHR) - gci->ringbuff_dma));
		dev_err(gci->dev, "GC: Error at command:\n");

		dev_err(gci->dev, "GC: Before: ");
		for(n=9; n > 0; n--)
			printk("%08lx ", *(l - n));
		printk("\n");

		dev_err(gci->dev, "GC: Command: ");
		n = (*l & 0x0F) + 1; /* lowest nibble is number of extra ulongs */
		if(n > 20)n = 20;
		for(; n > 0; n--)
			printk("%08lx ", *l++);

		dev_err(gci->dev, "GC: After: ");
		for(n=0; n < 8; n++)
			printk("%08lx ", *l++);
		printk("\n");

		/* Stop the controller, it will need to be reset. */
		gc_writel(gci, GCCR, gc_readl(gci, GCCR) | GCCR_STOP);
	}

	/* disable and clear the interrupts that occured */
	gc_writel(gci, GCIECR, gc_readl(gci, GCIECR) & ~gciscr);
	gc_writel(gci, GCISCR, gciscr);
	return IRQ_HANDLED;
}

/** Setup the ring buffer */
static int __devinit pxa3xx_gc_map_memory(struct pxa3xx_gc_info *gci) {

	/* The ring buffer loading is a kind-of DMA
	 * and I only know how to alloc that way */
	gci->ringbuff_size = PAGE_ALIGN(1024);	//1KB ought to do for now
	gci->ringbuff_cpu = dma_alloc_coherent(gci->dev, gci->ringbuff_size,
					&gci->ringbuff_dma, GFP_KERNEL);

	if (!gci->ringbuff_cpu){
		dev_err(gci->dev, "Failed to allocate ring buffer memory.\n");
		return -ENOMEM;
	}

	gc_debug(gci, "GC: Ringbuffer: virt=%p phys=%08x, size=%i\n",
			gci->ringbuff_cpu, gci->ringbuff_dma, gci->ringbuff_size);

	return 0;
}


static int __devinit pxa3xx_gc_probe(struct platform_device *pdev) {

	struct pxa3xx_gc_info *gci;
	struct resource *r;
	int irq, ret;

	#ifdef DEBUG
	printk("GC: pxa3xx_gc_probe\n");
	#endif

	ret = -ENOMEM;
	gci = kmalloc(sizeof(struct pxa3xx_gc_info), GFP_KERNEL);
	if (!gci)
		goto failed;

	memset(gci, 0, sizeof(struct pxa3xx_gc_info));
	gci->dev = &pdev->dev;
	#ifdef DEBUG
		gci->debug = -1;
	#else
		gci->debug = 0;
	#endif

	gci->clk = clk_get(&pdev->dev, "GCCLK");
	if (IS_ERR(gci->clk)) {
		ret = -EBUSY;
		goto failed_gci;
	}

	clk_enable(gci->clk);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no I/O memory resource defined\n");
		ret = -ENODEV;
		goto failed_gci;
	}

	r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		ret = -EBUSY;
		goto failed_gci;
	}

	gci->mmio_base = ioremap(r->start, r->end - r->start + 1);
	if (gci->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to map I/O memory\n");
		ret = -EBUSY;
		goto failed_free_res;
	}

	/* Initialize various buffer memory */
	ret = pxa3xx_gc_map_memory(gci);
	if (ret) {
		dev_err(&pdev->dev, "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto failed_free_io;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		ret = -ENODEV;
		goto failed_free_mem;
	}

	ret = request_irq(irq, pxa3xx_gc_handle_irq, IRQF_DISABLED, "GFX", gci);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed: %d\n", ret);
		ret = -EBUSY;
		goto failed_free_mem;
	}

	platform_set_drvdata(pdev, gci);

	ret = pxa3xx_gc_init_chardev(gci);
	if (ret)
		goto failed_free_irq;

	ret = pxa3xx_gc_init_sysdev(gci);
	if (ret)
		goto failed_free_cdev;

	/* Setup the ringbuffer regs, reset the GC and do a simple test */
	ret = pxa3xx_gc_full_reset(gci);
	if (ret)
		goto failed_free_cdev;

	/* pxa3xx_gc_super_tux(gci); */

	return 0;

failed_free_cdev:
	misc_deregister(&pxa3xx_gc_dev);
failed_free_irq:
	free_irq(irq, gci);
failed_free_mem:
	dma_free_writecombine(&pdev->dev, gci->ringbuff_size,
			gci->ringbuff_cpu, gci->ringbuff_dma);
failed_free_io:
	iounmap(gci->mmio_base);
failed_free_res:
	release_mem_region(r->start, r->end - r->start + 1);
failed_gci:
	clk_put(gci->clk);
	platform_set_drvdata(pdev, NULL);
	kfree(gci);
failed:
	return ret;

}

/* Module removal - (untested) */
static int pxa3xx_gc_remove(struct platform_device *pdev) {
	struct pxa3xx_gc_info *gci = platform_get_drvdata(pdev);

	misc_deregister(&pxa3xx_gc_dev);
	/* free_irq(irq, gci); */
	dma_free_writecombine(&pdev->dev, gci->ringbuff_size,
			gci->ringbuff_cpu, gci->ringbuff_dma);
	iounmap(gci->mmio_base);
	/* release_mem_region(r->start, r->end - r->start + 1); */
	clk_put(gci->clk);
	platform_set_drvdata(pdev, NULL);
	kfree(gci);

	return 0;
}

/** Obtain the phys address of the primary or first framebuffer */
static unsigned long pxa3xx_gc_get_lcd_base(struct pxa3xx_gc_info *gci) {

	struct fb_info *info = NULL;
	int i;

	/* Use the primary framebuffer if we have one, if not use the first */
	for(i=0; i < num_registered_fb; i++) {
		if (registered_fb[i] != NULL) {
			if (!info)
				info = registered_fb[i];
			if (fb_is_primary_device(registered_fb[i])) {
				gc_debug(gci, "GC: IS PRIMARY\n");
				info = registered_fb[i];
				break;
			}
		}
	}

	if (!info) {
		gc_debug(gci, "GC: No framebuffers found\n");
		return 0;
	}

	gc_debug(gci, "GC: Using LCD base phys addr = %08lx\n",
		info->fix.smem_start);
	return info->fix.smem_start;
}

/** If used during boot, enlarges tux to cover the whole screen */
static int pxa3xx_gc_super_tux(struct pxa3xx_gc_info *gci) {
	char str[PAGE_SIZE];
	unsigned long lcd_base;

	lcd_base = pxa3xx_gc_get_lcd_base(gci);
	pxa3xx_gc_cmd_set_buff(gci, GC_BUFFI_DEST0, lcd_base, 2, 960, PF_16B_RGB565);
	pxa3xx_gc_cmd_set_buff(gci, GC_BUFFI_SOURCE0, lcd_base, 2, 960, PF_16B_RGB565);

	/* need to copy it to the bottom right first,
	   so we don't overwrite as we go */
	pxa3xx_gc_cmd_copy_blt(gci, 0, 0, 380, 0, 100, 100);
	pxa3xx_gc_cmd_stretch_blt(gci,   0, 160, 480, 480,  0, 0, 100, 100);
	pxa3xx_gc_show_regs(gci->dev, NULL, str);
	printk(str);
	return 0;
}

/** This provides quite a good test as it performs lots of an operation
 * which takes longer to perform then to add, so it will run out
 * of ring-buffer and have to wait. */
static int pxa3xx_gc_do_tests(struct pxa3xx_gc_info *gci) {
	int i,j;
	unsigned long lcd_base, col;
	int full_count = 0;

	dev_dbg(gci->dev, "+pxa3xx_gc_do_tests()\n");
	/* clear the completion state since we're going to enable the IRQ handler */
	init_completion(&gci->allDoneOrError);
	gc_writel(gci, GCIECR, GCI_STOP_INT | GCI_PF_INT | GCI_EEOB_INT | GCI_IIN_INT |
		GCI_IOP_INT | GCI_BF_INT | GCI_IN_INT | GCI_EOB_INT);

	/* Get the primary/first framebuffer base physical address,
	 * This assumes it's in 16bpp RGB(565) at 480x640 here */
	lcd_base = pxa3xx_gc_get_lcd_base(gci);
	pxa3xx_gc_cmd_set_buff(gci, GC_BUFFI_DEST0, lcd_base, 2, 960,
							PF_16B_RGB565);
	pxa3xx_gc_cmd_set_buff(gci, GC_BUFFI_SOURCE0, lcd_base, 2, 960,
							PF_16B_RGB565);
	
	mdelay(10);

	for(i=0;i<540;i++)
		for(j=0;j<380;j++){
			col = 0xFF000000 | (i << 12) | (j << 0);
			if (pxa3xx_gc_cmd_color_fill(gci, j, i, 
						100, 100, col) < 0) {

				/* wait for it all to finish */
				full_count = 0;
				if (pxa3xx_gc_wait_all_complete(gci))
					goto tests_fail;

				/* now try again */
				if ( pxa3xx_gc_cmd_color_fill(gci, j, i, 
						100, 100, col) < 0){
					goto tests_fail;
				}
			}

			if ( gc_readl(gci, GCCR) & GCCR_STOP ) {
				dev_err(gci->dev, "GC: Detected GCCR_STOP at"
					" i=%i, j=%i, stopping.\n", i, j);
				return 0;
			}
		}

	pxa3xx_gc_wait_all_complete(gci);
	pxa3xx_gc_cmd_copy_blt(gci, 0, 0, 200, 100, 50, 50);
	gc_debug(gci, "GC: Tests complete OK. Ran out of buffer %i times.\n",
		full_count);


	return 0;

tests_fail:
	dev_err(gci->dev, "GC: Color fill/wait failed at i=%i, j=%i "
			"after/during waiting for all complete.\n", i, j);

	return -EFAULT;
}

static int pxa3xx_gc_suspend(struct platform_device *pdev,
						pm_message_t pmstate) {

	struct pxa3xx_gc_info *gci = platform_get_drvdata(pdev);

	/* try to wait for everything to finish */
	pxa3xx_gc_wait_all_complete(gci);

	/* clear interrupts, signal to abort anything left and stop altogether */
	gc_writel(gci, GCIECR, 0);
	gc_writel(gci, GCCR, gc_readl(gci, GCCR) | GCCR_ABORT);

	return 0;
}

static int pxa3xx_gc_resume(struct platform_device *pdev) {
	struct pxa3xx_gc_info *gci = platform_get_drvdata(pdev);

	/* Hopefully we completed everything before suspend
	*  so we can start from scratch here. */
	pxa3xx_gc_full_reset(gci);

	return 0;
}

static struct platform_driver pxa3xx_gc_driver = {
	.driver		= {	
		.owner	= THIS_MODULE,
		.name	= "pxa3xx-gcu",
	},
	.probe		= pxa3xx_gc_probe,
	.suspend	= pxa3xx_gc_suspend,
	.resume		= pxa3xx_gc_resume,
	.remove		= pxa3xx_gc_remove,
};

static int __init pxa3xx_gc_init(void)
{
	printk(KERN_INFO "PXA3xx 2D Graphics Basic Driver\n");
	platform_driver_register(&pxa3xx_gc_driver);

	return 0;
}

static void __exit pxa3xx_gc_exit(void)
{
        platform_driver_unregister(&pxa3xx_gc_driver);
}

module_init(pxa3xx_gc_init);
module_exit(pxa3xx_gc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oliver Ford  <ipaqcode-at-oliford-co-uk>");
MODULE_DESCRIPTION("PXA3xx 2D Graphics Basic Driver");


#endif
