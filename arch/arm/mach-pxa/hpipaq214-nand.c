/*
 * linux/arch/arm/mach-pxa/hpipaq214-nand.c
 *
 * NAND Flash Support for the Hewlett-Packard iPAQ 21x.
 *
 * Copyright (C) 2008 Oliver Ford <ipaqcode-at-oliford-co-uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * The OBM and initial areas are used for booting and should NEVER need to be
 * overwritten. To be totally sure, anywhere before 0x40000 is write protected
 * here and this is forced at low level in pxa3xx-nand.c.
 *
 * Everywhere after 0x40000 can be used but overwriting it
 * will stop windows booting. Even with a direct recovery with 'dd' etc
 * will not make windows work again since it stores data in the OOB.
 *
 * The 'IPQ1' area can be safely overwritten as it will only destroy what 
 * winCE calls the 'iPAQ File Store'. Windows re-format it when it boots.
 *
 * There are the following possible layouts (in drivers/mtd/nand/Kconfig):
 * 
 * CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_ALL_IN_ONE is useful for development/debugging. Be VERY careful when using this.
 * CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_WIN sets the layout as windows has it - useful for just overwriting the IPQ1 area
 * CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_LINUX gives a layout useful for having linux as the only OS on the device
 */

#include <linux/kernel.h>

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <plat/pxa3xx_nand.h>

#include "generic.h"


#if defined(CONFIG_MTD_NAND_HPIPAQ214)


#if defined(CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_ALL_IN_ONE)

static struct mtd_partition hpipaq214_nand_partitions[] = {
	[0] = {
		.name        = "TheWhole",
		.offset      = 0x00000000,	/* Single partition covering whole flash chip */
		.size        = 0x10000000,	/* mostly for backing up winCE etc already on */
		.mask_flags  = MTD_WRITEABLE,	/* the device. */
	},
};

#else // !CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_ALL_IN_ONE
static struct mtd_partition hpipaq214_nand_partitions[] = {
	/* Ignore everything important like OBM */
#if defined(CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_LINUX)
	{
		.name        = "u-boot",	/* U-Boot bootloader*/
		.offset      = 0x00040000,	/* The OBM will only load this much (256KiB) */
		.size        = 0x00040000,
	},
	{
		.name        = "boot",		/* Android boot.img 4MB */
		.offset      = 0x00080000,
		.size        = 0x00400000,
	},
	{
		.name        = "recovery",	/* Android recovery.img 4MB */
		.offset      = 0x00480000,
		.size        = 0x00400000,
	},
	{
		.name        = "misc",		/* misc partition 0.5MB */
		.offset      = 0x00880000,
		.size        = 0x00080000,
	},
	{
		.name        = "persist",	/* persist 1.5MB */
		.offset      = 0x00900000,
		.size        = 0x00180000,
	},
	{
		.name        = "system",	/* system partition 128MB */
		.offset      = 0x00A80000,
		.size        = 0x08000000,
	},
	{
		.name        = "userdata",	/* userdata partition 96MB */
		.offset      = 0x08A80000,
		.size        = 0x06000000,
	},
	{
		.name        = "cache",		/* cache partition 18MB */
		.offset      = 0x0EA80000,
		.size        = 0x01200000,
	},
#elif defined(CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_WIN)
	{
		.name        = "Bootloader",	/* Windows CE Bootloader stages 1 and 2 */
		.offset      = 0x00040000,
	},
	{
		.name        = "ARGS",		/* ARGS Structure used by WinCE bootloader up to 0xA1000*/
		.offset      = 0x000A0000,	/* and then unknown (must be included to be in same erase block) */
		.size        = 0x00120000,
	},
	{
		.name        = "DSK1",		/* Main windows block device 'DSK1' */
		.offset      = 0x001C0000,
		.size        = 0x0E1C0000,
	},
	{
		.name        = "IPQ1",		/* Second windows block device 'IPQ1'/'iPAQ File Store' */
		.offset      = 0x0E380000,
		.size        = 0x01900000,
	},
#else
	/* Otherwise just leave the OBM */
#endif // end CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_WIN/LIN
	/* Unknown stuff up to the end, might be important for OBM? */
	/* Or probably the wear-levelling things... */
};
#endif // end !CONFIG_MTD_NAND_HPIPAQ214_LAYOUT_ALL_IN_ONE

/* Flash Timings for the k9f2g08UXA */
static struct pxa3xx_nand_timing samsung_k9f2g08UXA_timing = { //using 1.8V settings
	.tCH	= 10, //min 5
	.tCS	= 25,
	.tWH	= 15,
	.tWP	= 21,
	.tRH	= 15, //min 10
	.tRP	= 25, //min 21
	.tR	= 25000,
	.tWHR	= 60,
	.tAR	= 10,
};

/* Large page command set for the k9f2g08UXA */
static struct pxa3xx_nand_cmdset largepage_cmdset = {
	.read1		= 0x3000,
	.read2		= 0x0050,
	.program	= 0x1080,
	.read_status	= 0x0070,
	.read_id	= 0x0090,
	.erase		= 0xD060,
	.reset		= 0x00FF,
	.lock		= 0x002A,
	.unlock		= 0x2423,
	.lock_status	= 0x007A,
};

/* Flash chip details for the k9f2g08UXA */
static struct pxa3xx_nand_flash hpipaq214_flash_types[] = {
	{ // samsung_k9f2g08UXA
		.timing		= &samsung_k9f2g08UXA_timing,
		.cmdset		= &largepage_cmdset,	//read2, lock, unlock and  lock_status not verified correct
		.page_per_block	= 64,
		.page_size	= 2048,
		.flash_width	= 8,
		.dfc_width	= 8,
		.num_blocks	= 2048,
		.chip_id	= 0xaaec,
	},

};

static struct pxa3xx_nand_platform_data hpipaq214_nand_info = {
	.enable_arbiter	= 2,
	.flash		= hpipaq214_flash_types,
	.num_flash	= ARRAY_SIZE(hpipaq214_flash_types),
	.parts		= hpipaq214_nand_partitions,
	.nr_parts	= ARRAY_SIZE(hpipaq214_nand_partitions),
};

void __init hpipaq214_init_nand(void)
{
	pxa3xx_set_nand_info(&hpipaq214_nand_info);
}
#else
void hpipaq214_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_HPIPAQ214 */

