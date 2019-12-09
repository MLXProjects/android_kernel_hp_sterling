
#define GCBASE		0x54000000

#define GCCR		0x00	/* Configuration */
#define	GCISCR		0x04	/* Interrupt Status */
#define	GCIECR		0x08	/* Interrupt Enable */
#define GCNOPID		0x0C	/* NOP ID */
#define GCALPHASET	0x10	/* Default Alpha Value */
#define GCTSET		0x14	/* Default Transparency */

#define GCRBBR		0x20	/* Ring Buffer Base Address */
#define GCRBLR		0x24	/* Ring Buffer Length */
#define GCRBHR		0x28	/* Ring Buffer Head */
#define GCRBTR		0x2C	/* Ring Buffer Tail */
#define GCRBEXHR	0x30	/* Ring Buffer Execution Head */

#define GCBBBR		0x40	/* Batch Buffer Base Address */
#define GCBBHR		0x44	/* Batch Buffer Head */
#define GCBBEXHR	0x48	/* Batch Buffer Execution Head */

#define GCD0BR		0x60	/* Destination 0 Base Address */
#define GCD0STP		0x64	/* Destination 0 Step Size */
#define GCD0STR		0x68	/* Destination 0 Stride Size */
#define GCD0PF		0x6C	/* Destination 0 Pixel Type */

#define GCD1BR		0x70	/* Destination 1 Base Address */
#define GCD1STP		0x74	/* Destination 1 Step Size */
#define GCD1STR		0x78	/* Destination 1 Stride Size */
#define GCD1PF		0x7C	/* Destination 1 Pixel Type */

#define GCD2BR		0x80	/* Destination 2 Base Address */
#define GCD2STP		0x84	/* Destination 2 Step Size */
#define GCD2STR		0x88	/* Destination 2 Stride Size */
#define GCD2PF		0x8C	/* Destination 2 Pixel Type */

#define GCS0BR		0xE0	/* Source 0 Base Address */
#define GCS0STP		0xE4	/* Source 0 Step Size */
#define GCS0STR		0xE8	/* Source 0 Stride Size */
#define GCS0PF		0xEC	/* Source 0 Pixel Type */

#define GCS1BR		0xF0	/* Source 1 Base Address */
#define GCS1STP		0xF4	/* Source 1 Step Size */
#define GCS1STR		0xF8	/* Source 1 Stride Size */
#define GCS1PF		0xFC	/* Source 1 Pixel Type */

#define GCCABADDR	0x1E0	/* Illegal Access Bad Address */
#define GCTABADDR	0x1E4	/* Target Abort */
#define GCMABADDR	0x1E8	/* Master Abort */

#define GCSCxWDy(x,y)	(0x160 + (x)*8 + (y))

#define	GCCR_INT_ERR_EN		(1 << 10)
#define	GCCR_SYNC_CLR		(1 << 9)
#define	GCCR_BP_RST		(1 << 8)
#define	GCCR_ABORT		(1 << 6)
#define	GCCR_STOP		(1 << 4)
#define	GCCR_CURR_DEST		2
#define	GCCR_CURR_DEST_MASK	0x03
#define	GCCR_DEST		0
#define	GCCR_DEST_MASK		0x03

/** Interrupt bits common to both GCISCR and GCIECR */
#define GCI_IN_INT_ID		12
#define GCI_STOP_INT		(1 << 7)
#define GCI_PF_INT		(1 << 6)
#define GCI_EEOB_INT		(1 << 5)
#define GCI_IIN_INT		(1 << 4)
#define GCI_IOP_INT		(1 << 3)
#define GCI_BF_INT		(1 << 2)
#define GCI_IN_INT		(1 << 1)
#define GCI_EOB_INT		(1 << 0)

/* Pixel formats */
#define PF_8B_LOOKUP		0x00
#define PF_15B_RGB555		0x01
#define	PF_16B_RGBT5551		0x02
#define	PF_16B_RGB565		0x03
#define	PF_18B_RGB666		0x04
#define	PF_19B_RGBT6661		0x05
#define	PF_24B_RGB888		0x06
#define	PF_24B_RGBA6666		0x07
#define	PF_24B_RGBT8881		0x08
#define	PF_32B_RGBA8888		0x09
#define	PF_48B_RGB161616	0x0A
#define	PF_64B_RGBA16161616	0x0B

/** Instruction Opcodes */
#define	GCI_BIT		29
#define	GCI_MASK	0x7
#define	GCI_MEMORY	(0x0 << GCI_BIT)
#define	GCI_2DGFX	(0x2 << GCI_BIT)

#define OPCODE_BIT	24
#define OPCODE_MASK	0x1F
#define	GC_BBST		(GCI_MEMORY | (0x00 << OPCODE_BIT))	/* Batch Buffer Start */
#define	GC_BBEND	(GCI_MEMORY | (0x01 << OPCODE_BIT))	/* Batch Buffer nd */
#define	GC_BUFFI	(GCI_MEMORY | (0x02 << OPCODE_BIT))	/* Buffer Info */
#define	GC_LREG		(GCI_MEMORY | (0x04 << OPCODE_BIT))	/* Load Register */
#define	GC_NOP		(GCI_MEMORY | (0x05 << OPCODE_BIT))	/* NOP */
#define	GC_DBFLIP	(GCI_MEMORY | (0x06 << OPCODE_BIT))	/* Destination Buffer Flip */
#define	GC_STREG	(GCI_MEMORY | (0x07 << OPCODE_BIT))	/* Store Register */	
#define	GC_INT		(GCI_MEMORY | (0x08 << OPCODE_BIT))	/* Interrupt CPU */
#define	GC_WAIT		(GCI_MEMORY | (0x09 << OPCODE_BIT))	/* Wait For Event */

#define GC_CFILL	(GCI_2DGFX | (0x00 << OPCODE_BIT))	/* Colour Fill */
#define GC_CKBLT	(GCI_2DGFX | (0x01 << OPCODE_BIT))	/* Chroma Key BLT */
#define GC_LINE		(GCI_2DGFX | (0x02 << OPCODE_BIT))	/* Line Draw */
#define GC_AALINE	(GCI_2DGFX | (0x03 << OPCODE_BIT))	/* Antialiased Line Draw */
#define GC_STRBLT	(GCI_2DGFX | (0x05 << OPCODE_BIT))	/* Strech BLT */
#define GC_ABLND	(GCI_2DGFX | (0x07 << OPCODE_BIT))	/* Alpha Blend BLT */
#define GC_SCALE	(GCI_2DGFX | (0x08 << OPCODE_BIT))	/* Scale BLT */
#define GC_BIAS		(GCI_2DGFX | (0x09 << OPCODE_BIT))	/* Bias BLT */
#define GC_RAST		(GCI_2DGFX | (0x0B << OPCODE_BIT))	/* Raster OP BLT */
#define GC_PATT		(GCI_2DGFX | (0x0C << OPCODE_BIT))	/* Pattern Copy BLT */
#define GC_DECBLT	(GCI_2DGFX | (0x0D << OPCODE_BIT))	/* Deceminate BLT */

/** Buffer identifies for GC_BUFFI */
#define GC_BUFFI_ADDR_BIT	4
#define GC_BUFFI_SOURCE0	0x00
#define GC_BUFFI_SOURCE1	0x01
#define GC_BUFFI_DEST0		0x08
#define GC_BUFFI_DEST1		0x09
#define GC_BUFFI_DEST2		0x0A

/* Immeadiate color value rather than register */
#define GC_CFILL_IMM		(1 << 4)

struct pxa3xx_gc_info {
	struct device		*dev;
	void __iomem		*mmio_base;
	struct clk		*clk;

	/* raw memory addresses */
	dma_addr_t		ringbuff_dma;	/* physical */
	u_char *		ringbuff_cpu;	/* virtual */
	u_int			ringbuff_size;

	atomic_t		cdevOpenCount;	/* open count for char dev */
	int 			debug;	 	/* true or false */

	struct completion	allDoneOrError;

};

static int pxa3xx_gc_ringbuff_add(struct pxa3xx_gc_info *gci, unsigned long *data, int len);
static int pxa3xx_gc_ringbuff_free(struct pxa3xx_gc_info *gci);
static inline int pxa3xx_gc_cmd_copy_blt(struct pxa3xx_gc_info *gci, 
			int s0x, int s0y, int dx, int dy, int w, int h);
static int pxa3xx_gc_cmd_raster_blt(struct pxa3xx_gc_info *gci, 
				int s0x, int s0y, int s1x, int s1y,
				int dx, int dy, int w, int h, unsigned char op);
static int pxa3xx_gc_cmd_set_buff(struct pxa3xx_gc_info *gci, unsigned int buff, 
				unsigned long base, unsigned char step,
				unsigned int stride, unsigned char pixelFormat);
static int pxa3xx_gc_cmd_nop(struct pxa3xx_gc_info *gci, unsigned long nop_id);
static int pxa3xx_gc_do_tests(struct pxa3xx_gc_info *gci);
static int pxa3xx_gc_wait_all_complete(struct pxa3xx_gc_info *gci);
static int pxa3xx_gc_test_ready(struct pxa3xx_gc_info *gci);
static void pxa3xx_gc_setup_ringbuff(struct pxa3xx_gc_info *gci);
static int pxa3xx_gc_full_reset(struct pxa3xx_gc_info *gci);
static int pxa3xx_gc_super_tux(struct pxa3xx_gc_info *gci);
