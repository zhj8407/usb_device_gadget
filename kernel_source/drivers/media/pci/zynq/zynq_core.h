#ifndef ZYNQ_CORE_H
#define ZYNQ_CORE_H

#include <linux/io.h>
#include <linux/videodev2.h>
#include <linux/pci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
/* Header files */
#include "zynq_types.h"
#include "modules/zynq_regs.h"
#include "zynq_fpga_verify.h"
#include "zynq_debug.h"
#include "modules/zynq_scaler.h"
#include "zynq_control.h"
/* Maximum channel allowed */
#define VPIF_NUM_CHANNELS		(4)
#define VPIF_CAPTURE_NUM_CHANNELS	(6)
#define VPIF_DISPLAY_NUM_CHANNELS	(1)

extern unsigned int g_video_cap_nr[VPIF_CAPTURE_NUM_CHANNELS];
extern unsigned int g_video_cap_en[VPIF_CAPTURE_NUM_CHANNELS];
extern unsigned int g_video_display_nr[VPIF_DISPLAY_NUM_CHANNELS];
extern unsigned int g_video_display_en[VPIF_DISPLAY_NUM_CHANNELS];
extern unsigned int g_video_control_nr[1];
extern  int  m10mo_get_resolution(unsigned int *width, unsigned int *height);

/* Macros to read/write registers */
extern void __iomem *zynq_reg_base;
extern u32 zynq_reg_len;
extern spinlock_t vpif_lock;

extern void __iomem *zynq_video_input_window_base;
extern u32 zynq_video_input_window_base_dma_handle;
extern u32 zynq_video_input_window_len;

//The followings operations are for "bar_type is memory".
#define regr(reg)               readl((reg) + zynq_reg_base)
#define regw(value, reg)        writel(value, (reg + zynq_reg_base))

//The followings operations are for "bar_type is I/O port".
#define ioregr(reg)	 		inl((reg) + res_start)
#define ioregw(value, reg)        outl(value, (reg + res_start))

/* Functions for bit Manipulation */
static inline void vpif_set_bit(u32 reg, u32 bit)
{
    regw((regr(reg)) | (0x01 << bit), reg);
}

static inline void vpif_clr_bit(u32 reg, u32 bit)
{
    regw(((regr(reg)) & ~(0x01 << bit)), reg);
}

/* Macro for Generating mask */
#ifdef GENERATE_MASK
#undef GENERATE_MASK
#endif

#define GENERATE_MASK(bits, pos) \
		((((0xFFFFFFFF) << (32 - bits)) >> (32 - bits)) << pos)


extern u8 irq_vpif_capture_channel[VPIF_NUM_CHANNELS];
///////////////////////////////////////////////////////////////////////////////

/*

 void fpga_reg_write(void __iomem *base, u32 reg, u32 val);
u32 fpga_reg_read(void __iomem *base, u32  reg);
void fpga_reg_rmw(void __iomem *base, u32 reg, u32 clr_bits, u32 set_bits);
void fpga_reg_rmw_clr(void __iomem *base, u32 reg, u32 val);
void fpga_reg_rmw_set(void __iomem *base, u32 reg, u32 val);
 */


static inline void enable_channel0_video(int enable) {
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, 0);
    } else {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, FPGA_VIDEO0_READY_MASK);
    }
}

static inline void enable_channel1_video(int enable) {
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, 0);
    } else {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, FPGA_VIDEO1_READY_MASK);
    }

}

static inline void enable_channel2_video(int enable) {
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, 0);
    } else {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, FPGA_VIDEO2_READY_MASK);
    }
}

static inline void enable_channel3_video(int enable) {
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, 0);
    } else {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, FPGA_VIDEO3_READY_MASK);
    }
}

static inline void enable_channel4_video(int enable) {
	void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, 0);
    } else {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, FPGA_VIDEO6_READY_MASK);
    }
}

static inline void enable_channel5_video(int enable) {
	void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, 0);
    } else {
        fpga_reg_write(base, FPGA_VIDEO_READY_REG, FPGA_VIDEO7_READY_MASK);
    }
}
///////////////////////////////////////////////////////////////////////////////

static inline void enable_channel0_intr(int enable) {
	void __iomem *base =   zynq_reg_base;
//	fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO0_INTERRUPT_MASK);
    if (enable == 0) {
        //	fpga_reg_rmw_clr(base, FPGA_INTERRUPT_REG, FPGA_VIDEO0_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO0_INTERRUPT_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO0_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO0_INTERRUPT_MASK | FPGA_VIDEO0_INTERRUPT_ST_MASK);
    }
    return;
}


static inline void enable_channel1_intr(int enable) {
    void __iomem *base =   zynq_reg_base;
    //fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO1_INTERRUPT_MASK);
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_INTERRUPT_REG, FPGA_VIDEO1_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO1_INTERRUPT_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO1_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO1_INTERRUPT_MASK | FPGA_VIDEO1_INTERRUPT_ST_MASK);
    }
    return;

}

static inline void enable_channel2_intr(int enable) {
    void __iomem *base =   zynq_reg_base;
    //fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO2_INTERRUPT_MASK);
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_INTERRUPT_REG, FPGA_VIDEO2_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO2_INTERRUPT_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO2_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO2_INTERRUPT_MASK | FPGA_VIDEO2_INTERRUPT_ST_MASK);
    }
    return;
}


static inline void enable_channel3_intr(int enable) {
    void __iomem *base =   zynq_reg_base;
    //fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO3_INTERRUPT_MASK);
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_INTERRUPT_REG, FPGA_VIDEO3_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO3_INTERRUPT_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_INTERRUPT_REG, FPGA_VIDEO3_INTERRUPT_ST_MASK);
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO3_INTERRUPT_MASK | FPGA_VIDEO3_INTERRUPT_ST_MASK);
    }
    return;
}

static inline void enable_channel4_intr(int enable) {
	void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO6_INTERRUPT_MASK);
    } else {
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO6_INTERRUPT_MASK | FPGA_VIDEO6_INTERRUPT_ST_MASK);
    }
    return;
}

static inline void enable_channel5_intr(int enable) {
		void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO7_INTERRUPT_MASK);
    } else {
        fpga_reg_write(base, FPGA_INTERRUPT_REG,  FPGA_VIDEO7_INTERRUPT_MASK | FPGA_VIDEO7_INTERRUPT_ST_MASK);
    }
    return;
}
///////////////////////////////////////////////////////////////////////////////

/* inline function to enable/disable channel0 */
static inline void enable_channel0(int enable)
{
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO0_PCI_PUSH_READY_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO0_PCI_PUSH_READY_MASK);
        fpga_reg_write(base,  FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO0_PCI_PUSH_READY_MASK);
    }
    return;
}

/* inline function to enable/disable channel1 */
static inline void enable_channel1(int enable)
{
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO1_PCI_PUSH_READY_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO1_PCI_PUSH_READY_MASK);
        fpga_reg_write(base,  FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO1_PCI_PUSH_READY_MASK);
    }
    return;
}

/* inline function to enable/disable channel2 */
static inline void enable_channel2(int enable)
{
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO2_PCI_PUSH_READY_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO2_PCI_PUSH_READY_MASK);
        fpga_reg_write(base,  FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO2_PCI_PUSH_READY_MASK);
    }
    return;
}

/* inline function to enable/disable channel3 */
static inline void enable_channel3(int enable)
{
    void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO3_PCI_PUSH_READY_MASK);
    } else {
        //fpga_reg_rmw_set(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO3_PCI_PUSH_READY_MASK);
        fpga_reg_write(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO3_PCI_PUSH_READY_MASK);
    }
    return;
}
static inline void enable_channel4(int enable) {
	void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO6_PCI_PUSH_READY_MASK);
    } else {
        fpga_reg_write(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO6_PCI_PUSH_READY_MASK);
    }
    return;
}
static inline void enable_channel5(int enable) {
	void __iomem *base =   zynq_reg_base;
    if (enable == 0) {
        //fpga_reg_rmw_clr(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO7_PCI_PUSH_READY_MASK);
    } else {
        fpga_reg_write(base, FPGA_PCI_PUSH_READY_REG, FPGA_VIDEO7_PCI_PUSH_READY_MASK);
    }
    return;
}
///////////////////////////////////////////////////////////////////////////////
static inline int ch0_set_videobuf_res(unsigned long width, unsigned long height) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base,  FPGA_VIDEO_RES_REG, ((width << FPGA_VIDEO_RES_WIDTH_OFFSET) | height));
	return 0;
}

static inline int ch1_set_videobuf_res(unsigned long width, unsigned long height) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base,  FPGA_VIDEO_RES_REG, ((width << FPGA_VIDEO_RES_WIDTH_OFFSET) | height));
	return 0;
}
#if 0
static inline int  webcam_stream_enable(unsigned int enable) {
  	u32 val = 0;
	
    if (!zynq_reg_base) return -1;

	val = fpga_reg_read(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG);
	if (enable)
			fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffff7) | 0x8);
	else
			fpga_reg_write(zynq_reg_base, FPGA_VIDEO_STREAM_ENABLE_REG, (val&0xfffffff7) | 0x0);
	
    return 0;	
}
#endif
static inline int webcam_set_videobuf_res(unsigned long width, unsigned long height) {
#if 0
	void __iomem *base =   zynq_reg_base;
	unsigned int in_width =(unsigned int)-1;
    unsigned int in_height =(unsigned int)-1;
	
	m10mo_get_resolution(&in_width, &in_height);
	
	if ((in_width == (unsigned int) -1) ||  (in_height == (unsigned int) -1)) {
		zynq_printk(0, "[zynq_core] Could not set the resolution of web cam !!\n");
		return;
	} else {
		zynq_printk(0, "[zynq_core] Set the resolution of web cam from (%u, %u) to (%lu, %lu)\n", in_width, in_height, width, height);
	}
	webcam_stream_enable(0);
	fpga_reg_write(base,  FPGA_WEBCAM_VIDEO_RES_REG, ((width << FPGA_WEBCAM_VIDEO_RES_WIDTH_OFFSET) | height));
	webcam_stream_enable(1);
#endif
	return vpif_control_config_webcam_res(width, height );
	
}

static inline int ch2_set_videobuf_res(unsigned long width, unsigned long height) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base,  FPGA_VIDEO_RES_REG, ((width << FPGA_VIDEO_RES_WIDTH_OFFSET) | height));
	return 0;
}

static inline int ch3_set_videobuf_res(unsigned long width, unsigned long height) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base,  FPGA_VIDEO_RES_REG, ((width << FPGA_VIDEO_RES_WIDTH_OFFSET) | height));
	return 0;

}
static inline int ch4_set_videobuf_res(unsigned long width, unsigned long height) {
	void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base,  FPGA_VIDEO_RES_REG, ((width << FPGA_VIDEO_RES_WIDTH_OFFSET) | height));
	return 0;
}
static inline int ch5_set_videobuf_res(unsigned long width, unsigned long height) {
	void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base,  FPGA_VIDEO_RES_REG, ((width << FPGA_VIDEO_RES_WIDTH_OFFSET) | height));
	return 0;
}
///////////////////////////////////////////////////////////////////////////////

static inline void ch0_set_videobuf_addr(unsigned long y_addr, unsigned long uv_addr) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base, FPGA_VIDEO0_Y_ADDR_REG, y_addr);
    fpga_reg_write(base, FPGA_VIDEO0_UV_ADDR_REG, uv_addr);
    return;
}

static inline void ch1_set_videobuf_addr(unsigned long y_addr, unsigned long uv_addr) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base, FPGA_VIDEO1_Y_ADDR_REG, y_addr);
    fpga_reg_write(base, FPGA_VIDEO1_UV_ADDR_REG, uv_addr);
    return;
}

static inline void ch2_set_videobuf_addr(unsigned long y_addr, unsigned long uv_addr) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base, FPGA_VIDEO2_Y_ADDR_REG, y_addr);
    fpga_reg_write(base, FPGA_VIDEO2_UV_ADDR_REG, uv_addr);
    return;
}

static inline void ch3_set_videobuf_addr(unsigned long y_addr, unsigned long uv_addr) {
    void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base, FPGA_VIDEO3_Y_ADDR_REG, y_addr);
    fpga_reg_write(base, FPGA_VIDEO3_UV_ADDR_REG, uv_addr);
    return;
}

static inline void ch4_set_videobuf_addr(unsigned long y_addr, unsigned long uv_addr) {
	void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base, FPGA_VIDEO6_Y_ADDR_REG, y_addr);
    fpga_reg_write(base, FPGA_VIDEO6_UV_ADDR_REG, uv_addr);
    return;
}

static inline void ch5_set_videobuf_addr(unsigned long y_addr, unsigned long uv_addr) {
	void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base, FPGA_VIDEO7_Y_ADDR_REG, y_addr);
    fpga_reg_write(base, FPGA_VIDEO7_UV_ADDR_REG, uv_addr);
    return;
}

///////////////////////////////////////////////////////////////////////////////

static inline int vpif_intr_status(int channel)
{
    int status = 0;

    return status;
}

//////////////////////////////////////////////////////////////////////////////////

#define VPIF_MAX_NAME	(30)

/* This structure will store size parameters as per the mode selected by user */
struct vpif_channel_config_params {
    char name[VPIF_MAX_NAME];	/* Name of the mode */
    u16 width;						/* Indicates width of the image */
    u16 height;						/* Indicates height of the image */
    u8 frm_fmt;						/* Interlaced (0) or progressive (1) */
    u16 eav2sav;					/* length of eav 2 sav */
    u16 sav2eav;					/* length of sav 2 eav */
    u16 l1, l3, l5, l7, l9, l11;	/* Other parameter configurations */
    u16 vsize;							/* Vertical size of the image */
    u8 capture_format;		/* Indicates whether capture format is in BT or in CCD/CMOS */
    u8  vbi_supported;		/* Indicates whether this mode supports capturing vbi or not */
    u8 hd_sd;							/* HDTV (1) or SDTV (0) format */
    v4l2_std_id stdid;			/* SDTV format */
    struct v4l2_dv_timings dv_timings;	/* HDTV format */
    u32 pixelformat;
};

extern const unsigned int vpif_ch_params_count;
extern const struct vpif_channel_config_params vpif_ch_params[];

struct vpif_video_params;
struct vpif_params;


int vpif_set_video_params(struct vpif_params *vpifparams, u8 channel_id);
int vpif_pci_probe(struct pci_dev *pdev);
int vpif_pci_remove(struct pci_dev *pdev);

#ifdef CONFIG_PM_SLEEP
int vpif_pci_suspend(struct pci_dev *pdev);
int vpif_pci_resume(struct pci_dev *pdev);
#endif

enum data_size {
    _8BITS = 0,
    _10BITS,
    _12BITS,
};


/* structure for vpif parameters */
struct vpif_video_params {
    __u8 storage_mode;	/* Indicates field or frame mode */
    unsigned long hpitch;
    v4l2_std_id stdid;
};

struct vpif_params {
    struct vpif_interface iface;
    struct vpif_video_params video_params;
    struct vpif_channel_config_params std_info;
    union param {
        enum data_size data_sz;
    } params;
};

#endif				/* End of #ifndef VPIF_H */

