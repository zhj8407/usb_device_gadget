/*
 * adv761x Analog Devices ADV761X HDMI receiver driver
 *
 * Copyright (C) 2013 Cogent Embedded, Inc.
 * Copyright (C) 2013 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <media/adv761x.h>
#include <media/soc_camera.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

//#define IS_VIRTUAL_DEV 1

#define ADV761X_DRIVER_NAME "adv761x"

/* VERT_FILTER_LOCKED and DE_REGEN_FILTER_LOCKED flags */
#define ADV761X_HDMI_F_LOCKED(v)	(((v) & 0xa0) == 0xa0)

/* Maximum supported resolution */
#define ADV761X_MAX_WIDTH		1920
#define ADV761X_MAX_HEIGHT		1080

/* Use SoC camera subdev desc private data for platform_data */
#define ADV761X_SOC_CAM_QUIRK		0x1

#ifdef IS_VIRTUAL_DEV
	#define  ADV761X_FMT_CHANGE_EVENT_NOTIFY_CTRL 	(V4L2_CID_DV_CLASS_BASE + 0x1003)
#endif

static int debug  = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

struct adv761x_color_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};


/* Supported color format list */
static const struct adv761x_color_format adv761x_cfmts[] = {
	{
		.code		= V4L2_MBUS_FMT_RGB888_1X24,
		.colorspace	= V4L2_COLORSPACE_SRGB,
	},
};

/* ADV761X descriptor structure */
struct adv761x_state {
	struct v4l2_subdev			sd;
	struct media_pad			pad;
	struct v4l2_ctrl_handler		ctrl_hdl;

	u8					edid[256];
	unsigned				edid_blocks;

	struct rw_semaphore			rwsem;
	const struct adv761x_color_format	*cfmt;
	u32					width;
	u32					height;
	enum v4l2_field				scanmode;
	u32					status;
	int					irq;

	struct workqueue_struct			*work_queue;
	struct delayed_work			enable_hotplug;
	struct work_struct			interrupt_service;

	struct i2c_client			*i2c_cec;
	struct i2c_client			*i2c_inf;
	struct i2c_client			*i2c_dpll;
	struct i2c_client			*i2c_rep;
	struct i2c_client			*i2c_edid;
	struct i2c_client			*i2c_hdmi;
	struct i2c_client			*i2c_cp;
#ifdef IS_VIRTUAL_DEV	
		/*Customized control*/
	struct v4l2_ctrl *test_ctrl;
#endif
};


static inline void adv761x_delay(unsigned int sec) {
	
	//unsigned long timeout = jiffies + HZ * sec; 
	//while (time_before(jiffies, timeout)) cpu_relax(  );
	 msleep_interruptible(sec * 1000);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct adv761x_state, ctrl_hdl)->sd;
}

static inline struct adv761x_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv761x_state, sd);
}

/* I2C I/O operations */
static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
	s32 ret = 0 ;
#ifndef IS_VIRTUAL_DEV	
	s32 i = 0;
	for (i = 0; i < 3; i++) {
		ret = i2c_smbus_read_byte_data(client, command);
		if (ret >= 0)
			return ret;
	}

	v4l_err(client, "Reading addr:%02x reg:%02x\n failed",
		client->addr, command);
#endif
	return ret;
}

static s32 adv_smbus_write_byte_data(struct i2c_client *client, u8 command,
				     u8 value)
{
	s32 ret = 0;
#ifndef IS_VIRTUAL_DEV	
	s32 i =0;
	for (i = 0; i < 3; i++) {
		ret = i2c_smbus_write_byte_data(client, command, value);
		if (!ret)
			return 0;
	}

	v4l_err(client, "Writing addr:%02x reg:%02x val:%02x failed\n",
		client->addr, command, value);
#endif
	return ret;
}

static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client, u8 command,
					  u8 length, const u8 *values)
{
	s32 ret = 0;
#ifndef IS_VIRTUAL_DEV	
	s32 i = 0;
	ret = i2c_smbus_write_i2c_block_data(client, command, length, values);
	if (!ret)
		return 0;

	for (i = 0; i < length; i++) {
		ret = adv_smbus_write_byte_data(client, command + i, values[i]);
		if (ret)
			break;
	}
#endif
	return ret;
}

static inline int io_read(struct v4l2_subdev *sd, u8 reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return adv_smbus_read_byte_data(client, reg);
}

static inline int io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return adv_smbus_write_byte_data(client, reg, val);
}

static inline int cec_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_cec, reg);
}

static inline int cec_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_cec, reg, val);
}

static inline int infoframe_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_inf, reg);
}

static inline int infoframe_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_inf, reg, val);
}

static inline int dpll_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_dpll, reg);
}

static inline int dpll_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_dpll, reg, val);
}

static inline int rep_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_rep, reg);
}

static inline int rep_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_rep, reg, val);
}

static inline int edid_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_edid, reg);
}

static inline int edid_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_edid, reg, val);
}

static inline int edid_write_block(struct v4l2_subdev *sd,
				   unsigned len, const u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv761x_state *state = to_state(sd);
	int ret = 0;
	int i;

	v4l2_dbg(2, debug, sd, "Writing EDID block (%d bytes)\n", len);

	v4l2_subdev_notify(sd, ADV761X_HOTPLUG, (void *)0);

	/* Disable I2C access to internal EDID ram from DDC port */
	rep_write(sd, 0x74, 0x0);

	for (i = 0; !ret && i < len; i += I2C_SMBUS_BLOCK_MAX)
		ret = adv_smbus_write_i2c_block_data(state->i2c_edid, i,
				I2C_SMBUS_BLOCK_MAX, val + i);
	if (ret)
		return ret;

	/*
	 * ADV761x calculates the checksums and enables I2C access
	 * to internal EDID ram from DDC port.
	 */
	rep_write(sd, 0x74, 0x01);

	for (i = 0; i < 1000; i++) {
		if (rep_read(sd, 0x76) & 0x1) {
			/* Enable hotplug after 100 ms */
			queue_delayed_work(state->work_queue,
					   &state->enable_hotplug, HZ / 10);
			return 0;
		}
		schedule();
	}

	v4l_err(client, "Enabling EDID failed\n");
	return -EIO;
}

static inline int hdmi_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_hdmi, reg);
}

static inline int hdmi_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_hdmi, reg, val);
}

static inline int cp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_read_byte_data(state->i2c_cp, reg);
}

static inline int cp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv761x_state *state = to_state(sd);

	return adv_smbus_write_byte_data(state->i2c_cp, reg, val);
}

static inline int adv761x_power_off(struct v4l2_subdev *sd)
{	
	io_write(sd,   0x0c, 0x62);//Power down part
//	io_write(sd,   0x0b, 0x47);//Power down part
	 
	 return  0;
}



static int adv761x_core_init(struct v4l2_subdev *sd)
{	
	
	v4l2_info(sd, "Enetr  adv761x_core_init().\n");
	
	io_write(sd,   0x0c, 0x62);//Power down part
	//io_write(sd,   0x0b, 0x47);//Power down part
	
	io_write(sd,  0x00, 0x02);	//Sets the input video standard mode as default value: 000010b .
	io_write(sd,  0x01, 0x06); //Prim_Mode = 0110b HDMI-GR,, V_FREQ[2:0], IO, Address 0x01[6:4]
	io_write(sd,  0x02, 0xf5); // Auto CSC, YCrCb out, Set op_656 bit
	io_write(sd,  0x03, 0x80); // 16-Bit SDR ITU-R BT.656 4:2:2 Mode 0
	io_write(sd,  0x04, 0xa2); //101 - P[23:16] V/R, P[15:8] U/CrCb/B, P[7:0] Y/G
	io_write(sd,   0x05, 0x28);//AV Codes Off
	io_write(sd,   0x06, 0xa6); // Invert VS,HS pins
	io_write(sd,   0x0b, 0x44);// Power up part

	io_write(sd,   0x0c, 0x42);//Power up part
	io_write(sd,   0x14, 0x7f);//Max Drive Strength
	io_write(sd,   0x15, 0x80); // Disable Tristate of Pins
	io_write(sd,   0x19, 0x83); //LLC DLL phase
	io_write(sd,   0x33, 0x40); //LLC DLL enable

	io_write(sd,   0x40, 0xe2);// (1)Active until cleared, (2)Drives high when active
	//io_write(sd,   0x6e, 0x03); //INT1 HDMI DE_REGEN and V_LOCK
	io_write(sd,   0x6e, 0x02); //INT1 V_LOCK
	//io_write(sd, 0x6e, 0x53); //(1)Enables Port A TMDSPLL Lock interrupt for INT1, (2)Enables Port A TMDS Clock Detection interrupt for INT1
	
	cp_write(sd,  0x6c, 0x00); //44 6C 00 ADI required setting
	cp_write(sd,  0x3e, 0x80); //	Enable color adjustments
	cp_write(sd,  0xba, 0x01); // Set HDMI FreeRun
	
	rep_write(sd,  0x40, 0x81); //Disable HDCP 1.1 features
	rep_write(sd,  0x74, 0x00); // Disable the Internal EDID for all ports
	
	/*hdmi 0x48 [6](default: 0000 0000)
		This control disables the reset effects of cable detection. DIS_CABLE_DET_RST should be set to 1 if the +5 V pins are
		unused and left unconnected.
		0 - Resets the HDMI section if the 5 V input pin corresponding to the selected HDMI port (e.g. RXA_5V for port A) is inactive
		1 - Do not use the 5 V input pins as reset signal for the HDMI section*/
	hdmi_write(sd,  0x48, 0x40);
	
	hdmi_write(sd, 0x9b, 0x03); //ADI recommended setting
	hdmi_write(sd, 0xc1, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc2, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc3, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc4, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc5, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc6, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc7, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc8, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xc9, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xca, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xcb, 0x01); //ADI recommended setting
	hdmi_write(sd, 0xcc, 0x01); //ADI recommended setting
	hdmi_write(sd, 0x00, 0x00); //Set HDMI Input Port A
	hdmi_write(sd, 0x83, 0xfe);  //Enable clock terminator for port A
	hdmi_write(sd, 0x6f, 0x0c); //ADI recommended setting
	hdmi_write(sd, 0x85, 0x1f); //ADI recommended setting
	hdmi_write(sd, 0x87, 0x70); //ADI recommended setting
	hdmi_write(sd, 0x8d, 0x04);//LFG
	hdmi_write(sd, 0x8e, 0x1e); //HFG
	hdmi_write(sd, 0x1a, 0x8a);//unmute audio
	hdmi_write(sd, 0x57, 0xda); //ADI recommended setting
	hdmi_write(sd, 0x58, 0x01);//ADI recommended setting
	hdmi_write(sd, 0x03, 0x18);//DIS_I2C_ZERO_COMPR
	hdmi_write(sd, 0x75, 0x10); //DDC drive strength

	
	//v4l2_info(sd, "VIC : %u\n", infoframe_read(sd, 0x4));
	
	v4l2_ctrl_handler_setup(sd->ctrl_handler);
	
	v4l2_info(sd, "Leave adv761x_core_init().\n");
	
	return 0;
}

static int adv761x_hdmi_info(struct v4l2_subdev *sd, enum v4l2_field *scanmode,
			     u32 *width, u32 *height)
{
	int msb, val;

	/* Line width */
	msb = hdmi_read(sd, 0x07);
	if (msb < 0)
		return msb;

	if (!ADV761X_HDMI_F_LOCKED(msb))
		return -EAGAIN;

	/* Interlaced or progressive */
	val = hdmi_read(sd, 0x0b);
	if (val < 0)
		return val;

	*scanmode = (val & 0x20) ? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;
	val = hdmi_read(sd, 0x08);
	if (val < 0)
		return val;

	val |= (msb & 0x1f) << 8;
	*width = val;

	/* Lines per frame */
	msb = hdmi_read(sd, 0x09);
	if (msb < 0)
		return msb;

	val = hdmi_read(sd, 0x0a);
	if (val < 0)
		return val;

	val |= (msb & 0x1f) << 8;
	if (*scanmode == V4L2_FIELD_INTERLACED)
		val <<= 1;
	*height = val;

	if (*width == 0 || *height == 0)
		return -EIO;

	return 0;
}

/* Hotplug work */
static void adv761x_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct adv761x_state *state = container_of(dwork, struct adv761x_state,
						   enable_hotplug);
	struct v4l2_subdev *sd = &state->sd;

	v4l2_dbg(2, debug, sd, "Enable hotplug\n");
	v4l2_subdev_notify(sd, ADV761X_HOTPLUG, (void *)1);
}

#define TDMSPLL_LCK_A_ST_MASK 0x40 
#define TDMS_CLK_A_ST_MASK 0x10 
#define  V_LOCKED_ST_MASK 0x02

/* IRQ work */
static void adv761x_interrupt_service(struct work_struct *work)
{
	struct adv761x_state *state = container_of(work, struct adv761x_state,
						   interrupt_service);
	struct v4l2_subdev *sd = &state->sd;
	u32 width = 0, height = 0;
#ifdef IS_VIRTUAL_DEV		
	static unsigned int flag = 0;
	
	if (flag == 2) flag = 0;
	
	if (flag == 0) {
		width = 1920; 
		height = 1080;
	
	} else {
		width = 1280;
		height =720;
	}
	
	flag++;
	state->status = 0;
	
	v4l2_info(sd, "Got the interrupt (%u, %u, %u)\n", flag, width, height);
#else	

	unsigned int status = 0;
	
	//v4l2_info(sd, "Enter  adv761x_interrupt_service()\n");
	
	status  = io_read(sd,  0x6b);
	//v4l2_info(sd, " Interrupt status =  0x%02x\n", status);
#if 0
	if (status & TDMSPLL_LCK_A_ST_MASK){
		val = io_read(sd, 0x6c);
		//v4l2_info(sd, "TMDSPLL_LCK_A_RAW val = 0x%02x\n", val);
		val = val & ~(TDMSPLL_LCK_A_ST_MASK);
		val = val | TDMSPLL_LCK_A_ST_MASK;
		io_write(sd, 0x6c, val);
		//v4l2_info(sd, "TMDSPLL_LCK_A_RAW has been changed (val = 0x%02x) !!\n", val);
		
		do {
			val = io_read(sd, 0x6a);
			val = val & TDMSPLL_LCK_A_ST_MASK;
		} while (retries-- && val != TDMSPLL_LCK_A_ST_MASK);
		v4l2_info(sd, "TMDSPLL_LCK_A_RAW has been changed (val = 0x%02x) !!\n", val);
	}
	
	retries = 100;
	
	if (status & TDMS_CLK_A_ST_MASK)	{
		val = io_read(sd, 0x6c);
		//v4l2_info(sd, "TMDS_CLK_A_RAW val = 0x%02x\n", val);
		val = val & ~(TDMS_CLK_A_ST_MASK);
		val = val | TDMS_CLK_A_ST_MASK;
		io_write(sd, 0x6c, val);
		//v4l2_info(sd, "TMDS_CLK_A_RAW has been changed (val = 0x%02x) !!\n", val);
		do {
			val = io_read(sd, 0x6a);
			val = val & TDMS_CLK_A_ST_MASK;
		} while (retries-- && val != TDMS_CLK_A_ST_MASK);
		v4l2_info(sd, "TMDS_CLK_A_RAW has been changed (val = 0x%02x) !!\n", val);
	}
#endif
	io_write(sd, 0x6c, 0xff);

	if (status & V_LOCKED_ST_MASK) {
		
		enum v4l2_field scanmode;
		u32 hdmi_status = 0;
		int ret = 0;
		
		ret = adv761x_hdmi_info(sd, &scanmode, &width, &height);
		if (ret) {
			width = ADV761X_MAX_WIDTH;
			height = ADV761X_MAX_HEIGHT;
			scanmode = V4L2_FIELD_NONE;
			hdmi_status = V4L2_IN_ST_NO_SIGNAL;
		}
#if 0
		if (hdmi_status){
			v4l2_info(sd, "No HDMI video input detected\n");
		} else {
			v4l2_info(sd, "HDMI video input detected (%ux%u%c)(VIC:%u)\n", width, height,  scanmode == V4L2_FIELD_NONE ? 'p' : 'i',  infoframe_read(sd, 0x4));
		}
#endif
		down_write(&state->rwsem);
		state->width = width;
		state->height = height;
		state->scanmode = scanmode;
		state->status = hdmi_status;
		up_write(&state->rwsem);

	}
#endif	

	if (!state->status && (state->scanmode  == V4L2_FIELD_NONE) ) {
		struct adv761x_fmt_event_data event_data;
		v4l2_info(sd, "Attach the video input device. Change FMT to (%u, %u)\n", width, height);
		event_data.width = width;
		event_data.height = height;
		v4l2_subdev_notify(sd, ADV761X_FMT_CHANGE, &event_data);
	} else {
		if (status & V_LOCKED_ST_MASK) {
			struct adv761x_fmt_event_data event_data;
			v4l2_info(sd, "Detach the video input device.\n");
			event_data.width = 0;
			event_data.height = 0;
			v4l2_subdev_notify(sd, ADV761X_FMT_CHANGE, &event_data);
		}
	}
	//v4l2_info(sd, "Leave  adv761x_interrupt_service()\n");
}

/* IRQ handler */
static irqreturn_t adv761x_irq_handler(int irq, void *devid)
{
	struct adv761x_state *state = (struct adv761x_state *)devid;
	//struct v4l2_subdev *sd = &state->sd;
	//v4l2_info(sd, "Got interrupt!!\n");
	queue_work(state->work_queue, &state->interrupt_service);
	return IRQ_HANDLED;
}

/* v4l2_subdev_core_ops */
#ifdef CONFIG_VIDEO_ADV_DEBUG
static void adv761x_inv_register(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "0x000-0x0ff: IO Map\n");
	v4l2_info(sd, "0x100-0x1ff: CEC Map\n");
	v4l2_info(sd, "0x200-0x2ff: InfoFrame Map\n");
	v4l2_info(sd, "0x300-0x3ff: DPLL Map\n");
	v4l2_info(sd, "0x400-0x4ff: Repeater Map\n");
	v4l2_info(sd, "0x500-0x5ff: EDID Map\n");
	v4l2_info(sd, "0x600-0x6ff: HDMI Map\n");
	v4l2_info(sd, "0x700-0x7ff: CP Map\n");
}

static int adv761x_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	reg->size = 1;
	switch (reg->reg >> 8) {
	case 0:
		reg->val = io_read(sd, reg->reg & 0xff);
		break;
	case 1:
		reg->val = cec_read(sd, reg->reg & 0xff);
		break;
	case 2:
		reg->val = infoframe_read(sd, reg->reg & 0xff);
		break;
	case 3:
		reg->val = dpll_read(sd, reg->reg & 0xff);
		break;
	case 4:
		reg->val = rep_read(sd, reg->reg & 0xff);
		break;
	case 5:
		reg->val = edid_read(sd, reg->reg & 0xff);
		break;
	case 6:
		reg->val = hdmi_read(sd, reg->reg & 0xff);
		break;
	case 7:
		reg->val = cp_read(sd, reg->reg & 0xff);
		break;
	default:
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		adv761x_inv_register(sd);
		break;
	}
	return 0;
}

static int adv761x_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	switch (reg->reg >> 8) {
	case 0:
		io_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 1:
		cec_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 2:
		infoframe_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 3:
		dpll_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 4:
		rep_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 5:
		edid_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 6:
		hdmi_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	case 7:
		cp_write(sd, reg->reg & 0xff, reg->val & 0xff);
		break;
	default:
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		adv761x_inv_register(sd);
		break;
	}
	return 0;
}
#endif	/* CONFIG_VIDEO_ADV_DEBUG */

/* v4l2_subdev_video_ops */
static int adv761x_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct adv761x_state *state = to_state(sd);

	down_read(&state->rwsem);
	*status = state->status;
	up_read(&state->rwsem);
	return 0;
}

static int adv761x_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *mf)
{
	struct adv761x_state *state = to_state(sd);

	down_read(&state->rwsem);
	mf->width = state->width;
	mf->height = state->height;
	mf->field = state->scanmode;
	mf->code = state->cfmt->code;
	mf->colorspace = state->cfmt->colorspace;
	up_read(&state->rwsem);
	return 0;
}

static int adv761x_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	/* Check requested format index is within range */
	if (index >= ARRAY_SIZE(adv761x_cfmts))
		return -EINVAL;

	*code = adv761x_cfmts[index].code;

	return 0;
}

static int adv761x_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_MASTER |
		V4L2_MBUS_VSYNC_ACTIVE_LOW | V4L2_MBUS_HSYNC_ACTIVE_LOW |
		V4L2_MBUS_DATA_ACTIVE_HIGH;
	cfg->type = V4L2_MBUS_PARALLEL;

	return 0;
}

/* v4l2_subdev_pad_ops */
static int adv761x_get_edid(struct v4l2_subdev *sd,
			    struct v4l2_subdev_edid *edid)
{
	struct adv761x_state *state = to_state(sd);

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->blocks == 0)
		return -EINVAL;

	if (edid->start_block >= state->edid_blocks)
		return -EINVAL;

	if (edid->start_block + edid->blocks > state->edid_blocks)
		edid->blocks = state->edid_blocks - edid->start_block;
	if (!edid->edid)
		return -EINVAL;

	memcpy(edid->edid + edid->start_block * 128,
	       state->edid + edid->start_block * 128,
	       edid->blocks * 128);
	return 0;
}

static int adv761x_set_edid(struct v4l2_subdev *sd,
			    struct v4l2_subdev_edid *edid)
{
	struct adv761x_state *state = to_state(sd);
	int ret;

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks == 0) {
		/* Pull down the hotplug pin */
		v4l2_subdev_notify(sd, ADV761X_HOTPLUG, (void *)0);
		/* Disable I2C access to internal EDID RAM from DDC port */
		rep_write(sd, 0x74, 0x0);
		state->edid_blocks = 0;
		return 0;
	}

	if (edid->blocks > 2)
		return -E2BIG;

	if (!edid->edid)
		return -EINVAL;

	memcpy(state->edid, edid->edid, 128 * edid->blocks);
	state->edid_blocks = edid->blocks;

	ret = edid_write_block(sd, 128 * edid->blocks, state->edid);
	if (ret < 0)
		v4l2_err(sd, "Writing EDID failed\n");

	return ret;
}

/* v4l2_ctrl_ops */
static int adv761x_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	u8 val = ctrl->val;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = cp_write(sd, 0x3c, val);
		break;
	case V4L2_CID_CONTRAST:
		ret = cp_write(sd, 0x3a, val);
		break;
	case V4L2_CID_SATURATION:
		ret = cp_write(sd, 0x3b, val);
		break;
	case V4L2_CID_HUE:
		ret = cp_write(sd, 0x3d, val);
		break;
#ifdef IS_VIRTUAL_DEV		
	case ADV761X_FMT_CHANGE_EVENT_NOTIFY_CTRL :
		if (ctrl->val == 0) {
			struct adv761x_fmt_event_data event_data;
			event_data.width = 1920;
			event_data.height = 1080;
			v4l2_subdev_notify(sd, ADV761X_FMT_CHANGE, &event_data);
			//v4l2_info(sd,"[adv761x]ADV761X_FMT_CHANGE_EVENT_NOTIFY_CTRL (%d, %u,%u)\n", ctrl->val, event_data.width, event_data.height);
		} else if (ctrl->val == 1) {
			struct adv761x_fmt_event_data event_data;
			event_data.width = 1280;
			event_data.height = 720;
			v4l2_subdev_notify(sd, ADV761X_FMT_CHANGE, &event_data);
			//v4l2_info(sd,"[adv761x]ADV761X_FMT_CHANGE_EVENT_NOTIFY_CTRL (%d, %u,%u)\n", ctrl->val, event_data.width, event_data.height);
		}
		break;	
#endif		
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* V4L structures */

static const struct v4l2_ctrl_ops adv761x_ctrl_ops = {
	.s_ctrl = adv761x_s_ctrl,
};

static const struct v4l2_subdev_core_ops adv761x_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= adv761x_g_register,
	.s_register	= adv761x_s_register,
#endif
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
};

static const struct v4l2_subdev_video_ops adv761x_video_ops = {
	.g_input_status = adv761x_g_input_status,
	.g_mbus_fmt	= adv761x_g_mbus_fmt,
	.try_mbus_fmt	= adv761x_g_mbus_fmt,
	.s_mbus_fmt	= adv761x_g_mbus_fmt,
	.enum_mbus_fmt	= adv761x_enum_mbus_fmt,
	.g_mbus_config	= adv761x_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops adv761x_pad_ops = {
	.get_edid = adv761x_get_edid,
	.set_edid = adv761x_set_edid,
};

static const struct v4l2_subdev_ops adv761x_ops = {
	.core = &adv761x_core_ops,
	.video = &adv761x_video_ops,
	.pad = &adv761x_pad_ops,
};

/* Device initialization and clean-up */
static void adv761x_unregister_clients(struct adv761x_state *state)
{
	if (state->i2c_cec)
		i2c_unregister_device(state->i2c_cec);
	if (state->i2c_inf)
		i2c_unregister_device(state->i2c_inf);
	if (state->i2c_dpll)
		i2c_unregister_device(state->i2c_dpll);
	if (state->i2c_rep)
		i2c_unregister_device(state->i2c_rep);
	if (state->i2c_edid)
		i2c_unregister_device(state->i2c_edid);
	if (state->i2c_hdmi)
		i2c_unregister_device(state->i2c_hdmi);
	if (state->i2c_cp)
		i2c_unregister_device(state->i2c_cp);
}

#ifndef  IS_VIRTUAL_DEV
static struct i2c_client *adv761x_dummy_client(struct v4l2_subdev *sd,
					       u8 addr, u8 def_addr, u8 io_reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
//	static unsigned int is_first_run = 1;
	
	if (!addr)
		addr = def_addr;
#if 0
	if (is_first_run) {
		io_write(sd, 0xff, 0x80);// I2C reset
		is_first_run = 0;
	}
#endif
	io_write(sd, io_reg, addr << 1);
	return i2c_new_dummy(client->adapter, addr);
}
#endif

static inline int adv761x_check_rev(struct i2c_client *client)
{
#ifndef IS_VIRTUAL_DEV	
	int msb, rev;

	msb = adv_smbus_read_byte_data(client, 0xea);
	if (msb < 0)
		return msb;

	rev = adv_smbus_read_byte_data(client, 0xeb);
	if (rev < 0)
		return rev;

	rev |= msb << 8;

	switch (rev) {
	case 0x2051:
		return 7611;
	case 0x2041:
		return 7612;
	default:
		break;
	}
	return -ENODEV;
#else
	return  0;	
#endif	
}
/////////////////////////////////////////////////////////////
#ifdef IS_VIRTUAL_DEV
static const struct v4l2_ctrl_config adv761x_ctrl_test = {
	.ops = &adv761x_ctrl_ops,
	.id = ADV761X_FMT_CHANGE_EVENT_NOTIFY_CTRL,
	.name = "Testing FMT-changing event control",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0,
	.max = 0xff,
	.step = 0x1,
	.def = 0x7,
};
#endif
////////////////////////////////////////////////////////////
static int adv761x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adv761x_platform_data *pdata;
	struct adv761x_state *state;
	struct v4l2_ctrl_handler *ctrl_hdl;
	struct v4l2_subdev *sd;
	int irq = -1, ret = -1;
#ifndef  IS_VIRTUAL_DEV
	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
#endif
	/* Check chip revision */
	ret = adv761x_check_rev(client);
	if (ret < 0) {
		v4l_err(client, "Chip not found !!\n");
		return ret;
	}
	
	v4l_info(client, "Chip found @ 0x%02x (adv%d)\n", client->addr, ret);

	
	/* Get platform data */
	if (id->driver_data == ADV761X_SOC_CAM_QUIRK) {
		struct soc_camera_subdev_desc *ssdd;

		v4l_info(client, "Using SoC camera glue\n");
		ssdd = soc_camera_i2c_to_desc(client);
		pdata = ssdd ? ssdd->drv_priv : NULL;
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		v4l_err(client, "No platform data found\n");
		return -ENODEV;
	}

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (state == NULL) {
		v4l_err(client, "Memory allocation failed\n");
		return -ENOMEM;
	}

	init_rwsem(&state->rwsem);

	/* Setup default values */
	state->cfmt = &adv761x_cfmts[0];
	state->width = ADV761X_MAX_WIDTH;
	state->height = ADV761X_MAX_HEIGHT;
	state->scanmode = V4L2_FIELD_NONE;
	state->status = V4L2_IN_ST_NO_SIGNAL;
	state->irq = -1;
	/* Setup subdev */
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &adv761x_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#ifndef  IS_VIRTUAL_DEV
	/* Setup I2C clients */
	
	//i2cset  -y 0x4 0x4c  0x1b 0x1
	//if ( client->addr == 0x4c)  io_write(sd, 0x1b, 0x1);// I2C reset
	v4l2_info(sd, "Chip ID High Byte Chip ID High Byte (should be 0x20):%02x\n", 	io_read(sd, 0xea));
	v4l2_info(sd, "Chip ID Low Byte Chip ID Low Byte (shoudl be 0x51):%02x\n", 	io_read(sd, 0xeb));
	//NOTE: When setting the reg 0xff, there will be writing error. 
	//io_write(sd, 0xff, 0x80);// I2C reset

	//adv761x_delay(1);
	state->i2c_cec = adv761x_dummy_client(sd, pdata->i2c_cec, 0x40, 0xf4);
	state->i2c_inf = adv761x_dummy_client(sd, pdata->i2c_inf, 0x3e, 0xf5);
	state->i2c_dpll = adv761x_dummy_client(sd, pdata->i2c_dpll, 0x26, 0xf8);
	state->i2c_rep = adv761x_dummy_client(sd, pdata->i2c_rep, 0x32, 0xf9);
	state->i2c_edid = adv761x_dummy_client(sd, pdata->i2c_edid, 0x36, 0xfa);
	state->i2c_hdmi = adv761x_dummy_client(sd, pdata->i2c_hdmi, 0x34, 0xfb);
	state->i2c_cp = adv761x_dummy_client(sd, pdata->i2c_cp, 0x22, 0xfd);
	if (!state->i2c_cec || !state->i2c_inf || !state->i2c_dpll ||
	    !state->i2c_rep || !state->i2c_edid ||
	    !state->i2c_hdmi || !state->i2c_cp) {
		ret = -ENODEV;
		v4l2_err(sd, "I2C clients setup failed\n");
		goto err_i2c;
	}
#endif
	/* Setup control handlers */
	ctrl_hdl = &state->ctrl_hdl;
	v4l2_ctrl_handler_init(ctrl_hdl, 4);
	v4l2_ctrl_new_std(ctrl_hdl, &adv761x_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
	v4l2_ctrl_new_std(ctrl_hdl, &adv761x_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 255, 1, 128);
	v4l2_ctrl_new_std(ctrl_hdl, &adv761x_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std(ctrl_hdl, &adv761x_ctrl_ops,
			  V4L2_CID_HUE, 0, 255, 1, 0);
#ifdef IS_VIRTUAL_DEV		
	state->test_ctrl = v4l2_ctrl_new_custom(ctrl_hdl, &adv761x_ctrl_test, NULL);
	state->test_ctrl->is_private = false;
#endif
	sd->ctrl_handler = ctrl_hdl;
	if (ctrl_hdl->error) {
		ret = ctrl_hdl->error;
		v4l2_err(sd, "Control handlers setup failed\n");
		goto err_hdl;
	}
	/* Setup media entity */
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 1, &state->pad, 0);
	if (ret) {
		v4l2_err(sd, "Media entity setup failed\n");
		goto err_hdl;
	}

	/* Setup work queue */
	state->work_queue = create_singlethread_workqueue(client->name);
	if (!state->work_queue) {
		ret = -ENOMEM;
		v4l2_err(sd, "Work queue setup failed\n");
		goto err_entity;
	}

	INIT_DELAYED_WORK(&state->enable_hotplug, adv761x_enable_hotplug);
	INIT_WORK(&state->interrupt_service, adv761x_interrupt_service);

	/* Setup IRQ */
	
	irq = gpio_to_irq(pdata->gpio);
	
	if (irq <= 0) {
		ret = -ENODEV;
		v4l_err(client, "IRQ not found\n");
		goto err_gpio;
	}

	ret = request_irq(irq, adv761x_irq_handler, IRQF_SHARED | IRQF_TRIGGER_RISING,
			  ADV761X_DRIVER_NAME, state);
	if (ret) {
		v4l_err(client, "IRQ setup failed\n");
		goto err_gpio;
	}
	v4l2_info(sd, "Set GPIO %d as interrupt %d.\n", pdata->gpio, irq);
	state->irq = irq;
	
	/* Setup core registers */
	ret = adv761x_core_init(sd);
	if (ret < 0) {
		v4l_err(client, "Core setup failed\n");
		goto err_core;
	}
	
	return 0;

err_core:
	adv761x_power_off(sd);
err_gpio:
	if (state->irq != -1) free_irq(state->irq, state);
	cancel_work_sync(&state->interrupt_service);
	cancel_delayed_work_sync(&state->enable_hotplug);
	destroy_workqueue(state->work_queue);
err_entity:
	media_entity_cleanup(&sd->entity);
err_hdl:
	v4l2_ctrl_handler_free(ctrl_hdl);
#ifndef  IS_VIRTUAL_DEV	
err_i2c:
	adv761x_unregister_clients(state);
#endif
	return ret;
}

static int adv761x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv761x_state *state = to_state(sd);
	
	v4l2_info(sd, "%s removed @ 0x%x (%s)\n", client->name, client->addr, client->adapter->name);
	
	/* Release IRQ/GPIO */
	if (state->irq != -1) free_irq(state->irq, state);

	/* Destroy workqueue */
	cancel_work_sync(&state->interrupt_service);
	cancel_delayed_work_sync(&state->enable_hotplug);
	destroy_workqueue(state->work_queue);

	/* Power off */
	adv761x_power_off(sd);

	/* Clean up*/
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	adv761x_unregister_clients(state);
	
	return 0;
}

static const struct i2c_device_id adv761x_id[] = {
	{ "adv761x", 0 },
	{ "adv761x-soc_cam", ADV761X_SOC_CAM_QUIRK },
	{ },
};

MODULE_DEVICE_TABLE(i2c, adv761x_id);

static struct i2c_driver adv761x_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= ADV761X_DRIVER_NAME,
	},
	.probe		= adv761x_probe,
	.remove		= adv761x_remove,
	.id_table	= adv761x_id,
};

module_i2c_driver(adv761x_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ADV761X HDMI receiver video decoder driver");
MODULE_AUTHOR("Valentine Barshak <valentine.barshak@cogentembedded.com>");
