#ifndef ZYNQ_CAPTURE_PRIV_H
#define ZYNQ_CAPTURE_PRIV_H

#ifdef __KERNEL__
#include <media/adv761x.h>
#include "zynq_types.h"
#include "zynq_core.h"
#include "zynq_board.h"
#include "zynq_capture.h"
/* Macros */
#define VPIF_CAPTURE_VERSION		"0.0.0.1"

#define VPIF_VALID_FIELD(field)		(((V4L2_FIELD_ANY == field) || \
	(V4L2_FIELD_NONE == field)) || \
	(((V4L2_FIELD_INTERLACED == field) || \
	(V4L2_FIELD_SEQ_TB == field)) || \
	(V4L2_FIELD_SEQ_BT == field)))
	
#define VPIF_VIDEO_INDEX		0
#define VPIF_NUMBER_OF_OBJECTS		1
/* Enumerated data type to give id to each device per channel */
enum vpif_channel_id {
    VPIF_CHANNEL0_VIDEO = 0,
    VPIF_CHANNEL1_VIDEO = 1,
    VPIF_CHANNEL2_VIDEO = 2,
    VPIF_CHANNEL3_VIDEO = 3,
	VPIF_CHANNEL4_VIDEO = 4,
	VPIF_CHANNEL5_VIDEO = 5,
};

struct video_obj {
    enum v4l2_field buf_field;
    /* Currently selected or default standard */
    v4l2_std_id stdid;
    struct v4l2_dv_timings dv_timings;
};

struct vpif_cap_buffer {
    struct vb2_buffer vb;
    struct list_head list;
};

struct common_obj {
    /* Pointer pointing to current v4l2_buffer */
    struct vpif_cap_buffer *cur_frm;
    /* Pointer pointing to current v4l2_buffer */
    struct vpif_cap_buffer *next_frm;
    /*
     * This field keeps track of type of buffer exchange mechanism
     * user has selected
     */
    enum v4l2_memory memory;
    /* Used to store pixel format */
    struct v4l2_format fmt;
    /* Buffer queue used in video-buf */
    struct vb2_queue buffer_queue;
    /* allocator-specific contexts for each plane */
    struct vb2_alloc_ctx *alloc_ctx;
    /* Queue of filled frames */
    struct list_head dma_queue;
    /* Used in video-buf */
    spinlock_t irqlock;
    /* lock used to access this structure */
    struct mutex lock;
    /* number of users performing IO */
    u32 io_usrs;
    /* Indicates whether streaming started */
    u8 started;
    /* Function pointer to set the addresses */
    void (*set_addr) (unsigned long, unsigned long);
    int (*set_res) (unsigned long, unsigned long);
    void (*enable_channel) (int);
    void (*enable_channel_intr)(int);
    void (*enable_channel_video)(int);

    /* offset where starts from the starting of the Y buffer */
    u32 y_off;
    /* offset where starts from the starting of the UV buffer */
    u32 uv_off;
    /* Indicates width of the image data */
    u32 width;
    /* Indicates height of the image data */
    u32 height;

    u32 dummy_buffer_handle;
    char *dummy_buffer;
    unsigned int dummy_buffer_size;

    u8 is_start_streaming;
    u8 is_stop_streaming;

    atomic_t refcount;
	
	unsigned int reqbuf_count; 
};

struct channel_obj {
    /* Identifies video device for this channel */
    struct video_device *video_dev;
    /* Used to keep track of state of the priority */
    struct v4l2_prio_state prio;
    /* number of open instances of the channel */
    int usrs;
    /* Indicates id of the field which is being displayed */
    u32 field_id;
    /* flag to indicate whether decoder is initialized */
    u8 initialized;
    /* Identifies channel */
    enum vpif_channel_id channel_id;
    /* Current input */
    u32 input_idx;
    /* subdev corresponding to the current input, may be NULL */
    struct v4l2_subdev *sd;
    /* vpif configuration params */
    struct vpif_params vpifparams;
    /* common object array */
    struct common_obj common[VPIF_NUMBER_OF_OBJECTS];
    /* video object */
    struct video_obj video;

    struct mutex chan_lock;

    struct workqueue_struct *event_proc_work_queues;
    struct delayed_work delayed_event_proc_work;
    struct adv761x_fmt_event_data fmt_change_event_data;

    struct workqueue_struct			*work_queue;
    struct work_struct			interrupt_service;
    char  work_queue_name[128];
	
	u32 interrupt_count;
	u32 interrupt_dummy_buffer_count;
	
	unsigned int en_non_cache_map;
};

/* File handle structure */
struct vpif_fh {
    struct v4l2_fh fh;
    /* pointer to channel object for opened device */
    struct channel_obj *channel;
    /* Indicates whether this file handle is doing IO */
    u8 io_allowed[VPIF_NUMBER_OF_OBJECTS];
    /* Used to keep track priority of this instance */
    enum v4l2_priority prio;
    /* Used to indicate channel is initialize or not */
    u8 initialized;

};

struct vpif_config_params {
    u8 min_numbuffers;
    u8 numbuffers[VPIF_CAPTURE_NUM_CHANNELS];
    s8 device_type;
    u32 min_bufsize[VPIF_CAPTURE_NUM_CHANNELS];
    u32 channel_bufsize[VPIF_CAPTURE_NUM_CHANNELS];
    u8 default_device[VPIF_CAPTURE_NUM_CHANNELS];
    u32 video_limit[VPIF_CAPTURE_NUM_CHANNELS];
    u8 max_device_type;
    u32 channel_bufstride[VPIF_CAPTURE_NUM_CHANNELS];
    u32  pixelformat[VPIF_CAPTURE_NUM_CHANNELS];
};

const struct vpif_input dm6467_ch0_inputs[] = {
#if 1
    {
        .input = {
            .index = 0,
            .name = "HDMI",
            .type = V4L2_INPUT_TYPE_CAMERA,
            .capabilities = V4L2_IN_CAP_DV_TIMINGS,
            .std =  V4L2_STD_UNKNOWN,
        },
        .subdev_name = SUBDEV_CH0,
        .input_route = 0,
        .output_route = 0
    },
#endif
};

const struct vpif_input dm6467_ch1_inputs[] = {
#if 1
    {
        .input = {
            .index = 0,
            .name = "HDMI",
            .type = V4L2_INPUT_TYPE_CAMERA,
            .capabilities = V4L2_IN_CAP_DV_TIMINGS,
            .std =  V4L2_STD_UNKNOWN,
        },
        .subdev_name = SUBDEV_CH1,
        .input_route = 0,
        .output_route = 0
    },
#endif
};

const struct vpif_input dm6467_ch2_inputs[] = {
#if 1
    {
        .input = {
            .index = 0,
            .name = "HDMI",
            .type = V4L2_INPUT_TYPE_CAMERA,
            .capabilities = V4L2_IN_CAP_DV_TIMINGS,
            .std =  V4L2_STD_UNKNOWN,
        },
        .subdev_name = SUBDEV_CH2,
        .input_route = 0,
        .output_route = 0
    },
#endif
};

const struct vpif_input dm6467_ch3_inputs[] = {
#if 1
    {
        .input = {
            .index = 0,
            .name = "HDMI",
            .type = V4L2_INPUT_TYPE_CAMERA,
            .capabilities = V4L2_IN_CAP_DV_TIMINGS,
            .std =  V4L2_STD_UNKNOWN,
        },
        .subdev_name = SUBDEV_CH3,
        .input_route = 0,
        .output_route = 0
    },
#endif
};


const struct vpif_input dm6467_ch4_inputs[] = {
#if 1
    {
        .input = {
            .index = 0,
            .name = "HDMI",
            .type = V4L2_INPUT_TYPE_CAMERA,
            .capabilities = V4L2_IN_CAP_DV_TIMINGS,
            .std =  V4L2_STD_UNKNOWN,
        },
        .subdev_name =NULL,
        .input_route = 0,
        .output_route = 0
    },
#endif
};


const struct vpif_input dm6467_ch5_inputs[] = {
#if 1
    {
        .input = {
            .index = 0,
            .name = "HDMI",
            .type = V4L2_INPUT_TYPE_CAMERA,
            .capabilities = V4L2_IN_CAP_DV_TIMINGS,
            .std =  V4L2_STD_UNKNOWN,
        },
        .subdev_name = NULL,
        .input_route = 0,
        .output_route = 0
    },
#endif
};

int setup_vpif_input_path(int channel, const char *sub_dev_name)
{

    return 0;
}

int setup_vpif_input_channel_mode(int mux_mode)
{
    return 0;
}

/*
 VPIF_IF_RAW_BAYER --> Indicates this interface/channel supports raw capture from sensor

VPIF_IF_BT1120 --> Indicates this interface/channel supports HD

VPIF_IF_BT656--> Indicates this interface/channel supports SD to be precise NTSC & PAL.

 */
struct vpif_capture_config  vpif_capture_cfg = {
    .setup_input_path = setup_vpif_input_path,
    .setup_input_channel_mode = setup_vpif_input_channel_mode,
    .subdev_info = board_subdev_info,
    .subdev_count = 3, // board_subdev_info_num
    .chan_config[0] = {
        .inputs = dm6467_ch0_inputs,
        .input_count = ARRAY_SIZE(dm6467_ch0_inputs),
        .vpif_if = {
            .if_type = VPIF_IF_BT1120,
            .hd_pol = 1,
            .vd_pol = 1,
            .fid_pol = 0,
        },
    },
    .chan_config[1] = {
        .inputs = dm6467_ch1_inputs,
        .input_count = ARRAY_SIZE(dm6467_ch1_inputs),
        .vpif_if = {
            .if_type = VPIF_IF_BT1120,
            .hd_pol = 1,
            .vd_pol = 1,
            .fid_pol = 0,
        },dm6467_ch3_inputs
    },
    .chan_config[2] = {
        .inputs = dm6467_ch2_inputs,
        .input_count = ARRAY_SIZE(dm6467_ch2_inputs),
        .vpif_if = {
            .if_type =VPIF_IF_BT1120,
            .hd_pol = 1,
            .vd_pol = 1,
            .fid_pol = 0,
        },
    },
    .chan_config[3] = {
        .inputs = dm6467_ch3_inputs,
        .input_count = ARRAY_SIZE(dm6467_ch3_inputs),
        .vpif_if = {
            .if_type = VPIF_IF_BT1120,
            .hd_pol = 1,
            .vd_pol = 1,
            .fid_pol = 0,
        },
    },
	.chan_config[4] = {
        .inputs = dm6467_ch4_inputs,
        .input_count = ARRAY_SIZE(dm6467_ch4_inputs),
        .vpif_if = {
            .if_type = VPIF_IF_BT1120,
            .hd_pol = 1,
            .vd_pol = 1,
            .fid_pol = 0,
        },
    },
	.chan_config[5] = {
        .inputs = dm6467_ch5_inputs,
        .input_count = ARRAY_SIZE(dm6467_ch5_inputs),
        .vpif_if = {
            .if_type = VPIF_IF_BT1120,
            .hd_pol = 1,
            .vd_pol = 1,
            .fid_pol = 0,
        },
    },
    .card_name = "Zynq Video Capture",
};

#endif				/* End of __KERNEL__ */
#endif				/* VPIF_CAPTURE_H */
