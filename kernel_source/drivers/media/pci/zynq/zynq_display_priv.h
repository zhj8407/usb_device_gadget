#ifndef	ZYNQ_DISPLAY_PRIV_H
#define ZYNQ_DISPLAY_PRIV_H

#include "zynq_types.h"
#include "zynq_core.h"
#include "zynq_display.h"

#define VPIF_DISPLAY_VERSION "0.0.0.1"
#define VPIF_DISPLAY_CARD_NAME "Zynq Video Display"

#define VPIF_VALID_FIELD(field) \
	(((V4L2_FIELD_ANY == field) || (V4L2_FIELD_NONE == field)) || \
	(((V4L2_FIELD_INTERLACED == field) || (V4L2_FIELD_SEQ_TB == field)) || \
	(V4L2_FIELD_SEQ_BT == field)))

#define VPIF_DISPLAY_MAX_DEVICES	(1)
#define VPIF_SLICED_BUF_SIZE		(256)
#define VPIF_SLICED_MAX_SERVICES	(3)
#define VPIF_VIDEO_INDEX		(0)
#define VPIF_VBI_INDEX			(1)
#define VPIF_HBI_INDEX			(2)

/* Setting it to 1 as HBI/VBI support yet to be added , else 3*/
#define VPIF_NUMOBJECTS	 1

/* Macros */
#define ISALIGNED(a)    (0 == ((a) & 7))

/* enumerated data types */
/* Enumerated data type to give id to each device per channel */
enum vpif_channel_id {
    VPIF_CHANNEL0_VIDEO = 0,	/* Channel2 Video */
    VPIF_CHANNEL1_VIDEO,		/* Channel3 Video */
};

/* structures */

struct video_obj {
    enum v4l2_field buf_field;
    u32 latest_only;		/* indicate whether to return
					 * most recent displayed frame only */
    v4l2_std_id stdid;		/* Currently selected or default
					 * standard */
    struct v4l2_dv_timings dv_timings;
};

struct vpif_disp_buffer {
    struct vb2_buffer vb;
    struct list_head list;
};

struct common_obj {
    /* Buffer specific parameters */
    u8 *fbuffers[VIDEO_MAX_FRAME];		/* List of buffer pointers for
						 * storing frames */
    u32 numbuffers;				/* number of buffers */
    struct vpif_disp_buffer *cur_frm;	/* Pointer pointing to current
						 * vb2_buffer */
    struct vpif_disp_buffer *next_frm;	/* Pointer pointing to next
						 * vb2_buffer */
    enum v4l2_memory memory;		/* This field keeps track of
						 * type of buffer exchange
						 * method user has selected */
    struct v4l2_format fmt;			/* Used to store the format */
    struct vb2_queue buffer_queue;		/* Buffer queue used in
						 * video-buf */
    /* allocator-specific contexts for each plane */
    struct vb2_alloc_ctx *alloc_ctx;

    struct list_head dma_queue;		/* Queue of filled frames */
    spinlock_t irqlock;			/* Used in video-buf */

    /* channel specific parameters */
    struct mutex lock;			/* lock used to access this
						 * structure */
    u32 io_usrs;				/* number of users performing
						 * IO */
    u8 started;				/* Indicates whether streaming
						 * started */
    u32 ytop_off;				/* offset of Y top from the
						 * starting of the buffer */
    u32 ybtm_off;				/* offset of Y bottom from the
						 * starting of the buffer */
    u32 ctop_off;				/* offset of C top from the
						 * starting of the buffer */
    u32 cbtm_off;				/* offset of C bottom from the
						 * starting of the buffer */

    /* Function pointer to set the addresses */
    void (*set_addr) (unsigned long);
    void (*set_res) (unsigned long, unsigned long);
    void (*enable_read) (unsigned int);
    void (*enable_channel_intr)(int);

    u32 height;
    u32 width;

    u32 dummy_buffer_handle;
    char *dummy_buffer;
    unsigned int dummy_buffer_size;

    atomic_t refcount;

};

struct channel_obj {
    /* V4l2 specific parameters */
    struct video_device *video_dev;	/* Identifies video device for
					 * this channel */
    struct v4l2_prio_state prio;	/* Used to keep track of state of
					 * the priority */
    atomic_t usrs;			/* number of open instances of
					 * the channel */
    u32 field_id;			/* Indicates id of the field
					 * which is being displayed */
    u8 initialized;			/* flag to indicate whether
					 * encoder is initialized */
    u32 output_idx;			/* Current output index */
    struct v4l2_subdev *sd;		/* Current output subdev(may be NULL) */

    enum vpif_channel_id channel_id;/* Identifies channel */
    struct vpif_params vpifparams;
    struct common_obj common[VPIF_NUMOBJECTS];
    struct video_obj video;

    unsigned int interrupt_count;
    unsigned int interrupt_dummy_buffer_count;
    unsigned int is_initial_subdev;
};

/* File handle structure */
struct vpif_fh {
    struct channel_obj *channel;	/* pointer to channel object for
					 * opened device */
    u8 io_allowed[VPIF_NUMOBJECTS];	/* Indicates whether this file handle
					 * is doing IO */
    enum v4l2_priority prio;	/* Used to keep track priority of
					 * this instance */
    u8 initialized;			/* Used to keep track of whether this
					 * file handle has initialized
					 * channel or not */
};

struct vpif_config_params {
    u32 min_bufsize[VPIF_DISPLAY_NUM_CHANNELS];
    u32 channel_bufsize[VPIF_DISPLAY_NUM_CHANNELS];
    u8 numbuffers[VPIF_DISPLAY_NUM_CHANNELS];
    u32 video_limit[VPIF_DISPLAY_NUM_CHANNELS];
    u32 channel_bufstride[VPIF_DISPLAY_NUM_CHANNELS];
    u32 pixelformat[VPIF_DISPLAY_NUM_CHANNELS];
    u8 min_numbuffers;
};

#endif				/* DAVINCIHD_DISPLAY_H */
