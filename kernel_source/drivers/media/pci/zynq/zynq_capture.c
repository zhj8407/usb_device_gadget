#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include "zynq_malloc.h"
#include "zynq_capture.h"
#include "zynq_capture_priv.h"
#include "zynq_debug.h"
#include "zynq_control.h"
#include "modules/zynq_scaler.h"
#define USE_DMA_COUNTING 1
//#define USE_ZYNQ_MALLOC 1
#include "zynq_fpga_verify.h"

#if defined(USE_DMA_COUNTING) 
	#include <media/videobuf2-dma-contig.h>
#elif defined(USE_ZYNQ_MALLOC)
	#include "zynq_malloc.h"
#else
	#include <media/videobuf2-vmalloc.h>
#endif

#ifndef FPGA_VERIF	
#define VIRTUAL_CAPTURE_DEVICE 1
#endif


#ifdef VIRTUAL_CAPTURE_DEVICE
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/sched.h>
#include "zynq_draw_buffer.c"
#endif

static u32 ch0_numbuffers = 3;
static u32 ch1_numbuffers = 3;
static u32 ch2_numbuffers = 3;
static u32 ch3_numbuffers = 3;

static u32 ch0_bufsize = 1920 * 1080 * 2;
static u32 ch1_bufsize = 1920 * 1080 * 2;
static u32 ch2_bufsize = 1920 * 1080 * 2;
static u32 ch3_bufsize = 1920 * 1080 * 2;

module_param(ch0_numbuffers, uint, S_IRUGO);
module_param(ch1_numbuffers, uint, S_IRUGO);
module_param(ch2_numbuffers, uint, S_IRUGO);
module_param(ch3_numbuffers, uint, S_IRUGO);
module_param(ch0_bufsize, uint, S_IRUGO);
module_param(ch1_bufsize, uint, S_IRUGO);
module_param(ch2_bufsize, uint, S_IRUGO);
module_param(ch3_bufsize, uint, S_IRUGO);
MODULE_PARM_DESC(ch0_numbuffers, "Channel0 buffer count (default:3)");
MODULE_PARM_DESC(ch1_numbuffers, "Channel1 buffer count (default:3)");
MODULE_PARM_DESC(ch2_numbuffers, "Channel2 buffer count (default:3)");
MODULE_PARM_DESC(ch3_numbuffers, "Channel3 buffer count (default:3)");
MODULE_PARM_DESC(ch0_bufsize, "Channel0 buffer size (default:1920 x 1080 x 2)");
MODULE_PARM_DESC(ch1_bufsize, "Channel1 buffer size (default:1920 x 1080 x 2)");
MODULE_PARM_DESC(ch2_bufsize, "Channel2 buffer size (default:1920 x 1080 x 2)");
MODULE_PARM_DESC(ch3_bufsize, "Channel3 buffer size (default:1920 x 1080 x 2)");

static struct vpif_config_params config_params = {
	.min_numbuffers = 3,
	.numbuffers[VPIF_CHANNEL0_VIDEO] = 3,
	.numbuffers[VPIF_CHANNEL1_VIDEO] = 3,
	.numbuffers[VPIF_CHANNEL2_VIDEO] = 3,
	.numbuffers[VPIF_CHANNEL3_VIDEO] = 3,
	.min_bufsize[VPIF_CHANNEL0_VIDEO] = 1920 * 1080 *  2,
	.min_bufsize[VPIF_CHANNEL1_VIDEO] = 1920 * 1080 *  2,
	.min_bufsize[VPIF_CHANNEL2_VIDEO] = 1920 * 1080 *  2,
	.min_bufsize[VPIF_CHANNEL3_VIDEO] = 1920 * 1080 *  2,
	.channel_bufsize[VPIF_CHANNEL0_VIDEO] = 1920 * 1080 *  2,
	.channel_bufsize[VPIF_CHANNEL1_VIDEO] = 1920 * 1080 *  2,
	.channel_bufsize[VPIF_CHANNEL2_VIDEO] = 1920 * 1080 *  2,
	.channel_bufsize[VPIF_CHANNEL3_VIDEO] = 1920 * 1080 *  2,
	.channel_bufstride[VPIF_CHANNEL0_VIDEO] = 1920,
	.channel_bufstride[VPIF_CHANNEL1_VIDEO] = 1920,
	.channel_bufstride[VPIF_CHANNEL2_VIDEO] = 1920,
	.channel_bufstride[VPIF_CHANNEL3_VIDEO] = 1920,
	.video_limit[VPIF_CHANNEL0_VIDEO] = 1920 * 1920 *2 * 5,
	.video_limit[VPIF_CHANNEL1_VIDEO] = 1920 * 1920 *2 * 5,
	.video_limit[VPIF_CHANNEL2_VIDEO] = 1920 * 1920 *2 * 5,
	.video_limit[VPIF_CHANNEL3_VIDEO] = 1920 * 1920 *2 * 5,
	.pixelformat[VPIF_CHANNEL0_VIDEO] =  V4L2_PIX_FMT_YUV422P,
	.pixelformat[VPIF_CHANNEL1_VIDEO] =  V4L2_PIX_FMT_YUV422P,
	.pixelformat[VPIF_CHANNEL2_VIDEO] =  V4L2_PIX_FMT_YUV422P,
	.pixelformat[VPIF_CHANNEL3_VIDEO] =  V4L2_PIX_FMT_YUV422P
};

static u8 channel_first_int[VPIF_NUMBER_OF_OBJECTS][4] = { {1, 1,1,1} };

/* global variables */
static unsigned int is_initial_vpif_obj = 0;
static struct vpif_cap_device vpif_obj = { 	
	.dev ={NULL},
	.sd = NULL
};
static struct device *vpif_dev = NULL;

static void print_pixel_format( struct v4l2_pix_format *pixfmt)
{
	if (!pixfmt) return;
	
	zynq_printk(0, "[zynq_capture]Video format: %c%c%c%c (%08x) %ux%u\n",
		(pixfmt->pixelformat >> 0) & 0xff,
		(pixfmt->pixelformat >> 8) & 0xff,
		(pixfmt->pixelformat >> 16) & 0xff,
		(pixfmt->pixelformat >> 24) & 0xff,
		pixfmt->pixelformat,
		pixfmt->width, pixfmt->height);
}

/*Initialize the configurations and parameters of one channel*/
static 	int  initial_channel_vpif_parameters(struct channel_obj *ch)
{
	int ret = 0;
	struct vpif_params *params = &ch->vpifparams;
	struct vpif_capture_config *config = NULL;
	struct vpif_capture_chan_config *chan_cfg = NULL;
	
	if (!vpif_dev) return -1;
	
	config = vpif_dev->platform_data;
	
	memset(params, 0x0, sizeof(struct vpif_params));

	chan_cfg = &config->chan_config[ch->channel_id];
	
	params->iface = chan_cfg->vpif_if;
	params->std_info = vpif_ch_params[0];// use the 1080p30 std info
	params->video_params.storage_mode = params->std_info.frm_fmt; //	 0:field mode  1:frame mode
	params->video_params.hpitch = config_params.channel_bufstride[ch->channel_id];
	params->video_params.stdid = V4L2_STD_UNKNOWN;
	//TODO: Should vbi_params and data_sz of params be cleared ?
	
	return ret;
}
		

static int initialize_channel_pxiel_format(struct channel_obj *ch)
{
	 	int ret = 0;
		struct common_obj *common = NULL;
		
		if (!ch) return -1;
		
		common = &(ch->common[VPIF_VIDEO_INDEX]);
		
		common->fmt.fmt.pix.width = config_params.channel_bufstride[ch->channel_id ];
		common->fmt.fmt.pix.pixelformat = config_params.pixelformat[ch->channel_id ];
		
		if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P)
			common->fmt.fmt.pix.height = (config_params.channel_bufsize[ch->channel_id ]/2) /config_params.channel_bufstride[ch->channel_id ];
		else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
			common->fmt.fmt.pix.height = ((config_params.channel_bufsize[ch->channel_id ]*3) /2) /config_params.channel_bufstride[ch->channel_id ];
		else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
			common->fmt.fmt.pix.height = ((config_params.channel_bufsize[ch->channel_id ]*3) /2) /config_params.channel_bufstride[ch->channel_id ];
		
		common->fmt.fmt.pix.field = V4L2_FIELD_NONE;
		common->fmt.fmt.pix.bytesperline = config_params.channel_bufstride[ch->channel_id ];
		common->fmt.fmt.pix.sizeimage = config_params.channel_bufsize[ch->channel_id ];
		common->fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE240M ;
		common->fmt.fmt.pix.priv = 0;
		
		print_pixel_format(&common->fmt.fmt.pix);
		return ret;
}

static int initialize_channel_video_obj(struct channel_obj *ch)
{
	int ret = 0;
	
	//NOTE:Because V4L2_DV_BT_CEA_1920X1080P30 is a macro, we define a struct 'dv_timings_t' for assignment.
	typedef struct dv_timings_type 
	{
		struct v4l2_dv_timings value;	/* HDTV format */
	}  dv_timings_t;
	
	dv_timings_t  dv_times_1080_p30 = {
		.value = V4L2_DV_BT_CEA_1920X1080P30
	}; 
	
	if (!ch) return -1;
	
	ch->video.buf_field =V4L2_FIELD_NONE;
	ch->video.stdid =  V4L2_STD_UNKNOWN ;
	ch->video.dv_timings = dv_times_1080_p30.value;
	
	return ret;
}
///////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_ZYNQ_MALLOC

#define MY_DATA_SIZE (64*1024*1024)
dma_addr_t 	capture_dma_handle;
void *capture_dma_addr = NULL;
size_t capture_dma_size = PAGE_ALIGN(MY_DATA_SIZE + PAGE_SIZE);

#define FRAME_BUFFER_OFFSET 0
dma_addr_t  capture_dam_framebuffer_handle;
void *capture_dam_framebuffer_addr = NULL;
size_t capture_dma_framebuffer_size = PAGE_ALIGN(MY_DATA_SIZE - FRAME_BUFFER_OFFSET + PAGE_SIZE);

#if 0
static void output_test(char *kbuf, dma_addr_t handle, size_t size, char *string)
{
	unsigned long diff;
	diff = (unsigned long)kbuf - handle;
	printk( KERN_INFO"kbuf=%p, handle=%p phys=%p, size=%d\n", kbuf, (void *)handle, (void *)virt_to_phys((void *)kbuf), (int)size);
	printk( KERN_INFO"(kbuf-handle)=%p, %lx, PAGE_OFFSET=%lx, compare=%lx\n",(void *)diff, diff, PAGE_OFFSET, diff - PAGE_OFFSET);
	strcpy(kbuf, string);
	printk( KERN_INFO"string written was, %s\n", kbuf);
}
#endif

static int allocate_reserved_memory(void)
{
	int  status = 0;
	
	capture_dma_addr =  dma_alloc_coherent(NULL, capture_dma_size , &capture_dma_handle, GFP_KERNEL);
	if (!capture_dma_addr)
	{
		zynq_printk(0,"[zynq_capture]failed to allocate  %d memory !!\n", capture_dma_size);
		status = -1;
		goto exit;
	} else {
			zynq_printk(0, "[zynq_capture]successfull to allocate %d memory (virt: %p, phy:%p) !!\n", capture_dma_size, capture_dma_addr, (void *)capture_dma_handle);
		//	output_test(capture_dma_addr ,capture_dma_handle, capture_dma_size, "This is the dma_alloc_coherent() string");			
			capture_dam_framebuffer_handle = (dma_addr_t) ((u8 *)capture_dma_handle + FRAME_BUFFER_OFFSET);
			capture_dam_framebuffer_addr = capture_dma_addr  + FRAME_BUFFER_OFFSET;		
	}
exit:	
	return status;
}

static int release_reserved_memory(void)
{
	if (capture_dma_addr	!=	NULL)
	{	
		dma_free_coherent(NULL, capture_dma_size, capture_dma_addr, capture_dma_handle);
		capture_dma_addr = NULL;
	}
	return  0;
}

#endif
///////////////////////////////////////////////////////////////////////////////////////
#ifdef VIRTUAL_CAPTURE_DEVICE

struct thread_context {
	struct channel_obj *channel;
	unsigned 		   ms;
	unsigned long  jiffies;
	struct v4l2_fract          timeperframe;
	
	int ini_jiffies;
	struct task_struct *kthread;
	wait_queue_head_t wq;
	
	unsigned long  frame_count; 
	
};

#define FPS_MAX 1000

static const struct v4l2_fract
	tpf_min     = {.numerator = 1,		.denominator = FPS_MAX},
	tpf_max     = {.numerator = FPS_MAX,	.denominator = 1},
	tpf_30fps = {.numerator = 1001,	.denominator = 30000},	/* 30 fps*/
	tpf_15fps = {.numerator = 1001,	.denominator = 15000},	/* 15 fps*/
	tpf_60fps = {.numerator = 1001,	.denominator = 60000},	/* 60 fps*/
	tpf_10fps = {.numerator = 1001,	.denominator = 10000};	/* 10 fps*/

static struct thread_context  gThreadContext[ VPIF_CAPTURE_MAX_DEVICES];

static void process_progressive_mode(struct common_obj *common)
{
	unsigned long addr = 0;
	
	spin_lock(&common->irqlock);
	/* Get the next buffer from buffer queue */
	common->next_frm = list_entry(common->dma_queue.next,
				struct vpif_cap_buffer, list);
	/* Remove that buffer from the buffer queue */
	list_del(&common->next_frm->list);
	spin_unlock(&common->irqlock);
	/* Mark status of the buffer as active */
	common->next_frm->vb.state = VB2_BUF_STATE_ACTIVE;

	/* Set top and bottom field addrs in VPIF registers */
#if defined(USE_ZYNQ_MALLOC)
	addr =  zynq_malloc_plane_dma_addr(&common->next_frm->vb, 0);
#elif defined(USE_DMA_COUNTING)	
	addr = vb2_dma_contig_plane_dma_addr(&common->next_frm->vb, 0);
#endif	
	if ((addr != 0) && (common->set_addr != NULL)) common->set_addr(addr+common->y_off,  addr+common->uv_off);
	return;
}

static void thread_tick(struct thread_context  *ctx)
{
	struct channel_obj *ch = ctx->channel;
	struct common_obj *common = NULL;
	unsigned int channel_id = 0;
	if (!ch)
	{
		zynq_printk(0, "[zynq_capture]The channel obj is NULL!!\n");
		goto exit;
	}
	
	common = &ch->common[VPIF_VIDEO_INDEX];
	channel_id = (unsigned int)ch->channel_id;
	
	spin_lock(&common->irqlock);
	if (list_empty(&common->dma_queue)) {
				spin_unlock(&common->irqlock);
				//zynq_printk(0, "[zynq_capture]The buffer queue is emty!!\n");
				goto exit;
	}
	spin_unlock(&common->irqlock);
	
	if (!channel_first_int[VPIF_VIDEO_INDEX][channel_id]) {
				/* Mark status of the cur_frm to
				 * done and unlock semaphore on it */
				v4l2_get_timestamp(&common->cur_frm->vb.
						   v4l2_buf.timestamp);
				{
						void *vbuf = vb2_plane_vaddr(&common->cur_frm->vb, 0);
						struct v4l2_format *fmt_ptr = &common->fmt;
						unsigned int width = fmt_ptr->fmt.pix.width;
						unsigned int height = fmt_ptr->fmt.pix.height;
						unsigned int stride = fmt_ptr->fmt.pix.bytesperline;
						unsigned long count = ctx->frame_count;
						unsigned int fourcc = fmt_ptr->fmt.pix.pixelformat;
						//zynq_printk(0, "[zynq_capture](w, h, s, c, f) ---> (%u, %u, %u, %lu, %d) \n", width, height, stride, count, fourcc );
						//NOTE:For RGB table: http://www.wahart.com.hk/rgb.htm
						if (fourcc == V4L2_PIX_FMT_YUV422P)
							draw_yuv422p(vbuf, 255, 255, 0, 255, 0, 0, width, height, stride,  count);//Yellow1 (255, 255, 0)  , Red (255, 0, 0)
						else if (fourcc == V4L2_PIX_FMT_YUV420)
							draw_yuv420(vbuf, 0, 238, 0, 0, 0, 255, width, height, stride,  count);//Green2 (0, 238, 0), Blue(0, 0, 255) 
						ctx->frame_count++;	
				}
				
				vb2_buffer_done(&common->cur_frm->vb,VB2_BUF_STATE_DONE);
				/* Make cur_frm pointing to next_frm */
				common->cur_frm = common->next_frm;
	}
	channel_first_int[VPIF_VIDEO_INDEX][channel_id] = 0;
	process_progressive_mode(common);
exit:
	{
		unsigned long ms = 0;
		char str[100];
		
		ctx->ms += jiffies_to_msecs(jiffies - ctx->jiffies);
		ctx->jiffies = jiffies;
		ms = ctx->ms;
		memset(str, 0x0, sizeof(str));
		snprintf(str, sizeof(str), "%02lu:%02lu:%02lu:%03lu ",
				(ms / (60 * 60 * 1000)) % 24,
				(ms / (60 * 1000)) % 60,
				(ms / 1000) % 60,
				ms % 1000);
		//zynq_printk(1, "[zynq_capture]timestamp: %s\n", str);
	}
	return;
}

#define frames_to_ms(ctx, frames)				\
	((frames * ctx->timeperframe.numerator * 1000) / ctx->timeperframe.denominator)

static void thread_sleep(struct thread_context  *ctx)
{
	int timeout = 0;
	DECLARE_WAITQUEUE(wait, current);
	add_wait_queue(&ctx->wq, &wait);
	if (kthread_should_stop())
		goto stop_task;
	
	/* Calculate time to wake up */
	timeout = msecs_to_jiffies(frames_to_ms(ctx, 1));

	thread_tick(ctx);

	schedule_timeout_interruptible(timeout);
	
stop_task:

	remove_wait_queue(&ctx->wq, &wait);
	try_to_freeze();
	return;
}


static int thread_handler(void *data)
{
	struct thread_context *ctx = (	struct thread_context *)data;

	 zynq_printk(0, "[zynq_capture]thread started\n");

	set_freezable();

	for (;;) {
		thread_sleep(ctx);

		if (kthread_should_stop())
			break;
	}
	 zynq_printk(0,  "[zynq_capture] thread: exit\n");
	return 0;
}

static int thread_init(struct channel_obj *ch)
{
	struct thread_context *ctx =NULL;
	struct v4l2_fract *fract = NULL;
	
	if (!ch) return  -1;
	
	ctx = &gThreadContext[ch->channel_id];
	fract = &ctx->timeperframe;
	ctx->channel = ch;
	ctx->ms = 0;
	ctx->jiffies = jiffies;
	//NOTE: The default speed of scheduling the next buffer is 60 fps.
	(*fract) = tpf_60fps;// tpf_30fps;
	
	ctx->frame_count = 0;
	ctx->ini_jiffies = ctx->jiffies;
	init_waitqueue_head(&ctx->wq);
	ctx->kthread = kthread_run(thread_handler, ctx, "zynq_capture_thread");

	if (IS_ERR(ctx->kthread)) {
		 zynq_printk(0,  "[zynq_capture]kernel_thread() failed\n");
		return PTR_ERR(ctx->kthread);
	}
	/* Wakes thread */
	wake_up_interruptible(&ctx->wq);
	return  0;
}

static int thread_rls(struct channel_obj *ch)
{
	struct thread_context *ctx =NULL;
	
	if (!ch) return  -1;
	
	ctx = &gThreadContext[ch->channel_id];
	
	/* shutdown control thread */
	if (ctx->kthread) {
		kthread_stop(ctx->kthread);
		ctx->kthread = NULL;
	}
	return 0;
}


#endif
///////////////////////////////////////////////////////////////////////////////////////
/**
 * buffer_prepare :  callback function for buffer prepare
 * @vb: ptr to vb2_buffer
 *
 * This is the callback function for buffer prepare when vb2_qbuf()
 * function is called. The buffer is prepared and user space virtual address
 * or user address is converted into  physical address
 */
static int vpif_buffer_prepare(struct vb2_buffer *vb)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;
	
	common = &ch->common[VPIF_VIDEO_INDEX];
	
	if (vb->state != VB2_BUF_STATE_ACTIVE &&
		vb->state != VB2_BUF_STATE_PREPARED) {
		vb2_set_plane_payload(vb, 0, common->fmt.fmt.pix.sizeimage);
		if (vb2_plane_vaddr(vb, 0) && (vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)))
		{
			zynq_printk(0, "[zynq_capture]payload size:%lu, size:%lu\n", vb2_get_plane_payload(vb, 0), vb2_plane_size(vb, 0));
			//zynq_printk(0, );
			goto exit;
		}
	}
	return 0;
	
exit:
	zynq_printk(0, "[zynq_capture]buffer_prepare failed!!\n");
	return -EINVAL;
}

/**
 * vpif_buffer_queue_setup : Callback function for buffer setup.
 * @vq: vb2_queue ptr
 * @fmt: v4l2 format
 * @nbuffers: ptr to number of buffers requested by application
 * @nplanes:: contains number of distinct video planes needed to hold a frame
 * @sizes[]: contains the size (in bytes) of each plane.
 * @alloc_ctxs: ptr to allocation context
 *
 * This callback function is called when reqbuf() is called to adjust
 * the buffer count and buffer size
 */
static int vpif_buffer_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;
	unsigned long size = 0;
	common = &ch->common[VPIF_VIDEO_INDEX];

	/* Calculate the size of the buffer */
	//size = 1920 * 1920 *2;/*common->fmt.fmt.pix.sizeimage*/
	size = common->fmt.fmt.pix.sizeimage;
	
	if (config_params.video_limit[ch->channel_id])
				while (size * *nbuffers > config_params.video_limit[ch->channel_id])
					(*nbuffers)--;

	if (*nbuffers < config_params.min_numbuffers)
		*nbuffers = config_params.min_numbuffers;
	
	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = common->alloc_ctx;
	
	zynq_printk(0, "[zynq_capture]>>>  nuffers = %d\n", *nbuffers);
	return 0;
}

/**
 * vpif_buffer_queue : Callback function to add buffer to DMA queue
 * @vb: ptr to vb2_buffer
 */
static void vpif_buffer_queue(struct vb2_buffer *vb)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct channel_obj *ch = fh->channel;
	struct vpif_cap_buffer *buf = container_of(vb,
				struct vpif_cap_buffer, vb);
	struct common_obj *common;
	unsigned long flags;

	common = &ch->common[VPIF_VIDEO_INDEX];

	spin_lock_irqsave(&common->irqlock, flags);
	/* add the buffer to the DMA queue */
	list_add_tail(&buf->list, &common->dma_queue);
	spin_unlock_irqrestore(&common->irqlock, flags);
}

/**
 * vpif_buf_cleanup : Callback function to free buffer
 * @vb: ptr to vb2_buffer
 *
 * This function is called from the videobuf2 layer to free memory
 * allocated to  the buffers
 */
static void vpif_buf_cleanup(struct vb2_buffer *vb)
{
	/* Get the file handle object and channel object */
	struct vpif_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct vpif_cap_buffer *buf = container_of(vb,
					struct vpif_cap_buffer, vb);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;
	unsigned long flags;

	common = &ch->common[VPIF_VIDEO_INDEX];

	spin_lock_irqsave(&common->irqlock, flags);
	if (vb->state == VB2_BUF_STATE_ACTIVE)
		list_del_init(&buf->list);
	spin_unlock_irqrestore(&common->irqlock, flags);

}

static void vpif_wait_prepare(struct vb2_queue *vq)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;

	common = &ch->common[VPIF_VIDEO_INDEX];
	mutex_unlock(&common->lock);
}

static void vpif_wait_finish(struct vb2_queue *vq)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;

	common = &ch->common[VPIF_VIDEO_INDEX];
	mutex_lock(&common->lock);
}

static int vpif_buffer_init(struct vb2_buffer *vb)
{
	struct vpif_cap_buffer *buf = container_of(vb,
					struct vpif_cap_buffer, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int vpif_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	unsigned long addr = 0;
	unsigned long flags;
	struct v4l2_format *fmt_ptr = &common->fmt;
	
	/* If buffer queue is empty, return error */
	spin_lock_irqsave(&common->irqlock, flags);
	if (list_empty(&common->dma_queue)) {
		spin_unlock_irqrestore(&common->irqlock, flags);
		zynq_printk(0, "[zynq_capture]DMA buffer queue is empty!!\n");
		return -EIO;
	}
	/* Get the next frame from the buffer queue */
	common->cur_frm = common->next_frm = list_entry(common->dma_queue.next,
				    struct vpif_cap_buffer, list);
	/* Remove buffer from the buffer queue */
	list_del(&common->cur_frm->list);
	spin_unlock_irqrestore(&common->irqlock, flags);
	/* Mark state of the current frame to active */
	common->cur_frm->vb.state = VB2_BUF_STATE_ACTIVE;
	
	/* Initialize field_id and started member */
	ch->field_id = 0;
	common->started = 1;
	
	{
		unsigned int height = fmt_ptr->fmt.pix.height;
		unsigned int stride = fmt_ptr->fmt.pix.bytesperline;
		common->y_off  =0;
		common->uv_off = stride * height;
		switch (ch->channel_id)
		{
			case VPIF_CHANNEL0_VIDEO:
				common->set_addr = ch0_set_videobuf_addr;
				common->set_res = ch0_set_videobuf_res;
				common->enable_channel = enable_channel0;
				common->enable_channel_intr = enable_channel0_intr;
				common->enable_channel_video = enable_channel0_video;
				break;
			case VPIF_CHANNEL1_VIDEO:
				common->set_addr = ch1_set_videobuf_addr;
				common->set_res = ch1_set_videobuf_res;
				common->enable_channel = enable_channel1;
				common->enable_channel_intr = enable_channel1_intr;
				common->enable_channel_video = enable_channel1_video;
				break;
			case VPIF_CHANNEL2_VIDEO:
				common->set_addr = ch2_set_videobuf_addr;
				common->set_res = ch2_set_videobuf_res;
				common->enable_channel = enable_channel2;
				common->enable_channel_intr = enable_channel2_intr;
				common->enable_channel_video = enable_channel2_video;
				break;
			case VPIF_CHANNEL3_VIDEO:	
				common->set_addr = ch3_set_videobuf_addr;
				common->set_res = ch3_set_videobuf_res;
				common->enable_channel = enable_channel3;
				common->enable_channel_intr = enable_channel3_intr;
				common->enable_channel_video = enable_channel3_video;
				break;
			default:
				common->set_addr = NULL;
				common->set_res = NULL;
				common->enable_channel = NULL;
				common->enable_channel_intr = NULL;
				common->enable_channel_video = NULL;
		}
		
	}
	
	channel_first_int[VPIF_VIDEO_INDEX][ch->channel_id] = 1;
	
#if defined(USE_ZYNQ_MALLOC)
	addr =  zynq_malloc_plane_dma_addr(&common->cur_frm->vb, 0);
#elif defined(USE_DMA_COUNTING)	
	addr = vb2_dma_contig_plane_dma_addr(&common->cur_frm->vb, 0);
#endif
	if ((addr != 0) && (common->set_addr != NULL)) {
		common->set_addr(addr+common->y_off,  addr+common->uv_off);
		if (common->set_res != NULL){
			common->set_res(fmt_ptr->fmt.pix.width, fmt_ptr->fmt.pix.height);
		}
		if (common->enable_channel != NULL) common->enable_channel(1);
	}
	
	if (common->enable_channel_video != NULL)
		common->enable_channel_video(1);

#ifdef VIRTUAL_CAPTURE_DEVICE	
		 thread_init(ch);
#endif
		 
	common->is_start_streaming = 1;
	common->is_stop_streaming = 0;
	return 0;
}

/* abort streaming and wait for last buffer */
static int vpif_stop_streaming(struct vb2_queue *vq)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	unsigned long flags = 0;
	struct common_obj *common = NULL;
	
	if (!vb2_is_streaming(vq)) return 0;
	
	common = &ch->common[VPIF_VIDEO_INDEX];
	
	#ifdef VIRTUAL_CAPTURE_DEVICE	
		 thread_rls(ch);
	#endif
		 
	/* release all active buffers */
	spin_lock_irqsave(&common->irqlock, flags);
	while (!list_empty(&common->dma_queue)) {
		common->next_frm = list_entry(common->dma_queue.next,
						struct vpif_cap_buffer, list);
		list_del(&common->next_frm->list);
		vb2_buffer_done(&common->next_frm->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&common->irqlock, flags);
#if 0
	if (common->enable_channel_intr != NULL){
		common->enable_channel_intr(0);
	}
#endif

	if (common->enable_channel_video != NULL)common->enable_channel_video(0);
	
	common->is_start_streaming = 0;
	common->is_stop_streaming = 1;
	
	return 0;
}

static struct vb2_ops video_qops = {
	.queue_setup		= vpif_buffer_queue_setup,
	.wait_prepare		= vpif_wait_prepare,
	.wait_finish		= vpif_wait_finish,
	.buf_init		= vpif_buffer_init,
	.buf_prepare		= vpif_buffer_prepare,
	.start_streaming	= vpif_start_streaming,
	.stop_streaming		= vpif_stop_streaming,
	.buf_cleanup		= vpif_buf_cleanup,
	.buf_queue		= vpif_buffer_queue,
};

/**
 * vpif_update_std_info() - update standard related info
 * @ch: ptr to channel object
 *
 * For a given standard selected by application, update values
 * in the device data structures
 */
static int vpif_update_std_info(struct channel_obj *ch)
{
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct vpif_params *vpifparams = &ch->vpifparams;
	const struct vpif_channel_config_params *config;
	struct vpif_channel_config_params *std_info = &vpifparams->std_info;
	struct video_obj *vid_ch = &ch->video;
	int index;

	//vpif_dbg(2, debug_capture, "vpif_update_std_info\n");

	for (index = 0; index < vpif_ch_params_count; index++) {
		config = &vpif_ch_params[index];
		if (config->hd_sd == 0) {
			zynq_printk(0, "[zynq_capture]SD format\n");
			if (config->stdid & vid_ch->stdid) {
				memcpy(std_info, config, sizeof(*config));
				break;
			}
		} else {
			zynq_printk(0, "[zynq_capture]HD format\n");
			if (!memcmp(&config->dv_timings, &vid_ch->dv_timings,
				sizeof(vid_ch->dv_timings))) {
				memcpy(std_info, config, sizeof(*config));
				break;
			}
		}
	}
	
		/* standard not found */
	if (index == vpif_ch_params_count)
		return -EINVAL;
	
	if ((common->fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) && (common->fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV422P) &&  (common->fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_NV12) )
		return -EINVAL;

	common->fmt.fmt.pix.width = std_info->width;
	common->width = std_info->width;
	common->fmt.fmt.pix.height = std_info->height;
	common->height = std_info->height;
	common->fmt.fmt.pix.pixelformat = std_info->pixelformat;
	
	common->fmt.fmt.pix.bytesperline = std_info->width; 
	
	if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P)
		common->fmt.fmt.pix.sizeimage = common->fmt.fmt.pix.bytesperline * common->fmt.fmt.pix.height  * 2;
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
		common->fmt.fmt.pix.sizeimage = (common->fmt.fmt.pix.bytesperline * common->fmt.fmt.pix.height  * 3 )/2;
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
		common->fmt.fmt.pix.sizeimage = (common->fmt.fmt.pix.bytesperline * common->fmt.fmt.pix.height  * 3 )/2;
	
	vpifparams->video_params.hpitch = std_info->width;
	vpifparams->video_params.storage_mode = std_info->frm_fmt;
	
	return 0;
}

/**
 * vpif_config_format: configure default frame format in the device
 * ch : ptr to channel object
 */
static void vpif_config_format(struct channel_obj *ch)
{
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	common->fmt.fmt.pix.field = V4L2_FIELD_NONE;
	if (config_params.numbuffers[ch->channel_id] == 0)
		common->memory = V4L2_MEMORY_USERPTR;
	else
		common->memory = V4L2_MEMORY_MMAP;

	//common->fmt.fmt.pix.sizeimage = config_params.channel_bufsize[ch->channel_id];
	//common->fmt.fmt.pix.bytesperline = config_params.channel_bufstride[ch->channel_id];
	//common->fmt.fmt.pix.pixelformat = config_params.pixelformat[ch->channel_id];
	common->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

/**
 * vpif_get_default_field() - Get default field type based on interface
 * @vpif_params - ptr to vpif params
 */
static inline enum v4l2_field vpif_get_default_field(
				struct vpif_interface *iface)
{
	return V4L2_FIELD_NONE ;
}

/**
 * vpif_check_format()  - check given pixel format for compatibility
 * @ch - channel  ptr
 * @pixfmt - Given pixel format
 * @update - update the values as per hardware requirement
 *
 * Check the application pixel format for S_FMT and update the input
 * values as per hardware limits for TRY_FMT. The default pixel and
 * field format is selected based on interface type.
 */
static int vpif_check_format(struct channel_obj *ch,
			     struct v4l2_pix_format *pixfmt,
			     int update)
{

	struct vpif_params *vpif_params = &ch->vpifparams;
	enum v4l2_field field = pixfmt->field;
	int ret = -EINVAL;
	
	if ((pixfmt->pixelformat != V4L2_PIX_FMT_YUV422P) && (pixfmt->pixelformat != V4L2_PIX_FMT_YUV420)  && (pixfmt->pixelformat != V4L2_PIX_FMT_NV12)) {
			if (!update) {
				print_pixel_format(pixfmt);
				zynq_printk(0, "[zynq_capture]invalid pixel format\n");
				goto exit;
			}
			//defaul is YUV 422P
			pixfmt->pixelformat = V4L2_PIX_FMT_YUV422P;
	}
	
	if (!(VPIF_VALID_FIELD(field))  ||  (field != V4L2_FIELD_NONE)) {
		if (!update) {
			zynq_printk(0, "[zynq_capture]invalid field format\n");
			goto exit;
		}
		/**
		 * By default use FIELD_NONE for RAW Bayer capture
		 * and FIELD_INTERLACED for other interfaces
		 */
		field = vpif_get_default_field(&vpif_params->iface);
	} 
	return 0;
exit:
	return ret;
}

/**
 * vpif_mmap : It is used to map kernel space buffers into user spaces
 * @filep: file pointer
 * @vma: ptr to vm_area_struct
 */
static int vpif_mmap(struct file *filep, struct vm_area_struct *vma)
{
	/* Get the channel object and file handle object */
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &(ch->common[VPIF_VIDEO_INDEX]);
	int ret = 0;

	if (mutex_lock_interruptible(&common->lock))
		return -ERESTARTSYS;
	ret = vb2_mmap(&common->buffer_queue, vma);
	mutex_unlock(&common->lock);
	return ret;
}

/**
 * vpif_poll: It is used for select/poll system call
 * @filep: file pointer
 * @wait: poll table to wait
 */
static unsigned int vpif_poll(struct file *filep, poll_table * wait)
{
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *channel = fh->channel;
	struct common_obj *common = &(channel->common[VPIF_VIDEO_INDEX]);
	unsigned int res = 0;

	if (common->started) {
		mutex_lock(&common->lock);
		res = vb2_poll(&common->buffer_queue, filep, wait);
		mutex_unlock(&common->lock);
	}
	return res;
}

/**
 * vpif_open : vpif open handler
 * @filep: file ptr
 *
 * It creates object of file handle structure and stores it in private_data
 * member of filepointer
 */
static int vpif_open(struct file *filep)
{
	struct video_device *vdev = video_devdata(filep);
	struct common_obj *common;
	struct video_obj *vid_ch;
	struct channel_obj *ch;
	struct vpif_fh *fh;
	
	ch = video_get_drvdata(vdev);

	vid_ch = &ch->video;
	common = &ch->common[VPIF_VIDEO_INDEX];

	/* Allocate memory for the file handle object */
	fh = kzalloc(sizeof(struct vpif_fh), GFP_KERNEL);
	if (NULL == fh) {
		zynq_printk(0, "[zynq_capture]unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}

	if (mutex_lock_interruptible(&common->lock)) {
		kfree(fh);
		return -ERESTARTSYS;
	}
	/* store pointer to fh in private_data member of filep */
	filep->private_data = fh;
	fh->channel = ch;
	v4l2_fh_init(&fh->fh, vdev);
	v4l2_fh_add(&fh->fh);
	fh->initialized = 0;
	/* If decoder is not initialized. initialize it */
	if (!ch->initialized) {
		fh->initialized = 1;
		ch->initialized = 1;
	}
	/* Increment channel usrs counter */
	ch->usrs++;
	/* Set io_allowed member to false */
	fh->io_allowed[VPIF_VIDEO_INDEX] = 0;
	/* Initialize priority of this instance to default priority */
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&ch->prio, &fh->prio);	
	mutex_unlock(&common->lock);
	return 0;
}

/**
 * vpif_release : function to clean up file close
 * @filep: file pointer
 *
 * This function deletes buffer queue, frees the buffers and the vpif file
 * handle
 */
static int vpif_release(struct file *filep)
{
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = NULL;

	common = &ch->common[VPIF_VIDEO_INDEX];

	mutex_lock(&common->lock);
	/* if this instance is doing IO */
	if (fh->io_allowed[VPIF_VIDEO_INDEX]) {
		/* Reset io_usrs member of channel object */
		common->io_usrs = 0;
		/* Disable channel as per its device type and channel id */
		//if (common->enable_channel != NULL)common->enable_channel(0);	
		common->started = 0;
		/* Free buffers allocated */
		vb2_queue_release(&common->buffer_queue);
#if  defined(USE_DMA_COUNTING	)	
		vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
#elif defined(USE_ZYNQ_MALLOC)
		 zynq_malloc_cleanup_ctx(common->alloc_ctx);
#endif		
	}
	/* Decrement channel usrs counter */
	ch->usrs--;

	v4l2_fh_del(&fh->fh);
	v4l2_fh_exit(&fh->fh);
	
	/* Close the priority */
	v4l2_prio_close(&ch->prio, fh->prio);

	if (fh->initialized)
		ch->initialized = 0;
	
	mutex_unlock(&common->lock);
	filep->private_data = NULL;
	kfree(fh);
	return 0;
}

/**
 * vpif_reqbufs() - request buffer handler
 * @file: file ptr
 * @priv: file handle
 * @reqbuf: request buffer structure ptr
 */
static int vpif_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbuf)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;
	u8 index = 0;
	struct vb2_queue *q;
	int ret;

	//vpif_dbg(2, debug_capture, "vpif_reqbufs\n");

	/**
	 * This file handle has not initialized the channel,
	 * It is not allowed to do settings
	 */
	if ((VPIF_CHANNEL0_VIDEO == ch->channel_id)
	    || (VPIF_CHANNEL1_VIDEO == ch->channel_id) 
		|| (VPIF_CHANNEL2_VIDEO == ch->channel_id)
		|| (VPIF_CHANNEL3_VIDEO == ch->channel_id)) {
		if (!fh->initialized) {
			zynq_printk(0, "[zynq_capture]Channel Busy\n");
			return -EBUSY;
		}
	}

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != reqbuf->type || !vpif_dev)
		return -EINVAL;

	index = VPIF_VIDEO_INDEX;

	common = &ch->common[index];

	if (0 != common->io_usrs)
		return -EBUSY;
#if  defined(USE_DMA_COUNTING)
	/* Initialize videobuf2 queue as per the buffer type */
	common->alloc_ctx = vb2_dma_contig_init_ctx(vpif_dev);

	if (IS_ERR(common->alloc_ctx)) {
		zynq_printk(0, "[zynq_capture]Failed to get the context\n");
		return PTR_ERR(common->alloc_ctx);
	}
#elif defined(USE_ZYNQ_MALLOC)
		{
			zynq_malloc_conf_t  ctx;
			
			ctx.is_always_get_first_memory = 0;
			ctx.pool_start_address = capture_dam_framebuffer_addr;
			ctx.pool_size = capture_dma_framebuffer_size;
			
			common->alloc_ctx = zynq_malloc_init_ctx(&ctx);
			if (IS_ERR(common->alloc_ctx)) {
				zynq_printk(0, "[zynq_capture]Failed to get the context\n");
				return PTR_ERR(common->alloc_ctx);
			}
		}
#else
		common->alloc_ctx = NULL;
#endif
	q = &common->buffer_queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = fh;
	q->ops = &video_qops;
#if  defined(USE_DMA_COUNTING)	
	q->mem_ops = &vb2_dma_contig_memops;
#elif  defined(USE_ZYNQ_MALLOC)
	q->mem_ops = &zynq_malloc_memops;
#else	
	q->mem_ops = &vb2_vmalloc_memops;
#endif
	q->buf_struct_size = sizeof(struct vpif_cap_buffer);
#if (LINUX_VERSION_CODE <=  KERNEL_VERSION(3,14,0))	
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#else
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#endif	
	ret = vb2_queue_init(q);
	if (ret) {
		zynq_printk(0, "[zynq_capture]vpif_capture: vb2_queue_init() failed\n");
#if  defined(USE_DMA_COUNTING)			
		vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
#elif defined(USE_ZYNQ_MALLOC)
		zynq_malloc_cleanup_ctx(common->alloc_ctx);
#endif		
		return ret;
	}
	/* Set io allowed member of file handle to TRUE */
	fh->io_allowed[index] = 1;
	/* Increment io usrs member of channel object to 1 */
	common->io_usrs = 1;
	/* Store type of memory requested in channel object */
	common->memory = reqbuf->memory;
	INIT_LIST_HEAD(&common->dma_queue);

	/* Allocate buffers */
	return vb2_reqbufs(&common->buffer_queue, reqbuf);
}

/**
 * vpif_querybuf() - query buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vpif_querybuf(struct file *file, void *priv,
				struct v4l2_buffer *buf)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	//vpif_dbg(2, debug_capture, "vpif_querybuf\n");

	if (common->fmt.type != buf->type)
		return -EINVAL;

	if (common->memory != V4L2_MEMORY_MMAP) {
		zynq_printk(0, "[zynq_capture]Invalid memory\n");
		return -EINVAL;
	}

	return vb2_querybuf(&common->buffer_queue, buf);
}

/**
 * vpif_qbuf() - query buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vpif_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{

	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct v4l2_buffer tbuf = *buf;

//	vpif_dbg(2, debug_capture, "vpif_qbuf\n");

	if (common->fmt.type != tbuf.type) {
		zynq_printk(0, "[zynq_capture]invalid buffer type\n");
		return -EINVAL;
	}

	if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
		zynq_printk(0, "[zynq_capture]fh io not allowed\n");
		return -EACCES;
	}

	return vb2_qbuf(&common->buffer_queue, buf);
}

/**
 * vpif_dqbuf() - query buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vpif_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	//vpif_dbg(2, debug_capture, "vpif_dqbuf\n");

	return vb2_dqbuf(&common->buffer_queue, buf,
			 (file->f_flags & O_NONBLOCK));
}

/**
 * vpif_streamon() - streamon handler
 * @file: file ptr
 * @priv: file handle
 * @buftype: v4l2 buffer type
 */
static int vpif_streamon(struct file *file, void *priv,
				enum v4l2_buf_type buftype)
{

	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct vpif_params *vpif;
	int ret = 0;

	//vpif_dbg(2, debug_capture, "vpif_streamon\n");

	vpif = &ch->vpifparams;

	if (buftype != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		zynq_printk(0, "[zynq_capture]buffer type not supported\n");
		return -EINVAL;
	}

	/* If file handle is not allowed IO, return error */
	if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
		zynq_printk(0, "[zynq_capture]fh->io_allowed == 0\n");
		return -EACCES;
	}

	/* If Streaming is already started, return error */
	if (common->started) {
		zynq_printk(0, "[zynq_capture]channel->started == 1\n");
		return -EBUSY;
	}

	ret = vpif_check_format(ch, &common->fmt.fmt.pix, 0);
	if (ret)
		return ret;

	if (ch->sd != NULL)
	{
		/* Enable streamon on the sub device */
		ret = v4l2_subdev_call(ch->sd, video, s_stream, 1);
	}
	
	if (ret && ret != -ENOIOCTLCMD && ret != -ENODEV) {
		zynq_printk(0, "[zynq_capture]stream on failed in subdev\n");
		return ret;
	}

	/* Call vb2_streamon to start streaming in videobuf2 */
	ret = vb2_streamon(&common->buffer_queue, buftype);
	if (ret) {
		zynq_printk(0, "[zynq_capture]call vb2_streamon() failed!!\n");
		return ret;
	}

	return ret;
}

/**
 * vpif_streamoff() - streamoff handler
 * @file: file ptr
 * @priv: file handle
 * @buftype: v4l2 buffer type
 */
static int vpif_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type buftype)
{

	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	int ret = 0;

	//vpif_dbg(2, debug_capture, "vpif_streamoff\n");

	if (buftype != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		zynq_printk(0, "[zynq_capture]buffer type not supported\n");
		return -EINVAL;
	}

	/* If io is allowed for this file handle, return error */
	if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
		zynq_printk(0, "[zynq_capture] fh->io_allowed == 0\n");
		return -EACCES;
	}

	/* If streaming is not started, return error */
	if (!common->started) {
		zynq_printk(0, "[zynq_capture] channel->started == 0\n");
		return -EINVAL;
	}

	/* disable channel */
	//if (common->enable_channel != NULL) common->enable_channel(0);

	common->started = 0;
	
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, video, s_stream, 0);

	if (ret && ret != -ENOIOCTLCMD && ret != -ENODEV)
		zynq_printk(0, "[zynq_capture]stream off failed in subdev\n");

	return vb2_streamoff(&common->buffer_queue, buftype);
}

/**
 * vpif_input_to_subdev() - Maps input to sub device
 * @vpif_cfg - global config ptr
 * @chan_cfg - channel config ptr
 * @input_index - Given input index from application
 *
 * lookup the sub device information for a given input index.
 * we report all the inputs to application. inputs table also
 * has sub device name for the each input
 */
static int vpif_input_to_subdev(
		struct vpif_capture_config *vpif_cfg,
		struct vpif_capture_chan_config *chan_cfg,
		int input_index)
{
	struct vpif_subdev_info *subdev_info;
	const char *subdev_name;
	int i;

	//vpif_dbg(2, debug_capture, "vpif_input_to_subdev\n");

	subdev_name = chan_cfg->inputs[input_index].subdev_name;
	if (subdev_name == NULL)
		return -1;

	/* loop through the sub device list to get the sub device info */
	for (i = 0; i < vpif_cfg->subdev_count; i++) {
		subdev_info = &vpif_cfg->subdev_info[i];
		if (!strcmp(subdev_info->name, subdev_name))
			return i;
	}
	return -1;
}

/**
 * vpif_set_input() - Select an input
 * @vpif_cfg - global config ptr
 * @ch - channel
 * @_index - Given input index from application
 *
 * Select the given input.
 */
static int vpif_set_input(
		struct vpif_capture_config *vpif_cfg,
		struct channel_obj *ch,
		int index)
{
	struct vpif_capture_chan_config *chan_cfg =
			&vpif_cfg->chan_config[ch->channel_id];
	struct vpif_subdev_info *subdev_info = NULL;
	struct v4l2_subdev *sd = NULL;
	u32 input = 0, output = 0;
	int sd_index;
	int ret;

	sd_index = vpif_input_to_subdev(vpif_cfg, chan_cfg, index);
	if (sd_index >= 0) {
		if (vpif_obj.sd != NULL){
			sd = vpif_obj.sd[sd_index];
			subdev_info = &vpif_cfg->subdev_info[sd_index];
		}
	}

	/* first setup input path from sub device to vpif */
	if (sd && vpif_cfg->setup_input_path) {
		ret = vpif_cfg->setup_input_path(ch->channel_id,
				       subdev_info->name);
		if (ret < 0) {
			zynq_printk(0, "[zynq_capture]couldn't setup input path for the" \
			" sub device %s, for input index %d\n",
			subdev_info->name, index);
			return ret;
		}
	}

	if (sd) {
		input = chan_cfg->inputs[index].input_route;
		output = chan_cfg->inputs[index].output_route;
		ret = v4l2_subdev_call(sd, video, s_routing,
				input, output, 0);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			zynq_printk(0, "[zynq_capture]Failed to set input\n");
			return ret;
		}
	}
	
	ch->input_idx = index;
	ch->sd = sd;
	/* copy interface parameters to vpif */
	ch->vpifparams.iface = chan_cfg->vpif_if;

	/* update tvnorms from the sub device input info */
	ch->video_dev->tvnorms = chan_cfg->inputs[index].input.std;
	
	return 0;
}

/**
 * vpif_querystd() - querystd handler
 * @file: file ptr
 * @priv: file handle
 * @std_id: ptr to std id
 *
 * This function is called to detect standard at the selected input
 */
static int vpif_querystd(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	int ret = 0;
	
	if (ch->sd != NULL)
	{
		/* Call querystd function of decoder device */
		ret = v4l2_subdev_call(ch->sd, video, querystd, std_id);
	}
	
	if (ret == -ENOIOCTLCMD || ret == -ENODEV)
		return -ENODATA;
	if (ret) {
		zynq_printk(0, "[zynq_capture]Failed to query standard for sub devices\n");
		return ret;
	}

	return 0;
}

/**
 * vpif_g_std() - get STD handler
 * @file: file ptr
 * @priv: file handle
 * @std_id: ptr to std id
 */
static int vpif_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;

	*std = ch->video.stdid;
	return 0;
}

/**
 * vpif_s_std() - set STD handler
 * @file: file ptr
 * @priv: file handle
 * @std_id: ptr to std id
 */
static int vpif_s_std(struct file *file, void *priv, v4l2_std_id std_id)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	int ret = 0;

	//vpif_dbg(2, debug_capture, "vpif_s_std\n");

	if (common->started) {
		zynq_printk(0, "[zynq_capture]streaming in progress\n");
		return -EBUSY;
	}

	if ((VPIF_CHANNEL0_VIDEO == ch->channel_id) ||
	    (VPIF_CHANNEL1_VIDEO == ch->channel_id) ||  
	    (VPIF_CHANNEL2_VIDEO == ch->channel_id) ||
		(VPIF_CHANNEL3_VIDEO == ch->channel_id)) {
		if (!fh->initialized) {
			zynq_printk(0, "[zynq_capture]Channel Busy\n");
			return -EBUSY;
		}
	}

	ret = v4l2_prio_check(&ch->prio, fh->prio);
	if (0 != ret)
		return ret;

	fh->initialized = 1;

	/* Call encoder subdevice function to set the standard */
	ch->video.stdid = std_id;
	memset(&ch->video.dv_timings, 0, sizeof(ch->video.dv_timings));

	/* Get the information about the standard */
	if (vpif_update_std_info(ch)) {
		zynq_printk(0, "[zynq_capture]Error getting the standard info\n");
		return -EINVAL;
	}

	/* Configure the default format information */
	vpif_config_format(ch);

	if (ch->sd != NULL)
	{
		/* set standard in the sub device */
		ret = v4l2_subdev_call(ch->sd, core, s_std, std_id);
	}
	if (ret && ret != -ENOIOCTLCMD && ret != -ENODEV) {
		zynq_printk(0, "[zynq_capture]Failed to set standard for sub devices\n");
		return ret;
	}
	return 0;
}

/**
 * vpif_enum_input() - ENUMINPUT handler
 * @file: file ptr
 * @priv: file handle
 * @input: ptr to input structure
 */
static int vpif_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{

	struct vpif_capture_config *config = vpif_dev->platform_data;
	struct vpif_capture_chan_config *chan_cfg;
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;

	chan_cfg = &config->chan_config[ch->channel_id];

	if (input->index >= chan_cfg->input_count) {
		zynq_printk(0, "[zynq_capture]Invalid input index\n");
		return -EINVAL;
	}

	memcpy(input, &chan_cfg->inputs[input->index].input,
		sizeof(*input));
	return 0;
}

/**
 * vpif_g_input() - Get INPUT handler
 * @file: file ptr
 * @priv: file handle
 * @index: ptr to input index
 */
static int vpif_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;

	*index = ch->input_idx;
	return 0;
}

/**
 * vpif_s_input() - Set INPUT handler
 * @file: file ptr
 * @priv: file handle
 * @index: input index
 */
static int vpif_s_input(struct file *file, void *priv, unsigned int index)
{
	struct vpif_capture_config *config = vpif_dev->platform_data;
	struct vpif_capture_chan_config *chan_cfg;
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	int ret;

	chan_cfg = &config->chan_config[ch->channel_id];

	if (index >= chan_cfg->input_count)
		return -EINVAL;

	if (common->started) {
		zynq_printk(0, "[zynq_capture]Streaming in progress\n");
		return -EBUSY;
	}

	if ((VPIF_CHANNEL0_VIDEO == ch->channel_id) ||
	    (VPIF_CHANNEL1_VIDEO == ch->channel_id) ||  
	    (VPIF_CHANNEL2_VIDEO == ch->channel_id) ||
	    (VPIF_CHANNEL3_VIDEO == ch->channel_id)) {
		if (!fh->initialized) {
			zynq_printk(0, "[zynq_capture]Channel Busy\n");
			return -EBUSY;
		}
	}

	ret = v4l2_prio_check(&ch->prio, fh->prio);
	if (0 != ret)
		return ret;

	fh->initialized = 1;
	return vpif_set_input(config, ch, index);
}

/**
 * vpif_enum_fmt_vid_cap() - ENUM_FMT handler
 * @file: file ptr
 * @priv: file handle
 * @index: input index
 */
static int vpif_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *fmt)
{
//	struct vpif_fh *fh = priv;
//	struct channel_obj *ch = fh->channel;
	if (fmt->index != 0) {
		zynq_printk(0, "[zynq_capture](%d)Invalid format (index = %d)\n", __LINE__,fmt->index);
		return -EINVAL;
	}
#if 0
	/* Fill in the information about format */
	if (ch->vpifparams.iface.if_type == VPIF_IF_RAW_BAYER) {
		fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		zynq_printk(0, "[zynq_capture]Raw Mode -Bayer Pattern GrRBGb");
		fmt->pixelformat = V4L2_PIX_FMT_SBGGR8;
	} else {
		fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		zynq_printk(0, "[zynq_capture]YCbCr4:2:2 YC Planar");
		fmt->pixelformat = V4L2_PIX_FMT_YUV422P;
	}
#endif
	
	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt->pixelformat = V4L2_PIX_FMT_YUV420;
	zynq_printk(0, "[zynq_capture](%d) V4L2_PIX_FMT_YUV420 (index = %d)\n", __LINE__, fmt->index);
	
	return 0;
}

/**
 * vpif_try_fmt_vid_cap() - TRY_FMT handler
 * @file: file ptr
 * @priv: file handle
 * @fmt: ptr to v4l2 format structure
 */
static int vpif_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

	return vpif_check_format(ch, pixfmt, 1);
}


/**
 * vpif_g_fmt_vid_cap() - Set INPUT handler
 * @file: file ptr
 * @priv: file handle
 * @fmt: ptr to v4l2 format structure
 */
static int vpif_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
//	vpif_dbg(1, debug_capture, " [vpif_capture](%d) fmt->type = %d common->fmt.type=%d \n", __LINE__,  fmt->type, common->fmt.type);
	/* Check the validity of the buffer type */
	if (common->fmt.type != fmt->type) {
		zynq_printk(0, "[zynq_capture](%d)\n", __LINE__);
		return -EINVAL;
	}

	/* Fill in the information about format */
	*fmt = common->fmt;
	return 0;
}

/**
 * vpif_s_fmt_vid_cap() - Set FMT handler
 * @file: file ptr
 * @priv: file handle
 * @fmt: ptr to v4l2 format structure
 */
static int vpif_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct v4l2_pix_format *pixfmt;
	int ret = 0;

	//vpif_dbg(2, debug_capture, "%s\n", __func__);

	/* If streaming is started, return error */
	if (common->started) {
		zynq_printk(0, "[zynq_capture]Streaming is started\n");
		return -EBUSY;
	}

	if ((VPIF_CHANNEL0_VIDEO == ch->channel_id) ||
	    (VPIF_CHANNEL1_VIDEO == ch->channel_id) ||
	    (VPIF_CHANNEL2_VIDEO == ch->channel_id) ||
	    (VPIF_CHANNEL3_VIDEO == ch->channel_id)) {
		if (!fh->initialized) {
			zynq_printk(0, "[zynq_capture]Channel Busy\n");
			return -EBUSY;
		}
	}

	ret = v4l2_prio_check(&ch->prio, fh->prio);
	if (0 != ret)
		return ret;

	fh->initialized = 1;

	pixfmt = &fmt->fmt.pix;
	/* Check for valid field format */
	ret = vpif_check_format(ch, pixfmt, 0);

	if (ret)
		return ret;
	/* store the format in the channel object */
	//common->fmt = *fmt;
	common->fmt.fmt.pix.width = pixfmt->width;
	common->fmt.fmt.pix.height = 	pixfmt->height;
	common->fmt.fmt.pix.pixelformat = 	pixfmt->pixelformat;
	if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P) {
		common->fmt.fmt.pix.sizeimage = common->fmt.fmt.pix.width * common->fmt.fmt.pix.height *2;
		zynq_printk(0, "[zynq_capture](%d)V4L2_PIX_FMT_YUV422P\n", __LINE__);
	}else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {
		common->fmt.fmt.pix.sizeimage = (common->fmt.fmt.pix.width * common->fmt.fmt.pix.height * 3 )/2;
		zynq_printk(0, "[zynq_capture](%d)V4L2_PIX_FMT_YUV420\n", __LINE__);
	}else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12) {
		common->fmt.fmt.pix.sizeimage = (common->fmt.fmt.pix.width * common->fmt.fmt.pix.height * 3 )/2;
		zynq_printk(0, "[zynq_capture](%d)V4L2_PIX_FMT_NV12\n", __LINE__);
	}
	common->fmt.fmt.pix.bytesperline = common->fmt.fmt.pix.width;
	
	
	
	
	return 0;
}

/**
 * vpif_querycap() - QUERYCAP handler
 * @file: file ptr
 * @priv: file handle
 * @cap: ptr to v4l2_capability structure
 */
static int vpif_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	struct vpif_capture_config *config = vpif_dev->platform_data;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	snprintf(cap->driver, sizeof(cap->driver), "%s", dev_name(vpif_dev));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(vpif_dev));
	strlcpy(cap->card, config->card_name, sizeof(cap->card));

	return 0;
}

/**
 * vpif_g_priority() - get priority handler
 * @file: file ptr
 * @priv: file handle
 * @prio: ptr to v4l2_priority structure
 */
static int vpif_g_priority(struct file *file, void *priv,
			   enum v4l2_priority *prio)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;

	*prio = v4l2_prio_max(&ch->prio);

	return 0;
}

/**
 * vpif_s_priority() - set priority handler
 * @file: file ptr
 * @priv: file handle
 * @prio: ptr to v4l2_priority structure
 */
static int vpif_s_priority(struct file *file, void *priv, enum v4l2_priority p)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;

	return v4l2_prio_change(&ch->prio, &fh->prio, p);
}

/**
 * vpif_cropcap() - cropcap handler
 * @file: file ptr
 * @priv: file handle
 * @crop: ptr to v4l2_cropcap structure
 */
static int vpif_cropcap(struct file *file, void *priv,
			struct v4l2_cropcap *crop)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != crop->type)
		return -EINVAL;

	crop->bounds.left = 0;
	crop->bounds.top = 0;
	crop->bounds.height = common->height;
	crop->bounds.width = common->width;
	crop->defrect = crop->bounds;
	
	return 0;
}

/**
 * vpif_s_crop() - vidioc_s_crop handler
 * @file: file ptr
 * @priv: file handle
 * @crop: ptr to v4l2_crop structure
 */
static int vpif_s_crop(struct file *file, void *priv, const struct v4l2_crop *crop) {

#if 0	
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	//struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct v4l2_rect rect = crop->c;
	
	unsigned int crop_start_x = rect.left;
	unsigned int crop_start_y = rect.top;
	unsigned int crop_width = rect.width;
	unsigned int crop_height = rect.height;
	
	//NOTE: Use the '' IOCTL to configure the SCALER2 cropping function.
	config_pipeline_crop(ch, crop_start_x, crop_start_y, crop_width, crop_height);
#endif	
	return  0;
}

/**
 * vpif_enum_dv_timings() - ENUM_DV_TIMINGS handler
 * @file: file ptr
 * @priv: file handle
 * @timings: input timings
 */
static int
vpif_enum_dv_timings(struct file *file, void *priv,
		     struct v4l2_enum_dv_timings *timings)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	int ret = 0;
	
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, video, enum_dv_timings, timings);
	
	if (ret == -ENOIOCTLCMD || ret == -ENODEV)
		return -EINVAL;
	return ret;
}

/**
 * vpif_query_dv_timings() - QUERY_DV_TIMINGS handler
 * @file: file ptr
 * @priv: file handle
 * @timings: input timings
 */
static int
vpif_query_dv_timings(struct file *file, void *priv,
		      struct v4l2_dv_timings *timings)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	int ret = 0;
	
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, video, query_dv_timings, timings);
	
	if (ret == -ENOIOCTLCMD || ret == -ENODEV)
		return -ENODATA;
	return ret;
}

/**
 * vpif_s_dv_timings() - S_DV_TIMINGS handler
 * @file: file ptr
 * @priv: file handle
 * @timings: digital video timings
 */
static int vpif_s_dv_timings(struct file *file, void *priv,
		struct v4l2_dv_timings *timings)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct vpif_params *vpifparams = &ch->vpifparams;
	struct vpif_channel_config_params *std_info = &vpifparams->std_info;
	struct video_obj *vid_ch = &ch->video;
	struct v4l2_bt_timings *bt = &vid_ch->dv_timings.bt;
	int ret = 0;

	if (timings->type != V4L2_DV_BT_656_1120) {
		zynq_printk(0, "[zynq_capture]Timing type not defined\n");
		return -EINVAL;
	}

	/* Configure subdevice timings, if any */
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, video, s_dv_timings, timings);
	
	if (ret == -ENOIOCTLCMD || ret == -ENODEV)
		ret = 0;
	if (ret < 0) {
		zynq_printk(0, "[zynq_capture]Error setting custom DV timings\n");
		return ret;
	}

	if (!(timings->bt.width && timings->bt.height &&
				(timings->bt.hbackporch ||
				 timings->bt.hfrontporch ||
				 timings->bt.hsync) &&
				timings->bt.vfrontporch &&
				(timings->bt.vbackporch ||
				 timings->bt.vsync))) {
		zynq_printk(0, "[zynq_capture]Timings for width, height, "
				"horizontal back porch, horizontal sync, "
				"horizontal front porch, vertical back porch, "
				"vertical sync and vertical back porch "
				"must be defined\n");
		return -EINVAL;
	}

	vid_ch->dv_timings = *timings;

	/* Configure video port timings */

	std_info->eav2sav = bt->hbackporch + bt->hfrontporch +
		bt->hsync - 8;
	std_info->sav2eav = bt->width;

	std_info->l1 = 1;
	std_info->l3 = bt->vsync + bt->vbackporch + 1;

	if (bt->interlaced) {
		if (bt->il_vbackporch || bt->il_vfrontporch || bt->il_vsync) {
			std_info->vsize = bt->height * 2 +
				bt->vfrontporch + bt->vsync + bt->vbackporch +
				bt->il_vfrontporch + bt->il_vsync +
				bt->il_vbackporch;
			std_info->l5 = std_info->vsize/2 -
				(bt->vfrontporch - 1);
			std_info->l7 = std_info->vsize/2 + 1;
			std_info->l9 = std_info->l7 + bt->il_vsync +
				bt->il_vbackporch + 1;
			std_info->l11 = std_info->vsize -
				(bt->il_vfrontporch - 1);
		} else {
			zynq_printk(0, "[zynq_capture]Required timing values for "
					"interlaced BT format missing\n");
			return -EINVAL;
		}
	} else {
		std_info->vsize = bt->height + bt->vfrontporch +
			bt->vsync + bt->vbackporch;
		std_info->l5 = std_info->vsize - (bt->vfrontporch - 1);
	}
	zynq_printk(0, "[zynq_capture]Custom timings BT656/1120\n");
	std_info->width = bt->width;
	std_info->height = bt->height;
	std_info->frm_fmt = bt->interlaced ? 0 : 1;
	std_info->capture_format = 0;
	std_info->hd_sd = 1;
	std_info->stdid = 0;

	vid_ch->stdid = 0;
	return 0;
}

/**
 * vpif_g_dv_timings() - G_DV_TIMINGS handler
 * @file: file ptr
 * @priv: file handle
 * @timings: digital video timings
 */
static int vpif_g_dv_timings(struct file *file, void *priv,
		struct v4l2_dv_timings *timings)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct video_obj *vid_ch = &ch->video;

	*timings = vid_ch->dv_timings;

	return 0;
}

#if (LINUX_VERSION_CODE <  KERNEL_VERSION(3,11,0))	
/*
 * vpif_g_chip_ident() - Identify the chip
 * @file: file ptr
 * @priv: file handle
 * @chip: chip identity
 *
 * Returns zero or -EINVAL if read operations fails.
 */
static int vpif_g_chip_ident(struct file *file, void *priv,
		struct v4l2_dbg_chip_ident *chip)
{

	chip->ident = V4L2_IDENT_NONE;
	chip->revision = 0;
	if (chip->match.type != V4L2_CHIP_MATCH_I2C_DRIVER &&
			chip->match.type != V4L2_CHIP_MATCH_I2C_ADDR) {
		zynq_printk(0, "[zynq_capture]match_type is invalid.\n");
		return -EINVAL;
	}

	return v4l2_device_call_until_err(&vpif_obj.v4l2_dev, 0, core,
			g_chip_ident, chip);
}
#endif

#ifdef CONFIG_VIDEO_ADV_DEBUG
/*
 * vpif_dbg_g_register() - Read register
 * @file: file ptr
 * @priv: file handle
 * @reg: register to be read
 *
 * Debugging only
 * Returns zero or -EINVAL if read operations fails.
 */
static int vpif_dbg_g_register(struct file *file, void *priv,
	struct v4l2_dbg_register *reg){
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	int ret = 0;
	
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, core, g_register, reg);
	 
	 return ret;
}

/*
 * vpif_dbg_s_register() - Write to register
 * @file: file ptr
 * @priv: file handle
 * @reg: register to be modified
 *
 * Debugging only
 * Returns zero or -EINVAL if write operations fails.
 */
static int vpif_dbg_s_register(struct file *file, void *priv,
		const struct v4l2_dbg_register *reg)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	int ret = 0;
	
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, core, s_register, reg);
	
	return ret;
}
#endif

/*
 * vpif_log_status() - Status information
 * @file: file ptr
 * @priv: file handle
 *
 * Returns zero.
 */
static int vpif_log_status(struct file *filep, void *priv)
{
	/* status for sub devices */
	v4l2_device_call_all(&vpif_obj.v4l2_dev, 0, core, log_status);

	return 0;
}


static int vpif_queryctrl(struct file *file, void *priv, struct v4l2_queryctrl *ctrl)
{
	int ret = 0;
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	//zynq_printk(1, "Call vpif_queryctrl.\n");
#if 0
	/* we only support hue/saturation/contrast/brightness */
	if (ctrl->id < V4L2_CID_BRIGHTNESS || ctrl->id > V4L2_CID_HUE)
			return -EINVAL;
#endif	
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, core,queryctrl, ctrl);
	
	return ret;
}


static int vpif_g_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
#if 0	
	/* we only support hue/saturation/contrast/brightness */
	if (ctrl->id < V4L2_CID_BRIGHTNESS || ctrl->id > V4L2_CID_HUE)
			return -EINVAL;
#endif
	if (ch->sd != NULL)
	{	
		mutex_lock(&ch->chan_lock);
		ret = v4l2_subdev_call(ch->sd, core, g_ctrl, ctrl);
		mutex_unlock(&ch->chan_lock);
	}
	return ret;
}



static int vpif_s_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
#if 0	
	/* we only support hue/saturation/contrast/brightness */
	if (ctrl->id < V4L2_CID_BRIGHTNESS || ctrl->id > V4L2_CID_HUE)
			return -EINVAL;
#endif
	if (ch->sd != NULL)
	{	
		mutex_lock(&ch->chan_lock);
		ret = v4l2_subdev_call(ch->sd, core, s_ctrl, ctrl);
		mutex_unlock(&ch->chan_lock);
	}
	return ret;
	
}

static int vpif_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub) {
		switch (sub->type) {
			case V4L2_EVENT_CTRL:	
				return v4l2_event_subscribe(fh, sub, 0, NULL);
			default:
				return -EINVAL;
		}
}

/* vpif capture ioctl operations */
static const struct v4l2_ioctl_ops vpif_ioctl_ops = {
	.vidioc_querycap        	= vpif_querycap,
	.vidioc_g_priority		= vpif_g_priority,
	.vidioc_s_priority		= vpif_s_priority,
	.vidioc_enum_fmt_vid_cap	= vpif_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap  		= vpif_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= vpif_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= vpif_try_fmt_vid_cap,
	.vidioc_enum_input		= vpif_enum_input,
	.vidioc_s_input			= vpif_s_input,
	.vidioc_g_input			= vpif_g_input,
	.vidioc_reqbufs         	= vpif_reqbufs,
	.vidioc_querybuf        	= vpif_querybuf,
	.vidioc_querystd		= vpif_querystd,
	.vidioc_s_std           	= vpif_s_std,
	.vidioc_g_std			= vpif_g_std,
	.vidioc_qbuf            	= vpif_qbuf,
	.vidioc_dqbuf           	= vpif_dqbuf,
	.vidioc_streamon        	= vpif_streamon,
	.vidioc_streamoff       	= vpif_streamoff,
	.vidioc_cropcap         	= vpif_cropcap,
	.vidioc_s_crop  			= vpif_s_crop,
	.vidioc_enum_dv_timings         = vpif_enum_dv_timings,
	.vidioc_query_dv_timings        = vpif_query_dv_timings,
	.vidioc_s_dv_timings            = vpif_s_dv_timings,
	.vidioc_g_dv_timings            = vpif_g_dv_timings,
#if (LINUX_VERSION_CODE <  KERNEL_VERSION(3,11,0))		
	.vidioc_g_chip_ident		= vpif_g_chip_ident,
#endif	
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register		= vpif_dbg_g_register,
	.vidioc_s_register		= vpif_dbg_s_register,
#endif
	.vidioc_log_status		= vpif_log_status,
	.vidioc_queryctrl                   = vpif_queryctrl,
	.vidioc_s_ctrl                      = vpif_s_ctrl,
	.vidioc_g_ctrl                      = vpif_g_ctrl,
	.vidioc_subscribe_event             =  vpif_subscribe_event,//v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event           = v4l2_event_unsubscribe
};

/* vpif file operations */
static struct v4l2_file_operations vpif_fops = {
	.owner = THIS_MODULE,
	.open = vpif_open,
	.release = vpif_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vpif_mmap,
	.poll = vpif_poll
};

/* vpif video template */
static struct video_device vpif_video_template = {
	.name		= "vpif-cap",
	.fops		= &vpif_fops,
	.minor		= -1,
	.ioctl_ops	= &vpif_ioctl_ops,
};

/**
 * initialize_vpif() - Initialize vpif data structures
 *
 * Allocate memory for data structures and initialize them
 */
static int initialize_vpif(void)
{
	int err = 0, i, j;
	int free_channel_objects_index;

	/* Default number of buffers should be 3 */
	if ((ch0_numbuffers > 0) && (ch0_numbuffers < config_params.min_numbuffers))
		ch0_numbuffers = config_params.min_numbuffers;
	if ((ch1_numbuffers > 0) && (ch1_numbuffers < config_params.min_numbuffers))
		ch1_numbuffers = config_params.min_numbuffers;
	if ((ch2_numbuffers > 0) && (ch2_numbuffers < config_params.min_numbuffers))
		ch2_numbuffers = config_params.min_numbuffers;
	if ((ch3_numbuffers > 0) && (ch3_numbuffers < config_params.min_numbuffers))
		ch3_numbuffers = config_params.min_numbuffers;

	/* Set buffer size to min buffers size if it is invalid */
	if (ch0_bufsize < config_params.min_bufsize[VPIF_CHANNEL0_VIDEO])
		ch0_bufsize =  config_params.min_bufsize[VPIF_CHANNEL0_VIDEO];
	if (ch1_bufsize < config_params.min_bufsize[VPIF_CHANNEL1_VIDEO])
		ch1_bufsize = config_params.min_bufsize[VPIF_CHANNEL1_VIDEO];
	if (ch2_bufsize < config_params.min_bufsize[VPIF_CHANNEL2_VIDEO])
		ch2_bufsize = config_params.min_bufsize[VPIF_CHANNEL2_VIDEO];		
	if (ch3_bufsize < config_params.min_bufsize[VPIF_CHANNEL3_VIDEO])
		ch3_bufsize = config_params.min_bufsize[VPIF_CHANNEL3_VIDEO];	
	
	config_params.numbuffers[VPIF_CHANNEL0_VIDEO] = ch0_numbuffers;
	config_params.numbuffers[VPIF_CHANNEL1_VIDEO] = ch1_numbuffers;
	config_params.numbuffers[VPIF_CHANNEL2_VIDEO] = ch2_numbuffers;
	config_params.numbuffers[VPIF_CHANNEL3_VIDEO] = ch3_numbuffers;
	
	if (ch0_numbuffers) config_params.channel_bufsize[VPIF_CHANNEL0_VIDEO] = ch0_bufsize;
	
	if (ch1_numbuffers) config_params.channel_bufsize[VPIF_CHANNEL1_VIDEO] = ch1_bufsize;
	
	if (ch2_numbuffers) config_params.channel_bufsize[VPIF_CHANNEL2_VIDEO] = ch2_bufsize;
	
	if (ch3_numbuffers) config_params.channel_bufsize[VPIF_CHANNEL3_VIDEO] = ch3_bufsize;
	
	/* Allocate memory for six channel objects */
	for (i = 0; i < VPIF_CAPTURE_MAX_DEVICES; i++) {
		vpif_obj.dev[i] =
		    kzalloc(sizeof(*vpif_obj.dev[i]), GFP_KERNEL);
		/* If memory allocation fails, return error */
		if (!vpif_obj.dev[i]) {
			free_channel_objects_index = i;
			err = -ENOMEM;
			goto vpif_init_free_channel_objects;
		}
	}
	return 0;

vpif_init_free_channel_objects:
	for (j = 0; j < free_channel_objects_index; j++)
		kfree(vpif_obj.dev[j]);
	return err;
}
/////////////////////////////////////////////////////////////////////////////////////


static void vpif_process_buffer_complete(struct common_obj *common)
{
	v4l2_get_timestamp(&common->cur_frm->vb.v4l2_buf.timestamp);
	vb2_buffer_done(&common->cur_frm->vb,
					    VB2_BUF_STATE_DONE);
	/* Make curFrm pointing to nextFrm */
	common->cur_frm = common->next_frm;
}

static void vpif_schedule_next_buffer(struct common_obj *common)
{
	unsigned long addr = 0;

	spin_lock(&common->irqlock);
	common->next_frm = list_entry(common->dma_queue.next,
				     struct vpif_cap_buffer, list);
	/* Remove that buffer from the buffer queue */
	list_del(&common->next_frm->list);
	spin_unlock(&common->irqlock);
	common->next_frm->vb.state = VB2_BUF_STATE_ACTIVE;

#if defined(USE_ZYNQ_MALLOC)
	addr =  zynq_malloc_plane_dma_addr(&common->next_frm->vb, 0);
#elif defined(USE_DMA_COUNTING)	
	addr = vb2_dma_contig_plane_dma_addr(&common->next_frm->vb, 0);
#endif
	if ((addr != 0) && (common->set_addr != NULL)) {
		common->set_addr(addr+common->y_off,  addr+common->uv_off);
		if (common->enable_channel != NULL) common->enable_channel(1);
		if (common->enable_channel_video != NULL)common->enable_channel_video(1);
	}

}

static void set_to_dummy_buffer(struct common_obj *common) {
	u32 addr = common->dummy_buffer_handle;
	if (common->set_addr != NULL) {
		common->set_addr(addr+common->y_off,  addr+common->uv_off);
		if (common->enable_channel != NULL) common->enable_channel(1);
		if (common->enable_channel_video != NULL)common->enable_channel_video(1);
	}
}
#if 0
static void channel_interrupt_service(struct work_struct *work) {
	
	struct channel_obj  *ch = container_of(work, struct channel_obj, interrupt_service);
	struct common_obj *common = NULL;
	int channel_id = 0;
	
	if (!ch) {
		goto exit;
	}
	channel_id = ch->channel_id;
	common = &ch->common[VPIF_VIDEO_INDEX];

	if (0 == common->started){
		goto exit;
	}
	
	spin_lock(&common->irqlock);
	if (list_empty(&common->dma_queue)) {
				spin_unlock(&common->irqlock);
				set_to_dummy_buffer(common);
				goto exit;
	}
	spin_unlock(&common->irqlock);

	if (!channel_first_int[VPIF_VIDEO_INDEX][channel_id]) vpif_process_buffer_complete(common);
	
	channel_first_int[VPIF_VIDEO_INDEX][channel_id] = 0;
	vpif_schedule_next_buffer(common);
exit:
	return;
	
}
#endif
static irqreturn_t vpif_channel_isr(int irq, void *dev_id)
{
	struct common_obj *common = NULL;
	struct channel_obj *ch;
	int channel_id = 0;
	
	channel_id = *(int *)(dev_id);
	
	ch =vpif_obj.dev[channel_id];
	
	if (!ch) {
		goto exit;
	}
	
	common = &ch->common[VPIF_VIDEO_INDEX];
	
	if (atomic_inc_and_test(&common->refcount)) {
		if (common->enable_channel_intr != NULL){
			common->enable_channel_intr(0);
		}
		if (common->is_enter_suspend) {
			common->is_completed_one_intr =1;
			goto exit;
		}
	}
	
#if 0	
	queue_work(ch->work_queue, &ch->interrupt_service);
#else
	{
		if (0 == common->started) goto exit;
		spin_lock(&common->irqlock);
		if (list_empty(&common->dma_queue)) {
				spin_unlock(&common->irqlock);
				set_to_dummy_buffer(common);
				goto exit;
		}
		spin_unlock(&common->irqlock);

		if (!channel_first_int[VPIF_VIDEO_INDEX][channel_id]) vpif_process_buffer_complete(common);
	
		channel_first_int[VPIF_VIDEO_INDEX][channel_id] = 0;
		vpif_schedule_next_buffer(common);
	}
#endif
exit:
	atomic_dec(&common->refcount);
	return IRQ_HANDLED;
}
/////////////////////////////////////////////////////////////////////////////////////
struct vpif_cap_device * vpif_capture_get_instnace(void)
{
	if ( is_initial_vpif_obj == 1) 
		return  &vpif_obj;
	else 
		return NULL;
}

int vpif_capture_release(struct pci_dev *pdev)
{
	int i = 0;
	struct channel_obj *ch = NULL;
	struct common_obj *common = NULL;
	
	if (vpif_obj.sd != NULL) kfree(vpif_obj.sd);
	
	if (vpif_obj.sd_of_sd) kfree(vpif_obj.sd_of_sd);
	
	/* un-register device */
	for (i = 0; i < VPIF_CAPTURE_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
		ch = vpif_obj.dev[i];
		if (ch != NULL)
		{
			unsigned irq = zynq_get_irq(i);
			common = &(ch->common[VPIF_VIDEO_INDEX]);
			///////////////////////////////////////////
			if (common->dummy_buffer  != NULL)
				dma_free_coherent(NULL, common->dummy_buffer_size, common->dummy_buffer, common->dummy_buffer_handle);
			///////////////////////////////////////////
			mutex_destroy(&common->lock);
			mutex_destroy(&ch->chan_lock);
			video_unregister_device(ch->video_dev);
			if (irq != (unsigned) -1) free_irq(irq, (void *)(&ch->channel_id));
			
			if ((ch->channel_id == 0) || (ch->channel_id == 1)) {
				if (ch->event_proc_work_queues != NULL) {
						cancel_delayed_work(&ch->delayed_event_proc_work);
						destroy_workqueue(ch->event_proc_work_queues);
				}
			}
#if 0			
			if (ch->work_queue != NULL) {
						cancel_work_sync(&ch->interrupt_service);
						destroy_workqueue(ch->work_queue);
			}
#endif
			kfree(ch);
			ch = NULL;
		}
	}
	v4l2_device_unregister(&vpif_obj.v4l2_dev);
#ifdef USE_ZYNQ_MALLOC		
	release_reserved_memory();
#endif
	return 0;
}

#define ADV761X_FMT_CHANGE_TO_1080P_EVENT (V4L2_EVENT_PRIVATE_START + 0x03)
#define ADV761X_FMT_CHANGE_TO_720P_EVENT (V4L2_EVENT_PRIVATE_START + 0x04)

static void delayed_event_proc_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct channel_obj *ch  = container_of(dwork, struct channel_obj,
						delayed_event_proc_work);
	struct v4l2_event event = {
		 .type = V4L2_EVENT_CTRL
	};
	if (ch != NULL) {
		if ((ch->video_dev != NULL) && (ch->usrs > 0)) {
			
			u32 width = ch->fmt_change_event_data.width;
			u32 height = ch->fmt_change_event_data.height;
			
			zynq_printk(1, "[zynq_capture] Issue the  ADV761X_FMT_CHANGE_EVENT!! (%u, %u)\n", 	ch->fmt_change_event_data.width, 	ch->fmt_change_event_data.height);
			if ((width == 1920) && (height == 1080)) {
				event.u.ctrl.type = ADV761X_FMT_CHANGE_TO_1080P_EVENT;
				v4l2_event_queue(ch->video_dev , &event);
			} else if ((width == 1280) && (height == 720)) {
				event.u.ctrl.type =  ADV761X_FMT_CHANGE_TO_720P_EVENT;
				v4l2_event_queue(ch->video_dev , &event);
			}
			ch->fmt_change_event_data.width  = 0;
			ch->fmt_change_event_data.height = 0;
		}
	}
}

static void zynq_subdev_notify(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct channel_obj *ch = NULL;
	struct i2c_client *client = NULL;
	int bus = 0;
	unsigned short addr = 0;
	int channel_id = -1;
	
	if(!sd) return;
	
	client = v4l2_get_subdevdata(sd);
	
	if (!client) return;
	
	if (!client->adapter) return;
	
	bus = i2c_adapter_id(client->adapter);	 
	addr = client->addr;	 
	
	if ((bus == SUBDEV_CH0_I2C_BUS) && (addr == SUBDEV_CH0_I2C_ADDR ))
		channel_id = 0;
	else if ((bus == SUBDEV_CH1_I2C_BUS) && (addr == SUBDEV_CH1_I2C_ADDR ))
		channel_id =1;
	else if ((bus == SUBDEV_CH2_I2C_BUS) && (addr == SUBDEV_CH2_I2C_ADDR ))
		channel_id = 2;
	
	if (channel_id == -1)	return;
	
	ch = vpif_obj.dev[channel_id];
	
	if ((cmd == ADV761X_FMT_CHANGE) && (arg != NULL)) {
		
		struct adv761x_fmt_event_data *event_data = (struct adv761x_fmt_event_data *)arg;
		zynq_printk(1, "[zynq_capture] Got ADV761X_FMT_CHANGE_EVENT (%u, %u) for  (%s) (0x%02x, 0x%02x)(id = %u)\n", event_data->width, event_data->height, sd->name,  (u32)bus, (u32)addr, (u32)channel_id);
		
		/*process the evnet after 100 ms */
		if ((ch->channel_id == 0) || (ch->channel_id == 1)) {
				if (ch->channel_id == 0)  {
					if ((event_data->width == 1920 && event_data->height == 1080 ) || (event_data->width == 1280  && event_data->height == 720))
						vpif_control_config_vin(EVIN0,  1);
					else 
						vpif_control_config_vin(EVIN0,  0);
				}else if (ch->channel_id == 1){
					if ((event_data->width == 1920 && event_data->height == 1080 ) || (event_data->width == 1280  && event_data->height == 720))
						vpif_control_config_vin(EVIN1,  1);
					else 
						vpif_control_config_vin(EVIN1,  0);
				}
				
				if (ch->event_proc_work_queues != NULL) {
					ch->fmt_change_event_data.width =  event_data->width;
					ch->fmt_change_event_data.height = event_data->height;
					queue_delayed_work(ch->event_proc_work_queues, &ch->delayed_event_proc_work, HZ / 10);
				}
		}
		
	}
	return;
}


int attach_mipi_to_parallel_bridge(int index, int is_just_check) {
	
	int i = 0;
	struct vpif_subdev_info *subdevdata = NULL;
	struct v4l2_subdev *sd = NULL;
	struct i2c_adapter *i2c_adap = NULL;
	unsigned count  = board_subdev_info_num;
	
	for (i = 0; i < count; i++) {
			subdevdata = &board_subdev_info[i];
			if (strncmp(subdevdata->name, TC358746A_ID, strlen(subdevdata->name)) == 0) {
				
				if (is_just_check) {
					zynq_printk(1, "[zynq_capture]The %s exits in the I2C board info. (idx = %d) (total num = %d)\n",subdevdata->name, i, board_subdev_info_num);
					break;
				}
				i2c_adap = zynq_get_i2c_adapter_by_bus_num(TC358746A_I2C_BUS);
				
				if (!i2c_adap) goto exit;
				
				sd =v4l2_i2c_new_subdev_board(&vpif_obj.v4l2_dev, i2c_adap, &subdevdata->board_info, NULL);
				
				if (!sd) goto exit;
				
				vpif_obj.sd_of_sd[index] = sd;
				
				if ((v4l2_device_register_subdev_nodes(&vpif_obj.v4l2_dev)) != 0){
					zynq_printk(0, "[zynq_capture]%s failed to call v4l2_device_register_subdev_nodes()!!\n",  subdevdata->name);
					goto exit;
				} 
				
				break;
			}
	}
	return 0;
exit:	
	return -1;
}

int vpif_capture_init(struct pci_dev *pdev)
{
	struct vpif_subdev_info *subdevdata;
	struct vpif_capture_config *config;
	int i = 0, j = 0, k = 0, err = 0;
//	int res_idx = 0;
	unsigned int actual_subdev_count = 0;
	struct i2c_adapter *i2c_adap = NULL;
	struct channel_obj *ch = NULL;
	struct common_obj *common = NULL;
	struct video_device *vfd = NULL;
//	struct resource *res;
	int subdev_count;
	
	pdev->dev.platform_data = &vpif_capture_cfg;
	
	vpif_dev = &pdev->dev;
	
//	vpif_dbg(1, debug_capture, "vpif_base = %p,  vpif_dev = %p vpif_dev->platform_data = %p\n", vpif_base, vpif_dev,vpif_dev->platform_data );
	
	err = initialize_vpif();
	if (err) {
		zynq_printk(0, "[zynq_capture]Error initializing vpif\n");
		return err;
	}
	
	vpif_obj.v4l2_dev.notify = zynq_subdev_notify;
	err = v4l2_device_register(vpif_dev, &vpif_obj.v4l2_dev);
	if (err) {
		zynq_printk(0, "[zynq_capture]Error registering v4l2 device\n");
		goto rls_ch_obj;
	}

	for (i = 0; i < VPIF_CAPTURE_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
		ch = vpif_obj.dev[i];
		/* Allocate memory for video device */
		vfd = video_device_alloc();
		if (NULL == vfd) {
			for (j = 0; j < i; j++) {
				ch = vpif_obj.dev[j];
				video_device_release(ch->video_dev);
			}
			err = -ENOMEM;
			goto unregister_v4l2_device;
		}

		/* Initialize field of video device */
		*vfd = vpif_video_template;
		vfd->v4l2_dev = &vpif_obj.v4l2_dev;
		vfd->release = video_device_release;
		snprintf(vfd->name, sizeof(vfd->name),
			 "VPIF_Capture_DRIVER_V%s",
			 VPIF_CAPTURE_VERSION);
		/* Set video_dev to the video device */
		ch->video_dev = vfd;
	}
	
	for (j = 0; j < VPIF_CAPTURE_MAX_DEVICES; j++) {
			ch = vpif_obj.dev[j];
			ch->channel_id = j;
	}

	for (i = 0; i < VPIF_CAPTURE_MAX_DEVICES; i++) {
		unsigned irq = zynq_get_irq(i);
		ch = vpif_obj.dev[i];
		if (!ch) break;
		if (irq == (unsigned) -1) continue;
		err = request_irq(irq,
					vpif_channel_isr,
					IRQF_ONESHOT   | IRQF_TRIGGER_HIGH,//IRQF_TRIGGER_HIGH, //IRQF_TRIGGER_RISING,
					"zynq_capture",  &ch->channel_id);
		if (err) {
			zynq_printk(0, "[zynq_capture] Call request_irq() for channel %d failed!!\n", ch->channel_id);
			err = -EBUSY;
			for (j = 0; j < i; j++) {
				 irq = zynq_get_irq(j);
				free_irq(irq, (void *)(&ch->channel_id));
		}
		goto rls_video_device;
	}
			zynq_printk(1, "[zynq_capture]xxx gg  irq %u for channel %d\n", irq, i);
#if 0
		/* Setup work queue */
		memset(ch->work_queue_name, 0x0, sizeof(ch->work_queue_name));
		snprintf(ch->work_queue_name, sizeof(ch->work_queue_name), "ch_workqueue%d", ch->channel_id);
		ch->work_queue = create_singlethread_workqueue(ch->work_queue_name);
		if (!ch->work_queue) {
				zynq_printk(1, "[zynq_capture] Work queue for %d setup failed!! \n", ch->channel_id);
		} else {
					INIT_WORK(&ch->interrupt_service, channel_interrupt_service);
		}
#endif
	}

	config = pdev->dev.platform_data;

	subdev_count = config->subdev_count;
	vpif_obj.sd = kzalloc(sizeof(struct v4l2_subdev *) * subdev_count, GFP_KERNEL);
	if (vpif_obj.sd == NULL) {
		zynq_printk(0, "[zynq_capture]unable to allocate memory for subdevice pointers\n");
		err = -ENOMEM;
		goto rls_irq;
	}
	
	vpif_obj.sd_of_sd=  kzalloc(sizeof(struct v4l2_subdev *) * subdev_count, GFP_KERNEL);
	if (vpif_obj.sd_of_sd == NULL) {
		zynq_printk(0, "[zynq_capture]unable to allocate memory for subdevice of subdevice pointers\n");
		err = -ENOMEM;
		goto rls_irq;
	}
	
	memset(vpif_obj.sd, 0x0 , sizeof(struct v4l2_subdev *) * subdev_count);
	memset(vpif_obj.sd_of_sd, 0x0 , sizeof(struct v4l2_subdev *) * subdev_count);
	
	for (i = 0; i < subdev_count; i++) {
		
		unsigned int bus_num = (unsigned int)-1;
		
		subdevdata = &config->subdev_info[i];
		
		if (!subdevdata) break;
		
		if (strncmp(subdevdata->name, M10MO_ID, strlen(subdevdata->name)) == 0) {
			zynq_printk(1, "[zynq_capture]The %s exits in the I2C board info. (idx = %d) (total num = %d)\n",subdevdata->name, i, board_subdev_info_num);
			attach_mipi_to_parallel_bridge(i, 1);
		}
		
		if (subdevdata->enable == 0) continue;
		
		if (i == 0)
			bus_num =  SUBDEV_CH0_I2C_BUS;
		else if (i == 1)
			bus_num =  SUBDEV_CH1_I2C_BUS;
		else if (i == 2)
			bus_num = SUBDEV_CH2_I2C_BUS;
		
		if (bus_num == (unsigned int)-1 )
		{
				zynq_printk(0, "[zynq_capture] I2C bus number is %d\n", (int)bus_num);
				err = -EBUSY;
				goto rls_sd_obj;
		}
		
		i2c_adap = zynq_get_i2c_adapter_by_bus_num(bus_num);
		
		if (i2c_adap == NULL) 
		{
				zynq_printk(0, "[zynq_capture]unable to get i2c bus %d obj\n", bus_num);
				err = -EBUSY;
				goto rls_sd_obj;
		}
		
		if (strncmp(subdevdata->name, M10MO_ID, strlen(subdevdata->name)) == 0) {
			if (attach_mipi_to_parallel_bridge(i, 0) != 0) {
				zynq_printk(0, "[zynq_capture]Failed to call attach_mipi_to_parallel_bridge()!!\n");
				err = -EBUSY;
				goto rls_sd_obj;
			}
		} 
		
		vpif_obj.sd[i] = v4l2_i2c_new_subdev_board(&vpif_obj.v4l2_dev, i2c_adap, &subdevdata->board_info, NULL);
		
		if (!vpif_obj.sd[i]) {
				zynq_printk(0, "[zynq_capture]Error registering v4l2 subdevice %s\n",  subdevdata->name);
				err = -EBUSY;
				goto rls_sd_obj;
		}
		
		if ((err = v4l2_device_register_subdev_nodes(&vpif_obj.v4l2_dev)) != 0){
			zynq_printk(0, "[zynq_capture]%s failed to call v4l2_device_register_subdev_nodes()!!\n",  subdevdata->name);
			goto rls_sd_obj;
		} 
		
		actual_subdev_count++;
		
		v4l2_info(&vpif_obj.v4l2_dev, "[zynq_capture]subdev : (name, bus, addr) - --> (%s, 0x%02x, 0x%02x)\n",  subdevdata->name, bus_num, subdevdata->board_info.addr);
	}
	
	config->subdev_count = subdev_count = actual_subdev_count;
	v4l2_info(&vpif_obj.v4l2_dev, "[zynq_capture]subdev number : %u\n", subdev_count);
	
	for (j = 0; j < VPIF_CAPTURE_MAX_DEVICES; j++) {
		ch = vpif_obj.dev[j];
		ch->usrs  = 0;
		ch->channel_id = j;
		mutex_init(&ch->chan_lock);
		common = &(ch->common[VPIF_VIDEO_INDEX]);
		
		atomic_set(&common->refcount, -1);
		common->is_start_streaming = 0;
		common->is_stop_streaming = 0;
	
		common-> is_completed_one_intr = 0;
		common->is_enter_suspend = 0;
		
		spin_lock_init(&common->irqlock);
		mutex_init(&common->lock);
		ch->video_dev->lock = &common->lock;
		//////////////////////////////////
		common->dummy_buffer_size = 1920 * 1080 * 2;
		common->dummy_buffer = dma_alloc_coherent(NULL, 	common->dummy_buffer_size , &common->dummy_buffer_handle, GFP_KERNEL);
		/////////////////////////////////
		/* Initialize prio member of channel object */
		v4l2_prio_init(&ch->prio);
		video_set_drvdata(ch->video_dev, ch);
		vpif_config_format(ch);
		/* select input 0 */
		err = vpif_set_input(config, ch, 0);
		if (err)
			goto rls_sd_obj;

		err = video_register_device(ch->video_dev,
					    VFL_TYPE_GRABBER, (j ? 1 : 0));
		if (err)
			goto rls_sd_obj;
		
		initialize_channel_video_obj(ch);
		initialize_channel_pxiel_format(ch);
		initial_channel_vpif_parameters(ch);
		
		//NOTE: /dev/video0 and /dev/video1 for the  adv7611 sources
		if ((ch->channel_id == 0) || (ch->channel_id == 1)) {
			char channel_name[128];
			memset(channel_name, 0x0, sizeof(channel_name));
			snprintf(channel_name, sizeof(channel_name), "zynq_capture_%d", ch->channel_id);
			ch->event_proc_work_queues = create_singlethread_workqueue(channel_name);
			if (!ch->event_proc_work_queues) {
				zynq_printk(0, "[zynq_capture]Could not create work queue for processing evnet.\n");
			} else {
				INIT_DELAYED_WORK(&ch->delayed_event_proc_work, delayed_event_proc_work_func);
			}
		}
	}

#ifdef USE_ZYNQ_MALLOC	
	 if (allocate_reserved_memory() != 0)  goto unregister_video_device;
#endif
	v4l2_info(&vpif_obj.v4l2_dev, "[zynq_capture] gg Zynq capture driver initialized\n");
	
	if (err == 0)  
		is_initial_vpif_obj = 1;
	else
		goto unregister_video_device;
	
	return err;
	
unregister_video_device:
	for (k = 0; k < j; k++) {
		/* Get the pointer to the channel object */
		ch = vpif_obj.dev[k];
		/* Unregister video device */
		video_unregister_device(ch->video_dev);
	}
rls_sd_obj:	
	if (vpif_obj.sd) kfree(vpif_obj.sd);
	if (vpif_obj.sd_of_sd) kfree(vpif_obj.sd_of_sd);
rls_irq:
	for (i = 0; i < VPIF_CAPTURE_MAX_DEVICES; i++) {
			ch = vpif_obj.dev[i];
			if (ch != NULL) {
				unsigned irq = zynq_get_irq(i);
				if (irq == (unsigned) -1) continue;
				free_irq(irq, (void *)(&ch->channel_id));
			}
	}
rls_video_device:
	for (i = 0; i < VPIF_CAPTURE_MAX_DEVICES; i++) {
		ch = vpif_obj.dev[i];
		if (ch != NULL) 
			if (ch->video_dev != NULL)
				video_unregister_device(ch->video_dev);
	}
unregister_v4l2_device:
	v4l2_device_unregister(&vpif_obj.v4l2_dev);	
	
rls_ch_obj:
	for (i = 0; i < VPIF_CAPTURE_MAX_DEVICES; i++) {
			ch = vpif_obj.dev[i];
			if (ch != NULL) {
				if ((ch->channel_id == 0) || (ch->channel_id == 1)) {
					if (ch->event_proc_work_queues != NULL) {
						cancel_delayed_work(&ch->delayed_event_proc_work);
						destroy_workqueue(ch->event_proc_work_queues);
					}
				}
#if 0
					if (ch->work_queue != NULL) {
						cancel_work_sync(&ch->interrupt_service);
						destroy_workqueue(ch->work_queue);
					}
#endif
					kfree(ch);
			}
	}
	
	v4l2_info(&vpif_obj.v4l2_dev, "failed to  initialize zynq capture driver !!\n");
	return err;
}

#ifdef CONFIG_PM_SLEEP

static inline void zynq_capture_delay(unsigned int sec) {
	
	//unsigned long timeout = jiffies + HZ * sec; 
	//while (time_before(jiffies, timeout)) cpu_relax(  );
	 msleep_interruptible(sec * 1000);
}
int vpif_capture_suspend(void)
{
	int j = 0;
	struct channel_obj *ch = NULL;
	struct common_obj *common = NULL;
	zynq_printk(0,"[zynq_capture]Enter vpif_capture_suspend().\n");
	for (j = 0; j < VPIF_CAPTURE_MAX_DEVICES; j++) {
			ch = vpif_obj.dev[j];
			if (ch) {
				common = &(ch->common[VPIF_VIDEO_INDEX]);
				if (common){
					if ((common->is_start_streaming == 1) && (common->is_stop_streaming == 0)) {
						common->is_enter_suspend = 1;
#if 0			
						while(1) {
							if (common->is_completed_one_intr ==1) {
								common->is_completed_one_intr = 0;
							}
							zynq_capture_delay(1);
						}
						common->enable_channel_video(0);
#endif
					}
				}
			}
	}
	zynq_printk(0,"[zynq_capture]Leave vpif_capture_suspend().\n");
	return  0;
}
int vpif_capture_resume(void)
{
	int j = 0;
	struct channel_obj *ch = NULL;
	struct common_obj *common = NULL;
	zynq_printk(0,"[zynq_capture]Enter vpif_capture_resume()\n");
	for (j = 0; j < VPIF_CAPTURE_MAX_DEVICES; j++) {
			ch = vpif_obj.dev[j];
			if (ch) {
				common = &(ch->common[VPIF_VIDEO_INDEX]);
				if (common){
					if ((common->is_start_streaming == 1) && (common->is_stop_streaming == 0)) {
						common->is_enter_suspend = 0;
#if 0			
						common->enable_channel(1);
						common->enable_channel_video(1);
#endif
					}
				}
			}
	}
	zynq_printk(0,"[zynq_capture]Leave vpif_capture_resume().\n");
	return 0;
}
#endif
