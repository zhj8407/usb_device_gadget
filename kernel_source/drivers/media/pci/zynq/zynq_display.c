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
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/irq.h>
#include <asm/page.h>

#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/sched.h>

#include <media/adv7343.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
//#include <media/v4l2-chip-ident.h>

//#define USE_DMA_COUNTING 1
#define USE_ZYNQ_MALLOC 1
//#define USE_VIDEO_INPUT_WINDOW_MEM 1


#if defined(USE_DMA_COUNTING) 
	#include <media/videobuf2-dma-contig.h>
#elif defined(USE_ZYNQ_MALLOC)
	#include "zynq_malloc.h"
#else
	#include <media/videobuf2-vmalloc.h>
#endif

#include "zynq_debug.h"
#include "zynq_display_priv.h"
#include "zynq_core.h"
#include "zynq_board.h"

#include "modules/zynq_scaler.h"
#include "modules/zynq_osd.h"
#include "modules/zynq_video_selector.h"
#include "modules/zynq_video_timing_controller.h"
#include "modules/zynq_resampler.h"

static u32 ch2_numbuffers = 3;
static u32 ch2_bufsize = 1920 * 1080 * 2;
static u8 channel_first_int[VPIF_NUMOBJECTS][2] = { {1, 1} };

#define MAX_DISPLAY_SUBDEV_NUM 2
#define DISPLAY_SUBDEV_IDX_0 3
#define DISPLAY_SUBDEV_IDX_1 4


static struct vpif_config_params config_params = {
	.min_numbuffers		= 3,
	.numbuffers[0]		= 3,
	.min_bufsize[0]		= 1920 * 480 * 2,
	.channel_bufsize[0]	= 1920 * 1080 * 2,
	.video_limit[0] = 1920 * 1080 *2 * 5,
	.channel_bufstride[0] = 1920,
	.pixelformat[0] = V4L2_PIX_FMT_YUV422P
};

static struct vpif_disp_device vpif_obj = { 
	.dev ={NULL},
	.sd = NULL																								
};
static struct device *vpif_dev;

static void print_pixel_format( struct v4l2_pix_format *pixfmt)
{
	if (!pixfmt) return;
	
	zynq_printk(0, "[zynq_display]Video format: %c%c%c%c (%08x) %ux%u\n",
		(pixfmt->pixelformat >> 0) & 0xff,
		(pixfmt->pixelformat >> 8) & 0xff,
		(pixfmt->pixelformat >> 16) & 0xff,
		(pixfmt->pixelformat >> 24) & 0xff,
		pixfmt->pixelformat,
		pixfmt->width, pixfmt->height);
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
		
		common->fmt.fmt.pix.field = V4L2_FIELD_NONE;
		common->fmt.fmt.pix.bytesperline = config_params.channel_bufstride[ch->channel_id ];
		common->fmt.fmt.pix.sizeimage = config_params.channel_bufsize[ch->channel_id ];
		common->fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE240M ;
		common->fmt.fmt.pix.priv = 0;
		
		print_pixel_format(&common->fmt.fmt.pix);
		return ret;
}
///////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_ZYNQ_MALLOC

#define MY_DATA_SIZE (16*1024*1024)
dma_addr_t 	display_dma_handle;
void *display_dma_addr = NULL;
size_t display_dma_size = PAGE_ALIGN(MY_DATA_SIZE + PAGE_SIZE);

#define FRAME_BUFFER_OFFSET 0
dma_addr_t  display_dam_framebuffer_handle;
void *display_dam_framebuffer_addr = NULL;
size_t display_dma_framebuffer_size = PAGE_ALIGN(MY_DATA_SIZE - FRAME_BUFFER_OFFSET + PAGE_SIZE);

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
#ifdef USE_VIDEO_INPUT_WINDOW_MEM	
	if (zynq_video_input_window_base == NULL) goto exit;
	
	if (zynq_video_input_window_base_dma_handle  == 0) goto exit;
	
	display_dam_framebuffer_handle = (dma_addr_t) (zynq_video_input_window_base_dma_handle + FRAME_BUFFER_OFFSET);
	display_dam_framebuffer_addr = zynq_video_input_window_base + FRAME_BUFFER_OFFSET;		
	display_dma_framebuffer_size = zynq_video_input_window_len - FRAME_BUFFER_OFFSET;
	
#else	
	display_dma_addr =  dma_alloc_coherent(NULL, display_dma_size , &display_dma_handle, GFP_KERNEL);
	if (!display_dma_addr)
	{
		zynq_printk(0,"[zynq_display]failed to allocate  %d memory !!\n", display_dma_size);
		status = -1;
		goto exit;
	} else {
		//zynq_printk(0, "[zynq_display]successfull to allocate %d memory (virt: %p, phy:%p) !!\n", display_dma_size, display_dma_addr, (void *)display_dma_handle);
		//	output_test(display_dma_addr ,display_dma_handle, display_dma_size, "This is the dma_alloc_coherent() string");			
			display_dam_framebuffer_handle = (dma_addr_t) ((u8 *)display_dma_handle + FRAME_BUFFER_OFFSET);
			display_dam_framebuffer_addr = display_dma_addr  + FRAME_BUFFER_OFFSET;		
	}	
#endif
	zynq_printk(0, "[zynq_display]successfull to allocate %d bytes video memory (virt: %p, phy:%p) !!\n", display_dma_framebuffer_size, display_dam_framebuffer_addr , (void *)display_dam_framebuffer_handle);
exit:	
	return status;
}

static int release_reserved_memory(void)
{
#ifndef USE_VIDEO_INPUT_WINDOW_MEM		
	if (display_dma_addr	!=	NULL){	
		dma_free_coherent(NULL, display_dma_size, display_dma_addr, display_dma_handle);
		display_dma_addr = NULL;
	}
#endif	
	return  0;
}

#endif
///////////////////////////////////////////////////////////////////////////////////////

struct thread_context {
	struct channel_obj *channel;
	unsigned 		   ms;
	unsigned long  jiffies;
	struct v4l2_fract          timeperframe;
	
	int ini_jiffies;
	struct task_struct *kthread;
	wait_queue_head_t wq;
	
};

#define FPS_MAX 1000

static const struct v4l2_fract
	tpf_min     = {.numerator = 1,		.denominator = FPS_MAX},
	tpf_max     = {.numerator = FPS_MAX,	.denominator = 1},
	tpf_30fps = {.numerator = 1001,	.denominator = 30000},	/* 30 fps*/
	tpf_15fps = {.numerator = 1001,	.denominator = 15000},	/* 15 fps*/
	tpf_60fps = {.numerator = 1001,	.denominator = 60000},	/* 60 fps*/
	tpf_10fps = {.numerator = 1001,	.denominator = 10000};	/* 10 fps*/

static struct thread_context  gThreadContext[VPIF_DISPLAY_MAX_DEVICES];

static void process_progressive_mode(struct common_obj *common)
{
	spin_lock(&common->irqlock);
	/* Get the next buffer from buffer queue */
	common->next_frm = list_entry(common->dma_queue.next,
				struct vpif_disp_buffer, list);
	/* Remove that buffer from the buffer queue */
	list_del(&common->next_frm->list);
	spin_unlock(&common->irqlock);
	/* Mark status of the buffer as active */
	common->next_frm->vb.state = VB2_BUF_STATE_ACTIVE;

	return;
}

static void thread_tick(struct thread_context  *ctx)
{
	struct channel_obj *ch = ctx->channel;
	struct common_obj *common = NULL;
	unsigned int channel_id = 0;
	if (!ch)
	{
		zynq_printk(0, "[zynq_display]The channel obj is NULL!!\n");
		goto exit;
	}
	
	common = &ch->common[VPIF_VIDEO_INDEX];
	channel_id = (unsigned int)ch->channel_id;
	
	spin_lock(&common->irqlock);
	if (list_empty(&common->dma_queue)) {
				spin_unlock(&common->irqlock);
				zynq_printk(1, "[zynq_display]The buffer queue is emty!!\n");
				goto exit;
	}
	spin_unlock(&common->irqlock);
	
	if (!channel_first_int[VPIF_VIDEO_INDEX][channel_id]) {
				/* Mark status of the cur_frm to
				 * done and unlock semaphore on it */
				v4l2_get_timestamp(&common->cur_frm->vb.
						   v4l2_buf.timestamp);
		
				//NOTE: Check the content of current frame.
				{
						void *vbuf = vb2_plane_vaddr(&common->cur_frm->vb, 0);
						unsigned char *y_buffer = (unsigned char *)vbuf;
						u32 bytesused = common->cur_frm->vb.v4l2_planes[0].bytesused;
						if (y_buffer != NULL)
							zynq_printk(2, "[zynq_display] [0, 1, 2] -> [%02x, %02x, %02x] (ptr, len)-->(%p,%u)\n", y_buffer[0], y_buffer[1], y_buffer[2], y_buffer,  bytesused);
				}
				
				vb2_buffer_done(&common->cur_frm->vb,VB2_BUF_STATE_DONE);
				/* Make cur_frm pointing to next_frm */
				common->cur_frm = common->next_frm;
	}
	channel_first_int[VPIF_VIDEO_INDEX][channel_id] = 0;
	process_progressive_mode(common);
exit:
#if 0
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
		zynq_printk(1, "[zynq_display] timestamp: %s\n", str);
	}
#endif	
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

	 zynq_printk(0, "[zynq_display]thread started\n");

	set_freezable();

	for (;;) {
		thread_sleep(ctx);

		if (kthread_should_stop())
			break;
	}
	 zynq_printk(0,  "[zynq_display] thread: exit\n");
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
	
	ctx->ini_jiffies = ctx->jiffies;
	init_waitqueue_head(&ctx->wq);
	ctx->kthread = kthread_run(thread_handler, ctx, "zynq_display_thread");

	if (IS_ERR(ctx->kthread)) {
		 zynq_printk(0,  "[zynq_display]kernel_thread() failed\n");
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

//////////////////////////////////////////////////////////////////////////////////////

/*
 * buffer_prepare: This is the callback function called from vb2_qbuf()
 * function the buffer is prepared and user space virtual address is converted
 * into physical address
 */
static int vpif_buffer_prepare(struct vb2_buffer *vb)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct common_obj *common;
	
	common = &fh->channel->common[VPIF_VIDEO_INDEX];
	if (vb->state != VB2_BUF_STATE_ACTIVE &&
		vb->state != VB2_BUF_STATE_PREPARED) {
		vb2_set_plane_payload(vb, 0, common->fmt.fmt.pix.sizeimage);
		if (vb2_plane_vaddr(vb, 0) &&
		vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0))
			goto buf_align_exit;
	}
	return 0;

buf_align_exit:
	zynq_printk(0, "[zynq_display]buffer offset not aligned to 8 bytes\n");
	return -EINVAL;
}

/*
 * vpif_buffer_queue_setup: This function allocates memory for the buffers
 */
static int vpif_buffer_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	unsigned int size;

	size = common->fmt.fmt.pix.sizeimage;

	if (config_params.video_limit[ch->channel_id])
				while (size * *nbuffers > config_params.video_limit[ch->channel_id])
					(*nbuffers)--;
				
	if (*nbuffers < config_params.min_numbuffers)
			*nbuffers = config_params.min_numbuffers;
	
	*nplanes = 1;
	sizes[0] = size;
	
	zynq_printk(0, "[zynq_display] nbuffers = %u size = %u\n", (*nbuffers), size);
	
	alloc_ctxs[0] = common->alloc_ctx;

	return 0;
}

/*
 * vpif_buffer_queue: This function adds the buffer to DMA queue
 */
static void vpif_buffer_queue(struct vb2_buffer *vb)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct vpif_disp_buffer *buf = container_of(vb,
				struct vpif_disp_buffer, vb);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;
	unsigned long flags;

	common = &ch->common[VPIF_VIDEO_INDEX];

	/* add the buffer to the DMA queue */
	spin_lock_irqsave(&common->irqlock, flags);
	list_add_tail(&buf->list, &common->dma_queue);
	spin_unlock_irqrestore(&common->irqlock, flags);
}

/*
 * vpif_buf_cleanup: This function is called from the videobuf2 layer to
 * free memory allocated to the buffers
 */
static void vpif_buf_cleanup(struct vb2_buffer *vb)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct vpif_disp_buffer *buf = container_of(vb,
					struct vpif_disp_buffer, vb);
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
	struct vpif_disp_buffer *buf = container_of(vb,
					struct vpif_disp_buffer, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static int vpif_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	unsigned long flags;
	
	/* If buffer queue is empty, return error */
	spin_lock_irqsave(&common->irqlock, flags);
	if (list_empty(&common->dma_queue)) {
		spin_unlock_irqrestore(&common->irqlock, flags);
		zynq_printk(0, "[zynq_display]buffer queue is empty\n");
		return -EIO;
	}

	/* Get the next frame from the buffer queue */
	common->next_frm = common->cur_frm =
			    list_entry(common->dma_queue.next,
				       struct vpif_disp_buffer, list);

	list_del(&common->cur_frm->list);
	spin_unlock_irqrestore(&common->irqlock, flags);
	/* Mark state of the current frame to active */
	common->cur_frm->vb.state = VB2_BUF_STATE_ACTIVE;
	
	channel_first_int[VPIF_VIDEO_INDEX][ch->channel_id] = 1;
	
	/* Initialize field_id and started member */
	ch->field_id = 0;
	common->started = 1;
	
	 thread_init(ch);
	 
	return 0;
}

/* abort streaming and wait for last buffer */
static int vpif_stop_streaming(struct vb2_queue *vq)
{
	struct vpif_fh *fh = vb2_get_drv_priv(vq);
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;
	unsigned long flags;

	if (!vb2_is_streaming(vq))
		return 0;
	
	common = &ch->common[VPIF_VIDEO_INDEX];

	 thread_rls(ch);
	
	/* release all active buffers */
	spin_lock_irqsave(&common->irqlock, flags);
	while (!list_empty(&common->dma_queue)) {
		common->next_frm = list_entry(common->dma_queue.next,
						struct vpif_disp_buffer, list);
		list_del(&common->next_frm->list);
		vb2_buffer_done(&common->next_frm->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&common->irqlock, flags);

	
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


static int vpif_update_std_info(struct channel_obj *ch)
{
	struct video_obj *vid_ch = &ch->video;
	struct vpif_params *vpifparams = &ch->vpifparams;
	struct vpif_channel_config_params *std_info = &vpifparams->std_info;
	const struct vpif_channel_config_params *config;

	int i;

	for (i = 0; i < vpif_ch_params_count; i++) {
		config = &vpif_ch_params[i];
		if (config->hd_sd == 0) {
			zynq_printk(0, "[zynq_display] SD format\n");
			if (config->stdid & vid_ch->stdid) {
				memcpy(std_info, config, sizeof(*config));
				break;
			}
		}
	}

	if (i == vpif_ch_params_count) {
		zynq_printk(0, "[zynq_display] Format not found\n");
		return -EINVAL;
	}

	return 0;
}

static int vpif_update_resolution(struct channel_obj *ch)
{
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct video_obj *vid_ch = &ch->video;
	struct vpif_params *vpifparams = &ch->vpifparams;
	struct vpif_channel_config_params *std_info = &vpifparams->std_info;

	if (!vid_ch->stdid && !vid_ch->dv_timings.bt.height)
		return -EINVAL;

	if (vid_ch->stdid) {
		if (vpif_update_std_info(ch))
			return -EINVAL;
	}

	common->fmt.fmt.pix.width = std_info->width;
	common->fmt.fmt.pix.height = std_info->height;
	zynq_printk(0, "[zynq_display] Pixel details: Width = %d,Height = %d\n",
			common->fmt.fmt.pix.width, common->fmt.fmt.pix.height);

	/* Set height and width paramateres */
	common->height = std_info->height;
	common->width = std_info->width;

	return 0;
}

static void vpif_config_format(struct channel_obj *ch)
{
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	common->fmt.fmt.pix.field = V4L2_FIELD_ANY;
	if (config_params.numbuffers[ch->channel_id] == 0)
		common->memory = V4L2_MEMORY_USERPTR;
	else
		common->memory = V4L2_MEMORY_MMAP;

	common->fmt.fmt.pix.sizeimage =
			config_params.channel_bufsize[ch->channel_id];
	common->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
	common->fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
}

static int vpif_check_format(struct channel_obj *ch,
			     struct v4l2_pix_format *pixfmt)
{
	return 0;
}


/*
 * vpif_mmap: It is used to map kernel space buffers into user spaces
 */
static int vpif_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &(ch->common[VPIF_VIDEO_INDEX]);
	int ret;

	if (mutex_lock_interruptible(&common->lock))
		return -ERESTARTSYS;
	ret = vb2_mmap(&common->buffer_queue, vma);
	mutex_unlock(&common->lock);
	return ret;
}

/*
 * vpif_poll: It is used for select/poll system call
 */
static unsigned int vpif_poll(struct file *filep, poll_table *wait)
{
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	unsigned int res = 0;

	if (common->started) {
		mutex_lock(&common->lock);
		res = vb2_poll(&common->buffer_queue, filep, wait);
		mutex_unlock(&common->lock);
	}

	return res;
}

/*
 * vpif_open: It creates object of file handle structure and stores it in
 * private_data member of filepointer
 */
static int vpif_open(struct file *filep)
{
	struct video_device *vdev = video_devdata(filep);
	struct channel_obj *ch = video_get_drvdata(vdev);
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct vpif_fh *fh;

	/* Allocate memory for the file handle object */
	fh = kzalloc(sizeof(struct vpif_fh), GFP_KERNEL);
	if (fh == NULL) {
		zynq_printk(0, "[zynq_display]unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}

	if (mutex_lock_interruptible(&common->lock)) {
		kfree(fh);
		return -ERESTARTSYS;
	}
	/* store pointer to fh in private_data member of filep */
	filep->private_data = fh;
	fh->channel = ch;
	fh->initialized = 0;
	if (!ch->initialized) {
		fh->initialized = 1;
		ch->initialized = 1;
		memset(&ch->vpifparams, 0, sizeof(ch->vpifparams));
	}

	/* Increment channel usrs counter */
	atomic_inc(&ch->usrs);
	/* Set io_allowed[VPIF_VIDEO_INDEX] member to false */
	fh->io_allowed[VPIF_VIDEO_INDEX] = 0;
	/* Initialize priority of this instance to default priority */
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&ch->prio, &fh->prio);
	mutex_unlock(&common->lock);

	return 0;
}

/*
 * vpif_release: This function deletes buffer queue, frees the buffers and
 * the vpif file handle
 */
static int vpif_release(struct file *filep)
{
	struct vpif_fh *fh = filep->private_data;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	mutex_lock(&common->lock);
	/* if this instance is doing IO */
	if (fh->io_allowed[VPIF_VIDEO_INDEX]) {
		/* Reset io_usrs member of channel object */
		common->io_usrs = 0;

		common->started = 0;

		/* Free buffers allocated */
		vb2_queue_release(&common->buffer_queue);
#if  defined(USE_DMA_COUNTING	)	
		vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
#elif defined(USE_ZYNQ_MALLOC)
		 zynq_malloc_cleanup_ctx(common->alloc_ctx);
#endif		
		common->numbuffers =
		    config_params.numbuffers[ch->channel_id];
	}

	/* Decrement channel usrs counter */
	atomic_dec(&ch->usrs);
	/* If this file handle has initialize encoder device, reset it */
	if (fh->initialized)
		ch->initialized = 0;

	/* Close the priority */
	v4l2_prio_close(&ch->prio, fh->prio);
	filep->private_data = NULL;
	fh->initialized = 0;
	mutex_unlock(&common->lock);
	kfree(fh);

	return 0;
}

/* functions implementing ioctls */
/**
 * vpif_querycap() - QUERYCAP handler
 * @file: file ptr
 * @priv: file handle
 * @cap: ptr to v4l2_capability structure
 */
static int vpif_querycap(struct file *file, void  *priv,
				struct v4l2_capability *cap)
{
	//struct vpif_display_config *config = vpif_dev->platform_data;

	cap->device_caps = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	snprintf(cap->driver, sizeof(cap->driver), "%s", dev_name(vpif_dev));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(vpif_dev));
	//if (config != NULL)
	//	strlcpy(cap->card, config->card_name, sizeof(cap->card));
	//else
		strlcpy(cap->card, VPIF_DISPLAY_CARD_NAME, sizeof(cap->card));


	return 0;
}

static int vpif_enum_fmt_vid_out(struct file *file, void  *priv,
					struct v4l2_fmtdesc *fmt)
{
	if (fmt->index != 0) {
		zynq_printk(0, "[zynq_display]Invalid format index\n");
		return -EINVAL;
	}

	/* Fill in the information about format */
	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	strcpy(fmt->description, "YCbCr4:2:2 YC Planar");
	fmt->pixelformat = V4L2_PIX_FMT_YUV422P;

	return 0;
}

static int vpif_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	/* Check the validity of the buffer type */
	if (common->fmt.type != fmt->type)
		return -EINVAL;
#if 0
	if (vpif_update_resolution(ch))
		return -EINVAL;
#endif
	*fmt = common->fmt;
	return 0;
}

static int vpif_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpif_fh *fh = priv;
	struct v4l2_pix_format *pixfmt;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	int ret = 0;

	if ((VPIF_CHANNEL2_VIDEO == ch->channel_id)
	    || (VPIF_CHANNEL3_VIDEO == ch->channel_id)) {
		if (!fh->initialized) {
			zynq_printk(0,  "[zynq_display]Channel Busy\n");
			return -EBUSY;
		}

		/* Check for the priority */
		ret = v4l2_prio_check(&ch->prio, fh->prio);
		if (0 != ret)
			return ret;
		fh->initialized = 1;
	}

	if (common->started) {
		zynq_printk(0, "[zynq_display]Streaming in progress\n");
		return -EBUSY;
	}

	pixfmt = &fmt->fmt.pix;
	/* Check for valid field format */
	ret = vpif_check_format(ch, pixfmt);
	if (ret)
		return ret;
#if 0
	/* store the pix format in the channel object */
	common->fmt.fmt.pix = *pixfmt;
	/* store the format in the channel object */
	common->fmt = *fmt;
#endif
		/* store the format in the channel object */
	//common->fmt = *fmt;
	common->fmt.fmt.pix.width = pixfmt->width;
	common->fmt.fmt.pix.height = 	pixfmt->height;
	common->fmt.fmt.pix.pixelformat = 	pixfmt->pixelformat;
	if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P)
		common->fmt.fmt.pix.sizeimage = common->fmt.fmt.pix.width * common->fmt.fmt.pix.height *2;
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
		common->fmt.fmt.pix.sizeimage = (common->fmt.fmt.pix.width * common->fmt.fmt.pix.height * 3 )/2;
	common->fmt.fmt.pix.bytesperline = common->fmt.fmt.pix.width;
	
	return 0;
}

static int vpif_try_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int ret = 0;

	ret = vpif_check_format(ch, pixfmt);
	if (ret) {
		*pixfmt = common->fmt.fmt.pix;
		pixfmt->sizeimage = pixfmt->width * pixfmt->height * 2;
	}

	return ret;
}

static int vpif_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbuf)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common;
	struct vb2_queue *q;
	u8 index = 0;
	int ret;

	/* This file handle has not initialized the channel,
	   It is not allowed to do settings */
	if ((VPIF_CHANNEL2_VIDEO == ch->channel_id)
	    || (VPIF_CHANNEL3_VIDEO == ch->channel_id)) {
		if (!fh->initialized) {
			zynq_printk(0, "[zynq_display]Channel Busy\n");
			return -EBUSY;
		}
	}

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != reqbuf->type  || !vpif_dev)
		return -EINVAL;

	index = VPIF_VIDEO_INDEX;

	common = &ch->common[index];

	if (0 != common->io_usrs)
		return -EBUSY;
#if  defined(USE_DMA_COUNTING)	
	/* Initialize videobuf2 queue as per the buffer type */
	common->alloc_ctx = vb2_dma_contig_init_ctx(vpif_dev);
	if (IS_ERR(common->alloc_ctx)) {
		zynq_printk(0, "[zynq_display]Failed to get the context\n");
		return PTR_ERR(common->alloc_ctx);
	}
#elif defined(USE_ZYNQ_MALLOC)
	{
		zynq_malloc_conf_t  ctx;
		ctx.is_always_get_first_memory = 1;
		ctx.pool_start_address = display_dam_framebuffer_addr;
		ctx.pool_size = display_dma_framebuffer_size;
			
		common->alloc_ctx = zynq_malloc_init_ctx(&ctx);
		if (IS_ERR(common->alloc_ctx)) {
				zynq_printk(0, "[zynq_display]Failed to get the context\n");
				return PTR_ERR(common->alloc_ctx);
		}
	}
#else
	common->alloc_ctx = NULL;
#endif	
	q = &common->buffer_queue;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = fh;
	q->ops = &video_qops;
#if defined(USE_DMA_COUNTING) 	
	q->mem_ops = &vb2_dma_contig_memops;
#elif defined(USE_ZYNQ_MALLOC)
	q->mem_ops = &zynq_malloc_memops;
#else
	q->mem_ops = &vb2_vmalloc_memops;
#endif	
	q->buf_struct_size = sizeof(struct vpif_disp_buffer);
#if (LINUX_VERSION_CODE <=  KERNEL_VERSION(3,14,0))	
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#else
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#endif

	ret = vb2_queue_init(q);
	if (ret) {
		zynq_printk(0, "[zynq_display]vpif_display: vb2_queue_init() failed\n");
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

static int vpif_querybuf(struct file *file, void *priv,
				struct v4l2_buffer *tbuf)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	if (common->fmt.type != tbuf->type)
		return -EINVAL;

	return vb2_querybuf(&common->buffer_queue, tbuf);
}

static int vpif_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct vpif_fh *fh = NULL;
	struct channel_obj *ch = NULL;
	struct common_obj *common = NULL;

	if (!buf || !priv)
		return -EINVAL;
	
	fh = (struct vpif_fh *)priv;
	if (!fh)
		return -EINVAL;

	ch = fh->channel;
	if (!ch)
		return -EINVAL;

	common = &(ch->common[VPIF_VIDEO_INDEX]);
	
	if (common->fmt.type != buf->type)
		return -EINVAL;

	if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
		return -EACCES;
	}
	
	return vb2_qbuf(&common->buffer_queue, buf);
}

static int vpif_s_std(struct file *file, void *priv, v4l2_std_id std_id)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	int ret = 0;

	if (common->started) {
		zynq_printk(0, "[zynq_display]streaming in progress\n");
		return -EBUSY;
	}

	/* Call encoder subdevice function to set the standard */
	ch->video.stdid = std_id;
	memset(&ch->video.dv_timings, 0, sizeof(ch->video.dv_timings));
	/* Get the information about the standard */
	if (vpif_update_resolution(ch))
		return -EINVAL;

	if ((ch->vpifparams.std_info.width *
		ch->vpifparams.std_info.height * 2) >
		config_params.channel_bufsize[ch->channel_id]) {
		zynq_printk(0, "[zynq_display]invalid std for this size\n");
		return -EINVAL;
	}

	common->fmt.fmt.pix.bytesperline = common->fmt.fmt.pix.width;
	/* Configure the default format information */
	vpif_config_format(ch);

	return ret;
}

static int vpif_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;

	*std = ch->video.stdid;
	return 0;
}



static int vpif_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	
	return vb2_dqbuf(&common->buffer_queue, p,
					(file->f_flags & O_NONBLOCK));
}

static int vpif_streamon(struct file *file, void *priv,
				enum v4l2_buf_type buftype)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	int ret = 0;
	
	if (buftype != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		zynq_printk(0, "[zynq_display]buffer type not supported\n");
		return -EINVAL;
	}

	if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
		zynq_printk(0, "[zynq_display]fh->io_allowed == 0\n");
		return -EACCES;
	}
	
	/* If Streaming is already started, return error */
	if (common->started) {
		zynq_printk(0, "[zynq_display]channel->started == 1\n");
		return -EBUSY;
	}
	
	/* Call vb2_streamon to start streaming in videobuf2 */
	ret = vb2_streamon(&common->buffer_queue, buftype);
	if (ret < 0) {
		zynq_printk(0, "[zynq_display]vb2_streamon\n");
		return ret;
	}
	return ret;
}

static int vpif_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type buftype)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];

	if (buftype != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		zynq_printk(0, "[zynq_display]buffer type not supported\n");
		return -EINVAL;
	}

	if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
		zynq_printk(0, "[zynq_display]fh->io_allowed == 0\n");
		return -EACCES;
	}

	if (!common->started) {
		zynq_printk(0, "[zynq_display]channel->started == 0\n");
		return -EINVAL;
	}

	common->started = 0;
	return vb2_streamoff(&common->buffer_queue, buftype);
}

static int vpif_cropcap(struct file *file, void *priv,
			struct v4l2_cropcap *crop)
{
	struct vpif_fh *fh = priv;
	struct channel_obj *ch = fh->channel;
	struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	if (V4L2_BUF_TYPE_VIDEO_OUTPUT != crop->type)
		return -EINVAL;

	crop->bounds.left = crop->bounds.top = 0;
	crop->defrect.left = crop->defrect.top = 0;
	crop->defrect.height = crop->bounds.height = common->height;
	crop->defrect.width = crop->bounds.width = common->width;

	return 0;
}

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

static int vpif_enum_output(struct file *file, void *fh,
				struct v4l2_output *output)
{
	return 0;
}

/**
 * vpif_set_output() - Select an output
 * @vpif_cfg - global config ptr
 * @ch - channel
 * @index - Given output index from application
 *
 * Select the given output.
 */
static int vpif_set_output(struct vpif_display_config *vpif_cfg,
		      struct channel_obj *ch, int index)
{

	return 0;
}

static int vpif_s_output(struct file *file, void *priv, unsigned int i)
{
	int ret = 0;
	
	return ret;
}

static int vpif_g_output(struct file *file, void *priv, unsigned int *i)
{

	return 0;
}

static int vpif_g_priority(struct file *file, void *priv, enum v4l2_priority *p)
{
	return 0;
}

static int vpif_s_priority(struct file *file, void *priv, enum v4l2_priority p)
{
	int ret = 0;
	return ret;
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
	int ret = 0;

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
	int ret = 0;

	return ret;
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
	int ret = 0;

	return ret;
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
	int ret = 0;

	return  ret;
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
	int ret = 0;

	return  ret;
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
	return 0;
}

/* vpif display ioctl operations */
static const struct v4l2_ioctl_ops vpif_ioctl_ops = {
	.vidioc_querycap        	= vpif_querycap,
	.vidioc_g_priority		= vpif_g_priority,
	.vidioc_s_priority		= vpif_s_priority,
	.vidioc_enum_fmt_vid_out	= vpif_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out  		= vpif_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out   	= vpif_s_fmt_vid_out,
	.vidioc_try_fmt_vid_out 	= vpif_try_fmt_vid_out,
	.vidioc_reqbufs         	= vpif_reqbufs,
	.vidioc_querybuf        	= vpif_querybuf,
	.vidioc_qbuf            	= vpif_qbuf,
	.vidioc_dqbuf           	= vpif_dqbuf,
	.vidioc_streamon        	= vpif_streamon,
	.vidioc_streamoff       	= vpif_streamoff,
	.vidioc_s_std           	= vpif_s_std,
	.vidioc_g_std			= vpif_g_std,
	.vidioc_enum_output		= vpif_enum_output,
	.vidioc_s_output		= vpif_s_output,
	.vidioc_g_output		= vpif_g_output,
	.vidioc_cropcap         	= vpif_cropcap,
	.vidioc_s_crop =   vpif_s_crop,
	.vidioc_enum_dv_timings         = vpif_enum_dv_timings,
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
};

static const struct v4l2_file_operations vpif_fops = {
	.owner		= THIS_MODULE,
	.open		= vpif_open,
	.release	= vpif_release,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vpif_mmap,
	.poll		= vpif_poll
};

static struct video_device vpif_video_template = {
	.name		= "vpif-display",
	.fops		= &vpif_fops,
	.ioctl_ops	= &vpif_ioctl_ops,
};

/*Configure the channels, buffer sizei, request irq */
static int initialize_vpif(void)
{
	int free_channel_objects_index = 0;
	int err = 0, i = 0, j = 0;

	/* Default number of buffers should be 3 */
	if ((ch2_numbuffers > 0) &&
	    (ch2_numbuffers < config_params.min_numbuffers))
		ch2_numbuffers = config_params.min_numbuffers;

	/* Set buffer size to min buffers size if invalid buffer size is
	 * given */
	if (ch2_bufsize < config_params.min_bufsize[VPIF_CHANNEL2_VIDEO])
		ch2_bufsize = config_params.min_bufsize[VPIF_CHANNEL2_VIDEO];

	config_params.numbuffers[VPIF_CHANNEL2_VIDEO] = ch2_numbuffers;

	if (ch2_numbuffers) {
		config_params.channel_bufsize[VPIF_CHANNEL2_VIDEO] = ch2_bufsize;
	}
	
	/* Allocate memory for six channel objects */
	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		vpif_obj.dev[i] =
		    kzalloc(sizeof(struct channel_obj), GFP_KERNEL);
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

/*
 * vpif_probe: This function creates device entries by register itself to the
 * V4L2 driver and initializes fields of each channel objects
 */
int vpif_display_init(struct pci_dev *pdev)
{
	struct vpif_display_config *config= NULL;
	int i, j = 0, k, err = 0;
	struct i2c_adapter *i2c_adap = NULL;
	struct common_obj *common = NULL;
	struct channel_obj *ch = NULL;
	struct video_device *vfd = NULL;
	int subdev_count = 0;
	int actual_subdev_count = 0;
	vpif_dev = &pdev->dev;
	err = initialize_vpif();

	if (err) {
		zynq_printk(0, "[zynq_display] Error initializing vpif\n");
		return err;
	}

	err = v4l2_device_register(vpif_dev, &vpif_obj.v4l2_dev);
	if (err) {
		zynq_printk(0, "[zynq_display]Error registering v4l2 device\n");
		return err;
	}
	
	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
		ch = vpif_obj.dev[i];

		/* Allocate memory for video device */
		vfd = video_device_alloc();
		if (vfd == NULL) {
			for (j = 0; j < i; j++) {
				ch = vpif_obj.dev[j];
				video_device_release(ch->video_dev);
			}
			err = -ENOMEM;
			goto vpif_int_err;
		}

		/* Initialize field of video device */
		*vfd = vpif_video_template;
		vfd->v4l2_dev = &vpif_obj.v4l2_dev;
		vfd->release = video_device_release;
		vfd->vfl_dir = VFL_DIR_TX;
		snprintf(vfd->name, sizeof(vfd->name),
			 "VPIF_Display_DRIVER_V%s",
			 VPIF_DISPLAY_VERSION);

		/* Set video_dev to the video device */
		ch->video_dev = vfd;
	}

	for (j = 0; j < VPIF_DISPLAY_MAX_DEVICES; j++) {
		ch = vpif_obj.dev[j];
		/* Initialize field of the channel objects */
		atomic_set(&ch->usrs, 0);
		for (k = 0; k < VPIF_NUMOBJECTS; k++) {
			ch->common[k].numbuffers = 0;
			common = &ch->common[k];
			common->io_usrs = 0;
			common->started = 0;
			spin_lock_init(&common->irqlock);
			mutex_init(&common->lock);
			common->numbuffers = 0;
			common->set_addr = NULL;
			common->ytop_off = common->ybtm_off = 0;
			common->ctop_off = common->cbtm_off = 0;
			common->cur_frm = common->next_frm = NULL;
			memset(&common->fmt, 0, sizeof(common->fmt));
			common->numbuffers = config_params.numbuffers[k];

		}
		ch->initialized = 0;
		
		ch->channel_id = j;
		ch->common[VPIF_VIDEO_INDEX].numbuffers = config_params.numbuffers[ch->channel_id];

		memset(&ch->vpifparams, 0, sizeof(ch->vpifparams));

		/* Initialize prio member of channel object */
		v4l2_prio_init(&ch->prio);
		ch->common[VPIF_VIDEO_INDEX].fmt.type =
						V4L2_BUF_TYPE_VIDEO_OUTPUT;
		ch->video_dev->lock = &common->lock;
		video_set_drvdata(ch->video_dev, ch);

		/* select output 0 */
		err = vpif_set_output(config, ch, 0);
		if (err)
			goto probe_out;

		/* register video device */
		err = video_register_device(ch->video_dev,
					  VFL_TYPE_GRABBER, (j ? 3 : 2));
		if (err < 0)
			goto probe_out;
		
		initialize_channel_pxiel_format(ch);
	}
	
#ifdef USE_ZYNQ_MALLOC
	if (allocate_reserved_memory() != 0) goto probe_out;
#endif
		
	//Initialize two ADV7511 subdevice.
	
	subdev_count = 	 MAX_DISPLAY_SUBDEV_NUM;
	vpif_obj.sd = kzalloc(sizeof(struct v4l2_subdev *) * subdev_count, GFP_KERNEL);
	if (vpif_obj.sd == NULL) {
		zynq_printk(0, "[zynq_display]Unable to allocate memory for subdevice(adv7511) pointers\n");
		err = -ENOMEM;
		goto probe_out;
	}
	
	for (i = 0; i < subdev_count; i++) {
		
		unsigned int bus_num = (unsigned int)-1;
		struct vpif_subdev_info *info = NULL;
		
		if (i == 0) {
			bus_num =  SUBDEV_CH3_I2C_BUS;
			info = &board_subdev_info[DISPLAY_SUBDEV_IDX_0];
		} else if (i == 1) {
			bus_num =  SUBDEV_CH4_I2C_BUS;
			info = &board_subdev_info[DISPLAY_SUBDEV_IDX_1];
		}
		
		if (info->enable == 0) continue;
		
		if (bus_num == (unsigned int)-1 )
		{
				zynq_printk(0, "[zynq_display] I2C bus number is %d\n", (int)bus_num);
				err = -EBUSY;
				goto rls_sd_obj;
		}
		
		i2c_adap = zynq_get_i2c_adapter_by_bus_num(bus_num);
		
		if (i2c_adap == NULL) 
		{
				zynq_printk(0, "[zynq_display]unable to get i2c bus %d obj\n", bus_num);
				err = -EBUSY;
				goto rls_sd_obj;
		}
		
		vpif_obj.sd[i] = v4l2_i2c_new_subdev_board(&vpif_obj.v4l2_dev, i2c_adap, &info->board_info, NULL);
		
	
		if (!vpif_obj.sd[i]) {
				zynq_printk(0, "[zynq_display]Error registering v4l2 subdevice %s\n",  info->name);
					err = -EBUSY;
				goto rls_sd_obj;
		}
		
		if ((err = v4l2_device_register_subdev_nodes(&vpif_obj.v4l2_dev)) != 0){
			zynq_printk(0, "[zynq_display]%s failed to call v4l2_device_register_subdev_nodes()!!\n", info->name);
			goto rls_sd_obj;
		} 
		
		actual_subdev_count++;
		
		v4l2_info(&vpif_obj.v4l2_dev, "[zynq_display]subdev : (name, bus, addr) - --> (%s, 0x%02x, 0x%02x)\n",  info->name, bus_num, info->board_info.addr);
	}
	
	v4l2_info(&vpif_obj.v4l2_dev, " VPIF display driver initialized\n");
	return 0;
	
rls_sd_obj:
	if (vpif_obj.sd != NULL) kfree(vpif_obj.sd);		
probe_out:
	for (k = 0; k < j; k++) {
		ch = vpif_obj.dev[k];
		video_unregister_device(ch->video_dev);
		ch->video_dev = NULL;
	}
vpif_int_err:
	v4l2_device_unregister(&vpif_obj.v4l2_dev);
	

	return err;
}

/*
 * vpif_remove: It un-register channels from V4L2 driver
 */
int vpif_display_release(struct pci_dev *pdev)
{
	struct channel_obj *ch;
	int i;

	v4l2_device_unregister(&vpif_obj.v4l2_dev);

	/* un-register device */
	for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
		ch = vpif_obj.dev[i];
		/* Unregister video device */
		video_unregister_device(ch->video_dev);

		ch->video_dev = NULL;
	}
#ifdef USE_ZYNQ_MALLOC
	release_reserved_memory();
#endif

	if (vpif_obj.sd != NULL) kfree(vpif_obj.sd);	
	return 0;
}

#ifdef CONFIG_PM_SLEEP
int vpif_display_suspend(void)
{
	zynq_printk(0,"[zynq_display]Enter vpif_display_suspend().\n");
	zynq_printk(0,"[zynq_display]Leave vpif_display_suspend().\n");
	return 0;
}
int vpif_display_resume(void)
{
	zynq_printk(0,"[zynq_display]Enter vpif_display_resume().\n");
	zynq_printk(0,"[zynq_display]Leave vpif_display_resume().\n");
	return  0;
}
#endif