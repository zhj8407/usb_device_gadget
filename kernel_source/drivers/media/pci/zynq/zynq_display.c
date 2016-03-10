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
#include <linux/gpio.h>

#include <asm/irq.h>

#include <media/adv7343.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

//#include <media/v4l2-chip-ident.h>

#include <media/videobuf2-dma-contig.h>

#include "zynq_debug.h"
#include "zynq_display_priv.h"
#include "zynq_core.h"
#include "zynq_board.h"
#include "zynq_gpio.h"

extern  unsigned int debug_print;

//NOTE: The value ' -1'  means that the interrupt gpio pin for FPGA DMA is not assigned. 
unsigned int zynq_fpga_dma_interrupt_gpio_pin0 = (unsigned int) -1;//GPIO_PU3;

static struct resource interrupt_resources [] = {
	    [0] = {
        	.name = "fpga_dma_interrupt0",
       	 	.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
    	}
};

static void set_interrupts(void) {
	if (zynq_fpga_dma_interrupt_gpio_pin0  != (unsigned int)-1) {
		interrupt_resources[0].start =  gpio_to_irq(zynq_fpga_dma_interrupt_gpio_pin0);
    	interrupt_resources[0].end =  gpio_to_irq(zynq_fpga_dma_interrupt_gpio_pin0);
	} else {
		interrupt_resources[0].start =  -1;
    	interrupt_resources[0].end = -1;
	}
	return;
}

static unsigned get_interrupt(unsigned int channel_id) {
	
	if (channel_id >= VPIF_DISPLAY_MAX_DEVICES) return -1;
	
	return (unsigned) interrupt_resources[channel_id].start;
}

static u32 ch0_numbuffers = 12;
static u32 ch0_bufsize = 1920 * 1080 * 2;
static u8 channel_first_int[VPIF_NUMOBJECTS][2] = { {1, 1} };

#define MAX_DISPLAY_SUBDEV_NUM 2
#define DISPLAY_SUBDEV_IDX_0 3
#define DISPLAY_SUBDEV_IDX_1 4


static struct vpif_config_params config_params = {
    .min_numbuffers		= 12,
    .numbuffers[0]		= 12,
    .min_bufsize[0]		= 1920 * 1080 * 2,
    .channel_bufsize[0]	= 1920 * 1080 * 2,
    .video_limit[0] = 1920* 1080 * 2 * 12,
    .channel_bufstride[0] = 1920,
    .pixelformat[0] = V4L2_PIX_FMT_YUV422P
};

static struct vpif_disp_device vpif_obj = {
    .dev ={NULL},
    .sd = NULL
};
static struct device *vpif_dev;

////////////////////////////////////////////////////////////////////
static void  ch0_set_dispbuf_addr (unsigned long addr) {
	void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base, FPGA_DMA_ADDR_REG, addr);
    return;
}
static void ch0_set_dispbuf_res (unsigned long width, unsigned long height) {
	void __iomem *base =   zynq_reg_base;
    fpga_reg_write(base,  FPGA_DMA_RES_REG, ((width << FPGA_DMA_RES_WIDTH_OFFSET) | height));
	return;
}

static void ch0_dispenable_read (unsigned int enable) {
	void __iomem *base =   zynq_reg_base;
    if (enable == 1)
        fpga_reg_write(base, FPGA_DMA_READ_ENABLE_REG, 1);
	return;
}
////////////////////////////////////////////////////////////////////

static void print_pixel_format( struct v4l2_pix_format *pixfmt)
{
    if (!pixfmt) return;

    zynq_printk(1, "[zynq_display]Video format: %c%c%c%c (%08x) %ux%u\n",
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
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
        common->fmt.fmt.pix.height = ((config_params.channel_bufsize[ch->channel_id ]*3) /2) /config_params.channel_bufstride[ch->channel_id ];
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
		common->fmt.fmt.pix.height = (config_params.channel_bufsize[ch->channel_id ]/2) /config_params.channel_bufstride[ch->channel_id ];
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY) 
		common->fmt.fmt.pix.height = (config_params.channel_bufsize[ch->channel_id ]/2) /config_params.channel_bufstride[ch->channel_id ];
	else if (common->fmt.fmt.pix.pixelformat ==  V4L2_PIX_FMT_VYUY)
		common->fmt.fmt.pix.height = (config_params.channel_bufsize[ch->channel_id ]/2) /config_params.channel_bufstride[ch->channel_id ];
	
    common->fmt.fmt.pix.field = V4L2_FIELD_NONE;
    common->fmt.fmt.pix.bytesperline = config_params.channel_bufstride[ch->channel_id ];
    common->fmt.fmt.pix.sizeimage = config_params.channel_bufsize[ch->channel_id ];
    common->fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE240M ;
    common->fmt.fmt.pix.priv = 0;

    //print_pixel_format(&common->fmt.fmt.pix);
    return ret;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
                                  struct vpif_disp_buffer, list);
    /* Remove that buffer from the buffer queue */
    list_del(&common->next_frm->list);
    spin_unlock(&common->irqlock);
	
    common->next_frm->vb.state = VB2_BUF_STATE_ACTIVE;
    addr = vb2_dma_contig_plane_dma_addr(&common->next_frm->vb, 0);

    if ((addr != 0) && (common->set_addr != NULL)) {
        common->set_addr(addr);
        if (common->enable_read != NULL) common->enable_read(1);
    }
	return;
}

static void set_to_dummy_buffer(struct common_obj *common)
{
    u32 addr = common->dummy_buffer_handle;
    if (common->set_addr != NULL) {
        common->set_addr(addr);
        if (common->enable_read != NULL) common->enable_read(1);
    }
}

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

    if (atomic_inc_and_test(&common->refcount) == 0) {
		zynq_printk(0, "[zynq_display] The isr does not execute normally!!\n");
	}
	
	if (0 == common->started) goto exit;
	
	spin_lock(&common->irqlock);
	
	if (debug_print >= 2) ch->interrupt_count++;
	
	if (list_empty(&common->dma_queue)) {
            spin_unlock(&common->irqlock);
            set_to_dummy_buffer(common);
			 if (debug_print >= 2) ch->interrupt_dummy_buffer_count++;
            goto exit;
	}
	
	spin_unlock(&common->irqlock);
	
	if (!channel_first_int[VPIF_VIDEO_INDEX][channel_id]) vpif_process_buffer_complete(common);
	
	channel_first_int[VPIF_VIDEO_INDEX][channel_id] = 0;
	
	vpif_schedule_next_buffer(common);
exit:
    atomic_dec(&common->refcount);
    return IRQ_HANDLED;
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
    zynq_printk(0, "[zynq_display]Buffer offset not aligned to 8 bytes !!\n");
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
    unsigned int size = 0;
	unsigned int req_buffers = *nbuffers;
	
    size = common->fmt.fmt.pix.sizeimage;
#if 0
    if (config_params.video_limit[ch->channel_id])
        while (size **nbuffers > config_params.video_limit[ch->channel_id])
            (*nbuffers)--;
#endif
		
    if (*nbuffers < config_params.min_numbuffers)
        *nbuffers = config_params.min_numbuffers;

    *nplanes = 1;
    sizes[0] = size;
	alloc_ctxs[0] = common->alloc_ctx;
	
	zynq_printk(1, "[zynq_display]real nbuffers = %u (req nbuffers = %u) (size for each buffer= %u bytes)\n", (*nbuffers), req_buffers, size);
    
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
	
	unsigned long addr = 0;
	struct v4l2_format *fmt_ptr = &common->fmt;
	  
    /* If buffer queue is empty, return error */
    spin_lock_irqsave(&common->irqlock, flags);
    if (list_empty(&common->dma_queue)) {
        spin_unlock_irqrestore(&common->irqlock, flags);
        zynq_printk(0, "[zynq_display]Buffer queue is empty!!\n");
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
	
	switch (ch->channel_id) {
            case VPIF_CHANNEL0_VIDEO:
                common->set_addr = ch0_set_dispbuf_addr;
                common->set_res = ch0_set_dispbuf_res;
                common->enable_read = ch0_dispenable_read;
                break;
            default:
                common->set_addr = NULL;
                common->set_res = NULL;
              common->enable_read= NULL;
		}

	
    channel_first_int[VPIF_VIDEO_INDEX][ch->channel_id] = 1;

	addr = vb2_dma_contig_plane_dma_addr(&common->cur_frm->vb, 0);
	
	
	if ((addr != 0) && (common->set_addr != NULL)) {
        common->set_addr(addr);
        if (common->set_res != NULL) {
            common->set_res(fmt_ptr->fmt.pix.width, fmt_ptr->fmt.pix.height);
        }
        if (common->enable_read != NULL) common->enable_read(1);
    }
	
    /* Initialize field_id and started member */
    ch->field_id = 0;
    common->started = 1;

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

    /* release all active buffers */
    spin_lock_irqsave(&common->irqlock, flags);
    while (!list_empty(&common->dma_queue)) {
        common->next_frm = list_entry(common->dma_queue.next,
                                      struct vpif_disp_buffer, list);
        list_del(&common->next_frm->list);
        vb2_buffer_done(&common->next_frm->vb, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&common->irqlock, flags);

	if (debug_print >= 2) {
		zynq_printk(2,  "[zynq_display]Interrupt count:  (intr, dummy) ---> (%u, %u) for channel %u\n", ch->interrupt_count, ch->interrupt_dummy_buffer_count, (unsigned int)ch->channel_id);
		ch->interrupt_count = 0;
		ch->interrupt_dummy_buffer_count = 0;
	}

	
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
            if (config->stdid & vid_ch->stdid) {
                memcpy(std_info, config, sizeof(*config));
                break;
            }
        }
    }

    if (i == vpif_ch_params_count) {
        zynq_printk(0, "[zynq_display] The STD format is not found!!\n");
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
    zynq_printk(1, "[zynq_display] Pixel details: Width = %d,Height = %d\n",
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
        zynq_printk(0, "[zynq_display]Unable to allocate memory for file handle object !!\n");
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
        //common->io_usrs = 0;

        common->started = 0;

		if (common->io_usrs != 0) {
			common->io_usrs = 0;
      	  /* Free buffers allocated */
        	vb2_queue_release(&common->buffer_queue);
			vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
		}	
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
	cap->capabilities = V4L2_CAP_STREAMING;
	cap->capabilities |= V4L2_CAP_VIDEO_OUTPUT;
	cap->device_caps = cap->capabilities;
	cap->capabilities |= V4L2_CAP_DEVICE_CAPS;
	
    snprintf(cap->driver, sizeof(cap->driver), "%s", dev_name(vpif_dev));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
             dev_name(vpif_dev));

    strlcpy(cap->card, VPIF_DISPLAY_CARD_NAME, sizeof(cap->card));
	
	memset(cap->reserved, 0, sizeof(cap->reserved));

    return 0;
}

static int vpif_enum_fmt_vid_out(struct file *file, void  *priv,
                                 struct v4l2_fmtdesc *fmt)
{
	 if (fmt->index > 3) {
        zynq_printk(0, "[zynq_display](%d)FPGA could support NV12, YUYV,  UYVY,   VYUY (index = %d)!!\n", __LINE__,fmt->index);
        return -EINVAL;
    }
    
	if (fmt->index == 0 ){
		/* Fill in the information about format */
    	strcpy(fmt->description, " YCbCr 4:2:0 (NV12)");
    	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    	fmt->pixelformat = V4L2_PIX_FMT_NV12;
    	zynq_printk(0, "[zynq_display](%d) V4L2_PIX_FMT_NV12 (index = %d)\n", __LINE__, fmt->index);
	} else if (fmt->index == 1) {
		/* Fill in the information about format */
    	strcpy(fmt->description, " YCbCr 4:2:2 (YUYV)");
    	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    	fmt->pixelformat = V4L2_PIX_FMT_YUYV;
    	zynq_printk(0, "[zynq_display](%d) V4L2_PIX_FMT_YUYV (index = %d)\n", __LINE__, fmt->index);
	} else if (fmt->index == 2) {
		/* Fill in the information about format */
    	strcpy(fmt->description, " YCbCr 4:2:2 (UYVY)");
    	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    	fmt->pixelformat = V4L2_PIX_FMT_UYVY;
    	zynq_printk(0, "[zynq_display](%d) V4L2_PIX_FMT_UYVY (index = %d)\n", __LINE__, fmt->index);
	} else if (fmt->index == 3) {
		/* Fill in the information about format */
    	strcpy(fmt->description, " YCbCr 4:2:2 ( VYUY )");
    	fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    	fmt->pixelformat = V4L2_PIX_FMT_VYUY;
    	zynq_printk(0, "[zynq_display](%d) V4L2_PIX_FMT_VYUY (index = %d)\n", __LINE__, fmt->index);
	} 
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

    if (VPIF_CHANNEL0_VIDEO == ch->channel_id) {
        if (!fh->initialized) {
            zynq_printk(0,  "[zynq_display]Channel Busy!!\n");
            return -EBUSY;
        }

        /* Check for the priority */
        ret = v4l2_prio_check(&ch->prio, fh->prio);
        if (0 != ret)
            return ret;
        fh->initialized = 1;
    }

    if (common->started) {
        zynq_printk(0, "[zynq_display]Streaming in progress!!\n");
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
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12) 
        common->fmt.fmt.pix.sizeimage = (common->fmt.fmt.pix.width * common->fmt.fmt.pix.height * 3 )/2;
    else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
        common->fmt.fmt.pix.sizeimage = common->fmt.fmt.pix.width * common->fmt.fmt.pix.height *2;
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
        common->fmt.fmt.pix.sizeimage = common->fmt.fmt.pix.width * common->fmt.fmt.pix.height *2;
	else if (common->fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_VYUY)
        common->fmt.fmt.pix.sizeimage = common->fmt.fmt.pix.width * common->fmt.fmt.pix.height *2;
	
    common->fmt.fmt.pix.bytesperline = common->fmt.fmt.pix.width;
	common->height = pixfmt->height;
    common->width = pixfmt->width;
	
    print_pixel_format( pixfmt);

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
    int ret = 0;
	
    /* This file handle has not initialized the channel,
       It is not allowed to do settings */
    if (VPIF_CHANNEL0_VIDEO == ch->channel_id) {
        if (!fh->initialized) {
            zynq_printk(0, "[zynq_display]Channel Busy\n");
            return -EBUSY;
        }
    }

    if (V4L2_BUF_TYPE_VIDEO_OUTPUT != reqbuf->type  || !vpif_dev)
        return -EINVAL;

    index = VPIF_VIDEO_INDEX;

    common = &ch->common[index];

	if ((common->io_usrs != 0) && (reqbuf->count == 0)) {
			common->io_usrs = 0;
        /* Free buffers allocated */
        vb2_queue_release(&common->buffer_queue);
        vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
		zynq_printk(0, "[zynq_display][vpif_reqbufs] (%d)Call vpif_reqbufs() to free buffer may be successfull!! (reqbuf->count = %d)\n", __LINE__, reqbuf->count);
		goto exit;
		 
	}
	
	if (0 != common->io_usrs) {
     	common->io_usrs = 0;
        /* Free buffers allocated */
        vb2_queue_release(&common->buffer_queue);
        vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
		 zynq_printk(0, "[zynq_display][vpif_reqbufs] (%d)Call vpif_reqbufs() to free buffer may be successfull!! (reqbuf->count = %d)\n", __LINE__, reqbuf->count);
		
		//zynq_printk(0, "[zynq_display][vpif_reqbufs] (%d)Call vpif_reqbufs() failed!! (common->io_usrs = %d)\n", __LINE__, common->io_usrs);
		//return -EBUSY;
	}

	common->alloc_ctx = vb2_dma_contig_init_ctx(vpif_dev);
    if (IS_ERR(common->alloc_ctx)) {
        	zynq_printk(0, "[zynq_display]Failed to get the context\n");
        	return PTR_ERR(common->alloc_ctx);
    }
    q = &common->buffer_queue;
    q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    q->io_modes = VB2_MMAP | VB2_USERPTR;
    q->drv_priv = fh;
    q->ops = &video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	
    q->buf_struct_size = sizeof(struct vpif_disp_buffer);
#if (LINUX_VERSION_CODE <=  KERNEL_VERSION(3,14,0))
    q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#else
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
#endif

    ret = vb2_queue_init(q);
    if (ret) {
        zynq_printk(0, "[zynq_display]vpif_display: vb2_queue_init() failed\n");
		vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
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
	ret = vb2_reqbufs(&common->buffer_queue, reqbuf);
	if (ret) {
		zynq_printk(0, "[zynq_display][vpif_reqbufs] (%d)Call vpif_reqbufs() failed!!(ret = %d)\n", __LINE__, ret);
	}else {
	  zynq_printk(0, "[zynq_display][vpif_reqbufs] (%d)Call vpif_reqbufs() to allocate buffer may be successfull!!\n", __LINE__);
	}
	
  exit: 
    return ret;
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
	int ret = -1;
	
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
    
    ret=  vb2_qbuf(&common->buffer_queue, buf);
	
	if (ret != 0)
		zynq_printk(0, "[zynq_display](%d) Call vb2_qbuf() failed (%d)!!!\n", __LINE__, ret);
	
	return ret;
}

static int vpif_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct vpif_fh *fh = priv;
    struct channel_obj *ch = fh->channel;
    struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
	int ret = -1;

    ret = vb2_dqbuf(&common->buffer_queue, p, (file->f_flags & O_NONBLOCK));
	
	if (ret != 0)
		zynq_printk(0, "[zynq_display](%d) Call vb2_dqbuf() failed (%d)!!!\n", __LINE__, ret);
	
	return ret;
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

static int vpif_streamon(struct file *file, void *priv,
                         enum v4l2_buf_type buftype)
{
    struct vpif_fh *fh = priv;
    struct channel_obj *ch = fh->channel;
    struct common_obj *common = &ch->common[VPIF_VIDEO_INDEX];
    int ret = 0;
	
    if (buftype != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
        zynq_printk(0, "[zynq_display]Buffer type %u is not supported !!\n", buftype);
        return -EINVAL;
    }

    if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
        zynq_printk(0, "[zynq_display]File handle is not allowed IO !!\n");
        return -EACCES;
    }
    
    /* If Streaming is already started, return error */
    if (common->started) {
        zynq_printk(0, "[zynq_display]Streaming is started !!\n");
        return -EBUSY;
    }
    
    /* Call vb2_streamon to start streaming in videobuf2 */
    ret = vb2_streamon(&common->buffer_queue, buftype);
    if (ret < 0) {
        zynq_printk(0, "[zynq_display]Call vb2_streamon() failed!!\n");
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
    int ret = -1;

    if (buftype != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
        zynq_printk(0, "[zynq_display]Buffer type %u is not supported !!\n", buftype);
        return -EINVAL;
    }

    if (!fh->io_allowed[VPIF_VIDEO_INDEX]) {
        zynq_printk(0, "[zynq_display]File handle is not allowed IO !!\\n");
        return -EACCES;
    }

    if (!common->started) {
        zynq_printk(0, "[zynq_display]Streaming is not started !!\n");
        return -EINVAL;
    }

    common->started = 0;

    ret = vb2_streamoff(&common->buffer_queue, buftype);
    if (ret < 0) {
        zynq_printk(0, "[zynq_display]Call vb2_streamoff() failed!!\n");
        return ret;
    }
    return ret;
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

static int vpif_s_crop(struct file *file, void *priv, const struct v4l2_crop *crop)
{
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
                               struct v4l2_dbg_register *reg)
{
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

/////////////////////////////////////////////////////////////////////
/* returns set of device inputs, in our case there is only one,
 * but later I may add more
 * called on VIDIOC_ENUMINPUT
 */
static int vpif_enum_input(struct file *file, void *fh, struct v4l2_input *inp)
{
		
		__u32 index = inp->index;

	if (0 != index)
		return -EINVAL;
	
	/* clear all data (including the reserved fields) */
	memset(inp, 0, sizeof(*inp));

	inp->index = index;
	strlcpy(inp->name, "loopback", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->audioset = 0;
	inp->tuner = 0;
	inp->status = 0;

#ifdef V4L2LOOPBACK_WITH_STD
	inp->std = V4L2_STD_ALL;
# ifdef V4L2_IN_CAP_STD
	inp->capabilities |= V4L2_IN_CAP_STD;
# endif
#endif /* V4L2LOOPBACK_WITH_STD */

	return 0;

}

/* which input is currently active,
 * called on VIDIOC_G_INPUT
 */
static int vpif_g_input(struct file *file, void *fh, unsigned int *i)
{

		return -ENOTTY;

}

/* set input, can make sense if we have more than one video src,
 * called on VIDIOC_S_INPUT
 */
static int vpif_s_input(struct file *file, void *fh, unsigned int i)
{

		return -ENOTTY;

}

static int vpif_queryctrl(struct file *file, void *priv, struct v4l2_queryctrl *ctrl)
{
    int ret = 0;
    struct vpif_fh *fh = priv;
    struct channel_obj *ch = fh->channel;
   // zynq_printk(1, "Call vpif_queryctrl.\n");

    /* we only support hue/saturation/contrast/brightness */
    if (ctrl->id < V4L2_CID_BRIGHTNESS || ctrl->id > V4L2_CID_HUE)
        return -EINVAL;
	
	if (ch->sd != NULL) ret = v4l2_subdev_call(ch->sd, core,queryctrl, ctrl);
    
	return ret;
}


///////////////////////////////////////////////////////////////////////

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
	
	.vidioc_enum_input       = &vpif_enum_input,
	.vidioc_g_input          = &vpif_g_input,
	.vidioc_s_input          = &vpif_s_input,
	
	    .vidioc_queryctrl                   = vpif_queryctrl,
	
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
    if ((ch0_numbuffers > 0) &&
        (ch0_numbuffers < config_params.min_numbuffers))
        ch0_numbuffers = config_params.min_numbuffers;

    /* Set buffer size to min buffers size if invalid buffer size is
     * given */
    if (ch0_bufsize < config_params.min_bufsize[VPIF_CHANNEL0_VIDEO])
        ch0_bufsize = config_params.min_bufsize[VPIF_CHANNEL0_VIDEO];

    config_params.numbuffers[VPIF_CHANNEL0_VIDEO] = ch0_numbuffers;

    if (ch0_numbuffers) {
        config_params.channel_bufsize[VPIF_CHANNEL0_VIDEO] = ch0_bufsize;
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
	unsigned irq = (unsigned)-1;
	
    vpif_dev = &pdev->dev;
    err = initialize_vpif();
    if (err) {
        zynq_printk(0, "[zynq_display] Failed to call  initialize_vpif()!!\n");
        return err;
    }

    err = v4l2_device_register(vpif_dev, &vpif_obj.v4l2_dev);
    if (err) {
        zynq_printk(0, "[zynq_display]Error registering v4l2 device!!\n");
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
        vfd->vfl_dir =  VFL_DIR_TX  | ~VFL_DIR_RX;
		snprintf(vfd->name, sizeof(vfd->name),
                 "VPIF_Display_DRIVER_V%s",
                 VPIF_DISPLAY_VERSION);
		if (debug_print >= 4)
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 20, 0)
			vfd->debug = V4L2_DEBUG_IOCTL | V4L2_DEBUG_IOCTL_ARG;
#else
			vfd->dev_debug = V4L2_DEV_DEBUG_IOCTL | V4L2_DEV_DEBUG_IOCTL_ARG;
#endif
		vfd->tvnorms =   V4L2_STD_ALL; 	
			
        /* Set video_dev to the video device */
        ch->video_dev = vfd;
    }
    
    set_interrupts();

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
			 atomic_set(&common->refcount, -1);
			 common->dummy_buffer_size = 1920 * 1080 * 2;
        	common->dummy_buffer = dma_alloc_coherent(NULL, 	common->dummy_buffer_size , &common->dummy_buffer_handle, GFP_KERNEL);
        }
        ch->initialized = 0;

        ch->channel_id = j;
		ch->interrupt_count = 0;
		ch->interrupt_dummy_buffer_count = 0;
		
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
		
		irq = get_interrupt(ch->channel_id);
		
		if (irq == (unsigned) -1) continue;
        
		err = request_irq(irq,
                          vpif_channel_isr,
                          IRQF_ONESHOT   | IRQF_TRIGGER_HIGH,//IRQF_TRIGGER_HIGH, //IRQF_TRIGGER_RISING,
                          "zynq_display",  &ch->channel_id);
        if (err) {
            zynq_printk(0, "[zynq_display] Call request_irq() for channel %d failed!!\n", ch->channel_id);
            err = -EBUSY;
            for (j = 0; j < i; j++) {
                irq = get_interrupt(j);
                free_irq(irq, (void *)(&ch->channel_id));
            }
            goto probe_out;
        }
       
    }
	

    subdev_count = 	 MAX_DISPLAY_SUBDEV_NUM;
    vpif_obj.sd = kzalloc(sizeof(struct v4l2_subdev *) * subdev_count, GFP_KERNEL);
    if (vpif_obj.sd == NULL) {
        zynq_printk(0, "[zynq_display]Unable to allocate memory for subdevice(adv7511) pointers\n");
        err = -ENOMEM;
       // goto probe_out;
		goto rls_irq;
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

        if (bus_num == (unsigned int)-1 ) {
            zynq_printk(0, "[zynq_display] I2C bus number %d is invalid !!\n", (int)bus_num);
            err = -EBUSY;
            goto rls_sd_obj;
        }

        i2c_adap = zynq_get_i2c_adapter_by_bus_num(bus_num);

        if (i2c_adap == NULL) {
            zynq_printk(0, "[zynq_display] The i2c_adap handle for %d is NULL !!\n", bus_num);
            err = -EBUSY;
            goto rls_sd_obj;
        }

        vpif_obj.sd[i] = v4l2_i2c_new_subdev_board(&vpif_obj.v4l2_dev, i2c_adap, &info->board_info, NULL);


        if (!vpif_obj.sd[i]) {
            zynq_printk(0, "[zynq_display]Error registering v4l2 subdevice %s\n",  info->name);
            err = -EBUSY;
            goto rls_sd_obj;
        }

        if ((err = v4l2_device_register_subdev_nodes(&vpif_obj.v4l2_dev)) != 0) {
            zynq_printk(0, "[zynq_display]%s failed to call v4l2_device_register_subdev_nodes()!!\n", info->name);
            goto rls_sd_obj;
        }

        actual_subdev_count++;

        //zynq_printk(1, "[zynq_display]subdev : (name, bus, addr) - --> (%s, 0x%02x, 0x%02x)\n",  info->name, bus_num, info->board_info.addr);
    }
	
	return 0;

rls_sd_obj:
    if (vpif_obj.sd != NULL) kfree(vpif_obj.sd);
rls_irq:
    for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
        ch = vpif_obj.dev[i];
        if (ch != NULL) {
            unsigned irq = get_interrupt(i);
            if (irq == (unsigned) -1) continue;
            free_irq(irq, (void *)(&ch->channel_id));
        }
    }	
probe_out:
    for (k = 0; k < j; k++) {
        ch = vpif_obj.dev[k];
		if (ch->video_dev != NULL) {
        	video_unregister_device(ch->video_dev);
        	ch->video_dev = NULL;
		}
    }
vpif_int_err:
    v4l2_device_unregister(&vpif_obj.v4l2_dev);

    zynq_printk(0, "[zynq_display]Failed to intialize display function !!\n");

    return err;
}

/*
 * vpif_remove: It un-register channels from V4L2 driver
 */
int vpif_display_release(struct pci_dev *pdev)
{
    struct channel_obj *ch = NULL;
	 struct common_obj *common = NULL;
    int i = 0;
	unsigned irq = (unsigned)-1;

    v4l2_device_unregister(&vpif_obj.v4l2_dev);

    /* un-register device */
    for (i = 0; i < VPIF_DISPLAY_MAX_DEVICES; i++) {
		/* Get the pointer to the channel object */
        ch = vpif_obj.dev[i];
		
		if (!ch) continue;
		
		irq = get_interrupt(ch->channel_id);
		
		if (irq != (unsigned) -1) free_irq(irq, (void *)(&ch->channel_id));
		
		common = &(ch->common[VPIF_VIDEO_INDEX]);
		if (common->dummy_buffer  != NULL)
                dma_free_coherent(NULL, common->dummy_buffer_size, common->dummy_buffer, common->dummy_buffer_handle);
        /* Unregister video device */
        video_unregister_device(ch->video_dev);

        ch->video_dev = NULL;
    }

    if (vpif_obj.sd != NULL) kfree(vpif_obj.sd);

    zynq_printk(1, "[zynq_display]The display function is  released!!\n");
    return 0;
}

#ifdef CONFIG_PM_SLEEP
int vpif_display_suspend(void)
{
    return 0;
}
int vpif_display_resume(void)
{
    return  0;
}
#endif