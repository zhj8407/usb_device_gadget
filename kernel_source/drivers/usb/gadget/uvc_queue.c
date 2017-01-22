/*
 *	uvc_queue.c  --  USB Video Class driver - Buffers management
 *
 *	Copyright (C) 2005-2010
 *	    Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */

#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>

#include "uvc.h"
#include "uvc_kmalloc.h"

#define MY_DATA_SIZE (4*1024*1024)

static int release_reserved_memory(struct uvc_video_queue *uvc_queue)
{
	int status = 0;
	int i = 0;

	for (i = 0 ; i < uvc_queue->output_buff_nums; i++) {
		if (uvc_queue->output_buff_addrs[i]  != NULL) {
			kfree(uvc_queue->output_buff_addrs[i]);
			uvc_queue->output_buff_addrs[i] = NULL;
		}
	}

	return status;
}

static int allocate_reserved_memory(struct uvc_video_queue *uvc_queue,
					unsigned int buffer_num)
{
	int status = 0;
	int i, j, num_of_pages;
	size_t output_buff_size = uvc_queue->output_buff_size;
	unsigned char *tmp;

	for (i  = 0; i < buffer_num; i++) {
		uvc_queue->output_buff_addrs[i] = kmalloc(output_buff_size, GFP_KERNEL);
		if (!uvc_queue->output_buff_addrs[i]) {
			printk(KERN_ERR "[uvc_queue]failed to kmalloc  %d memory for (index:%d) !!\n",
				output_buff_size, i);
			status = -1;
			goto exit;
		}

		/* Set the Page Flag to be reserved. */
		num_of_pages = (output_buff_size + PAGE_SIZE - 1 ) / PAGE_SIZE;
		tmp = (unsigned char *)(uvc_queue->output_buff_addrs[i]);

		for (j = 0; j < num_of_pages; j++) {
			SetPageReserved(virt_to_page(tmp));
			tmp += PAGE_SIZE;
		}

		printk(KERN_ERR "[uvc_queue]Successfull to allocate %d bytes video memory: (index:%d)"
			" --->  (virt:0x%p, phy:0x%p) !!\n",
			output_buff_size, i,
			uvc_queue->output_buff_addrs[i],
			(void *)virt_to_phys(uvc_queue->output_buff_addrs[i]));
	}
	uvc_queue->output_buff_nums = buffer_num;
	return status;
exit:
	for (i = 0 ; i < uvc_queue->output_buff_nums; i++) {
		if (uvc_queue->output_buff_addrs[i]  != NULL) {
			kfree(uvc_queue->output_buff_addrs[i]);
			uvc_queue->output_buff_addrs[i] = NULL;
		}
	}
	return status;
}

static int uvc_queue_buffer_init(struct uvc_video_queue *uvc_queue,
					unsigned int buffer_num)
{
	uvc_kmalloc_conf_t ctx;

	if (release_reserved_memory(uvc_queue))
		return -EINVAL;

	if (allocate_reserved_memory(uvc_queue, buffer_num))
		return -EINVAL;

	ctx.en_non_cache_map = 0;
	ctx.is_always_get_first_memory = 0;
	ctx.buffer_virt_addr_list = &uvc_queue->output_buff_addrs[0];
	ctx.buffer_num = uvc_queue->output_buff_nums;
	ctx.available_buffer_size = uvc_queue->output_buff_size;

	if (uvc_queue->alloc_ctx) {
		uvc_kmalloc_cleanup_ctx(uvc_queue->alloc_ctx);
		uvc_queue->alloc_ctx = NULL;
	}

	uvc_queue->alloc_ctx = uvc_kmalloc_init_ctx(&ctx);
	if (IS_ERR(uvc_queue->alloc_ctx)) {
		printk(KERN_ERR "[uvc_queue][uvc_queue_setup] (%d)"
			"Failed to call the uvc_kmalloc_init_ctx\n",
			__LINE__);
		return -1;
	}

	uvc_queue->static_memory_allocated = 1;

	return 0;
}

/*
 * Send video frame transfer done event through v4l2
 */
static void uvc_buffer_done_notify(struct uvc_video_queue *queue,
						struct uvc_buffer *buf,
						int buffer_status)
{
	struct uvc_video *video = container_of(queue, struct uvc_video, queue);
	struct uvc_device *uvc = container_of(video, struct uvc_device, video);
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_FRAMEDONE;

	uvc_event->frame_done.buffer_index =
		buf->buf.v4l2_buf.index;
	uvc_event->frame_done.bytes_transferred =
		buf->bytesused;
	uvc_event->frame_done.status = buffer_status;

	v4l2_event_queue(uvc->vdev, &v4l2_event);
}

/* ------------------------------------------------------------------------
 * Video buffers queue management.
 *
 * Video queues is initialized by uvc_queue_init(). The function performs
 * basic initialization of the uvc_video_queue struct and never fails.
 *
 * Video buffers are managed by videobuf2. The driver uses a mutex to protect
 * the videobuf2 queue operations by serializing calls to videobuf2 and a
 * spinlock to protect the IRQ queue that holds the buffers to be processed by
 * the driver.
 */

/* -----------------------------------------------------------------------------
 * videobuf2 queue operations
 */

static int uvc_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], void *alloc_ctxs[])
{
	struct uvc_video_queue *queue = vb2_get_drv_priv(vq);
	struct uvc_video *video = container_of(queue, struct uvc_video, queue);

	if (*nbuffers > UVC_MAX_VIDEO_BUFFERS)
		*nbuffers = UVC_MAX_VIDEO_BUFFERS;

	if (queue->vb2_kmalloc_flag &&
		(!queue->static_memory_allocated ||
		*nbuffers != queue->output_buff_nums)) {
		printk(KERN_ERR "[uvc_queue][uvc_queue_setup]real nbuffers = %u\n", *nbuffers);
		/* We need to re-allocate the buffers. */
		if (uvc_queue_buffer_init(queue, *nbuffers))
			return -1;
	}

	*nplanes = 1;

	sizes[0] = video->imagesize;

	alloc_ctxs[0] = queue->alloc_ctx;

	return 0;
}

static int uvc_buffer_prepare(struct vb2_buffer *vb)
{
	struct uvc_video_queue *queue = vb2_get_drv_priv(vb->vb2_queue);
	struct uvc_buffer *buf = container_of(vb, struct uvc_buffer, buf);

	if (vb->v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		uvc_trace(UVC_TRACE_CAPTURE, "[E] Bytes used out of bounds.\n");
		return -EINVAL;
	}

	if (unlikely(queue->flags & UVC_QUEUE_DISCONNECTED))
		return -ENODEV;

	buf->state = UVC_BUF_STATE_QUEUED;
	buf->mem = vb2_plane_vaddr(vb, 0);
	buf->length = vb2_plane_size(vb, 0);
	if (vb->v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		buf->bytesused = 0;
	else
		buf->bytesused = vb2_get_plane_payload(vb, 0);

	return 0;
}

static void uvc_buffer_queue(struct vb2_buffer *vb)
{
	struct uvc_video_queue *queue = vb2_get_drv_priv(vb->vb2_queue);
	struct uvc_buffer *buf = container_of(vb, struct uvc_buffer, buf);
	unsigned long flags;
	int notify = 0;

	spin_lock_irqsave(&queue->irqlock, flags);

	if (likely(!(queue->flags & UVC_QUEUE_DISCONNECTED))) {
		list_add_tail(&buf->queue, &queue->irqqueue);
	} else {
		/* If the device is disconnected return the buffer to userspace
		 * directly. The next QBUF call will fail with -ENODEV.
		 */
		buf->state = UVC_BUF_STATE_ERROR;
		vb2_buffer_done(&buf->buf, VB2_BUF_STATE_ERROR);
		notify = 1;
	}

	spin_unlock_irqrestore(&queue->irqlock, flags);

	if (notify)
		uvc_buffer_done_notify(queue, buf, buf->state);
}

static struct vb2_ops uvc_queue_qops = {
	.queue_setup = uvc_queue_setup,
	.buf_prepare = uvc_buffer_prepare,
	.buf_queue = uvc_buffer_queue,
};

static int uvc_queue_init(struct uvc_video_queue *queue,
			  enum v4l2_buf_type type,
			  unsigned int vb2kmalloc)
{
	int ret;

	/* Initialize the output buffers. */
	queue->vb2_kmalloc_flag = vb2kmalloc;

	if (queue->vb2_kmalloc_flag) {
		queue->output_buff_size = PAGE_ALIGN(MY_DATA_SIZE);
		queue->output_buff_nums = UVC_MAX_VIDEO_BUFFERS;
		ret = uvc_queue_buffer_init(queue, queue->output_buff_nums);
		if (ret)
			return ret;
	}

	queue->queue.type = type;
	queue->queue.io_modes = VB2_MMAP | VB2_USERPTR;
	queue->queue.drv_priv = queue;
	queue->queue.buf_struct_size = sizeof(struct uvc_buffer);
	queue->queue.ops = &uvc_queue_qops;
	if (queue->vb2_kmalloc_flag)
		queue->queue.mem_ops = &uvc_kmalloc_memops;
	else
		queue->queue.mem_ops = &vb2_vmalloc_memops;
	queue->queue.timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ret = vb2_queue_init(&queue->queue);
	if (ret)
		return ret;

	mutex_init(&queue->mutex);
	spin_lock_init(&queue->irqlock);
	INIT_LIST_HEAD(&queue->irqqueue);
	queue->flags = 0;

	return 0;
}

/*
 * Free the video buffers.
 */
static void uvc_free_buffers(struct uvc_video_queue *queue)
{
	mutex_lock(&queue->mutex);
	vb2_queue_release(&queue->queue);
	mutex_unlock(&queue->mutex);
}

/*
 * Allocate the video buffers.
 */
static int uvc_alloc_buffers(struct uvc_video_queue *queue,
			     struct v4l2_requestbuffers *rb)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_reqbufs(&queue->queue, rb);
	mutex_unlock(&queue->mutex);

	return ret ? ret : rb->count;
}

static int uvc_query_buffer(struct uvc_video_queue *queue,
			    struct v4l2_buffer *buf)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_querybuf(&queue->queue, buf);
	mutex_unlock(&queue->mutex);

	return ret;
}

static int uvc_queue_buffer(struct uvc_video_queue *queue,
			    struct v4l2_buffer *buf)
{
	unsigned long flags;
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_qbuf(&queue->queue, buf);
	if (ret < 0)
		goto done;

	spin_lock_irqsave(&queue->irqlock, flags);
	ret = (queue->flags & UVC_QUEUE_PAUSED) != 0;
	queue->flags &= ~UVC_QUEUE_PAUSED;
	spin_unlock_irqrestore(&queue->irqlock, flags);

done:
	mutex_unlock(&queue->mutex);
	return ret;
}

/*
 * Dequeue a video buffer. If nonblocking is false, block until a buffer is
 * available.
 */
static int uvc_dequeue_buffer(struct uvc_video_queue *queue,
			      struct v4l2_buffer *buf, int nonblocking)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_dqbuf(&queue->queue, buf, nonblocking);
	mutex_unlock(&queue->mutex);

	return ret;
}

/*
 * Poll the video queue.
 *
 * This function implements video queue polling and is intended to be used by
 * the device poll handler.
 */
static unsigned int uvc_queue_poll(struct uvc_video_queue *queue,
				   struct file *file, poll_table *wait)
{
	unsigned int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_poll(&queue->queue, file, wait);
	mutex_unlock(&queue->mutex);

	return ret;
}

static int uvc_queue_mmap(struct uvc_video_queue *queue,
			  struct vm_area_struct *vma)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_mmap(&queue->queue, vma);
	mutex_unlock(&queue->mutex);

	return ret;
}

#ifndef CONFIG_MMU
/*
 * Get unmapped area.
 *
 * NO-MMU arch need this function to make mmap() work correctly.
 */
static unsigned long uvc_queue_get_unmapped_area(struct uvc_video_queue *queue,
		unsigned long pgoff)
{
	unsigned long ret;

	mutex_lock(&queue->mutex);
	ret = vb2_get_unmapped_area(&queue->queue, 0, 0, pgoff, 0);
	mutex_unlock(&queue->mutex);
	return ret;
}
#endif

/*
 * Cancel the video buffers queue.
 *
 * Cancelling the queue marks all buffers on the irq queue as erroneous,
 * wakes them up and removes them from the queue.
 *
 * If the disconnect parameter is set, further calls to uvc_queue_buffer will
 * fail with -ENODEV.
 *
 * This function acquires the irq spinlock and can be called from interrupt
 * context.
 */
static void uvc_queue_cancel(struct uvc_video_queue *queue, int disconnect)
{
	struct uvc_buffer *buf;
	unsigned long flags;
	int notify = 0;

	spin_lock_irqsave(&queue->irqlock, flags);
	while (!list_empty(&queue->irqqueue)) {
		buf = list_first_entry(&queue->irqqueue, struct uvc_buffer,
				       queue);
		list_del(&buf->queue);
		buf->state = UVC_BUF_STATE_ERROR;
		vb2_buffer_done(&buf->buf, VB2_BUF_STATE_ERROR);
		notify = 1;
	}
	/* This must be protected by the irqlock spinlock to avoid race
	 * conditions between uvc_queue_buffer and the disconnection event that
	 * could result in an interruptible wait in uvc_dequeue_buffer. Do not
	 * blindly replace this logic by checking for the UVC_DEV_DISCONNECTED
	 * state outside the queue code.
	 */
	if (disconnect)
		queue->flags |= UVC_QUEUE_DISCONNECTED;
	spin_unlock_irqrestore(&queue->irqlock, flags);

	if (notify)
		uvc_buffer_done_notify(queue, buf, buf->state);
}

/*
 * Enable or disable the video buffers queue.
 *
 * The queue must be enabled before starting video acquisition and must be
 * disabled after stopping it. This ensures that the video buffers queue
 * state can be properly initialized before buffers are accessed from the
 * interrupt handler.
 *
 * Enabling the video queue initializes parameters (such as sequence number,
 * sync pattern, ...). If the queue is already enabled, return -EBUSY.
 *
 * Disabling the video queue cancels the queue and removes all buffers from
 * the main queue.
 *
 * This function can't be called from interrupt context. Use
 * uvc_queue_cancel() instead.
 */
static int uvc_queue_enable(struct uvc_video_queue *queue, int enable)
{
	unsigned long flags;
	int ret = 0;

	mutex_lock(&queue->mutex);
	if (enable) {
		ret = vb2_streamon(&queue->queue, queue->queue.type);
		if (ret < 0)
			goto done;

		queue->sequence = 0;
		queue->buf_used = 0;
	} else {
		ret = vb2_streamoff(&queue->queue, queue->queue.type);
		if (ret < 0)
			goto done;

		spin_lock_irqsave(&queue->irqlock, flags);
		INIT_LIST_HEAD(&queue->irqqueue);

		/*
		 * FIXME: We need to clear the DISCONNECTED flag to ensure that
		 * applications will be able to queue buffers for the next
		 * streaming run. However, clearing it here doesn't guarantee
		 * that the device will be reconnected in the meantime.
		 */
		queue->flags &= ~UVC_QUEUE_DISCONNECTED;
		spin_unlock_irqrestore(&queue->irqlock, flags);
	}

done:
	mutex_unlock(&queue->mutex);
	return ret;
}

/* called with &queue_irqlock held.. */
static struct uvc_buffer *uvc_queue_next_buffer(struct uvc_video_queue *queue,
						struct uvc_buffer *buf)
{
	struct uvc_buffer *nextbuf;

	if ((queue->flags & UVC_QUEUE_DROP_INCOMPLETE) &&
	     buf->length != buf->bytesused) {
		buf->state = UVC_BUF_STATE_QUEUED;
		vb2_set_plane_payload(&buf->buf, 0, 0);
		return buf;
	}

	list_del(&buf->queue);
	if (!list_empty(&queue->irqqueue))
		nextbuf = list_first_entry(&queue->irqqueue, struct uvc_buffer,
					   queue);
	else
		nextbuf = NULL;

	/*
	 * FIXME: with videobuf2, the sequence number or timestamp fields
	 * are valid only for video capture devices and the UVC gadget usually
	 * is a video output device. Keeping these until the specs are clear on
	 * this aspect.
	 */
	buf->buf.v4l2_buf.sequence = queue->sequence++;
	/* comment out this line if need to do stats*/
	do_gettimeofday(&buf->buf.v4l2_buf.timestamp);
	/*{
		struct timespec ts;

		ktime_get_ts(&ts);

		printk(KERN_INFO "[uvc_queue]: stats: cur=%lu.%lu, timestamp=%lu.%lu, delta=%lu.%lu\n",
			ts.tv_sec, ts.tv_nsec/1000, buf->buf.v4l2_buf.timestamp.tv_sec, buf->buf.v4l2_buf.timestamp.tv_usec,
			ts.tv_sec - buf->buf.v4l2_buf.timestamp.tv_sec, ts.tv_nsec/1000-buf->buf.v4l2_buf.timestamp.tv_usec);
	}*/

	vb2_set_plane_payload(&buf->buf, 0, buf->bytesused);
	vb2_buffer_done(&buf->buf, VB2_BUF_STATE_DONE);

	/* FIXME??? About the irqlock. */
	spin_unlock_irq(&queue->irqlock);
	uvc_buffer_done_notify(queue, buf, VB2_BUF_STATE_DONE);
	spin_lock_irq(&queue->irqlock);

	return nextbuf;
}

static struct uvc_buffer *uvc_queue_head(struct uvc_video_queue *queue)
{
	struct uvc_buffer *buf = NULL;

	if (!list_empty(&queue->irqqueue))
		buf = list_first_entry(&queue->irqqueue, struct uvc_buffer,
				       queue);
	else
		queue->flags |= UVC_QUEUE_PAUSED;

	return buf;
}

