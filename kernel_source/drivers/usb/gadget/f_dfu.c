/*
 * Gadget Driver for PLCM DFU
 *
 * Copyright (C) 2016 Polycom, Inc.
 * Author: Jerry Zhang <Jerry.Zhang@polycom.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include "f_dfu.h"

#define DFU_BULK_BUFFER_SIZE           4096

/* number of tx requests to allocate */
#define RX_REQ_MAX 4

/* ID for Microsoft WINUSB OS String */
#define WINUSB_OS_STRING_ID   0xEE

/* string IDs are assigned dynamically */

#define DFU_STRING_INTF_IDX				0

#define DFU_MAX_BUFFERS					8

#define DFU_MAX_BUFFER_SIZE				2 * 1024 * 1024

static struct usb_string dfu_en_us_strings[] = {
	[DFU_STRING_INTF_IDX].s = "DFU Interface",
	{  }
};

static struct usb_gadget_strings dfu_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = dfu_en_us_strings,
};

static struct usb_gadget_strings *dfu_function_strings[] = {
	&dfu_stringtab,
	NULL,
};

struct dfu_kevent_list {
	struct list_head list;
	struct dfu_event event;
};

static const char dfu_shortname[] = "plcm_dfu";

struct dfu_queue;

enum dfu_buffer_state {
	DFU_BUF_STATE_DEQUEUED,
	DFU_BUF_STATE_QUEUED,
	DFU_BUF_STATE_ACTIVE,
	DFU_BUF_STATE_DONE,
	DFU_BUF_STATE_ERROR,
};

struct dfu_buffer {
	struct dfu_usr_buf dfu_usr;
	struct dfu_queue *dfu_queue;

	enum dfu_buffer_state state;

	void *vaddr;
	unsigned int offset;

	struct list_head queued_entry;
	struct list_head done_entry;
};

struct dfu_queue {
	struct dfu_buffer *buffers[DFU_MAX_BUFFERS];
	unsigned int num_buffers;

	struct list_head queued_list;
	spinlock_t queued_lock;
	atomic_t queued_count;

	struct list_head done_list;
	spinlock_t done_lock;
	wait_queue_head_t done_wq;
};

struct dfu_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	struct usb_ep *ep_out;

	atomic_t open_excl;

	struct usb_request *rx_reqs[RX_REQ_MAX];

	int rx_qlen;

	/* Control event */
	spinlock_t events_lock;

	/* Dequeueable event */
	struct list_head events_avail;
	/* Number of available events. */
	unsigned int n_events_avail;
	/* Waiting for available events. */
	wait_queue_head_t events_wq;
	/* Saved control out request. */
	unsigned char ctrl_event_set;
	struct dfu_event ctrl_out;

	struct dfu_queue dfu_queue;

	spinlock_t rx_reqs_lock;
	struct list_head reqs_free_list;
	void *tmp_buffer;
	unsigned int tmp_bytesused;
};

static struct usb_interface_descriptor dfu_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 1,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xF0,
	.bInterfaceProtocol     = 0,
	.iInterface		= 0,
};

static struct usb_endpoint_descriptor dfu_superspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor dfu_superspeed_out_comp_desc = {
	.bLength =		sizeof dfu_superspeed_out_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor dfu_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor dfu_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_dfu_descs[] = {
	(struct usb_descriptor_header *) &dfu_interface_desc,
	(struct usb_descriptor_header *) &dfu_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_dfu_descs[] = {
	(struct usb_descriptor_header *) &dfu_interface_desc,
	(struct usb_descriptor_header *) &dfu_highspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *ss_dfu_descs[] = {
	(struct usb_descriptor_header *) &dfu_interface_desc,
	(struct usb_descriptor_header *) &dfu_superspeed_out_desc,
	(struct usb_descriptor_header *) &dfu_superspeed_out_comp_desc,
	NULL,
};

/* ----------------- Microsoft WinUSB Descriptors -------------------- */

/* Microsoft WINUSB OS String */
static u8 winusb_os_string[] = {
	18, /* sizeof(winusb_os_string) */
	USB_DT_STRING,
	/* Signature field: "MSFT100" */
	'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
	/* vendor code */
	0xFC,
	/* padding */
	0
};

/* Microsoft Extended Configuration Descriptor Header Section */
struct winusb_ext_config_desc_header {
	__le32	dwLength;
	__u16	bcdVersion;
	__le16	wIndex;
	__u8	bCount;
	__u8	reserved[7];
}__attribute__ ((__packed__));

/* Microsoft Extended Configuration Descriptor Function Section */
struct winusb_ext_config_desc_function {
	__u8	bFirstInterfaceNumber;
	__u8	bInterfaceCount;
	__u8	compatibleID[8];
	__u8	subCompatibleID[8];
	__u8	reserved[6];
}__attribute__ ((__packed__));

/* WINUSB Compatible ID Descriptor */
struct {
	struct winusb_ext_config_desc_header	header;
	struct winusb_ext_config_desc_function    function;
}__attribute__ ((__packed__)) winusb_ext_config_desc = {
	.header = {
		.dwLength = __constant_cpu_to_le32(sizeof(winusb_ext_config_desc)),
		.bcdVersion = __constant_cpu_to_le16(0x0100),
		.wIndex = __constant_cpu_to_le16(4),
		.bCount = 1,
	},
	.function = {
		.bFirstInterfaceNumber = 0,	/* Dynamical */
		.bInterfaceCount = 1,
		.compatibleID = { 'W', 'I', 'N', 'U', 'S', 'B' },
	},
};

/* Microsoft Extended Properties Descriptor Header Section */
struct winusb_ext_prop_desc_header {
	__le32	dwLength;
	__u16	bcdVersion;
	__le16	wIndex;
	__le16	wCount;
}__attribute__ ((__packed__));

/* Microsoft Extended Properties Descriptor GUID Section */
struct winusb_ext_prop_desc_guid {
	__le32	dwSize;
	__le32	dwPropertyDataType;
	__le16	wPropertyNameLength;
	__u8	bPropertyName[42];
	__le32	dwPropertyDataLength;
	__u8	bPropertyData[80];
}__attribute__ ((__packed__));

/* WINUSB DeviceInterface GUID Descriptor */
/* {4eb42bdb-92a9-4e9f-a299-52e3a2a147ae} */
struct {
	struct winusb_ext_prop_desc_header	header;
	struct winusb_ext_prop_desc_guid    custom;
}__attribute__ ((__packed__)) winusb_ext_prop_desc = {
	.header = {
		.dwLength = __constant_cpu_to_le32(0x00000092),
		.bcdVersion = __constant_cpu_to_le16(0x0100),
		.wIndex = __constant_cpu_to_le16(5),
		.wCount = __constant_cpu_to_le16(1),
	},
	.custom = {
		.dwSize = __constant_cpu_to_le32(0x00000088),
		.dwPropertyDataType = __constant_cpu_to_le32(0x00000007),
		.wPropertyNameLength = __constant_cpu_to_le16(0x002A),
		.bPropertyName = {'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0,
						'e', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
						'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
						'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0,
						0, 0
						},	/* DeviceInterfaceGUIDs */
		.dwPropertyDataLength = __constant_cpu_to_le32(0x00000050),
		.bPropertyData = {'{', 0, '6', 0, 'B', 0, '1', 0, '1', 0,
						'E', 0, '5', 0, 'B', 0, 'B', 0, '-', 0,
						'F', 0, '5', 0, 'C', 0, 'A', 0, '-', 0,
						'4', 0, '9', 0, '0', 0, '2', 0, '-', 0,
						'9', 0, '6', 0, '1', 0, '5', 0, '-', 0,
						'7', 0, '7', 0, '3', 0, 'B', 0, '3', 0,
						'D', 0, '2', 0, 'F', 0, '2', 0, 'F', 0,
						'E', 0, 'C', 0, '}', 0,  0,  0,  0,  0
						}, /* {6B11E5BB-F5CA-4902-9615-773B3D2F2FEC} */
	},
};

void dfu_buffer_done(struct dfu_buffer *db, enum dfu_buffer_state state);

/* ----------------- DFU Control Event Handlers -------------------- */
int dfu_event_queue(struct dfu_dev *dev, const struct dfu_event *ev)
{
	unsigned long flags;
	struct dfu_kevent_list *kevent;

	spin_lock_irqsave(&dev->events_lock, flags);

	if (dev->n_events_avail >= MAX_UAC_CTRL_EVENTS_COUNT) {
		list_for_each_entry_reverse(kevent, &dev->events_avail, list) {
			/* Copy the event to kevent. */
			memcpy(&kevent->event, ev, sizeof(struct dfu_event));
			break;
		}
		pr_warn("Override the last event\n");
		spin_unlock_irqrestore(&dev->events_lock, flags);
		return 0;
	}

	/* Use GFP_ATOMIC here. Can not sleep */
	kevent = kzalloc(sizeof(struct dfu_kevent_list), GFP_ATOMIC);
	if (!kevent) {
		spin_unlock_irqrestore(&dev->events_lock, flags);
		return -ENOMEM;
	}

	/* Copy the event to kevent. */
	memcpy(&kevent->event, ev, sizeof(struct dfu_event));

	list_add_tail(&kevent->list, &dev->events_avail);

	dev->n_events_avail++;

	/* Wake up the poll. */
	wake_up_all(&dev->events_wq);

	spin_unlock_irqrestore(&dev->events_lock, flags);

	return 0;
}

int __dfu_event_dequeue(struct dfu_dev *dev, struct dfu_event *ev)
{
	unsigned long flags;
	struct dfu_kevent_list *kevent = NULL;

	spin_lock_irqsave(&dev->events_lock, flags);

	if (list_empty(&dev->events_avail)) {
		spin_unlock_irqrestore(&dev->events_lock, flags);
		return -ENOENT;
	}

	WARN_ON(dev->n_events_avail == 0);

	kevent = list_first_entry(&dev->events_avail, struct dfu_kevent_list, list);
	list_del(&kevent->list);

	dev->n_events_avail--;

	/* Copy the kevent to ev. */
	memcpy(ev, &kevent->event, sizeof(struct dfu_event));

	/* Need to free the kevent to avoid memory leak. */
	kfree(kevent);

	spin_unlock_irqrestore(&dev->events_lock, flags);

	return 0;
}

int dfu_event_dequeue(struct dfu_dev *dev,
	struct dfu_event *ev, int nonblocking)
{
	int ret;

	if (nonblocking)
		return __dfu_event_dequeue(dev, ev);

	do {
		ret = wait_event_interruptible(dev->events_wq,
			dev->n_events_avail != 0);

		if (ret < 0)
			break;

		ret = __dfu_event_dequeue(dev, ev);
	} while(ret == -ENOENT);

	return ret;
}

/* temporary variable used between dfu_open() and dfu_gadget_bind() */
static struct dfu_dev *_dfu_dev;

static inline struct dfu_dev *func_to_dfu(struct usb_function *f)
{
	return container_of(f, struct dfu_dev, function);
}

static struct usb_request *dfu_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;
	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}
	return req;
}

static void dfu_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int dfu_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void dfu_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static void dfu_ctrl_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct dfu_dev *dev = req->context;
	int status = req->status;

	if (status == 0) {
		/* Enqueue the ctrl out event. */
		if (dev->ctrl_event_set) {
			memcpy(dev->ctrl_out.u.ctrl_event.data, req->buf, req->actual);
			dev->ctrl_out.u.ctrl_event.length = req->actual;
			dfu_event_queue(dev, &dev->ctrl_out);
			dev->ctrl_event_set = 0;
		}
	}
}

static void dfu_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct dfu_dev *dev = _dfu_dev;
	struct usb_composite_dev *cdev = dev->function.config->cdev;
	struct dfu_queue *queue = &dev->dfu_queue;
	struct dfu_buffer *db;
	unsigned int data_left, remain_space, copied_bytes;
	unsigned long flags;
	int status = req->status;
	int ret;

	if (status == -ECONNRESET || status == -ESHUTDOWN) {
		/* Probably caused by the usb_ep_dequeue. */
		/* Do not enqueue the req again. */
		WARNING(cdev, "error in usb request: %d\n", status);
		/* TODO Cancel all the queue. */
		return;
	} else if (status) {
		ERROR(cdev, "dfu_complete_out: status(%d), %d/%d\n",
			status, req->actual, req->length);
		/* TODO Cancel all the queue. */
		return;
	}

	data_left = req->actual;
	copied_bytes = 0;

	while (1) {
		spin_lock_irqsave(&queue->queued_lock, flags);

		if (list_empty(&queue->queued_list)) {
			/* Queued list is empty. */
			spin_unlock_irqrestore(&queue->queued_lock, flags);

			/* Copy the remain data to tmp buffer. */
			spin_lock_irqsave(&dev->rx_reqs_lock, flags);

			if (data_left) {
				memcpy(dev->tmp_buffer + dev->tmp_bytesused,
					req->buf + copied_bytes,
					data_left);
				dev->tmp_bytesused += data_left;
			}

			/* Add the usb request to free list. */
			list_add_tail(&req->list, &dev->reqs_free_list);

			spin_unlock_irqrestore(&dev->rx_reqs_lock, flags);

			break;
		}

		/* Get the first entry from the queued list. */
		db = list_first_entry(&queue->queued_list, struct dfu_buffer, queued_entry);

		remain_space = db->dfu_usr.length - db->dfu_usr.bytesused;

		if (data_left < remain_space) {
			/* Copy all the data to the buffer. */
			memcpy(db->vaddr + db->dfu_usr.bytesused,
					req->buf + copied_bytes,
					data_left);
			db->dfu_usr.bytesused += data_left;

			spin_unlock_irqrestore(&queue->queued_lock, flags);

			/* Move the dfu buffer to done list. */
			/*
			 * We need to check the req->actual before
			 * invoking the usb_ep_queue.
			 */
			if (req->actual < req->length &&
					db->dfu_usr.bytesused) {
				/*
				 * If we received a Zero Length Packet, or the
				 * transferred bytes is less than the required,
				 * move the dfu buffer to the done list.
				*/
				dfu_buffer_done(db, DFU_BUF_STATE_DONE);
			}

			/* Re-enqueue the usb request. */
			req->length = DFU_BULK_BUFFER_SIZE;
			req->status = 0;
			ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
			if (ret < 0) {
				ERROR(cdev, "dfu_complete_out: failed to queue req %p (%d)\n",
					req, ret);
				dfu_buffer_done(db, DFU_BUF_STATE_ERROR);
			}

			break;
		} else {
			memcpy(db->vaddr + db->dfu_usr.bytesused,
					req->buf + copied_bytes,
					remain_space);
			db->dfu_usr.bytesused += remain_space;
			spin_unlock_irqrestore(&queue->queued_lock, flags);

			copied_bytes += remain_space;
			data_left -= remain_space;

			/* Move the dfu buffer to done list */
			dfu_buffer_done(db, DFU_BUF_STATE_DONE);
		}
	}
}

static int dfu_create_bulk_endpoints(struct dfu_dev *dev,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;
	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for dfu ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	for (i = 0; i < dev->rx_qlen; i++) {
		req = dfu_request_new(dev->ep_out, DFU_BULK_BUFFER_SIZE);
		if (!req)
			goto fail;
		dev->rx_reqs[i] = req;
		req->complete = dfu_complete_out;
		req->context = dev;
	}

	return 0;
fail:
	printk(KERN_ERR "dfu_bind() could not allocate requests\n");
	return -1;
}

static int dfu_pump_data(struct dfu_dev *dev)
{
	unsigned long flags;
	struct dfu_queue *queue = &dev->dfu_queue;
	struct usb_request *req;
	struct dfu_buffer *db;
	int ret;

	while (1) {
		spin_lock_irqsave(&dev->rx_reqs_lock, flags);
		if (list_empty(&dev->reqs_free_list)) {
			spin_unlock_irqrestore(&dev->rx_reqs_lock, flags);
			return 0;
		}

		req = list_first_entry(&dev->reqs_free_list,
				struct usb_request, list);
		list_del(&req->list);
		spin_unlock_irqrestore(&dev->rx_reqs_lock, flags);

		db = list_first_entry(&queue->queued_list,
				struct dfu_buffer, queued_entry);

		spin_lock_irqsave(&dev->rx_reqs_lock, flags);
		if (dev->tmp_bytesused) {
			memcpy(db->vaddr, dev->tmp_buffer, dev->tmp_bytesused);
			db->dfu_usr.bytesused += dev->tmp_bytesused;
			dev->tmp_bytesused = 0;
		}
		spin_unlock_irqrestore(&dev->rx_reqs_lock, flags);

		req->length = DFU_BULK_BUFFER_SIZE;
		req->status = 0;
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			ERROR(dev->cdev, "Failed to enqueue usb request in dfu_pump_data\n");
			return ret;
		}
	}
}

static int dfu_open_read_pipe(struct dfu_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int i;
	unsigned long flags;
	DBG(cdev, "dfu_open_read_pipe\n");

	/* Add all the rx reqs to the free list. */
	for (i = 0; i < dev->rx_qlen; i++) {
		req = dev->rx_reqs[i];
		if (req) {
			spin_lock_irqsave(&dev->rx_reqs_lock, flags);
			list_add_tail(&req->list, &dev->reqs_free_list);
			spin_unlock_irqrestore(&dev->rx_reqs_lock, flags);
		}
	}

	return 0;
}

static void dfu_close_read_pipe(struct dfu_dev *dev)
{
	struct usb_composite_dev	*cdev = dev->cdev;
	int i;
	unsigned long flags;

	DBG(cdev, "dfu_close_read_pipe\n");


	/* Dequeue all the usb requests. */
	for(i = 0; i < dev->rx_qlen; i++) {
		if (dev->rx_reqs[i]) {
			usb_ep_dequeue(dev->ep_out, dev->rx_reqs[i]);
		}
	}

	INIT_LIST_HEAD(&dev->reqs_free_list);

	spin_lock_irqsave(&dev->rx_reqs_lock, flags);
	dev->tmp_bytesused = 0;
	spin_unlock_irqrestore(&dev->rx_reqs_lock, flags);
}

static int dfu_alloc_buffers(struct dfu_dev *dev,
	unsigned int *buff_length,
	unsigned int *num_of_buffers)
{
	unsigned int i, j;
	struct dfu_buffer *db;
	struct dfu_queue *dfu_queue = &dev->dfu_queue;

	if (*num_of_buffers > DFU_MAX_BUFFERS)
		*num_of_buffers = DFU_MAX_BUFFERS;

	if (*buff_length > DFU_MAX_BUFFER_SIZE)
		*buff_length = DFU_MAX_BUFFER_SIZE;

	/* Align the buffer length. */
	*buff_length = PAGE_ALIGN(*buff_length);

	for (i = 0; i < *num_of_buffers; i++) {
		/* Allocate the dfu_buffer structure. */
		db = kzalloc(sizeof(struct dfu_buffer), GFP_KERNEL);
		if (!db) {
			ERROR(dev->cdev, "failed to allocate the dfu buffer. index: %d\n",
				i);
			goto failed;
		}

		db->state = DFU_BUF_STATE_DEQUEUED;
		db->dfu_queue = dfu_queue;
		db->dfu_usr.index = i;

		dfu_queue->buffers[i] = db;

		db->vaddr = vmalloc_user(*buff_length);
		if (!db->vaddr) {
			ERROR(dev->cdev, "failed to allocate the dfu memory. index: %d\n",
				i);
			goto failed;
		}
		db->dfu_usr.length = *buff_length;
		db->offset = i * (*buff_length);
	}

	dfu_queue->num_buffers = *num_of_buffers;

	return 0;

failed:
	/* Clean up the allocated buffers. */
	for(j = 0; j <= i; j++) {
		db = dfu_queue->buffers[j];
		if (db) {
			if (db->vaddr)
				vfree(db->vaddr);
			kfree(db);

			dfu_queue->buffers[j] = NULL;
		}
	}

	dfu_queue->num_buffers = 0;
	return -ENOMEM;
}

static void dfu_deallocate_buffers(struct dfu_dev *dev)
{
	struct dfu_buffer *db;
	struct dfu_queue *dfu_queue = &dev->dfu_queue;
	int i;

	for (i = 0; i < dfu_queue->num_buffers; i++) {
		db = dfu_queue->buffers[i];
		if (db) {
			if (db->vaddr)
				vfree(db->vaddr);

			kfree(db);

			dfu_queue->buffers[i] = NULL;
		}
	}

	dfu_queue->num_buffers = 0;

	/* TODO Clean up the lists. */
	INIT_LIST_HEAD(&dfu_queue->queued_list);
	INIT_LIST_HEAD(&dfu_queue->done_list);
	atomic_set(&dfu_queue->queued_count, 0);
	wake_up_all(&dfu_queue->done_wq);
}

void dfu_buffer_done(struct dfu_buffer *db, enum dfu_buffer_state state)
{
	struct dfu_queue *queue = db->dfu_queue;
	unsigned long flags;

	if (state != DFU_BUF_STATE_DONE && state != DFU_BUF_STATE_ERROR)
		return;

	spin_lock_irqsave(&queue->done_lock, flags);
	db->state = state;
	list_add_tail(&db->done_entry, &queue->done_list);
	spin_unlock_irqrestore(&queue->done_lock, flags);

	spin_lock_irqsave(&queue->queued_lock, flags);
	list_del(&db->queued_entry);
	atomic_dec(&queue->queued_count);
	spin_unlock_irqrestore(&queue->queued_lock, flags);

	wake_up(&queue->done_wq);
}

static int __dfu_get_done_db(struct dfu_dev *dev,
	struct dfu_buffer **db,
	int nonblocking)
{
	unsigned long flags;
	struct dfu_queue *dfu_queue = &dev->dfu_queue;
	int ret;

	/* Wait for a done buffer. */
	for (;;) {
		if (!list_empty(&dfu_queue->done_list)) {
			/* Found a buffer that we are waiting for. */
			break;
		}

		if (nonblocking) {
			/* Non-blocking and no buffers to dequeue. */
			return -EAGAIN;
		}

		ret = wait_event_interruptible(dfu_queue->done_wq,
			!list_empty(&dfu_queue->done_list));

		if (ret) {
			ERROR(dev->cdev, "Sleep is interrupted\n");
			return ret;
		}
	}

	/* Get the first buffer from the done list. */
	spin_lock_irqsave(&dfu_queue->done_lock, flags);

	*db = list_first_entry(&dfu_queue->done_list, struct dfu_buffer, done_entry);

	list_del(&(*db)->done_entry);

	spin_unlock_irqrestore(&dfu_queue->done_lock, flags);

	return 0;
}

static int dfu_dequeue_buffer(struct dfu_dev *dev,
	struct dfu_usr_buf *b, bool nonblocking)
{
	struct dfu_buffer *db = NULL;
	int ret;

	ret = __dfu_get_done_db(dev, &db, nonblocking);
	if (ret < 0)
		return ret;

	/* Fill buffer information for the userspace */
	memcpy(b, &db->dfu_usr, sizeof(struct dfu_usr_buf));

	db->state = DFU_BUF_STATE_DEQUEUED;

	return 0;
}

static int dfu_enqueue_buffer(struct dfu_dev *dev,
	struct dfu_usr_buf *b)
{
	struct dfu_buffer *db;
	struct dfu_queue *dfu_queue = &dev->dfu_queue;
	unsigned long flags;

	if (b->index >= dfu_queue->num_buffers) {
		ERROR(dev->cdev, "The index(%u) is out of bound\n",
			b->index);
		return -EINVAL;
	}

	db = dfu_queue->buffers[b->index];
	if (NULL == db) {
		/* Should never happen. */
		ERROR(dev->cdev, "Buffer is NULL\n");
		return -EINVAL;
	}

	if (db->dfu_usr.index != b->index) {
		/* Should never happen. */
		ERROR(dev->cdev, "The index is not same\n");
		return -EINVAL;
	}

	/* Reset the bytesused. */
	db->dfu_usr.bytesused = 0;

	/*
	 * Add to the queued buffers list, a buffer will stay on it
	 * until dequeued in dequeue_buffer.
	 */
	spin_lock_irqsave(&dfu_queue->queued_lock, flags);
	list_add_tail(&db->queued_entry, &dfu_queue->queued_list);
	spin_unlock_irqrestore(&dfu_queue->queued_lock, flags);

	db->state = DFU_BUF_STATE_QUEUED;

	atomic_inc(&dfu_queue->queued_count);

	return 0;
}

static int dfu_read_buf(struct dfu_dev *dev,
	struct dfu_buf_read *buf_read)
{
	struct dfu_buffer *db;
	struct dfu_queue *dfu_queue = &dev->dfu_queue;
	void __user *umem = (void __user *)(buf_read->mem);
	int ret;

	if (buf_read->index >= dfu_queue->num_buffers) {
		ERROR(dev->cdev, "The index(%u) is out of bound\n",
			buf_read->index);
		return -EINVAL;
	}

	db = dfu_queue->buffers[buf_read->index];
	if (NULL == db) {
		/* Should never happen. */
		ERROR(dev->cdev, "Buffer is NULL\n");
		return -EINVAL;
	}

	if (db->dfu_usr.index != buf_read->index) {
		/* Should never happen. */
		ERROR(dev->cdev, "The index is not same\n");
		return -EINVAL;
	}

	if (buf_read->length > db->dfu_usr.bytesused) {
		ERROR(dev->cdev, "The required size is too big\n");
		return -EINVAL;
	}

	ret = copy_to_user(umem, db->vaddr, buf_read->length);
	if (ret) {
		ERROR(dev->cdev, "Still have %d bytes data to be copied\n",
			ret);

		return -EFAULT;
	}

	return ret;
}

static int dfu_open(struct inode *ip, struct file *fp)
{
	static DEFINE_RATELIMIT_STATE(rl, 10*HZ, 1);
	if (__ratelimit(&rl))
		pr_info("dfu_open\n");
	if (!_dfu_dev)
		return -ENODEV;
	if (dfu_lock(&_dfu_dev->open_excl))
		return -EBUSY;
	fp->private_data = _dfu_dev;

	return 0;
}

static int dfu_release(struct inode *ip, struct file *fp)
{
	static DEFINE_RATELIMIT_STATE(rl, 10*HZ, 1);
	if (__ratelimit(&rl))
		pr_info("dfu_release\n");

	fp->private_data = NULL;

	dfu_unlock(&_dfu_dev->open_excl);
	return 0;
}

static void dfu_vm_open(struct vm_area_struct *vma)
{
}

static void dfu_vm_close(struct vm_area_struct *vma)
{
}

/**
 * dfu_vm_ops - common vm_ops used for tracking refcount of mmaped
 * video buffers
 */
const struct vm_operations_struct dfu_vm_ops = {
	.open = dfu_vm_open,
	.close = dfu_vm_close,
};

static int dfu_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct dfu_dev *dev = filp->private_data;
	struct dfu_queue *queue = &dev->dfu_queue;
	struct dfu_buffer *db = NULL;

	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned int i;

	unsigned long vm_size;

	int ret;

	/*
	 * Check memory area access mode.
	 */
	if (!(vma->vm_flags & VM_SHARED)) {
		ERROR(dev->cdev, "dfu_mmap: Invaild vma flags, VM_SHARED needed\n");
		return -EINVAL;
	}

	if (!(vma->vm_flags & VM_READ)) {
		ERROR(dev->cdev, "dfu_mmap: Invaild vma flags, VM_READ needed\n");
		return -EINVAL;
	}

	vm_size = vma->vm_end - vma->vm_start;

	/* Try to find the dfu buffer by offset. */
	for (i = 0; i < queue->num_buffers; i++) {
		if (queue->buffers[i]->offset == offset) {
			db = queue->buffers[i];
			break;
		}
	}

	if (!db || vm_size != db->dfu_usr.length) {
		ERROR(dev->cdev, "dfu_mmap: Could not find the matched buffer block\n");
		return -EINVAL;
	}

	ret = remap_vmalloc_range(vma, db->vaddr, 0);
	if (ret) {
		ERROR(dev->cdev, "dfu_mmap: Remapping vmalloc memory, error: %d\n", ret);
		return ret;
	}

	/*
	 * Make sure that vm_areas for 2 buffers won't be merged together.
	 */
	vma->vm_flags |= VM_DONTEXPAND;

	vma->vm_ops = &dfu_vm_ops;
	vma->vm_private_data = db;

	vma->vm_ops->open(vma);

	return 0;
}

static unsigned int dfu_poll(struct file *filp, poll_table *wait)
{
	unsigned int ret = 0;
	unsigned long req_events = poll_requested_events(wait);
	struct dfu_dev *dev = filp->private_data;
	struct dfu_queue *queue = &dev->dfu_queue;

	if (dev->n_events_avail)
		ret = POLLPRI;
	else if (req_events & POLLPRI)
		poll_wait(filp, &dev->events_wq, wait);

	if (list_empty(&queue->done_list))
		poll_wait(filp, &queue->done_wq, wait);
	else
		return ret | POLLIN | POLLRDNORM;

	return ret;
}

static long dfu_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct dfu_dev *dev = filp->private_data;
	u8 __user *uarg = (u8 __user *)arg;
	struct dfu_event event;
	struct dfu_ctrl_event ctrl_event;
	struct usb_composite_dev *cdev = dev->function.config->cdev;
	struct usb_request *req = cdev->req;
	struct dfu_buf_alloc buf_alloc;
	struct dfu_usr_buf usr_buf;
	struct dfu_buf_read buf_read;
	int err = 0;
	unsigned int n = 0, count = 0;

	switch (cmd) {
	case DFU_IOC_SEND_CTRL_REPLY:
		if (!(filp->f_mode & FMODE_WRITE)) {
			err = -EBADF;
			break;
		}

		n = _IOC_SIZE(cmd);

		if (copy_from_user(&ctrl_event, uarg, n)) {
			err = -EFAULT;
			break;
		}

		count = ctrl_event.length;

		memcpy(req->buf, ctrl_event.data, count);

		req->zero = 0;
		req->length = count;
		err = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (err < 0)
			ERROR(cdev, "usb_ep_queue error on ep0 err=%d,length=%d\n",
				err, req->length);
		break;

	case DFU_IOC_DQ_EVENT:
		if (!(filp->f_mode & FMODE_READ)) {
			err = -EBADF;
			break;
		}

		err = dfu_event_dequeue(dev, &event,
			filp->f_mode & O_NONBLOCK);

		if (err)
			break;

		n = _IOC_SIZE(cmd);

		if (copy_to_user(uarg, &event, n)) {
			err = -EFAULT;
		}

		break;

	case DFU_IOC_CREATE_BUF:
		if (!(filp->f_mode & FMODE_WRITE)) {
			err = -EBADF;
			break;
		}

		n = _IOC_SIZE(cmd);

		if (copy_from_user(&buf_alloc, uarg, n)) {
			err = -EFAULT;
			break;
		}

		if (buf_alloc.num_of_buffs) {
			err = dfu_alloc_buffers(dev,
					&buf_alloc.buff_length,
					&buf_alloc.num_of_buffs);

			if (err < 0)
				break;

			if (copy_to_user(uarg, &buf_alloc, n)) {
				err = -EFAULT;
				break;
			}
		}
		else
			dfu_deallocate_buffers(dev);

		break;

	case DFU_IOC_ENQUEUE_BUF:
		if (!(filp->f_mode & FMODE_WRITE)) {
			err = -EBADF;
			break;
		}

		n = _IOC_SIZE(cmd);

		if (copy_from_user(&usr_buf, uarg, n)) {
			err = -EFAULT;
			break;
		}

		err = dfu_enqueue_buffer(dev, &usr_buf);
		if (err < 0) {
			ERROR(dev->cdev, "failed to enqueue the dfu buffer. err: %d\n",
				err);
			break;
		}

		dfu_pump_data(dev);

		break;

	case DFU_IOC_DEQUEUE_BUF:
		if (!(filp->f_mode & FMODE_READ)) {
			err = -EBADF;
			break;
		}

		err = dfu_dequeue_buffer(dev, &usr_buf, filp->f_flags & O_NONBLOCK);
		if (err < 0) {
			ERROR(dev->cdev, "failed to dequeue the dfu buffer. err: %d\n",
				err);
			break;
		}

		n = _IOC_SIZE(cmd);

		if (copy_to_user(uarg, &usr_buf, n)) {
			err = -EFAULT;
		}

		break;

	case DFU_IOC_READ_BUF:
		if (!(filp->f_mode & FMODE_READ)) {
			err = -EBADF;
			break;
		}

		n = _IOC_SIZE(cmd);

		if (copy_from_user(&buf_read, uarg, n)) {
			err = -EFAULT;
		}

		err = dfu_read_buf(dev, &buf_read);
		if (err < 0) {
			ERROR(dev->cdev, "failed to read the dfu buffer. err: %d\n",
				err);
		}

		break;

	case DFU_IOC_OPEN_STREAM:
		if (!(filp->f_mode & FMODE_WRITE)) {
			err = -EBADF;
			break;
		}

		err = dfu_open_read_pipe(dev);
		if (err)
			ERROR(cdev, "failed to open the dfu read pipe\n");

		break;

	case DFU_IOC_CLOSE_STREAM:
		if (!(filp->f_mode & FMODE_WRITE)) {
			err = -EBADF;
			break;
		}

		dfu_close_read_pipe(dev);

		err = 0;

		break;

	default:
		err = -ENOTTY;
		break;
	}

	return err;
}

/* file operations for DFU device /dev/plcm_dfu */
static const struct file_operations dfu_fops = {
	.owner = THIS_MODULE,
	.open = dfu_open,
	.release = dfu_release,
	.mmap = dfu_mmap,
	.poll = dfu_poll,
	.unlocked_ioctl = dfu_ioctl,
};

static struct miscdevice dfu_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = dfu_shortname,
	.fops = &dfu_fops,
};

static int
dfu_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct dfu_dev	*dev = func_to_dfu(f);
	int			id;
	int			ret;
	dev->cdev = cdev;
	DBG(cdev, "dfu_function_bind dev: %p\n", dev);
	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	dfu_interface_desc.bInterfaceNumber = id;
	winusb_ext_config_desc.function.bFirstInterfaceNumber = id;

	/* allocate endpoints */
	ret = dfu_create_bulk_endpoints(dev, &dfu_fullspeed_out_desc);
	if (ret)
		return ret;
	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		dfu_highspeed_out_desc.bEndpointAddress =
			dfu_fullspeed_out_desc.bEndpointAddress;
	}
	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		dfu_superspeed_out_desc.bEndpointAddress =
			dfu_fullspeed_out_desc.bEndpointAddress;
	}

	f->fs_descriptors = fs_dfu_descs;
	f->hs_descriptors = hs_dfu_descs;

	if (gadget_is_superspeed(c->cdev->gadget))
		f->ss_descriptors = ss_dfu_descs;

	DBG(cdev, "%s speed %s: OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_out->name);
	return 0;
}

static void
dfu_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct dfu_dev	*dev = func_to_dfu(f);
	int i;

	usb_ep_disable(dev->ep_out);
	for(i = 0; i < dev->rx_qlen; i++) {
		if (dev->rx_reqs[i]) {
			usb_ep_dequeue(dev->ep_out, dev->rx_reqs[i]);
			dfu_request_free(dev->rx_reqs[i], dev->ep_out);
		}
	}
}

static int dfu_function_setup(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct dfu_dev *dev = func_to_dfu(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	struct		dfu_event ctrl_in_event;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

#if 0
	pr_trace("dfu ctrl req: %02x.%02x v%04x i%04x l%d\n",
		ctrl->bRequestType, ctrl->bRequest, w_value, w_index, w_length);
#endif

	switch (ctrl->bRequestType) {
		case USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE:
			/* Save the setup packet here.
			 * The data will be received in
			 * Data stage.
			 */
			dev->ctrl_out.event_type = DFU_EVENT_TYPE_CTRL_REQ;
			dev->ctrl_out.u.ctrl_event.request = ctrl->bRequest;
			dev->ctrl_out.u.ctrl_event.value = w_value;
			dev->ctrl_out.u.ctrl_event.length = w_length;

			dev->ctrl_event_set = 1;

			value = w_length;
			break;

		case USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_INTERFACE:
			/* Enqueue the setup packet directly here.
			 * The data will be send in the ioctl callback.
			 */
			memset(&ctrl_in_event, 0, sizeof(struct dfu_event));
			ctrl_in_event.event_type = DFU_EVENT_TYPE_CTRL_REQ;
			ctrl_in_event.u.ctrl_event.request = ctrl->bRequest;
			ctrl_in_event.u.ctrl_event.value = w_value;
			ctrl_in_event.u.ctrl_event.length = w_length;

			dfu_event_queue(dev, &ctrl_in_event);

			return 0;

		default:
			ERROR(cdev, "invalid control req: %02x.%02x v%04x i%04x l%d\n",
					ctrl->bRequestType, ctrl->bRequest, w_value, w_index, w_length);
			break;
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		req->zero = 0;
		req->length = value;
		req->context = dev;
		req->complete = dfu_ctrl_complete;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "audio response on err %d\n", value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
};

static int dfu_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct dfu_dev	*dev = func_to_dfu(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG(cdev, "dfu_function_set_alt intf: %d alt: %d\n", intf, alt);

	if (dev->ep_out != NULL) {
		/* Restart the endpoint. */
		if (dev->ep_out->driver_data != NULL)
			usb_ep_disable(dev->ep_out);

		ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
		if (ret) {
			dev->ep_out->desc = NULL;
			ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_out->name, ret);
			return ret;
		}
		ret = usb_ep_enable(dev->ep_out);
		if (ret) {
			ERROR(cdev, "failed to enable ep %s, result %d\n",
					dev->ep_out->name, ret);
			return ret;
		}

		dev->ep_out->driver_data = dev;
	}

	return 0;
}

static void dfu_function_disable(struct usb_function *f)
{
	struct dfu_dev	*dev = func_to_dfu(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	DBG(cdev, "dfu_function_disable cdev %p\n", cdev);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static void dfu_function_resume(struct usb_function *f)
{
	struct dfu_dev	*dev = func_to_dfu(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	struct dfu_event dfu_event;

	/* Notify the app. */
	dfu_event.event_type = DFU_EVENT_TYPE_USB_STATE;
	dfu_event.u.usb_state = DFU_USB_STATE_RESUME;
	dfu_event_queue(dev, &dfu_event);

	DBG(cdev, "%s resume\n", dev->function.name);
}

static void dfu_function_suspend(struct usb_function *f)
{
	struct dfu_dev	*dev = func_to_dfu(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	struct dfu_kevent_list		*kev_list, *kev_next;
	unsigned long flags;
	struct dfu_event dfu_event;

	/* Clean up the list in events_avail. */
	spin_lock_irqsave(&dev->events_lock, flags);
	list_for_each_entry_safe(kev_list, kev_next, &dev->events_avail, list) {
		list_del(&kev_list->list);
		kfree(kev_list);
	}
	spin_unlock_irqrestore(&dev->events_lock, flags);

	/* Clean up the usb requests. */
	INIT_LIST_HEAD(&dev->reqs_free_list);

	dev->tmp_bytesused = 0;

	/* Notify the app. */
	dfu_event.event_type = DFU_EVENT_TYPE_USB_STATE;
	dfu_event.u.usb_state = DFU_USB_STATE_SUSPEND;
	dfu_event_queue(dev, &dfu_event);

	DBG(cdev, "%s suspended\n", dev->function.name);
}

static int dfu_bind_config(struct usb_configuration *c)
{
	struct dfu_dev *dev = _dfu_dev;
	int ret;

	printk(KERN_INFO "dfu_bind_config\n");

	if (dfu_en_us_strings[DFU_STRING_INTF_IDX].id == 0) {
		ret = usb_string_ids_tab(c->cdev, dfu_en_us_strings);
		if (ret)
			return -ENOMEM;
		dfu_interface_desc.iInterface =
			dfu_en_us_strings[DFU_STRING_INTF_IDX].id;
	}

	dev->cdev = c->cdev;
	dev->function.name = "dfu";
	dev->function.strings = dfu_function_strings;
	dev->function.bind = dfu_function_bind;
	dev->function.unbind = dfu_function_unbind;
	dev->function.setup = dfu_function_setup;
	dev->function.set_alt = dfu_function_set_alt;
	dev->function.disable = dfu_function_disable;
	dev->function.resume = dfu_function_resume;
	dev->function.suspend = dfu_function_suspend;

	return usb_add_function(c, &dev->function);
}

static int dfu_ctrlrequest(struct usb_composite_dev *cdev,
				const struct usb_ctrlrequest *ctrl)
{
	int	value = -EOPNOTSUPP;
	u16	w_index = le16_to_cpu(ctrl->wIndex);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_length = le16_to_cpu(ctrl->wLength);

#if 0
	DBG(cdev, "dfu_ctrlrequest "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
#endif

	/* Handle WINUSB OS string */
	if (ctrl->bRequestType ==
			(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE)
			&& ctrl->bRequest == USB_REQ_GET_DESCRIPTOR
			&& (w_value >> 8) == USB_DT_STRING
			&& (w_value & 0xFF) == WINUSB_OS_STRING_ID) {
		value = (w_length < sizeof(winusb_os_string)
				? w_length : sizeof(winusb_os_string));
		memcpy(cdev->req->buf, winusb_os_string, value);
	} else if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		/* Handle WINUSB OS descriptor */
#if 0
		DBG(cdev, "MS Vendor Request: %d index: %d value: %d length: %d\n",
			ctrl->bRequest, w_index, w_value, w_length);
#endif

		if (ctrl->bRequest == 0xFC
				&& (ctrl->bRequestType & USB_DIR_IN)
				&& (w_index == 4)) {
			value = (w_length < sizeof(winusb_ext_config_desc) ?
					w_length : sizeof(winusb_ext_config_desc));
			memcpy(cdev->req->buf, &winusb_ext_config_desc, value);
		} else if (ctrl->bRequest == 0xFC
				&& (ctrl->bRequestType & USB_DIR_IN)
				&& (w_index == 5)
				&& ((w_value & 0xFF) == winusb_ext_config_desc.function.bFirstInterfaceNumber)) {
			value = (w_length < sizeof(winusb_ext_prop_desc) ?
					w_length : sizeof(winusb_ext_prop_desc));
			memcpy(cdev->req->buf, &winusb_ext_prop_desc, value);
		}
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		int rc;
		cdev->req->zero = value < w_length;
		cdev->req->length = value;
		rc = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (rc < 0)
			ERROR(cdev, "%s: response queue error\n", __func__);
	}

	return value;
}

static int dfu_setup(void)
{
	struct dfu_dev *dev;
	int ret;
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	init_waitqueue_head(&dev->events_wq);

	atomic_set(&dev->open_excl, 0);

	/* config is disabled by default if dfu is present. */
	INIT_LIST_HEAD(&dev->events_avail);

	dev->rx_qlen = RX_REQ_MAX;

	/* initialize the dfu queue. */
	INIT_LIST_HEAD(&dev->dfu_queue.queued_list);
	spin_lock_init(&dev->dfu_queue.queued_lock);
	INIT_LIST_HEAD(&dev->dfu_queue.done_list);
	spin_lock_init(&dev->dfu_queue.done_lock);
	init_waitqueue_head(&dev->dfu_queue.done_wq);

	dev->tmp_buffer = kmalloc(dev->rx_qlen * DFU_BULK_BUFFER_SIZE, GFP_KERNEL);
	if (!dev->tmp_buffer) {
		kfree(dev);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&dev->reqs_free_list);
	spin_lock_init(&dev->rx_reqs_lock);

	_dfu_dev = dev;
	ret = misc_register(&dfu_device);
	if (ret)
		goto err;
	return 0;
err:
	kfree(dev->tmp_buffer);
	kfree(dev);
	printk(KERN_ERR "dfu gadget driver failed to initialize\n");
	return ret;
}

static void dfu_cleanup(void)
{
	misc_deregister(&dfu_device);
	/* TODO Clean up the queue. */
	kfree(_dfu_dev->tmp_buffer);
	kfree(_dfu_dev);
	_dfu_dev = NULL;
}
