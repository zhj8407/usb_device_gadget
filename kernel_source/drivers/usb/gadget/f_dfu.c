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

struct f_dfu_rx_req_list {
	struct usb_request	*req;
	unsigned int		pos;
	struct list_head	list;
};

struct dfu_kevent_list {
	struct list_head list;
	struct dfu_ctrl_event event;
};

static const char dfu_shortname[] = "plcm_dfu";

struct dfu_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	spinlock_t read_lock;

	struct usb_ep *ep_out;

	atomic_t online;
	atomic_t error;

	atomic_t read_excl;
	atomic_t open_excl;

	struct list_head rx_idle;

	wait_queue_head_t read_wq;

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
	struct dfu_ctrl_event ctrl_out;
};

static struct usb_interface_descriptor dfu_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 1,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0x00,
	.bInterfaceProtocol     = 0,
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
		.bPropertyData = {'{', 0, 'F', 0, '7', 0, '2', 0, 'F', 0,
						'E', 0, '0', 0, 'D', 0, '4', 0, '-', 0,
						'C', 0, 'B', 0, 'C', 0, 'B', 0, '-', 0,
						'4', 0, '0', 0, '7', 0, 'D', 0, '-', 0,
						'8', 0, '8', 0, '1', 0, '4', 0, '-', 0,
						'9', 0, 'E', 0, 'D', 0, '6', 0, '7', 0,
						'3', 0, 'D', 0, '0', 0, 'D', 0, 'D', 0,
						'6', 0, 'B', 0, '}', 0,  0,  0,  0,  0
						},
	},
};

/* ----------------- DFU Control Event Handlers -------------------- */
int dfu_event_queue(struct dfu_dev *dev, const struct dfu_ctrl_event *ev)
{
	unsigned long flags;
	struct dfu_kevent_list *kevent;

	spin_lock_irqsave(&dev->events_lock, flags);

	if (dev->n_events_avail >= MAX_UAC_CTRL_EVENTS_COUNT) {
		list_for_each_entry_reverse(kevent, &dev->events_avail, list) {
			/* Copy the event to kevent. */
			memcpy(&kevent->event, ev, sizeof(struct dfu_ctrl_event));
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
	memcpy(&kevent->event, ev, sizeof(struct dfu_ctrl_event));

	list_add_tail(&kevent->list, &dev->events_avail);

	dev->n_events_avail++;

	/* Wake up the poll. */
	wake_up_all(&dev->events_wq);

	spin_unlock_irqrestore(&dev->events_lock, flags);

	return 0;
}

int __dfu_event_dequeue(struct dfu_dev *dev, struct dfu_ctrl_event *ev)
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
	memcpy(ev, &kevent->event, sizeof(struct dfu_ctrl_event));

	spin_unlock_irqrestore(&dev->events_lock, flags);

	return 0;
}

int dfu_event_dequeue(struct dfu_dev *dev,
	struct dfu_ctrl_event *ev, int nonblocking)
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
			memcpy(dev->ctrl_out.data, req->buf, req->actual);
			dev->ctrl_out.length = req->actual;
			dfu_event_queue(dev, &dev->ctrl_out);
			dev->ctrl_event_set = 0;
		}
	}
}

static void dfu_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct dfu_dev *dev = _dfu_dev;
	struct usb_composite_dev *cdev = dev->function.config->cdev;
	struct f_dfu_rx_req_list *req_list;
	unsigned long flags;
	int status = req->status;
	int ret;

	if (status == -ECONNRESET || status == -ESHUTDOWN) {
		/* Probably caused by the usb_ep_dequeue. */
		/* Do not enqueue the req again. */
		WARNING(cdev, "error in usb request: %d\n", status);
		return;
	} else if (status) {
		ERROR(cdev, "dfu_complete_out: status(%d), %d/%d\n",
			status, req->actual, req->length);
		req->actual = 0;
	}

	/* If we received a Zero Length Packet, re-enqueue the request. */
	if (req->actual == 0) {
		req->length = DFU_BULK_BUFFER_SIZE;
		req->status = 0;
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			ERROR(cdev, "dfu_complete_out: failed to queue req %p (%d)\n",
				req, ret);
			atomic_set(&dev->error, 1);
			wake_up(&dev->read_wq);
		}
		return;
	}

	req_list = kzalloc(sizeof(*req_list), GFP_ATOMIC);
	if (!req_list)
		return;

	req_list->req = req;

	spin_lock_irqsave(&dev->read_lock, flags);
	list_add_tail(&req_list->list, &dev->rx_idle);
	spin_unlock_irqrestore(&dev->read_lock, flags);

	wake_up(&dev->read_wq);
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

static ssize_t dfu_read(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	struct dfu_dev *dev = fp->private_data;
	struct f_dfu_rx_req_list *req_list;
	struct usb_request *req;
	unsigned long flags;
	int ret;

#ifdef DFU_DEBUG
	pr_debug("dfu_read(%d)\n", count);
#endif

	if (!_dfu_dev)
		return -ENODEV;

	if (!count)
		return 0;

	if (count > DFU_BULK_BUFFER_SIZE)
		return -EINVAL;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	if (dfu_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while (!(atomic_read(&dev->online) || atomic_read(&dev->error))) {
		pr_debug("dfu_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq,
			(atomic_read(&dev->online) ||
			atomic_read(&dev->error)));
		if (ret < 0) {
			dfu_unlock(&dev->read_excl);
			return ret;
		}
	}
	if (atomic_read(&dev->error)) {
		ret = -EIO;
		goto done;
	}

	spin_lock_irqsave(&dev->read_lock, flags);

#define DFU_READ_COND (!list_empty(&dev->rx_idle))

	/* wait for at least one buffer to complete. */
	while (!(DFU_READ_COND || atomic_read(&dev->error))) {
		spin_unlock_irqrestore(&dev->read_lock, flags);

		if (fp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto done;
		}

		ret = wait_event_interruptible(dev->read_wq, DFU_READ_COND ||
				atomic_read(&dev->error));

		if (ret < 0) {
			if (ret != -ERESTARTSYS)
			atomic_set(&dev->error, 1);
			goto done;
		}

		spin_lock_irqsave(&dev->read_lock, flags);
	}

	if (atomic_read(&dev->error)) {
		spin_unlock_irqrestore(&dev->read_lock, flags);
		ret = -EIO;
		goto done;
	}

	/* pick the first one. */
	req_list = list_first_entry(&dev->rx_idle,
					struct f_dfu_rx_req_list, list);

	req = req_list->req;
	count = min_t(unsigned int, count, req->actual - req_list->pos);
	spin_unlock_irqrestore(&dev->read_lock, flags);

	/* copy to user outside spinlock */
	count -= copy_to_user(buf, req->buf + req_list->pos, count);
	req_list->pos += count;

	if (req_list->pos == req->actual) {
		spin_lock_irqsave(&dev->read_lock, flags);
		list_del(&req_list->list);
		kfree(req_list);
		spin_unlock_irqrestore(&dev->read_lock, flags);

		req->status = 0;
		req->length = DFU_BULK_BUFFER_SIZE;
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0)
			goto done;
	}

	ret = count;

done:
	dfu_unlock(&dev->read_excl);
#ifdef DFU_DEBUG
	pr_debug("dfu_read returning %d\n", ret);
#endif
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
	/* clear the error latch */
	atomic_set(&_dfu_dev->error, 0);
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

static unsigned int dfu_poll(struct file *filp, poll_table *wait)
{
	unsigned int ret = 0;
	unsigned long req_events = poll_requested_events(wait);
	struct dfu_dev *dev = filp->private_data;

	if (dev->n_events_avail)
		ret = POLLPRI;
	else if (req_events & POLLPRI)
		poll_wait(filp, &dev->events_wq, wait);

#if 0
	if (atomic_read(&dev->error))
		return ret | POLLERR;
#endif

	if (list_empty(&dev->rx_idle))
		poll_wait(filp, &dev->read_wq, wait);
	else
		return ret | POLLIN | POLLRDNORM;

	return ret;
}

static long dfu_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct dfu_dev *dev = filp->private_data;
	u8 __user *uarg = (u8 __user *)arg;
	struct dfu_ctrl_event event;
	struct usb_composite_dev *cdev = dev->function.config->cdev;
	struct usb_request *req = cdev->req;
	int err;
	unsigned int n = 0, count = 0;

	switch (cmd) {
	case DFU_IOC_SEND_CTRL_REPLY:
		if (!(filp->f_mode & FMODE_WRITE)) {
			err = -EBADF;
			break;
		}

		n = _IOC_SIZE(cmd);

		if (copy_from_user(&event, uarg, n)) {
			err = -EFAULT;
			break;
		}

		count = event.length;

		memcpy(req->buf, event.data, count);

		req->zero = 0;
		req->length = count;
		err = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (err < 0)
			ERROR(cdev, "usb_ep_queue error on ep0 err=%d,length=%d\n",
				err, req->length);
		break;

	case DFU_IOC_DQ_CTRL_EVENT:
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

	default:
		err = -ENOTTY;
		break;
	}

	return err;
}

/* file operations for DFU device /dev/plcm_dfu */
static const struct file_operations dfu_fops = {
	.owner = THIS_MODULE,
	.read = dfu_read,
	.open = dfu_open,
	.release = dfu_release,
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

	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);

	wake_up(&dev->read_wq);

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
	struct		dfu_ctrl_event ctrl_in_event;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	pr_trace("dfu ctrl req: %02x.%02x v%04x i%04x l%d\n",
		ctrl->bRequestType, ctrl->bRequest, w_value, w_index, w_length);

	switch (ctrl->bRequestType) {
		case USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE:
			/* Save the setup packet here.
			 * The data will be received in
			 * Data stage.
			 */
			dev->ctrl_out.request = ctrl->bRequest;
			dev->ctrl_out.value = w_value;
			dev->ctrl_out.length = w_length;

			dev->ctrl_event_set = 1;

			value = w_length;
			break;

		case USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE:
			/* Enqueue the setup packet directly here.
			 * The data will be send in the ioctl callback.
			 */
			memset(&ctrl_in_event, 0, sizeof(struct dfu_ctrl_event));
			ctrl_in_event.request = ctrl->bRequest;
			ctrl_in_event.value = w_value;
			ctrl_in_event.length = w_length;

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
	struct usb_request *req;
	int ret;
	int i;
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

	atomic_set(&dev->online, 1);
	atomic_set(&dev->error, 0);
	/* readers may be blocked waiting for us to go online */

	/* queue all the rx reqs at once. */
	for (i = 0; i < dev->rx_qlen; i++) {
		req = dev->rx_reqs[i];
		if (req) {
			req->length = DFU_BULK_BUFFER_SIZE;
			ret = usb_ep_queue(dev->ep_out, dev->rx_reqs[i],
				GFP_ATOMIC);
			if (ret)
				ERROR(cdev, "%s queue req --> %d\n",
					dev->ep_out->name, ret);
		}
	}

	wake_up(&dev->read_wq);

	return 0;
}

static void dfu_function_disable(struct usb_function *f)
{
	struct dfu_dev	*dev = func_to_dfu(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	DBG(cdev, "dfu_function_disable cdev %p\n", cdev);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static void dfu_function_suspend(struct usb_function *f)
{
	struct dfu_dev	*dev = func_to_dfu(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	struct f_dfu_rx_req_list *rx_list, *rx_next;
	struct dfu_kevent_list *kev_list, *kev_next;
	DBG(cdev, "dfu_function_suspend cdev %p\n", cdev);

	/*
	 * Cable disconnected.  No
	 * need to disable the configuration now.  We will
	 * set noify_close to true when device file is re-opened.
	 */
	atomic_set(&dev->online, 0);
	atomic_set(&dev->error, 1);

	/* Clean up the list in rx_idle. */
	list_for_each_entry_safe(rx_list, rx_next, &dev->rx_idle, list) {
		list_del(&rx_list->list);
		kfree(rx_list);
	}

	/* Clean up the list in events_avail. */
	list_for_each_entry_safe(kev_list, kev_next, &dev->events_avail, list) {
		list_del(&kev_list->list);
		kfree(kev_list);
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&dev->read_wq);
	VDBG(cdev, "%s suspended\n", dev->function.name);
}

static int dfu_bind_config(struct usb_configuration *c)
{
	struct dfu_dev *dev = _dfu_dev;

	printk(KERN_INFO "dfu_bind_config\n");

	dev->cdev = c->cdev;
	dev->function.name = "dfu";
	dev->function.bind = dfu_function_bind;
	dev->function.unbind = dfu_function_unbind;
	dev->function.setup = dfu_function_setup;
	dev->function.set_alt = dfu_function_set_alt;
	dev->function.disable = dfu_function_disable;
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

	DBG(cdev, "dfu_ctrlrequest "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);

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
		DBG(cdev, "MS Vendor Request: %d index: %d value: %d length: %d\n",
			ctrl->bRequest, w_index, w_value, w_length);

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

	spin_lock_init(&dev->read_lock);

	init_waitqueue_head(&dev->read_wq);
	init_waitqueue_head(&dev->events_wq);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);

	/* config is disabled by default if dfu is present. */
	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->events_avail);

	dev->rx_qlen = RX_REQ_MAX;

	_dfu_dev = dev;
	ret = misc_register(&dfu_device);
	if (ret)
		goto err;
	return 0;
err:
	kfree(dev);
	printk(KERN_ERR "dfu gadget driver failed to initialize\n");
	return ret;
}

static void dfu_cleanup(void)
{
	misc_deregister(&dfu_device);
	kfree(_dfu_dev);
	_dfu_dev = NULL;
}
