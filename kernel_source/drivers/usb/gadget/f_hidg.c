/*
 * f_hid.c -- USB HID function driver
 *
 * Copyright (C) 2015 Polycom, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hid.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/nls.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>

// DFSI: Display Field Support Index
#define USBHID_DFSI_LOCAL_NAME         1
#define USBHID_DFSI_LOCAL_STATUS       2
#define USBHID_DFSI_DATE               3
#define USBHID_DFSI_TIME               4
#define USBHID_DFSI_CALL_STATUS        5
#define USBHID_DFSI_OTHER_NAME         6
#define USBHID_DFSI_OTHER_TITIL        7
#define USBHID_DFSI_SUBJECT            8
#define USBHID_DFSI_DURATION           9
#define USBHID_DFSI_NUMBER             10
#define USBHID_DFSI_OTHER_NUMBER       11
#define USBHID_DFSI_CONV_ID            12

// based on usbhidDescriptor.txt
#define USBHID_TEL_REPORT_ID           0x02
#define USBHID_LED_REPORT_ID           0x03
#define USBHID_VENDOR_EXT_REPORT_ID    0x11
#define USBHID_DISP_ATTR_REPORT_ID     0x12
#define USBHID_DISP_CTRL_REPORT_ID     0x13
#define USBHID_CHAR_ATTR_REPORT_ID     0x14
#define USBHID_CHAR_REPORT_ID          0x15
#define USBHID_ICON_REPORT_ID          0x16

#define USB_HID_ReportType_NULL     0x00
#define USB_HID_ReportType_Input    0x01
#define USB_HID_ReportType_Output   0x02
#define USB_HID_ReportType_Feature  0x03

/*-------------------------------------------------------------------------*/
#define HID_REPORT_DESC_MAX_LENGTH	2048
#define USB_STRING_MAX_LENGTH 126

#define XFER_INT_OUT_REQ_MAX_COUNT		4

/*                            HID gadget struct                            */
struct hidg_device_config {
	int	device;
	struct device *dev;

	unsigned char bInterfaceSubClass;
	unsigned char bInterfaceProtocol;

	unsigned short report_length;
	unsigned short report_desc_length;
	unsigned char report_desc[HID_REPORT_DESC_MAX_LENGTH];
	unsigned char lync_ucq_string[USB_STRING_MAX_LENGTH];
};

struct f_hidg_req_list {
	struct usb_request	*req;
	unsigned int		pos;
	struct list_head	list;
	unsigned int		ctl_ep;
};

struct f_hidg {
	/* configuration */
	unsigned char			bInterfaceSubClass;
	unsigned char			bInterfaceProtocol;
	unsigned short			report_desc_length;
	char				*report_desc;
	unsigned short			report_length;

	/* recv report */
	struct list_head		completed_out_req;
	spinlock_t			spinlock;
	wait_queue_head_t		read_queue;
	unsigned int			qlen;

	/* send report */
	struct mutex			lock;
	bool				write_pending;
	wait_queue_head_t		write_queue;
	struct usb_request		*req;

	int				minor;
	struct cdev			cdev;
	struct usb_function		func;

	struct usb_ep			*in_ep;
	struct usb_ep			*out_ep;
	struct usb_request		*out_reqs[XFER_INT_OUT_REQ_MAX_COUNT];
};

static inline struct f_hidg *func_to_hidg(struct usb_function *f)
{
	return container_of(f, struct f_hidg, func);
}

static int major, minors;
static struct class *hidg_class;
static struct hidg_device_config *hidg_config;

/*-------------------------------------------------------------------------*/
/*                           Static descriptors                            */

static struct usb_interface_descriptor hidg_interface_desc = {
	.bLength		= sizeof(hidg_interface_desc),
	.bDescriptorType	= USB_DT_INTERFACE,
	/* .bInterfaceNumber	= DYNAMIC */
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 2,
	.bInterfaceClass	= USB_CLASS_HID,
	/* .bInterfaceSubClass	= DYNAMIC */
	/* .bInterfaceProtocol	= DYNAMIC */
	/* .iInterface		= DYNAMIC */
};

static struct hid_descriptor hidg_desc = {
	.bLength			= sizeof(hidg_desc),
	.bDescriptorType		= HID_DT_HID,
	.bcdHID				= __constant_cpu_to_le16(0x0101),
	.bCountryCode			= 0x00,
	.bNumDescriptors		= 0x1,
	/*.desc[0].bDescriptorType	= DYNAMIC */
	/*.desc[0].wDescriptorLenght	= DYNAMIC */
};

/* High-Speed Support */

static struct usb_endpoint_descriptor hidg_hs_in_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 4, /* FIXME: Add this field in the
				      * HID gadget configuration?
				      * (struct hidg_func_descriptor)
				      */
};

static struct usb_endpoint_descriptor hidg_hs_out_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 4, /* FIXME: Add this field in the
				      * HID gadget configuration?
				      * (struct hidg_func_descriptor)
				      */
};

static struct usb_descriptor_header *hidg_hs_descriptors[] = {
	(struct usb_descriptor_header *)&hidg_interface_desc,
	(struct usb_descriptor_header *)&hidg_desc,
	(struct usb_descriptor_header *)&hidg_hs_in_ep_desc,
	(struct usb_descriptor_header *)&hidg_hs_out_ep_desc,
	NULL,
};

/* Full-Speed Support */

static struct usb_endpoint_descriptor hidg_fs_in_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 10, /* FIXME: Add this field in the
				       * HID gadget configuration?
				       * (struct hidg_func_descriptor)
				       */
};

static struct usb_endpoint_descriptor hidg_fs_out_ep_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	/*.wMaxPacketSize	= DYNAMIC */
	.bInterval		= 10, /* FIXME: Add this field in the
				       * HID gadget configuration?
				       * (struct hidg_func_descriptor)
				       */
};

static struct usb_descriptor_header *hidg_fs_descriptors[] = {
	(struct usb_descriptor_header *)&hidg_interface_desc,
	(struct usb_descriptor_header *)&hidg_desc,
	(struct usb_descriptor_header *)&hidg_fs_in_ep_desc,
	(struct usb_descriptor_header *)&hidg_fs_out_ep_desc,
	NULL,
};


/*-------------------------------------------------------------------------*/
/*                              Char Device                                */
static ssize_t f_hidg_read(struct file *file, char __user *buffer,
			size_t count, loff_t *ptr)
{
	struct f_hidg *hidg = file->private_data;
	struct f_hidg_req_list *list;
	struct usb_request *req;
	unsigned long flags;
	int ret;

	if (!count)
		return 0;

	if (!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;

	spin_lock_irqsave(&hidg->spinlock, flags);

#define READ_COND (!list_empty(&hidg->completed_out_req))

	/* wait for at least one buffer to complete */
	while (!READ_COND) {
		spin_unlock_irqrestore(&hidg->spinlock, flags);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(hidg->read_queue, READ_COND))
			return -ERESTARTSYS;

		spin_lock_irqsave(&hidg->spinlock, flags);
	}

	/* pick the first one */
	list = list_first_entry(&hidg->completed_out_req,
				struct f_hidg_req_list, list);

	req = list->req;
	count = min_t(unsigned int, count, req->actual - list->pos);
	spin_unlock_irqrestore(&hidg->spinlock, flags);

	/* copy to user outside spinlock */
	count -= copy_to_user(buffer, req->buf + list->pos, count);
	list->pos += count;

	/*
	 * if this request is completely handled and transfered to
	 * userspace, remove its entry from the list and requeue it
	 * again. Otherwise, we will revisit it again upon the next
	 * call, taking into account its current read position.
	 */
	if (list->pos == req->actual) {
		spin_lock_irqsave(&hidg->spinlock, flags);
		list_del(&list->list);
		kfree(list);
		spin_unlock_irqrestore(&hidg->spinlock, flags);

		if (!list->ctl_ep) {
			req->length = hidg->report_length;
			ret = usb_ep_queue(hidg->out_ep, req, GFP_KERNEL);
			if (ret < 0)
				return ret;
		}
	}

	return count;
}

static void f_hidg_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_hidg *hidg = (struct f_hidg *)ep->driver_data;

	if (req->status != 0) {
		ERROR(hidg->func.config->cdev,
			"End Point Request ERROR: %d\n", req->status);
	}

	hidg->write_pending = 0;
	wake_up(&hidg->write_queue);
}
static ssize_t f_hidg_ioctl(struct file *file, const char __user *buffer,
				size_t count)
{
	struct f_hidg *hidg = file->private_data;
	ssize_t status = -ENOMEM;
	struct usb_composite_dev	*cdev = hidg->func.config->cdev;
	struct usb_request		*req  = cdev->req;
	unsigned char *buf = (unsigned char *)req->buf;
	count = min_t(unsigned int, count, hidg->report_length);
	status = copy_from_user(buf, buffer, count);

	if (status != 0) {
		ERROR(cdev,
			"copy_from_user error\n");
		return -EINVAL;
	}

	printk("buf[0]=%d,buf[1]=%d\n",buf[0],buf[1]);

	req->zero = 0;
	req->length = count;
	status = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (status < 0)
		ERROR(cdev, "usb_ep_queue error on ep0 status=%d,length=%d\n", status,req->length);
	return status;

}

static ssize_t f_hidg_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *offp)
{
	struct f_hidg *hidg = file->private_data;
	ssize_t status = -ENOMEM;

	if (!access_ok(VERIFY_READ, buffer, count))
		return -EFAULT;

	mutex_lock(&hidg->lock);

#define WRITE_COND (!hidg->write_pending)

	while (!WRITE_COND) {
		mutex_unlock(&hidg->lock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(hidg->write_queue, WRITE_COND))
			return -ERESTARTSYS;

		mutex_lock(&hidg->lock);
	}

	count = min_t(unsigned int, count, hidg->report_length);
	status = copy_from_user(hidg->req->buf, buffer, count);

	if (status != 0) {
		ERROR(hidg->func.config->cdev,
			"copy_from_user error\n");
		mutex_unlock(&hidg->lock);
		return -EINVAL;
	}

	hidg->req->status = 0;
	hidg->req->zero = 0;
	hidg->req->length = count;
	hidg->req->complete = f_hidg_req_complete;
	hidg->req->context = hidg;
	hidg->write_pending = 1;

	status = usb_ep_queue(hidg->in_ep, hidg->req, GFP_ATOMIC);
	if (status < 0) {
		ERROR(hidg->func.config->cdev,
			"usb_ep_queue error on int endpoint %zd\n", status);
		hidg->write_pending = 0;
		wake_up(&hidg->write_queue);
	} else {
		status = count;
	}

	mutex_unlock(&hidg->lock);

	return status;
}

static unsigned int f_hidg_poll(struct file *file, poll_table *wait)
{
	struct f_hidg *hidg = file->private_data;
	unsigned int ret = 0;

	poll_wait(file, &hidg->read_queue, wait);
	poll_wait(file, &hidg->write_queue, wait);

	if (WRITE_COND)
		ret |= (POLLOUT | POLLWRNORM);

	if (READ_COND)
		ret |= (POLLIN | POLLRDNORM);

	return ret;
}

#undef WRITE_COND
#undef READ_COND

static int f_hidg_release(struct inode *inode, struct file *fd)
{
	fd->private_data = NULL;
	return 0;
}

static int f_hidg_open(struct inode *inode, struct file *fd)
{
	struct f_hidg *hidg =
		container_of(inode->i_cdev, struct f_hidg, cdev);

	fd->private_data = hidg;

	return 0;
}

/*-------------------------------------------------------------------------*/
/*                                usb_function                             */

static struct usb_request *hidg_alloc_ep_req(struct usb_ep *ep, unsigned length)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (req) {
		req->length = length;
		req->buf = kmalloc(length, GFP_ATOMIC);
		if (!req->buf) {
			usb_ep_free_request(ep, req);
			req = NULL;
		}
	}

	return req;
}

static void hidg_set_report_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_hidg *hidg = (struct f_hidg *) req->context;
	struct usb_composite_dev *cdev = hidg->func.config->cdev;
	struct f_hidg_req_list *req_list;
	unsigned long flags;
	int status = req->status;

	if (status == -ECONNRESET || status == -ESHUTDOWN) {
		/* Probably caused by the usb_ep_dequeue. */
		/* Do not enqueue the req again. */
		WARNING(cdev, "error in usb request: %d\n", status);
		return;
	} else if (status) {
		ERROR(cdev, "hidg_set_report_complete: status(%d), %d/%d\n",
			status, req->actual, req->length);
	}

	req_list = kzalloc(sizeof(*req_list), GFP_ATOMIC);
	if (!req_list)
		return;

	req_list->req = req;
	/* Check if it is control ep0. */
	if (ep == cdev->gadget->ep0)
		req_list->ctl_ep = 1;

	spin_lock_irqsave(&hidg->spinlock, flags);
	list_add_tail(&req_list->list, &hidg->completed_out_req);
	spin_unlock_irqrestore(&hidg->spinlock, flags);

	wake_up(&hidg->read_queue);
}

static int hidg_setup(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct f_hidg			*hidg = func_to_hidg(f);
	struct usb_composite_dev	*cdev = f->config->cdev;
	struct usb_request		*req  = cdev->req;
	int status = 0;
	__u16 value, index, length;
	unsigned char *buf = (unsigned char *)req->buf;
	char **uevent_envp = NULL;
	char *hidevent1[2]   = { "HID_EVENT1=HID_VENDOR_EXT", NULL };
	char *hidevent2[2]   = { "HID_EVENT2=HID_DISP_ATTR", NULL };

	value	= __le16_to_cpu(ctrl->wValue);
	index	= __le16_to_cpu(ctrl->wIndex);
	length	= __le16_to_cpu(ctrl->wLength);

	VDBG(cdev, "hid_setup crtl_request : bRequestType:0x%x bRequest:0x%x "
		"Value:0x%x Index:0x%x Length:0x%x\n", ctrl->bRequestType, ctrl->bRequest,
		value, index, length);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_GET_REPORT):
		length = min_t(unsigned, length, hidg->report_length);
		/*TODO: we can send the whole get_report request to the app side to let the app check the content of the request. */
		if (((value >> 8) & 0xFF) == USB_HID_ReportType_Feature &&
			(value & 0xFF) == USBHID_VENDOR_EXT_REPORT_ID) {
			VDBG(cdev, "USBHID_VENDOR_EXT_REPORT_ID, length: %d\n", length);
			uevent_envp = hidevent1;
		} else if (((value >> 8) & 0xFF) == USB_HID_ReportType_Feature &&
			(value & 0xFF) == USBHID_DISP_ATTR_REPORT_ID) {
			VDBG(cdev, "USBHID_DISP_ATTR_REPORT_ID, length: %d\n", length);
			uevent_envp = hidevent2;
		} else {
			/* send an empty report */
			memset(req->buf, 0x0, length);
		}
		if (uevent_envp) {
			kobject_uevent_env(&hidg_config->dev->kobj, KOBJ_CHANGE, uevent_envp);
		} else {
			VDBG(cdev,"%s: did not send uevent \n", __func__);
			}
		return 0;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_GET_PROTOCOL):
		VDBG(cdev, "get_protocol\n");
		goto stall;
		break;

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_SET_REPORT):
		VDBG(cdev, "set_report | wLenght=%d\n", ctrl->wLength);
		req->context  = hidg;
		req->complete = hidg_set_report_complete;
		goto respond;
		break;

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8
		  | HID_REQ_SET_PROTOCOL):
		VDBG(cdev, "set_protocol\n");
		goto stall;
		break;

	case ((USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_INTERFACE) << 8
		  | USB_REQ_GET_DESCRIPTOR):
		switch (value >> 8) {
		case HID_DT_HID:
			VDBG(cdev, "USB_REQ_GET_DESCRIPTOR: HID\n");
			length = min_t(unsigned short, length,
						   hidg_desc.bLength);
			memcpy(req->buf, &hidg_desc, length);
			goto respond;
			break;
		case HID_DT_REPORT:
			VDBG(cdev, "USB_REQ_GET_DESCRIPTOR: REPORT\n");
			length = min_t(unsigned short, length,
						   hidg->report_desc_length);
			memcpy(req->buf, hidg->report_desc, length);
			goto respond;
			break;

		default:
			VDBG(cdev, "Unknown descriptor request 0x%x\n",
				 value >> 8);
			goto stall;
			break;
		}
		break;

	default:
		VDBG(cdev, "Unknown request 0x%x\n",
			 ctrl->bRequest);
		goto stall;
		break;
	}

stall:
	return -EOPNOTSUPP;

respond:
	req->zero = 0;
	req->length = length;
	status = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
	if (status < 0)
		ERROR(cdev, "usb_ep_queue error on ep0 %d\n", status);
	return status;
}

static void hidg_disable(struct usb_function *f)
{
	struct f_hidg *hidg = func_to_hidg(f);
	struct f_hidg_req_list *list, *next;

	usb_ep_disable(hidg->in_ep);
	hidg->in_ep->driver_data = NULL;

	usb_ep_disable(hidg->out_ep);
	hidg->out_ep->driver_data = NULL;

	list_for_each_entry_safe(list, next, &hidg->completed_out_req, list) {
		list_del(&list->list);
		kfree(list);
	}
}

static int hidg_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct usb_composite_dev		*cdev = f->config->cdev;
	struct f_hidg				*hidg = func_to_hidg(f);
	struct usb_request			*req = NULL;
	int i, status = 0;

	VDBG(cdev, "hidg_set_alt intf:%d alt:%d\n", intf, alt);

	if (hidg->in_ep != NULL) {
		/* restart endpoint */
		if (hidg->in_ep->driver_data != NULL)
			usb_ep_disable(hidg->in_ep);

		status = config_ep_by_speed(f->config->cdev->gadget, f,
					    hidg->in_ep);
		if (status) {
			ERROR(cdev, "config_ep_by_speed FAILED!\n");
			goto fail;
		}
		status = usb_ep_enable(hidg->in_ep);
		if (status < 0) {
			ERROR(cdev, "Enable IN endpoint FAILED!\n");
			goto fail;
		}
		hidg->in_ep->driver_data = hidg;
	}


	if (hidg->out_ep != NULL) {
		/* restart endpoint */
		if (hidg->out_ep->driver_data != NULL)
			usb_ep_disable(hidg->out_ep);

		status = config_ep_by_speed(f->config->cdev->gadget, f,
					    hidg->out_ep);
		if (status) {
			ERROR(cdev, "config_ep_by_speed FAILED!\n");
			goto fail;
		}
		status = usb_ep_enable(hidg->out_ep);
		if (status < 0) {
			ERROR(cdev, "Enable IN endpoint FAILED!\n");
			goto fail;
		}
		hidg->out_ep->driver_data = hidg;

		/*
		 * allocate a bunch of read buffers and queue them all at once.
		 */
		for (i = 0; i < hidg->qlen && status == 0; i++) {
			if (!hidg->out_reqs[i])
				hidg->out_reqs[i] =
						hidg_alloc_ep_req(hidg->out_ep, hidg->report_length);

			req = hidg->out_reqs[i];
			if (req) {
				req->complete = hidg_set_report_complete;
				req->context  = hidg;
				status = usb_ep_queue(hidg->out_ep, req,
						      GFP_ATOMIC);
				if (status)
					ERROR(cdev, "%s queue req --> %d\n",
						hidg->out_ep->name, status);
			} else {
				usb_ep_disable(hidg->out_ep);
				hidg->out_ep->driver_data = NULL;
				status = -ENOMEM;
				goto fail;
			}
		}
	}

fail:
	return status;
}

const struct file_operations f_hidg_fops = {
	.owner		= THIS_MODULE,
	.open		= f_hidg_open,
	.release	= f_hidg_release,
	.write		= f_hidg_write,
	.read		= f_hidg_read,
	.poll		= f_hidg_poll,
	.llseek		= noop_llseek,
	.unlocked_ioctl = f_hidg_ioctl,
};

static int hidg_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_ep		*ep;
	struct f_hidg		*hidg = func_to_hidg(f);
	int		status;
	dev_t		dev;

	/* allocate instance-specific interface IDs, and patch descriptors. */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	hidg_interface_desc.bInterfaceNumber = status;

	/* allocate instance-specific endpoints. */
	status = -ENODEV;
	ep = usb_ep_autoconfig(c->cdev->gadget, &hidg_fs_in_ep_desc);
	if (!ep)
		goto fail;
	ep->driver_data = c->cdev;	/* claim */
	hidg->in_ep = ep;

	ep = usb_ep_autoconfig(c->cdev->gadget, &hidg_fs_out_ep_desc);
	if (!ep)
		goto fail;
	ep->driver_data = c->cdev;	/* clain */
	hidg->out_ep = ep;

	/* preallocate request and buffer */
	status = -ENOMEM;
	hidg->req = usb_ep_alloc_request(hidg->in_ep, GFP_KERNEL);
	if (!hidg->req)
		goto fail;

	hidg->req->buf = kmalloc(hidg->report_length, GFP_KERNEL);
	if (!hidg->req->buf)
		goto fail;

	/* set descriptor dynamic values */
	hidg_interface_desc.bInterfaceSubClass = hidg->bInterfaceSubClass;
	hidg_interface_desc.bInterfaceProtocol = hidg->bInterfaceProtocol;
	hidg_hs_in_ep_desc.wMaxPacketSize = cpu_to_le16(hidg->report_length);
	hidg_fs_in_ep_desc.wMaxPacketSize = cpu_to_le16(hidg->report_length);
	hidg_hs_out_ep_desc.wMaxPacketSize = cpu_to_le16(hidg->report_length);
	hidg_fs_out_ep_desc.wMaxPacketSize = cpu_to_le16(hidg->report_length);
	hidg_desc.desc[0].bDescriptorType = HID_DT_REPORT;
	hidg_desc.desc[0].wDescriptorLength =
		cpu_to_le16(hidg->report_desc_length);

	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hidg_hs_in_ep_desc.bEndpointAddress =
			hidg_fs_in_ep_desc.bEndpointAddress;
		hidg_hs_out_ep_desc.bEndpointAddress =
			hidg_fs_out_ep_desc.bEndpointAddress;
	}

	f->fs_descriptors = hidg_fs_descriptors;
	f->hs_descriptors = hidg_hs_descriptors;

	mutex_init(&hidg->lock);
	spin_lock_init(&hidg->spinlock);
	init_waitqueue_head(&hidg->write_queue);
	init_waitqueue_head(&hidg->read_queue);
	INIT_LIST_HEAD(&hidg->completed_out_req);

	/* create char device */
	cdev_init(&hidg->cdev, &f_hidg_fops);
	dev = MKDEV(major, hidg->minor);
	status = cdev_add(&hidg->cdev, dev, 1);
	if (status)
		goto fail;

	device_create(hidg_class, NULL, dev, NULL, "%s%d", "hidg", hidg->minor);

	if (hidg_config)
		hidg_config->device = hidg->minor;

	return 0;

fail:
	ERROR(f->config->cdev, "hidg_bind FAILED\n");
	if (hidg->req != NULL) {
		kfree(hidg->req->buf);
		if (hidg->in_ep != NULL)
			usb_ep_free_request(hidg->in_ep, hidg->req);
	}

	/* usb_free_all_descriptors(f); */
	return status;
}

static void hidg_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_hidg *hidg = func_to_hidg(f);
	int i = 0;

	device_destroy(hidg_class, MKDEV(major, hidg->minor));
	cdev_del(&hidg->cdev);

	/* disable/free request and end point */
	usb_ep_disable(hidg->in_ep);
	usb_ep_dequeue(hidg->in_ep, hidg->req);
	kfree(hidg->req->buf);
	usb_ep_free_request(hidg->in_ep, hidg->req);

	usb_ep_disable(hidg->out_ep);
	for(i = 0; i < hidg->qlen; i++) {
		if (hidg->out_reqs[i]) {
			usb_ep_dequeue(hidg->out_ep, hidg->out_reqs[i]);
			kfree(hidg->out_reqs[i]->buf);
			usb_ep_free_request(hidg->out_ep, hidg->out_reqs[i]);
		}
	}

	/* usb_free_all_descriptors(f); */

	kfree(hidg->report_desc);
	kfree(hidg);
}

/*-------------------------------------------------------------------------*/
/*                                 Strings                                 */

#define CT_FUNC_HID_IDX 0

static struct usb_string ct_func_string_defs[] = {
	[CT_FUNC_HID_IDX].s = "HID Interface",
	{},			/* end of list */
};

static struct usb_gadget_strings ct_func_string_table = {
	.language = 0x0409,		/* en-US */
	.strings = ct_func_string_defs,
};

static struct usb_gadget_strings *ct_func_strings[] = {
	&ct_func_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/
/*                             usb_configuration                           */
int hidg_bind_config(struct usb_configuration *c,
			struct hidg_device_config *config,
			int index)
{
	struct f_hidg *hidg;
	int status;

	if (index >= minors)
		return -ENOENT;

	/* maybe allocate device-global string IDs, and patch descriptors */
	if (ct_func_string_defs[CT_FUNC_HID_IDX].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;

		ct_func_string_defs[CT_FUNC_HID_IDX].id = status;
		hidg_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	hidg = kzalloc(sizeof(*hidg), GFP_KERNEL);
	if (!hidg)
		return -ENOMEM;

	hidg->minor = index;

	if (config) {
		hidg->bInterfaceSubClass = config->bInterfaceSubClass;
		hidg->bInterfaceProtocol = config->bInterfaceProtocol;
		hidg->report_length = config->report_length;
		hidg->report_desc_length = config->report_desc_length;
		hidg->report_desc = kmemdup(config->report_desc,
						config->report_desc_length,
						GFP_KERNEL);
		if (!hidg->report_desc) {
			kfree(hidg);
			return -ENOMEM;
		}
	}

	hidg->func.name		= "hid";
	hidg->func.strings	= ct_func_strings;
	hidg->func.bind		= hidg_bind;
	hidg->func.unbind	= hidg_unbind;
	hidg->func.set_alt	= hidg_set_alt;
	hidg->func.disable	= hidg_disable;
	hidg->func.setup	= hidg_setup;

	/* this could be made configurable at some point */
	hidg->qlen		= 4;

	hidg_config = config;

	status = usb_add_function(c, &hidg->func);
	if (status)
		kfree(hidg);

	return status;
}

int hidg_ctrlrequest(struct usb_composite_dev *cdev,
				const struct usb_ctrlrequest *ctrl)
{
	int	value = -EOPNOTSUPP;
	u8 b_requestType = ctrl->bRequestType;
	u8 b_request = ctrl->bRequest;
	u16	w_index = le16_to_cpu(ctrl->wIndex);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_length = le16_to_cpu(ctrl->wLength);
	u8 *buf;
	int len;

	if (b_requestType == (USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
			b_request == USB_REQ_GET_DESCRIPTOR &&
			w_value == ((USB_DT_STRING << 8) | 33) &&
			w_index == 0x0409) {

		/* Lync get UCQ Request. */
		pr_info("hidg_ctrlrequest: Lync get UCQ String\n");
		if (hidg_config) {
			buf = cdev->req->buf;
			len = strlen(hidg_config->lync_ucq_string);
			len = utf8s_to_utf16s(hidg_config->lync_ucq_string, len,
				UTF16_LITTLE_ENDIAN, (wchar_t *) &buf[2],
				USB_STRING_MAX_LENGTH);
			buf[0] = (len + 1) * 2;
			buf[1] = USB_DT_STRING;

			value = buf[0];
		}

	}

	if (value >= 0) {
		cdev->req->zero = 0;
		cdev->req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "%s setup response queue error\n",
				__func__);
	}

	if (value == -EOPNOTSUPP)
		VDBG(cdev,
			"unknown class-specific control req "
			"%02x.%02x v%04x i%04x l%u\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	return value;
}

int ghid_setup(struct usb_gadget *g, int count)
{
	int status;
	dev_t dev;

	hidg_class = class_create(THIS_MODULE, "hidg");

	status = alloc_chrdev_region(&dev, 0, count, "hidg");
	if (!status) {
		major = MAJOR(dev);
		minors = count;
	}

	return status;
}

void ghid_cleanup(void)
{
	if (major) {
		unregister_chrdev_region(MKDEV(major, 0), minors);
		major = minors = 0;
	}

	class_destroy(hidg_class);
	hidg_class = NULL;
}
