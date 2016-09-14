/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Benoit Goby <benoit@android.com>
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/of_platform.h>

#include "gadget_chips.h"

//#define PLCM_USB_AUDIO_SOURCE
//#define PLCM_USB_AUDIO_SINK
#define PLCM_USB_AUDIO_DUAL

#include "f_nvusb.c"

#ifdef PLCM_USB_AUDIO_SOURCE
#include "f_audio_source.c"
#endif

#ifdef PLCM_USB_AUDIO_SINK
#include "f_audio_sink.c"
#endif

#ifdef PLCM_USB_AUDIO_DUAL
#include "f_uac_plcm.c"
#endif

#include "f_hidg.c"
#include "f_webcam.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Polycom Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Polycom";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x095D
#define PRODUCT_ID		0x9200

struct plcm_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for plcm_usb_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct plcm_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct plcm_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct plcm_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct plcm_usb_function *);

	int (*bind_config)(struct plcm_usb_function *,
			   struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct plcm_usb_function *,
			      struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct plcm_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct plcm_usb_dev {
	struct plcm_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	bool connected;
	bool sw_connected;
	bool sw_configured;
	struct work_struct work;
	char ffs_aliases[256];
	unsigned short ffs_string_ids;
};

static struct class *plcm_usb_class;
static struct plcm_usb_dev *_plcm_usb_dev;
static int plcm_usb_bind_config(struct usb_configuration *c);
static void plcm_usb_unbind_config(struct usb_configuration *c);

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];
static char maxim_string[256];

/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_configuration plcm_usb_config_driver = {
	.label		= "plcm_usb",
	.unbind		= plcm_usb_unbind_config,
	.bConfigurationValue = 1,
};

static void plcm_usb_gstring_cleanup(struct plcm_usb_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_gadget_string_container *uc, *tmp;

	list_for_each_entry_safe(uc, tmp, &cdev->gstrings, list) {
		list_del(&uc->list);
		kfree(uc);
	}
	/* reserve unfreed string ids */
	cdev->next_string_id = ARRAY_SIZE(strings_dev) +
		dev->ffs_string_ids - 1;
}

static void plcm_usb_work(struct work_struct *data)
{
	struct plcm_usb_dev *dev = container_of(data, struct plcm_usb_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);

	/* Connected status is changed. */
	if (dev->connected != dev->sw_connected)
		uevent_envp = dev->connected ? connected : disconnected;

	/* Configured status is changed. */
	if (dev->connected && cdev->config && !dev->sw_configured)
		uevent_envp = configured;

	dev->sw_connected = dev->connected;
	dev->sw_configured = (cdev->config != NULL);
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (uevent_envp) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_debug("%s: did not send uevent (%d %d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected,
			 dev->sw_configured, cdev->config);
	}
}

static void plcm_usb_enable(struct plcm_usb_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (WARN_ON(!dev->disable_depth))
		return;

	if (--dev->disable_depth == 0) {
		usb_add_config(cdev, &plcm_usb_config_driver,
					plcm_usb_bind_config);
		usb_gadget_connect(cdev->gadget);
	}
}

static void plcm_usb_disable(struct plcm_usb_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;

	if (dev->disable_depth++ == 0) {
		usb_gadget_disconnect(cdev->gadget);
		/* Cancel pending control requests */
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
		usb_remove_config(cdev, &plcm_usb_config_driver);
		plcm_usb_gstring_cleanup(dev);
	}
}

static int hidg_function_init(struct plcm_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct hidg_device_config *config;

	config = kzalloc(sizeof(struct hidg_device_config), GFP_KERNEL);

	if (!config)
		return -ENOMEM;

	config->device = -1;
	config->dev = f->dev;

	f->config = config;

	ghid_setup(NULL, 1);

	return 0;
}

static void hidg_function_cleanup(struct plcm_usb_function *f)
{
	ghid_cleanup();
	kfree(f->config);
}

static int hidg_function_bind_config(struct plcm_usb_function *f,
			struct usb_configuration *c)
{
	struct hidg_device_config *config = f->config;

	return hidg_bind_config(c, config, 0);
}

static void hidg_function_unbind_config(struct plcm_usb_function *f,
			struct usb_configuration *c)
{
	struct hidg_device_config *config = f->config;

	config->device = -1;
}

static int hidg_function_ctrlrequest(struct plcm_usb_function *f,
			struct usb_composite_dev *cdev,
			const struct usb_ctrlrequest *ctrlrequest)
{
	return hidg_ctrlrequest(cdev, ctrlrequest);
}

static ssize_t hidg_device_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct hidg_device_config *config = f->config;

	/* print hidg device numbers */
	return sprintf(buf, "%d\n", config->device);
}

static DEVICE_ATTR(hidg_device, S_IRUGO, hidg_device_show, NULL);

#define HIDG_CONFIG_ATTR(field, format_string, max_value)				\
static ssize_t								\
hidg_ ## field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{	struct plcm_usb_function *f = dev_get_drvdata(dev);			\
	struct hidg_device_config *config = f->config;				\
	return sprintf(buf, format_string, config->field);		\
}									\
static ssize_t								\
hidg_ ## field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	struct plcm_usb_function *f = dev_get_drvdata(dev);			\
	struct hidg_device_config *config = f->config;				\
	int value;							\
	if ((sscanf(buf, format_string, &value) == 1) && 			\
			value <= max_value) {								\
		config->field = value;				\
		return size;						\
	}								\
	return -EINVAL;							\
}									\
static DEVICE_ATTR(hidg_ ## field,	\
	S_IRUGO | S_IWUSR,				\
	hidg_ ## field ## _show,		\
	hidg_ ## field ## _store		\
	);

static ssize_t hidg_report_desc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct hidg_device_config *config = f->config;
	memcpy(buf, config->report_desc, config->report_desc_length);

	return config->report_desc_length;
}

static ssize_t hidg_report_desc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct hidg_device_config *config = f->config;
	if (size > HID_REPORT_DESC_MAX_LENGTH)
		return -EINVAL;

	memcpy(config->report_desc, buf, size);

	/* Update the length of report descriptor. */
	config->report_desc_length = size;

	return size;
}

static DEVICE_ATTR(hidg_report_desc, S_IRUGO | S_IWUSR,
					hidg_report_desc_show,
					hidg_report_desc_store);


static ssize_t hidg_ucq_string_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct hidg_device_config *config = f->config;

	return sprintf(buf, "%s\n", config->lync_ucq_string);
}

static ssize_t hidg_ucq_string_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct hidg_device_config *config = f->config;
	if (size > sizeof(config->lync_ucq_string))
		return -EINVAL;

	return strlcpy(config->lync_ucq_string,
		buf, sizeof(config->lync_ucq_string));
}

static DEVICE_ATTR(hidg_ucq_string, S_IRUGO | S_IWUSR,
					hidg_ucq_string_show,
					hidg_ucq_string_store);

HIDG_CONFIG_ATTR(bInterfaceSubClass, "%d\n", 255)
HIDG_CONFIG_ATTR(bInterfaceProtocol, "%d\n", 255)
HIDG_CONFIG_ATTR(report_length, "%d\n", 255)
HIDG_CONFIG_ATTR(report_desc_length, "%d\n", HID_REPORT_DESC_MAX_LENGTH)

static struct device_attribute *hidg_function_attributes[] = {
	&dev_attr_hidg_device,
	&dev_attr_hidg_bInterfaceSubClass,
	&dev_attr_hidg_bInterfaceProtocol,
	&dev_attr_hidg_report_length,
	&dev_attr_hidg_report_desc_length,
	&dev_attr_hidg_report_desc,
	&dev_attr_hidg_ucq_string,
	NULL
};

static struct plcm_usb_function hidg_function = {
	.name		= "hidg",
	.init		= hidg_function_init,
	.cleanup	= hidg_function_cleanup,
	.bind_config	= hidg_function_bind_config,
	.unbind_config	= hidg_function_unbind_config,
	.ctrlrequest	= hidg_function_ctrlrequest,
	.attributes	= hidg_function_attributes,
};

#ifdef PLCM_USB_AUDIO_SOURCE

static int audio_source_function_init(struct plcm_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	config->dev = f->dev;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct plcm_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct plcm_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct plcm_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO, audio_source_pcm_show, NULL);

static ssize_t audio_source_iad_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;
	return sprintf(buf, "%s\n", config->iad_string);
}

static ssize_t audio_source_iad_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;
	if (size >= sizeof(config->iad_string))
		return -EINVAL;
	return strlcpy(config->iad_string, buf, sizeof(config->iad_string));
}

static DEVICE_ATTR(iad_string, S_IRUGO | S_IWUSR,
					audio_source_iad_show,
					audio_source_iad_store);

static ssize_t audio_source_ac_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;
	return sprintf(buf, "%s\n", config->ac_string);
}

static ssize_t audio_source_ac_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;
	if (size >= sizeof(config->ac_string))
		return -EINVAL;
	return strlcpy(config->ac_string, buf, sizeof(config->ac_string));
}

static DEVICE_ATTR(ac_string, S_IRUGO | S_IWUSR,
					audio_source_ac_show,
					audio_source_ac_store);

static ssize_t audio_source_as_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;
	return sprintf(buf, "%s\n", config->as_string);
}

static ssize_t audio_source_as_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;
	if (size >= sizeof(config->as_string))
		return -EINVAL;
	return strlcpy(config->as_string, buf, sizeof(config->as_string));
}

static DEVICE_ATTR(as_string, S_IRUGO | S_IWUSR,
					audio_source_as_show,
					audio_source_as_store);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	&dev_attr_iad_string,
	&dev_attr_ac_string,
	&dev_attr_as_string,
	NULL
};

static struct plcm_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};

#endif

#ifdef PLCM_USB_AUDIO_SINK
static int
audio_sink_function_init(struct plcm_usb_function *f,
		struct usb_composite_dev *cdev)
{
	struct audio_sink_config *config;

	config = kzalloc(sizeof(struct audio_sink_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	config->dev = f->dev;
	f->config = config;
	return 0;
}

static void audio_sink_function_cleanup(struct plcm_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
audio_sink_function_bind_config(struct plcm_usb_function *f,
		struct usb_configuration *c)
{
	return audio_bind_config(c, f->config);
}

static void
audio_sink_function_unbind_config(struct plcm_usb_function *f,
		struct usb_configuration *c)
{
	struct audio_sink_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_sink_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_sink_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO, audio_sink_pcm_show, NULL);

static ssize_t audio_sink_iad_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_sink_config *config = f->config;
	return sprintf(buf, "%s\n", config->iad_string);
}

static ssize_t audio_sink_iad_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_sink_config *config = f->config;
	if (size >= sizeof(config->iad_string))
		return -EINVAL;
	return strlcpy(config->iad_string, buf, sizeof(config->iad_string));
}

static DEVICE_ATTR(iad_string, S_IRUGO | S_IWUSR,
					audio_sink_iad_show,
					audio_sink_iad_store);

static ssize_t audio_sink_ac_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_sink_config *config = f->config;
	return sprintf(buf, "%s\n", config->ac_string);
}

static ssize_t audio_sink_ac_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_sink_config *config = f->config;
	if (size >= sizeof(config->ac_string))
		return -EINVAL;
	return strlcpy(config->ac_string, buf, sizeof(config->ac_string));
}

static DEVICE_ATTR(ac_string, S_IRUGO | S_IWUSR,
					audio_sink_ac_show,
					audio_sink_ac_store);

static ssize_t audio_sink_as_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_sink_config *config = f->config;
	return sprintf(buf, "%s\n", config->as_string);
}

static ssize_t audio_sink_as_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_sink_config *config = f->config;
	if (size >= sizeof(config->as_string))
		return -EINVAL;
	return strlcpy(config->as_string, buf, sizeof(config->as_string));
}

static DEVICE_ATTR(as_string, S_IRUGO | S_IWUSR,
					audio_sink_as_show,
					audio_sink_as_store);

static struct device_attribute *audio_sink_function_attributes[] = {
	&dev_attr_pcm,
	&dev_attr_iad_string,
	&dev_attr_ac_string,
	&dev_attr_as_string,
	NULL
};

static struct plcm_usb_function audio_sink_function = {
	.name		= "audio_sink",
	.init		= audio_sink_function_init,
	.cleanup	= audio_sink_function_cleanup,
	.bind_config	= audio_sink_function_bind_config,
	.unbind_config	= audio_sink_function_unbind_config,
	.attributes	= audio_sink_function_attributes,
};

#endif

#ifdef PLCM_USB_AUDIO_DUAL
static int
audio_dual_function_init(struct plcm_usb_function *f,
		struct usb_composite_dev *cdev)
{
	struct audio_dual_config *config;

	config = kzalloc(sizeof(struct audio_dual_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	config->tmode = -1;
	config->dev = f->dev;
	f->config = config;

	return 0;
}

static void audio_dual_function_cleanup(struct plcm_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
audio_dual_function_bind_config(struct plcm_usb_function *f,
		struct usb_configuration *c)
{
	return audio_composite_bind_config(c, f->config);
}

static void
audio_dual_function_unbind_config(struct plcm_usb_function *f,
		struct usb_configuration *c)
{

	struct audio_dual_config *config = f->config;

	config->card = -1;
	config->device = -1;

	audio_unbind_config(c);
}

static ssize_t audio_dual_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO, audio_dual_pcm_show, NULL);

static ssize_t audio_dual_iad_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	return sprintf(buf, "%s\n", config->iad_string);
}

static ssize_t audio_dual_iad_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	if (size >= sizeof(config->iad_string))
		return -EINVAL;
	return strlcpy(config->iad_string, buf, sizeof(config->iad_string));
}

static DEVICE_ATTR(iad_string, S_IRUGO | S_IWUSR,
					audio_dual_iad_show,
					audio_dual_iad_store);

static ssize_t audio_dual_ac_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	return sprintf(buf, "%s\n", config->audio_control_string);
}

static ssize_t audio_dual_ac_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	if (size >= sizeof(config->audio_control_string))
		return -EINVAL;
	return strlcpy(config->audio_control_string, buf, sizeof(config->audio_control_string));
}

static DEVICE_ATTR(audio_control_string, S_IRUGO | S_IWUSR,
					audio_dual_ac_show,
					audio_dual_ac_store);

static ssize_t audio_dual_test_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	return sprintf(buf, "%d\n", config->tmode);
}

static ssize_t audio_dual_test_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	int value;
	if (sscanf(buf,"%d\n",&value)==1)
	{
		config->tmode = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(audio_tmode, S_IRUGO | S_IWUSR,
					audio_dual_test_mode_show,
					audio_dual_test_mode_store);


static ssize_t audio_dual_as_out_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	return sprintf(buf, "%s\n", config->audio_out_stream_string);
}

static ssize_t audio_dual_as_out_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	if (size >= sizeof(config->audio_out_stream_string))
		return -EINVAL;
	return strlcpy(config->audio_out_stream_string, buf, sizeof(config->audio_out_stream_string));
}

static DEVICE_ATTR(audio_out_stream_string, S_IRUGO | S_IWUSR,
					audio_dual_as_out_show,
					audio_dual_as_out_store);

static ssize_t audio_dual_as_in_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	return sprintf(buf, "%s\n", config->audio_in_stream_string);
}

static ssize_t audio_dual_as_in_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct audio_dual_config *config = f->config;
	if (size >= sizeof(config->audio_in_stream_string))
		return -EINVAL;
	return strlcpy(config->audio_in_stream_string, buf, sizeof(config->audio_in_stream_string));
}

static DEVICE_ATTR(audio_in_stream_string, S_IRUGO | S_IWUSR,
					audio_dual_as_in_show,
					audio_dual_as_in_store);

static struct device_attribute *audio_dual_function_attributes[] = {
	&dev_attr_pcm,
	&dev_attr_iad_string,
	&dev_attr_audio_control_string,
	&dev_attr_audio_out_stream_string,
	&dev_attr_audio_in_stream_string,
	&dev_attr_audio_tmode,
	NULL
};

static struct plcm_usb_function audio_dual_function = {
	.name		= "audio_dual",
	.init		= audio_dual_function_init,
	.cleanup	= audio_dual_function_cleanup,
	.bind_config	= audio_dual_function_bind_config,
	.unbind_config	= audio_dual_function_unbind_config,
	.attributes	= audio_dual_function_attributes,
};

#endif

static int
webcam_function_init(struct plcm_usb_function *f,
		struct usb_composite_dev *cdev)
{
	struct webcam_config *config;

	config = kzalloc(sizeof(struct webcam_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->device = -1;
	config->dev = f->dev;

	config->interval = 1;
	config->maxpacket = 1024;
	config->maxburst = 0;
	config->headersize = UVC_DEFAULT_PAYLOAD_HEADER_SIZE;
	config->bulkmode = 0;
	config->bulksize = UVC_DEFAULT_BULK_REQ_BUFFER_SIZE;
	config->maxpayload = UVC_DEFAULT_MAX_PAYLOAD_SIZE;
	config->usbreqnums = UVC_MAX_NUM_REQUESTS;

	f->config = config;

	return 0;
}

static void webcam_function_cleanup(struct plcm_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
webcam_function_bind_config(struct plcm_usb_function *f,
		struct usb_configuration *c)
{
	return webcam_config_bind(c, f->config);
}

static void
webcam_function_unbind_config(struct plcm_usb_function *f,
		struct usb_configuration *c)
{

	struct webcam_config *config = f->config;

	config->device = -1;

	config->interval = 1;
	config->maxpacket = 1024;
	config->maxburst = 0;
	config->headersize = UVC_DEFAULT_PAYLOAD_HEADER_SIZE;
	config->bulkmode = 0;
	config->bulksize = UVC_DEFAULT_BULK_REQ_BUFFER_SIZE;
	config->maxpayload = UVC_DEFAULT_MAX_PAYLOAD_SIZE;
	config->usbreqnums = UVC_MAX_NUM_REQUESTS;

	webcam_config_unbind(c);
}

static ssize_t webcam_device_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct plcm_usb_function *f = dev_get_drvdata(dev);
	struct webcam_config *config = f->config;

	/* print webcam device numbers */
	return sprintf(buf, "%d\n", config->device);
}

static DEVICE_ATTR(webcam_device, S_IRUGO, webcam_device_show, NULL);

#define WEBCAM_CONFIG_ATTR(field, format_string, min_value, max_value)				\
static ssize_t								\
webcam_ ## field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{	struct plcm_usb_function *f = dev_get_drvdata(dev);			\
	struct webcam_config *config = f->config;				\
	return sprintf(buf, format_string, config->field);		\
}									\
static ssize_t								\
webcam_ ## field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	struct plcm_usb_function *f = dev_get_drvdata(dev);			\
	struct webcam_config *config = f->config;				\
	int value;							\
	if ((sscanf(buf, format_string, &value) == 1) && 			\
			value >= min_value &&								\
			value <= max_value) {								\
		config->field = value;				\
		return size;						\
	}								\
	return -EINVAL;							\
}									\
static DEVICE_ATTR(webcam_ ## field,	\
	S_IRUGO | S_IWUSR,				\
	webcam_ ## field ## _show,		\
	webcam_ ## field ## _store		\
	);

#define WEBCAM_CONFIG_STRING_ATTR(field)			\
static ssize_t										\
webcam_ ## field ## _string_show(struct device *dev, struct device_attribute *attr,	\
		char *buf)									\
{													\
	struct plcm_usb_function *f = dev_get_drvdata(dev);		\
	struct webcam_config *config = f->config;				\
	return sprintf(buf, "%s\n", config->video_ ## field ## _string);	\
}													\
static ssize_t										\
webcam_ ## field ## _string_store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{													\
	struct plcm_usb_function *f = dev_get_drvdata(dev);		\
	struct webcam_config *config = f->config;				\
	if (size >= sizeof(config->video_ ## field ## _string))	\
		return -EINVAL;										\
	return strlcpy(config->video_ ## field ## _string, buf,	\
			sizeof(config->video_ ## field ## _string));	\
}													\
static DEVICE_ATTR(webcam_ ## field ## _string,	\
	S_IRUGO | S_IWUSR,				\
	webcam_ ## field ## _string_show,		\
	webcam_ ## field ## _string_store		\
	);

WEBCAM_CONFIG_ATTR(interval, "%d\n", 1, 16)
WEBCAM_CONFIG_ATTR(maxpacket, "%d\n", 1, 3072)
WEBCAM_CONFIG_ATTR(maxburst, "%d\n", 0, 15)
WEBCAM_CONFIG_ATTR(headersize, "%d\n", 2, 255)
WEBCAM_CONFIG_ATTR(bulkmode, "%d\n", 0, 1)
WEBCAM_CONFIG_ATTR(bulksize, "%d\n", 512, 16384)
WEBCAM_CONFIG_ATTR(maxpayload, "%x\n", 0, 0xFFFFFFFF)
WEBCAM_CONFIG_ATTR(usbreqnums, "%d\n", 1, UVC_MAX_NUM_REQUESTS)

WEBCAM_CONFIG_STRING_ATTR(iad)
WEBCAM_CONFIG_STRING_ATTR(control)
WEBCAM_CONFIG_STRING_ATTR(stream)

static struct device_attribute *webcam_function_attributes[] = {
	&dev_attr_webcam_device,
	&dev_attr_webcam_interval,
	&dev_attr_webcam_maxpacket,
	&dev_attr_webcam_maxburst,
	&dev_attr_webcam_headersize,
	&dev_attr_webcam_bulkmode,
	&dev_attr_webcam_bulksize,
	&dev_attr_webcam_maxpayload,
	&dev_attr_webcam_usbreqnums,
	&dev_attr_webcam_iad_string,
	&dev_attr_webcam_control_string,
	&dev_attr_webcam_stream_string,
	NULL
};

static struct plcm_usb_function webcam_function = {
	.name		= "webcam",
	.init		= webcam_function_init,
	.cleanup	= webcam_function_cleanup,
	.bind_config	= webcam_function_bind_config,
	.unbind_config	= webcam_function_unbind_config,
	.attributes	= webcam_function_attributes,
};

static int
nvusb_function_init(struct plcm_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return 0;
}

static void nvusb_function_cleanup(struct plcm_usb_function *f)
{
}

static int
nvusb_function_bind_config(struct plcm_usb_function *f,
		struct usb_configuration *c)
{
	return nvusb_bind_config(c);
}

static struct plcm_usb_function nvusb_function = {
	.name		= "nvusb",
	.init		= nvusb_function_init,
	.cleanup	= nvusb_function_cleanup,
	.bind_config	= nvusb_function_bind_config,
};

static struct plcm_usb_function *supported_functions[] = {
	&hidg_function,
#ifdef PLCM_USB_AUDIO_SOURCE
	&audio_source_function,
#endif
#ifdef PLCM_USB_AUDIO_SINK
	&audio_sink_function,
#endif
#ifdef PLCM_USB_AUDIO_DUAL
	&audio_dual_function,
#endif
	&webcam_function,
	&nvusb_function,
	NULL
};


static int plcm_usb_init_functions(struct plcm_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct plcm_usb_dev *dev = _plcm_usb_dev;
	struct plcm_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err;
	int index = 0;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(plcm_usb_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(plcm_usb_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void plcm_usb_cleanup_functions(struct plcm_usb_function **functions)
{
	struct plcm_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(plcm_usb_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
plcm_usb_bind_enabled_functions(struct plcm_usb_dev *dev,
			       struct usb_configuration *c)
{
	struct plcm_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
plcm_usb_unbind_enabled_functions(struct plcm_usb_dev *dev,
			       struct usb_configuration *c)
{
	struct plcm_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int plcm_usb_enable_function(struct plcm_usb_dev *dev, char *name)
{
	struct plcm_usb_function **functions = dev->functions;
	struct plcm_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list,
						&dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/plcm_usb/plcm%d/ interface */

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct plcm_usb_dev *dev = dev_get_drvdata(pdev);
	struct plcm_usb_function *f;
	char *buff = buf;

	mutex_lock(&dev->mutex);

	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += sprintf(buff, "%s,", f->name);

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct plcm_usb_dev *dev = dev_get_drvdata(pdev);
	char *name;
	char buf[256], *b;
	int err;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	INIT_LIST_HEAD(&dev->enabled_functions);

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (!name)
			continue;

		err = plcm_usb_enable_function(dev, name);
		if (err)
			pr_err("plcm_usb: Cannot enable '%s' (%d)",
							   name, err);
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct plcm_usb_dev *dev = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct plcm_usb_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct plcm_usb_function *f;
	int enabled = 0;


	if (!cdev)
		return -ENODEV;

	mutex_lock(&dev->mutex);

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		/*
		 * Update values in composite driver's copy of
		 * device descriptor.
		 */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->enable)
				f->enable(f);
		}
		plcm_usb_enable(dev);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		plcm_usb_disable(dev);
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->disable)
				f->disable(f);
		}
		dev->enabled = false;
	} else {
		pr_err("plcm_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}

	mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct plcm_usb_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "CONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected)
		state = "DISCONNECTED";
	else if (cdev->config)
		state = "CONFIGURED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return sprintf(buf, "%s\n", state);
}

#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, "%s", buffer);				\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer))					\
		return -EINVAL;						\
	return strlcpy(buffer, buf, sizeof(buffer));			\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR_MANUFACTURER(field, buffer)		\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, "%s", buffer);				\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int ret;							\
	if (size >= (sizeof(buffer) - strlen(maxim_string)))		\
		return -EINVAL;						\
	ret = strlcpy(buffer, buf, sizeof(buffer));			\
	strncat(buffer, maxim_string, strlen(maxim_string));		\
	return ret;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR_MANUFACTURER(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show,
						 functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);

static struct device_attribute *plcm_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	NULL
};

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int plcm_usb_bind_config(struct usb_configuration *c)
{
	struct plcm_usb_dev *dev = _plcm_usb_dev;
	int ret = plcm_usb_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void plcm_usb_unbind_config(struct usb_configuration *c)
{
	struct plcm_usb_dev *dev = _plcm_usb_dev;

	plcm_usb_unbind_enabled_functions(dev, c);
}

/* set maximum and minimum voltage for maxim */
static void plcm_usb_maxim14675_set_voltage(void)
{
	unsigned int min_voltage = 5000;
	unsigned int max_voltage;
	unsigned int voltage = 0;
	char buf[20];
	struct device_node *np;

	np = of_find_node_by_path("/usb-devices/maxim-charger");
	if (np  && !of_property_read_u32(np,
			"maxim,max-board-vbus-voltage-uv",
			&voltage))
		pr_info("%s: max_voltage (vbus) = %d\n",
			__func__, voltage);

	/* convert DT microvolts to millivolts */
	voltage = voltage/1000;
	if (voltage < min_voltage)
		max_voltage = min_voltage;
	else if (voltage > 20000)
		max_voltage = 20000;
	else
		max_voltage = voltage;

	pr_info("%s: max_voltage (vbus) = %d\n",
			__func__, max_voltage);
	min_voltage = ((min_voltage - 2000) / 1000) * 20;
	max_voltage = ((max_voltage - 2000) / 1000) * 20;

	memset(buf, 0, sizeof(buf));
	strcpy(maxim_string, "    gr8rFiV8US");
	snprintf(buf, 4, "%03X", max_voltage);
	strncat(maxim_string, buf, 3);
	snprintf(buf, 4, "%03X", min_voltage);
	strncat(maxim_string, buf, 3);
	strncat(maxim_string, "00", 2);
}

static int plcm_usb_bind(struct usb_composite_dev *cdev)
{
	struct plcm_usb_dev *dev = _plcm_usb_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			id, ret;

	/*
	 * Start disconnected. Userspace will connect the gadget once
	 * it is done configuring the functions.
	 */
	usb_gadget_disconnect(gadget);

	ret = plcm_usb_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

	/* maxim 14675 voltage */
	plcm_usb_maxim14675_set_voltage();

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	/* Default strings - should be updated by userspace */
	strncpy(manufacturer_string, "Polycom", sizeof(manufacturer_string)-1);
	strncat(manufacturer_string, maxim_string, strlen(maxim_string));
	strncpy(product_string, "CX Series", sizeof(product_string) - 1);
	strncpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	dev->cdev = cdev;

	return 0;
}

static int plcm_usb_unbind(struct usb_composite_dev *cdev)
{
	struct plcm_usb_dev *dev = _plcm_usb_dev;

	cancel_work_sync(&dev->work);
	plcm_usb_cleanup_functions(dev->functions);
	return 0;
}

/* HACK: we needs to override setup for accessory to work */
static int (*composite_setup_func)(struct usb_gadget *gadget, const struct usb_ctrlrequest *c);

static int
plcm_usb_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct plcm_usb_dev		*dev = _plcm_usb_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct plcm_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;

	req->zero = 0;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	if (value < 0)
		value = composite_setup_func(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		pr_debug("%s: connected\n", __func__);
		schedule_work(&dev->work);
	} else if (c->bRequest == USB_REQ_SET_CONFIGURATION &&
						cdev->config) {
		pr_debug("%s: configured\n", __func__);
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void plcm_usb_disconnect(struct usb_composite_dev *cdev)
{
	struct plcm_usb_dev *dev = _plcm_usb_dev;
	/*
	 * In the design of tegra_udc, this disconnect callback will
	 * be invoked while receiving the BUS RESET from USB Host(HUB).
	 * The BUS Reset does not mean the usb disconnection. So we
	 * should not set the connection to 0 here.
	 *
	 * Instread the configuration will be reset in composite dev,
	 * then the sw_configured should be set to 0.
	 */
	pr_info("%s: curr sw_configured: %d\n", __func__, dev->sw_configured);

	dev->sw_configured = 0;
	schedule_work(&dev->work);
}

static void plcm_usb_resume(struct usb_composite_dev *cdev)
{
	pr_info("%s: invoked\n", __func__);
}

static void plcm_usb_suspend(struct usb_composite_dev *cdev)
{
	struct plcm_usb_dev *dev = _plcm_usb_dev;
	pr_info("%s: curr connected: %d\n", __func__, dev->connected);

	dev->connected = 0;
	schedule_work(&dev->work);
}

static struct usb_composite_driver plcm_usb_driver = {
	.name		= "plcm_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= plcm_usb_bind,
	.unbind		= plcm_usb_unbind,
	.disconnect	= plcm_usb_disconnect,
	.resume		= plcm_usb_resume,
	.suspend	= plcm_usb_suspend,
	.max_speed	= USB_SPEED_HIGH,
};

static int plcm_usb_create_device(struct plcm_usb_dev *dev)
{
	struct device_attribute **attrs = plcm_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(plcm_usb_class, NULL,
					MKDEV(0, 0), NULL, "plcm0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(plcm_usb_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}


static int __init init(void)
{
	struct plcm_usb_dev *dev;
	int err;

	plcm_usb_class = class_create(THIS_MODULE, "plcm_usb");
	if (IS_ERR(plcm_usb_class))
		return PTR_ERR(plcm_usb_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto err_dev;
	}

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, plcm_usb_work);
	mutex_init(&dev->mutex);

	err = plcm_usb_create_device(dev);
	if (err) {
		pr_err("%s: failed to create plcm usb device %d", __func__, err);
		goto err_create;
	}

	_plcm_usb_dev = dev;

	err = usb_composite_probe(&plcm_usb_driver);
	if (err) {
		pr_err("%s: failed to probe driver %d", __func__, err);
		goto err_probe;
	}

	/* HACK: exchange composite's setup with ours */
	composite_setup_func = plcm_usb_driver.gadget_driver.setup;
	plcm_usb_driver.gadget_driver.setup = plcm_usb_setup;

	return 0;

err_probe:
	device_destroy(plcm_usb_class, dev->dev->devt);
err_create:
	kfree(dev);
err_dev:
	class_destroy(plcm_usb_class);
	return err;
}
late_initcall(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&plcm_usb_driver);
	class_destroy(plcm_usb_class);
	kfree(_plcm_usb_dev);
	_plcm_usb_dev = NULL;
}
module_exit(cleanup);
