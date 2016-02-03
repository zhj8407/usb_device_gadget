/*
 * f_audio.c -- USB Audio class function driver
  *
 * Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 * Copyright (C) 2008 Analog Devices, Inc
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/device.h>
#include <linux/usb/audio.h>
#include <linux/wait.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#define SAMPLE_RATE		48000
#define FRAMES_PER_MSEC	(SAMPLE_RATE / 1000)

#define OUT_EP_MAX_PACKET_SIZE	200

/* Number of requests to allocate */
#define OUT_EP_REQ_COUNT		4

/* string IDs are assigned dynamically */

#define UAC_STRING_IAD_IDX				0
#define UAC_STRING_CONTROL_IDX			1
#define UAC_STRING_STREAMING_IDX		2

static struct usb_string uac_en_us_strings[] = {
	[UAC_STRING_IAD_IDX].s = "UAC Audio",
	[UAC_STRING_CONTROL_IDX].s = "UAC Audio Control",
	[UAC_STRING_STREAMING_IDX].s = "UAC Audio Stream",
	{  }
};

static struct usb_gadget_strings uac_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = uac_en_us_strings,
};

static struct usb_gadget_strings *uac_function_strings[] = {
	&uac_stringtab,
	NULL,
};

static int generic_set_cmd(struct usb_audio_control *con, u8 cmd, int value);
static int generic_get_cmd(struct usb_audio_control *con, u8 cmd);

/*
 * DESCRIPTORS ... most are static, but strings and full
 * configuration descriptors are built on demand.
 */

/*
 * We have two interfaces- AudioControl and AudioStreaming
 * TODO: only supcard playback currently
 */
#define F_AUDIO_AC_INTERFACE	0
#define F_AUDIO_AS_INTERFACE	1
#define F_AUDIO_NUM_STREAM_INTERFACES	1
#define F_AUDIO_NUM_INTERFACES	2

/* B.3.0 Standard Interface Association Descriptor */
static struct usb_interface_assoc_descriptor iad_desc = {
	.bLength = sizeof iad_desc,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface = F_AUDIO_AC_INTERFACE,	/* Dynamic interface number */
	.bInterfaceCount = F_AUDIO_NUM_INTERFACES,
	.bFunctionClass = USB_CLASS_AUDIO,
	.bFunctionSubClass = USB_SUBCLASS_AUDIOCONTROL,
	.bFunctionProtocol = UAC_VERSION_1,
};

/* B.3.1  Standard AC Interface Descriptor */
static struct usb_interface_descriptor ac_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_AUDIO,
	.bInterfaceSubClass =	USB_SUBCLASS_AUDIOCONTROL,
};

DECLARE_UAC_AC_HEADER_DESCRIPTOR(2);

#define UAC_DT_AC_HEADER_LENGTH	UAC_DT_AC_HEADER_SIZE(F_AUDIO_NUM_STREAM_INTERFACES)
/* 1 input terminal, 1 output terminal and 1 feature unit */
#define UAC_DT_TOTAL_LENGTH (UAC_DT_AC_HEADER_LENGTH + UAC_DT_INPUT_TERMINAL_SIZE \
	+ UAC_DT_OUTPUT_TERMINAL_SIZE + UAC_DT_FEATURE_UNIT_SIZE(0))
/* B.3.2  Class-Specific AC Interface Descriptor */
static struct uac1_ac_header_descriptor_2 ac_header_desc = {
	.bLength =		UAC_DT_AC_HEADER_LENGTH,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_HEADER,
	.bcdADC =		__constant_cpu_to_le16(0x0100),
	.wTotalLength =		__constant_cpu_to_le16(UAC_DT_TOTAL_LENGTH),
	.bInCollection =	F_AUDIO_NUM_STREAM_INTERFACES,
	.baInterfaceNr = {
		[0] =		F_AUDIO_AS_INTERFACE,	/* Dynamic interface number */
	}
};

#define INPUT_TERMINAL_ID	1
static struct uac_input_terminal_descriptor input_terminal_desc = {
	.bLength =		UAC_DT_INPUT_TERMINAL_SIZE,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_INPUT_TERMINAL,
	.bTerminalID =		INPUT_TERMINAL_ID,
	.wTerminalType =	UAC_TERMINAL_STREAMING,
	.bAssocTerminal =	0,
	.wChannelConfig =	0x3,
};

DECLARE_UAC_FEATURE_UNIT_DESCRIPTOR(0);

#define FEATURE_UNIT_ID		2
static struct uac_feature_unit_descriptor_0 feature_unit_desc = {
	.bLength		= UAC_DT_FEATURE_UNIT_SIZE(0),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubtype	= UAC_FEATURE_UNIT,
	.bUnitID		= FEATURE_UNIT_ID,
	.bSourceID		= INPUT_TERMINAL_ID,
	.bControlSize		= 2,
	.bmaControls[0]		= (UAC_FU_MUTE | UAC_FU_VOLUME),
};

static struct usb_audio_control mute_control = {
	.list = LIST_HEAD_INIT(mute_control.list),
	.name = "Mute Control",
	.type = UAC_FU_MUTE,
	/* Todo: add real Mute control code */
	.set = generic_set_cmd,
	.get = generic_get_cmd,
};

static struct usb_audio_control volume_control = {
	.list = LIST_HEAD_INIT(volume_control.list),
	.name = "Volume Control",
	.type = UAC_FU_VOLUME,
	/* Todo: add real Volume control code */
	.set = generic_set_cmd,
	.get = generic_get_cmd,
};

static struct usb_audio_control_selector feature_unit = {
	.list = LIST_HEAD_INIT(feature_unit.list),
	.id = FEATURE_UNIT_ID,
	.name = "Mute & Volume Control",
	.type = UAC_FEATURE_UNIT,
	.desc = (struct usb_descriptor_header *)&feature_unit_desc,
};

#define OUTPUT_TERMINAL_ID	3
static struct uac1_output_terminal_descriptor output_terminal_desc = {
	.bLength		= UAC_DT_OUTPUT_TERMINAL_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubtype	= UAC_OUTPUT_TERMINAL,
	.bTerminalID		= OUTPUT_TERMINAL_ID,
	.wTerminalType		= UAC_OUTPUT_TERMINAL_SPEAKER,
	.bAssocTerminal		= FEATURE_UNIT_ID,
	.bSourceID		= FEATURE_UNIT_ID,
};

/* B.4.1  Standard AS Interface Descriptor */
static struct usb_interface_descriptor as_interface_alt_0_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bAlternateSetting =	0,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_AUDIO,
	.bInterfaceSubClass =	USB_SUBCLASS_AUDIOSTREAMING,
};

static struct usb_interface_descriptor as_interface_alt_1_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bAlternateSetting =	1,
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_AUDIO,
	.bInterfaceSubClass =	USB_SUBCLASS_AUDIOSTREAMING,
};

/* B.4.2  Class-Specific AS Interface Descriptor */
static struct uac1_as_header_descriptor as_header_desc = {
	.bLength =		UAC_DT_AS_HEADER_SIZE,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_AS_GENERAL,
	.bTerminalLink =	INPUT_TERMINAL_ID,
	.bDelay =		1,
	.wFormatTag =		UAC_FORMAT_TYPE_I_PCM,
};

DECLARE_UAC_FORMAT_TYPE_I_DISCRETE_DESC(1);

static struct uac_format_type_i_discrete_descriptor_1 as_type_i_desc = {
	.bLength =		UAC_FORMAT_TYPE_I_DISCRETE_DESC_SIZE(1),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype =	UAC_FORMAT_TYPE,
	.bFormatType =		UAC_FORMAT_TYPE_I,
	.bSubframeSize =	2,
	.bBitResolution =	16,
	.bSamFreqType =		1,
};

/* Standard ISO OUT Endpoint Descriptor for highspeed */
static struct usb_endpoint_descriptor hs_as_out_ep_desc  = {
	.bLength =		USB_DT_ENDPOINT_AUDIO_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_SYNC_ADAPTIVE
				| USB_ENDPOINT_XFER_ISOC,
	.wMaxPacketSize =	__constant_cpu_to_le16(OUT_EP_MAX_PACKET_SIZE),
	.bInterval =		4,	/* poll 1 per millisecond */
};

/* Standard ISO OUT Endpoint Descriptor for fullspeed */
static struct usb_endpoint_descriptor fs_as_out_ep_desc  = {
	.bLength =		USB_DT_ENDPOINT_AUDIO_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_SYNC_ADAPTIVE
				| USB_ENDPOINT_XFER_ISOC,
	.wMaxPacketSize =	__constant_cpu_to_le16(OUT_EP_MAX_PACKET_SIZE),
	.bInterval =		1,	/* poll 1 per millisecond */
};

/* Class-specific AS ISO OUT Endpoint Descriptor */
static struct uac_iso_endpoint_descriptor as_iso_out_desc = {
	.bLength =		UAC_ISO_ENDPOINT_DESC_SIZE,
	.bDescriptorType =	USB_DT_CS_ENDPOINT,
	.bDescriptorSubtype =	UAC_EP_GENERAL,
	.bmAttributes = 	1,
	.bLockDelayUnits =	1,
	.wLockDelay =		__constant_cpu_to_le16(1),
};

static struct usb_descriptor_header *hs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&ac_interface_desc,
	(struct usb_descriptor_header *)&ac_header_desc,

	(struct usb_descriptor_header *)&input_terminal_desc,
	(struct usb_descriptor_header *)&output_terminal_desc,
	(struct usb_descriptor_header *)&feature_unit_desc,

	(struct usb_descriptor_header *)&as_interface_alt_0_desc,
	(struct usb_descriptor_header *)&as_interface_alt_1_desc,
	(struct usb_descriptor_header *)&as_header_desc,

	(struct usb_descriptor_header *)&as_type_i_desc,

	(struct usb_descriptor_header *)&hs_as_out_ep_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	NULL,
};

static struct usb_descriptor_header *fs_audio_desc[] = {
	(struct usb_descriptor_header *)&iad_desc,
	(struct usb_descriptor_header *)&ac_interface_desc,
	(struct usb_descriptor_header *)&ac_header_desc,

	(struct usb_descriptor_header *)&input_terminal_desc,
	(struct usb_descriptor_header *)&output_terminal_desc,
	(struct usb_descriptor_header *)&feature_unit_desc,

	(struct usb_descriptor_header *)&as_interface_alt_0_desc,
	(struct usb_descriptor_header *)&as_interface_alt_1_desc,
	(struct usb_descriptor_header *)&as_header_desc,

	(struct usb_descriptor_header *)&as_type_i_desc,

	(struct usb_descriptor_header *)&fs_as_out_ep_desc,
	(struct usb_descriptor_header *)&as_iso_out_desc,
	NULL,
};

static struct snd_pcm_hardware audio_hw_info = {
	.info =			SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_BLOCK_TRANSFER,

	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min		= 2,
	.channels_max		= 2,
	.rate_min		= SAMPLE_RATE,
	.rate_max		= SAMPLE_RATE,

	.buffer_bytes_max =	1024 * 1024,
	.period_bytes_min =	64,
	.period_bytes_max =	512 * 1024,
	.periods_min =		2,
	.periods_max =		1024,
};

/*-------------------------------------------------------------------------*/

 #define MAX_STRING_NAME_LENGTH		32

struct audio_sink_config {
	int	card;
	int	device;
	struct device *dev;

	char iad_string[MAX_STRING_NAME_LENGTH];
	char ac_string[MAX_STRING_NAME_LENGTH];
	char as_string[MAX_STRING_NAME_LENGTH];
};

struct audio_dev {
	struct usb_function		func;
	struct snd_card			*card;
	struct snd_pcm			*pcm;
	struct snd_pcm_substream *substream;

	struct device			*dev;

	struct usb_ep			*out_ep;
	struct usb_request		*out_reqs[OUT_EP_REQ_COUNT];

	bool				ep_enabled;
	bool				stream_active;

	spinlock_t			lock;

	/* beginning, end and current position in our buffer */
	void				*buffer_start;
	void				*buffer_end;
	void				*buffer_pos;

	/* byte size of a "period" */
	unsigned int			period;
	/* bytes sent since last call to snd_pcm_period_elapsed */
	unsigned int			period_offset;

	/* Control Set command */
	struct list_head cs;
	u8 set_cmd;
	struct usb_audio_control *set_con;

	/* Dynamic interface number */
	u8 control_intf_num;
	u8 stream_intf_num;
};

static inline struct audio_dev *func_to_audio(struct usb_function *f)
{
	return container_of(f, struct audio_dev, func);
}

/*-------------------------------------------------------------------------*/

static struct usb_request *audio_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (!req)
		return NULL;

	req->buf = kzalloc(buffer_size, GFP_ATOMIC);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}
	req->length = buffer_size;
	return req;
}

static void audio_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static void audio_data_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct audio_dev *audio = req->context;
	unsigned pending;
	unsigned long flags;
	bool update_alsa = false;
	void *src, *dst;
	int status = req->status;
	struct snd_pcm_substream *substream = audio->substream;

	/* i/f shutting down */
	if (!audio->ep_enabled)
		return;

	if (status == -ECONNRESET || status == -ESHUTDOWN) {
		/* Probably caused by the usb_ep_dequeue. */
		/* Do not enqueue the req again. */
		return;
	} else if (status) {
		pr_err("audio_data_complete: status(%d), %d/%d\n",
			status, req->actual, req->length);
	}

	if (!audio->stream_active)
		goto exit;

	spin_lock_irqsave(&audio->lock, flags);

	src = req->buf;
	dst = audio->buffer_pos;
	pending = audio->buffer_pos - audio->buffer_start;

	audio->period_offset += req->actual;
	if (audio->period_offset >= audio->period) {
		update_alsa = true;
		audio->period_offset %= audio->period;
	}

	audio->buffer_pos = audio->buffer_start + ((pending + req->actual)
					% snd_pcm_lib_buffer_bytes(substream));

	spin_unlock_irqrestore(&audio->lock, flags);

	/* FIXME */
	if (unlikely((pending + req->actual) >
			snd_pcm_lib_buffer_bytes(substream))) {
		memcpy(dst, src, audio->buffer_end - dst);
		memcpy(audio->buffer_start, src + (audio->buffer_end - dst),
			audio->buffer_pos - audio->buffer_start);
	} else {
		memcpy(dst, src, req->actual);
	}

exit:
	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		pr_err("audio_data_complete: Failed to enqueue req\n");

	if (audio->stream_active && update_alsa)
		snd_pcm_period_elapsed(substream);

	return;
}

static void audio_control_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct audio_dev *audio = req->context;
	int status = req->status;
	u32 data = 0;

	switch (status) {

	case 0:				/* normal completion? */
		if (audio->set_con) {
			memcpy(&data, req->buf, req->length);
			audio->set_con->set(audio->set_con, audio->set_cmd,
					le16_to_cpu(data));
			audio->set_con = NULL;
		}
		break;
	default:
		break;
	}
}

static int audio_set_intf_req(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct audio_dev	*audio = func_to_audio(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	u8			id = ((le16_to_cpu(ctrl->wIndex) >> 8) & 0xFF);
	u16			len = le16_to_cpu(ctrl->wLength);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u8			con_sel = (w_value >> 8) & 0xFF;
	u8			cmd = (ctrl->bRequest & 0x0F);
	struct usb_audio_control_selector *cs;
	struct usb_audio_control *con;

	pr_debug("audio_set_intf_req: bRequest 0x%x, w_value 0x%04x, len %d, entity %d\n",
			ctrl->bRequest, w_value, len, id);

	list_for_each_entry(cs, &audio->cs, list) {
		if (cs->id == id) {
			list_for_each_entry(con, &cs->control, list) {
				if (con->type == con_sel) {
					audio->set_con = con;
					break;
				}
			}
			break;
		}
	}

	audio->set_cmd = cmd;
	req->context = audio;
	req->complete = audio_control_complete;

	return len;
}

static int audio_get_intf_req(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct audio_dev	*audio = func_to_audio(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u8			id = ((le16_to_cpu(ctrl->wIndex) >> 8) & 0xFF);
	u16			len = le16_to_cpu(ctrl->wLength);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u8			con_sel = (w_value >> 8) & 0xFF;
	u8			cmd = (ctrl->bRequest & 0x0F);
	struct usb_audio_control_selector *cs;
	struct usb_audio_control *con;

	DBG(cdev, "bRequest 0x%x, w_value 0x%04x, len %d, entity %d\n",
			ctrl->bRequest, w_value, len, id);

	list_for_each_entry(cs, &audio->cs, list) {
		if (cs->id == id) {
			list_for_each_entry(con, &cs->control, list) {
				if (con->type == con_sel && con->get) {
					value = con->get(con, cmd);
					break;
				}
			}
			break;
		}
	}

	req->context = audio;
	req->complete = audio_control_complete;
	len = min_t(size_t, sizeof(value), len);
	memcpy(req->buf, &value, len);

	return len;
}

static int audio_set_endpoint_req(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	int			value = -EOPNOTSUPP;
	u16			ep = le16_to_cpu(ctrl->wIndex);
	u16			len = le16_to_cpu(ctrl->wLength);
	u16			w_value = le16_to_cpu(ctrl->wValue);

	DBG(cdev, "bRequest 0x%x, w_value 0x%04x, len %d, endpoint %d\n",
			ctrl->bRequest, w_value, len, ep);

	switch (ctrl->bRequest) {
	case UAC_SET_CUR:
		value = len;
		break;

	case UAC_SET_MIN:
		break;

	case UAC_SET_MAX:
		break;

	case UAC_SET_RES:
		break;

	case UAC_SET_MEM:
		break;

	default:
		break;
	}

	return value;
}

static int audio_get_endpoint_req(struct usb_function *f,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	int value = -EOPNOTSUPP;
	u8 ep = ((le16_to_cpu(ctrl->wIndex) >> 8) & 0xFF);
	u16 len = le16_to_cpu(ctrl->wLength);
	u16 w_value = le16_to_cpu(ctrl->wValue);

	DBG(cdev, "bRequest 0x%x, w_value 0x%04x, len %d, endpoint %d\n",
			ctrl->bRequest, w_value, len, ep);

	switch (ctrl->bRequest) {
	case UAC_GET_CUR:
	case UAC_GET_MIN:
	case UAC_GET_MAX:
	case UAC_GET_RES:
		value = len;
		break;
	case UAC_GET_MEM:
		break;
	default:
		break;
	}

	return value;
}

static int
f_audio_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct audio_dev	*audio = func_to_audio(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	/* composite driver infrastructure handles everything; interface
	 * activation uses set_alt().
	 */
	switch (ctrl->bRequestType) {
	case USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE:
		value = audio_set_intf_req(f, ctrl);
		break;

	case USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE:
		value = audio_get_intf_req(f, ctrl);
		break;

	case USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_ENDPOINT:
		value = audio_set_endpoint_req(f, ctrl);
		break;

	case USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_ENDPOINT:
		value = audio_get_endpoint_req(f, ctrl);
		break;

	default:
		ERROR(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev, "audio req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		req->context = audio;
		req->complete = audio_control_complete;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "audio response on err %d\n", value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static char *audio_sink_active[2]    = { "AUDIO_SINK_STATE=ACTIVE", NULL };
static char *audio_sink_deactive[2]   = { "AUDIO_SINK_STATE=DEACTIVE", NULL };

static int f_audio_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct audio_dev	*audio = func_to_audio(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct usb_ep *out_ep = audio->out_ep;
	struct usb_request *req;
	int i = 0, err = 0;

	DBG(cdev, "intf %d, alt %d\n", intf, alt);

	/* No i/f has more than 2 alt settings */
	if (alt > 1) {
		pr_err("f_audio_set_alt: Wrong alternate settings: %d\n",
			alt);
		return -EINVAL;
	}

	if (intf == audio->control_intf_num) {
		/* Control I/f has only 1 AltSetting - 0 */
		if (alt) {
			pr_err("f_audio_set_alt: Wrong interface number: %d\n",
				intf);
			return -EINVAL;
		}
		return 0;
	}

	if (intf == audio->stream_intf_num) {
		config_ep_by_speed(gadget, f, out_ep);

		if (alt == 1) {
			usb_ep_enable(out_ep);
			out_ep->driver_data = audio;
			audio->ep_enabled = true;

			/*
			 * allocate a bunch of read buffers
			 * and queue them all at once.
			 */
			for (i = 0; i < OUT_EP_REQ_COUNT; i++) {
				if (audio->out_reqs[i]) {
					if(usb_ep_queue(out_ep, audio->out_reqs[i],
						GFP_ATOMIC))
						pr_err("f_audio_set_alt: Failed to enqueue the old req\n");
					continue;
				}

				req = audio_request_new(out_ep, OUT_EP_MAX_PACKET_SIZE);
				if (req == NULL) {
					pr_err("f_audio_set_alt: Failed to allocate the req\n");
					return -ENOMEM;
				}

				audio->out_reqs[i] = req;

				req->zero = 0;
				req->context = audio;
				req->length = OUT_EP_MAX_PACKET_SIZE;
				req->complete = audio_data_complete;

				if(usb_ep_queue(out_ep, audio->out_reqs[i],
					GFP_ATOMIC))
					pr_err("f_audio_set_alt: Failed to enqueue the new req\n");
			}

			/* Notify the userspace through uevent. */
			if (audio->dev)
				kobject_uevent_env(&audio->dev->kobj, KOBJ_CHANGE, audio_sink_active);
		} else {
			for (i = 0; i < OUT_EP_REQ_COUNT; i++) {
				req = audio->out_reqs[i];
				if (req) {
					usb_ep_dequeue(out_ep, req);
					audio_request_free(req, out_ep);
					audio->out_reqs[i] = NULL;
				}
			}
			if (audio->ep_enabled && usb_ep_disable(out_ep))
				pr_err("f_audio_set_alt: Failed to disable ep\n");
			audio->ep_enabled = false;

			/* Notify the userspace through uevent. */
			if (audio->dev)
				kobject_uevent_env(&audio->dev->kobj, KOBJ_CHANGE, audio_sink_deactive);
		}
	}

	return err;
}

static void f_audio_disable(struct usb_function *f)
{
	struct audio_dev	*audio = func_to_audio(f);

	pr_info("f_audio_disable: Disable Audio\n");
	usb_ep_disable(audio->out_ep);
}

/*-------------------------------------------------------------------------*/

static void f_audio_build_desc(struct audio_dev *audio)
{
	u8 *sam_freq;
	int rate;

	/* Set channel numbers */
	input_terminal_desc.bNrChannels = 2;
	as_type_i_desc.bNrChannels = 2;

	/* Set sample rates */
	rate = SAMPLE_RATE;
	sam_freq = as_type_i_desc.tSamFreq[0];
	memcpy(sam_freq, &rate, 3);

	/* Todo: Set Sample bits and other parameters */

	return;
}

/* audio function driver setup/binding */
static int
f_audio_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct audio_dev		*audio = func_to_audio(f);
	int			status;
	struct usb_ep		*ep = NULL;

	f_audio_build_desc(audio);

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	ac_interface_desc.bInterfaceNumber = status;
	audio->control_intf_num = status;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	as_interface_alt_0_desc.bInterfaceNumber = status;
	as_interface_alt_1_desc.bInterfaceNumber = status;
	audio->stream_intf_num = status;

	/* update the interface number to the header */
	iad_desc.bFirstInterface = ac_interface_desc.bInterfaceNumber;
	ac_header_desc.baInterfaceNr[0] = as_interface_alt_0_desc.bInterfaceNumber;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &fs_as_out_ep_desc);
	if (!ep)
		goto fail;
	audio->out_ep = ep;
	audio->out_ep->desc = &hs_as_out_ep_desc;
	ep->driver_data = audio;	/* claim */

	if (gadget_is_dualspeed(c->cdev->gadget))
		hs_as_out_ep_desc.bEndpointAddress =
			fs_as_out_ep_desc.bEndpointAddress;

	f->fs_descriptors = fs_audio_desc;
	f->hs_descriptors = hs_audio_desc;

	return 0;

fail:
	if (ep)
		ep->driver_data = NULL;
	return status;
}

static void
f_audio_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct audio_dev	*audio = func_to_audio(f);

	snd_card_free_when_closed(audio->card);
	audio->card = NULL;
	audio->pcm = NULL;
	audio->substream = NULL;
	audio->out_ep = NULL;

	uac_en_us_strings[UAC_STRING_IAD_IDX].id = 0;
}

static int audio_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_dev	*audio = substream->private_data;

	runtime->private_data = audio;
	runtime->hw = audio_hw_info;
	snd_pcm_limit_hw_rates(runtime);
	runtime->hw.channels_max = 2;

	audio->substream = substream;

	return 0;
}

static int audio_pcm_close(struct snd_pcm_substream *substream)
{
	struct audio_dev	*audio = substream->private_data;
	unsigned long flags;

	spin_lock_irqsave(&audio->lock, flags);
	audio->substream = NULL;
	spin_unlock_irqrestore(&audio->lock, flags);

	return 0;
}

static int audio_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);

	if (rate != SAMPLE_RATE)
		return -EINVAL;
	if (channels != 2)
		return -EINVAL;

	return snd_pcm_lib_alloc_vmalloc_buffer(substream,
		params_buffer_bytes(params));
}

static int audio_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int audio_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_dev	*audio = runtime->private_data;

	audio->period = snd_pcm_lib_period_bytes(substream);
	audio->period_offset = 0;
	audio->buffer_start = runtime->dma_area;
	audio->buffer_end = audio->buffer_start
		+ snd_pcm_lib_buffer_bytes(substream);
	audio->buffer_pos = audio->buffer_start;

	pr_info("audio_pcm_prepare: period_size: %d, buffer_size: %d\n",
		audio->period, snd_pcm_lib_buffer_bytes(substream));

	return 0;
}

static snd_pcm_uframes_t audio_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_dev	*audio = runtime->private_data;
	ssize_t bytes = audio->buffer_pos - audio->buffer_start;

	return bytes_to_frames(runtime, bytes);
}

static int audio_pcm_capture_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	struct audio_dev	*audio = substream->runtime->private_data;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&audio->lock, flags);

	/* Reset */
	audio->buffer_pos = 0;
	audio->period_offset = 0;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
			audio->stream_active = true;
			break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
			audio->stream_active = false;
			break;

		default:
			ret = -EINVAL;
	}

	spin_unlock_irqrestore(&audio->lock, flags);

	return ret;
}

static struct snd_pcm_ops audio_capture_ops = {
	.open		= audio_pcm_open,
	.close		= audio_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= audio_pcm_hw_params,
	.hw_free	= audio_pcm_hw_free,
	.prepare	= audio_pcm_prepare,
	.trigger	= audio_pcm_capture_trigger,
	.pointer	= audio_pcm_pointer,
};

/*-------------------------------------------------------------------------*/

static int generic_set_cmd(struct usb_audio_control *con, u8 cmd, int value)
{
	con->data[cmd] = value;

	return 0;
}

static int generic_get_cmd(struct usb_audio_control *con, u8 cmd)
{
	return con->data[cmd];
}

/* Todo: add more control selecotor dynamically */
int control_selector_init(struct audio_dev *audio)
{
	INIT_LIST_HEAD(&audio->cs);
	list_add(&feature_unit.list, &audio->cs);

	INIT_LIST_HEAD(&feature_unit.control);
	list_add(&mute_control.list, &feature_unit.control);
	list_add(&volume_control.list, &feature_unit.control);

	volume_control.data[UAC__CUR] = 0xffc0;
	volume_control.data[UAC__MIN] = 0xe3a0;
	volume_control.data[UAC__MAX] = 0xfff0;
	volume_control.data[UAC__RES] = 0x0030;

	return 0;
}

/**
 * audio_bind_config - add USB audio function to a configuration
 * @c: the configuration to supcard the USB audio function
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 */
int audio_bind_config(struct usb_configuration *c,
		struct audio_sink_config *config)
{
	struct audio_dev *audio;
	struct snd_card *card;
	struct snd_pcm *pcm;
	int status;

	config->card = -1;
	config->device = -1;

	/* allocate and initialize one new instance */
	audio = kzalloc(sizeof *audio, GFP_KERNEL);
	if (!audio)
		return -ENOMEM;

	status = snd_card_create(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
				THIS_MODULE, 0, &card);
	if (status)
		goto setup_fail;

	snd_card_set_dev(card, &c->cdev->gadget->dev);

	status = snd_pcm_new(card, "USB audio sink", 0, 0, 1, &pcm);
	if (status)
		goto pcm_fail;
	pcm->private_data = audio;
	pcm->info_flags = 0;
	audio->pcm = pcm;

	strlcpy(pcm->name, "USB gadget audio out", sizeof(pcm->name));

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &audio_capture_ops);
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
				NULL, 0, 64 * 1024);

	strlcpy(card->driver, "audio_sink", sizeof(card->driver));
	strlcpy(card->shortname, card->driver, sizeof(card->shortname));
	strlcpy(card->longname, "USB accessory audio sink",
		sizeof(card->longname));

	status = snd_card_register(card);
	if (status)
		goto register_fail;

	/* Update the strings. */
	if (strlen(config->iad_string) > 0)
		uac_en_us_strings[UAC_STRING_IAD_IDX].s = config->iad_string;
	if (strlen(config->ac_string) > 0)
		uac_en_us_strings[UAC_STRING_CONTROL_IDX].s = config->ac_string;
	if (strlen(config->as_string) > 0)
		uac_en_us_strings[UAC_STRING_STREAMING_IDX].s = config->as_string;

	if (uac_en_us_strings[UAC_STRING_IAD_IDX].id == 0) {
		status = usb_string_ids_tab(c->cdev, uac_en_us_strings);
		if (status)
			goto register_fail;
		iad_desc.iFunction =
			uac_en_us_strings[UAC_STRING_IAD_IDX].id;
		ac_interface_desc.iInterface =
			uac_en_us_strings[UAC_STRING_CONTROL_IDX].id;
		status = uac_en_us_strings[UAC_STRING_STREAMING_IDX].id;
		as_interface_alt_0_desc.iInterface = status;
		as_interface_alt_1_desc.iInterface = status;
	}

	spin_lock_init(&audio->lock);

	audio->func.name = "audio_sink";
	audio->func.strings = uac_function_strings;
	audio->func.bind = f_audio_bind;
	audio->func.unbind = f_audio_unbind;
	audio->func.set_alt = f_audio_set_alt;
	audio->func.setup = f_audio_setup;
	audio->func.disable = f_audio_disable;

	control_selector_init(audio);

	status = usb_add_function(c, &audio->func);
	if (status)
		goto add_fail;

	config->card = pcm->card->number;
	config->device = pcm->device;
	audio->card = card;
	audio->dev = config->dev;

	return status;

add_fail:
register_fail:
	/* snd_pcm_free(pcm); */
pcm_fail:
	snd_card_free(audio->card);
setup_fail:
	kfree(audio);
	return status;
}
