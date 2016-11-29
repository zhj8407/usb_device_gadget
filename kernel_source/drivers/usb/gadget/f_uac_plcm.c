/*
 * f_uac_plcm.c -- UAC 1.0 echo-cancelling speakerphone (0x0405)
 * derived from f_uac2.c and f_uac1.c
 *
 * Copyright (C) 2014 Polycom, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/usb/audio.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "f_uac_plcm.h"

#if 1
#define pr_trace(fmt, ...) \
	printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_trace(fmt, ...) \
	no_printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#endif

/* Playback(USB-IN) Default Stereo - Fl/Fr */
static int p_chmask = 0x1;
module_param(p_chmask, uint, S_IRUGO);
MODULE_PARM_DESC(p_chmask, "Playback Channel Mask");

/* Playback Default 48 KHz */
static int p_srate = 48000;
module_param(p_srate, uint, S_IRUGO);
MODULE_PARM_DESC(p_srate, "Playback Sampling Rate");

/* Playback Default 16bits/sample */
static int p_ssize = 2;
module_param(p_ssize, uint, S_IRUGO);
MODULE_PARM_DESC(p_ssize, "Playback Sample Size(bytes)");

/* Capture(USB-OUT) Default Stereo - Fl/Fr */
static int c_chmask = 0x1;
module_param(c_chmask, uint, S_IRUGO);
MODULE_PARM_DESC(c_chmask, "Capture Channel Mask");

/* Capture Default 64 KHz */
static int c_srate = 48000;
module_param(c_srate, uint, S_IRUGO);
MODULE_PARM_DESC(c_srate, "Capture Sampling Rate");

/* Capture Default 16bits/sample */
static int c_ssize = 2;
module_param(c_ssize, uint, S_IRUGO);
MODULE_PARM_DESC(c_ssize, "Capture Sample Size(bytes)");

#define MIC_NUM_CHANNELS 1
#define SPK_NUM_CHANNELS 1
#define MIC_CHANNEL_CONFIG 0
#define SPK_CHANNEL_CONFIG 0
#define MIC_SAMPLE_RATE 32000
#define SPK_SAMPLE_RATE 32000
#define MIC_EP_MAX_PACKET_SIZE	64		/* MIC_SAMPLE_RATE * MIC_NUM_CHANNELS * 2 byte * 0.0001 msec */
#define SPK_EP_MAX_PACKET_SIZE	64

/* Keep everyone on toes */
#define USB_XFERS	2

const char *uac_plcm_name = "snd_uac_plcm";
static char *uac_sink_active[2]    = { "AUDIO_SINK_STATE=ACTIVE", NULL };
static char *uac_sink_deactive[2]   = { "AUDIO_SINK_STATE=DEACTIVE", NULL };
static char *uac_source_active[2]    = { "AUDIO_SOURCE_STATE=ACTIVE", NULL };
static char *uac_source_deactive[2]   = { "AUDIO_SOURCE_STATE=DEACTIVE", NULL };
static char *uac_suspend[2]	= { "AUDIO_SOURCE_STATE=SUSPEND", NULL };

#ifndef MAX_STRING_NAME_LENGTH
#define MAX_STRING_NAME_LENGTH		64
#endif

struct audio_dual_config {
	int	card;
	int	device;
	int tmode;
	struct device *dev;

	char iad_string[MAX_STRING_NAME_LENGTH];
	char audio_control_string[MAX_STRING_NAME_LENGTH];
	char audio_out_stream_string[MAX_STRING_NAME_LENGTH];
	char audio_in_stream_string[MAX_STRING_NAME_LENGTH];
};

struct uac_plcm_req {
	struct uac_plcm_rtd_params *pp; /* parent param */
	struct usb_request *req;
};

struct uac_plcm_rtd_params {
	struct snd_uac_plcm_chip *uac_plcmc; /* parent chip */
	bool ep_enabled; /* if the ep is enabled */
	/* Size of the ring buffer */
	size_t dma_bytes;
	unsigned char *dma_area;

	struct snd_pcm_substream *ss;

	ssize_t hw_ptr; /* Ring buffer */

	void *rbuf;

	size_t period_size;

	unsigned max_psize;
	struct uac_plcm_req ureq[USB_XFERS];

	spinlock_t lock;

	/* Control Set command */
	struct list_head cs;
	u8 set_cmd;
	struct usb_audio_control *set_con;

	/* Control event */
	struct uac_ctrl_event ctrl_event;
	u8 ctrl_event_set;
};

struct snd_uac_plcm_chip {
	struct platform_device pdev;
	struct platform_driver pdrv;

	struct uac_plcm_rtd_params pm_prm;
	struct uac_plcm_rtd_params cs_prm;

	struct snd_card *card;
	struct snd_pcm *pcm;
};

#define BUFF_SIZE_MAX	(PAGE_SIZE * 16)
#define PRD_SIZE_MAX	PAGE_SIZE
#define MIN_PERIODS	4

static struct snd_pcm_hardware uac_plcm_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER
		| SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID
		| SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.periods_max = BUFF_SIZE_MAX / PRD_SIZE_MAX,
	.buffer_bytes_max = BUFF_SIZE_MAX,
	.period_bytes_max = PRD_SIZE_MAX,
	.periods_min = MIN_PERIODS,
};

/* --------- UAC Control Event Structures ------------ */

struct uac_kevent_list {
	struct list_head	list;
	struct uac_ctrl_event	event;
};

struct audio_dev {
	u8 ac_intf, ac_alt;
	u8 as_out_intf, as_out_alt;
	u8 as_in_intf, as_in_alt;

	struct usb_ep *in_ep, *out_ep;
	struct usb_function func;

	/* The ALSA Sound Card it represents on the USB-Client side */
	struct snd_uac_plcm_chip uac_plcmc;

	/* Stores the state from usb host */
	unsigned suspended : 1;
	unsigned sink_status_changed: 1;
	unsigned sink_active : 1;
	unsigned source_status_changed: 1;
	unsigned source_active : 1;

	/* Protecting the status change request */
	spinlock_t			status_lock;

	/* character device */
	struct miscdevice miscdev;

	/* Work queue for uevent notification. */
	struct work_struct	status_notify_work;

	/* Protecting the uac control events list. */
	spinlock_t			ctrl_events_lock;

	/* Dequeueable event */
	struct list_head	ctrl_events_avail;
	/* Number of available events. */
	unsigned int		nctrl_events_avail;
	/* Waiting for available events. */
	wait_queue_head_t	ctrl_events_wait;
};

static struct audio_dev *agdev_g;
static struct audio_dual_config *uac_plcm_config = NULL;

static inline struct audio_dev *func_to_agdev(struct usb_function *f)
{
	return container_of(f, struct audio_dev, func);
}

static inline struct audio_dev *uac_plcmc_to_agdev(struct snd_uac_plcm_chip *u)
{
	return container_of(u, struct audio_dev, uac_plcmc);
}

static inline struct snd_uac_plcm_chip *pdev_to_uac_plcmc(struct platform_device *p)
{
	return container_of(p, struct snd_uac_plcm_chip, pdev);
}

/* --------- UAC Control Event Handlers ------------ */
int uac_ctrl_event_queue(struct audio_dev *agdev, const struct uac_ctrl_event *ev)
{
	unsigned long flags;
	struct uac_kevent_list *kevent;

	spin_lock_irqsave(&agdev->ctrl_events_lock, flags);

	if (agdev->nctrl_events_avail >= MAX_UAC_CTRL_EVENTS_COUNT) {
		list_for_each_entry_reverse(kevent, &agdev->ctrl_events_avail, list) {
			/* Copy the event to kevent. */
			kevent->event.request = ev->request;
			/* Copy payload. */
			kevent->event.u = ev->u;
			break;
		}
		pr_warn("Override the last event\n");
		spin_unlock_irqrestore(&agdev->ctrl_events_lock, flags);
		return 0;
	}

	/* Use GFP_ATOMIC here. Can not sleep. */
	kevent = kzalloc(sizeof(struct uac_kevent_list), GFP_ATOMIC);
	if (!kevent) {
		spin_unlock_irqrestore(&agdev->ctrl_events_lock, flags);
		return -ENOMEM;
	}

	/* Copy the event to kevent. */
	kevent->event.request = ev->request;
	/* Copy payload. */
	kevent->event.u = ev->u;

	list_add_tail(&kevent->list, &agdev->ctrl_events_avail);

	agdev->nctrl_events_avail++;

	/* Wake up the poll. */
	wake_up_all(&agdev->ctrl_events_wait);

	spin_unlock_irqrestore(&agdev->ctrl_events_lock, flags);

	return 0;
}

int __uac_ctrl_event_dequeue(struct audio_dev *agdev, struct uac_ctrl_event *ev)
{
	unsigned long flags;
	struct uac_kevent_list *kevent = NULL;

	spin_lock_irqsave(&agdev->ctrl_events_lock, flags);

	if (list_empty(&agdev->ctrl_events_avail)) {
		spin_unlock_irqrestore(&agdev->ctrl_events_lock, flags);
		pr_warn("Ctrl event list is empty\n");
		return -ENOENT;
	}

	WARN_ON(agdev->nctrl_events_avail == 0);

	kevent = list_first_entry(&agdev->ctrl_events_avail, struct uac_kevent_list, list);
	list_del(&kevent->list);
	agdev->nctrl_events_avail--;

	*ev = kevent->event;

	spin_unlock_irqrestore(&agdev->ctrl_events_lock, flags);

	if (kevent)
		kfree(kevent);

	return 0;
}

int uac_ctrl_event_dequeue(struct audio_dev *agdev,
	struct uac_ctrl_event *ev, int nonblocking)
{
	int ret;

	if (nonblocking)
		return __uac_ctrl_event_dequeue(agdev, ev);

	do {
		ret = wait_event_interruptible(agdev->ctrl_events_wait,
			agdev->nctrl_events_avail != 0);

		if (ret < 0)
			break;

		ret = __uac_ctrl_event_dequeue(agdev, ev);
	} while(ret == -ENOENT);

	return ret;
}

static int uac_ctrl_node_open(struct inode *inode, struct file *filp)
{
	struct audio_dev *agdev = container_of(filp->private_data,
		struct audio_dev, miscdev);
	int ret;

	filp->private_data = agdev;
	ret = nonseekable_open(inode, filp);
	if (ret) {
		pr_err("nonseekable-open failed\n");
	}

	return ret;
}

static int uac_ctrl_node_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static unsigned int uac_ctrl_node_poll(struct file *filp, poll_table *wait)
{
	unsigned int ret = 0;
	struct audio_dev *agdev = filp->private_data;

	poll_wait(filp, &agdev->ctrl_events_wait, wait);

	if (!list_empty(&agdev->ctrl_events_avail))
		ret = POLL_IN | POLLRDNORM;

	return ret;
}

static long uac_ctrl_node_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct audio_dev *agdev = filp->private_data;
	u8 __user *uarg = (u8 __user *)arg;
	struct uac_feature_unit_value fu_value;
	struct uac_ctrl_event ctrl_event;
	struct usb_composite_dev *cdev = agdev->func.config->cdev;
	struct usb_request *req = cdev->req;
	int err;
	unsigned int n = 0, count = 0;

	switch(cmd) {
	case UAC_IOC_SEND_FU_VALUE:
		if (!(filp->f_mode & FMODE_WRITE)) {
			err = -EBADF;
			break;
		}

		n = _IOC_SIZE(cmd);

		if (copy_from_user(&fu_value, uarg, n)) {
			err = -EFAULT;
			break;
		}

		count = fu_value.length;

		memcpy(req->buf, fu_value.data, count);

		req->zero = 0;
		req->length = count;
		err = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (err < 0)
			ERROR(cdev, "usb_ep_queue error on ep0 err=%d,length=%d\n",
				err, req->length);
		break;

	case UAC_IOC_DQEVENT:
		if (!(filp->f_mode & FMODE_READ)) {
			err = -EBADF;
			break;
		}

		err = uac_ctrl_event_dequeue(agdev, &ctrl_event,
			filp->f_mode & O_NONBLOCK);

		if (err)
			break;

		n = _IOC_SIZE(cmd);

		if (copy_to_user(uarg, &ctrl_event, n)) {
			err = -EFAULT;
			break;
		}

		break;

	default:
		err = -ENOTTY;
		break;
	}

	return err;
}

/*---------------------File Operations--------------------------*/
const struct file_operations uac_ctrl_node_fops = {
	.owner				= THIS_MODULE,
	.open 				= uac_ctrl_node_open,
	.release 			= uac_ctrl_node_release,
	.poll				= uac_ctrl_node_poll,
	.unlocked_ioctl		= uac_ctrl_node_ioctl,
};

static void agdev_iso_complete(struct usb_ep *ep, struct usb_request *req)
{
	unsigned pending;
	unsigned long flags;
	bool update_alsa = false;
	unsigned char *src, *dst;
	int status = req->status;
	struct uac_plcm_req *ur = req->context;
	struct snd_pcm_substream *substream;
	struct uac_plcm_rtd_params *prm = ur->pp;
	struct snd_uac_plcm_chip *uac_plcmc = prm->uac_plcmc;

	/* i/f shutting down */
	if (!prm->ep_enabled || req->status == -ESHUTDOWN)
		return;

	/*
	 * We can't really do much about bad xfers.
	 * Afterall, the ISOCH xfers could fail legitimately.
	 */
	if (status)
		pr_debug("%s: iso_complete status(%d) %d/%d\n",
				__func__, status, req->actual, req->length);

	substream = prm->ss;

	/* Do nothing if ALSA isn't active */
	if (!substream)
		goto exit;

	spin_lock_irqsave(&prm->lock, flags);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		src = prm->dma_area + prm->hw_ptr;
		req->actual = req->length;
		dst = req->buf;
	} else {
		dst = prm->dma_area + prm->hw_ptr;
		src = req->buf;
	}

	pending = prm->hw_ptr % prm->period_size;
	pending += req->actual;
	if (pending >= prm->period_size)
		update_alsa = true;

	prm->hw_ptr = (prm->hw_ptr + req->actual) % prm->dma_bytes;

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Pack USB load in ALSA ring buffer */
	memcpy(dst, src, req->actual);
exit:
	if (usb_ep_queue(ep, req, GFP_ATOMIC))
		dev_err(&uac_plcmc->pdev.dev, "%d Error!\n", __LINE__);

	if (update_alsa)
		snd_pcm_period_elapsed(substream);

	return;
}

static int uac_plcm_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_uac_plcm_chip *uac_plcmc = snd_pcm_substream_chip(substream);
	struct uac_plcm_rtd_params *prm;
	unsigned long flags;
	int err = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac_plcmc->pm_prm;
	else
		prm = &uac_plcmc->cs_prm;

	spin_lock_irqsave(&prm->lock, flags);

	/* Reset */
	prm->hw_ptr = 0;

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
			prm->ss = substream;
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
			prm->ss = NULL;
			break;
		default:
			err = -EINVAL;
	}

	spin_unlock_irqrestore(&prm->lock, flags);

	/* Clear buffer after Play stops */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK && !prm->ss)
		memset(prm->rbuf, 0, prm->max_psize * USB_XFERS);

	return err;
}

static snd_pcm_uframes_t uac_plcm_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_uac_plcm_chip *uac_plcmc = snd_pcm_substream_chip(substream);
	struct uac_plcm_rtd_params *prm;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac_plcmc->pm_prm;
	else
		prm = &uac_plcmc->cs_prm;

	return bytes_to_frames(substream->runtime, prm->hw_ptr);
}

static int uac_plcm_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	struct snd_uac_plcm_chip *uac_plcmc = snd_pcm_substream_chip(substream);
	struct uac_plcm_rtd_params *prm;
	int err;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac_plcmc->pm_prm;
	else
		prm = &uac_plcmc->cs_prm;

	err = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));
	if (err >= 0) {
		prm->dma_bytes = substream->runtime->dma_bytes;
		prm->dma_area = substream->runtime->dma_area;
		prm->period_size = params_period_bytes(hw_params);
		pr_trace("%s:%d prm->period_size:%d prm->dma_bytes:%d\n", __func__, __LINE__, prm->period_size,prm->dma_bytes);
	}

	return err;
}

static int uac_plcm_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_uac_plcm_chip *uac_plcmc = snd_pcm_substream_chip(substream);
	struct uac_plcm_rtd_params *prm;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prm = &uac_plcmc->pm_prm;
	else
		prm = &uac_plcmc->cs_prm;

	prm->dma_area = NULL;
	prm->dma_bytes = 0;
	prm->period_size = 0;

	return snd_pcm_lib_free_pages(substream);
}

static int uac_plcm_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_uac_plcm_chip *uac_plcmc = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_trace("%s:%d\n", __func__, __LINE__);

	runtime->hw = uac_plcm_pcm_hardware;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		spin_lock_init(&uac_plcmc->pm_prm.lock);
		runtime->hw.rate_min = MIC_SAMPLE_RATE;
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
		runtime->hw.channels_min =  MIC_NUM_CHANNELS;
		runtime->hw.period_bytes_min = 2 * uac_plcmc->pm_prm.max_psize
			/ runtime->hw.periods_min;
	} else {
		spin_lock_init(&uac_plcmc->cs_prm.lock);
		runtime->hw.rate_min = SPK_SAMPLE_RATE;
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S16_LE;
		runtime->hw.channels_min =  SPK_NUM_CHANNELS;
		runtime->hw.period_bytes_min = 2 * uac_plcmc->cs_prm.max_psize
			/ runtime->hw.periods_min;
	}

	runtime->hw.rate_max = runtime->hw.rate_min;
	runtime->hw.channels_max = runtime->hw.channels_min;

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);

	return 0;
}

/* ALSA cries without these function pointers */
static int uac_plcm_pcm_null(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_pcm_ops uac_plcm_pcm_ops = {
	.open = uac_plcm_pcm_open,
	.close = uac_plcm_pcm_null,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = uac_plcm_pcm_hw_params,
	.hw_free = uac_plcm_pcm_hw_free,
	.trigger = uac_plcm_pcm_trigger,
	.pointer = uac_plcm_pcm_pointer,
	.prepare = uac_plcm_pcm_null,
};

static int snd_uac_plcm_probe(struct platform_device *pdev)
{
	struct snd_uac_plcm_chip *uac_plcmc = pdev_to_uac_plcmc(pdev);
	struct snd_card *card;
	struct snd_pcm *pcm;
	int err;

	pr_trace("%s:%d\n", __func__, __LINE__);

	/* Choose any slot, with no id */
	err = snd_card_create(-1, NULL, THIS_MODULE, 0, &card);
	if (err < 0)
		return err;

	uac_plcmc->card = card;

	/*
	 * Create first PCM device
	 * Create a substream only for non-zero channel streams
	 */
	err = snd_pcm_new(uac_plcmc->card, "Plcm PCM", 0, 1, 1 , &pcm);
	if (err < 0)
		goto snd_fail;

	strcpy(pcm->name, "Plcm PCM");
	pcm->private_data = uac_plcmc;

	uac_plcmc->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &uac_plcm_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &uac_plcm_pcm_ops);

	strcpy(card->driver, "Plcm_Gadget");
	strcpy(card->shortname, "Plcm_Gadget");
	sprintf(card->longname, "Plcm_Gadget %i", pdev->id);

	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL), 0, BUFF_SIZE_MAX);

	err = snd_card_register(card);
	if (!err) {
		platform_set_drvdata(pdev, card);
		/* Update the config */
		if (uac_plcm_config) {
			uac_plcm_config->device = pcm->device;
			uac_plcm_config->card = pcm->card->number;
		}
		return 0;
	}

snd_fail:
	snd_card_free(card);

	uac_plcmc->pcm = NULL;
	uac_plcmc->card = NULL;

	return err;
}

static int snd_uac_plcm_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	pr_trace("%s:%d\n", __func__, __LINE__);

	if (card)
		return snd_card_free(card);

	return 0;
}

static void alsa_uac_plcm_release(struct device *dev)
{
}

static int alsa_uac_plcm_init(struct audio_dev *agdev)
{
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;
	int err;

	pr_trace("%s:%d\n", __func__, __LINE__);

	uac_plcmc->pdrv.probe = snd_uac_plcm_probe;
	uac_plcmc->pdrv.remove = snd_uac_plcm_remove;
	uac_plcmc->pdrv.driver.name = uac_plcm_name;

	uac_plcmc->pdev.id = 0;
	uac_plcmc->pdev.name = uac_plcm_name;
	uac_plcmc->pdev.dev.release = alsa_uac_plcm_release;

	/* Register snd_uac2 driver */
	err = platform_driver_register(&uac_plcmc->pdrv);
	if (err)
		return err;

	/* Register snd_uac2 device */
	err = platform_device_register(&uac_plcmc->pdev);
	if (err)
		platform_driver_unregister(&uac_plcmc->pdrv);

	return err;
}

static void alsa_uac_plcm_exit(struct audio_dev *agdev)
{
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;

	pr_trace("%s:%d\n", __func__, __LINE__);

	platform_driver_unregister(&uac_plcmc->pdrv);
	platform_device_unregister(&uac_plcmc->pdev);
}

/* --------- UAC 1.0 Descriptors ------------- */
enum {
	STR_ASSOC,
	STR_AS_OUT_ALT0,
	STR_AS_OUT_ALT1,
	STR_AS_IN_ALT0,
	STR_AS_IN_ALT1,
};

static struct usb_string strings_fn[] = {
	[STR_ASSOC].s = "Polycom Speakerphone",
	[STR_AS_OUT_ALT0].s = "Playback Inactive",
	[STR_AS_OUT_ALT1].s = "Playback Active",
	[STR_AS_IN_ALT0].s = "Capture Inactive",
	[STR_AS_IN_ALT1].s = "Capture Active",
	{ },
};

static struct usb_gadget_strings str_fn = {
	.language = 0x0409,	/* en-us */
	.strings = strings_fn,
};

static struct usb_gadget_strings *fn_strings[] = {
	&str_fn,
	NULL,
};


static struct usb_interface_assoc_descriptor ac_intf_assoc_desc = {
	.bLength 		= sizeof ac_intf_assoc_desc,
	.bDescriptorType 	= USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface 	= 0x00,	/* Dynamic interface number. */
	.bInterfaceCount 	= 0x03,
	.bFunctionClass 	= USB_CLASS_AUDIO,
	.bFunctionSubClass 	= 0x00,
	.bFunctionProtocol 	= 0,
};

/* Audio control Interface descriptor */
static struct usb_interface_descriptor ac_interface_desc = {
	.bLength 		= sizeof ac_interface_desc,
	.bDescriptorType 	= USB_DT_INTERFACE,
	.bInterfaceNumber	= 0x00,
	.bAlternateSetting 	= 0x00,
	.bNumEndpoints 		= 0x00,
	.bInterfaceClass 	= USB_CLASS_AUDIO,
	.bInterfaceSubClass 	= USB_SUBCLASS_AUDIOCONTROL,
	.bInterfaceProtocol 	= 0,
};

DECLARE_UAC_AC_HEADER_DESCRIPTOR(2);

#define F_AUDIO_NUM_INTERFACES	2
#define UAC_DT_AC_HEADER_LENGTH	UAC_DT_AC_HEADER_SIZE(F_AUDIO_NUM_INTERFACES)
#define UAC_DT_TOTAL_LENGTH (UAC_DT_AC_HEADER_LENGTH \
		+ UAC_DT_INPUT_TERMINAL_SIZE + UAC_DT_OUTPUT_TERMINAL_SIZE + UAC_DT_FEATURE_UNIT_SIZE(0) \
		+ UAC_DT_INPUT_TERMINAL_SIZE + UAC_DT_OUTPUT_TERMINAL_SIZE + UAC_DT_FEATURE_UNIT_SIZE(0) )
/* B.3.2  Class-Specific AC Interface Descriptor */
static struct uac1_ac_header_descriptor_2 ac_header_desc2 = {
	.bLength = UAC_DT_AC_HEADER_LENGTH,
	.bDescriptorType = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = UAC_HEADER,
	.bcdADC = __constant_cpu_to_le16(0x0100),
	.wTotalLength = __constant_cpu_to_le16(UAC_DT_TOTAL_LENGTH),
	.bInCollection = F_AUDIO_NUM_INTERFACES,
	.baInterfaceNr[0] = 0x01,	/* Dynamic interface number. */
	.baInterfaceNr[1] = 0x02,	/* Dynamic interface number. */
};

/** Audio Topology
 * SPK unit (prefix:spk_,cs_)
 *     host:spk -> USB:OUT -> IT(1) -> FU(5) -> OT(2) -> this:capture
 * MIC unit (prefix:mic_,pm_)
 *     this:playback -> IT(3) -> FU(6) -> OT(4) -> USB:IN -> host:mic
 */
#define SPK_INPUT_TERMINAL_ID	1
#define SPK_FEATURE_UNIT_ID		2
#define SPK_OUTPUT_TERMINAL_ID	3
#define MIC_INPUT_TERMINAL_ID	4
#define MIC_FEATURE_UNIT_ID		5
#define MIC_OUTPUT_TERMINAL_ID	6

#define UAC_BIDIR_TERMINAL_ECHOCANCELING_SPEAKERPHONE                0x405

static struct uac_input_terminal_descriptor spk_input_terminal_desc = {
	.bLength = 		sizeof spk_input_terminal_desc,
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = 	UAC_INPUT_TERMINAL,
	.bTerminalID = 		SPK_INPUT_TERMINAL_ID,
	.wTerminalType = 	__constant_cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 	MIC_OUTPUT_TERMINAL_ID,
	.wChannelConfig = 	 SPK_CHANNEL_CONFIG,
};

DECLARE_UAC_FEATURE_UNIT_DESCRIPTOR(0);

static struct uac_feature_unit_descriptor_0 spk_feature_unit_desc = {
	.bLength = 		UAC_DT_FEATURE_UNIT_SIZE(0),
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = 	UAC_FEATURE_UNIT,
	.bUnitID = 		SPK_FEATURE_UNIT_ID,
	.bSourceID = 		SPK_INPUT_TERMINAL_ID,
	.bControlSize = 	1,
	.bmaControls[0] = 	(UAC_FU_MUTE | UAC_FU_VOLUME),
};

static struct uac1_output_terminal_descriptor spk_output_terminal_desc = {
	.bLength = 		sizeof spk_output_terminal_desc,
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = 	UAC_OUTPUT_TERMINAL,
	.bTerminalID = 		SPK_OUTPUT_TERMINAL_ID,
	.wTerminalType = 	__constant_cpu_to_le16(UAC_BIDIR_TERMINAL_ECHOCANCELING_SPEAKERPHONE),
	.bAssocTerminal = 	MIC_INPUT_TERMINAL_ID,
	.bSourceID =		SPK_FEATURE_UNIT_ID,
};

static struct uac_input_terminal_descriptor mic_input_terminal_desc = {
	.bLength = 		sizeof mic_input_terminal_desc,
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = 	UAC_INPUT_TERMINAL,
	.bTerminalID = 		MIC_INPUT_TERMINAL_ID,
	.wTerminalType = 	__constant_cpu_to_le16(UAC_BIDIR_TERMINAL_ECHOCANCELING_SPEAKERPHONE),
	.bAssocTerminal = 	SPK_OUTPUT_TERMINAL_ID,
	.wChannelConfig = 	MIC_CHANNEL_CONFIG,
};

static struct uac_feature_unit_descriptor_0 mic_feature_unit_desc = {
	.bLength = 		UAC_DT_FEATURE_UNIT_SIZE(0),
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = 	UAC_FEATURE_UNIT,
	.bUnitID = 		MIC_FEATURE_UNIT_ID	,
	.bSourceID = 		MIC_INPUT_TERMINAL_ID,
	.bControlSize = 	1,
	.bmaControls[0] = 	(UAC_FU_MUTE | UAC_FU_VOLUME),
};

static struct uac1_output_terminal_descriptor mic_output_terminal_desc = {
	.bLength = 		sizeof mic_output_terminal_desc,
	.bDescriptorType = 	USB_DT_CS_INTERFACE,
	.bDescriptorSubtype = 	UAC_OUTPUT_TERMINAL,
	.bTerminalID = 		MIC_OUTPUT_TERMINAL_ID,
	.wTerminalType = 	__constant_cpu_to_le16(UAC_TERMINAL_STREAMING),
	.bAssocTerminal = 	SPK_INPUT_TERMINAL_ID,
	.bSourceID = 		MIC_FEATURE_UNIT_ID	,
};

/* B.4.1  Standard AS Interface Descriptor */
static struct usb_interface_descriptor spk_as_intf_alt0_desc = {
	.bLength 		= sizeof spk_as_intf_alt0_desc,
	.bDescriptorType 	= USB_DT_INTERFACE,
	.bInterfaceNumber 	= 0x1,
	.bAlternateSetting 	= 0x00,
	.bNumEndpoints 		= 0x00,
	.bInterfaceClass 	= USB_CLASS_AUDIO,
	.bInterfaceSubClass 	= USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol 	= 0,
};

static struct usb_interface_descriptor spk_as_intf_alt1_desc = {
	.bLength 		= sizeof spk_as_intf_alt1_desc,
	.bDescriptorType 	= USB_DT_INTERFACE,
	.bInterfaceNumber 	= 0x1,
	.bAlternateSetting 	= 0x01,
	.bNumEndpoints 		= 0x01,
	.bInterfaceClass 	= USB_CLASS_AUDIO,
	.bInterfaceSubClass 	= USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol 	= 0,
};

static struct usb_interface_descriptor mic_as_intf_alt0_desc = {
	.bLength = 		sizeof mic_as_intf_alt0_desc,
	.bDescriptorType = 	USB_DT_INTERFACE,
	.bInterfaceNumber = 	0x02,
	.bAlternateSetting = 	0x00,
	.bNumEndpoints = 	0x00,
	.bInterfaceClass = 	USB_CLASS_AUDIO,
	.bInterfaceSubClass = 	USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 	0,
};

static struct usb_interface_descriptor mic_as_intf_alt1_desc = {
	.bLength = 		sizeof mic_as_intf_alt1_desc,
	.bDescriptorType = 	USB_DT_INTERFACE,
	.bInterfaceNumber = 	0x02,
	.bAlternateSetting = 	0x01,
	.bNumEndpoints = 	0x01,
	.bInterfaceClass = 	USB_CLASS_AUDIO,
	.bInterfaceSubClass = 	USB_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 	0,
};


/* B.4.2  Class-Specific AS Interface Descriptor */
static struct uac1_as_header_descriptor spk_as_header_desc = {
	.bLength 		= sizeof spk_as_header_desc,
	.bDescriptorType 	= USB_DT_CS_INTERFACE,
	.bDescriptorSubtype 	= UAC_AS_GENERAL,
	.bTerminalLink 		= SPK_INPUT_TERMINAL_ID,
	.bDelay 		= 0x1,
	.wFormatTag 		= __constant_cpu_to_le16(UAC_FORMAT_TYPE_I_PCM),
};

static struct uac1_as_header_descriptor mic_as_header_desc = {
	.bLength 		= sizeof mic_as_header_desc,
	.bDescriptorType 	= USB_DT_CS_INTERFACE,
	.bDescriptorSubtype 	= UAC_AS_GENERAL,
	.bTerminalLink 		= MIC_OUTPUT_TERMINAL_ID,
	.bDelay			= 0x1,
	.wFormatTag		= __constant_cpu_to_le16(UAC_FORMAT_TYPE_I_PCM),
};


DECLARE_UAC_FORMAT_TYPE_I_DISCRETE_DESC(1);

static struct uac_format_type_i_discrete_descriptor_1 spk_as_type_i_desc = {
	.bLength                = UAC_FORMAT_TYPE_I_DISCRETE_DESC_SIZE(1),
	.bDescriptorType        = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype     = UAC_FORMAT_TYPE,
	.bFormatType            = UAC_FORMAT_TYPE_I,
	.bSubframeSize          = 2,
	.bBitResolution         = 16,
	.bSamFreqType           = 1,
};

static struct uac_format_type_i_discrete_descriptor_1 mic_as_type_i_desc = {
	.bLength                = UAC_FORMAT_TYPE_I_DISCRETE_DESC_SIZE(1),
	.bDescriptorType        = USB_DT_CS_INTERFACE,
	.bDescriptorSubtype     = UAC_FORMAT_TYPE,
	.bFormatType            = UAC_FORMAT_TYPE_I,
	.bSubframeSize          = 2,
	.bBitResolution         = 16,
	.bSamFreqType           = 1,
};

/* Standard ISO IN Endpoint Descriptor for highspeed */
static struct usb_endpoint_descriptor spk_hs_as_out_ep_desc = {
	.bLength 		= sizeof spk_hs_as_out_ep_desc,
	.bDescriptorType 	= USB_DT_ENDPOINT,
	.bEndpointAddress 	= USB_DIR_OUT,
	.bmAttributes 		= USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.wMaxPacketSize 	= __constant_cpu_to_le16(SPK_EP_MAX_PACKET_SIZE),
	.bInterval 		= 4, /* poll 1 per millisecond */
};

static struct usb_endpoint_descriptor mic_hs_as_in_ep_desc = {
	.bLength 		= sizeof mic_hs_as_in_ep_desc,
	.bDescriptorType 	= USB_DT_ENDPOINT,
	.bEndpointAddress 	= USB_DIR_IN,
	.bmAttributes 		= USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.wMaxPacketSize 	= __constant_cpu_to_le16(MIC_EP_MAX_PACKET_SIZE),
	.bInterval 		= 4, /* poll 1 per millisecond */
};


/* Standard ISO IN Endpoint Descriptor for fullspeed */
static struct usb_endpoint_descriptor spk_fs_as_out_ep_desc = {
	.bLength 		= sizeof spk_fs_as_out_ep_desc,
	.bDescriptorType 	= USB_DT_ENDPOINT,
	.bEndpointAddress 	= USB_DIR_OUT,
	.bmAttributes 		= USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.wMaxPacketSize 	= __constant_cpu_to_le16(SPK_EP_MAX_PACKET_SIZE),
	.bInterval 		= 1, /* poll 1 per millisecond */
};

static struct usb_endpoint_descriptor mic_fs_as_in_ep_desc = {
	.bLength 		= sizeof mic_fs_as_in_ep_desc,
	.bDescriptorType 	= USB_DT_ENDPOINT,
	.bEndpointAddress 	= USB_DIR_IN,
	.bmAttributes 		= USB_ENDPOINT_XFER_ISOC | USB_ENDPOINT_SYNC_ASYNC,
	.wMaxPacketSize 	= __constant_cpu_to_le16(MIC_EP_MAX_PACKET_SIZE),
	.bInterval 		= 1, /* poll 1 per millisecond */
};

/* Class-specific AS ISO Endpoint Descriptor */
static struct uac_iso_endpoint_descriptor spk_as_iso_out_desc = {
	.bLength 		= sizeof spk_as_iso_out_desc,
	.bDescriptorType 	= USB_DT_CS_ENDPOINT,
	.bDescriptorSubtype 	= UAC_EP_GENERAL,
	.bmAttributes 		= 0x00,
	.bLockDelayUnits 	= 0x00,
	.wLockDelay 		= 0x0000,
};

static struct uac_iso_endpoint_descriptor mic_as_iso_in_desc = {
	.bLength 		= sizeof mic_as_iso_in_desc,
	.bDescriptorType 	= USB_DT_CS_ENDPOINT,
	.bDescriptorSubtype 	= UAC_EP_GENERAL,
	.bmAttributes 		= 0x00,
	.bLockDelayUnits 	= 0x00,
	.wLockDelay 		= 0x0000,
};


/*Full Speed Descriptors*/
static struct usb_descriptor_header *uac_plcm_fs_descriptors[] = {
	(struct usb_descriptor_header *)&ac_intf_assoc_desc,
	(struct usb_descriptor_header *)&ac_interface_desc,
	(struct usb_descriptor_header *)&ac_header_desc2,
	(struct usb_descriptor_header *)&spk_input_terminal_desc,
	(struct usb_descriptor_header *)&spk_feature_unit_desc,
	(struct usb_descriptor_header *)&spk_output_terminal_desc,
	(struct usb_descriptor_header *)&mic_input_terminal_desc,
	(struct usb_descriptor_header *)&mic_feature_unit_desc,
	(struct usb_descriptor_header *)&mic_output_terminal_desc,

	(struct usb_descriptor_header *)&spk_as_intf_alt0_desc,
	(struct usb_descriptor_header *)&spk_as_intf_alt1_desc,
	(struct usb_descriptor_header *)&spk_as_header_desc,
	(struct usb_descriptor_header *)&spk_as_type_i_desc,
	(struct usb_descriptor_header *)&spk_fs_as_out_ep_desc,
	(struct usb_descriptor_header *)&spk_as_iso_out_desc,

	(struct usb_descriptor_header *)&mic_as_intf_alt0_desc,
	(struct usb_descriptor_header *)&mic_as_intf_alt1_desc,
	(struct usb_descriptor_header *)&mic_as_header_desc,
	(struct usb_descriptor_header *)&mic_as_type_i_desc,
	(struct usb_descriptor_header *)&mic_fs_as_in_ep_desc,
	(struct usb_descriptor_header *)&mic_as_iso_in_desc,
	0,
};

static struct usb_descriptor_header *uac_plcm_hs_descriptors[] = {
	(struct usb_descriptor_header *)&ac_intf_assoc_desc,
	(struct usb_descriptor_header *)&ac_interface_desc,
	(struct usb_descriptor_header *)&ac_header_desc2,

	(struct usb_descriptor_header *)&spk_input_terminal_desc,
	(struct usb_descriptor_header *)&spk_feature_unit_desc,
	(struct usb_descriptor_header *)&spk_output_terminal_desc,
	(struct usb_descriptor_header *)&mic_input_terminal_desc,
	(struct usb_descriptor_header *)&mic_feature_unit_desc,
	(struct usb_descriptor_header *)&mic_output_terminal_desc,

	(struct usb_descriptor_header *)&spk_as_intf_alt0_desc,
	(struct usb_descriptor_header *)&spk_as_intf_alt1_desc,
	(struct usb_descriptor_header *)&spk_as_header_desc,
	(struct usb_descriptor_header *)&spk_as_type_i_desc,
	(struct usb_descriptor_header *)&spk_hs_as_out_ep_desc,
	(struct usb_descriptor_header *)&spk_as_iso_out_desc,

	(struct usb_descriptor_header *)&mic_as_intf_alt0_desc,
	(struct usb_descriptor_header *)&mic_as_intf_alt1_desc,
	(struct usb_descriptor_header *)&mic_as_header_desc,
	(struct usb_descriptor_header *)&mic_as_type_i_desc,
	(struct usb_descriptor_header *)&mic_hs_as_in_ep_desc,
	(struct usb_descriptor_header *)&mic_as_iso_in_desc,
	0,
};

/* --------- USB Audio Controls ------------- */
static int control_spk_mute_set(struct usb_audio_control *con, u8 cmd, int value)
{
	if (cmd != UAC__CUR) {
		printk(KERN_WARNING "%s: set cmd=%d not support\n", __func__, cmd);
		return 0;
	}
	con->data[UAC__CUR] = value;
	return 0;
}

static int control_spk_mute_get(struct usb_audio_control *con, u8 cmd)
{
	return con->data[cmd];
}


static int control_spk_volume_set(struct usb_audio_control *con, u8 cmd, int value)
{
	/* value is s16 for volume */
	int volume = (s16)value;

	if (cmd != UAC__CUR) {
		printk(KERN_WARNING "%s: set cmd=%d not support\n", __func__, cmd);
		return 0;
	}
	con->data[UAC__CUR] = volume;
	return 0;
}

static int control_spk_volume_get(struct usb_audio_control *con, u8 cmd)
{
	return con->data[cmd];
}

static int control_mic_mute_set(struct usb_audio_control *con, u8 cmd, int value)
{
	if (cmd != UAC__CUR) {
		printk(KERN_WARNING "%s: set cmd=%d not support\n", __func__, cmd);
		return 0;
	}
	con->data[UAC__CUR] = value;
	return 0;
}

static int control_mic_mute_get(struct usb_audio_control *con, u8 cmd)
{
	return con->data[cmd];
}

static int control_mic_volume_set(struct usb_audio_control *con, u8 cmd, int value)
{
	/* value is s16 for volume */
	int volume = (s16)value;

	if (cmd != UAC__CUR) {
		printk(KERN_WARNING "%s: set cmd=%d not support\n", __func__, cmd);
		return 0;
	}
	con->data[UAC__CUR] = volume;
	return 0;
}

static int control_mic_volume_get(struct usb_audio_control *con, u8 cmd)
{
	return con->data[cmd];
}


static struct usb_audio_control spk_mute_control = {
	.list = LIST_HEAD_INIT(spk_mute_control.list),
	.name = "Speaker Mute Control",
	.type = UAC_FU_MUTE,
	.set = control_spk_mute_set,
	.get = control_spk_mute_get,
};

static struct usb_audio_control spk_volume_control = {
	.list = LIST_HEAD_INIT(spk_volume_control.list),
	.name = "Speaker Volume Control",
	.type = UAC_FU_VOLUME,
	.set = control_spk_volume_set,
	.get = control_spk_volume_get,
};

static struct usb_audio_control_selector spk_feature_unit = {
	.list = LIST_HEAD_INIT(spk_feature_unit.list),
	.id = SPK_FEATURE_UNIT_ID,
	.name = "Speaker Mute & Volume Control",
	.type = UAC_FEATURE_UNIT,
	.desc = (struct usb_descriptor_header *)&spk_feature_unit_desc,
};

static struct usb_audio_control mic_mute_control = {
	.list = LIST_HEAD_INIT(mic_mute_control.list),
	.name = "Mic Mute Control",
	.type = UAC_FU_MUTE,
	.set = control_mic_mute_set,
	.get = control_mic_mute_get,
};

static struct usb_audio_control mic_volume_control = {
	.list = LIST_HEAD_INIT(mic_volume_control.list),
	.name = "Mic Volume Control",
	.type = UAC_FU_VOLUME,
	.set = control_mic_volume_set,
	.get = control_mic_volume_get,
};

static struct usb_audio_control_selector mic_feature_unit = {
	.list = LIST_HEAD_INIT(mic_feature_unit.list),
	.id = MIC_FEATURE_UNIT_ID,
	.name = "Mic Mute & Volume Control",
	.type = UAC_FEATURE_UNIT,
	.desc = (struct usb_descriptor_header *)&mic_feature_unit_desc,
};


static int control_selector_init(struct audio_dev *agdev)
{
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;

	pr_trace("%s:%d\n", __func__, __LINE__);

	INIT_LIST_HEAD(&uac_plcmc->cs_prm.cs);
	list_add(&spk_feature_unit.list, &uac_plcmc->cs_prm.cs);

	INIT_LIST_HEAD(&spk_feature_unit.control);
	list_add(&spk_mute_control.list, &spk_feature_unit.control);
	list_add(&spk_volume_control.list, &spk_feature_unit.control);

	spk_volume_control.data[UAC__MIN] = -48*256;
	spk_volume_control.data[UAC__MAX] = 0*256;
	spk_volume_control.data[UAC__RES] = 256;
	spk_volume_control.data[UAC__CUR] = spk_volume_control.data[UAC__MAX];

	spk_mute_control.data[UAC__CUR] = 0;
	spk_mute_control.data[UAC__MIN] = 0;
	spk_mute_control.data[UAC__MAX] = 1;
	spk_mute_control.data[UAC__RES] = 1;

	INIT_LIST_HEAD(&uac_plcmc->pm_prm.cs);
	list_add(&mic_feature_unit.list, &uac_plcmc->pm_prm.cs);

	INIT_LIST_HEAD(&mic_feature_unit.control);
	list_add(&mic_mute_control.list, &mic_feature_unit.control);
	list_add(&mic_volume_control.list, &mic_feature_unit.control);

	mic_volume_control.data[UAC__MIN] = -48*256;
	mic_volume_control.data[UAC__MAX] = 0*256;
	mic_volume_control.data[UAC__RES] = 256;
	mic_volume_control.data[UAC__CUR] = mic_volume_control.data[UAC__MAX];

	mic_mute_control.data[UAC__CUR] = 0;
	mic_mute_control.data[UAC__MIN] = 0;
	mic_mute_control.data[UAC__MAX] = 1;
	mic_mute_control.data[UAC__RES] = 1;

	return 0;
}

/* --------- USB Function Interface ------------- */
static inline void free_ep(struct uac_plcm_rtd_params *prm, struct usb_ep *ep)
{
	struct snd_uac_plcm_chip *uac_plcmc = prm->uac_plcmc;
	int i;

	if (!prm->ep_enabled)
		return;

	prm->ep_enabled = false;

	for (i = 0; i < USB_XFERS; i++) {
		if (prm->ureq[i].req) {
			usb_ep_dequeue(ep, prm->ureq[i].req);
			usb_ep_free_request(ep, prm->ureq[i].req);
			prm->ureq[i].req = NULL;
		}
	}

	if (usb_ep_disable(ep))
		dev_err(&uac_plcmc->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
}

static void audio_build_desc(struct audio_dev *agdev)
{
	u8 *sam_freq;
	int rate;

	/* Set channel numbers */
	spk_input_terminal_desc.bNrChannels = SPK_NUM_CHANNELS;
	spk_as_type_i_desc.bNrChannels = SPK_NUM_CHANNELS;
	mic_input_terminal_desc.bNrChannels = MIC_NUM_CHANNELS;
	mic_as_type_i_desc.bNrChannels = MIC_NUM_CHANNELS;

	/* Set sample rates */
	rate = SPK_SAMPLE_RATE;
	sam_freq = spk_as_type_i_desc.tSamFreq[0];
	memcpy(sam_freq, &rate, 3);
	rate = MIC_SAMPLE_RATE;
	sam_freq = mic_as_type_i_desc.tSamFreq[0];
	memcpy(sam_freq, &rate, 3);
}

static int afunc_bind(struct usb_configuration *cfg, struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;
	struct usb_composite_dev *cdev = cfg->cdev;
	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &uac_plcmc->pdev.dev;
	struct uac_plcm_rtd_params *prm;
	int ret;

	pr_trace("%s:%d\n", __func__, __LINE__);

	control_selector_init(agdev);

	audio_build_desc(agdev);

	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	ac_interface_desc.bInterfaceNumber = ret;
	agdev->ac_intf = ret;
	ac_intf_assoc_desc.bFirstInterface = ret;
	agdev->ac_alt = 0;

	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	spk_as_intf_alt0_desc.bInterfaceNumber = ret;
	spk_as_intf_alt1_desc.bInterfaceNumber = ret;
	ac_header_desc2.baInterfaceNr[0] = ret;
	agdev->as_out_intf = ret;
	agdev->as_out_alt = 0;

	ret = usb_interface_id(cfg, fn);
	if (ret < 0) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return ret;
	}
	mic_as_intf_alt0_desc.bInterfaceNumber = ret;
	mic_as_intf_alt1_desc.bInterfaceNumber = ret;
	ac_header_desc2.baInterfaceNr[1] = ret;
	agdev->as_in_intf = ret;
	agdev->as_in_alt = 0;

	fn->fs_descriptors = NULL;
	fn->hs_descriptors = NULL;
	agdev->uac_plcmc.pm_prm.rbuf = NULL;
	agdev->uac_plcmc.cs_prm.rbuf = NULL;

	agdev->out_ep = usb_ep_autoconfig(gadget, &spk_fs_as_out_ep_desc);
	if (!agdev->out_ep) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		goto err;
	}
	agdev->out_ep->driver_data = agdev;

	agdev->in_ep = usb_ep_autoconfig(gadget, &mic_fs_as_in_ep_desc);
	if (!agdev->in_ep) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		goto err;
	}
	agdev->in_ep->driver_data = agdev;

	uac_plcmc->pm_prm.uac_plcmc = uac_plcmc;
	uac_plcmc->cs_prm.uac_plcmc = uac_plcmc;

	spk_hs_as_out_ep_desc.bEndpointAddress = spk_fs_as_out_ep_desc.bEndpointAddress;
	spk_hs_as_out_ep_desc.wMaxPacketSize = spk_fs_as_out_ep_desc.wMaxPacketSize;
	mic_hs_as_in_ep_desc.bEndpointAddress = mic_fs_as_in_ep_desc.bEndpointAddress;
	mic_hs_as_in_ep_desc.wMaxPacketSize = mic_fs_as_in_ep_desc.wMaxPacketSize;

	fn->fs_descriptors = usb_copy_descriptors(uac_plcm_fs_descriptors);
	if (!fn->fs_descriptors) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		goto err;
	}
	if (gadget_is_dualspeed(gadget)) {
		fn->hs_descriptors = usb_copy_descriptors(uac_plcm_hs_descriptors);
		if (!fn->hs_descriptors) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			goto err;
		}
	}

	prm = &agdev->uac_plcmc.cs_prm;
	prm->max_psize = spk_hs_as_out_ep_desc.wMaxPacketSize;
	prm->rbuf = kzalloc(prm->max_psize * USB_XFERS, GFP_KERNEL);
	if (!prm->rbuf) {
		prm->max_psize = 0;
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		goto err;
	}

	prm = &agdev->uac_plcmc.pm_prm;
	prm->max_psize = mic_hs_as_in_ep_desc.wMaxPacketSize;
	prm->rbuf = kzalloc(prm->max_psize * USB_XFERS, GFP_KERNEL);
	if (!prm->rbuf) {
		prm->max_psize = 0;
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		goto err;
	}

	ret = alsa_uac_plcm_init(agdev);
	if (ret)
		goto err;
	return 0;
err:
	if (agdev->uac_plcmc.pm_prm.rbuf)
		kfree(agdev->uac_plcmc.pm_prm.rbuf);
	if (agdev->uac_plcmc.cs_prm.rbuf)
		kfree(agdev->uac_plcmc.cs_prm.rbuf);
	if (gadget_is_dualspeed(gadget) && fn->hs_descriptors)
		usb_free_descriptors(fn->hs_descriptors);
	if (fn->fs_descriptors)
		usb_free_descriptors(fn->fs_descriptors);
	if (agdev->in_ep)
		agdev->in_ep->driver_data = NULL;
	if (agdev->out_ep)
		agdev->out_ep->driver_data = NULL;
	return -EINVAL;
}

static void afunc_unbind(struct usb_configuration *cfg, struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);

	pr_trace("%s:%d\n", __func__, __LINE__);
	/* Cancel the pending work queue */
	cancel_work_sync(&agdev->status_notify_work);

	alsa_uac_plcm_exit(agdev);

	/*TODO	while ((req = audio_req_get(audio)))
	  audio_request_free(req, audio->in_ep); */

	kfree(agdev->uac_plcmc.pm_prm.rbuf);
	kfree(agdev->uac_plcmc.cs_prm.rbuf);

	if (gadget_is_dualspeed(cfg->cdev->gadget))
		usb_free_descriptors(fn->hs_descriptors);
	usb_free_descriptors(fn->fs_descriptors);
	if (agdev->in_ep)
		agdev->in_ep->driver_data = NULL;
	if (agdev->out_ep)
		agdev->out_ep->driver_data = NULL;
}

static void snd_uac_plcm_event_send(struct usb_function *fn, unsigned sink, unsigned source, unsigned alt)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	unsigned long flags;

	spin_lock_irqsave(&agdev->status_lock, flags);
	if (source == 1) {
		agdev->source_status_changed = 1;
		if (alt)
			agdev->source_active = 1;
		else
			agdev->source_active = 0;
	} else if (sink == 1) {
		agdev->sink_status_changed = 1;
		if (alt)
			agdev->sink_active = 1;
		else
			agdev->sink_active = 0;
	}

	schedule_work(&agdev->status_notify_work);
	spin_unlock_irqrestore(&agdev->status_lock, flags);
}

static void audio_status_notify(struct work_struct *data)
{
	struct audio_dev *agdev = container_of(data, struct audio_dev, status_notify_work);
	char **uevent_envp = NULL;
	struct device *dev = NULL;
	unsigned long flags;

	if(!uac_plcm_config)
		return;

	dev = uac_plcm_config->dev;

	spin_lock_irqsave(&agdev->status_lock, flags);
	if (agdev->suspended)
		uevent_envp = uac_suspend;
	else if (agdev->source_status_changed) {
		if (agdev->source_active)
			uevent_envp = uac_source_active;
		else
			uevent_envp = uac_source_deactive;
		agdev->source_status_changed = 0;
	} else if (agdev->sink_status_changed) {
		if (agdev->sink_active)
			uevent_envp = uac_sink_active;
		else
			uevent_envp = uac_sink_deactive;
		agdev->sink_status_changed = 0;
	}
	spin_unlock_irqrestore(&agdev->status_lock, flags);

	if (uevent_envp) {
		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, uevent_envp);
		pr_info("%s: audio sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_info("%s: audio did not send uevent (suspended: %d, sink_active: %d, source_active: %d\n",
			__func__, agdev->suspended, agdev->sink_active, agdev->source_active);
	}
}

static int afunc_set_alt(struct usb_function *fn, unsigned intf, unsigned alt)
{
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;
	struct usb_gadget *gadget = cdev->gadget;
	struct device *dev = &uac_plcmc->pdev.dev;
	struct usb_request *req;
	struct usb_ep *ep;
	struct uac_plcm_rtd_params *prm;
	unsigned sink;
	unsigned source;
	int i;

	pr_trace("%s:%d\n", __func__, __LINE__);
	pr_trace("%s: intf %d, alt %d\n", __func__, intf, alt);

	sink = 0;
	source = 0;
	/* No i/f has more than 2 alt settings */
	if (alt > 1) {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (intf == agdev->ac_intf) {
		/* Control I/f has only 1 AltSetting - 0 */
		if (alt) {
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
			return -EINVAL;
		}
		return 0;
	}

	if (intf == agdev->as_out_intf) {
		ep = agdev->out_ep;
		prm = &uac_plcmc->cs_prm;
		config_ep_by_speed(gadget, fn, ep);
		agdev->as_out_alt = alt;
		sink = 1;
	} else if (intf == agdev->as_in_intf) {
		ep = agdev->in_ep;
		prm = &uac_plcmc->pm_prm;
		config_ep_by_speed(gadget, fn, ep);
		agdev->as_in_alt = alt;
		source = 1;
	} else {
		dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (alt == 0) {
		free_ep(prm, ep);
		snd_uac_plcm_event_send(fn, sink, source, alt);
		return 0;
	}

	prm->ep_enabled = true;
	usb_ep_enable(ep);

	for (i = 0; i < USB_XFERS; i++) {
		if (prm->ureq[i].req) {
			if (usb_ep_queue(ep, prm->ureq[i].req, GFP_ATOMIC))
				dev_err(&uac_plcmc->pdev.dev, "%d Error!\n",
						__LINE__);
			continue;
		}

		req = usb_ep_alloc_request(ep, GFP_ATOMIC);
		if (req == NULL) {
			dev_err(&uac_plcmc->pdev.dev,
					"%s:%d Error!\n", __func__, __LINE__);
			return -EINVAL;
		}

		prm->ureq[i].req = req;
		prm->ureq[i].pp = prm;

		req->zero = 0;
		req->context = &prm->ureq[i];
		req->length = prm->max_psize;
		req->complete = agdev_iso_complete;
		req->buf = prm->rbuf + i * req->length;

		if (usb_ep_queue(ep, req, GFP_ATOMIC))
			dev_err(dev, "%s:%d Error!\n", __func__, __LINE__);
	}
	snd_uac_plcm_event_send(fn, sink, source, alt);

	return 0;
}

static int afunc_get_alt(struct usb_function *fn, unsigned intf)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;

	pr_trace("%s:%d\n", __func__, __LINE__);

	if (intf == agdev->ac_intf)
		return agdev->ac_alt;
	else if (intf == agdev->as_out_intf)
		return agdev->as_out_alt;
	else if (intf == agdev->as_in_intf)
		return agdev->as_in_alt;
	else
		dev_err(&uac_plcmc->pdev.dev,
				"%s:%d Invalid Interface %d!\n",
				__func__, __LINE__, intf);

	return -EINVAL;
}

static void afunc_disable(struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;

	pr_trace("%s:%d\n", __func__, __LINE__);

	free_ep(&uac_plcmc->pm_prm, agdev->in_ep);
	agdev->as_in_alt = 0;

	free_ep(&uac_plcmc->cs_prm, agdev->out_ep);
	agdev->as_out_alt = 0;
}

static void afunc_resume(struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	unsigned long flags;

	pr_trace("%s:%d\n", __func__, __LINE__);

	spin_lock_irqsave(&agdev->status_lock, flags);
	agdev->suspended = 0;
	spin_unlock_irqrestore(&agdev->status_lock, flags);
}

static void afunc_suspend(struct usb_function *fn)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	unsigned long flags;

	pr_trace("%s:%d\n", __func__, __LINE__);

	spin_lock_irqsave(&agdev->status_lock, flags);
	agdev->suspended = 1;
	schedule_work(&agdev->status_notify_work);
	spin_unlock_irqrestore(&agdev->status_lock, flags);
}


static void audio_intf_req_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct uac_plcm_rtd_params *prm = req->context;
	struct snd_uac_plcm_chip *uac_plcmc = prm->uac_plcmc;
	struct audio_dev *agdev = uac_plcmc_to_agdev(uac_plcmc);
	int status = req->status;
	u32 data = 0;

	switch (status) {

		case 0:				/* normal completion? */
			if (prm->set_con) {
				memcpy(&data, req->buf, req->length);
				prm->set_con->set(prm->set_con, prm->set_cmd,	le16_to_cpu(data));
				prm->set_con = NULL;
			}
			if (prm->ctrl_event_set) {
				prm->ctrl_event.u.fu.value = le16_to_cpu(data);
				uac_ctrl_event_queue(agdev, &prm->ctrl_event);
				prm->ctrl_event_set = 0;
			}
			break;
		default:
			break;
	}
}

static int audio_set_intf_req(struct usb_function *fn,
		const struct usb_ctrlrequest *ctrl)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;
	struct uac_plcm_rtd_params *prm;
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct usb_request	*req = cdev->req;
	u8			id = ((le16_to_cpu(ctrl->wIndex) >> 8) & 0xFF);
	u16			len = le16_to_cpu(ctrl->wLength);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u8			con_sel = (w_value >> 8) & 0xFF;
	u8			cmd = (ctrl->bRequest & 0x0F);
	struct usb_audio_control_selector *cs;
	struct usb_audio_control *con;

	if (id == SPK_FEATURE_UNIT_ID) {
		prm = &uac_plcmc->cs_prm;
	} else if (id == MIC_FEATURE_UNIT_ID) {
		prm = &uac_plcmc->pm_prm;
	} else {
		dev_err(&uac_plcmc->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	/*
	 * Test. Enqueue the event.
	 */
	prm->ctrl_event.request = ctrl->bRequest;
	prm->ctrl_event.u.fu.unit_id = id;
	prm->ctrl_event.u.fu.control_selector = con_sel;
	prm->ctrl_event_set = 1;

	list_for_each_entry(cs, &prm->cs, list) {
		if (cs->id == id) {
			list_for_each_entry(con, &cs->control, list) {
				if (con->type == con_sel) {
					prm->set_con = con;
					break;
				}
			}
			break;
		}
	}

	prm->set_cmd = cmd;
	req->context = prm;
	req->complete = audio_intf_req_complete;

	return len;
}


static int audio_get_intf_req(struct usb_function *fn,
		const struct usb_ctrlrequest *ctrl)
{
	struct audio_dev *agdev = func_to_agdev(fn);
	struct snd_uac_plcm_chip *uac_plcmc = &agdev->uac_plcmc;
	struct uac_plcm_rtd_params *prm;
	struct usb_composite_dev *cdev = fn->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u8			id = ((le16_to_cpu(ctrl->wIndex) >> 8) & 0xFF);
	u16			len = le16_to_cpu(ctrl->wLength);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u8			con_sel = (w_value >> 8) & 0xFF;
	u8			cmd = (ctrl->bRequest & 0x0F);
	struct usb_audio_control_selector *cs;
	struct usb_audio_control *con;
	struct uac_ctrl_event ctrl_event;

	/*
	 * Test. Enqueue the event.
	 */
	memset(&ctrl_event, 0, sizeof(ctrl_event));
	ctrl_event.request = ctrl->bRequest;
	ctrl_event.u.fu.unit_id = id;
	ctrl_event.u.fu.control_selector = con_sel;

	uac_ctrl_event_queue(agdev, &ctrl_event);

	if (id == SPK_FEATURE_UNIT_ID) {
		prm = &uac_plcmc->cs_prm;
	} else if (id == MIC_FEATURE_UNIT_ID) {
		prm = &uac_plcmc->pm_prm;
	} else {
		dev_err(&uac_plcmc->pdev.dev,
				"%s:%d Error!\n", __func__, __LINE__);
		return -EINVAL;
	}

	list_for_each_entry(cs, &prm->cs, list) {
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

	req->context = prm;
	req->complete = audio_intf_req_complete;
	len = min_t(size_t, sizeof(value), len);
	memcpy(req->buf, &value, len);

	return len;
}

static int afunc_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	/*	pr_trace("%s:%d\n", __func__, __LINE__);
		pr_trace("audio ctrl req: %02x.%02x v%04x i%04x l%d\n",
		ctrl->bRequestType, ctrl->bRequest, w_value, w_index, w_length);
		*/

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

		default:
			ERROR(cdev, "invalid control req: %02x.%02x v%04x i%04x l%d\n",
					ctrl->bRequestType, ctrl->bRequest, w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		/*pr_trace("  queue req: %02x.%02x v%04x i%04x l%d\n",
		  ctrl->bRequestType, ctrl->bRequest,
		  w_value, w_index, w_length); */
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "audio response on err %d\n", value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

/* --------- USB Config Interface ------------- */
static int audio_composite_bind_config(struct usb_configuration *cfg,
		struct audio_dual_config *config)
{
	int res;

	pr_trace("%s:%d\n", __func__, __LINE__);

	agdev_g = kzalloc(sizeof *agdev_g, GFP_KERNEL);
	if (agdev_g == NULL) {
		printk(KERN_ERR "Unable to allocate audio gadget\n");
		return -ENOMEM;
	}

	uac_plcm_config = config;

	/* Update the strings. */
	if (strlen(config->iad_string) > 0)
		strings_fn[STR_ASSOC].s = config->iad_string;
	if (strlen(config->audio_out_stream_string) > 0)
		strings_fn[STR_AS_OUT_ALT1].s = config->audio_out_stream_string;
	if (strlen(config->audio_in_stream_string) > 0)
		strings_fn[STR_AS_IN_ALT1].s = config->audio_in_stream_string;

	if (strings_fn[STR_ASSOC].id == 0) {
		res = usb_string_ids_tab(cfg->cdev, strings_fn);
		if (res) {
			kfree(agdev_g);
			return res;
		}
		ac_intf_assoc_desc.iFunction = strings_fn[STR_ASSOC].id;
		spk_as_intf_alt1_desc.iInterface = strings_fn[STR_AS_OUT_ALT1].id;
		mic_as_intf_alt1_desc.iInterface = strings_fn[STR_AS_IN_ALT1].id;
	}

	/* Initialize the status lock */
	spin_lock_init(&agdev_g->status_lock);

	/* Initialize the work queue */
	INIT_WORK(&agdev_g->status_notify_work, audio_status_notify);

	/* Initialize the uac control events related. */
	spin_lock_init(&agdev_g->ctrl_events_lock);
	INIT_LIST_HEAD(&agdev_g->ctrl_events_avail);
	init_waitqueue_head(&agdev_g->ctrl_events_wait);

	/* Register the Misc Device. */
	agdev_g->miscdev.minor = MISC_DYNAMIC_MINOR;
	agdev_g->miscdev.name = "uac_plcm_control";
	agdev_g->miscdev.fops = &uac_ctrl_node_fops;

	res = misc_register(&agdev_g->miscdev);
	if (res) {
		pr_err("Failed to register the misc device. ret = %d\n",
			res);
		kfree(agdev_g);
		return res;
	}

	agdev_g->func.name = "uac_plcm_func";
	agdev_g->func.strings = fn_strings;
	agdev_g->func.bind = afunc_bind;
	agdev_g->func.unbind = afunc_unbind;
	agdev_g->func.set_alt = afunc_set_alt;
	agdev_g->func.get_alt = afunc_get_alt;
	agdev_g->func.resume = afunc_resume;
	agdev_g->func.suspend = afunc_suspend;
	agdev_g->func.disable = afunc_disable;
	agdev_g->func.setup = afunc_setup;

	res = usb_add_function(cfg, &agdev_g->func);
	if (res < 0) {
		pr_err("Failed to add usb functions\n");
		misc_deregister(&agdev_g->miscdev);
		kfree(agdev_g);
	}

	return res;
}

static int audio_bind_config(struct usb_configuration *cfg)
{
	return audio_composite_bind_config(cfg, NULL);
}

static void audio_unbind_config(struct usb_configuration *cfg)
{
	pr_trace("%s:%d\n", __func__, __LINE__);
	strings_fn[STR_ASSOC].id = 0;
	uac_plcm_config = NULL;
	misc_deregister(&agdev_g->miscdev);
	/* FIXME. Remove the list?. */
	while (!list_empty(&agdev_g->ctrl_events_avail)) {
		struct uac_kevent_list *kevent;

		kevent = list_first_entry(&agdev_g->ctrl_events_avail,
				struct uac_kevent_list, list);
		list_del(&kevent->list);
		kfree(kevent);
	}
	kfree(agdev_g);
	agdev_g = NULL;
}
