/*
 *	webcam.c -- USB webcam gadget driver
 *
 *	Copyright (C) 2009-2010
 *	    Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/video.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>

#include "uvc.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "uvc_kmalloc.c"
#include "uvc_queue.c"
#include "uvc_video.c"
#include "uvc_v4l2.c"

/*-------------------------------------------------------------------------*/

unsigned int uvc_gadget_trace_param;

#ifndef MAX_STRING_NAME_LENGTH
#define MAX_STRING_NAME_LENGTH		64
#endif
//#define USE_NV12 0
struct webcam_config {
	int	device;
	struct device *dev;
	bool *soft_connected_ptr;

	unsigned int interval;
	unsigned int maxpacket;
	unsigned int maxburst;
	unsigned char headersize;
	unsigned char bulkmode;
	unsigned int bulksize;
	unsigned int maxpayload;
	unsigned int usbreqnums;

	char video_iad_string[MAX_STRING_NAME_LENGTH];
	char video_control_string[MAX_STRING_NAME_LENGTH];
	char video_stream_string[MAX_STRING_NAME_LENGTH];
};

static struct webcam_config *_webcam_config = NULL;

/* string IDs are assigned dynamically */

#define UVC_STRING_IAD_IDX				0
#define UVC_STRING_CONTROL_IDX			1
#define UVC_STRING_STREAMING_IDX		2

static struct usb_string uvc_en_us_strings[] = {
	[UVC_STRING_IAD_IDX].s = "UVC_Camera",
	[UVC_STRING_CONTROL_IDX].s = "UVC Camera",
	[UVC_STRING_STREAMING_IDX].s = "Video Streaming",
	{  }
};

static struct usb_gadget_strings uvc_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = uvc_en_us_strings,
};

static struct usb_gadget_strings *uvc_function_strings[] = {
	&uvc_stringtab,
	NULL,
};

/* uvc control and stream settings */

DECLARE_UVC_HEADER_DESCRIPTOR(1);

static const struct UVC_HEADER_DESCRIPTOR(1) uvc_control_header = {
	.bLength		= UVC_DT_HEADER_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_HEADER,
	.bcdUVC			= cpu_to_le16(0x0100),
	.wTotalLength		= 0, /* dynamic */
	.dwClockFrequency	= cpu_to_le32(48000000),
	.bInCollection		= 0, /* dynamic */
	.baInterfaceNr[0]	= 0, /* dynamic */
};

static const struct uvc_camera_terminal_descriptor uvc_camera_terminal = {
	.bLength		= UVC_DT_CAMERA_TERMINAL_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_INPUT_TERMINAL,
	.bTerminalID		= 1,
	.wTerminalType		= cpu_to_le16(0x0201),
	.bAssocTerminal		= 0,
	.iTerminal		= 0,
	.wObjectiveFocalLengthMin	= cpu_to_le16(0),
	.wObjectiveFocalLengthMax	= cpu_to_le16(0),
	.wOcularFocalLength		= cpu_to_le16(0),
	.bControlSize		= 3,
	.bmControls[0]		= 0,
	.bmControls[1]		= 0,
	.bmControls[2]		= 0,
};

static const struct uvc_processing_unit_descriptor uvc_processing = {
	.bLength		= UVC_DT_PROCESSING_UNIT_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_PROCESSING_UNIT,
	.bUnitID		= 2,
	.bSourceID		= 1,
	.wMaxMultiplier		= cpu_to_le16(16*1024),
	.bControlSize		= 2,
	.bmControls[0]		= 0,
	.bmControls[1]		= 0,
	.iProcessing		= 0,
};

static const struct uvc_output_terminal_descriptor uvc_output_terminal = {
	.bLength		= UVC_DT_OUTPUT_TERMINAL_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_OUTPUT_TERMINAL,
	.bTerminalID		= 3,
	.wTerminalType		= cpu_to_le16(0x0101),
	.bAssocTerminal		= 0,
	.bSourceID		= 2,
	.iTerminal		= 0,
};

DECLARE_UVC_INPUT_HEADER_DESCRIPTOR(1, 2);

static const struct UVC_INPUT_HEADER_DESCRIPTOR(1, 2) uvc_input_header_2 = {
	.bLength		= UVC_DT_INPUT_HEADER_SIZE(1, 2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_INPUT_HEADER,
	.bNumFormats		= 2,
	.wTotalLength		= 0, /* dynamic */
	.bEndpointAddress	= 0, /* dynamic */
	.bmInfo			= 0,
	.bTerminalLink		= 3,
	.bStillCaptureMethod	= 0,
	.bTriggerSupport	= 0,
	.bTriggerUsage		= 0,
	.bControlSize		= 1,
	.bmaControls[0][0]	= 0,
	.bmaControls[1][0]	= 4,
};


DECLARE_UVC_INPUT_HEADER_DESCRIPTOR(1, 3);

static const struct UVC_INPUT_HEADER_DESCRIPTOR(1, 3) uvc_input_header_3 = {
	.bLength		= UVC_DT_INPUT_HEADER_SIZE(1, 3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_INPUT_HEADER,
	.bNumFormats		= 3,
	.wTotalLength		= 0, /* dynamic */
	.bEndpointAddress	= 0, /* dynamic */
	.bmInfo			= 0,
	.bTerminalLink		= 3,
	.bStillCaptureMethod	= 0,
	.bTriggerSupport	= 0,
	.bTriggerUsage		= 0,
	.bControlSize		= 1,
	.bmaControls[0][0]	= 0,
	.bmaControls[1][0]	= 0,
	.bmaControls[2][0]	= 4,
};

DECLARE_UVC_INPUT_HEADER_DESCRIPTOR(1, 4);

static const struct UVC_INPUT_HEADER_DESCRIPTOR(1, 4) uvc_input_header_4 = {
	.bLength		= UVC_DT_INPUT_HEADER_SIZE(1, 4),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_INPUT_HEADER,
	.bNumFormats		= 4,
	.wTotalLength		= 0, /* dynamic */
	.bEndpointAddress	= 0, /* dynamic */
	.bmInfo			= 0,
	.bTerminalLink		= 3,
	.bStillCaptureMethod	= 0,
	.bTriggerSupport	= 0,
	.bTriggerUsage		= 0,
	.bControlSize		= 1,
	.bmaControls[0][0]	= 0,
	.bmaControls[1][0]	= 0,
	.bmaControls[2][0]	= 4,
};


static const struct uvc_format_uncompressed uvc_format_yuv = {
	.bLength		= UVC_DT_FORMAT_UNCOMPRESSED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_UNCOMPRESSED,
	.bFormatIndex		= 1,
	.bNumFrameDescriptors	= 2,
	.guidFormat		=
	{
		'Y',  'U',  'Y',  '2', 0x00, 0x00, 0x10, 0x00,
		0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
	},
	.bBitsPerPixel		= 16,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};

DECLARE_UVC_FRAME_UNCOMPRESSED(1);
DECLARE_UVC_FRAME_UNCOMPRESSED(2);

static const struct UVC_FRAME_UNCOMPRESSED(2) uvc_frame_yuv_360p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(360),
	.dwMinBitRate		= cpu_to_le32(110592000),
	.dwMaxBitRate		= cpu_to_le32(110592000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(666666),
	.bFrameIntervalType	= 2,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
};

static const struct UVC_FRAME_UNCOMPRESSED(2) uvc_frame_yuv_480p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(480),
	.dwMinBitRate		= cpu_to_le32(147456000),
	.dwMaxBitRate		= cpu_to_le32(147456000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(614400),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 1,
	.bFrameIntervalType = 2,
	.dwFrameInterval[0] = cpu_to_le32(333333),
	.dwFrameInterval[1] = cpu_to_le32(666666),

};

/******NOT IN USE*******/
static const struct UVC_FRAME_UNCOMPRESSED(2) uvc_frame_yuv_720p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(368640000),
	.dwMaxBitRate		= cpu_to_le32(442368000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(500000),
	.bFrameIntervalType	= 2,
	.dwFrameInterval[0]	= cpu_to_le32(400000),	/* 25 fps. */
	.dwFrameInterval[1]	= cpu_to_le32(666666),	/* 15 fps. */
};
/******NOT IN USE*******/
static const struct uvc_format_uncompressed uvc_format_i420 = {
	.bLength		= UVC_DT_FORMAT_UNCOMPRESSED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_UNCOMPRESSED,
	.bFormatIndex		= 2,
	.bNumFrameDescriptors	= 2,
	.guidFormat		=
	{
		'I',  '4',  '2',  '0', 0x00, 0x00, 0x10, 0x00,
		0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
	},
	.bBitsPerPixel		= 12,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};
/******NOT IN USE*******/
static const struct UVC_FRAME_UNCOMPRESSED(2) uvc_frame_i420_360p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(360),
	.dwMinBitRate		= cpu_to_le32(82944000),
	.dwMaxBitRate		= cpu_to_le32(82944000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(345600),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 2,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1] = cpu_to_le32(666666),
};
/******NOT IN USE*******/
static const struct UVC_FRAME_UNCOMPRESSED(2) uvc_frame_i420_480p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1382400),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType = 2,
	.dwFrameInterval[0] = cpu_to_le32(333333),
	.dwFrameInterval[1] = cpu_to_le32(666666),

};
/******NOT IN USE*******/
static const struct UVC_FRAME_UNCOMPRESSED(2) uvc_frame_i420_720p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(331776000),
	.dwMaxBitRate		= cpu_to_le32(331776000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1382400),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 2,
	.dwFrameInterval[0]	= cpu_to_le32(666666),
	.dwFrameInterval[1]	= cpu_to_le32(1000000),
};
/******NOT IN USE*******/
static const struct uvc_format_uncompressed uvc_format_nv12 = {
	.bLength		= UVC_DT_FORMAT_UNCOMPRESSED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_UNCOMPRESSED,
	.bFormatIndex		= 4,
	.bNumFrameDescriptors	= 3,
	.guidFormat		=
	{
		'N',  'V',  '1',  '2', 0x00, 0x00, 0x10, 0x00,
		0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
	},
	.bBitsPerPixel		= 12,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 1,
	.bCopyProtect		= 0,
};

DECLARE_UVC_FRAME_UNCOMPRESSED(4);
/******NOT IN USE*******/
static const struct UVC_FRAME_UNCOMPRESSED(4) uvc_frame_nv12_360p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(4),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(360),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1382400),
	.dwDefaultFrameInterval	= cpu_to_le32(5000000),
	.bFrameIntervalType	= 4,
	.dwFrameInterval[0] = cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
	.dwFrameInterval[3]	= cpu_to_le32(5000000),
};

/******NOT IN USE*******/
static const struct UVC_FRAME_UNCOMPRESSED(4) uvc_frame_nv12_480p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(4),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(480),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(3110400),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 4,
	.dwFrameInterval[0] = cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
	.dwFrameInterval[3]	= cpu_to_le32(5000000),
};
/******NOT IN USE*******/
static const struct UVC_FRAME_UNCOMPRESSED(4) uvc_frame_nv12_720p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(4),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(3110400),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 4,
	.dwFrameInterval[0] = cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
	.dwFrameInterval[3]	= cpu_to_le32(5000000),
};

static const struct uvc_format_mjpeg uvc_format_mjpg = {
	.bLength		= UVC_DT_FORMAT_MJPEG_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_MJPEG,
	.bFormatIndex		= 3,
	.bNumFrameDescriptors	= 3,
	.bmFlags		= 0,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};


DECLARE_UVC_FRAME_MJPEG(2);

static const struct UVC_FRAME_MJPEG(2) uvc_frame_mjpg_360p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(640),
	.wHeight		= cpu_to_le16(360),
	.dwMinBitRate		= cpu_to_le32(18432000),
	.dwMaxBitRate		= cpu_to_le32(55296000),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(460800),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 2,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
};

static const struct UVC_FRAME_MJPEG(2) uvc_frame_mjpg_720p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1843200),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 2,
	.dwFrameInterval[0] = cpu_to_le32(333333),
	.dwFrameInterval[1] = cpu_to_le32(666666),
};

static const struct UVC_FRAME_MJPEG(2) uvc_frame_mjpg_1080p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(29491200),
	.dwMaxBitRate		= cpu_to_le32(29491200),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(4147200),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 2,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1] = cpu_to_le32(666666),
};

static const struct uvc_color_matching_descriptor uvc_color_matching = {
	.bLength		= UVC_DT_COLOR_MATCHING_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_COLORFORMAT,
	.bColorPrimaries	= 1,
	.bTransferCharacteristics	= 1,
	.bMatrixCoefficients	= 4,
};

static const struct uvc_descriptor_header * const uvc_fs_control_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_control_header,
	(const struct uvc_descriptor_header *) &uvc_camera_terminal,
	(const struct uvc_descriptor_header *) &uvc_processing,
	(const struct uvc_descriptor_header *) &uvc_output_terminal,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_ss_control_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_control_header,
	(const struct uvc_descriptor_header *) &uvc_camera_terminal,
	(const struct uvc_descriptor_header *) &uvc_processing,
	(const struct uvc_descriptor_header *) &uvc_output_terminal,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_fs_streaming_cls[] = {
#ifdef USE_NV12
	(const struct uvc_descriptor_header *) &uvc_input_header_4,
#else
	(const struct uvc_descriptor_header *) &uvc_input_header_2,
#endif
	(const struct uvc_descriptor_header *) &uvc_format_yuv,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_480p,
	(const struct uvc_descriptor_header *) &uvc_format_mjpg,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_720p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_1080p,
#ifdef USE_NV12
	(const struct uvc_descriptor_header *) &uvc_format_nv12,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_480p,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_720p,
#endif
	(const struct uvc_descriptor_header *) &uvc_color_matching,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_hs_streaming_cls[] = {
#ifdef USE_NV12
	(const struct uvc_descriptor_header *) &uvc_input_header_4,
#else
	(const struct uvc_descriptor_header *) &uvc_input_header_2,
#endif
	(const struct uvc_descriptor_header *) &uvc_format_yuv,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_480p,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_360p,
	(const struct uvc_descriptor_header *) &uvc_format_mjpg,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_720p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_1080p,
#ifdef USE_NV12
	(const struct uvc_descriptor_header *) &uvc_format_nv12,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_480p,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_720p,
#endif
	(const struct uvc_descriptor_header *) &uvc_color_matching,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_ss_streaming_cls[] = {
#ifdef USE_NV12
	(const struct uvc_descriptor_header *) &uvc_input_header_4,
#else
	(const struct uvc_descriptor_header *) &uvc_input_header_3,
#endif
	(const struct uvc_descriptor_header *) &uvc_format_yuv,
	(const struct uvc_descriptor_header *) &uvc_frame_yuv_360p,
	(const struct uvc_descriptor_header *) &uvc_format_i420,
	(const struct uvc_descriptor_header *) &uvc_frame_i420_360p,
	(const struct uvc_descriptor_header *) &uvc_format_mjpg,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_720p,
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_1080p,
#ifdef USE_NV12
	(const struct uvc_descriptor_header *) &uvc_format_nv12,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_360p,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_480p,
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_720p,
#endif
	(const struct uvc_descriptor_header *) &uvc_color_matching,
	NULL,
};

/* uvc interface and endpoint settings */

#define UVC_INTF_VIDEO_CONTROL			0
#define UVC_INTF_VIDEO_STREAMING		1

#define UVC_STATUS_MAX_PACKET_SIZE		16	/* 16 bytes status */

static struct usb_interface_assoc_descriptor uvc_iad = {
	.bLength		= sizeof(uvc_iad),
	.bDescriptorType	= USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface	= 0,
	.bInterfaceCount	= 2,
	.bFunctionClass		= USB_CLASS_VIDEO,
	.bFunctionSubClass	= UVC_SC_VIDEO_INTERFACE_COLLECTION,
	.bFunctionProtocol	= 0x00,
	.iFunction		= 0,
};

static struct usb_interface_descriptor uvc_control_intf = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_CONTROL,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOCONTROL,
	.bInterfaceProtocol	= 0x00,
	.iInterface		= 0,
};

static struct usb_endpoint_descriptor uvc_control_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize		= cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
	.bInterval		= 8,
};

static struct usb_ss_ep_comp_descriptor uvc_ss_control_comp = {
	.bLength		= sizeof(uvc_ss_control_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	/* The following 3 values can be tweaked if necessary. */
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
	.wBytesPerInterval	= cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
};

static struct uvc_control_endpoint_descriptor uvc_control_cs_ep = {
	.bLength		= UVC_DT_CONTROL_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_CS_ENDPOINT,
	.bDescriptorSubType	= UVC_EP_INTERRUPT,
	.wMaxTransferSize	= cpu_to_le16(UVC_STATUS_MAX_PACKET_SIZE),
};

static struct usb_interface_descriptor uvc_streaming_intf_alt0 = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_STREAMING,
	.bAlternateSetting	= 0,
	.bNumEndpoints		= 0,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOSTREAMING,
	.bInterfaceProtocol	= 0x00,
	.iInterface		= 0,
};

static struct usb_interface_descriptor uvc_streaming_intf_alt1 = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber	= UVC_INTF_VIDEO_STREAMING,
	.bAlternateSetting	= 1,
	.bNumEndpoints		= 1,
	.bInterfaceClass	= USB_CLASS_VIDEO,
	.bInterfaceSubClass	= UVC_SC_VIDEOSTREAMING,
	.bInterfaceProtocol	= 0x00,
	.iInterface		= 0,
};

static struct usb_endpoint_descriptor uvc_fs_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
	.wMaxPacketSize		= 0,
	.bInterval		= 0,
};

static struct usb_endpoint_descriptor uvc_hs_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
	.wMaxPacketSize		= 0,
	.bInterval		= 0,
};

static struct usb_endpoint_descriptor uvc_ss_streaming_ep = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,

	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_SYNC_ASYNC
				| USB_ENDPOINT_XFER_ISOC,
	/* The wMaxPacketSize and bInterval values will be initialized from
	 * module parameters.
	 */
	.wMaxPacketSize		= 0,
	.bInterval		= 0,
};

static struct usb_ss_ep_comp_descriptor uvc_ss_streaming_comp = {
	.bLength		= sizeof(uvc_ss_streaming_comp),
	.bDescriptorType	= USB_DT_SS_ENDPOINT_COMP,
	/* The following 3 values can be tweaked if necessary. */
	.bMaxBurst		= 0,
	.bmAttributes		= 0,
	.wBytesPerInterval	= cpu_to_le16(1024),
};

static const struct usb_descriptor_header * const uvc_fs_streaming[] = {
	(struct usb_descriptor_header *) &uvc_fs_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_hs_streaming[] = {
	(struct usb_descriptor_header *) &uvc_hs_streaming_ep,
	NULL,
};

static const struct usb_descriptor_header * const uvc_ss_streaming[] = {
	(struct usb_descriptor_header *) &uvc_ss_streaming_ep,
	(struct usb_descriptor_header *) &uvc_ss_streaming_comp,
	NULL,
};

/* --------------------------------------------------------------------------
 * USB configuration
 */
/* --------------------------------------------------------------------------
 * Control requests
 */

static void
uvc_function_ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct uvc_device *uvc = req->context;
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;

	if (uvc->event_setup_out) {
		uvc->event_setup_out = 0;

		memset(&v4l2_event, 0, sizeof(v4l2_event));
		v4l2_event.type = UVC_EVENT_DATA;
		uvc_event->data.length = req->actual;
		memcpy(&uvc_event->data.data, req->buf, req->actual);
		v4l2_event_queue(uvc->vdev, &v4l2_event);
	}
}

static int
uvc_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
	struct usb_request *req = uvc->control_req;
	struct usb_composite_dev *cdev = uvc->func.config->cdev;
	int ret;

	/* printk(KERN_INFO "setup request %02x %02x value %04x index %04x %04x\n",
	 *	ctrl->bRequestType, ctrl->bRequest, le16_to_cpu(ctrl->wValue),
	 *	le16_to_cpu(ctrl->wIndex), le16_to_cpu(ctrl->wLength));
	 */

	if ((ctrl->bRequestType & USB_TYPE_MASK) != USB_TYPE_CLASS) {
		INFO(f->config->cdev, "invalid request type\n");
		return -EINVAL;
	}

	/* Stall too big requests. */
	if (le16_to_cpu(ctrl->wLength) > UVC_MAX_REQUEST_SIZE)
		return -EINVAL;

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_SETUP;
	memcpy(&uvc_event->req, ctrl, sizeof(uvc_event->req));
	v4l2_event_queue(uvc->vdev, &v4l2_event);

	/* Tell the complete callback to generate an event for
	 * the next request that will be enqueued by
	 * uvc_event_write.
	 */
	 uvc->event_setup_out =
		!(uvc_event->req.bRequestType & USB_DIR_IN);

	if (uvc->event_setup_out) {
		DBG(f->config->cdev, "uvc_function_setup: queue the Control OUT SETUP immediatly. %s\n",
			(ctrl->wValue >> 8) == UVC_VS_PROBE_CONTROL ? "PROBE" : ((ctrl->wValue >> 8) == UVC_VS_COMMIT_CONTROL ? "COMMIT" : "UNDEFINED"));
		req->length = le16_to_cpu(ctrl->wLength);
		req->zero = 0;
		req->context = uvc;

		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (ret < 0)
			ERROR(f->config->cdev, "Failed to queue the ep0\n");
	}

	return 0;
}

void uvc_function_setup_continue(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;

	usb_composite_setup_continue(cdev);
}

static int
uvc_function_get_alt(struct usb_function *f, unsigned interface)
{
	struct uvc_device *uvc = to_uvc(f);

	INFO(f->config->cdev, "uvc_function_get_alt(%u)\n", interface);

	if (interface == uvc->control_intf)
		return 0;
	else if (interface != uvc->streaming_intf)
		return -EINVAL;
	else
		return uvc->state == UVC_STATE_STREAMING ? 1 : 0;
}

static int
uvc_function_set_alt(struct usb_function *f, unsigned interface, unsigned alt)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;
	struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
	int ret;

	INFO(f->config->cdev, "uvc_function_set_alt(%u, %u)\n", interface, alt);

	if (interface == uvc->control_intf) {
		if (alt)
			return -EINVAL;

		if (uvc->state == UVC_STATE_DISCONNECTED) {
			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_CONNECT;
			uvc_event->speed = f->config->cdev->gadget->speed;
			v4l2_event_queue(uvc->vdev, &v4l2_event);

			uvc->state = UVC_STATE_CONNECTED;
		}

		return 0;
	}

	if (interface != uvc->streaming_intf)
		return -EINVAL;

	/* TODO
	if (usb_endpoint_xfer_bulk(&uvc->desc.vs_ep))
		return alt ? -EINVAL : 0;
	*/
	if (uvc->video.bulk_mode) {
		if (alt)
			return -EINVAL;

		if (uvc->video.ep) {
			/* Restart Endpoint. */
			usb_ep_disable(uvc->video.ep);

			ret = config_ep_by_speed(f->config->cdev->gadget,
					&(uvc->func), uvc->video.ep);
			if (ret)
				return ret;

			usb_ep_enable(uvc->video.ep);
		}

		uvc->state = UVC_STATE_CONNECTED;

		return 0;
	} else {
		switch (alt) {
		case 0:
			if (uvc->state != UVC_STATE_STREAMING)
				return 0;

			if (uvc->video.ep)
				usb_ep_disable(uvc->video.ep);

			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_STREAMOFF;
			v4l2_event_queue(uvc->vdev, &v4l2_event);

			uvc->state = UVC_STATE_CONNECTED;
			return 0;

		case 1:
			if (uvc->state != UVC_STATE_CONNECTED)
				return 0;

			if (uvc->video.ep) {
				ret = config_ep_by_speed(f->config->cdev->gadget,
						&(uvc->func), uvc->video.ep);
				if (ret)
					return ret;
				usb_ep_enable(uvc->video.ep);
			}

			memset(&v4l2_event, 0, sizeof(v4l2_event));
			v4l2_event.type = UVC_EVENT_STREAMON;
			v4l2_event_queue(uvc->vdev, &v4l2_event);
			return USB_GADGET_DELAYED_STATUS;

		default:
			return -EINVAL;
		}
	}
}

static void
uvc_function_resume(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);

	INFO(f->config->cdev, "uvc_function_resume\n");

	uvc->suspended = 0;
}

static void
uvc_function_suspend(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;

	INFO(f->config->cdev, "uvc_function_suspend\n");

	uvc->suspended = 1;

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_PLUGOUT;
	v4l2_event_queue(uvc->vdev, &v4l2_event);

	uvc->state = UVC_STATE_DISCONNECTED;
}

static void
uvc_function_disable(struct usb_function *f)
{
	struct uvc_device *uvc = to_uvc(f);
	struct v4l2_event v4l2_event;

	INFO(f->config->cdev, "uvc_function_disable\n");

	memset(&v4l2_event, 0, sizeof(v4l2_event));
	v4l2_event.type = UVC_EVENT_DISCONNECT;
	v4l2_event_queue(uvc->vdev, &v4l2_event);

	uvc->state = UVC_STATE_DISCONNECTED;
}

/* --------------------------------------------------------------------------
 * Connection / disconnection
 */

void
uvc_function_connect(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;
	int ret;

	/* If the global switch is off. Do not connect here. */
	if (_webcam_config && _webcam_config->soft_connected_ptr
		&& (!*(_webcam_config->soft_connected_ptr)))
		return;

	if ((ret = usb_function_activate(&uvc->func)) < 0)
		INFO(cdev, "UVC connect failed with %d\n", ret);
}

void
uvc_function_disconnect(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;
	int ret;

	/* If the global switch is off. Do not do it again here. */
	if (_webcam_config && _webcam_config->soft_connected_ptr
		&& (!*(_webcam_config->soft_connected_ptr)))
		return;

	if ((ret = usb_function_deactivate(&uvc->func)) < 0)
		INFO(cdev, "UVC disconnect failed with %d\n", ret);
}

/* --------------------------------------------------------------------------
 * USB probe and disconnect
 */

static int
uvc_register_video(struct uvc_device *uvc)
{
	struct usb_composite_dev *cdev = uvc->func.config->cdev;
	struct video_device *video;
	int ret = 0;

	/* TODO reference counting. */
	video = video_device_alloc();
	if (video == NULL)
		return -ENOMEM;

	video->parent = &cdev->gadget->dev;
	video->fops = &uvc_v4l2_fops;
	video->release = video_device_release;
	strlcpy(video->name, cdev->gadget->name, sizeof(video->name));

	uvc->vdev = video;
	video_set_drvdata(video, uvc);

	ret = video_register_device(video, VFL_TYPE_GRABBER, -1);

	if (_webcam_config)
		_webcam_config->device = video->num;

	return ret;
}

#define UVC_COPY_DESCRIPTOR(mem, dst, desc) \
	do { \
		memcpy(mem, desc, (desc)->bLength); \
		*(dst)++ = mem; \
		mem += (desc)->bLength; \
	} while (0);

#define UVC_COPY_DESCRIPTORS(mem, dst, src) \
	do { \
		const struct usb_descriptor_header * const *__src; \
		for (__src = src; *__src; ++__src) { \
			memcpy(mem, *__src, (*__src)->bLength); \
			*dst++ = mem; \
			mem += (*__src)->bLength; \
		} \
	} while (0)

static struct usb_descriptor_header **
uvc_copy_descriptors(struct uvc_device *uvc, enum usb_device_speed speed)
{
	struct uvc_input_header_descriptor *uvc_streaming_header;
	struct uvc_header_descriptor *uvc_control_header;
	const struct uvc_descriptor_header * const *uvc_control_desc;
	const struct uvc_descriptor_header * const *uvc_streaming_cls;
	const struct usb_descriptor_header * const *uvc_streaming_std;
	const struct usb_descriptor_header * const *src;
	struct usb_descriptor_header **dst;
	struct usb_descriptor_header **hdr;
	unsigned int control_size;
	unsigned int streaming_size;
	unsigned int n_desc;
	unsigned int bytes;
	void *mem;

	switch (speed) {
	case USB_SPEED_SUPER:
		uvc_control_desc = uvc->desc.ss_control;
		uvc_streaming_cls = uvc->desc.ss_streaming;
		uvc_streaming_std = uvc_ss_streaming;
		break;

	case USB_SPEED_HIGH:
		uvc_control_desc = uvc->desc.fs_control;
		uvc_streaming_cls = uvc->desc.hs_streaming;
		uvc_streaming_std = uvc_hs_streaming;
		break;

	case USB_SPEED_FULL:
	default:
		uvc_control_desc = uvc->desc.fs_control;
		uvc_streaming_cls = uvc->desc.fs_streaming;
		uvc_streaming_std = uvc_fs_streaming;
		break;
	}

	/* Descriptors layout
	 *
	 * uvc_iad
	 * uvc_control_intf
	 * Class-specific UVC control descriptors
	 * uvc_control_ep
	 * uvc_control_cs_ep
	 * uvc_ss_control_comp (for SS only)
	 * uvc_streaming_intf_alt0
	 * Class-specific UVC streaming descriptors
	 * uvc_{fs|hs}_streaming
	 */

	/* Count descriptors and compute their size. */
	control_size = 0;
	streaming_size = 0;
	bytes = uvc_iad.bLength + uvc_control_intf.bLength
	      + uvc_control_ep.bLength + uvc_control_cs_ep.bLength
	      + uvc_streaming_intf_alt0.bLength
	      + uvc_streaming_intf_alt1.bLength;

	if (speed == USB_SPEED_SUPER) {
		bytes += uvc_ss_control_comp.bLength;
		n_desc = 7;
	} else {
		n_desc = 6;
	}

	if (_webcam_config && _webcam_config->bulkmode) {
		/*In Bulk mode, no alt0 needed. */
		bytes -= uvc_streaming_intf_alt0.bLength;
		n_desc--;
	}

	for (src = (const struct usb_descriptor_header **)uvc_control_desc;
	     *src; ++src) {
		control_size += (*src)->bLength;
		bytes += (*src)->bLength;
		n_desc++;
	}
	for (src = (const struct usb_descriptor_header **)uvc_streaming_cls;
	     *src; ++src) {
		streaming_size += (*src)->bLength;
		bytes += (*src)->bLength;
		n_desc++;
	}
	for (src = uvc_streaming_std; *src; ++src) {
		bytes += (*src)->bLength;
		n_desc++;
	}

	mem = kmalloc((n_desc + 1) * sizeof(*src) + bytes, GFP_KERNEL);
	if (mem == NULL)
		return NULL;

	hdr = mem;
	dst = mem;
	mem += (n_desc + 1) * sizeof(*src);

	/* Copy the descriptors. */
	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_iad);
	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_intf);

	uvc_control_header = mem;
	UVC_COPY_DESCRIPTORS(mem, dst,
		(const struct usb_descriptor_header **)uvc_control_desc);
	uvc_control_header->wTotalLength = cpu_to_le16(control_size);
	uvc_control_header->bInCollection = 1;
	uvc_control_header->baInterfaceNr[0] = uvc->streaming_intf;

	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_ep);
	if (speed == USB_SPEED_SUPER)
		UVC_COPY_DESCRIPTOR(mem, dst, &uvc_ss_control_comp);

	UVC_COPY_DESCRIPTOR(mem, dst, &uvc_control_cs_ep);

	if (_webcam_config && _webcam_config->bulkmode) {
		uvc_streaming_intf_alt1.bAlternateSetting = 0;
		UVC_COPY_DESCRIPTOR(mem, dst, &uvc_streaming_intf_alt1);
	} else {
		UVC_COPY_DESCRIPTOR(mem, dst, &uvc_streaming_intf_alt0);
		uvc_streaming_intf_alt1.bAlternateSetting = 1;
	}

	uvc_streaming_header = mem;
	UVC_COPY_DESCRIPTORS(mem, dst,
		(const struct usb_descriptor_header**)uvc_streaming_cls);
	uvc_streaming_header->wTotalLength = cpu_to_le16(streaming_size);
	uvc_streaming_header->bEndpointAddress = uvc->video.ep->address;

	if (!_webcam_config || !_webcam_config->bulkmode) {
		UVC_COPY_DESCRIPTOR(mem, dst, &uvc_streaming_intf_alt1);
	}

	UVC_COPY_DESCRIPTORS(mem, dst, uvc_streaming_std);

	*dst = NULL;
	return hdr;
}

static void
uvc_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct uvc_device *uvc = to_uvc(f);

	INFO(cdev, "uvc_function_unbind\n");

	video_unregister_device(uvc->vdev);
	uvc->control_ep->driver_data = NULL;
	uvc->video.ep->driver_data = NULL;

	uvc_en_us_strings[UVC_STRING_CONTROL_IDX].id = 0;
	usb_ep_free_request(cdev->gadget->ep0, uvc->control_req);
	kfree(uvc->control_buf);

	usb_free_all_descriptors(f);

	kfree(uvc);
}

static int
uvc_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct uvc_device *uvc = to_uvc(f);
	unsigned int max_packet_mult;
	unsigned int max_packet_size;
	struct usb_ep *ep;
	int ret = -EINVAL;
	unsigned int streaming_interval = 1;
	unsigned int streaming_maxpacket = 1024;
	unsigned int streaming_maxburst = 0;
	unsigned char headersize = UVC_DEFAULT_PAYLOAD_HEADER_SIZE;
	unsigned char bulkmode = 0;
	unsigned int bulksize = UVC_DEFAULT_BULK_REQ_BUFFER_SIZE;
	unsigned int maxpayload = UVC_DEFAULT_MAX_PAYLOAD_SIZE;
	unsigned int usbreqnums = UVC_MAX_REQUEST_SIZE;

	INFO(cdev, "uvc_function_bind\n");

	/* Sanity check the streaming endpoint module parameters.
	 */
	if (_webcam_config) {
		streaming_interval = clamp(_webcam_config->interval, 1U, 16U);
		streaming_maxpacket = clamp(_webcam_config->maxpacket, 1U, 3072U);
		streaming_maxburst = min(_webcam_config->maxburst, 15U);
		headersize = clamp(_webcam_config->headersize, (unsigned char)2U, (unsigned char)255U);
		bulkmode = _webcam_config->bulkmode;
		bulksize = _webcam_config->bulksize;
		maxpayload = _webcam_config->maxpayload;
		usbreqnums = _webcam_config->usbreqnums;
	}

	/* Fill in the FS/HS/SS Video Streaming specific descriptors from the
	 * module parameters.
	 *
	 * NOTE: We assume that the user knows what they are doing and won't
	 * give parameters that their UDC doesn't support.
	 */
	if (streaming_maxpacket <= 1024) {
		max_packet_mult = 1;
		max_packet_size = streaming_maxpacket;
	} else if (streaming_maxpacket <= 2048) {
		max_packet_mult = 2;
		max_packet_size = streaming_maxpacket / 2;
	} else {
		max_packet_mult = 3;
		max_packet_size = streaming_maxpacket / 3;
	}

	uvc_fs_streaming_ep.wMaxPacketSize = min(streaming_maxpacket, 1023U);
	uvc_fs_streaming_ep.bInterval = streaming_interval;

	uvc_hs_streaming_ep.wMaxPacketSize = max_packet_size;
	uvc_hs_streaming_ep.wMaxPacketSize |= ((max_packet_mult - 1) << 11);
	uvc_hs_streaming_ep.bInterval = streaming_interval;

	uvc_ss_streaming_ep.wMaxPacketSize = max_packet_size;
	uvc_ss_streaming_ep.bInterval = streaming_interval;
	uvc_ss_streaming_comp.bmAttributes = max_packet_mult - 1;
	uvc_ss_streaming_comp.bMaxBurst = streaming_maxburst;
	uvc_ss_streaming_comp.wBytesPerInterval =
		max_packet_size * max_packet_mult * streaming_maxburst;

	if (bulkmode) {
		/* TODO Fix the maxpacket size in bulk. */
		uvc_fs_streaming_ep.bmAttributes = USB_ENDPOINT_XFER_BULK;
		uvc_fs_streaming_ep.wMaxPacketSize = clamp((unsigned short)max_packet_size,
			(unsigned short)1U, (unsigned short)64U);
		uvc_fs_streaming_ep.bInterval = 0;

		uvc_hs_streaming_ep.bmAttributes = USB_ENDPOINT_XFER_BULK;
		uvc_hs_streaming_ep.wMaxPacketSize = clamp((unsigned short)max_packet_size,
			(unsigned short)1U, (unsigned short)512U);
		uvc_hs_streaming_ep.bInterval = 0;

		uvc_ss_streaming_ep.bmAttributes = USB_ENDPOINT_XFER_BULK;
		uvc_ss_streaming_ep.wMaxPacketSize = clamp((unsigned short)max_packet_size,
			(unsigned short)1U, (unsigned short)1024U);
		uvc_ss_streaming_ep.bInterval = 0;
	} else {
		uvc_fs_streaming_ep.bmAttributes = USB_ENDPOINT_SYNC_ASYNC
									| USB_ENDPOINT_XFER_ISOC;

		uvc_hs_streaming_ep.bmAttributes = USB_ENDPOINT_SYNC_ASYNC
									| USB_ENDPOINT_XFER_ISOC;

		uvc_ss_streaming_ep.bmAttributes = USB_ENDPOINT_SYNC_ASYNC
									| USB_ENDPOINT_XFER_ISOC;
	}

	pr_info("max_packet_size, fs: %d, hs: %d, ss: %d\n",
		uvc_fs_streaming_ep.wMaxPacketSize,
		uvc_hs_streaming_ep.wMaxPacketSize,
		uvc_ss_streaming_ep.wMaxPacketSize);

	/* Allocate endpoints. */
	ep = usb_ep_autoconfig(cdev->gadget, &uvc_control_ep);
	if (!ep) {
		INFO(cdev, "Unable to allocate control EP\n");
		goto error;
	}
	uvc->control_ep = ep;
	ep->driver_data = uvc;

	if (gadget_is_superspeed(c->cdev->gadget))
		ep = usb_ep_autoconfig_ss(cdev->gadget, &uvc_ss_streaming_ep,
					  &uvc_ss_streaming_comp);
	else if (gadget_is_dualspeed(cdev->gadget))
		ep = usb_ep_autoconfig(cdev->gadget, &uvc_hs_streaming_ep);
	else
		ep = usb_ep_autoconfig(cdev->gadget, &uvc_fs_streaming_ep);

	if (!ep) {
		INFO(cdev, "Unable to allocate streaming EP\n");
		goto error;
	}
	uvc->video.ep = ep;
	ep->driver_data = uvc;

	uvc_fs_streaming_ep.bEndpointAddress = uvc->video.ep->address;
	uvc_hs_streaming_ep.bEndpointAddress = uvc->video.ep->address;
	uvc_ss_streaming_ep.bEndpointAddress = uvc->video.ep->address;

	pr_info("max_packet_size, fs: %d, hs: %d, ss: %d\n",
		uvc_fs_streaming_ep.wMaxPacketSize,
		uvc_hs_streaming_ep.wMaxPacketSize,
		uvc_ss_streaming_ep.wMaxPacketSize);

	/* Allocate interface IDs. */
	if ((ret = usb_interface_id(c, f)) < 0)
		goto error;
	uvc_iad.bFirstInterface = ret;
	uvc_control_intf.bInterfaceNumber = ret;
	uvc->control_intf = ret;

	if ((ret = usb_interface_id(c, f)) < 0)
		goto error;
	uvc_streaming_intf_alt0.bInterfaceNumber = ret;
	uvc_streaming_intf_alt1.bInterfaceNumber = ret;
	uvc->streaming_intf = ret;

	/* Copy descriptors */
	f->fs_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_FULL);
	if (gadget_is_dualspeed(cdev->gadget))
		f->hs_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_HIGH);
	if (gadget_is_superspeed(c->cdev->gadget))
		f->ss_descriptors = uvc_copy_descriptors(uvc, USB_SPEED_SUPER);

	/* Preallocate control endpoint request. */
	uvc->control_req = usb_ep_alloc_request(cdev->gadget->ep0, GFP_KERNEL);
	uvc->control_buf = kmalloc(UVC_MAX_REQUEST_SIZE, GFP_KERNEL);
	if (uvc->control_req == NULL || uvc->control_buf == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	uvc->control_req->buf = uvc->control_buf;
	uvc->control_req->complete = uvc_function_ep0_complete;
	uvc->control_req->context = uvc;

	/* Avoid letting this gadget enumerate until the userspace server is
	 * active.
	 */
	uvc_function_disconnect(uvc);

	/* Initialise video. */
	ret = uvc_video_init(&uvc->video, bulkmode, bulksize,
			headersize, maxpayload, usbreqnums);
	if (ret < 0)
		goto error;

	/* Register a V4L2 device. */
	ret = uvc_register_video(uvc);
	if (ret < 0) {
		printk(KERN_INFO "Unable to register video device\n");
		goto error;
	}

	return 0;

error:
	if (uvc->vdev)
		video_device_release(uvc->vdev);

	if (uvc->control_ep)
		uvc->control_ep->driver_data = NULL;
	if (uvc->video.ep)
		uvc->video.ep->driver_data = NULL;

	if (uvc->control_req) {
		usb_ep_free_request(cdev->gadget->ep0, uvc->control_req);
		kfree(uvc->control_buf);
	}

	usb_free_all_descriptors(f);
	return ret;
}

/* --------------------------------------------------------------------------
 * USB gadget function
 */

/**
 * uvc_bind_config - add a UVC function to a configuration
 * @c: the configuration to support the UVC instance
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @uvc_setup(). Caller is also responsible for
 * calling @uvc_cleanup() before module unload.
 */
int
uvc_bind_config(struct usb_configuration *c,
		const struct uvc_descriptor_header * const *fs_control,
		const struct uvc_descriptor_header * const *ss_control,
		const struct uvc_descriptor_header * const *fs_streaming,
		const struct uvc_descriptor_header * const *hs_streaming,
		const struct uvc_descriptor_header * const *ss_streaming)
{
	struct uvc_device *uvc;
	int ret = 0;

	/* TODO Check if the USB device controller supports the required
	 * features.
	 */
	if (!gadget_is_dualspeed(c->cdev->gadget))
		return -EINVAL;

	uvc = kzalloc(sizeof(*uvc), GFP_KERNEL);
	if (uvc == NULL)
		return -ENOMEM;

	uvc->state = UVC_STATE_DISCONNECTED;

	/* Validate the descriptors. */
	if (fs_control == NULL || fs_control[0] == NULL ||
	    fs_control[0]->bDescriptorSubType != UVC_VC_HEADER)
		goto error;

	if (ss_control == NULL || ss_control[0] == NULL ||
	    ss_control[0]->bDescriptorSubType != UVC_VC_HEADER)
		goto error;

	if (fs_streaming == NULL || fs_streaming[0] == NULL ||
	    fs_streaming[0]->bDescriptorSubType != UVC_VS_INPUT_HEADER)
		goto error;

	if (hs_streaming == NULL || hs_streaming[0] == NULL ||
	    hs_streaming[0]->bDescriptorSubType != UVC_VS_INPUT_HEADER)
		goto error;

	if (ss_streaming == NULL || ss_streaming[0] == NULL ||
	    ss_streaming[0]->bDescriptorSubType != UVC_VS_INPUT_HEADER)
		goto error;

	uvc->desc.fs_control = fs_control;
	uvc->desc.ss_control = ss_control;
	uvc->desc.fs_streaming = fs_streaming;
	uvc->desc.hs_streaming = hs_streaming;
	uvc->desc.ss_streaming = ss_streaming;

	/* Update the strings. */
	if (_webcam_config) {
		if (strlen(_webcam_config->video_iad_string) > 0)
			uvc_en_us_strings[UVC_STRING_IAD_IDX].s =
				_webcam_config->video_iad_string;
		if (strlen(_webcam_config->video_control_string) > 0)
			uvc_en_us_strings[UVC_STRING_CONTROL_IDX].s =
				_webcam_config->video_control_string;
		if (strlen(_webcam_config->video_stream_string) > 0)
			uvc_en_us_strings[UVC_STRING_STREAMING_IDX].s =
				_webcam_config->video_stream_string;
	}
	/* String descriptors are global, we only need to allocate string IDs
	 * for the first UVC function. UVC functions beyond the first (if any)
	 * will reuse the same IDs.
	 */

	if (uvc_en_us_strings[UVC_STRING_IAD_IDX].id == 0) {
		ret = usb_string_ids_tab(c->cdev, uvc_en_us_strings);
		if (ret)
			goto error;
		uvc_iad.iFunction =
			uvc_en_us_strings[UVC_STRING_IAD_IDX].id;
		uvc_control_intf.iInterface =
			uvc_en_us_strings[UVC_STRING_CONTROL_IDX].id;
		ret = uvc_en_us_strings[UVC_STRING_STREAMING_IDX].id;
		uvc_streaming_intf_alt0.iInterface = ret;
		uvc_streaming_intf_alt1.iInterface = ret;
	}

	/* Register the function. */
	uvc->func.name = "uvc";
	uvc->func.strings = uvc_function_strings;
	uvc->func.bind = uvc_function_bind;
	uvc->func.unbind = uvc_function_unbind;
	uvc->func.get_alt = uvc_function_get_alt;
	uvc->func.set_alt = uvc_function_set_alt;
	uvc->func.resume = uvc_function_resume;
	uvc->func.suspend = uvc_function_suspend;
	uvc->func.disable = uvc_function_disable;
	uvc->func.setup = uvc_function_setup;

	ret = usb_add_function(c, &uvc->func);
	if (ret)
		kfree(uvc);

	return ret;

error:
	kfree(uvc);
	return ret;
}

static int
webcam_config_bind(struct usb_configuration *c,
		struct webcam_config *config)
{
	/* Set the global _webcam_config. */
	_webcam_config = config;

	return uvc_bind_config(c, uvc_fs_control_cls, uvc_ss_control_cls,
		uvc_fs_streaming_cls, uvc_hs_streaming_cls,
		uvc_ss_streaming_cls);
}

static void
webcam_config_unbind(struct usb_configuration *c)
{
	/* TODO */
	_webcam_config = NULL;
}

module_param_named(trace, uvc_gadget_trace_param, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(trace, "Trace level bitmask");
