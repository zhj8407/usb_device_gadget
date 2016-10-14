/*
 * UVC gadget test application
 *
 * Copyright (C) 2010 Ideas on board SPRL <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 */

#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/wait.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

#include <gio/gio.h>
#include <glib.h>
#include <gst/gst.h>

#include "uvc.h"

#include "log_str.h"
#include "uvc-gadget-func.h"

#define clamp(val, min, max) ({                 \
        typeof(val) __val = (val);              \
        typeof(min) __min = (min);              \
        typeof(max) __max = (max);              \
        (void) (&__val == &__min);              \
        (void) (&__val == &__max);              \
        __val = __val < __min ? __min: __val;   \
        __val > __max ? __max: __val; })

#define UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID     (1)
#define UVC_PROCESSING_UNIT_CONTROL_UNIT_ID     (2)

#define WEBCAM_VIDEO_CONTROL_DEVICE_SYS_PATH    "/sys/class/plcm_usb/plcm0/f_webcam/webcam_v4l2_ctrl_device"
#define WEBCAM_VIDEO_STREAM_DEVICE_SYS_PATH    "/sys/class/plcm_usb/plcm0/f_webcam/webcam_v4l2_strm_device"
#define WEBCAM_MAXPACKET_SYS_PATH   "/sys/class/plcm_usb/plcm0/f_webcam/webcam_maxpacket"
#define WEBCAM_HEADERSIZE_SYS_PATH  "/sys/class/plcm_usb/plcm0/f_webcam/webcam_headersize"
#define WEBCAM_BULKMODE_SYS_PATH    "/sys/class/plcm_usb/plcm0/f_webcam/webcam_bulkmode"
#define WEBCAM_MAXPAYLOAD_SYS_PATH  "/sys/class/plcm_usb/plcm0/f_webcam/webcam_maxpayload"


#define VISAGE_LED_NOTIFICATION 1

#define MAX_GST_LINKED_ELEMENTS_COUNT 6

/*struct uvc_device {
    const char *v4l2_src_device;
    char v4l2_sink_device[32];
    int fd;
    unsigned char stream_on;

    struct uvc_streaming_control probe;
    struct uvc_streaming_control commit;

    int control;
    int unit;

    unsigned int fcc;
    unsigned int width;
    unsigned int height;

    unsigned int nbufs;
    unsigned int bufsize;

    unsigned int bulk;

    int v4l2_ctrl_devnum;
    int v4l2_strm_devnum;
    unsigned int maxpacketsize;
    unsigned int headersize;
    unsigned int maxpayloadsize;

    //Gstreamer related.
    GstElement *pipeline;
    GstElement *video_source;
    GstElement *video_scaler;
    GstElement *video_converter;
    GstElement *video_encoder;
    GstElement *video_sink;

    //GLib timer
    guint timeout_watch_id;

    //GDBusConnection
    GDBusConnection *conn;

    //DEBUG. Test Video src
    unsigned char test_video_flag;
};*/
struct gst_device {
    //Gstreamer related.
    GstElement *pipeline;
    GstElement *video_source;
    GstElement *video_scaler;
    GstElement *video_converter;
    GstElement *video_encoder;
    GstElement *video_sink;

    //GLib timer
    guint timeout_watch_id;

    //DEBUG. Test Video src
    unsigned char test_video_flag;
};
struct gst_device *gst_dev;
struct uvc_device *global_uvc;
//GDBusConnection
GDBusConnection *dbus_conn;



static int start_gst_video_stream(struct uvc_device *dev);
static void stop_gst_video_stream(struct uvc_device *dev);

static void uvc_set_timeout(struct uvc_device *dev, guint timeout_in_ms);

static void
send_get_property_signal(struct uvc_device *dev, uint8_t req, uint8_t cs,
                         uint8_t unit_id, uint16_t length);

static void
send_set_property_signal(struct uvc_device *dev, uint8_t req, uint8_t cs,
                         uint8_t unit_id, uint16_t length, uint8_t *data);

static void
send_uvc_stream_status_signal(struct uvc_device *dev);

static int
uvc_read_value_from_file(const char* filename,
                         const char* format,
                         int *value)
{
    int fd = 0;
    char buf[8];
    int len;

    fd = open(filename, O_RDONLY);

    if (fd < 0) {
        fprintf(stderr, "Can not open the file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        return -1;
    }

    len = read(fd, buf, sizeof(buf));

    if (len <= 0) {
        fprintf(stderr, "Can not read data from file: %s, error: %d(%s)\n",
                filename, errno, strerror(errno));
        close(fd);
        return -1;
    }

    len = sscanf(buf, format, value);

    if (len <= 0) {
        fprintf(stderr, "Can not parse the value from %s\n", filename);
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}


static int
uvc_video_init(struct uvc_device *dev)
{
    uvc_read_value_from_file(WEBCAM_VIDEO_CONTROL_DEVICE_SYS_PATH,
                             "%d\n",
                             &dev->v4l2_ctrl_devnum);

    uvc_read_value_from_file(WEBCAM_VIDEO_STREAM_DEVICE_SYS_PATH,
                             "%d\n",
                             &dev->v4l2_strm_devnum);

    uvc_read_value_from_file(WEBCAM_MAXPACKET_SYS_PATH,
                             "%d\n",
                             (int *)&dev->maxpacketsize);

    uvc_read_value_from_file(WEBCAM_HEADERSIZE_SYS_PATH,
                             "%d\n",
                             (int *)&dev->headersize);

    uvc_read_value_from_file(WEBCAM_BULKMODE_SYS_PATH,
                             "%d\n",
                             (int *)&dev->bulk);

    uvc_read_value_from_file(WEBCAM_MAXPAYLOAD_SYS_PATH,
                             "%x\n",
                             (int *)&dev->maxpayloadsize);

    return 0;
}

static struct uvc_device *
uvc_open(const char *devname)
{
    struct uvc_device *dev;
    char v4lctrlname[64];
    int fd = 0;

    dev = malloc(sizeof * dev);

    if (dev == NULL) {
        return NULL;
    }

    memset(dev, 0, sizeof * dev);
    dev->v4l2_ctrl_devnum = -1;
    dev->v4l2_strm_devnum = -1;
    dev->maxpacketsize = 1024;
    dev->headersize = 2;
    dev->bulk = 0;
    dev->v4l2_src_device = devname;

    uvc_video_init(dev);

    if (dev->v4l2_ctrl_devnum == -1 ||
            dev->v4l2_strm_devnum == -1) {
        fprintf(stderr, "Failed to read the video dev numbers\n");
        free(dev);
        return NULL;
    }

    snprintf(dev->v4l2_sink_device, sizeof(dev->v4l2_sink_device),
             "/dev/video%d", dev->v4l2_strm_devnum);

    snprintf(v4lctrlname, sizeof(v4lctrlname),
             "/dev/video%d", dev->v4l2_ctrl_devnum);

    printf("We are trying to open the dev: %s\n", v4lctrlname);

    fd = open(v4lctrlname, O_RDWR | O_NONBLOCK);

    if (fd == -1) {
        printf("v4l2 open failed: %s (%d)\n", strerror(errno), errno);
        free(dev);
        return NULL;
    }

    printf("open succeeded, file descriptor = %d\n", fd);

    printf("The config values are as below\n");
    printf("\t\tv4l2_ctrl_devnum: %d\n", dev->v4l2_ctrl_devnum);
    printf("\t\tmaxpacketsize(iso): %d\n", dev->maxpacketsize);
    printf("\t\theadersize: %d\n", dev->headersize);
    printf("\t\tbulkmode: %d\n", dev->bulk);
    printf("\t\tmaxpayloadsize: 0x%x\n", dev->maxpayloadsize);
    dev->fd = fd;
    return dev;
}

static void
uvc_close(struct uvc_device *dev)
{
    close(dev->fd);
    free(dev);
}

/* ---------------------------------------------------------------------------
 * video stream
 */

// #define VIDEO_TEST_SRC             1

#define CAM_MAX_WIDTH 1920
#define CAM_MAX_HEIGHT 1080

#define DUMMY_VIDEO_DEF_WIDTH 1280
#define DUMMY_VIDEO_DEF_HEIGHT 720
#define DUMMY_VIDEO_DEF_FRAMERATE 30

#define CAM_DEF_WIDTH 1920
#define CAM_DEF_HEIGHT 1080
#define CAM_DEF_FRAMERATE 60


unsigned int camera_format = V4L2_PIX_FMT_NV12;
#ifdef VIDEO_TEST_SRC
unsigned int camera_width = CAM_DEF_WIDTH;
unsigned int camera_height = CAM_DEF_HEIGHT;
unsigned int camera_framerate = CAM_DEF_FRAMERATE;
#else
unsigned int camera_width = DUMMY_VIDEO_DEF_WIDTH;
unsigned int camera_height = DUMMY_VIDEO_DEF_HEIGHT;
unsigned int camera_framerate = DUMMY_VIDEO_DEF_FRAMERATE;
#endif

unsigned int output_framerate = CAM_DEF_FRAMERATE;
unsigned int encoder_quality = 85;

#ifdef USE_EXTERNAL_GST_LAUNCH

static const char FST_FMT_YUY2[] = "video/x-raw, format='(string)'YUY2";
static const char FST_FMT_NV12[] = "video/x-raw, format='(string)'NV12";
static const char FST_FMT_I420[] = "video/x-raw, format='(string)'I420";
static const char FST_FMT_MJPG[] = "image/jpeg";

const char * v4l2format2gstformat(unsigned int v4l2format)
{
    switch (v4l2format) {
        case V4L2_PIX_FMT_YUYV:
            return FST_FMT_YUY2;

        case V4L2_PIX_FMT_MJPEG:
            return FST_FMT_MJPG;

        case V4L2_PIX_FMT_YUV420:
            return FST_FMT_I420;

        case V4L2_PIX_FMT_NV12:
            return FST_FMT_NV12;

        default:
            return "[Unknown Format]";
    }
}
#define GST_CMD_TOTAL_LEN 4096
#define GST_CMD_LEN 512

static char gst_camera_cmd[GST_CMD_LEN];
static char gst_convertor_cmd[GST_CMD_LEN];
static char gst_scaler_cmd[GST_CMD_LEN];
static char gst_encoder_cmd[GST_CMD_LEN];
static char gst_output_cmd[GST_CMD_LEN];

static const char *get_gst_camera_cmd(const char *v4l2_camera_dev,
                                      unsigned int format, unsigned int width,
                                      unsigned int height, unsigned int framerate)
{
    memset(gst_camera_cmd, 0, sizeof(gst_camera_cmd));
#ifdef VIDEO_TEST_SRC
    (void)v4l2_camera_dev;
    snprintf(gst_camera_cmd, GST_CMD_LEN,
             "videotestsrc pattern=smpte ! %s, width='(int)'%u, height='(int)'%u, framerate='(fraction)'%u/1",
             v4l2format2gstformat(format), width, height, framerate);
#else
    snprintf(gst_camera_cmd, GST_CMD_LEN,
             "v4l2src device=%s ! %s, width='(int)'%u, height='(int)'%u, framerate='(fraction)'%u/1",
             v4l2_camera_dev, v4l2format2gstformat(format), width, height, framerate);
#endif
    return gst_camera_cmd;
}

static const char *get_gst_convertor_cmd(unsigned int format)
{
    memset(gst_convertor_cmd, 0, sizeof(gst_convertor_cmd));

    if (format == camera_format) {
        return "";
    }

    if (format == V4L2_PIX_FMT_MJPEG) {
        format = V4L2_PIX_FMT_YUV420;
    }

    snprintf(gst_convertor_cmd, GST_CMD_LEN,
             " ! videoconvert ! %s",
             v4l2format2gstformat(format));

    return gst_convertor_cmd;
}

static const char *get_gst_scaler_cmd(unsigned int format,
                                      unsigned int width, unsigned int height)
{
    memset(gst_scaler_cmd, 0, sizeof(gst_scaler_cmd));

    if (width == camera_width && height == camera_height) {
        return "";
    }

    snprintf(gst_scaler_cmd, GST_CMD_LEN,
             " ! videoscale ! %s, width='(int)'%u, height='(int)'%u",
             v4l2format2gstformat(format), width, height);

    return gst_scaler_cmd;
}

static const char *get_gst_encoder_cmd(unsigned int format, unsigned int quality)
{
    memset(gst_encoder_cmd, 0, sizeof(gst_encoder_cmd));

    if (format != V4L2_PIX_FMT_MJPEG) {
        return "";
    }

    snprintf(gst_encoder_cmd, GST_CMD_LEN,
             " ! nvjpegenc quality=%u", quality);

    return gst_encoder_cmd;
}

static const char *get_gst_v4l2output_cmd(unsigned int v4l2_output_devnum)
{
    memset(gst_output_cmd, 0, sizeof(gst_output_cmd));
    snprintf(gst_output_cmd, GST_CMD_LEN,
             " ! v4l2sink device=/dev/video%u",
             v4l2_output_devnum);
    return gst_output_cmd;
}

static unsigned int get_gst_running_cmd(char *gst_cmd,
                                        const char *v4l2_src_dev,
                                        unsigned int format,
                                        unsigned int width,
                                        unsigned int height,
                                        unsigned int v4l2_output_devnum)
{
    return snprintf(gst_cmd, GST_CMD_TOTAL_LEN, "sleep 2 && gst-launch-1.0 -e -v --gst-debug=3 %s%s%s%s%s",
                    get_gst_camera_cmd(v4l2_src_dev, camera_format, camera_width, camera_height, camera_framerate),
                    get_gst_scaler_cmd(camera_format, width, height),
                    get_gst_convertor_cmd(format),
                    get_gst_encoder_cmd(format, encoder_quality),
                    get_gst_v4l2output_cmd(v4l2_output_devnum));
}
#endif

void start_uvc_stream(struct uvc_device *dev)
{
#ifdef USE_EXTERNAL_GST_LAUNCH
    char gst_running_cmd[GST_CMD_TOTAL_LEN];
    get_gst_running_cmd(gst_running_cmd,
                        dev->v4l2_src_device,
                        dev->fcc,
                        dev->width,
                        dev->height,
                        dev->v4l2_strm_devnum);

    strncat(gst_running_cmd, " > /dev/null 2>&1 &", GST_CMD_TOTAL_LEN);
    printf("gstreamer running cmd: %s\n", gst_running_cmd);

    system(gst_running_cmd);
#else
    start_gst_video_stream(dev);
#endif
    dev->stream_on = 1;
}

void stop_uvc_stream(struct uvc_device *dev)
{
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(dev->fd, VIDIOC_STREAMOFF, &type);
#ifdef USE_EXTERNAL_GST_LAUNCH

    if (dev->streaming_pid > 0)
        system("sudo killall -9 gst-launch-1.0");

#else
    stop_gst_video_stream(dev);
#endif

    dev->stream_on = 0;
}

/* ---------------------------------------------------------------------------
 * Request processing
 */
/*
struct uvc_frame_info {
    unsigned int width;
    unsigned int height;
    unsigned int intervals[8];
};

struct uvc_format_info {
    unsigned int fcc;
    const struct uvc_frame_info *frames;
};

static const struct uvc_frame_info uvc_frames_yuyv[] = {
    {  640, 360, { 666666, 10000000, 50000000, 0 }, },
    { 1280, 720, { 50000000, 0 }, },
    { 0, 0, { 0, }, },
};

static const struct uvc_frame_info uvc_frames_i420[] = {
    {  640, 360, { 333333, 666666, 10000000, 50000000, 0 }, },
    { 1280, 720, { 10000000, 50000000, 0 }, },
    { 0, 0, { 0, }, },
};

static const struct uvc_frame_info uvc_frames_mjpeg[] = {
    {  640, 360, { 333333, 666666, 10000000, 0 }, },
    { 1280, 720, { 333333, 666666, 0 }, },
    { 1920, 1080, { 333333, 666666, 0 }, },
    { 0, 0, { 0, }, },
};

static const struct uvc_format_info uvc_formats[] = {
    { V4L2_PIX_FMT_YUYV, uvc_frames_yuyv },
    { V4L2_PIX_FMT_YUV420, uvc_frames_i420 },
    { V4L2_PIX_FMT_MJPEG, uvc_frames_mjpeg },
};

static void
uvc_fill_streaming_control(struct uvc_device *dev,
                           struct uvc_streaming_control *ctrl,
                           int iframe, int iformat)
{
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    unsigned int nframes;

    if (iformat < 0)
        iformat = ARRAY_SIZE(uvc_formats) + iformat;

    if (iformat < 0 || iformat >= (int)ARRAY_SIZE(uvc_formats))
        return;

    format = &uvc_formats[iformat];
    nframes = 0;

    while (format->frames[nframes].width != 0)
        ++nframes;

    if (iframe < 0)
        iframe = nframes + iframe;

    if (iframe < 0 || iframe >= (int)nframes)
        return;

    frame = &format->frames[iframe];
    printf("uvc_fill_streaming_control:fcc=0x%x, Format=%s, iformat=0x%x, iframe=0x%x\n", format->fcc, getV4L2FormatStr(format->fcc), iformat, iframe);
    memset(ctrl, 0, sizeof * ctrl);
    ctrl->bmHint = 1;
    ctrl->bFormatIndex = iformat + 1;
    ctrl->bFrameIndex = iframe + 1;
    ctrl->dwFrameInterval = frame->intervals[0];

    switch (format->fcc) {
        case V4L2_PIX_FMT_YUYV:
            ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 2;
            break;

        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_NV12:
            ctrl->dwMaxVideoFrameSize = frame->width * frame->height *  3 / 2;
            break;

        case V4L2_PIX_FMT_MJPEG:
            ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 3 / 2;
            break;
    }

    if (!dev->bulk)
        ctrl->dwMaxPayloadTransferSize = dev->maxpacketsize;   // TODO this should be filled by the driver.
    else if (!dev->maxpayloadsize)
        ctrl->dwMaxPayloadTransferSize = ctrl->dwMaxVideoFrameSize + dev->headersize;

    ctrl->bmFramingInfo = 3;
    ctrl->bPreferedVersion = 1;
    ctrl->bMaxVersion = 1;
}

static void
uvc_events_process_standard(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                            struct uvc_request_data *resp)
{
    printf("standard request\n");
    (void)dev;
    (void)ctrl;
    resp->length = 0;
}

static void
uvc_events_process_control(struct uvc_device *dev, uint8_t req, uint8_t cs,
                           uint8_t unit_id, uint16_t length, struct uvc_request_data *resp)
{
    printf("control request (req %02x cs %02x)\n", req, cs);
    (void)dev;

    if (req != UVC_SET_CUR) {
        send_get_property_signal(dev, req, cs, unit_id, length);
    } else {
        dev->control = cs;
        dev->unit = unit_id;
    }

    // Do not send the reply here.
    // We have posted a singal. The client will do it.
    resp->length = -EL2HLT;
}

static void
uvc_events_process_streaming(struct uvc_device *dev, uint8_t req, uint8_t cs,
                             struct uvc_request_data *resp)
{
    struct uvc_streaming_control *ctrl;
    printf("streaming request (req 0x%02x cs 0x%02x, ) %s\n", req, cs,  getVSIntfControlSelectorStr(cs));

    if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL)
        return;

    ctrl = (struct uvc_streaming_control *)&resp->data;
    resp->length = sizeof * ctrl;
    printf("UVC OPS  %s resp->length=%u\n", getUVCOpStr(req), resp->length);

    switch (req) {
        case UVC_SET_CUR:
            dev->control = cs;
            dev->unit = 0;
            resp->length = 34;
            break;

        case UVC_GET_CUR:
            if (cs == UVC_VS_PROBE_CONTROL)
                memcpy(ctrl, &dev->probe, sizeof * ctrl);
            else
                memcpy(ctrl, &dev->commit, sizeof * ctrl);

            break;

        case UVC_GET_MIN:
        case UVC_GET_MAX:
        case UVC_GET_DEF:
            uvc_fill_streaming_control(dev, ctrl, req == UVC_GET_MAX ? -1 : 0,
                                       req == UVC_GET_MAX ? -1 : 0);
            break;

        case UVC_GET_RES:
            memset(ctrl, 0, sizeof * ctrl);
            break;

        case UVC_GET_LEN:
            resp->data[0] = 0x00;
            resp->data[1] = 0x22;
            resp->length = 2;
            break;

        case UVC_GET_INFO:
            resp->data[0] = 0x03;
            resp->length = 1;
            break;
    }
}

static void
uvc_events_process_class(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                         struct uvc_request_data *resp)
{
    if ((ctrl->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE)
        return;

    if ((ctrl->wIndex >> 8) & 0xff) {
        //has unit id. Control event
        uvc_events_process_control(dev, ctrl->bRequest, ctrl->wValue >> 8, ctrl->wIndex >> 8, ctrl->wLength, resp);
    } else {
        uvc_events_process_streaming(dev, ctrl->bRequest, ctrl->wValue >> 8, resp);
    }
}

static void
uvc_events_process_setup(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                         struct uvc_request_data *resp)
{
    dev->control = 0;
    printf("bRequestType %02x bRequest %02x wValue %04x wIndex %04x "
           "wLength %04x\n", ctrl->bRequestType, ctrl->bRequest,
           ctrl->wValue, ctrl->wIndex, ctrl->wLength);

    switch (ctrl->bRequestType & USB_TYPE_MASK) {
        case USB_TYPE_STANDARD:
            uvc_events_process_standard(dev, ctrl, resp);
            break;

        case USB_TYPE_CLASS:
            uvc_events_process_class(dev, ctrl, resp);
            break;

        default:
            printf("Unhandled bRequestType %02x bRequest %02x wValue %04x wIndex %04x "
                   "wLength %04x\n", ctrl->bRequestType, ctrl->bRequest,
                   ctrl->wValue, ctrl->wIndex, ctrl->wLength);
            break;
    }
}

static void
uvc_events_process_data(struct uvc_device *dev, struct uvc_request_data *data)
{
    struct uvc_streaming_control *target;
    struct uvc_streaming_control *ctrl;
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    const unsigned int *interval;
    unsigned int iformat, iframe;
    unsigned int nframes;

    switch (((dev->unit) << 8) | dev->control) {
        case UVC_VS_PROBE_CONTROL:
            printf("setting probe control, length = %d\n", data->length);
            target = &dev->probe;
            break;

        case UVC_VS_COMMIT_CONTROL:
            printf("setting commit control, length = %d\n", data->length);
            target = &dev->commit;
            break;

        default:
            send_set_property_signal(dev, UVC_SET_CUR, dev->control, dev->unit, data->length, data->data);
            return;
    }

    ctrl = (struct uvc_streaming_control *)&data->data;
    iformat = clamp((unsigned int)ctrl->bFormatIndex, 1U,
                    (unsigned int)ARRAY_SIZE(uvc_formats));
    format = &uvc_formats[iformat - 1];
    nframes = 0;

    while (format->frames[nframes].width != 0)
        ++nframes;

    iframe = clamp((unsigned int)ctrl->bFrameIndex, 1U, nframes);
    frame = &format->frames[iframe - 1];
    interval = frame->intervals;

    while (interval[0] < ctrl->dwFrameInterval && interval[1])
        ++interval;

    target->bFormatIndex = iformat;
    target->bFrameIndex = iframe;

    switch (format->fcc) {
        case V4L2_PIX_FMT_YUYV:
            target->dwMaxVideoFrameSize = frame->width * frame->height * 2;
            break;

        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_NV12:
            target->dwMaxVideoFrameSize = frame->width * frame->height *  3 / 2;
            break;

        case V4L2_PIX_FMT_MJPEG:
            target->dwMaxVideoFrameSize = frame->width * frame->height;
            target->wCompQuality = encoder_quality;
            break;
    }

    target->dwFrameInterval = *interval;

    if (dev->control == UVC_VS_COMMIT_CONTROL) {
        dev->fcc = format->fcc;
        dev->width = frame->width;
        dev->height = frame->height;

        if (dev->bulk) {
            stop_uvc_stream(dev);
            start_uvc_stream(dev);
            //Notify the app with the streaming status
            send_uvc_stream_status_signal(dev);

            uvc_set_timeout(dev, 5000);
        }
    } else if (dev->control == UVC_VS_PROBE_CONTROL) {
        if (dev->bulk) {
            //Cacel the timer
            uvc_set_timeout(dev, 0);
            stop_uvc_stream(dev);
        }
    }
}

//#define TIME_DEBUG

#ifdef TIME_DEBUG
static void get_current_timestamp(long *tv_sec, long *tv_usec)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    if (tv_sec)
        *tv_sec = tv.tv_sec;

    if (tv_usec)
        *tv_usec = tv.tv_usec;
}
#endif

static void handle_frame_done_event(struct uvc_device *dev)
{
#ifdef TIME_DEBUG
    static unsigned long prev_time = 0;
    long tv_sec;
    long tv_usec;

    unsigned long delta_time;    //ms

    get_current_timestamp(&tv_sec, &tv_usec);

    unsigned long current_time = tv_sec * 1000 + tv_usec / 1000;

    if (prev_time) {
        delta_time = current_time - prev_time;
        printf("Delta time is %lu\n", delta_time);
    }

    prev_time = current_time;
#endif

    (void)dev;

    if (dev->bulk) {
        // Reset the alarm.
        uvc_set_timeout(dev, 1000);
    }
}


static void
uvc_events_process(struct uvc_device *dev)
{
    struct v4l2_event v4l2_event;
    struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
    struct uvc_request_data resp;
    struct usb_ctrlrequest *ctrl_req = &uvc_event->req;
    int ret;
    ret = ioctl(dev->fd, VIDIOC_DQEVENT, &v4l2_event);

    if (ret < 0) {
        printf("VIDIOC_DQEVENT failed: %s (%d)\n", strerror(errno),
               errno);
        return;
    }

    memset(&resp, 0, sizeof resp);
    resp.length = -EL2HLT;
    //intf=(ctrl_req->wIndex&0xff);
    //printf("[BEGIN]uvc_events_process: Receive V4L2 evnet [0x%x] %s\n", v4l2_event.type, getUVCEventStr(v4l2_event.type));

    switch (v4l2_event.type) {
        case UVC_EVENT_CONNECT:
        case UVC_EVENT_DISCONNECT:
            break;

        case UVC_EVENT_SETUP:
            printf("bRequestType %02x bRequest %02x wValue %04x wIndex %04x wLength %04x [intf=]\n",
                   ctrl_req->bRequestType, ctrl_req->bRequest,
                   ctrl_req->wValue, ctrl_req->wIndex, ctrl_req->wLength);
            printHexData((char *)ctrl_req, sizeof(*ctrl_req), getUVCEventStr(v4l2_event.type));
            uvc_events_process_setup(dev, ctrl_req, &resp);
            break;

        case UVC_EVENT_DATA:
            printHexData((char *)&uvc_event->data, sizeof(uvc_event->data), getUVCEventStr(v4l2_event.type));
            uvc_events_process_data(dev, &uvc_event->data);
            break;

        case UVC_EVENT_STREAMON:
            // Start uvc video stream
            start_uvc_stream(dev);
            //Notify the app with the streaming status
            send_uvc_stream_status_signal(dev);

            break;

        case UVC_EVENT_STREAMOFF:

            // Cacel the alarm.
            if (dev->bulk)
                uvc_set_timeout(dev, 0);

            // Stop the uvc video stream
            stop_uvc_stream(dev);
            //Notify the app with the streaming status
            send_uvc_stream_status_signal(dev);

            break;

        case UVC_EVENT_FRAMEDONE:
            handle_frame_done_event(dev);
            break;
    }

    if (resp.length < 0) {
        return;
    }

    //Ignore the SET_CUR event. Because we have triggle the transfer
    //in the kernel side.
    if ((ctrl_req->bRequestType & USB_DIR_IN) != 0 ||
            (ctrl_req->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE ||
            ctrl_req->bRequest != UVC_SET_CUR) {

        ioctl(dev->fd, UVCIOC_SEND_RESPONSE, &resp);

        if (ret < 0) {
            printf("UVCIOC_S_EVENT failed: %s (%d)\n", strerror(errno),
                   errno);
            printHexData((char *)ctrl_req, sizeof(*ctrl_req), getUVCEventStr(v4l2_event.type));
            return;
        }
    }

    printHexData((char *)(&resp), sizeof(resp), "REPLY");
    printf("[END]uvc_events_process: DONE with V4L2 event [0x%x] %s\n", v4l2_event.type, getUVCEventStr(v4l2_event.type));
}

static void
uvc_events_init(struct uvc_device *dev)
{
    struct v4l2_event_subscription sub;
    uvc_fill_streaming_control(dev, &dev->probe, 0, 0);
    uvc_fill_streaming_control(dev, &dev->commit, 0, 0);

    if (dev->bulk) {
        // FIXME Crude hack, must be negotiated with the driver.
        dev->probe.dwMaxPayloadTransferSize = dev->maxpayloadsize;
        dev->commit.dwMaxPayloadTransferSize = dev->maxpayloadsize;
    }

    memset(&sub, 0, sizeof sub);
    sub.type = UVC_EVENT_SETUP;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_DATA;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMON;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMOFF;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_FRAMEDONE;
    ioctl(dev->fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
}
*/

static GMainLoop *_global_loop = NULL;

void sig_handle(int sig)
{
    printf("Received signal: %d(%s)\n", sig, strsignal(sig));

    // Alarm timeout. Stop the video
    if (_global_loop)
        g_main_loop_quit(_global_loop);
}

static gboolean
io_watch(GIOChannel *channel, GIOCondition condition, gpointer data)
{
    struct uvc_device *dev = (struct uvc_device *)data;

    if (!(condition & (G_IO_PRI))) {
        g_print("Wrong events\n");
        return TRUE;
    }

    (void)channel;

    uvc_events_process(dev);

    return TRUE;
}

/* ---------------------------------------------------------------------------
 * GStreamer related.
 */

static gboolean
tiemout_callback(gpointer arg)
{
    struct uvc_device *dev = (struct uvc_device *)arg;

    g_print("Timeout. Stop the stream\n");

    stop_uvc_stream(dev);

    //Notify the app with the streaming status
    send_uvc_stream_status_signal(dev);

    /* FALSE means the timer will be destroyed. */
    return FALSE;
}

static void uvc_set_timeout(struct uvc_device *dev, guint timeout_in_ms)
{
    if (gst_dev->timeout_watch_id) {
        g_source_remove(gst_dev->timeout_watch_id);
        gst_dev->timeout_watch_id = 0;
    }

    if (timeout_in_ms) {
        gst_dev->timeout_watch_id =
            g_timeout_add(timeout_in_ms, tiemout_callback, dev);
    }
}

static gboolean
bus_call(GstBus *bus,
         GstMessage *msg,
         gpointer data)
{
    struct uvc_device *dev = (struct uvc_device *)data;

    (void)bus;

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("End of stream\n");
            break;

        case GST_MESSAGE_ERROR: {
            gchar *debug;
            GError *error;

            gst_message_parse_error(msg, &error, &debug);
            g_free(debug);

            g_print("Error: %s\n", error->message);
            g_error_free(error);

            //Stop the stream
            stop_gst_video_stream(dev);

            break;
        }

        case GST_MESSAGE_STATE_CHANGED: {
#if 0
            GstState old_state, new_state;

            gst_message_parse_state_changed(msg, &old_state, &new_state, NULL);
            g_print("Element %s state changed from %s to %s.\n",
                    GST_OBJECT_NAME(msg->src),
                    gst_element_state_get_name(old_state),
                    gst_element_state_get_name(new_state));
#endif
            break;
        }

        default:
            break;
    }

    return TRUE;
}

static gboolean
link_elements_with_video_formats(
    GstElement *element1,
    GstElement *element2,
    int width,
    int height,
    int framerate,
    unsigned int fcc)
{
    gboolean link_ok;
    GstCaps *caps;
    GstStructure *caps_structure;

    if (fcc != V4L2_PIX_FMT_MJPEG)
        caps_structure = gst_structure_new_empty("video/x-raw");
    else
        caps_structure = gst_structure_new_empty("image/jpeg");

    switch (fcc) {
        case V4L2_PIX_FMT_YUYV:
            gst_structure_set(caps_structure, "format", G_TYPE_STRING, "YUY2", NULL);
            break;

        case V4L2_PIX_FMT_YUV420:
            gst_structure_set(caps_structure, "format", G_TYPE_STRING, "I420", NULL);
            break;

        case V4L2_PIX_FMT_NV12:
            gst_structure_set(caps_structure, "format", G_TYPE_STRING, "NV12", NULL);
            break;

        case V4L2_PIX_FMT_MJPEG:
            break;

        default:
            g_warning("[link_elements_with_video_formats]Unknown video format! %d\n", fcc);
            break;
    }

    if (width != -1)
        gst_structure_set(caps_structure, "width", G_TYPE_INT, width, NULL);

    if (height != -1)
        gst_structure_set(caps_structure, "height", G_TYPE_INT, height, NULL);

    if (framerate != -1)
        gst_structure_set(caps_structure, "framerate", GST_TYPE_FRACTION, framerate, 1, NULL);

    caps = gst_caps_new_full(caps_structure, NULL);

    link_ok = gst_element_link_filtered(element1, element2, caps);
    gst_caps_unref(caps);

    if (!link_ok)
        g_warning("Failed to link element: %s and element: %s\n",
                  GST_ELEMENT_NAME(element1),
                  GST_ELEMENT_NAME(element2));

    return link_ok;
}

static int start_gst_video_stream(struct uvc_device *dev)
{
    int retval = 0;

    if (!gst_dev->test_video_flag)
        gst_dev->video_source = gst_element_factory_make("v4l2src", "v4l2 source");
    else
        gst_dev->video_source = gst_element_factory_make("videotestsrc", "v4l2 source");

    gst_dev->video_scaler = gst_element_factory_make("videoscale", "video scaler");
    gst_dev->video_converter = gst_element_factory_make("videoconvert", "video converter");
    gst_dev->video_encoder = gst_element_factory_make("nvjpegenc", "jpeg encoder");
    gst_dev->video_sink = gst_element_factory_make("v4l2sink", "v4l2 sink");

    if (!gst_dev->video_source || !gst_dev->video_sink ||
            !gst_dev->video_scaler || !gst_dev->video_converter || !gst_dev->video_encoder) {
        g_printerr("One of the elements could not be created, Exiting. \n");
        return -1;
    }

    if (!gst_dev->test_video_flag) {
        /* We set the video capture device name to the source element. */
        g_object_set(G_OBJECT(gst_dev->video_source), "device", dev->v4l2_src_device, NULL);
        camera_width = CAM_DEF_WIDTH;
        camera_height = CAM_DEF_HEIGHT;
        camera_framerate = CAM_DEF_FRAMERATE;
    } else {
        camera_width = DUMMY_VIDEO_DEF_WIDTH;
        camera_height = DUMMY_VIDEO_DEF_HEIGHT;
        camera_framerate = DUMMY_VIDEO_DEF_FRAMERATE;
    }

    printf("[start_gst_video_stream]: try start gst video [%u][%s] %ux%u %ufps\n"
           , dev->fcc, getV4L2FormatStr(dev->fcc), dev->width, dev->height, camera_framerate);

    /* We set the video output device name to the sink element. */
    g_object_set(G_OBJECT(gst_dev->video_sink), "device", dev->v4l2_sink_device, NULL);

    /* Add the elements. */
    gst_bin_add_many(GST_BIN(gst_dev->pipeline), gst_dev->video_source, gst_dev->video_scaler,
                     gst_dev->video_converter, gst_dev->video_encoder, gst_dev->video_sink, NULL);

    switch (dev->fcc) {
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_YUV420: {
            if (!(dev->width == camera_width) || !(dev->height == camera_height)) {
                link_elements_with_video_formats(gst_dev->video_source, gst_dev->video_scaler,
                                                 camera_width, camera_height, camera_framerate, camera_format);
                link_elements_with_video_formats(gst_dev->video_scaler, gst_dev->video_converter,
                                                 dev->width, dev->height, -1, -1);
            } else {
                link_elements_with_video_formats(gst_dev->video_source, gst_dev->video_converter,
                                                 camera_width, camera_height, camera_framerate, camera_format);
            }

            link_elements_with_video_formats(gst_dev->video_converter, gst_dev->video_sink,
                                             -1, -1, -1, dev->fcc);
            break;
        }

        case V4L2_PIX_FMT_NV12: {
            if (!(dev->width == camera_width) || !(dev->height == camera_height)) {
                link_elements_with_video_formats(gst_dev->video_source, gst_dev->video_scaler,
                                                 camera_width, camera_height, camera_framerate, camera_format);
                link_elements_with_video_formats(gst_dev->video_scaler, gst_dev->video_sink,
                                                 dev->width, dev->height, -1, -1);
            } else {
                link_elements_with_video_formats(gst_dev->video_source, gst_dev->video_sink,
                                                 camera_width, camera_height, camera_framerate, camera_format);
            }

            break;
        }

        case V4L2_PIX_FMT_MJPEG: {
            if (!(dev->width == camera_width) || !(dev->height == camera_height)) {
                link_elements_with_video_formats(gst_dev->video_source, gst_dev->video_scaler,
                                                 camera_width, camera_height, camera_framerate, camera_format);
                link_elements_with_video_formats(gst_dev->video_scaler, gst_dev->video_converter,
                                                 dev->width, dev->height, -1, -1);
            } else {
                link_elements_with_video_formats(gst_dev->video_source, gst_dev->video_converter,
                                                 camera_width, camera_height, camera_framerate, camera_format);
            }

            link_elements_with_video_formats(gst_dev->video_converter, gst_dev->video_encoder,
                                             -1, -1, -1, V4L2_PIX_FMT_YUV420);
            link_elements_with_video_formats(gst_dev->video_encoder, gst_dev->video_sink,
                                             -1, -1, -1, dev->fcc);
            break;
        }

        default:
            g_warning("Unknown video format: %d\n", dev->fcc);
            return -1;
    }

    gst_element_set_state(gst_dev->pipeline, GST_STATE_PLAYING);

    return retval;
}

static void stop_gst_video_stream(struct uvc_device *dev)
{
    (void)dev;
    gst_element_set_state(gst_dev->pipeline, GST_STATE_NULL);

    gst_bin_remove_many(GST_BIN(gst_dev->pipeline), gst_dev->video_source, gst_dev->video_scaler,
                        gst_dev->video_converter, gst_dev->video_encoder, gst_dev->video_sink, NULL);
}

/* ---------------------------------------------------------------------------
 * GDBus related
 */
static GDBusNodeInfo *introspection_data = NULL;

#define PLCM_USB_VISAGE_UVC_OBJ_PATH        "/com/polycom/visage/uvc"
#define PLCM_USB_VISAGE_UVC_INTF_NAME       "com.polycom.visage.uvc"
#define PLCM_USB_VISAGE_UVC_GET_PROP_SIGNAL  "GetCameraProperty"
#define PLCM_USB_VISAGE_UVC_SET_PROP_SIGNAL  "SetCameraProperty"
#define PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL  "video_control"

/* Introspection data for the service we are exporting */
static const gchar introspection_xml[] =
    "<node>"
    "  <interface name='com.polycom.visage.uvc'>"
    "    <method name='SendResponse'>"
    "      <arg type='i' name='length' direction='in'/>"
    "      <arg type='ay' name='data' direction='in'/>"
    "      <arg type='i' name='retval' direction='out'/>"
    "    </method>"
    "    <method name='TestVideoSource'>"
    "      <arg type='b' name='test_video_flag' direction='in'/>"
    "    </method>"
    "    <method name='GetVideoStatus'>"
    "      <arg type='s' name='streaming_status' direction='out'/>"
    "      <arg type='s' name='image_format' direction='out'/>"
    "      <arg type='u' name='image_width' direction='out'/>"
    "      <arg type='u' name='image_height' direction='out'/>"
    "    </method>"
    "    <signal name='GetCameraProperty'>"
    "      <annotation name='org.gtk.GDBus.Annotation' value='Onsignal'/>"
    "      <arg type='s' name='prop_name'/>"
    "      <arg type='s' name='request_type'/>"
    "      <arg type='q' name='request_length'/>"
    "    </signal>"
    "    <signal name='SetCameraProperty'>"
    "      <annotation name='org.gtk.GDBus.Annotation' value='Onsignal'/>"
    "      <arg type='s' name='prop_name'/>"
    "      <arg type='s' name='request_type'/>"
    "      <arg type='q' name='prop_length'/>"
    "      <arg type='ay' name='prop_data'/>"
    "    </signal>"
    "    <signal name='video_control'>"
    "      <annotation name='org.gtk.GDBus.Annotation' value='Onsignal'/>"
    "      <arg type='s' name='streaming_status'/>"
    "      <arg type='s' name='image_format'/>"
    "      <arg type='u' name='image_width'/>"
    "      <arg type='u' name='image_height'/>"
    "    </signal>"
    "  </interface>"
    "</node>";

/* ---------------------------------------------------------------------------------------------------- */

static void
send_uvc_stream_status_signal(struct uvc_device *dev)
{
    GError *error;
    const char *streaming_status =
        dev->stream_on ? "start stream" : "stop stream";
    const char *format_str = getV4L2FormatStr(dev->fcc);

    error = NULL;

    if (dbus_conn) {
        g_dbus_connection_emit_signal(dbus_conn,
                                      NULL,
                                      PLCM_USB_VISAGE_UVC_OBJ_PATH,
                                      PLCM_USB_VISAGE_UVC_INTF_NAME,
                                      PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL,
                                      g_variant_new("(ssuu)",
                                              streaming_status,
                                              format_str,
                                              dev->width,
                                              dev->height),
                                      &error);

        if (error != NULL) {
            g_printerr("Failed to emit signals. error: %s\n", error->message);
            g_clear_error(&error);
        }
    }
}

static void
send_get_property_signal(struct uvc_device *dev, uint8_t req, uint8_t cs,
                         uint8_t unit_id, uint16_t length)
{
    GError *error;
    const char *prop_name, *request_type;
    (void)dev;
    error = NULL;
    request_type = getUVCReqStr(req);
    prop_name = "";

    if (unit_id == UVC_PROCESSING_UNIT_CONTROL_UNIT_ID)
        prop_name = getUVCPUCS(cs);
    else if (unit_id == UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID)
        prop_name = getUVCCTCS(cs);

    if (dbus_conn) {
        g_dbus_connection_emit_signal(dbus_conn,
                                      NULL,
                                      PLCM_USB_VISAGE_UVC_OBJ_PATH,
                                      PLCM_USB_VISAGE_UVC_INTF_NAME,
                                      PLCM_USB_VISAGE_UVC_GET_PROP_SIGNAL,
                                      g_variant_new("(ssq)",
                                              prop_name,
                                              request_type,
                                              length),
                                      &error);

        if (error != NULL) {
            g_printerr("Failed to emit signals. error: %s\n", error->message);
            g_clear_error(&error);
        }
    }
}

static void
send_set_property_signal(struct uvc_device *dev, uint8_t req, uint8_t cs,
                         uint8_t unit_id, uint16_t length, uint8_t *data)
{
    GError *error;
    const char *prop_name, *request_type;
    GVariantBuilder *builder;
    uint16_t i = 0;
    (void)dev;

    error = NULL;
    request_type = getUVCReqStr(req);
    prop_name = "";

    if (unit_id == UVC_PROCESSING_UNIT_CONTROL_UNIT_ID)
        prop_name = getUVCPUCS(cs);
    else if (unit_id == UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID)
        prop_name = getUVCCTCS(cs);

    builder = g_variant_builder_new(G_VARIANT_TYPE("ay"));

    for (i = 0; i < length; i++)
        g_variant_builder_add(builder, "y", data[i]);

    if (dbus_conn) {
        g_dbus_connection_emit_signal(dbus_conn,
                                      NULL,
                                      PLCM_USB_VISAGE_UVC_OBJ_PATH,
                                      PLCM_USB_VISAGE_UVC_INTF_NAME,
                                      PLCM_USB_VISAGE_UVC_SET_PROP_SIGNAL,
                                      g_variant_new("(ssqay)",
                                              prop_name,
                                              request_type,
                                              length,
                                              builder),
                                      &error);

        g_variant_builder_unref(builder);

        if (error != NULL) {
            g_printerr("Failed to emit signals. error: %s\n", error->message);
            g_clear_error(&error);
        }
    }
}

static void
handle_method_call(GDBusConnection       *connection,
                   const gchar           *sender,
                   const gchar           *object_path,
                   const gchar           *interface_name,
                   const gchar           *method_name,
                   GVariant              *parameters,
                   GDBusMethodInvocation *invocation,
                   gpointer               user_data)
{
    struct uvc_device *dev = (struct uvc_device *)user_data;
    int retval = 0;
    int len = 0;
    struct uvc_request_data resp;

    GVariantIter *iter;
    guchar str;

    gboolean dummy_video = FALSE;

    (void)connection;
    (void)sender;
    (void)object_path;
    (void)interface_name;

    if (g_strcmp0(method_name, "SendResponse") == 0) {


        g_variant_get(parameters, "(iay)", &resp.length, &iter);

        if ((uint32_t)resp.length > sizeof(resp.data)) {
            g_printerr("length: %u is out of boundary\n", resp.length);
            g_dbus_method_invocation_return_error(invocation,
                                                  G_IO_ERROR,
                                                  G_IO_ERROR_FAILED_HANDLED,
                                                  "length is too big. Out of boundary");
            g_variant_iter_free(iter);
            return;
        }

        while (g_variant_iter_loop(iter, "y", &str))
            resp.data[len++] = str;

        g_variant_iter_free(iter);

        retval = ioctl(dev->fd, UVCIOC_SEND_RESPONSE, &resp);

        if (retval < 0) {
            g_printerr("SendResponse failed: %s (%d)\n", strerror(errno),
                       errno);
        }

        g_dbus_method_invocation_return_value(invocation,
                                              g_variant_new("(i)", retval));
    } else if (g_strcmp0(method_name, "TestVideoSource") == 0) {
        g_variant_get(parameters, "(b)", &dummy_video);
        g_print("Set video test flag: %d\n", dummy_video);

        gst_dev->test_video_flag = (dummy_video == TRUE);
    } else if (g_strcmp0(method_name, "GetVideoStatus") == 0) {
        const char *streaming_status =
            dev->stream_on ? "stream on" : "stream off";
        const char *format_str = getV4L2FormatStr(dev->fcc);

        g_print("Get video status\n");

        g_dbus_method_invocation_return_value(invocation,
                                              g_variant_new("(ssuu)",
                                                      streaming_status,
                                                      format_str,
                                                      dev->width,
                                                      dev->height));
    }
}

/* for now */
static const GDBusInterfaceVTable interface_vtable = {
    .method_call = handle_method_call,
    .get_property = NULL,
    .set_property = NULL,
    .padding = { NULL }
};

static void
on_bus_acquired(GDBusConnection *connection,
                const gchar     *name,
                gpointer         user_data)
{
    struct uvc_device *dev = (struct uvc_device *)user_data;
    guint registration_id;
    (void)dev;
    (void)name;

    dbus_conn = connection;

    registration_id = g_dbus_connection_register_object(connection,
                      "/com/polycom/visage/uvc",
                      introspection_data->interfaces[0],
                      &interface_vtable,
                      user_data,
                      NULL,  /* user_data_free_func */
                      NULL); /* GError** */
    g_assert(registration_id > 0);
}

static void
on_name_acquired(GDBusConnection *connection,
                 const gchar     *name,
                 gpointer         user_data)
{
    (void)connection;
    (void)user_data;

    g_print("bus name acquired :%s\n", name);
}

static void
on_name_lost(GDBusConnection *connection,
             const gchar     *name,
             gpointer         user_data)
{
    (void)connection;
    (void)user_data;

    g_print("bus name lost :%s\n", name);
}

/*
* Callbacks for uvc v4l2 device management
*
*/
void on_stream_on()
{
    printf("============on_stream_on============\n");
    // Start uvc video stream
    start_uvc_stream(global_uvc);
    //Notify the app with the streaming status
    send_uvc_stream_status_signal(global_uvc);
    uvc_set_timeout(global_uvc, 5000);
    //uvc_video_stream(global_uvc,1, 0);
    printf("============on_stream_on done============\n");
}
void on_stream_off()
{
    printf("============on_stream_off============\n");
    // Stop the uvc video stream
    stop_uvc_stream(global_uvc);
    //Notify the app with the streaming status
    send_uvc_stream_status_signal(global_uvc);

    uvc_set_timeout(global_uvc, 0);
    //uvc_video_stream(global_uvc,0, 0);
    printf("============on_stream_off done============\n");

}
void on_get_cam_param(uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length)
{
    printf("============on_get_cam_param req[%u] cs[%u] unit_id[%u] length[%u]============\n", req, cs, unit_id, length);
    send_get_property_signal(global_uvc, req, cs, unit_id, length);
}
void on_set_cam_param(uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length, uint8_t* data)
{
    printf("============on_get_cam_param req[%u] cs[%u] unit_id[%u] length[%u]============\n", req, cs, unit_id, length);
    struct uvc_request_data * param = (struct uvc_request_data *)data;
    send_set_property_signal(global_uvc, req, cs, unit_id, param->length, param->data);

}

void on_get_video_param()
{
}

void on_set_video_param(uint32_t format, uint32_t width, uint32_t height, uint32_t framerate)
{
    (void)framerate;
    (void)format;
    (void)width;
    (void)height;
}


int on_req_frame(uint8_t * buffer, uint32_t buffer_max_len, uint32_t *buffer_act_len)
{
    (void)buffer;
    (void)buffer_max_len;
    (void)buffer_act_len;
    return 0;
}

void on_frame_done()
{
    uvc_set_timeout(global_uvc, 1000);
}


/* ---------------------------------------------------------------------------
 * main
 */

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr, " -d device Video device\n");
    fprintf(stderr, " -t use the dummy video source\n");
    fprintf(stderr, " -h        Print this help screen and exit\n");
}

int main(int argc, char *argv[])
{
    char *device = "/dev/video6";
    struct uvc_device *dev = NULL;
    int opt;
    unsigned char dummy_video = 0;
    /* Glib relevant. */
    GMainContext *context;
    GMainLoop *loop;
    GIOChannel *channel;

    GstBus *bus;
    guint bus_watch_id, io_watch_id;

    guint owner_id;

    while ((opt = getopt(argc, argv, "d:ht")) != -1) {
        switch (opt) {
            case 'd':
                device = optarg;
                break;

            case 'h':
                usage(argv[0]);
                return 0;

            case 't':
                dummy_video = 1;
                break;

            default:
                fprintf(stderr, "Invalid option '-%c'\n", opt);
                usage(argv[0]);
                return 1;
        }
    }

    /*
     * Setup the signal handler for SIGALRM. It is used
     * for the bulk transfer mode. Because in bulk mode,
     * the driver will not send the STREAM_OFF event when
     * the host stops the video stream. We need to have a
     * timer that if we can not receive the video frame
     * transfer done event in 1 second. We will stop the
     * video and clean the buffer.
     */

    signal(SIGINT, sig_handle);
    signal(SIGTERM, sig_handle);
    signal(SIGABRT, sig_handle);

    dev = uvc_open(device);

    if (dev == NULL)
        return 1;

    global_uvc = dev;
    gst_dev = malloc(sizeof(struct gst_device));
    gst_dev->test_video_flag = dummy_video;

    struct uvc_callback_table cb_table = {
        on_stream_on,
        on_stream_off,
        on_get_cam_param,
        on_set_cam_param,
        on_get_video_param,
        on_set_video_param,
        on_req_frame,
        on_frame_done
    };

    uvc_events_init(dev, &cb_table);


    /* Initialize the g_main_loop. */
    loop = g_main_loop_new(NULL, FALSE);

    _global_loop = loop;

    context = g_main_loop_get_context(loop);

    /* Add io watch function. */
    channel = g_io_channel_unix_new(dev->fd);

    /* The poll return type of v4l2 event is POLLPRI.
     * So add G_IO_PRI here.
     */
    io_watch_id = g_io_add_watch(channel, G_IO_PRI, io_watch, dev);

    /* Intialize the gstreamer */
    gst_init(&argc, &argv);

    /* Create gstreamer elements. */
    gst_dev->pipeline = gst_pipeline_new("video-looper");

    if (!gst_dev->pipeline) {
        g_printerr("Failed to allocate the gstreamer pipeline. Exiting. \n");
        goto exit;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE(gst_dev->pipeline));
    bus_watch_id = gst_bus_add_watch(bus, bus_call, dev);
    gst_object_unref(bus);

    /* We are lazy here - we don't want to manually provide
    * the introspection data structures - so we just build
    * them from XML.
    */
    introspection_data = g_dbus_node_info_new_for_xml(introspection_xml, NULL);
    g_assert(introspection_data != NULL);

    owner_id = g_bus_own_name(G_BUS_TYPE_SYSTEM,
                              "com.polycom.visage.uvc",
                              G_BUS_NAME_OWNER_FLAGS_NONE,
                              on_bus_acquired,
                              on_name_acquired,
                              on_name_lost,
                              dev,
                              NULL);

    /* Iterate. */
    g_print("Running...\n");
    g_main_loop_run(loop);

    g_print("Loop returned\n");

    /* Clean up the dbus. */
    g_bus_unown_name(owner_id);
    g_dbus_node_info_unref(introspection_data);

    /* Clean up the gstreamer. */
    g_print("Stop the video looper pipeline\n");
    gst_element_set_state(gst_dev->pipeline, GST_STATE_NULL);
    g_print("Deleting the pipeline\n");

    gst_object_unref(GST_OBJECT(gst_dev->pipeline));

    /* Clean up the gstreamer bus. */
    g_source_remove(bus_watch_id);

    if (gst_dev->timeout_watch_id) {
        g_source_remove(gst_dev->timeout_watch_id);
        gst_dev->timeout_watch_id = 0;
    }

exit:
    g_source_remove(io_watch_id);
    g_io_channel_shutdown(channel, TRUE, NULL);
    g_io_channel_unref(channel);

    g_main_context_unref(context);
    g_main_loop_unref(loop);

    uvc_close(dev);

    if (gst_dev != NULL)
        free(gst_dev);

    gst_dev = NULL;

    return 0;
}
