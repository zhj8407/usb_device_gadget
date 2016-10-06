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

#include "uvc.h"

#include "log_str.h"
#define clamp(val, min, max) ({                 \
        typeof(val) __val = (val);              \
        typeof(min) __min = (min);              \
        typeof(max) __max = (max);              \
        (void) (&__val == &__min);              \
        (void) (&__val == &__max);              \
        __val = __val < __min ? __min: __val;   \
        __val > __max ? __max: __val; })

#define ARRAY_SIZE(a)   ((sizeof(a) / sizeof(a[0])))

#define UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID     (1)
#define UVC_PROCESSING_UNIT_CONTROL_UNIT_ID     (2)

#define WEBCAM_VIDEO_CONTROL_DEVICE_SYS_PATH    "/sys/class/plcm_usb/plcm0/f_webcam/webcam_v4l2_ctrl_device"
#define WEBCAM_VIDEO_STREAM_DEVICE_SYS_PATH    "/sys/class/plcm_usb/plcm0/f_webcam/webcam_v4l2_strm_device"
#define WEBCAM_MAXPACKET_SYS_PATH   "/sys/class/plcm_usb/plcm0/f_webcam/webcam_maxpacket"
#define WEBCAM_HEADERSIZE_SYS_PATH  "/sys/class/plcm_usb/plcm0/f_webcam/webcam_headersize"
#define WEBCAM_BULKMODE_SYS_PATH    "/sys/class/plcm_usb/plcm0/f_webcam/webcam_bulkmode"
#define WEBCAM_MAXPAYLOAD_SYS_PATH  "/sys/class/plcm_usb/plcm0/f_webcam/webcam_maxpayload"


#define VISAGE_LED_NOTIFICATION 1

struct uvc_device {
    const char *v4l2_src_device;
    int fd;

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

    pid_t streaming_pid;
};

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
    char v4ldevname[64];
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

    snprintf(v4ldevname, sizeof(v4ldevname), "/dev/video%d", dev->v4l2_ctrl_devnum);

    printf("We are trying to open the dev: %s\n", v4ldevname);

    fd = open(v4ldevname, O_RDWR | O_NONBLOCK);

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

//#define VIDEO_TEST_SRC             1
//#define FORK_GST_LAUNCH_CHILD      1

#define CAM_DEF_WIDTH 1920
#define CAM_DEF_HEIGHT 1080
#define CAM_MAX_WIDTH 1920
#define CAM_MAX_HEIGHT 1080

#ifdef VIDEO_TEST_SRC
#define CAM_DEF_FRAMERATE 30
#else
#define CAM_DEF_FRAMERATE 60
#endif

#ifdef VIDEO_TEST_SRC
unsigned int camera_format = V4L2_PIX_FMT_YUYV;
#else
unsigned int camera_format = V4L2_PIX_FMT_NV12;
#endif
unsigned int camera_width = CAM_DEF_WIDTH;
unsigned int camera_height = CAM_DEF_HEIGHT;
unsigned int camera_framerate = CAM_DEF_FRAMERATE;

unsigned int output_framerate = CAM_DEF_FRAMERATE;
unsigned int encoder_quality = 85;

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
                    get_gst_camera_cmd(v4l2_src_dev, camera_format, camera_width, camera_height, CAM_DEF_FRAMERATE),
                    get_gst_scaler_cmd(camera_format, width, height),
                    get_gst_convertor_cmd(format),
                    get_gst_encoder_cmd(format, encoder_quality),
                    get_gst_v4l2output_cmd(v4l2_output_devnum));
}

pid_t start_uvc_stream(struct uvc_device *dev)
{
#ifdef FORK_GST_LAUNCH_CHILD
    pid_t child;
    char gst_running_cmd[GST_CMD_TOTAL_LEN];

    if ((child = fork()) == 0) {
        get_gst_running_cmd(gst_running_cmd,
                            dev->v4l2_src_device,
                            dev->fcc,
                            dev->width,
                            dev->height,
                            dev->v4l2_strm_devnum);
        printf("gstreamer running cmd: %s\n", gst_running_cmd);
        //Child process
        execl("/bin/sh", "sh", "-c",
              gst_running_cmd, (char *) 0);
        perror("gst-launch-1.0");
        exit(errno);
    } else {
        //Parent process
    }

    dev->streaming_pid = child;

    return child;
#else
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

    dev->streaming_pid = 999;

    return dev->streaming_pid;
#endif
}

void stop_uvc_stream(struct uvc_device *dev)
{
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(dev->fd, VIDIOC_STREAMOFF, &type);
#ifdef FORK_GST_LAUNCH_CHILD

    if (dev->streaming_pid > 0)
        kill(dev->streaming_pid, SIGKILL);

#else

    if (dev->streaming_pid > 0)
        system("sudo killall -9 gst-launch-1.0");

#endif
    dev->streaming_pid = -1;
}

/* ---------------------------------------------------------------------------
 * Request processing
 */

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
        ctrl->dwMaxPayloadTransferSize = dev->maxpacketsize;   /* TODO this should be filled by the driver. */
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

__u16 brightness = 0x0004;

static void
uvc_events_process_control(struct uvc_device *dev, uint8_t req, uint8_t cs,
                           uint8_t unit_id, struct uvc_request_data *resp)
{
    __u16 *wValuePtr = (__u16 *)(resp->data);
    printf("control request (req %02x cs %02x)\n", req, cs);
    (void)dev;

    switch (cs) {
        case UVC_PU_BRIGHTNESS_CONTROL:
            switch (req) {
                case UVC_GET_INFO:
                    resp->data[0] = 0x03;
                    resp->length = 1;
                    break;

                case UVC_GET_MIN:
                    *wValuePtr = 0x0000;
                    resp->length = 2;
                    break;

                case UVC_GET_MAX:
                    *wValuePtr = 0x0009;
                    resp->length = 2;
                    break;

                case UVC_GET_RES:
                    *wValuePtr = 0x0001;
                    resp->length = 2;
                    break;

                case UVC_GET_DEF:
                    *wValuePtr = 0x0004;
                    resp->length = 2;
                    break;

                case UVC_GET_CUR:
                    *wValuePtr = brightness;
                    resp->length = 2;
                    break;

                case UVC_GET_LEN:
                    *wValuePtr = sizeof(brightness);
                    resp->length = 2;
                    break;

                case UVC_SET_CUR:
                    //TODO
                    dev->control = cs;
                    dev->unit = unit_id;
                    resp->length = 2;
                    break;

                default:
                    resp->length = 0;
                    break;
            }

            break;

        default:
            resp->length = 0;
            break;
    }
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
        uvc_events_process_control(dev, ctrl->bRequest, ctrl->wValue >> 8, ctrl->wIndex >> 8, resp);
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

        case UVC_PROCESSING_UNIT_CONTROL_UNIT_ID << 8 | UVC_PU_BRIGHTNESS_CONTROL:
            printf("setting UVC_PU_BRIGHTNESS_CONTROL, length = %d\n", data->length);
            brightness = *(__u16 *)(data->data);
            return;

        default:
            printf("setting unknown control, length = %d\n", data->length);
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
            alarm(8);
        }
    } else if (dev->control == UVC_VS_PROBE_CONTROL) {
        if (dev->bulk) {
            alarm(0);
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
        alarm(1);
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
            return;

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
            return;

        case UVC_EVENT_STREAMON:
            //TODO Start uvc video stream
            start_uvc_stream(dev);
            alarm(8);

            return;

        case UVC_EVENT_STREAMOFF:
            // Cacel the alarm.
            alarm(0);

            //TODO Stop the uvc video stream
            stop_uvc_stream(dev);

            return;

        case UVC_EVENT_FRAMEDONE:
            handle_frame_done_event(dev);
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
        /* FIXME Crude hack, must be negotiated with the driver. */
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

/* ---------------------------------------------------------------------------
 * main
 */

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr, " -d device	Video device\n");
    fprintf(stderr, " -h		Print this help screen and exit\n");
}

static struct uvc_device *global_uvc = NULL;

void sig_handle(int sig)
{
#ifdef FORK_GST_LAUNCH_CHILD
    pid_t pid;
    int status;
#endif

    printf("Received signal: %d(%s)\n", sig, strsignal(sig));

    // Alarm timeout. Stop the video
    if (sig == SIGALRM) {
        if (global_uvc)
            stop_uvc_stream(global_uvc);
    }

#ifdef FORK_GST_LAUNCH_CHILD
    else if (sig == SIGCHLD) {
        printf("Child process has been killed, Sig: %d\n", sig);

        while ((pid = waitpid(-1, &status, WNOHANG)) > 0);
    }

#endif
}

int main(int argc, char *argv[])
{
    char *device = "/dev/video2";
    struct uvc_device *dev = NULL;
    fd_set fds;
    int ret, opt;

    while ((opt = getopt(argc, argv, "d:h")) != -1) {
        switch (opt) {
            case 'd':
                device = optarg;
                break;

            case 'h':
                usage(argv[0]);
                return 0;

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

    signal(SIGALRM, sig_handle);
#ifdef FORK_GST_LAUNCH_CHILD
    signal(SIGCHLD, sig_handle);
#endif

    dev = uvc_open(device);

    if (dev == NULL)
        return 1;

    global_uvc = dev;

    uvc_events_init(dev);
    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    while (1) {
        fd_set efds = fds;
        ret = select(dev->fd + 1, NULL, NULL, &efds, NULL);

        if (ret == -1) {
            if (errno != EINTR) {
                printf("Error in select\n");
                break;
            }
        } else {
            if (FD_ISSET(dev->fd, &efds)) {
                uvc_events_process(dev);
            }
        }
    }

    uvc_close(dev);

    return 0;
}
