#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include "uvc.h"

//#include "dbus_utils.h"
//#include "plcm_usb_intf.h"

#include "log_str.h"
#include "uvc-gadget-func.h"
#include "diag_utils.h"
/* ---------------------------------------------------------------------------
 * Request processing
 */
#define clamp(val, min, max) ({                 \
        typeof(val) __val = (val);              \
        typeof(min) __min = (min);              \
        typeof(max) __max = (max);              \
        (void) (&__val == &__min);              \
        (void) (&__val == &__max);              \
        __val = __val < __min ? __min: __val;   \
        __val > __max ? __max: __val; })

unsigned int deqBufFailCount = 0;
unsigned int reqBufFailCount = 0;
unsigned int frame_count = 0;

/*********************************
* stats
*********************************/
unsigned int print_interval = STATS_PRINT_INTERVAL;
unsigned long getFrameTime = 0; //GetTimeInMilliSec();
unsigned long getNextFrameTime = 0; //GetTimeInMilliSec();
//unsigned long videoStartTime = 0;

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

static const struct uvc_frame_info uvc_frames_nv12[] = {
    { 1280, 720, { 333333, 666666, 10000000, 50000000, 0 }, },
    { 1920, 1080, { 10000000, 50000000, 0 }, },
    { 0, 0, { 0, }, },
};


static const struct uvc_frame_info uvc_frames_mjpeg[] = {
    {  640, 360, { 333333, 666666, 50000000, 0 }, },
    { 1280, 720, { 50000000, 0 }, },
    { 1920, 1080, { 333333, 666666, 0 }, },
    { 0, 0, { 0, }, },
};

static const struct uvc_format_info uvc_formats[] = {
    { V4L2_PIX_FMT_YUYV, uvc_frames_yuyv },
    { V4L2_PIX_FMT_YUV420, uvc_frames_i420 },
    //{ V4L2_PIX_FMT_NV12, uvc_frames_nv12 },
    { V4L2_PIX_FMT_MJPEG, uvc_frames_mjpeg },
};

void uvc_fill_streaming_control(struct uvc_device *dev,
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

    ctrl->bmFramingInfo = 3;
    ctrl->bPreferedVersion = 1;
    ctrl->bMaxVersion = 1;
}

void
uvc_events_process_standard(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                            struct uvc_request_data *resp)
{
    printf("standard request\n");
    (void)dev;
    (void)ctrl;
    resp->length = 0;
}

__u16 brightness = 0x0004;

void
uvc_events_process_control(struct uvc_device *dev, uint8_t req, uint8_t cs,
                           uint8_t unit_id, uint16_t length, struct uvc_request_data *resp)
{
#if SYNC_CON_APP
    printf("control request (req %02x cs %02x)\n", req, cs);
    (void)dev;

    /*if (req != UVC_SET_CUR) {
        send_get_property_signal(dev->dbus_con, req, cs, unit_id, length);
    } else {
        dev->control = cs;
        dev->unit = unit_id;
    }*/
    dev->callbacks.on_get_param(req, cs, unit_id, length);
    // Do not send the reply here.
    // We have posted a singal. The client will do it.
    resp->length = -EL2HLT;

#else
    (void)length;
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

#endif
}

void
uvc_events_process_streaming(struct uvc_device *dev, uint8_t req, uint8_t cs,
                             struct uvc_request_data *resp)
{
    struct uvc_streaming_control *ctrl;
    //int ret = -1;
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

            //ret = sendEvent2Fifo(dev->stack2app_fd, e_get_format, dev->fcc, dev->width, dev->height);
            dev->callbacks.on_get_video_param();//is it needed?

            //if (ret < 0)
            //    printf("send %s %ux%u to app failed %d %s\n", getEventDescStr(e_get_format), dev->width, dev->height, errno, strerror(errno));

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

void
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

void
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

void
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

        default://ctrl
            //send_set_property_signal(dev->dbus_con, UVC_SET_CUR, dev->control, dev->unit, data->length, data->data);
            dev->callbacks.on_set_cam_param(UVC_SET_CUR, dev->control, dev->unit, data->length, data->data);
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
            target->wCompQuality = JPEG_QUALITY;
            break;
    }

    target->dwFrameInterval = *interval;

    if (dev->control == UVC_VS_COMMIT_CONTROL) {
        dev->fcc = format->fcc;
        dev->width = frame->width;
        dev->height = frame->height;
        uvc_video_set_format(dev);
        printf("[COMMIT]: set format=%u, %ux%u\n", dev->fcc, dev->width, dev->height);
        //sendEvent2Fifo(dev->stack2app_fd, e_set_format, dev->fcc, dev->width, dev->height);
        dev->callbacks.on_set_video_param(dev->fcc, dev->width, dev->height, 30);

        if (dev->bulk) {
            /* In bulk mode, we can't receive the set_alt
             * request from the host. That means no STREAM_ON
             * or STREAM_OFF event from the gadget driver. So
             * we need to start the transfer immediatly after
             * receiving the COMMIT Set_CUR.
             */
            //Cancel the alarm. Stop the stream if possible
            alarm(0);
            uvc_video_stream(dev, 0);
            uvc_video_reqbufs(dev, 0);

            uvc_video_reqbufs(dev, 2);
            uvc_video_stream(dev, 1);
            alarm(3);
        }
    }
}

void handle_frame_done_event(struct uvc_device *dev,
                             struct uvc_frame_done_info *frame_done_info)
{
#if 0
    printf("Image transfer done. index: %u, bytes_transferred: %u, status: %d\n",
           frame_done_info->buffer_index,
           frame_done_info->bytes_transferred,
           frame_done_info->status);
#endif
    (void)frame_done_info;

    if (dev->bulk) {
        // Reset the alarm.
        alarm(3);
    }
}


void
uvc_events_process(struct uvc_device *dev)
{
    struct v4l2_event v4l2_event;
    struct uvc_event *uvc_event = (void *)&v4l2_event.u.data;
    struct uvc_request_data resp;
    struct usb_ctrlrequest *ctrl_req = &uvc_event->req;
    struct uvc_frame_done_info *frame_done_info;
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
            //printHexData((char *)ctrl_req, sizeof(*ctrl_req), getUVCEventStr(v4l2_event.type));
            uvc_events_process_setup(dev, ctrl_req, &resp);
            break;

        case UVC_EVENT_DATA:
            //printHexData((char *)&uvc_event->data, sizeof(uvc_event->data), getUVCEventStr(v4l2_event.type));
            uvc_events_process_data(dev, &uvc_event->data);
            break;

        case UVC_EVENT_STREAMON:
            uvc_video_reqbufs(dev, 2);
            uvc_video_stream(dev, 1);
            break;

        case UVC_EVENT_STREAMOFF:
            // Cacel the alarm.
            alarm(0);
            uvc_video_stream(dev, 0);
            uvc_video_reqbufs(dev, 0);
            break;

        case UVC_EVENT_FRAMEDONE:
            frame_done_info = (void *)&v4l2_event.u.data;
            handle_frame_done_event(dev, frame_done_info);
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
            return;
        }
    }

    //printHexData((char *)(&resp), sizeof(resp), "REPLY");
    printf("[END]uvc_events_process: DONE with V4L2 event [0x%x] %s\n", v4l2_event.type, getUVCEventStr(v4l2_event.type));
}

void
uvc_events_init(struct uvc_device *dev, struct uvc_callback_table * callback_table)
{
    if (dev == NULL || callback_table == NULL) {
        printf("[uvc_events_init]: invalid param: dev=%p, callbacks=%p\n", dev, callback_table);
    }

    struct v4l2_event_subscription sub;

    uvc_fill_streaming_control(dev, &dev->probe, 0, 0);

    uvc_fill_streaming_control(dev, &dev->commit, 0, 0);

    if (dev->bulk) {
        /* FIXME Crude hack, must be negotiated with the driver. */
        dev->probe.dwMaxPayloadTransferSize = dev->maxpayloadsize;
        dev->commit.dwMaxPayloadTransferSize = dev->maxpayloadsize;
    }

    memcpy(&(dev->callbacks), callback_table, sizeof(struct uvc_callback_table));

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

int uvc_video_stream(struct uvc_device *dev, int enable)
{
    struct v4l2_buffer buf;
    unsigned int i;
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int ret;

    if (!enable) {
        printf("Stopping video stream.\n");
        ioctl(dev->fd, VIDIOC_STREAMOFF, &type);
#ifdef VISAGE_LED_NOTIFICATION
        /* Set the led to red. */
        system("/usr/sbin/commanduC lightLed 1 1");
#endif
        frame_count = 0;
        dev->stream_on = 0;
        dev->callbacks.on_stream_off();

        return 0;
    }

    dev->stream_on = 1;
    dev->callbacks.on_stream_on();

    for (i = 0; i < dev->nbufs; ++i) {
        memset(&buf, 0, sizeof buf);
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.length = dev->bufsize;
        memset(dev->mem[buf.index], 0x00, buf.length);
        buf.bytesused = 0;
        printf("Queueing buffer %u.\n", i);

        if ((ret = ioctl(dev->fd, VIDIOC_QBUF, &buf)) < 0) {
            printf("Unable to queue buffer: %s (%d).\n",
                   strerror(errno), errno);
            break;
        }
    }

    ioctl(dev->fd, VIDIOC_STREAMON, &type);
#ifdef VISAGE_LED_NOTIFICATION

    /* If the host selects the I420 30fp format, we
     * will set the led to Green, Fast blink.
     * Else set the led to Amber, Slow blink.
     */
    if (dev->fcc == V4L2_PIX_FMT_YUV420 &&
            dev->width == 1280 &&
            dev->height == 720)
        system("/usr/sbin/commanduC lightLed 2 3");
    else
        system("/usr/sbin/commanduC lightLed 4 2");

#endif
    return ret;
}

int uvc_video_set_format(struct uvc_device *dev)
{
    struct v4l2_format fmt;
    int ret;
    printf("Setting format to 0x%08x[%s] %ux%u [%s]\n",
           dev->fcc, getV4L2FormatStr(dev->fcc), dev->width, dev->height, (char*)(&(dev->fcc)));
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = dev->width;
    fmt.fmt.pix.height = dev->height;
    fmt.fmt.pix.pixelformat = dev->fcc;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (dev->fcc == V4L2_PIX_FMT_MJPEG) {
        fmt.fmt.pix.sizeimage = dev->width * dev->height * 1.5;
    }

    if ((ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt)) < 0)
        printf("Unable to set format: %s (%d).\n",
               strerror(errno), errno);

    return ret;
}

int uvc_video_process(struct uvc_device *dev)
{
    struct v4l2_buffer buf;
    int ret;
    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_MMAP;

    if ((ret = ioctl(dev->fd, VIDIOC_DQBUF, &buf)) < 0) {
        if (dev->stream_on) {
            printf("frame[%u]Unable to dequeue buffer: count=%u %s (%d). \n", frame_count, deqBufFailCount++, strerror(errno), errno);
        }

        return ret;
    } else {
        //printf("frame[%u] successfully Dequeue buffer %p\n", frame_count, &buf);
        deqBufFailCount = 0;
    }

    ret = dev->callbacks.on_req_frame(dev->mem[buf.index], buf.length, &(buf.bytesused));

    if (ret)
        buf.bytesused = 0;

    frame_count++;

    if ((ret = ioctl(dev->fd, VIDIOC_QBUF, &buf)) < 0) {
        if (dev->stream_on) {
            printf("frame[%u]Unable to requeue buffer: count=%u  %s (%d).\n", frame_count, reqBufFailCount++, strerror(errno), errno);
        }

        return ret;
    } else {
        //printf("frame[%u] successfully Requeue buffer %p\n", frame_count, &buf);
        reqBufFailCount = 0;
    }

    if (frame_count % print_interval == 0) {
        if (getFrameTime == 0) {
            getFrameTime = GetTimeInMilliSec();
        }

        getNextFrameTime = GetTimeInMilliSec();
        printf("Sending %d frame(total: %u) of [%s] %ux%u delta time=%lu ms\n",
               print_interval, frame_count, getV4L2FormatStr(dev->fcc), dev->width, dev->height, getNextFrameTime - getFrameTime);
        getFrameTime = getNextFrameTime;
    }

    return 0;
}

int uvc_video_reqbufs(struct uvc_device *dev, int nbufs)
{
    struct v4l2_requestbuffers rb;
    struct v4l2_buffer buf;
    unsigned int i;
    int ret;

    for (i = 0; i < dev->nbufs; ++i)
        munmap(dev->mem[i], dev->bufsize);

    free(dev->mem);
    dev->mem = 0;
    dev->nbufs = 0;

    if (nbufs <= 0)
        return 0;

    memset(&rb, 0, sizeof rb);
    rb.count = nbufs;
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rb.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);

    if (ret < 0) {
        printf("Unable to allocate buffers: %s (%d).\n",
               strerror(errno), errno);
        return ret;
    }

    printf("%u buffers allocated.\n", rb.count);
    /* Map the buffers. */
    dev->mem = malloc(rb.count * sizeof dev->mem[0]);

    for (i = 0; i < rb.count; ++i) {
        memset(&buf, 0, sizeof buf);
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);

        if (ret < 0) {
            printf("Unable to query buffer %u: %s (%d).\n", i,
                   strerror(errno), errno);
            return -1;
        }

        printf("length: %u offset: %u\n", buf.length, buf.m.offset);
        dev->mem[i] = mmap(0, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd, buf.m.offset);

        if (dev->mem[i] == MAP_FAILED) {
            printf("Unable to map buffer %u: %s (%d)\n", i,
                   strerror(errno), errno);
            return -1;
        }

        printf("Buffer %u mapped at address %p.\n", i, dev->mem[i]);
    }

    dev->bufsize = buf.length;
    dev->nbufs = rb.count;
    return 0;
}

