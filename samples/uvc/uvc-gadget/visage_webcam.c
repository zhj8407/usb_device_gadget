#include "stdio.h"
#include "unistd.h"
#include <sys/select.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/videodev2.h>

#include "visage_webcam.h"
#include "visage_shared_mem.h"
#include "plcm_usb_intf.h"
#include "scale.h"


static const char * V4L2PixFormatStr [] = {
    "[V4L2_PIX_FMT_YUYV]",
    "[V4L2_PIX_FMT_MJPEG]",
    "[V4L2_PIX_FMT_H264]",
    "[V4L2_PIX_FMT_NV12]",
    "[V4L2_PIX_FMT_YUV420]",
    "[Unknown Format]"
};

inline const char * getV4L2FormatStr(unsigned int fcc)
{
    switch (fcc) {
        case V4L2_PIX_FMT_YUYV:
            return V4L2PixFormatStr[0];

        case V4L2_PIX_FMT_MJPEG:
            return V4L2PixFormatStr[1];;

        case V4L2_PIX_FMT_H264:
            return V4L2PixFormatStr[2];

        case V4L2_PIX_FMT_NV12:
            return V4L2PixFormatStr[3];

        case V4L2_PIX_FMT_YUV420:
            return V4L2PixFormatStr[4];

        default:
            return "[Unknown Format]";
    }
};

/*******************for camera capture begin*********/
struct cam_buffer {
    unsigned int size;
    void *mem;
};
struct cam_buffer *buffers = NULL;
int fd = -1;
unsigned int nbufs = 2;
unsigned char *pattern = NULL;
unsigned int bSelectIO = 1;
enum v4l2_buf_type buftype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
unsigned int pixelformat = V4L2_PIX_FMT_YUV420;
//unsigned int pixelformat = V4L2_PIX_FMT_YUYV;
enum v4l2_memory memtype = V4L2_MEMORY_MMAP ;
#define CAM_MAX_WIDTH 1920
#define CAM_MAX_HEIGHT 1080
#define CAM_DEF_WIDTH 1280
#define CAM_DEF_HEIGHT 720
#define CAM_DEF_FRAMERATE 60

unsigned int bytesperline = CAM_DEF_WIDTH;
unsigned int imagesize = CAM_DEF_WIDTH * CAM_DEF_HEIGHT;

unsigned int camera_format = V4L2_PIX_FMT_NV12;
unsigned int camera_width = CAM_DEF_WIDTH;
unsigned int camera_height = CAM_DEF_HEIGHT;
unsigned int camera_framerate = CAM_DEF_FRAMERATE;

unsigned int output_format = V4L2_PIX_FMT_NV12;
unsigned int output_width = CAM_DEF_WIDTH;
unsigned int output_height = CAM_DEF_HEIGHT;
unsigned int output_framerate = CAM_DEF_FRAMERATE;


#define VISAGE_CAM_DEV "/dev/video2"
int app2stack;
char * fifo_in_name = "/tmp/to_usb_stack";
int stack2app;
char * fifo_out_name = "/tmp/from_usb_stack";

pthread_mutex_t * shm_lock = NULL;

int video_queue_buffer(int index)
{
    struct v4l2_buffer buf;
    int ret = 0;
    memset(&buf, 0, sizeof buf);
    buf.index = index;
    buf.type = buftype;
    buf.memory = memtype;
    buf.length = buffers[index].size;

    if (buftype == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
        buf.bytesused = buf.length;

        if (pattern != NULL) {
            //if(bUseOptimizedMemcpy)
            //  my_memcpy(buffers[buf.index].mem, pattern, buf.bytesused);
            //else
            memcpy(buffers[buf.index].mem, pattern, buf.bytesused);
        } else {
            memset(buffers[buf.index].mem, 0x0, buf.bytesused);
        }
    } else {
        //memset(buffers[buf.index].mem, 0x55, buf.length);
    }

    ret = ioctl(fd, VIDIOC_QBUF, &buf);

    if (ret < 0)
        printf("Unable to queue buffer (%d).\n", errno);

    return ret;
}

int  video_dequeue_buffer(struct v4l2_buffer *buf)
{
    int ret = 0;
    memset(buf, 0, sizeof(struct v4l2_buffer));
    buf->type = buftype;
    buf->memory = memtype;
    ret = ioctl(fd, VIDIOC_DQBUF, buf);
    return ret;
}

void video_int_capture(const char * devname)
{
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(struct v4l2_capability));

    if (bSelectIO)
        fd = open(devname, O_RDWR | O_NONBLOCK);
    else
        fd = open(devname, O_RDWR);

    ioctl(fd, VIDIOC_QUERYCAP, &cap);

    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        buftype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        printf("buf type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
    } else if (cap.capabilities & V4L2_CAP_VIDEO_OUTPUT) {
        buftype = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        printf("buf type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
    } else {
        printf("unknown buf type:%u\n", cap.capabilities);
    }

#if 0

    if (bDump && (buftype == V4L2_BUF_TYPE_VIDEO_CAPTURE))
        fpDump = fopen(szDump, "wb");
    else if (bDump && (buftype == V4L2_CAP_VIDEO_OUTPUT))
        fpDump = fopen(szDump, "rb");

#endif
    return;
}

int  video_set_format(unsigned int capture_width, unsigned int capture_height)
{
    int ret = 0;
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = buftype;
    fmt.fmt.pix.width = capture_width;
    fmt.fmt.pix.height = capture_height;
    fmt.fmt.pix.pixelformat = pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    ret =  ioctl(fd, VIDIOC_S_FMT, &fmt);

    if (ret < 0) {
        printf("Unable to set format: %s (%d).\n", strerror(errno), errno);
        return ret;
    }

    printf("Video format set: width: %u height: %u buffer size: %u\n",
           fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
    return ret;
}
int video_get_format()
{
    struct v4l2_format fmt;
    int ret;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = buftype;
    ret = ioctl(fd, VIDIOC_G_FMT, &fmt);

    if (ret < 0) {
        printf("Unable to get format: %s (%d).\n", strerror(errno),
               errno);
        return ret;
    }

    camera_width = fmt.fmt.pix.width;
    camera_height = fmt.fmt.pix.height;
    bytesperline = fmt.fmt.pix.bytesperline;
    imagesize =  fmt.fmt.pix.sizeimage ;
    printf("Video format: %c%c%c%c (%08x) , wxh - ->%ux%u, stride --> %u, imgesize--->%u\n",
           (fmt.fmt.pix.pixelformat >> 0) & 0xff,
           (fmt.fmt.pix.pixelformat >> 8) & 0xff,
           (fmt.fmt.pix.pixelformat >> 16) & 0xff,
           (fmt.fmt.pix.pixelformat >> 24) & 0xff,
           fmt.fmt.pix.pixelformat,
           fmt.fmt.pix.width, fmt.fmt.pix.height, bytesperline, imagesize);
    return 0;
}

int video_prepare_capture()
{
    struct v4l2_requestbuffers rb;
    struct v4l2_buffer buf;
    int i  = 0;
    int ret = 0;
    memset(&rb, 0, sizeof rb);
    rb.count = nbufs;
    rb.type = buftype;
    rb.memory = memtype;
    ret = ioctl(fd, VIDIOC_REQBUFS, &rb);
    buffers = malloc(rb.count * sizeof(struct cam_buffer));
    memset(buffers, 0x0, rb.count * sizeof(struct cam_buffer));

    for (i = 0; i < (int)rb.count; ++i) {
        memset(&buf, 0, sizeof buf);
        buf.index = i;
        buf.type = buftype;
        buf.memory = memtype;
        ioctl(fd, VIDIOC_QUERYBUF, &buf);
        printf("length: %u offset: %u\n", buf.length, buf.m.offset);
        buffers[i].mem = mmap(0,  buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (buffers[i].mem == MAP_FAILED) {
            printf("Unable to map buffer %u (%d)\n", i, errno);
            return ret;
        }

        buffers[i].size = buf.length;
        printf("Buffer %u mapped at address %p size %u\n", i, buffers[i].mem, buffers[i].size);
        memset(buffers[i].mem, 0x0, buffers[i].size);
    }

    nbufs = rb.count;

    /*if (buftype == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
    ret = video_allocate_display_buffer(buffers[0].size);
    if (ret < 0)
       return ret;
    }*/
    for (i = 0; i < (int)nbufs; ++i) {
        ret = video_queue_buffer(i);

        if (ret  != 0) break;
    }

    return ret;
}

void video_enable(int enable)
{
    ioctl(fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &buftype);
    return;
}

int start_camera(uint32_t captureWidth, uint32_t captureHeight)
{
    int ret = 0;
    video_int_capture(VISAGE_CAM_DEV);
    ret = video_set_format(captureWidth, captureHeight);

    if (ret != 0) {
        printf("[start_camera]: set format failed: %d\n", ret);
        return ret;
    } else
        printf("[start_camera]: set format success: %d\n", ret);

    ret = video_get_format();

    if (ret != 0) {
        printf("[start_camera]: get format failed: %d\n", ret);
        return ret;
    } else
        printf("[start_camera]: get format success: %d\n", ret);

    ret = video_prepare_capture();

    if (ret != 0) {
        printf("[start_camera]: prepare capture failed: %d\n", ret);
        return ret;
    } else
        printf("[start_camera]: prepare capture success: %d\n", ret);

    video_enable(1);
    return 0;
}

void stop_camera()
{
    video_enable(0);
}
#define LEN_720P_I420 (1280*720*3/2)
#define LEN_720P_YUYV (1280*720*2)
#define LEN_1080P_YUYV (1920*1080*2)

#define VBUF_LEN LEN_1080P_YUYV
//static unsigned int frame_count = 0;
uint8_t vBuf[VBUF_LEN];
//char vBuf[LEN_1080P_YUYV];


int read_one_camera_frame(void * buffer, unsigned int bufferLen, unsigned int * readLen)
{
    struct v4l2_buffer buf;
    int ret = 0;
    int op_ret = 0;
    unsigned int count = 0;
#if 0

    if (bufferLen == 0) {
        printf("[read_one_camera_frame]: bufferlen=0\n");
        return -1;
    }

    memset(buffer, (frame_count * 4) & 0xff, bufferLen);
    *readLen = bufferLen;
    return 0;
#else
    //do
    {
        op_ret = video_dequeue_buffer(&buf);

        if (op_ret < 0) {
            if (errno == EAGAIN) {
                //printf("[%d]Got EAGAIN!! [%u] %u\n", __LINE__, buf.index, count);
            }

            else {// if (errno != EIO) {
                //printf("Unable to dequeue buffer [%u] (%d) (%s).count=%u\n",buf.index, errno, strerror(errno), count);
            }

            buf.type = buftype;
            buf.memory = memtype;
            ret = -1;
        } else {
            if (buf.bytesused > bufferLen) {
                printf("ERROR! buffer not enough: len=%u, need %u\n", bufferLen, buf.bytesused);
                ret = -1;
            } else {
                //printf("Successfully read from camera: len=%u, need %u, buf.index=%u count=%u\n", bufferLen, buf.bytesused, buf.index, count);
                //printf("lock and copy!\n");
                /*int ret=pthread_mutex_lock(shm_lock);
                if (ret!=0) {
                     printf("pthread_mutex_lock failed: %d %s\n", errno, strerror(errno));
                }*/
                memcpy(buffer, buffers[buf.index].mem, buf.bytesused);
                /*ret=pthread_mutex_unlock(shm_lock);
                if (ret!=0) {
                     printf("pthread_mutex_unlock failed: %d %s\n", errno, strerror(errno));
                }*/
                //printf("lock and copy done!\n");
                *readLen = buf.bytesused;
            }

            op_ret = video_queue_buffer(buf.index);

            if (op_ret < 0) {
                printf("Unable to queue buffer (%d) (%s).count=%u [%u]\n", errno, strerror(errno), count, buf.index);
            }

            count = 30;
        }

        count++;
    }//while(count <30);
    count = 0;
    return ret;
#endif
}

/*******************for camera capture end*********/
#define FST_FMT_YUY2 "video/x-raw, format=(string)YUY2"
#define FST_FMT_NV12 "video/x-raw, format=(string)NV12"
#define FST_FMT_I420 "video/x-raw, format=(string)I420"
#define FST_FMT_MJPG "image/jpeg"

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
#define GST_CMD_TOTAL_LEN 10240
#define GST_CMD_LEN 1024

char gst_camera_cmd[GST_CMD_LEN];
char gst_convertor_cmd[GST_CMD_LEN];
char gst_scaler_cmd[GST_CMD_LEN];
char gst_encoder_cmd[GST_CMD_LEN];
char gst_output_cmd[GST_CMD_LEN];

const char *getGstCameraCmd(const char * dev_name, unsigned int format, unsigned int width, unsigned int height, unsigned int framerate)
{
    memset(gst_camera_cmd, 0, sizeof(gst_camera_cmd));
    sprintf(gst_camera_cmd, " -ve v4l2src device=%s ! \'%s, width=(int)%u, height=(int)%u, framerate=%u/1\'",
            dev_name, v4l2format2gstformat(format), width, height, framerate);
    return gst_camera_cmd;
}
const char *getGstConvertorCmd(unsigned int format, unsigned int width, unsigned int height, unsigned int framerate)
{
    memset(gst_convertor_cmd, 0, sizeof(gst_convertor_cmd));

    if (format == camera_format) {
        return "";
    }

    if (format == V4L2_PIX_FMT_MJPEG) {
        format = V4L2_PIX_FMT_YUV420;
    }

    sprintf(gst_convertor_cmd, " ! videoconvert ! \'%s, width=(int)%u, height=(int)%u, framerate=%u/1\'",
            v4l2format2gstformat(format), width, height, framerate);
    return gst_convertor_cmd;
}
const char *getGstScalerCmd(unsigned int format, unsigned int width, unsigned int height, unsigned int framerate)
{
    memset(gst_scaler_cmd, 0, sizeof(gst_scaler_cmd));

    if (width == camera_width && height == camera_height && framerate == camera_framerate) {
        return "";
    }

    sprintf(gst_scaler_cmd, " ! videoscale ! \'%s, width=(int)%u, height=(int)%u, framerate=%u/1\'",
            v4l2format2gstformat(format), width, height, framerate);
    return gst_scaler_cmd;
}

const char *getGstEncoderCmd(unsigned int format, unsigned int width, unsigned int height, unsigned int framerate)
{
    memset(gst_encoder_cmd, 0, sizeof(gst_encoder_cmd));

    if (format != V4L2_PIX_FMT_MJPEG) {
        return "";
    }

    sprintf(gst_encoder_cmd, " ! nvjpegenc ! \'%s, width=(int)%u, height=(int)%u, framerate=%u/1\'",
            v4l2format2gstformat(format), width, height, framerate);
    return gst_encoder_cmd;
}
/*const char *getGstVideoCmd(char * cm_str, const char * plugin_name, unsigned int format, unsigned int width, unsigned int height, unsigned int framerate)
{
    memset(cm_str, 0, sizeof(GST_CMD_LEN));
    sprintf(cm_str, " ! %s ! \'%s, width=(int)%u, height=(int)%u, framerate=%u/1\'",
       plugin_name, v4l2format2gstformat(format), width, height, framerate);
   return cm_str;
}*/
const char *getGstShmOutputCmd(const char * name, unsigned int size, const char * socket_path, const char * extra_param)
{
    memset(gst_output_cmd, 0, sizeof(gst_output_cmd));
    sprintf(gst_output_cmd, " ! shmsink name=%s shm-size=%u socket-path=%s %s",
            name, size, socket_path, extra_param);
    return gst_output_cmd;
}



void handleFormatChange(unsigned int format, unsigned int width, unsigned int height, unsigned int framerate)
{
    if (output_format != format || output_width != width || output_height != height) {
        printf("change output format from [%s][%u] %ux%u t0 [%s][%u] %ux%u \n"
               , getV4L2FormatStr(output_format), output_format, output_width, output_height
               , getV4L2FormatStr(format), format, width, height);
        output_format = format;
        output_width = width;
        output_height = height;
        output_framerate = framerate;
    } else
        printf("Same format, nothing to change\n");
}

void handleStartStream(unsigned int format, unsigned int width, unsigned int height)
{
    printf("start stream with: %s %u x %u\n", v4l2format2gstformat(format), width, height);
//#ifndef USE_V4L2
    /*****************USE GSTREAMER***/
    char gst_cmd[10240];
    memset(gst_cmd, 0, sizeof(gst_cmd));
    /*
    sprintf(gst_cmd, "gst-launch-1.0 --gst-debug=3 --verbose -ve v4l2src device=/dev/video1 "
        "! \'video/x-raw, width=(int)1920, height=(int)1080, format=(string)NV12, framerate=60/1\' "
        "! videoconvert "
        "! \'video/x-raw, width=(int)1920, height=(int)1080, format=(string)I420, framerate=60/1\' "
        "! nvjpegenc quality=85 "
        "! \'image/jpeg, framerate=60/1\' "
        "! shmsink name=/usb_v_in shm-size=3110400 socket-path=/tmp/mjpeg_socket wait-for-connection=false &"
        //camera_width, camera_height,
        );*/
    sprintf(gst_cmd, "gst-launch-1.0 --gst-debug=3 --verbose%s%s%s%s%s &"
            , getGstCameraCmd("/dev/video1", V4L2_PIX_FMT_NV12, camera_width, camera_height, CAM_DEF_FRAMERATE)
            , getGstScalerCmd(V4L2_PIX_FMT_NV12, output_width, output_height, output_framerate)
            , getGstConvertorCmd(output_format, output_width, output_height, output_framerate)
            , getGstEncoderCmd(output_format, output_width, output_height, output_framerate)
            , getGstShmOutputCmd("/usb_v_in", output_width * output_height * 6, "/tmp/v_in_sock", "wait-for-connection=false")
           );

    printf("run cmd: %s\n", gst_cmd);
    system(gst_cmd);
    //usleep(5000);
    struct plcm_uvc_event_msg_t event;
    event.m_event = e_stream_ready;
    event.m_format.m_height = camera_height;
    event.m_format.m_width = camera_width;
    write(app2stack, &event, sizeof(struct plcm_uvc_event_msg_t));
//#endif
}

void handleStopStream()
{
    //-2 =SIGINT, gst-launch will do auto clean up when receive SIGINT
    system("sudo killall -2 gst-launch-1.0");
}

/*******************Main and related begin*********/

inline unsigned long GetTimeInMilliSec()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr, " -d device	Video device\n");
    fprintf(stderr, " -h		Print this help screen and exit\n");
    fprintf(stderr, " -i image	MJPEG image\n");
}

#define MSG_2_STACK "message to stack"

int main(int argc, char *argv[])
{
    //char *device = "/dev/video0";
    //struct uvc_device *dev;
    //char *mjpeg_image = NULL;

    int ret, opt;
    int fd = -1;

    while ((opt = getopt(argc, argv, "d:hi:k:g:")) != -1) {
        switch (opt) {
            case 'd':
                //device = optarg;

                break;

            case 'h':
                usage(argv[0]);
                return 0;

            case 'i':
                //mjpeg_image = optarg;

                break;

            case 'k':
                camera_width = atoi(optarg);
                bytesperline = camera_width;
                break;

            case 'g':
                camera_height = atoi(optarg);
                break;

            default:
                fprintf(stderr, "Invalid option '-%c'\n", opt);
                usage(argv[0]);
                return 1;
        }
    }

    //create_scaler_thread(ROW_NUM, COLUMN_NUM);
    //return;
    /*
     * Setup the signal handler for SIGALRM. It is used
     * for the bulk transfer mode. Because in bulk mode,
     * the driver will not send the STREAM_OFF event when
     * the host stops the video stream. We need to have a
     * timer that if we can not receive the video frame
     * transfer done event in 1 second. We will stop the
     * video and clean the buffer.
     */
    imagesize = camera_width * camera_height;
    //signal(SIGALRM, sig_handle);
#ifdef USE_V4L2
    /* load camera */
    start_camera(camera_width, camera_height);
#endif
    //read_camera();
    //stop_camera();
    //return 0;
    /* load camera end */
    //struct usb_video_info_t video_info;
    //video_info.video_width=camera_width;
    //video_info.video_height=camera_height;
    //int usbDevFd = initUSBDevice();

    fd_set fds;
    FD_ZERO(&fds);
    //FD_SET(dev->fd, &fds);
    //FD_SET(fd, &fds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; //1ms
    //unsigned long getFrameTime = 0; //GetTimeInMilliSec();
    //unsigned long getNextFrameTime = 0; //GetTimeInMilliSec();

    /*****inter process communication begin*****/
#ifdef USE_V4L2
    uint8_t * pSharedMem = allocSharedMem(USB_SHM_VIDEO_IN_BUFFER, imagesize * USB_SHM_MAX_NUM);

    if (pSharedMem == NULL) {
        printf("Cannot alloc shared memory\n");
        return -1;
    }

    shm_lock = allocSharedMemMutex(USB_SHM_VIDEO_IN_MUTEX);

    if (shm_lock == NULL) {
        printf("Cannot alloc shared memory mutex\n");
        return -1;
    }

#endif

    app2stack = open(fifo_in_name, O_RDWR | O_NONBLOCK);
    stack2app = open(fifo_out_name, O_RDWR | O_NONBLOCK);
    //FD_SET(stack2app, &rfds);
    /*****inter process communication end*****/
    printf("=========visage uvc sample app init done w=%u h=%u, imagesize=%u======\n",  camera_width, camera_height, imagesize);
    //uint32_t buffer_page_index = 0;
    //uint32_t one_frame_size = imagesize;
    FD_SET(stack2app, &fds);
    int maxfd = stack2app > fd ? stack2app : fd;
    struct plcm_uvc_event_msg_t event;
    event.m_event = e_app_ready;
    event.m_format.m_height = camera_height;
    event.m_format.m_width = camera_width;
    write(app2stack, &event, sizeof(struct plcm_uvc_event_msg_t));

    while (1) {
        fd_set efds = fds;
        fd_set wfds = fds;
        fd_set rfds = fds;
        ret = select(maxfd + 1, &rfds, &wfds, &efds, &timeout);

        if (ret == -1) {
            if (errno != EINTR) {
                printf("Error in select\n");
                break;
            }
        } else {
            if (FD_ISSET(fd, &efds)) {
                printf("should not be here, efd:\n");
            }

#ifdef USE_V4L2

            if (FD_ISSET(fd, &wfds)) {
                //printf("wfd:\n");
                unsigned int readLen = 0;

                if (buffer_page_index >= USB_SHM_MAX_NUM) {
                    buffer_page_index = 0;
                }

                read_one_camera_frame(pSharedMem + one_frame_size * buffer_page_index, one_frame_size, &readLen);


                if (readLen != 0) {
                    if (getFrameTime == 0) {
                        getFrameTime = GetTimeInMilliSec();
                    } else {
                        getNextFrameTime = GetTimeInMilliSec();
                        printf("get one frame delta time=%lu, buffer_page_index=%u\n", getNextFrameTime - getFrameTime, buffer_page_index);
                        getFrameTime = getNextFrameTime;
                        /*testcode */
                        /*struct plcm_uvc_event_t event;
                        event.m_event=e_start_stream;
                        event.m_format.m_height=camera_height;
                        event.m_format.m_width=camera_width;
                        write(app2stack, &event, sizeof(struct plcm_uvc_event_t));*/
                    }

                    buffer_page_index++;
                    frame_count++;
                }

            }

#endif

            if (FD_ISSET(stack2app, &rfds)) {
                struct plcm_uvc_event_msg_t event;
                memset(&event, 0, sizeof(struct plcm_uvc_event_msg_t));
                int len = read(stack2app, &event, sizeof(struct plcm_uvc_event_msg_t));

                //printf("read finished:len=%d %d %s\n", len, errno, strerror(errno));
                if (len > 0) {
                    printf("Receive %s[%d] from stack: format=%ux%u\n", getEventDescStr(event.m_event), event.m_event, event.m_format.m_width, event.m_format.m_height);

                    switch (event.m_event) {
                        case e_stack_ready:
                            break;

                        case e_get_format:
                            break;

                        case e_set_format:
                            handleFormatChange(event.m_format.m_video_format, event.m_format.m_width, event.m_format.m_height, event.m_format.m_framerate);
                            break;

                        case e_start_stream:
                            handleStopStream();
                            handleStartStream(event.m_format.m_video_format, event.m_format.m_width, event.m_format.m_height);
                            break;

                        case e_stop_stream:
                            handleStopStream();
                            break;

                        default:
                            break;
                    }
                } else {
                    printf("read failed: %d %s\n", errno, strerror(errno));
                }
            }

        }
    }

    printf("=========visage uvc sample app done======\n");

#ifdef USE_V4L2
    //uvc_close(dev);
    stop_camera();
    freeSharedMem(USB_SHM_VIDEO_IN_BUFFER, pSharedMem, VBUF_LEN);
    freeSharedMemMutex(shm_lock, USB_SHM_VIDEO_IN_MUTEX);
#endif
    return 0;
}
/*******************Main and related end*********/


