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

/*************LOG str helper begin****/
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
/*************LOG str helper end****/

#define CAM_DEF_WIDTH 1920
#define CAM_DEF_HEIGHT 1080
#define CAM_MAX_WIDTH 1920
#define CAM_MAX_HEIGHT 1080

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
unsigned int encoder_quality = 85;


#define VISAGE_CAM_DEV "/dev/video2"
char * camera_device = VISAGE_CAM_DEV;
int app2stack;
char * fifo_in_name = "/tmp/to_usb_stack";
int stack2app;
char * fifo_out_name = "/tmp/from_usb_stack";



static const char FST_FMT_YUY2[] = "video/x-raw, format=(string)YUY2";
static const char FST_FMT_NV12[] =  "video/x-raw, format=(string)NV12";
static const char FST_FMT_I420[] =  "video/x-raw, format=(string)I420";
static const char FST_FMT_MJPG[] =  "image/jpeg";

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

const char *getGstEncoderCmd(unsigned int format, unsigned int width, unsigned int height, unsigned int framerate, unsigned int quality)
{
    memset(gst_encoder_cmd, 0, sizeof(gst_encoder_cmd));

    if (format != V4L2_PIX_FMT_MJPEG) {
        return "";
    }

    sprintf(gst_encoder_cmd, " ! nvjpegenc quality=%u ! \'%s, width=(int)%u, height=(int)%u, framerate=%u/1\'",
            quality, v4l2format2gstformat(format), width, height, framerate);
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
            , getGstCameraCmd(camera_device, V4L2_PIX_FMT_NV12, camera_width, camera_height, CAM_DEF_FRAMERATE)
            , getGstScalerCmd(V4L2_PIX_FMT_NV12, output_width, output_height, output_framerate)
            , getGstConvertorCmd(output_format, output_width, output_height, output_framerate)
            , getGstEncoderCmd(output_format, output_width, output_height, output_framerate, encoder_quality)
            , getGstShmOutputCmd("/usb_v_in", output_width * output_height * 6, "/tmp/v_in_sock", "wait-for-connection=false")
           );

    printf("run gstreamer cmd: %s\n", gst_cmd);
    system(gst_cmd);
    //usleep(5000);
    struct plcm_uvc_event_msg_t event;
    event.m_event = e_stream_ready;
    event.m_format.m_height = camera_height;
    event.m_format.m_width = camera_width;
    write(app2stack, &event, sizeof(struct plcm_uvc_event_msg_t));
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

int main(int argc, char *argv[])
{
    int ret, opt;
    int fd = -1;

    while ((opt = getopt(argc, argv, "d:hi:k:g:q:")) != -1) {
        switch (opt) {
            case 'd':
                camera_device = optarg;
                break;

            case 'h':
                usage(argv[0]);
                return 0;

            case 'i':

                break;

            case 'k':
                camera_width = atoi(optarg);
                bytesperline = camera_width;
                break;

            case 'g':
                camera_height = atoi(optarg);
                break;

			case 'q':
				encoder_quality = atoi(optarg);
				break;

            default:
                fprintf(stderr, "Invalid option '-%c'\n", opt);
                usage(argv[0]);
                return 1;
        }
    }
    imagesize = camera_width * camera_height;
    fd_set fds;
    FD_ZERO(&fds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; //1ms

    /*****inter process communication begin*****/

    app2stack = open(fifo_in_name, O_RDWR | O_NONBLOCK);
    stack2app = open(fifo_out_name, O_RDWR | O_NONBLOCK);

    FD_SET(stack2app, &fds);
    int maxfd = stack2app > fd ? stack2app : fd;
    struct plcm_uvc_event_msg_t event;
    event.m_event = e_app_ready;
    event.m_format.m_height = camera_height;
    event.m_format.m_width = camera_width;
    write(app2stack, &event, sizeof(struct plcm_uvc_event_msg_t));
    /*****inter process communication end*****/
    printf("=========visage camera-usb sample app init done w=%u h=%u, imagesize=%u======\n",  camera_width, camera_height, imagesize);

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
            if (FD_ISSET(stack2app, &rfds)) {
                struct plcm_uvc_event_msg_t event;
                memset(&event, 0, sizeof(struct plcm_uvc_event_msg_t));
                int len = read(stack2app, &event, sizeof(struct plcm_uvc_event_msg_t));
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
                            handleFormatChange(event.m_format.m_video_format, event.m_format.m_width, event.m_format.m_height, event.m_format.m_framerate);
                            handleStartStream(event.m_format.m_video_format, event.m_format.m_width, event.m_format.m_height);
                            break;

                        case e_stop_stream:
                            handleStopStream();
                            break;

                        default:
                            break;
                    }
                }
				else {
                    printf("read failed: %d %s\n", errno, strerror(errno));
                }
            }
        }
    }

    printf("=========visage uvc sample app done======\n");
    return 0;
}
/*******************Main and related end*********/


