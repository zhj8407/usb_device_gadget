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
#include <sys/socket.h>
#include <sys/un.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>
#include <time.h>
//#include <turbojpeg.h>
#include <stdio.h>

#include "uvc.h"

#include "log_str.h"
#include "scale.h"
#include "visage_shared_mem.h"
#include "plcm_usb_intf.h"
#include "dbus_utils.h"
#include "uvc-gadget-func.h"
#include "diag_utils.h"


#define WEBCAM_DEVICE_SYS_PATH      "/sys/class/plcm_usb/plcm0/f_webcam/webcam_v4l2_ctrl_device"
#define WEBCAM_DEVICE_SYS_PATH_old      "/sys/class/plcm_usb/plcm0/f_webcam/webcam_device"

#define WEBCAM_MAXPACKET_SYS_PATH   "/sys/class/plcm_usb/plcm0/f_webcam/webcam_maxpacket"
#define WEBCAM_HEADERSIZE_SYS_PATH  "/sys/class/plcm_usb/plcm0/f_webcam/webcam_headersize"
#define WEBCAM_BULKMODE_SYS_PATH    "/sys/class/plcm_usb/plcm0/f_webcam/webcam_bulkmode"
#define WEBCAM_MAXPAYLOAD_SYS_PATH  "/sys/class/plcm_usb/plcm0/f_webcam/webcam_maxpayload"
#define COLOR_COMPONENTS    3

//#define SYNC_CON_APP 1

#ifdef SYNC_CON_APP
DBusHandlerResult object_visage_handler(DBusConnection *, DBusMessage *, void *);
void object_unregister_handler(DBusConnection *, void *);
DBusObjectPathVTable objectPathVTable[] = {
    {
        .unregister_function = object_unregister_handler,
        .message_function = object_visage_handler
    }
};

#endif

char *objectPaths[] = {
    PLCM_USB_VISAGE_UVC_OBJ_PATH
};


DBusConnection *dbus_con;
int stack2app_fd;
int app2stack_fd;

//==========for GStreamer usage=======//
enum {
    COMMAND_NEW_SHM_AREA = 1,
    COMMAND_CLOSE_SHM_AREA = 2,
    COMMAND_NEW_BUFFER = 3,
    COMMAND_ACK_BUFFER = 4
};

struct CommandBuffer {
    unsigned int type;
    int area_id;

    union {
        struct {
            size_t size;
            unsigned int path_size;
            /* Followed by path */
        } new_shm_area;
        struct {
            unsigned long offset;
            unsigned long size;
        } buffer;
        struct {
            unsigned long offset;
        } ack_buffer;
    } payload;
};
//========for GStreamer usage end=====//


unsigned long jpeg_size = 0;
unsigned long jpeg_offset = 0;
//unsigned long getFrameTime = 0; //GetTimeInMilliSec();
//unsigned long getNextFrameTime = 0; //GetTimeInMilliSec();
unsigned long videoStartTime = 0;

fd_set fds;
int gst_socket = -1;
int max_fd = -1;
static int stream_on = 0;
uint8_t * pSharedMem;
char pSharedMemName[64];
unsigned int sharedMemSize = 0;
int awaiting_shm = 0;

//int app2stack;
char * fifo_in_name = "/tmp/to_usb_stack";
//int stack2app;
char * fifo_out_name = "/tmp/from_usb_stack";

static struct uvc_device *global_uvc = NULL;
#ifdef SYNC_CON_APP
DBusHandlerResult object_visage_handler(DBusConnection* conn, DBusMessage* msg, void* data)
{
    (void)data;
    int retval = 0;
    int msg_type = dbus_message_get_type(msg);

    switch (msg_type) {
        case DBUS_MESSAGE_TYPE_METHOD_CALL:
            if (strcmp(dbus_message_get_interface(msg), PLCM_USB_VISAGE_UVC_INTF_NAME)) {
                return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
            }

            if (!strcmp(dbus_message_get_member(msg), "SendResponse")) {
                int value = 0;
                int len = dbus_extract_int(msg, &value);
                printf("[object_visage_handler]: value=%d\n", value);

                if (len < 0) {
                    dbus_send_method_call_reply_with_results(conn, msg, DBUS_TYPE_UINT16, &len, DBUS_TYPE_INVALID);
                    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
                }

                struct uvc_request_data resp;

                resp.length = sizeof(int);

                memcpy(resp.data, &value, sizeof(int));

                if (global_uvc != NULL)
                    retval = ioctl(global_uvc->fd, UVCIOC_SEND_RESPONSE, &resp);

                dbus_send_method_call_reply_with_results(conn, msg, DBUS_TYPE_UINT16, &retval, DBUS_TYPE_INVALID);
            } else {
                fprintf(stdout, "object_visage_handler(call): cannot handle.\n");
                return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
            }

        default:
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }
}

void object_unregister_handler(DBusConnection* conn, void* data)
{
    (void)conn;
    (void)data;
    printf("object_unregister_handler:\n");
}

/**
 * filter messages that already stayed in the incoming queue,
 * decide whether a further process is needed.
 */
DBusHandlerResult msg_filter(DBusConnection *conn,
                             DBusMessage *msg, void *data)
{
    (void)conn;
    (void)data;
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    printf("incoming msg: %s\n", _verbose_message(msg));

    switch (dbus_message_get_type(msg)) {
        case DBUS_MESSAGE_TYPE_METHOD_CALL:
            if (!strcmp(dbus_message_get_member(msg), "ignore")) {
                DBusMessage *errMsg;
                errMsg = dbus_message_new_error(msg,
                                                "com.redflag.csy.IgnoreService",
                                                "this demonstrate the filter.");
                dbus_connection_send(conn, errMsg, NULL);
                return DBUS_HANDLER_RESULT_HANDLED;
            } else
                break;

        case DBUS_MESSAGE_TYPE_METHOD_RETURN:
            // never reach here.

            break;

        case DBUS_MESSAGE_TYPE_SIGNAL:
            printf("Signal Received!!! Interface: %s, Member: %s\n",
                   dbus_message_get_interface(msg),
                   dbus_message_get_member(msg));
            break;

        case DBUS_MESSAGE_TYPE_ERROR:
            break;
    }

    // set this flag is very important, if not, dbus may not

    // process messages for you. it pass the control to dbus

    // default filter.

    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}
#endif

int g_gst_connect_flag = 0;

void close_gst_socket(int *fd)
{
    //printf("[close_gst_socket] fd=%d, g_gst_connect_flag=%d\n", *fd, g_gst_connect_flag);
    if (g_gst_connect_flag > 0)
        FD_CLR(*fd, &fds);

    if (*fd >= 0) {
        //printf("close socket %d\n", main_socket);
        close(*fd);
        *fd = -1;
    }

    g_gst_connect_flag = 0;
}

int connect_socket_block(const char * so_path)
{
    int connectRetry = -1;
    int ret = 0;
    struct sockaddr_un sock_un;
    sock_un.sun_family = AF_UNIX;
    strncpy(sock_un.sun_path, so_path, sizeof(sock_un.sun_path) - 1);

    int socket_fd = socket(PF_UNIX, SOCK_STREAM, 0);

    if (socket_fd < 0) {
        printf("open socket failed %d %s\n", errno, strerror(errno));
        //close_gst_socket(&socket_fd);
        return -1;
    }

    while (connectRetry <= 256 && g_gst_connect_flag == 0) {
        ret = connect(socket_fd, (struct sockaddr *) &sock_un, sizeof(struct sockaddr_un));

        if (ret < 0) {
            //printf("[%u]connect socket [%s] failed %d %s\n", connectRetry, sock_un.sun_path, errno, strerror(errno));
            //usleep(1000);
            //unlink (sock_un.sun_path);
            //free (sock_un.sun_path);
            connectRetry++;
            snprintf(sock_un.sun_path, sizeof(sock_un.sun_path), "%s.%d", so_path, connectRetry);
        } else { //connected
            g_gst_connect_flag = 1;
            printf("connect socket [%s] success!\n", sock_un.sun_path);
            break;
            //return main_socket;
        }
    }

    if (g_gst_connect_flag == 1) {
        char tempName[64];
        memset(tempName, 0, sizeof(tempName));

        //int ret=0;
        if (connectRetry != -1) {
            ret = unlink(so_path);
            printf("unlink %s gets ret= %d %d %s\n", so_path, ret, errno, strerror(errno));
        }

        for (; connectRetry > 0; connectRetry--) {
            snprintf(tempName, sizeof(tempName), "%s.%d", so_path, connectRetry - 1);
            ret = unlink(tempName);
            printf("unlink %s gets ret= %d %d %s\n", so_path, ret, errno, strerror(errno));
        }

        return socket_fd;
    } else {
        close_gst_socket(&socket_fd);
        return -1;
    }
}



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
    uvc_read_value_from_file(WEBCAM_DEVICE_SYS_PATH,
                             "%d\n",
                             &dev->v4ldevnum);

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

static struct uvc_device *uvc_open(const char *devname)
{
    struct uvc_device *dev;
    struct v4l2_capability cap;
    char v4ldevname[64];
    int ret = -1;
    int fd = 0;

    dev = malloc(sizeof * dev);

    if (dev == NULL) {
        close(fd);
        return NULL;
    }

    memset(dev, 0, sizeof * dev);
    dev->v4ldevnum = -1;
    dev->maxpacketsize = 1024;
    dev->headersize = 2;
    dev->bulk = 0;

    uvc_video_init(dev);

    if (dev->v4ldevnum != -1) {
        snprintf(v4ldevname, sizeof(v4ldevname), "/dev/video%d", dev->v4ldevnum);
    } else {
        snprintf(v4ldevname, sizeof(v4ldevname), "%s", devname);
    }

    printf("We are trying to open the dev: %s\n", v4ldevname);

    fd = open(v4ldevname, O_RDWR | O_NONBLOCK);

    if (fd == -1) {
        printf("v4l2 open failed: %s (%d)\n", strerror(errno), errno);
        return NULL;
    }

    printf("open succeeded, file descriptor = %d\n", fd);
    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);

    if (ret < 0) {
        printf("unable to query device: %s (%d)\n", strerror(errno),
               errno);
        close(fd);
        return NULL;
    }

    printf("device is %s on bus %s\n", cap.card, cap.bus_info);

    printf("The config values are as below\n");
    printf("\t\tv4ldevnum: %d\n", dev->v4ldevnum);
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
    free(dev->imgdata);
    free(dev->mem);
    free(dev);
}

/********read one frame from shared memory *****/

static uint32_t frame_count = 0;
#define CAM_MAX_WIDTH 1920
#define CAM_MAX_HEIGHT 1080
#define CAM_DEF_WIDTH 1920
#define CAM_DEF_HEIGHT 1080
unsigned int camera_width = CAM_DEF_WIDTH;
unsigned int camera_height = CAM_DEF_HEIGHT;
unsigned int frame_size = CAM_DEF_WIDTH * CAM_DEF_HEIGHT * 2;
#define VBUF_LEN (CAM_MAX_WIDTH*CAM_MAX_HEIGHT*2)
uint8_t vBuf[VBUF_LEN];

/*
* Callbacks for uvc v4l2 device management
*
*/
void on_stream_on()
{
    int ret = 0;
    stream_on = 1;
    ret = sendEvent2Fifo(stack2app_fd, e_start_stream, global_uvc->fcc, global_uvc->width, global_uvc->height);
    notifyApplication(dbus_con, e_start_stream, global_uvc->fcc, global_uvc->width, global_uvc->height);
    videoStartTime = GetTimeInMilliSec();

    if (ret < 0)
        printf("[on_stream_on]send %s %ux%u to app failed %d %s\n", getEventDescStr(e_start_stream), global_uvc->width, global_uvc->height, errno, strerror(errno));

    uvc_video_reqbufs(global_uvc, 2);
    uvc_video_stream(global_uvc, 1);
    alarm(3);
    printf("[on_stream_on]Starting video stream with %s %ux%u\n", getV4L2FormatStr(global_uvc->fcc), global_uvc->width, global_uvc->height);
}
void on_stream_off()
{
    int ret = 0;
    frame_count = 0;
    stream_on = 0;

    alarm(0);
    uvc_video_stream(global_uvc, 0);
    uvc_video_reqbufs(global_uvc, 0);

    close_gst_socket(&gst_socket);
    max_fd = global_uvc->fd > app2stack_fd ? global_uvc->fd : app2stack_fd;
    ret = sendEvent2Fifo(stack2app_fd, e_stop_stream, global_uvc->fcc, global_uvc->width, global_uvc->height);
    notifyApplication(dbus_con, e_stop_stream, global_uvc->fcc, global_uvc->width, global_uvc->height);

    if (ret < 0)
        printf("send %s %ux%u to app failed %d %s\n", getEventDescStr(e_stop_stream), global_uvc->width, global_uvc->height, errno, strerror(errno));

    if (pSharedMem != NULL) {
        freeSharedMem(pSharedMemName, pSharedMem, sharedMemSize);
        pSharedMem = NULL;
    }
}
void on_get_cam_param(uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length)
{
    if (req != UVC_SET_CUR) {
        send_get_property_signal(dbus_con, req, cs, unit_id, length);
    } else {
        global_uvc->control = cs;
        global_uvc->unit = unit_id;
    }
}
void on_set_cam_param(uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length, uint8_t* data)
{
    (void)req;
    (void)cs;
    (void)unit_id;
    (void)length;
    struct uvc_request_data * param = (struct uvc_request_data *)data;
    send_set_property_signal(dbus_con, UVC_SET_CUR, global_uvc->control, global_uvc->unit, param->length, param->data);
}

void on_get_video_param()
{
}
void on_set_video_param(uint32_t format, uint32_t width, uint32_t height, uint32_t framerate)
{
    (void)framerate;
    sendEvent2Fifo(stack2app_fd, e_set_format, format, width, height);
}


int on_req_frame(uint8_t * buffer, uint32_t buffer_max_len, uint32_t *buffer_act_len)
{
    if (buffer_max_len == 0) {
        printf("[read_one_camera_frame]: bufferlen=0\n");
        return -1;
    }

    if (jpeg_size <= buffer_max_len && pSharedMem) {
        memcpy(buffer, pSharedMem + jpeg_offset, jpeg_size);
        *buffer_act_len = jpeg_size;
    } else {
        *buffer_act_len = 0;
    }

    jpeg_size = 0;
    jpeg_offset = 0;

    return 0;
}

void on_frame_done()
{
    alarm(1);
}
/* ---------------------------------------------------------------------------
 * util
 */

void handle_app_event(struct uvc_device * dev)
{
    int ret = 0;
    char buf[256];
    memset(buf, 0, sizeof(buf));
    int len = read(app2stack_fd, buf, sizeof(buf));

    if (len > 0) {
        struct plcm_uvc_event_msg_t *event = (struct plcm_uvc_event_msg_t*)(buf);

        if (event != NULL) {
            //printf("a message from app 2 stack: event=%u[%s], format=[%ux%u] streamon=%d\n"
            //    , event->m_event, plcm_usb_video_event_str[event->m_event]
            //    , event->m_format.m_width, event->m_format.m_height
            //    , stream_on);
        } else {
            printf("a message from app 2 stack: wrong format!\n");
        }

        if (event->m_event == e_app_ready) {
            sendEvent2Fifo(stack2app_fd, e_stack_ready, dev->fcc, dev->width, dev->height);
        }

        if (event->m_event == e_stream_ready) {
            //printf("Receive event: stream ready\n");
            gst_socket = connect_socket_block("/tmp/v_in_sock");

            if (gst_socket > 0) {
                FD_SET(gst_socket, &fds);
            } else {
                if (stream_on) {
                    ret = sendEvent2Fifo(stack2app_fd, e_retry_socket, dev->fcc, dev->width, dev->height);

                    if (ret < 0) {
                        printf("send %s %ux%u to app failed %d %s\n", getEventDescStr(e_retry_socket), dev->width, dev->height, errno, strerror(errno));
                    }
                }
            }
        }
    } else {
        printf("failed to read from app socket, %u %s!\n", errno, strerror(errno));
    }
}

void handle_gstreamer_event(struct uvc_device * dev, struct CommandBuffer* response_cb)
{
    int ret = 0;
    (void)dev;
    struct CommandBuffer recvCB;
    memset(&recvCB, 0, sizeof(struct CommandBuffer));

    if (awaiting_shm == 0) {
        ret = recv(gst_socket, &recvCB, sizeof(recvCB), MSG_DONTWAIT);
    } else {
        ret = recv(gst_socket, pSharedMemName, sizeof(pSharedMemName), MSG_DONTWAIT);
    }

    if (ret > 0) {
        if (recvCB.type == COMMAND_NEW_SHM_AREA) {
            printf("receive shm cmd: new shm area id=%d\n", recvCB.area_id);
            awaiting_shm = 1;
            sharedMemSize = recvCB.payload.new_shm_area.size;
        } else if (recvCB.type == COMMAND_CLOSE_SHM_AREA) {
            printf("closing shared memory\n");
            awaiting_shm = 0;
        } else if (recvCB.type == COMMAND_NEW_BUFFER) {
            jpeg_size = recvCB.payload.buffer.size;
            jpeg_offset = recvCB.payload.buffer.offset;
            response_cb->type = COMMAND_ACK_BUFFER;
            response_cb->area_id = recvCB.area_id;
            response_cb->payload.ack_buffer.offset = recvCB.payload.buffer.offset;

            /*if (frame_count % print_interval == 0) {
                if (getFrameTime == 0) {
                    getFrameTime = GetTimeInMilliSec();
                }

                getNextFrameTime = GetTimeInMilliSec();
                printf("Sending %d frame(total: %u) of [%s] %ux%u delta time=%lu ms\n",
                       print_interval, frame_count, getV4L2FormatStr(dev->fcc), dev->width, dev->height, getNextFrameTime - getFrameTime);
                getFrameTime = getNextFrameTime;
            }*/

        } else {
            if (awaiting_shm > 0) {
                printf("Receive SHM param path=%s ret=%d\n", pSharedMemName, ret);
                pSharedMem = allocSharedMem(pSharedMemName, sharedMemSize);

                if (pSharedMem == NULL) {
                    printf("Cannot alloc shared memory\n");
                    return;
                }

                printf("=========share memory init done %p, name=%s======\n", pSharedMem, pSharedMemName);
                awaiting_shm = 0;
            } else
                printf("Receive a cmd from gst ret[%d]: msg.type=%u areaid=%u\n",
                       ret, recvCB.type, recvCB.area_id);
        }
    }
}

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr, " -h		Print this help screen and exit\n");
}

void sig_handle(int sig)
{
    printf("Received signal: %d(%s)\n", sig, strsignal(sig));

    // Alarm timeout. Stop the video
    if (sig != SIGALRM && sig != SIGINT)
        return;

    if (global_uvc) {
        uvc_video_stream(global_uvc, 0);
        uvc_video_reqbufs(global_uvc, 0);
    }

    if (sig == SIGINT) {
        printf("Close GST socket before exit\n");
        close_gst_socket(&gst_socket);
        exit(0);
    }
}

int main(int argc, char *argv[])
{
    char *device = "/dev/video0";
    struct uvc_device *dev;
    int ret, opt;

    while ((opt = getopt(argc, argv, "d:hi:k:g:")) != -1) {
        switch (opt) {
            case 'd':
                device = optarg;
                break;

            case 'h':
                usage(argv[0]);
                return 0;

            case 'k':
                camera_width = atoi(optarg);
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

    frame_size = camera_width * camera_height * 3 / 2;
    printf("=========start with width=%u height=%u======\n", camera_width, camera_height);

    /*****image processing init begin*****/
    create_scaler_thread(ROW_NUM, COLUMN_NUM);
    printf("=========scaler init done======\n");
    /*****image processing init end *****/

    /*****uvc device init begin*****/
    dev = uvc_open(device);

    if (dev == NULL)
        return 1;

    global_uvc = dev;
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
    signal(SIGINT, sig_handle);

    //image_load(dev, mjpeg_image);
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
    printf("=========uvc init done======\n");
    /*****uvc device init end*****/

    /*****inter process communication init*****/


    /*************control pipe from/to app*********/
    ret = mkfifo(fifo_in_name, 0666);

    if (ret == -1) {
        printf("mkfifo[%s]: %d %s\n", fifo_in_name, errno, strerror(errno));
    }

    //printf("=========fifo init 1======\n");
    ret = mkfifo(fifo_out_name, 0666);

    if (ret == -1) {
        printf("mkfifo[%s]: %d %s\n", fifo_out_name, errno, strerror(errno));
    }

    //printf("=========fifo init 2======\n");
    app2stack_fd = open(fifo_in_name, O_RDONLY | O_NONBLOCK);
    //printf("=========fifo init 3======\n");
    stack2app_fd = open(fifo_out_name, O_RDWR | O_NONBLOCK);

    if (app2stack_fd == 0 || stack2app_fd == 0) {
        printf("open fifo file faiedl app2stack[%d], stack2app[%d]\n", app2stack_fd, stack2app_fd);
    }


    printf("=========fifo init done======\n");

    //setup dbus connection

    ret = dbus_setup_connection(&(dbus_con), "com.polycom.visage.uvc",
                                "type='signal',interface='test.signal.Type'");

    if (ret) {
        fprintf(stderr, "No dbus connection. Exit: (%d)\n", ret);

    } else printf("dbus connection setup successfully\n");

#ifdef SYNC_CON_APP
    dbus_register_message_filter(dbus_con, msg_filter);

    dbus_register_object_patch(dbus_con,
                               objectPaths, objectPathVTable,
                               sizeof(objectPaths) / sizeof(objectPaths[0]));

    if (ret) {
        fprintf(stderr, "Debus register failed. Exit: (%d)\n", ret);

    } else printf("dbus register successfully\n");

#endif
    printf("dbus init done\n");

    /*****inter process communication end *****/
    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);
    FD_SET(app2stack_fd, &fds);

    max_fd = dev->fd > app2stack_fd ? dev->fd : app2stack_fd;
    max_fd = max_fd > gst_socket ? max_fd : gst_socket;

    printf("=========stack shim init done======\n");
    struct timeval gst_stream_int;
    gst_stream_int.tv_sec = 2;
    gst_stream_int.tv_usec = 0;


    while (1) {
        fd_set efds = fds;
        fd_set wfds = fds;
        fd_set rfds = fds;
        int dbus_fd = dbus_setup_listen_fds(&rfds, &wfds, &efds);
        max_fd = max_fd > dbus_fd ? max_fd : dbus_fd;
        max_fd = max_fd > gst_socket ? max_fd : gst_socket;
        //ret = select(max_fd + 1, &rfds, &wfds, &efds, NULL);
        ret = select(max_fd + 1, &rfds, NULL, &efds, &gst_stream_int);

        if (ret == -1) {
            if (errno != EINTR) {
                printf("Error in select : %u %s\n", errno, strerror(errno));
                break;
            }
        } else if (ret == 0) {//time out
            /*if (stream_on == 1 && frame_count < 2) { //after stream on, 2 sec has passed but no video stream
                printf("no video input in %lu msec\n", GetTimeInMilliSec() - videoStartTime);
            }*/

            gst_stream_int.tv_sec = 2;
            gst_stream_int.tv_usec = 0;
        } else {
            if (FD_ISSET(dev->fd, &efds)) {

                uvc_events_process(dev);

            } else if (FD_ISSET(app2stack_fd, &rfds)) {

                handle_app_event(dev);

            } else if (FD_ISSET(gst_socket, &rfds)) {
                struct CommandBuffer resp_cb;
                memset(&resp_cb, 0, sizeof(struct CommandBuffer));
                handle_gstreamer_event(dev, &resp_cb);

                //printf("resp_cb.type=%d\n", resp_cb.type);
                if (COMMAND_ACK_BUFFER == resp_cb.type) {
                    uvc_video_process(dev);
                    ret = send(gst_socket, &resp_cb, sizeof(struct CommandBuffer), MSG_NOSIGNAL);

                    if (ret == -1) {
                        printf("send ACK[size=%u] to socket failed %d %s\n", sizeof(struct CommandBuffer), errno, strerror(errno));
                    }
                }

            } else {
#ifdef SYNC_CON_APP
                //printf("Select was woken up by dbus message\n");
                dbus_handle_listen_fds(&rfds, &wfds, &efds);

                dbus_handle_all_watches(dev->dbus_con);

                dbus_handle_all_timeout();
#endif
            }
        }
    }

    uvc_close(dev);
    freeSharedMem(pSharedMemName, pSharedMem, sharedMemSize);
    //freeSharedMemMutex(shm_lock, USB_SHM_VIDEO_IN_MUTEX);
    return 0;
}
