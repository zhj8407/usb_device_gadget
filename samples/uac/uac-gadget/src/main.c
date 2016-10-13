#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "dbus_utils.h"
#include "udev_utils.h"

#include "libudev.h"
#include "f_uac_plcm.h"
#include "audiolinxtypes.h"
#define DEBUG 1
#define PLCM_USB_VISAGE_UDM_STATE_SYSFS     "/sys/class/plcm_usb/plcm0/f_audio_dual/audio_in_state"
//#define PLCM_USB_VISAGE_UDM_FUNCTIONS_SYSFS "/sys/class/plcm_usb/plcm0/functions"
//#define PLCM_USB_VISAGE_UDM_ENABLE_SYSFS    "/sys/class/plcm_usb/plcm0/enable"

#define PLCM_USB_VISAGE_UAC_OBJ_PATH        "/com/polycom/visage/uac"
#define PLCM_USB_VISAGE_UAC_INTF_NAME       "com.polycom.visage.uac"

#define PLCM_USB_VISAGE_UAC_STATE_SIGNAL    "uac_usb_state"
#define PLCM_USB_VISAGE_UAC_DATA_SIGNAL    "uac_usb_data"

#define MIC_FEATURE_UNIT_ID       0x5
#define USBAUDIO_bControlSelector_FU_MUTE   0x01
#define USBAUDIO_bControlSelector_FU_VOLUME 0x02

#define USB_bRequest_Audio_UNDEFINED  0x00  /* undefined */
#define USB_bRequest_Audio_SET_CUR  0x01    /* set current */
#define USB_bRequest_Audio_GET_CUR  0x81    /* get current */
#define USB_bRequest_Audio_GET_MIN  0x82    /* get minimum */
#define USB_bRequest_Audio_GET_MAX  0x83    /* get maximum */
#define USB_bRequest_Audio_GET_RES  0x84    /* get resolution */

#define USB_UAC_STATE_STR_LEN       32
//#define ENABLE_MUTE_VOLUME_DBUS
typedef struct USBAudioFUControl {
    unsigned short                  bMute;
    unsigned short                  wMinVol;
    unsigned short                  wMaxVol;
    unsigned short                  wResVol;
    unsigned short                  wCurVol;
} USBAudioFUControl, *USBAudioFUControlPtr;

struct uac_dbus_data_event {
    unsigned char bRequest;
    unsigned char bUnitId;
    unsigned char bControlSelector;//mute:0x01, volume: 0x02
    unsigned char reserved[1];
    unsigned short wValue;
};

struct usb_uac_device {
    DBusConnection *conn;
    struct udev *udev;
    struct udev_monitor *mon;
    int uac_ctrl_fd;

    char usb_state[USB_UAC_STATE_STR_LEN];
    struct uac_dbus_data_event event;
};
int UacDevState;

#ifdef ENABLE_MUTE_VOLUME_DBUS
int e_uac_ctrl_fd;
#endif
DBusHandlerResult object_uac_msg_handler(DBusConnection *, DBusMessage *, void *);
void object_unregister_uac_msg_handler(DBusConnection *, void *);

char *objectPaths[] = {
    PLCM_USB_VISAGE_UAC_OBJ_PATH
};

DBusObjectPathVTable objectPathVTable[] = {
    {
        .unregister_function = object_unregister_uac_msg_handler,
        .message_function = object_uac_msg_handler
    }
};
DBusHandlerResult object_uac_msg_handler(DBusConnection* conn,
        DBusMessage* msg, void* data)
{
    (void)data;

    switch (dbus_message_get_type(msg)) {
        case DBUS_MESSAGE_TYPE_SIGNAL:
        case DBUS_MESSAGE_TYPE_ERROR:
            fprintf(stdout, "object_uac_msg_handler(error/signal):\n\t%s\n",
                    _verbose_message(msg));
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

        case DBUS_MESSAGE_TYPE_METHOD_RETURN:
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

        case DBUS_MESSAGE_TYPE_METHOD_CALL:
            printf("object_uac_msg_handler: method call\n");

            if (strcmp(dbus_message_get_interface(msg), PLCM_USB_VISAGE_UAC_INTF_NAME))
                return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

#ifdef ENABLE_MUTE_VOLUME_DBUS

            if (!strcmp(dbus_message_get_member(msg), "GetUacValue")) {
                unsigned short CurVal = 0;
                int32_t retVal = 0;
                struct uac_feature_unit_value fu_value;

                dbus_read_params_from_msg(msg, DBUS_TYPE_UINT16, &CurVal, DBUS_TYPE_INVALID);
                memset(fu_value.data, 0, sizeof(fu_value.data));
                fu_value.length = 0x2;
                fu_value.data[0] = CurVal & 0xff;
                fu_value.data[1] = (CurVal >> 8) & 0xff;
                retVal = ioctl(e_uac_ctrl_fd, UAC_IOC_SEND_FU_VALUE, &fu_value);
                printf("retVal =0x%x, value=0x%x\n", retVal, fu_value.data[0]);

                dbus_send_method_call_reply_with_results(conn, msg,
                        DBUS_TYPE_INT32, &retVal,
                        DBUS_TYPE_INVALID);

                return DBUS_HANDLER_RESULT_HANDLED;
            }

#endif
            else {
                fprintf(stdout, "conn=0x%xobject_visage_handler(call): cannot handle.\n", (int)conn);
                return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
            }

        default:
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }
}

void object_unregister_uac_msg_handler(DBusConnection* conn, void* data)
{
    (void)conn;
    (void)data;
    printf("object_unregister_uac_msg_handler:\n");
}

/**----------- all handlers -------------------------------------*/

/**
 * filter messages that already stayed in the incoming queue,
 * decide whether a further process is needed.
 */
DBusHandlerResult msg_filter(DBusConnection *conn,
                             DBusMessage *msg, void *data)
{
    (void)conn;
    (void)data;
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

int notify_usb_state(struct usb_uac_device *usb_dev)
{
    const char *tmp = usb_dev->usb_state;
    printf("notify_usb_state\n");
    return dbus_send_signal_with_params(usb_dev->conn, PLCM_USB_VISAGE_UAC_OBJ_PATH,
                                        PLCM_USB_VISAGE_UAC_INTF_NAME,
                                        PLCM_USB_VISAGE_UAC_STATE_SIGNAL,
                                        DBUS_TYPE_STRING, &tmp,
                                        DBUS_TYPE_INVALID);
}

int notify_usb_mute_volume_ctl(struct usb_uac_device *usb_dev)
{
    printf("notify_usb_mute_ctl value=0x%x\n", usb_dev->event.wValue);

    if (usb_dev->event.bUnitId != MIC_FEATURE_UNIT_ID) {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    return dbus_send_signal_with_params(usb_dev->conn, PLCM_USB_VISAGE_UAC_OBJ_PATH,
                                        PLCM_USB_VISAGE_UAC_INTF_NAME,
                                        PLCM_USB_VISAGE_UAC_DATA_SIGNAL,
                                        DBUS_TYPE_BYTE, &usb_dev->event.bRequest,
                                        DBUS_TYPE_BYTE, &usb_dev->event.bUnitId,
                                        DBUS_TYPE_BYTE, &usb_dev->event.bControlSelector,
                                        DBUS_TYPE_UINT16, &usb_dev->event.wValue,
                                        DBUS_TYPE_INVALID);
}
void set_usb_ctrl_value(struct usb_uac_device *usb_dev, struct uac_ctrl_event *ctr_event)
{
    usb_dev->event.bControlSelector = ctr_event->u.fu.control_selector;
    usb_dev->event.bRequest = ctr_event->request;
    usb_dev->event.bUnitId = ctr_event->u.fu.unit_id;
    usb_dev->event.wValue = ctr_event->u.fu.value;
}
void process_event(struct usb_uac_device * usb_dev)
{
    int ret;
    fd_set rfds, wfds, efds;
    int max_fd = -1, udev_mon_fd;
#ifdef ENABLE_MUTE_VOLUME_DBUS
    int uac_ctrl_fd;
    uac_ctrl_fd = open("/dev/uac_plcm_control", O_RDWR | O_NONBLOCK);

    if (uac_ctrl_fd < 0) {
        printf("open uac_ctrl_fd failed\n");
        fprintf(stderr, "Failed to open the audio control file, error : %d(%s)\n",
                errno, strerror(errno));
        exit(-1);
    }

    usb_dev->uac_ctrl_fd = uac_ctrl_fd;
    e_uac_ctrl_fd = uac_ctrl_fd;
#endif

    while (1) {
        // Prepare for the select
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_ZERO(&efds);
        max_fd = 0;

        // Add dbus watcher fds
        max_fd = dbus_setup_listen_fds(&rfds, &wfds, &efds);

        // Add udev fd
        udev_mon_fd = udev_monitor_get_fd(usb_dev->mon);
        FD_SET(udev_mon_fd, &rfds);
        max_fd = udev_mon_fd > max_fd ? udev_mon_fd : max_fd;
#ifdef ENABLE_MUTE_VOLUME_DBUS
        //Add uac ctrl fd
        FD_SET(uac_ctrl_fd, &rfds);
        max_fd = uac_ctrl_fd > max_fd ? uac_ctrl_fd : max_fd;
#endif
        ret = select(max_fd + 1, &rfds, &wfds, &efds, NULL);

        if (ret <= 0)
            break;

        if (FD_ISSET(udev_mon_fd, &rfds)) {
            ret = handle_usb_device_uevents(usb_dev->mon,
                                            usb_dev->usb_state, sizeof(usb_dev->usb_state));

            if (ret > 0) {
                notify_usb_state(usb_dev);

                if (!strcmp(usb_dev->usb_state, "ACTIVE")) {
                    UacDevState = UAC_DEVICE_ACTIVE;
                } else if (!strcmp(usb_dev->usb_state, "DEACTIVE")) {
                    UacDevState = UAC_DEVICE_DEACTIVE;
                } else if (!strcmp(usb_dev->usb_state, "SUSPEND")) {
                    UacDevState = UAC_DEVICE_SUSPEND;
                }
            } else {
                continue;
            }
        }

#ifdef ENABLE_MUTE_VOLUME_DBUS
        else if (FD_ISSET(uac_ctrl_fd, &rfds)) {
            struct uac_ctrl_event event;
            ret = ioctl(uac_ctrl_fd, UAC_IOC_DQEVENT, &event);

            if (ret) {
                printf("Failed to Dequeue event, ret = %d\n", ret);
                continue;
            }

            set_usb_ctrl_value(usb_dev, &event);
#if DEBUG

            printf("SERVER Dequeued the event\n");
            printf("\t\tEvent.request:         0x%02x\n", usb_dev->event.bRequest);
            printf("\t\tEvent.unit_id:         0x%02x\n", usb_dev->event.bUnitId);
            printf("\t\tEvent.contro_selector: 0x%02x\n", usb_dev->event.bControlSelector);
            printf("\t\tEvent.value:           0x%04x\n", usb_dev->event.wValue);
#endif
            ret = notify_usb_mute_volume_ctl(usb_dev);

            if (ret != 0) {
                printf("notify_usb_mute_volume_ctl failed\n ret=0x%x\n", ret);
            }
        }

#endif
        else {
            //printf("Select was woken up by dbus message\n");
            dbus_handle_listen_fds(&rfds, &wfds, &efds);
            dbus_handle_all_watches(usb_dev->conn);
            dbus_handle_all_timeout();
        }
    }
}
int main(void)
{
    struct usb_uac_device *usb_dev = NULL;
    int ret;
    UacDevState = UAC_DEVICE_STATE_MAX;

    struct linxed_struct args;
    usb_dev = (struct usb_uac_device *)malloc(sizeof(*usb_dev));

    if (!usb_dev) {
        fprintf(stderr, "Failed to allocate the usb_dev, error: %d(%s)\n",
                errno, strerror(errno));
        ret = -1;
        goto exit;
    }

    memset(usb_dev, 0, sizeof(*usb_dev));

    ret = read_value_from_file(PLCM_USB_VISAGE_UDM_STATE_SYSFS, "%s\n", usb_dev->usb_state);

    if (!strcmp(usb_dev->usb_state, "AUDIO_SOURCE_STATE=ACTIVE")) {
        UacDevState = UAC_DEVICE_ACTIVE;
    } else if (!strcmp(usb_dev->usb_state, "AUDIO_SOURCE_STATE=DEACTIVE")) {
        UacDevState = UAC_DEVICE_DEACTIVE;
    } else if (!strcmp(usb_dev->usb_state, "AUDIO_SOURCE_STATE=SUSPEND")) {
        UacDevState = UAC_DEVICE_SUSPEND;
    }

    printf("Get current usb uac state: %s\n", usb_dev->usb_state);

    ret = dbus_setup_connection(&usb_dev->conn, "com.polycom.visage.uac",
                                "type='signal',interface='test.signal.Type'");

    if (ret) {
        fprintf(stderr, "No dbus connection. Exit: (%d)\n", ret);
        goto exit;
    }

    //dbus_register_message_filter(usb_dev->conn, msg_filter);

    dbus_register_object_patch(usb_dev->conn,
                               objectPaths, objectPathVTable,
                               sizeof(objectPaths) / sizeof(objectPaths[0]));

    ret = setup_udev(&usb_dev->udev, &usb_dev->mon);

    if (ret) {
        fprintf(stderr, "Failed to set up the udev. Exit: (%d)\n", ret);
        goto exit;
    }

    ret = audio_fifo_process(&args);

    if (ret < 0) {
        fprintf(stderr, "Failed to create audio transwfer fifo\n");
        //goto exit;
    }

    process_event(usb_dev);

exit:

    if (usb_dev) {

        cleanup_udev_monitor(usb_dev->mon);

        cleanup_udev(usb_dev->udev);

        dbus_cleanup_connection(usb_dev->conn);

        free(usb_dev);
        close(args.fifofd);
    }

    return ret;
}
