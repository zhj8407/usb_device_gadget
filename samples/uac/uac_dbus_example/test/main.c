#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "dbus_utils.h"
#include "dbus_event_utils.h"
#define DEBUG

#define PLCM_USB_VISAGE_UAC_OBJ_PATH        "/com/polycom/visage/uac"
#define PLCM_USB_VISAGE_UAC_INTF_NAME       "com.polycom.visage.uac"

#define PLCM_USB_VISAGE_UAC_STATE_SIGNAL    "uac_usb_state"
#define PLCM_USB_VISAGE_UAC_DATA_SIGNAL    "uac_usb_data"

#define USBAUDIO_bControlSelector_FU_MUTE           0x01
#define USBAUDIO_bControlSelector_FU_VOLUME         0x02
#define MIC_INPUT_FEATURE_UNIT_ID 0x2

#define USB_bRequest_Audio_UNDEFINED  0x00  /* undefined */
#define USB_bRequest_Audio_SET_CUR  0x01    /* set current */
#define USB_bRequest_Audio_GET_CUR  0x81    /* get current */
#define USB_bRequest_Audio_SET_MIN  0x02    /* set minimum */
#define USB_bRequest_Audio_GET_MIN  0x82    /* get minimum */
#define USB_bRequest_Audio_SET_MAX  0x03    /* set maximum */
#define USB_bRequest_Audio_GET_MAX  0x83    /* get maximum */
#define USB_bRequest_Audio_SET_RES  0x04    /* set resolution */
#define USB_bRequest_Audio_GET_RES  0x84    /* get resolution */
#define USB_bRequest_Audio_SET_MEM  0x05    /* set memory space */
#define USB_bRequest_Audio_GET_MEM  0x85    /* get memory space */
#define USB_bRequest_Audio_GET_STAT 0xFF    /* get status */

typedef struct USBAudioFUControl {
    unsigned short                   bMute;
    unsigned short                  wMinVol;
    unsigned short                  wMaxVol;
    unsigned short                  wResVol;
    unsigned short                  wCurVol;
} USBAudioFUControl;

struct udm_client {
    DBusConnection *conn;
    struct event *timeout_event;
    struct timeval *timeout;     //For timeout
};

struct uac_dbus_data_event {
    unsigned char bRequest;
    unsigned char bUnitId;
    unsigned char bControlSelector;//mute:0x01, volume: 0x02
    unsigned char reserved[1];
    unsigned short wValue;
};

USBAudioFUControl usbAudioFuCtrl;

void UsbAudioDataProcess(DBusConnection *conn, struct uac_dbus_data_event *event, USBAudioFUControl *usbAudioFuCtrl)
{
    DBusMessage *reply;
    int ret = 0;
    int bMsgCall = 0;

    switch (event->bControlSelector) {
        case USBAUDIO_bControlSelector_FU_MUTE: {
            if (event->bRequest == USB_bRequest_Audio_SET_CUR) {
                printf("MUTE SET_CUR is received=0x%x\n", event->wValue);
                usbAudioFuCtrl->bMute = event->wValue;
            } else if (event->bRequest == USB_bRequest_Audio_GET_CUR) {
                printf("VOLUME GET_CUR is received val=0x%x\n", event->wValue);
                ret = dbus_send_method_call_with_params(conn,
                                                        &reply,
                                                        "com.polycom.visage.uac",
                                                        PLCM_USB_VISAGE_UAC_OBJ_PATH,
                                                        PLCM_USB_VISAGE_UAC_INTF_NAME,
                                                        "GetUacValue",
                                                        -1,
                                                        DBUS_TYPE_UINT16,
                                                        &usbAudioFuCtrl->bMute,
                                                        DBUS_TYPE_INVALID);
                bMsgCall = 1;
            }
        }
        break;

        case USBAUDIO_bControlSelector_FU_VOLUME: {
            switch (event->bRequest) {
                case USB_bRequest_Audio_SET_CUR: {
                    printf("VOLUME SET_CUR is received, val=0x%x\n", event->wValue);
                    usbAudioFuCtrl->wCurVol = event->wValue;
                }
                break;

                case USB_bRequest_Audio_SET_MIN: {
                    printf("VOLUME SET_MIN is received, val=0x%x\n", event->wValue);
                    usbAudioFuCtrl->wMinVol = event->wValue;
                }
                break;

                case USB_bRequest_Audio_SET_MAX: {
                    printf("VOLUME SET_MIN is received, val=0x%x\n", event->wValue);
                    usbAudioFuCtrl->wMaxVol = event->wValue;
                }
                break;

                case USB_bRequest_Audio_SET_RES: {
                    printf("VOLUME SET_RES is received, val=0x%x\n", event->wValue);
                    usbAudioFuCtrl->wMaxVol = event->wValue;
                }
                break;

                case USB_bRequest_Audio_GET_CUR: {

                    printf("VOLUME GET_CUR is received,val=0x%x\n", usbAudioFuCtrl->wCurVol);
                    ret = dbus_send_method_call_with_params(conn,
                                                            &reply,
                                                            "com.polycom.visage.uac",
                                                            PLCM_USB_VISAGE_UAC_OBJ_PATH,
                                                            PLCM_USB_VISAGE_UAC_INTF_NAME,
                                                            "GetUacValue",
                                                            -1,
                                                            DBUS_TYPE_UINT16,
                                                            &usbAudioFuCtrl->wCurVol,
                                                            DBUS_TYPE_INVALID);
                    bMsgCall = 1;
                }
                break;

                case USB_bRequest_Audio_GET_MAX: {
                    printf("VOLUME GET_MAX is received,val=0x%x\n", usbAudioFuCtrl->wMaxVol);
                    ret = dbus_send_method_call_with_params(conn,
                                                            &reply,
                                                            "com.polycom.visage.uac",
                                                            PLCM_USB_VISAGE_UAC_OBJ_PATH,
                                                            PLCM_USB_VISAGE_UAC_INTF_NAME,
                                                            "GetUacValue",
                                                            -1,
                                                            DBUS_TYPE_UINT16,
                                                            &usbAudioFuCtrl->wMaxVol,
                                                            DBUS_TYPE_INVALID);
                    bMsgCall = 1;
                }
                break;

                case USB_bRequest_Audio_GET_MIN: {
                    printf("VOLUME GET_MAX is received,val=0x%x\n", usbAudioFuCtrl->wMinVol);
                    ret = dbus_send_method_call_with_params(conn,
                                                            &reply,
                                                            "com.polycom.visage.uac",
                                                            PLCM_USB_VISAGE_UAC_OBJ_PATH,
                                                            PLCM_USB_VISAGE_UAC_INTF_NAME,
                                                            "GetUacValue",
                                                            -1,
                                                            DBUS_TYPE_UINT16,
                                                            &usbAudioFuCtrl->wMinVol,
                                                            DBUS_TYPE_INVALID);
                    bMsgCall = 1;
                }
                break;

                case USB_bRequest_Audio_GET_RES: {
                    printf("VOLUME GET_MAX is received,val=0x%x\n", usbAudioFuCtrl->wResVol);
                    ret = dbus_send_method_call_with_params(conn,
                                                            &reply,
                                                            "com.polycom.visage.uac",
                                                            PLCM_USB_VISAGE_UAC_OBJ_PATH,
                                                            PLCM_USB_VISAGE_UAC_INTF_NAME,
                                                            "GetUacValue",
                                                            -1,
                                                            DBUS_TYPE_UINT16,
                                                            &usbAudioFuCtrl->wResVol,
                                                            DBUS_TYPE_INVALID);
                    bMsgCall = 1;
                }
                break;

                default:
                    break;

            }
        }
        break;

        default:
            break;

    }

    if (ret) {
        fprintf(stderr, "Failed to call the remote method, ret: %d\n", ret);
        return;
    }

    if (bMsgCall == 1) {
        dbus_read_params_from_msg(reply, DBUS_TYPE_INT32, &ret, DBUS_TYPE_INVALID);

        if (!ret) {
            printf("method call return value: %d\n", ret);
        }

        //Do not forget to unref the message.
        dbus_message_unref(reply);
    }

}

DBusHandlerResult msg_filter(DBusConnection *conn,
                             DBusMessage *msg, void *data)
{
    (void)conn;
    (void)data;
#ifdef DEBUG
    printf("incoming msg: %s\n", _verbose_message(msg));
#endif

    if (dbus_message_get_type(msg) != DBUS_MESSAGE_TYPE_SIGNAL) {
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
    }

    if (!strcmp(dbus_message_get_member(msg), PLCM_USB_VISAGE_UAC_STATE_SIGNAL)) {
        char *usb_state;
        int ret = dbus_read_params_from_msg(msg, DBUS_TYPE_STRING, &usb_state, DBUS_TYPE_INVALID);

        if (ret == 0)
#ifdef DEBUG
            printf("Current usb state: %s\n", usb_state);

#endif
        return DBUS_HANDLER_RESULT_HANDLED;
    } else if (!strcmp(dbus_message_get_member(msg), PLCM_USB_VISAGE_UAC_DATA_SIGNAL)) {
        struct uac_dbus_data_event data;
        int ret = dbus_read_params_from_msg(msg, DBUS_TYPE_BYTE, &data.bRequest,
                                            DBUS_TYPE_BYTE, &data.bUnitId,
                                            DBUS_TYPE_BYTE, &data.bControlSelector,
                                            DBUS_TYPE_UINT16, &data.wValue,
                                            DBUS_TYPE_INVALID);

        if (ret == 0) {
            printf("client received the event\n");
            UsbAudioDataProcess(conn, &data, &usbAudioFuCtrl);
#ifdef DEBUG
            printf("\t\tEvent.request:         0x%02x\n", data.bRequest);
            printf("\t\tEvent.unit_id:         0x%02x\n", data.bUnitId);
            printf("\t\tEvent.contro_selector: 0x%02x\n", data.bControlSelector);
            printf("\t\tEvent.value:           0x%04x\n", data.wValue);
#endif
        } else {
            printf("Dequeued the event fail\n");
        }

        return DBUS_HANDLER_RESULT_HANDLED;
    }

    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

static void timeout_cb(evutil_socket_t fd, short what, void *arg)
{
    struct udm_client *client = (struct udm_client *)arg;
    DBusMessage *reply;
    int ret;
    return;
    (void)fd;
    (void)what;

    ret = dbus_send_method_call_with_params(client->conn,
                                            &reply,
                                            "com.polycom.visage.uac",   //Destination
                                            PLCM_USB_VISAGE_UAC_OBJ_PATH,   //Object Path
                                            PLCM_USB_VISAGE_UAC_INTF_NAME,  //Interface Name
                                            "GetDefaultUsbFunctions",       //Member Name
                                            -1,                             //Timeout
                                            DBUS_TYPE_INVALID);

    if (ret) {
        fprintf(stderr, "Failed to call the remote method, ret: %d\n", ret);
        //Stop the timer
        return;
    }

    char *default_funcs;

    ret = dbus_read_params_from_msg(reply, DBUS_TYPE_STRING, &default_funcs, DBUS_TYPE_INVALID);

    if (!ret) {
        printf("Current usb functions: %s\n", default_funcs);
    }

    //Do not forget to unref the message.
    dbus_message_unref(reply);

    evtimer_add(client->timeout_event, client->timeout);
}

int main(void)
{
    struct udm_client client;
    struct dbus_ctx *ctx = NULL;
    struct event_base *ev_base = NULL;
    char msg_match[128];
    struct timeval tv = {
        .tv_sec = 50,
        .tv_usec = 0,
    };
    int ret;

    ev_base = event_base_new();

    if (!ev_base) {
        fprintf(stderr, "Failed to allocate the event base\n");
        exit(-1);
    }

    snprintf(msg_match, sizeof(msg_match),
             "type='signal',interface='%s'",
             PLCM_USB_VISAGE_UAC_INTF_NAME);

    ret = setup_dbus_event(&ctx, NULL,
                           msg_match, msg_filter, ev_base);

    if (ret) {
        fprintf(stderr, "Failed to setup the dbus event\n");
        exit(-2);
    }

    client.conn = ctx->conn;
    client.timeout = &tv;

    //Create the timeout event
    client.timeout_event = evtimer_new(ev_base, timeout_cb, &client);

    //Add the timeout event
    evtimer_add(client.timeout_event, &tv);

    //event loop started
    event_base_dispatch(ev_base);

    cleanup_dbus_event(ctx);

    event_base_free(ev_base);

    return 0;
}
