#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "dbus_utils.h"
#include "dbus_event_utils.h"

#define PLCM_USB_VISAGE_UDM_OBJ_PATH        "/com/polycom/visage/udm"
#define PLCM_USB_VISAGE_UDM_INTF_NAME       "com.polycom.visage.udm"

#define PLCM_USB_VISAGE_USB_STATE_SIGNAL    "usb_state"


struct udm_client {
    DBusConnection *conn;
    struct event *timeout_event;
    struct timeval *timeout;     //For timeout
};


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
#ifdef DEBUG
    printf("incoming msg: %s\n", _verbose_message(msg));
#endif

    if (dbus_message_get_type(msg) != DBUS_MESSAGE_TYPE_SIGNAL)
        return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

    if (!strcmp(dbus_message_get_member(msg),
                PLCM_USB_VISAGE_USB_STATE_SIGNAL)) {
        char *usb_state;

        int ret = dbus_read_params_from_msg(msg, DBUS_TYPE_STRING, &usb_state, DBUS_TYPE_INVALID);

        if (ret == 0)
            printf("Current usb state: %s\n", usb_state);

        return DBUS_HANDLER_RESULT_HANDLED;
    }

    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

static void timeout_cb(evutil_socket_t fd, short what, void *arg)
{
    struct udm_client *client = (struct udm_client *)arg;
    DBusMessage *reply;
    int ret;

    (void)fd;
    (void)what;

    ret = dbus_send_method_call_with_params(client->conn,
                                    &reply,
                                    "com.polycom.visage.udm",   //Destination
                                    PLCM_USB_VISAGE_UDM_OBJ_PATH,   //Object Path
                                    PLCM_USB_VISAGE_UDM_INTF_NAME,  //Interface Name
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
        .tv_sec = 5,
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
             PLCM_USB_VISAGE_UDM_INTF_NAME);

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
