#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "dbus_utils.h"
#include "udev_utils.h"
#include "file_utils.h"

#define PLCM_USB_VISAGE_UDM_STATE_SYSFS     "/sys/class/plcm_usb/plcm0/state"
#define PLCM_USB_VISAGE_UDM_FUNCTIONS_SYSFS "/sys/class/plcm_usb/plcm0/functions"
#define PLCM_USB_VISAGE_UDM_ENABLE_SYSFS    "/sys/class/plcm_usb/plcm0/enable"

#define PLCM_USB_VISAGE_UDM_OBJ_PATH        "/com/polycom/visage/udm"
#define PLCM_USB_VISAGE_UDM_INTF_NAME       "com.polycom.visage.udm"

#define PLCM_USB_VISAGE_USB_STATE_SIGNAL    "usb_state"

struct usb_device_manager {
    DBusConnection *conn;
    struct udev *udev;
    struct udev_monitor *mon;

    char usb_state[16];
};

DBusHandlerResult object_visage_handler(DBusConnection *, DBusMessage *, void *);
void object_unregister_handler(DBusConnection *, void *);

char *objectPaths[] = {
    PLCM_USB_VISAGE_UDM_OBJ_PATH
};

DBusObjectPathVTable objectPathVTable[] = {
    {
        .unregister_function = object_unregister_handler,
        .message_function = object_visage_handler
    }
};

DBusHandlerResult object_visage_handler(DBusConnection* conn,
                                        DBusMessage* msg, void* data)
{
    (void)data;

    switch (dbus_message_get_type(msg)) {
        case DBUS_MESSAGE_TYPE_SIGNAL:
        case DBUS_MESSAGE_TYPE_ERROR:
            fprintf(stdout, "object_visage_handler(error/signal):\n\t%s\n",
                    _verbose_message(msg));
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

        case DBUS_MESSAGE_TYPE_METHOD_RETURN:
            return handle_method_return(conn, msg);

        case DBUS_MESSAGE_TYPE_METHOD_CALL:
            printf("object_visage_handler: method call\n");

            if (strcmp(dbus_message_get_interface(msg), PLCM_USB_VISAGE_UDM_INTF_NAME))
                return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

            if (!strcmp(dbus_message_get_member(msg), "GetCurrentUsbState")) {
                char *usb_state = (char *)malloc(128);
                int ret = read_value_from_file(PLCM_USB_VISAGE_UDM_STATE_SYSFS, "%s\n", usb_state);

                if (ret)
                    usb_state[0] = '\0';

                printf("Get current usb state: %s\n", usb_state);
                dbus_send_method_call_reply_with_results(conn, msg,
                        DBUS_TYPE_STRING, &usb_state,
                        DBUS_TYPE_INVALID);
                free(usb_state);

                return DBUS_HANDLER_RESULT_HANDLED;
            } else if (!strcmp(dbus_message_get_member(msg), "GetDefaultUsbFunctions")) {
                char *usb_functions = (char *)malloc(128);
                int ret = read_value_from_file(PLCM_USB_VISAGE_UDM_FUNCTIONS_SYSFS, "%s\n", usb_functions);

                if (ret)
                    usb_functions[0] = '\0';

                printf("Get default usb functions: %s\n", usb_functions);
                dbus_send_method_call_reply_with_results(conn, msg,
                        DBUS_TYPE_STRING, &usb_functions,
                        DBUS_TYPE_INVALID);
                free(usb_functions);

                return DBUS_HANDLER_RESULT_HANDLED;
            }  else if (!strcmp(dbus_message_get_member(msg), "SetCurrentUsbFunctions")) {
                char *usb_functions;

                int ret = dbus_read_params_from_msg(msg, DBUS_TYPE_STRING, &usb_functions, DBUS_TYPE_INVALID);

                if (ret)
                    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;

                ret = write_value_to_file(PLCM_USB_VISAGE_UDM_FUNCTIONS_SYSFS, "%s", usb_functions);

                printf("Set current usb functions: %s\n", usb_functions);
                dbus_send_method_call_reply_with_results(conn, msg,
                        DBUS_TYPE_INT32, &ret,
                        DBUS_TYPE_INVALID);

                //Do not need to free the usb_functions here.
                //Otherwise it would core dump

                return DBUS_HANDLER_RESULT_HANDLED;
            } else if (!strcmp(dbus_message_get_member(msg), "StartUsbDevice") ||
                       !strcmp(dbus_message_get_member(msg), "StopUsbDevice")) {
                int enabled = !(strcmp(dbus_message_get_member(msg), "startUsbDevice"));
                printf("USB Device enable: %d\n", enabled);
                int ret = write_value_to_file(PLCM_USB_VISAGE_UDM_ENABLE_SYSFS, "%d", enabled);

                dbus_send_method_call_reply_with_results(conn, msg,
                        DBUS_TYPE_INT32, &ret,
                        DBUS_TYPE_INVALID);
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

int notify_usb_state(struct usb_device_manager *usb_dev)
{
    const char *tmp = usb_dev->usb_state;

    return dbus_send_signal_with_params(usb_dev->conn, PLCM_USB_VISAGE_UDM_OBJ_PATH,
                                        PLCM_USB_VISAGE_UDM_INTF_NAME,
                                        PLCM_USB_VISAGE_USB_STATE_SIGNAL,
                                        DBUS_TYPE_STRING, &tmp,
                                        DBUS_TYPE_INVALID);
}

void listen_to_dbus_udev(struct usb_device_manager * usb_dev)
{
    int ret;
    fd_set rfds, wfds, efds;
    int max_fd = -1, udev_mon_fd;

    while (1) {
        // Prepare for the select
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_ZERO(&efds);
        max_fd = 0;

        // Add dbus watcher fds
        max_fd = setupListenFds(&rfds, &wfds, &efds);

        // Add udev fd
        udev_mon_fd = udev_monitor_get_fd(usb_dev->mon);

        FD_SET(udev_mon_fd, &rfds);

        max_fd = udev_mon_fd > max_fd ? udev_mon_fd : max_fd;

        ret = select(max_fd + 1, &rfds, &wfds, &efds, NULL);

        if (ret <= 0)
            break;

        if (FD_ISSET(udev_mon_fd, &rfds)) {
            ret = handle_usb_device_uevents(usb_dev->mon,
                                            usb_dev->usb_state, sizeof(usb_dev->usb_state));

            if (ret > 0)
                notify_usb_state(usb_dev);
            else
                continue;
        } else {
            //printf("Select was woken up by dbus message\n");
            handleListenFds(&rfds, &wfds, &efds);

            watchHandle(usb_dev->conn);

            timeoutHandle();
        }
    }
}

int main(void)
{
    struct usb_device_manager *usb_dev = NULL;
    int ret;

    usb_dev = (struct usb_device_manager *)malloc(sizeof(*usb_dev));

    if (!usb_dev) {
        fprintf(stderr, "Failed to allocate the usb_dev, error: %d(%s)\n",
                errno, strerror(errno));
        ret = -1;
        goto exit;
    }

    memset(usb_dev, 0, sizeof(*usb_dev));

    ret = setup_dbus_connection(&usb_dev->conn, "com.polycom.visage.udm",
                                "type='signal',interface='test.signal.Type'");

    if (ret) {
        fprintf(stderr, "No dbus connection. Exit: (%d)\n", ret);
        goto exit;
    }

    dbus_register_message_filter(usb_dev->conn, msg_filter);

    dbus_register_object_patch(usb_dev->conn,
                               objectPaths, objectPathVTable,
                               sizeof(objectPaths) / sizeof(objectPaths[0]));

    ret = setup_udev(&usb_dev->udev, &usb_dev->mon);

    if (ret) {
        fprintf(stderr, "Failed to set up the udev. Exit: (%d)\n", ret);
        goto exit;
    }

    listen_to_dbus_udev(usb_dev);

exit:

    if (usb_dev) {

        cleanup_udev_monitor(usb_dev->mon);

        cleanup_udev(usb_dev->udev);

        cleanup_dbus_connection(usb_dev->conn);

        free(usb_dev);
    }

    return ret;
}
