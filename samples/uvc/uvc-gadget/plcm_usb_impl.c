/*******************************************************************
this file implements the communication related functions.
********************************************************************/


#include "plcm_usb_intf.h"
#include "log_str.h"

#define UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID     (1)
#define UVC_PROCESSING_UNIT_CONTROL_UNIT_ID     (2)


int send_get_property_signal(DBusConnection * dbus_con, uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length)
{
    const char *prop_name;
    const char *request_type;

    request_type = getUVCReqStr(req);

    if (unit_id == UVC_PROCESSING_UNIT_CONTROL_UNIT_ID)
        prop_name = getUVCPUCS(cs);
    else if (unit_id == UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID)
        prop_name = getUVCCTCS(cs);

    return dbus_send_signal_with_params(dbus_con, PLCM_USB_VISAGE_UVC_OBJ_PATH,
                                        PLCM_USB_VISAGE_UVC_INTF_NAME,
                                        PLCM_USB_VISAGE_UVC_CAMERA_GET_SIGNAL,
                                        DBUS_TYPE_STRING, &prop_name,
                                        DBUS_TYPE_STRING, &request_type,
                                        DBUS_TYPE_UINT16, &length,
                                        DBUS_TYPE_INVALID);
}

int send_set_property_signal(DBusConnection * dbus_con, uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length, uint8_t* data)
{
    (void)length;
    (void)data;
    const char *prop_name;
    const char *request_type;

    request_type = getUVCReqStr(req);

    if (unit_id == UVC_PROCESSING_UNIT_CONTROL_UNIT_ID)
        prop_name = getUVCPUCS(cs);
    else if (unit_id == UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID)
        prop_name = getUVCCTCS(cs);

    return dbus_send_signal_with_params(dbus_con, PLCM_USB_VISAGE_UVC_OBJ_PATH,
                                        PLCM_USB_VISAGE_UVC_INTF_NAME,
                                        PLCM_USB_VISAGE_UVC_CAMERA_SET_SIGNAL,
                                        DBUS_TYPE_STRING, &request_type,
                                        DBUS_TYPE_STRING, &prop_name,
                                        DBUS_TYPE_INVALID);
}

int notifyApplication(DBusConnection * dbus_con, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height)
{
    (void)format;
    (void)width;
    (void)height;
    const char * tmp = getEventDescStr(event);
    return dbus_send_signal_with_params(dbus_con, PLCM_USB_VISAGE_UVC_OBJ_PATH,
                                        PLCM_USB_VISAGE_UVC_INTF_NAME,
                                        PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL,
                                        DBUS_TYPE_STRING, &tmp,
                                        DBUS_TYPE_INVALID);
}
int sendEvent2Fifo(int fd, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height)
{
    struct plcm_uvc_event_msg_t event_msg;
    event_msg.m_event = event;
    event_msg.m_format.m_video_format = format;
    event_msg.m_format.m_height = height;
    event_msg.m_format.m_width = width;
    event_msg.m_format.m_framerate = 60;
    return write(fd, &event_msg, sizeof(struct plcm_uvc_event_msg_t));
}

