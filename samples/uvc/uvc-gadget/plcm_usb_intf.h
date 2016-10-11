#ifndef __PLCM_USB_INTF_H__
#define __PLCM_USB_INTF_H__


#include "stdint.h"
#include "dbus_utils.h"

#define PLCM_USB_VISAGE_UVC_OBJ_PATH        "/com/polycom/visage/uvc"
#define PLCM_USB_VISAGE_UVC_INTF_NAME       "com.polycom.visage.uvc"
#define PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL  "video_control"
#define PLCM_USB_VISAGE_UVC_CAMERA_GET_SIGNAL  "GetCameraProperty"
#define PLCM_USB_VISAGE_UVC_CAMERA_SET_SIGNAL  "SetCameraProperty"


/*************USB video Events Begin********/
enum plcm_usb_video_event {
    e_app_ready      = 0,
    e_stack_ready,    //= 1,
    e_stream_ready,   //= 2,
    e_get_format,     //= 3,
    e_set_format,     //= 4,
    e_start_stream,   //= 5,
    e_stop_stream,    //= 6,
    e_retry_socket,         //= 7,
    e_last_event      //=8
};

struct plcm_uvc_format_t {
    unsigned int m_video_format;
    unsigned int m_width;
    unsigned int m_height;
    unsigned int m_framerate;
};

struct plcm_uvc_event_msg_t {
    enum plcm_usb_video_event m_event;
    struct plcm_uvc_format_t m_format;
};

static const char* plcm_usb_video_event_str[e_last_event + 1] = {
    "app ready",
    "stack ready",
    "stream ready",
    "get format",
    "set format",
    "start stream",
    "stop stream",
    "retry connect socket",
    "unknown"
};

/*************USB video Events End********/

enum usb_data_transfer_type {
    e_usb_isochronous,
    e_usb_bulk
};

struct usb_video_info_t {
    uint32_t video_width;
    uint32_t video_height;
};

static inline const char * getEventDescStr(enum plcm_usb_video_event event)
{
    if (event < e_last_event)
        return plcm_usb_video_event_str[event];
    else
        return plcm_usb_video_event_str[e_last_event];;
}
extern int send_get_property_signal(DBusConnection * dbus_con, uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length);
extern int send_set_property_signal(DBusConnection * dbus_con, uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length, uint8_t* data);
extern int notifyApplication(DBusConnection * dbus_con, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height);
extern int sendEvent2Fifo(int fd, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height);

/*#define UVC_CAMERA_TERMINAL_CONTROL_UNIT_ID     (1)
#define UVC_PROCESSING_UNIT_CONTROL_UNIT_ID     (2)


static inline int send_get_property_signal(DBusConnection * dbus_con, uint8_t req, uint8_t cs,
                         uint8_t unit_id)
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
                    PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL,
                    DBUS_TYPE_STRING, &request_type,
                    DBUS_TYPE_STRING, &prop_name,
                    DBUS_TYPE_INVALID);
}

static inline int send_set_property_signal(DBusConnection * dbus_con, uint8_t req, uint8_t cs,
                         uint8_t unit_id, uint16_t length, uint8_t* data)
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
                    PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL,
                    DBUS_TYPE_STRING, &request_type,
                    DBUS_TYPE_STRING, &prop_name,
                    DBUS_TYPE_INVALID);
}

static inline int notifyApplication(DBusConnection * dbus_con, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height)
{
    const char * tmp = getEventDescStr(event);
    return dbus_send_signal_with_params(dbus_con, PLCM_USB_VISAGE_UVC_OBJ_PATH,
                    PLCM_USB_VISAGE_UVC_INTF_NAME,
                    PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL,
                    DBUS_TYPE_STRING, &tmp,
                    DBUS_TYPE_INVALID);
}
static inline int sendEvent2Fifo(int fd, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height)
{
    struct plcm_uvc_event_msg_t event_msg;
    event_msg.m_event = event;
    event_msg.m_format.m_video_format = format;
    event_msg.m_format.m_height = height;
    event_msg.m_format.m_width = width;
    event_msg.m_format.m_framerate = 60;
    return write(fd, &event_msg, sizeof(struct plcm_uvc_event_msg_t));
}*/


#endif
