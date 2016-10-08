#ifndef __PLCM_USB_INTF_H__
#define __PLCM_USB_INTF_H__

#include "dbus_utils.h"
#define PLCM_USB_VISAGE_UVC_OBJ_PATH        "/com/polycom/visage/uvc"
#define PLCM_USB_VISAGE_UVC_INTF_NAME       "com.polycom.visage.uvc"
#define PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL  "video_control"

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

inline int notifyApplication(DBusConnection * dbus_con, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height)
{
    const char * tmp = getEventDescStr(event);
    return dbus_send_signal_with_params(dbus_con, PLCM_USB_VISAGE_UVC_OBJ_PATH,
                    PLCM_USB_VISAGE_UVC_INTF_NAME,
                    PLCM_USB_VISAGE_UVC_CONTROL_SIGNAL,
                    DBUS_TYPE_STRING, &tmp,
                    DBUS_TYPE_INVALID);
}
inline int sendEvent2Fifo(int fd, enum plcm_usb_video_event event, unsigned int format, unsigned int width, unsigned int height)
{
    struct plcm_uvc_event_msg_t event_msg;
    event_msg.m_event = event;
    event_msg.m_format.m_video_format = format;
    event_msg.m_format.m_height = height;
    event_msg.m_format.m_width = width;
    event_msg.m_format.m_framerate = 60;
    return write(fd, &event_msg, sizeof(struct plcm_uvc_event_msg_t));
}
#endif
