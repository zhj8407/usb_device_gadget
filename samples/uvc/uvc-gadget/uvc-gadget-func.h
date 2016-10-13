#ifndef UVC_GADGET_FUNC_H
#define UVC_GADGET_FUNC_H
#include <stdint.h>
#include "uvc.h"

#define JPEG_QUALITY        80

#define VISAGE_LED_NOTIFICATION 1

#define ARRAY_SIZE(a)   ((sizeof(a) / sizeof(a[0])))

struct uvc_callback_table {
    void (*on_stream_on)();
    void (*on_stream_off)();
    void (*on_get_param)(uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length);
    void (*on_set_param)(uint8_t req, uint8_t cs, uint8_t unit_id, uint16_t length, uint8_t* data);
    int (*on_req_frame)(uint8_t * buffer, uint32_t buffer_max_len, uint32_t *buffer_act_len);
};

struct uvc_device {
    int fd;

    struct uvc_streaming_control probe;
    struct uvc_streaming_control commit;

    int control;
    int unit;

    unsigned int fcc;
    unsigned int width;
    unsigned int height;

    void **mem;
    unsigned int nbufs;
    unsigned int bufsize;

    unsigned int bulk;
    uint8_t color;
    unsigned int imgsize;
    void *imgdata;

    int v4ldevnum;
    unsigned int maxpacketsize;
    unsigned int headersize;
    unsigned int maxpayloadsize;

    int stream_on;
    //communication modules
    DBusConnection *dbus_con;
    int stack2app_fd;
    int app2stack_fd;

    //callbacks
    struct uvc_callback_table callbacks;
};

void uvc_fill_streaming_control(struct uvc_device *dev,
                                struct uvc_streaming_control *ctrl,
                                int iframe, int iformat);
void uvc_events_process_standard(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                                 struct uvc_request_data *resp);
void uvc_events_process_control(struct uvc_device *dev, uint8_t req, uint8_t cs,
                                uint8_t unit_id, uint16_t length, struct uvc_request_data *resp);
void uvc_events_process_streaming(struct uvc_device *dev, uint8_t req, uint8_t cs,
                                  struct uvc_request_data *resp);
void uvc_events_process_class(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                              struct uvc_request_data *resp);
void uvc_events_process_setup(struct uvc_device *dev, struct usb_ctrlrequest *ctrl,
                              struct uvc_request_data *resp);

void uvc_events_process_data(struct uvc_device *dev, struct uvc_request_data *data);

void handle_frame_done_event(struct uvc_device *dev,
                             struct uvc_frame_done_info *frame_done_info);
void uvc_events_process(struct uvc_device *dev);

void uvc_events_init(struct uvc_device *dev, struct uvc_callback_table * callback_table);

int uvc_video_set_format(struct uvc_device *dev);

int uvc_video_stream(struct uvc_device *dev, int enable);

int uvc_video_process(struct uvc_device *dev);

int uvc_video_reqbufs(struct uvc_device *dev, int nbufs);


#endif

