#ifndef _DFU_FUNC_H_
#define _DFU_FUNC_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define DFU_EVENT_TYPE_CTRL_REQ		0
#define DFU_EVENT_TYPE_USB_STATE	1

#define DFU_USB_STATE_SUSPEND		0
#define DFU_USB_STATE_RESUME		1

struct dfu_ctrl_event {
	__u8 request;
	__u8 reserved[3];
	__u16 value;
	__u16 length;
	__u8 data[64];
};

struct dfu_event {
	__u32 event_type;
	union {
		__u32 usb_state;
		struct dfu_ctrl_event ctrl_event;
		__u8 data[72];
	} u;
};

struct dfu_usr_buf {
	__u32 index;
	__u32 bytesused;
	__u32 flags;
	__u32 length;
};

struct dfu_buf_alloc {
	__u32 num_of_buffs;
	__u32 buff_length;
};

struct dfu_buf_read {
	__u32 index;
	__u32 length;
	void *mem;
};

#define DFU_IOC_SEND_CTRL_REPLY			_IOW('U', 0, struct dfu_ctrl_event)
#define DFU_IOC_DQ_EVENT				_IOR('U', 1, struct dfu_event)
#define DFU_IOC_OPEN_STREAM				_IOW('U', 2, int)
#define DFU_IOC_CLOSE_STREAM			_IOW('U', 3, int)
#define DFU_IOC_CREATE_BUF				_IOW('U', 4, struct dfu_buf_alloc)
#define DFU_IOC_ENQUEUE_BUF				_IOW('U', 5, struct dfu_usr_buf)
#define DFU_IOC_DEQUEUE_BUF				_IOR('U', 6, struct dfu_usr_buf)
#define DFU_IOC_READ_BUF				_IOR('U', 7, struct dfu_buf_read)

#define MAX_UAC_CTRL_EVENTS_COUNT	10

#endif /* _DFU_FUNC_H_ */
