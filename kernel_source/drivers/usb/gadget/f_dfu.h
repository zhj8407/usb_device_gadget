#ifndef _DFU_FUNC_H_
#define _DFU_FUNC_H_

#include <linux/ioctl.h>
#include <linux/types.h>

struct dfu_ctrl_event {
	__u8 request;
	__u8 reserved[3];
	__u16 value;
	__u16 length;
	__u8 data[64];
};

#define DFU_IOC_SEND_CTRL_REPLY			_IOW('U', 0, struct dfu_ctrl_event)
#define DFU_IOC_DQ_CTRL_EVENT			_IOR('U', 1, struct dfu_ctrl_event)
#define DFU_IOC_OPEN_STREAM				_IOW('U', 2, int)
#define DFU_IOC_CLOSE_STREAM			_IOW('U', 3, int)

#define MAX_UAC_CTRL_EVENTS_COUNT	10

#endif /* _DFU_FUNC_H_ */
