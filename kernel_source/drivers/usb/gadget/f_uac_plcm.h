/*
 * f_uac_plcm.h -- Header file for PLCM UAC gadget driver
 *
 * Copyright (C) 2016 Jerry Zhang <Jerry.Zhang@polycom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __LINUX_USB_F_UAC_PLCM_H
#define __LINUX_USB_F_UAC_PLCM_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define MAX_UAC_CTRL_EVENTS_COUNT	10

struct uac_feature_unit {
    __u8 unit_id;
	__u8 control_selector;
	__u16 value;
};

struct uac_ctrl_event {
	__u8 request;
	union {
		struct uac_feature_unit fu;
		__u8 data[64];
	}u;
	__u8 reserved[3];
};

struct uac_feature_unit_value {
	__u32 length;
	__u8 data[16];
};

#define UAC_IOC_SEND_FU_VALUE			_IOW('U', 0, struct uac_feature_unit_value)
#define UAC_IOC_DQEVENT					_IOR('U', 1, struct uac_ctrl_event)

#endif /* __LINUX_USB_F_UAC_PLCM_H */
