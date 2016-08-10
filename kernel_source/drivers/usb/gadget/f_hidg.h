/*
 * f_hidg.h -- Header file for USB HID gadget driver
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

#ifndef __LINUX_USB_F_HIDG_H
#define __LINUX_USB_F_HIDG_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define HID_REPORT_DESC_MAX_LENGTH	2048
#define USB_STRING_MAX_LENGTH 126

struct hidg_device_config {
	int	device;
	struct device *dev;

	unsigned char bInterfaceSubClass;
	unsigned char bInterfaceProtocol;

	unsigned short report_length;
	unsigned short report_desc_length;
	unsigned char report_desc[HID_REPORT_DESC_MAX_LENGTH];
	unsigned char lync_ucq_string[USB_STRING_MAX_LENGTH];
};

struct hidg_report_data
{
	__s32 length;
	__u8 data[60];
};

#define HIDG_IOC_SEND_VENDOR_REPORT		_IOW('U', 1, struct hidg_report_data)

#endif /* __LINUX_USB_F_HIDG_H */
