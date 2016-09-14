#ifndef UDEV_UTILS_H_INCLUDED
#define UDEV_UTILS_H_INCLUDED

#include <libudev.h>

#define PLCM_USB_DEVICE_UEVENT_SUBSYSTEM    "plcm_usb"
#define PLCM_USB_DEVICE_UEVENT_SYSPATH      "/sys/devices/virtual/plcm_usb/plcm0"
#define PLCM_USB_DEVICE_UEVENT_ACTION       "change"
#define PLCM_USB_DEVICE_UEVENT_USB_STATE    "USB_STATE"

int setup_udev(struct udev **udev,
               struct udev_monitor **mon);

int handle_usb_device_uevents(struct udev_monitor *mon,
                              char *usb_state, size_t max_len);

void cleanup_udev_monitor(struct udev_monitor *mon);

void cleanup_udev(struct udev *udev);

#endif // UDEV_UTILS_H_INCLUDED