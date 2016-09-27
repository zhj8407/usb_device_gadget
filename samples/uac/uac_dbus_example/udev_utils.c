#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include "udev_utils.h"

int setup_udev(struct udev **udev,
               struct udev_monitor **mon)
{
    *udev = udev_new();

    if (!*udev) {
        printf("Can't create udev\n");
        return -1;
    }

    /* This section sets up a monitor which will report events when
       devices attached to the system change.  Events include "add",
       "remove", "change", "online", and "offline".

       This section sets up and starts the monitoring. Events are
       polled for (and delivered) later in the file.

       It is important that the monitor be set up before the call to
       udev_enumerate_scan_devices() so that events (and devices) are
       not missed.  For example, if enumeration happened first, there
       would be no event generated for a device which was attached after
       enumeration but before monitoring began.

       Note that a filter is added so that we only get events for
       "hidraw" devices. */
    /* Set up a monitor to monitor usb devices */
    *mon = udev_monitor_new_from_netlink(*udev, "udev");

    if (!*mon) {
        fprintf(stderr, "Can not create the udev monitor\n");
        udev_unref(*udev);
        return -2;
    }

    udev_monitor_filter_add_match_subsystem_devtype(*mon,
            PLCM_USB_DEVICE_UEVENT_SUBSYSTEM, NULL);
    udev_monitor_enable_receiving(*mon);

    return 0;
}

int handle_usb_device_uevents(struct udev_monitor *mon,
                              char *usb_state, size_t max_len)
{
    struct udev_device *dev;
    struct udev_list_entry *properties, *props_list_entry;
    int ret = 0;

    dev = udev_monitor_receive_device(mon);

    if (!dev)
        return -1;

    if (strcmp(udev_device_get_subsystem(dev), PLCM_USB_DEVICE_UEVENT_SUBSYSTEM) ||
            strcmp(udev_device_get_syspath(dev), PLCM_USB_DEVICE_UEVENT_SYSPATH) ||
            strcmp(udev_device_get_action(dev), PLCM_USB_DEVICE_UEVENT_ACTION)) {
        udev_device_unref(dev);
        return -2;
    }

    properties = udev_device_get_properties_list_entry(dev);
    udev_list_entry_foreach(props_list_entry, properties) {
        if (!strcmp(udev_list_entry_get_name(props_list_entry), PLCM_USB_DEVICE_UEVENT_USB_STATE)) {
            strncpy(usb_state, udev_list_entry_get_value(props_list_entry), max_len);
            ret = 1;
        }
    }

    udev_device_unref(dev);

    return ret;
}

void cleanup_udev_monitor(struct udev_monitor *mon)
{
    if (mon)
        udev_monitor_unref(mon);
}

void cleanup_udev(struct udev *udev)
{
    if (udev)
        udev_unref(udev);
}
