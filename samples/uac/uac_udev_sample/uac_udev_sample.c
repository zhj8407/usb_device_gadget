#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>
#include <sys/types.h>

#include "libudev.h"

void sig_child(int sig)
{
    pid_t pid;
    int status;
    printf("Child process has been killed, Sig: %d\n", sig);

    while ((pid = waitpid(-1, &status, WNOHANG)) > 0);
}

pid_t start_audio_in()
{
    pid_t child;

    if ((child = fork()) == 0) {
        //Child process
        execlp("audio_capture", "audio_capture", "-m 0", "-n 16",
               "-t2", "-d2", "-r32000", "-b 1", "-c -1","-o 1", NULL);
        perror("audio_capture");
        exit(errno);
    } else {
        //Parent process
    }

    return child;
}

void stop_audio_in(pid_t child)
{
    if (child > 0)
        kill(child, SIGABRT);
}


int main()
{
    struct udev *udev;
    struct udev_device *dev;
    struct udev_list_entry *properties, *props_list_entry;
    struct udev_monitor *mon;
    int fd;
    pid_t child = 0;
    sigset_t sigset;
    struct sigaction act;
    act.sa_handler = sig_child;

    if (sigemptyset(&act.sa_mask) == -1) {
        printf("Can't empty the signal set\n");
        exit(1);
    }

    act.sa_flags = 0;

    if (sigaction(SIGCHLD, &act, NULL) == -1) {
        printf("Can't set the signal action");
        exit(1);
    }

    if (sigemptyset(&sigset) == -1) {
        printf("Can't empty the signal set\n");
        exit(1);
    }

    if (sigaddset(&sigset, SIGCHLD) == -1) {
        printf("Can't add the SIGCHLD to sigset");
        exit(-1);
    }

    // signal(SIGCHLD, sig_child);
    /* Create the udev object */
    udev = udev_new();

    if (!udev) {
        printf("Can't create udev\n");
        exit(1);
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
    /* Set up a monitor to monitor hidraw devices */
    mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(mon, "plcm_usb", NULL);
    udev_monitor_enable_receiving(mon);
    /* Get the file descriptor (fd) for the monitor.
       This fd will get passed to select() */
    fd = udev_monitor_get_fd(mon);

    /* Begin polling for udev events. Events occur when devices
       attached to the system are added, removed, or change state.
       udev_monitor_receive_device() will return a device
       object representing the device which changed and what type of
       change occured.

       The select() system call is used to ensure that the call to
       udev_monitor_receive_device() will not block.

       The monitor was set up earler in this file, and monitoring is
       already underway.

       This section will run continuously, calling usleep() at the end
       of each pass. This is to demonstrate how to use a udev_monitor
       in a non-blocking way. */
    while (1) {
        /* Set up the call to select(). In this case, select() will
           only operate on a single file descriptor, the one
           associated with our udev_monitor. Note that the timeval
           object is set to 0, which will cause select() to not
           block. */
        fd_set fds;
        // struct timeval tv;
        struct timespec tv;
        int ret;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        tv.tv_sec = 0;
        tv.tv_nsec = 250 * 1000 * 1000;
        ret = pselect(fd + 1, &fds, NULL, NULL, &tv, &sigset);

        /* Check if our file descriptor has received data. */
        if (ret == -1) {
            // if (errno != EINTR) {
            printf("Error in select\n");
            break;
            // }
        } else if (ret) {
            // printf("\nselect() says there should be data\n");
            if (FD_ISSET(fd, &fds)) {
                /* Make the call to receive the device.
                   select() ensured that this will not block. */
                dev = udev_monitor_receive_device(mon);

                if (dev) {
                    // printf("Got Device\n");
                    // printf("   Node: %s\n", udev_device_get_devnode(dev));
                    // printf("   Subsystem: %s\n", udev_device_get_subsystem(dev));
                    // printf("   Devtype: %s\n", udev_device_get_devtype(dev));
                    // printf("   Syspath: %s\n", udev_device_get_syspath(dev));
                    // printf("   Action: %s\n", udev_device_get_action(dev));
                    // properties = udev_device_get_properties_list_entry(dev);
                    // printf("   Properties:\n");
                    // udev_list_entry_foreach(props_list_entry, properties) {
                    //     printf("       %s : %s\n",
                    //            udev_list_entry_get_name(props_list_entry),
                    //            udev_list_entry_get_value(props_list_entry));
                    // }
                    if (strcmp(udev_device_get_subsystem(dev), "plcm_usb") ||
                            strcmp(udev_device_get_syspath(dev), "/sys/devices/virtual/plcm_usb/plcm0/f_audio_dual") ||
                            strcmp(udev_device_get_action(dev), "change")) {
                        udev_device_unref(dev);
                        continue;
                    }

                    properties = udev_device_get_properties_list_entry(dev);
                    udev_list_entry_foreach(props_list_entry, properties) {
                        if (!strcmp(udev_list_entry_get_name(props_list_entry), "AUDIO_SOURCE_STATE")) {
                            if (!strcmp(udev_list_entry_get_value(props_list_entry), "ACTIVE")) {
                                //TODO
                                child = start_audio_in();
                                printf("\nAudio Source Active, child: %d\n", child);
                            } else if (!strcmp(udev_list_entry_get_value(props_list_entry), "DEACTIVE")) {
                                //TODO
                                stop_audio_in(child);
                                printf("\nAudio Source Deactive, child: %d\n", child);
                                child = 0;
                            } else if (!strcmp(udev_list_entry_get_value(props_list_entry), "SUSPEND")) {
                                //TODO
                                stop_audio_in(child);
                                printf("\nAudio Source Suspend, child: %d\n", child);
                                child = 0;
                            }
                        }
                    }
                    udev_device_unref(dev);
                } else {
                    printf("No Device from receive_device(). An error occured.\n");
                }
            }
        } else {
            printf(".");
            fflush(stdout);
        }
    }

    udev_unref(udev);
    return 0;
}
