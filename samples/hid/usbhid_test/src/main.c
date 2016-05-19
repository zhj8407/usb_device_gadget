#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <libudev.h>
#include <linux/fs.h>
#include <fcntl.h>

#ifdef ENABLE_CALL_DISPLAY
#include <ctype.h>
#include "hid_lync_display.h"
#endif

#define BUF_LEN 512
struct options {
    const char    *opt;
    unsigned char val;
    unsigned char val2;
};

static struct options kmod[] = {
    {.opt = "--left-ctrl",      .val = 0x01},
    {.opt = "--right-ctrl",     .val = 0x10},
    {.opt = "--left-shift",     .val = 0x02},
    {.opt = "--right-shift",    .val = 0x20},
    {.opt = "--left-alt",       .val = 0x04},
    {.opt = "--right-alt",      .val = 0x40},
    {.opt = "--left-meta",      .val = 0x08},
    {.opt = "--right-meta",     .val = 0x80},
    {.opt = NULL}
};

static struct options kval[] = {
    {.opt = "--return", .val = 0x28},
    {.opt = "--esc",    .val = 0x29},
    {.opt = "--bckspc", .val = 0x2a},
    {.opt = "--tab",    .val = 0x2b},
    {.opt = "--spacebar",   .val = 0x2c},
    {.opt = "--caps-lock",  .val = 0x39},
    {.opt = "--f1",     .val = 0x3a},
    {.opt = "--f2",     .val = 0x3b},
    {.opt = "--f3",     .val = 0x3c},
    {.opt = "--f4",     .val = 0x3d},
    {.opt = "--f5",     .val = 0x3e},
    {.opt = "--f6",     .val = 0x3f},
    {.opt = "--f7",     .val = 0x40},
    {.opt = "--f8",     .val = 0x41},
    {.opt = "--f9",     .val = 0x42},
    {.opt = "--f10",    .val = 0x43},
    {.opt = "--f11",    .val = 0x44},
    {.opt = "--f12",    .val = 0x45},
    {.opt = "--insert", .val = 0x49},
    {.opt = "--home",   .val = 0x4a},
    {.opt = "--pageup", .val = 0x4b},
    {.opt = "--del",    .val = 0x4c},
    {.opt = "--end",    .val = 0x4d},
    {.opt = "--pagedown",   .val = 0x4e},
    {.opt = "--right",  .val = 0x4f},
    {.opt = "--left",   .val = 0x50},
    {.opt = "--down",   .val = 0x51},
    {.opt = "--kp-enter",   .val = 0x58},
    {.opt = "--up",     .val = 0x52},
    {.opt = "--num-lock",   .val = 0x53},
    {.opt = NULL}
};

int keyboard_fill_report(char report[8], char buf[BUF_LEN], int *hold)
{
    char *tok = strtok(buf, " ");
    int key = 0;
    int i = 0;

    for (; tok != NULL; tok = strtok(NULL, " ")) {
        if (strcmp(tok, "--quit") == 0)
            return -1;

        if (strcmp(tok, "--hold") == 0) {
            *hold = 1;
            continue;
        }

        if (key < 6) {
            for (i = 0; kval[i].opt != NULL; i++)
                if (strcmp(tok, kval[i].opt) == 0) {
                    report[2 + key++] = kval[i].val;
                    break;
                }

            if (kval[i].opt != NULL)
                continue;
        }

        if (key < 6)
            if (islower(tok[0])) {
                report[2 + key++] = (tok[0] - ('a' - 0x04));
                continue;
            }

        for (i = 0; kmod[i].opt != NULL; i++)
            if (strcmp(tok, kmod[i].opt) == 0) {
                report[0] = report[0] | kmod[i].val;
                break;
            }

        if (kmod[i].opt != NULL)
            continue;

        if (key < 6)
            fprintf(stderr, "unknown option: %s\n", tok);
    }

    return 8;
}

static struct options mmod[] = {
    {.opt = "--b1", .val = 0x01},
    {.opt = "--b2", .val = 0x02},
    {.opt = "--b3", .val = 0x04},
    {.opt = NULL}
};

int mouse_fill_report(char report[8], char buf[BUF_LEN], int *hold)
{
    char *tok = strtok(buf, " ");
    int mvt = 0;
    int i = 0;

    for (; tok != NULL; tok = strtok(NULL, " ")) {
        if (strcmp(tok, "--quit") == 0)
            return -1;

        if (strcmp(tok, "--hold") == 0) {
            *hold = 1;
            continue;
        }

        for (i = 0; mmod[i].opt != NULL; i++)
            if (strcmp(tok, mmod[i].opt) == 0) {
                report[0] = report[0] | mmod[i].val;
                break;
            }

        if (mmod[i].opt != NULL)
            continue;

        if (!(tok[0] == '-' && tok[1] == '-') && mvt < 2) {
            errno = 0;
            report[1 + mvt++] = (char)strtol(tok, NULL, 0);

            if (errno != 0) {
                fprintf(stderr, "Bad value:'%s'\n", tok);
                report[1 + mvt--] = 0;
            }

            continue;
        }

        fprintf(stderr, "unknown option: %s\n", tok);
    }

    return 3;
}

static struct options jmod[] = {
    {.opt = "--b1",     .val = 0x10},
    {.opt = "--b2",     .val = 0x20},
    {.opt = "--b3",     .val = 0x40},
    {.opt = "--b4",     .val = 0x80},
    {.opt = "--hat1",   .val = 0x00},
    {.opt = "--hat2",   .val = 0x01},
    {.opt = "--hat3",   .val = 0x02},
    {.opt = "--hat4",   .val = 0x03},
    {.opt = "--hatneutral", .val = 0x04},
    {.opt = NULL}
};

int joystick_fill_report(char report[8], char buf[BUF_LEN], int *hold)
{
    char *tok = strtok(buf, " ");
    int mvt = 0;
    int i = 0;
    *hold = 1;
    /* set default hat position: neutral */
    report[3] = 0x04;

    for (; tok != NULL; tok = strtok(NULL, " ")) {
        if (strcmp(tok, "--quit") == 0)
            return -1;

        for (i = 0; jmod[i].opt != NULL; i++)
            if (strcmp(tok, jmod[i].opt) == 0) {
                report[3] = (report[3] & 0xF0) | jmod[i].val;
                break;
            }

        if (jmod[i].opt != NULL)
            continue;

        if (!(tok[0] == '-' && tok[1] == '-') && mvt < 3) {
            errno = 0;
            report[mvt++] = (char)strtol(tok, NULL, 0);

            if (errno != 0) {
                fprintf(stderr, "Bad value:'%s'\n", tok);
                report[mvt--] = 0;
            }

            continue;
        }

        fprintf(stderr, "unknown option: %s\n", tok);
    }

    return 4;
}

static struct options lval[] = {
    {.opt = "--volumeup",   .val = 0x01, .val2 = 0x01},
    {.opt = "--volumedown", .val = 0x01, .val2 = 0x02},
    {.opt = "--flash",      .val = 0x02, .val2 = 0x08},
    {.opt = "--delete",     .val = 0x02, .val2 = 0x10},
    {.opt = "--micmute",    .val = 0x02, .val2 = 0x04},
    {.opt = "--hookon",     .val = 0x02, .val2 = 0x03},
    {.opt = "--hookoff",    .val = 0x02, .val2 = 0x01},
    {.opt = NULL}
};

int lync_hid_fill_report(char report[8], char buf[BUF_LEN], int *hold)
{
    char *tok = strtok(buf, " ");
    int i = 0;

    if (!tok)
        return 2;

    if (strcmp(tok, "--quit") == 0)
        return -1;
    else if (strcmp(tok, "--hold") == 0) {
        *hold = 1;
        return 2;
    }

    for (i = 0; lval[i].opt != NULL; i++) {
        if (strcmp(tok, lval[i].opt) == 0) {
            report[0] = lval[i].val;
            report[1] = lval[i].val2;
            break;
        }
    }

    return 2;
}

void print_options(char c)
{
    int i = 0;

    if (c == 'k') {
        printf("	keyboard options:\n"
               "		--hold\n");

        for (i = 0; kmod[i].opt != NULL; i++)
            printf("\t\t%s\n", kmod[i].opt);

        printf("\n	keyboard values:\n"
               "		[a-z] or\n");

        for (i = 0; kval[i].opt != NULL; i++)
            printf("\t\t%-8s%s", kval[i].opt, i % 2 ? "\n" : "");

        printf("\n");
    } else if (c == 'm') {
        printf("	mouse options:\n"
               "		--hold\n");

        for (i = 0; mmod[i].opt != NULL; i++)
            printf("\t\t%s\n", mmod[i].opt);

        printf("\n	mouse values:\n"
               "		Two signed numbers\n"
               "--quit to close\n");
    } else if (c == 'j') {
        printf("	joystick options:\n");

        for (i = 0; jmod[i].opt != NULL; i++)
            printf("\t\t%s\n", jmod[i].opt);

        printf("\n	joystick values:\n"
               "		three signed numbers\n"
               "--quit to close\n");
    } else if (c == 'l') {
        printf("    lynchid options:\n");

        for (i = 0; lval[i].opt != NULL; i++)
            printf("\t\t%s\n", lval[i].opt);

        printf("\n  lynchid commands:\n"
               "        two unsigned numbers\n"
               "--quit to close\n");
    }
}

int main(int argc, const char *argv[])
{
    const char *filename = NULL;
    int fd = 0;
    char buf[BUF_LEN];
    int cmd_len;
    char report[8];
    int to_send = 8;
    int hold = 0;
    fd_set rfds;
    int retval, i;
    int fd_max = 0;

    if (argc < 3) {
        fprintf(stderr, "Usage: %s devname mouse|keyboard|joystick|lynchid\n",
                argv[0]);
        return 1;
    }

    if (argv[2][0] != 'k' && argv[2][0] != 'm' && argv[2][0] != 'j' && argv[2][0] != 'l')
        return 2;

    filename = argv[1];

    if ((fd = open(filename, O_RDWR, 0666)) == -1) {
        perror(filename);
        return 3;
    }

    print_options(argv[2][0]);

#ifdef ENABLE_CALL_DISPLAY
    struct udev *udev;
    struct udev_device *dev;
    struct udev_list_entry *properties, *props_list_entry;
    struct udev_monitor *mon;
    int fd_dev;
    char hid_cmd[8];
    int status = 0;
    udev = udev_new();

    if (!udev) {
        printf("Can't create udev\n");
        exit(1);
    }

    mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(mon, "plcm_usb", NULL);
    udev_monitor_enable_receiving(mon);
    fd_dev = udev_monitor_get_fd(mon);
#endif

    while (42) {
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(fd, &rfds);
        fd_max = (fd + 1 > STDIN_FILENO + 1) ? fd + 1 : STDIN_FILENO + 1;
#ifdef ENABLE_CALL_DISPLAY
        FD_SET(fd_dev, &rfds);
        fd_max = (fd_max > fd_dev + 1) ? fd_max : fd_dev + 1;
#endif
        retval = select(fd_max, &rfds, NULL, NULL, NULL);

        if (retval == -1 && errno == EINTR)
            continue;

        if (retval < 0) {
            perror("select()");
            return 4;
        }

#ifdef ENABLE_CALL_DISPLAY

        if (FD_ISSET(fd_dev, &rfds)) {
            /* Make the call to receive the device.
               select() ensured that this will not block. */
            dev = udev_monitor_receive_device(mon);

            if (dev) {

                if (strcmp(udev_device_get_subsystem(dev), "plcm_usb") ||
                        strcmp(udev_device_get_syspath(dev), "/sys/devices/virtual/plcm_usb/plcm0/f_hidg") ||
                        strcmp(udev_device_get_action(dev), "change")) {
                    udev_device_unref(dev);
                    continue;
                }

                properties = udev_device_get_properties_list_entry(dev);
                udev_list_entry_foreach(props_list_entry, properties) {
                    if (!strcmp(udev_list_entry_get_name(props_list_entry), "HID_EVENT1")) {
                        if (!strcmp(udev_list_entry_get_value(props_list_entry), "HID_VENDOR_EXT")) {
                            hid_cmd[0] = USBHID_VENDOR_EXT_REPORT_ID;
                            hid_cmd[1] = USBHID_VENDOR_EXT_REPORT_VENDOR_ID & 0xff;
                            hid_cmd[2] = (USBHID_VENDOR_EXT_REPORT_VENDOR_ID >> 8) & 0xff;
                            hid_cmd[3] = USBHID_VENDOR_EXT_REPORT_VERSION & 0xff;
                            hid_cmd[4] = (USBHID_VENDOR_EXT_REPORT_VERSION >> 8) & 0xff;

                            status = ioctl(fd, &hid_cmd, 5);
                            printf("\n HID uEVENT IS RECEIVED status=%d\n", status);
                        }
                    }

                    if (!strcmp(udev_list_entry_get_name(props_list_entry), "HID_EVENT2")) {
                        if (!strcmp(udev_list_entry_get_value(props_list_entry), "HID_DISP_ATTR")) {
                            hid_cmd[0] = USBHID_DISP_ATTR_REPORT_ID;
                            hid_cmd[1] = USBHID_DISP_ATTR_REPORT_ROWS;
                            hid_cmd[2] = USBHID_DISP_ATTR_REPORT_COLS;
                            hid_cmd[3] = USBHID_DISP_ATTR_REPORT_DFSI & 0xff;
                            hid_cmd[4] = (USBHID_DISP_ATTR_REPORT_DFSI >> 8) & 0xff;
                            status = ioctl(fd, &hid_cmd, 5);
                            printf("\n HID uEVENT IS RECEIVED status=%d\n", status);
                        }
                    }
                }
                udev_device_unref(dev);
            } else {
                printf("No Device from receive_device(). An error occured.\n");
            }
        } else {
            printf(".");
            fflush(stdout);
        }

#endif

        if (FD_ISSET(fd, &rfds)) {
            cmd_len = read(fd, buf, BUF_LEN - 1);
            printf("\nrecv report:");

            for (i = 0; i < cmd_len; i++)
                printf(" %02x", buf[i]);

            printf("\n");

            if (argv[2][0] == 'l' && cmd_len > 0)
                lync_display_process_set_report((unsigned char *)buf, (unsigned int)cmd_len);
        }

        if (FD_ISSET(STDIN_FILENO, &rfds)) {
            memset(report, 0x0, sizeof(report));
            cmd_len = read(STDIN_FILENO, buf, BUF_LEN - 1);

            if (cmd_len == 0)
                break;

            buf[cmd_len - 1] = '\0';
            hold = 0;
            memset(report, 0x0, sizeof(report));

            if (argv[2][0] == 'k')
                to_send = keyboard_fill_report(report, buf, &hold);
            else if (argv[2][0] == 'm')
                to_send = mouse_fill_report(report, buf, &hold);
            else if (argv[2][0] == 'j')
                to_send = joystick_fill_report(report, buf, &hold);
            else if (argv[2][0] == 'l')
                to_send = lync_hid_fill_report(report, buf, &hold);

            if (to_send == -1)
                break;

            if (write(fd, report, to_send) != to_send) {
                perror(filename);
                return 5;
            }

            if (!hold) {
                memset(report + 1, 0x0, sizeof(report) - 1);

                if (write(fd, report, to_send) != to_send) {
                    perror(filename);
                    return 6;
                }
            }
        }
    }

    close(fd);
    return 0;
}

