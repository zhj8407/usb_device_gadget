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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifdef ENABLE_CALL_DISPLAY
#include <ctype.h>
#include "hid_lync_display.h"
#endif

#include "f_hidg.h"

#define BUF_LEN 512

#define MYPORT 1234

#define BACKLOG 5

#define BUF_SIZE 200

#define GET_MIC_MUTE_CMD "GetMicMute"

int fd_A[BACKLOG];
int conn_amount;

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
    {.opt = "volumeup",   .val = 0x01, .val2 = 0x01},
    {.opt = "volumedown", .val = 0x01, .val2 = 0x02},
    {.opt = "flash",      .val = 0x02, .val2 = 0x08},
    {.opt = "delete",     .val = 0x02, .val2 = 0x10},
    {.opt = "micmute",    .val = 0x02, .val2 = 0x04},
    {.opt = "hookon",     .val = 0x02, .val2 = 0x03},
    {.opt = "hookoff",    .val = 0x02, .val2 = 0x01},
    {.opt = NULL}
};

int lync_hid_fill_report(char report[8], char buf[BUF_LEN], int *hold)
{
    char *tok = strtok(buf, " ");
    int i = 0;

    if (!tok)
        return 2;

    if (strncmp(tok, "quit", 4) == 0)
        return -1;
    else if (strncmp(tok, "hold", 4) == 0) {
        *hold = 1;
        return 2;
    }

    for (i = 0; lval[i].opt != NULL; i++) {
        if (strncmp(tok, lval[i].opt, strlen(lval[i].opt)) == 0) {
            report[0] = lval[i].val;
            report[1] = lval[i].val2;
            return 2;
        }
    }

    return 0;
}

void print_options(char c)
{
    int i = 0;

    if (c == 'k') {
        printf("    keyboard options:\n"
               "        --hold\n");

        for (i = 0; kmod[i].opt != NULL; i++)
            printf("\t\t%s\n", kmod[i].opt);

        printf("\n  keyboard values:\n"
               "        [a-z] or\n");

        for (i = 0; kval[i].opt != NULL; i++)
            printf("\t\t%-8s%s", kval[i].opt, i % 2 ? "\n" : "");

        printf("\n");
    } else if (c == 'm') {
        printf("    mouse options:\n"
               "        --hold\n");

        for (i = 0; mmod[i].opt != NULL; i++)
            printf("\t\t%s\n", mmod[i].opt);

        printf("\n  mouse values:\n"
               "        Two signed numbers\n"
               "--quit to close\n");
    } else if (c == 'j') {
        printf("    joystick options:\n");

        for (i = 0; jmod[i].opt != NULL; i++)
            printf("\t\t%s\n", jmod[i].opt);

        printf("\n  joystick values:\n"
               "        three signed numbers\n"
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

#ifndef TIPS_LENGTH
#define TIPS_LENGTH 1024
#endif

char *generate_help_tips(char c)
{
    char *tips = NULL;
    int i;

    if (c != 'l')
        return NULL;

    tips = (char *)malloc(TIPS_LENGTH);

    if (!tips) {
        perror("tips buffer malloc");
        return NULL;
    }

    strncpy(tips, "    lynchid options:\n", TIPS_LENGTH);

    for (i = 0; lval[i].opt != NULL; i++) {
        strncat(tips, "\t\t", TIPS_LENGTH);
        strncat(tips, lval[i].opt, TIPS_LENGTH);
        strncat(tips, "\n", TIPS_LENGTH);
    }

    strncat(tips, "\n  lynchid commands:\n"
               "        two unsigned numbers\n"
               "--quit to close\n>", TIPS_LENGTH);

    return tips;
}

int create_socket(int port)
{
    int socket_fd;
    struct sockaddr_in server_addr;
    int yes = 1;

    if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("Error while create the socket\n");
        return -1;
    }

    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
        perror("Error while set socket opt\n");
        return -2;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));

    if (bind(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("Error while bind the socket\n");
        return -3;
    }

    if (listen(socket_fd, BACKLOG) == -1) {
        perror("Error while listen\n");
        return -4;
    }

    printf("Listening the port: %d\n", MYPORT);

    return socket_fd;
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
    int socket_fd, new_fd;
    struct sockaddr_in client_addr;
    socklen_t sin_size;
    char socket_buf[BUF_SIZE];
    int maxfd = 0;
    fd_set rfds;
    int daemon = 0;
    int retval, i;

    if (argc < 3) {
        fprintf(stderr, "Usage: %s devname mouse|keyboard|joystick|lynchid daemon\n",
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

    if ((socket_fd = create_socket(MYPORT)) <= 0) {
        perror("Error in create_socket\n");
        return 4;
    }

    if (argc >= 4 && strncmp("daemon", argv[3], 6) == 0) {
        daemon = 1;
    }

    print_options(argv[2][0]);

    conn_amount = 0;
    sin_size = sizeof(client_addr);
    maxfd = socket_fd > fd ? socket_fd : fd;
    memset(fd_A, -1, BACKLOG * sizeof(int));

#ifdef ENABLE_CALL_DISPLAY
    struct udev *udev;
    struct udev_device *dev;
    struct udev_list_entry *properties, *props_list_entry;
    struct udev_monitor *mon;
    int fd_dev;
    struct hidg_report_data hidg_report;
    int status = 0;
    udev = udev_new();
    int value;
    int length = 0;

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
        if (!daemon)
            FD_SET(STDIN_FILENO, &rfds);
        FD_SET(fd, &rfds);
		FD_SET(socket_fd, &rfds);

        for (i = 0; i < BACKLOG; i++) {
            if (fd_A[i] != -1) {
                FD_SET(fd_A[i], &rfds);
                if (fd_A[i] > maxfd)
                    maxfd = fd_A[i];
            }
        }

#ifdef ENABLE_CALL_DISPLAY
        FD_SET(fd_dev, &rfds);
        maxfd = (maxfd > fd_dev) ? maxfd : fd_dev;
#endif

        retval = select(maxfd + 1, &rfds, NULL, NULL, NULL);

        if (retval == -1 && errno == EINTR)
            continue;

        if (retval < 0) {
            perror("select()");
            return 5;
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
                    if (!strcmp(udev_list_entry_get_name(props_list_entry), "HID_EVENT")) {
                        memset(&hidg_report, 0, sizeof(struct hidg_report_data));
                        value = atoi(udev_list_entry_get_value(props_list_entry));
                        length = value & 0xFF;
                        value = value >> 8;

                        if (((value >> 8) & 0xFF) == USB_HID_ReportType_Feature &&
                                (value & 0xFF) == USBHID_VENDOR_EXT_REPORT_ID) {
                            hidg_report.length = 5;
                            hidg_report.data[0] = USBHID_VENDOR_EXT_REPORT_ID;
                            hidg_report.data[1] = USBHID_VENDOR_EXT_REPORT_VENDOR_ID & 0xff;
                            hidg_report.data[2] = (USBHID_VENDOR_EXT_REPORT_VENDOR_ID >> 8) & 0xff;
                            hidg_report.data[3] = USBHID_VENDOR_EXT_REPORT_VERSION & 0xff;
                            hidg_report.data[4] = (USBHID_VENDOR_EXT_REPORT_VERSION >> 8) & 0xff;
                        } else if (((value >> 8) & 0xFF) == USB_HID_ReportType_Feature
                                   && (value & 0xFF) == USBHID_DISP_ATTR_REPORT_ID) {
                            hidg_report.length = 5;
                            hidg_report.data[0] = USBHID_DISP_ATTR_REPORT_ID;
                            hidg_report.data[1] = USBHID_DISP_ATTR_REPORT_ROWS;
                            hidg_report.data[2] = USBHID_DISP_ATTR_REPORT_COLS;
                            hidg_report.data[3] = USBHID_DISP_ATTR_REPORT_DFSI & 0xff;
                            hidg_report.data[4] = (USBHID_DISP_ATTR_REPORT_DFSI >> 8) & 0xff;
                        } else {
                            printf("WARNING:The HID_EVENT is not processed yet!\n");
                        }

                        if (hidg_report.length > 0) {
                            status = ioctl(fd, HIDG_IOC_SEND_VENDOR_REPORT, &hidg_report);

                            if (status != 0) {
                                printf("sent ioctl message failed! status=%d\n", status);
                            }
                        }

                        printf("HID uevent value=0x%x,length=%d, response status=%d\n", value, length, status);
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

        if (!daemon && FD_ISSET(STDIN_FILENO, &rfds)) {
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
                continue;

            if (write(fd, report, to_send) != to_send) {
                perror(filename);
                return 6;
            }

            if (!hold) {
                memset(report + 1, 0x0, sizeof(report) - 1);

                if (write(fd, report, to_send) != to_send) {
                    perror(filename);
                    return 7;
                }
            }
        }

        for (i = 0; i < BACKLOG; i++) {
            if (FD_ISSET(fd_A[i], &rfds)) {
                retval = recv(fd_A[i], socket_buf, sizeof(socket_buf), 0);
                if (retval <= 0) {
                    printf("client[%d] close\n", i);
                    close(fd_A[i]);
                    FD_CLR(fd_A[i], &rfds);
                    fd_A[i] = -1;
                    conn_amount--;
                } else {
                    if (retval < BUF_SIZE)
                        memset(&socket_buf[retval], 0, 1);
                    printf("client[%d] send: %s\n", i, socket_buf);
                    hold = 0;
                    to_send = -1;
                    memset(report, 0x0, sizeof(report));

                    if (strncmp(GET_MIC_MUTE_CMD, socket_buf,
                            strlen(GET_MIC_MUTE_CMD)) == 0) {
                        if (lync_display_get_mic_mute())
                            send(fd_A[i], "Mute\n>", 6, 0);
                        else
                            send(fd_A[i], "Not Mute\n>", 10, 0);
                        break;
                    }

                    if (argv[2][0] == 'l')
                        to_send = lync_hid_fill_report(report, socket_buf, &hold);

                    if (to_send == 2) {
                        if (write(fd, report, to_send) != to_send) {
                            perror(filename);
                            send(fd_A[i], "FAIL\n>", 6, 0);
                            return 6;
                        }

                        if (!hold) {
                            memset(report + 1, 0x0, sizeof(report) - 1);

                            if (write(fd, report, to_send) != to_send) {
                                perror(filename);
                                send(fd_A[i], "FAIL\n>", 6, 0);
                                return 7;
                            }
                        }

                        send(fd_A[i], "OK\n>", 4, 0);
                        break;
                    } else if (to_send == -1) {
                        //Quit
                        send(fd_A[i], "Bye!\n", 5, 0);
                        close(fd_A[i]);
                        FD_CLR(fd_A[i], &rfds);
                        fd_A[i] = -1;
                        conn_amount--;
                        break;
                    }

                    send(fd_A[i], "Unkown Command\n>", 16, 0);
                }
            }
        }

        if (FD_ISSET(socket_fd, &rfds)) {
            new_fd = accept(socket_fd, (struct sockaddr *)&client_addr, &sin_size);
            if (new_fd < 0) {
                perror("Error while receiving the new client\n");
                continue;
            }

            if (conn_amount < BACKLOG) {
                //Find the proper place to store the new_fd
                for (i = 0; i < BACKLOG; i++) {
                    if (fd_A[i] == -1) {
                        fd_A[i] = new_fd;
                        break;
                    }
                }
                conn_amount++;
                printf("new connection client[%d] %s:%d\n", i,
                    inet_ntoa(client_addr.sin_addr),
                    ntohs(client_addr.sin_port));
                char *tips = generate_help_tips('l');
                if (tips) {
                    send(new_fd, tips, strnlen(tips, TIPS_LENGTH), 0);
                    free(tips);
                }
            } else {
                printf("max connection arrives, exit\n");
                send(new_fd, "bye", 4, 0);
                close(new_fd);
            }
        }
    }

    //close other connections
    for (i = 0; i < BACKLOG; i++) {
        if (fd_A[i] != -1)
            close(fd_A[i]);
    }

    close(socket_fd);
    close(fd);
    return 0;
}

