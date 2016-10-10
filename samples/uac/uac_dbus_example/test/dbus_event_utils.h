#ifndef DBUS_EVENT_UTILS_H_INCLUDED
#define DBUS_EVENT_UTILS_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <sys/time.h>
#include <sys/select.h>

#include <dbus/dbus.h>
#include <event.h>

struct dbus_ctx {
    DBusConnection *conn;
    struct event_base *ev_base;
    struct event dispatch_ev;
    void *extra;
};

int setup_dbus_event(struct dbus_ctx **ctx,
                     const char *wellknown_name,
                     const char *msg_match,
                     DBusHandleMessageFunction msg_filter,
                     struct event_base *ev_base);

void cleanup_dbus_event(struct dbus_ctx *ctx);

#endif // DBUS_EVENT_UTILS_H_INCLUDED
