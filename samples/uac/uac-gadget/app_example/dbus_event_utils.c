#include "dbus_event_utils.h"

/* ------------------------------------------------------------ */

/* --------------------DBus Watch Handler---------------------- */
static void dispatch(int fd, short event, void *data)
{
    struct dbus_ctx *ctx = (struct dbus_ctx *)data;
    DBusConnection *conn = ctx->conn;

    (void)fd;
    (void)event;

    printf("dispatching\n");

    while (dbus_connection_get_dispatch_status(conn) == DBUS_DISPATCH_DATA_REMAINS)
        dbus_connection_dispatch(conn);
}

static void handle_dispatch_status(DBusConnection *conn,
                                   DBusDispatchStatus status,
                                   void *data)
{
    struct dbus_ctx *ctx = (struct dbus_ctx *)data;

    (void)conn;

    printf("New dbus dispatch status: %d\n", status);

    if (status == DBUS_DISPATCH_DATA_REMAINS) {
        struct timeval tv = {
            .tv_sec = 0,
            .tv_usec = 0,
        };
        event_add(&ctx->dispatch_ev, &tv);
    }

}

static void handle_watch(int fd, short events, void *data)
{
    struct dbus_ctx *ctx = (struct dbus_ctx *)data;
    struct DBusWatch *watch = ctx->extra;

    unsigned int flags = 0;

    if (events & EV_READ)
        flags |= DBUS_WATCH_READABLE;

    if (events & EV_WRITE)
        flags |= DBUS_WATCH_WRITABLE;

#if 0

    if (events & ERR)
        flags |= DBUS_WATCH_ERROR;

    if (events & HUP)
        flags |= DBUS_WATCH_HANGUP;

#endif

    printf("got dbus watch event fd=%d watch=%p ev=%d\n",
           fd, watch, events);

    if (dbus_watch_handle(watch, flags) == FALSE)
        printf("dbus_watch_handle() failed\n");

    handle_dispatch_status(ctx->conn, DBUS_DISPATCH_DATA_REMAINS, ctx);
}

static dbus_bool_t add_watch(DBusWatch *w, void *data)
{
    if (!dbus_watch_get_enabled(w))
        return TRUE;

    struct dbus_ctx *ctx = (struct dbus_ctx *)data;
    ctx->extra = w;

    int fd = dbus_watch_get_unix_fd(w);
    unsigned int flags = dbus_watch_get_flags(w);
    short cond = EV_PERSIST;

    if (flags & DBUS_WATCH_READABLE)
        cond |= EV_READ;

    if (flags & DBUS_WATCH_WRITABLE)
        cond |= EV_WRITE;

#if 0

    if (flags & DBUS_WATCH_ERROR)
        cond |= EV_ERR;

    if (flags & DBUS_WATCH_HANGUP)
        cond |= EV_HUP;

#endif
    //TODO

    struct event *event = event_new(ctx->ev_base, fd, cond, handle_watch, ctx);

    if (!event)
        return FALSE;

    event_add(event, NULL);

    dbus_watch_set_data(w, event, NULL);

    printf("added bus watch fd=%d watch=%p cond=%u\n", fd, w, cond);

    return TRUE;
}

static void remove_watch(DBusWatch *w, void *data)
{
    struct event *event = dbus_watch_get_data(w);

    (void)data;

    if (event)
        event_free(event);

    dbus_watch_set_data(w, NULL, NULL);

    printf("removed dbus watch watch=%p\n", w);
}

static void toggle_watch(DBusWatch *w, void *data)
{
    printf(" WATCH:    toggle dbus watch watch=%p\n", w);

    if (dbus_watch_get_enabled(w))
        add_watch(w, data);
    else
        remove_watch(w, data);
}

/* --------------------DBus Timeout Handler---------------------- */
static void handle_timeout(int fd, short events, void *data)
{
    DBusTimeout *t = (DBusTimeout *)data;

    (void)fd;
    (void)events;

    printf("Got dbus handle timeout event %p\n", t);

    dbus_timeout_handle(t);
}

static dbus_bool_t add_timeout(DBusTimeout *t, void *data)
{
    struct dbus_ctx *ctx = (struct dbus_ctx *)data;

    if (!dbus_timeout_get_enabled(t))
        return TRUE;

    printf("adding timeout %p\n", t);

    struct event *event = event_new(ctx->ev_base, -1, EV_TIMEOUT | EV_PERSIST,
                                    handle_timeout, t);

    if (!event) {
        printf("Failed to allocate new event for timeout\n");
        return FALSE;
    }

    int ms = dbus_timeout_get_interval(t);
    struct timeval tv = {
        .tv_sec = ms / 1000,
        .tv_usec = (ms % 1000) * 1000,
    };

    event_add(event, &tv);

    dbus_timeout_set_data(t, event, NULL);

    return TRUE;
}

static void remove_timeout(DBusTimeout *t, void *data)
{
    struct event *event = dbus_timeout_get_data(t);

    (void)data;

    if (event)
        event_free(event);

    dbus_timeout_set_data(t, NULL, NULL);

    printf("Removed timeout %p\n", t);
}

static void toggle_timeout(DBusTimeout *t, void *data)
{
    printf("toggling timeout %p\n", t);

    if (dbus_timeout_get_enabled(t))
        add_timeout(t, data);
    else
        remove_timeout(t, data);
}

int setup_dbus_event(struct dbus_ctx **ctx,
                     const char *wellknown_name,
                     const char *msg_match,
                     DBusHandleMessageFunction msg_filter,
                     struct event_base *ev_base)
{
    DBusError error;
    int ret = 0;

    *ctx = NULL;
    *ctx = (struct dbus_ctx *)calloc(1, sizeof(struct dbus_ctx));

    if (!*ctx) {
        fprintf(stderr, "Failed to allocate the dbus_ctx\n");
        ret = -ENOMEM;
        goto out;
    }

    (*ctx)->conn = dbus_bus_get_private(DBUS_BUS_SYSTEM, NULL);

    if ((*ctx)->conn == NULL) {
        fprintf(stderr, "Failed to allocate the dbus connection\n");
        ret = -ENOMEM;
        goto out;
    }

    dbus_connection_set_exit_on_disconnect((*ctx)->conn, FALSE);

    dbus_error_init(&error);

    if (wellknown_name) {
        ret = dbus_bus_request_name((*ctx)->conn, wellknown_name,
                                    DBUS_NAME_FLAG_REPLACE_EXISTING, &error);

        if (dbus_error_is_set(&error)) {
            fprintf(stderr, "Failed to request name %s, Err: %s\n",
                    wellknown_name, error.message);
            dbus_error_free(&error);
            ret = -ENOMEM;
            goto out;
        }

        if (ret != DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER) {
            ret = -EINVAL;
            goto out;
        }

        ret = 0;
    }

    (*ctx)->ev_base = ev_base;
    event_assign(&((*ctx)->dispatch_ev), ev_base, -1, EV_TIMEOUT, dispatch, (*ctx));

    if (!dbus_connection_set_watch_functions((*ctx)->conn, add_watch, remove_watch, toggle_watch, (*ctx), NULL)) {
        fprintf(stderr, "Failed to set watch functions\n");
        ret = -ENOMEM;
        goto out;
    }

    if (!dbus_connection_set_timeout_functions((*ctx)->conn, add_timeout, remove_timeout, toggle_timeout, (*ctx), NULL)) {
        fprintf(stderr, "Failed to set timeout functions\n");
        ret = -ENOMEM;
        goto out;
    }

    if (!dbus_connection_add_filter((*ctx)->conn, msg_filter, (*ctx), NULL)) {
        fprintf(stderr, "Failed to add filter\n");
        ret = -ENOMEM;
        goto out;
    }

    dbus_connection_set_dispatch_status_function((*ctx)->conn, handle_dispatch_status, (*ctx), NULL);

    dbus_bus_add_match((*ctx)->conn, msg_match, &error);

    if (dbus_error_is_set(&error)) {
        fprintf(stderr, "dbus_bus_add_match %s failed: %s\n",
                msg_match, error.message);
        ret = -ENOMEM;
        dbus_error_free(&error);
        goto out;
    }

    dbus_connection_flush((*ctx)->conn);

    return ret;

out:

    if (*ctx) {
        if ((*ctx)->conn) {
            dbus_connection_close((*ctx)->conn);
            dbus_connection_unref((*ctx)->conn);
        }

        free(*ctx);
    }

    return ret;
}

void cleanup_dbus_event(struct dbus_ctx *ctx)
{
    if (ctx) {
        if (ctx->conn) {
            dbus_connection_flush(ctx->conn);
            dbus_connection_close(ctx->conn);
            dbus_connection_unref(ctx->conn);
            event_del(&ctx->dispatch_ev);
        }

        free(ctx);
    }
}

