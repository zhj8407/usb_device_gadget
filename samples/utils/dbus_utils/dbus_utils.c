#include "dbus_utils.h"

/* ------------------------------------------------------------ */

/**
 * for debug purpose
 */
char debug_msg[1024];

char* _verbose_watch(DBusWatch *watch)
{
    char *s_flags[] = {
        "readable",
        "writable"
    };
    char p[1024] = "";

    if (dbus_watch_get_flags(watch) & DBUS_WATCH_READABLE)
        strncpy(p, s_flags[0], strlen(s_flags[0]));
    else if (dbus_watch_get_flags(watch) & DBUS_WATCH_WRITABLE) {
        if (p[0])
            strncat(p, "&", strlen("&"));

        strncat(p, s_flags[1], strlen(s_flags[1]));
    }

    sprintf(debug_msg, "%d:%s", dbus_watch_get_unix_fd(watch), p);
    return debug_msg;
}

char* _verbose_message(DBusMessage *msg)
{
    char s_msg[1024] = "";
    int bc = sprintf(s_msg, "\ttype: %s\n\tpath: %s\n"
                     "\tmember: %s\n\t",
                     dbus_message_type_to_string(dbus_message_get_type(msg)),
                     dbus_message_get_path(msg),
                     dbus_message_get_member(msg));
    strncpy(debug_msg, s_msg, bc + 1);

    if (dbus_message_get_serial(msg)) {
        bc = sprintf(s_msg, "serial: %ud\n\t",
                     dbus_message_get_serial(msg));
        strncat(debug_msg, s_msg, bc + 1);
    }

    if (dbus_message_get_reply_serial(msg)) {
        bc = sprintf(s_msg, "reply_serial: %ud\n\t",
                     dbus_message_get_reply_serial(msg));
        strncat(debug_msg, s_msg, bc + 1);
    }

    DBusMessageIter args, subargs;
    char *s;
    int i;
    dbus_message_iter_init(msg, &args);
    bc = sprintf(s_msg, "args: ");
    strncat(debug_msg, s_msg, bc + 1);

    /** here is not the best way to parse the params, i just want to test some of
     * nested situations and different types of params. */
    while (DBUS_TYPE_INVALID != dbus_message_iter_get_arg_type(&args)) {
        // demo here: only care about int and string, igore other types

        switch (dbus_message_iter_get_arg_type(&args)) {
            case DBUS_TYPE_STRING:
                dbus_message_iter_get_basic(&args, &s);
                bc = sprintf(s_msg, " %s", s);
                strncat(debug_msg, s_msg, bc + 1);
                break;

            case DBUS_TYPE_INT32:
                dbus_message_iter_get_basic(&args, &i);
                bc = sprintf(s_msg, " %d", i);
                strncat(debug_msg, s_msg, bc + 1);
                break;

            case DBUS_TYPE_ARRAY:
                dbus_message_iter_recurse(&args, &subargs);
                strcat(debug_msg, " [ ");

                while (dbus_message_iter_get_arg_type(&subargs)
                        != DBUS_TYPE_INVALID) {
                    switch (dbus_message_iter_get_arg_type(&subargs)) {
                        case DBUS_TYPE_STRING:
                            dbus_message_iter_get_basic(&subargs, &s);
                            bc = sprintf(s_msg, " %s", s);
                            strncat(debug_msg, s_msg, bc + 1);
                            break;

                        case DBUS_TYPE_INT32:
                            dbus_message_iter_get_basic(&subargs, &i);
                            bc = sprintf(s_msg, " %d", i);
                            strncat(debug_msg, s_msg, bc + 1);
                            break;
                    }

                    dbus_message_iter_next(&subargs);
                }

                strcat(debug_msg, " ] ");
        }

        dbus_message_iter_next(&args);
    }

    return debug_msg;
}

char* _verbose_timeout(DBusTimeout *timeout)
{
    sprintf(debug_msg, "timeout %p<#%x>: \n",
            timeout, dbus_timeout_get_interval(timeout));
    return debug_msg;
}

/* watch */

struct watchlist_t {
    DBusWatch *watch;
    struct watchlist_t *next;
};

static struct watchlist_t *watchlist = NULL;

static dbus_bool_t add_watch(DBusWatch *w, void *data)
{
    (void)data;
    struct watchlist_t *l;

    if (!dbus_watch_get_enabled(w))
        return TRUE;

    for (l = watchlist; l != NULL; l = l->next) {
        if (l->watch == w)
            return TRUE;
    }

    l = dbus_new(struct watchlist_t, 1);

    if (l == NULL)
        return FALSE;

    l->watch = w;
    l->next = watchlist;
    watchlist = l;

    return TRUE;
}
static void remove_watch(DBusWatch *w, void *data)
{
    (void)data;
    struct watchlist_t *l, *pre;

    for (pre = l = watchlist; l != NULL; pre = l, l = l->next) {
        if (l->watch == w) {
            printf("remove_watch: %d\n", dbus_watch_get_unix_fd(l->watch));

            if (l == watchlist) {
                watchlist = l->next;
                dbus_free(l);
            } else {
                pre->next = l->next;
                dbus_free(l);
            }

            break;
        }
    }

    printf(" WATCH:    remove dbus watch watch=%p\n", w);
}

static void toggle_watch(DBusWatch *w, void *data)
{
    printf(" WATCH:    toggle dbus watch watch=%p\n", w);

    if (dbus_watch_get_enabled(w))
        add_watch(w, data);
    else
        remove_watch(w, data);
}

/* timeout */
/**---- timeout process functions ------------------------------*/

static struct timeoutlist_t {
    DBusTimeout *timeout;
    struct timeoutlist_t *next;
} *timeoutlist = NULL;

static dbus_bool_t add_timeout(DBusTimeout *timeout, void *data)
{
    (void)data;
    struct timeoutlist_t *l;

    for (l = timeoutlist; l != NULL; l = l->next) {
        if (l->timeout == timeout)
            return TRUE;
    }

    l = dbus_new(struct timeoutlist_t, 1);

    if (NULL == l)
        return FALSE;

    l->timeout = timeout;
    fprintf(stdout, "timeoutAdd:%s\n", _verbose_timeout(timeout));
    l->next = timeoutlist;
    timeoutlist = l;
    return TRUE;
}

static void remove_timeout(DBusTimeout *timeout, void *data)
{
    (void)data;
    struct timeoutlist_t *pre = NULL, *l = timeoutlist;

    while (l != NULL) {
        if (l->timeout == timeout) {
            if (pre == NULL)
                timeoutlist = l->next;
            else
                pre->next = l->next;

            fprintf(stdout, "timeoutRemove:%s\n",
                    _verbose_timeout(timeout));
            break;
        }

        pre = l;
        l = l->next;
    }
}

static void toggle_timeout(DBusTimeout *timeout, void *data)
{
    fprintf(stdout, "timeoutToggle: %s\n", _verbose_timeout(timeout));

    if (dbus_timeout_get_enabled(timeout))
        add_timeout(timeout, data);
    else
        remove_timeout(timeout, data);
}

/**
 * in this function, we call dbus_timeout_handle to handle all timeout
 * events, it will call internal predefined handler to process.
 */
void dbus_handle_all_timeout()
{
    struct timeoutlist_t *l = timeoutlist;

    for (; l != NULL; l = l->next) {
        if (dbus_timeout_get_enabled(l->timeout)) {
            printf("dbus_handle_all_timeout: %s\n", _verbose_timeout(l->timeout));
            dbus_timeout_handle(l->timeout);
        }
    }
}

void dbus_handle_all_watches(DBusConnection *conn)
{
    DBusDispatchStatus dispatch_rc = dbus_connection_get_dispatch_status(conn);

    if (DBUS_DISPATCH_DATA_REMAINS != dispatch_rc) {
        printf(" ERROR recv no message in queue \n");
    }

    while (DBUS_DISPATCH_DATA_REMAINS == dispatch_rc) {
        dbus_connection_dispatch(conn);
        dispatch_rc = dbus_connection_get_dispatch_status(conn);
    }
}

DBusHandlerResult handle_method_return(DBusConnection *conn,
                                       DBusMessage *reply)
{
    (void)conn;
    (void)reply;
    return DBUS_HANDLER_RESULT_HANDLED;
}


static dbus_bool_t dbus_setup_watch_functions(DBusConnection *conn)
{
    return dbus_connection_set_watch_functions(conn, add_watch, remove_watch, toggle_watch,
            NULL, NULL);
}

static dbus_bool_t dbus_setup_timeout_functions(DBusConnection *conn)
{
    return dbus_connection_set_timeout_functions(conn, add_timeout, remove_timeout, toggle_timeout,
            NULL, NULL);
}

int dbus_setup_listen_fds(fd_set *rfds, fd_set *wfds, fd_set *efds)
{
    struct watchlist_t *l;
    int maxfd = -1, fd;
    unsigned int flags;

    // prepare all readable and writable fds
    for (l = watchlist; l != NULL; l = l->next) {

        fd = dbus_watch_get_unix_fd(l->watch);
        flags = dbus_watch_get_flags(l->watch);

        if (flags & DBUS_WATCH_READABLE) {
            FD_SET(fd, rfds);
            maxfd = (maxfd < fd ? fd : maxfd);
        }

        if (flags & DBUS_WATCH_WRITABLE) {
            FD_SET(fd, wfds);
            maxfd = (maxfd < fd ? fd : maxfd);
        }

        if (flags & DBUS_WATCH_ERROR) {
            FD_SET(fd, efds);
            maxfd = (maxfd < fd ? fd : maxfd);
        }
    }

    return maxfd;
}

void dbus_handle_listen_fds(fd_set *rfds, fd_set *wfds, fd_set *efds)
{
    struct watchlist_t *l;
    int fd;

    for (l = watchlist; l != NULL; l = l->next) {
        fd = dbus_watch_get_unix_fd(l->watch);

        if (FD_ISSET(fd, rfds))
            dbus_watch_handle(l->watch, DBUS_WATCH_READABLE);

        if (FD_ISSET(fd, wfds))
            dbus_watch_handle(l->watch, DBUS_WATCH_WRITABLE);

        if (FD_ISSET(fd, efds))
            dbus_watch_handle(l->watch, DBUS_WATCH_ERROR);
    }

}

int dbus_setup_connection(DBusConnection **conn,
                          const char *wellknown_name,
                          const char *msg_match)
{
    DBusError err;
    int ret;

    dbus_error_init(&err);
    *conn = dbus_bus_get_private(DBUS_BUS_SYSTEM, &err);

    if (dbus_error_is_set(&err)) {
        fprintf(stderr, "ConnectionError %s\n", err.message);
        dbus_error_free(&err);
    }

    if (*conn == NULL) {
        ret = -ENOMEM;
        goto out;
    }

    if (wellknown_name) {
        ret = dbus_bus_request_name(*conn, wellknown_name,
                                    DBUS_NAME_FLAG_REPLACE_EXISTING, &err);

        if (dbus_error_is_set(&err)) {
            fprintf(stderr, "Name Error%s\n", err.message);
            dbus_error_free(&err);
        }

        if (ret != DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER) {
            ret = -EINVAL;
            goto out;
        }

        ret = 0;
    }

    if (msg_match) {
        dbus_bus_add_match(*conn,
                           msg_match,
                           &err);

        dbus_connection_flush(*conn);

        if (dbus_error_is_set(&err)) {
            fprintf(stderr, "add match failed.\n");
            dbus_error_free(&err);
            ret = -ENOMEM;
            goto out;
        }
    }

    if (!dbus_setup_watch_functions(*conn)) {
        fprintf(stderr, "Failed to setup the watch functions\n");
        ret = -ENOMEM;
        goto out;
    }

    if (!dbus_setup_timeout_functions(*conn)) {
        fprintf(stderr, "Failed to setup the timeout functions\n");
        ret = -ENOMEM;
        goto out;
    }

    return 0;

out:

    if (*conn) {
        dbus_connection_close(*conn);
        dbus_connection_unref(*conn);
    }

    return ret;
}

void dbus_cleanup_connection(DBusConnection *conn)
{
    if (conn != NULL) {
        dbus_connection_flush(conn);
        dbus_connection_close(conn);
        dbus_connection_unref(conn);
    }
}

int dbus_register_message_filter(DBusConnection *conn, DBusHandleMessageFunction function)
{
    return dbus_connection_add_filter(conn, function, NULL, NULL);
}

int dbus_register_object_patch(DBusConnection *conn, char *path[], DBusObjectPathVTable *vtable, unsigned int size)
{
    unsigned int i = 0;
    int count = 0;
    dbus_bool_t ret;

    for (i = 0; i < size; i++) {
        ret = dbus_connection_register_object_path(conn,
                path[i], &vtable[i], NULL);

        if (ret)
            count++;
    }

    return count;
}

int dbus_read_params_from_msg(DBusMessage *msg, int first_arg_type, ...)
{
    DBusMessageIter args;
    DBusError err;
    va_list arg_ptr;

    if (!dbus_message_iter_init(msg, &args)) {
        fprintf(stderr, "init arg error\n");
        return -ENOMEM;
    }

    dbus_error_init(&err);

    va_start(arg_ptr, first_arg_type);

    dbus_message_get_args_valist(msg, &err,
                                 first_arg_type,
                                 arg_ptr);

    va_end(arg_ptr);

    if (dbus_error_is_set(&err)) {
        fprintf(stderr, "get arg error");
        return -EINVAL;
    }

    return 0;
}

int dbus_send_method_call_reply_with_results(DBusConnection *conn,
        DBusMessage *msg, int first_arg_type, ...)
{
    DBusMessage *reply;
    DBusMessageIter args;
    va_list arg_ptr;
    dbus_uint32_t serial;
    dbus_bool_t retval;

    reply = dbus_message_new_method_return(msg);

    if (NULL == reply) {
        fprintf(stderr, "Memory is not enough.\n");
        return -ENOMEM;
    }

    dbus_message_iter_init_append(reply, &args);

    va_start(arg_ptr, first_arg_type);

    retval = dbus_message_append_args_valist(reply,
             first_arg_type,
             arg_ptr);

    va_end(arg_ptr);

    if (!retval) {
        fprintf(stderr, "Can not append the args\n");
        dbus_message_unref(reply);
        return -EINVAL;
    }

    if (!dbus_connection_send(conn, reply, &serial)) {
        fprintf(stderr, "Out of Memory in send!\n");
        dbus_message_unref(msg);
        return -ENOMEM;
    }

    dbus_connection_flush(conn);

    printf("build reply msg and send: \n\t%s\n", _verbose_message(reply));
    dbus_message_unref(reply);

    return 0;
}

int dbus_send_signal_with_params(DBusConnection *conn,
                                 const char *object_patch,
                                 const char *interface,
                                 const char *name,
                                 int first_arg_type, ...)
{
    DBusMessage * msg;
    DBusMessageIter arg;
    va_list arg_ptr;
    dbus_uint32_t serial;
    dbus_bool_t retval;

    if ((msg = dbus_message_new_signal(object_patch, interface, name)) == NULL) {
        fprintf(stderr, "MessageNULL\n");
        return -ENOMEM;
    }

    dbus_message_iter_init_append(msg, &arg);

    va_start(arg_ptr, first_arg_type);

    retval = dbus_message_append_args_valist(msg,
             first_arg_type,
             arg_ptr);

    va_end(arg_ptr);

    if (!retval) {
        fprintf(stderr, "Can not append the args\n");
        dbus_message_unref(msg);
        return -EINVAL;
    }

    if (!dbus_connection_send(conn, msg, &serial)) {
        fprintf(stderr, "Out of Memory in send!\n");
        dbus_message_unref(msg);
        return -ENOMEM;
    }

    dbus_connection_flush(conn);

    printf("build signal msg and send: \n\t%s\n", _verbose_message(msg));

    dbus_message_unref(msg);

    return 0;
}

int dbus_send_method_call_with_params(DBusConnection *conn,
                                      DBusMessage **reply,
                                      const char *destination,
                                      const char *object_patch,
                                      const char *interface,
                                      const char *name,
                                      int timeout_ms,
                                      int first_arg_type, ...)
{
    DBusError error;
    DBusMessage *msg;
    DBusMessageIter arg;
    DBusPendingCall *pending;
    va_list arg_ptr;
    dbus_bool_t retval;

    dbus_error_init(&error);

    msg = dbus_message_new_method_call(destination, object_patch, interface, name);

    if (msg == NULL) {
        fprintf(stderr, "Failed to allocate the method call message\n");
        return -ENOMEM;
    }

    //Add parameters to the meesage
    dbus_message_iter_init_append(msg, &arg);

    va_start(arg_ptr, first_arg_type);

    retval = dbus_message_append_args_valist(msg,
             first_arg_type,
             arg_ptr);

    va_end(arg_ptr);

    if (!retval) {
        fprintf(stderr, "Can not append the args\n");
        dbus_message_unref(msg);
        return -EINVAL;
    }

    if (!dbus_connection_send_with_reply(conn, msg, &pending, timeout_ms)) {
        fprintf(stderr, "Can not send out the method call\n");
        dbus_message_unref(msg);
        return -EINVAL;
    }

    if (pending == NULL) {
        fprintf(stderr, "Pending Call NULL\n");
        dbus_message_unref(msg);
        return -ECONNRESET;
    }

    dbus_connection_flush(conn);

    dbus_message_unref(msg);

    dbus_pending_call_block(pending);

    *reply = dbus_pending_call_steal_reply(pending);

    if (*reply == NULL) {
        fprintf(stderr, "Failed to get the reply message\n");
        return -ENOMEM;
    }

    return 0;
}
