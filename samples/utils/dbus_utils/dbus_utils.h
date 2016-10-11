#ifndef DBUS_UTILS_H_INCLUDED
#define DBUS_UTILS_H_INCLUDED

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

int dbus_extract_int(DBusMessage *msg, int * value);

char* _verbose_watch(DBusWatch *watch);

char* _verbose_message(DBusMessage *msg);

char* _verbose_timeout(DBusTimeout *timeout);

void dbus_handle_all_timeout();

void dbus_handle_all_watches(DBusConnection *conn);

int dbus_setup_listen_fds(fd_set *rfds, fd_set *wfds, fd_set *efds);

void dbus_handle_listen_fds(fd_set *rfds, fd_set *wfds, fd_set *efds);

int dbus_setup_connection(DBusConnection **conn, const char *wellknown_name,
                          const char *msg_match);

void dbus_cleanup_connection(DBusConnection *conn);

int dbus_register_message_filter(DBusConnection *conn, DBusHandleMessageFunction function);

int dbus_register_object_patch(DBusConnection *conn, char *path[], DBusObjectPathVTable *vtable, unsigned int count);

int dbus_read_params_from_msg(DBusMessage *msg, int first_arg_type, ...);

int dbus_send_method_call_reply_with_results(DBusConnection *conn,
        DBusMessage *msg, int first_arg_type, ...);

int dbus_send_signal_with_params(DBusConnection *conn,
                                 const char *object_patch,
                                 const char *interface,
                                 const char *name,
                                 int first_arg_type, ...);

/*
 * Issue a remote method call.
 * The reply message is stored in **reply.
 * Do not forget to delete the reply by dbus_message_unref()
 * About the timeout_ms, -1 is the default value.
 */
int dbus_send_method_call_with_params(DBusConnection *conn,
                                      DBusMessage **reply,
                                      const char *destination,
                                      const char *object_patch,
                                      const char *interface,
                                      const char *name,
                                      int timeout_ms,
                                      int first_arg_type, ...);


#endif // DBUS_UTILS_H_INCLUDED
