#!/usr/bin/env python

import sys
from traceback import print_exc

import dbus
from dbus import Byte, ByteArray

from gi.repository import GLib

import dbus.mainloop.glib

from struct import *

global_uvc_cs_table = {
    "PU_Brightness": [0x0004,
                      {
                          "UVC_GET_MIN": 0x0000,
                          "UVC_GET_MAX": 0x0009,
                          "UVC_GET_INFO": 0x03,
                          "UVC_GET_RES": 0x0001,
                          "UVC_GET_DEF": 0x0004,
                          "UVC_GET_LEN": 0x0002
                      }],

    "CT_AE_Mode": [0x01,
                   {
                       "UVC_GET_MIN": 0x00,
                       "UVC_GET_MAX": 0x01,
                       "UVC_GET_INFO": 0x01,
                       "UVC_GET_RES": 0x01,
                       "UVC_GET_DEF": 0x01,
                       "UVC_GET_LEN": 0x01
                   }],

    "CT_Exposure_Time_Absolute": [0x0004,
                                  {
                                      "UVC_GET_MIN": 0x0000,
                                      "UVC_GET_MAX": 0x0009,
                                      "UVC_GET_INFO": 0x03,
                                      "UVC_GET_RES": 0x0001,
                                      "UVC_GET_DEF": 0x0004,
                                      "UVC_GET_LEN": 0x0002
                                  }],

    "CT_IRIS_Absolute": [0x0004,
                         {
                             "UVC_GET_MIN": 0x0000,
                             "UVC_GET_MAX": 0x0009,
                             "UVC_GET_INFO": 0x03,
                             "UVC_GET_RES": 0x0001,
                             "UVC_GET_DEF": 0x0004,
                             "UVC_GET_LEN": 0x0002
                         }]
}


def get_property_signal_handler(prop_name, request_type, length):
    value = 0
    if "CUR" in request_type:
        value = global_uvc_cs_table[prop_name][0]
    else:
        value = global_uvc_cs_table[prop_name][1][request_type]
    print("%s, %s, len: %d, value: %d" %
          (prop_name, request_type, length, value))

    bus = dbus.SystemBus()
    try:
        remote_object = bus.get_object(
            "com.polycom.visage.uvc", "/com/polycom/visage/uvc")
        reply = remote_object.SendResponse(length, ByteArray(
            pack("i", value)), dbus_interface="com.polycom.visage.uvc")
    except dbus.DBusException:
        print_exc()


def set_property_signal_handler(prop_name, request_type, length, data):
    if length > 4:
        return
    i = 0
    value = 0
    for d in data:
        value += ((int(d)) << (8 * i))
        i += 1
    print("%s, %s, len: %d, value: %d" %
          (prop_name, request_type, length, value))
    global_uvc_cs_table[prop_name][0] = value


def streaming_status_change_signal_handler(status, format, width, height):
    print("%s, format: %s, width: %d, height: %d" %
          (status, format, width, height))


def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    # try:
    #     remote_object = bus.get_object("com.polycom.visage.uvc",
    #                                    "/com/polycom/visage/uvc")
    #     reply = remote_object.SendResponse(4, ByteArray(bytearray([1, 2, 3, 4])),
    #                                        dbus_interface="com.polycom.visage.uvc")
    #     print(reply)
    # except dbus.DBusException:
    #     print_exc()
    #     sys.exit()

    bus.add_signal_receiver(get_property_signal_handler,
                            "GetCameraProperty",
                            "com.polycom.visage.uvc")

    bus.add_signal_receiver(set_property_signal_handler,
                            "SetCameraProperty",
                            "com.polycom.visage.uvc")

    bus.add_signal_receiver(streaming_status_change_signal_handler,
                            "video_control",
                            "com.polycom.visage.uvc")

    loop = GLib.MainLoop()
    loop.run()

if __name__ == '__main__':
    main()
