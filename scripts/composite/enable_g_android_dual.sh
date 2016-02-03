#!/bin/sh

echo "Disable Host mode"
echo 0 > /sys/devices/platform/tegra-otg/enable_host

echo "Enable device mode"
echo 1 > /sys/devices/platform/tegra-otg/enable_device

#modprobe g_audio fn_play=/dev/snd/pcmC0D3p fn_cntl=/dev/snd/controlC0 idVendor=0x095D idProduct=0x9105 iManufacturer="Polycom, Inc." iProduct="CX5501" iSerialNumber="Visage12345"

#modprobe g_webcam idVendor=0x095D idProduct=0x9105 iManufacturer="Polycom, Inc." iProduct="CX5501" iSerialNumber="Visage12345"

#modprobe g_hid idVendor=0x095D idProduct=0x9105 iManufacturer="Polycom, Inc." iProduct="CX5501" iSerialNumber="Visage12345"

echo "Install g_android module"
modprobe g_android

echo "Connect the device"
echo connect > /sys/devices/platform/tegra-udc.0/udc/tegra-udc.0/soft_connect

echo "Disable the g_android"
echo 0 > /sys/class/android_usb/android0/enable

echo "Set the device descriptors"
echo 095d > /sys/class/android_usb/android0/idVendor
echo 9132 > /sys/class/android_usb/android0/idProduct
echo 0310 > /sys/class/android_usb/android0/bcdDevice
echo 239  > /sys/class/android_usb/android0/bDeviceClass
echo 2    > /sys/class/android_usb/android0/bDeviceSubClass
echo 1    > /sys/class/android_usb/android0/bDeviceProtocol
echo "Polycom, Inc." > /sys/class/android_usb/android0/iManufacturer
echo "CX5552" > /sys/class/android_usb/android0/iProduct
echo "BCD12345678" > /sys/class/android_usb/android0/iSerial

echo "Polycom CX5552 Audio" > /sys/class/android_usb/android0/f_audio_dual/iad_string
echo "Polycom CX5552 Audio Control" > /sys/class/android_usb/android0/f_audio_dual/audio_control_string
echo "Polycom CX5552 Speaker" > /sys/class/android_usb/android0/f_audio_dual/audio_out_stream_string
echo "Polycom CX5552 Micphone" > /sys/class/android_usb/android0/f_audio_dual/audio_in_stream_string

echo "Set function lists"
#echo "webcam,audio_source,hidg" > /sys/class/android_usb/android0/functions
#echo "webcam,audio_source,hidg" > /sys/class/android_usb/android0/functions
#echo "hidg,audio_source" > /sys/class/android_usb/android0/functions
echo "webcam,audio_dual,hidg" > /sys/class/android_usb/android0/functions
#echo "audio_source,webcam,hidg" > /sys/class/android_usb/android0/functions

echo "Enable the g_android"
echo 1 > /sys/class/android_usb/android0/enable
