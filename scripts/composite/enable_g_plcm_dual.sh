#!/bin/sh

echo "Disable Host mode"
echo 0 > /sys/devices/platform/tegra-otg/enable_host

echo "Enable device mode"
echo 1 > /sys/devices/platform/tegra-otg/enable_device

#modprobe g_audio fn_play=/dev/snd/pcmC0D3p fn_cntl=/dev/snd/controlC0 idVendor=0x095D idProduct=0x9105 iManufacturer="Polycom, Inc." iProduct="CX5501" iSerialNumber="Visage12345"

#modprobe g_webcam idVendor=0x095D idProduct=0x9105 iManufacturer="Polycom, Inc." iProduct="CX5501" iSerialNumber="Visage12345"

#modprobe g_hid idVendor=0x095D idProduct=0x9105 iManufacturer="Polycom, Inc." iProduct="CX5501" iSerialNumber="Visage12345"

echo "Install g_plcmusb module"
modprobe g_plcmusb

echo "Connect the device"
echo connect > /sys/devices/platform/tegra-udc.0/udc/tegra-udc.0/soft_connect

echo "Disable the g_plcmusb"
echo 0 > /sys/class/plcm_usb/plcm0/enable

echo "Set the device descriptors"
echo 095d > /sys/class/plcm_usb/plcm0/idVendor
echo 9132 > /sys/class/plcm_usb/plcm0/idProduct
echo 0310 > /sys/class/plcm_usb/plcm0/bcdDevice
echo 239  > /sys/class/plcm_usb/plcm0/bDeviceClass
echo 2    > /sys/class/plcm_usb/plcm0/bDeviceSubClass
echo 1    > /sys/class/plcm_usb/plcm0/bDeviceProtocol
echo "Polycom, Inc." > /sys/class/plcm_usb/plcm0/iManufacturer
echo "CX5552" > /sys/class/plcm_usb/plcm0/iProduct
echo "BCD12345678" > /sys/class/plcm_usb/plcm0/iSerial

echo "Polycom CX5552 Audio" > /sys/class/plcm_usb/plcm0/f_audio_dual/iad_string
echo "Polycom CX5552 Audio Control" > /sys/class/plcm_usb/plcm0/f_audio_dual/audio_control_string
echo "Polycom CX5552 Speaker" > /sys/class/plcm_usb/plcm0/f_audio_dual/audio_out_stream_string
echo "Polycom CX5552 Micphone" > /sys/class/plcm_usb/plcm0/f_audio_dual/audio_in_stream_string

echo "Set function lists"
#echo "webcam,audio_source,hidg" > /sys/class/plcm_usb/plcm0/functions
#echo "webcam,audio_source,hidg" > /sys/class/plcm_usb/plcm0/functions
#echo "hidg,audio_source" > /sys/class/plcm_usb/plcm0/functions
echo "webcam,audio_dual,hidg" > /sys/class/plcm_usb/plcm0/functions
#echo "audio_source,webcam,hidg" > /sys/class/plcm_usb/plcm0/functions

echo "Enable the g_plcmusb"
echo 1 > /sys/class/plcm_usb/plcm0/enable
