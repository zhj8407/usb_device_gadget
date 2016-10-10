CROSS_COMPILE	?= arm-linux-gnueabihf-
VISAGE_SOURCE_DIR ?= /home/liwang/byoc/l4t_21_3
KERNEL_DIR	?= $(VISAGE_SOURCE_DIR)/sources/kernel_source
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
DBUS_INCLUDE := /usr/include/dbus-1.0 -I/usr/lib/x86_64-linux-gnu/dbus-1.0/include
DBUS_UTILS_INCLUDE := ../../utils/dbus_utils
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(DBUS_UTILS_INCLUDE) -I$(DBUS_INCLUDE)
#LDFLAGS		:= -g -L. -ldbus-1 -ludev -L../utils/dbus_utils -ldbus_utils
#Static linked the libdbus_utils
LDFLAGS		:= -g -L../../utils/dbus_utils -Wl,-Bstatic -ldbus_utils -Wl,-Bdynamic -L. -ldbus-1 -ludev

all: uac_dbus

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

uac_dbus: udev_utils.o main.o
	ln -sf $(CROSS_LIB_PATH)/libudev.so.1.3.5 ./libudev.so
	ln -sf $(CROSS_LIB_PATH)/libdbus-1.so.3.7.6 ./libdbus-1.so
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *.o
	rm -f uac_dbus
	rm ./libudev.so ./libdbus-1.so
