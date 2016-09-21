CROSS_COMPILE	?= arm-linux-gnueabihf-
VISAGE_SOURCE_DIR ?= /home/jiezhang/Dev/visage/platform/new/l4t_21_3
KERNEL_DIR	?= $(VISAGE_SOURCE_DIR)/sources/kernel_source
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf

CC		:= $(CROSS_COMPILE)gcc
AR		:= $(CROSS_COMPILE)ar
DBUS_INCLUDE := /usr/include/dbus-1.0 -I/usr/lib/x86_64-linux-gnu/dbus-1.0/include
CFLAGS		:= -W -Wall -g -I$(DBUS_INCLUDE) -fPIC
LDFLAGS		:= -L. -ldbus-1

all: libdbus_utils.so libdbus_utils.a

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

libdbus_utils.so: dbus_utils.o
	ln -sf $(CROSS_LIB_PATH)/libdbus-1.so.3.7.6 ./libdbus-1.so
	$(CC) -shared -o $@ $^ $(LDFLAGS)

libdbus_utils.a: dbus_utils.o
	$(AR) rcs $@ $^


clean:
	rm -f *.o
	rm -f libdbus_utils.so
	rm -f libdbus_utils.a
	rm ./libdbus-1.so
