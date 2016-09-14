CROSS_COMPILE	?= arm-linux-gnueabihf-
VISAGE_SOURCE_DIR ?= /home/jiezhang/Dev/visage/platform/new/l4t_21_3
KERNEL_DIR	?= $(VISAGE_SOURCE_DIR)/sources/kernel_source
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
INC_FOLDER	:=../include/
DBUS_INCLUDE := /usr/include/dbus-1.0 -I/usr/lib/x86_64-linux-gnu/dbus-1.0/include
EVENT_INCLUDE := /usr/include
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(INC_FOLDER) -I$(DBUS_INCLUDE) -I$(EVENT_INCLUDE)
LDFLAGS		:= -g -L. -ldbus-1 -levent-2.0

all: udm_client

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

udm_client: dbus_utils.o dbus_event_utils.o main.o
	ln -sf $(CROSS_LIB_PATH)/libdbus-1.so.3.7.6 ./libdbus-1.so
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

clean:
	rm -f *.o
	rm -f udm_client
	rm ./libdbus-1.so
