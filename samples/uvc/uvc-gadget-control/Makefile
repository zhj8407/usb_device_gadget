CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../../kernel_source
VISAGE_SOURCE_DIR ?= /home/jiezhang/Dev/visage/platform/latest/l4t_21_3
CROSS_INC_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/usr/include/
CROSS_USER_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/usr/lib/arm-linux-gnueabihf
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
GSTREAMER_INCLUDE := $(shell pkg-config --cflags gstreamer-1.0)
GSTREAMER_LIB := $(shell pkg-config --libs gstreamer-1.0)
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(CROSS_INC_PATH) $(GSTREAMER_INCLUDE)
LDFLAGS		:= -g -L. $(GSTREAMER_LIB) -lpcre -lgmodule-2.0 -lffi -lgio-2.0 -lz -lselinux

all: uvc-gadget-v4l2sink

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

uvc-gadget-v4l2sink: uvc-gadget.o
	ln -sf $(CROSS_USER_LIB_PATH)/libgstreamer-1.0.so.0.204.0 libgstreamer-1.0.so
	ln -sf $(CROSS_USER_LIB_PATH)/libgobject-2.0.so.0.4000.0 libgobject-2.0.so
	ln -sf $(CROSS_USER_LIB_PATH)/libgmodule-2.0.so.0.4000.0 libgmodule-2.0.so
	ln -sf $(CROSS_USER_LIB_PATH)/libgobject-2.0.so.0.4000.0 libgobject-2.0.so
	ln -sf $(CROSS_USER_LIB_PATH)/libgio-2.0.so.0.4000.0 libgio-2.0.so
	ln -sf $(CROSS_USER_LIB_PATH)/libffi.so.6.0.1 libffi.so
	ln -sf $(CROSS_LIB_PATH)/libglib-2.0.so.0.4000.0 libglib-2.0.so
	ln -sf $(CROSS_LIB_PATH)/libpcre.so.3.13.1 libpcre.so
	ln -sf $(CROSS_LIB_PATH)/libz.so.1.2.8 libz.so
	ln -sf $(CROSS_LIB_PATH)/libselinux.so.1 libselinux.so
	$(CC) -o $@ $^  $(LDFLAGS)

clean:
	rm -f *.o
	rm -f *.so
	rm -f uvc-gadget-v4l2sink
