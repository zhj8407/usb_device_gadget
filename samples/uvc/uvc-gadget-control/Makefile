CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../../kernel_source
VISAGE_SOURCE_DIR ?= /home/jiezhang/Dev/visage/platform/l4t_21_3
CROSS_INC_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/include/
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/lib/

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(CROSS_INC_PATH)
LDFLAGS		:= -g

all: uvc-gadget-v4l2sink

uvc-gadget-v4l2sink: uvc-gadget.o
	$(CC) -o $@ $^  $(LDFLAGS)

clean:
	rm -f *.o
	rm -f uvc-gadget-v4l2sink
