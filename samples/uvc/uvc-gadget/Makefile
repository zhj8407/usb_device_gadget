CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../../kernel_source
VISAGE_SOURCE_DIR ?= /home/rdeng/code/usb_byoc/l4t_21_3
CROSS_INC_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/include/
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/lib/

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(CROSS_INC_PATH)
LDFLAGS		:= -g -L$(CROSS_LIB_PATH) -lturbojpeg

all: uvc-gadget

uvc-gadget: uvc-gadget.o
	$(CC) -o $@ $^  $(LDFLAGS)
	cp uvc-gadget test_uvc

clean:
	rm -f *.o
	rm -f uvc-gadget
	rm -f test_uvc
