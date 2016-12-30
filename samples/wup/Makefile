CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../kernel_source
VISAGE_SOURCE_DIR ?= /home/jiezhang/Dev/visage/platform/latest/trunk/l4t_21_5
CROSS_INC_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/usr/include/
CROSS_USER_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/usr/lib/arm-linux-gnueabihf
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf
GADGET_INC_PATH ?= $(VISAGE_SOURCE_DIR)/sources/kernel_source/drivers/usb/gadget

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I/usr/include/x86_64-linux-gnu -I/usr/include
LDFLAGS		:= -g -L. -lssl -lcrypto

all: dfu-gadget

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

dfu-gadget: dfu_gadget.c
	ln -sf $(CROSS_LIB_PATH)/libssl.so.1.0.0 libssl.so
	ln -sf $(CROSS_LIB_PATH)/libcrypto.so.1.0.0 libcrypto.so
	$(CC) -o $@ $^  $(LDFLAGS)

clean:
	rm -f *.o
	rm -f ../*.o
	rm -f *.so
	rm -f dfu-gadget
