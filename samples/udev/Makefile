CROSS_COMPILE	?= arm-linux-gnueabihf-
KERNEL_DIR	?= /home/rdeng/code/usb_byoc/l4t_21_3/sources/kernel_source
CROSS_LIB_PATH ?= /home/rdeng/code/usb_byoc/l4t_21_3/rootfs/lib/arm-linux-gnueabihf

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
INC_FOLDER	:=../include/
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(INC_FOLDER)
LDFLAGS		:= -g -L$(CROSS_LIB_PATH) -ludev

all: udev_example

udev_example: udev_example.o
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

clean:
	rm -f *.o
	rm -f udev_example
