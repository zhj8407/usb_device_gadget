CROSS_COMPILE	?= arm-linux-gnueabihf-
KERNEL_DIR	?= /home/jiezhang/Dev/visage/platform/l4t_21_3/sources/kernel_source
CROSS_LIB_PATH ?= /home/jiezhang/Dev/visage/platform/l4t_21_3/rootfs/lib/arm-linux-gnueabihf

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE)
LDFLAGS		:= -g -L$(CROSS_LIB_PATH) -ludev

all: udev_example

udev_example: udev_example.o
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *.o
	rm -f udev_example
