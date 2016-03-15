CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../../kernel_source

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:= 
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE)
LDFLAGS		:= -g

all: uvc-gadget

uvc-gadget: uvc-gadget.o
	$(CC) $(LDFLAGS) -o $@ $^
	cp uvc-gadget test_uvc

clean:
	rm -f *.o
	rm -f uvc-gadget
