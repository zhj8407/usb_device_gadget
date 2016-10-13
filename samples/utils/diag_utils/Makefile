CROSS_COMPILE	?= arm-linux-gnueabihf-
VISAGE_SOURCE_DIR ?=  /home/rdeng/code/usb_byoc/l4t_21_3
KERNEL_DIR	?= $(VISAGE_SOURCE_DIR)/sources/kernel_source
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf

CC		:= $(CROSS_COMPILE)gcc
AR		:= $(CROSS_COMPILE)ar
CFLAGS		:= -W -Wall -g -fPIC
LDFLAGS		:= -lrt

SHARED_TARGET	:= libdiag_utils.so
STATIC_TARGET	:= libdiag_utils.a

all: $(SHARED_TARGET) $(STATIC_TARGET)

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(SHARED_TARGET): diag_utils.o
	$(CC) -shared -o $@ $^ $(LDFLAGS)

$(STATIC_TARGET): diag_utils.o
	$(AR) rcs $@ $^


clean:
	rm -f *.o
	rm -f $(STATIC_TARGET)
	rm -f $(SHARED_TARGET)
