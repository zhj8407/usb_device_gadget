###########################################################
#
#               Makefile of USB HID
#
###########################################################
CC=/usr/bin/arm-linux-gnueabihf-gcc
PROGRAM=UsbHid
VISAGE_PLATFORM_DIR=/home/jiezhang/Dev/visage/platform/l4t_21_3
# set compiler argument

CROSS_LIB_PATH ?= $(VISAGE_PLATFORM_DIR)/rootfs/lib/arm-linux-gnueabihf
#DFLAGS= -lpthread -lrt -L$(CROSS_LIB_PATH) -ludev
CALL_DISPLAY=1

ifeq ($(CALL_DISPLAY),1)
CFLAGS = -DENABLE_CALL_DISPLAY=1
LDFLAGS= -lpthread -lrt -L$(CROSS_LIB_PATH) -ludev
else
LDFLAGS= -lpthread -lrt
endif

INCLUDES=-I./inc
#LIBRARY=-L../lib
SRCDIR = ./src
OBJDIR = ./bin
CFLAGS:=$(CFLAGS) $(INCLUDES)

SRCS = drvUsbHidDevice.c Queue.c utils.c main.c hid_lync_display.c
OBJS=$(patsubst   %.c,$(OBJDIR)/%.o,$(SRCS))

all: $(PROGRAM)
$(PROGRAM):$(OBJS)
	echo $(OBJS)
	$(CC) $ -o $(OBJDIR)/$@ $^ $(LDFLAGS)
#	cp -pf $(OBJDIR)/$(PROGRAM) ../

$(OBJDIR)/%.o : $(SRCDIR)/%.c
	mkdir -p ./bin
	$(CC) -c $(CFLAGS) $< -o $@

.PHONY:clean
clean:
	rm -rf $(OBJDIR)

