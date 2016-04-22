###########################################################
#
#               Makefile of USB HID
#
###########################################################
CC=/usr/bin/arm-linux-gnueabihf-gcc
PROGRAM=UsbHid
# set compiler argument

CROSS_LIB_PATH ?= /home/liwang/l4t_21_3/rootfs/lib/arm-linux-gnueabihf
LDFLAGS= -lpthread -lrt -L$(CROSS_LIB_PATH) -ludev

ifeq ($(CALL_DISPlAY),1)
CFLAGS = -DENABLE_CALL_DISPLAY=1
CROSS_LIB_PATH ?= /home/liwang/l4t_21_3/rootfs/lib/arm-linux-gnueabihf
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

