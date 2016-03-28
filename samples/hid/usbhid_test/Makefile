###########################################################
#
#               Makefile of USB HID
#
###########################################################
CC=/usr/bin/arm-linux-gnueabihf-gcc
PROGRAM=UsbHid
# set compiler argument

CFLAGS 	= -DNV_IS_LDK=1
LDFLAGS= -lpthread -lrt
INCLUDES=-I./inc
#LIBRARY=-L../lib
SRCDIR = ./src
OBJDIR = ./bin
CFLAGS:=$(CFLAGS) $(INCLUDES)

SRCS = drvUsbHidDevice.c Queue.c utils.c evtest.c
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

