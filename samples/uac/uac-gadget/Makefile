CROSS_COMPILE	?= arm-linux-gnueabihf-
VISAGE_SOURCE_DIR ?= /home/liwang/byoc/l4t_21_3
KERNEL_DIR	?= $(VISAGE_SOURCE_DIR)/sources/kernel_source
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf

ALSA_DIR ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/lib
ALSA_IDIR ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/include

CC		:= $(CROSS_COMPILE)gcc -g

KERNEL_INCLUDE	:= -I$(ALSA_IDIR)
DBUS_INCLUDE := /usr/include/dbus-1.0 -I/usr/lib/x86_64-linux-gnu/dbus-1.0/include
DBUS_UTILS_INCLUDE := ../../utils/dbus_utils/
INCLUDES= ./inc
SRCDIR = ./src
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(DBUS_UTILS_INCLUDE) -I$(DBUS_INCLUDE) -I$(INCLUDES)

LDFLAGS		:= -g -L../../utils/dbus_utils -Wl,-Bstatic -ldbus_utils -Wl,-Bdynamic -L. -ldbus-1 -lpthread -ludev -L$(ALSA_DIR) -lasound -lm

PROGRAM=UsbUac
OBJDIR = .
SRCS = main.c audiofifos.c audiosource.c audioutils.c udev_utils.c
OBJS=$(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: $(PROGRAM)
$(PROGRAM):$(OBJS)
	echo $(OBJS)
	ln -sf $(CROSS_LIB_PATH)/libudev.so.1.3.5 ./libudev.so
	ln -sf $(CROSS_LIB_PATH)/libdbus-1.so.3.7.6 ./libdbus-1.so
	$(CC) $ -o $(OBJDIR)/$@ $^ $(LDFLAGS)

$(OBJDIR)/%.o : $(SRCDIR)/%.c
#	mkdir -p ./bin
	$(CC) -c $(CFLAGS) $< -o $@
.PHONY:clean

clean:
	rm -f *.o
	rm -f UsbUac
	rm -f ./libudev.so ./libdbus-1.so
