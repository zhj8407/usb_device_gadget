CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../../kernel_source
VISAGE_SOURCE_DIR ?= /home/rdeng/code/visage_byoc/svn/trunk/l4t_21_3
CROSS_INC_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/include/
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/lib/
DBUS_INC ?= -I../../utils/dbus_utils -I/usr/include/dbus-1.0 -I/usr/lib/x86_64-linux-gnu/dbus-1.0/include
DBUS_LIB_PATH ?= ../../utils/dbus_utils/

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(CROSS_INC_PATH) $(DBUS_INC)
LDFLAGS		:= -g -L$(CROSS_LIB_PATH) -lturbojpeg -lm -lpthread -lrt -L$(DBUS_LIB_PATH) -ldbus-1 -ldbus_utils
SOURCE_FILE	:= uvc-gadget.c \
		scale.c \
		visage_shared_mem.c \

SOURCE_FILE_AIO	:= uvc-gadget-aio.c \
		scale.c \

APP_SOURCE_FILE	:= visage_webcam.c \
		visage_shared_mem.c \

APP_SOURCE_FILE_2	:= visage_webcam_v4l2.c \
			visage_shared_mem.c \


OBJ_FILE	:= $(patsubst %.c,%.o,$(SOURCE_FILE))

OBJ_FILE_AIO	:= $(patsubst %.c,%.o,$(SOURCE_FILE_AIO))

APP_OBJ_FILE	:= $(patsubst %.c,%.o,$(APP_SOURCE_FILE))

APP_OBJ_FILE_2	:= $(patsubst %.c,%.o,$(APP_SOURCE_FILE_2))

APP_TARGET	:=visage_app
APP_TARGET_2	:=visage_app_v4l2
STACK_TARGET	:=uvc_stack
STACK_TARGET_AIO	:=uvc_stack_aio

all: $(APP_TARGET) $(STACK_TARGET) $(STACK_TARGET_AIO)

$(STACK_TARGET): $(OBJ_FILE)
	@echo $(OBJ_FILE)
	@$(CC) -o $@ $(SOURCE_FILE) $(CFLAGS)  $(LDFLAGS)
	cp $(STACK_TARGET) tu

$(STACK_TARGET_AIO): $(OBJ_FILE_AIO)
	@echo $(OBJ_FILE_AIO)
	@$(CC) -o $@ $(SOURCE_FILE_AIO) $(CFLAGS)  $(LDFLAGS)
	cp $(STACK_TARGET_AIO) tui_aio

$(APP_TARGET): $(APP_OBJ_FILE)
	@echo compiling $(APP_OBJ_FILE)
	@$(CC) -o $@ $(APP_SOURCE_FILE) $(CFLAGS)  $(LDFLAGS)
	@echo $(APP_TARGET) done

$(APP_TARGET_2): $(APP_OBJ_FILE_2)
	@echo compiling $(APP_OBJ_FILE_2)
	@$(CC) -o $@ $(APP_SOURCE_FILE_2) $(CFLAGS)  $(LDFLAGS)
	@echo $(APP_TARGET_2) done
clean:
	rm -f *.o
	rm -f $(APP_TARGET)
	rm -f $(APP_TARGET_2)
	rm -f $(STACK_TARGET)
	rm -f $(STACK_TARGET_AIO)
