CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../../kernel_source
VISAGE_SOURCE_DIR ?= /home/rdeng/code/visage_byoc/svn/trunk/l4t_21_3
CROSS_INC_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/usr/include/
CROSS_USER_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/usr/lib/arm-linux-gnueabihf
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/rootfs/lib/arm-linux-gnueabihf

DBUS_INC ?= -I../../utils/dbus_utils -I/usr/include/dbus-1.0 -I/usr/lib/x86_64-linux-gnu/dbus-1.0/include
DBUS_LIB_PATH ?= ../../utils/dbus_utils/
DIAG_UTILS_FOLDER	?= ../../utils/diag_utils/
DIAG_UTILS_INC	?= $(DIAG_UTILS_FOLDER)
DIAG_UTILS_LIB	?= $(DIAG_UTILS_FOLDER)libdiag_utils.a
GADGET_INC_PATH ?= $(VISAGE_SOURCE_DIR)/sources/kernel_source/drivers/usb/gadget

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(CROSS_INC_PATH) $(DBUS_INC) -I$(DIAG_UTILS_INC) -I$(GADGET_INC_PATH) -I. -I../
LDFLAGS		:= -g -L$(CROSS_LIB_PATH) -lm -lpthread -lrt -L$(DBUS_LIB_PATH) -ldbus-1 -ldbus_utils $(DIAG_UTILS_LIB)
SOURCE_FILE	:= uvc-gadget.c \
		scale.c \
		visage_shared_mem.c \
		plcm_usb_impl.c \
		../uvc-gadget-func.c \

SOURCE_FILE_AIO	:= uvc-gadget-aio.c \
		scale.c \

APP_SOURCE_FILE	:= visage_webcam.c \
		visage_shared_mem.c \
		plcm_usb_impl.c \


OBJ_FILE	:= $(patsubst %.c,%.o,$(SOURCE_FILE))

OBJ_FILE_AIO	:= $(patsubst %.c,%.o,$(SOURCE_FILE_AIO))

APP_OBJ_FILE	:= $(patsubst %.c,%.o,$(APP_SOURCE_FILE))

APP_TARGET	:=visage_app
STACK_TARGET	:=uvc_stack
STACK_TARGET_AIO	:=uvc_stack_aio

all: $(APP_TARGET) $(STACK_TARGET) $(STACK_TARGET_AIO)

$(DIAG_UTILS_LIB):
	make -C $(DIAG_UTILS_FOLDER)

$(STACK_TARGET): $(OBJ_FILE) $(DIAG_UTILS_LIB)
	@echo $(OBJ_FILE)
	@$(CC) -o $@ $(SOURCE_FILE) $(CFLAGS)  $(LDFLAGS)
	cp $(STACK_TARGET) tu

$(STACK_TARGET_AIO): $(OBJ_FILE_AIO)
	@echo $(OBJ_FILE_AIO)
	@$(CC) -o $@ $(SOURCE_FILE_AIO) $(CFLAGS)  $(LDFLAGS)
	cp $(STACK_TARGET_AIO) tui_aio

$(APP_TARGET): $(APP_OBJ_FILE) $(DIAG_UTILS_LIB)
	@echo compiling $(APP_OBJ_FILE)
	@$(CC) -o $@ $(APP_SOURCE_FILE) $(CFLAGS)  $(LDFLAGS)
	@echo $(APP_TARGET) done
	cp $(APP_TARGET) ca

clean:
	rm -f *.o
	rm -f ../*.o
	rm -f $(APP_TARGET)
	rm -f $(STACK_TARGET)
	rm -f $(STACK_TARGET_AIO)
	rm -f tu
	rm -f tui_aio
	rm -f ca
