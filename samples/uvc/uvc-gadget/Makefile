CROSS_COMPILE	?= arm-linux-gnueabihf-
ARCH		?= arm
KERNEL_DIR	?= ../../../kernel_source
VISAGE_SOURCE_DIR ?= /home/rdeng/code/usb_byoc/l4t_21_3
CROSS_INC_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/include/
CROSS_LIB_PATH ?= $(VISAGE_SOURCE_DIR)/build-output/_rootfs_/usr/lib/

CC		:= $(CROSS_COMPILE)gcc
#KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
KERNEL_INCLUDE	:=
CFLAGS		:= -W -Wall -g $(KERNEL_INCLUDE) -I$(CROSS_INC_PATH)
LDFLAGS		:= -g -L$(CROSS_LIB_PATH) -lturbojpeg -lm -lpthread -lrt
SOURCE_FILE	:= uvc-gadget.c \
		scale.c \
		visage_shared_mem.c \

APP_SOURCE_FILE	:= visage_webcam.c \
		visage_shared_mem.c \


OBJ_FILE	:= $(patsubst %.c,%.o,$(SOURCE_FILE))

APP_OBJ_FILE	:= $(patsubst %.c,%.o,$(APP_SOURCE_FILE))

APP_TARGET	:=visage_app
STACK_TARGET	:=uvc_stack

all: $(APP_TARGET) $(STACK_TARGET)

$(STACK_TARGET): $(OBJ_FILE)
	@echo $(OBJ_FILE)
	@$(CC) -o $@ $(SOURCE_FILE) $(CFLAGS)  $(LDFLAGS)
	cp $(STACK_TARGET) tu

$(APP_TARGET): $(APP_OBJ_FILE)
	@echo $(APP_OBJ_FILE)
	@$(CC) -o $@ $(APP_SOURCE_FILE) $(CFLAGS)  $(LDFLAGS)
	@echo $(APP_TARGET) done

clean:
	rm -f *.o
	rm -f $(APP_TARGET)
	rm -f $(APP_OBJ_FILE)
	rm -f $(STACK_TARGET)
	rm -f $(STACK_OBJ_FILE)
