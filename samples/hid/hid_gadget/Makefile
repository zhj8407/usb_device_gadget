CROSS_COMPILE	?= arm-linux-gnueabihf-

CC		:= $(CROSS_COMPILE)gcc
CFLAGS		:= -W -Wall
LDFLAGS		:= -g

all: hid_gadget_test

hid_gadget_test: hid_gadget_test.o hid_lync_display.o
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	rm -f *.o
	rm -f hid_gadget_test
