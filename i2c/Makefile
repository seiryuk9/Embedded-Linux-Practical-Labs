obj-m := tiny_i2c_chip.o
CFLAGS += -w
CC = $(CROSS_COMPILE)gcc
all:
	make -C $(KDIR) M=$(shell pwd) modules

clean:
	make -C $(KDIR) M=$(shell pwd) clean