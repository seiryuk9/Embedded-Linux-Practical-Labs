obj-m += dt_gpio.o

CFLAGS += -w
CC = $(CROSS_COMPILE)gcc

all:
	make -C $(KDIR) M=$(shell pwd) modules

clean:
	make -C $(KDIR) M=$(shell pwd) clean

