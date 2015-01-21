obj-m += great.o
all:
	make -C /usr/src/linux-headers-`uname -r` M=$(PWD) modules
clean:
	make -C /usr/src/linux-headers-`uname -r` M=$(PWD) clean
