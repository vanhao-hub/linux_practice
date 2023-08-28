MODULE = gpio_driver
obj-m += ${MODULE}.o

CROSS=/home/my-linux-os/bbb/kernelbuildscripts/dl/gcc-8.5.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-
KERNEL=/home/my-linux-os/bbb/kernelbuildscripts/KERNEL/

all:
	make ARCH=arm CROSS_COMPILE=${CROSS} -C ${KERNEL} M=$(PWD) modules
clean:
	make -C ${KERNEL} M=$(PWD) clean
install: 
	rsync -avz ./${MODULE}.ko debian@192.168.1.13:/home/debian/kernel_module