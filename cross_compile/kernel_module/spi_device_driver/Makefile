MODULE = spi_simple_device
obj-m += ${MODULE}.o

CROSS=/home/vanhao/bbb/kernelbuildscripts/dl/gcc-8.5.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-
KERNEL=/home/vanhao/bbb/kernelbuildscripts/KERNEL/

all:
	make ARCH=arm CROSS_COMPILE=${CROSS} -C ${KERNEL} M=$(PWD) modules
clean:
	make -C ${KERNEL} M=$(PWD) clean
install: 
	rsync -avz ./${MODULE}.ko debian@192.168.1.10:/home/debian/kernel_module