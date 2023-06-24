#include <linux/module.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/fs.h>
// #include <linux/proc_fs.h> no need to be used in character device driver development
// headr use for device drvier
#include <linux/cdev.h>
#include <linux/device.h>

#define DV_LICENSE		"GPL"
#define DV_AU 			"vovanhao97.hcmute@gmail.com"
#define DV_DES			"new device for testing"
#define DV_VER			"1.0"
#define DRIVER_NAME "MyDeviceDriver"
#define DRIVER_CLASS	"MyModulesClass"

static dev_t mydevice_nr;
static struct class *my_class;
static struct cdev my_device;

// static struct proc_dir_entry *driver_proc = NULL; not use for device driver
static int driver_open(struct inode *device,struct file *instance)
{
	printk("Device open was called \n");
	return 0;
}

static int driver_close(struct inode *device, struct file *instance)
{
	printk("Device close wasl called\n");
	return 0;
}

static ssize_t driver_read(struct file *File, char *user_buffer, size_t size, loff_t *offs)
{
	return (copy_to_user(user_buffer, "Hello user\n",11)?0:11);
}

static ssize_t driver_write(struct file *File, const char *user_buffer, size_t size, loff_t *offs)
{
	char user_data[10];
	memset(user_data, 0, 10);
	if(copy_from_user(user_data, user_buffer, size))
		return 0;
	printk("user write : %s \n", user_data);
	return size;
};

static const struct file_operations f_ops = {
	.owner = THIS_MODULE,
	.open	= driver_open,
	.read	= driver_read,
	.release	= driver_close,
	.write	= driver_write,
};

static int my_driver_init(void)
{
	printk("Init device driver my-driver \n");
	if(alloc_chrdev_region(&mydevice_nr, 0, 1, DRIVER_NAME) < 0)
	{
		printk("Driver could not be alllocated \n");
		return -1;
	}
	printk("My-driver : Major %d , Minor %d \n", mydevice_nr >> 20, mydevice_nr&&0xfffff);

	if((my_class = class_create(THIS_MODULE, DRIVER_CLASS)) == NULL)
	{
		printk("Device class can not be created \n");
		unregister_chrdev_region(mydevice_nr, 1);
		return -1;
	}
	if(device_create(my_class, NULL, mydevice_nr, NULL, DRIVER_NAME)== NULL)
	{
		printk("Can not create device file \n");
		class_destroy(my_class);
		unregister_chrdev_region(mydevice_nr, 1);
	}
	cdev_init(&my_device, &f_ops);
	if(cdev_add(&my_device, mydevice_nr, 1)==1)
	{
		printk("Can not add device driver into system \n");
		device_destroy(my_class, mydevice_nr);
		class_destroy(my_class);
		unregister_chrdev_region(mydevice_nr, 1);
		return -1;
	}
	return 0;
}

static void my_driver_exit(void)
{
	printk("Exit device driver my-driver \n");
	cdev_del(&my_device);
	device_destroy(my_class, mydevice_nr);
	class_destroy(my_class);
	unregister_chrdev_region(mydevice_nr,1);
}


module_init(my_driver_init);
module_exit(my_driver_exit);

// Module Information
MODULE_LICENSE(DV_LICENSE);
MODULE_AUTHOR(DV_AU);
MODULE_DESCRIPTION(DV_DES);
MODULE_VERSION(DV_VER);

