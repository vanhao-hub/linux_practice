#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

#define DEVICE_NAME "simple_i2c_device"
#define REGISTER_ADDRESS 0x33
#define DATA_TO_SEND 0x05

static int simple_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

    printk(KERN_INFO "Probe function called\n");

    i2c_smbus_write_byte_data(client, REGISTER_ADDRESS, DATA_TO_SEND);

    return 0;
}

static int simple_i2c_remove(struct i2c_client *client)
{
    printk(KERN_INFO "Remove function called\n");

    // Add cleanup code here if needed

    return 0;
}
static const struct i2c_device_id simple_i2c_id[] = {
    {DEVICE_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, simple_i2c_id);

static struct i2c_driver simple_i2c_driver = {
    .driver = {
        .name = DEVICE_NAME,
    },
    .probe = simple_i2c_probe,
    .remove = simple_i2c_remove,
    .id_table = simple_i2c_id,
};

module_i2c_driver(simple_i2c_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Simple I2C Driver");
MODULE_LICENSE("GPL");