#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

struct my_gpio_data {
    struct gpio_desc *gpio;
};

static int my_platform_probe(struct platform_device *pdev)
{
    struct my_gpio_data *data;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    // Get the GPIO from the platform data
    data->gpio = devm_gpiod_get(&pdev->dev, "led", GPIOD_OUT_HIGH);
    if (IS_ERR(data->gpio)) {
        dev_err(&pdev->dev, "Failed to get GPIO\n");
        return PTR_ERR(data->gpio);
    }

    // Store private data for this device
    platform_set_drvdata(pdev, data);

    pr_info("GPIO %s set to HIGH\n", dev_name(&pdev->dev));
    gpiod_set_value(data->gpio, 1);
    pr_info("Probe GPIO driver done \n");
    return 0;
}

static int my_platform_remove(struct platform_device *pdev)
{
    struct my_gpio_data *data = platform_get_drvdata(pdev);

    gpiod_set_value(data->gpio, 0);
    pr_info("Remove GPIO driver done \n");
    return 0;
}

static const struct of_device_id my_platform_of_match[] = {
    { .compatible = "my,gpio-test", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, my_platform_of_match);

static struct platform_driver my_platform_driver = {
    .probe = my_platform_probe,
    .remove = my_platform_remove,
    .driver = {
        .name = "my_gpio_driver", 
        .of_match_table = my_platform_of_match,
        .owner = THIS_MODULE,
    },
};

module_platform_driver(my_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("vanhao.vo");
MODULE_DESCRIPTION("A simple GPIO platform driver");