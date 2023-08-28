#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>

struct backlight_pdim_data {
    struct gpio_desc *led_gpio;
    struct pinctrl *pinctrl;
};

static int backlight_pdim_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct backlight_pdim_data *data;

    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    devm_pinctrl_get_select(dev, "default");

    platform_set_drvdata(pdev, data);
    pr_info("Probe GPIO driver done \n");
    return 0;
}

static int backlight_pdim_remove(struct platform_device *pdev)
{
    // Clean up and release resources
    return 0;
}

static const struct of_device_id backlight_pdim_of_match[] = {
    { .compatible = "gpio-descriptor-based" },
    { },
};
MODULE_DEVICE_TABLE(of, backlight_pdim_of_match);

static struct platform_driver backlight_pdim_driver = {
    .probe = backlight_pdim_probe,
    .remove = backlight_pdim_remove,
    .driver = {
        .name = "backlight-pdim",
        .of_match_table = backlight_pdim_of_match,
        .owner = THIS_MODULE,
    },
};

module_platform_driver(backlight_pdim_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("vanhao.vo");
MODULE_DESCRIPTION("Backlight PDIM Driver");