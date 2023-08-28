#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(.gnu.linkonce.this_module) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section(__versions) = {
	{ 0x2c111234, "module_layout" },
	{ 0xfe990052, "gpio_free" },
	{ 0x2a8f365, "driver_unregister" },
	{ 0xce7febb, "__spi_register_driver" },
	{ 0x608ccaab, "gpiod_direction_output_raw" },
	{ 0x47229b5c, "gpio_request" },
	{ 0xd262cb80, "spi_setup" },
	{ 0xf9a482f9, "msleep" },
	{ 0xf1658bf4, "gpiod_set_raw_value" },
	{ 0x4cb272b1, "gpio_to_desc" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0x317a623c, "spi_sync" },
	{ 0x68f31cbd, "__list_add_valid" },
	{ 0x5f754e5a, "memset" },
	{ 0xc5850110, "printk" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("spi:simple_spi_driver");
