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
	{ 0x77f78ecc, "sys_imageblit" },
	{ 0xa94c7d06, "sys_copyarea" },
	{ 0xc55583a7, "sys_fillrect" },
	{ 0x9393c836, "fb_sys_write" },
	{ 0x7d957ca1, "platform_device_unregister" },
	{ 0xae107ae2, "platform_driver_unregister" },
	{ 0xd945b152, "platform_device_put" },
	{ 0x3eed9f6e, "platform_device_add" },
	{ 0xd7c2fc5f, "platform_device_alloc" },
	{ 0x4e1d8f72, "__platform_driver_register" },
	{ 0xf8cd3290, "register_framebuffer" },
	{ 0x45cec796, "fb_deferred_io_init" },
	{ 0xd6ee688f, "vmalloc" },
	{ 0x9d669763, "memcpy" },
	{ 0xda8c9b92, "framebuffer_alloc" },
	{ 0x26e04dfd, "kmem_cache_alloc_trace" },
	{ 0xbaa7d8e7, "kmalloc_caches" },
	{ 0x936ec15a, "put_device" },
	{ 0xc5850110, "printk" },
	{ 0x608ccaab, "gpiod_direction_output_raw" },
	{ 0x47229b5c, "gpio_request" },
	{ 0xd145d95, "spi_new_device" },
	{ 0x877e7dc1, "spi_busnum_to_master" },
	{ 0xf9a482f9, "msleep" },
	{ 0xf1658bf4, "gpiod_set_raw_value" },
	{ 0x4cb272b1, "gpio_to_desc" },
	{ 0xdecd0b29, "__stack_chk_fail" },
	{ 0x317a623c, "spi_sync" },
	{ 0x68f31cbd, "__list_add_valid" },
	{ 0x5f754e5a, "memset" },
	{ 0x37a0cba, "kfree" },
	{ 0x711586e8, "framebuffer_release" },
	{ 0x20e364f1, "unregister_framebuffer" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");

