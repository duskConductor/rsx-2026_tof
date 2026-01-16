#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x4e8e25be, "i2c_register_driver" },
	{ 0xcd47056b, "__spi_register_driver" },
	{ 0x2cfa9196, "i2c_del_driver" },
	{ 0x122c3a7e, "_printk" },
	{ 0xe2964344, "__wake_up" },
	{ 0x92893115, "driver_unregister" },
	{ 0x94090688, "misc_deregister" },
	{ 0x5a1c908c, "devm_gpiod_get_optional" },
	{ 0x2795b5f0, "gpiod_to_irq" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x3ce80115, "devm_request_threaded_irq" },
	{ 0x4cd6ab7e, "of_find_property" },
	{ 0x80192c9b, "of_property_read_variable_u32_array" },
	{ 0x637962ed, "_dev_info" },
	{ 0xf11396ac, "_dev_err" },
	{ 0xdcb764ad, "memset" },
	{ 0xce12c86b, "spi_sync" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xbbe69956, "i2c_transfer" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x4829a47e, "memcpy" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0x656e4a6e, "snprintf" },
	{ 0x2002cbd1, "misc_register" },
	{ 0x36a78de3, "devm_kmalloc" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x1000e51, "schedule" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x474e54d2, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("spi:stmvl53l8cx");
MODULE_ALIAS("of:N*T*Cst,stmvl53l8cx");
MODULE_ALIAS("of:N*T*Cst,stmvl53l8cxC*");
MODULE_ALIAS("i2c:stmvl53l8cx");
MODULE_ALIAS("of:N*T*Cst,stmvl53l8cx");
MODULE_ALIAS("of:N*T*Cst,stmvl53l8cxC*");

MODULE_INFO(srcversion, "A15837324E5A79F7D629040");
