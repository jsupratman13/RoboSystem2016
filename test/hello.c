#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joshua Supratman");
MODULE_DESCRIPTION("Test");
MODULE_VERSION("1.0");

static char *name = "world";
module_param(name, charp, S_IRUGO);
//MODULE_PARAM_DESC(name, "string displayed in log");

static int __init helloworld_init(void){
	printk(KERN_INFO "Hello is a %s !\n", name);
	return 0;
}

static void __exit helloworld_exit(void){
	printk(KERN_INFO "Goodbye is a %s !\n", name);
}

module_init(helloworld_init);
module_exit(helloworld_exit);

