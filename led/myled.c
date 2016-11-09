#include <linux/module.h> //header files
#include <linux/fs.h>
#include <linux/cdev.h> //header for character device
#include <linux/device.h> //header for device
#include <asm/uaccess.h> //header for accessing from device file
#include <linux/io.h> //header file for io

//module information
MODULE_AUTHOR("JOSHUA");
MODULE_DESCRIPTION("driver for LED control");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

//define static variable
static dev_t dev; //define device variable
static struct cdev cdv; //structure character device info
static struct class *cls = NULL; //structure class info
static volatile u32 *gpio_base = NULL; //array for mapping address

//function to execute when device file is read (ex cat)
static ssize_t sushi_read(struct file* flip, char* buf, size_t count, loff_t* pos){
	int size = 0;
	char sushi[] = {0xF0, 0x9F, 0x8D, 0xA3, 0x0A}; //binary for drawing sushi
	if(copy_to_user(buf+size, (const char *)sushi, sizeof(sushi))){ //send from kernel to user
		printk(KERN_INFO, "sushi: copy_to_user\n");
		return -EFAULT;
	}
	size += sizeof(sushi);
	return size;
}

//function to execute when device file is writing
static ssize_t led_write(struct file* flip, const char* buf, size_t count, loff_t* pos){
	//printk(KERN_INFO "led_write is called \n"); //just to check if properly called
	char c; //variable to read input text
	if (copy_from_user(&c, buf, sizeof(char))){ //receive from user to kernel
		return -EFAULT;
	}
	printk(KERN_INFO "receive %c\n", c); //print receive text
	
	//if 0 off, 1 on for led
	if (c=='0'){
		//gpio_base[10] = 1 << 25;
		gpio_base[10] = 1 << 8;
	}
	else if(c == '1'){
		//gpio_base[7] = 1 << 25;
		gpio_base[7] = 1 << 8;
	}

	return 1; //return number when called
}

//structure
static struct file_operations led_fops = {
	.owner = THIS_MODULE,	//define owner, register to kernel
	.write = led_write,	//if write occur, call the led_write function
	.read  = sushi_read //if read occur, call sushi_read function
};

//initial function when loaded
static int __init init_mod(void)
{
	//get device number
	int retval;

	gpio_base = ioremap_nocache(0x3f200000, 0xA0); //bind this address(to 0xA0 range) to gpio_base variable to make call easier
	const u32 led = 8;//25; //use GPIO25
	const u32 index = led/10; //GPFSEL2
	const u32 shift = (led%10)*3; //15bit
	const u32 mask = ~(0x7 << shift); //1111111111111100011111111111111 .etc
	gpio_base[index] = (gpio_base[index] & mask) | (0x1 << shift); //001:output

	retval = alloc_chrdev_region(&dev, 0, 1, "myled"); //request device number, &dev is address, want 0 to 1 number, device name
	if(retval < 0){
		printk(KERN_ERR "alloc_chr_dev_region failed.\n");
		return retval;
	}
	printk(KERN_INFO "%s is loaded. major:%d\n", __FILE__,MAJOR(dev)); //macro function and returns device number
	
	//initialize as character device
	cdev_init(&cdv, &led_fops); //initialize character device and pass structure to cdev_init
	retval = cdev_add(&cdv, dev,1);	//register character device to kernel
	if (retval < 0){
		printk(KERN_ERR, "cdev_add failed. \n");
		return retval;
	}

	//initialize class
	cls = class_create(THIS_MODULE, "myled"); //THIS_MODULE is a contructor pointer to manage this module
	if(IS_ERR(cls)){
		printk(KERN_ERR, "class_create failed. \n");
		return PTR_ERR(cls);
	}
	device_create(cls, NULL, dev, NULL, "myled%d",MINOR(dev)); //create device information
	return 0;
}

//exit function when exited
static void __exit cleanup_mod(void)
{
	//return id and delete character device
	cdev_del(&cdv); //destroy character device
	device_destroy(cls,dev); //destroy device information
	class_destroy(cls); //destroy class
	unregister_chrdev_region(dev,1); //return device number
	printk(KERN_INFO "%s is unloaded. major:%d\n", __FILE__,MAJOR(dev));
}

//macro function
module_init(init_mod);
module_exit(cleanup_mod);
