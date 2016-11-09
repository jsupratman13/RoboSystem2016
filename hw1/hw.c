#include <linux/module.h> //header files
#include <linux/fs.h>
#include <linux/cdev.h> //header for character device
#include <linux/device.h> //header for device
#include <asm/uaccess.h> //header for accessing from device file
#include <linux/io.h> //header file for io
#include <linux/gpio.h>

//module information
MODULE_AUTHOR("JOSHUA");
MODULE_DESCRIPTION("driver for linetracer control");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

//define static variable
static dev_t dev; //define device variable
static struct cdev cdv; //structure character device info
static struct class *cls = NULL; //structure class info
static volatile u32 *gpio_base = NULL; //array for mapping address
static unsigned int gpioPI =21;	//photo interrupter gpio
static unsigned int gpioMT = 12;	//motor gpio

//tomosoft.jp/design/?p=5521
//function to execute when device file is read (gpio)
static ssize_t pi_read(struct file* flip, char* buf, size_t count, loff_t* pos){
	char byte;
	int size = 0;
	byte = '0'+gpio_get_value(gpioPI);
	put_user(byte,buf);
	return 1;
}

//function to execute when device file is writing
static ssize_t motor_write(struct file* flip, const char* buf, size_t count, loff_t* pos){
	//printk(KERN_INFO "led_write is called \n"); //just to check if properly called
	char c; //variable to read input text
	if (copy_from_user(&c, buf, sizeof(char))){ //receive from user to kernel
		return -EFAULT;
	}
	printk(KERN_INFO "receive %c\n", c); //print receive text
	
	//if 0 off, 1 on for led
	if (c=='0'){
		gpio_base[10] = 1 << gpioMT;
	}
	else if(c == '1'){
		gpio_base[7] = 1 << gpioMT;
	}

	return 1; //return number when called
}

//structure
static struct file_operations led_fops = {
	.owner = THIS_MODULE,	//define owner, register to kernel
	.write = motor_write,	//if write occur, call the motor_write function
	.read = pi_read,		//if read occur, call the pi_read function   
};

//initial function when loaded
static int __init init_mod(void)
{
	//get device number
	int retval;

	//set up gpio write
	gpio_base = ioremap_nocache(0x3f200000, 0xA0); //bind this address(to 0xA0 range) to gpio_base variable to make call easier
	const u32 led = gpioMT; //use GPIO25
	const u32 index = led/10; //GPFSEL2
	const u32 shift = (led%10)*3; //15bit
	const u32 mask = ~(0x7 << shift); //1111111111111100011111111111111 .etc
	gpio_base[index] = (gpio_base[index] & mask) | (0x1 << shift); //001:output

	retval = alloc_chrdev_region(&dev, 0, 1, "hw"); //request device number, &dev is address, want 0 to 1 number, device name
	if(retval < 0){
		printk(KERN_ERR "alloc_chr_dev_region failed.\n");
		return retval;
	}
	printk(KERN_INFO "%s is loaded. major:%d\n", __FILE__,MAJOR(dev)); //macro function and returns device number
	
	//set up gpio read
	printk(KERN_INFO "GPIO_TEST\n");
	if(!gpio_is_valid(gpioPI)){
		printk(KERN_INFO "INVALID gpioTest\n");
		return -ENODEV;
	}
	gpio_request(gpioPI, "sysfs");
	gpio_direction_input(gpioPI);
	gpio_set_debounce(gpioPI,200);
	gpio_export(gpioPI, false);
	printk(KERN_INFO "current GPIO state is %d\n", gpio_get_value(gpioPI));

	//initialize as character device
	cdev_init(&cdv, &led_fops); //initialize character device and pass structure to cdev_init
	retval = cdev_add(&cdv, dev,1);	//register character device to kernel
	if (retval < 0){
		printk(KERN_ERR, "cdev_add failed. \n");
		return retval;
	}

	//initialize class
	cls = class_create(THIS_MODULE, "hw"); //THIS_MODULE is a contructor pointer to manage this module
	if(IS_ERR(cls)){
		printk(KERN_ERR, "class_create failed. \n");
		return PTR_ERR(cls);
	}
	device_create(cls, NULL, dev, NULL, "hw%d",MINOR(dev)); //create device information
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
	gpio_unexport(gpioPI);
	gpio_free(gpioPI);
	printk(KERN_INFO "%s is unloaded. major:%d\n", __FILE__,MAJOR(dev));
}

//macro function
module_init(init_mod);
module_exit(cleanup_mod);
