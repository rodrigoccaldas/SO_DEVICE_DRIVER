#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include "serial_reg.h"

#define PORT_UART 0x3F8 //From VirtualBox's defaults for COM1
#define LSR_ADDRESS (PORT_UART + UART_LSR)
#define LCR_ADDRESS (PORT_UART + UART_LCR)
#define DLL_ADDRESS (PORT_UART + UART_DLL)
#define DLM_ADDRESS (PORT_UART + UART_DLM)
#define IER_ADDRESS (PORT_UART + UART_IER)
#define RECEIVE_ADRESS (PORT_UART + UART_RX)
#define TRANSMIT_ADRESS (PORT_UART + UART_TX)

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Celso e Rodrigo");
struct serp_device {
	struct cdev cdev;
};

static int serp_open(struct inode *inodep, struct file *filep);
ssize_t serp_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp);
ssize_t serp_read(struct file *filep, char __user *buff, size_t count, loff_t *offp);
static int serp_close(struct inode *inodep, struct file *filep);

struct file_operations serp_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = serp_read,
	.write = serp_write,
	.open = serp_open,
	.release = serp_close,
};

//Global variables
dev_t *device;
struct file *file;
struct inode *inode;
static struct serp_device *serp_device;

//ULTIMA ALTERAÇÃO  27 MAIO 19:30 (não esquecer de compilar com o makefile)

static int serp_init(void) {
	unsigned char lcr = 0, ier = 0,  dlm = 0, dll = 0;

	//Store dev_t structure
	device = kmalloc(sizeof(dev_t), GFP_KERNEL);
		if( device == NULL) {
			printk(KERN_ALERT "NO MEMORY FOR THE DEVICE - ERRORR\n");
			return -ENOMEM;
		}
	//Ask kernel to allocate a major device number to the device
	int region = alloc_chrdev_region(device, 0, 1, "serp");
		if(region < 0) {
			printk(KERN_ALERT "MAJOR NUMBER ERROR\n");
			return -ENOMEM;
		}
	//Store serp_device structure
	serp_device = kmalloc(sizeof (struct serp_device), GFP_KERNEL);
		if(serp_device == NULL) {
			printk(KERN_ALERT "SERP DEVICE ERROR - MEMORY\n");
			return -ENOMEM;
		}

		//Initialize cdev. Already allocated, embedded in serp_device
	cdev_init(&serp_device->cdev, &serp_fops);
	(&serp_device->cdev)->owner = THIS_MODULE;

	
	int add_dev = cdev_add(&serp_device->cdev, *device, 1); //Register device in the kernel
		if( add_dev < 0) {
			printk(KERN_ALERT "REGISTER THE DD - ERROR\n");
			return -ENOMEM;
		}
	//Store inode structure
	inode = kmalloc(sizeof(struct inode), GFP_KERNEL);
		if(inode == NULL) {
			printk(KERN_ALERT "INODE MEMORY - ERROR\n");
			return -ENOMEM;
		}

	inode->i_cdev = &serp_device->cdev;

	//Store a file descriptor structure
	file = kmalloc(sizeof(struct file), GFP_KERNEL);
		if(file == NULL) {
			printk(KERN_ALERT "FILE MEMORY - ERROR\n");
			return -ENOMEM;
		}
	//Request I/O Port range used by the serial port device
 	 if(request_region(PORT_UART, 7, "serp") == NULL) {
   		 printk(KERN_ALERT "REQUEST REGION - ERROR\n");
		return -ENOMEM;
  }
	
	//even parity, 2 stop bits, 8-bit words, set DLAB to access DLL and DLM
	lcr = UART_LCR_DLAB | UART_LCR_EPAR | UART_LCR_PARITY | UART_LCR_STOP | UART_LCR_WLEN8;
	outb(lcr, LCR_ADDRESS);
	//Divisor to set bit rate of 1200 bps
	dll = UART_DIV_1200;
  outb(dll, DLL_ADDRESS);
  outb(dlm, DLM_ADDRESS); 
	//Reset DLAB to access other registers later
	lcr &= ~UART_LCR_DLAB;
	outb(lcr, LCR_ADDRESS);
	//Disable interrupts
	outb(ier, IER_ADDRESS);

  return 0;
}

static void serp_exit(void) {

	release_region(PORT_UART, 7);
  	kfree(file);
	kfree(inode);
	cdev_del(&serp_device->cdev);
	unregister_chrdev_region(*device, 1);
	kfree(device);
  	printk(KERN_INFO "EXITED WITH SUCCESS\n");
}

int serp_open(struct inode *inodep, struct file *filep) {
	//copied from echo.c - from pratical classes
	struct serp_device *serp_device_pointer;
	serp_device_pointer = container_of(inodep->i_cdev, struct serp_device, cdev);
	filep->private_data = serp_device_pointer;
	nonseekable_open(inodep,filep);
	printk(KERN_INFO "SERP FILE OPENED SUCCESSFULLY\n");
	return 0;
}

int serp_close(struct inode *inodep, struct file *filep) {
	printk(KERN_INFO "SERP CLOSED WITH SUCCESS\n");
	return 0;
}

ssize_t serp_write(struct file *filep, const char __user *buff, size_t count, loff_t *offp) {
	unsigned char lsr = 0;
	unsigned char *kernel_buff;
	unsigned long bytes_not_copied = 0, char_count = 0, number_of_characters_written = 0;

	kernel_buff = (unsigned char*) kzalloc(sizeof(unsigned char)*count, GFP_KERNEL); //Allocate kernel level buffer
	
	//copy user level buffer to kernel level buffer
	bytes_not_copied = copy_from_user(kernel_buff, buff, count); // returns number of bytes it couldnt copy

	if(bytes_not_copied != 0) {
		printk(KERN_ALERT "THERE ARE %lu BYTES NOT COPIED - ERROR\n", bytes_not_copied);
		kfree(kernel_buff);
		return -EAGAIN;
	}

	while(char_count < count) {
		lsr = inb(LSR_ADDRESS);

		//checking whether the transmit holding register is empty (THRE)
		if(lsr & UART_LSR_THRE) {
			outb(kernel_buff[char_count], TRANSMIT_ADRESS); 
			char_count++;
		}
		else {
			//the THR is not empty, the driver should wait
			schedule();
		}
	}

	number_of_characters_written = char_count;
	kfree(kernel_buff);
	return count-number_of_characters_written;
}

ssize_t serp_read(struct file *filep, char __user *buff, size_t count, loff_t *offp) {
	
	unsigned char *kernel_buff;		
	unsigned char lsr = 0;
	unsigned long clock=0, bytes_not_copied = 0, characters_read = 0, char_count = 0;
	
	//Allocate kernel level buffer
	kernel_buff = (unsigned char*) kzalloc(sizeof(unsigned char)*count, GFP_KERNEL);
	
	clock = jiffies;
	
	while(char_count < count) {

		lsr = inb(LSR_ADDRESS);
		//signals that at least one character in the buffer receiver register was overwritten by another character, i.e. lost
		if(lsr & UART_LSR_OE) {	
			kfree(kernel_buff);
			return -EIO;
		}
		//Data Ready bit - the UART puts a character received in the Receiver Buffer Register
		else if(lsr & UART_LSR_DR) { 
			
			kernel_buff[char_count] = inb(RECEIVE_ADRESS); //read a character via UART by the receiver if it has data

			if((kernel_buff+char_count) == NULL) { //no data
				kfree(kernel_buff);
				printk(KERN_ALERT "UART_RX DATA ERROR\n");
				return -EBADE;
			}
			clock = jiffies;
			char_count++;

			continue;
		}
		//timeout/break - not data ready bit or time exceeded
		else if((!(lsr & UART_LSR_DR)) && (jiffies-clock > 5*HZ)) {
			printk(KERN_ALERT "-> WAITING TO MUCH FOR DATA...BREAKING: CTRL+C TO EXIT.\n");

			break;
		}
		
	
	}

	characters_read = char_count;

	bytes_not_copied = copy_to_user(buff, kernel_buff, characters_read); // returns number of bytes it couldnt copy
	if(bytes_not_copied != 0) {
		kfree(kernel_buff);
		return -EBADE;
	}

	kfree(kernel_buff);
	return count-bytes_not_copied;
}

module_init(serp_init);
module_exit(serp_exit);
