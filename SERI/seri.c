#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/fcntl.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include "serial_reg.h"

#define PORT_UART 0x3F8	//From VirtualBox's defaults for COM1
#define LSR_ADDRESS (PORT_UART + UART_LSR)
#define LCR_ADDRESS (PORT_UART + UART_LCR)
#define DLL_ADDRESS (PORT_UART + UART_DLL)
#define DLM_ADDRESS (PORT_UART + UART_DLM)
#define IER_ADDRESS (PORT_UART + UART_IER)
#define IIR_ADDRESS (PORT_UART + UART_IIR)
#define FCR_ADDRESS (PORT_UART + UART_FCR)
#define RECEIVE_ADDRESS (PORT_UART + UART_RX)
#define TRANSMIT_ADDRESS (PORT_UART + UART_TX)
#define KFIFO_SIZE 32768

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Celso e Rodrigo");

static int seri_open (struct inode *inodep, struct file *filep);
static int seri_write (struct file *filep, const char __user *buff, size_t count, loff_t *offp);
ssize_t seri_read (struct file *filep, char __user *buff, size_t count, loff_t *offp);
static int seri_close (struct inode *inodep, struct file *filep);
irqreturn_t short_interrupt(int irq, void *dev_id);

struct seri_dev_t
{
	struct cdev cdev;
	struct semaphore mutex;		//Kernel level semaphore
	int sem_status;						//Should not access semaphore count directly (has type atomic_t), so another variable tracks the count
	struct kfifo *rxfifo;
	struct kfifo *txfifo;
	wait_queue_head_t rxwq;
	wait_queue_head_t txwq;
	spinlock_t rxfifo_lock;
	spinlock_t txfifo_lock;
};

struct file_operations seri_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = seri_read,
	.write = seri_write,
	.open = seri_open,
	.release = seri_close,
};

dev_t *device;
struct file *file;
struct inode *inode;
static struct seri_dev_t *seri_device;

static int seri_init(void)
{
	//Store dev_t structure
	device = kmalloc(sizeof(dev_t), GFP_KERNEL);

  if (device == NULL)
  {
    printk(KERN_ALERT "NO MEMORY FOR THE DEVICE - ERROR\n");
    return -1;
  }

	//Ask kernel to allocate a major device number to the device
	int region = alloc_chrdev_region(device, 0, 1, "seri");

  if (region < 0)
  {
    printk(KERN_ALERT "DEVICE MAJOR NUMBER - ERROR\n");
    return -1;
  }

	//Store seri_device structure
  seri_device = kmalloc(sizeof (struct seri_dev_t), GFP_KERNEL);

  if (seri_device == NULL)
  {
    printk(KERN_ALERT "SER_DEV_T STRUCT - ERROR\n");
    return -1;
  }

	//Initialize cdev. Already allocated, embedded in seri_device
  cdev_init(&seri_device->cdev, &seri_fops);
	(&seri_device->cdev)->owner = THIS_MODULE;

	//Initialize kfifo structures
	seri_device->rxfifo = kfifo_alloc(KFIFO_SIZE, GFP_KERNEL, &(seri_device->rxfifo_lock));
	seri_device->txfifo = kfifo_alloc(KFIFO_SIZE, GFP_KERNEL, &(seri_device->txfifo_lock));

	//Initialize queues for read and write processes/threads
	init_waitqueue_head(&(seri_device->rxwq));
	init_waitqueue_head(&(seri_device->txwq));

	//Initialize spin locks needed by each kfifo
	spin_lock_init(&(seri_device->rxfifo_lock));
	spin_lock_init(&(seri_device->txfifo_lock));

	//Initialize mutex semaphore with 1 to prevent race conditions (enhancement)
	//Allow only one process to use the device at a time.
	sema_init(&(seri_device->mutex), 1);
	seri_device->sem_status = 1;

	//Register device in the kernel
	int add_dev = cdev_add(&seri_device->cdev, *device, 1);

	if(add_dev < 0)
	{
		printk(KERN_ALERT "REGISTER THE DD - ERROR\n");
		return -1;
	}

	//Store inode structure
	inode = kmalloc(sizeof(struct inode), GFP_KERNEL);

	if(inode == NULL)
	{
		printk(KERN_ALERT "INODE MEMORY - ERROR\n");
		return -1;
	}

	inode->i_cdev = &seri_device->cdev;

	//Store a file descriptor structure
	file = kmalloc(sizeof(struct file), GFP_KERNEL);

	if(file == NULL)
	{
		printk(KERN_ALERT "FILE MEMORY - ERROR\n");
		return -1;
	}

	//Request I/O Port range used by the serial port device
 	if(request_region(PORT_UART, 7, "seri") == NULL)
	{
  	 printk(KERN_ALERT "REQUEST REGION - ERROR\n");
		 return -1;
  }


	//even parity, 2 stop bits, 8-bit words, set DLAB to access DLL and DLM
	unsigned char lcr = UART_LCR_DLAB | UART_LCR_EPAR | UART_LCR_PARITY | UART_LCR_STOP | UART_LCR_WLEN8;
	outb(lcr, LCR_ADDRESS);
	//Divisor to set bit rate of 1200 bps
	unsigned char dll = UART_DIV_1200;
  outb(dll, DLL_ADDRESS);
  outb(0, DLM_ADDRESS);
	//Reset DLAB to access other registers later
	lcr &= ~UART_LCR_DLAB;
	outb(lcr, LCR_ADDRESS);
	//Enable RX and TX interrupts
	unsigned char ier = UART_IER_RDI | UART_IER_THRI;
	outb(ier, IER_ADDRESS);

	if(request_irq(4, short_interrupt, SA_INTERRUPT, "seri", seri_device) != 0)
	{
		printk(KERN_ALERT "REGISTER INTERRUPT REQUEST - ERROR\n");
		return -1;
	}

	return 0;
}

static void seri_exit(void)
{
	//Reverse seri_init: free resources
	free_irq (4, seri_device);
	release_region(PORT_UART, 7);
  kfree(file);
	kfree(inode);
	cdev_del(&seri_device->cdev);
	kfifo_free(seri_device->txfifo);
  kfifo_free(seri_device->rxfifo);
	unregister_chrdev_region(*device, 1);
	kfree(device);
  printk(KERN_INFO "EXITED WITH SUCCESS\n");
}


static int seri_open (struct inode *inodep, struct file *filep)
{
	struct seri_dev_t *seri_device_pointer;
	// Takes cdev member struct out of the struct it is embbedded in (seri_dev_t)
	seri_device_pointer = container_of(inodep->i_cdev, struct seri_dev_t, cdev);
	filep->private_data = seri_device_pointer;

		if (seri_device->sem_status == 0)
	{
		printk(KERN_ALERT "DEVICE IS ALREADY IN USE - ERROR\n");
		return -1;
	}

	//Acquire mutex - preventing more than 1 user enhancement
	down_interruptible(&(seri_device->mutex));
	seri_device->sem_status--;

	/*
	//Old code which did not run printk and return because process
	//was blocked right before when semaphore was decremented
	if (down_interruptible(&(seri_device_pointer->mutex)))
	{
		printk(KERN_ALERT "DEVICE IS ALREADY IN USE - ERROR\n");
		return -1;
	}
	*/

	nonseekable_open(inodep,filep);
	printk(KERN_INFO "SERI FILE OPENED SUCCESSFULLY\n");

  return 0;
}

static int seri_close (struct inode *inodep, struct file *filep)
{
	//Release mutex - allow other process to use the device
	up(&(seri_device->mutex));
	seri_device->sem_status++;
	printk(KERN_INFO "SERI CLOSED WITH SUCCESS\n");

	return 0;
}

static int seri_write (struct file *filep, const char __user *buff, size_t count, loff_t *offp)
{
	//Allocate kernel level buffer
	unsigned char *kernel_buff = (unsigned char*) kzalloc(sizeof(unsigned char)*count, GFP_KERNEL);

	if (kernel_buff == NULL)
	{
			printk(KERN_ALERT "NO MEMORY FOR KERNEL WRITE BUFFER - ERROR\n");
			return -1;
	}

	//copy user level buffer to kernel level buffer
	unsigned int bytes_not_copied = copy_from_user(kernel_buff, buff, count); // returns number of bytes it couldnt copy

	if(bytes_not_copied != 0) {
		printk(KERN_ALERT "THERE ARE %d BYTES NOT COPIED - ERROR\n", bytes_not_copied);
		kfree(kernel_buff);
		return -1;
	}

	//Honor the O_NONBLOCK flag: return immediately is no I/O will be made
	if(kfifo_len(((struct seri_dev_t *)filep->private_data)->txfifo) == KFIFO_SIZE && (filep->f_flags & O_NONBLOCK))
	{
		kfree(kernel_buff);
		printk(KERN_ALERT "RX KFIFO IS FULL - RETURNED IMMEDIATELY\n");
		return -1;
	}

	//copy from kernel level buffer to TX kfifo
	unsigned int bytes_in_kfifo = kfifo_put(((struct seri_dev_t *)filep->private_data)->txfifo, kernel_buff, count);

	//Start transmitting the first char in kfifo so that interrupts are generated
	unsigned char lsr = inb(LSR_ADDRESS);
	unsigned char first_char_fifo;

	if (lsr & UART_LSR_THRE)
	{
		kfifo_get(((struct seri_dev_t *)filep->private_data)->txfifo, &first_char_fifo, 1);
		outb(first_char_fifo, TRANSMIT_ADDRESS);
	}

	kfree(kernel_buff);

	return bytes_in_kfifo;
}

ssize_t seri_read (struct file *filep, char __user *buff, size_t count, loff_t *offp)
{
	//Allocate kernel level buffer
	unsigned char *kernel_buff = (unsigned char*) kzalloc(sizeof(unsigned char)*count, GFP_KERNEL);

	if (kernel_buff == NULL)
	{
			printk(KERN_ALERT "NO MEMORY FOR KERNEL READ BUFFER - ERROR\n");
			return -1;
	}

	unsigned int bytes_in_kfifo;
	unsigned int wait_next = 1;

	//rxfifo size will increase at each interrupt until it reaches the argument count
	while ( (bytes_in_kfifo = kfifo_len(((struct seri_dev_t *)filep->private_data)->rxfifo)) < count)
	{

		if ((bytes_in_kfifo == 0) && (filep->f_flags & O_NONBLOCK)) 
		{
			//Honor the O_NONBLOCK flag: return immediately is no I/O is to be made
			kfree(kernel_buff);
			printk(KERN_ALERT "RXFIFO IS EMPTY - RETURNED IMMEDIATELY\n");
			return -1;
		}

		wait_next = wait_event_interruptible_timeout(((struct seri_dev_t *)filep->private_data)->rxwq, kfifo_len(((struct seri_dev_t *)filep->private_data)->rxfifo) > bytes_in_kfifo, 1000);

		if (wait_next == -ERESTARTSYS)
		{
				//sleeping was interrupted by signal, allows for interrupting read process (enhancement)
				kfree(kernel_buff);
				printk(KERN_ALERT "READ PROCESS WAS INTERRUPTED BY USER ACTION\n");
				return -ERESTARTSYS;
		}
	}

	bytes_in_kfifo = kfifo_len(((struct seri_dev_t *)filep->private_data)->rxfifo);

	//copy from RX kfifo to kernel level buffer
	kfifo_get(((struct seri_dev_t *)filep->private_data)->rxfifo, kernel_buff, bytes_in_kfifo);

	//copy kernel level buffer to user level buffer
	unsigned long bytes_not_copied = copy_to_user(buff, kernel_buff, bytes_in_kfifo);

	if(bytes_not_copied != 0)
	{
		kfree(kernel_buff);
		return -1;
	}

	kfree(kernel_buff);

	return count-bytes_not_copied;
}

irqreturn_t short_interrupt (int irq, void *dev_id)
{
	//Read this register to find out the source of the interrupt
	unsigned char iir = inb(IIR_ADDRESS);
	unsigned char rbr, thr;

	//In case new receiver data available:
	//Read value from rbr register to kfifo and wake up sleeping process who called read
	if (iir & UART_IIR_RDI)
	{
		rbr = inb(RECEIVE_ADDRESS);
		kfifo_put(((struct seri_dev_t *)dev_id)->rxfifo, &rbr, 1);
		wake_up_interruptible(&(((struct seri_dev_t *)dev_id)->rxwq));
	}

	//In case transmitter register is empty and ready
	//Place value from kfifo in thr register and wake up sleeping process who called write
	if (iir & UART_IIR_THRI && kfifo_get(((struct seri_dev_t *)dev_id)->txfifo, &thr, 1) > 0)
	{
		outb(thr, TRANSMIT_ADDRESS);
		wake_up_interruptible(&(((struct seri_dev_t *)dev_id)->txwq));
	}

	return IRQ_HANDLED;
}

module_init(seri_init);
module_exit(seri_exit);
