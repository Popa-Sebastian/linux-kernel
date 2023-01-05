// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>  
#include <linux/pm_runtime.h>
#include <linux/serial_reg.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>


/* Add your code here */
#define SERIAL_DEVICE       1
#define MAX_WRITE_SIZE      256
#define SERIAL_BUFSIZE      16

#define SERIAL_RESET_COUNTER 0
#define SERIAL_GET_COUNTER   1

/* Serial device private structure */
struct serial_dev {
        void __iomem *regs;             /* base of memory mapped io */
        struct miscdevice miscdev;      /* misc device */
        unsigned int count;             /* number of char printed to console */
        char buf[SERIAL_BUFSIZE];       /* fifo */
        unsigned int buf_rd;            /* read index */
        unsigned int buf_wr;            /* write index */
        unsigned int current_size;      /* current fifo size */
        wait_queue_head_t wait;         /* wait queue */
        spinlock_t lock;                /* spinlock */
};

/* UART register manipulation */
static u32 reg_read(struct serial_dev *serial, unsigned int reg)
{       
        return readl(serial->regs + (reg * 4));
}

static void reg_write(struct serial_dev *serial, u32 val, unsigned int reg)
{       
        writel(val, (serial->regs + (reg * 4)));
}

static void char_write(struct serial_dev *serial, char val)
{
        u32 reg_value;

        /* check line status register for tx empty */
        do {
                reg_value = reg_read(serial, UART_LSR);
                cpu_relax();
        } while (!(reg_value & UART_LSR_THRE));
                
        reg_write(serial, val, UART_TX);
}

/* System calls */
ssize_t serial_read(struct file *file, char __user *user_buf, size_t count, loff_t *f_pos)
{
        struct serial_dev *serial = container_of(file->private_data, struct serial_dev, miscdev);
        char read_val;

        /* no data available */
        if (serial->buf_rd == serial->buf_wr)
                wait_event_interruptible(serial->wait, (serial->buf_rd != serial->buf_wr));

        /* Read 1 character at a time */
        spin_lock(&serial->lock);
        read_val = serial->buf[serial->buf_rd];
        serial->buf_rd = (serial->buf_rd + 1) % SERIAL_BUFSIZE;
        serial->current_size--;
        spin_unlock(&serial->lock);

        if (copy_to_user(user_buf, &read_val, sizeof(read_val)))
                return -EINVAL;
        
        return 1;
}

ssize_t serial_write(struct file *file, const char __user *user_buff, size_t count, loff_t *f_pos)
{
        int i, ret;
        char print_buff[MAX_WRITE_SIZE];
        struct serial_dev *serial = container_of(file->private_data, struct serial_dev, miscdev);
        pr_info("Writing to: %s\n", serial->miscdev.name);

        if (count >= MAX_WRITE_SIZE) {
                return -ENOMEM;
        }

        ret = copy_from_user(&print_buff, user_buff, count);
        if (ret)
                return -ENOMEM;

        for (i = 0; i < count; i++) {
                /* send '\n\r' instead of user space '\n'*/
                if (print_buff[i] == '\n') {
                        char_write(serial, '\n');
                        char_write(serial, '\r');
                        serial->count += 2;
                }
                else {
                        char_write(serial, print_buff[i]);
                        serial->count++;
                }
        }

        return count;
}

long serial_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        struct serial_dev *serial = container_of(file->private_data, struct serial_dev, miscdev);

        switch (cmd)
        {
        case SERIAL_RESET_COUNTER:
                serial->count = 0;
                pr_info("%s counter reset to 0\n", serial->miscdev.name);
                break;
        case SERIAL_GET_COUNTER:
                if (copy_to_user((u32 *)arg, &(serial->count), sizeof(serial->count))) {
                        pr_err("Data read failed\n");
                }
                break;
        default:
                pr_info("Default\n");
                break;
        }

        return 0;
}

/* File operations */
struct file_operations misc_fops = {
        .owner = THIS_MODULE,
        .read = serial_read,
        .write = serial_write,
        .unlocked_ioctl = serial_ioctl,
};

/* IRQ Handler: Receive Data Interrupt */
static irqreturn_t serial_rx_irq(int irq, void *devp)
{
        struct serial_dev *serial = (struct serial_dev *)devp;
        char read_val;
        read_val = reg_read(serial, UART_RX);

        /* Echo to terminal */
        char_write(serial, read_val);

        if (serial->current_size == SERIAL_BUFSIZE) {
                pr_err("FIFO FULL\n");
                return IRQ_HANDLED;
        }

        /* Copy data to FIFO */
        spin_lock(&serial->lock);
        serial->buf[serial->buf_wr] = read_val;
        serial->buf_wr = (serial->buf_wr + 1) % SERIAL_BUFSIZE;
        serial->current_size++;
        spin_unlock(&serial->lock);

        /* Wake up read function */
        wake_up(&serial->wait);

        return IRQ_HANDLED;
}

static int serial_probe(struct platform_device *pdev)
{
        int ret;
        unsigned int baud_divisor, uartclk;
        struct serial_dev *serial;
        struct resource *res;
        int irq;

        pr_info("Called %s\n", __func__);

        /* Alocate memory for device structure */
        serial = devm_kzalloc(&pdev->dev, sizeof(*serial), GFP_KERNEL);
        if (!serial)
                return -ENOMEM;

        /* Save device struct pointer in driver data */
        platform_set_drvdata(pdev, serial);

        /* Alocate memory for device registers */
        serial->regs = devm_platform_ioremap_resource(pdev, 0);
        if (IS_ERR(serial->regs))
                return PTR_ERR(serial->regs);
        
        /* Power management initialization */
        pm_runtime_enable(&pdev->dev);
        pm_runtime_get_sync(&pdev->dev);
	
        /* Configure the baud rate to 115200 */
        ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
                                &uartclk);
        if (ret) {
                dev_err(&pdev->dev,
                        "clock-frequency property not found in Device Tree\n");
                return ret;
        }

        baud_divisor = uartclk / 16 / 115200;
        reg_write(serial, 0x07, UART_OMAP_MDR1);
        reg_write(serial, 0x00, UART_LCR);
        reg_write(serial, UART_LCR_DLAB, UART_LCR);
        reg_write(serial, baud_divisor & 0xff, UART_DLL);
        reg_write(serial, (baud_divisor >> 8) & 0xff, UART_DLM);
        reg_write(serial, UART_LCR_WLEN8, UART_LCR);
        reg_write(serial, 0x00, UART_OMAP_MDR1);

        /* Clear UART FIFOs */
        reg_write(serial, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);

        /* Enable Interrupts */
        reg_write(serial, UART_IER_RDI, UART_IER);

        /* Initialize miscdevice structure*/
        serial->miscdev.minor = MISC_DYNAMIC_MINOR;
        
        res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if (IS_ERR(res))
                return PTR_ERR(res);

        serial->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "serial-%x", res->start);
        if (IS_ERR(serial->miscdev.name))
                return PTR_ERR(serial->miscdev.name);

        serial->miscdev.fops = &misc_fops;
        serial->miscdev.parent = &pdev->dev;

        /* Register IRQ */
        irq = platform_get_irq(pdev, 0);
        ret = devm_request_irq(&pdev->dev, irq, serial_rx_irq, 0, pdev->name, serial);
        if (ret < 0) {
                dev_err(&pdev->dev, "failed to attach serial irq\n");
                return ret;
        }

        /* Init wait queue */
        init_waitqueue_head(&serial->wait);

        /* Init spinlock */
        spin_lock_init(&serial->lock);

        /* Register misc device */
        misc_register(&serial->miscdev);

        dev_info(&pdev->dev, "Probe Succesful");

	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
        struct serial_dev *serial;

	pr_info("Called %s\n", __func__);
        serial = (struct serial_dev *)platform_get_drvdata(pdev);

        misc_deregister(&serial->miscdev);
        pm_runtime_disable(&pdev->dev);

        return 0;
}

struct of_device_id serial_of_match[] = {
        {.compatible = "bootlin,serial", .data = (const void *)SERIAL_DEVICE},
        { }
};
MODULE_DEVICE_TABLE(of, serial_of_match);

static struct platform_driver serial_driver = {
        .driver = {
                .name = "serial",
                .of_match_table = serial_of_match,
                .owner = THIS_MODULE,
        },
        .probe = serial_probe,
        .remove = serial_remove,
};
module_platform_driver(serial_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Popa: sebastian.popa@raptor-technologies.ro");
MODULE_DESCRIPTION("Misc serial device driver");

