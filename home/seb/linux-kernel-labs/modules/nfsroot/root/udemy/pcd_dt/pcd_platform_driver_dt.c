/* 
 * linux platform driver handling multiple pseudo-char devices
 * @author sebastian.popa@raptor-technologies.ro
 */

/* Includes */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_device.h>    

#include "platform.h"

/* Defines */
/* Concatenate functio name in printk */
#ifdef pr_fmt
    #undef pr_fmt
    #define pr_fmt(fmt) "%s: " fmt,__func__
#endif

#define PARENT_NULL         NULL
#define DRIVER_DATA_NULL    NULL

/* Device specific configs */
struct device_config 
{
    int config_item1;
    int config_item2;
};

enum pcdev_names
{
    PCDEVA1X,
    PCDEVB1X,
    PCDEVC1X,
    PCDEVD1X,
};

struct device_config pcdev_config[] = {
    [PCDEVA1X] = {.config_item1 = 60, .config_item2 = 21},
    [PCDEVB1X] = {.config_item1 = 50, .config_item2 = 22},
    [PCDEVC1X] = {.config_item1 = 40, .config_item2 = 23},
    [PCDEVD1X] = {.config_item1 = 30, .config_item2 = 24},
};

/* Device private data structure */
struct pcdev_private_data
{
    struct pcdev_platform_data pdata;  // platform device custom data
	char *buffer;		       // pointer to start address
    dev_t dev_num;             // device number
	struct cdev cdev;	       // cdev structure, device specific
};

/* Driver private data structure */
struct pcdrv_private_data
{
	int total_devices;
    dev_t dev_num_base;                 /* base number for devices */
    struct class *class_pcd;            /* device class in sysfs */
    struct device *device_pcd;          /* device pointer */
};

/* Global driver private data struct */
struct pcdrv_private_data pcdrv_data;

int check_permission(int dev_perm, int acc_mode)
{
    if (dev_perm == RDWR)
        return 0;

    if ((dev_perm == RDONLY) && ((acc_mode & FMODE_READ) && !(acc_mode & FMODE_WRITE)))
        return 0;

    if ((dev_perm == WRONLY) && (!(acc_mode & FMODE_READ) && (acc_mode & FMODE_WRITE)))
        return 0;

    return -EPERM;
}

/* Char Device File Operations */
int pcd_open(struct inode *inode, struct file *filp)
{
    int ret;
    int minor;
    struct pcdev_private_data *dev_data;

    minor = MINOR(inode->i_rdev);
    pr_info("minor access = %d\n", minor);

    /* get device's private data structure, starting from cdev member of inode*/
    dev_data = container_of(inode->i_cdev, struct pcdev_private_data, cdev);

    /* supply device private data to other methods of the driver*/
    filp->private_data = dev_data;

    /* check permission */
    ret = check_permission(dev_data->pdata.perm, filp->f_mode);

    (!ret)?pr_info("open successfull\n"):pr_info("open unsucessfull\n");

    return ret;
}
 
int pcd_release(struct inode *inode, struct file *filp)
{
    pr_info("close sucessful\n");
 	return 0;
}
 
ssize_t pcd_read(struct file *filp,	char __user *buff, size_t count, loff_t *f_pos)
{   
    struct pcdev_private_data *dev_data = (struct pcdev_private_data *)filp->private_data;
    int max_size = dev_data->pdata.size;

    pr_info("read requested for %zu bytes\n", count);
    pr_info("current file pos: %lld\n", *f_pos);

    /* adjust count */
    if ((*f_pos + count) > max_size) {
        count = max_size - *f_pos;
    }

    /* copy to user */
    if (copy_to_user(buff, dev_data->buffer + (*f_pos), count)) {
        return -EFAULT;
    }

    /* update current file position */
    *f_pos += count;

    /* number of bytes succesfully read */
    pr_info("bytes read: %zu\n", count);
    pr_info("updated file position: %lld\n", *f_pos);

    return count;
}
 
ssize_t pcd_write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
    struct pcdev_private_data *pcdev_data = (struct pcdev_private_data *)filp->private_data;
    int max_size = pcdev_data->pdata.size;

    pr_info("write requested for %zu bytes\n", count);
    pr_info("current file pos: %lld\n", *f_pos);

    /* adjust count */
    if ((*f_pos + count) > max_size) {
        count = max_size - *f_pos;
        pr_info("write exceeds memory limit!\n");
    }

    if (!count) {
        return -ENOMEM;
    }

    /* copy from user */
    if (copy_from_user(pcdev_data->buffer + (*f_pos), buff, count)) {
        return -EFAULT;
    }

    /* update current file position */
    *f_pos += count;

    /* number of bytes succesfully written */
    pr_info("bytes written: %zu\n", count);
    pr_info("updated file position: %lld\n", *f_pos);

    return count;
}
 
loff_t pcd_lseek(struct file *filp, loff_t offset, int whence)
{
    struct pcdev_private_data *pcdev_data = (struct pcdev_private_data *)filp->private_data;
    int max_size = pcdev_data->pdata.size;

    loff_t temp = 0;

    pr_info("llseek requested\n");
    pr_info("current file pos: %lld\n", filp->f_pos);
    
    switch(whence) {
    case SEEK_SET:
        if ((offset > max_size) || (offset < 0)) {
            return -EINVAL;
        }
        
        filp->f_pos = offset;
        break;
    case SEEK_CUR:
        temp = filp->f_pos + offset;
        if ((temp > max_size) || (temp < 0)) {
            return -EINVAL;
        }

        filp->f_pos += offset;
        break;
    case SEEK_END:
        temp = max_size + offset;
        if ((temp > max_size) || (temp < 0)) {
            return -EINVAL;
        }

        filp->f_pos = max_size + offset;
        break;
    default:
        return -EINVAL;
        break;
    } /* switch*/

    pr_info("updated file position: %lld\n", filp->f_pos);

    return filp->f_pos;
}

/* Initialize file operations*/
struct file_operations pcd_fops = 
{
    .owner = THIS_MODULE,
    .open = &pcd_open,
    .release = &pcd_release,
    .write = &pcd_write,
    .read = &pcd_read,
    .llseek = &pcd_lseek,
};

struct pcdev_platform_data* pcdev_get_platdata_from_dt(struct device *dev)
{
    struct device_node *dev_node = dev->of_node;
    struct pcdev_platform_data *pdata;

    if (!dev_node) {
        /* no device tree setup*/
        return NULL;
    }

    /* device tree data handling with functions in linux/of.h*/
    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata) {
        dev_info(dev, "Cannot allocate memory\n");
        return ERR_PTR(-ENOMEM);
    }

    /* get serial number*/
    if (of_property_read_string(dev_node, "org,device-serial-num", &pdata->serial_number)) {
        dev_info(dev, "Missing serial number proprety\n");
        return ERR_PTR(-EINVAL);
    }

    /* get size */
    if (of_property_read_u32(dev_node, "org,size", &pdata->size)) {
        dev_info(dev, "Missing size property\n");
        return ERR_PTR(-EINVAL);
    }

    /* get permisison */
    if (of_property_read_u32(dev_node, "org,perm", &pdata->perm)) {
        dev_info(dev, "Missing permission property\n");
        return ERR_PTR(-EINVAL);
    }

    return pdata;
}

/* called when matched platform device is found */
int pcd_platform_driver_probe(struct platform_device *pdev)
{    
    int ret;
    struct pcdev_private_data *dev_data;
    struct pcdev_platform_data *pdata;
    struct device *dev = &pdev->dev;
    int driver_data;

    dev_info(dev, "Device detected!\n");

    /*1. Get the platform data */
    /* Test for Device Tree initialization*/
    pdata = pcdev_get_platdata_from_dt(dev);
    if (IS_ERR(pdata)) {
        return PTR_ERR(pdata);
    }

    /* Get device initialization from device setup*/
    if (!pdata) {
        pdata = (struct pcdev_platform_data *)dev_get_platdata(dev);
        if (!pdata) {
            pr_info("No platform data available\n");
            return -EINVAL;
        }
        
        driver_data = pdev->id_entry->driver_data; /* device setup platform data */
    } else {
        // match = of_match_device(pdev->dev.driver->of_match_table, &pdev->dev);
        // driver_data = (int) match->data;
        driver_data = (int) of_device_get_match_data(dev); /* device tree platform data */
    }

    /*2. Dynamically allocate memory for the device private data */
    dev_data = devm_kzalloc(&pdev->dev, sizeof(struct pcdev_private_data), GFP_KERNEL);
    if (!dev_data) {
        pr_err("Cannot allocate memory\n");
        return -ENOMEM;
    }

    /* Save device private data pointer in platfrom device structure */
    // pdev->dev.driver_data = dev_data;
    dev_set_drvdata(&pdev->dev, dev_data);

    /* Copy platform data into device data */
    dev_data->pdata.size = pdata->size;
    dev_data->pdata.perm = pdata->perm;
    dev_data->pdata.serial_number = pdata->serial_number;

    /* Print device private data */
    pr_info("Device serial number = %s\n", dev_data->pdata.serial_number);
    pr_info("Device size = %d\n", dev_data->pdata.size);
    pr_info("Device permission = %x\n", dev_data->pdata.perm);
    
    /* Print device specific config data */
    pr_info("Config item 1 = %d\n", pcdev_config[driver_data].config_item1);
    pr_info("Config item 2 = %d\n", pcdev_config[driver_data].config_item2);

    /*3. Dynamically allocate memory for the device buffer using size
     *   information from the platform data
     */
    dev_data->buffer = devm_kzalloc(&pdev->dev, dev_data->pdata.size, GFP_KERNEL);
    if (!dev_data->buffer) {
        pr_err("Cannot allocate memory\n");
        return -ENOMEM;
    }

    /*4. Get the device number */
    dev_data->dev_num = pcdrv_data.dev_num_base + pcdrv_data.total_devices;

    /*5. Do cdev init and cdev add */
    cdev_init(&dev_data->cdev, &pcd_fops);
    dev_data->cdev.owner = THIS_MODULE;

    ret = cdev_add(&dev_data->cdev, dev_data->dev_num, 1);
    if (ret < 0) {
        pr_err("cdev add failed\n");
        return ret;
    }

    /*6. Create device file for the detected platform device */
    /*   Populate sysfs with device information */
    pcdrv_data.device_pcd = device_create(pcdrv_data.class_pcd, 
                                            dev, 
                                            dev_data->dev_num, 
                                            DRIVER_DATA_NULL, 
                                            "pcdev-%d", pcdrv_data.total_devices);
    if (IS_ERR(pcdrv_data.device_pcd)) {
        pr_err("device creation failed\n");
        ret = PTR_ERR(pcdrv_data.device_pcd);
        cdev_del(&dev_data->cdev);
        return ret;
    }

    pcdrv_data.total_devices++;

    pr_info("Probe function sucessful\n");    
    return 0;
}

/* called when the device is removed from the system */
int pcd_platform_driver_remove(struct platform_device *pdev)
{
    pr_info("A device has been removed\n");
    return 0;

#if 0
    struct pcdev_private_data *dev_data = dev_get_drvdata(&pdev->dev);

    /*1. Remove a device that was created with device_create */
    device_destroy(pcdrv_data.class_pcd, dev_data->dev_num);

    /*2. Remove a cdev entry from the system */
    cdev_del(&dev_data->cdev);

    /*3. Memory is freed automatically by devm_kzalloc */

    pcdrv_data.total_devices--;

    pr_info("A device is removed\n");
    return 0;
#endif
}

/* Supported device id table */
struct platform_device_id pcdevs_ids[] = {
    // store index from device_config pcdev_config inside .driver_data
    {.name = "pcdev-A1x", .driver_data = PCDEVA1X}, 
    {.name = "pcdev-B1x", .driver_data = PCDEVB1X},
    {.name = "pcdev-C1x", .driver_data = PCDEVC1X},
    { } // NULL terminated array
};

struct of_device_id org_pcdev_dt_match[] = {
    {.compatible = "pcdev-A1x", .data = (void *)PCDEVA1X},
    {.compatible = "pcdev-B1x", .data = (void *)PCDEVB1X},
    {.compatible = "pcdev-C1x", .data = (void *)PCDEVC1X},
    {.compatible = "pcdev-D1x", .data = (void *)PCDEVD1X},
    {}
};

struct platform_driver pcd_platform_driver = {
    .probe = pcd_platform_driver_probe,
    .remove = pcd_platform_driver_remove,
    .id_table = pcdevs_ids,
    .driver = {
        .name = "pseudo-char-device",
        .of_match_table = org_pcdev_dt_match
    }
};

/* Module entry / exit */
#define MAX_DEVICES           10

static int __init pseudo_char_driver_init(void)
{
    int ret;

    /*1. Dynamically allocate a device number for MAX_DEVICES */
    ret = alloc_chrdev_region(&pcdrv_data.dev_num_base, 
                        0, 
                        MAX_DEVICES, 
                        "pseudo_char_devices"
                        );
    if (ret < 0) {
        pr_err("Alloc chrdev failed\n");
        return ret;
    }

    /*2. Create device class under /sys/class */
    pcdrv_data.class_pcd = class_create(THIS_MODULE, "pcd_class");
    if (IS_ERR(pcdrv_data.class_pcd)) {
        pr_err("class creation failed\n");
        ret = PTR_ERR(pcdrv_data.class_pcd);
        unregister_chrdev_region(pcdrv_data.dev_num_base, MAX_DEVICES);
        return ret;
    }

    /*3. Register a platform driver */
    platform_driver_register(&pcd_platform_driver);

    pr_info("pcd platform driver loaded\n");

    return 0;
}

static void __exit pseudo_char_driver_cleanup(void)
{
    /*1. Unregister platform driver*/
    platform_driver_unregister(&pcd_platform_driver);

    /*2. Class destroy */
    class_destroy(pcdrv_data.class_pcd);

    /*3. Unregister device numbers for MAX_DEVICES */
    unregister_chrdev_region(pcdrv_data.dev_num_base, MAX_DEVICES);

    pr_info("pcd platform driver unloaded\n");
}

/* Init / Exit functions registration */
module_init(pseudo_char_driver_init);
module_exit(pseudo_char_driver_cleanup);

/* Module metadata */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Popa: sebastian.popa@raptor-technologies.ro");
MODULE_DESCRIPTION("Pseudo Char Module handling multiple devices");
MODULE_INFO(board, "Beaglebone black Rev C1");

/* End of file */