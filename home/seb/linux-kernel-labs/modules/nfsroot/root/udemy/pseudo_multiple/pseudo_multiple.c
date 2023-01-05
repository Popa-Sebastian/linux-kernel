/* 
 * linux driver handling multiple pseudo char devices
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

/* Defines */
/* Concatenate functio name in printk */
#ifdef pr_fmt
    #undef pr_fmt
    #define pr_fmt(fmt) "%s: " fmt,__func__
#endif

#define NO_OF_DEVICES           4

#define MEM_SIZE_MAX_PCDEV1     1024
#define MEM_SIZE_MAX_PCDEV2     512
#define MEM_SIZE_MAX_PCDEV3     1024
#define MEM_SIZE_MAX_PCDEV4     512

#define PCD_DEV_BASE_MINOR 0
#define PCD_DEV_COUNT  1

#define PARENT_NULL         NULL
#define DRIVER_DATA_NULL    NULL

#define RDONLY 0x01
#define WRONLY 0x10
#define RDWR   0x11

/* Variable Declaration */
char device_buffer_pcdev1[MEM_SIZE_MAX_PCDEV1];
char device_buffer_pcdev2[MEM_SIZE_MAX_PCDEV2];
char device_buffer_pcdev3[MEM_SIZE_MAX_PCDEV3];
char device_buffer_pcdev4[MEM_SIZE_MAX_PCDEV4];

/* Device private data structure */
struct pcdev_private_data
{
	char *buffer;		       // pointer to start address
	unsigned size;		       // size of dev mem
	const char *serial_number; // string of device
	int perm;		           // permission (read only, write only, rw)
	struct cdev cdev;	       // cdev structure, device specific
};

/* Driver private data structure */
struct pcdrv_private_data
{
    int total_devices;
    dev_t base_device_number;
    struct class *class_pcd;            /* device class in sysfs */
    struct device *device_pcd;          /* device pointer */
    struct pcdev_private_data pcdev_data[NO_OF_DEVICES];
};

struct pcdrv_private_data pcdrv_data = 
{
    .total_devices = NO_OF_DEVICES,
    .pcdev_data = {
        [0] = {
            .buffer = device_buffer_pcdev1,
            .size = MEM_SIZE_MAX_PCDEV1,
            .serial_number = "PCDEV1",
            .perm = RDONLY
        },

        [1] = {
            .buffer = device_buffer_pcdev2,
            .size = MEM_SIZE_MAX_PCDEV2,
            .serial_number = "PCDEV2",
            .perm = WRONLY
        },

        [2] = {
            .buffer = device_buffer_pcdev3,
            .size = MEM_SIZE_MAX_PCDEV3,
            .serial_number = "PCDEV3",
            .perm = RDWR
        },

        [3] = {
            .buffer = device_buffer_pcdev4,
            .size = MEM_SIZE_MAX_PCDEV4,
            .serial_number = "PCDEV4",
            .perm = RDWR
        },
    }
};

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

/* Char Device*/
/**
 * @brief open Method
 *        Used ot init device
 *        Check open permission (read/write, read only, etc)
 *        Udpate file position
 *        Open method is optional (default open will be used)
 * @param inode pointer of Inode associated with filename
 * @param filp pointer of file object
 * @return 0 for success
 */
int pcd_open(struct inode *inode, struct file *filp)
{
    int ret;
    int minor;
    struct pcdev_private_data *pcdev_data;

    minor = MINOR(inode->i_rdev);
    pr_info("minor access = %d\n", minor);

    /* get device's private data structure, starting from cdev member of inode*/
    pcdev_data = container_of(inode->i_cdev, struct pcdev_private_data, cdev);

    /* supply device private data to other methods of the driver*/
    filp->private_data = pcdev_data;

    /* check permission */
    ret = check_permission(pcdev_data->perm, filp->f_mode);

    (!ret)?pr_info("open successfull\n"):pr_info("open unsucessfull\n");

    return ret;
}
 
/**
 * @brief release Method
 *        Releases the file object. Called when the last reference to an open file is closed 
 *        (f_count becomes 0)
 *        Performes reverse operation of open, leave device in default state (e.g. low power mode)
 * @param inode pointer of Inode associated with filename
 * @param filp pointer of file object
 * @return 0 for success
 */
int pcd_release(struct inode *inode, struct file *filp)
{
    pr_info("close sucessful\n");
 	return 0;
}
 
/**
 * @brief: read Method
 *         Read count bytes from a device starting at position f_pos
 *         Update the f_pos by adding the number of bytes successfully read
 * @param filp pointer of file object
 * @param buff pointer of user buffer (__user macro alerts this is a user lvl pointer, cannot be trusted)
 * @param count read count given by user
 * @param f_pos pointer of current file position from which the read has to begin
 * @return number of bytes succesfully read, or error code
 */
ssize_t pcd_read(struct file *filp,	char __user *buff, size_t count, loff_t *f_pos)
{   
    struct pcdev_private_data *pcdev_data = (struct pcdev_private_data *)filp->private_data;
    int max_size = pcdev_data->size;

    pr_info("read requested for %zu bytes\n", count);
    pr_info("current file pos: %lld\n", *f_pos);

    /* adjust count */
    if ((*f_pos + count) > max_size) {
        count = max_size - *f_pos;
    }

    /* copy to user */
    if (copy_to_user(buff, pcdev_data->buffer + (*f_pos), count)) {
        return -EFAULT;
    }

    /* update current file position */
    *f_pos += count;

    /* number of bytes succesfully read */
    pr_info("bytes read: %zu\n", count);
    pr_info("updated file position: %lld\n", *f_pos);

    return count;
}
 
/**
 * @brief write Method
 *        Write count bytes to a device starting at position f_pos
 *        Update the f_pos by adding the number of bytes successfully written
 * @param filp pointer of file object
 * @param buff pointer of user buffer (__user macro alerts this is a user lvl pointer, cannot be trusted)
 * @param count write count given by user
 * @param f_pos pointer of current file position from which the write has to begin
 * @return number of bytes succesfully written, or error code
 */
ssize_t pcd_write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
    struct pcdev_private_data *pcdev_data = (struct pcdev_private_data *)filp->private_data;
    int max_size = pcdev_data->size;

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
 
/**
 * @brief llseek method
 *        Update the file position pointer using offset and whence information
 * @param whence must use one of the following:
 *          SEEK_SET:= file offset is set to 'off' bytes
 *          SEEK_CUR:= file offset is set to current pos + 'off' bytes
 *          SEEK_END:= file offset is set to the size of the file + off bytes
 * @return newly update file position, or error
 */
loff_t pcd_lseek(struct file *filp, loff_t offset, int whence)
{
    struct pcdev_private_data *pcdev_data = (struct pcdev_private_data *)filp->private_data;
    int max_size = pcdev_data->size;

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

/* Module entry / exit */
/**
 * @brief Perform init
 * 
 * @return int 0 for success
 */
static int __init pseudo_char_driver_init(void)
{

    int ret;
    int i;

    /* Dynamically allocate a device numbers */
    ret = alloc_chrdev_region(&pcdrv_data.base_device_number, 
                        PCD_DEV_BASE_MINOR, 
                        NO_OF_DEVICES, 
                        "pseudo_char_devices"
                        );
    if (ret < 0) {
        pr_err("alloc chrdev failed\n");
        goto out;
    }

    /* Create device class under sysfs */
    pcdrv_data.class_pcd = class_create(THIS_MODULE, "pcd_class");
    if (IS_ERR(pcdrv_data.class_pcd)) {
        pr_err("class creation failed\n");
        ret = PTR_ERR(pcdrv_data.class_pcd);
        goto unreg_chrdev;
    }

    for (i = 0; i < NO_OF_DEVICES; i++) {
        pr_info("Device number <major>:<minor> = %d:%d\n", MAJOR(pcdrv_data.base_device_number + i), MINOR(pcdrv_data.base_device_number + i));

        /* Init cdev structure with fops*/
        cdev_init(&pcdrv_data.pcdev_data[i].cdev, &pcd_fops);

        /* Register a device (cdev structure) with VFS */
        pcdrv_data.pcdev_data[i].cdev.owner = THIS_MODULE; // declare owner of this device
        ret = cdev_add(&pcdrv_data.pcdev_data[i].cdev, pcdrv_data.base_device_number + i, 1);
        if (ret < 0) {
            pr_err("cdev add failed\n");
            goto cdev_del;
        }

        /* Populate sysfs with device information */
        pcdrv_data.device_pcd = device_create(pcdrv_data.class_pcd, PARENT_NULL, pcdrv_data.base_device_number + i, DRIVER_DATA_NULL, "pcdev-%d", i + 1);
        if (IS_ERR(pcdrv_data.device_pcd)) {
            pr_err("device creation failed\n");
            ret = PTR_ERR(pcdrv_data.device_pcd);
            goto class_del;
        }
    }

    pr_info("pseud_char module init successfull\n");

    return 0;

/* error handling */
cdev_del:
class_del:
    for (; i >= 0; i--) {
        device_destroy(pcdrv_data.class_pcd, pcdrv_data.base_device_number + i);
        cdev_del(&pcdrv_data.pcdev_data[i].cdev);
    }
    class_destroy(pcdrv_data.class_pcd);

unreg_chrdev:
    unregister_chrdev_region(pcdrv_data.base_device_number, NO_OF_DEVICES);

out:
    pr_info("module insertion failed\n");
    return ret;
}

/**
 * @brief perform reverse order of init function
 * 
 */
static void __exit pseudo_char_driver_cleanup(void)
{
    int i;

    for (i = 0; i < NO_OF_DEVICES; i++) {
        device_destroy(pcdrv_data.class_pcd, pcdrv_data.base_device_number + i);
        cdev_del(&pcdrv_data.pcdev_data[i].cdev);
    }

    class_destroy(pcdrv_data.class_pcd);

    unregister_chrdev_region(pcdrv_data.base_device_number, NO_OF_DEVICES);

    pr_info("module cleanup sucessful!\n");
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
