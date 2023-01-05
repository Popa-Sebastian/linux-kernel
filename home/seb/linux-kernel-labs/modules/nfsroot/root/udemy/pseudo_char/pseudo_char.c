/* 
 * pseudo_char linux driver
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

#define PCD_DEV_MEM_SIZE 512

#define PCD_DEV_BASE_MINOR 0
#define PCD_DEV_COUNT  1

#define PARENT_NULL         NULL
#define DRIVER_DATA_NULL    NULL

/* Variable Declaration */
static char device_buffer[PCD_DEV_MEM_SIZE];

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
    pr_info("open succesful\n");
    return 0;
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
    pr_info("read requested for %zu bytes\n", count);
    pr_info("current file pos: %lld\n", *f_pos);

    /* adjust count */
    if ((*f_pos + count) > PCD_DEV_MEM_SIZE) {
        count = PCD_DEV_MEM_SIZE - *f_pos;
    }

    /* copy to user */
    if (copy_to_user(buff, (void *)(&device_buffer[*f_pos]), count)) {
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
    pr_info("write requested for %zu bytes\n", count);
    pr_info("current file pos: %lld\n", *f_pos);

    /* adjust count */
    if ((*f_pos + count) > PCD_DEV_MEM_SIZE) {
        count = PCD_DEV_MEM_SIZE - *f_pos;
        pr_info("write exceeds memory limit!\n");
    }

    if (!count) {
        return -ENOMEM;
    }

    /* copy from user */
    if (copy_from_user((void *)(&device_buffer[*f_pos]), buff, count)) {
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
    loff_t temp = 0;

    pr_info("llseek requested\n");
    pr_info("current file pos: %lld\n", filp->f_pos);
    
    switch(whence) {
    case SEEK_SET:
        if ((offset > PCD_DEV_MEM_SIZE) || (offset < 0)) {
            return -EINVAL;
        }
        
        filp->f_pos = offset;
        break;
    case SEEK_CUR:
        temp = filp->f_pos + offset;
        if ((temp > PCD_DEV_MEM_SIZE) || (temp < 0)) {
            return -EINVAL;
        }

        filp->f_pos += offset;
        break;
    case SEEK_END:
        temp = PCD_DEV_MEM_SIZE + offset;
        if ((temp > PCD_DEV_MEM_SIZE) || (temp < 0)) {
            return -EINVAL;
        }

        filp->f_pos = PCD_DEV_MEM_SIZE + offset;
        break;
    default:
        return -EINVAL;
        break;
    } /* switch*/

    pr_info("updated file position: %lld\n", filp->f_pos);

    return filp->f_pos;
}

/* Device variables */
static dev_t device_number;          /* Device Number */
static struct cdev pcd_cdev;         /* Cdev */

/* Initialize file operations*/
static struct file_operations pcd_fops = 
{
    .owner = THIS_MODULE,
    .open = &pcd_open,
    .release = &pcd_release,
    .write = &pcd_write,
    .read = &pcd_read,
    .llseek = &pcd_lseek,
};

struct class *class_pcd;            /* device class in sysfs */
struct device *device_pcd;          /* device pointer */

/* Module entry / exit */
/**
 * @brief Perform init
 * 
 * @return int 0 for success
 */
static int __init pseudo_char_driver_init(void)
{
    int ret;
    /*1. Dynamically allocate a device number*/
    ret = alloc_chrdev_region(&device_number, 
                        PCD_DEV_BASE_MINOR, 
                        PCD_DEV_COUNT, 
                        "pseudo_char_devices"
                        );
    if (ret < 0) {
        pr_err("alloc chrdev failed\n");
        goto out;
    }

    pr_info("Device number <major>:<minor> = %d:%d\n", MAJOR(device_number), MINOR(device_number));

    /*2. Init cdev structure with fops*/
    cdev_init(&pcd_cdev, &pcd_fops);

    /*3. Register a device (cdev structure) with VFS */
    pcd_cdev.owner = THIS_MODULE; // declare owner of this device
    ret = cdev_add(&pcd_cdev, device_number, 1);
    if (ret < 0) {
        pr_err("cdev add failed\n");
        goto unreg_chrdev;
    }

    /*4. Create device class under sysfs */
    class_pcd = class_create(THIS_MODULE, "pcd_class");
    if (IS_ERR(class_pcd)) {
        pr_err("class creation failed\n");
        ret = PTR_ERR(class_pcd);
        goto cdev_del;
    }

    /*5. Populate sysfs with device information */
    device_pcd = device_create(class_pcd, PARENT_NULL, device_number, DRIVER_DATA_NULL, "pcd");
    if (IS_ERR(device_pcd)) {
        pr_err("device creation failed\n");
        ret = PTR_ERR(device_pcd);
        goto class_del;
    }

    pr_info("pseud_char module init successfull\n");

    return 0;

/* error handling */
class_del:
    class_destroy(class_pcd);
cdev_del:
    cdev_del(&pcd_cdev);
unreg_chrdev:
    unregister_chrdev_region(device_number, PCD_DEV_COUNT);
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
    device_destroy(class_pcd, device_number);
    class_destroy(class_pcd);
    cdev_del(&pcd_cdev);
    unregister_chrdev_region(device_number, PCD_DEV_COUNT);

    pr_info("module unloaded\n");
}

/* Init / Exit functions registration */
module_init(pseudo_char_driver_init);
module_exit(pseudo_char_driver_cleanup);

/* Module metadata */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Popa: sebastian.popa@raptor-technologies.ro");
MODULE_DESCRIPTION("Simple Pseudo Char Module");
MODULE_INFO(board, "Beaglebone black Rev C1");

/* End of file */