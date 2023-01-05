#include <linux/module.h>
#include <linux/platform_device.h>
#include "platform.h"

#ifdef pr_fmt
    #undef pr_fmt
    #define pr_fmt(fmt) "%s: " fmt,__func__
#endif

void pcdev_release(struct device *dev)
{
    pr_info("Device released\n");
}

/* Create 2 platform data structures
 * Platform specific device custom data
 * Included from platform.h
 */
struct pcdev_platform_data pcdev_pdata[4] = {
    [0] = {.size = 512, .perm = RDWR, .serial_number = "PCDEVABC1111"},
    [1] = {.size = 1024, .perm = RDWR, .serial_number = "PCDEVXYZ2222"},
    [2] = {.size = 128, .perm = RDONLY, .serial_number = "PCDEVABC3333"},
    [3] = {.size = 32, .perm = WRONLY, .serial_number = "PCDEVXYZ4444"}
};

/* create 2 platform devices 
 * struct from linux/platform_device.h
 */
struct platform_device platform_pcdev_1 = {
    .name = "pcdev-A1x",
    .id = 0,
    .dev = {
        .platform_data = &pcdev_pdata[0], /* initialize platform specific data*/
        .release = pcdev_release,
    }
};

struct platform_device platform_pcdev_2 = {
    .name = "pcdev-B1x",
    .id = 1,
        .dev = {
            .platform_data = &pcdev_pdata[1],
            .release = pcdev_release,
    }
};

struct platform_device platform_pcdev_3 = {
    .name = "pcdev-C1x",
    .id = 2,
    .dev = {
        .platform_data = &pcdev_pdata[2], /* initialize platform specific data*/
        .release = pcdev_release,
    }
};

struct platform_device platform_pcdev_4 = {
    .name = "pcdev-D1x",
    .id = 3,
        .dev = {
            .platform_data = &pcdev_pdata[3],
            .release = pcdev_release,
    }
};

struct platform_device *platform_pcdevs[] =
{
    &platform_pcdev_1,
    &platform_pcdev_2,
    &platform_pcdev_3,
    &platform_pcdev_4,
};

/* __init __ exit functions*/
static int __init pcdev_platform_init(void)
{
    // register platform device
    // platform_device_register(&platform_pcdev_1);
    // platform_device_register(&platform_pcdev_2);

    platform_add_devices(platform_pcdevs, ARRAY_SIZE(platform_pcdevs));
    pr_info("Device setup module loaded\n");

    return 0;
}

static void __exit pcdev_platform_exit(void)
{
    platform_device_unregister(&platform_pcdev_1);
    platform_device_unregister(&platform_pcdev_2);
    platform_device_unregister(&platform_pcdev_3);
    platform_device_unregister(&platform_pcdev_4);

    pr_info("Device setup module unloaded\n");
}

/* register init / exit functions */
module_init(pcdev_platform_init);
module_exit(pcdev_platform_exit);


/* Module metadata */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sebastian Popa: sebastian.popa@raptor-technologies.ro");
MODULE_DESCRIPTION("Module registers platform devices");
MODULE_INFO(board, "Beaglebone black Rev C1");