#define RDONLY 0x01
#define WRONLY 0x10
#define RDWR   0x11

struct pcdev_platform_data
{
    int size;
    int perm;
    const char *serial_number;
};