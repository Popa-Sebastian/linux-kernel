ifneq ($(KERNELRELEASE),)
ARCH=arm
CROSS_COMPILE=arm-linux-gnueabi-
obj-m := pcd_device_setup.o pcd_platform_driver.o
else
KDIR := $(HOME)/linux-kernel-labs/src/linux
all:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$$PWD clean
endif
