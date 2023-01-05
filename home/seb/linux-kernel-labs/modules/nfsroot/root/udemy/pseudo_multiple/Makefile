ifneq ($(KERNELRELEASE),)
obj-m := pseudo_multiple.o
else
KDIR := $(HOME)/linux-kernel-labs/src/linux
all:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
