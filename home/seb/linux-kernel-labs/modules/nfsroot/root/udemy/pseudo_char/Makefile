ifneq ($(KERNELRELEASE),)
obj-m := pseudo_char.o
else
KDIR := $(HOME)/linux-kernel-labs/src/linux
all:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
