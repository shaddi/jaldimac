obj-m := jaldi.o
jaldi-objs := main.o ahb.o pci.o

PWD :=	$(shell pwd)

default:
	$(MAKE) -C $(KLIB_BUILD) M=$(PWD) modules
