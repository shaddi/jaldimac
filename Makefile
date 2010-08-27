obj-m := jaldi.o
jaldi-objs := main.o

#KLIB:=          /home/shaddi/Code/packetmac/backfire/build_dir/linux-ar71xx/linux-2.6.32.10
#KLIB_BUILD :=	$(KLIB)/build
PWD :=	$(shell pwd)

default:
	$(MAKE) -C $(KLIB_BUILD) M=$(PWD) modules
