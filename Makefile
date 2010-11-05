obj-m := jaldi.o
jaldi-objs := main.o ahb.o pci.o hw.o phy.o eeprom.o eeprom_def.o init.o debug.o

PWD :=	$(shell pwd)

default:
	$(MAKE) -C $(KLIB_BUILD) M=$(PWD) modules
