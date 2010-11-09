obj-m := jaldi.o
jaldi-objs :=  \
		main.o \
		ahb.o \
		pci.o \
		phy.o \
		hw.o \
		init.o \
		eeprom.o \
		eeprom_def.o \
		debug.o

PWD :=	$(shell pwd)

default:
	$(MAKE) -C $(KLIB_BUILD) M=$(PWD) modules
