ifneq ($(KERNELRELEASE),)
include Kbuild

else
KVERSION ?= `uname -r`
KDIR ?= /lib/modules/${KVERSION}/build

DTS_I2C = stmvl53l8cx.dts
DTS_SPI = stmvl53l8cx_spi.dts


# Check for the macro parameter to select the device tree source file
ifdef SPI
    DTS_SELECTED = $(DTS_SPI)
else
    DTS_SELECTED = $(DTS_I2C)
endif


default:
	CONFIG_DRIVER_ST_VL53L8CX=m $(MAKE) -C $(KDIR) M=$$PWD

clean:
	CONFIG_DRIVER_ST_VL53L8CX=m $(MAKE) -C $(KDIR) M=$$PWD clean

dtb:
	sudo rm -f /boot/overlays/stmvl53l8cx.dtbo
	dtc -I dts -O dtb -o stmvl53l8cx.dtbo $(DTS_SELECTED)
	sudo cp stmvl53l8cx.dtbo /boot/overlays/stmvl53l8cx.dtbo

insert :
	sudo insmod stmvl53l8cx.ko

remove :
	sudo rmmod stmvl53l8cx
	
endif
