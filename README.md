# STMICROELECTRONICS - VL53L8CX Linux driver
Official VL53L8CX Linux driver and test applications for linux and android platforms

## Introduction
The proposed implementation is customized to run on a raspberry pi v3, but can be adapted to run on any linux embedded platform,
as far as the VL53L8CX device is connected through I2C
Two options are offered to the user
- 1. compile and run this driver with a kernel module responsible for handling i2c/spi bus and the interruption. This is the kernel mode
- 2. compile and run this driver in a full user mode, where the i2c commnication is handled with the /dev/i2c-1 file descriptor. This is the user mode


Option 1 supports the interruption line of the VL53L8CX but needs a kernel module to be compiled and inserted.
Option 2 may be more suitable for simple application, but needs the /dev/i2c-1 or/dev/spidev 0.0 to be available which may not be the case on some secured platforms


I2C bus guide
 
## How to run a test application on raspberry pi
    Note that the following instructions were tested on raspberrypi 3.

### Install the raspberry pi kernel source headers (kernel mode only)
    refer to raspberrypi official documentation to download the headers matching your kernel version
    $ sudo apt-get install raspberrypi-kernel-headers

### update /boot/config.txt file (kernel mode only)
    $ sudo nano /boot/config.txt
    --> add or uncomment the following lines at the end of the /boot/config.txt
    dtparam=i2c_arm=on
    dtparam=i2c1=on
    dtparam=i2c1_baudrate=1000000
    dtoverlay=stmvl53l8cx
### compile the device tree blob (kernel mode only)
    $ cd vl53l8cx-uld-driver/kernel
    $ make dtb
    $ sudo reboot
### compile the test examples, the platform adaptation layer and the uld driver
    $ nano vl53l8cx-uld-driver/user/test/Makefile
    Enable or disable the STMVL53L8CX_KERNEL cflags option depending on the wished uld driver mode : with a kernel module of fully in user.
    $ cd vl53l8cx-uld-driver/user/test
    $ make
### compile the kernel module (kernel mode only)
    $ cd vl53l8cx-uld-driver/kernel
    $ make clean
    $ make
    $ sudo make insert
### run the test application menu
    $ cd vl53l8cx-uld-driver/user/test
    $ ./menu





SPI bus guide

Single SPI device user space 
### How to run a SPI test application on raspberry pi
### Note that the following instructions were tested on raspberrypi 3B.

### go to test folder
	$ cd vl53l8cx-uld-driver/user/test
### enable the SPI cflags option in test makefile
	$ nano vl53l8cx-uld-driver/user/test/Makefile
	CFLAGS_RELEASE += -DSPI
### build program
	$ make
### run the test application menu
    $ ./menu


Single SPI device kernel space 
### How to run a SPI test application on raspberry pi
### Note that the following instructions were tested on raspberrypi 3B.

### Install the raspberry pi kernel source headers
    refer to raspberrypi official documentation to download the headers matching your kernel version
    $ sudo apt-get install raspberrypi-kernel-headers

### update /boot/config.txt file
    $ sudo nano /boot/config.txt
    --> add or uncomment the following lines at the end of the /boot/config.txt
    dtparam=spi=on
    dtoverlay=stmvl53l8cx
	
### compile the device tree blob
    $ cd vl53l8cx-uld-driver/kernel
    $ make SPI=1 dtb
    $ sudo reboot
	
### compile the test examples, the platform adaptation layer and the uld driver
    $ nano vl53l8cx-uld-driver/user/test/Makefile
    --> Enable the STMVL53L8CX_KERNEL cflags option.
	CFLAGS_RELEASE += -DSTMVL53L8CX_KERNEL
    $ cd vl53l8cx-uld-driver/user/test
    $ make
		
### compile the kernel module
    $ cd vl53l8cx-uld-driver/kernel
    $ make clean
    $ make
    $ sudo make insert
### run the test application menu
    $ cd vl53l8cx-uld-driver/user/test
    $ ./menu
	

	
	
	
Dual SPI devices

Dual SPI devices user space guide
### How to run a SPI test application on raspberry pi
### Note that the following instructions were tested on raspberrypi 3.

### go to test folder
	$ cd vl53l8cx-uld-driver/user/test
### enable the SPI cflags option in test makefile
	$ nano vl53l8cx-uld-driver/user/test/Makefile
	CFLAGS_RELEASE += -DSPI
### build program
	$ make
### run the test application multi
    $ ./multi



Dual SPI devices kernel space guide
### How to run a SPI test application on raspberry pi
### Note that the following instructions were tested on raspberrypi 3.

### Install the raspberry pi kernel source headers
    refer to raspberrypi official documentation to download the headers matching your kernel version
    $ sudo apt-get install raspberrypi-kernel-headers

### update /boot/config.txt file
    $ sudo nano /boot/config.txt
    --> add or uncomment the following lines at the end of the /boot/config.txt
    dtparam=spi=on
    dtoverlay=stmvl53l8cx
	
### update the dts file
	$ cd vl53l8cx-uld-driver/kernel 
	$ sudo nano stmvl53l8cx_spi.dts
	--> add the following line to enable 2nd SPI device
	     stmvl53l8cx1:stmv53l8cx@1 {
                compatible = "st,stmvl53l8cx";
		        reg = <1>;
                spi-cpol;
                spi-cpha;
                spi-max-frequency = <2000000>;
                pwr-gpios = <&gpio 16 0>;                
                irq-gpios = <&gpio 19 2>;
                dev_num = <1>;
                status = "okay";
            };
	
 
### compile the device tree blob
    $ cd vl53l8cx-uld-driver/kernel
    $ make SPI=1 dtb
    $ sudo reboot
	
	
### For dule SPI devices, compile the test examples, the platform adaptation layer and the uld driver
    $ nano vl53l8cx-uld-driver/user/test/Makefile
    --> Enable the STMVL53L8CX_KERNEL cflags option. 
	CFLAGS_RELEASE += -DSTMVL53L8CX_KERNEL
    $ cd vl53l8cx-uld-driver/user/test
    $ make
	rename ./menu to ./menu0
	
	$ nano vl53l8cx-uld-driver/user/test/platform/platform.c
	--> modify the device name for 2nd device.
	p_platform->fd = open("/dev/stmvl53l8cx1", O_RDONLY);

    $ make
	rename ./menu to ./menu1
	
### compile the kernel module
    $ cd vl53l8cx-uld-driver/kernel
    $ make clean
    $ make
    $ sudo make insert
	
### For dule SPI device run the test application menu
    $ cd vl53l8cx-uld-driver/user/test
    $ ./menu0
    $ ./menu1

