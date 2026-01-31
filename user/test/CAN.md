# How to set up the CAN hat for RPi

## How to hook up?

SERVO CANL to CANHAT CANL
SERVO CANH to CANHAT CANH
SERVO GROUND to ground rail
SERVO POWER to power rail

DC-DC Supply Power to power rail
DC-DC Supply Ground to ground rail

PI ground to ground rail 

ground rail to CANHAT gnd

## Enable CAN interface

Code to edit the config file:

`sudo nano /boot/firmware/config.txt`

Need to add this to enable the CAN hat:

```dtparam=spi=on
dtoverlay=mcp2515,spi1-1,oscillator=16000000,interrupt=22
dtoverlay=mcp2515,spi1-2,oscillator=16000000,interrupt=13```

## Install can-utils

Not sure if it's needed, but I used it to double-check motor ids and motor heartbeat.

`sudo apt-get install can-utils`

(If you want to check heartbeat: `candump can0`)

## Reboot the Pi

## Ensure SPI is working

`dmesg | grep spi1`

Should receive a reply saying something like:

> mcp251x spi1.2 can0: MCP2515 successfully initialized
> mcp251x spi1.1 can1: MCP2515 successfully initialized

## Initialize CAN interface

**If the servo is hooked up to CAN_0 set of wires:**
`sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536`

**If the servo is hooked up to CAN_1 set of wires:**
`sudo ip link set can1 up type can bitrate 1000000
sudo ifconfig can1 txqueuelen 65536`

## Check and make sure CAN is up

`ifconfig`

You should see the can that you enabled.


## How to turn off the CAN

**If you turned on CAN_0 set**

`sudo ifconfig can0 down`

**If you turned on CAN_1 set**

`sudo ifconfig can1 down`


