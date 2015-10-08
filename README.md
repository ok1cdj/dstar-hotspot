# DSTAR-Hotspot with Arduino RPi.
D-STAR hotspot with open HW and SW under GNU GPL2 based on
DV-MEGA - by Guus van Dooren (PE1PLM) and [DVSP2](http://tech4.pl/SQ9MDD/?p=92) by SQ9MDD.

Tested with Raspberry PI 2 and [Western D-STAR image](http://www.westerndstar.co.uk/html/downloads.html).

## TODO

** add partlist
** add more info about HW and building
** add gerber files for production
** add configuration for Ircddbgateway and D-STAR repeater

![hotspot](https://raw.githubusercontent.com/ok1cdj/dstar-hotspot/master/pics/hotspot.jpg)

## How to compile SW for Arduino

Use Arduino 1.5.X

Before programming Arduino be sure the "HardwareSerial.cpp" under Arduino instalation folder\hardware\arduino\cores\arduino is changed.

From:
```
#if (RAMEND < 1000)
#define SERIAL_BUFFER_SIZE 16
#else
#define SERIAL_BUFFER_SIZE 64
#endif
```

To:
```
#if (RAMEND < 1000)
#define SERIAL_BUFFER_SIZE 16
#else
#define SERIAL_BUFFER_SIZE 128
#endif

```
## HOW to remove console from serial on RPi

$ sudo nano /boot/cmdline.txt

From:
```
　GNU nano 2.2.6　　　　　　　　　　File: /boot/cmdline.txt
dwc_otg.lpm_enable=0 console=ttyAMA0,115200 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait 
```
To:
```
dwc_otg.lpm_enable=0 rpitestmode=1 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait
```

$ sudo nano /etc/inittab

From:
```
　GNU nano 2.2.6　　　　　　　　　　File: /etc/inittab
#Spawn a getty on Raspberry Pi serial line
T0:23:respawn:/sbin/getty -L ttyAMA0 115200 vt100
```
To:
```
#T0:23:respawn:/sbin/getty -L ttyAMA0 115200 vt100
```

## You need configure Ircddbgateway and D-STAR repeater package  to run hotspot
