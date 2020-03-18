smooker@sw1 ~/src/stm32/other/CubeMX/STM32F407_FREQ_METER $ ~/src/dfu-util/src/dfu-util -U backup.bin --list
dfu-util 0.9

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2020 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

Found DFU: [0483:df11] ver=2200, devnum=52, cfg=1, intf=0, path="3-6", alt=3, name="@Device Feature/0xFFFF0000/01*004 e", serial="208F3681554D"
Found DFU: [0483:df11] ver=2200, devnum=52, cfg=1, intf=0, path="3-6", alt=2, name="@OTP Memory /0x1FFF7800/01*512 e,01*016 e", serial="208F3681554D"
Found DFU: [0483:df11] ver=2200, devnum=52, cfg=1, intf=0, path="3-6", alt=1, name="@Option Bytes  /0x1FFFC000/01*016 e", serial="208F3681554D"
Found DFU: [0483:df11] ver=2200, devnum=52, cfg=1, intf=0, path="3-6", alt=0, name="@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg", serial="208F3681554D"




smooker@sw1 ~/src/stm32/other/CubeMX/STM32F407_FREQ_METER $ ~/src/dfu-util/src/dfu-util -U backup_flash.bin -s 0x08000000 --alt 0 
dfu-util 0.9

Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
Copyright 2010-2020 Tormod Volden and Stefan Schmidt
This program is Free Software and has ABSOLUTELY NO WARRANTY
Please report bugs to http://sourceforge.net/p/dfu-util/tickets/

Opening DFU capable USB device...
ID 0483:df11
Run-time device DFU version 011a
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 011a
Device returned transfer size 2048
DfuSe interface name: "Internal Flash  "
Limiting upload to end of memory segment, 65536 bytes
Upload	[=========================] 100%        65536 bytes
Upload done.


[Mon Feb 24 17:35:40 2020] usb 3-6: new full-speed USB device number 54 using xhci_hcd
[Mon Feb 24 17:35:41 2020] usb 3-6: New USB device found, idVendor=0483, idProduct=df11, bcdDevice=22.00
[Mon Feb 24 17:35:41 2020] usb 3-6: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[Mon Feb 24 17:35:41 2020] usb 3-6: Product: STM32  BOOTLOADER
[Mon Feb 24 17:35:41 2020] usb 3-6: Manufacturer: STMicroelectronics
[Mon Feb 24 17:35:41 2020] usb 3-6: SerialNumber: 208F3681554D


smooker@sw1 ~/src/stm32/other/CubeMX/STM32F407_FREQ_METER $ ~/src/dfu-util/src/dfu-util -U backup_flash.bin -s 0x08000000:leave --alt 0 -v

leave izliza ot dfu mode

#define BOARD_IDENT       "Black Magic Probe (STLINK), (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_DFU   "Black Magic (Upgrade) for STLink/Discovery, (Firmware " FIRMWARE_VERSION ")"
#define BOARD_IDENT_UPD   "Black Magic (DFU Upgrade) for STLink/Discovery, (Firmware " FIRMWARE_VERSION ")"
#define DFU_IDENT         "Black Magic Firmware Upgrade (STLINK)"
#define UPD_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Kg"

/* Hardware definitions... */
#define TDI_PORT  GPIOA
#define TMS_PORT  GPIOB
#define TCK_PORT  GPIOA
#define TDO_PORT  GPIOA
#define TDI_PIN   GPIO7
#define TMS_PIN   GPIO14
#define TCK_PIN   GPIO5
#define TDO_PIN   GPIO6

#define SWDIO_PORT  TMS_PORT
#define SWCLK_PORT  TCK_PORT
#define SWDIO_PIN TMS_PIN
#define SWCLK_PIN TCK_PIN

#define SRST_PORT GPIOB
#define SRST_PIN_V1 GPIO1
#define SRST_PIN_V2 GPIO0

#define LED_PORT  GPIOA
/* Use PC14 for a "dummy" uart led. So we can observere at least with scope*/
#define LED_PORT_UART GPIOC
#define LED_UART  GPIO13


ednokratno raboteshto:

Loading section .sec1, size 0x188 lma 0x8000000
Loading section .sec2, size 0x74b0 lma 0x8000190
Start address 0x8005bb0, load size 30264
Transfer rate: 34 KB/sec, 945 bytes/write.
Section .isr_vector, range 0x8000000 -- 0x8000188: matched.
Section .text, range 0x8000190 -- 0x8006c38: matched.
Section .rodata, range 0x8006c38 -- 0x800744c: matched.
Section .ARM, range 0x800744c -- 0x8007454: matched.
Section .init_array, range 0x8007454 -- 0x8007458: matched.
Section .fini_array, range 0x8007458 -- 0x800745c: matched.
Section .data, range 0x800745c -- 0x8007640: matched.





