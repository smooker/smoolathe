#!/bin/bash
#dfu-util -a 0 -U flash2.bin -s 0x08000000:1048576
#./dfu-util -a 0 -D ~/src/dfu-util/src/sp3evo_cflight_1.13.0.bin -s 0x08000000
~/src/dfu-util/src/dfu-util -a 0 -D build/STM32F407_FREQ_METER.bin -s 0x08000000:leave -R
