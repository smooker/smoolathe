#set debug remote 1
set target-async on
set mem inaccessible-by-default off
target extended-remote /dev/ttyBmpGdb
monitor connect_srst enable
monitor hard_srst disable
#monitor tpwr enable
#monitor swdp_scan
monitor jtag_scan
attach 1
stop

#file ./build/modbus.elf 
load build/STM32F407_FREQ_METER.hex 
compare-sections
hbreak main
#next
#watch huart
#watch data[0]
continue
