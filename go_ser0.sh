#!/bin/bash
stty -F /dev/ttyBmpTarg 115200 -parenb -parodd cs8 -cstopb -crtscts -ixon -ixoff -echo raw
#stdbuf -i0 -o0 -e0 hexdump -v -C /dev/ttyACM1
screen /dev/ttyBmpTarg 115200,cs8,-parenb,-parodd,cstopb
