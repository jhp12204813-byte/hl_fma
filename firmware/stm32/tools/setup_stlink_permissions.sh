#!/bin/sh
set -eu

RULE_SOURCE=/home/idp2/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Projects/STM32F401RE-Nucleo/EncoderTest/tools/49-stlinkv2-1-dialout.rules
TARGET_USER=idp2

install -m 0644 "$RULE_SOURCE" /etc/udev/rules.d/49-stlinkv2-1-dialout.rules
usermod -a -G dialout "$TARGET_USER"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=0483

if command -v setfacl >/dev/null 2>&1 && [ -e /dev/ttyACM0 ]; then
  setfacl -m "u:$TARGET_USER:rw" /dev/ttyACM0
fi
