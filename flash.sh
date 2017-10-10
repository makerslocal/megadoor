#!/bin/bash
set -ex
echo "[BEEFB0BOOT]" > $1
avrdude -v -e -c arduino -P $1 -p atmega328p -b $2 -U flash:w:bin/Debug/${3}.hex:i
