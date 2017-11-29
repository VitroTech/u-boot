#!/bin/bash

DEV="$1"

sudo dd if=/dev/zero of=$DEV bs=1k seek=512 conv=fsync count=8
sudo dd if=SPL of=$DEV bs=1k seek=1 && sync
sudo dd if=u-boot.img of=$DEV bs=1k seek=69 && sync
