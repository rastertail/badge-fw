#!/usr/bin/env bash

PI=10.42.0.2

cargo build --release

scp openocd.cfg root@$PI:/root/openocd.cfg
scp target/thumbv6m-none-eabi/release/badge-fw root@$PI:/root/fw.bin

ssh root@$PI openocd
