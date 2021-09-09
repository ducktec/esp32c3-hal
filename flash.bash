#!/usr/bin/bash

cargo build --example $1
riscv32-elf-objcopy -O binary ./target/riscv32imc-unknown-none-elf/debug/examples/$1 $1.bin
esptool.py --port /dev/ttyUSB0 --chip esp32c3 write_flash --flash_mode dio --flash_size detect --flash_freq 80m 0x0 $1.bin
