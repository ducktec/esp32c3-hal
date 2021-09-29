# ESP32-C3 HAL

[![Build Status](https://github.com/ducktec/esp32c3-hal/actions/workflows/rust.yml/badge.svg)](https://github.com/ducktec/esp32c3-hal/actions/workflows/rust.yml)

This is an **experimental** Rust HAL crate for the [ESP32-C3 SoC](https://www.espressif.com/en/products/socs/esp32-c3).

> :warning: Please note the emphasis on **experimental**! Very little works (yet) and what works is not comprehsively tested and will almost certainly contain bugs. Please don't use for real projects at this point in time. The API can and will change.

## Status

The following table contains a list of ESP32-C3 functional components that are planned to be supported at this point and their support status within this hal crate:

| Functional Component             | Prototype/Partial Support  |
| -------------------------------- | -------------------------- |
| TIMG (Timer + WDT)               | :heavy_check_mark:         |
| UART                             | :heavy_check_mark:         |
| SPI                              | :x:                        |
| I2C                              | :heavy_check_mark:         |
| GPIO                             | :heavy_check_mark:         |
| System Timer                     | :x:                        |
| (Low-Power) Management           | :x:                        |
| LED PWM                          | :x:                        |
| USB Serial/JTAG Controller       | :x:                        |
| Remote Control Peripheral        | :x:                        |

## Necessary Tools

A number of utilities are required to build and flash applications based on this HAL onto the ESP32-C3:
- riscv32-elf-objcopy (e.g. part of `riscv32-elf-binutils` in Arch Linux)
- [esptool.py](https://github.com/espressif/esptool) 


## Getting Started

The ESP32-C3 is based on the RISCV architecture. To build code for this architecture, download the rust toolchain for this architecture:

```bash
rustup target add riscv32imc-unknown-none-elf
```

Next, build one of the examples:

```bash
cargo build --example empty
```

The file was compiled into an elf file, more specifically a `ELF 32-bit LSB executable, UCB RISC-V, RVC, soft-float ABI, version 1 (SYSV), statically linked`. Next, it needs to be converted into a binary file that can be flashed:

```bash
riscv32-elf-objcopy -O binary ./target/riscv32imc-unknown-none-elf/debug/examples/empty empty.bin
```

Finally, that file can be flashed onto the ESP32-C3 (the port may vary):

```bash
esptool.py --port /dev/ttyUSB0 --chip esp32c3 write_flash --flash_mode dio --flash_size detect --flash_freq 80m 0x0 empty.bin
```

## Examples

The HAL comes with a number of examples:
- `empty.rs`: Start and enter an endless loop. This is to demonstrate that all relevant watchdogs can be disabled and the SoC does not continuously reset.
- `hello_world.rs`: Write "Hello World" to UART0 (with the default pins) every second.
- `uart_loopback.rs`: Read from UART0 and write back to UART0.
- `blinky.rs`: Toggle a LED that is connected to GPIO2 every second.

## MSRV

This project follows the rust embedded devices working group [MSRV policy](https://github.com/rust-embedded/wg/blob/master/ops/msrv.md).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

This project makes use of original and modified code from the [esp32-hal](https://github.com/esp-rs/esp32-hal) project, which is published under the same two licensing options and is used under the MIT license in this context. This code use is reflected in the included license file.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

## Resources
- [Technical Reference Manual v0.3](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf)
