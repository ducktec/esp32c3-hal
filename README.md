# ESP32-C3 HAL

This is an **experimental** HAL crate for the [ESP32-C3 SoC](https://www.espressif.com/en/products/socs/esp32-c3).

> :warning: Please note the emphasis on **experimental**! Very little works (yet) and what works is not comprehsively tested and will almost certainly contain bugs 

## Status

The following table contains a list of ESP32-C3 relevant functional components and their support status within this hal crate:

| Functional Component             | Fully Documented in TRM v0.3 | Prototype/Partial Support  | Full Support       |
| -------------------------------- | ---------------------------- | -------------------------- | ------------------ |
| TIMG (Timer + WDT)               | :x:                          | :heavy_check_mark:         | :x:                |
| UART                             | :x:                          | :x:                        | :x:                |
| SPI                              | :x:                          | :x:                        | :x:                |
| I2C                              | :x:                          | :x:                        | :x:                |
| GPIO                             | :x:                          | :x:                        | :x:                |
| System Timer                     | :x:                          | :x:                        | :x:                |
| (Low-Power) Management           | :x:                          | :x:                        | :x:                |
| LED PWM                          | :x:                          | :x:                        | :x:                |
| USB Serial/JTAG Controller       | :x:                          | :x:                        | :x:                |
| SHA Accelerator                  | :x:                          | :x:                        | :x:                |
| AES Accelerator                  | :x:                          | :x:                        | :x:                |
| RSA Accelerator                  | :x:                          | :x:                        | :x:                |
| HMAC Accelerator                 | :x:                          | :x:                        | :x:                |
| Digital Signature                | :x:                          | :x:                        | :x:                |
| Random Number Generator          | :x:                          | :x:                        | :x:                |
| Remote Control Peripheral        | :x:                          | :x:                        | :x:                |

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

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

## Resources
- [Technical Reference Manual v0.3](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf)