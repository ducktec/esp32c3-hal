> # :warning: Abandoned!
>
>**This implementation was abandoned in favour of the implementation in [esp-rs/esp-hal](https://github.com/esp-rs/esp-hal). This (already more mature) implementation supports multiple ESP32 variants, the `ESP32-C3` variant being one of them. It is strongly recommended to use `esp-hal` instead!**

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

## Getting Started

The ESP32-C3 is based on the RISCV architecture. To build code for this architecture, download the rust toolchain for this architecture:

```bash
rustup target add riscv32imc-unknown-none-elf
```

Next, install the [`espflash`](https://github.com/esp-rs/espflash) tool that will take care of building and flashing applications and examples:
```bash
cargo install --git https://github.com/esp-rs/espflash cargo-espflash
```
The installation from the git repository is required as the `--package` support of the tool is merged but not yet released.

Once `--package` support is part of the release, it will be enough to run
```bash
cargo install cargo-espflash
```

Use the following command to build and flash an example application to a connected ESP32-C3 board:
```bash
cargo espflash --package hello-world-demo /dev/tty.usbserial-10
```
Update the port (last argument) depending on your host system.

## Examples

The HAL [cargo workspace](https://doc.rust-lang.org/book/ch14-03-cargo-workspaces.html) currently comes with the following example packages (to be built/flashed using the `--package` option)
- `blinky-demo`: Toggle a LED that is connected to GPIO2 every second.
- `empty-demo`: Start and enter an endless loop. This is to demonstrate that all relevant watchdogs can be disabled and the SoC does not continuously reset.
- `hello-world-demo`: Write "Hello World" to UART0 (with the default pins) every second.
- `i2c-display-demo`: Display text on an monochrome OLED with a SSD1306 driver IC (via I2C).
- `uart-loopback-demo`: Read from UART0 and write back to UART0.

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
- [Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf)
