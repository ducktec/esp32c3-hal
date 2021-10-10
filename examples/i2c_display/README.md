# I2C Display

This example displays text on an monochrome OLED with a SSD1306 driver IC
which is controlled via the I2C-Bus.

The following wiring is assumed:
* `SDA` => `GPIO2`
* `SCL` => `GPIO3`

The example will toggle between displaying two different text messages
"esp32c3-hal - v0.1.0" and "Hello World" every five seconds.

For driving the OLED with the SSD1306, the examples uses the crate [`ssd1306`](https://crates.io/crates/ssd1306).