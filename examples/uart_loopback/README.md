# UART Loopback Demo

This example reads an arbitrary number of bytes from the serial0
(UART0) peripheral and writes them back to the same peripheral,
thus creating a loopback.

Without any additional GPIO configuration (as with this example)
the serial0 peripheral is routed to the GPIO20/21 (U0TXD/U0RXD)
pins after startup.
