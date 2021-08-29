//! I2C Functionality (I2C)
//!
//! Reference: ESP32-C3 TRM v0.3 Section XX (as TRM)
//!
//! The ESP32-C3 has a single I2C Peripheral (I2C0).
//!
//! The preliminary ESP32-C3 TRM v0.3 currently does not contain a
//! description of the I2C component. Based on the registers, it appears
//! that the ESP32-S2 I2C controller is very simular. Therefore, for the
//! time being, this functionality is implemented based on the ESP32-S2
//! TRM description.
//!
//! This file is derived from the esp32-hal implementation.
//!
//! TODO:
//! - Prototypical support
//! - Interrupt support

use crate::gpio::{InputPin, InputSignal, OutputPin, OutputSignal};
use crate::pac::I2C0;

use embedded_hal::blocking::i2c::*;

const SOURCE_CLK_FREQ: u32 = 40_000_000;

/// I2C-specific errors
#[derive(Debug)]
pub enum Error {
    ExceedingFifo,
    AckCheckFailed,
}

/// I2C peripheral (I2C)
#[allow(dead_code)]
pub struct I2C<I2C0> {
    reg: I2C0,
}

impl I2C<I2C0> {
    pub fn new<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin>(
        instance: I2C0,
        mut pins: Pins<SDA, SCL>,
        frequency: u32,
    ) -> Result<Self, Error> {
        let mut i2c = I2C { reg: instance };

        // Setup Pins
        pins.sda
            .set_to_open_drain_output()
            .enable_input(true)
            .internal_pull_up(true)
            .connect_peripheral_to_output(OutputSignal::I2CEXT0_SDA)
            .connect_input_to_peripheral(InputSignal::I2CEXT0_SDA);

        pins.sda.set_output_high(true);

        pins.scl
            .set_to_open_drain_output()
            .enable_input(true)
            .internal_pull_up(true)
            .connect_peripheral_to_output(OutputSignal::I2CEXT0_SCL)
            .connect_input_to_peripheral(InputSignal::I2CEXT0_SCL);

        // Disable all I2C interrupts
        i2c.reg.int_ena.write(|w| unsafe { w.bits(0) });
        // Clear all I2C interrupts
        i2c.reg.int_clr.write(|w| unsafe { w.bits(0x3FFF) });

        i2c.reg.ctr.modify(|_, w| unsafe {
            // Clear register
            w.bits(0)
                // Set I2C controller to master mode
                .ms_mode()
                .set_bit()
                // Use open drain output for SDA and SCL
                .sda_force_out()
                .set_bit()
                .scl_force_out()
                .set_bit()
                // Use Most Significant Bit first for sending and receiving data
                .tx_lsb_first()
                .clear_bit()
                .rx_lsb_first()
                .clear_bit()
                .clk_en()
                .set_bit()
        });

        i2c.reg
            .clk_conf
            .modify(|_, w| unsafe { w.sclk_sel().clear_bit().sclk_div_num().bits(1) });

        // Reset entire peripheral (also resets fifo)
        i2c.reset();

        // Configure filter
        i2c.set_filter(Some(7), Some(7));

        // Configure frequency
        i2c.set_frequency(frequency);

        Ok(i2c)
    }

    /// Resets the transmit and receive FIFO buffers
    fn reset_fifo(&mut self) {
        self.reg.fifo_conf.modify(|_, w| w.tx_fifo_rst().set_bit());
        self.reg
            .fifo_conf
            .modify(|_, w| w.tx_fifo_rst().clear_bit());

        self.reg.fifo_conf.modify(|_, w| w.rx_fifo_rst().set_bit());
        self.reg
            .fifo_conf
            .modify(|_, w| w.rx_fifo_rst().clear_bit());

        // Make sure that the FIFO operates in FIFO-mode
        self.reg
            .fifo_conf
            .modify(|_, w| w.nonfifo_en().clear_bit().fifo_prt_en().clear_bit());
    }

    /// Resets the I2C controller (FIFO + FSM)
    fn reset(&mut self) {
        // Reset fifo
        self.reset_fifo();

        // Reset the FSM
        self.reg.ctr.modify(|_, w| w.fsm_rst().set_bit());
        self.reg.ctr.modify(|_, w| w.fsm_rst().clear_bit());
    }

    /// Sets the filter with a supplied threshold in clock cycles for which a pulse must be present to pass the filter
    fn set_filter(&mut self, sda_threshold: Option<u8>, scl_threshold: Option<u8>) {
        match sda_threshold {
            Some(threshold) => {
                self.reg
                    .filter_cfg
                    .modify(|_, w| unsafe { w.sda_filter_thres().bits(threshold) });
                self.reg
                    .filter_cfg
                    .modify(|_, w| w.sda_filter_en().set_bit());
            }
            None => self
                .reg
                .filter_cfg
                .modify(|_, w| w.sda_filter_en().clear_bit()),
        }

        match scl_threshold {
            Some(threshold) => {
                self.reg
                    .filter_cfg
                    .modify(|_, w| unsafe { w.scl_filter_thres().bits(threshold) });
                self.reg
                    .filter_cfg
                    .modify(|_, w| w.scl_filter_en().set_bit());
            }
            None => self
                .reg
                .filter_cfg
                .modify(|_, w| w.scl_filter_en().clear_bit()),
        }
    }

    /// Sets the frequency of the I2C interface by calculating and applying the associated timings
    fn set_frequency(&mut self, freq: u32) {
        // FIXME: Consolitate implementation
        let half_cycle = ((SOURCE_CLK_FREQ / freq) / 2) as u16;
        let scl_low = half_cycle;
        let scl_high = half_cycle;
        let sda_hold = half_cycle / 2;
        let sda_sample = scl_high / 2;
        let setup = half_cycle;
        let hold = half_cycle;
        // By default we set the timeout value to 10 bus cycles
        let tout = (half_cycle * 20) as u8;

        unsafe {
            // scl period
            self.reg.scl_low_period.write(|w| w.period().bits(scl_low));
            self.reg
                .scl_high_period
                .write(|w| w.period().bits(scl_high));

            // sda sample
            self.reg.sda_hold.write(|w| w.time().bits(sda_hold));
            self.reg.sda_sample.write(|w| w.time().bits(sda_sample));

            // setup
            self.reg.scl_rstart_setup.write(|w| w.time().bits(setup));
            self.reg.scl_stop_setup.write(|w| w.time().bits(setup));

            // hold
            self.reg.scl_start_hold.write(|w| w.time().bits(hold));
            self.reg.scl_stop_hold.write(|w| w.time().bits(hold));

            // timeout
            self.reg.to.write(|w| w.time_out_reg().bits(tout.into()));
        }
    }

    /// Send bytes to a target slave with the address `addr`
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Reset FIFO
        self.reset_fifo();

        // RSTART command
        self.reg
            .comd0
            .write(|w| unsafe { w.command0().bits(Command::Start.into()) });

        // Load address and R/W bit into FIFO
        self.reg
            .data
            .write(|w| unsafe { w.fifo_rdata().bits(addr << 1 | OperationType::WRITE as u8) });

        // Load actual data into fifo
        // TODO: Handle the case where we transfer an amount of data that is exceeding the
        // FIFO size (i.e. > 32 bytes?)
        if bytes.len() > 31 {
            return Err(Error::ExceedingFifo);
        }

        for byte in bytes {
            self.reg
                .data
                .write(|w| unsafe { w.fifo_rdata().bits(*byte) });
        }

        // WRITE command
        self.reg.comd1.write(|w| unsafe {
            w.command1().bits(
                Command::Write {
                    ack_exp: Ack::ACK,
                    ack_check_en: false,
                    length: 1 + bytes.len() as u8,
                }
                .into(),
            )
        });

        // STOP command
        self.reg
            .comd2
            .write(|w| unsafe { w.command2().bits(Command::Stop.into()) });

        // Clear all I2C interrupts
        self.reg.int_clr.write(|w| unsafe { w.bits(0x3FFF) });
        // Start transmission
        self.reg.ctr.modify(|_, w| w.trans_start().set_bit());

        // Busy wait for all three commands to be marked as done
        // TODO: Evaluate the interrupts to check agains timeout and ack failure interrupts
        // while self.reg.comd0.read().command0_done().bit() != true {}
        // while self.reg.comd1.read().command1_done().bit() != true {}
        // while self.reg.comd2.read().command2_done().bit() != true {}

        // while self.reg.int_raw.read().rxfifo_ovf_int_raw().bit_is_clear() {}

        Ok(())
    }

    pub fn get_interrupts(&self) -> u32 {
        self.reg.int_raw.read().bits()
    }

    // // TODO: Enable ACK checks and return error if ACK check fails
    // fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
    //     // Reset FIFO
    //     self.reset_fifo();

    //     // RSTART command
    //     self.0
    //         .comd0
    //         .write(|w| unsafe { w.command0().bits(Command::Start.into()) });

    //     // Load address into FIFO
    //     let fifo_addr = self.fifo_addr(OperationType::READ) as *mut u8;
    //     unsafe { ptr::write_volatile(fifo_addr, addr << 1 | OperationType::READ as u8) };

    //     // WRITE command
    //     self.0.comd1.write(|w| unsafe {
    //         w.command1().bits(
    //             Command::Write {
    //                 ack_exp: Ack::ACK,
    //                 ack_check_en: false,
    //                 length: 1,
    //             }
    //             .into(),
    //         )
    //     });

    //     if buffer.len() > 1 {
    //         // READ command for first n - 1 bytes
    //         self.0.comd2.write(|w| unsafe {
    //             w.command2().bits(
    //                 Command::Read {
    //                     ack_value: Ack::ACK,
    //                     length: buffer.len() as u8 - 1,
    //                 }
    //                 .into(),
    //             )
    //         });

    //         // READ command for final byte
    //         self.0.comd3.write(|w| unsafe {
    //             w.command3().bits(
    //                 Command::Read {
    //                     ack_value: Ack::NACK,
    //                     length: 1,
    //                 }
    //                 .into(),
    //             )
    //         });

    //         // STOP command
    //         self.0
    //             .comd4
    //             .write(|w| unsafe { w.command4().bits(Command::Stop.into()) });
    //     } else {
    //         // READ command for byte
    //         self.0.comd2.write(|w| unsafe {
    //             w.command2().bits(
    //                 Command::Read {
    //                     ack_value: Ack::NACK,
    //                     length: 1,
    //                 }
    //                 .into(),
    //             )
    //         });

    //         // STOP command
    //         self.0
    //             .comd3
    //             .write(|w| unsafe { w.command3().bits(Command::Stop.into()) });
    //     }

    //     // Start transmission
    //     self.0.ctr.modify(|_, w| w.trans_start().set_bit());

    //     // Busy wait for all commands to be marked as done
    //     while self.0.comd0.read().command0_done().bit() != true {}
    //     while self.0.comd1.read().command1_done().bit() != true {}

    //     if buffer.len() > 1 {
    //         while self.0.comd2.read().command2_done().bit() != true {}
    //         while self.0.comd3.read().command3_done().bit() != true {}
    //         while self.0.comd4.read().command4_done().bit() != true {}
    //     } else {
    //         while self.0.comd2.read().command2_done().bit() != true {}
    //         while self.0.comd3.read().command3_done().bit() != true {}
    //     }

    //     // Read bytes from FIFO
    //     for byte in buffer.iter_mut() {
    //         *byte = unsafe { ptr::read_volatile(fifo_addr) };
    //     }

    //     Ok(())
    // }

    // // TODO: Enable ACK checks and return error if ACK check fails
    // fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
    //     // Reset FIFO
    //     self.reset_fifo();

    //     // START
    //     self.0
    //         .comd0
    //         .write(|w| unsafe { w.command0().bits(Command::Start.into()) });

    //     // load into FIFO
    //     let fifo_addr = self.fifo_addr(OperationType::WRITE) as *mut u8;
    //     unsafe {
    //         // load address
    //         ptr::write_volatile(fifo_addr, addr << 1 | OperationType::WRITE as u8);

    //         // load data
    //         for byte in bytes {
    //             ptr::write_volatile(fifo_addr, *byte);
    //         }
    //     }

    //     // WRITE addr + data
    //     self.0.comd1.write(|w| unsafe {
    //         w.command1().bits(
    //             Command::Write {
    //                 ack_exp: Ack::ACK,
    //                 ack_check_en: false,
    //                 length: 1 + bytes.len() as u8,
    //             }
    //             .into(),
    //         )
    //     });

    //     // repeat START
    //     self.0
    //         .comd2
    //         .write(|w| unsafe { w.command2().bits(Command::Start.into()) });

    //     // load slave address into FIFO
    //     unsafe { ptr::write_volatile(fifo_addr, addr << 1 | OperationType::READ as u8) };

    //     // WRITE slave address
    //     self.0.comd3.write(|w| unsafe {
    //         w.command3().bits(
    //             Command::Write {
    //                 ack_exp: Ack::ACK,
    //                 ack_check_en: false,
    //                 length: 1,
    //             }
    //             .into(),
    //         )
    //     });

    //     if buffer.len() > 1 {
    //         // READ first n - 1 bytes
    //         self.0.comd4.write(|w| unsafe {
    //             w.command4().bits(
    //                 Command::Read {
    //                     ack_value: Ack::ACK,
    //                     length: buffer.len() as u8 - 1,
    //                 }
    //                 .into(),
    //             )
    //         });

    //         // READ last byte
    //         self.0.comd5.write(|w| unsafe {
    //             w.command5().bits(
    //                 Command::Read {
    //                     ack_value: Ack::NACK,
    //                     length: 1,
    //                 }
    //                 .into(),
    //             )
    //         });

    //         // STOP
    //         self.0
    //             .comd6
    //             .write(|w| unsafe { w.command6().bits(Command::Stop.into()) });
    //     } else {
    //         // READ byte
    //         self.0.comd4.write(|w| unsafe {
    //             w.command4().bits(
    //                 Command::Read {
    //                     ack_value: Ack::NACK,
    //                     length: 1,
    //                 }
    //                 .into(),
    //             )
    //         });

    //         // STOP
    //         self.0
    //             .comd5
    //             .write(|w| unsafe { w.command5().bits(Command::Stop.into()) });
    //     }

    //     // Start transmission
    //     self.0.ctr.modify(|_, w| w.trans_start().set_bit());

    //     // Busy wait for all commands to be marked as done
    //     while self.0.comd0.read().command0_done().bit() != true {}
    //     while self.0.comd1.read().command1_done().bit() != true {}
    //     while self.0.comd2.read().command2_done().bit() != true {}
    //     while self.0.comd3.read().command3_done().bit() != true {}
    //     while self.0.comd4.read().command4_done().bit() != true {}
    //     while self.0.comd5.read().command5_done().bit() != true {}
    //     if buffer.len() > 1 {
    //         while self.0.comd6.read().command6_done().bit() != true {}
    //     }

    //     // read bytes from FIFO
    //     let fifo_addr = self.fifo_addr(OperationType::READ) as *mut u8;
    //     for byte in buffer.iter_mut() {
    //         *byte = unsafe { ptr::read_volatile(fifo_addr) };
    //     }

    //     Ok(())
    // }

    // /// Return the raw interface to the underlying I2C peripheral
    // fn free(self) -> T {
    //     self.0
    // }
}

impl Read for I2C<I2C0> {
    type Error = Error;

    fn read(&mut self, _address: u8, _buffer: &mut [u8]) -> Result<(), Self::Error> {
        todo!()
    }
}

impl Write for I2C<I2C0> {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write(addr, bytes)
    }
}

impl WriteRead for I2C<I2C0> {
    type Error = Error;

    fn write_read(
        &mut self,
        _address: u8,
        _bytes: &[u8],
        _buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        todo!()
    }
}

/// Pins used by the I2C interface
///
/// Note that any two pins may be used
/// TODO: enforce this in the type system
pub struct Pins<SDA: OutputPin + InputPin, SCL: OutputPin + InputPin> {
    pub sda: SDA,
    pub scl: SCL,
}

/// I2C Command
enum Command {
    Start,
    Stop,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is 255, while the minimum
        /// is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte has been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255, while the minimum is 1.
        length: u8,
    },
}

impl From<Command> for u16 {
    fn from(c: Command) -> u16 {
        let opcode = match c {
            Command::Start => Opcode::RSTART,
            Command::Stop => Opcode::STOP,
            Command::Write { .. } => Opcode::WRITE,
            Command::Read { .. } => Opcode::READ,
        };

        let length = match c {
            Command::Start | Command::Stop => 0,
            Command::Write { length: l, .. } | Command::Read { length: l, .. } => l,
        };

        let ack_exp = match c {
            Command::Start | Command::Stop | Command::Read { .. } => Ack::NACK,
            Command::Write { ack_exp: exp, .. } => exp,
        };

        let ack_check_en = match c {
            Command::Start | Command::Stop | Command::Read { .. } => false,
            Command::Write {
                ack_check_en: en, ..
            } => en,
        };

        let ack_value = match c {
            Command::Start | Command::Stop | Command::Write { .. } => Ack::NACK,
            Command::Read { ack_value: ack, .. } => ack,
        };

        let mut cmd: u16 = length.into();

        if ack_check_en {
            cmd |= 1 << 8;
        } else {
            cmd &= !(1 << 8);
        }

        if ack_exp == Ack::NACK {
            cmd |= 1 << 9;
        } else {
            cmd &= !(1 << 9);
        }

        if ack_value == Ack::NACK {
            cmd |= 1 << 10;
        } else {
            cmd &= !(1 << 10);
        }

        cmd |= (opcode as u16) << 11;

        cmd
    }
}

enum OperationType {
    WRITE = 0,
    READ = 1,
}

#[derive(Eq, PartialEq, Copy, Clone)]
enum Ack {
    ACK,
    NACK,
}

#[allow(dead_code)]
enum Opcode {
    RSTART = 0,
    WRITE = 1,
    READ = 2,
    STOP = 3,
    END = 4,
}
