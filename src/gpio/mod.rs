//! GPIO and pin configuration
//!
//! ESP32-C3 has very flexible pin assignment via the GPIO mux. It also has a separate RTC CTL
//! peripheral for low power and analog functions.
//!
//! To support this flexibility two sets of traits are supported:
//! - The various embedded_hal properties
//! - Dedicated [InputPin], [OutputPin], [RTCInputPin] and [RTCOutputPin]
//!
//! The advantage of using the dedicated traits in peripherals is that the configuration of the
//! IO can be done inside the peripheral instead of having to be done upfront.
//!
//! This file is derived from the esp32-hal implementation.

use {
    crate::pac::{GPIO, IO_MUX},
    core::{convert::Infallible, marker::PhantomData},
    embedded_hal::digital::v2::{OutputPin as _, StatefulOutputPin as _},
};

mod mux;
pub use crate::prelude::*;
pub use mux::*;

/// Wrapper for the (pin-based) IO functionality
#[allow(dead_code)]
pub struct IO {
    io_mux: IO_MUX,
    pub pins: Pins,
}

impl IO {
    pub fn new(gpio: GPIO, io_mux: IO_MUX) -> Self {
        let pins = gpio.split();
        let io = IO { io_mux, pins };
        io
    }
}

/// Extension trait to split a GPIO peripheral into independent pins and registers
pub trait GpioExt {
    /// The type to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self) -> Self::Parts;
}

/// Functions available on all pins
pub trait Pin {
    /// Enable/Disable the sleep mode of the pad
    fn sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Set the alternate function
    fn set_alternate_function(&mut self, alternate: AlternateFunction) -> &mut Self;

    /// Start listening to pin interrupt event
    ///
    /// The event sets the type of edge or level triggering.
    ///
    /// This is a wrapper around [listen_with_options][
    /// Pin::listen_with_options], which enables the interrupt for the current core and disables
    /// all other options.
    fn listen(&mut self, event: Event) {
        self.listen_with_options(event, true, false, false)
    }

    /// Start listening to pin interrupt event
    ///
    /// The event sets the type of edge or level triggering.
    /// Interrupts can be individually enabled for the app and pro cores and either a regular
    /// interrupt can be fired or a non-maskable interrupt (NMI). Also wake-up from light sleep
    /// can be enabled
    ///
    /// This function overwrites any previous settings, so if any of the boolean are set to false
    /// the interrupt of that type and to that core are disabled.
    ///
    /// *Note: Edge triggering is not supported for wake-up (FIXME: Verify for ESP32-C3, but makes sense)
    ///
    /// *Note: Even though the interrupt is called NMI it can be routed to any level via the
    /// [interrupt::enable_with_priority][crate::interrupt::enable_with_priority] function.*
    fn listen_with_options(
        &mut self,
        event: Event,
        int_enable: bool,
        nmi_enable: bool,
        wake_up_from_light_sleep: bool,
    );

    /// Stop listening to all pin interrupts
    fn unlisten(&mut self);

    /// Clear a pending interrupt
    fn clear_interrupt(&mut self);

    /// Check if interrupt for this pin is set for the current core
    fn is_interrupt_set(&mut self) -> bool;

    /// Check if the non maskable interrupt for this pin is set for the current core
    fn is_non_maskable_interrupt_set(&mut self) -> bool;

    /// Enable/Disable holding of the pads current state even through reset or deep sleep
    fn enable_hold(&mut self, on: bool);
}

/// Functions available on input pins
pub trait InputPin: Pin {
    /// Set pad as input
    ///
    /// Disables output, pull up/down resistors and sleep mode.
    /// Sets function to GPIO. Does not change sleep mode settings
    fn set_to_input(&mut self) -> &mut Self;

    /// Enable/Disable input circuitry
    fn enable_input(&mut self, on: bool) -> &mut Self;

    /// Enable/Disable input circuitry while in sleep mode
    fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Get state of input
    fn is_input_high(&mut self) -> bool;

    /// Connect input to peripheral using default options
    ///
    /// This is a wrapper around [connect_input_to_peripheral_with_options][
    /// InputPin::connect_input_to_peripheral_with_options], which sets
    /// all the options to false.
    fn connect_input_to_peripheral(&mut self, signal: InputSignal) -> &mut Self {
        self.connect_input_to_peripheral_with_options(signal, false, false)
    }

    /// Connect input to peripheral
    ///
    /// `invert` inverts the output signal and `force_via_gpio_mux` forces the signal
    /// to be routed through the gpio mux even when it could be routed directly via
    /// the io mux.
    fn connect_input_to_peripheral_with_options(
        &mut self,
        signal: InputSignal,
        invert: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self;
}

/// Functions available on output pins
pub trait OutputPin: Pin {
    /// Set pad to open drain output
    ///
    /// Disables input, pull up/down resistors and sleep mode.
    /// Sets function to GPIO and drive strength to default (20mA).
    /// Does not change sleep mode settings.
    fn set_to_open_drain_output(&mut self) -> &mut Self;

    /// Set pad to push/pull output
    ///
    /// Disables input, pull up/down resistors and sleep mode.
    /// Sets function to GPIO and drive strength to default (20mA).
    /// Does not change sleep mode settings.
    fn set_to_push_pull_output(&mut self) -> &mut Self;

    /// Enable/disable the output
    fn enable_output(&mut self, on: bool) -> &mut Self;

    /// Set the output to high or low
    fn set_output_high(&mut self, on: bool) -> &mut Self;

    /// Set drive strength
    fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self;

    /// Enable/Disable open drain
    fn enable_open_drain(&mut self, on: bool) -> &mut Self;

    /// Enable/disable the output while in sleep mode
    fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Enable/Disable internal pull up resistor while in sleep mode
    fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Enable/Disable internal pull down resistor while in sleep mode
    fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self;

    /// Connect peripheral to output using default options
    ///
    /// This is a wrapper around [connect_peripheral_to_output_with_options][
    /// OutputPin::connect_peripheral_to_output_with_options], which sets
    /// all the options to false.
    fn connect_peripheral_to_output(&mut self, signal: OutputSignal) -> &mut Self {
        self.connect_peripheral_to_output_with_options(signal, false, false, false, false)
    }

    /// Connect peripheral to output
    ///
    /// `invert` inverts the output signal, `invert_enable` inverts the output
    /// enable signal, `enable_from_gpio` uses the output enable signal from the gpio
    /// control register instead of controlling it by the peripheral and
    /// `force_via_gpio_mux` forces the signal to be routed through the gpio mux even
    /// when it could be routed directly via the io mux.
    fn connect_peripheral_to_output_with_options(
        &mut self,
        signal: OutputSignal,
        invert: bool,
        invert_enable: bool,
        enable_from_gpio: bool,
        force_via_gpio_mux: bool,
    ) -> &mut Self;

    /// Enable/Disable internal pull up resistor
    fn internal_pull_up(&mut self, on: bool) -> &mut Self;

    /// Enable/Disable internal pull down resistor
    fn internal_pull_down(&mut self, on: bool) -> &mut Self;
}

/// Functions available on RTC pins only
pub trait RTCPin {
    // Not supported yet (not documented in TRM v0.3)
}

/// Functions available on analog pins only
pub trait AnalogPin {
    // Not supported yet (not documented in TRM v0.3)
}

/// Interrupt events
///
/// FIXME: Check applicability of not for ESP32-C3
/// *Note: ESP32 has a bug (3.14), which prevents correct triggering of interrupts when
/// multiple GPIO's are configured for edge triggering in a group (GPIO0-31 is one group,
/// GPIO32-39 is the other group). This can be worked around by using level triggering on the
/// GPIO with edge triggering on the CPU.*
//
// Value must correspond to values in the register
#[derive(Copy, Clone)]
pub enum Event {
    /// Trigger on the rising edge
    RisingEdge = 1,
    /// Trigger on the falling edge
    FallingEdge = 2,
    /// Trigger on any edge
    AnyEdge = 3,
    /// Trigger while low level
    LowLevel = 4,
    /// Trigger while high level
    HighLevel = 5,
}

/// Unknown mode (type state)
pub struct Unknown {}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Input mode via RTC (type state)
pub struct RTCInput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;

/// Pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Output mode via RTC (type state)
pub struct RTCOutput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Push pull output (type state)
pub struct PushPull;

/// Analog mode (type state)
pub struct Analog;

/// Alternate function (type state)
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}

/// Alternate Function 0
pub struct AF0;

/// Alternate Function 1
pub struct AF1;

/// Alternate Function 2
pub struct AF2;

/// Drive strength (values are approximates)
pub enum DriveStrength {
    I5mA = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

/// Alternative pin functions
#[derive(PartialEq)]
pub enum AlternateFunction {
    Function0 = 0,
    Function1 = 1,
    Function2 = 2,
}

/// Connect fixed low to peripheral
pub fn connect_low_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
        w.sel()
            .set_bit()
            .in_inv_sel()
            .bit(false)
            .in_sel()
            .bits(0x1f)
    });
}

/// Connect fixed high to peripheral
pub fn connect_high_to_peripheral(signal: InputSignal) {
    unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
        w.sel()
            .set_bit()
            .in_inv_sel()
            .bit(false)
            .in_sel()
            .bits(0x1e)
    });
}

macro_rules! impl_output {
    ($pxi:ident:
        (
            $pin_num:expr, $bit:expr, $out_en_set:ident, $out_en_clear:ident,
            $out_set:ident, $out_clear:ident, $out_reg:ident
        ) $( ,( $( $af_signal:ident: $af:ident ),* ))?
    ) => {
        impl<MODE> embedded_hal::digital::v2::OutputPin for $pxi<Output<MODE>> {
            type Error = Infallible;

            fn set_high(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe) atomic write to a stateless register
                unsafe { (*GPIO::ptr()).$out_set.write(|w| w.bits(1 << $bit)) };
                Ok(())
            }

            fn set_low(&mut self) -> Result<(), Self::Error> {
                // NOTE(unsafe) atomic write to a stateless register
                unsafe { (*GPIO::ptr()).$out_clear.write(|w| w.bits(1 << $bit)) };
                Ok(())
            }
        }

        impl<MODE> embedded_hal::digital::v2::StatefulOutputPin for $pxi<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                // NOTE(unsafe) atomic read to a stateless register
                unsafe { Ok((*GPIO::ptr()).$out_reg.read().bits() & (1 << $bit) != 0) }
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_set_high()?)
            }
        }

        impl<MODE> embedded_hal::digital::v2::ToggleableOutputPin for $pxi<Output<MODE>> {
            type Error = Infallible;

            fn toggle(&mut self) -> Result<(), Self::Error> {
                if self.is_set_high()? {
                    Ok(self.set_low()?)
                } else {
                    Ok(self.set_high()?)
                }
            }
        }

        impl<MODE> $pxi<MODE> {
            pub fn into_pull_up_input(self) -> $pxi<Input<PullUp>> {
                self.init_input(false, false);
                $pxi { _mode: PhantomData }
            }

            pub fn into_pull_down_input(self) -> $pxi<Input<PullDown>> {
                self.init_input(true, false);
                $pxi { _mode: PhantomData }
            }

            fn init_output(&self, alternate: AlternateFunction, open_drain: bool) {
                let gpio = unsafe { &*GPIO::ptr() };
                let iomux = unsafe { &*IO_MUX::ptr() };

                // NOTE(unsafe) atomic read to a stateless register
                gpio.$out_en_set.write(|w| unsafe { w.bits(1 << $bit) });
                gpio.pin[$pin_num].modify(|_, w| w.pad_driver().bit(open_drain));
                gpio.func_out_sel_cfg[$pin_num]
                    .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as u8) });

                iomux.gpio[$pin_num].modify(|_, w| unsafe {
                    w.mcu_sel()
                        .bits(alternate as u8)
                        .fun_ie()
                        .clear_bit()
                        .fun_wpd()
                        .clear_bit()
                        .fun_wpu()
                        .clear_bit()
                        .fun_drv()
                        .bits(DriveStrength::I20mA as u8)
                        .slp_sel()
                        .clear_bit()
                });
            }

            pub fn into_push_pull_output(self) -> $pxi<Output<PushPull>> {
                self.init_output(AlternateFunction::Function1, false);
                $pxi { _mode: PhantomData }
            }

            pub fn into_open_drain_output(self) -> $pxi<Output<OpenDrain>> {
                self.init_output(AlternateFunction::Function1, true);
                $pxi { _mode: PhantomData }
            }

            pub fn into_alternate_1(self) -> $pxi<Alternate<AF1>> {
                self.init_output(AlternateFunction::Function1, false);
                $pxi { _mode: PhantomData }
            }

            pub fn into_alternate_2(self) -> $pxi<Alternate<AF2>> {
                self.init_output(AlternateFunction::Function2, false);
                $pxi { _mode: PhantomData }
            }
        }

        impl<MODE> OutputPin for $pxi<MODE> {

            fn set_to_open_drain_output(&mut self) -> &mut Self {
                self.init_output(AlternateFunction::Function1, true);
                self
            }

            fn set_to_push_pull_output(&mut self) -> &mut Self {
                self.init_output(AlternateFunction::Function1, false);
                self
            }

            fn enable_output(&mut self, on: bool) -> &mut Self {
                // NOTE(unsafe) atomic read to a stateless register
                if on {
                    unsafe { &*GPIO::ptr() }
                        .$out_en_set
                        .write(|w| unsafe { w.bits(1 << $bit) });
                } else {
                    unsafe { &*GPIO::ptr() }
                        .$out_en_clear
                        .write(|w| unsafe { w.bits(1 << $bit) });
                }
                self
            }

            fn set_output_high(&mut self, high: bool) -> &mut Self {
                // NOTE(unsafe) atomic read to a stateless register
                if high {
                    unsafe { (*GPIO::ptr()).$out_set.write(|w| w.bits(1 << $bit)) };
                } else {
                    unsafe { (*GPIO::ptr()).$out_clear.write(|w| w.bits(1 << $bit)) };
                }
                self
            }

            fn set_drive_strength(&mut self, strength: DriveStrength) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| unsafe { w.fun_drv().bits(strength as u8) });
                self
            }

            fn enable_open_drain(&mut self, on: bool) -> &mut Self {
                unsafe { &*GPIO::ptr() }.pin[$pin_num].modify(|_, w| w.pad_driver().bit(on));
                self
            }

            fn internal_pull_up_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_wpu().bit(on));
                self
            }

            fn internal_pull_down_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_wpd().bit(on));
                self
            }

            fn enable_output_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_oe().bit(on));
                self
            }

            fn connect_peripheral_to_output_with_options(
                &mut self,
                signal: OutputSignal,
                invert: bool,
                invert_enable: bool,
                enable_from_gpio: bool,
                force_via_gpio_mux: bool,
            ) -> &mut Self {

                let af = if force_via_gpio_mux {
                    AlternateFunction::Function1
                } else {
                    match signal {
                        $( $(
                            OutputSignal::$af_signal => AlternateFunction::$af,
                        )* )?
                        _ => AlternateFunction::Function1
                    }
                };

                if af == AlternateFunction::Function1 && signal as usize > 128 {
                    panic!("Cannot connect this peripheral to GPIO");
                }

                self.set_alternate_function(af);

                let clipped_signal = if signal as usize <= 128 { signal as u8 } else { 128u8 };

                unsafe { &*GPIO::ptr() }.func_out_sel_cfg[$pin_num].modify(|_, w| unsafe {
                    w
                        .out_sel().bits(clipped_signal)
                        .out_inv_sel().bit(invert)
                        .oen_sel().bit(enable_from_gpio)
                        .oen_inv_sel().bit(invert_enable)
                });

                self
            }

            fn internal_pull_up(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }.gpio[$pin_num].modify(|_, w| w.fun_wpu().bit(on));
                self
            }

            fn internal_pull_down(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }.gpio[$pin_num].modify(|_, w| w.fun_wpd().bit(on));
                self
            }
        }
    };
}

macro_rules! impl_input {
    ($pxi:ident:
        ($pin_num:expr, $bit:expr, $out_en_clear:ident, $reg:ident, $reader:ident,
            $status_w1tc:ident, $pcpu_int:ident, $pcpu_nmi:ident
        ) $( ,( $( $af_signal:ident : $af:ident ),* ))?
    ) => {
        impl<MODE> embedded_hal::digital::v2::InputPin for $pxi<Input<MODE>> {
            type Error = Infallible;

            fn is_high(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { &*GPIO::ptr() }.$reg.read().$reader().bits() & (1 << $bit) != 0)
            }

            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(!self.is_high()?)
            }
        }

        impl<MODE> $pxi<MODE> {
            fn init_input(&self, pull_down: bool, pull_up: bool) {
                let gpio = unsafe { &*GPIO::ptr() };
                let iomux = unsafe { &*IO_MUX::ptr() };

                // NOTE(unsafe) atomic read to a stateless register
                gpio.$out_en_clear
                    .write(|w| unsafe { w.bits(1 << $bit) });

                gpio.func_out_sel_cfg[$pin_num]
                    .modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as u8) });

                iomux.gpio[$pin_num].modify(|_, w| unsafe {
                    w.mcu_sel()
                        .bits(2)
                        .fun_ie()
                        .set_bit()
                        .fun_wpd()
                        .bit(pull_down)
                        .fun_wpu()
                        .bit(pull_up)
                        .slp_sel()
                        .clear_bit()
                });
            }

            pub fn into_floating_input(self) -> $pxi<Input<Floating>> {
                self.init_input(false, false);
                $pxi { _mode: PhantomData }
            }
        }

        impl<MODE> InputPin for $pxi<MODE> {
            fn set_to_input(&mut self) -> &mut Self {
                self.init_input(false, false);
                self
            }

            fn enable_input(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.fun_ie().bit(on));
                self
            }

            fn enable_input_in_sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.mcu_ie().bit(on));
                self
            }

            fn is_input_high(&mut self) -> bool {
                unsafe { &*GPIO::ptr() }.$reg.read().$reader().bits() & (1 << $bit) != 0
            }

            fn connect_input_to_peripheral_with_options(
                &mut self,
                signal: InputSignal,
                invert: bool,
                force_via_gpio_mux: bool,
            ) -> &mut Self {

                let af = if force_via_gpio_mux
                {
                    AlternateFunction::Function1
                }
                else {
                    match signal {
                        $( $(
                            InputSignal::$af_signal => AlternateFunction::$af,
                        )* )?
                        _ => AlternateFunction::Function1
                    }
                };

                if af == AlternateFunction::Function1 && signal as usize >= 128 {
                    panic!("Cannot connect GPIO to this peripheral");
                }

                self.set_alternate_function(af);

                if (signal as usize) < 128 {
                    unsafe { &*GPIO::ptr() }.func_in_sel_cfg[signal as usize].modify(|_, w| unsafe {
                        w.sel()
                            .set_bit()
                            .in_inv_sel()
                            .bit(invert)
                            .in_sel()
                            .bits($pin_num)
                    });
                }
                self
            }
        }

        impl<MODE> Pin for $pxi<MODE> {
            fn sleep_mode(&mut self, on: bool) -> &mut Self {
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| w.slp_sel().bit(on));
                self
            }

            fn set_alternate_function(&mut self, alternate: AlternateFunction) -> &mut Self {
                // NOTE(unsafe) atomic read to a stateless register
                unsafe { &*IO_MUX::ptr() }
                    .gpio[$pin_num]
                    .modify(|_, w| unsafe { w.mcu_sel().bits(alternate as u8) });
                self
            }

            fn listen_with_options(&mut self, event: Event,
                int_enable: bool, nmi_enable: bool,
                wake_up_from_light_sleep: bool
            ) {
                if wake_up_from_light_sleep {
                    match event {
                        Event::AnyEdge | Event::RisingEdge | Event::FallingEdge => {
                            panic!("Edge triggering is not supported for wake-up from light sleep");
                        },
                        _ => {}
                    }
                }
                unsafe {
                    (&*GPIO::ptr()).pin[$pin_num].modify(|_, w|
                        w
                            .int_ena().bits(int_enable as u8 | ((nmi_enable as u8) << 1))
                            .int_type().bits(event as u8)
                            .wakeup_enable().bit(wake_up_from_light_sleep)
                    );
                }
            }

            fn unlisten(&mut self) {
                unsafe { (&*GPIO::ptr()).pin[$pin_num].modify(|_, w|
                    w.int_ena().bits(0).int_type().bits(0).int_ena().bits(0) );
                }
            }

            fn clear_interrupt(&mut self) {
                unsafe {&*GPIO::ptr()}.$status_w1tc.write(|w|
                    unsafe {w.bits(1 << $bit)})
            }

            fn is_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$pcpu_int.read().bits() & (1 << $bit)) !=0
            }

            fn is_non_maskable_interrupt_set(&mut self) -> bool {
                (unsafe {&*GPIO::ptr()}.$pcpu_nmi.read().bits() & (1 << $bit)) !=0
            }

            fn enable_hold(&mut self, _on: bool) {
                // Not implemented yet, waiting for documentation in TRM
                // (RTCCNTL peripheral is not documented in TRM v0.3)
                todo!();
            }
        }
    };
}

macro_rules! impl_pin_wrap {
    ($pxi:ident, $pin_num:expr, IO
        $( ,( $( $af_input_signal:ident : $af_input:ident ),* ) )?
    ) => {
        impl_input!($pxi: ($pin_num, $pin_num % 32, enable_w1tc, in_, in_data,
            status_w1tc, pcpu_int, pcpu_nmi_int)
            $( ,( $( $af_input_signal: $af_input ),* ) )? );
    };
}

macro_rules! impl_output_wrap {
    ($pxi:ident, $pin_num:expr, IO
        $( ,( $( $af_output_signal:ident : $af_output:ident ),* ))?
    ) => {
        impl_output!($pxi:
            ($pin_num, $pin_num % 32, enable_w1ts, enable_w1tc, out_w1ts, out_w1tc, out)

            $( ,( $( $af_output_signal: $af_output ),* ) )? );
    };
}

macro_rules! gpio {
    ( $($pxi:ident: ($pname:ident, $pin_num:literal,
        $type:ident, $rtc:tt ),
        $(
            ( $( $af_input_signal:ident: $af_input:ident ),* ),
            $(
            ( $( $af_output_signal:ident: $af_output:ident ),* ),
            )?
        )?
        )+ ) => {

        impl GpioExt for GPIO {
            type Parts = Pins;

            fn split(self) -> Self::Parts {
                Pins {
                    $(
                        $pname: $pxi { _mode: PhantomData },
                    )+
                }
            }
        }

        /// Collection of all GPIO pins
        pub struct Pins {
            $(
                /// Pin
                pub $pname: $pxi<Unknown>,
            )+
        }

        // create all the pins, we can also add functionality
        // applicable to all pin states here
        $(
            /// Pin
            pub struct $pxi<MODE> {
                _mode: PhantomData<MODE>,
            }

            impl_pin_wrap!($pxi, $pin_num, $type
                $( ,( $( $af_input_signal: $af_input ),* ) )? );
            impl_output_wrap!($pxi, $pin_num, $type
                $($( ,( $( $af_output_signal: $af_output ),* ) )? )? );
        )+
    };
}

// All info on reset state pulled from 4.10 IO_MUX Pad List in the reference manual
// TODO these pins have a reset mode of 0 (apart from Gpio27),
// input disable, does that mean they are actually in output mode on reset?

// Pins 0~5 are RTC pins, all others are not (TRM v0.3, section 5.8)
gpio! {
    Gpio0: (gpio0, 0, IO, RTC),
    Gpio1: (gpio1, 1, IO, RTC),
    Gpio2: (gpio2, 2, IO, RTC),(FSPIQ: Function2),(FSPIQ: Function2),
    Gpio3: (gpio3, 3, IO, RTC),
    Gpio4: (gpio4, 4, IO, RTC),(FSPIHD: Function2),(USB_JTAG_TMS: Function0, FSPIHD: Function2),
    Gpio5: (gpio5, 5, IO, RTC),(FSPIWP: Function2),(USB_JTAG_TDI: Function0, FSPIWP: Function2),
    Gpio6: (gpio6, 6, IO, 0),(FSPICLK: Function2),(USB_JTAG_TCK: Function0, FSPICLK_MUX: Function2),
    Gpio7: (gpio7, 7, IO, 0),(FSPID: Function2),(USB_JTAG_TDO: Function0, FSPID: Function2),
    Gpio8: (gpio8, 8, IO, 0),
    Gpio9: (gpio9, 9, IO, 0),
    Gpio10: (gpio10, 10, IO, 0),(FSPICS0: Function2),(FSPICS0: Function2),
    Gpio11: (gpio11, 11, IO, 0),
    Gpio12: (gpio12, 12, IO, 0),(SPIHD: Function0),(SPIHD: Function0),
    Gpio13: (gpio13, 13, IO, 0),(SPIWP: Function0),(SPIWP: Function0),
    Gpio14: (gpio14, 14, IO, 0),(),(SPICS0: Function0),
    Gpio15: (gpio15, 15, IO, 0),(),(SPICLK_MUX: Function0),
    Gpio16: (gpio16, 16, IO, 0),(SPID: Function0),(SPID: Function0),
    Gpio17: (gpio17, 17, IO, 0),(SPIQ: Function0),(SPIQ: Function0),
    Gpio18: (gpio18, 18, IO, 0),
    Gpio19: (gpio19, 19, IO, 0),
    Gpio20: (gpio20, 20, IO, 0),(U0RXD: Function0),(),
    Gpio21: (gpio21, 21, IO, 0),(),(U0TXD: Function0),
}
