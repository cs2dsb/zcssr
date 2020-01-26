use stm32f1xx_hal::gpio::State;
use core::convert::Infallible;
use embedded_hal::digital::v2::OutputPin;
use super::aliases::*;

/// This trait hides if an output is active low or active high.
/// Infallible error type allows .unwrap on set_high and set_low
pub trait EnableOutput: OutputPin<Error = Infallible> {
    /// The State for when this output is on
    const ON_STATE: State;
    /// The State for when this output is off
    const OFF_STATE: State;
    /// Turns the output on
    fn on(&mut self);
    /// Turns the output off
    fn off(&mut self);
}


const ACTIVE_HIGH: (State, State) = (State::High, State::Low);
const ACTIVE_LOW: (State, State) = (State::Low, State::High);

macro_rules! define_enable_output {
    ($name: ty, $active_state: expr) => (
        impl EnableOutput for $name {
            const ON_STATE: State = $active_state.0;
            const OFF_STATE: State = $active_state.1;
            fn on(&mut self) {
                match Self::ON_STATE {
                    State::High => self.set_high().unwrap(),
                    State::Low => self.set_low().unwrap(),
                }
            }
            fn off(&mut self) {
                match Self::ON_STATE {
                    State::High => self.set_low().unwrap(),
                    State::Low => self.set_high().unwrap(),
                }
            }
        }
    )
}

define_enable_output!(LedUsr, ACTIVE_LOW);
define_enable_output!(LedStatus, ACTIVE_HIGH);
define_enable_output!(MiscEn, ACTIVE_HIGH);
define_enable_output!(KThermoNss, ACTIVE_LOW);