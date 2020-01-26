use embedded_hal::PwmPin;
use super::aliases::*;

/// Convenience trait to treat multiple Pwm channels as one 
pub trait MultiPwm {
    /// Get the maxiumum duty (aka ARR) for the channels of this group
    fn get_max_duty(&self) -> u16;
    /// Set the duty of all channels of this group
    fn set_duty(&mut self, duty: u16);
    /// Enable all channels of this group
    fn enable(&mut self);
}

// Used here to make sure ssr status led is on the same time as the ssr output
impl MultiPwm for Ssr {    
    fn get_max_duty(&self) -> u16 {
        self.0.get_max_duty()
    }
    fn set_duty(&mut self, duty: u16) {
        self.0.set_duty(duty);
        self.1.set_duty(duty);
    }
    fn enable(&mut self) {
        self.0.enable();
        self.1.enable();
    }
}