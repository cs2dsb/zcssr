use itm_logger::*;
use super::{
    aliases::{Ssr, SsrTim},
    MultiPwm,
    SSR_TIMER_PERIOD_US,
};

/// Enables timer to be configured to a one shot pulse of a given microsecond duration
///
/// TODO: should live inside hal 
///
/// TODO: The SSR pulsing only works by virtue of the way the PSC and ARR are calculated
///       in the hal. Currently it picks the lowest PSC that will allow ARR to fit in the
///       16-bit counter which is great because it gives us the maximum resolution. 
///       However, the hal doesn't guarantee that's how it will calculate these values
///       so in future it might not work as expected. There are requests on some of the
///       hals to have a manual config where you provide PSC and ARR yourself which would
///       probably be preferable in this case. Currently it doesn't check how close to the
///       desired us the timer can actually achieve so if the timer was low resolution it 
//        could enable the SSR for much longer than desired, which could be very bad.
pub trait PulseSsr {
    /// Configures the SSR timer for a one shot pulse of the desired_us
    fn pulse(&mut self, desired_us: u32);
}

impl PulseSsr for Ssr {
    fn pulse(&mut self, desired_us: u32) {
        assert!(desired_us < SSR_TIMER_PERIOD_US - 1);

        let max_duty = self.get_max_duty() as u32;
        // This is how long the output should be ON for in timer counts
        let on_count = desired_us * max_duty / SSR_TIMER_PERIOD_US;
        // Which is inverted so when the timer overflows and resets to 0 it's in the off state
        let duty = (max_duty - on_count + 1) as u16;

        self.set_duty(duty);
    
        debug!("Starting SSR pulse, desired_us: {}, duty: {}/{}",
            desired_us, duty, max_duty);

        unsafe {
            let tim = &(*SsrTim::ptr());

            // Timer must not be triggered when it's already running
            assert!(tim.cr1.read().cen().bit_is_clear());

            // Trigger an update event to push changes to shadow/buffer registers (PSC and ARR for sure, possibly others)
            tim.egr.write(|w| w.ug().update());

            // Update the current timer count to be just before the on pulse
            tim.cnt.write(|w| w.cnt().bits(duty - 1));

            tim.cr1.modify(|_, w| w
                // Just to make sure one pulse mode is turned on
                .opm().enabled()
                // Enable the timer
                .cen().set_bit()
            );
        }

    }    
}