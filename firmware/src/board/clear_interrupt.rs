use super::aliases::{Zc, ZcTim};

/// Convenience trait for clearing interrupts of the various peripherals
pub trait ClearInterrupt {
    /// Clear the interrupt associated with `Self`
    fn clear_interrupt(&mut self);
}

impl ClearInterrupt for Zc {
    fn clear_interrupt(&mut self) {
        #[cfg(feature = "dummy_zc")]
        unsafe {
            &(*ZcTim::ptr()).sr.modify(|_, w| w
                .uif().clear_bit()
            );
        }
        #[cfg(not(feature = "dummy_zc"))]
        unsafe {
            &(*ZcTim::ptr()).sr.modify(|_, w| w
                .cc1if().clear_bit()
                .cc2if().clear_bit()
            );
        }
    }
}