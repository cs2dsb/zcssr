use stm32f1xx_hal::rcc::Clocks;
#[cfg(feature = "itm")]
use {
    itm_logger::{ 
        logger_init, 
        update_tpiu_baudrate,
    },
    stm32f1xx_hal::time::Hertz,
    super::constants::{ ITM_BAUDRATE, HSI },
}; 

/// Resets the TPIU clock scaler so ITM works after the chip is reset
pub fn itm_reset() {
    #[cfg(feature = "itm")]
    {        
        let hsi: Hertz = HSI.into();
        let baud: Hertz = ITM_BAUDRATE.into();
        update_tpiu_baudrate(hsi.0, baud.0).expect("Failed to reset TPIU baudrate");
        logger_init();
    }
}

/// Updates the TPIU clock scaler after the sysclk has been changed
pub fn itm_update_clocks(
    #[cfg_attr(not(feature = "itm"), allow(unused_variables))]
    clocks: &Clocks,
) {
    #[cfg(feature = "itm")]
    {
        // Update the baud rate for the new core clock
        let sysclk: Hertz = clocks.sysclk().into();
        let baud: Hertz = ITM_BAUDRATE.into();
        update_tpiu_baudrate(sysclk.0, baud.0).expect("Failed to reset TPIU baudrate");     
    }
}