#![no_std]
#![no_main]

#![feature(const_fn)]
#![cfg_attr(feature = "bench", feature(test))]

#[allow(unused_imports)]
use cortex_m::asm;

use core::{
    panic::PanicInfo,
    sync::atomic::{self, Ordering},
};
use cortex_m::{
    interrupt,
    iprintln,
};

#[cfg(feature = "itm")] use cortex_m::peripheral::{
    ITM,
};
//#[cfg(feature = "itm")] use itm_packets::*;
#[cfg(feature = "itm")] use itm_logger::*;
#[cfg(not(feature = "itm"))] use itm_logger::{ info };

use zcssr::board::*;

//Used for interrupt vectors if nothing else
use stm32f1xx_hal::time::Hertz;

#[rtfm::app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    #[init]
    fn init() {
        #[cfg(feature = "itm")]
        {
            //After reset the clock is set to HSI and the TPIU clock scaler doesn't get reset (unless openocd relaunches or gdb is set to do this)
            let hsi: Hertz = HSI.into();
            let baud: Hertz = ITM_BAUDRATE.into();
            update_tpiu_baudrate(hsi.0, baud.0).expect("Failed to reset TPIU baudrate");
            logger_init();

            info!("ITM reset");
        }

        info!("Configured max freq");
        info!("Init done");
    }

    #[idle]
    fn idle() -> ! {
        loop {
            asm::wfi();
        }
    }
};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    interrupt::disable();

    #[cfg(feature = "itm")]
    {
        let itm = unsafe { &mut *ITM::ptr() };
        let stim = &mut itm.stim[0];

        iprintln!(stim, "{}", info);
    }

    loop {
        // add some side effect to prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        atomic::compiler_fence(Ordering::SeqCst)
    }
}