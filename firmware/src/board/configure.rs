use stm32f1xx_hal::{
    rcc::{self, Clocks},
    afio,
    flash,
    gpio::{
        State,
        Input,
        Floating,
        gpioa,
        gpiob,
        Edge,
        ExtiPin,
    },    
    timer::{
        Timer,
        Event,
    },
    spi::{
        Spi,
        NoMosi,
    },
    i2c::{
        I2c,
        blocking_i2c,
        Error as I2cError,
    },
    pac::{
        I2C2,
        DWT,
        SPI2,
        EXTI,
        DBGMCU,
    },
    delay::Delay,
    qei::QeiOptions,
};
use i2c_hung_fix::{
    try_unhang_i2c,
    RECOMMENDED_MAX_CLOCK_CYCLES,
    Error as HungError,
};
use itm_logger::*;
use super::{
    constants::*,
    aliases::*,
    MultiPwm,
    EnableOutput,
};

#[cfg(not(feature = "dummy_zc"))]
use stm32f1xx_hal::{ 
    pwm_input::{
        Configuration as PwmInputConfig,
        ReadMode,
    },
};   

use ht16k33::{
    HT16K33,
    Display as HtDisplay,
};

use nb::Error as NbError;

use core::convert::Infallible;

/// Errors that can be returned by Configure::configure
#[derive(Debug)]
pub enum Error {
    /// A Non-Blocking I2C error
    I2cNbError(NbError<I2cError>),
    /// An I2C error
    I2cError(I2cError),
    /// The hang fix has failed and the bus appears to be hung
    I2cHung(HungError<Infallible>),
}

impl From<NbError<I2cError>> for Error {
    fn from(e: NbError<I2cError>) -> Self {
        Error::I2cNbError(e)
    }
}

impl From<I2cError> for Error {
    fn from(e: I2cError) -> Self {
        Error::I2cError(e)
    }
}

impl From<HungError<Infallible>> for Error {
    fn from(e: HungError<Infallible>) -> Self {
        Error::I2cHung(e)
    }
}

/// This trait allows a configure function to be implemented on the type aliases
/// from super::aliases. This provides a somewhat convenient interface to configure
/// the various board devices without the caller having to do it all manually.
pub trait Configure<'a> 
where Self: Sized
{
    /// The type of the parameters that need to be provided to the configure function
    type Params;
    /// Configure the peripherals required to construct `Self`
    fn configure(_: Self::Params) -> Result<Self, Error>;
}

impl<'a> Configure<'a> for LedUsr {
    type Params = (
        gpioa::PA7<Input<Floating>>, 
        &'a mut gpioa::CRL,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_push_pull_output_with_state(cr, Self::OFF_STATE))
    }
}

impl<'a> Configure<'a> for Display {
    type Params = (
        &'a mut DWT, 
        gpiob::PB10<Input<Floating>>, 
        gpiob::PB11<Input<Floating>>,
        &'a mut gpiob::CRH,
        &'a mut Delay,
        I2C2,
        Clocks,
        &'a mut rcc::APB1,
    );
    fn configure((dwt, scl, sda, cr, delay, i2c2, clocks, apb1): Self::Params) -> Result<Self, Error> {
        // Extra logging is because this tends to fail/hang so it's nice to see what was happening in the log
        debug!("Initializing I2C display...");

        // BlockingI2c relies upon DWT being running
        dwt.enable_cycle_counter();

        // Temporarily configure the scl as output to run the hung fux
        let mut scl = scl.into_push_pull_output_with_state(cr, State::High);
        
        // Run the hung fix
        try_unhang_i2c(
            &mut scl,
            &sda,
            delay,
            DISP_BAUDRATE.0,
            RECOMMENDED_MAX_CLOCK_CYCLES,
        )?;

        // Convert the pins into alternate mode for I2C
        let pins = (
            scl.into_alternate_open_drain(cr),
            sda.into_alternate_open_drain(cr),
        );
        
        // Create the hal i2c peripheral
        let i2c2 = blocking_i2c(
            I2c::i2c2(
                i2c2, 
                pins,
                DISP_I2C_MODE,
                clocks,
                apb1,
            ),
            clocks,
            DISP_START_TIMEOUT_US,
            DISP_START_RETRIES,
            DISP_ADDR_TIMEOUT_US,
            DISP_DATA_TIMEOUT_US,
        );

        let mut ht16k33 = HT16K33::new(i2c2, DISP_I2C_ADDR);
        ht16k33.initialize()?;
        ht16k33.set_display(HtDisplay::ON)?;
        ht16k33.write_display_buffer()?;

        debug!("... i2c done");

        Ok(ht16k33)
    }
}

impl<'a> Configure<'a> for KThermoNss {
    type Params = (
        gpiob::PB12<Input<Floating>>, 
        &'a mut gpiob::CRH,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_push_pull_output_with_state(cr, Self::OFF_STATE))
    }
}

impl<'a> Configure<'a> for KThermoSck {
    type Params = (
        gpiob::PB13<Input<Floating>>, 
        &'a mut gpiob::CRH,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_alternate_push_pull(cr))
    }
}

impl<'a> Configure<'a> for KThermoMiso {
    type Params = (
        gpiob::PB14<Input<Floating>>, 
        &'a mut gpiob::CRH,
    );
    fn configure((pin, _cr): Self::Params) -> Result<Self, Error> {
        Ok(pin)
    }
}

impl<'a> Configure<'a> for Thermocouple {
    type Params = (
        KThermoNss,
        KThermoSck,
        KThermoMiso,
        SPI2,
        Clocks,
        &'a mut rcc::APB1,
    );
    fn configure((cs, sck, miso, spi, clocks, apb1): Self::Params) -> Result<Self, Error> {
        // Create the hal spi peripheral
        let spi = Spi::spi2(
            spi,
            (sck, miso, NoMosi),
            MAX31855_SPI_MODE,
            MAX31855_BAUDRATE,
            clocks,
            apb1
        );

        Ok((spi, cs))
    }
}

impl<'a> Configure<'a> for Ntc {
    type Params = (
        gpiob::PB1<Input<Floating>>, 
        &'a mut gpiob::CRL,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_analog(cr))
    }
}


impl<'a> Configure<'a> for EncA {
    type Params = (
        gpiob::PB6<Input<Floating>>, 
        &'a mut gpiob::CRL,
    );
    fn configure((pin, _cr): Self::Params) -> Result<Self, Error> {
        Ok(pin)
    }
}


impl<'a> Configure<'a> for EncB {
    type Params = (
        gpiob::PB7<Input<Floating>>, 
        &'a mut gpiob::CRL,
    );
    fn configure((pin, _cr): Self::Params) -> Result<Self, Error> {
        Ok(pin)
    }
}


impl<'a> Configure<'a> for Encoder {
    type Params = (
        EncA,
        EncB,
        QuadTim, 
        Clocks,
        &'a mut rcc::APB1,
        &'a mut afio::MAPR,
    );
    fn configure((a, b, tim, clocks, apb1, mapr): Self::Params) -> Result<Self, Error> {
        Ok(Timer::tim4(tim, &clocks, apb1)
            .qei((a, b), mapr, QeiOptions::default()))
    }
}


impl<'a> Configure<'a> for EncBtn {
    type Params = (
        gpiob::PB5<Input<Floating>>, 
        &'a mut gpiob::CRL,
        &'a mut EXTI,
        &'a mut afio::Parts,
    );
    fn configure((mut pin, _cr, exti, afio): Self::Params) -> Result<Self, Error> {
        pin.make_interrupt_source(afio);
        pin.trigger_on_edge(exti, Edge::FALLING);
        pin.enable_interrupt(exti);

        Ok(pin)
    }
}


impl<'a> Configure<'a> for Trigger {
    type Params = (
        gpioa::PA8<Input<Floating>>, 
        &'a mut gpioa::CRH,
        &'a mut EXTI,
        &'a mut afio::Parts,
    );
    fn configure((mut pin, _cr, exti, afio): Self::Params) -> Result<Self, Error> {
        pin.make_interrupt_source(afio);
        pin.trigger_on_edge(exti, Edge::RISING);
        pin.enable_interrupt(exti);

        Ok(pin)
    }
}


impl<'a> Configure<'a> for LedStatus {
    type Params = (
        gpioa::PA2<Input<Floating>>, 
        &'a mut gpioa::CRL,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_push_pull_output_with_state(cr, Self::OFF_STATE))
    }
}


impl<'a> Configure<'a> for MiscEn {
    type Params = (
        gpiob::PB15<Input<Floating>>, 
        &'a mut gpiob::CRH,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_push_pull_output_with_state(cr, Self::OFF_STATE))
    }
}


impl<'a> Configure<'a> for ZcRising {
    type Params = (
        gpioa::PA0<Input<Floating>>, 
        &'a mut gpioa::CRL,
    );
    fn configure((pin, _cr): Self::Params) -> Result<Self, Error> {
        Ok(pin)
    }
}


impl<'a> Configure<'a> for ZcFalling {
    type Params = (
        gpioa::PA1<Input<Floating>>, 
        &'a mut gpioa::CRL,
    );
    fn configure((pin, _cr): Self::Params) -> Result<Self, Error> {
        Ok(pin)
    }
}


impl<'a> Configure<'a> for Zc {
    type Params = (
        ZcRising, 
        ZcFalling,
        ZcTim,
        Clocks,
        &'a mut rcc::APB1,
        &'a mut afio::MAPR, 
        &'a mut DBGMCU, 
    );
    #[cfg_attr(feature = "dummy_zc", allow(unused_variables))]
    fn configure((rising, falling, tim, clocks, apb1, mapr, dbg): Self::Params) -> Result<Self, Error> {
        let zc_in;
        
        #[cfg(feature = "dummy_zc")]
        {
            // Dummy ZC is just the same timer running in count down mode
            let mut zc_in_ = Timer::tim2(tim, &clocks, apb1)
                .start_count_down(DUMMY_FREQ);
            zc_in_.listen(Event::Update);
            zc_in = zc_in_;
        }       
        
        #[cfg(not(feature = "dummy_zc"))]
        {
            // Real ZC uses pwm input mode
            zc_in = Timer::tim2(tim, &clocks, apb1)
                .pwm_input((rising, falling), mapr, dbg, PwmInputConfig::Frequency(ZC_INPUT_FREQ));
        
            if WAIT_FOR_ZC_INIT {
                info!("Waiting for ZC timer to stabilize");
                while zc_in.read_frequency(ReadMode::Instant, &clocks).is_err() {}
            }
        }

        Ok(zc_in)
    }
}


impl<'a> Configure<'a> for SsrEn {
    type Params = (
        gpioa::PA9<Input<Floating>>, 
        &'a mut gpioa::CRH,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_alternate_push_pull(cr))
    }
}


impl<'a> Configure<'a> for LedSsrStatus {
    type Params = (
        gpioa::PA10<Input<Floating>>, 
        &'a mut gpioa::CRH,
    );
    fn configure((pin, cr): Self::Params) -> Result<Self, Error> {
        Ok(pin.into_alternate_push_pull(cr))
    }
}


impl<'a> Configure<'a> for Ssr {
    type Params = (
        SsrEn,
        LedSsrStatus,
        SsrTim,
        Clocks,
        &'a mut rcc::APB2,
        &'a mut afio::MAPR, 
        
    );
    fn configure((en, led, tim, clocks, apb2, mapr): Self::Params) -> Result<Self, Error> {
        let mut ssr_timer = Timer::tim1(tim, &clocks, apb2)
            .pwm((en, led), mapr, SSR_TIMER_FREQ);
        
        ssr_timer.set_duty(0);
        ssr_timer.enable();

        // TODO: move this functionality into the HAL
        // moe is a bug in the advanced timer impl
        // opm is an extra feature but straighforward
        // pwm mode should probably be a param to .pwm(...)
        unsafe {
            // Can't use the mut ref we already have because the hal function resets the peripheral 
            // and keeps the reference. 

            let tim = &(*SsrTim::ptr());
            tim.cr1.modify(|_, w| w
                // One pulse mode
                .opm().enabled()
            );
            tim.bdtr.modify(|_, w| w
                // Master output enable
                .moe().enabled()
            );
            tim.dier.modify(|_, w| w
                // Enable channel 2 compare interrupt for debugging
                //.cc2ie().enabled()
            );
            tim.ccmr1_output().modify(|_, w| w
                // This is channel 2, the SSR EN output
                // Mode 2 is idle low, hal sets it to mode 1                
                .oc2m().pwm_mode2() 
            );
            tim.ccmr2_output().modify(|_, w| w
                // This is channel 3, the SSR Status LED
                // Mode 2 is idle low, hal sets it to mode 1
                .oc3m().pwm_mode2() 
            );
        }

        Ok(ssr_timer)
    }
}


impl<'a> Configure<'a> for UiTimer {
    type Params = (
        UiTim,
        Clocks,
        &'a mut rcc::APB2,
    );
    fn configure((tim, clocks, apb2): Self::Params) -> Result<Self, Error> {
        let mut ui_timer = Timer::tim8(tim, &clocks, apb2)
            .start_count_down(UI_TIMER_FREQ);
        ui_timer.listen(Event::Update);        

        Ok(ui_timer)
    }
}

impl <'a> Configure<'a> for Clocks {
    type Params = (
        rcc::CFGR,
        &'a mut flash::ACR,
    );
    fn configure((cfgr, acr): Self::Params) -> Result<Self, Error> {
        Ok(cfgr
            .use_hse(HSE)
            .sysclk(SYSCLK_FREQ)
            .pclk1(PCLK1_FREQ)
            .pclk2(PCLK2_FREQ)
            .adcclk(ADC_FREQ)
            .freeze(acr))
    }
}