#![no_std]
#![no_main]

#![feature(const_fn)]
#![cfg_attr(feature = "bench", feature(test))]

#[allow(unused_imports)]
use cortex_m::{asm, singleton};

use core::{
    panic::PanicInfo,
    sync::atomic::{self, Ordering},
    ops::RangeInclusive,
};
use cortex_m::interrupt;

// For ln
use libm::F32Ext;

#[cfg(feature = "itm")] 
use {
    cortex_m::{iprintln, peripheral::ITM},
    itm_logger::*,
};

#[cfg(not(feature = "itm"))] 
use itm_logger::{ info, error, warn, debug, log, Level };

use zcssr::{
    board::*,
    ui::welder::WelderUi,
};

#[cfg_attr(not(feature = "thermocouple"), allow(unused_imports))]
use max31855::{
    Max31855,
    Unit,
};

use embedded_hal::digital::v2::OutputPin;

use ht16k33::{
    HT16K33,
    Display,
};

use stm32f1xx_hal::{
    prelude::*,
    rcc::Clocks,
    stm32::{
        TIM1,
        TIM2 as TIM2_REG,
        TIM4,
        TIM8,
        DMA1,
        I2C2,
        SPI2,
    },
    gpio::{
        PushPull,
        Alternate,
        ExtiPin,
        Edge,
    },    
    timer::*,
    pwm::{
        C2,
        C3,
        Pwm,
    },
    qei::Qei,
    adc::{
        Adc,
        AdcPayload,
        Continuous,
    },
    dma::{
        RxDma,
        CircBuffer,
        dma1::{
            C1 as DmaC1,
        },
    },
    i2c::{
        I2c,
        blocking_i2c,
        BlockingI2c,
        HungFix,
    },    
    spi::{
        Spi,
        Spi2NoRemap,
        NoMosi,
    },
    delay::Delay,
};

#[cfg(not(feature = "dummy_zc"))]
use stm32f1xx_hal::{
    pwm_input::{
        Configuration as PwmInputConfig,
        ReadMode,
        PwmInput,
    },
};
#[cfg(any(feature = "dummy_zc", feature = "itm"))]
use stm32f1xx_hal::{
    time::Hertz,
};

// If true, panic on failure to configure HT16K33, else just log it
const HT16K33_MISSING_FATAL: bool = true;

// If true, wait (possible forever) for first zero crossing
#[cfg(not(feature = "dummy_zc"))]
const WAIT_FOR_ZC_INIT: bool = true;

#[cfg(feature = "dummy_zc")]
const DUMMY_FREQ: Hertz = Hertz(100);

// If supplied, make sure NTC temperature is in this range before welding
const NTC_RANGE: Option<(f32, f32)> = Some((1., 30.));

// If supplied, make sure thermocouple temperature is in this range before welding
#[cfg(feature = "thermocouple")]
const THERMOCOUPLE_RANGE: Option<(f32, f32)> = Some((1., 30.));

// These ranges specify the min, max and actual frequencies.
// Mains frequency is generally very stable in most countries
// so if we're reading outside these values there is likely something
// wrong with our hardware. Attempting to start a weld with a 
// measured frequency outside of one of these ranges will cause
// a panic.
// The actual frequency is provided so our microsecond to zero crossing
// calculation is more accurate.
const ACCEPTABLE_ZC_FREQ_RANGES: [(RangeInclusive<u16>, u16); 2] = [
    (98..=102, 100),
    (118..=122, 120),
];

#[derive(Clone, Copy, PartialEq)]
pub enum WeldState {
    Idle,
    AWeld,
    ADelay,
    BWeld,
    BDelay,
}

/// Zero crossing timer reads pwm input from the zero crossing detection circuit (or a dummy count down timer if dummy_zc feature is enabled)
type ZcTimer = TIM2_REG;
/// Quadrature encoder timer reads the quad input from the rotary encoder
type QuadTimer = TIM4;
/// The UI timer provides a periodic interrupt to update the display, read temperature and periodically log useful info
type UiTimer = TIM8;
/// The SSR timer drives the SsrEn pin to enable the welder. A timer is used so the control software crashing should never result in the SSR being left enabled
type SsrTimer = TIM1;

// Aliases for the various handles used as RTFM resources
type EncoderHandle = Qei<QuadTimer, Tim4NoRemap, (EncA, EncB)>;
type NtcDmaBufferHandle = CircBuffer<[u16; 8], RxDma<AdcPayload<Ntc, Continuous>, DmaC1>>;
type Max31855Handle = Spi<SPI2, Spi2NoRemap, (KThermoSck, KThermoMiso, NoMosi)>;
type Max31855CsHandle = KThermoNss;
type DisplayHandle = HT16K33<BlockingI2c<I2C2, (DispScl, DispSda)>>;
type ActivityLedHandle = LedUsr;
type UiTimerHandle = CountDownTimer<UiTimer>;
type SsrTimerHandle = (Pwm<SsrTimer, C2>, Pwm<SsrTimer, C3>);
#[cfg(not(feature = "dummy_zc"))] 
type ZcHandle = PwmInput<ZcTimer, Tim2NoRemap, (ZcRising, ZcFalling)>;
#[cfg(feature = "dummy_zc")] 
type ZcHandle = CountDownTimer<ZcTimer>;

// Convenience trait to treat multiple Pwm channels as one 
trait Pwm_{
    fn get_max_duty(&self) -> u16;
    fn set_duty(&mut self, duty: u16);
    fn enable(&mut self);
}
// Used here to make sure ssr status led is on the same time as the ssr output
impl Pwm_ for SsrTimerHandle {    
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

#[rtfm::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        ui_timer: UiTimerHandle,
        enc_quad: EncoderHandle,
        zc_in: ZcHandle,
        clocks: Clocks,
        ntc_dma_buffer: NtcDmaBufferHandle,
        display: DisplayHandle,
        max31855k: Max31855Handle,
        max31855k_cs: KThermoNss,
        activity_led: ActivityLedHandle,
        ssr_timer: SsrTimerHandle,
        enc_button: EncBtn,
        trigger_button: Trigger,
        status_led: LedStatus,
        welder_ui: WelderUi,

        #[init(-400.)]
        last_ntc_temp: f32,

        #[init(-400.)]
        last_thermocouple_temp: f32,

        #[init(false)]
        weld_triggered: bool,
    }
    
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let device = cx.device;
        let mut core = cx.core;

        // Constrain as much hardware here as possible without rcc::Clocks
        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let dma1 = device.DMA1.split(&mut rcc.ahb);

        // PCB activity LED
        // It's configured here and set low (on) so that it's possible to tell
        // if the program has loaded without ITM for debugging. The `on`
        // can be moved down line by line to debug if the init function is ok
        // with OpenOCD connected but failing standalone. (Example of this
        // was that ITM logging wasn't checking ITM was enabled before querying
        // the STIM FIFO status.
        let mut usr_led = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);
        usr_led.off();
        usr_led.on();

        #[cfg(feature = "itm")]
        {        
            // After reset the clock is set to HSI and the TPIU clock scaler doesn't get reset (unless openocd relaunches or gdb is set to do this)
            let hsi: Hertz = HSI.into();
            let baud: Hertz = ITM_BAUDRATE.into();
            update_tpiu_baudrate(hsi.0, baud.0).expect("Failed to reset TPIU baudrate");
            logger_init();

            unsafe {
                //TODO: move to itm_logger crate
                (*ITM::ptr()).tcr.modify(|w|
                    // Enable local timestamps
                    w | (1 << 1)
                );
            }
        }

        // Configure the clock for full speed
        let clocks = rcc.cfgr
            .use_hse(HSE)
            .sysclk(SYSCLK_FREQ)
            .pclk1(PCLK1_FREQ)
            .pclk2(PCLK2_FREQ)
            .adcclk(ADC_FREQ)
            .freeze(&mut flash.acr);

        // The rest of the constraining now that we have rcc::Clocks
        let adc1 = Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
        let mut delay = Delay::new(core.SYST, clocks);

        #[cfg(feature = "itm")]
        {
            // Update the baud rate for the new core clock
            let sysclk: Hertz = clocks.sysclk().into();
            let baud: Hertz = ITM_BAUDRATE.into();
            update_tpiu_baudrate(sysclk.0, baud.0).expect("Failed to reset TPIU baudrate");     
        }

        info!("Configured max freq");


        // Initialize the I2C display
        // Extra logging is because this tends to fail/hang so it's nice to see what was happening in the log
        debug!("Initializing I2C display...");
        // BlockingI2c relies upon DWT being running
        core.DWT.enable_cycle_counter();
        let i2c_pins: (DispScl, DispSda) = {
            let mut pins = (
                gpiob.pb10.into_push_pull_output_with_state(&mut gpiob.crh, stm32f1xx_hal::gpio::State::High),
                gpiob.pb11.into_floating_input(&mut gpiob.crh),
            );

            pins.try_hung_fix(&mut delay, DISP_I2C_MODE).expect("I2C bus is hung");
            (
                pins.0.into_alternate_open_drain(&mut gpiob.crh),
                pins.1.into_alternate_open_drain(&mut gpiob.crh),
            )
        };
        let i2c2 = blocking_i2c(
            I2c::i2c2(
                device.I2C2, 
                i2c_pins,
                DISP_I2C_MODE,
                clocks,
                &mut rcc.apb1,
            ),
            clocks,
            DISP_START_TIMEOUT_US,
            DISP_START_RETRIES,
            DISP_ADDR_TIMEOUT_US,
            DISP_DATA_TIMEOUT_US,
        );
        let mut ht16k33 = HT16K33::new(i2c2, DISP_I2C_ADDR);
        if ht16k33.initialize().is_err() ||
           ht16k33.set_display(Display::ON).is_err() ||
           ht16k33.write_display_buffer().is_err()
        {
            if HT16K33_MISSING_FATAL {
                panic!("Failed to initialize ht16k33");
            } else {
                error!("Failed to initialize ht16k33");
            }
        }
        debug!("...done");

        
        // Initialize the spi thermocouple chip
        let sck_pin: KThermoSck = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
        let miso_pin: KThermoMiso = gpiob.pb14.into_floating_input(&mut gpiob.crh);
        let mut max31855_cs_pin: KThermoNss = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        max31855_cs_pin.set_high().unwrap();
        let max31855 = Spi::spi2(
            device.SPI2,
            (sck_pin, miso_pin, NoMosi),
            MAX31855_SPI_MODE,
            MAX31855_BAUDRATE,
            clocks,
            &mut rcc.apb1
        );


        // Configure the ADC and DMA for reading the NTC thermistor
        let ntc_pin: Ntc = gpiob.pb1.into_analog(&mut gpiob.crl);
        let ntc_dma = adc1.with_dma(
            ntc_pin,
            dma1.1,
        );
        let ntc_buffer = singleton!(: [[u16; 8]; 2] = [[0; 8]; 2]).unwrap();
        let ntc_circ_buffer = ntc_dma.circ_read(ntc_buffer);


        // Configure timer for rotary encoder quadrature input
        let enc_a_pin: EncA = gpiob.pb6.into_floating_input(&mut gpiob.crl);
        let enc_b_pin: EncB = gpiob.pb7.into_floating_input(&mut gpiob.crl);
        let mut quad_input = Timer::tim4(device.TIM4, &clocks, &mut rcc.apb1)
            .qei(
                (enc_a_pin, enc_b_pin),
                &mut afio.mapr
            );


        // Configure EXTI interrupt for rotary encoder button
        let exti = device.EXTI;
        let mut enc_button: EncBtn = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        enc_button.make_interrupt_source(&mut afio);
        enc_button.trigger_on_edge(&exti, Edge::FALLING);
        enc_button.enable_interrupt(&exti);
        

        // Configure EXTI interrupt for trigger button
        let mut trigger_button: Trigger = gpioa.pa8.into_floating_input(&mut gpioa.crh);
        trigger_button.make_interrupt_source(&mut afio);
        trigger_button.trigger_on_edge(&exti, Edge::RISING);
        trigger_button.enable_interrupt(&exti);


        // Status LED
        let mut led_status: LedStatus = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
        led_status.off();


        // Misc en
        let mut misc_en: MiscEn = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
        misc_en.off();


        // ZC input
        let zc_in;
        #[cfg(feature = "dummy_zc")]
        {
            // Dummy ZC is just the same timer running in count down mode
            let mut zc_in_ = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1)
                .start_count_down(DUMMY_FREQ);
            zc_in_.listen(Event::Update);
            zc_in = zc_in_;
        }         
        #[cfg(not(feature = "dummy_zc"))]
        {
            // Real ZC uses pwm input mode
            let mut dbg = device.DBGMCU;
            let zc_rising_pin: ZcRising = gpioa.pa0.into_floating_input(&mut gpioa.crl);
            let zc_falling_pin: ZcFalling = gpioa.pa1.into_floating_input(&mut gpioa.crl);
            zc_in = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1)
                .pwm_input(
                    (zc_rising_pin, zc_falling_pin),
                    &mut afio.mapr, 
                    &mut dbg, 
                    PwmInputConfig::Frequency(300.hz()),
                );

            if WAIT_FOR_ZC_INIT {
                info!("Waiting for ZC timer to stabilize");
                while zc_in.read_frequency(ReadMode::Instant, &clocks).is_err() {}
            }
        }
        
        // SSR Timer        
        let ssr_en: SsrEn<Alternate<PushPull>> = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let ssr_led: LedSsrStatus<Alternate<PushPull>> = gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh);
        let mut ssr_timer: SsrTimerHandle = Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2)
            .pwm(
                (ssr_en, ssr_led),
                &mut afio.mapr,
                SSR_TIMER_FREQ,
            );
        ssr_timer.set_duty(0);
        ssr_timer.enable();
        unsafe {
            let tim = &(*TIM1::ptr());
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


        // UI Timer
        let mut ui_timer = Timer::tim8(device.TIM8, &clocks, &mut rcc.apb2)
            .start_count_down(3.hz());
        ui_timer.listen(Event::Update);

        let mut welder_ui = WelderUi::new();
        // Update the quad counter to match the current setting
        update_quad_from_setting_value(&mut quad_input, *welder_ui.get_value_mut() as u16);


        info!("Init done");

        init::LateResources {
            ui_timer,
            enc_quad: quad_input,
            zc_in,
            clocks,
            ntc_dma_buffer: ntc_circ_buffer,
            display: ht16k33,
            max31855k: max31855,
            max31855k_cs: max31855_cs_pin,
            activity_led: usr_led,
            ssr_timer,
            enc_button,
            trigger_button,
            status_led: led_status,
            welder_ui,
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    /// EXTI9_5 Interrupt handles the quad encoder button and the trigger button
    #[task(binds = EXTI9_5, resources = [
        enc_quad, 
        welder_ui,
        last_ntc_temp,
        last_thermocouple_temp,
        weld_triggered,
        enc_button,
        trigger_button,
        display,
    ])]
    fn exti9_5(cx: exti9_5::Context) {        
        if cx.resources.enc_button.check_interrupt() {
            cx.resources.enc_button.clear_interrupt_pending_bit();
            debug!("EXTI encoder button pressed");
            encoder_button_pressed(cx.resources.welder_ui, cx.resources.enc_quad, cx.resources.display);
        } 

        if cx.resources.trigger_button.check_interrupt() {
            cx.resources.trigger_button.clear_interrupt_pending_bit();
            debug!("EXTI trigger pressed");
            trigger_pressed(cx.resources.last_ntc_temp, cx.resources.last_thermocouple_temp, cx.resources.weld_triggered);
        }
    }

    /// TIM8_UP Interrupt handles reading temperatures and updating the UI
    #[task(binds = TIM8_UP, resources = [
        ui_timer, 
        clocks, 
        display, 
        ntc_dma_buffer, 
        enc_quad,
        welder_ui,
        activity_led,
        max31855k,
        max31855k_cs,
        last_ntc_temp,
        last_thermocouple_temp,
    ])]
    fn tim8_up(cx: tim8_up::Context) {
        static mut COUNT: u16 = 0;

        cx.resources.ui_timer.clear_update_interrupt_flag();

        ui_timer_elapsed(
            COUNT,
            cx.resources.activity_led,
            cx.resources.ntc_dma_buffer, 
            cx.resources.last_ntc_temp,
            cx.resources.max31855k, 
            cx.resources.max31855k_cs, 
            cx.resources.last_thermocouple_temp,
            cx.resources.welder_ui, 
            cx.resources.enc_quad, 
            cx.resources.display,
        );
    }

    /// TIM2 Interrupt
    /// This timer is connected to the ZC circuit (or countdown if dummy_zc feature is enabled)
    #[task(binds = TIM2, resources = [
        weld_triggered,
        welder_ui,
        ssr_timer,
        zc_in,
        clocks,
        status_led,
    ])]
    fn tim2(cx: tim2::Context) {
        // The current welding state
        static mut WELD_STATE: WeldState = WeldState::Idle;
        // Counter to progress through the current state
        static mut WELD_COUNTER: u16 = 0;
        // Local copies of config at trigger time
        static mut A_DELAY: u16 = 0;
        static mut B_ON: u16 = 0;
        static mut B_DELAY: u16 = 0;
        // The measured ZC period in microseconds. Measured just before weld is started and referred to until weld is finished
        static mut ZC_PERIOD_US: u16 = 0;

        // This is purely to divide the zc rate down to a level that is visible on the status led
        static mut COUNT: u16 = 0;

        // Clear interrupt
        #[cfg(feature = "dummy_zc")]
        unsafe {
            &(*ZcTimer::ptr()).sr.modify(|_, w| w
                .uif().clear_bit()
            );
        }
        #[cfg(not(feature = "dummy_zc"))]
        unsafe {
            &(*ZcTimer::ptr()).sr.modify(|_, w| w
                .cc1if().clear_bit()
                .cc2if().clear_bit()
            );
        }

        *COUNT += 1;
        if *COUNT % 20 == 0 {
            cx.resources.status_led.on();
        } else {
            cx.resources.status_led.off();
        }

        zero_crossing_event(
            WELD_STATE,
            cx.resources.weld_triggered,
            WELD_COUNTER,
            cx.resources.welder_ui,
            A_DELAY,
            B_ON,
            B_DELAY,
            ZC_PERIOD_US,
            cx.resources.ssr_timer,
            cx.resources.zc_in,
            cx.resources.clocks,
        );
    }
};

/// Enables timer to be configured to a one shot pulse of a given microsecond duration
///TODO: should live inside hal 
pub trait PulseSsr {
    fn pulse(&mut self, desired_us: u32);
}

impl PulseSsr for SsrTimerHandle {
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
            let tim = &(*SsrTimer::ptr());

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

/// Called when the encoder button has been pressed, updates the UI
fn encoder_button_pressed(welder_ui: &mut WelderUi, enc_quad: &mut EncoderHandle, display: &mut DisplayHandle) {
    if let Err(e) = welder_ui.next_screen(display) {
        if HT16K33_MISSING_FATAL {
            panic!("Error moving to next UI screen: {:?}", e);
        } else {
            error!("Error moving to next UI screen: {:?}", e);
        }
    } else {
        if let Err(e) = display.write_display_buffer() {
            if HT16K33_MISSING_FATAL {
                panic!("HT16K33 error: {:?}", e);
            }
        }
    }
    update_quad_from_setting_value(enc_quad, *welder_ui.get_value_mut() as u16);
    info!("Encoder: {}", enc_quad.count());
}

/// Called when the trigger is pressed, checks the temperatures are safe and initiates a weld
fn trigger_pressed(ntc_temp: &f32, thermocouple_temp: &f32, weld_triggered: &mut bool) {
    let (ntc_ok, thermocouple_ok) = temps_ok(*ntc_temp, *thermocouple_temp);
    if ntc_ok && thermocouple_ok {
        info!("Trigger weld");
        *weld_triggered = true;
    } else {
        warn!("Temperature error: {} = {}, {} = {}", 
            *thermocouple_temp,
            ntc_ok,
            *thermocouple_temp,
            thermocouple_ok,
        );
    }
}

/// Updates the last_ntc_temp with a reading from the dma_buffer
fn read_ntc_temp(dma_buffer: &mut NtcDmaBufferHandle, last_ntc_temp: &mut f32) {
    //This is a hack to get around the overflow detection in the hal implementation
    //TODO: replace this with something better
    //      currently there are occasional erroneous readings due to the buffer being written while
    //      in use because of this hack
    unsafe { &(*DMA1::ptr()).ifcr.write(|w| w.chtif1().set_bit()); }
    let ntc_sample = dma_buffer
        .peek(|half, _| {
            half[0]
        }).unwrap_or(0);
    *last_ntc_temp = ntc_sample_to_degrees(ntc_sample);
}

/// Updates the last_thermocouple_temp with a value read from the max31855
#[cfg(feature = "thermocouple")]
fn read_thermocouple_temp(max31855: &mut Max31855Handle, max31855_cs: &mut Max31855CsHandle, last_thermocouple_temp: &mut f32) {
    match max31855.read_thermocouple(max31855_cs, Unit::Celsius) {
        Ok(v) => *last_thermocouple_temp = v,
        Err(e) => warn!("Error reading thermocouple: {:?}", e),
    };
}

/// Called at 3hz to update the UI from the quad reading & due to elapsed time
fn tick_ui(welder_ui: &mut WelderUi, enc_quad: &EncoderHandle, display: &mut DisplayHandle) {
    let value = welder_ui.get_value_mut();
    let new_value = setting_value_from_quad(enc_quad) as u32;
    let changed = *value != new_value;

    let result = if changed {
        *value = new_value;
        welder_ui.reset_tick(display)
    } else {
        welder_ui.tick(Some(display))
    };

    if let Err(e) = result {
       if HT16K33_MISSING_FATAL {
            panic!("Error ticking UI: {:?}", e);
        } else {
            error!("Error ticking UI: {:?}", e);
        } 
    }

    if let Err(e) = display.write_display_buffer() {
        if HT16K33_MISSING_FATAL {
            panic!("HT16K33 error: {:?}", e);
        }
    }
}

/// Checks the temperatures are in acceptable range if range is specified
fn temps_ok(
    ntc_temp: f32, 
    #[cfg_attr(not(feature = "thermocouple"), allow(unused_variables))]
    thermocouple_temp: f32,
) -> (bool, bool) {
    let ntc_ok = if let Some((min, max)) = NTC_RANGE {
        ntc_temp >= min && ntc_temp <= max
    } else {
        true
    };
    let thermo_ok;

    #[cfg(feature = "thermocouple")]
    {
        thermo_ok = if let Some((min, max)) = THERMOCOUPLE_RANGE {
            thermocouple_temp >= min && thermocouple_temp <= max
        } else {
            true
        };
    }

    #[cfg(not(feature = "thermocouple"))]
    {
        thermo_ok = true;
    }

    (ntc_ok, thermo_ok)
}

/// Called at 3hz to read temperatures and update the UI
fn ui_timer_elapsed(
    count: &mut u16, 
    activity_led: &mut ActivityLedHandle,
    dma_buffer: &mut NtcDmaBufferHandle, 
    last_ntc_temp: &mut f32,
    #[cfg_attr(not(feature = "thermocouple"), allow(unused_variables))]
    max31855: &mut Max31855Handle, 
    #[cfg_attr(not(feature = "thermocouple"), allow(unused_variables))]
    max31855_cs: &mut Max31855CsHandle, 
    last_thermocouple_temp: &mut f32,
    welder_ui: &mut WelderUi, 
    enc_quad: &EncoderHandle, 
    display: &mut DisplayHandle,
) {
    // Turn on the led to show activity
    activity_led.on();

    // Update the NTC temp
    read_ntc_temp(dma_buffer, last_ntc_temp);        

    // Update the thermocouple temp
    #[cfg(feature = "thermocouple")]
    read_thermocouple_temp(max31855, max31855_cs, last_thermocouple_temp);

    // Tick the UI
    tick_ui(welder_ui, enc_quad, display);

    if *count % 20 == 0 {
        #[cfg_attr(not(feature = "thermocouple"), allow(unused_variables))]
        let (ntc_ok, thermocouple_ok) = temps_ok(*last_ntc_temp, *last_thermocouple_temp);
        info!("ntc_temp: {} (ok? {})",
            *last_ntc_temp, 
            ntc_ok, 
        );

        #[cfg(feature = "thermocouple")]
        info!("thermocouple_temperature: {} (ok? {})",
            *last_thermocouple_temp, 
            thermocouple_ok,
        );
    }

    // Turn the led off again
    activity_led.off();
    *count += 1;
}

/// Called when a zero crossing even occurs, typically at 100hz or 120hz depending on mains frequency
/// Performs weld specified by weld_settings if weld_triggered is set
fn zero_crossing_event(
    weld_state: &mut WeldState,
    weld_triggered: &mut bool,
    weld_counter: &mut u16,
    welder_ui: &WelderUi,
    a_delay: &mut u16,
    b_on: &mut u16,
    b_delay: &mut u16,
    zc_period_us: &mut u16,
    ssr_timer: &mut SsrTimerHandle,
    #[cfg_attr(feature = "dummy_zc", allow(unused_variables))]
    zc_in: &ZcHandle,
    #[cfg_attr(feature = "dummy_zc", allow(unused_variables))]
    clocks: &Clocks,
) {
    // Converts microseconds to zero crossing count
    let pulse_us_to_zc = |us: u32, zc_period_us: u16| -> u16 {
        (us as u16 + (zc_period_us / 2)) / zc_period_us
    };

    // Converts number of zero crossings to a pulse period microseconds
    // Adds half a period to make sure we catch the start of the final zero crossing
    let zc_to_pulse_us = |zc: u16, zc_period_us: u16| -> u16 {
        zc * zc_period_us + zc_period_us / 2
    };

    if *weld_state == WeldState::Idle {
        if *weld_triggered {
            let freq;
            #[cfg(feature = "dummy_zc")]
            {
                freq = DUMMY_FREQ.0 as u16;
            }
            #[cfg(not(feature = "dummy_zc"))]
            {
                freq = zc_in
                    .read_frequency(ReadMode::Instant, clocks)
                    .unwrap_or(0.hz()).0 as u16;
            }

            let (_, actual_mains_freq) = ACCEPTABLE_ZC_FREQ_RANGES
                .iter()
                .find(|(range, _)| range.contains(&freq))
                //TODO: Do something nicer than panic once UI is a bit more polished
                .unwrap_or_else(|| panic!("Actual frequency ({}) is outside of acceptable ranges", freq));

            *zc_period_us = 1000 / *actual_mains_freq;

            let snapshot = welder_ui.settings_snapshot();

            *weld_state = WeldState::AWeld;
            *weld_counter = pulse_us_to_zc(snapshot.a_on, *zc_period_us);
            *a_delay = pulse_us_to_zc(snapshot.a_delay, *zc_period_us);
            *b_on = pulse_us_to_zc(snapshot.b_on, *zc_period_us);
            *b_delay = pulse_us_to_zc(snapshot.b_delay, *zc_period_us);
            

            debug!("WELDING. Config ([set] => [actual]):");
            debug!("  a_on: {} => {}",
                snapshot.a_on,
                zc_to_pulse_us(*weld_counter, *zc_period_us),
            );
            debug!("  a_delay: {} => {}",
                snapshot.a_delay,
                zc_to_pulse_us(*a_delay, *zc_period_us),                
            );
            debug!("  b_on: {} => {}", 
                snapshot.b_on,
                zc_to_pulse_us(*b_on, *zc_period_us),                
            );
            debug!("  b_delay: {} => {}",
                snapshot.b_delay,
                zc_to_pulse_us(*b_delay, *zc_period_us),               
            );
            debug!("  mains freq: {}", actual_mains_freq);

            ssr_timer.pulse(zc_to_pulse_us(*weld_counter, *zc_period_us) as u32);
        }
    } else {
        let done = if *weld_counter > 0 {
            *weld_counter -= 1;
            false
        } else {
            true 
        };

        if done {
            match *weld_state {
                WeldState::Idle => unreachable!(),
                WeldState::AWeld => {
                    *weld_counter = *a_delay;
                    *weld_state = WeldState::ADelay;
                },
                WeldState::ADelay => {
                    *weld_counter = *b_on;
                    *weld_state = WeldState::BWeld;
                    ssr_timer.pulse(
                        zc_to_pulse_us(
                            *weld_counter, 
                            *zc_period_us) as u32
                    );
                },
                WeldState::BWeld => {
                    *weld_counter = *b_delay;
                    *weld_state = WeldState::BDelay;
                },
                WeldState::BDelay => {
                    *weld_state = WeldState::Idle;
                    *weld_triggered = false;
                },
            }
        }
    }
}
    

/// Disables the timer, updates the count register, re-enables the timer.
/// Doesn't actually use the parameter because this functionality isn't available on hal interface
fn update_quad_count(_quad: &mut EncoderHandle, count: u16) {
    unsafe {
        let tim = &(*QuadTimer::ptr());
        tim.cr1.modify(|_, w| w.cen().clear_bit());
        tim.cnt.write(|w| w.cnt().bits(count));
        tim.cr1.modify(|_, w| w.cen().set_bit());
    }
}

/// Returns the quad count scaled to the settings range
fn setting_value_from_quad(quad: &EncoderHandle) -> u16 {
    quad.count() * QUAD_SCALE % 999
}

/// Reverses the setting scaling and updates the quad count
fn update_quad_from_setting_value(quad: &mut EncoderHandle, value: u16) {
    update_quad_count(quad, value / QUAD_SCALE);
}

/// Steinhart conversion from NTC sample to degreec celsius
fn ntc_sample_to_degrees(sample: u16) -> f32 {
    let adc_sample = sample as f32;
    let adc_v = VCC_F * adc_sample / ADC_MAX_F;
    let ntc_r = adc_v * NTC_FIXED_RES / (VCC_F - adc_v);

    let mut steinhart = ntc_r;

    steinhart /= NTC_NOMINAL_RES;
    steinhart = steinhart.ln();
    steinhart /= NTC_B_COEFF;
    steinhart += 1. / (NTC_NOMINAL_TEMP + CELCIUS_KELVIN_OFFSET);
    if steinhart == 0. {
        panic!("Div by 0");
    }
    steinhart = 1. / steinhart;
    steinhart -= CELCIUS_KELVIN_OFFSET;

    steinhart
}

#[panic_handler]
fn panic(
    #[cfg_attr(not(feature = "itm"), allow(unused_variables))]
    info: &PanicInfo
) -> ! {
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