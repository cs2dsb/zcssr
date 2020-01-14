#![no_std]
#![no_main]

#![feature(const_fn)]
#![cfg_attr(feature = "bench", feature(test))]

#[allow(unused_imports)]
use cortex_m::{asm, singleton};

use core::{
    panic::PanicInfo,
    sync::atomic::{self, Ordering},
};
use cortex_m::{
    interrupt,
    iprintln,
};

// For ln
use libm::F32Ext;

#[cfg(feature = "itm")] 
use {
    cortex_m::peripheral::ITM,
    itm_logger::*,
};

#[cfg(not(feature = "itm"))] 
use itm_logger::{ info, error, warn, debug };

use zcssr::board::*;

use adafruit_alphanum4::{
    AlphaNum4,
    Index,
    AsciiChar,
};

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
    time::Hertz,
    pwm::{
        C2,
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
        Mode as I2cMode,
        blocking_i2c,
        BlockingI2c,
    },    
    spi::{
        Spi,
        Spi2NoRemap,
        NoMosi,
    },
};

#[cfg(not(feature = "dummy_zc"))]
use stm32f1xx_hal::{
    pwm_input::{
        Configuration as PwmInputConfig,
        ReadMode,
        PwmInput,
    },
};

// If true, panic on failure to configure HT16K33, else just log it
const HT16K33_MISSING_FATAL: bool = false;

// If true, wait (possible forever) for first zero crossing
#[cfg(not(feature = "dummy_zc"))]
const WAIT_FOR_ZC_INIT: bool = false;

#[cfg(feature = "dummy_zc")]
const DUMMY_FREQ: Hertz = Hertz(100);

// If supplied, make sure NTC temperature is in this range before welding
const NTC_RANGE: Option<(f32, f32)> = Some((1., 30.));

// If supplied, make sure thermocouple temperature is in this range before welding
const THERMOCOUPLE_RANGE: Option<(f32, f32)> = Some((1., 30.));

#[derive(Clone, Copy, PartialEq)]
pub enum WeldState {
    Idle,
    AWeld,
    ADelay,
    BWeld,
    BDelay,
}

const MAX_SETTING_CHARS: usize = 32;

#[derive(Debug)]
pub struct Setting {
    pub characters: [AsciiChar; MAX_SETTING_CHARS],
    pub len: usize,
    pub single_char: AsciiChar,
    pub single_char_dot: bool,
    pub value: u16,
}

impl Setting {
    pub fn new(string: &str, single_char: char, single_char_dot: bool, value: u16) -> Self {
        assert!(string.len() <= MAX_SETTING_CHARS);
        let mut characters = [AsciiChar::new(' '); MAX_SETTING_CHARS];
        for (i, c) in string.chars().enumerate() {
            characters[i] = AsciiChar::from_ascii(c).unwrap();
        }
        let single_char = AsciiChar::from_ascii(single_char).unwrap();
        Self {
            characters,
            len: string.len(),
            single_char,
            single_char_dot,
            value,
        }
    }
}

#[derive(Debug)]
pub struct DisplayMessage {
    characters: [AsciiChar; MAX_SETTING_CHARS],
    len: usize,
    pos: usize,
    tick: usize,
    ticks_per_scroll: usize,
    ticks_before_expand: usize,
    single_char: AsciiChar,
    single_char_dot: bool,
    expanded: bool,
    value: f32,
}

impl DisplayMessage {
    const fn blank() -> AsciiChar {
        AsciiChar::new(' ')
    }

    pub const fn default() -> Self {
        Self {
            characters: [Self::blank(); MAX_SETTING_CHARS],
            len: 0,
            pos: 0,
            tick: 0,
            ticks_per_scroll: 2,
            ticks_before_expand: 15,
            single_char: Self::blank(),
            single_char_dot: false,
            expanded: false,
            value: 0.,
        }
    }

    pub fn tick<T, I>(&mut self, display: &mut T)
    where 
        T: AlphaNum4<I>
    {
        if self.expanded {
            for i in 0..4 {
                let p = self.pos + i;
                let ch = if p < self.len { self.characters[p] } else { Self::blank() };
                display.update_buffer_with_char(Index::from(i as u8), ch);
            }
        } else {
            display.update_buffer_with_char(Index::One, self.single_char);
            if self.single_char_dot {
                display.update_buffer_with_dot(Index::One, true);
            }
            //TODO: return result
            display.update_buffer_with_float(Index::Two, self.value, 0, 10).unwrap();
        }

        self.tick += 1;
        if self.expanded && self.tick >= self.ticks_per_scroll {
            self.tick = 0;
            if self.pos >= self.len {
                self.pos = 0;
                self.expanded = false;
            } else {
                self.pos += 1;
            }
        } else if !self.expanded && self.tick >= self.ticks_before_expand {
            self.tick = 0;
            self.expanded = true;
        }
    }

    pub fn set_text(&mut self, text: [AsciiChar; MAX_SETTING_CHARS], len: usize, char: AsciiChar, dot: bool) {
        assert!(len < MAX_SETTING_CHARS);
        self.characters = text;
        self.len = len;
        self.single_char = char;
        self.single_char_dot = dot;
    }

    pub fn set_value(&mut self, value: f32) {
        if self.value != value {
            self.value = value;
            self.tick = 0;
            self.expanded = false;
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum WeldSetting {
    PulseAOn,
    PulseADelay,
    PulseBOn,
    PulseBDelay,
}

impl Default for WeldSetting {
    fn default() -> Self {
        WeldSetting::PulseAOn
    }
}

impl WeldSetting {
    pub fn next(self) -> Self {
        match self {
            WeldSetting::PulseAOn => WeldSetting::PulseADelay,
            WeldSetting::PulseADelay => WeldSetting::PulseBOn,
            WeldSetting::PulseBOn => WeldSetting::PulseBDelay,
            WeldSetting::PulseBDelay => WeldSetting::PulseAOn,
        }
    }
}

#[derive(Debug)]
pub struct WeldSettings {
    pulse_a_on: Setting,
    pulse_a_delay: Setting,
    pulse_b_on: Setting,
    pulse_b_delay: Setting,
    current_setting: WeldSetting,
    display_message: DisplayMessage,
}

impl WeldSettings {
    pub fn new() -> Self {
        let mut self_ = Self {
            pulse_a_on: Setting::new("A-PULSE ON TIME", 'A', false, 200),
            pulse_a_delay: Setting::new("A DELAY TIME", 'A', true, 300),
            pulse_b_on: Setting::new("B-PULSE ON TIME", 'B', false, 100),
            pulse_b_delay: Setting::new("B DELAY TIME", 'B', true, 400),
            current_setting: WeldSetting::default(),
            display_message: DisplayMessage::default(),
        };
        self_.update_message();
        self_
    }

    fn current_setting(&self) -> &Setting {
        match self.current_setting {
            WeldSetting::PulseAOn => &self.pulse_a_on,
            WeldSetting::PulseADelay => &self.pulse_a_delay,
            WeldSetting::PulseBOn => &self.pulse_b_on,
            WeldSetting::PulseBDelay => &self.pulse_b_delay,
        }
    }

    fn current_setting_mut(&mut self) -> &mut Setting {
        match self.current_setting {
            WeldSetting::PulseAOn => &mut self.pulse_a_on,
            WeldSetting::PulseADelay => &mut self.pulse_a_delay,
            WeldSetting::PulseBOn => &mut self.pulse_b_on,
            WeldSetting::PulseBDelay => &mut self.pulse_b_delay,
        }
    }

    fn update_message(&mut self) {
        let (text, len, sc, dot, value) = {
            let cs = self.current_setting();
            (cs.characters.clone(), cs.len, cs.single_char, cs.single_char_dot, cs.value)
        };
        self.display_message.set_text(text, len, sc, dot);
        self.display_message.set_value(value as f32);
    }

    pub fn set_value(&mut self, value: u16) {
        let mut cs = self.current_setting_mut();
        if cs.value != value {
            cs.value = value;
            let valuef = value as f32;
            self.display_message.set_value(valuef);
        }
    }

    pub fn tick<T, I>(&mut self, display: &mut T)
    where 
        T: AlphaNum4<I>
    {
        self.display_message.tick(display);
    }

    pub fn next(&mut self) {
        self.current_setting = self.current_setting.next();
        self.update_message();
    }

    pub fn value(&self) -> u16 {
        self.current_setting().value
    }

    pub fn update_from_quad(&mut self, quad: &EncoderHandle) {
        self.set_value(quad.count() % 99 * 10);
    }

    pub fn update_quad_from_value(&self, quad: &mut EncoderHandle) {
        update_quad_count(quad, self.value() / 10);
    }
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
type SsrTimerHandle = Pwm<SsrTimer, C2>;
#[cfg(not(feature = "dummy_zc"))] 
type ZcHandle = PwmInput<ZcTimer, Tim2NoRemap, (ZcRising, ZcFalling)>;
#[cfg(feature = "dummy_zc")] 
type ZcHandle = CountDownTimer<ZcTimer>;

#[rtfm::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        ui_timer: UiTimerHandle,
        enc_quad: EncoderHandle,
        zc_in: ZcHandle,
        clocks: Clocks,
        ntc_dma_buffer: NtcDmaBufferHandle,
        display: DisplayHandle,
        weld_settings: WeldSettings,
        max31855k: Max31855Handle,
        max31855k_cs: KThermoNss,
        activity_led: ActivityLedHandle,
        ssr_timer: SsrTimerHandle,
        enc_button: EncBtn,
        trigger_button: Trigger,

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

            info!("ITM reset");
        }

        // Configure the clock for full speed
        let mut rcc = device
            .RCC
            .constrain();
        let mut flash = device
            .FLASH
            .constrain();
        let clocks = rcc.cfgr
            .use_hse(HSE)
            .sysclk(SYSCLK_FREQ)
            .pclk1(PCLK1_FREQ)
            .pclk2(PCLK2_FREQ)
            .adcclk(ADC_FREQ)
            .freeze(&mut flash.acr);


        #[cfg(feature = "itm")]
        {
            // Update the baud rate for the new core clock
            let sysclk: Hertz = clocks.sysclk().into();
            let baud: Hertz = ITM_BAUDRATE.into();
            update_tpiu_baudrate(sysclk.0, baud.0).expect("Failed to reset TPIU baudrate");     
        }

        info!("Configured max freq");

        // BlockingI2c relies upon DWT being running
        core.DWT.enable_cycle_counter();

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        //let gpioc = device.GPIOC.split(&mut rcc.apb2);
        let adc1 = Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
        let dma1 = device.DMA1.split(&mut rcc.ahb);

        // Initialize the I2C display
        // Extra logging is because this tends to fail/hang so it's nice to see what was happening in the log
        //TODO: this is "timing out" fairly frequently since stm32f1-hal/pull/#150 was merged. Increasing the timeout and retries 
        //      doesn't work so suspect there's a logic bug in there somewhere
        debug!("Initializing I2C display...");
        let scl_pin: DispScl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
        let sda_pin: DispSda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
        let i2c2 = blocking_i2c(
            I2c::i2c2(
                device.I2C2, 
                (scl_pin, sda_pin),
                I2cMode::Standard {
                    frequency: DISP_BAUDRATE.into(),
                },
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
        ht16k33.initialize().unwrap();
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


        // User led (red)
        let mut usr_led = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);
        usr_led.set_high().unwrap();
        
        
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
        led_status.set_high().unwrap();


        // SSR status LED
        let mut led_ssr: LedSsrStatus = gpioa.pa10.into_push_pull_output(&mut gpioa.crh);
        led_ssr.set_high().unwrap();   


        // Misc en
        let mut misc_en: MiscEn = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);
        misc_en.set_high().unwrap();


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
        let mut ssr_timer = Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2)
            .pwm(
                ssr_en,
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
                .cc2ie().enabled()
            );
            tim.ccmr1_output().modify(|_, w| w
                // Mode 2 is idle low, hal sets it to mode 1
                .oc2m().pwm_mode2() 
            );
        }


        // UI Timer
        let mut ui_timer = Timer::tim8(device.TIM8, &clocks, &mut rcc.apb2)
            .start_count_down(3.hz());
        ui_timer.listen(Event::Update);

        // WeldSettings
        let weld_settings = WeldSettings::new();

        // Update the quad counter to match the current setting
        weld_settings.update_quad_from_value(&mut quad_input);

        info!("Init done");

        init::LateResources {
            ui_timer: ui_timer,
            enc_quad: quad_input,
            zc_in: zc_in,
            clocks: clocks,
            ntc_dma_buffer: ntc_circ_buffer,
            display: ht16k33,
            weld_settings: weld_settings,
            max31855k: max31855,
            max31855k_cs: max31855_cs_pin,
            activity_led: usr_led,
            ssr_timer: ssr_timer,
            enc_button: enc_button,
            trigger_button: trigger_button,
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
        weld_settings,
        last_ntc_temp,
        last_thermocouple_temp,
        weld_triggered,
        enc_button,
        trigger_button,
    ])]
    fn exti9_5(cx: exti9_5::Context) {        
        if cx.resources.enc_button.check_interrupt() {
            cx.resources.enc_button.clear_interrupt_pending_bit();
            debug!("EXTI ENC");
            encoder_button_pressed(cx.resources.weld_settings, cx.resources.enc_quad);
        } 

        if cx.resources.trigger_button.check_interrupt() {
            cx.resources.trigger_button.clear_interrupt_pending_bit();
            debug!("EXTI TRIGGER");
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
        weld_settings,
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
            cx.resources.weld_settings, 
            cx.resources.enc_quad, 
            cx.resources.display,
        );
    }

    /// TIM2 Interrupt
    /// This timer is connected to the ZC circuit (or countdown if dummy_zc feature is enabled)
    #[task(binds = TIM2, resources = [
        weld_triggered,
        weld_settings,
        ssr_timer,
        zc_in,
        clocks,
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

        zero_crossing_event(
            WELD_STATE,
            cx.resources.weld_triggered,
            WELD_COUNTER,
            cx.resources.weld_settings,
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
    
        info!("Pulsing: {}us, duty: {}", desired_us, duty);
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
fn encoder_button_pressed(weld_settings: &mut WeldSettings, enc_quad: &mut EncoderHandle) {
    weld_settings.next();
    weld_settings.update_quad_from_value(enc_quad);
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
fn read_thermocouple_temp(max31855: &mut Max31855Handle, max31855_cs: &mut Max31855CsHandle, last_thermocouple_temp: &mut f32) {
    match max31855.read_thermocouple(max31855_cs, Unit::Celsius) {
        Ok(v) => *last_thermocouple_temp = v,
        Err(e) => warn!("Error reading thermocouple: {:?}", e),
    };
}

/// Called at 3hz to update the UI from the quad reading & due to elapsed time
fn tick_ui(weld_settings: &mut WeldSettings, enc_quad: &EncoderHandle, display: &mut DisplayHandle) {
    weld_settings.update_from_quad(enc_quad);
    weld_settings.tick(display);
    if let Err(e) = display.write_display_buffer() {
        if HT16K33_MISSING_FATAL {
            panic!("HT16K33 error: {:?}", e);
        }
    }
}

/// Checks the temperatures are in acceptable range if range is specified
fn temps_ok(ntc_temp: f32, thermocouple_temp: f32) -> (bool, bool) {
    let ntc_ok = if let Some((min, max)) = NTC_RANGE {
        ntc_temp >= min && ntc_temp <= max
    } else {
        true
    };
    let thermo_ok = if let Some((min, max)) = THERMOCOUPLE_RANGE {
        thermocouple_temp >= min && thermocouple_temp <= max
    } else {
        true
    };
    (ntc_ok, thermo_ok)
}

/// Called at 3hz to read temperatures and update the UI
fn ui_timer_elapsed(
    count: &mut u16, 
    activity_led: &mut ActivityLedHandle,
    dma_buffer: &mut NtcDmaBufferHandle, 
    last_ntc_temp: &mut f32,
    max31855: &mut Max31855Handle, 
    max31855_cs: &mut Max31855CsHandle, 
    last_thermocouple_temp: &mut f32,
    weld_settings: &mut WeldSettings, 
    enc_quad: &EncoderHandle, 
    display: &mut DisplayHandle,
) {
    // Turn on the led to show activity
    activity_led.set_low().unwrap();

    // Update the NTC temp
    read_ntc_temp(dma_buffer, last_ntc_temp);        

    // Update the thermocouple temp
    read_thermocouple_temp(max31855, max31855_cs, last_thermocouple_temp);

    // Tick the UI
    tick_ui(weld_settings, enc_quad, display);

    if *count % 20 == 0 {
        let (ntc_ok, thermocouple_ok) = temps_ok(*last_ntc_temp, *last_thermocouple_temp);
        info!("ntc_temp: {} (ok? {}), thermocouple_temperature: {} (ok? {})", 
            *last_ntc_temp, 
            ntc_ok, 
            *last_thermocouple_temp, 
            thermocouple_ok,
        );
    }

    // Turn the led off again
    activity_led.set_high().unwrap();
    *count += 1;
}

/// Called when a zero crossing even occurs, typically at 100hz or 120hz depending on mains frequency
/// Performs weld specified by weld_settings if weld_triggered is set
fn zero_crossing_event(
    weld_state: &mut WeldState,
    weld_triggered: &mut bool,
    weld_counter: &mut u16,
    weld_settings: &WeldSettings,
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
    // Converts the weld counter from microseconds to zc events
    //TODO: fix this smell (counter is in zc events but pulses are in microseconds)
    let convert_weld_counter = |weld_counter: &mut u16, zc_period_us: &mut u16| {
        *weld_counter = (*weld_counter + (*zc_period_us / 2)) / *zc_period_us;
    };

    if *weld_state == WeldState::Idle {
        if *weld_triggered {
            let freq;
            #[cfg(feature = "dummy_zc")]
            {
                freq = DUMMY_FREQ.0;
            }
            #[cfg(not(feature = "dummy_zc"))]
            {
                freq = zc_in
                    .read_frequency(ReadMode::Instant, clocks)
                    .unwrap_or(0.hz()).0;
            }

            *zc_period_us = 1000 / freq as u16;

            *weld_state = WeldState::AWeld;
            *weld_counter = weld_settings.pulse_a_on.value;
            *a_delay = weld_settings.pulse_a_delay.value;
            *b_on = weld_settings.pulse_b_on.value;
            *b_delay = weld_settings.pulse_b_delay.value;
            
            info!("Configure OPM, start. ({}, {}, {}, {})", *weld_counter, *a_delay, *b_on, *b_delay);

            ssr_timer.pulse(*weld_counter as u32);
            convert_weld_counter(weld_counter, zc_period_us);

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
                    info!("AOn done. Waiting: {}", *a_delay);
                },
                WeldState::ADelay => {
                    *weld_counter = *b_on;
                    *weld_state = WeldState::BWeld;
                    ssr_timer.pulse(*weld_counter as u32);
                    info!("ADelay done. Configure OPM(B). Welding: {}", *b_on);
                },
                WeldState::BWeld => {
                    *weld_counter = *b_delay;
                    *weld_state = WeldState::BDelay;
                    info!("BOn done. Waiting: {}", *b_delay);
                },
                WeldState::BDelay => {
                    *weld_state = WeldState::Idle;
                    *weld_triggered = false;
                    info!("BDelay done. Returning to idle");
                },
            }
            convert_weld_counter(weld_counter, zc_period_us);
        }
    }
}
    

/// Disables the timer, updates the count register, re-enables the timer.
/// Doesn't actually use the parameter because this functionality isn't available on hal interface
fn update_quad_count(_quad: &mut EncoderHandle, count: u16) {
    unsafe {
        let tim = &(*TIM4::ptr());
        tim.cr1.modify(|_, w| w.cen().clear_bit());
        tim.cnt.write(|w| w.cnt().bits(count));
        tim.cr1.modify(|_, w| w.cen().set_bit());
    }
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