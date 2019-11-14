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

//For ln
use libm::F32Ext;

#[cfg(feature = "itm")] use cortex_m::peripheral::{
    ITM,
};
//#[cfg(feature = "itm")] use itm_packets::*;
#[cfg(feature = "itm")] use itm_logger::*;
#[cfg(not(feature = "itm"))] use itm_logger::{ info };

use zcssr::board::*;

use adafruit_alphanum4::{
    AlphaNum4,
    Index,
    AsciiChar,
};

/*
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;
*/

use ht16k33::{
    HT16K33,
    Display,
};

//Used for interrupt vectors if nothing else
#[allow(unused_imports)]
use stm32f1xx_hal;

use stm32f1xx_hal::{
    prelude::*,
    rcc::Clocks,
    stm32::{
        TIM1,
        TIM2 as TIM2_REG,
        TIM4,
        EXTI,
        TIM3,
        TIM8,
        DMA1,
        I2C2,
    },
    gpio::{
        PushPull,
        Alternate,
        gpioc::{
            PC7,
            PC9,
        },
    },    
    timer::*,
    delay::Delay,
    time::Hertz,
    pwm::{
        C2,
        C4,
        Pwm,
        Pins as PwmOutPins,
    },
    pwm_input::{
        Configuration as PwmInputConfig,
        //ReadMode,
        PwmInput,
    },
    qei::Qei,
    adc::{
        Adc,
        AdcPayload,
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
};

//Represents both the welding config and the current welding state
//One copy is accessible to UI for config
//Another copy is accessible only to ZC interrupt for tracking the output state
#[derive(Clone, Copy)]
pub struct OutputConfig {
    /*
    //Triggers a weld if one is not alredy in progress
    enable: bool,

    //How many zero crossings for each step
    //Initial pulse (on)
    pulse_one_count: u8,
    //Delay after initial pulse (off)
    delay_one_count: u8,
    //Second pulse (on)
    pulse_two_count: u8,
    //Delay after second pulse (off)
    delay_two_count: u8,
    */
}

impl OutputConfig {
    pub const fn default() -> Self {
        Self {
            /*
            enable: false,
            pulse_one_count: 10,
            delay_one_count: 20,
            pulse_two_count: 15,
            delay_two_count: 100,
            */
        }
    }
}

/*
struct LedUsrChannels(PA7<Alternate<PushPull>>, PC8<Alternate<PushPull>>);
impl PwmOutPins<TIM3>  for LedUsrChannels {
    const REMAP: u8 = 0b00; 
    const C1: bool = false;
    const C2: bool = true;  // use channel C2
    const C3: bool = true;
    const C4: bool = false;
    type Channels = (Pwm<TIM3, C2>, Pwm<TIM3, C3>);
}

struct LedStatusChannels(PC8<Alternate<PushPull>>);
impl PwmOutPins<TIM8>  for LedStatusChannels {
    const REMAP: u8 = 0b00; 
    const C1: bool = false;
    const C2: bool = false; 
    const C3: bool = true; // use channel C3
    const C4: bool = false;
    type Channels = Pwm<TIM8, C3>;
}
*/

struct LedSsrStatusChannels(PC7<Alternate<PushPull>>, PC9<Alternate<PushPull>>);
impl PwmOutPins<TIM3> for LedSsrStatusChannels {
    const REMAP: u8 = 0b11;
    const C1: bool = false;
    const C2: bool = true;
    const C3: bool = false;
    const C4: bool = true;
    type Channels = (Pwm<TIM3, C2>, Pwm<TIM3, C4>);
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
            pulse_a_on: Setting::new("A-PULSE ON TIME", 'A', false, 10),
            pulse_a_delay: Setting::new("A DELAY TIME", 'A', true, 20),
            pulse_b_on: Setting::new("B-PULSE ON TIME", 'B', false, 30),
            pulse_b_delay: Setting::new("B DELAY TIME", 'B', true, 40),
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

    pub fn update_from_quad(&mut self, quad: &Qei<TIM4,(EncA, EncB)>) {
        self.set_value(quad.count() % 99 * 10);
    }

    pub fn update_quad_from_value(&self, quad: &mut Qei<TIM4,(EncA, EncB)>) {
        update_quad_count(quad, self.value() / 10);
    }
}

#[rtfm::app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    static mut EXTI_HANDLE: EXTI = ();
    static mut TIM1_HANDLE: CountDownTimer<TIM1> = ();
    static mut ENC_QUAD_HANDLE: Qei<TIM4,(EncA, EncB)> = ();
    static mut ZC_IN_HANDLE: PwmInput<TIM2_REG, (ZcRising, ZcFalling)> = ();
    static mut CLOCKS: Clocks = ();
    //static mut STATUS_LED_HANDLE: Pwm<TIM8, C3> = ();
    /// Config accessible to UI 
    static mut OUTPUT_CONFIG: OutputConfig = OutputConfig::default();
    /// Used for actual output control by ZC interrupt only
    static mut OUTPUT_STATE: OutputConfig = OutputConfig::default();
    static mut NTC_DMA_BUFFER: CircBuffer<[u16; 8], RxDma<AdcPayload<Ntc>, DmaC1>> = ();
    static mut DISPLAY: HT16K33<BlockingI2c<I2C2, (DispScl, DispSda)>> = ();
    //static mut SSR_EN_HANDLE: SSR_EN = ();
    //static mut NTC_TEST_OUT: PB1<Output<PushPull>> = ();
    //static mut TIM3_HANDLE: TIM3 = ();
    static mut WELD_SETTINGS: WeldSettings = ();
    #[init]
    fn init() -> init::LateResources {
        #[cfg(feature = "itm")]
        {
            //After reset the clock is set to HSI and the TPIU clock scaler doesn't get reset (unless openocd relaunches or gdb is set to do this)
            let hsi: Hertz = HSI.into();
            let baud: Hertz = ITM_BAUDRATE.into();
            update_tpiu_baudrate(hsi.0, baud.0).expect("Failed to reset TPIU baudrate");
            logger_init();

            /*
            unsafe {
                (*ITM::ptr()).tcr.modify(|w|
                    //Enable local timestamps
                    w | (1 << 1)
                );
            }
            */

            info!("ITM reset");
        }

        let flash_size = FlashSize::get();
        info!("FlashSize: {}", flash_size.kilo_bytes());


        //Configure the clock for full speed
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
            let sysclk: Hertz = clocks.sysclk().into();
            let baud: Hertz = ITM_BAUDRATE.into();
            update_tpiu_baudrate(sysclk.0, baud.0).expect("Failed to reset TPIU baudrate");     
        }

        info!("Configured max freq");

        //BlockingI2c relies upon DWT being running
        core.DWT.enable_cycle_counter();

        let _delay = Delay::new(core.SYST, clocks);
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut dbg = device.DBGMCU;
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
        let adc1 = Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
        let dma1 = device.DMA1.split(&mut rcc.ahb);

        // I2C disp
        let freq: Hertz = 20.khz().into();

        let i2c2 = blocking_i2c(
            I2c::i2c2(
                device.I2C2, 
                (
                    gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh),
                    gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh),
                ),
                I2cMode::Standard {
                    frequency: freq.0,
                },
                clocks,
                &mut rcc.apb1,
            ),
            clocks,
            500,
            2,
            500,
            500,
        );

        const DISP_I2C_ADDR: u8 = 112;

        // I2C display
        let mut ht16k33 = HT16K33::new(i2c2, DISP_I2C_ADDR);
        ht16k33.initialize().expect("Failed to initialize ht16k33");
        ht16k33.set_display(Display::ON).unwrap();
        ht16k33.write_display_buffer().unwrap();
        // -I2C display

        // NTC adc
        let ntc_dma = adc1.with_dma(
            gpiob.pb1.into_analog(&mut gpiob.crl),
            dma1.1,
        );
        let ntc_buffer = singleton!(: [[u16; 8]; 2] = [[0; 8]; 2]).unwrap();
        let ntc_circ_buffer = ntc_dma.circ_read(ntc_buffer);
        // - NTC adc

        // Rotary encoder quadrature input
        let mut quad_input = Timer::tim4(device.TIM4, &clocks, &mut rcc.apb1)
            .qei((
                gpiob.pb6.into_floating_input(&mut gpiob.crl), //Encoder A input
                gpiob.pb7.into_floating_input(&mut gpiob.crl), //Encoder B input
            ), &mut afio.mapr);
        // - quad

        // Debug LED       
        /*
        let mut pwm = Timer::tim3(device.TIM3, &clocks, &mut rcc.apb1)
            .pwm(
                LedUsrChannels(
                    gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
                    gpioc.pc8.into_alternate_push_pull(&mut gpioc.crh),
                ), 
                &mut afio.mapr, 
                1.khz(),
            );
        pwm.1.enable();
        pwm.1.set_duty(pwm.1.get_max_duty() / 2);
        */
        // - LED

        // Status LED
        /*
        let mut status_led = Timer::tim8(device.TIM8, &clocks, &mut rcc.apb2)
            .pwm(
                LedStatusChannels(gpioc.pc8.into_alternate_push_pull(&mut gpioc.crh)),
                &mut afio.mapr,
                1.khz(),
            );
        status_led.enable();
        status_led.set_duty(status_led.get_max_duty() / 2);
        unsafe {
            &(*TIM8::ptr()).dier.write(|w| w.cc3ie().set_bit());
        }
        */
        // - status led

        // SSR Status LED
        let mut ssr_status_led = Timer::tim3(device.TIM3, &clocks, &mut rcc.apb1)
            .pwm(
                LedSsrStatusChannels(
                    gpioc.pc7.into_alternate_push_pull(&mut gpioc.crl),
                    gpioc.pc9.into_alternate_push_pull(&mut gpioc.crh),
                ),
                &mut afio.mapr,
                10.hz(),
            );
        ssr_status_led.0.set_duty(ssr_status_led.0.get_max_duty() / 2);
        ssr_status_led.1.set_duty(ssr_status_led.1.get_max_duty() / 2);
        unsafe {
            let tim = &(*TIM3::ptr());
            tim.cr1.modify(|_, w|
                w.opm().enabled()
            );
            tim.ccmr1_output()
                .modify(|_, w| 
                    w
                        .oc2pe().set_bit()
                        .oc2m().pwm_mode2() 
                );
        }
        ssr_status_led.0.enable();
        //ssr_status_led.1.enable();

        // - SSR Status LED

        // SSR en
        //let ssr_en = gpioc.pc9.into_push_pull_output(&mut gpioc.crh);
        // - ssr en

        // ZC input
        let zc_in = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1)
            .pwm_input(
                (
                    gpioa.pa0.into_floating_input(&mut gpioa.crl), //Rising
                    gpioa.pa1.into_floating_input(&mut gpioa.crl), //Falling
                ), 
                &mut afio.mapr, 
                &mut dbg, 
                PwmInputConfig::Frequency(300.hz()),
            );

        unsafe {
            let tim = &(*TIM2_REG::ptr());
            //info!("{:b}", tim.sr.read().bits());
            tim.dier.reset();
            tim.dier.modify(|_, w|
                w.cc2ie().set_bit()
            );
            //info!("{:b}", tim.dier.read().bits());
        }

        info!("Waiting for ZC timer to stabilize");
        //while zc_in.read_frequency(ReadMode::Instant, &clocks).is_err() {}
        // -zc input

        // Rotary encoder button input
        let _rot_enc_btn: EncBtn = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        let exti = device.EXTI;
        unsafe {
            //Set EXTI5 to port B
            afio.exticr2.exticr2().modify(|_, w| w.exti5().bits(0b0001));
            //Enable falling edge trigger
            exti.ftsr.modify(|_, w| w.tr5().set_bit());        
            //Enable exti 5 interrupt
            exti.imr.modify(|_, w| w.mr5().set_bit());
        }
        // - rot enc button
        
        let mut tim1 = Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2)
            .start_count_down(3.hz());
        tim1.listen(Event::Update);

        let mut tim8 = Timer::tim8(device.TIM8, &clocks, &mut rcc.apb2)
            .start_count_down(1.hz());
        tim8.listen(Event::Update);

        /*let tim3 = Timer::tim3(device.TIM3, &clocks, &mut rcc.apb1)
            .start_count_down(1.hz())
            .release();*/

        let weld_settings = WeldSettings::new();
        //Update the quad counter to match the current setting
        weld_settings.update_quad_from_value(&mut quad_input);

        info!("WELD_SETTINGS: {:#?}", weld_settings);

        info!("Init done");

        init::LateResources {
            EXTI_HANDLE: exti,
            TIM1_HANDLE: tim1,
            ENC_QUAD_HANDLE: quad_input,
            ZC_IN_HANDLE: zc_in,
            CLOCKS: clocks,
            //STATUS_LED_HANDLE: status_led,
            NTC_DMA_BUFFER: ntc_circ_buffer,
            DISPLAY: ht16k33,
            //SSR_EN_HANDLE: ssr_en,
            //NTC_TEST_OUT: gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
            //TIM3_HANDLE: tim3,
            WELD_SETTINGS: weld_settings,
        }
    }

    #[idle]
    fn idle() -> ! {
        loop {
            //info!("idle");
            asm::wfi();
        }
    }

    #[interrupt(resources = [EXTI_HANDLE, ENC_QUAD_HANDLE, WELD_SETTINGS])]
    fn EXTI9_5() {
        //Clear interrupt
        resources.EXTI_HANDLE.pr.write(|w| w.pr5().set_bit());

        resources.WELD_SETTINGS.next();
        resources.WELD_SETTINGS.update_quad_from_value(&mut resources.ENC_QUAD_HANDLE);
    }

    #[interrupt(resources = [
        TIM1_HANDLE, 
        ZC_IN_HANDLE, 
        CLOCKS, 
        DISPLAY, 
        NTC_DMA_BUFFER, 
        ENC_QUAD_HANDLE,
        WELD_SETTINGS,
    ])]
    fn TIM1_UP() {
        static mut COUNT: u32 = 0;
        *COUNT += 1;

        //Clear interrupt
        resources.TIM1_HANDLE.clear_update_interrupt_flag();

        /*
        let freq = resources.ZC_IN_HANDLE
            .read_frequency(ReadMode::Instant, &resources.CLOCKS)
            .unwrap_or(0.hz());

        let duty = resources.ZC_IN_HANDLE
            .read_duty(ReadMode::Instant)
            .unwrap_or((0, 0));
        */

        if *COUNT % 6 == 0 {
            unsafe {
                &(*DMA1::ptr())
                    .ifcr.write(|w|
                        w.chtif1().set_bit()
                    );
            }
            let ntc_sample = resources.NTC_DMA_BUFFER
                .peek(|half, _| {
                    half[0]
                }).unwrap_or(0);
            let temperature = ntc_sample_to_degrees(ntc_sample);
            info!("ntc_temp: {:?}", temperature);
        }

        resources.WELD_SETTINGS.update_from_quad(&resources.ENC_QUAD_HANDLE);
        resources.WELD_SETTINGS.tick(&mut *resources.DISPLAY);
        resources.DISPLAY.write_display_buffer().unwrap();
    }

    #[interrupt]
    fn TIM8_UP() {
        //info!("TIM8_UP interrupt");
        unsafe {
            &(*TIM8::ptr()).sr.modify(|_, w| w.uif().clear_bit());
        }
        unsafe {
            // This is for one pulse mode testing, associated LED is blinking briefly once per second
            let tim = &(*TIM3::ptr());
            tim.cr1.modify(|_, w| w
                .cen().set_bit()
            );
        }
    }

    #[interrupt(resources = [OUTPUT_CONFIG])]
    fn TIM2() {
        static mut COUNT: u16 = 0;

        unsafe {
            &(*TIM2_REG::ptr()).sr.modify(|_, w| w
                .cc1if().clear_bit()
                .cc2if().clear_bit()
            );
        }

        *COUNT += 1;
        let cm = *COUNT % 100;
        if cm == 0 {
            info!("TIM2: {}", *COUNT);
        }
        /*
        if cm < 5 {
            resources.SSR_EN_HANDLE.set_high();
        } else {
            resources.SSR_EN_HANDLE.set_low();
        }*/

        /*
        if *COUNT % 2 == 0 {
            resources.NTC_TEST_OUT.set_high();
        } else {
            resources.NTC_TEST_OUT.set_low();
        }
        */
    }
};

/// Disables the timer, updates the count register, re-enables the timer.
/// Doesn't actually use the parameter because this functionality isn't available on hal interface
fn update_quad_count(_quad: &mut Qei<TIM4,(EncA, EncB)>, count: u16) {
    unsafe {
        let tim = &(*TIM4::ptr());
        tim.cr1.modify(|_, w| w.cen().clear_bit());
        tim.cnt.write(|w| w.cnt().bits(count));
        tim.cr1.modify(|_, w| w.cen().set_bit());
    }
}

#[allow(dead_code)]
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