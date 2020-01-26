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
    peripheral::Peripherals as CorePeripherals,
};

// For ln
use libm::F32Ext;

#[cfg(feature = "itm")] 
use cortex_m::{iprintln, peripheral::ITM};
    
use itm_logger::*;

use zcssr::{
    board::*,
    ui::welder::WelderUi,
};

#[cfg_attr(not(feature = "thermocouple"), allow(unused_imports))]
use max31855::{
    Max31855,
    Unit,
};

use stm32f1xx_hal::{
    prelude::*,
    rcc::Clocks,
    pac::{
        DMA1,
        Peripherals as DevicePeripherals,
    },
    gpio::{
        ExtiPin,
    },    
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
    delay::Delay,
};

#[cfg(not(feature = "dummy_zc"))]
use stm32f1xx_hal::{
    pwm_input::{
        ReadMode,
    },
};

#[derive(Clone, Copy, PartialEq)]
pub enum WeldState {
    Idle,
    AWeld,
    ADelay,
    BWeld,
    BDelay,
}

type NtcDmaBufferHandle = CircBuffer<[u16; 8], RxDma<AdcPayload<Ntc, Continuous>, DmaC1>>;

#[rtfm::app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        ui_timer: UiTimer,
        encoder: Encoder,
        zc_in: Zc,
        clocks: Clocks,
        ntc_dma_buffer: NtcDmaBufferHandle,
        display: Display,
        thermocouple: Thermocouple,
        activity_led: LedUsr,
        ssr_timer: Ssr,
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
        configure(cx.device, cx.core).unwrap()
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::wfi();
        }
    }

    /// EXTI9_5 Interrupt handles the quad encoder button and the trigger button
    #[task(binds = EXTI9_5, resources = [
        encoder, 
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
            encoder_button_pressed(cx.resources.welder_ui, cx.resources.encoder, cx.resources.display);
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
        encoder,
        welder_ui,
        activity_led,
        thermocouple,
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
            cx.resources.thermocouple, 
            cx.resources.last_thermocouple_temp,
            cx.resources.welder_ui, 
            cx.resources.encoder, 
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

        // Clear the interrupt
        cx.resources.zc_in.clear_interrupt();

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

/// This configures all peripherals. Mainly broken out to a function to allow the use of `?`
fn configure(mut device: DevicePeripherals, mut core: CorePeripherals) -> Result<init::LateResources, Error>{
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
    let mut usr_led = LedUsr::configure((gpioa.pa7, &mut gpioa.crl))?;
    usr_led.on();

    // After reset the clock is set to HSI and the TPIU clock scaler doesn't get reset 
    // (unless openocd relaunches or gdb is set to do this). Probably not necessary 
    // since we're immediatly configuring the clock and updating the itm again but it has
    // been useful in the past for debugging clock config code and doesn't do any harm
    itm_reset();

    // Configure the clock for full speed
    let clocks = Clocks::configure((rcc.cfgr, &mut flash.acr))?;

    itm_update_clocks(&clocks);

    info!("Configured max freq");

    let adc1 = Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
    let mut delay = Delay::new(core.SYST, clocks);

    let ht16k33 = {
        let res = Display::configure((
            &mut core.DWT,
            gpiob.pb10, //SCL
            gpiob.pb11, //SDA
            &mut gpiob.crh,
            &mut delay,
            device.I2C2, // I2C peripheral
            clocks,
            &mut rcc.apb1,
        ));
        // This is just for extra logging to quickly flag that it's the display that's failed (I2C is a bit flakey right now)
        if res.is_err() {
            error!("Configuring display failed");
        }
        res?
    };

    let thermocouple = Thermocouple::configure((
        KThermoNss::configure((gpiob.pb12, &mut gpiob.crh))?,
        KThermoSck::configure((gpiob.pb13, &mut gpiob.crh))?,
        KThermoMiso::configure((gpiob.pb14, &mut gpiob.crh))?,
        device.SPI2,
        clocks,
        &mut rcc.apb1,
    ))?;

    let ntc = Ntc::configure((gpiob.pb1, &mut gpiob.crl))?;
    //TODO: Configure::configure for these?
    // Configure the ADC and DMA for reading the NTC thermistor
    let ntc_dma = adc1.with_dma(
        ntc,
        dma1.1,
    );
    let ntc_buffer = singleton!(: [[u16; 8]; 2] = [[0; 8]; 2]).unwrap();
    let ntc_circ_buffer = ntc_dma.circ_read(ntc_buffer);

    let mut encoder = Encoder::configure((
        EncA::configure((gpiob.pb6, &mut gpiob.crl))?, 
        EncB::configure((gpiob.pb7, &mut gpiob.crl))?,
        device.TIM4, 
        clocks,
        &mut rcc.apb1,
        &mut afio.mapr,
    ))?;

    let enc_button = EncBtn::configure((
        gpiob.pb5,
        &mut gpiob.crl,
        &mut device.EXTI,
        &mut afio,
    ))?;

    let trigger_button = Trigger::configure((
        gpioa.pa8,
        &mut gpioa.crh,
        &mut device.EXTI,
        &mut afio,
    ))?;

    let led_status = LedStatus::configure((gpioa.pa2, &mut gpioa.crl))?;

    let _misc_en = MiscEn::configure((gpiob.pb15, &mut gpiob.crh))?;

    let zc_in = Zc::configure((
        ZcRising::configure((gpioa.pa0, &mut gpioa.crl))?,
        ZcFalling::configure((gpioa.pa1, &mut gpioa.crl))?,
        device.TIM2,
        clocks,
        &mut rcc.apb1,
        &mut afio.mapr,
        &mut device.DBGMCU,
    ))?;

    let ssr_timer = Ssr::configure((
        SsrEn::configure((gpioa.pa9, &mut gpioa.crh))?,
        LedSsrStatus::configure((gpioa.pa10, &mut gpioa.crh))?,
        device.TIM1,
        clocks,
        &mut rcc.apb2,
        &mut afio.mapr,
    ))?;
    
    let ui_timer = UiTimer::configure((device.TIM8, clocks, &mut rcc.apb2))?;

    let mut welder_ui = WelderUi::new();
    // Update the quad counter to match the current setting
    update_quad_from_setting_value(&mut encoder, *welder_ui.get_value_mut() as u16);

    info!("Init done");

    Ok(init::LateResources {
        ui_timer,
        encoder,
        zc_in,
        clocks,
        ntc_dma_buffer: ntc_circ_buffer,
        display: ht16k33,
        thermocouple,
        activity_led: usr_led,
        ssr_timer,
        enc_button,
        trigger_button,
        status_led: led_status,
        welder_ui,
    })
}

/// Called when the encoder button has been pressed, updates the UI
fn encoder_button_pressed(welder_ui: &mut WelderUi, encoder: &mut Encoder, display: &mut Display) {
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
    update_quad_from_setting_value(encoder, *welder_ui.get_value_mut() as u16);
    info!("Encoder: {}", encoder.count());
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
fn read_thermocouple_temp(thermocouple: &mut Thermocouple, last_thermocouple_temp: &mut f32) {
    match thermocouple.0.read_thermocouple(&mut thermocouple.1, Unit::Celsius) {
        Ok(v) => *last_thermocouple_temp = v,
        Err(e) => warn!("Error reading thermocouple: {:?}", e),
    };
}

/// Called at 3hz to update the UI from the quad reading & due to elapsed time
fn tick_ui(welder_ui: &mut WelderUi, encoder: &Encoder, display: &mut Display) {
    let value = welder_ui.get_value_mut();
    let new_value = setting_value_from_quad(encoder) as u32;
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
    activity_led: &mut LedUsr,
    dma_buffer: &mut NtcDmaBufferHandle, 
    last_ntc_temp: &mut f32,
    #[cfg_attr(not(feature = "thermocouple"), allow(unused_variables))]
    thermocouple: &mut Thermocouple, 
    last_thermocouple_temp: &mut f32,
    welder_ui: &mut WelderUi, 
    encoder: &Encoder, 
    display: &mut Display,
) {
    // Turn on the led to show activity
    activity_led.on();

    // Update the NTC temp
    read_ntc_temp(dma_buffer, last_ntc_temp);        

    // Update the thermocouple temp
    #[cfg(feature = "thermocouple")]
    read_thermocouple_temp(thermocouple, last_thermocouple_temp);

    // Tick the UI
    tick_ui(welder_ui, encoder, display);

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
    ssr_timer: &mut Ssr,
    #[cfg_attr(feature = "dummy_zc", allow(unused_variables))]
    zc_in: &Zc,
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
fn update_quad_count(_quad: &mut Encoder, count: u16) {
    unsafe {
        let tim = &(*QuadTim::ptr());
        tim.cr1.modify(|_, w| w.cen().clear_bit());
        tim.cnt.write(|w| w.cnt().bits(count));
        tim.cr1.modify(|_, w| w.cen().set_bit());
    }
}

/// Returns the quad count scaled to the settings range
fn setting_value_from_quad(quad: &Encoder) -> u16 {
    quad.count() * QUAD_SCALE % 999
}

/// Reverses the setting scaling and updates the quad count
fn update_quad_from_setting_value(quad: &mut Encoder, value: u16) {
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