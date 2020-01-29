use stm32f1xx_hal::{
    gpio::{
        PushPull,
        Alternate,
        Input,
        Output,
        Floating,
        OpenDrain,
        Analog,
        gpioa::*,
        gpiob::*,
    },    
    timer::{
        Tim4NoRemap,
        CountDownTimer,
    },
    spi::{
        Spi,
        Spi2NoRemap,
        NoMosi,
    },
    i2c::BlockingI2c,
    pac::{
        I2C2,
        SPI2,
        TIM2,
        TIM4,
        TIM8,
        TIM1,
    },
    qei::Qei,
    pwm::{
        Pwm,
        C2,
        C3,
    },
};

#[cfg(not(feature = "dummy_zc"))]
use stm32f1xx_hal::{ 
    pwm_input::PwmInput,
    timer::Tim2NoRemap,
};
use ht16k33::HT16K33;

/// Zero crossing timer reads pwm input from the zero crossing detection circuit (or a dummy count down timer if dummy_zc feature is enabled)
pub type ZcTim = TIM2;
/// Quadrature encoder timer reads the quad input from the rotary encoder
pub type QuadTim = TIM4;
/// The UI timer provides a periodic interrupt to update the display, read temperature and periodically log useful info
pub type UiTim = TIM8;
/// The SSR timer drives the SsrEn pin to enable the welder. A timer is used so the control software crashing should never result in the SSR being left enabled
pub type SsrTim = TIM1;

/// The complete handle for the UI timer
pub type UiTimer = CountDownTimer<UiTim>;

/// Debug LED on PCB near MCU
/// TIM3_CH2
pub type LedUsr         = PA7<Output<PushPull>>;

/// Status LED to indicate ready, error, etc.
/// TIM5_CH3 (or TIM2_CH3)
pub type LedStatus      = PA2<Output<PushPull>>;

/// Status LED to indicate SSR status. Typically same state as SSR_EN
/// TIM1_CH3
pub type LedSsrStatus   = PA10<Alternate<PushPull>>;

/// Output that triggers SSR to allow current flow. Active high
/// TIM1_CH2
pub type SsrEn          = PA9<Alternate<PushPull>>;

/// Complete handle for the SSR output
pub type Ssr = (Pwm<SsrTim, C2>, Pwm<SsrTim, C3>);

/// Misc output (fans etc)
pub type MiscEn         = PB15<Output<PushPull>>;

/// Rotary encoder push button input. Active low
/// EXTI5
pub type EncBtn         = PB5<Input<Floating>>;

/// Rotary encoder quadrature input A
/// TIM4_CH1
pub type EncA           = PB6<Input<Floating>>;

/// Rotary encoder quadrature input B
/// TIM4_CH2
pub type EncB           = PB7<Input<Floating>>;

/// Complete handle for the encoder
pub type Encoder        = Qei<QuadTim, Tim4NoRemap, (EncA, EncB)>;

/// NTC thermister input. Half way point of resistor divider VCC-10k-\[NTC\]-GND
/// ADC1_CH9 (or ADC2_CH9)
pub type Ntc            = PB1<Analog>;

/// Input from zero crossing detection circuit. Active low
/// TIM2_CH1
pub type ZcRising       = PA0<Input<Floating>>;
/// Input from zero crossing detection circuit. Active low
/// TIM2_CH2
pub type ZcFalling      = PA1<Input<Floating>>;

#[cfg(not(feature = "dummy_zc"))] 
/// Complete handle for the Zc timer
pub type Zc = PwmInput<ZcTim, Tim2NoRemap, (ZcRising, ZcFalling)>;

#[cfg(feature = "dummy_zc")] 
/// Complete handle for the Zc timer
pub type Zc = CountDownTimer<ZcTim>;

/// User input trigger. Active low
/// EXTI8
pub type Trigger        = PA8<Input<Floating>>;

/// I2C clock for quad 14(?) segment display
/// I2C2
pub type DispScl        = PB10<Alternate<OpenDrain>>;

/// I2C data for quad 14(?) segment display
/// I2C2
pub type DispSda        = PB11<Alternate<OpenDrain>>;

/// The complete handle for the HT16K33 Display
pub type Display        = HT16K33<BlockingI2c<I2C2, (DispScl, DispSda)>>;

/// USB D-
pub type UsbDm<T>       = PA11<T>;

/// USB D+
pub type UsbDp<T>       = PA12<T>;

/// MAX31855K thermocouple converter - slave select, active low
/// Spi2
pub type KThermoNss     = PB12<Output<PushPull>>;

/// MAX31855K thermocouple converter - serial-clock
/// Spi2
pub type KThermoSck     = PB13<Alternate<PushPull>>;

/// MAX31855K thermocouple converter - serial data out
/// Spi2
pub type KThermoMiso    = PB14<Input<Floating>>;

/// Complete handle for the MAX31855 thermocouple
pub type Thermocouple = (Spi<SPI2, Spi2NoRemap, (KThermoSck, KThermoMiso, NoMosi)>, KThermoNss);