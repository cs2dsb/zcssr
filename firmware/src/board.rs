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
    time::{
        *,
    },    
    spi::{Mode, Polarity, Phase},
    i2c::Mode as I2cMode,
};
use core::convert::Infallible;
use embedded_hal::digital::v2::OutputPin;

/*
    MCU is STM32F103RCTx
    256K Flash
    48K SRAM
*/

/// High speed internal oscillator clock frequency
pub const HSI: MegaHertz = MegaHertz(8);

/// High speed external crystal oscillator frequency
pub const HSE: MegaHertz = MegaHertz(8);

/// Target systclk frequency after PLL is enabled
pub const SYSCLK_FREQ: MegaHertz = MegaHertz(72);

pub const PCLK1_FREQ: MegaHertz = MegaHertz(36);
pub const PCLK2_FREQ: MegaHertz = MegaHertz(72);
pub const ADC_FREQ: MegaHertz = MegaHertz(2);

/// Baud rate used for ITM communication
pub const ITM_BAUDRATE: MegaHertz = MegaHertz(2);

/// Baud rate for I2C display
pub const DISP_BAUDRATE: Hertz = Hertz(20_000);
pub const DISP_I2C_MODE: I2cMode = I2cMode::Standard { frequency: DISP_BAUDRATE };
/// Address for I2C display
pub const DISP_I2C_ADDR: u8 = 112;
pub const DISP_START_TIMEOUT_US: u32 = 5000;
pub const DISP_START_RETRIES: u8 = 50;
pub const DISP_ADDR_TIMEOUT_US: u32 = 5000;
pub const DISP_DATA_TIMEOUT_US: u32 = 10000;

pub const SSR_TIMER_FREQ: Hertz = Hertz(1);
pub const SSR_TIMER_PERIOD_US: u32 = 1_000 / SSR_TIMER_FREQ.0;

/// Scaling to apply to quadrature count
/// This is dependent on the encoder's count/rev value
pub const QUAD_SCALE: u16 = 5;

/// Baud rate for MAX31855 SPI
//pub const MAX31855_BAUDRATE: KiloHertz = KiloHertz(100);
pub const MAX31855_BAUDRATE: MegaHertz = MegaHertz(4);
/// SPI mode for MAX31855
pub const MAX31855_SPI_MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition
};

/// Assumed VCC voltage
pub const VCC_F: f32 = 3.3;
/// Maximum value ADC can return
pub const ADC_MAX_F: f32 = 4095.;
/// The resistance of the fixed part of the NTC resistor divider
pub const NTC_FIXED_RES: f32 = 10_000.;
/// The nominal resistance of the variable part of the NTC resistor divider
pub const NTC_NOMINAL_RES: f32 = 10_000.;
/// The B coefficient of the NTC
pub const NTC_B_COEFF: f32 = 3936.;
/// The temperature at which the nominal resistance applies
pub const NTC_NOMINAL_TEMP: f32 = 25.;
/// 0C in kelvin
pub const CELCIUS_KELVIN_OFFSET: f32 = 273.15;

macro_rules! define_ptr_type {
    ($name: ident, $ptr: expr) => (
        impl $name {
            fn ptr() -> *const Self {
                $ptr as *const _
            }

            /// Returns a wrapped reference to the value in flash memory
            pub fn get() -> &'static Self {
                unsafe { &*Self::ptr() }
            }
        }
    )
}

#[derive(Debug)]
#[repr(C)]
pub struct FlashSize(u16);
define_ptr_type!(FlashSize, 0x1FFF_F7E0);

impl FlashSize {
    /// Read flash size in kilobytes
    pub fn kilo_bytes(&self) -> u16 {
        self.0
    }

    /// Read flash size in bytes
    pub fn bytes(&self) -> usize {
        usize::from(self.kilo_bytes()) * 1024
    }
}

/// This trait hides if an output is active low or active high
/// Infallible error type allows .unwrap on set_high and set_low
pub trait EnableOutput: OutputPin<Error = Infallible> {
    fn on(&mut self);
    fn off(&mut self);
}

const ACTIVE_HIGH: bool = true;
const ACTIVE_LOW: bool = false;

macro_rules! define_enable_output {
    ($name: ty, $active_high: expr) => (
        impl EnableOutput for $name {
            fn on(&mut self) {
                if $active_high {
                    self.set_high().unwrap()
                } else {
                    self.set_low().unwrap()
                }
            }
            fn off(&mut self) {
                if $active_high {
                    self.set_low().unwrap()
                } else {
                    self.set_high().unwrap()
                }
            }
        }
    )
}

/// Debug LED on PCB near MCU
/// TIM3_CH2
pub type LedUsr         = PA7<Output<PushPull>>;
define_enable_output!(LedUsr, ACTIVE_LOW);

/// Status LED to indicate ready, error, etc.
/// TIM5_CH3 (or TIM2_CH3)
pub type LedStatus      = PA2<Output<PushPull>>;
define_enable_output!(LedStatus, ACTIVE_HIGH);

/// Status LED to indicate SSR status. Typically same state as SSR_EN
/// TIM1_CH3
pub type LedSsrStatus<M>   = PA10<M>;

/// Output that triggers SSR to allow current flow. Active high
/// TIM1_CH2
pub type SsrEn<M>       = PA9<M>;

/// Misc output (fans etc)
pub type MiscEn         = PB15<Output<PushPull>>;
define_enable_output!(MiscEn, ACTIVE_HIGH);

/// Rotary encoder push button input. Active low
/// EXTI5
pub type EncBtn         = PB5<Input<Floating>>;

/// Rotary encoder quadrature input A
/// TIM4_CH1
pub type EncA           = PB6<Input<Floating>>;

/// Rotary encoder quadrature input B
/// TIM4_CH2
pub type EncB           = PB7<Input<Floating>>;

/// NTC thermister input. Half way point of resistor divider VCC-10k-\[NTC\]-GND
/// ADC1_CH9 (or ADC2_CH9)
pub type Ntc            = PB1<Analog>;

/// Input from zero crossing detection circuit. Active low
/// TIM2_CH1
pub type ZcRising       = PA0<Input<Floating>>;
/// Input from zero crossing detection circuit. Active low
/// TIM2_CH2
pub type ZcFalling      = PA1<Input<Floating>>;

/// User input trigger. Active low
/// EXTI8
pub type Trigger        = PA8<Input<Floating>>;

/// I2C clock for quad 14(?) segment display
/// I2C2
pub type DispScl        = PB10<Alternate<OpenDrain>>;

/// I2C data for quad 14(?) segment display
/// I2C2
pub type DispSda        = PB11<Alternate<OpenDrain>>;

/// USB D-
pub type UsbDm<T>       = PA11<T>;

/// USB D+
pub type UsbDp<T>       = PA12<T>;

/// MAX31855K thermocouple converter - slave select, active low
/// Spi2
pub type KThermoNss     = PB12<Output<PushPull>>;
define_enable_output!(KThermoNss, ACTIVE_LOW);

/// MAX31855K thermocouple converter - serial-clock
/// Spi2
pub type KThermoSck     = PB13<Alternate<PushPull>>;

/// MAX31855K thermocouple converter - serial data out
/// Spi2
pub type KThermoMiso    = PB14<Input<Floating>>;

/// MAX31855K thermocouple converter - serial data in
/// Spi2
/// MAX31855K doesn't have this pin but HAL SPI interface requires it
pub type KThermoMosiNotConnected = PB15<Alternate<PushPull>>;


/*
RGB led is rotated incorrectly on current batch of PCBs
pub type RgbRLed = PA4<Output<PushPull>>;
pub type RgbBLed = PA5<Output<PushPull>>;
pub type RgbGLed = PA6<Output<PushPull>>;
*/