use stm32f1xx_hal::{
    time::*,    
    spi::{
        Mode, 
        Polarity, 
        Phase,
    },
    i2c::{
        Mode as I2cMode,
        
    },
};

use core::ops::RangeInclusive;

/// If true, wait (possible forever) for first zero crossing
///
/// TODO: maybe have a timeout?
#[cfg(not(feature = "dummy_zc"))]
pub(crate) const WAIT_FOR_ZC_INIT: bool = true;

#[cfg(feature = "dummy_zc")]
/// The frequency to configure the dummy timer to
pub const DUMMY_FREQ: Hertz = Hertz(100);

/// High speed internal oscillator clock frequency
pub const HSI: MegaHertz = MegaHertz(8);

/// High speed external crystal oscillator frequency
pub const HSE: MegaHertz = MegaHertz(8);

/// Target systclk frequency after PLL is enabled
pub const SYSCLK_FREQ: MegaHertz = MegaHertz(72);

/// Target PCLK1 frequency 
pub const PCLK1_FREQ: MegaHertz = MegaHertz(36);
/// Target PCLK2 frequency
pub const PCLK2_FREQ: MegaHertz = MegaHertz(72);
/// Target ADC frequency
pub const ADC_FREQ: MegaHertz = MegaHertz(2);

/// Baud rate used for ITM communication
pub const ITM_BAUDRATE: MegaHertz = MegaHertz(2);

/// Baud rate for I2C display
pub const DISP_BAUDRATE: Hertz = Hertz(20_000);
/// The I2C mode for the display
pub const DISP_I2C_MODE: I2cMode = I2cMode::Standard { frequency: DISP_BAUDRATE };
/// Address for I2C display
pub const DISP_I2C_ADDR: u8 = 112;
/// Timeout for I2C start
pub const DISP_START_TIMEOUT_US: u32 = 5000;
/// How many times to retry start if it fails
pub const DISP_START_RETRIES: u8 = 50;
/// Timeout for I2C address
pub const DISP_ADDR_TIMEOUT_US: u32 = 5000;
/// Timeout for I2C data
pub const DISP_DATA_TIMEOUT_US: u32 = 10000;

/// The frequency the SSR timer is configured for
pub const SSR_TIMER_FREQ: Hertz = Hertz(1);
/// The period of the SSR timer in microseconds
pub const SSR_TIMER_PERIOD_US: u32 = 1_000 / SSR_TIMER_FREQ.0;

/// Scaling to apply to quadrature count
/// This is dependent on the encoder's count/rev value
pub const QUAD_SCALE: u16 = 5;

/// Baud rate for MAX31855 SPI
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
/// The frequency the pwm input timer runs at to detect the zero crossings
pub const ZC_INPUT_FREQ: Hertz = Hertz(300);
/// The frequency the UI timer runs at to update the UI
pub const UI_TIMER_FREQ: Hertz = Hertz(3);

/// If true, panic on display errors, else just log. 
/// This doesn't include config - if i2c or the display fail to 
/// respond during configuration at start up it will still panic
///
/// TODO: maybe no display is a valid config?
pub const HT16K33_MISSING_FATAL: bool = true;

/// If supplied, make sure NTC temperature is in this range before welding
pub const NTC_RANGE: Option<(f32, f32)> = Some((1., 30.));

/// If supplied, make sure thermocouple temperature is in this range before welding
#[cfg(feature = "thermocouple")]
pub const THERMOCOUPLE_RANGE: Option<(f32, f32)> = Some((1., 30.));

/// These ranges specify the min, max and actual mains frequencies.
///
/// Mains frequency is generally very stable in most countries
/// so if we're reading outside these values there is likely something
/// wrong with our hardware. Attempting to start a weld with a 
/// measured frequency outside of one of these ranges will cause
/// a panic.
/// The actual frequency is provided so our microsecond to zero crossing
/// calculation is more accurate.
pub const ACCEPTABLE_ZC_FREQ_RANGES: [(RangeInclusive<u16>, u16); 2] = [
    (98..=102, 100),
    (118..=122, 120),
];