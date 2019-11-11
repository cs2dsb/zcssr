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
        gpioc::*,
    },    
    time::{
        *,
    },
};

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

/// Debug LED on PCB near MCU
/// TIM3_CH2
pub type LedUsr        = PA7<Alternate<PushPull>>;

/// Status LED to indicate ready, error, etc.
/// TIM8_CH3
pub type LedStatus     = PC8<Alternate<PushPull>>;

/// Status LED to indicate SSR status. Typically same state as SSR_EN
/// TIM8_CH2
pub type LedSsrStatus = PC7<Output<PushPull>>;

/// Rotary encoder push button input. Active low
/// EXTI5
pub type EncBtn        = PB5<Input<Floating>>;

/// Rotary encoder quadrature input A
/// TIM4_CH1
pub type EncA          = PB6<Input<Floating>>;

/// Rotary encoder quadrature input B
/// TIM4_CH2
pub type EncB          = PB7<Input<Floating>>;

/// NTC thermister input. Half way point of resistor divider VCC-10k-[NTC]-GND
/// ADC1_CH9
pub type Ntc            = PB1<Analog>;

/// Input from zero crossing detection circuit. Active low
/// TIM2_CH1
pub type ZcRising       = PA0<Input<Floating>>;
/// Input from zero crossing detection circuit. Active low
/// TIM2_CH1
pub type ZcFalling      = PA1<Input<Floating>>;

/// User input trigger. Active low
/// EXTI8
pub type Trigger        = PA8<Output<PushPull>>;

/// Output that triggers SSR to allow current flow. Active high
pub type SsrEn         = PC9<Output<PushPull>>;

/// I2C clock for quad 14(?) segment display
/// I2C2
pub type DispScl            = PB10<Alternate<OpenDrain>>;

/// I2C data for quad 14(?) segment display
/// I2C2
pub type DispSda            = PB11<Alternate<OpenDrain>>;

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


/********************************************** 
    * V0.2 Board:

/// Debug LED on PCB near MCU
/// TIM3_CH2
pub type LedUsr         = PA7<Alternate<PushPull>>;

/// Status LED to indicate ready, error, etc.
/// TIM5_CH3 (or TIM2_CH3)
pub type LedStatus      = PA2<Alternate<PushPull>>;

/// Status LED to indicate SSR status. Typically same state as SSR_EN
/// TIM1_CH3
pub type LedSsrStatus   = PA10<Alternate<PushPull>>;

/// Output that triggers SSR to allow current flow. Active high
/// TIM1_CH2
pub type SsrEn          = PA9<Alternate<PushPull>>;

/// Rotary encoder push button input. Active low
/// EXTI5
pub type EncBtn         = PB5<Input<Floating>>;

/// Rotary encoder quadrature input A
/// TIM4_CH1
pub type EncA           = PB6<Input<Floating>>;

/// Rotary encoder quadrature input B
/// TIM4_CH2
pub type EncB           = PB7<Input<Floating>>;

/// NTC thermister input. Half way point of resistor divider VCC-10k-[NTC]-GND
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
pub type KThermoNss     = PB12<Output<PushPull>>;
/// MAX31855K thermocouple converter - serial-clock
pub type KThermoSck     = PB13<Alternate<PushPull>>;
/// MAX31855K thermocouple converter - serial data out
pub type KThermoMiso    = PB14<Input<Floating>>;
/// MAX31855K thermocouple converter - serial data in
/// MAX31855K doesn't have this pin but HAL SPI interface requires it
pub type KThermoMosiNotConnected = PB13<Alternate<PushPull>>;

**********************************************/