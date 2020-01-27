use typenum::{U18, U19, U23, U24};
use ascii::AsciiChar;
use generic_array::GenericArray;
use adafruit_alphanum4::{
    AlphaNum4,
    Error as AlphaNumError,
};
use helper_macros::str_to_generic_array_of_ascii_char;

use crate::ui::{ 
    Ui, 
    Screen, 
    Setting, 
    unwrap_setting_value_mut, 
    unwrap_setting_value,
};

const MESSAGE_A_ON: GenericArray<AsciiChar, U18> = str_to_generic_array_of_ascii_char!("FIRST PULSE LENGTH");
const MESSAGE_A_DELAY: GenericArray<AsciiChar, U23> = str_to_generic_array_of_ascii_char!("DELAY AFTER FIRST PULSE");
const MESSAGE_B_ON: GenericArray<AsciiChar, U19> = str_to_generic_array_of_ascii_char!("SECOND PULSE LENGTH");
const MESSAGE_B_DELAY: GenericArray<AsciiChar, U24> = str_to_generic_array_of_ascii_char!("DELAY AFTER SECOND PULSE");

/// The UI for a spot welder
pub struct WelderUi {
    ui: Ui,
    screen_1: Screen<U18, u32>,
    screen_2: Screen<U23, u32>,
    screen_3: Screen<U19, u32>,
    screen_4: Screen<U24, u32>,
    current_screen: u8,
}


/// The values of settings configured by this UI
pub struct WelderSettings {
    /// The on time for the first pulse
    pub a_on: u32,
    /// The delay after the first pulse
    pub a_delay: u32,
    /// The on time for the second pulse
    pub b_on: u32,
    /// The delay after the second pulse
    pub b_delay: u32,
}

// Calls the provided closure with &mut self.ui and &mut self.screen_x as parameters
// Used to avoid duplication of the match current_screen {...}
// Required because the methods on ui that we need are monomorphized so there's no
// single type you can pass the screens in as.
// Dynamic dispatch and passing in a screen trait is an option but playing with
// that on a microcontroller showed it caused a few hundred cycles extra delay.
// The delay probably isn't important in the grand scheme of things but since this 
// UI stuff is getting called in an ISR it's worth shaving off time wherever
// possible. Maybe if the UI code was more general it would make sense to use trait 
// objects for ergonomics but a 4 digit LCD controlled by a rotary encoder is pretty
// niche...
macro_rules! match_screen {
    ($s:ident, $x:expr) => (
        match $s.current_screen {
            0 => $x(&mut $s.ui, &mut $s.screen_1),
            1 => $x(&mut $s.ui, &mut $s.screen_2),
            2 => $x(&mut $s.ui, &mut $s.screen_3),
            3 => $x(&mut $s.ui, &mut $s.screen_4),
            _ => unreachable!(),
        }
    )    
}

impl WelderUi {
    /// Create an instance of the WelderUi
    pub const fn new() -> Self {
        Self {
            ui: Ui::new(2, Some(15)),
            screen_1: Screen::Setting(Setting {
                message: MESSAGE_A_ON,
                single_character: AsciiChar::new('A'),
                dot: false,
                value: 50,
            }),
            screen_2: Screen::Setting(Setting {
                message: MESSAGE_A_DELAY,
                single_character: AsciiChar::new('A'),
                dot: true,
                value: 200,
            }),
            screen_3: Screen::Setting(Setting {
                message: MESSAGE_B_ON,
                single_character: AsciiChar::new('B'),
                dot: false,
                value: 200,
            }),
            screen_4: Screen::Setting(Setting {
                message: MESSAGE_B_DELAY,
                single_character: AsciiChar::new('B'),
                dot: true,
                value: 800,
            }),
            current_screen: 0,
        }
    }

    /// Move to the next screen
    pub fn next_screen<D, T>(&mut self, display: &mut D) -> Result<(), AlphaNumError>
    where
        D: AlphaNum4<T>,
    {
        self.current_screen += 1;
        if self.current_screen > 3 {
            self.current_screen = 0;
        }
        self.ui.reset();

        match_screen!(self, |ui: &mut Ui, screen| ui.update_display(display, screen))
    }

    /// Tick the ui display. This should be called regularly to cause the text
    /// to update, scroll, expand, etc. The Ui is configured to be ticked at 3Hz
    /// by default but this could be changed to tune responsiveness or scroll speed
    pub fn tick<D, T>(&mut self, display: Option<&mut D>) -> Result<(), AlphaNumError>
    where
        D: AlphaNum4<T>,
    {
        match_screen!(self, |ui: &mut Ui, screen| ui.tick(display, screen))
    }

    /// Get a mutable reference to the current screen's value
    /// panics if the current screen is not a Setting (which they all currently are)
    pub fn get_value_mut(&mut self) -> &mut u32 {
        match_screen!(self, |_: &mut Ui, screen| unwrap_setting_value_mut(screen))
    }

    /// Reset the effects of ticking the screen
    /// This collapses the screen if it was expanded and resets the scrolling pos 
    pub fn reset_tick<D, T>(&mut self, display: &mut D) -> Result<(), AlphaNumError>
    where 
        D: AlphaNum4<T>,
    {
        self.ui.reset();
        match_screen!(self, |ui: &mut Ui, screen| ui.update_display(display, screen))
    }

    /// Grabs the current values from each screen and returns them as a WelderSettings
    pub fn settings_snapshot(&self) -> WelderSettings {
        WelderSettings {
            a_on: *unwrap_setting_value(&self.screen_1),
            a_delay: *unwrap_setting_value(&self.screen_2),
            b_on: *unwrap_setting_value(&self.screen_3),
            b_delay: *unwrap_setting_value(&self.screen_4),
        }
    }
}