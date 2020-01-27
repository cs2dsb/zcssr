use ascii::AsciiChar;
use generic_array::{GenericArray, ArrayLength};
use adafruit_alphanum4::{
    AlphaNum4,
    Index,
    Error as AlphaNumError,
};

/// A UI specific to a spot welder
pub mod welder;

const BLANK: AsciiChar = AsciiChar::new(' ');
const DISPLAY_CHARS: usize = 4;

/// A text based message with no value or collapsed form
pub type Message<N> = GenericArray<AsciiChar, N>;

/// Trait to make different values displayable on the adafruit_alphanum4 display
///
/// TODO: move into adafruit_alphanum4 library maybe?
pub trait Displayable: Default + Copy {
    /// Display self on the provided display at provided index if possible
    fn display<D, T>(&self, display: &mut D, index: Index) -> Result<(), AlphaNumError> 
    where
        D: AlphaNum4<T>;
}

impl Displayable for f32 {
    fn display<D, T>(&self, display: &mut D, index: Index) -> Result<(), AlphaNumError> 
    where
        D: AlphaNum4<T>,
    {
        display.update_buffer_with_float(index, *self, 0, 10)
    }
}

impl Displayable for u32 {
    fn display<D, T>(&self, display: &mut D, index: Index) -> Result<(), AlphaNumError> 
    where
        D: AlphaNum4<T>,
    {
        //TODO: perhaps move this into Alphanum crate?
        const SCREEN_DIGITS: u32 = 4;
        let index = u32::from(u8::from(index));
        let remaining = SCREEN_DIGITS - index;

        let mut digits = 1;
        while digits <= SCREEN_DIGITS && (self / 10_u32.pow(digits)) > 0 {
            digits += 1;
        }
        if digits > remaining {
            Err(AlphaNumError::InsufficientDigits)?
        }

        for d in 0..remaining {
            let index = Index::from((index + d) as u8);
            if d < digits {
                let digit = (self / 10_u32.pow(digits - d - 1) % 10) as u8;
                display.update_buffer_with_digit(index, digit);
            } else {
                display.update_buffer_with_char(index, BLANK);
            }
        }
        
        Ok(())
    }
}

/// A setting that has a collapsed form, expanded message and a value
pub struct Setting<N: ArrayLength<AsciiChar>, V: Displayable> {
    message: Message<N>,
    single_character: AsciiChar,
    dot: bool,
    value: V,
}

/// Possible screen variants
pub enum Screen<N: ArrayLength<AsciiChar>, V: Displayable> {
    /// A text message with no associated value or collapsed form
    Message(Message<N>),
    /// A setting with a value, collapsed form and text message expansion
    Setting(Setting<N, V>),
}

/// Gets the mutable value of a setting. Panics if the screen isn't a setting
pub fn unwrap_setting_value_mut<N, V>(screen: &mut Screen<N, V>) -> &mut V
where
    N: ArrayLength<AsciiChar>,
    V: Displayable,
{
    match screen {
        Screen::Setting(s) => &mut s.value,
        _ => panic!("unwrap_setting_value_mut called on non-Setting Screen variant"),
    }
}


/// Gets the immutable value of a setting. Panics if the screen isn't a setting
pub fn unwrap_setting_value<N, V>(screen: &Screen<N, V>) -> &V
where
    N: ArrayLength<AsciiChar>,
    V: Displayable,
{
    match screen {
        Screen::Setting(s) => &s.value,
        _ => panic!("unwrap_setting_value called on non-Setting Screen variant"),
    }
}

/// The data required to show the 4 character representation of a setting
#[derive(Default)]
pub struct CollapsedForm<V> {
    character: AsciiChar,
    dot: bool,
    value: V,
}

impl<N: ArrayLength<AsciiChar>, V: Displayable> Screen<N, V> {
    fn collapsable(&self) -> bool {
        match self {
            Screen::Message(_) => false,
            Screen::Setting(_) => true,
        }
    }
    fn message(&self) -> &Message<N> {
        match self {
            Screen::Message(m) => m,
            Screen::Setting(Setting { message: m, .. }) => m,
        }
    }
    fn collapsed_form(&self) -> CollapsedForm<V> {
        match self {
            Screen::Message(m) => CollapsedForm { character: m[0], .. Default::default()  },
            Screen::Setting(s) => CollapsedForm { character: s.single_character, dot: s.dot, value: s.value },
        }
    }
}

/// Structure that tracks the current state of the UI text scrolling and expanding
pub struct Ui {
    pos: usize,
    expanded: bool,
    tick: usize,
    ticks_per_scroll: usize,
    ticks_before_expand: Option<usize>,   
}

impl Ui {
    /// Creates a new UI with the position, tick and expanded state set to defaults
    pub const fn new(ticks_per_scroll: usize, ticks_before_expand: Option<usize>) -> Self {
        Self {
            pos: 0,
            expanded: false,
            tick: 0,
            ticks_per_scroll,
            ticks_before_expand,  
        }
    }

    /// Clears the tick, pos and expanded state (used when user provides input and we want to reset the timeouts/scrolling)
    pub fn reset(&mut self) {
        self.tick = 0;
        self.pos = 0;
        self.expanded = false;
    }

    /// Updates the display with a new screen, resets character position and expanded state
    pub fn set_screen<N, D, T, V>(&mut self, display: &mut D, screen: &Screen<N, V>) -> Result<(), AlphaNumError>
    where
        N: ArrayLength<AsciiChar>,
        D: AlphaNum4<T>,
        V: Displayable,
    {
        self.pos = 0;
        self.expanded = false;
        self.tick = 0;

        self.update_display(display, screen)
    }

    /// Updates the display with the current pos, tick, etc. Doesn't advance them
    pub fn update_display<N, D, T, V>(&mut self, display: &mut D, screen: &Screen<N, V>) -> Result<(), AlphaNumError>
    where
        N: ArrayLength<AsciiChar>,
        D: AlphaNum4<T>,
        V: Displayable,
    {
        if !screen.collapsable() || self.expanded {
            let message = screen.message();
            for i in 0..DISPLAY_CHARS {
                let p = self.pos + i;
                let ch = if p < N::to_usize() { message[p] } else { BLANK };
                display.update_buffer_with_char(Index::from(i as u8), ch);
            }
        } else {
            let collapsed_form = screen.collapsed_form();
            display.update_buffer_with_char(Index::One, collapsed_form.character);
            if collapsed_form.dot {
                display.update_buffer_with_dot(Index::One, true);
            }
            collapsed_form.value.display(display, Index::Two)?;
        }

        Ok(())
    }

    /// Tick the ui and update the display if it's provided
    pub fn tick<N, D, T, V>(&mut self, display: Option<&mut D>, screen: &Screen<N, V>) -> Result<(), AlphaNumError>
    where
        N: ArrayLength<AsciiChar>,
        D: AlphaNum4<T>,
        V: Displayable,
    {
        self.tick += 1;
        let expanded = !screen.collapsable() || self.expanded;

        if expanded && self.tick >= self.ticks_per_scroll {
            self.tick = 0;
            if self.pos >= N::to_usize() {
                self.pos = 0;
                self.expanded = false;
            } else {
                self.pos += 1;
            }
        } else if !expanded {
            if let Some(ticks) = self.ticks_before_expand {
                if self.tick >= ticks {
                    self.tick = 0;
                    self.expanded = true;
                }
            }
        }

        if let Some(display) = display {
            self.update_display(display, screen)?
        }

        Ok(())
    }
}





/*


impl<N: ArrayLength<AsciiChar>> CurrentScreen<N> {
    fn is_expanded(&self) -> bool {
        match self.screen {
            Screen::Message(_) => true,
            Screen::Setting(_) => self.expanded,
        }
    }

    fn message(&self) -> &Message<N> {
        match &self.screen {
            Screen::Message(m) => m,
            Screen::Setting(Setting { message: m, .. }) => m,
        }
    }

    fn tick<A, I>(&mut self, display: &mut A) 
    where 
        A: AlphaNum4<I>
    {
        let expanded = self.is_expanded();
        if expanded {
            let message = self.message();
            for i in 0..4 {
                let p = self.pos + i;
                let ch = if p < N::to_usize() { message[p] } else { BLANK };
                display.update_buffer_with_char(Index::from(i as u8), ch);
            }
        } else {
            if let Screen::Setting(setting) = &self.screen {
            } else {
                unreachable!()
            }            
        }

        self.tick += 1;
        if expanded && self.tick >= self.ticks_per_scroll {
            self.tick = 0;
            if self.pos >= N::to_usize() {
                self.pos = 0;
                self.expanded = false;
            } else {
                self.pos += 1;
            }
        } else if !expanded && self.tick >= self.ticks_before_expand {
            self.tick = 0;
            self.expanded = true;
        }
    }
}



fn poo<N>(a: &N) 
where 
    N: ArrayLength<AsciiChar>
{
    //N::to_usize()
}*/