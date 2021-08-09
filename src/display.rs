use display_interface::DisplayError;

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

pub struct Display<I> {
    display: Ssd1306<I2CInterface<I>, DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>,
}

impl<I> Display<I>
where
    I: embedded_hal::blocking::i2c::Write,
{
    pub fn new(i2c: I) -> Self {
        let interface = I2CDisplayInterface::new(i2c);
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        Display { display }
    }

    pub fn update<'a>(&mut self, msg: &'a str) -> Result<(), DisplayError> {
        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        self.display.clear();

        Text::with_baseline(msg, Point::new(0, 6), text_style, Baseline::Top)
            .draw(&mut self.display)?;

        self.display.flush()
    }
}
