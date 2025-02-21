#![no_std]
#![no_main]

use rp_pico::{hal::{self, pac, Sio}, entry};

use panic_halt as _;
use ssd1306::{
    I2CDisplayInterface, size::DisplaySize128x64, rotation::DisplayRotation, Ssd1306,
    prelude::{DisplayConfig, Brightness}
};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    text::Text
};
use fugit::{RateExtU32};

use core::fmt::Write;
use heapless::String;

type I2CDisplay = Ssd1306<ssd1306::prelude::I2CInterface<rp2040_hal::I2C<pac::I2C1, (rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio18, rp2040_hal::gpio::FunctionI2c, rp2040_hal::gpio::PullUp>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio19, rp2040_hal::gpio::FunctionI2c, rp2040_hal::gpio::PullUp>)>>, DisplaySize128x64, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>>;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio18.reconfigure();
    let scl_pin: hal::gpio::Pin<_, hal::gpio::FunctionI2C, _> = pins.gpio19.reconfigure();

    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    let interface = I2CDisplayInterface::new(i2c);

    let mut display: I2CDisplay = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();
    display.set_brightness(Brightness::BRIGHTEST).unwrap();


    let mut value = 0;

    loop {
        display_value(&mut display, value);

        value += 10;
    }

    fn display_value(display: &mut I2CDisplay, value: u32 ) 
    {

        let mut data: String<32> = String::<32>::new(); 

        let _ = write!(data, "data:{value}");

        let _ = display.clear(BinaryColor::Off);

        let style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);
        let position = Point::new(40, 60);

        Text::new(&data, position, style).draw(display).unwrap();

        display.flush().unwrap();
   }





}