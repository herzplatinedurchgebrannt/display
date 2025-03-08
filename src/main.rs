#![no_std]
#![no_main]

use embedded_graphics::mono_font::iso_8859_1::FONT_10X20;
use embedded_graphics::mono_font::iso_8859_10::FONT_9X15;
use itoa::Buffer;
use panic_halt as _;
use core::char::MAX;
use core::fmt::Write;
use heapless::String;
use fugit::{RateExtU32};

// pico
use rp2040_hal;
use rp_pico::hal::pio::PIOExt;
use rp_pico::hal::prelude::*;
use rp_pico::hal::timer::Timer;
use rp_pico::{hal::{self, pac, Sio}, entry};

// led
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

// display 
use ssd1306::{
    I2CDisplayInterface, size::DisplaySize128x64, rotation::DisplayRotation, Ssd1306,
    prelude::{DisplayConfig, Brightness}
};
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    mono_font::{MonoTextStyle},
    text::Text
};

// type definition
type I2CDisplay = Ssd1306<ssd1306::prelude::I2CInterface<rp2040_hal::I2C<pac::I2C1, (rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio18, rp2040_hal::gpio::FunctionI2c, rp2040_hal::gpio::PullUp>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio19, rp2040_hal::gpio::FunctionI2c, rp2040_hal::gpio::PullUp>)>>, DisplaySize128x64, ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>>;
//type Strip = Ws2812<pac::PIO0, rp2040_hal::pio::SM0, rp2040_hal::timer::CountDown<'_>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio16, rp2040_hal::gpio::FunctionPio0, rp2040_hal::gpio::PullDown>>;

// constants
const TOTAL_LED_COUNT: usize = 43;

const STRIP_0_OFFSET: usize = 0;
const STRIP_0_LENGTH: usize = 8;
const STRIP_1_OFFSET: usize = 0;
const STRIP_1_LENGTH: usize = 13;
const STRIP_2_OFFSET: usize = 0;
const STRIP_2_LENGTH: usize = 8;
const STRIP_3_OFFSET: usize = 0;
const STRIP_3_LENGTH: usize = 13;

const FORCE_OVERLOAD: i32 = 4500;
const FORCE_MAX: i32 = 5000;

// statics
static RED : RGB8 = RGB8::new(30, 0, 0);
static GREEN: RGB8 = RGB8::new(0, 30, 0);
static BLUE: RGB8 = RGB8::new(0, 0, 255);
static BLACK: RGB8 = RGB8::new(0, 0, 0);
static WHITE: RGB8 = RGB8::new(255, 255, 255);

struct StripConfig {
    offset: usize,
    length: usize
}

struct Forces {
    fx: i32,
    fy: i32, 
    fz: i32,
    fr: i32
}

#[entry]
fn main() -> ! 
{
    // pico setup
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    let mut frame_delay =
        cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
        
    // led strip setup
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut ws = Ws2812::new(
        pins.gpio16.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // display setup
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

    is_led_config_valid();

    let mut value: i32 = 0;

    let strip_config: [StripConfig;4] = [
        StripConfig {offset: 0, length: 8},
        StripConfig {offset: 0, length: 13},
        StripConfig {offset: 0, length: 8},
        StripConfig {offset: 0, length: 13}];

    loop {

        strip_multiple_force(&mut ws, value);

        frame_delay.delay_ms(100); 

        display_forces(&mut display, value, value, value, value);

        value += 150;

        if value > FORCE_MAX {
            value = 0;
        }
    }

    fn is_led_config_valid () -> bool {

        let calculated_led_amount: usize = STRIP_0_OFFSET + STRIP_0_LENGTH + STRIP_1_OFFSET + STRIP_1_LENGTH + STRIP_2_OFFSET + STRIP_2_LENGTH + STRIP_3_OFFSET + STRIP_3_LENGTH;


        if calculated_led_amount < TOTAL_LED_COUNT {
            return true
        }

        false

        // => PANIC!
    }


    /// display a full set of force values on the SSD1306 display
    fn display_forces(display: &mut I2CDisplay, fx: i32, fy: i32, fz: i32, fr: i32) {
        let _ = display.clear(BinaryColor::Off);

        let mut data: String<32> = String::<32>::new(); 
        let style = MonoTextStyle::new(&FONT_9X15, BinaryColor::On);

        let fx_pos = Point::new(0, 10);
        let fy_pos = Point::new(0, 30);
        let fz_pos = Point::new(0, 50);
        let fr_pos = Point::new(80, 30);
        let ol_pos = Point::new(30, 30);

        if fr > FORCE_OVERLOAD {
            data.clear();

            let _ = write!(data, "OVERLOAD!");
    
            Text::new(&data, ol_pos, style).draw(display).unwrap();
            display.flush().unwrap();
            return;
        }
        
        let _ = write!(data, "Fx {fx}");

        Text::new(&data, fx_pos, style).draw(display).unwrap();

        data.clear();

        let _ = write!(data, "Fy {fy}");

        Text::new(&data, fy_pos, style).draw(display).unwrap();

        data.clear();

        let _ = write!(data, "Fz {fz}");

        Text::new(&data, fz_pos, style).draw(display).unwrap();

        data.clear();

        let _ = write!(data, "{fr}");

        Text::new(&data, fr_pos, MonoTextStyle::new(&FONT_10X20, BinaryColor::On)).draw(display).unwrap();

        display.flush().unwrap();
    }


    /// visualize the resulting force via led strip
    fn strip_single_force(fr: i32) -> [RGB8; TOTAL_LED_COUNT] {
        let mut result: [RGB8; TOTAL_LED_COUNT] = [BLACK; TOTAL_LED_COUNT];

        let mut led_index_offset: usize = 0;


        // check for overload
        if fr > FORCE_OVERLOAD {
            result.fill(RED);
        }
        else {

            led_index_offset += STRIP_0_OFFSET;

            let active_steps: usize = STRIP_0_LENGTH * (fr as usize) / (FORCE_OVERLOAD as usize);

            for n  in 0..active_steps {
                result[n + led_index_offset] = GREEN;
            }

            led_index_offset += STRIP_0_LENGTH;



            led_index_offset += STRIP_1_OFFSET;

            let active_steps: usize = STRIP_1_LENGTH * (fr as usize) / (FORCE_OVERLOAD as usize);

            for n  in 0..active_steps {
                result[n + led_index_offset] = GREEN;
            }

            led_index_offset += STRIP_1_LENGTH;



            led_index_offset += STRIP_2_OFFSET;

            let active_steps: usize = STRIP_2_LENGTH * (fr as usize) / (FORCE_OVERLOAD as usize);

            for n  in 0..active_steps {
                result[n + led_index_offset] = GREEN;
            }

            led_index_offset += STRIP_2_LENGTH;



            led_index_offset += STRIP_3_OFFSET;

            let active_steps: usize = STRIP_3_LENGTH * (fr as usize) / (FORCE_OVERLOAD as usize);

            for n  in 0..active_steps {
                result[n + led_index_offset] = GREEN;
            }
        }
        result
    }

    fn strip_multiple_force(strip: &mut Ws2812<pac::PIO0, rp2040_hal::pio::SM0, rp2040_hal::timer::CountDown<'_>, rp2040_hal::gpio::Pin<rp2040_hal::gpio::bank0::Gpio16, rp2040_hal::gpio::FunctionPio0, rp2040_hal::gpio::PullDown>>, fr: i32){
         
        strip.write(strip_single_force(fr).iter().copied()).unwrap();


    }


}