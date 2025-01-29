#![no_std]
#![no_main]  

use core::cell::Cell;
use cortex_m::delay::Delay;
use panic_halt as _;
use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::Pins,
    pac,
    pio::PIOExt,
    watchdog::Watchdog,  
    Sio,
    Timer,
};
use smart_leds::{brightness, SmartLedsWrite, RGB8}; 
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000;
const WIDTH: usize = 8;
const HEIGHT: usize = 8;
const NUM_LEDS: usize = WIDTH * HEIGHT;

pub struct Screen<'a> {
    leds: &'a [Cell<RGB8>],
}

impl<'a> Screen<'a> {
    pub fn new(leds: &'a [Cell<RGB8>]) -> Self {
        Self { leds }
    }

    pub fn initialize(&self) {
        for led in self.leds.iter() {
            led.set(RGB8::default());
        }
    }

    pub fn fill(&self, color: RGB8) {
        for led in self.leds.iter() {
            led.set(color);
        } 
    }
}

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.gpio16.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let colors = [
        RGB8::new(255, 0, 0), // Red
        RGB8::new(0, 255, 0), // Green
        RGB8::new(0, 0, 255), // Blue
    ];

    let leds: [Cell<smart_leds::RGB<u8>>; NUM_LEDS] = {
        const DEFAULT: Cell<RGB8> = Cell::new(RGB8 { r: 0, g: 0, b: 0 });
        [DEFAULT; NUM_LEDS]
    };

    let screen = Screen::new(&leds);
    screen.initialize();

    let mut current_color_index = 0;

    loop {
        screen.fill(colors[current_color_index]);
        current_color_index = (current_color_index + 1) % 3;

        ws.write(brightness(leds.iter().map(|c| c.get()), 32))
            .unwrap();

        delay.delay_ms(1000);
    }
}