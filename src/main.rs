#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp_pico::Pins;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::clocks;
use rp_pico::hal::prelude::*;
use embedded_time::rate::*;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let core = pac::CorePeripherals::take().unwrap();
    let clocks = clocks::init_clocks_and_plls(rp_pico::XOSC_CRYSTAL_FREQ, 
        pac.XOSC, 
        pac.CLOCKS, 
        pac.PLL_SYS,
        pac.PLL_USB, 
        &mut pac.RESETS, 
        &mut watchdog).ok().unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = Pins::new(pac.IO_BANK0, 
        pac.PADS_BANK0, 
        sio.gpio_bank0, 
        &mut pac.RESETS);


    // Set a pin to drive output
    let mut output_pin = pins.led.into_push_pull_output();
    let button = pins.gpio15.into_pull_down_input();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());


    loop {
        if button.is_high().unwrap() {
            output_pin.set_high().unwrap();
            delay.delay_ms(100);
            output_pin.set_low().unwrap();
            delay.delay_ms(100);
        } else {
            output_pin.set_high().unwrap();
        }
    }
}
