#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;

use embedded_time::fixed_point::FixedPoint;
use usb_device::{class_prelude::*, prelude::*};

use rp_pico::Pins;
use rp_pico::hal;
use rp_pico::hal::prelude::*;
use rp_pico::hal::pac;
use rp_pico::hal::pac::interrupt;

use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;
/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
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
        &mut watchdog
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = Pins::new(
        pac.IO_BANK0, 
        pac.PADS_BANK0, 
        sio.gpio_bank0, 
        &mut pac.RESETS,
    );

    // HID Stuff
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 60);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27db)) 
        .manufacturer("Fake company")
        .product("One Button Keyboard")
        .serial_number("TEST")
        .device_class(0xEF) // misc
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Set a pin to drive output
    let mut output_pin = pins.led.into_push_pull_output();
    let button = pins.gpio10.into_pull_up_input();

    loop {
        if button.is_low().unwrap() {
            // delay.delay_ms(500);
            output_pin.set_high().unwrap();
            keyboard_press(key_report(0x17)).ok().unwrap_or(0);
            //keyboard_press(key_report(0x00)).ok().unwrap_or(0);
        } else {
            //keyboard_press(key_report(0x17)).ok().unwrap_or(0);
             output_pin.set_low().unwrap();
             keyboard_press(key_report(0x0)).ok().unwrap_or(0);
            // delay.delay_ms(1000);
        }
    }
}

fn key_report(key: u8) -> KeyboardReport {
    KeyboardReport {
        modifier: 0,
        reserved: 0,
        leds: 0,
        keycodes: [key,0,0,0,0,0]
    }
}

fn keyboard_press(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

// More HID stuff
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}
