//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manual for an explanation. This is not an issue on the blue pill.

#![no_std]
#![no_main]
#![allow(dead_code)]

use cortex_m::interrupt::Mutex;

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use nb::block;
use panic_halt as _;

use core::cell::Cell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU8, Ordering};
use stm32f1xx_hal::gpio::{Edge, ExtiPin, Input, PullUp};
use stm32f1xx_hal::pac::{interrupt, Interrupt, NVIC};
use stm32f1xx_hal::serial::Serial;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{pac, prelude::*};
use unwrap_infallible::UnwrapInfallible;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut INT_PIN: MaybeUninit<stm32f1xx_hal::gpio::gpioa::PA7<Input<PullUp>>> =
    MaybeUninit::uninit();

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let mut cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let mut dp = pac::Peripherals::take().unwrap();
    let mut afio = dp.AFIO.constrain();
    let _stim = &mut cp.ITM.stim[0];

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    // Acquire the GPIO peripheral
    let mut gpioa = dp.GPIOA.split();
    let mut gpioc = dp.GPIOC.split();
    let mut gpiob = dp.GPIOB.split();

    //              USB Setup
    // BluePill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    delay(clocks.sysclk().raw() / 100);

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };

    // Unsafe to allow access to static variables
    unsafe {
        let bus = UsbBus::new(usb);

        USB_BUS = Some(bus);

        USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        USB_DEVICE = Some(usb_dev);
    }

    // Red button setup, work on interrupt
    {
        let red_buton = unsafe { &mut *INT_PIN.as_mut_ptr() };
        *red_buton = gpioa.pa7.into_pull_up_input(&mut gpioa.crl);
        red_buton.make_interrupt_source(&mut afio);
        red_buton.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        red_buton.enable_interrupt(&mut dp.EXTI);
    }

    //          USART1 setup
    let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let rx = gpiob.pb7;

    let (mut tx, mut rx) =
        Serial::new(dp.USART1, (tx, rx), &mut afio.mapr, 115200.bps(), &clocks).split();

    //              Enable all interrupt
    unsafe {
        NVIC::unmask(Interrupt::USB_HP_CAN_TX);
        NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        NVIC::unmask(pac::Interrupt::EXTI9_5);
    }

    //onboard led
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.set_high();

    //relay module output
    let mut relay = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);
    relay.set_high();
    let mut delay = cp.SYST.delay(&clocks);

    //green button setup
    let green_buton = gpioa.pa0.into_pull_up_input(&mut gpioa.crl);

    loop {
        let mut command_mutex: Option<Command> = None;
        cortex_m::interrupt::free(|cs| command_mutex = COMMAND_MUTEX.borrow(cs).get());
        if (command_mutex.is_some() && command_mutex.unwrap() == Command::Open)
            || green_buton.is_low()
        {
            led.set_low();
            relay.set_low();
            delay.delay_ms(2_000_u16);
            led.set_high();
            relay.set_high();
            cortex_m::interrupt::free(|cs| COMMAND_MUTEX.borrow(cs).replace(None));
        }

        if let Ok(r) = rx.read() {
            cortex_m::interrupt::free(|cs| COMMAND_MUTEX.borrow(cs).set(Command::from_byte(r)));
            cortex_m::interrupt::free(|cs| command_mutex = COMMAND_MUTEX.borrow(cs).get());
            if command_mutex.is_some() && command_mutex.unwrap() == Command::GetMes {
                cortex_m::interrupt::free(|cs| COMMAND_MUTEX.borrow(cs).replace(None));

                if DOOR_BELL.load(Ordering::Relaxed) == 2 {
                    block!(tx.write(1)).unwrap_infallible();
                }

                DOOR_BELL.store(0, Ordering::Relaxed);
            }
        }

        if DOOR_BELL.load(Ordering::Relaxed) == 1 {
            let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };
            int_pin.disable_interrupt(&mut dp.EXTI);
            delay.delay_ms(100_u16);
            if int_pin.is_low() {
                DOOR_BELL.store(2, Ordering::Relaxed);
            }
            delay.delay_ms(900_u16);
            int_pin.enable_interrupt(&mut dp.EXTI);
        }
    }
}

static DOOR_BELL: AtomicU8 = AtomicU8::new(0);
static COMMAND_MUTEX: Mutex<Cell<Option<Command>>> = Mutex::new(Cell::new(None));

#[derive(Clone, Copy, PartialEq)]
enum Command {
    Open,
    SaveKey,
    GetMes,
}

impl Command {
    fn from_byte(b: u8) -> Option<Command> {
        match b {
            0x30 => Some(Command::SaveKey),
            0x31 => Some(Command::Open),
            0x32 => Some(Command::GetMes),
            _ => None,
        }
    }
}

#[interrupt]
fn EXTI9_5() {
    let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };
    if int_pin.check_interrupt() {
        DOOR_BELL.store(1, Ordering::Relaxed);
        // if we don't clear this bit, the ISR would trigger indefinitely
        int_pin.clear_interrupt_pending_bit();
    }
}

#[interrupt]
fn USB_HP_CAN_TX() {
    usb_interrupt();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    usb_interrupt();
}

fn usb_interrupt() {
    let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf_rx = [0u8; 8];
    let mut buf_tx = [0u8; 1];
    match serial.read(&mut buf_rx) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf_rx[0..count].iter_mut() {
                let mut usb_command_mutex: Option<Command> = None;
                cortex_m::interrupt::free(|cs| {
                    COMMAND_MUTEX.borrow(cs).set(Command::from_byte(*c))
                });
                cortex_m::interrupt::free(|cs| usb_command_mutex = COMMAND_MUTEX.borrow(cs).get());

                if usb_command_mutex.is_some() && usb_command_mutex.unwrap() == Command::GetMes {
                    buf_tx[0] = DOOR_BELL.load(Ordering::Relaxed);

                    cortex_m::interrupt::free(|cs| COMMAND_MUTEX.borrow(cs).replace(None));

                    serial.write(&buf_tx[0..1]).ok();

                    DOOR_BELL.store(0, Ordering::Relaxed);
                }
            }
        }
        _ => {}
    }
}
