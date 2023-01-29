#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(alloc_error_handler)]

// use arduino_hal::port::mode::Output;
// use arduino_hal::port::Pin;
use arduino_hal::prelude::*;
// use avr_device::atmega328p::tc1::tccr1b::CS1_A;
// use avr_device::atmega328p::TC1;
// use core::mem;
use core::ops::Range;
use panic_halt as _;
use panic_halt as _;
use ufmt::uWrite;
// use arduino_hal::port::{mode, Pin};
// use either::*;

enum TaskState {
    RUNNING,
    READY,
    SUSPEND,
}
struct Task {
    task_id: u32,
    task_state: TaskState,
    task_priority: u16,
}

fn blink_for_range<W: uWrite<Error = void::Void>>(range: Range<u16>, serial: &mut W) {
    let task = Task {
        task_id: 1,
        task_state: TaskState::READY,
        task_priority: 2,
    };
    let ticks = task.task_priority;

    range.map(|i| i * 100).for_each(|ms| {
        ufmt::uwriteln!(
            serial,
            "configuring timer output compare register = {}",
            ticks
        )
        .void_unwrap();
        ufmt::uwriteln!(serial, "Hello from Arduino!\r").void_unwrap();
        arduino_hal::delay_ms(ms as u16);
    });
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_OVF() {
    arduino_hal::delay_ms(100);
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x02));
    // Enable the INT0 interrupt source.
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    loop {
        blink_for_range(0..10, &mut serial);
        unsafe {
            avr_device::interrupt::enable();
        };
    }
}
