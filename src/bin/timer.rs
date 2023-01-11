#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::prelude::*;
use avr_device::atmega328p::tc1::tccr1b::CS1_A;
use avr_device::atmega328p::TC1;
use panic_halt as _;
use panic_halt as _;
use ufmt::{uWrite, uwriteln};

use core::cell::{self};

use arduino_hal::port::mode::Output;
use arduino_hal::port::Pin;

use arduino_hal::hal::port::{PD4, PD5, PD6};

static mut PRIORITY_STACK: Vec<&usize, 8> = Vec::new();

static HIGH_PRIORITY_TASK_ID: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

static mut TASKS: Vec<TaskControlBlock, 8> = Vec::new();
pub trait GlobalLog: Sync {
    fn log(&self, address: u8);
}

use heapless::Vec; // fixed capacity `std::Vec`

enum TaskState {
    RUNNING,
    READY,
    SUSPEND,
}

pub struct TaskManager<'a> {
    pub task_control_block: &'a TaskControlBlock,
    pub task_handler: (),
}
pub struct TaskControlBlock {
    task_id: u32,
    task_state: TaskState,
    task_priority: &'static usize,
    task_handler: (),
}

impl TaskManager<'_> {
    pub fn update<W: uWrite<Error = void::Void>>(&mut self, serial: &mut W) {
        {
            let task_control_block: &TaskControlBlock = &self.task_control_block;
            let _serial: &mut W = serial;
            let _task_id: &u32 = &task_control_block.task_id;
            // let task_state = &task_control_block.task_state;
            let task_priority = &task_control_block.task_priority;
            // ufmt::uwriteln!(serial, "new push task_id = {}", task_id).void_unwrap();
            unsafe {
                PRIORITY_STACK.push(task_priority);
            }
        };
    }
}
pub fn context_switch() {
    let running = TaskState::RUNNING;
    let ready = TaskState::READY;
    let top_priority = get_top_priority();
    avr_device::interrupt::free(|cs| {
        for tcb_stack in unsafe { &TASKS } {
            let task_id = tcb_stack.task_id;
            let mut _task_state = &tcb_stack.task_state;
            let mut _task_priority = tcb_stack.task_priority;
            if &top_priority == _task_priority {
                avr_device::interrupt::free(|cs| {
                    HIGH_PRIORITY_TASK_ID.borrow(cs).set(task_id);
                });
                // TODO: stateの切り替え(→RUNNING)
                _task_state = &running;
                // unreachable!();
            } else {
                _task_state = &ready;
            }
        }
    });
}

pub fn task_init<W: uWrite<Error = void::Void>>(serial: &mut W) {
    unsafe {
        if PRIORITY_STACK.is_empty() {
            all_set_task(serial);
        }
    }
}

fn high_priotiry_task_id() -> u32 {
    avr_device::interrupt::free(|cs| HIGH_PRIORITY_TASK_ID.borrow(cs).get())
}

pub fn start_task<W: uWrite<Error = void::Void>>(serial: &mut W) {
    task_init(serial);

    let mut _task_id = high_priotiry_task_id() as usize;

    if _task_id <= 0 {
        return;
    }

    ufmt::uwriteln!(serial, "current high task priority task_id= {}", _task_id).void_unwrap();
    // unsafe { TASKS[_task_id].task_handler }
}

pub fn get_top_priority() -> usize {
    let max: usize;
    unsafe {
        // 最も優先順位の高いものを検索する
        match PRIORITY_STACK.iter().max() {
            Some(n) => max = **n,
            None => unreachable!(),
        };
        // 優先順位の低い順にソートする
        PRIORITY_STACK.sort_unstable();
        // 優先順位の最も高い要素のインデックスを取り除く
        PRIORITY_STACK.remove(PRIORITY_STACK.len() - 1);
        max
    }
}
#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    // TMR_OVERFLOW.store(true, atomic::Ordering::SeqCst);
    context_switch();
}

fn all_set_task<W: uWrite<Error = void::Void>>(serial: &mut W) {
    // stackの所有権がtaskに移動しまわないように、参照を借用する
    for task in unsafe { &TASKS } {
        let mut task_manager = TaskManager {
            task_control_block: &task,
            task_handler: task.task_handler,
        };
        task_manager.update(serial);
    }
}

fn led4(pin: &mut Pin<Output, PD4>) {
    pin.toggle();
}
fn led5(pin: &mut Pin<Output, PD5>) {
    pin.toggle();
}
fn led6(pin: &mut Pin<Output, PD6>) {
    pin.toggle();
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x02));
    // Enable the INT0 interrupt source.
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let mut pin4 = pins.d4.into_output();
    let mut pin5 = pins.d5.into_output();
    let mut pin6 = pins.d6.into_output();

    // taskをグローバルなスレッド(context/TaskManager)にpushする
    let mut _task1 = TaskControlBlock {
        task_id: 1,
        task_state: TaskState::READY,
        task_priority: &9,
        // 任意のタスクをハンドラにセットする
        task_handler: led4(&mut pin4),
    };

    let mut _task2 = TaskControlBlock {
        task_id: 2,
        task_state: TaskState::READY,
        task_priority: &2,
        task_handler: led5(&mut pin5),
    };

    let mut _task3 = TaskControlBlock {
        task_id: 3,
        task_state: TaskState::READY,
        task_priority: &3,
        task_handler: led6(&mut pin6),
    };

    ufmt::uwriteln!(serial, "os start").void_unwrap();
    unsafe {
        TASKS.push(_task1);
        TASKS.push(_task2);
        TASKS.push(_task3);
    }

    let tmr1: TC1 = dp.TC1;

    unsafe {
        avr_device::interrupt::enable();
    }

    rig_timer(&tmr1, &mut serial);
    loop {
        start_task(&mut serial);
        arduino_hal::delay_ms(100);
        avr_device::asm::sleep();
    }
}

pub const fn calc_overflow(clock_hz: u32, target_hz: u32, prescale: u32) -> u32 {
    /*
    https://github.com/Rahix/avr-hal/issues/75
    reversing the formula F = 16 MHz / (256 * (1 + 15624)) = 4 Hz
     */
    clock_hz / target_hz / prescale - 1
}

pub fn rig_timer<W: uWrite<Error = void::Void>>(tmr1: &TC1, serial: &mut W) {
    /*
     https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
     section 15.11
    */
    use arduino_hal::clock::Clock;

    const ARDUINO_UNO_CLOCK_FREQUENCY_HZ: u32 = arduino_hal::DefaultClock::FREQ;
    const CLOCK_SOURCE: CS1_A = CS1_A::PRESCALE_256;
    let clock_divisor: u32 = match CLOCK_SOURCE {
        CS1_A::DIRECT => 1,
        CS1_A::PRESCALE_8 => 8,
        CS1_A::PRESCALE_64 => 64,
        CS1_A::PRESCALE_256 => 256,
        CS1_A::PRESCALE_1024 => 1024,
        CS1_A::NO_CLOCK | CS1_A::EXT_FALLING | CS1_A::EXT_RISING => {
            uwriteln!(serial, "uhoh, code tried to set the clock source to something other than a static prescaler {}", CLOCK_SOURCE as usize)
                .void_unwrap();
            1
        }
    };

    let ticks = calc_overflow(ARDUINO_UNO_CLOCK_FREQUENCY_HZ, 4, clock_divisor) as u16;
    ufmt::uwriteln!(
        serial,
        "configuring timer output compare register = {}",
        ticks
    )
    .void_unwrap();

    tmr1.tccr1a.write(|w| w.wgm1().bits(0b00));
    tmr1.tccr1b.write(|w| {
        w.cs1()
            //.prescale_256()
            .variant(CLOCK_SOURCE)
            .wgm1()
            .bits(0b01)
    });
    tmr1.ocr1a.write(|w| unsafe { w.bits(ticks) });
    tmr1.timsk1.write(|w| w.ocie1a().set_bit()); //enable this specific interrupt
}
