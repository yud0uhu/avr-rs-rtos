#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::hal::delay;
use arduino_hal::prelude::*;
use avr_device::atmega328p::tc1::tccr1b::CS1_A;
use avr_device::atmega328p::TC1;
use panic_halt as _;
use panic_halt as _;
use ufmt::{uWrite, uwriteln};

static MYGLOBAL: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

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
    pub task_handler: &'a TaskHandler,
}
pub struct TaskControlBlock {
    task_id: u32,
    task_state: TaskState,
    task_priority: &'static usize,
    task_handler: TaskHandler,
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
                priority_stack.push(task_priority);
            }
        };
    }
}

static mut priority_stack: Vec<&usize, 8> = Vec::new();
static mut high_priority_task_id: u32 = 0;

static mut tasks: Vec<TaskControlBlock, 8> = Vec::new();
pub fn context_switch() {
    let running = TaskState::RUNNING;
    let ready = TaskState::READY;
    let top_priority = get_top_priority();
    unsafe {
        for tcb_stack in &tasks {
            let task_id = tcb_stack.task_id;
            let mut task_state = &tcb_stack.task_state;
            let mut task_priority = tcb_stack.task_priority;
            if &top_priority == task_priority {
                unsafe {
                    high_priority_task_id = task_id;
                }
                // TODO: stateの切り替え(→RUNNING)
                task_state = &running;
                // unreachable!();
            } else {
                task_state = &ready;
            }
        }
    }
}

pub fn task_init<W: uWrite<Error = void::Void>>(serial: &mut W) {
    unsafe {
        if priority_stack.is_empty() {
            all_set_task(serial);
        }
    }
}

pub fn start_task<W: uWrite<Error = void::Void>>(serial: &mut W) {
    task_init(serial);
    unsafe {
        if high_priority_task_id == 0 {
            return;
        }
        ufmt::uwriteln!(
            serial,
            "current high task priority task_id= {}",
            high_priority_task_id
        )
        .void_unwrap();
    }
}

pub fn get_top_priority() -> usize {
    let max: usize;
    unsafe {
        // 最も優先順位の高いものを検索する
        match priority_stack.iter().max() {
            Some(n) => max = **n,
            None => unreachable!(),
        };
        // 優先順位の低い順にソートする
        priority_stack.sort_unstable();
        // 優先順位の最も高い要素のインデックスを取り除く
        priority_stack.remove(priority_stack.len() - 1);
        max
    }
}
use avr_device::interrupt::Mutex;
use core::cell::Cell;
use core::sync::atomic;

static TMR_OVERFLOW: atomic::AtomicBool = atomic::AtomicBool::new(false);
#[avr_device::interrupt(atmega328p)]
fn TIMER1_COMPA() {
    // TMR_OVERFLOW.store(true, atomic::Ordering::SeqCst);
    context_switch();
}

fn all_set_task<W: uWrite<Error = void::Void>>(serial: &mut W) {
    unsafe {
        // stackの所有権がtaskに移動しまわないように、参照を借用する
        for task in &tasks {
            let mut task_manager = TaskManager {
                task_control_block: task,
                task_handler: &task.task_handler,
            };
            task_manager.update(serial);
        }
    }
}

// ユーザー定義の関数

// #[derive(Clone, Copy)]
pub struct Calc1 {
    pub width: u32,
    pub height: u32,
}
impl Calc1 {
    pub fn run(&mut self) -> u32 {
        self.width * self.height
    }
}

// #[derive(Clone, Copy)]
pub struct Calc2 {
    pub width: u32,
    pub height: u32,
}
impl Calc2 {
    pub fn run(&mut self) -> u32 {
        self.width * self.height
    }
}
// #[derive(Clone, Copy)]
pub struct TaskHandler {
    pub ans: u32,
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x02));
    // Enable the INT0 interrupt source.
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let mut led = pins.d13.into_output();
    led.set_high();

    // 任意のタスク(縦*横を計算する)を生成
    let mut calc1 = Calc1 {
        width: 30,
        height: 50,
    };
    let mut calc2 = Calc2 {
        width: 10,
        height: 10,
    };
    // taskをグローバルなスレッド(context/TaskManager)にpushする
    let task1 = TaskControlBlock {
        task_id: 1,
        task_state: TaskState::READY,
        task_priority: &9,
        // 任意のタスクをハンドラにセットする
        task_handler: TaskHandler { ans: calc1.run() },
    };

    let task2 = TaskControlBlock {
        task_id: 2,
        task_state: TaskState::READY,
        task_priority: &2,
        task_handler: TaskHandler { ans: calc2.run() },
    };

    let task3 = TaskControlBlock {
        task_id: 3,
        task_state: TaskState::READY,
        task_priority: &3,
        task_handler: TaskHandler { ans: calc2.run() },
    };

    ufmt::uwriteln!(serial, "os start").void_unwrap();
    unsafe {
        tasks.push(task1);
        tasks.push(task2);
        tasks.push(task3);
    }

    let tmr1: TC1 = dp.TC1;

    unsafe {
        avr_device::interrupt::enable();
    }

    // ufmt::uwriteln!(
    //     &mut serial,
    //     "configured timer output compare register = {}",
    //     tmr1.ocr1a.read().bits()
    // )
    // .void_unwrap();
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
