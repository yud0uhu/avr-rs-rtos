#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::slice::SliceIndex;

// use arduino_hal::port::mode::Output;
// use arduino_hal::port::Pin;
use arduino_hal::prelude::*;
// use avr_device::atmega328p::tc1::tccr1b::CS1_A;
// use avr_device::atmega328p::TC1;
// use core::mem;
use panic_halt as _;
use ufmt::{uWrite};
use panic_halt as _;
// use arduino_hal::port::{mode, Pin};
// use either::*;

pub trait GlobalLog: Sync {
    fn log(&self, address: u8);
}

use heapless::Vec; // fixed capacity `std::Vec`

enum TaskState {
    RUNNING,
    READY,
    SUSPEND
}

#[derive(Clone, Copy)]
pub struct Calc {
    pub width: u32,
    pub height: u32,
}
impl Calc {
    pub fn run(&mut self) -> u32 {
        self.width * self.height
    }
}

pub struct TaskManager<'a> {
    pub task_control_block: &'a TaskControlBlock,
    pub task_handler: &'a TaskHandler,
}
pub struct TaskControlBlock {
    task_id: u32,
    task_state: TaskState,
    task_priority: &'static usize,
}

#[derive (Clone, Copy)]
pub struct TaskHandler {
    pub calc: u32,
}

impl TaskManager<'_> {
    pub fn update<W: uWrite<Error = void::Void>>(&mut self,serial: &mut W) {
        {
            let task_control_block: &TaskControlBlock = &self.task_control_block;
            let serial: &mut W = serial;
            let task_id:&u32 = &task_control_block.task_id;
            // let task_state = &task_control_block.task_state;
            let task_priority= &task_control_block.task_priority;
            ufmt::uwriteln!(
                serial,
                "push task_id = {}",
                task_id
            )
            .void_unwrap();
            unsafe {
                priority_stack.push(task_priority);
            }
        };
    }
}

static mut priority_stack: Vec<&usize, 8> = Vec::new();
static mut high_priority_task_id: u32 = 0;
static mut stack: Vec<TaskControlBlock, 8> = Vec::new();

pub fn ContextSwitch() {
    let running= TaskState::RUNNING;
    let ready = TaskState::READY;
    let top_priority = GetTopPriority();
    unsafe {
        for tcb_stack in &stack {
            let mut task_id = tcb_stack.task_id;
            let mut task_state = &tcb_stack.task_state;
            let mut task_priority = tcb_stack.task_priority;
            if &top_priority == task_priority {
                unsafe {
                    high_priority_task_id = task_id;
                }
                // TODO: stateの切り替え(→RUNNING)
                task_state= &running;
                // unreachable!();
            } else {
                task_state= &ready;
            }
        }
    }
}
pub fn StartTask<W: uWrite<Error = void::Void>>(serial: &mut W,methods: u32){
    // NOTE: StartTaskはソフトウェア側で設定する。ここで任意のタスクのメソッドを実行
    ufmt::uwriteln!(
        serial,
        "high task priority methods= {}",
        methods
    )
    .void_unwrap();

    ufmt::uwriteln!(
        serial,
        "high task priority task_id= {}",
        unsafe {
            high_priority_task_id
        }
    )
    .void_unwrap();
}

pub fn GetTopPriority() -> usize {
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
        priority_stack.remove(priority_stack.len()-1);
        max
    }
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_OVF() {
    ContextSwitch();
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x02));
    // Enable the INT0 interrupt source.
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);    
    
    // taskをグローバルなスレッド(context/TaskManager)にpushする
    let task1 = TaskControlBlock {
        task_id: 1,
        task_state: TaskState::READY,
        task_priority: &9
    };
    let task2 = TaskControlBlock {
        task_id: 2,
        task_state: TaskState::READY,
        task_priority: &2
    };
    
    // 任意のタスク(縦*横を計算する)を生成
    let mut calc = Calc { width: 30, height: 50 };
    // 任意のタスクをハンドラにセットする
    let task_handler = TaskHandler { calc: calc.run() };

    unsafe{
        stack.push(task1);
        stack.push(task2);
        // stackの所有権がxに移動しまわないように、参照を借用する
        for x in &stack {
            let mut task_manager = TaskManager {
                task_control_block: x,
                task_handler: &task_handler,
            };
            task_manager.update(&mut serial);
        }
    }
    let mut led = pins.d13.into_output();
    led.set_high();
    loop {
        led.toggle();
        arduino_hal::delay_ms(800);
        ContextSwitch();
        StartTask(&mut serial,task_handler.calc);
        unsafe { 
            avr_device::interrupt::enable();
        };
    }
}