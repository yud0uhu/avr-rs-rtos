#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

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
    task_priority: &'static u16,
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
            let task_id = &task_control_block.task_id;
            let task_state = &task_control_block.task_state;
            let task_priority = &task_control_block.task_priority;
            ufmt::uwriteln!(
                serial,
                "push task_id = {}",
                task_id
            )
            .void_unwrap();
            // let mut priority_stack: Vec<&u16, 8> = Vec::new();
            unsafe {
                priority_stack.push(&task_priority);
            }
        };
    }
}

static mut priority_stack: Vec<&u16, 8> = Vec::new();
static mut high_priority_task_id: u32 = 0;

pub fn ContextSwitch(stack: &Vec<TaskControlBlock, 8>) {
    let top_priority = GetTopPriority();
    for tcb_stack in stack {
        if top_priority == tcb_stack.task_priority {
            unsafe {
                priority_stack.push(&top_priority);
                high_priority_task_id = tcb_stack.task_id;
                // TODO: stateの切り替え(→RUNNING)
            }
        }
    }
}
pub fn StartTask<W: uWrite<Error = void::Void>>(serial: &mut W,methods: u32){
    // NOTE: StartTaskはソフトウェア側で設定する。ここで任意のタスクのメソッドを実行
    ufmt::uwriteln!(
        serial,
        "high task priority methods= {}",
        methods as u16
    )
    .void_unwrap();

    ufmt::uwriteln!(
        serial,
        "high task priority task_id= {}",
        unsafe {
            high_priority_task_id as u16
        }
    )
    .void_unwrap();
}

pub fn GetTopPriority() -> &'static u16 {
    let max: &u16;
    unsafe {
        match priority_stack.iter().max() {
            Some(n) => max = *n,
            None => unreachable!(),
        }
    }
    max
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
    
    // taskをグローバルなスレッド(context/TaskManager)にpushする
    let task1 = TaskControlBlock {
        task_id: 1,
        task_state: TaskState::READY,
        task_priority: &9
    };
    let task2 = TaskControlBlock {
        task_id: 2,
        task_state: TaskState::RUNNING,
        task_priority: &2
    };
    
    // 任意のタスク(縦*横を計算する)を生成
    let mut calc = Calc { width: 30, height: 50 };
    // 任意のタスクをハンドラにセットする
    let task_handler = TaskHandler { calc: calc.run() };

    let mut stack: Vec<TaskControlBlock, 8> = Vec::new();
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
    ContextSwitch(&stack);
    StartTask(&mut serial,task_handler.calc);
    
    loop {
        unsafe { 
            avr_device::interrupt::enable();
         };
    }
}