#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::hal::Pins;
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer0Pwm};
use arduino_hal::{prelude::*, Peripherals};
use avr_device::atmega328p::tc1::tccr1b::CS1_A;
use avr_device::atmega328p::TC1;
use panic_halt as _;
use panic_halt as _;
use ufmt::{uWrite, uwriteln};

use core::cell::{self};

use arduino_hal::port::mode::{Analog, Output};
use arduino_hal::port::Pin;

use arduino_hal::hal::port::{PC1, PD3, PD4};

static mut PRIORITY_STACK: Vec<&usize, 8> = Vec::new();

static HIGH_PRIORITY_TASK_ID: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

extern crate avr_device as device;
use device::interrupt::Mutex;

static mut TASKS: Mutex<Vec<TaskControlBlock, 8>> = Mutex::new(Vec::new());

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
        for tcb_stack in unsafe { TASKS.get_mut() } {
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

fn high_priority_task_id() -> u32 {
    avr_device::interrupt::free(|cs| HIGH_PRIORITY_TASK_ID.borrow(cs).get())
}

pub fn start_task<W: uWrite<Error = void::Void>>(
    serial: &mut W,
    pin3: &mut Pin<Output, PD3>,
    pin4: &mut Pin<Output, PD4>,
    _i_monitor: usize,
) {
    task_init(serial);

    let mut _task_id = high_priority_task_id() as usize;

    if _task_id <= 0 {
        return;
    }
    unsafe {
        let mut vec = TASKS.get_mut();
        // (vec[_task_id - 1].task_handler)();

        if vec[_task_id - 1].task_id == 1 {
            Tasks::task_display()
        } else if vec[_task_id - 1].task_id == 2 {
            Tasks::task_pwm(serial, pin3, _i_monitor);
        } else if vec[_task_id - 1].task_id == 3 {
            Tasks::task_relay(serial, pin4);
        }
        ufmt::uwriteln!(
            serial,
            "current high task priority task_id= {}",
            vec[_task_id - 1].task_id
        )
        .void_unwrap();
    }
    // ufmt::uwriteln!(serial, "current high task priority task_id= {}", _task_id).void_unwrap();
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
    for task in unsafe { TASKS.get_mut() } {
        let mut task_manager = TaskManager {
            task_control_block: &task,
            task_handler: task.task_handler,
        };
        task_manager.update(serial);
    }
}

const THRESHOLD: usize = 100;
const RELAY_DELAY: usize = 0;
const PWM_DELAY: usize = 0;
static mut _flg: bool = true;
// static mut _f_coeff_p: f32 = 0.3;
// static mut _fcoeff_I: f32 = 0.4;
// static mut _fcoeff_d: f32 = 2.8;

// // unsafe
// static mut _i_target: usize = 80;
// // static mut _i_pwm: usize = 128;
// static mut _fp_error: f32 = 0.0;
// static mut _fI_error: f32 = 0.0;
// static mut _fd_error: f32 = 0.0;
// static mut _fp_error_previous: f32 = 0.0;
static mut _previous_flag: bool = true;

struct Tasks {}
impl Tasks {
    pub fn led4() {
        arduino_hal::delay_ms(50_u16);
        return Self::led4();
    }
    pub fn led5() {
        arduino_hal::delay_ms(50_u16);
    }

    pub fn led6() {
        arduino_hal::delay_ms(50_u16);
    }

    pub fn task_pwm<W: uWrite<Error = void::Void>>(
        serial: &mut W,
        pin3: &mut Pin<Output, PD3>,
        _i_monitor: usize,
    ) {
        let mut _f_pwm: f32 = 128.0;
        let mut _i_pwm: usize = 128;
        let mut _f_coeff_p: f32 = 0.3;
        let mut _fcoeff_i: f32 = 0.4;
        let mut _fcoeff_d: f32 = 2.8;
        let mut _i_target: usize = 80;
        let mut _fp_error: f32 = 0.0;
        let mut _fI_error: f32 = 0.0;
        let mut _fd_error: f32 = 0.0;
        let mut _fp_error_previous: f32 = 0.0;

        // TODO 255は_i_monitor
        let _i_monitor = _i_monitor;
        _fp_error = _f_coeff_p * (_i_monitor - _i_target) as f32 / 1.5;
        _fI_error = _fcoeff_i * _fp_error;
        _fd_error = _fcoeff_d * (_fp_error - _fp_error_previous);
        _fp_error_previous = _fp_error;
        _f_pwm -= _fp_error + _fI_error + _fd_error;
        _i_pwm = _f_pwm as usize;

        pin3.toggle();

        // if _i_pwm > 255 {
        //     _i_pwm = 255;
        //     pin3.is_set_high();
        // } else if _i_pwm < 0 {
        //     _i_pwm = 0;
        //     pin3.is_set_low();
        // }
    }

    pub fn task_relay<W: uWrite<Error = void::Void>>(serial: &mut W, pin4: &mut Pin<Output, PD4>) {
        unsafe {
            if 128 < THRESHOLD - 1 {
                _flg = true;
            }
            if 128 > THRESHOLD {
                _flg = false;
            }
            if _flg == true {
                pin4.toggle();
                ufmt::uwriteln!(serial, "Relay ON Control",).void_unwrap();
                if _previous_flag == false {
                    _previous_flag = true;
                }
            }
            if _flg == false {
                pin4.toggle();
                ufmt::uwriteln!(serial, "Relay OFF Control",).void_unwrap();
                if _previous_flag == true {
                    _previous_flag = false;
                }
            }
        }
    }

    pub fn task_display() {
        // PWM control
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();

    let mut pins = arduino_hal::pins!(dp);
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x02));
    // Enable the INT0 interrupt source.
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let mut pin1 = pins.a1.into_analog_input(&mut adc);
    let mut pin3 = pins.d3.into_output();
    let mut pin4 = pins.d4.into_output();
    // let mut pin5 = pins.d5.into_output();
    // let pin6 = pins.d6.into_output();

    let _i_monitor: usize = pin1.analog_read(&mut adc).into();

    let task1 = Tasks::task_display();
    let task2 = Tasks::task_pwm(&mut serial, &mut pin3, _i_monitor);
    let task3 = Tasks::task_relay(&mut serial, &mut pin4);

    ufmt::uwriteln!(serial, "os start={}", _i_monitor).void_unwrap();
    // taskをグローバルなスレッド(context/TaskManager)にpushする
    let mut _task1 = TaskControlBlock {
        task_id: 1,
        task_state: TaskState::READY,
        task_priority: &9,
        // 任意のタスクをハンドラにセットする
        task_handler: task1,
    };

    let mut _task2 = TaskControlBlock {
        task_id: 2,
        task_state: TaskState::READY,
        task_priority: &2,
        task_handler: task2,
    };

    let mut _task3 = TaskControlBlock {
        task_id: 3,
        task_state: TaskState::READY,
        task_priority: &3,
        task_handler: task3,
    };

    ufmt::uwriteln!(serial, "os loading").void_unwrap();

    unsafe {
        let mut vec = TASKS.get_mut();

        vec.push(_task1);
        vec.push(_task2);
        vec.push(_task3);
    }

    let tmr1: TC1 = dp.TC1;
    unsafe {
        avr_device::interrupt::enable();
    }

    rig_timer(&tmr1, &mut serial);
    loop {
        start_task(&mut serial, &mut pin3, &mut pin4, _i_monitor);

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
    // TCCR1B |= _BV(CS12);  // 256分周, CTCモード
    // TIMSK1 |= _BV(TOIE1); // オーバーフロー割り込みを許可
    tmr1.ocr1a.write(|w| unsafe { w.bits(ticks) });
    tmr1.timsk1.write(|w| w.ocie1a().set_bit()); //enable this specific interrupt
}
