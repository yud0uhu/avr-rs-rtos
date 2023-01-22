#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::pac::tc0::tccr0a::WGM0_A;
use arduino_hal::pac::tc0::tccr0b::CS0_A;
use arduino_hal::pac::TC0;
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer0Pwm, Timer2Pwm};
use arduino_hal::{prelude::*, Peripherals};
use avr_device::atmega328p::tc1::tccr1b::CS1_A;
use avr_device::atmega328p::TC1;
use core::cell::{self};
use micromath::F32Ext;
use panic_halt as _;
use panic_halt as _;
use ufmt::{uWrite, uwriteln};

use arduino_hal::port::mode::{Analog, Output, PwmOutput};
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
    pin4: &mut Pin<Output, PD4>,
    d3: &mut Pin<PwmOutput<Timer2Pwm>, PD3>,
    _i_monitor: u16,
    _i_pwm: u8,
) {
    task_init(serial);

    let mut _task_id = high_priority_task_id() as usize;

    if _task_id <= 0 {
        return;
    }
    unsafe {
        let vec = TASKS.get_mut();
        // (vec[_task_id - 1].task_handler)();

        // ufmt::uwriteln!(
        //     serial,
        //     "current high task priority task_id= {}",
        //     vec[_task_id - 1].task_id
        // )
        // .void_unwrap();
        if vec[_task_id - 1].task_id == 1 {
            Tasks::task_display()
        } else if vec[_task_id - 1].task_id == 2 {
            Tasks::task_pwm(_i_monitor, d3);
        } else if vec[_task_id - 1].task_id == 3 {
            Tasks::task_relay(serial, pin4, _i_pwm);
        }
    }
    ufmt::uwriteln!(serial, "current high task priority task_id= {}", _task_id).void_unwrap();
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

// 割り込みハンドラ
// #[avr_device::interrupt(atmega328p)]
unsafe fn TIMER1_COMPA() {
    // タスクの切り替え処理
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
struct Tasks {}
impl Tasks {
    pub fn task_pwm(values: u16, d3: &mut Pin<PwmOutput<Timer2Pwm>, PD3>) {
        let mut _f_pwm: f32 = 128.0;
        let mut _f_coe_ff_p: f32 = 0.3;
        let mut _f_coe_ff_i: f32 = 0.4;
        let mut _f_coe_ff_d: f32 = 2.8;
        let mut _i_target: u16 = 80;
        let mut _fp_error: f32 = 0.0;
        let mut _fi_error: f32 = 0.0;
        let mut _fd_error: f32 = 0.0;
        let mut _fp_error_previous: f32 = 0.0;

        if values < 0 || values > 255 {
            panic!("values is out of range");
        }
        let mut _i_monitor: u16 = values;
        if _i_monitor > _i_target {
            _fp_error = _f_coe_ff_p * (_i_monitor - _i_target) as f32 / 1.5;
        } else {
            // _i_monitor<_i_targetのときに、_fp_errorが負の値になり、その後の計算結果も負の値になり、最終的に_f_pwmが負の値になってしまいフリーズするため_i_monitor-_i_target>0にする
            _fp_error = _f_coe_ff_p * (_i_target - _i_monitor) as f32 / 1.5;
        }
        _fi_error = _f_coe_ff_i * _fp_error;
        _fd_error = _f_coe_ff_d * (_fp_error - _fp_error_previous);
        _fp_error_previous = _fp_error;
        _f_pwm -= _fp_error + _fi_error + _fd_error;
        // _f_pwmが小数点以下を持っているため、小数点以下が切り捨てられ、結果が予期しない値になるためpanicする
        // この問題を防ぐために、round()関数で小数点以下を四捨五入し、f_pwmをu16にキャストする前に整数に変換する
        _i_monitor = _i_monitor.max(_i_monitor.min(_f_pwm.round() as u16));

        d3.set_duty(_i_monitor as u8);
    }

    pub fn task_relay<W: uWrite<Error = void::Void>>(
        serial: &mut W,
        pin4: &mut Pin<Output, PD4>,
        _i_pwm: u8,
    ) {
        const THRESHOLD: u8 = 100;
        let mut _flg: bool = true;
        let mut _previous_flag: bool = true;

        if _i_pwm < THRESHOLD - 1 {
            _flg = true;
        }
        if _i_pwm > THRESHOLD {
            _flg = false;
        }
        if _flg == true {
            pin4.set_high();
            if _previous_flag == false {
                _previous_flag = true;
                ufmt::uwriteln!(serial, "Relay ON Control",).void_unwrap();
            }
        } else if _flg == false {
            pin4.set_low();
            if _previous_flag == true {
                _previous_flag = false;
                ufmt::uwriteln!(serial, "Relay OFF Control",).void_unwrap();
            }
        }
    }

    pub fn task_display() {

        // PWM control
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let mut _i_pwm: u8 = 128;
    let dp = arduino_hal::Peripherals::take().unwrap();

    let mut pins = arduino_hal::pins!(dp);
    dp.EXINT.eicra.modify(|_, w| w.isc0().bits(0x02));
    // Enable the INT0 interrupt source.
    dp.EXINT.eimsk.modify(|_, w| w.int0().set_bit());

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let mut tmr2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    let mut d3 = pins.d3.into_output().into_pwm(&tmr2);
    d3.enable();

    let mut pin4 = pins.d4.into_output();
    pin4.set_high();

    let a1 = pins.a1.into_analog_input(&mut adc);
    let values: u16 = a1.analog_read(&mut adc);

    ufmt::uwrite!(&mut serial, "A1: {} ", values).void_unwrap();
    let _i_monitor = 128;

    // taskをグローバルなスレッド(context/TaskManager)にpushする
    let mut _task1 = TaskControlBlock {
        task_id: 1,
        task_state: TaskState::READY,
        task_priority: &9,
        // 任意のタスクをハンドラにセットする
        task_handler: Tasks::task_display(),
    };

    let mut _task2 = TaskControlBlock {
        task_id: 2,
        task_state: TaskState::READY,
        task_priority: &2,
        task_handler: Tasks::task_pwm(values, &mut d3),
    };

    let mut _task3 = TaskControlBlock {
        task_id: 3,
        task_state: TaskState::READY,
        task_priority: &3,
        task_handler: Tasks::task_relay(&mut serial, &mut pin4, _i_pwm),
    };

    ufmt::uwriteln!(serial, "os loading").void_unwrap();

    unsafe {
        let vec = TASKS.get_mut();
        vec.push(_task1);
        vec.push(_task2);
        vec.push(_task3);
    }

    let tmr1: TC1 = dp.TC1;

    rig_timer(&tmr1, &mut serial);

    start_task(&mut serial, &mut pin4, &mut d3, values, _i_pwm);

    unsafe {
        avr_device::interrupt::enable();
    }

    loop {
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

    // 1秒周期に設定
    let ticks = (ARDUINO_UNO_CLOCK_FREQUENCY_HZ / clock_divisor) as u16;
    // let ticks = calc_overflow(ARDUINO_UNO_CLOCK_FREQUENCY_HZ, 4, clock_divisor) as u16;
    // ufmt::uwriteln!(
    //     serial,
    //     "configuring timer output compare register = {}",
    //     ticks
    // )
    // .void_unwrap();

    tmr1.tccr1a.write(|w| w.wgm1().bits(0b00));
    tmr1.tccr1b.write(|w| {
        w.cs1()
            //.prescale_256()
            .variant(CLOCK_SOURCE)
            .wgm1()
            .bits(0b01)
    });
    // 1秒周期の割り込みを設定
    tmr1.ocr1a.write(|w| unsafe { w.bits(ticks) });
    tmr1.timsk1.write(|w| w.ocie1a().set_bit()); // オーバーフロー割り込みを許可
}
