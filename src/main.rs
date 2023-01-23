#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::prelude::*;
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer2Pwm};
use avr_device::atmega328p::TC1;
use micromath::F32Ext;
use panic_halt as _;
use ufmt::uwriteln;

use arduino_hal::port::mode::{Output, PwmOutput};
use arduino_hal::port::Pin;

use arduino_hal::hal::port::{PD3, PD4};

mod os;

pub fn task_reload(
    _task1: os::tcb::TaskControlBlock,
    _task2: os::tcb::TaskControlBlock,
    _task3: os::tcb::TaskControlBlock,
) {
    unsafe {
        let vec = os::TASKS.get_mut();
        vec.push(_task1);
        vec.push(_task2);
        vec.push(_task3);
    }
}

trait Tasks {
    fn call(&mut self);
    fn init(&mut self);
}

impl Tasks for TaskPwm {
    fn call(&mut self) {
        let mut _f_pwm: f32 = 128.0;
        let mut _f_coe_ff_p: f32 = 0.3;
        let mut _f_coe_ff_i: f32 = 0.4;
        let mut _f_coe_ff_d: f32 = 2.8;
        let mut _i_target: u16 = 80;
        let mut _fp_error: f32 = 0.0;
        let mut _fi_error: f32 = 0.0;
        let mut _fd_error: f32 = 0.0;
        let mut _fp_error_previous: f32 = 0.0;
        let d3 = &mut self.d3;

        let values = self.values;

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
        // // _f_pwmが小数点以下を持っているため、小数点以下が切り捨てられ、結果が予期しない値になるためpanicする
        // // この問題を防ぐために、round()関数で小数点以下を四捨五入し、f_pwmをu16にキャストする前に整数に変換・0以上255以下に制限する
        // // 以下のようにして、_i_monitor変数に対して_f_pwm.round() as u16から得られた値を四捨五入し、0以上255以下に制限する処理を行う
        // // 1. _i_monitor.max(): _i_monitor変数と_f_pwmの四捨五入した値をu16型にキャストした値を比較し、値の大きい方を_i_monitorに代入
        // // 2. _i_monitor.min(): _i_monitor変数と255を比較し、値の小さい方を_i_monitorに代入
        _i_monitor = _i_monitor.max(_i_monitor.min(_f_pwm.round() as u16));

        d3.set_duty(255);
    }

    fn init(&mut self) {}
}

impl Tasks for TaskRelay {
    fn call(&mut self) {
        let pin4 = &mut self.pin4;
        // let serial = self.serial;
        let _i_pwm = self._i_pwm;

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
                // uwriteln!(serial, "Relay ON Control",).void_unwrap();
            }
        } else if _flg == false {
            pin4.set_low();
            if _previous_flag == true {
                _previous_flag = false;
                // uwriteln!(serial, "Relay OFF Control",).void_unwrap();
            }
        }
    }
    fn init(&mut self) {}
}

impl Tasks for TaskDisplay {
    fn call(&mut self) {}
    fn init(&mut self) {}
}
struct TaskPwm {
    values: u16,
    d3: Pin<PwmOutput<Timer2Pwm>, PD3>,
}

struct TaskRelay {
    pin4: Pin<Output, PD4>,
    _i_pwm: u8,
}

struct TaskDisplay {}

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

    uwriteln!(&mut serial, "A1: {} ", values).void_unwrap();
    // let _i_monitor = 128;
    let mut task_pwm = TaskPwm {
        values: values,
        d3: d3,
    };

    let mut task_relay = TaskRelay {
        pin4: pin4,
        _i_pwm: _i_pwm,
    };

    let mut task_display = TaskDisplay {};

    // taskをグローバルなスレッド(context/TaskManager)にpushする
    let mut _task1 = os::tcb::TaskControlBlock {
        task_id: 1,
        task_state: os::tcb::TaskState::READY,
        task_priority: &9,
        // 任意のタスクをハンドラにセットする
        task_handler: task_display.call(),
    };

    let mut _task2 = os::tcb::TaskControlBlock {
        task_id: 2,
        task_state: os::tcb::TaskState::READY,
        task_priority: &2,
        task_handler: task_pwm.call(),
    };

    let mut _task3 = os::tcb::TaskControlBlock {
        task_id: 3,
        task_state: os::tcb::TaskState::READY,
        task_priority: &3,
        task_handler: task_relay.call(),
    };

    uwriteln!(serial, "os loading").void_unwrap();

    let tmr1: TC1 = dp.TC1;

    task_reload(_task1, _task2, _task3);

    os::os_timer::timer_create(&tmr1, &mut serial);

    os::os_start(&mut serial, values, _i_pwm);

    unsafe {
        avr_device::interrupt::enable();
    }

    loop {
        avr_device::asm::sleep();
    }
}
