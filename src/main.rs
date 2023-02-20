#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::hal::delay;
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer2Pwm};
use arduino_hal::{delay_us, prelude::*};
use avr_device::atmega328p::TC1;
use micromath::F32Ext;
use panic_halt as _;
use ufmt::uwriteln;

use arduino_hal::port::mode::{Output, PwmOutput};
use arduino_hal::port::Pin;

use arduino_hal::hal::port::{PD3, PD4};

mod os;

/**
 * タスクをセットする関数
 */
pub fn create_task(
    _task1: os::tcb::TaskControlBlock,
    _task2: os::tcb::TaskControlBlock,
    _task3: os::tcb::TaskControlBlock,
) {
    unsafe {
        let vec = os::TASKS.get_mut();
        vec.push_unchecked(_task1);
        vec.push_unchecked(_task2);
        vec.push_unchecked(_task3);
    }
}

trait Tasks {
    fn call(&mut self);
    fn u_call(&mut self) -> u8;
    fn init(&mut self);
}

impl Tasks for TaskPwm {
    fn call(&mut self) {
        self.u_call();
    }
    fn u_call(&mut self) -> u8 {
        let mut _i_pwm: u8 = self._i_pwm;
        let mut _i_monitor: u8 = self.values;
        let mut _i_target: u8 = 80;
        let mut _ip_error: u8 = 0;
        let mut _new_i_pwm: u8 = 0;

        if self.values > 255 {
            _new_i_pwm = 255;
            // self.d3.set_duty(_new_i_pwm as u8);
            return 255;
        }

        if _i_monitor > _i_target {
            _ip_error = (_i_monitor - _i_target) / 3;
            _i_pwm -= _ip_error;
            _new_i_pwm = _i_pwm;
            // self.d3.set_duty(_new_i_pwm as u8);
            return _new_i_pwm as u8;
        } else if _i_monitor < _i_target {
            _ip_error = (_i_target - _i_monitor) / 3;
            _i_pwm += _ip_error;
            _new_i_pwm = _i_pwm;
            // self.d3.set_duty(_new_i_pwm as u8);
            return _new_i_pwm as u8;
        }
        return _new_i_pwm as u8;
    }

    fn init(&mut self) {}
}

impl Tasks for TaskRelay {
    fn call(&mut self) {
        let pin4 = &mut self.pin4;
        // let serial = self.serial;

        const THRESHOLD: u8 = 100;
        let mut _flg: bool = true;
        let mut _previous_flag: bool = true;

        if self._i_pwm < THRESHOLD - 1 {
            _flg = true;
        }
        if self._i_pwm > THRESHOLD {
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
    fn u_call(&mut self) -> u8 {
        return 0;
    }
    fn init(&mut self) {}
}

impl Tasks for TaskDisplay {
    fn call(&mut self) {}
    fn init(&mut self) {}
    fn u_call(&mut self) -> u8 {
        return 0;
    }
}
struct TaskPwm {
    values: u8,
    _i_pwm: u8,
    // d3: Pin<PwmOutput<Timer2Pwm>, PD3>,
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

    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let tmr2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    let mut d3 = pins.d3.into_output().into_pwm(&tmr2);
    d3.enable();

    let mut pin4 = pins.d4.into_output();
    pin4.set_high();

    let a1 = pins.a1.into_analog_input(&mut adc);

    let mut value = a1.analog_read(&mut adc);

    if value == 0 {
        value = value + 1
    }
    value = value;

    uwriteln!(&mut serial, "A1: {} ", value).void_unwrap();

    let mut task_pwm = TaskPwm {
        values: value as u8,
        _i_pwm: _i_pwm,
        // d3: d3,
    };

    _i_pwm = task_pwm.u_call();

    d3.set_duty(_i_pwm);

    let mut task_relay = TaskRelay {
        pin4: pin4,
        _i_pwm: _i_pwm,
    };

    // uwriteln!(&mut serial, "MONITOR: {} ", _i_pwm).void_unwrap();

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

    create_task(_task1, _task2, _task3);

    os::os_timer::timer_create(&tmr1, &mut serial);

    os::os_start(&mut serial);

    unsafe {
        avr_device::interrupt::enable();
    }

    loop {
        avr_device::asm::sleep();
    }
}
