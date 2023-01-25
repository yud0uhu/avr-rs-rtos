/*!
 * ベアメタルでアナログピン(A1)の入力からデジタルピン(PD3)のON/OFFとリレー(PD4)のスイッチング制御を行う
 */
#![no_std]
#![no_main]

use arduino_hal::hal::port::{PD3, PD4};
use arduino_hal::port::mode::{Output, PwmOutput};
use arduino_hal::port::Pin;
use arduino_hal::prelude::*;
use panic_halt as _;

use arduino_hal::adc;

use arduino_hal::hal::Pins;
use arduino_hal::simple_pwm::{IntoPwmPin, Prescaler, Timer0Pwm, Timer2Pwm};

use micromath::F32Ext;
use ufmt::{uWrite, uwriteln};

#[arduino_hal::entry]
fn main() -> ! {
    let mut _i_pwm: u8 = 128;
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());

    let (vbg, gnd, tmp) = (
        adc.read_blocking(&adc::channel::Vbg),
        adc.read_blocking(&adc::channel::Gnd),
        adc.read_blocking(&adc::channel::Temperature),
    );
    ufmt::uwriteln!(&mut serial, "Vbandgap: {}", vbg).void_unwrap();
    ufmt::uwriteln!(&mut serial, "Ground: {}", gnd).void_unwrap();
    ufmt::uwriteln!(&mut serial, "Temperature: {}", tmp).void_unwrap();

    let a0 = pins.a0.into_analog_input(&mut adc);
    let a1 = pins.a1.into_analog_input(&mut adc);
    let a2 = pins.a2.into_analog_input(&mut adc);
    let a3 = pins.a3.into_analog_input(&mut adc);
    let a4 = pins.a4.into_analog_input(&mut adc);
    let a5 = pins.a5.into_analog_input(&mut adc);

    let tmr2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    let mut d3 = pins.d3.into_output().into_pwm(&tmr2);
    d3.enable();

    let mut pin4 = pins.d4.into_output();
    pin4.set_high();

    loop {
        let mut values = [
            a0.analog_read(&mut adc),
            a1.analog_read(&mut adc),
            a2.analog_read(&mut adc),
            a3.analog_read(&mut adc),
            a4.analog_read(&mut adc),
            a5.analog_read(&mut adc),
        ];

        for (i, v) in values.iter().enumerate() {
            ufmt::uwrite!(&mut serial, "A{}: {} ", i, v).void_unwrap();
        }

        ufmt::uwriteln!(&mut serial, "").void_unwrap();

        arduino_hal::delay_ms(5);

        if values[1] == 0 {
            values[1] = values[1] + 1
        }

        // アナログピンから読み取れる電圧が微弱すぎるため暫定的に*100
        // 0.48ミリ秒

        _i_pwm = task_pwm(&mut serial, values[1], _i_pwm as u16);

        ufmt::uwriteln!(&mut serial, "{}", _i_pwm).void_unwrap();

        d3.set_duty(_i_pwm);
        arduino_hal::delay_ms(5);

        task_relay(&mut serial, &mut pin4, _i_pwm);
    }
}

pub fn task_pwm<W: uWrite<Error = void::Void>>(serial: &mut W, values: u16, _i_pwm: u16) -> u8 {
    let mut _i_pwm: u16 = _i_pwm;
    let mut _i_monitor: u16 = values;
    let mut _i_target: u16 = 80;
    let mut _ip_error: u16 = 0;
    let mut _new_i_pwm: u16 = 0;

    if values > 255 {
        return 255;
    }
    ufmt::uwriteln!(serial, "i_monitor={}", _i_monitor).void_unwrap();

    if _i_monitor > _i_target {
        _ip_error = (_i_monitor - _i_target) / 3;
        ufmt::uwriteln!(serial, " _ip_error???={}", _ip_error).void_unwrap();
        _i_pwm -= _ip_error;
        ufmt::uwriteln!(serial, " _i_pwm???={}", _i_pwm).void_unwrap();
        _new_i_pwm = _i_pwm;
        return _new_i_pwm as u8;
    } else if _i_monitor < _i_target {
        _ip_error = (_i_target - _i_monitor) / 3;
        ufmt::uwriteln!(serial, " _ip_error???={}", _ip_error).void_unwrap();
        _i_pwm += _ip_error;
        ufmt::uwriteln!(serial, " _i_pwm???={}", _i_pwm).void_unwrap();
        _new_i_pwm = _i_pwm;
        return _new_i_pwm as u8;
    }

    // _f_pwmが小数点以下を持っているため、小数点以下が切り捨てられ、結果が予期しない値になるためpanicする
    // この問題を防ぐために、round()関数で小数点以下を四捨五入し、f_pwmをu16にキャストする前に整数に変換する

    return _i_pwm as u8;
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
    }
    if _flg == false {
        pin4.set_low();
        if _previous_flag == true {
            _previous_flag = false;
            ufmt::uwriteln!(serial, "Relay OFF Control",).void_unwrap();
        }
    }
}
