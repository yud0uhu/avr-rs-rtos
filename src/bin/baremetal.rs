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

    let mut tmr2 = Timer2Pwm::new(dp.TC2, Prescaler::Prescale64);
    let mut d3 = pins.d3.into_output().into_pwm(&tmr2);
    d3.enable();

    let mut pin4 = pins.d4.into_output();
    pin4.set_high();

    loop {
        let values = [
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

        arduino_hal::delay_ms(1000);

        _i_pwm = task_pwm(values[1]);
        ufmt::uwriteln!(&mut serial, "{}", _i_pwm).void_unwrap();
        d3.set_duty(_i_pwm);
        arduino_hal::delay_ms(1000);

        if _i_pwm < 0 || _i_pwm > 255 {
            panic!("values is out of range");
        }
        task_relay(&mut serial, &mut pin4, _i_pwm);
    }
}

pub fn task_pwm(values: u16) -> u8 {
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

    return _i_monitor as u8;
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
