use arduino_hal::prelude::*;
use avr_device::atmega328p::tc1::tccr1b::CS1_A;
use avr_device::atmega328p::TC1;
use core::cell;
use panic_halt as _;
use panic_halt as _;
use ufmt::{uWrite, uwriteln};

/**
 * 優先順位スタックから最も優先度が高く(1~9で最も数字の大きいもの)設定されているタスクID
 */
pub static HIGH_PRIORITY_TASK_ID: avr_device::interrupt::Mutex<cell::Cell<u32>> =
    avr_device::interrupt::Mutex::new(cell::Cell::new(0));

/**
 * 優先順位スタックから最も優先度が高く(1~9で最も数字の大きいもの)設定されているタスクIDを取得する関数
 */
pub fn high_priority_task_id() -> u32 {
    avr_device::interrupt::free(|cs| HIGH_PRIORITY_TASK_ID.borrow(cs).get())
}
/**
 * 1秒周期のタイマー割り込みをセットする関数。タスクを設定後に宣言して使う。タイマー1のインスタンス変数を第1引数に渡す
 */
pub fn timer_create<W: uWrite<Error = void::Void>>(tmr1: &TC1, serial: &mut W) {
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
    // let ticks = (ARDUINO_UNO_CLOCK_FREQUENCY_HZ / clock_divisor) as u16;
    // ufmt::uwriteln!(
    //     serial,
    //     "configuring timer output compare register = {}",
    //     ticks
    // )
    // .void_unwrap();

    // 0.48ms周期に設定

    let ticks = (ARDUINO_UNO_CLOCK_FREQUENCY_HZ / clock_divisor / 2083) as u16; // 2083 = (1/0.48) *10^6
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
    // 1秒周期の割り込みを設定
    tmr1.ocr1a.write(|w| unsafe { w.bits(ticks) });
    tmr1.timsk1.write(|w| w.ocie1a().set_bit()); // オーバーフロー割り込みを許可
}
