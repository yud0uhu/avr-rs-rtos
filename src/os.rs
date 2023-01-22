extern crate avr_device as device;
use device::interrupt::Mutex;
use panic_halt as _;

use arduino_hal::{prelude::*, Delay};
use panic_halt as _;
use ufmt::{uWrite, uwriteln};

pub mod os_timer;
pub mod tcb;

pub static mut TASKS: Mutex<tcb::Vec<tcb::TaskControlBlock, 8>> = Mutex::new(tcb::Vec::new());

pub trait GlobalLog: Sync {
    fn log(&self, address: u8);
}

pub fn context_switch() {
    let running = tcb::TaskState::RUNNING;
    let ready = tcb::TaskState::READY;
    let top_priority = get_top_priority();

    avr_device::interrupt::free(|cs| {
        for tcb_stack in unsafe { TASKS.get_mut() } {
            let task_id = tcb_stack.task_id;
            let mut _task_state = &tcb_stack.task_state;
            let task_priority = tcb_stack.task_priority;
            if task_priority == &top_priority {
                avr_device::interrupt::free(|cs| {
                    os_timer::HIGH_PRIORITY_TASK_ID.borrow(cs).set(task_id);
                });
                _task_state = &running;
            } else {
                _task_state = &ready;
            }
        }
    });
}

pub fn task_init<W: uWrite<Error = void::Void>>(serial: &mut W) {
    unsafe {
        if tcb::PRIORITY_STACK.is_empty() {
            all_set_task(serial);
        }
    }
}

pub fn task_reload(
    _task1: tcb::TaskControlBlock,
    _task2: tcb::TaskControlBlock,
    _task3: tcb::TaskControlBlock,
) {
    unsafe {
        let vec = TASKS.get_mut();
        vec.push(_task1);
        vec.push(_task2);
        vec.push(_task3);
    }
}

static mut MAX_TACK_ID: usize = 3;
static mut COUNT: usize = 0;
pub fn os_start<W: uWrite<Error = void::Void>>(serial: &mut W, _i_monitor: u16, _i_pwm: u8) {
    task_init(serial);

    while unsafe { COUNT < MAX_TACK_ID } {
        // 割り込みハンドラ
        context_switch();
        let mut _task_id = os_timer::high_priority_task_id() as usize;
        unsafe {
            let vec = TASKS.get_mut();

            uwriteln!(
                serial,
                "current high task priority task_id= {}",
                vec[_task_id - 1].task_id
            )
            .void_unwrap();

            if vec[_task_id - 1].task_state == tcb::TaskState::RUNNING {
                vec[_task_id - 1].task_handler;
            }
        }
        os_delay(1000);

        unsafe {
            COUNT += 1;
            if COUNT >= MAX_TACK_ID {
                break;
            }
        }
    }
}

pub fn os_delay(ms: u16) {
    Delay::new().delay_ms(ms)
}

/**
 * PRIORITY_STACKから最大値を取得し、その値を返す関数
 * PRIORITY_STACKが空の場合にはNoneが返されるため、max = **nの行が実行されず、maxは初期値である1を返す
 * */
pub fn get_top_priority() -> usize {
    // match文の返り値を参照で取得する
    let max: usize;
    unsafe {
        // 最も優先順位の高いものを検索する
        match tcb::PRIORITY_STACK.iter().max() {
            Some(n) => max = **n,
            None => unreachable!(),
        };
        // 優先順位の低い順にソートする
        tcb::PRIORITY_STACK.sort_unstable();
        // 優先順位の最も高い要素のインデックスを取り除く
        tcb::PRIORITY_STACK.remove(tcb::PRIORITY_STACK.len() - 1);

        max
    }
}

fn all_set_task<W: uWrite<Error = void::Void>>(serial: &mut W) {
    // stackの所有権がtaskに移動しまわないように、参照を借用する
    for task in unsafe { TASKS.get_mut() } {
        let mut task_manager = tcb::TaskManager {
            task_control_block: &task,
            task_handler: task.task_handler,
        };
        task_manager.update(serial);
    }
}
