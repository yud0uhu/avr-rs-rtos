use panic_halt as _;
use panic_halt as _;
use ufmt::uWrite;

#[derive(PartialEq)]
pub enum TaskState {
    RUNNING,
    READY,
    SUSPEND,
}

pub static mut PRIORITY_STACK: Vec<&usize, 8> = Vec::new();

pub use heapless::Vec; // fixed capacity `std::Vec`

use panic_halt as _;
pub struct TaskManager<'a> {
    pub task_control_block: &'a TaskControlBlock,
    pub task_handler: (),
}
pub struct TaskControlBlock {
    pub task_id: u32,
    pub task_state: TaskState,
    pub task_priority: &'static usize,
    pub task_handler: (),
}

impl TaskManager<'_> {
    pub fn update<W: uWrite<Error = void::Void>>(&mut self, serial: &mut W) {
        {
            let task_control_block: &TaskControlBlock = &self.task_control_block;
            let _serial: &mut W = serial;
            let _task_id: &u32 = &task_control_block.task_id;
            // let task_state = &task_control_block.task_state;
            let task_priority = &task_control_block.task_priority;
            // uwriteln!(serial, "new push task_id = {}", task_priority).void_unwrap();
            unsafe {
                /*
                 * DOC: https://docs.rs/arrayvec/0.6.0/arrayvec/struct.ArrayVec.html#method.push_unchecked
                 * Push element to the end of the vector without checking the capacity.
                 * It is up to the caller to ensure the capacity of the vector is sufficiently large.
                 * This method uses debug assertions to check that the arrayvec is not full.
                 */
                PRIORITY_STACK.push_unchecked(task_priority);
            }
        };
    }
}
