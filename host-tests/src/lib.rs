#![no_std]
#![allow(dead_code)]

mod board_config {
    pub(crate) const TMC_CHANNEL_COUNT: u8 = 2;
}

#[path = "../../src/state/stepper.rs"]
mod stepper;
