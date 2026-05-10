//! Board-level static state, decomposed by subsystem.
//!
//! Each submodule holds only the board-local statics (RTIC channel senders,
//! hardware-capacity queues, HardwareStepper handles). Cross-priority
//! atomics and the TMC disable signal have moved to `foci_firmware::state::*`.

pub mod stepper;
pub mod tmc;
pub mod trace;
