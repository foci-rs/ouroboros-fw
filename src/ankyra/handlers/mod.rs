//! Board-local ankyra `#[klipper_command]` handlers.
//!
//! After Task 24, only `foci` remained here. Task 6 restores a tiny `tmc`
//! shim module to surface `tmc_impedance_test` through the board-local
//! provider while keeping the implementation in `foci-firmware`.

pub mod foci;
pub mod tmc;
