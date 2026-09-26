//! Protocol-shape items and dispatch surfaces for the ankyra Klipper
//! protocol framework.
//!
//! After Task 24, owns only:
//! - `constants` — board-specific items: `CLOCK_FREQ`, `MCU`, `RESERVE_PINS_*`.
//!   Firmware-wide constants are accessed directly from `foci_firmware::ankyra::constants`.
//! - `enumerations::Pins` — the physical pin set depends on the MCU / PCB.
//! - `handlers::foci` — `foci_tune` and the stepper execution counters; all
//!   other handlers live in foci-firmware and are dispatched via
//!   `FOCI_FIRMWARE_PROVIDER` in `foci_ankyra_config!`.
//! - `provider::BOARD_PROVIDER` — composes board-specific items only.
//!
//! The `ankyra_config!` invocation stays at the crate root (`main.rs`)
//! because ankyra requires it there.

pub mod constants;
pub mod enumerations;
pub mod handlers;
pub mod provider;
pub(crate) mod replies;
