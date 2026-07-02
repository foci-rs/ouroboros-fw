//! Board-specific ankyra provider for Ouroboros. Composed with
//! `foci_firmware::ankyra::provider::FOCI_FIRMWARE_PROVIDER` inside
//! the `foci_ankyra_config!` invocation in `main.rs` via
//! `ankyra_reexport_provider!`.
//!
//! This provider carries only items that can't live in the shared
//! crate:
//! - `foci_tune` — emits replies through the board-local ankyra sender
//!   (`crate::_ankyra_config::Sender`) directly; cannot be dispatched
//!   through a capability trait.
//! - `Pins` enumeration — the physical pin set depends on the MCU / PCB.
//! - Board-specific constants: `CLOCK_FREQ`, `MCU`, and the
//!   `RESERVE_PINS_*` family describing which physical pins are
//!   unavailable to printer.cfg.

ankyra::ankyra_provider! {
    name: BOARD_PROVIDER,
    commands: [
        crate::ankyra::handlers::foci::foci_stepper_exec_stats,
        crate::ankyra::handlers::foci::foci_stepper_perf_stats,
        crate::ankyra::handlers::foci::foci_stepper_stats,
        crate::ankyra::handlers::foci::foci_stepper_stop_stats,
        crate::ankyra::handlers::foci::foci_stepper_timing_stats,
        crate::ankyra::handlers::foci::foci_tune,
        crate::ankyra::handlers::foci::tmc_high_rate_capture_test,
    ],
    replies: [],
    outputs: [],
    enumerations: [
        crate::ankyra::enumerations::Pins,
    ],
    constants: [
        crate::ankyra::constants::CLOCK_FREQ,
        crate::ankyra::constants::MCU,
        crate::ankyra::constants::RESERVE_PINS_DRV,
        crate::ankyra::constants::RESERVE_PINS_LED,
        crate::ankyra::constants::RESERVE_PINS_SPI,
        crate::ankyra::constants::RESERVE_PINS_STEP,
        crate::ankyra::constants::RESERVE_PINS_SWD,
        crate::ankyra::constants::RESERVE_PINS_TMC,
        crate::ankyra::constants::RESERVE_PINS_USB,
    ],
}
