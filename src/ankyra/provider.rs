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
    ],
    replies: [],
    outputs: [],
    enumerations: [
        crate::ankyra::enumerations::Pins,
    ],
    constants: [
        crate::ankyra::constants::CLOCK_FREQ,
        crate::ankyra::constants::ENVELOPE_ABSOLUTE_MARGIN_MREV_S,
        crate::ankyra::constants::ENVELOPE_PROPORTIONAL_DEN,
        crate::ankyra::constants::ENVELOPE_PROPORTIONAL_NUM,
        crate::ankyra::constants::FOCI_VM_ADC_CENTER_COUNTS,
        crate::ankyra::constants::FOCI_VM_ADC_REFERENCE_MILLIVOLTS,
        crate::ankyra::constants::FOCI_VM_DIVIDER_HIGH_OHMS,
        crate::ankyra::constants::FOCI_VM_DIVIDER_LOW_OHMS,
        crate::ankyra::constants::MCU,
        crate::ankyra::constants::RESERVE_PINS_DRV,
        crate::ankyra::constants::RESERVE_PINS_LED,
        crate::ankyra::constants::RESERVE_PINS_SPI,
        crate::ankyra::constants::RESERVE_PINS_STEP,
        crate::ankyra::constants::RESERVE_PINS_SWD,
        crate::ankyra::constants::RESERVE_PINS_TMC,
        crate::ankyra::constants::RESERVE_PINS_USB,
        crate::ankyra::constants::STAGE_B_EVIDENCE_SCHEMA_REVISION,
        crate::ankyra::constants::INTEGRAL_EVIDENCE_SCHEMA_REVISION,
        crate::ankyra::constants::INTEGRAL_MAX_POSITIVE_RUNGS,
        crate::ankyra::constants::INTEGRAL_OBSERVATIONS_PER_DIRECTION,
        crate::ankyra::constants::INTEGRAL_OBSERVATIONS_PER_RUNG,
        crate::ankyra::constants::VELOCITY_LIMIT_EVIDENCE_CAPABILITY,
    ],
}
