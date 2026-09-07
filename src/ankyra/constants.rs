//! `#[klipper_constant]` declarations registered into the binary's
//! data dictionary `config` section. Consumed by the `constants: [...]`
//! list in `src/ankyra/provider.rs`.
//!
//! All constant identifiers are SCREAMING_SNAKE_CASE; ankyra preserves
//! them verbatim as wire names (per `pascal_to_snake` passthrough rule).
//!
//! Several constants below (MCU, STEPPER_STEP_BOTH_EDGE, RECEIVE_WINDOW,
//! STATS_SUMSQ_BASE, RESERVE_PINS_*) are only consumed over the wire and
//! are never referenced from Rust — the `#[allow(dead_code)]` suppresses
//! the unused-const lint without masking genuinely dead code elsewhere.

use ankyra::klipper_constant;

use crate::board_config;

#[klipper_constant]
pub const CLOCK_FREQ: u32 = 260_000_000;

#[klipper_constant]
pub const ENVELOPE_PROPORTIONAL_NUM: u32 = 3;
const _: () = assert!(
    ENVELOPE_PROPORTIONAL_NUM
        == foci_firmware::commissioning::outer::velocity::sweep_plan::ENVELOPE_PROPORTIONAL_NUM
);

#[klipper_constant]
pub const ENVELOPE_PROPORTIONAL_DEN: u32 = 2;
const _: () = assert!(
    ENVELOPE_PROPORTIONAL_DEN
        == foci_firmware::commissioning::outer::velocity::sweep_plan::ENVELOPE_PROPORTIONAL_DEN
);

#[klipper_constant]
pub const ENVELOPE_ABSOLUTE_MARGIN_MREV_S: u32 = 2_000;
const _: () = assert!(
    ENVELOPE_ABSOLUTE_MARGIN_MREV_S
        == foci_firmware::commissioning::outer::velocity::sweep_plan::ENVELOPE_ABSOLUTE_MARGIN_MREV_S
);

#[klipper_constant]
pub const PROPORTIONAL_EVIDENCE_SCHEMA_REVISION: u32 = 14;
const _: () = assert!(
    PROPORTIONAL_EVIDENCE_SCHEMA_REVISION
        == foci_firmware::commissioning::outer::velocity::combined_plan::PROPORTIONAL_PLAN_SCHEMA_REVISION
            as u32
);

#[klipper_constant]
pub const INTEGRAL_EVIDENCE_SCHEMA_REVISION: u32 = 13;
const _: () = assert!(
    INTEGRAL_EVIDENCE_SCHEMA_REVISION
        == foci_firmware::commissioning::outer::velocity::combined_plan::INTEGRAL_PLAN_SCHEMA_REVISION
            as u32
);

#[klipper_constant]
pub const VELOCITY_LIMIT_EVIDENCE_CAPABILITY: u32 = 0;
const _: () = assert!(
    VELOCITY_LIMIT_EVIDENCE_CAPABILITY
        == foci_firmware::commissioning::outer::velocity::VELOCITY_LIMIT_EVIDENCE_CAPABILITY_BEST_EFFORT
);

#[klipper_constant]
pub const INTEGRAL_MAX_POSITIVE_RUNGS: u32 = 30;
const _: () = assert!(
    INTEGRAL_MAX_POSITIVE_RUNGS
        == foci_firmware::commissioning::outer::velocity::integral_plan::INTEGRAL_MAX_POSITIVE_RUNGS
            as u32
);

#[klipper_constant]
pub const INTEGRAL_OBSERVATIONS_PER_DIRECTION: u32 = 4;
const _: () = assert!(
    INTEGRAL_OBSERVATIONS_PER_DIRECTION
        == foci_firmware::commissioning::outer::velocity::integral_plan::INTEGRAL_OBSERVATIONS_PER_DIRECTION
            as u32
);

#[klipper_constant]
pub const INTEGRAL_OBSERVATIONS_PER_RUNG: u32 = 8;
const _: () = assert!(
    INTEGRAL_OBSERVATIONS_PER_RUNG
        == foci_firmware::commissioning::outer::velocity::integral_plan::INTEGRAL_OBSERVATIONS_PER_RUNG
);

#[allow(dead_code)]
#[klipper_constant]
pub const MCU: &str = "stm32h723xx";

// ---- Reserved pin constants ----
// Tells the Klipper host which pins are firmware-owned and must not be
// assigned to user-facing config sections (steppers, heaters, fans, etc.).

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_USB: &str = "PA9,PA11,PA12";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_SPI: &str = "PB13,PB14,PB15,PD0,PD2";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_TMC: &str = "PD0,PD1,PD2,PD6,PE2,PA2";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_LED: &str = "";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_SWD: &str = "PA13,PA14";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_DRV: &str = "";

/// STEP/DIR GPIO pins for the two TMC4671 hardware step/dir channels.
#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_STEP: &str = "PD4,PD3,PC12,PC11";

#[klipper_constant]
pub const FOCI_VM_DIVIDER_HIGH_OHMS: u32 = 71_500;
const _: () = assert!(FOCI_VM_DIVIDER_HIGH_OHMS == board_config::VM_DIVIDER_HIGH_OHMS);

#[klipper_constant]
pub const FOCI_VM_DIVIDER_LOW_OHMS: u32 = 1_500;
const _: () = assert!(FOCI_VM_DIVIDER_LOW_OHMS == board_config::VM_DIVIDER_LOW_OHMS);

#[klipper_constant]
pub const FOCI_VM_ADC_REFERENCE_MILLIVOLTS: u32 = 2_500;
const _: () =
    assert!(FOCI_VM_ADC_REFERENCE_MILLIVOLTS == board_config::VM_ADC_REFERENCE_MILLIVOLTS);

#[klipper_constant]
pub const FOCI_VM_ADC_CENTER_COUNTS: u32 = 32_767;
const _: () = assert!(FOCI_VM_ADC_CENTER_COUNTS == board_config::VM_ADC_CENTER_COUNTS);
