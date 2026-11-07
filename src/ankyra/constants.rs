//! `#[klipper_constant]` declarations registered into the binary's
//! data dictionary `config` section. Consumed by the `constants: [...]`
//! list in `src/ankyra/provider.rs`.
//!
//! All constant identifiers are `SCREAMING_SNAKE_CASE`; ankyra preserves
//! them verbatim as wire names (per `pascal_to_snake` passthrough rule).

use ankyra::klipper_constant;

use crate::board_config;

#[klipper_constant]
pub const CLOCK_FREQ: u32 = 260_000_000;

#[allow(dead_code)]
#[klipper_constant]
pub const MCU: &str = "stm32h723xx";

/// Fixed 12-bit ADC resolution shared by every analog-in channel on this
/// board. Must be identical across `TH0/TH1/MCU_TEMP`; do not vary this per
/// channel. Klipper reads this via `get_constant_float`, which parses the
/// wire string regardless of the declared type here, so a plain `u32`
/// literal is sufficient -- `#[klipper_constant]` does not support a
/// float-typed constant.
#[klipper_constant]
pub const ADC_MAX: u32 = 4095;

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_USB: &str = "PA9,PA11,PA12";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_SPI: &str = "PB13,PB14,PB15,PD0,PD2";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_TMC: &str = "PD1,PD6,PE2,PA2,PE14";

#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_SWD: &str = "PA13,PA14";

/// STEP/DIR GPIO pins for the two TMC4671 hardware step/dir channels.
#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_STEP: &str = "PD4,PD3,PC12,PC11";

/// Board status LED, driven directly by the MCU (distinct from the PE2/PA2
/// pins repurposed as TMC4671 STATUS inputs on production v1.0 hardware).
#[allow(dead_code)]
#[klipper_constant]
pub const RESERVE_PINS_LED: &str = "PA15";

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

#[klipper_constant]
pub const FOCI_MAX_RUN_CURRENT_MA: u32 = 10_000;
const _: () = assert!(FOCI_MAX_RUN_CURRENT_MA == board_config::TMC_CONFIG.max_run_current_ma);
