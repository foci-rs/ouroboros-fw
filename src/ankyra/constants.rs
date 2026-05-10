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

#[klipper_constant]
pub const CLOCK_FREQ: u32 = 260_000_000;

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
