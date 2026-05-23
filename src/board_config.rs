//! Ouroboros production-v1 TMC4671 hardware configuration constants.

use crate::ankyra::enumerations::Pins;

/// Number of board-local TMC4671 motor channels.
pub(crate) const TMC_CHANNEL_COUNT: u8 = 2;

/// Current scaling for Ouroboros: milliamps per ADC least significant bit.
///
/// Maker-confirmed production-v1 current scale.
pub(crate) const CURRENT_SCALE_MA_PER_LSB: f32 = 1.272;

/// Return the motor channel represented by a virtual STEP pin.
pub(crate) fn step_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::STEP0) => Some(0),
        Ok(Pins::STEP1) => Some(1),
        _ => None,
    }
}

/// Return the motor channel represented by a virtual DIR pin.
pub(crate) fn dir_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::DIR0) => Some(0),
        Ok(Pins::DIR1) => Some(1),
        _ => None,
    }
}

/// Return the motor channel represented by a virtual ENA pin.
pub(crate) fn enable_pin_channel(pin: u32) -> Option<u8> {
    let pin = u8::try_from(pin).ok()?;
    match Pins::try_from(pin) {
        Ok(Pins::ENA0) => Some(0),
        Ok(Pins::ENA1) => Some(1),
        _ => None,
    }
}

/// Board-specific TMC4671 hardware configuration for Ouroboros production v1.
///
/// Initial values borrow existing regression-board defaults except for the
/// current scale. They are compile-only defaults until hardware bring-up
/// confirms production-v1 current sensing and power-stage timing.
pub(crate) const TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
    pwm_maxcnt: 3999,
    tmc_clock_hz: 25_000_000,
    bbm_h: 10,
    bbm_l: 10,
    dsadc_mdec: 331,
    // Preserve the pre-bringup routing until Ouroboros current sense is verified.
    adc_i_select: 0x1000_0100,
    excitation_voltage: 512,
    didt_cycles: 5000,
    bandwidth_hz: 150.0,
};
