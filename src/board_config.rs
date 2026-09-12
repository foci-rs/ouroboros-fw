//! Ouroboros production-v1 TMC4671 hardware configuration constants.

use crate::ankyra::enumerations::Pins;

/// Number of board-local TMC4671 motor channels.
pub(crate) const TMC_CHANNEL_COUNT: u8 = 2;

/// Ouroboros production-v1 VM sense divider high-side resistor.
///
/// Cached schematic excerpt `docs/datasheets/Ouroboros-schematic-2025-01-26.pdf`
/// shows R13/R46 = 71.5 kOhm for the two TMC4671 ADC_VM channels.
pub(crate) const VM_DIVIDER_HIGH_OHMS: u32 = 71_500;

/// Ouroboros production-v1 VM sense divider low-side resistor.
///
/// Cached schematic excerpt shows R14/R47 = 1.5 kOhm for the two TMC4671
/// ADC_VM channels.
pub(crate) const VM_DIVIDER_LOW_OHMS: u32 = 1_500;

/// TMC4671 ADC_VM reference span in millivolts.
pub(crate) const VM_ADC_REFERENCE_MILLIVOLTS: u32 = 2_500;

/// Centered positive ADC_VM count span for the TMC4671 16-bit raw ADC.
pub(crate) const VM_ADC_CENTER_COUNTS: u32 = 32_767;

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
/// Current-loop P uses the physical-bandwidth formula. Keep the
/// compile-time default aligned with OpenFFBoard; Ouroboros hardware still
/// needs target validation before this is treated as signed off.
const OUROBOROS_PROD_V1_PWM_MAXCNT: u16 = 3999;
const OUROBOROS_PROD_V1_DSADC_MDEC: u16 =
    foci_firmware::config::dsadc_mdec_from_pwm_maxcnt(OUROBOROS_PROD_V1_PWM_MAXCNT);

pub(crate) const TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
    pwm_maxcnt: OUROBOROS_PROD_V1_PWM_MAXCNT,
    tmc_clock_hz: 25_000_000,
    bbm_h: 10,
    bbm_l: 10,
    dsadc_mdec_a: OUROBOROS_PROD_V1_DSADC_MDEC,
    dsadc_mdec_b: OUROBOROS_PROD_V1_DSADC_MDEC,
    // Preserve the pre-bringup routing until Ouroboros current sense is verified.
    adc_i_select: 0x1000_0100,
    excitation_voltage: 512,
    bandwidth_hz: 600.0,
    electrical_model_theta_e_us: 160,
    current_validation_flux_max_encoder_delta_counts: 4,
    current_validation_torque_max_encoder_delta_counts: 16,
    resistance_max_expected_current_count: 1600,
    resistance_scale_metadata_validated: false,
    // Milliamps per ADC least significant bit; maker-confirmed
    // production-v1 current scale.
    current_scale_ma_per_lsb: 1.272,
};

/// Per-channel feedback wiring. Every board channel is ABN-only today; see
/// the spec for the analog-encoder-enablement follow-up.
pub(crate) const FEEDBACK_CONFIGS: [foci_firmware::config::FeedbackConfig;
    TMC_CHANNEL_COUNT as usize] = [foci_firmware::config::FeedbackConfig {
    phi_e_source: foci_firmware::config::FeedbackSource::Abn,
    position_source: foci_firmware::config::FeedbackSource::Abn,
}; TMC_CHANNEL_COUNT as usize];

/// Return the `shared/foci-firmware` `ANALOG_CHANNELS` slot index for a
/// virtual analog-in pin. `TH0` (index 0) is the schematic's `TH1` net
/// (TMC1 MOSFET thermistor, physical pin PC4); `TH1` (index 1) is the
/// schematic's `TH2` net (TMC2, PC5); `MCU_TEMP` is index 2. The 0-index
/// wire names intentionally follow this project's `STEP0`/`DIR0`
/// convention, not the schematic's 1-indexed net labels -- do not "fix"
/// this offset.
pub(crate) fn analog_pin_channel(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::TH0) => Some(0),
        Ok(Pins::TH1) => Some(1),
        Ok(Pins::MCU_TEMP) => Some(2),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn analog_pin_channel_resolves_th0_th1_and_mcu_temp() {
        assert_eq!(analog_pin_channel(Pins::TH0 as u8), Some(0));
        assert_eq!(analog_pin_channel(Pins::TH1 as u8), Some(1));
        assert_eq!(analog_pin_channel(Pins::MCU_TEMP as u8), Some(2));
    }

    #[test]
    fn analog_pin_channel_rejects_unrelated_pins() {
        assert_eq!(analog_pin_channel(Pins::STEP0 as u8), None);
        assert_eq!(analog_pin_channel(Pins::PC0 as u8), None);
    }
}
