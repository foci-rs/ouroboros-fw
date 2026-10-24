//! Ouroboros production-v1 TMC4671 hardware configuration constants.

use crate::ankyra::enumerations::Pins;

/// USB product string; trace builds are told apart by their vendor-specific interface.
pub(crate) const USB_PRODUCT: &str = "FOCI Ouroboros";

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
/// Values validated on the live production-v1 board; the PWM rate is the
/// one that keeps a 1600 Hz current loop damped.
const OUROBOROS_PROD_V1_PWM_MAXCNT: u16 = 1999;
const OUROBOROS_PROD_V1_DSADC_MDEC: u16 =
    foci_firmware::config::dsadc_mdec_from_pwm_maxcnt(OUROBOROS_PROD_V1_PWM_MAXCNT);

pub(crate) const TMC_CONFIG: foci_firmware::config::TmcConfig = foci_firmware::config::TmcConfig {
    pwm_maxcnt: OUROBOROS_PROD_V1_PWM_MAXCNT,
    tmc_clock_hz: 25_000_000,
    bbm_h: 10,
    bbm_l: 10,
    dsadc_mdec_a: OUROBOROS_PROD_V1_DSADC_MDEC,
    dsadc_mdec_b: OUROBOROS_PROD_V1_DSADC_MDEC,
    adc_i_select: foci_tmc4671::registers::AdcISelect::new()
        .with_i0_select(0)
        .with_i1_select(1)
        .with_ux_select(0)
        .with_v_select(0)
        .with_wy_select(1)
        .into_bits(),
    excitation_voltage: 512,
    electrical_model_theta_e_us: 160,
    current_validation_flux_max_encoder_delta_counts: 4,
    current_validation_torque_max_encoder_delta_counts: 16,
    resistance_max_expected_current_count: 1600,
    resistance_scale_metadata_validated: false,
    // Milliamps per ADC least significant bit; maker-confirmed
    // production-v1 current scale.
    current_scale_ma_per_lsb: 1.272,
    max_run_current_ma: 10_000,
    vm_divider_high_ohms: VM_DIVIDER_HIGH_OHMS,
    vm_divider_low_ohms: VM_DIVIDER_LOW_OHMS,
    vm_adc_reference_millivolts: VM_ADC_REFERENCE_MILLIVOLTS,
    vm_adc_center_counts: VM_ADC_CENTER_COUNTS,
};

/// Per-channel feedback wiring. Every board channel is ABN-only today.
pub(crate) const FEEDBACK_CONFIGS: [foci_firmware::config::FeedbackConfig;
    TMC_CHANNEL_COUNT as usize] = [foci_firmware::config::FeedbackConfig {
    phi_e_source: foci_firmware::config::FeedbackSource::Abn,
    position_source: foci_firmware::config::FeedbackSource::Abn,
}; TMC_CHANNEL_COUNT as usize];

/// STM32H723 ADC instance owning PC4 (TH0) and PC5 (TH1).
///
/// Verified against the STM32H723xE/G datasheet (DS13313 Rev 5, cached at
/// `docs/datasheets/STM32H723xx_datasheet.pdf`), Table 7 "STM32H723 pin and
/// ball descriptions" (p. 60): the PC4 row lists additional functions
/// `ADC12_INP4` / `OPAMP1_VOUT` / `COMP1_INM`, and the PC5 row lists
/// `ADC12_INN4` / `ADC12_INP8` / `OPAMP1_VINM`. The `ADC12` prefix means
/// these channels are wired to the shared ADC1/ADC2 analog input bank, not
/// ADC3.
/// ADC1 and ADC2 see identical external inputs on this family, so either
/// works; ADC1 is picked here for no reason other than being first.
pub(crate) type ExternalAdcInstance = embassy_stm32::peripherals::ADC1;

/// STM32H723 ADC instance carrying the internal temperature sensor channel.
///
/// Verified against RM0468 (STM32H723 reference manual, Rev 3) Section
/// 29.4.3 "ADC internal channels", Table 242 "ADC interconnection": `VSENSE`
/// (internal temperature sensor), `VREFINT`, and `VBAT/4` are wired only to
/// `ADC3 VINP[17]`, `VINP[18]`, and `VINP[16]` respectively -- these internal
/// channels are not present on ADC1/ADC2 at all. This differs from
/// `ExternalAdcInstance` above, so init must construct and hold both ADC1
/// and ADC3 peripheral instances rather than sharing one.
pub(crate) type InternalTempAdcInstance = embassy_stm32::peripherals::ADC3;

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

pub(crate) fn endstop_connector_pin(pin: u8) -> Option<u8> {
    match Pins::try_from(pin) {
        Ok(Pins::STOP0) => Some(Pins::PD7 as u8),
        Ok(Pins::STOP1) => Some(Pins::PB3 as u8),
        _ => None,
    }
}

/// STM32H723 internal temperature sensor factory calibration, read from
/// ST's documented system-memory addresses (RM0468 device electronic
/// signature section). `TS_CAL1` is captured at 30C, `TS_CAL2` at 130C,
/// both at 3.3V VDDA (DS13313 Table 90), as 12-bit counts, matching
/// Klipper's `config_stm32h723`.
const TS_CAL1_ADDR: *const u16 = 0x1FF1_E820 as *const u16;
const TS_CAL2_ADDR: *const u16 = 0x1FF1_E840 as *const u16;
const TS_CAL1_TEMP_C: f32 = 30.0;
const TS_CAL2_TEMP_C: f32 = 130.0;

/// Read the STM32H723's factory-programmed temperature sensor calibration.
///
/// # Safety
/// These addresses are ST's documented read-only system memory calibration
/// area, valid on every STM32H723 device -- reading them is always safe;
/// this function exists only to centralize the `unsafe` block, not because
/// the read itself is conditionally unsafe.
pub(crate) fn read_mcu_temp_calibration() -> foci_firmware::mcu_temp::FactoryCalibration {
    // SAFETY: TS_CAL1_ADDR/TS_CAL2_ADDR are ST's documented factory
    // calibration addresses in system memory, present and readable on
    // every STM32H723 device.
    let cal1 = unsafe { core::ptr::read_volatile(TS_CAL1_ADDR) };
    let cal2 = unsafe { core::ptr::read_volatile(TS_CAL2_ADDR) };
    foci_firmware::mcu_temp::FactoryCalibration {
        cal_low: (TS_CAL1_TEMP_C, cal1),
        cal_high: (TS_CAL2_TEMP_C, cal2),
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
    fn endstop_connector_pins_resolve_to_pd7_and_pb3() {
        assert_eq!(endstop_connector_pin(Pins::STOP0 as u8), Some(55));
        assert_eq!(endstop_connector_pin(Pins::STOP1 as u8), Some(19));
        assert_eq!(endstop_connector_pin(Pins::STALL0 as u8), None);
        assert_eq!(endstop_connector_pin(Pins::PD7 as u8), None);
    }

    #[test]
    fn analog_pin_channel_rejects_unrelated_pins() {
        assert_eq!(analog_pin_channel(Pins::STEP0 as u8), None);
        assert_eq!(analog_pin_channel(Pins::PC0 as u8), None);
    }
}
