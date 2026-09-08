//! Ouroboros TMC control board accessors.

use core::sync::atomic::Ordering::Relaxed;

use embassy_stm32::gpio::{Input, Output};
use foci_firmware::commission_dispatch::CommissionRetention;
use foci_firmware::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;
use foci_firmware::shutdown::ShutdownReason;
use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc_control::TmcControlRuntime;
use foci_firmware::tmc_control::helpers::refresh_velocity_scale_from_state;
use foci_firmware::tmc_control::{
    ChannelDriver, DriveCtx, ParkReason, TmcControlConfig, TmcSafetyBootstrapFailure,
};
use foci_tmc4671::spi::{RetryPolicy, Tmc4671Spi};

use crate::ankyra::constants::CLOCK_FREQ;
use crate::app_types::TmcSpiDevice;
use crate::board_config::{TMC_CHANNEL_COUNT, TMC_CONFIG};
use crate::commissioning_backend::OuroborosBackend;
use crate::tmc_mux::{SelectedTmcDevice, TmcSpiMux};

pub(crate) type TmcEnablePins = [Output<'static>; TMC_CHANNEL_COUNT as usize];

const _: [(); 2] = [(); TMC_CHANNEL_COUNT as usize];

/// `STATUS_FLAGS` bit routed to each channel's STATUS pin. Set by the
/// safety bootstrap in `STATUS_MASK`; the emergency-disable audit reads it
/// back to correlate the pin level with the flag.
pub(crate) const STATUS_CRITICAL_MASK: u32 = 1 << 26;

pub(crate) fn status_levels(status0: &Input<'static>) -> [bool; 2] {
    [status0.is_high(), crate::board_init::status1_level()]
}

pub(crate) fn drive_ctx<'a>(
    now_us: u64,
    homing_snapshot: u8,
    status_pin_asserted: bool,
    config: &'a TmcControlConfig,
    integral_candidate: &'a mut IntegralRetentionSlot,
) -> DriveCtx<'a> {
    DriveCtx {
        now_us,
        homing_active: HOMING_STATE.load(Relaxed) != 0,
        homing_snapshot,
        status_pin_asserted,
        status_mask: STATUS_CRITICAL_MASK,
        config,
        retention: CommissionRetention::new(integral_candidate),
    }
}

/// How a parked channel is reported to the host. The first three failures
/// keep the fixed messages Klipper configs and dashboards already match on
/// -- each carries its own variant because `klipper_shutdown_from!` requires
/// a string literal at the call site, not a runtime `&str`; the safety
/// bootstrap reports its stage-specific shutdown reason.
pub(crate) enum ParkShutdown {
    SpiFailedAtBoot,
    ChipIdMismatch,
    PiRepresentationFailedAtBoot,
    Reason(ShutdownReason),
}

pub(crate) fn park_shutdown(channel: u8, reason: ParkReason) -> ParkShutdown {
    match reason {
        ParkReason::SpiUnavailable => ParkShutdown::SpiFailedAtBoot,
        ParkReason::ChipIdMismatch(_) => ParkShutdown::ChipIdMismatch,
        ParkReason::PiRepresentation => ParkShutdown::PiRepresentationFailedAtBoot,
        ParkReason::SafetyBootstrap(stage) => {
            ParkShutdown::Reason(TmcSafetyBootstrapFailure { stage }.shutdown_reason(channel))
        }
    }
}

/// Enable the TMC4671 STATUS interrupt on PE2 (channel 0) once channel 0's
/// safety bootstrap has written `STATUS_MASK`.
pub(crate) fn enable_status_exti_ch0() {
    let syscfg = embassy_stm32::pac::SYSCFG;
    // EXTICR1[11:8] = 4 selects port E for EXTI line 2.
    syscfg.exticr(0).modify(|w| w.set_exti(2, 4));
    let exti = embassy_stm32::pac::EXTI;
    exti.rtsr(0).modify(|w| w.set_line(2, true));
    exti.ftsr(0).modify(|w| w.set_line(2, false));
    exti.pr(0).write(|w| w.set_line(2, true));
    exti.imr(0).modify(|w| w.set_line(2, true));
    defmt::info!("TMC4671 STATUS EXTI2 enabled");
}

/// Enable the TMC4671 STATUS TIM15 capture interrupt on PA2 (channel 1)
/// once channel 1's safety bootstrap has written `STATUS_MASK`.
pub(crate) fn enable_status_capture_ch1() {
    let tim = embassy_stm32::pac::TIM15;
    tim.sr().modify(|w| w.set_ccif(0, false));
    tim.dier().modify(|w| w.set_ccie(0, true));
    defmt::info!("TMC4671 channel 1 STATUS TIM15 capture enabled");
}

pub(crate) enum ChannelDevice<'a> {
    Device(SelectedTmcDevice<'a>),
    Invalid,
}

pub(crate) fn device_or_invalid(mux: &mut TmcSpiMux, channel: u8) -> ChannelDevice<'_> {
    match mux.device(channel) {
        Some(device) => ChannelDevice::Device(device),
        None => ChannelDevice::Invalid,
    }
}

pub(crate) fn tmc_driver<'spi>(device: TmcSpiDevice<'spi>) -> Tmc4671Spi<TmcSpiDevice<'spi>> {
    Tmc4671Spi::new(device).with_retry_policy(RetryPolicy::production())
}

pub(crate) fn tmc_state(channel: u8) -> &'static foci_firmware::state::tmc::TmcChannelState {
    foci_firmware::state::tmc::tmc_channel(channel).expect("validated TMC channel")
}

pub(crate) fn stepper_state(
    channel: u8,
) -> &'static foci_firmware::state::stepper::StepperChannelState {
    foci_firmware::state::stepper::stepper_channel(channel).expect("validated TMC channel")
}

pub(crate) fn ouroboros_backend<'a, 'spi>(
    channel: u8,
    tmc: &'a mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    enable_pin: &'a mut Output<'static>,
) -> OuroborosBackend<'a, 'spi> {
    OuroborosBackend::new(channel, tmc, enable_pin, tmc_state(channel))
}

pub(crate) fn enable_pin(enable_pins: &mut TmcEnablePins, channel: u8) -> &mut Output<'static> {
    &mut enable_pins[channel as usize]
}

pub(crate) fn channel_needs_safe_state(channel: u8, driver: &ChannelDriver) -> bool {
    driver.runtime.is_motor_enabled()
        || driver.runtime.is_commission_active()
        || tmc_state(channel).motor_armed.load(Relaxed)
        || tmc_state(channel).commission_in_flight.load(Relaxed)
}

pub(crate) fn commissioning_request_blocked(
    drivers: &[ChannelDriver; TMC_CHANNEL_COUNT as usize],
    _request_channel: u8,
) -> bool {
    for channel in 0..TMC_CHANNEL_COUNT {
        if drivers[channel as usize].runtime.is_commission_active() {
            return true;
        }
    }
    false
}

#[allow(dead_code)]
pub(crate) fn channel_may_be_energized(channel: u8, driver: &ChannelDriver) -> bool {
    driver.runtime.is_motor_enabled()
        || driver.runtime.is_commission_active()
        || tmc_state(channel).motor_armed.load(Relaxed)
}

pub(crate) fn refresh_velocity_scale(channel: u8, runtime: &mut TmcControlRuntime) {
    refresh_velocity_scale_from_state(
        runtime,
        tmc_state(channel),
        CLOCK_FREQ,
        TMC_CONFIG.pwm_freq_hz(),
    );
}
