//! Ouroboros TMC control board accessors.

use core::sync::atomic::Ordering::Relaxed;

use embassy_stm32::gpio::Output;
use foci_firmware::tmc_control::TmcControlRuntime;
use foci_firmware::tmc_control::helpers::refresh_velocity_scale_from_state;
use foci_tmc4671::spi::{RetryPolicy, Tmc4671Spi};

use crate::ankyra::constants::CLOCK_FREQ;
use crate::app_types::TmcSpiDevice;
use crate::board_config::{TMC_CHANNEL_COUNT, TMC_CONFIG};
use crate::commissioning_backend::OuroborosBackend;

pub(crate) type TmcEnablePins = [Output<'static>; TMC_CHANNEL_COUNT as usize];
pub(crate) type TmcRuntimes = [TmcControlRuntime; TMC_CHANNEL_COUNT as usize];

const _: [(); 2] = [(); TMC_CHANNEL_COUNT as usize];

pub(crate) fn tmc_driver<'spi>(device: TmcSpiDevice<'spi>) -> Tmc4671Spi<TmcSpiDevice<'spi>> {
    Tmc4671Spi::new(device).with_retry_policy(RetryPolicy::production())
}

pub(crate) fn new_tmc_runtimes() -> TmcRuntimes {
    [
        TmcControlRuntime::new_for_channel(0),
        TmcControlRuntime::new_for_channel(1),
    ]
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

pub(crate) fn channel_needs_safe_state(channel: u8, runtime: &TmcControlRuntime) -> bool {
    runtime.is_motor_enabled()
        || runtime.is_commission_active()
        || tmc_state(channel).motor_armed.load(Relaxed)
        || tmc_state(channel).commission_in_flight.load(Relaxed)
}

pub(crate) fn commissioning_request_blocked(runtimes: &TmcRuntimes, _request_channel: u8) -> bool {
    for channel in 0..TMC_CHANNEL_COUNT {
        if runtimes[channel as usize].is_commission_active() {
            return true;
        }
    }
    false
}

pub(crate) fn channel_may_be_energized(channel: u8, runtime: &TmcControlRuntime) -> bool {
    runtime.is_motor_enabled()
        || runtime.is_commission_active()
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

pub(crate) fn next_commission_trace_seq(
    commission_trace_engine_seq: &mut [u32; TMC_CHANNEL_COUNT as usize],
    channel: u8,
) -> u32 {
    let idx = channel as usize;
    let seq = commission_trace_engine_seq[idx];
    commission_trace_engine_seq[idx] = commission_trace_engine_seq[idx].wrapping_add(1);
    seq
}
