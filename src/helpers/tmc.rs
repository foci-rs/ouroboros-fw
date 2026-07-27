//! TMC4671 helpers: request enqueue to the `tmc_control` task and the
//! inner half of the `foci_tune` handler. Promoted from `mod app` in
//! `main.rs` by Task 8c so handlers and background reply emitters
//! share a single implementation.

use foci_firmware::context::tmc::{self as tmc_context, TmcRequestQueueBoard};
use foci_firmware::tmc::TmcRequest;

use crate::state::tmc::TMC_REQUEST_SENDER;

pub(crate) struct OuroborosTmcQueue;

impl TmcRequestQueueBoard for OuroborosTmcQueue {
    fn try_send_tmc_request(&self, req: TmcRequest) -> bool {
        critical_section::with(|cs| {
            if let Some(sender) = TMC_REQUEST_SENDER.borrow(cs).borrow_mut().as_mut() {
                return sender.try_send(req).is_ok();
            }
            false
        })
    }
}

/// Parameters for `foci_tune` grouped into a struct to satisfy
/// clippy's `too_many_arguments` lint. The `#[klipper_command]`
/// handler signature is protocol-bound and cannot be changed.
pub(crate) struct FociTuneParams {
    pub(crate) channel: u8,
    pub(crate) oid: u8,
    pub(crate) action: u8,
    pub(crate) profile: u8,
    pub(crate) mode: u8,
    pub(crate) inner_lambda: u32,
    pub(crate) theta_e: u32,
    pub(crate) current_ringing: u8,
    pub(crate) current_bw: u32,
    pub(crate) inner_warning_flags: u8,
    pub(crate) requested_velocity_mrev_s: u32,
    pub(crate) machine_velocity_ceiling_mrev_s: u32,
    pub(crate) requested_velocity_source: u8,
    pub(crate) max_stroke_travel_mrev: u32,
    pub(crate) settle_travel_reserve_mrev: u32,
    pub(crate) negative_position_headroom_mrev: u32,
    pub(crate) positive_position_headroom_mrev: u32,
    pub(crate) max_duration_ms: u16,
}

/// Send a request to the tmc_control task.
///
/// Returns `true` if the request was enqueued, `false` if the channel
/// was full or not yet initialized. Callers with async reply contracts
/// must send a failure reply to the host when this returns `false`.
pub(crate) fn tmc_send(req: TmcRequest) -> bool {
    let sent = tmc_context::enqueue_tmc_request(&OuroborosTmcQueue, req).is_ok();
    if !sent {
        defmt::warn!("TMC request channel full, command dropped");
    }
    sent
}

/// Inner implementation of `foci_tune`.
pub(crate) fn foci_tune_inner(params: FociTuneParams) {
    use core::sync::atomic::Ordering::Relaxed;
    let FociTuneParams {
        oid,
        action,
        profile,
        mode,
        inner_lambda,
        theta_e,
        current_ringing,
        current_bw,
        inner_warning_flags,
        requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s,
        requested_velocity_source,
        max_stroke_travel_mrev,
        settle_travel_reserve_mrev,
        negative_position_headroom_mrev,
        positive_position_headroom_mrev,
        max_duration_ms,
        channel,
    } = params;
    let Some(action) = foci_firmware::tmc::AutotuneAction::from_u8(action) else {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = 0,
            status: u8 = foci_firmware::autotune::CommissionError::InvalidSchedule.status_code(),
        );
        return;
    };
    let requested_velocity_source = match requested_velocity_source {
        0 => foci_firmware::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Explicit,
        1 => foci_firmware::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
        _ => {
            ::ankyra::klipper_reply_from!(
                &mut crate::_ankyra_config::Sender,
                foci_firmware::ankyra::replies::FociCommissionPhase,
                oid: u8 = oid,
                phase: u8 = 0,
                status: u8 = foci_firmware::autotune::CommissionError::InvalidSchedule.status_code(),
            );
            return;
        }
    };
    let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
        defmt::error!("foci_tune: invalid channel={}", channel);
        return;
    };
    if tmc_state.commission_in_flight.swap(true, Relaxed) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = 0,
            status: u8 = foci_firmware::autotune::CommissionError::AlreadyRunning.status_code(),
        );
        return;
    }
    if !tmc_send(foci_firmware::tmc::tmc_tune_with_action(
        channel,
        oid,
        action,
        profile,
        mode,
        inner_lambda,
        theta_e,
        current_ringing,
        current_bw,
        inner_warning_flags,
        requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s,
        requested_velocity_source,
        max_stroke_travel_mrev,
        settle_travel_reserve_mrev,
        negative_position_headroom_mrev,
        positive_position_headroom_mrev,
        max_duration_ms,
    )) {
        tmc_state.commission_in_flight.store(false, Relaxed);
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = 0,
            status: u8 = foci_firmware::autotune::CommissionError::QueueFull.status_code(),
        );
    }
}
