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
    pub(crate) profile: u8,
    pub(crate) mode: u8,
    pub(crate) inner_lambda: u32,
    pub(crate) theta_e: u32,
    pub(crate) current_ringing: u8,
    pub(crate) current_bw: u32,
    pub(crate) inner_warning_flags: u8,
    pub(crate) max_travel_mrev: u32,
    pub(crate) max_velocity_mrev_s: u32,
    pub(crate) max_duration_ms: u16,
    pub(crate) direction_mask: u8,
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
        profile,
        mode,
        inner_lambda,
        theta_e,
        current_ringing,
        current_bw,
        inner_warning_flags,
        max_travel_mrev,
        max_velocity_mrev_s,
        max_duration_ms,
        direction_mask,
        channel,
    } = params;
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
    if !tmc_send(foci_firmware::tmc::tmc_tune(
        channel,
        oid,
        profile,
        mode,
        inner_lambda,
        theta_e,
        current_ringing,
        current_bw,
        inner_warning_flags,
        max_travel_mrev,
        max_velocity_mrev_s,
        max_duration_ms,
        direction_mask,
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
