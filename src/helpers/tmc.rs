//! TMC4671 helpers: request enqueue to the `tmc_control` task and the
//! inner half of the `foci_tune` handler. Promoted from `mod app` in
//! `main.rs` by Task 8c so handlers and background reply emitters
//! share a single implementation.

use foci_firmware::tmc::TmcRequest;

use crate::state::tmc::TMC_REQUEST_SENDER;

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
    pub(crate) tau_e_us: u32,
    pub(crate) tau_e_crosscheck_us: u32,
    pub(crate) tau_residual_permille: u16,
    pub(crate) inner_warning_flags: u8,
}

/// Send a request to the tmc_control task.
///
/// Returns `true` if the request was enqueued, `false` if the channel
/// was full or not yet initialized. Callers with async reply contracts
/// must send a failure reply to the host when this returns `false`.
pub(crate) fn tmc_send(req: TmcRequest) -> bool {
    critical_section::with(|cs| {
        if let Some(sender) = TMC_REQUEST_SENDER.borrow(cs).borrow_mut().as_mut() {
            if sender.try_send(req).is_err() {
                defmt::warn!("TMC request channel full, command dropped");
                return false;
            }
            return true;
        }
        false
    })
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
        tau_e_us,
        tau_e_crosscheck_us,
        tau_residual_permille,
        inner_warning_flags,
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
        tau_e_us,
        tau_e_crosscheck_us,
        tau_residual_permille,
        inner_warning_flags,
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
