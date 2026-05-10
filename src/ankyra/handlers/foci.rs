//! Board-local FOCI handler: `foci_tune`.
//!
//! The six other FOCI handlers (`foci_selftest`, `foci_calibrate`,
//! `foci_dump_registers`, `foci_trace_info`, `foci_trace_fetch`,
//! `foci_commission`) were lifted to `foci_firmware::ankyra::handlers::foci`
//! in Task 17 of the foci-firmware handler-lift plan. The glob re-export
//! below brings in the lifted handlers (and their macro-generated ankyra
//! bookkeeping items) so that `crate::ankyra::handlers::foci::*` in
//! `provider.rs` resolves all seven handlers correctly.
//!
//! `foci_tune` stays board-local because its implementation calls
//! `foci_tune_inner`, which emits replies via the board-local ankyra
//! transport sender.

#[allow(unused_imports)]
pub use foci_firmware::ankyra::handlers::foci::*;

use ankyra::prelude::klipper_command;
use foci_firmware::context::{ClockSource, ShutdownRecovery, StateAccess};
use foci_firmware::shutdown::ShutdownReason;

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::tmc::{FociTuneParams, foci_tune_inner};

/// Start Stage 2 installed tuning (mechanical ID + velocity/position/filters).
///
/// The parameter list is dictated by the Klipper wire protocol — collapsing
/// it into a struct would break the host command format, so the clippy lint
/// is suppressed at the handler level.
#[allow(clippy::too_many_arguments)]
#[klipper_command]
pub fn foci_tune(
    ctx: &mut FirmwareCtx<'_>,
    oid: u8,
    profile: u8,
    mode: u8,
    inner_lambda: u32,
    theta_e: u32,
    current_ringing: u8,
    current_bw: u32,
    tau_e_us: u32,
    tau_e_crosscheck_us: u32,
    tau_residual_permille: u16,
    inner_warning_flags: u8,
) {
    let channel = match ctx.state().foci_channel_for_oid(oid) {
        Some(channel) => channel,
        None => {
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
            }
            return;
        }
    };
    foci_tune_inner(FociTuneParams {
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
    });
}
