//! Klipper reply emitters for commission/calibrate/selftest/trace-mode outcomes.
//!
//! The reply payload mapping and the sender-generic sink live in
//! `foci_firmware::commission_dispatch::reply_sender`; this module only names the board's
//! generated ankyra sender.

use foci_firmware::commission_dispatch::{ReplyPayload, reply_sender};
use foci_firmware::commissioning::run_types::CommissionPhase;
use foci_firmware::trace::sampling::TraceMode;

/// Emit terminal commissioning-family replies.
pub(crate) fn emit_terminal_reply(channel: u8, payload: ReplyPayload) {
    reply_sender::emit_terminal_reply(&mut crate::_ankyra_config::Sender, channel, payload);
}

/// Emit the live trace mode after the control task applied a `foci_set_trace_mode` request.
pub(crate) fn emit_trace_mode_state(oid: u8, mode: TraceMode) {
    reply_sender::emit_trace_mode_state(&mut crate::_ankyra_config::Sender, oid, mode);
}

/// Emit a `FociCommissionPhase` reply for a phase transition.
pub(crate) fn emit_phase_reply(oid: u8, phase_id: CommissionPhase) {
    reply_sender::emit_phase_reply(&mut crate::_ankyra_config::Sender, oid, phase_id);
}
