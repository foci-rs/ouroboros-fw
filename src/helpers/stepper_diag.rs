//! Stepper diagnostic event capture.

use core::sync::atomic::Ordering::Relaxed;

use foci_firmware::diagnostic::{
    StepperDiagnosticEvent, StepperEventReason, stepper_event_pairs_perf_snapshot,
};

/// Queue length value used when a call site cannot safely inspect the queue.
pub(crate) const UNKNOWN_QUEUE_LEN: u16 = u16::MAX;

/// Capture a bounded stepper diagnostic event and schedule host emission.
pub(crate) fn record_stepper_event(
    reason: StepperEventReason,
    channel: u8,
    clock: u32,
    queue_len: u16,
    data0: u32,
    data1: u32,
) {
    let Some(state) = foci_firmware::state::stepper::stepper_channel(channel) else {
        return;
    };
    crate::state::stepper_diag::push_event(StepperDiagnosticEvent {
        reason,
        channel,
        position: state.position.load(Relaxed),
        clock,
        timer_active: state.timer_active_mirror.load(Relaxed),
        queue_len,
        direction: state.direction.load(Relaxed),
        data0,
        data1,
    });
    if stepper_event_pairs_perf_snapshot(reason) {
        crate::state::stepper_diag::record_perf_event(reason, channel, clock);
    }
    foci_firmware::state::lifecycle::STEPPER_DIAG_PENDING.store(true, Relaxed);
    crate::app::stepper_diag_task::spawn().ok();
}
