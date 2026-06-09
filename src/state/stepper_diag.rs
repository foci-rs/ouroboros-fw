//! Stepper execution and performance diagnostic counters.
//!
//! This module intentionally excludes async stepper diagnostic event queues.


use foci_firmware::diagnostic::{
    StepperEventReason, StepperExecutionSnapshot, StepperExecutionStats, StepperPerfCounters,
};

use crate::state::stepper::STEPPER_CHANNEL_COUNT;

static STEPPER_EXECUTION_STATS: StepperExecutionStats<STEPPER_CHANNEL_COUNT> =
    StepperExecutionStats::new();
static STEPPER_PERF_COUNTERS: StepperPerfCounters<520> = StepperPerfCounters::new();

pub(crate) fn record_timer_activation(channel: u8, clock: u32) {
    STEPPER_EXECUTION_STATS.record_timer_activation(channel, clock);
}

pub(crate) fn record_queued_segment(channel: u8, steps: u32) {
    STEPPER_EXECUTION_STATS.record_queued_segment(channel, steps);
}

pub(crate) fn record_loaded_segment(channel: u8, steps: u32) {
    STEPPER_EXECUTION_STATS.record_loaded_segment(channel, steps);
}

pub(crate) fn record_loaded_segment_timing(
    channel: u8,
    steps: u32,
    now: u32,
    scheduled: u32,
    compare: u32,
) {
    STEPPER_EXECUTION_STATS.record_loaded_segment_timing(channel, steps, now, scheduled, compare);
    let lateness = now.wrapping_sub(scheduled) as i32;
    STEPPER_PERF_COUNTERS.record_stepper_load_lateness(lateness);
}

pub(crate) fn record_executed_step(channel: u8, forward: bool, clock: u32) {
    STEPPER_EXECUTION_STATS.record_executed_step(channel, forward, clock);
}

pub(crate) fn record_discarded_segment(channel: u8, steps: u32) {
    STEPPER_EXECUTION_STATS.record_discarded_segment(channel, steps);
}

pub(crate) fn record_queue_empty(channel: u8) {
    STEPPER_EXECUTION_STATS.record_queue_empty(channel);
}

pub(crate) fn record_missed_deadline(channel: u8) {
    STEPPER_EXECUTION_STATS.record_missed_deadline(channel);
}

pub(crate) fn record_physical_stop(
    channel: u8,
    reason: StepperEventReason,
    remaining_events: u32,
    queue_len: u16,
    drained_segments: u32,
    drained_steps: u32,
) {
    STEPPER_EXECUTION_STATS.record_physical_stop(
        channel,
        reason,
        remaining_events,
        queue_len,
        drained_segments,
        drained_steps,
    );
}

pub(crate) fn record_reset(channel: u8, drained_segments: u32, drained_steps: u32) {
    STEPPER_EXECUTION_STATS.record_reset(channel, drained_segments, drained_steps);
}

pub(crate) fn snapshot(
    channel: u8,
    position: i32,
    timer_active: bool,
    queue_len: u16,
) -> Option<StepperExecutionSnapshot> {
    STEPPER_EXECUTION_STATS.snapshot(channel, position, timer_active, queue_len)
}

pub(crate) fn perf_snapshot(
    channel: u8,
    clear: bool,
) -> foci_firmware::diagnostic::StepperPerfSnapshot {
    STEPPER_PERF_COUNTERS.snapshot(channel, clear, cfg!(feature = "trace"))
}
