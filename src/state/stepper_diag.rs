//! Stepper execution counters, performance counters, and async diagnostic queues.
//!
//! Events are produced from P1/P2/P3 contexts and drained from a P2 task.
//! Queue access is protected by short interrupt-free critical sections so
//! producers never touch the transport from an ISR or P1 stop path.

use foci_firmware::diagnostic::{
    StepperDiagnosticEvent, StepperDiagnosticQueue, StepperEventReason, StepperExecutionSnapshot,
    StepperExecutionStats, StepperPerfCounters, StepperPerfDiagnosticEvent,
};

use crate::state::stepper::STEPPER_CHANNEL_COUNT;

static STEPPER_EXECUTION_STATS: StepperExecutionStats<STEPPER_CHANNEL_COUNT> =
    StepperExecutionStats::new();
static STEPPER_PERF_COUNTERS: StepperPerfCounters<520> = StepperPerfCounters::new();

pub(crate) const STEPPER_DIAG_QUEUE_DEPTH: usize = 16;
pub(crate) const STEPPER_PERF_EVENT_QUEUE_DEPTH: usize = 4;

static mut STEPPER_DIAG_EVENTS: StepperDiagnosticQueue<
    StepperDiagnosticEvent,
    STEPPER_DIAG_QUEUE_DEPTH,
> = StepperDiagnosticQueue::new();
static mut STEPPER_PERF_EVENTS: StepperDiagnosticQueue<
    StepperPerfDiagnosticEvent,
    STEPPER_PERF_EVENT_QUEUE_DEPTH,
> = StepperDiagnosticQueue::new();

pub(crate) const PERF_SITE_TRIGGER_DOMAIN: u8 = 0;
pub(crate) const PERF_SITE_QUEUE_STEP: u8 = 1;
pub(crate) const PERF_SITE_TIM5: u8 = 2;

pub(crate) fn push_event(event: StepperDiagnosticEvent) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_DIAG_EVENTS) }.push_drop_oldest(event);
    });
}

pub(crate) fn pop_event() -> Option<StepperDiagnosticEvent> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_DIAG_EVENTS) }.pop()
    })
}

pub(crate) fn push_perf_event(event: StepperPerfDiagnosticEvent) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_PERF_EVENTS) }.push_drop_oldest(event);
    });
}

pub(crate) fn pop_perf_event() -> Option<StepperPerfDiagnosticEvent> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: all accesses to this static go through this interrupt-free
        // critical section.
        unsafe { &mut *core::ptr::addr_of_mut!(STEPPER_PERF_EVENTS) }.pop()
    })
}

pub(crate) fn record_perf_event(reason: StepperEventReason, channel: u8, clock: u32) {
    push_perf_event(StepperPerfDiagnosticEvent {
        reason,
        clock,
        snapshot: perf_snapshot(channel, false),
    });
}

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

/// Record one executed logical motor step and its emitted physical pulses.
pub(crate) fn record_executed_step(channel: u8, forward: bool, physical_pulses: u8, clock: u32) {
    STEPPER_EXECUTION_STATS.record_executed_step(channel, forward, physical_pulses, clock);
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

pub(crate) fn record_critical_section(site: u8, cycles: u32) {
    STEPPER_PERF_COUNTERS.record_critical_section(site, cycles);
}

pub(crate) fn record_tim5_entry_latency(lateness_ticks: u32) {
    STEPPER_PERF_COUNTERS.record_tim5_entry_latency(lateness_ticks);
}

pub(crate) fn record_tim5_defer() {
    STEPPER_PERF_COUNTERS.record_tim5_defer();
}

pub(crate) fn record_scheduler_pop_lateness(lateness_ticks: u32) {
    STEPPER_PERF_COUNTERS.record_scheduler_pop_lateness(lateness_ticks);
}

pub(crate) fn record_scheduler_full() {
    STEPPER_PERF_COUNTERS.record_scheduler_full();
}

pub(crate) fn record_scheduler_cycles(scheduler_cycles: u32, event_count: u32) {
    STEPPER_PERF_COUNTERS.record_scheduler_cycles(scheduler_cycles, event_count);
}

pub(crate) fn record_tim5_irq(irq_cycles: u32, dispatch_cycles: u32, event_count: u32) {
    STEPPER_PERF_COUNTERS.record_tim5_irq(irq_cycles, dispatch_cycles, event_count);
}
