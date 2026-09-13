//! Periodic board heartbeat tasks: stats output and trsync-report drain.
//! Each function is the body of an RTIC task declared in `main.rs`; the
//! task shells handle only RTIC plumbing (Mono, local-resource binding)
//! and delegate all work here.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};
use embassy_stm32::peripherals;
use embassy_stm32::wdg::IndependentWatchdog;
use foci_firmware::ankyra::replies::{
    AnalogInState, FociStepperEvent, FociStepperPerfEvent, Stats, TrsyncState,
};

/// Period between independent-watchdog service ticks.
pub const WATCHDOG_INTERVAL_MS: u64 = 5_000;

/// Carries the last stats-send clock between ticks.
pub struct StatsAccumulator {
    last_clock: u32,
}

impl StatsAccumulator {
    pub fn new(initial_clock: u32) -> Self {
        Self {
            last_clock: initial_clock,
        }
    }
}

/// Service the independent watchdog from its dedicated RTIC task.
pub fn watchdog_tick(wdg: &mut IndependentWatchdog<'static, peripherals::IWDG1>) {
    wdg.pet();
}

/// Track 32-bit TIM5-clock wraps for `get_uptime`'s high word and report the
/// CPU load sample via a Klipper `stats` message.
///
/// `stats_send_time` and `stats_send_time_high` are the atomics shared
/// with `get_uptime`. Write-order matters: the `swap` on
/// `stats_send_time` must happen before the `fetch_add` on
/// `stats_send_time_high` so a preempting `get_uptime` on P2 cannot
/// observe an incremented high with a stale low.
pub fn stats_tick(
    accumulator: &mut StatsAccumulator,
    now: u32,
    stats_send_time: &AtomicU32,
    stats_send_time_high: &AtomicU32,
) {
    let wall = now.wrapping_sub(accumulator.last_clock);
    accumulator.last_clock = now;

    let sample = foci_firmware::cpu_load::take(wall);

    // Track 32-bit clock wraps for `get_uptime`'s high word.
    let prev = stats_send_time.swap(now, Relaxed);
    if now < prev {
        stats_send_time_high.fetch_add(1, Relaxed);
    }

    defmt::info!("stats tick");
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        Stats,
        count: u32 = sample.count,
        sum: u32 = sample.sum,
        sumsq: u32 = sample.sumsq,
    );
}

/// Drain pending trsync reports and emit `trsync_state` replies.
///
/// Spawned by the TIM5 CC2 ISR when any trsync has a pending report. The
/// caller-supplied `drain_snapshots` closure collects snapshots in one
/// brief critical section (typically inside the trigger-domain lock); the
/// replies are then sent at P2 without holding that lock.
///
/// Runs until a pass finds `report_pending` still clear afterward, via
/// [`foci_firmware::helpers::drain::drain_until_quiescent`], so a report
/// that a producer raises while this task is already running is never
/// stranded behind the single-capacity spawn slot.
pub fn emit_trigger_reports<F>(mut drain_snapshots: F, report_pending: &AtomicBool)
where
    F: FnMut(&mut heapless::Vec<foci_klipper::trigger::ReportSnapshot, 4>),
{
    foci_firmware::helpers::drain::drain_until_quiescent(report_pending, || {
        let mut snapshots: heapless::Vec<foci_klipper::trigger::ReportSnapshot, 4> =
            heapless::Vec::new();
        drain_snapshots(&mut snapshots);
        for snap in &snapshots {
            ::ankyra::klipper_reply_from!(
                &mut crate::_ankyra_config::Sender,
                TrsyncState,
                oid: u8 = snap.oid,
                can_trigger: u8 = snap.can_trigger,
                trigger_reason: u8 = snap.trigger_reason,
                clock: u32 = snap.clock,
            );
        }
    });
}

/// Sample every configured analog-in channel that is due, sending
/// `analog_in_state` replies. Returns `true` if any channel's sustained
/// out-of-range readings require the caller to enter the shutdown state --
/// this function itself has no access to `app_state` and must not dispatch
/// a shutdown on its own. Called from the `analog_sampling` RTIC task's
/// loop body; see `main.rs` for the `Mono::delay` wrapper and the
/// clock-domain note on why `now` must be a TIM5 tick, not a `Mono` tick.
#[must_use]
pub fn analog_sampling_tick(
    adc: &mut embassy_stm32::adc::Adc<'static, crate::board_config::ExternalAdcInstance>,
    adc_internal: &mut embassy_stm32::adc::Adc<
        'static,
        crate::board_config::InternalTempAdcInstance,
    >,
    temp_channel: &mut embassy_stm32::adc::Vbat,
    th0_pin: &mut embassy_stm32::Peri<'static, peripherals::PC4>,
    th1_pin: &mut embassy_stm32::Peri<'static, peripherals::PC5>,
    mcu_temp_calibration: &foci_firmware::mcu_temp::FactoryCalibration,
    now: u32,
) -> bool {
    use embassy_stm32::adc::SampleTime;

    let mut shutdown_requested = false;

    if let Some(channel) = foci_firmware::analog_in::analog_channel(0)
        && channel.is_due(now)
    {
        let raw = adc.blocking_read(th0_pin, SampleTime::CYCLES810_5);
        shutdown_requested |= emit_analog_sample(channel, raw, now);
    }
    if let Some(channel) = foci_firmware::analog_in::analog_channel(1)
        && channel.is_due(now)
    {
        let raw = adc.blocking_read(th1_pin, SampleTime::CYCLES810_5);
        shutdown_requested |= emit_analog_sample(channel, raw, now);
    }
    if let Some(channel) = foci_firmware::analog_in::analog_channel(2)
        && channel.is_due(now)
    {
        let raw_internal = adc_internal.blocking_read(temp_channel, SampleTime::CYCLES810_5);
        let synthesized = foci_firmware::mcu_temp::synthesize_mcu_temp_reading(
            mcu_temp_calibration,
            raw_internal,
            crate::ankyra::constants::ADC_MAX as f32,
            1, // AnalogChannelState::record_sample's own accumulation enforces the host's sample_count
        );
        shutdown_requested |= emit_analog_sample(channel, synthesized, now);
    }
    shutdown_requested
}

/// Record one raw read, send its `analog_in_state` reply if a burst just
/// completed, and report whether that report was a sustained out-of-range
/// trip. Never dispatches a shutdown itself -- see the note above on why
/// that must happen in the caller, which alone can lock `app_state`.
#[must_use]
fn emit_analog_sample(
    channel: &foci_firmware::analog_in::AnalogChannelState,
    raw: u16,
    now: u32,
) -> bool {
    let Some(oid) = channel.oid() else {
        return false;
    };
    if let foci_firmware::analog_in::SampleStep::Report {
        value,
        next_due_clock,
        shutdown,
        ..
    } = channel.record_sample(raw, now)
    {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            AnalogInState,
            oid: u8 = oid,
            next_clock: u32 = next_due_clock,
            value: u16 = value,
        );
        shutdown
    } else {
        false
    }
}

/// Drain pending stepper diagnostic events and emit host-visible replies.
///
/// Runs until a pass finds `stepper_diag_pending` still clear afterward, via
/// [`foci_firmware::helpers::drain::drain_until_quiescent`], so an event
/// queued while this task is already running is never stranded behind the
/// single-capacity spawn slot.
pub fn emit_stepper_diagnostics(stepper_diag_pending: &AtomicBool) {
    foci_firmware::helpers::drain::drain_until_quiescent(stepper_diag_pending, || {
        emit_stepper_diagnostics_pass();
    });
}

fn emit_stepper_diagnostics_pass() {
    while let Some(event) = crate::state::stepper_diag::pop_event() {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            FociStepperEvent,
            reason: u8 = event.reason as u8,
            channel: u8 = event.channel,
            position: i32 = event.position,
            clock: u32 = event.clock,
            timer_active: u8 = event.timer_active as u8,
            queue_len: u16 = event.queue_len,
            direction: u8 = event.direction as u8,
            data0: u32 = event.data0,
            data1: u32 = event.data1,
        );
    }
    while let Some(event) = crate::state::stepper_diag::pop_perf_event() {
        let snapshot = event.snapshot;
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            FociStepperPerfEvent,
            reason: u8 = event.reason as u8,
            channel: u8 = snapshot.channel,
            clock: u32 = event.clock,
            sample_count: u32 = snapshot.sample_count,
            crit_count: u32 = snapshot.crit_count,
            crit_max_cycles: u32 = snapshot.crit_max_cycles,
            crit_max_site: u8 = snapshot.crit_max_site,
            crit_over_10us: u32 = snapshot.crit_over_10us,
            crit_over_50us: u32 = snapshot.crit_over_50us,
            crit_over_100us: u32 = snapshot.crit_over_100us,
            crit_over_1000us: u32 = snapshot.crit_over_1000us,
            queue_step_count: u32 = snapshot.queue_step_count,
            queue_step_max_cycles: u32 = snapshot.queue_step_max_cycles,
            tim5_activation_count: u32 = snapshot.tim5_activation_count,
            tim5_irq_max_cycles: u32 = snapshot.tim5_irq_max_cycles,
            tim5_dispatch_max_cycles: u32 = snapshot.tim5_dispatch_max_cycles,
            tim5_dispatch_max_cycles_events: u32 = snapshot.tim5_dispatch_max_cycles_events,
            tim5_events_max_per_irq: u32 = snapshot.tim5_events_max_per_irq,
            tim5_event_count_total: u32 = snapshot.tim5_event_count_total,
            tim5_defer_count: u32 = snapshot.tim5_defer_count,
            tim5_empty_count: u32 = snapshot.tim5_empty_count,
            tim5_events_last_activation: u32 = snapshot.tim5_events_last_activation,
            tim5_burst_cycles_per_event_max: u32 = snapshot.tim5_burst_cycles_per_event_max,
            tim5_burst_cycles_per_event_max_cycles: u32 =
                snapshot.tim5_burst_cycles_per_event_max_cycles,
            tim5_burst_cycles_per_event_max_events: u32 =
                snapshot.tim5_burst_cycles_per_event_max_events,
            tim5_burst_cycles_per_event_floor3_max: u32 =
                snapshot.tim5_burst_cycles_per_event_floor3_max,
            tim5_entry_latency_max_ticks: u32 = snapshot.tim5_entry_latency_max_ticks,
            tim5_pop_lateness_max_ticks: u32 = snapshot.tim5_pop_lateness_max_ticks,
            scheduler_cycles_max: u32 = snapshot.scheduler_cycles_max,
            scheduler_cycles_events_at_max: u32 = snapshot.scheduler_cycles_events_at_max,
            scheduler_cycles_per_event_max: u32 = snapshot.scheduler_cycles_per_event_max,
            scheduler_cycles_per_event_max_cycles: u32 =
                snapshot.scheduler_cycles_per_event_max_cycles,
            scheduler_cycles_per_event_max_events: u32 =
                snapshot.scheduler_cycles_per_event_max_events,
            scheduler_cycles_per_event_floor3_max: u32 =
                snapshot.scheduler_cycles_per_event_floor3_max,
            scheduler_full_count: u32 = snapshot.scheduler_full_count,
            stepper_load_lateness_max_ticks: u32 = snapshot.stepper_load_lateness_max_ticks,
            stepper_load_lateness_last_ticks: i32 = snapshot.stepper_load_lateness_last_ticks,
            build_trace_enabled: u8 = snapshot.build_trace_enabled as u8,
            shutdown_site_count: u32 = snapshot.shutdown_site_count,
            shutdown_site_max_cycles: u32 = snapshot.shutdown_site_max_cycles,
            reset_site_count: u32 = snapshot.reset_site_count,
            reset_site_max_cycles: u32 = snapshot.reset_site_max_cycles,
            trigger_stop_site_count: u32 = snapshot.trigger_stop_site_count,
            trigger_stop_site_max_cycles: u32 = snapshot.trigger_stop_site_max_cycles,
        );
    }
}
