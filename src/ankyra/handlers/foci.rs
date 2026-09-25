//! Board-local FOCI handler: `foci_tune`.
//!
//! The other shared FOCI handlers (`foci_selftest`, `foci_calibrate`,
//! `foci_dump_registers`, `foci_commission`) were lifted to
//! `foci_firmware::ankyra::handlers::foci` in Task 17 of the foci-firmware
//! handler-lift plan. The glob re-export
//! below brings in the lifted handlers (and their macro-generated ankyra
//! bookkeeping items) so that `crate::ankyra::handlers::foci::*` in
//! `provider.rs` resolves all seven handlers correctly.
//!
//! `foci_tune` stays board-local because its implementation calls
//! `foci_tune_inner`, which emits replies via the board-local ankyra
//! transport sender.

#[allow(unused_imports)]
pub use foci_firmware::ankyra::handlers::foci::*;

use core::sync::atomic::Ordering::Relaxed;

use ankyra::prelude::klipper_command;
use foci_firmware::context::{ClockSource, ShutdownRecovery, StateAccess};
use foci_firmware::diagnostic::{MotionScaleDiagnosticSnapshot, StepperExecutionSnapshot};
use foci_firmware::shutdown::ShutdownReason;
use foci_firmware::state::lifecycle::{STATS_SEND_TIME, STATS_SEND_TIME_HIGH};

use crate::firmware_ctx::FirmwareCtx;
use crate::helpers::tmc::{FociTuneParams, foci_tune_inner};
use crate::state::stepper::step_pulse_timing;
use crate::tmc_control::board::{stepper_state, tmc_state};
use crate::trigger_domain::with_trigger_domain;

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
    action: u8,
    profile: u8,
    mode: u8,
    inner_lambda: u32,
    theta_e: u32,
    current_ringing: u8,
    current_bw: u32,
    inner_warning_flags: u8,
    requested_velocity_mrev_s: u32,
    machine_velocity_ceiling_mrev_s: u32,
    requested_velocity_source: u8,
    max_stroke_travel_mrev: u32,
    settle_travel_reserve_mrev: u32,
    negative_position_headroom_mrev: u32,
    positive_position_headroom_mrev: u32,
    max_duration_ms: u16,
    homing_speed_mrev_s: u32,
    max_accel_mrev_s2: u32,
) {
    let channel = match ctx.state().stepper_channel(oid) {
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
        homing_speed_mrev_s,
        max_accel_mrev_s2,
    });
}

fn stepper_execution_snapshot(
    ctx: &mut FirmwareCtx<'_>,
    oid: u8,
) -> Option<StepperExecutionSnapshot> {
    let channel = match ctx.state().stepper_channel(oid) {
        Some(channel) => channel,
        None => {
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
            }
            return None;
        }
    };

    let (timer_active, queue_len) = with_trigger_domain(|td| {
        let queue_len = crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize]
            .len()
            .min(u16::MAX as usize) as u16;
        (td.timer_active(channel as usize), queue_len)
    });
    let position = foci_firmware::state::stepper::stepper_channel(channel)
        .map(|state| state.position.load(Relaxed))
        .unwrap_or(0);
    let snapshot = crate::state::stepper_diag::snapshot(channel, position, timer_active, queue_len);
    if snapshot.is_none() && ctx.enter_shutdown() {
        let clock = ctx.now();
        ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
    }
    snapshot
}

/// Query board-local step queue counters.
#[klipper_command]
pub fn foci_stepper_stats(ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let Some(snapshot) = stepper_execution_snapshot(ctx, oid) else {
        return;
    };

    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociStepperStatsResult,
        oid: u8 = oid,
        channel: u8 = snapshot.channel,
        position: i32 = snapshot.position,
        queued_segments: u32 = snapshot.queued_segments,
        queued_steps: u32 = snapshot.queued_steps,
        loaded_segments: u32 = snapshot.loaded_segments,
        loaded_steps: u32 = snapshot.loaded_steps,
        discarded_segments: u32 = snapshot.discarded_segments,
        discarded_steps: u32 = snapshot.discarded_steps,
        timer_active: u8 = u8::from(snapshot.timer_active),
        queue_len: u16 = snapshot.queue_len,
    );
}

/// Query board-local executed-step and timer-idle counters.
#[klipper_command]
pub fn foci_stepper_exec_stats(ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let Some(snapshot) = stepper_execution_snapshot(ctx, oid) else {
        return;
    };
    let motion = MotionScaleDiagnosticSnapshot::from_state(
        tmc_state(snapshot.channel),
        stepper_state(snapshot.channel),
        step_pulse_timing(),
    );

    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociStepperExecStatsResult,
        oid: u8 = oid,
        channel: u8 = snapshot.channel,
        executed_pos_steps: u32 = snapshot.executed_pos_steps,
        executed_neg_steps: u32 = snapshot.executed_neg_steps,
        physical_pos_pulses: u32 = snapshot.physical_pos_pulses,
        physical_neg_pulses: u32 = snapshot.physical_neg_pulses,
        planner_steps_per_rev: u32 = motion.planner_steps_per_rev,
        encoder_ppr: u32 = motion.encoder_ppr,
        encoder_counts_per_rev: u32 = motion.encoder_counts_per_rev,
        tmc_grid: u32 = motion.tmc_grid,
        physical_step_width: u32 = motion.physical_step_width,
        motion_scale_configured: u8 = u8::from(motion.configured),
        step_half_period_ticks: u32 = motion.step_half_period_ticks,
        dir_setup_ticks: u32 = motion.dir_setup_ticks,
        waveform_worst_case_ticks: u32 = motion.waveform_worst_case_ticks,
        fatal_lateness_ticks: u32 = motion.fatal_lateness_ticks,
        queue_empty_count: u32 = snapshot.queue_empty_count,
        missed_deadline_count: u32 = snapshot.missed_deadline_count,
    );
}

/// Query board-local first-load and first-step timing counters.
#[klipper_command]
pub fn foci_stepper_timing_stats(ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let Some(snapshot) = stepper_execution_snapshot(ctx, oid) else {
        return;
    };

    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociStepperTimingStatsResult,
        oid: u8 = oid,
        channel: u8 = snapshot.channel,
        activation_count: u32 = snapshot.activation_count,
        last_activation_clock: u32 = snapshot.last_activation_clock,
        first_load_now: u32 = snapshot.first_load_now,
        first_load_scheduled: u32 = snapshot.first_load_scheduled,
        first_load_compare: u32 = snapshot.first_load_compare,
        first_load_lead_ticks: i32 = snapshot.first_load_lead_ticks,
        first_load_compare_delay_ticks: i32 = snapshot.first_load_compare_delay_ticks,
        first_step_clock: u32 = snapshot.first_step_clock,
        first_step_delay_ticks: i32 = snapshot.first_step_delay_ticks,
    );
}

/// Query board-local step-dispatch performance counters.
#[klipper_command]
pub fn foci_stepper_perf_stats(ctx: &mut FirmwareCtx<'_>, oid: u8, clear: u8) {
    let channel = match ctx.state().stepper_channel(oid) {
        Some(channel) => channel,
        None => {
            if ctx.enter_shutdown() {
                let clock = ctx.now();
                ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
            }
            return;
        }
    };
    let cur = ctx.now();
    let high = STATS_SEND_TIME_HIGH.load(Relaxed) + u32::from(cur < STATS_SEND_TIME.load(Relaxed));
    let now_ticks = (u64::from(high) << 32) | u64::from(cur);
    let (snapshot, (elapsed_cycles_lo, elapsed_cycles_hi)) =
        crate::state::stepper_diag::perf_snapshot_and_reset_window(channel, clear != 0, now_ticks);

    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociStepperPerfStatsResult,
        oid: u8 = oid,
        channel: u8 = snapshot.channel,
        crit_max_cycles: u32 = snapshot.crit_max_cycles,
        crit_max_site: u8 = snapshot.crit_max_site,
        queue_step_count: u32 = snapshot.queue_step_count,
        queue_step_max_cycles: u32 = snapshot.queue_step_max_cycles,
        tim5_activation_count: u32 = snapshot.tim5_activation_count,
        tim5_irq_max_cycles: u32 = snapshot.tim5_irq_max_cycles,
        tim5_dispatch_max_cycles: u32 = snapshot.tim5_dispatch_max_cycles,
        tim5_dispatch_max_cycles_events: u32 = snapshot.tim5_dispatch_max_cycles_events,
        tim5_events_max_per_irq: u32 = snapshot.tim5_events_max_per_irq,
        tim5_event_count_total: u32 = snapshot.tim5_event_count_total,
        tim5_defer_count: u32 = snapshot.tim5_defer_count,
        tim5_burst_cycles_per_event_max_cycles: u32 = snapshot.tim5_burst_cycles_per_event_max_cycles,
        tim5_burst_cycles_per_event_max_events: u32 = snapshot.tim5_burst_cycles_per_event_max_events,
        tim5_entry_latency_max_ticks: u32 = snapshot.tim5_entry_latency_max_ticks,
        tim5_pop_lateness_max_ticks: u32 = snapshot.tim5_pop_lateness_max_ticks,
        scheduler_cycles_max: u32 = snapshot.scheduler_cycles_max,
        scheduler_cycles_events_at_max: u32 = snapshot.scheduler_cycles_events_at_max,
        scheduler_full_count: u32 = snapshot.scheduler_full_count,
        stepper_load_lateness_max_ticks: u32 = snapshot.stepper_load_lateness_max_ticks,
        build_trace_enabled: u8 = snapshot.build_trace_enabled as u8,
        shutdown_site_count: u32 = snapshot.shutdown_site_count,
        reset_site_count: u32 = snapshot.reset_site_count,
        trigger_stop_site_count: u32 = snapshot.trigger_stop_site_count,
        total_irq_cycles_lo: u32 = snapshot.total_irq_cycles_lo,
        total_irq_cycles_hi: u32 = snapshot.total_irq_cycles_hi,
        total_dispatch_cycles_lo: u32 = snapshot.total_dispatch_cycles_lo,
        total_dispatch_cycles_hi: u32 = snapshot.total_dispatch_cycles_hi,
        elapsed_cycles_lo: u32 = elapsed_cycles_lo,
        elapsed_cycles_hi: u32 = elapsed_cycles_hi,
    );
}

/// Query board-local physical-stop and reset counters.
#[klipper_command]
pub fn foci_stepper_stop_stats(ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let Some(snapshot) = stepper_execution_snapshot(ctx, oid) else {
        return;
    };

    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociStepperStopStatsResult,
        oid: u8 = oid,
        channel: u8 = snapshot.channel,
        stop_count: u32 = snapshot.stop_count,
        stop_drained_segments: u32 = snapshot.stop_drained_segments,
        stop_drained_steps: u32 = snapshot.stop_drained_steps,
        reset_count: u32 = snapshot.reset_count,
        reset_drained_segments: u32 = snapshot.reset_drained_segments,
        reset_drained_steps: u32 = snapshot.reset_drained_steps,
        last_stop_reason: u8 = snapshot.last_stop_reason,
        last_stop_remaining_events: u32 = snapshot.last_stop_remaining_events,
        last_stop_queue_len: u16 = snapshot.last_stop_queue_len,
        last_stop_drained_segments: u32 = snapshot.last_stop_drained_segments,
        last_stop_drained_steps: u32 = snapshot.last_stop_drained_steps,
    );
}
