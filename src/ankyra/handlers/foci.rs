//! Board-local FOCI handlers: the stepper execution counters.

use core::sync::atomic::Ordering::Relaxed;

use ankyra::prelude::klipper_command;
use foci_firmware::context::{ClockSource, ShutdownRecovery, StateAccess};
use foci_firmware::diagnostic::StepperExecutionSnapshot;
use foci_firmware::shutdown::ShutdownReason;

use crate::firmware_ctx::FirmwareCtx;
use crate::tmc_control::board::{stepper_state, tmc_state};
use crate::trigger_domain::with_trigger_domain;

fn stepper_execution_snapshot(
    ctx: &mut FirmwareCtx<'_>,
    oid: u8,
) -> Option<StepperExecutionSnapshot> {
    let Some(channel) = ctx.state().stepper_channel(oid) else {
        if ctx.enter_shutdown() {
            let clock = ctx.now();
            ctx.send_shutdown(ShutdownReason::InvalidTmcChannel, clock);
        }
        return None;
    };

    let (timer_active, queue_len) = with_trigger_domain(|td| {
        let queue_len = crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize]
            .len()
            .min(u16::MAX as usize) as u16;
        (
            crate::trigger_domain::timer_active(td.state(), channel as usize),
            queue_len,
        )
    });
    let position = foci_firmware::state::stepper::stepper_channel(channel)
        .map_or(0, |state| state.position.load(Relaxed));
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
        oversize_frame_drops: u32 = ::ankyra::oversize_frame_drops(),
    );
}

/// Query board-local executed-step and timer-idle counters.
#[klipper_command]
pub fn foci_stepper_exec_stats(ctx: &mut FirmwareCtx<'_>, oid: u8) {
    let Some(snapshot) = stepper_execution_snapshot(ctx, oid) else {
        return;
    };
    let tmc = tmc_state(snapshot.channel);
    let planner_steps_per_rev = stepper_state(snapshot.channel)
        .planner_steps_per_rev(tmc)
        .unwrap_or(0);
    let encoder_ppr = tmc.motion_scale.encoder_ppr().unwrap_or(0);

    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociStepperExecStatsResult,
        oid: u8 = oid,
        executed_pos_steps: u32 = snapshot.executed_pos_steps,
        executed_neg_steps: u32 = snapshot.executed_neg_steps,
        physical_pos_pulses: u32 = snapshot.physical_pos_pulses,
        physical_neg_pulses: u32 = snapshot.physical_neg_pulses,
        planner_steps_per_rev: u32 = planner_steps_per_rev,
        encoder_ppr: u32 = encoder_ppr,
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
