//! Stepper helpers: TIM5 CC1 arming and P1-safe physical stepper halt.
//! Promoted from `mod app` in `main.rs` by Task 8c so handlers and the
//! step-timer ISR share a single implementation.

use crate::trigger_domain::stop_physical_stepper_no_lock;
use foci_firmware::commission_dispatch::{
    DispatchState, StepperPostAction, StepperTaskDirective, apply_stepper_post_action_state,
};
use foci_firmware::diagnostic::StepperEventReason;

/// Minimum lead time when arming TIM5 CC1.
///
/// If CC1 is programmed too close to the live CNT value, the compare
/// can be missed before CC1IE is fully armed. TIM5 is free-running
/// 32-bit at 260 MHz, so a missed match would not reoccur until wrap
/// (~16 s later).
pub(crate) const STEP_TIMER_ARM_MIN_LEAD_TICKS: u32 = 128;

#[inline]
fn timer_is_before(a: u32, b: u32) -> bool {
    (a as i32).wrapping_sub(b as i32) < 0
}

#[inline]
fn clamp_step_timer_clock(clock: u32) -> u32 {
    let now = embassy_stm32::pac::TIM5.cnt().read();
    let min_clock = now.wrapping_add(STEP_TIMER_ARM_MIN_LEAD_TICKS);
    if timer_is_before(clock, min_clock) {
        min_clock
    } else {
        clock
    }
}

/// Arm the TIM5 CC1 step timer for a compare at `clock`.
///
/// Ensures NVIC is unmasked, clamps too-close compares forward, then
/// enables CC1 interrupt. Does NOT update `TriggerDomain`'s
/// `timer_active` — the caller must do that inside the critical
/// section. Callable from task context or the TIM5 ISR.
pub(crate) fn arm_step_timer(clock: u32) {
    let tim5 = embassy_stm32::pac::TIM5;
    // SAFETY: NVIC::unmask is idempotent; TIM5 is owned by this
    // firmware image and only this helper arms CC1 from task/ISR context.
    unsafe { cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5) };
    let clock = clamp_step_timer_clock(clock);
    tim5.ccr(0).write_value(clock);
    tim5.sr().modify(|w| w.set_ccif(0, false)); // Clear pending CC1
    tim5.dier().modify(|w| w.set_ccie(0, true)); // Enable CC1 interrupt
}

/// Arm TIM5 CC1 only if `clock` is earlier than the active compare.
///
/// This is for task/P2 paths that are adding work for one channel while the
/// shared CC1 timer may already be armed for another channel. It preserves an
/// already-pending compare flag and never clears `CC1IF` while CC1 is enabled.
/// Use [`arm_step_timer`] from the TIM5 ISR after selecting the global earliest
/// due channel.
pub(crate) fn arm_step_timer_if_earlier(clock: u32) {
    let tim5 = embassy_stm32::pac::TIM5;
    let clock = clamp_step_timer_clock(clock);

    // SAFETY: NVIC::unmask is idempotent; TIM5 is owned by this firmware image.
    unsafe { cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5) };

    if !tim5.dier().read().ccie(0) {
        tim5.ccr(0).write_value(clock);
        tim5.sr().modify(|w| w.set_ccif(0, false));
        tim5.dier().modify(|w| w.set_ccie(0, true));
        return;
    }

    if tim5.sr().read().ccif(0) {
        return;
    }

    let current_compare = tim5.ccr(0).read();
    if timer_is_before(clock, current_compare) {
        tim5.ccr(0).write_value(clock);
    }
}

/// Quiesce the physical stepper from a P1 context without entering
/// `TriggerDomain`.
///
/// P1 performs the physical quiesce immediately (CC1 disable, queue
/// drain via `stop_physical_stepper_no_lock`) then spawns
/// `stop_check` at P2 to update the authoritative `timer_active`
/// inside `TriggerDomain`. P1 must never enter `TriggerDomain` per
/// spec.
pub(crate) fn stepper_stop_from_p1(channel: u8) {
    let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
        defmt::error!("stepper_stop_from_p1: invalid channel {}", channel);
        return;
    };
    let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
        defmt::error!("stepper_stop_from_p1: invalid stepper channel {}", channel);
        return;
    };
    tmc_state
        .motor_armed
        .store(false, core::sync::atomic::Ordering::Relaxed);
    stop_physical_stepper_no_lock(channel as usize, StepperEventReason::P1Stop);
    crate::helpers::stepper_diag::record_stepper_event(
        StepperEventReason::P1Stop,
        channel,
        crate::helpers::lifecycle::read_tim5(),
        0,
        0,
        0,
    );
    stepper_state
        .timer_active_mirror
        .store(false, core::sync::atomic::Ordering::Relaxed);
    crate::app::stop_check::spawn(channel).ok();
}

/// Apply a board-owned stepper runtime transition after commissioning dispatch.
///
/// This is the single owner of firmware step-epoch zeroing, channel motor arming,
/// and `arm_check` spawning. Dispatch only decides which transition is needed.
pub(crate) fn apply_stepper_post_action(
    action: StepperPostAction,
    state: &mut DispatchState<'_>,
    channel: u8,
) {
    match apply_stepper_post_action_state(action, state) {
        StepperTaskDirective::None => {}
        StepperTaskDirective::StopCheck => {
            stepper_stop_from_p1(channel);
        }
        StepperTaskDirective::ArmCheck => {
            crate::app::arm_check::spawn(channel).ok();
        }
    }
}
