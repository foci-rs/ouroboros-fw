//! Stepper helpers: TIM5 CC1 arming and P1-safe physical stepper halt.
//! Promoted from `mod app` in `main.rs` by Task 8c so handlers and the
//! step-timer ISR share a single implementation.

use crate::trigger_domain::stop_physical_stepper_no_lock;
use foci_firmware::commission_dispatch::{DispatchState, StepperPostAction};
use foci_firmware::tmc_ops::StepperTaskDirective;

/// Minimum lead time when arming TIM5 CC1 from thread context.
///
/// If CC1 is programmed too close to the live CNT value, the compare
/// can be missed before CC1IE is fully armed. TIM5 is free-running
/// 32-bit at 260 MHz, so a missed match would not reoccur until wrap
/// (~16 s later).
pub(crate) const STEP_TIMER_ARM_MIN_LEAD_TICKS: u32 = 128;

fn tmc_state() -> &'static foci_firmware::state::tmc::TmcChannelState {
    foci_firmware::state::tmc::tmc_channel0()
}

fn stepper_state() -> &'static foci_firmware::state::stepper::StepperChannelState {
    foci_firmware::state::stepper::stepper_channel0()
}

/// Arm the TIM5 CC1 step timer for a compare at `clock`.
///
/// Ensures NVIC is unmasked, clamps too-close compares forward, then
/// enables CC1 interrupt. Does NOT update `TriggerDomain`'s
/// `timer_active` — the caller must do that inside the critical
/// section.
pub(crate) fn arm_step_timer(clock: u32) {
    let tim5 = embassy_stm32::pac::TIM5;
    // SAFETY: NVIC::unmask is idempotent; TIM5 is owned by this
    // firmware image and only this helper arms CC1 from thread
    // context.
    unsafe { cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5) };
    let now = tim5.cnt().read();
    let min_clock = now.wrapping_add(STEP_TIMER_ARM_MIN_LEAD_TICKS);
    let clock = if (clock as i32).wrapping_sub(min_clock as i32) < 0 {
        min_clock
    } else {
        clock
    };
    tim5.ccr(0).write_value(clock);
    tim5.sr().modify(|w| w.set_ccif(0, false)); // Clear pending CC1
    tim5.dier().modify(|w| w.set_ccie(0, true)); // Enable CC1 interrupt
}

/// Quiesce the physical stepper from a P1 context without entering
/// `TriggerDomain`.
///
/// P1 performs the physical quiesce immediately (CC1 disable, queue
/// drain via `stop_physical_stepper_no_lock`) then spawns
/// `stop_check` at P2 to update the authoritative `timer_active`
/// inside `TriggerDomain`. P1 must never enter `TriggerDomain` per
/// spec.
pub(crate) fn stepper_stop_from_p1() {
    tmc_state()
        .motor_armed
        .store(false, core::sync::atomic::Ordering::Relaxed);
    stop_physical_stepper_no_lock(0);
    stepper_state()
        .timer_active_mirror
        .store(false, core::sync::atomic::Ordering::Relaxed);
    crate::app::stop_check::spawn().ok();
}

/// Apply a board-owned stepper runtime transition after commissioning dispatch.
///
/// This is the single owner of firmware step-epoch zeroing, channel motor arming,
/// and `arm_check` spawning. Dispatch only decides which transition is needed.
pub(crate) fn apply_stepper_post_action(action: StepperPostAction, state: &mut DispatchState<'_>) {
    match foci_firmware::tmc_ops::apply_stepper_post_action_state(action, state) {
        StepperTaskDirective::None => {}
        StepperTaskDirective::StopCheck => {
            stepper_stop_from_p1();
        }
        StepperTaskDirective::ArmCheck => {
            crate::app::arm_check::spawn().ok();
        }
    }
}
