//! Stepper-subsystem board-local state (queue, hardware stepper handles).
//! The cross-priority atomics (position, interval, direction, timer mirror)
//! have moved to `foci_firmware::state::stepper`.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use foci_klipper::pulse_mapper::PulseMapperConfig;
use foci_klipper::stepper::{HardwareStepper, LogicalStepEvent, MotionScaleConfigError};

/// Ouroboros TIM5 input clock in hertz.
pub(crate) const STEP_TIMER_HZ: u32 = crate::ankyra::constants::CLOCK_FREQ;

/// Eleven TIM5 ticks provide the 40 ns minimum high/low pulse phase.
pub(crate) const STEP_HALF_PERIOD_TICKS: u32 = 11;

/// Two half-periods provide the project-owned DIR setup interval.
pub(crate) const STEP_DIR_SETUP_TICKS: u32 = 22;

pub(crate) use foci_klipper::step_queue::STEP_QUEUE_DEPTH;

/// Number of hardware stepper runtime channels on Ouroboros.
pub(crate) const STEPPER_CHANNEL_COUNT: usize = crate::board_config::TMC_CHANNEL_COUNT as usize;

/// Physical pulses completed since each channel's most recent mapper installation.
pub(crate) static PHYSICAL_PULSE_COUNTS: [AtomicU32; STEPPER_CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; STEPPER_CHANNEL_COUNT];

/// Last mapper phase published after STEP returned low for each channel.
pub(crate) static MAPPER_PHASES: [AtomicI32; STEPPER_CHANNEL_COUNT] =
    [const { AtomicI32::new(0) }; STEPPER_CHANNEL_COUNT];

/// Lock-free step/dir move queues with explicit producer/consumer ownership.
/// Split once at RTIC `init` via `crate::main::init` -- never touched
/// through the old unsafe accessor again.
pub(crate) static STEP_QUEUE_CHANNELS: [foci_klipper::step_queue::StepQueueChannel<
    STEP_QUEUE_DEPTH,
>; STEPPER_CHANNEL_COUNT] =
    [const { foci_klipper::step_queue::StepQueueChannel::new() }; STEPPER_CHANNEL_COUNT];

/// P2-owned producer handles, populated once by `init_step_queues`. Nothing
/// else ever touches them, so aliasing is inexpressible.
pub(crate) static mut STEP_QUEUE_PRODUCERS: [Option<
    foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>,
>; STEPPER_CHANNEL_COUNT] = [const { None }; STEPPER_CHANNEL_COUNT];

/// Split each `STEP_QUEUE_CHANNELS` entry once and store its producer half.
/// Must be called exactly once, from RTIC `init`, before any other stepper
/// task runs.
///
/// # Safety
/// Caller (RTIC `init`) guarantees this runs exactly once, before any
/// other task touches `STEP_QUEUE_PRODUCERS`/`STEP_QUEUE_CHANNELS`.
pub(crate) unsafe fn init_step_queues() {
    // SAFETY: `init` runs once, before any other task starts. `addr_of_mut!`
    // plus a raw-pointer element write avoids the edition-2024
    // `static_mut_refs` lint (never spells `&mut STEP_QUEUE_PRODUCERS`
    // directly), matching this file's existing `P2_STEPPERS`/`ISR_STEPPERS`
    // accessor pattern.
    let base = core::ptr::addr_of_mut!(STEP_QUEUE_PRODUCERS).cast::<Option<
        foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>,
    >>();
    for (idx, channel) in STEP_QUEUE_CHANNELS.iter().enumerate() {
        unsafe {
            *base.add(idx) = Some(channel.split_once());
        }
    }
}

/// Borrow the P2-owned producer for `channel`. Returns `None` for an
/// out-of-range channel or if `init_step_queues` has not yet run.
///
/// # Safety
/// Callable only from P2, or from P1 inside `with_trigger_domain` (the
/// commissioning stimulus): either context excludes every other producer.
pub(crate) unsafe fn step_queue_producer(
    channel: u8,
) -> Option<&'static mut foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>>
{
    let idx = channel as usize;
    if idx >= STEPPER_CHANNEL_COUNT {
        return None;
    }
    // SAFETY: idx was bounds-checked above; caller guarantees exclusive
    // producer access; `init_step_queues` has run by the time any P2 command
    // handler executes. `addr_of_mut!` avoids the edition-2024
    // `static_mut_refs` lint the same way as `init_step_queues` above.
    let base = core::ptr::addr_of_mut!(STEP_QUEUE_PRODUCERS).cast::<Option<
        foci_klipper::step_queue::Producer<'static, foci_klipper::stepper::StepMove>,
    >>();
    (unsafe { &mut *base.add(idx) }).as_mut()
}

/// Producer-side HardwareStepper state (direction tracking, prepare_move).
/// Accessed from usb_task (P2) command handlers, and from the P1
/// commissioning stimulus inside `with_trigger_domain`.
pub(crate) static mut P2_STEPPERS: [foci_klipper::stepper::HardwareStepper; STEPPER_CHANNEL_COUNT] =
    [const { foci_klipper::stepper::HardwareStepper::new() }; STEPPER_CHANNEL_COUNT];

/// P3-local HardwareStepper state (active segment, step execution).
/// Only accessed from TIM5 CC1 ISR (P3).
pub(crate) static mut ISR_STEPPERS: [foci_klipper::stepper::HardwareStepper;
    STEPPER_CHANNEL_COUNT] =
    [const { foci_klipper::stepper::HardwareStepper::new() }; STEPPER_CHANNEL_COUNT];

/// Return the P2-local hardware stepper state for `channel`.
///
/// # Safety
///
/// Callers must uphold Ouroboros' per-channel RTIC access discipline for
/// the returned stepper state and must not create overlapping mutable
/// references to the same channel.
#[inline]
pub(crate) unsafe fn p2_stepper(
    channel: u8,
) -> Option<&'static mut foci_klipper::stepper::HardwareStepper> {
    let idx = channel as usize;
    if idx >= STEPPER_CHANNEL_COUNT {
        return None;
    }
    let ptr = core::ptr::addr_of_mut!(P2_STEPPERS).cast::<foci_klipper::stepper::HardwareStepper>();
    // SAFETY: idx was bounds-checked above. The caller guarantees exclusive
    // mutable access to this channel under the board's RTIC access discipline.
    Some(unsafe { &mut *ptr.add(idx) })
}

/// Return the ISR-local hardware stepper state for `channel`.
///
/// # Safety
///
/// Callers must uphold Ouroboros' per-channel RTIC access discipline for
/// the returned stepper state and must not create overlapping mutable
/// references to the same channel.
#[inline]
pub(crate) unsafe fn isr_stepper(
    channel: u8,
) -> Option<&'static mut foci_klipper::stepper::HardwareStepper> {
    let idx = channel as usize;
    if idx >= STEPPER_CHANNEL_COUNT {
        return None;
    }
    let ptr =
        core::ptr::addr_of_mut!(ISR_STEPPERS).cast::<foci_klipper::stepper::HardwareStepper>();
    // SAFETY: idx was bounds-checked above. The caller guarantees exclusive
    // mutable access to this channel under the board's RTIC access discipline.
    Some(unsafe { &mut *ptr.add(idx) })
}

/// Clear any old mapper, then install a freshly centered replacement.
///
/// Failure is fail-closed: the prior mapper is cleared before configuration.
pub(crate) fn install_motion_scale_mapper(
    stepper: &mut HardwareStepper,
    config: PulseMapperConfig,
) -> Result<(), MotionScaleConfigError> {
    stepper.clear_motion_scale();
    stepper.configure_motion_scale(config)
}

/// Failure to replace the board-local mapper while fail-closed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MotionScaleMapperReplacementError {
    /// A board quiescence guard rejected installation.
    NotAdmitted,
    /// The scheduler rejected the replacement configuration.
    InvalidState,
}

/// Clear any prior mapper before applying board-hook admission and replacement.
pub(crate) fn replace_motion_scale_mapper(
    stepper: &mut HardwareStepper,
    config: PulseMapperConfig,
    admitted: bool,
) -> Result<(), MotionScaleMapperReplacementError> {
    stepper.clear_motion_scale();
    if !admitted {
        return Err(MotionScaleMapperReplacementError::NotAdmitted);
    }
    install_motion_scale_mapper(stepper, config)
        .map_err(|_| MotionScaleMapperReplacementError::InvalidState)
}

/// Admit mapper installation only while every board quiescence guard holds.
pub(crate) const fn motion_scale_install_admitted(
    motor_armed: bool,
    queue_empty: bool,
    timer_active: bool,
) -> bool {
    !motor_armed && queue_empty && !timer_active
}

/// Publish mapper-observable state after a channel's complete pulse train is low.
pub(crate) fn publish_logical_event(
    channel: u8,
    stepper: &HardwareStepper,
    event: LogicalStepEvent,
) {
    let idx = channel as usize;
    PHYSICAL_PULSE_COUNTS[idx].fetch_add(u32::from(event.physical_pulses), Ordering::Relaxed);
    let phase = stepper
        .mapper_phase()
        .expect("logical event requires an installed mapper");
    MAPPER_PHASES[idx].store(phase as i32, Ordering::Relaxed);
}

/// Reset mapper-observable counters for one channel at its new common origin.
pub(crate) fn reset_mapper_observables(channel: u8) {
    let idx = channel as usize;
    PHYSICAL_PULSE_COUNTS[idx].store(0, Ordering::Relaxed);
    MAPPER_PHASES[idx].store(0, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_klipper::pulse_mapper::PulseMapperConfig;
    use foci_klipper::stepper::StepMove;

    #[test]
    fn failed_mapper_replacement_clears_prior_mapper() {
        let mut stepper = foci_klipper::stepper::HardwareStepper::new();
        stepper
            .configure_motion_scale(PulseMapperConfig::new(3_200).unwrap())
            .unwrap();
        stepper.load_move(StepMove::new(100, 2, 0));
        let _ = stepper.step_event();
        let prior_phase = stepper.mapper_phase().unwrap();
        PHYSICAL_PULSE_COUNTS[0].store(9, Ordering::Relaxed);
        MAPPER_PHASES[0].store(prior_phase as i32, Ordering::Relaxed);

        assert!(
            install_motion_scale_mapper(&mut stepper, PulseMapperConfig::new(4_000).unwrap())
                .is_err()
        );
        assert!(!stepper.motion_scale_configured());
        assert_eq!(stepper.planner_steps_per_rev(), None);
        assert_eq!(stepper.mapper_phase(), None);
        reset_mapper_observables(0);
    }

    #[test]
    fn mapper_install_admission_requires_every_idle_guard() {
        assert!(motion_scale_install_admitted(false, true, false));
        assert!(!motion_scale_install_admitted(true, true, false));
        assert!(!motion_scale_install_admitted(false, false, false));
        assert!(!motion_scale_install_admitted(false, true, true));
    }

    #[test]
    fn busy_mapper_replacement_clears_prior_mapper_for_every_hook_guard() {
        let replacement = PulseMapperConfig::new(4_000).unwrap();

        for guards in [
            (true, true, false),
            (false, false, false),
            (false, true, true),
        ] {
            let mut stepper = foci_klipper::stepper::HardwareStepper::new();
            stepper
                .configure_motion_scale(PulseMapperConfig::new(3_200).unwrap())
                .unwrap();
            let admitted = motion_scale_install_admitted(guards.0, guards.1, guards.2);

            assert!(replace_motion_scale_mapper(&mut stepper, replacement, admitted).is_err());
            assert!(!stepper.motion_scale_configured());
        }
    }
}
