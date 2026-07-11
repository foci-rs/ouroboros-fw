//! Stepper-subsystem board-local state (queue, hardware stepper handles).
//! The cross-priority atomics (position, interval, direction, timer mirror)
//! have moved to `foci_firmware::state::stepper`.

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use foci_firmware::step_pulse::StepPulseTiming;
use foci_klipper::pulse_mapper::PulseMapperConfig;
use foci_klipper::stepper::{HardwareStepper, LogicalStepEvent, MotionScaleConfigError, StepMove};

/// Ouroboros TIM5 input clock in hertz.
pub(crate) const STEP_TIMER_HZ: u32 = 260_000_000;

/// Ouroboros' software-audited P3 handler ceiling is not a measurement.
pub(crate) const STEP_TIMING_PROVISIONAL: bool = true;

/// Eleven TIM5 ticks provide the 40 ns minimum high/low pulse phase.
pub(crate) const STEP_HALF_PERIOD_TICKS: u32 = 11;

/// Two half-periods provide the project-owned DIR setup interval.
pub(crate) const STEP_DIR_SETUP_TICKS: u32 = 22;

/// DIR setup plus two complete high/low pulse periods.
pub(crate) const STEP_WAVEFORM_WORST_CASE_TICKS: u32 = 66;

/// PROVISIONAL exception entry/exit, status, lock, and diagnostic wrapper budget.
pub(crate) const PROVISIONAL_ISR_WRAPPER_TICKS: u32 = 512;

/// Maximum CC2 trigger events collected and processed by one TIM5 interrupt.
pub(crate) const PROVISIONAL_MAX_TRIGGER_EVENTS_PER_IRQ: usize = 13;

/// PROVISIONAL processing budget for one CC2 trigger event.
pub(crate) const PROVISIONAL_TRIGGER_EVENT_TICKS: u32 = 2_048;

/// PROVISIONAL fixed `process_due_step_event` entry/exit and empty-queue budget.
pub(crate) const PROVISIONAL_STEPPER_CALLBACK_FIXED_TICKS: u32 = 512;

/// Maximum logical events emitted by one CC1 dispatch.
pub(crate) const PROVISIONAL_MAX_LOGICAL_EVENTS_PER_IRQ: usize = 8;

/// PROVISIONAL mapper arithmetic budget per logical event.
///
/// `map_step` has two signed 64-bit Euclidean quotient/remainder paths. This
/// charges each possible software helper 256 TIM5 ticks (512 CPU cycles),
/// enough for 64 quotient bits at eight CPU cycles per bit.
pub(crate) const PROVISIONAL_MAPPER_DIVISION_TICKS: u32 = 512;

/// PROVISIONAL H723 GPIO, counter-read, loop, and bounded-spin overhead.
///
/// No Ouroboros logic-analyzer timing is available. The exact 66-tick waveform
/// plus this 958-tick term reserves a full 1,024 TIM5 ticks for the unmeasured
/// H723 GPIO and counter path instead of importing OpenFFBoard costs.
pub(crate) const PROVISIONAL_PULSE_CONTROL_TICKS: u32 = 958;

/// PROVISIONAL mapper publication, atomics, and fixed diagnostic queue work.
pub(crate) const PROVISIONAL_EVENT_STATE_TICKS: u32 = 256;

/// PROVISIONAL fixed-capacity move-queue transition budget per event.
///
/// Heapless SPSC dequeue is constant time. Charging this on every event also
/// covers the maximum 1,024 transitions through the fixed queue conservatively.
pub(crate) const PROVISIONAL_QUEUE_TRANSITION_TICKS: u32 = 256;

/// Itemized PROVISIONAL per-event subtotal before upward rounding.
pub(crate) const PROVISIONAL_LOGICAL_EVENT_SUBTOTAL_TICKS: u32 = PROVISIONAL_MAPPER_DIVISION_TICKS
    + STEP_WAVEFORM_WORST_CASE_TICKS
    + PROVISIONAL_PULSE_CONTROL_TICKS
    + PROVISIONAL_EVENT_STATE_TICKS
    + PROVISIONAL_QUEUE_TRANSITION_TICKS;

/// PROVISIONAL rounded per-logical-event control-flow budget.
///
/// The itemized subtotal is 2,048 ticks: 512 mapper + 66 waveform + 958
/// unmeasured H723 pulse control + 256 state publication + 256 queue
/// transition. No runtime measurement contributes to this bound.
pub(crate) const PROVISIONAL_LOGICAL_EVENT_TICKS: u32 =
    PROVISIONAL_LOGICAL_EVENT_SUBTOTAL_TICKS.next_power_of_two();

/// PROVISIONAL final CC1 compare and CC2 scheduler reprogramming budget.
pub(crate) const PROVISIONAL_TIMER_REPROGRAM_TICKS: u32 = 2_048;

/// PROVISIONAL defer, lock-exit, diagnostic, and ISR-tail budget.
pub(crate) const PROVISIONAL_POST_DISPATCH_TICKS: u32 = 1_024;

/// Itemized PROVISIONAL bounded-control-flow subtotal before upward rounding.
pub(crate) const PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS: u32 = PROVISIONAL_ISR_WRAPPER_TICKS
    + PROVISIONAL_MAX_TRIGGER_EVENTS_PER_IRQ as u32 * PROVISIONAL_TRIGGER_EVENT_TICKS
    + PROVISIONAL_STEPPER_CALLBACK_FIXED_TICKS
    + PROVISIONAL_MAX_LOGICAL_EVENTS_PER_IRQ as u32 * PROVISIONAL_LOGICAL_EVENT_TICKS
    + PROVISIONAL_TIMER_REPROGRAM_TICKS
    + PROVISIONAL_POST_DISPATCH_TICKS;

/// PROVISIONAL total P3 handler ceiling derived from bounded control flow.
///
/// One interrupt can process at most 13 CC2 trigger events followed by eight
/// CC1 logical events. The 47,104-tick subtotal also charges fixed callback,
/// timer-reprogram, wrapper, and tail work, then rounds upward to 65,536. This
/// is a deliberately restrictive static audit, not a measurement; the
/// disabled-output Ouroboros logic-analyzer checkpoint remains deferred.
pub(crate) const PROVISIONAL_TOTAL_HANDLER_WCET_TICKS: u32 =
    PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS.next_power_of_two();

/// PROVISIONAL admission margin for flash/bus contention and audit error.
///
/// The separate 8,192 ticks are one eighth of the rounded ceiling and keep
/// nonzero uncertainty headroom outside the control-flow subtotal.
pub(crate) const PROVISIONAL_HANDLER_MARGIN_TICKS: u32 = 8_192;

/// Depth of each hardware step/dir move queue.
pub(crate) const STEP_QUEUE_DEPTH: usize = 1024;

/// Number of hardware stepper runtime channels on Ouroboros.
pub(crate) const STEPPER_CHANNEL_COUNT: usize = crate::board_config::TMC_CHANNEL_COUNT as usize;

/// Physical pulses completed since each channel's most recent mapper installation.
pub(crate) static PHYSICAL_PULSE_COUNTS: [AtomicU32; STEPPER_CHANNEL_COUNT] =
    [const { AtomicU32::new(0) }; STEPPER_CHANNEL_COUNT];

/// Last mapper phase published after STEP returned low for each channel.
pub(crate) static MAPPER_PHASES: [AtomicI32; STEPPER_CHANNEL_COUNT] =
    [const { AtomicI32::new(0) }; STEPPER_CHANNEL_COUNT];

/// Lock-free move queues for hardware step/dir.
///
/// P2 enqueues, P3 dequeues. When timer is disabled, P1/P2 may drain.
///
/// SAFETY: Access protocol enforced by RTIC priorities:
/// - Timer active: P2 enqueues only (Producer), P3 dequeues only (Consumer)
/// - Timer disabled: P1/P2 may drain (Consumer side safe, P3 quiesced)
pub(crate) static mut STEP_QUEUES: [heapless::spsc::Queue<
    foci_klipper::stepper::StepMove,
    STEP_QUEUE_DEPTH,
>; STEPPER_CHANNEL_COUNT] = [const { heapless::spsc::Queue::new() }; STEPPER_CHANNEL_COUNT];

/// P2-local HardwareStepper state (direction tracking, prepare_move).
/// Only accessed from usb_task (P2) command handlers.
pub(crate) static mut P2_STEPPERS: [foci_klipper::stepper::HardwareStepper; STEPPER_CHANNEL_COUNT] =
    [const { foci_klipper::stepper::HardwareStepper::new() }; STEPPER_CHANNEL_COUNT];

/// P3-local HardwareStepper state (active segment, step execution).
/// Only accessed from TIM5 CC1 ISR (P3).
pub(crate) static mut ISR_STEPPERS: [foci_klipper::stepper::HardwareStepper;
    STEPPER_CHANNEL_COUNT] =
    [const { foci_klipper::stepper::HardwareStepper::new() }; STEPPER_CHANNEL_COUNT];

/// Return the hardware step queue for `channel`.
///
/// # Safety
///
/// Callers must uphold Ouroboros' per-channel RTIC access discipline for
/// the returned queue and must not create overlapping mutable references to
/// the same channel.
#[inline]
pub(crate) unsafe fn step_queue(
    channel: u8,
) -> Option<&'static mut heapless::spsc::Queue<foci_klipper::stepper::StepMove, STEP_QUEUE_DEPTH>> {
    let idx = channel as usize;
    if idx >= STEPPER_CHANNEL_COUNT {
        return None;
    }
    let ptr = core::ptr::addr_of_mut!(STEP_QUEUES)
        .cast::<heapless::spsc::Queue<foci_klipper::stepper::StepMove, STEP_QUEUE_DEPTH>>();
    // SAFETY: idx was bounds-checked above. The caller guarantees exclusive
    // mutable access to this channel under the board's RTIC access discipline.
    Some(unsafe { &mut *ptr.add(idx) })
}

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

/// Return Ouroboros' immutable, PROVISIONAL logical-event timing budget.
pub(crate) fn step_pulse_timing() -> StepPulseTiming {
    let timing = StepPulseTiming::new(
        STEP_TIMER_HZ,
        PROVISIONAL_TOTAL_HANDLER_WCET_TICKS,
        PROVISIONAL_HANDLER_MARGIN_TICKS,
    )
    .expect("Ouroboros step timing constants are valid");
    debug_assert_eq!(
        timing.waveform_worst_case_ticks(),
        STEP_WAVEFORM_WORST_CASE_TICKS
    );
    timing
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

/// Prepare and enqueue without consuming P2 direction when the queue is full.
pub(crate) fn prepare_and_enqueue<const N: usize>(
    stepper: &mut HardwareStepper,
    queue: &mut heapless::spsc::Queue<StepMove, N>,
    move_: StepMove,
) -> Result<usize, usize> {
    if queue.is_full() {
        return Err(queue.len());
    }
    let prepared = stepper.prepare_move(move_);
    // P2 is the only producer, so a consumer can only create more space after
    // the full check. Enqueue therefore cannot fail after direction changes.
    queue.enqueue(prepared).map_err(|_| queue.len())?;
    Ok(queue.len())
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
    fn ouroboros_timing_uses_provisional_wcet_and_margin() {
        let timing = step_pulse_timing();
        assert!(STEP_TIMING_PROVISIONAL);
        assert_eq!(timing.half_period_ticks(), 11);
        assert_eq!(timing.dir_setup_ticks(), 22);
        assert_eq!(timing.waveform_worst_case_ticks(), 66);
        assert_eq!(timing.half_period_ticks(), STEP_HALF_PERIOD_TICKS);
        assert_eq!(timing.dir_setup_ticks(), STEP_DIR_SETUP_TICKS);
        assert_eq!(
            timing.waveform_worst_case_ticks(),
            STEP_WAVEFORM_WORST_CASE_TICKS
        );
        assert_eq!(
            timing.total_handler_wcet_ticks(),
            PROVISIONAL_TOTAL_HANDLER_WCET_TICKS
        );
        assert_eq!(timing.margin_ticks(), PROVISIONAL_HANDLER_MARGIN_TICKS);
        assert_eq!(
            PROVISIONAL_MAX_LOGICAL_EVENTS_PER_IRQ,
            foci_firmware::stepper_runtime::MAX_STEP_EVENTS_PER_IRQ
        );
        assert_eq!(PROVISIONAL_MAX_TRIGGER_EVENTS_PER_IRQ, 13);
        assert_eq!(
            STEP_WAVEFORM_WORST_CASE_TICKS + PROVISIONAL_PULSE_CONTROL_TICKS,
            1_024
        );
        assert_eq!(PROVISIONAL_LOGICAL_EVENT_SUBTOTAL_TICKS, 2_048);
        assert_eq!(PROVISIONAL_LOGICAL_EVENT_TICKS, 2_048);
        assert_eq!(PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS, 47_104);
        assert!(PROVISIONAL_TOTAL_HANDLER_WCET_TICKS >= PROVISIONAL_CONTROL_FLOW_SUBTOTAL_TICKS);
        assert_eq!(PROVISIONAL_TOTAL_HANDLER_WCET_TICKS, 65_536);
        assert_eq!(PROVISIONAL_HANDLER_MARGIN_TICKS, 8_192);
    }

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

    #[test]
    fn queue_full_does_not_consume_pending_direction_change() {
        let mut stepper = foci_klipper::stepper::HardwareStepper::new();
        stepper.set_next_step_dir(false);
        let mut queue = heapless::spsc::Queue::<StepMove, 2>::new();
        queue.enqueue(StepMove::new(100, 1, 0)).unwrap();

        assert_eq!(
            prepare_and_enqueue(&mut stepper, &mut queue, StepMove::new(100, 1, 0)),
            Err(1)
        );
        assert!(stepper.next_move_changes_direction());
    }
}
