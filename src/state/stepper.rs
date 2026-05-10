//! Stepper-subsystem board-local state (queue, hardware stepper handles).
//! The cross-priority atomics (position, interval, direction, timer mirror)
//! have moved to `foci_firmware::state::stepper`.

/// Lock-free move queue for hardware step/dir.
/// P2 enqueues, P3 dequeues. When timer is disabled, P1/P2 may drain.
///
/// SAFETY: Access protocol enforced by RTIC priorities:
/// - Timer active: P2 enqueues only (Producer), P3 dequeues only (Consumer)
/// - Timer disabled: P1/P2 may drain (Consumer side safe, P3 quiesced)
pub(crate) static mut STEP_QUEUE: heapless::spsc::Queue<foci_klipper::stepper::StepMove, 1024> =
    heapless::spsc::Queue::new();

/// P2-local HardwareStepper state (direction tracking, prepare_move).
/// Only accessed from usb_task (P2) command handlers.
pub(crate) static mut P2_STEPPER: foci_klipper::stepper::HardwareStepper =
    foci_klipper::stepper::HardwareStepper::new();

/// P3-local HardwareStepper state (active segment, step execution).
/// Only accessed from TIM5 CC1 ISR (P3).
pub(crate) static mut ISR_STEPPER: foci_klipper::stepper::HardwareStepper =
    foci_klipper::stepper::HardwareStepper::new();
