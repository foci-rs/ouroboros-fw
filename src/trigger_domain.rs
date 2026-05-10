//! Trigger domain: the critical-section-guarded coordination point
//! between the step-queue producer (P2 context), the step-timer ISR
//! (P3 context), and the trsync/endstop subsystem.
//!
//! Promoted from `mod app` in Task 8b2 — previously declared inline
//! alongside the RTIC state it coordinates. Moving it here alongside
//! the subsystem `state/*` modules makes the coordination surface
//! inspectable independently of RTIC's macro expansion.

use crate::state::stepper::{ISR_STEPPER, STEP_QUEUE};
use foci_firmware::state::trsync::HOMING_TRIGGERED;

// ---- TriggerDomain: unified trsync/endstop/stepper-stop resource ----

/// Physical stepper indices that drive real motors via TIM5 CC1.
/// The current compile-only scaffold drives channel 0 physically.
pub(crate) const PHYSICAL_STEPPER_INDICES: &[usize] = &[0];

fn tmc_state() -> &'static foci_firmware::state::tmc::TmcChannelState {
    foci_firmware::state::tmc::tmc_channel0()
}

fn stepper_state() -> &'static foci_firmware::state::stepper::StepperChannelState {
    foci_firmware::state::stepper::stepper_channel0()
}

/// Unified resource owning all trsync, endstop, and stepper-stop state.
///
/// Accessed via `with_trigger_domain()` which uses
/// `cortex_m::interrupt::free()` for an interrupt-disabled critical
/// section, equivalent to RTIC P3 ceiling. This is NOT an RTIC shared
/// resource — it is a `static mut` with manual locking.
pub(crate) struct TriggerDomain {
    pub(crate) trsync: [foci_klipper::trigger::TrSyncRuntime; foci_klipper::state::NUM_TRSYNC],
    pub(crate) trsync_by_oid: heapless::LinearMap<u8, usize, { foci_klipper::state::NUM_TRSYNC }>,
    pub(crate) stop_signals: [Option<foci_klipper::trigger::StopSignal>; 8],
    pub(crate) stepper_stop:
        [foci_klipper::trigger::StepperStopRuntime; foci_klipper::state::NUM_STEPPERS],
    pub(crate) endstops: [foci_klipper::endstop::EndstopRuntime; foci_klipper::state::NUM_ENDSTOPS],
    pub(crate) endstop_by_oid:
        heapless::LinearMap<u8, usize, { foci_klipper::state::NUM_ENDSTOPS }>,
    pub(crate) scheduler: foci_klipper::scheduler::TriggerScheduler,
}

impl TriggerDomain {
    pub(crate) const fn new() -> Self {
        Self {
            trsync: [const { foci_klipper::trigger::TrSyncRuntime::new() };
                foci_klipper::state::NUM_TRSYNC],
            trsync_by_oid: heapless::LinearMap::new(),
            stop_signals: [None; 8],
            stepper_stop: [const { foci_klipper::trigger::StepperStopRuntime::new() };
                foci_klipper::state::NUM_STEPPERS],
            endstops: [const { foci_klipper::endstop::EndstopRuntime::new() };
                foci_klipper::state::NUM_ENDSTOPS],
            endstop_by_oid: heapless::LinearMap::new(),
            scheduler: foci_klipper::scheduler::TriggerScheduler::new(),
        }
    }

    /// Set `timer_active` for a stepper, updating the P1 mirror
    /// for physical steppers.
    pub(crate) fn set_timer_active(&mut self, stepper_idx: usize, active: bool) {
        self.stepper_stop[stepper_idx].timer_active = active;
        // Mirror to P1-readable atomic for physical steppers
        if stepper_idx == 0 {
            stepper_state()
                .timer_active_mirror
                .store(active, core::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Read `timer_active` for a stepper.
    pub(crate) fn timer_active(&self, stepper_idx: usize) -> bool {
        self.stepper_stop[stepper_idx].timer_active
    }

    /// Attempt to trigger a trsync, dispatching stop signals to all
    /// linked steppers. Returns the trigger result.
    ///
    /// Must be called inside `with_trigger_domain()`.
    pub(crate) fn trigger(
        &mut self,
        trsync_oid: u8,
        reason: u8,
        clock: u32,
    ) -> foci_klipper::trigger::TriggerResult {
        let idx = match self.trsync_by_oid.get(&trsync_oid) {
            Some(&idx) => idx,
            None => {
                return foci_klipper::trigger::TriggerResult::UnknownOid;
            }
        };
        let result = self.trsync[idx].trigger(trsync_oid, reason, clock);
        if matches!(result, foci_klipper::trigger::TriggerResult::First { .. }) {
            // Collect stepper indices from matching stop signals, then
            // clear the slots. Two-pass avoids borrow conflict between
            // stop_signals iteration and set_timer_active(&mut self).
            let mut stopped_indices: heapless::Vec<usize, 8> = heapless::Vec::new();
            for slot in self.stop_signals.iter_mut() {
                if let Some(signal) = slot
                    && signal.trsync_oid == trsync_oid
                {
                    let _ = stopped_indices.push(signal.stepper_idx);
                    *slot = None;
                }
            }
            // Dispatch physical stops and update stepper state
            for &stepper_idx in &stopped_indices {
                stop_physical_stepper_no_lock(stepper_idx);
                self.stepper_stop[stepper_idx].need_reset = true;
                self.stepper_stop[stepper_idx].blocked_by_trigger = true;
                self.stepper_stop[stepper_idx].blocked_by_trsync = Some(trsync_oid);
                self.set_timer_active(stepper_idx, false);
            }
            // Signal homing I-suppression only when a stop signal
            // actually fired — prevents unrelated trsync triggers
            // from advancing the suppression state machine.
            if !stopped_indices.is_empty() {
                HOMING_TRIGGERED.store(trsync_oid, core::sync::atomic::Ordering::Relaxed);
            }
        }
        result
    }

    /// Reprogram TIM5 CC2 from the scheduler's earliest deadline.
    ///
    /// Called inside the TriggerDomain lock from both P2 command handlers
    /// and the P3 ISR after processing events.
    pub(crate) fn reprogram_cc2(&mut self) {
        let tim5 = embassy_stm32::pac::TIM5;
        if let Some(earliest) = self.scheduler.earliest_deadline() {
            let now = tim5.cnt().read();
            let delta = earliest.wrapping_sub(now) as i32;
            let clamped = if delta <= TRIGGER_TIMER_MIN_LEAD_TICKS as i32 {
                now.wrapping_add(TRIGGER_TIMER_MIN_LEAD_TICKS)
            } else {
                earliest
            };
            tim5.ccr(1).write_value(clamped);
            tim5.sr().modify(|w| w.set_ccif(1, false));
            tim5.dier().modify(|w| w.set_ccie(1, true));
            // Ensure NVIC is unmasked (idempotent)
            unsafe {
                cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5);
            }
            self.scheduler.set_cc2_active(true);
        } else {
            tim5.dier().modify(|w| w.set_ccie(1, false));
            self.scheduler.set_cc2_active(false);
        }
    }
}

/// Minimum lead time for TIM5 CC2 arming (trigger scheduler).
/// 128 ticks at 260 MHz = ~0.5 us.
pub(crate) const TRIGGER_TIMER_MIN_LEAD_TICKS: u32 = 128;

/// TriggerDomain static. Initialized in RTIC `init`, accessed via
/// `with_trigger_domain()`.
pub(crate) static mut TRIGGER_DOMAIN: core::mem::MaybeUninit<TriggerDomain> =
    core::mem::MaybeUninit::uninit();

/// Lock TriggerDomain with interrupt-disabled critical section.
///
/// Equivalent to P3 ceiling since P3 is the highest application priority.
/// Callable from P2 command handlers and P3 ISR.
///
/// Uses `addr_of_mut!` to avoid the `static_mut_refs` lint (matching the
/// `STEP_QUEUE` access pattern in the existing codebase).
#[inline]
pub(crate) fn with_trigger_domain<F, R>(f: F) -> R
where
    F: FnOnce(&mut TriggerDomain) -> R,
{
    cortex_m::interrupt::free(|_| {
        // SAFETY: All access to TRIGGER_DOMAIN goes through this function.
        // interrupt::free ensures exclusive access across all priorities.
        // addr_of_mut! avoids creating a reference to the static mut.
        unsafe { f(&mut *core::ptr::addr_of_mut!(TRIGGER_DOMAIN).cast::<TriggerDomain>()) }
    })
}

/// Stop a physical stepper without locking TriggerDomain.
///
/// Called from two contexts:
/// 1. Inside `with_trigger_domain()` during trigger dispatch (P2 holding
///    an `interrupt::free` critical section; P2 producers cannot preempt).
/// 2. From [`crate::helpers::stepper::stepper_stop_from_p1`] (P1 context
///    with no critical section; P2 producers can preempt).
///
/// Only acts on physical stepper indices (CC1 timer, GPIO, queue).
///
/// Per spec: disable CC1, clear interval, reset ISR stepper engine,
/// drain queue, reset direction GPIO.
pub(crate) fn stop_physical_stepper_no_lock(stepper_idx: usize) {
    use core::sync::atomic::Ordering::Relaxed;
    if !PHYSICAL_STEPPER_INDICES.contains(&stepper_idx) {
        return;
    }
    let tim5 = embassy_stm32::pac::TIM5;
    // 1. Disable CC1 interrupt — quiesce P3 step timer
    tim5.dier().modify(|w| w.set_ccie(0, false));
    tim5.sr().modify(|w| w.set_ccif(0, false));
    // 2. Clear step interval
    stepper_state().interval.store(0, Relaxed);
    tmc_state().pid_error_sums_clear.store(true, Relaxed);
    // 3. Reset ISR stepper engine state (clear stale partial segment)
    // SAFETY: ISR_STEPPER is exclusively touched by the P3 step-timer IRQ,
    // which is quiesced by the CC1IE disable above. Sole accessor here.
    unsafe {
        (*core::ptr::addr_of_mut!(ISR_STEPPER)).reset();
    }
    // 4. Drain queue.
    //
    // SAFETY: STEP_QUEUE is a `heapless::spsc::Queue` — single-producer
    // (P2 `queue_step_move`), single-consumer (P3 step-timer IRQ). The P3
    // consumer is quiesced by the CC1IE disable above; here we act as the
    // consumer and call `dequeue()`. The SPSC implementation supports
    // concurrent producer/consumer access without a lock, so a P2 enqueue
    // that races this drain is well-defined.
    //
    // Caller invariants for entries that survive the drain (i.e. P2
    // enqueues completing after the loop exits):
    // - From `with_trigger_domain` context: the `interrupt::free` critical
    //   section blocks P2 producers, so no enqueue can race the drain.
    // - From `stepper_stop_from_p1`: the caller has already cleared the
    //   channel's motor-armed flag, so the next `arm_check` and the step-timer
    //   IRQ both bail; any surviving entry is gated from execution and either
    //   gets processed at the next motor enable (against the freshly-zeroed
    //   channel position applied by `apply_stepper_post_action_state`) or is
    //   drained again by the next `stepper_stop_from_p1`.
    unsafe { while (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue().is_some() {} }
    // 5. Reset direction GPIO (PD3) to known low state.
    let gpio_d = embassy_stm32::pac::GPIOD;
    gpio_d
        .odr()
        .modify(|w| w.set_odr(3, embassy_stm32::pac::gpio::vals::Odr::LOW));
    stepper_state().direction.store(true, Relaxed); // Forward = default
    // 6. Reset STEP GPIO (PD4) to LOW.
    //
    // Full-mode scheduling relies on the ISR toggling from a known
    // starting state: the first event of each segment must drive STEP
    // LOW → HIGH (the rising edge the TMC4671 counts). If the previous
    // segment ended mid-pulse (shutdown or stop between step and unstep
    // events) the pin could still be HIGH, which would flip the parity
    // for the next segment and lose the first motor step.
    gpio_d
        .odr()
        .modify(|w| w.set_odr(4, embassy_stm32::pac::gpio::vals::Odr::LOW));
}
