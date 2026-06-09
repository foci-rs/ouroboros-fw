//! Trigger domain: the critical-section-guarded coordination point
//! between the step-queue producer (P2 context), the step-timer ISR
//! (P3 context), and the trsync/endstop subsystem.
//!
//! Promoted from `mod app` in Task 8b2 — previously declared inline
//! alongside the RTIC state it coordinates. Moving it here alongside
//! the subsystem `state/*` modules makes the coordination surface
//! inspectable independently of RTIC's macro expansion.

use core::ops::{Deref, DerefMut};

use crate::state::stepper::{isr_stepper, step_queue};
use foci_firmware::context::trsync::{self, TriggerDispatchBoard, TriggerDomainState};
use foci_firmware::diagnostic::StepperEventReason;

// ---- TriggerDomain: unified trsync/endstop/stepper-stop resource ----

/// Physical stepper indices that drive real motors via TIM5 CC1.
/// Ouroboros production-v1 has two physical stepper channels.
pub(crate) const PHYSICAL_STEPPER_INDICES: &[usize] = &[0, 1];

fn tmc_state(channel: u8) -> Option<&'static foci_firmware::state::tmc::TmcChannelState> {
    foci_firmware::state::tmc::tmc_channel(channel)
}

fn stepper_state(
    channel: u8,
) -> Option<&'static foci_firmware::state::stepper::StepperChannelState> {
    foci_firmware::state::stepper::stepper_channel(channel)
}

/// Unified resource owning all trsync, endstop, and stepper-stop state.
///
/// Accessed via `with_trigger_domain()` which uses
/// `cortex_m::interrupt::free()` for an interrupt-disabled critical
/// section, equivalent to RTIC P3 ceiling. This is NOT an RTIC shared
/// resource — it is a `static mut` with manual locking.
pub(crate) struct TriggerDomain {
    state: TriggerDomainState,
}

impl TriggerDomain {
    pub(crate) const fn new() -> Self {
        Self {
            state: TriggerDomainState::new(),
        }
    }

    pub(crate) fn state_mut(&mut self) -> &mut TriggerDomainState {
        &mut self.state
    }

    pub(crate) fn state(&self) -> &TriggerDomainState {
        &self.state
    }

    /// Set `timer_active` for a stepper, updating the P1 mirror
    /// for physical steppers.
    pub(crate) fn set_timer_active(&mut self, stepper_idx: usize, active: bool) {
        set_timer_active(&mut self.state, stepper_idx, active);
    }

    /// Return whether the step timer is active for a stepper.
    pub(crate) fn timer_active(&self, stepper_idx: usize) -> bool {
        self.state
            .stepper_stop
            .get(stepper_idx)
            .map(|runtime| runtime.timer_active)
            .unwrap_or(false)
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
        let mut board = OuroborosDispatchBoard;
        trsync::dispatch_trigger(&mut self.state, &mut board, trsync_oid, reason, clock)
    }
}

impl Deref for TriggerDomain {
    type Target = TriggerDomainState;

    fn deref(&self) -> &Self::Target {
        &self.state
    }
}

impl DerefMut for TriggerDomain {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.state
    }
}

pub(crate) struct OuroborosDispatchBoard;

impl TriggerDispatchBoard for OuroborosDispatchBoard {
    fn stop_physical_stepper(
        &mut self,
        _state: &TriggerDomainState,
        stepper_idx: usize,
        reason: StepperEventReason,
    ) {
        stop_physical_stepper_no_lock(stepper_idx, reason);
    }

    fn record_trsync_stop(&mut self, stepper_idx: usize, clock: u32, trsync_oid: u8, reason: u8) {
        crate::helpers::stepper_diag::record_stepper_event(
            StepperEventReason::TrsyncStop,
            stepper_idx as u8,
            clock,
            0,
            trsync_oid as u32,
            reason as u32,
        );
    }

    fn set_timer_active(
        &mut self,
        state: &mut TriggerDomainState,
        stepper_idx: usize,
        active: bool,
    ) {
        set_timer_active(state, stepper_idx, active);
    }
}

pub(crate) fn set_timer_active(state: &mut TriggerDomainState, stepper_idx: usize, active: bool) {
    state.stepper_stop[stepper_idx].timer_active = active;
    // Mirror to P1-readable atomic for physical steppers
    if let Some(state) = stepper_state(stepper_idx as u8) {
        state
            .timer_active_mirror
            .store(active, core::sync::atomic::Ordering::Relaxed);
    }
}

pub(crate) fn disarm_trigger_scheduler(state: &mut TriggerDomainState) {
    let tim5 = embassy_stm32::pac::TIM5;
    tim5.dier().modify(|w| w.set_ccie(1, false));
    state.scheduler.set_cc2_active(false);
}

pub(crate) fn reprogram_cc2_measured(
    state: &mut TriggerDomainState,
    scheduler_cycles: &core::cell::Cell<u32>,
) {
    reprogram_cc2_inner(state, Some(scheduler_cycles));
}

pub(crate) fn reprogram_cc2(state: &mut TriggerDomainState) {
    reprogram_cc2_inner(state, None);
}

fn reprogram_cc2_inner(
    state: &mut TriggerDomainState,
    scheduler_cycles: Option<&core::cell::Cell<u32>>,
) {
    let tim5 = embassy_stm32::pac::TIM5;
    let start = cortex_m::peripheral::DWT::cycle_count();
    let earliest = state.scheduler.earliest_deadline();
    if let Some(accumulator) = scheduler_cycles {
        accumulator.set(
            accumulator
                .get()
                .wrapping_add(cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start)),
        );
    }
    if let Some(earliest) = earliest {
        let now = tim5.cnt().read();
        let clamped =
            foci_klipper::scheduler::timer_arm_clock(now, earliest, TRIGGER_TIMER_MIN_LEAD_TICKS);
        tim5.ccr(1).write_value(clamped);
        tim5.sr().modify(|w| w.set_ccif(1, false));
        tim5.dier().modify(|w| w.set_ccie(1, true));
        // Ensure NVIC is unmasked (idempotent)
        unsafe {
            cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5);
        }
        state.scheduler.set_cc2_active(true);
    } else {
        disarm_trigger_scheduler(state);
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
/// Uses `addr_of_mut!` to avoid the `static_mut_refs` lint.
#[inline]
pub(crate) fn with_trigger_domain<F, R>(f: F) -> R
where
    F: FnOnce(&mut TriggerDomain) -> R,
{
    with_trigger_domain_site(crate::state::stepper_diag::PERF_SITE_TRIGGER_DOMAIN, f)
}

#[inline]
pub(crate) fn with_trigger_domain_site<F, R>(site: u8, f: F) -> R
where
    F: FnOnce(&mut TriggerDomain) -> R,
{
    let start = cortex_m::peripheral::DWT::cycle_count();
    let result = cortex_m::interrupt::free(|_| {
        // SAFETY: All access to TRIGGER_DOMAIN goes through this function.
        // interrupt::free ensures exclusive access across all priorities.
        // addr_of_mut! avoids creating a reference to the static mut.
        unsafe { f(&mut *core::ptr::addr_of_mut!(TRIGGER_DOMAIN).cast::<TriggerDomain>()) }
    });
    let cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start);
    crate::state::stepper_diag::record_critical_section(site, cycles);
    result
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
/// Per spec: disable CC1, clear interval, snapshot stop stats, reset ISR
/// stepper engine, drain queue, reset direction GPIO.
pub(crate) fn stop_physical_stepper_no_lock(stepper_idx: usize, reason: StepperEventReason) {
    use core::sync::atomic::Ordering::Relaxed;
    if !PHYSICAL_STEPPER_INDICES.contains(&stepper_idx) {
        return;
    }
    let channel = stepper_idx as u8;
    let Some(tmc_state) = tmc_state(channel) else {
        return;
    };
    let Some(channel_stepper_state) = stepper_state(channel) else {
        return;
    };
    let tim5 = embassy_stm32::pac::TIM5;
    let other_active = PHYSICAL_STEPPER_INDICES
        .iter()
        .copied()
        .filter(|&idx| idx != stepper_idx)
        .any(|idx| {
            stepper_state(idx as u8)
                .map(|state| state.timer_active_mirror.load(Relaxed))
                .unwrap_or(false)
        });
    // 1. Disable CC1 only when this stop leaves no other channel active.
    if !other_active {
        tim5.dier().modify(|w| w.set_ccie(0, false));
        tim5.sr().modify(|w| w.set_ccif(0, false));
    }
    // 2. Clear channel-local TMC/stepper atomics.
    tmc_state.motor_armed.store(false, Relaxed);
    channel_stepper_state.interval.store(0, Relaxed);
    channel_stepper_state
        .timer_active_mirror
        .store(false, Relaxed);
    tmc_state.pid_error_sums_clear.store(true, Relaxed);
    let remaining_events = unsafe {
        isr_stepper(channel)
            .map(|stepper| stepper.active_event_count())
            .unwrap_or(0)
    };
    let queue_len = unsafe {
        step_queue(channel)
            .map(|queue| queue.len().min(u16::MAX as usize) as u16)
            .unwrap_or(0)
    };
    // 3. Reset ISR stepper engine state (clear stale partial segment)
    // SAFETY: ISR stepper state is exclusively touched by the P3 step-timer
    // IRQ. If another channel is still active, the stopped channel is first
    // gated by `motor_armed=false`, so the ISR will skip this channel.
    unsafe {
        if let Some(stepper) = isr_stepper(channel) {
            stepper.reset();
        }
    }
    // 4. Drain queue.
    //
    // SAFETY: The step queue is a `heapless::spsc::Queue` — single-producer
    // (P2 `queue_step_move`), single-consumer (P3 step-timer IRQ). The P3
    // consumer is gated from this channel by `motor_armed=false`; here we act
    // as the channel's consumer and call `dequeue()`. The SPSC implementation
    // supports concurrent producer/consumer access without a lock, so a P2
    // enqueue that races this drain is well-defined.
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
    let mut drained_segments = 0u32;
    let mut drained_steps = 0u32;
    while let Some(m) = unsafe { step_queue(channel).and_then(|queue| queue.dequeue()) } {
        drained_segments = drained_segments.saturating_add(1);
        drained_steps = drained_steps.saturating_add(u32::from(m.count));
    }
    crate::state::stepper_diag::record_physical_stop(
        channel,
        reason,
        remaining_events,
        queue_len,
        drained_segments,
        drained_steps,
    );
    // 5. Reset direction state to the default forward value.
    channel_stepper_state.direction.store(true, Relaxed);
    // 6. Reset STEP/DIR GPIOs to LOW.
    //
    // Full-mode scheduling relies on the ISR toggling from a known
    // starting state: the first event of each segment must drive STEP
    // LOW → HIGH (the rising edge the TMC4671 counts). If the previous
    // segment ended mid-pulse (shutdown or stop between step and unstep
    // events) the pin could still be HIGH, which would flip the parity
    // for the next segment and lose the first motor step.
    let _ = crate::stepper_pins::reset_idle(channel);
}
