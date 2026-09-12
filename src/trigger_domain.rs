//! Trigger domain: the critical-section-guarded coordination point
//! between the step-queue producer (P2 context), the step-timer ISR
//! (P3 context), and the trsync/endstop subsystem.
//!
//! Promoted from `mod app` in Task 8b2 — previously declared inline
//! alongside the RTIC state it coordinates. Moving it here alongside
//! the subsystem `state/*` modules makes the coordination surface
//! inspectable independently of RTIC's macro expansion.

use core::ops::{Deref, DerefMut};

use foci_firmware::context::trsync::{self, TriggerDispatchBoard, TriggerDomainState};
use foci_firmware::diagnostic::StepperEventReason;
use foci_klipper::step_queue::Consumer;
use foci_klipper::stepper::StepMove;

use crate::helpers::stepper::clear_tim5_compare_flag;
use crate::state::stepper::isr_stepper;

/// Detach a channel's `StepQueueChannel` consumer under a short,
/// independent critical section, for a caller about to drain it outside
/// any masked region. Returns `None` if already detached.
fn detach_consumer(channel: u8) -> Option<Consumer<'static, StepMove>> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: this critical section provides the exclusive access
        // `take_consumer` requires. It is independent of
        // `with_trigger_domain` (never touches `TriggerDomain`), so it is
        // safe to nest inside an outer `interrupt::free` region if the
        // caller happens to be in one. `channel` is a valid
        // `STEP_QUEUE_CHANNELS` index because every caller (physical
        // stepper indices, or an explicit `STEPPER_CHANNEL_COUNT` bounds
        // check) validates it before reaching here.
        unsafe { crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].take_consumer() }
    })
}

/// Reattach a consumer previously detached from a channel's
/// `StepQueueChannel` by `detach_consumer`. Private: the only correct
/// caller is the function that just called `detach_consumer` and is
/// handing its own local variable straight back.
fn attach_consumer(channel: u8, consumer: Consumer<'static, StepMove>) {
    cortex_m::interrupt::free(|_| {
        // SAFETY: exclusive access from the critical section; `channel` is
        // a valid `STEP_QUEUE_CHANNELS` index (see detach_consumer), and
        // `consumer` is that same channel's own value per this function's
        // contract -- the only correct caller just detached it from this
        // exact channel and is handing it straight back. `Consumer` carries
        // no channel identity of its own, so giving a different channel's
        // consumer here would silently corrupt that channel's queue.
        unsafe {
            crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].give_consumer(consumer)
        }
    })
}

/// Borrow a channel's `StepQueueChannel` consumer in place, under a short
/// independent critical section, for a caller already inside an outer
/// mask that stays in place here (reset, shutdown, trigger-dispatch
/// stop). Returns `None` if a concurrent unmasked drain
/// (`stop_physical_stepper_unmasked`) currently holds the consumer
/// detached -- callers must perform their non-queue side effects and
/// report zero *additional* drained work in that case: the in-flight
/// unmasked drain already owns draining this queue.
pub(crate) fn with_consumer_locked<R>(
    channel: u8,
    f: impl FnOnce(&mut Consumer<'static, StepMove>) -> R,
) -> Option<R> {
    cortex_m::interrupt::free(|_| {
        // SAFETY: see detach_consumer -- same critical section and
        // channel-validity argument, applied to `borrow_consumer_mut`.
        unsafe {
            crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].borrow_consumer_mut(f)
        }
    })
}

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
        stop_physical_stepper_masked(stepper_idx, reason);
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
        clear_tim5_compare_flag(1);
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

/// Distinguishes how `stop_physical_stepper_inner` may access the step
/// queue's consumer, based on what the caller already holds.
enum QueueAccess {
    /// P1 direct-stop path: detach the consumer, drain with interrupts
    /// enabled, reattach.
    Unmasked,
    /// Trigger-dispatch/shutdown: caller already holds an outer mask this
    /// call is not unmasking; drain in place under it.
    LockedInPlace,
}

/// Stop a physical stepper. Only acts on physical stepper indices (CC1
/// timer, GPIO, queue).
///
/// Called from two contexts:
/// 1. Inside `with_trigger_domain()` during trigger dispatch (P2 holding
///    an `interrupt::free` critical section; P2 producers cannot preempt).
/// 2. From [`crate::helpers::stepper::stepper_stop_from_p1`] (P1 context
///    with no critical section; P2 producers can preempt).
///
/// Per spec: disable CC1, clear interval, snapshot stop stats, reset ISR
/// stepper engine, drain queue, reset direction GPIO.
fn stop_physical_stepper_inner(
    stepper_idx: usize,
    reason: StepperEventReason,
    access: QueueAccess,
) {
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
        clear_tim5_compare_flag(0);
    }
    // 2. Clear channel-local stepper atomics. Trigger-domain stops preserve
    // the electrical armed state; explicit P1 stop/disable paths clear
    // `motor_armed` before calling this helper.
    channel_stepper_state.interval.store(0, Relaxed);
    channel_stepper_state
        .timer_active_mirror
        .store(false, Relaxed);
    tmc_state.pid_error_sums_clear.store(true, Relaxed);
    let remaining_events = unsafe {
        isr_stepper(channel)
            .map(|stepper| u32::from(stepper.active_step_count()))
            .unwrap_or(0)
    };
    let queue_len = crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize]
        .len()
        .min(u16::MAX as usize) as u16;
    // 3. Reset ISR stepper engine state (clear stale partial segment)
    // SAFETY: ISR stepper state is exclusively touched by the P3 step-timer
    // IRQ. This helper clears the stopped channel's `timer_active_mirror`, so
    // the ISR's channel-ready gate skips it even when another channel keeps
    // the shared CC1 timer active.
    unsafe {
        if let Some(stepper) = isr_stepper(channel) {
            stepper.reset();
        }
    }
    // 4. Drain queue -- access mode depends on caller context.
    let mut drained_segments = 0u32;
    let mut drained_steps = 0u32;
    match access {
        QueueAccess::Unmasked => {
            let mut consumer = detach_consumer(channel);
            if let Some(consumer) = consumer.as_mut() {
                while let Some(m) = consumer.dequeue() {
                    drained_segments = drained_segments.saturating_add(1);
                    drained_steps = drained_steps.saturating_add(u32::from(m.count));
                }
            }
            if let Some(consumer) = consumer {
                attach_consumer(channel, consumer);
            }
            // else: a concurrent locked drain already owns the queue --
            // this path is unreachable for QueueAccess::Unmasked in
            // practice (P1 is the only Unmasked caller and detach_consumer
            // only returns None if *this* channel's consumer was already
            // detached by another Unmasked call, which cannot happen
            // concurrently since P1 stops are not reentrant), but the
            // detach/no-op fallback keeps this branch total.
        }
        QueueAccess::LockedInPlace => {
            let result = with_consumer_locked(channel, |consumer| {
                let mut segments = 0u32;
                let mut steps = 0u32;
                while let Some(m) = consumer.dequeue() {
                    segments = segments.saturating_add(1);
                    steps = steps.saturating_add(u32::from(m.count));
                }
                (segments, steps)
            });
            if let Some((segments, steps)) = result {
                drained_segments = segments;
                drained_steps = steps;
            }
            // else: None means a P1 unmasked drain currently holds the
            // consumer detached. Per the spec's recovery rule: report zero
            // additional drained work here and continue with the rest of
            // this sequence (steps 5-6 below) unconditionally -- P1's own
            // in-flight drain already owns draining the queue.
        }
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

/// Stop a physical stepper from the P1 direct-stop path
/// (`stepper_stop_from_p1`), with no critical section held. The queue
/// drain runs entirely with interrupts enabled.
///
/// This is the only correct caller. Calling this from inside an active
/// `with_trigger_domain` closure is a bug (not compiler-prevented --
/// enforced by this being this function's single call site, verified by
/// workspace reference search as part of code review).
pub(crate) fn stop_physical_stepper_unmasked(stepper_idx: usize, reason: StepperEventReason) {
    stop_physical_stepper_inner(stepper_idx, reason, QueueAccess::Unmasked);
}

/// Stop a physical stepper from inside an active `with_trigger_domain`
/// closure (trigger-dispatch, shutdown). The queue drain stays inline
/// under the mask the caller already holds -- moving it outside that
/// mask requires restructuring the shared-crate call contract this
/// closure runs inside, which is a separate, larger change.
///
/// This is the only correct caller. Calling this without an active mask
/// already held is a bug (not compiler-prevented -- enforced by single
/// call sites, verified by workspace reference search as part of code
/// review).
pub(crate) fn stop_physical_stepper_masked(stepper_idx: usize, reason: StepperEventReason) {
    stop_physical_stepper_inner(stepper_idx, reason, QueueAccess::LockedInPlace);
}
