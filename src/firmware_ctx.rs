//! Concrete context passed to every ankyra command handler in this
//! binary.
//!
//! Starts as a thin wrapper around `foci_klipper::state::State` and
//! grows into the board-side implementation of every capability trait
//! declared in `foci_firmware::context`. The trait-method bodies
//! forward verbatim to the board-local helpers (`helpers::*`,
//! `trigger_domain::with_trigger_domain`, `state::tmc::
//! TMC_REQUEST_SENDER`) that the handlers call today — no new logic.
//!
//! The full handler audit driving this surface lives at
//! `docs/notes/foci-firmware-lift-handler-audit.md`. Tasks 10–17 move
//! the handler bodies into `foci-firmware`; this file is what they
//! bind against.

use foci_firmware::context::{
    BoardChannels, ClockSource, EndstopHost, QueueFull, ShutdownRecovery, StateAccess, StepperHost,
    TmcHost, TrsyncHost,
};
use foci_firmware::state::trsync::ENDSTOP_PIN_IDS;
use foci_firmware::tmc::TmcRequest;
use foci_klipper::commands::CommandError;
use foci_klipper::state::State;

use crate::helpers::lifecycle::{KATAPULT_BYPASS, katapult_reset, read_tim5};
use crate::helpers::stepper::arm_step_timer;
use crate::helpers::trsync::{gpio_configure_input, gpio_read_pin};
use crate::state::stepper::{ISR_STEPPER, P2_STEPPER, STEP_QUEUE};
use crate::state::tmc::TMC_REQUEST_SENDER;
use crate::trigger_domain::{
    PHYSICAL_STEPPER_INDICES, stop_physical_stepper_no_lock, with_trigger_domain,
};

fn tmc_state() -> &'static foci_firmware::state::tmc::TmcChannelState {
    foci_firmware::state::tmc::tmc_channel0()
}

/// Full shutdown: Klipper-level transition, board-local trigger-domain
/// quiesce, and post-quiesce atomic resets. Returns `true` if this call
/// performed the transition (first call).
///
/// This wraps the shared `foci_firmware::helpers::lifecycle::enter_shutdown`
/// with the trigger-domain cleanup that only the board can perform. Call
/// it from RTIC task contexts that have a `&mut State` but not a full
/// `FirmwareCtx`. The `ShutdownRecovery::enter_shutdown` impl below
/// delegates to this function so handler-path and RTIC-path shutdowns
/// share the same cleanup.
pub(crate) fn enter_shutdown(state: &mut State) -> bool {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::state::lifecycle::REPORT_PENDING;
    use foci_firmware::state::trsync::{HOMING_STATE, HOMING_TRIGGERED};

    let entered = foci_firmware::helpers::lifecycle::enter_shutdown(state);
    if entered {
        // Quiesce TriggerDomain: cancel timers, stop steppers
        with_trigger_domain(|td| {
            td.scheduler.clear();
            let tim5 = embassy_stm32::pac::TIM5;
            tim5.dier().modify(|w| w.set_ccie(1, false));
            // Stop physical steppers
            for &idx in PHYSICAL_STEPPER_INDICES {
                stop_physical_stepper_no_lock(idx);
                td.set_timer_active(idx, false);
            }
            // Clear transient stepper state
            for ss in td.stepper_stop.iter_mut() {
                ss.need_reset = true;
                ss.blocked_by_trigger = false;
                ss.blocked_by_trsync = None;
            }
            // Clear transient trsync state (preserve OID maps).
            // Trigger each trsync to clear can_trigger per spec,
            // then cancel timers and pending reports.
            for (trsync_idx, ts) in td.trsync.iter_mut().enumerate() {
                // Find OID for this index (if configured)
                let oid = td
                    .trsync_by_oid
                    .iter()
                    .find(|&(_, &idx)| idx == trsync_idx)
                    .map(|(&oid, _)| oid)
                    .unwrap_or(0);
                ts.trigger(oid, 0, 0); // Sets can_trigger=false
                ts.cancel_timers();
                ts.clear_pending_report();
            }
            // Clear stop signals and endstop homing
            for sig in td.stop_signals.iter_mut() {
                *sig = None;
            }
            for es in td.endstops.iter_mut() {
                es.reset();
            }
        });
        // Post-quiesce atomics — order preserved from pre-lift body
        REPORT_PENDING.store(false, Relaxed);
        HOMING_TRIGGERED.store(0xFF, Relaxed);
        HOMING_STATE.store(0, Relaxed);
    }
    entered
}

pub struct FirmwareCtx<'a> {
    pub state: &'a mut State,
}

impl<'a> FirmwareCtx<'a> {
    pub fn new(state: &'a mut State) -> Self {
        Self { state }
    }
}

impl<'a> ankyra::ShutdownState for FirmwareCtx<'a> {
    fn is_shutdown(&self) -> bool {
        self.state.is_shutdown()
    }
}

impl<'a> StateAccess for FirmwareCtx<'a> {
    fn state(&mut self) -> &mut State {
        self.state
    }
}

impl<'a> ClockSource for FirmwareCtx<'a> {
    fn now(&self) -> u32 {
        read_tim5()
    }
}

impl<'a> ShutdownRecovery for FirmwareCtx<'a> {
    fn enter_shutdown(&mut self) -> bool {
        enter_shutdown(self.state)
    }

    fn reset_to_bootloader(&mut self) -> ! {
        katapult_reset(KATAPULT_BYPASS);
        // `katapult_reset` always triggers `SCB::sys_reset`.
        // SAFETY: reachable only if the reset faulted; treat as unrecoverable.
        #[allow(clippy::empty_loop)]
        loop {}
    }

    fn send_shutdown(&mut self, reason: foci_firmware::shutdown::ShutdownReason, clock: u32) {
        foci_firmware::dispatch_shutdown!(reason, clock, &mut crate::_ankyra_config::Sender);
    }
}

impl<'a> BoardChannels for FirmwareCtx<'a> {
    fn tmc_channel_count(&self) -> u8 {
        crate::board_config::TMC_CHANNEL_COUNT
    }

    fn step_pin_channel(&self, pin: u8) -> Option<u8> {
        crate::board_config::step_pin_channel(pin)
    }

    fn dir_pin_channel(&self, pin: u8) -> Option<u8> {
        crate::board_config::dir_pin_channel(pin)
    }

    fn enable_pin_channel(&self, pin: u32) -> Option<u8> {
        crate::board_config::enable_pin_channel(pin)
    }
}

impl<'a> StepperHost for FirmwareCtx<'a> {
    fn set_next_step_dir(&mut self, forward: bool) {
        // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
        unsafe {
            (*core::ptr::addr_of_mut!(P2_STEPPER)).set_next_step_dir(forward);
        }
    }

    fn queue_step_move(
        &mut self,
        move_: foci_klipper::stepper::StepMove,
    ) -> Result<(), CommandError> {
        use foci_klipper::commands::stepper::{QueueStepResult, queue_step_hw};

        with_trigger_domain(|td| {
            let need_reset = td.stepper_stop[0].need_reset;

            // SAFETY: P2 is the sole producer. Queue len read is safe from P2.
            let queue_was_empty = unsafe { (*core::ptr::addr_of_mut!(STEP_QUEUE)).is_empty() };

            // `queue_step_hw` takes the move's step count — the
            // caller-provided `count` field on `StepMove`.
            match queue_step_hw(need_reset, move_.count, queue_was_empty) {
                Ok(QueueStepResult::Discarded) => Ok(()),
                Err(e) => Err(e),
                Ok(_result) => {
                    // SAFETY: P2_STEPPER is only accessed from P2 command handlers.
                    let prepared =
                        unsafe { (*core::ptr::addr_of_mut!(P2_STEPPER)).prepare_move(move_) };

                    // SAFETY: P2 is the sole producer.
                    let ok = unsafe {
                        (*core::ptr::addr_of_mut!(STEP_QUEUE))
                            .enqueue(prepared)
                            .is_ok()
                    };
                    if !ok {
                        return Err(CommandError::QueueFull);
                    }

                    if !td.timer_active(0)
                        && tmc_state()
                            .motor_armed
                            .load(core::sync::atomic::Ordering::Relaxed)
                    {
                        let now = read_tim5();
                        arm_step_timer(now.wrapping_add(1));
                        td.set_timer_active(0, true);
                    }
                    Ok(())
                }
            }
        })
    }

    fn reset_step_clock(&mut self, clock: u32) -> Result<(), CommandError> {
        with_trigger_domain(|td| {
            foci_klipper::commands::stepper::reset_step_clock_hw(td.stepper_stop[0].timer_active)?;

            // Drain any stale items from a previous stop.
            // SAFETY: Timer is not active (checked above), P3 is quiesced.
            unsafe { while (*core::ptr::addr_of_mut!(STEP_QUEUE)).dequeue().is_some() {} }

            // SAFETY: reset_step_clock_hw accepted only because the timer is
            // inactive, so P3 is not using ISR_STEPPER. P2_STEPPER is only
            // accessed from P2 command handlers.
            unsafe {
                foci_klipper::commands::stepper::reset_stepper_clocks(
                    &mut *core::ptr::addr_of_mut!(P2_STEPPER),
                    &mut *core::ptr::addr_of_mut!(ISR_STEPPER),
                    clock,
                );
            }

            if !td.stepper_stop[0].blocked_by_trigger {
                td.stepper_stop[0].need_reset = false;
            }
            Ok(())
        })
    }
}

impl<'a> EndstopHost for FirmwareCtx<'a> {
    fn configure_input(&mut self, pin: u8, pull_up: bool) {
        gpio_configure_input(pin, pull_up);
    }

    fn read_pin(&self, pin: u8) -> bool {
        gpio_read_pin(pin)
    }
}

impl<'a> TrsyncHost for FirmwareCtx<'a> {
    fn reset_all(&mut self) {
        with_trigger_domain(|td| {
            td.scheduler.clear();
            let tim5 = embassy_stm32::pac::TIM5;
            tim5.dier().modify(|w| w.set_ccie(1, false));
            for ts in td.trsync.iter_mut() {
                ts.reset();
            }
            for es in td.endstops.iter_mut() {
                es.reset();
            }
            for sig in td.stop_signals.iter_mut() {
                *sig = None;
            }
            td.trsync_by_oid.clear();
            td.endstop_by_oid.clear();
            for &idx in PHYSICAL_STEPPER_INDICES {
                stop_physical_stepper_no_lock(idx);
                td.set_timer_active(idx, false);
            }
            for ss in td.stepper_stop.iter_mut() {
                *ss = foci_klipper::trigger::StepperStopRuntime::new();
            }
        });
    }

    fn register_trsync(&mut self, oid: u8) -> Result<(), ()> {
        with_trigger_domain(|td| {
            if td.trsync_by_oid.contains_key(&oid) {
                return Err(());
            }
            let idx = td.trsync_by_oid.len();
            if idx >= foci_klipper::state::NUM_TRSYNC {
                return Err(());
            }
            td.trsync_by_oid.insert(oid, idx).map_err(|_| ())?;
            Ok(())
        })
    }

    fn trsync_arm(
        &mut self,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) -> Result<(), ()> {
        with_trigger_domain(|td| {
            let idx = match td.trsync_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            for slot in td.stop_signals.iter_mut() {
                if let Some(signal) = slot
                    && signal.trsync_oid == oid
                {
                    *slot = None;
                }
            }
            for stepper in td.stepper_stop.iter_mut() {
                if stepper.blocked_by_trsync == Some(oid) {
                    stepper.blocked_by_trigger = false;
                    stepper.blocked_by_trsync = None;
                }
            }
            td.scheduler.cancel(|k| {
                matches!(k,
                    foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(i)
                    | foci_klipper::scheduler::SchedulerEventKind::TrSyncReport(i)
                    if *i == idx
                )
            });
            td.trsync[idx].arm(report_clock, report_ticks, expire_reason);
            if report_ticks > 0 {
                td.scheduler
                    .schedule(foci_klipper::scheduler::SchedulerEvent {
                        wake_clock: report_clock,
                        kind: foci_klipper::scheduler::SchedulerEventKind::TrSyncReport(idx),
                    });
            }
            td.reprogram_cc2();
            Ok(())
        })
    }

    fn trsync_set_timeout(&mut self, oid: u8, clock: u32) -> Result<(), ()> {
        with_trigger_domain(|td| {
            let idx = match td.trsync_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            if td.trsync[idx].can_trigger() {
                td.trsync[idx].set_timeout(clock);
                td.scheduler.cancel(|k| {
                    matches!(k,
                        foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(i) if *i == idx
                    )
                });
                td.scheduler
                    .schedule(foci_klipper::scheduler::SchedulerEvent {
                        wake_clock: clock,
                        kind: foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(idx),
                    });
                td.reprogram_cc2();
            }
            Ok(())
        })
    }

    fn trsync_trigger(&mut self, oid: u8, reason: u8) -> Result<u8, ()> {
        with_trigger_domain(|td| {
            let idx = match td.trsync_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            let _result = td.trigger(oid, reason, 0);
            let trigger_reason = td.trsync[idx].trigger_reason();

            td.scheduler.cancel(|k| {
                matches!(k,
                    foci_klipper::scheduler::SchedulerEventKind::TrSyncExpire(i)
                    | foci_klipper::scheduler::SchedulerEventKind::TrSyncReport(i)
                    if *i == idx
                )
            });
            td.trsync[idx].clear_pending_report();
            td.trsync[idx].cancel_timers();

            for slot in td.stop_signals.iter_mut() {
                if let Some(signal) = slot
                    && signal.trsync_oid == oid
                {
                    *slot = None;
                }
            }

            for stepper in td.stepper_stop.iter_mut() {
                if stepper.blocked_by_trsync == Some(oid) {
                    stepper.blocked_by_trigger = false;
                    stepper.blocked_by_trsync = None;
                }
            }

            td.reprogram_cc2();
            Ok(trigger_reason)
        })
    }

    fn register_endstop(&mut self, oid: u8) -> Result<usize, ()> {
        with_trigger_domain(|td| {
            if td.endstop_by_oid.contains_key(&oid) {
                return Err(());
            }
            let idx = td.endstop_by_oid.len();
            if idx >= foci_klipper::state::NUM_ENDSTOPS {
                return Err(());
            }
            td.endstop_by_oid.insert(oid, idx).map_err(|_| ())?;
            Ok(idx)
        })
    }

    fn endstop_start_homing(
        &mut self,
        oid: u8,
        clock: u32,
        sample_ticks: u32,
        sample_count: u8,
        rest_ticks: u32,
        pin_value_expected: bool,
        trsync_oid: u8,
        trigger_reason: u8,
    ) -> Result<(), ()> {
        with_trigger_domain(|td| {
            let idx = match td.endstop_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            // Zero sample_count is a disarm request — no trsync needed.
            if sample_count != 0 && !td.trsync_by_oid.contains_key(&trsync_oid) {
                return Err(());
            }
            // Retrieve pin_id from config-time storage inside the
            // critical section so the endstop runtime gets the correct
            // physical pin without an extra atomic read by the handler.
            let pin_id = ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
            td.scheduler.cancel(|k| {
                matches!(k,
                    foci_klipper::scheduler::SchedulerEventKind::EndstopSample(i) if *i == idx
                )
            });
            td.endstops[idx].start_homing(
                clock,
                sample_ticks,
                sample_count,
                rest_ticks,
                pin_value_expected,
                trsync_oid,
                trigger_reason,
                pin_id,
            );
            if sample_count > 0 {
                td.scheduler
                    .schedule(foci_klipper::scheduler::SchedulerEvent {
                        wake_clock: clock,
                        kind: foci_klipper::scheduler::SchedulerEventKind::EndstopSample(idx),
                    });
            }
            td.reprogram_cc2();
            Ok(())
        })
    }

    fn endstop_query(&mut self, oid: u8) -> Result<(bool, u32, u8), ()> {
        with_trigger_domain(|td| {
            let idx = match td.endstop_by_oid.get(&oid) {
                Some(&idx) => idx,
                None => return Err(()),
            };
            let (homing, next_clock) = td.endstops[idx].query_state();
            let pin_id = ENDSTOP_PIN_IDS[idx].load(core::sync::atomic::Ordering::Relaxed);
            Ok((homing, next_clock, pin_id))
        })
    }

    fn link_stepper_stop(&mut self, stepper_idx: usize, trsync_oid: u8) -> Result<(), ()> {
        with_trigger_domain(|td| {
            if !td.trsync_by_oid.contains_key(&trsync_oid) {
                return Err(());
            }
            for signal in td.stop_signals.iter().flatten() {
                if signal.stepper_idx == stepper_idx {
                    return Err(());
                }
            }
            for slot in td.stop_signals.iter_mut() {
                if slot.is_none() {
                    *slot = Some(foci_klipper::trigger::StopSignal {
                        trsync_oid,
                        stepper_idx,
                    });
                    return Ok(());
                }
            }
            Err(())
        })
    }
}

impl<'a> TmcHost for FirmwareCtx<'a> {
    fn enqueue_request(&self, req: TmcRequest) -> Result<(), QueueFull> {
        let sent = critical_section::with(|cs| {
            if let Some(sender) = TMC_REQUEST_SENDER.borrow(cs).borrow_mut().as_mut() {
                sender.try_send(req).is_ok()
            } else {
                false
            }
        });
        if sent { Ok(()) } else { Err(QueueFull) }
    }
}
