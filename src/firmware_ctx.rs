//! Concrete context passed to every ankyra command handler in this
//! binary.
//!
//! Starts as a thin wrapper around `foci_klipper::state::State` and
//! grows into the board-side implementation of every capability trait
//! declared in `foci_firmware::context`. The implementations bind those
//! capabilities to board-local helpers and state.
//!
//! The full handler audit driving this surface lives at
//! `docs/notes/foci-firmware-lift-handler-audit.md`. Tasks 10–17 move
//! the handler bodies into `foci-firmware`; this file is what they
//! bind against.

use foci_firmware::context::stepper::{
    self, StepperDrainStats, StepperEnqueueError, StepperQueueBoard, StepperResetBoard,
    StepperTimerStateBoard,
};
use foci_firmware::context::tmc as tmc_context;
use foci_firmware::context::trsync::{self, TriggerDispatchBoard, TriggerDomainState, TrsyncBoard};
use foci_firmware::context::{
    AnalogInHost, BoardChannels, ClockSource, EndstopHost, QueueFull, ShutdownRecovery,
    StateAccess, StepperHost, TmcHost, TrsyncHost,
};
use foci_firmware::diagnostic::StepperEventReason;
use foci_firmware::state::trsync::ENDSTOP_PIN_IDS;
use foci_firmware::tmc::TmcRequest;
use foci_klipper::commands::CommandError;
use foci_klipper::scheduler::ScheduleResult;
use foci_klipper::state::State;

use crate::helpers::lifecycle::{KATAPULT_BYPASS, katapult_reset, read_tim5};
use crate::helpers::stepper::arm_step_timer_if_earlier;
use crate::helpers::stepper_diag::record_stepper_event;
use crate::helpers::tmc::OuroborosTmcQueue;
use crate::helpers::trsync::{gpio_configure_input, gpio_read_pin};
use crate::state::stepper::{isr_stepper, p2_stepper};
use crate::state::tmc::{HOMING_START_PENDING, TMC_CONTROL_SENDER, TmcControlEvent};
use crate::trigger_domain::{
    OuroborosDispatchBoard, PHYSICAL_STEPPER_INDICES, disarm_trigger_scheduler, reprogram_cc2,
    set_timer_active, with_trigger_domain, with_trigger_domain_site,
};

fn emit_invalid_stepper_channel_and_shutdown(state: &mut State, channel: u8) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociDiagnostic,
        fault: u8 = foci_firmware::diagnostic::FaultClass::InvalidChannel as u8,
        operation: u8 = foci_firmware::diagnostic::operation::SET_NEXT_STEP_DIR,
        oid: u8 = 255,
        channel: u8 = channel,
        data0: u32 = 0,
        data1: u32 = 0,
    );
    if crate::firmware_ctx::enter_shutdown(state) {
        ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "Invalid stepper channel",
            read_tim5(),
        );
    }
}

fn emit_queue_full_diagnostic(channel: u8, queue_len: usize) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociDiagnostic,
        fault: u8 = foci_firmware::diagnostic::FaultClass::QueueFull as u8,
        operation: u8 = foci_firmware::diagnostic::operation::QUEUE_STEP,
        oid: u8 = 255,
        channel: u8 = channel,
        data0: u32 = queue_len as u32,
        data1: u32 = crate::state::stepper::STEP_QUEUE_DEPTH as u32,
    );
}

pub(crate) struct OuroborosStepperBoard {
    pub(crate) tmc_state: Option<&'static foci_firmware::state::tmc::TmcChannelState>,
}

impl StepperTimerStateBoard for OuroborosStepperBoard {
    fn set_timer_active(
        &mut self,
        state: &mut TriggerDomainState,
        stepper_idx: usize,
        active: bool,
    ) {
        set_timer_active(state, stepper_idx, active);
    }
}

impl StepperQueueBoard for OuroborosStepperBoard {
    fn queue_stepper_index(&self, channel: u8) -> Result<usize, CommandError> {
        if self.tmc_state.is_some() {
            Ok(channel as usize)
        } else {
            Err(CommandError::InvalidChannel(channel))
        }
    }

    fn queue_was_empty(&self, channel: u8) -> Result<bool, CommandError> {
        if channel as usize >= crate::state::stepper::STEPPER_CHANNEL_COUNT {
            return Err(CommandError::InvalidChannel(channel));
        }
        Ok(crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].is_empty())
    }

    fn motion_scale_configured(&self, channel: u8) -> Result<bool, CommandError> {
        self.queue_stepper_index(channel)?;
        Ok(self
            .tmc_state
            .expect("validated Ouroboros channel has TMC state")
            .motion_scale_configured())
    }

    fn prepare_and_enqueue(
        &mut self,
        channel: u8,
        move_: foci_klipper::stepper::StepMove,
    ) -> Result<usize, usize> {
        // SAFETY: P2 owns both the direction state and the queue producer.
        let Some((stepper, producer)) = (unsafe { p2_stepper(channel) })
            .zip(unsafe { crate::state::stepper::step_queue_producer(channel) })
        else {
            return Err(0);
        };
        // heapless::spsc::Producer has no `is_full()` -- only the un-split
        // `Queue` does. `!producer.ready()` is the producer-side equivalent
        // (`ready()` is true iff there is room to enqueue).
        if !producer.ready() {
            return Err(producer.len());
        }
        let prepared = stepper.prepare_move(move_);
        // P2 is the only producer, so a consumer can only create more space
        // after the full check above -- enqueue therefore cannot fail here.
        producer.enqueue(prepared).map_err(|_| producer.len())?;
        Ok(producer.len())
    }

    fn motor_armed(&self, _channel: u8) -> bool {
        self.tmc_state
            .map(|state| {
                state
                    .motor_armed
                    .load(core::sync::atomic::Ordering::Relaxed)
            })
            .unwrap_or(false)
    }

    fn record_queue_full(&mut self, channel: u8, queue_len: usize) {
        emit_queue_full_diagnostic(channel, queue_len);
    }

    fn record_discarded_segment(&mut self, channel: u8, steps: u32) {
        crate::state::stepper_diag::record_discarded_segment(channel, steps);
    }

    fn record_queued_segment(&mut self, channel: u8, steps: u32, queue_len: usize) {
        #[cfg(feature = "trace")]
        crate::usb_trace::record_step_queue_depth(queue_len);
        #[cfg(not(feature = "trace"))]
        let _ = queue_len;
        crate::state::stepper_diag::record_queued_segment(channel, steps);
    }

    fn arm_queue_step_timer(
        &mut self,
        state: &mut TriggerDomainState,
        stepper_idx: usize,
        channel: u8,
    ) {
        let now = read_tim5();
        crate::state::stepper_diag::record_timer_activation(channel, now);
        arm_step_timer_if_earlier(now.wrapping_add(1));
        self.set_timer_active(state, stepper_idx, true);
    }
}

impl StepperResetBoard for OuroborosStepperBoard {
    fn reset_stepper_index(&self, channel: u8) -> Result<usize, CommandError> {
        if self.tmc_state.is_some() {
            Ok(channel as usize)
        } else {
            Err(CommandError::InvalidChannel(channel))
        }
    }

    fn drain_step_queue(&mut self, channel: u8) -> Result<StepperDrainStats, CommandError> {
        if channel as usize >= crate::state::stepper::STEPPER_CHANNEL_COUNT {
            return Err(CommandError::InvalidChannel(channel));
        }
        let mut stats = StepperDrainStats::ZERO;
        let drained = crate::trigger_domain::with_consumer_locked(channel, |consumer| {
            while let Some(m) = consumer.dequeue() {
                stats.segments = stats.segments.saturating_add(1);
                stats.steps = stats.steps.saturating_add(u32::from(m.count));
            }
        });
        if drained.is_none() {
            // The consumer is detached elsewhere: either a concurrent P1
            // unmasked drain, or a pending trigger-stop hand-off for this
            // channel awaiting trigger_drain_task (both P2, so this can
            // race a not-yet-serviced hand-off). Whichever it is already
            // owns draining this queue -- report zero additional drained
            // work for this reset -- see the spec's recovery rule.
        }
        Ok(stats)
    }

    fn reset_stepper_clocks(&mut self, channel: u8, clock: u32) -> Result<(), CommandError> {
        // SAFETY: Shared policy accepted only because the timer is inactive, so
        // P3 is not using the ISR stepper. P2 stepper is only accessed from P2.
        unsafe {
            foci_klipper::commands::stepper::reset_stepper_clocks(
                p2_stepper(channel).ok_or(CommandError::InvalidChannel(channel))?,
                isr_stepper(channel).ok_or(CommandError::InvalidChannel(channel))?,
                clock,
            );
        }
        Ok(())
    }

    fn record_reset_step_clock(&mut self, channel: u8, clock: u32, stats: StepperDrainStats) {
        crate::state::stepper_diag::record_reset(channel, stats.segments, stats.steps);
        record_stepper_event(
            StepperEventReason::ResetStepClock,
            channel,
            read_tim5(),
            0,
            clock,
            0,
        );
    }
}

struct OuroborosTrsyncBoard<'a> {
    state: &'a State,
}

impl<'a> TriggerDispatchBoard for OuroborosTrsyncBoard<'a> {
    fn stop_physical_stepper(
        &mut self,
        state: &TriggerDomainState,
        stepper_idx: usize,
        reason: StepperEventReason,
    ) {
        let mut dispatch = OuroborosDispatchBoard;
        dispatch.stop_physical_stepper(state, stepper_idx, reason);
    }

    fn record_trsync_stop(&mut self, stepper_idx: usize, clock: u32, trsync_oid: u8, reason: u8) {
        let mut dispatch = OuroborosDispatchBoard;
        dispatch.record_trsync_stop(stepper_idx, clock, trsync_oid, reason);
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

impl<'a> TrsyncBoard for OuroborosTrsyncBoard<'a> {
    fn physical_steppers(&self) -> &'static [usize] {
        PHYSICAL_STEPPER_INDICES
    }

    fn map_physical_stepper(&self, klipper_stepper_idx: usize) -> Option<usize> {
        let channel = self
            .state
            .stepper_channels
            .get(klipper_stepper_idx)
            .and_then(|channel| *channel)?;
        let channel_idx = channel as usize;
        PHYSICAL_STEPPER_INDICES
            .contains(&channel_idx)
            .then_some(channel_idx)
    }

    fn endstop_pin_id(&self, endstop_idx: usize) -> u8 {
        ENDSTOP_PIN_IDS[endstop_idx].load(core::sync::atomic::Ordering::Relaxed)
    }

    fn record_schedule_result(&mut self, result: ScheduleResult) {
        if matches!(result, ScheduleResult::Full) {
            crate::state::stepper_diag::record_scheduler_full();
        }
    }

    fn reprogram_trigger_timer(&mut self, state: &mut TriggerDomainState) {
        reprogram_cc2(state);
    }

    fn disarm_trigger_timer(&mut self, state: &mut TriggerDomainState) {
        disarm_trigger_scheduler(state);
    }
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
    use core::sync::atomic::Ordering::{Relaxed, Release};
    use foci_firmware::state::lifecycle::REPORT_PENDING;
    use foci_firmware::state::trsync::{HOMING_REENTER_PENDING, HOMING_STATE, HOMING_TRIGGERED};

    let entered = foci_firmware::helpers::lifecycle::enter_shutdown(state);
    if entered {
        with_trigger_domain_site(crate::state::stepper_diag::PERF_SITE_SHUTDOWN, |td| {
            let mut board = OuroborosTrsyncBoard { state: &*state };
            trsync::shutdown_quiesce(td.state_mut(), &mut board);
        });
        // Post-quiesce atomics — order preserved from pre-lift body
        REPORT_PENDING.store(false, Relaxed);
        HOMING_TRIGGERED.store(0xFF, Relaxed);
        HOMING_STATE.store(0, Relaxed);
        HOMING_REENTER_PENDING.store(false, Relaxed);
        HOMING_START_PENDING.store(false, Release);
    }
    entered
}

pub struct FirmwareCtx<'a> {
    pub state: &'a mut State,
}

impl<'a> FirmwareCtx<'a> {
    pub fn new(state: &'a mut State) -> Self {
        #[cfg(feature = "trace")]
        crate::usb_trace::store_last_klipper_command_service_tick(
            crate::usb_trace::trace_now_ticks(),
        );
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
    fn set_next_step_dir(&mut self, channel: u8, forward: bool) {
        let Some(_tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
            emit_invalid_stepper_channel_and_shutdown(self.state, channel);
            return;
        };

        // SAFETY: P2 stepper state is only accessed from P2 command handlers.
        unsafe {
            let Some(stepper) = p2_stepper(channel) else {
                emit_invalid_stepper_channel_and_shutdown(self.state, channel);
                return;
            };
            stepper.set_next_step_dir(forward);
        }
    }

    fn queue_step_move(
        &mut self,
        channel: u8,
        move_: foci_klipper::stepper::StepMove,
    ) -> Result<(), StepperEnqueueError> {
        let tmc_state = foci_firmware::state::tmc::tmc_channel(channel).ok_or(
            StepperEnqueueError::Command(CommandError::InvalidChannel(channel)),
        )?;

        with_trigger_domain_site(crate::state::stepper_diag::PERF_SITE_QUEUE_STEP, |td| {
            let mut board = OuroborosStepperBoard {
                tmc_state: Some(tmc_state),
            };
            stepper::queue_step_move(td.state_mut(), &mut board, channel, move_)
        })
    }

    fn reset_step_clock(&mut self, channel: u8, clock: u32) -> Result<(), CommandError> {
        let tmc_state = foci_firmware::state::tmc::tmc_channel(channel)
            .ok_or(CommandError::InvalidChannel(channel))?;

        with_trigger_domain_site(crate::state::stepper_diag::PERF_SITE_RESET, |td| {
            let mut board = OuroborosStepperBoard {
                tmc_state: Some(tmc_state),
            };
            stepper::reset_step_clock(td.state_mut(), &mut board, channel, clock)
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

    fn stall_channel_for_pin(&self, pin: u8) -> Option<u8> {
        use crate::ankyra::enumerations::Pins;
        match Pins::try_from(pin) {
            Ok(Pins::STALL0) => Some(0),
            Ok(Pins::STALL1) => Some(1),
            _ => None,
        }
    }
}

impl<'a> AnalogInHost for FirmwareCtx<'a> {
    fn resolve_analog_pin(&self, pin: u8) -> Option<u8> {
        crate::board_config::analog_pin_channel(pin)
    }
}

impl<'a> TrsyncHost for FirmwareCtx<'a> {
    fn reset_all(&mut self) {
        with_trigger_domain(|td| {
            let mut board = OuroborosTrsyncBoard {
                state: &*self.state,
            };
            trsync::reset_all(td.state_mut(), &mut board);
        });
    }

    fn register_trsync(&mut self, oid: u8) -> Result<(), ()> {
        with_trigger_domain(|td| trsync::register_trsync(td.state_mut(), oid).map_err(|_| ()))
    }

    fn trsync_arm(
        &mut self,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) -> Result<(), ()> {
        with_trigger_domain(|td| {
            let mut board = OuroborosTrsyncBoard {
                state: &*self.state,
            };
            trsync::trsync_arm(
                td.state_mut(),
                &mut board,
                oid,
                report_clock,
                report_ticks,
                expire_reason,
            )
            .map_err(|_| ())
        })
    }

    fn trsync_set_timeout(&mut self, oid: u8, clock: u32) -> Result<(), ()> {
        with_trigger_domain(|td| {
            let mut board = OuroborosTrsyncBoard {
                state: &*self.state,
            };
            trsync::trsync_set_timeout(td.state_mut(), &mut board, oid, clock).map_err(|_| ())
        })
    }

    fn trsync_trigger(&mut self, oid: u8, reason: u8) -> Result<u8, ()> {
        with_trigger_domain(|td| {
            let mut board = OuroborosTrsyncBoard {
                state: &*self.state,
            };
            trsync::trsync_trigger(td.state_mut(), &mut board, oid, reason).map_err(|_| ())
        })
    }

    fn register_endstop(&mut self, oid: u8) -> Result<usize, ()> {
        with_trigger_domain(|td| trsync::register_endstop(td.state_mut(), oid).map_err(|_| ()))
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
            let mut board = OuroborosTrsyncBoard {
                state: &*self.state,
            };
            trsync::endstop_start_homing(
                td.state_mut(),
                &mut board,
                oid,
                clock,
                sample_ticks,
                sample_count,
                rest_ticks,
                pin_value_expected,
                trsync_oid,
                trigger_reason,
            )
            .map_err(|_| ())
        })
    }

    fn endstop_query(&mut self, oid: u8) -> Result<(bool, u32, u8), ()> {
        with_trigger_domain(|td| {
            let board = OuroborosTrsyncBoard {
                state: &*self.state,
            };
            trsync::endstop_query(td.state(), &board, oid).map_err(|_| ())
        })
    }

    fn link_stepper_stop(&mut self, stepper_idx: usize, trsync_oid: u8) -> Result<u32, ()> {
        with_trigger_domain(|td| {
            let board = OuroborosTrsyncBoard {
                state: &*self.state,
            };
            trsync::link_stepper_stop(td.state_mut(), &board, stepper_idx, trsync_oid)
                .map_err(|_| ())
        })
    }

    fn notify_homing_start(&mut self) -> Result<(), ()> {
        use core::sync::atomic::Ordering::AcqRel;
        use rtic_sync::channel::TrySendError;

        if HOMING_START_PENDING.swap(true, AcqRel) {
            return Ok(());
        }
        critical_section::with(|cs| {
            let mut sender = TMC_CONTROL_SENDER.borrow(cs).borrow_mut();
            let Some(sender) = sender.as_mut() else {
                return Err(());
            };
            match sender.try_send(TmcControlEvent::HomingStart) {
                Ok(()) | Err(TrySendError::Full(_)) | Err(TrySendError::NoReceiver(_)) => Ok(()),
            }
        })
    }
}

impl<'a> TmcHost for FirmwareCtx<'a> {
    fn enqueue_request(&self, req: TmcRequest) -> Result<(), QueueFull> {
        tmc_context::enqueue_tmc_request(&OuroborosTmcQueue, req)
    }

    fn motion_scale_idle(
        &self,
        channel: u8,
    ) -> Result<(), foci_firmware::motion_scale::MotionScaleRejection> {
        let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
            return Err(foci_firmware::motion_scale::MotionScaleRejection::InvalidChannel);
        };
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            return Err(foci_firmware::motion_scale::MotionScaleRejection::InvalidChannel);
        };
        if tmc_state
            .motor_armed
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            return Err(foci_firmware::motion_scale::MotionScaleRejection::MotorArmed);
        }
        if !crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].is_empty() {
            return Err(foci_firmware::motion_scale::MotionScaleRejection::QueueNotEmpty);
        }
        if stepper_state
            .timer_active_mirror
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            return Err(foci_firmware::motion_scale::MotionScaleRejection::TimerActive);
        }
        Ok(())
    }

    fn motion_quiescent(&self) -> bool {
        (0..crate::board_config::TMC_CHANNEL_COUNT)
            .all(|channel| self.motion_scale_idle(channel).is_ok())
    }

    fn report_motion_scale_rejection(
        &mut self,
        oid: u8,
        rejection: foci_firmware::motion_scale::MotionScaleRejection,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociMotionScaleRejected,
            oid: u8 = oid,
            stage: u8 = foci_firmware::motion_scale::MOTION_SCALE_REJECTION_STAGE_P2,
            reason: u8 = rejection.code(),
        );
    }
}

#[cfg(test)]
mod prepare_and_enqueue_tests {
    use foci_klipper::stepper::{HardwareStepper, StepMove};

    #[test]
    fn queue_full_does_not_consume_pending_direction_change() {
        let mut queue = heapless::spsc::Queue::<StepMove, 2>::new();
        let (mut producer, _consumer) = queue.split();
        let mut stepper = HardwareStepper::new();
        stepper.set_next_step_dir(false);
        producer.enqueue(StepMove::new(100, 1, 0)).unwrap();

        assert!(!producer.ready());
        assert_eq!(producer.len(), 1);
        // Mirrors OuroborosStepperBoard::prepare_and_enqueue's full-check
        // short-circuit: direction is not consumed when the producer is full.
        assert!(stepper.next_move_changes_direction());
    }
}
