#![no_std]

#[cfg(test)]
extern crate std;

mod board_config {
    pub(crate) const TMC_CHANNEL_COUNT: u8 = 2;
}

mod ankyra {
    pub(crate) mod constants {
        pub(crate) const CLOCK_FREQ: u32 = 260_000_000;
    }
}

mod tmc_control {
    pub(crate) mod board {
        pub(crate) fn tmc_state(
            channel: u8,
        ) -> &'static foci_firmware::state::tmc::TmcChannelState {
            foci_firmware::state::tmc::tmc_channel(channel).expect("validated TMC channel")
        }
    }
}

#[cfg(test)]
mod reserve_pins_contract_tests {
    const CONSTANTS: &str = include_str!("../../src/ankyra/constants.rs");
    const PROVIDER: &str = include_str!("../../src/ankyra/provider.rs");

    #[test]
    fn board_never_declares_an_empty_reserve_pins_category() {
        // RESERVE_PINS_DRV has no genuine pin to reserve: the gate drivers
        // are driven entirely by TMC4671 PWM outputs, with no discrete
        // STM32 GPIO enable line (confirmed against the schematic). Unlike
        // RESERVE_PINS_LED (see board_declares_the_status_led_reserve_pin
        // below), there is nothing to declare here.
        {
            let empty_const = "RESERVE_PINS_DRV";
            assert!(
                !CONSTANTS.contains(&std::format!("pub const {empty_const}: &str = \"\";")),
                "{empty_const} is declared with an empty pin list; Klipper's \
                 mcu_identify reserves the empty string as a real pin, so a \
                 second empty category collides with the first. Real Klipper \
                 firmware omits a RESERVE_PINS_* category entirely instead of \
                 sending an empty one; do the same, or drop the constant."
            );
            assert!(
                !PROVIDER.contains(&std::format!("crate::ankyra::constants::{empty_const},")),
                "provider still wires {empty_const} into the wire constants list"
            );
        }
    }

    #[test]
    fn board_never_lists_the_same_pin_under_two_reserve_pins_categories() {
        use std::vec::Vec;
        let mut categories: Vec<(&str, Vec<&str>)> = Vec::new();
        for line in CONSTANTS.lines() {
            let line = line.trim();
            let Some(rest) = line.strip_prefix("pub const RESERVE_PINS_") else {
                continue;
            };
            let (name, rest) = rest
                .split_once(':')
                .expect("malformed RESERVE_PINS_ declaration");
            let value = rest
                .trim()
                .strip_prefix("&str = \"")
                .and_then(|s| s.strip_suffix("\";"))
                .expect("malformed RESERVE_PINS_ declaration");
            let pins: Vec<&str> = value.split(',').filter(|p| !p.is_empty()).collect();
            categories.push((name, pins));
        }
        assert!(!categories.is_empty(), "no RESERVE_PINS_ constants found");
        for i in 0..categories.len() {
            for j in (i + 1)..categories.len() {
                for pin in &categories[i].1 {
                    assert!(
                        !categories[j].1.contains(pin),
                        "pin {pin} is listed under both RESERVE_PINS_{} and \
                         RESERVE_PINS_{}; Klipper's mcu_identify rejects reserving \
                         the same pin under two different owners",
                        categories[i].0,
                        categories[j].0
                    );
                }
            }
        }
    }
}

#[allow(dead_code)]
#[path = "../../src/state/stepper.rs"]
mod stepper;

#[allow(dead_code)]
#[path = "../../src/pulse_train.rs"]
mod pulse_train;

#[allow(dead_code)]
#[path = "../../src/step_dispatch.rs"]
mod step_dispatch;

#[allow(dead_code)]
#[path = "../../src/tmc_control/containment.rs"]
mod containment;

#[cfg(test)]
mod homing_wake_policy_tests {
    mod policy {
        include!("../../src/tmc_control/wake.rs");
    }

    use policy::{
        HomingPreparationProgress, HomingReceive, HomingWait, HomingWakeEvent, HomingWakePolicy,
    };

    #[test]
    fn homing_wake_dispatches_unlatched_requests_and_doorbells() {
        let mut policy = HomingWakePolicy::new();

        assert_eq!(
            policy.on_event(HomingWakeEvent::Request(7), false),
            HomingReceive::Process(7)
        );
        assert_eq!(
            policy.on_event(HomingWakeEvent::<u8>::HomingStart, true),
            HomingReceive::Doorbell
        );
        assert_eq!(policy.take_retained_request(), None);
    }

    #[test]
    fn homing_wake_retains_a_racing_request_without_overwrite() {
        let mut policy = HomingWakePolicy::new();

        assert_eq!(
            policy.on_event(HomingWakeEvent::Request(7), true),
            HomingReceive::Retained
        );
        assert_eq!(
            policy.on_event(HomingWakeEvent::Request(9), true),
            HomingReceive::RetainedSlotOccupied(9)
        );
        assert_eq!(policy.take_retained_request(), Some(7));
    }

    #[test]
    fn homing_wake_uses_only_maintenance_while_urgent() {
        let mut policy = HomingWakePolicy::new();
        assert_eq!(
            policy.on_event(HomingWakeEvent::Request(7), true),
            HomingReceive::Retained
        );

        assert_eq!(policy.next_wait(true, false), HomingWait::MaintenanceOnly);
        assert_eq!(policy.next_wait(true, true), HomingWait::RunScheduler);
        assert_eq!(policy.next_wait(false, false), HomingWait::ProcessRetained);
    }

    #[test]
    fn homing_wake_retry_keeps_the_latch_and_success_releases_once() {
        let mut policy = HomingWakePolicy::new();
        assert_eq!(
            policy.on_event(HomingWakeEvent::Request(7), true),
            HomingReceive::Retained
        );

        assert!(!HomingWakePolicy::<u8>::should_clear_latch(
            HomingPreparationProgress::Retry
        ));
        assert_eq!(policy.next_wait(true, true), HomingWait::RunScheduler);
        assert!(
            HomingWakePolicy::<u8>::should_clear_latch(HomingPreparationProgress::Deferred),
            "a deferred clamp window is not a failure: release the latch so P1 waits \
             on its normal deadline instead of spinning on urgent maintenance"
        );
        assert!(HomingWakePolicy::<u8>::should_clear_latch(
            HomingPreparationProgress::Advanced
        ));
        assert_eq!(policy.next_wait(false, false), HomingWait::ProcessRetained);
        assert_eq!(policy.take_retained_request(), Some(7));
        assert_eq!(policy.take_retained_request(), None);
        assert_eq!(
            policy.next_wait(false, false),
            HomingWait::ReceiverOrDeadline
        );
    }

    #[test]
    fn homing_wake_fail_closed_releases_latch_ownership() {
        let mut policy = HomingWakePolicy::new();
        assert_eq!(
            policy.on_event(HomingWakeEvent::Request(7), true),
            HomingReceive::Retained
        );

        assert!(HomingWakePolicy::<u8>::should_clear_latch(
            HomingPreparationProgress::FailClosed
        ));
        assert_eq!(policy.next_wait(false, false), HomingWait::ProcessRetained);
        assert_eq!(policy.take_retained_request(), Some(7));
        assert_eq!(policy.take_retained_request(), None);
    }

    #[test]
    fn homing_wake_queue_flush_discards_the_retained_epoch() {
        let mut policy = HomingWakePolicy::new();
        assert_eq!(
            policy.on_event(HomingWakeEvent::Request(7), true),
            HomingReceive::Retained
        );

        policy.reset_for_queue_flush();

        assert_eq!(policy.take_retained_request(), None);
        assert_eq!(
            policy.next_wait(false, false),
            HomingWait::ReceiverOrDeadline
        );
    }
}

#[cfg(test)]
mod pulse_train_tests {
    use foci_klipper::stepper::LogicalStepEvent;
    use std::vec::Vec;

    use crate::pulse_train::{PulseTrainIo, emit_logical_step_event};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Operation {
        Step(bool),
        Dir(bool),
        Wait(u32),
    }

    struct RecordingIo {
        operations: Vec<Operation>,
        step_high: bool,
    }

    impl RecordingIo {
        fn new() -> Self {
            Self {
                operations: Vec::new(),
                step_high: true,
            }
        }
    }

    impl PulseTrainIo for RecordingIo {
        type Error = ();

        fn set_step(&mut self, high: bool) -> Result<(), Self::Error> {
            self.operations.push(Operation::Step(high));
            self.step_high = high;
            Ok(())
        }

        fn set_dir(&mut self, forward: bool) -> Result<(), Self::Error> {
            self.operations.push(Operation::Dir(forward));
            Ok(())
        }

        fn wait_ticks(&mut self, ticks: u32) {
            self.operations.push(Operation::Wait(ticks));
        }
    }

    fn event(physical_pulses: u8, dir_setup_required: bool) -> LogicalStepEvent {
        LogicalStepEvent {
            direction: false,
            physical_pulses,
            dir_setup_required,
        }
    }

    #[test]
    fn zero_pulse_train_forces_step_low_without_waiting() {
        let mut io = RecordingIo::new();

        emit_logical_step_event(&mut io, event(0, false), 22, 11).unwrap();

        assert_eq!(io.operations, [Operation::Step(false)]);
        assert!(!io.step_high);
    }

    #[test]
    fn one_pulse_train_orders_high_and_low_phases_exactly() {
        let mut io = RecordingIo::new();

        emit_logical_step_event(&mut io, event(1, false), 22, 11).unwrap();

        assert_eq!(
            io.operations,
            [
                Operation::Step(false),
                Operation::Step(true),
                Operation::Wait(11),
                Operation::Step(false),
                Operation::Wait(11),
            ]
        );
        assert!(!io.step_high);
    }

    #[test]
    fn two_pulse_train_keeps_exact_low_spacing_between_back_to_back_pulses() {
        let mut io = RecordingIo::new();

        emit_logical_step_event(&mut io, event(2, false), 22, 11).unwrap();

        assert_eq!(
            io.operations,
            [
                Operation::Step(false),
                Operation::Step(true),
                Operation::Wait(11),
                Operation::Step(false),
                Operation::Wait(11),
                Operation::Step(true),
                Operation::Wait(11),
                Operation::Step(false),
                Operation::Wait(11),
            ]
        );
        assert!(!io.step_high);
    }

    #[test]
    fn direction_change_precedes_setup_wait_and_first_pulse() {
        let mut io = RecordingIo::new();

        emit_logical_step_event(&mut io, event(1, true), 22, 11).unwrap();

        assert_eq!(
            io.operations,
            [
                Operation::Step(false),
                Operation::Dir(false),
                Operation::Wait(22),
                Operation::Step(true),
                Operation::Wait(11),
                Operation::Step(false),
                Operation::Wait(11),
            ]
        );
        assert!(!io.step_high);
    }

    #[test]
    #[should_panic(expected = "logical event exceeds two physical pulses")]
    fn pulse_train_rejects_more_than_two_pulses_in_release_code() {
        let mut io = RecordingIo::new();

        let _ = emit_logical_step_event(&mut io, event(3, false), 22, 11);
    }
}

#[cfg(test)]
mod step_dispatch_tests {
    use std::vec::Vec;

    use crate::step_dispatch::{StepDispatchOutcome, run_step_dispatch};

    #[test]
    fn fatal_channel_stops_dispatch_before_later_channel_callback() {
        let channels = [0_u8, 1];
        let mut next_channel = 0;
        let mut callbacks = Vec::new();

        let outcome = run_step_dispatch(|| {
            let channel = channels[next_channel];
            next_channel += 1;
            callbacks.push(channel);
            if channel == 0 {
                StepDispatchOutcome::Fatal
            } else {
                StepDispatchOutcome::Complete
            }
        });

        assert_eq!(outcome, StepDispatchOutcome::Fatal);
        assert_eq!(callbacks, [0]);
    }
}

#[allow(dead_code)]
#[path = "../../src/usb_trace_policy.rs"]
mod usb_trace_policy;

#[cfg(test)]
mod usb_trace_policy_tests {
    use foci_firmware::trace::sampling::TraceMode;

    use foci_firmware::context::QueueFull;
    use foci_firmware::trace::control_state::{
        AppliedRequest, ControlStateHeader, ControlStateRecord,
    };

    use crate::usb_trace_policy::{
        EmitOutcome, MAX_CONTROL_STATE_FRAMES_PER_TICK, TraceModeRequests, emit_pending,
        skipped_control_detail_for, skipped_critical_for,
    };

    #[test]
    fn pre_config_skipped_critical_counts_only_critical_mode_time() {
        assert_eq!(skipped_critical_for(2000, 0), 0);
        assert_eq!(skipped_critical_for(2000, 500), 500);
    }

    #[test]
    fn pre_config_skipped_control_detail_uses_100_hz() {
        assert_eq!(skipped_control_detail_for(2000), 200);
    }

    #[test]
    fn mode_requests_are_serviced_in_order_with_their_oid() {
        let q = TraceModeRequests::new();
        assert!(q.push(TraceMode::Critical, 3).is_ok());
        assert!(q.push(TraceMode::Diagnostic, 4).is_ok());
        assert!(!q.is_empty());
        assert_eq!(q.pop(), Some((TraceMode::Critical, 3)));
        assert_eq!(q.pop(), Some((TraceMode::Diagnostic, 4)));
        assert_eq!(q.pop(), None);
    }

    #[test]
    fn a_full_ring_rejects_the_push_and_keeps_the_queued_modes() {
        let q = TraceModeRequests::new();
        for m in [
            TraceMode::Critical,
            TraceMode::Dormant,
            TraceMode::Diagnostic,
            TraceMode::Critical,
        ] {
            assert!(q.push(m, 3).is_ok());
        }
        assert!(q.push(TraceMode::Dormant, 3).is_err());
        assert_eq!(q.pop(), Some((TraceMode::Critical, 3)));
        assert!(q.push(TraceMode::Dormant, 3).is_ok());
    }

    use core::cell::RefCell;

    type Sent = RefCell<heapless::Vec<ControlStateHeader, 8>>;

    fn accept_into(sent: &Sent) -> impl FnMut(ControlStateHeader) -> Result<(), QueueFull> + '_ {
        |h| {
            sent.borrow_mut().push(h).unwrap();
            Ok(())
        }
    }

    fn sequences(sent: &Sent) -> heapless::Vec<u32, 8> {
        sent.borrow().iter().map(|h| h.change_sequence).collect()
    }

    #[test]
    fn four_filter_requests_yield_four_frames_in_order_with_consecutive_sequences() {
        let r = ControlStateRecord::new();
        for kind in [
            AppliedRequest::VelocityFilter,
            AppliedRequest::TorqueFilter,
            AppliedRequest::PositionFilter,
            AppliedRequest::FluxFilter,
        ] {
            r.record(kind);
        }
        r.stamp(10);
        let sent = Sent::default();
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        let sent = sent.borrow();
        let kinds: heapless::Vec<_, 4> = sent.iter().map(|h| h.applied_request).collect();
        assert_eq!(
            kinds.as_slice(),
            [
                AppliedRequest::VelocityFilter,
                AppliedRequest::TorqueFilter,
                AppliedRequest::PositionFilter,
                AppliedRequest::FluxFilter,
            ]
        );
        let seqs: heapless::Vec<_, 4> = sent.iter().map(|h| h.change_sequence).collect();
        assert_eq!(seqs.as_slice(), [1, 2, 3, 4]);
    }

    #[test]
    fn a_sender_that_takes_two_frames_leaves_the_rest_for_the_next_call_without_repeats() {
        let r = ControlStateRecord::new();
        for _ in 0..4 {
            r.record(AppliedRequest::RunCurrent);
        }
        r.stamp(10);
        let sent = Sent::default();
        let mut room = 2;
        assert_eq!(
            emit_pending(&r, |h| {
                if room == 0 {
                    return Err(QueueFull);
                }
                room -= 1;
                sent.borrow_mut().push(h).unwrap();
                Ok(())
            }),
            EmitOutcome::Retained
        );
        assert_eq!(sequences(&sent).as_slice(), [1, 2]);
        assert!(r.is_dirty());
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        assert_eq!(sequences(&sent).as_slice(), [1, 2, 3, 4]);
        assert_eq!(
            emit_pending(&r, |_| panic!("must not emit twice")),
            EmitOutcome::Clean
        );
    }

    #[test]
    fn a_refused_entry_is_offered_again_and_accepted_exactly_once() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.stamp(10);
        assert_eq!(emit_pending(&r, |_| Err(QueueFull)), EmitOutcome::Retained);
        assert!(r.is_dirty());
        r.record(AppliedRequest::RunCurrent);
        r.stamp(11);
        let sent = Sent::default();
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        assert_eq!(sequences(&sent).as_slice(), [1, 2]);
        assert!(!r.is_dirty());
        assert_eq!(
            emit_pending(&r, |_| panic!("must not emit twice")),
            EmitOutcome::Clean
        );
    }

    #[test]
    fn snapshot_frame_precedes_mutator_frames_and_retries_independently() {
        let r = ControlStateRecord::new();
        r.record(AppliedRequest::OuterGains);
        r.record(AppliedRequest::RunCurrent);
        r.stamp(10);
        r.mark_snapshot();
        let sent = Sent::default();
        let mut room = 1;
        assert_eq!(
            emit_pending(&r, |h| {
                if room == 0 {
                    return Err(QueueFull);
                }
                room -= 1;
                sent.borrow_mut().push(h).unwrap();
                Ok(())
            }),
            EmitOutcome::Retained
        );
        assert_eq!(sent.borrow()[0].applied_request, AppliedRequest::None);
        assert!(!r.is_snapshot_pending());
        assert!(r.is_dirty());
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        let kinds: heapless::Vec<_, 3> = sent.borrow().iter().map(|h| h.applied_request).collect();
        assert_eq!(
            kinds.as_slice(),
            [
                AppliedRequest::None,
                AppliedRequest::OuterGains,
                AppliedRequest::RunCurrent,
            ]
        );
    }

    #[test]
    fn mutator_frames_per_call_are_capped_and_the_remainder_goes_next_call() {
        let r = ControlStateRecord::new();
        for _ in 0..MAX_CONTROL_STATE_FRAMES_PER_TICK + 2 {
            r.record(AppliedRequest::VoltageLimit);
        }
        r.stamp(10);
        let sent = Sent::default();
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Retained);
        assert_eq!(sequences(&sent).as_slice(), [1, 2, 3, 4]);
        assert_eq!(emit_pending(&r, accept_into(&sent)), EmitOutcome::Queued);
        assert_eq!(sequences(&sent).as_slice(), [1, 2, 3, 4, 5, 6]);
    }
}
