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
mod maintenance_quiescence_tests {
    mod predicate {
        include!("../../src/tmc_control/maintenance.rs");
    }

    use foci_firmware::tmc_control::ChannelMode;
    use predicate::{
        ChannelMaintenanceSnapshot, MaintenanceArm, MaintenanceSnapshot, maintenance_arm,
    };

    fn quiescent_channel() -> ChannelMaintenanceSnapshot {
        ChannelMaintenanceSnapshot {
            mode: ChannelMode::Ready,
            driver_obligations_clear: true,
            enable_pin_asserted: false,
            motor_armed: false,
            commission_in_flight: false,
            config_apply_pending: 0,
            feedforward_reset: false,
            pid_error_sums_clear: false,
            homing_clamp_active: false,
            stall_latched: false,
            step_timer_active: false,
            step_queue_empty: true,
        }
    }

    fn quiescent_snapshot() -> MaintenanceSnapshot {
        MaintenanceSnapshot {
            shutdown_active: false,
            fault_active: false,
            queue_flush_pending: false,
            disable_signal_pending: false,
            homing_state: 0,
            homing_triggered: 0xFF,
            homing_start_pending: false,
            channels: [quiescent_channel(), quiescent_channel()],
            trace_enabled: false,
        }
    }

    #[test]
    fn maintenance_quiescence_requires_every_snapshot_field() {
        type Invalidate = fn(&mut MaintenanceSnapshot);
        let invalidations: &[(&str, Invalidate)] = &[
            ("shutdown", |s| s.shutdown_active = true),
            ("fault", |s| s.fault_active = true),
            ("queue flush", |s| s.queue_flush_pending = true),
            ("disable signal", |s| s.disable_signal_pending = true),
            ("homing state", |s| s.homing_state = 1),
            ("homing trigger", |s| s.homing_triggered = 0),
            ("homing start", |s| s.homing_start_pending = true),
            ("channel 0 mode", |s| {
                s.channels[0].mode = ChannelMode::Parked
            }),
            ("channel 1 mode", |s| {
                s.channels[1].mode = ChannelMode::Parked
            }),
            ("channel 0 driver", |s| {
                s.channels[0].driver_obligations_clear = false
            }),
            ("channel 1 driver", |s| {
                s.channels[1].driver_obligations_clear = false
            }),
            ("channel 0 enable", |s| {
                s.channels[0].enable_pin_asserted = true
            }),
            ("channel 1 enable", |s| {
                s.channels[1].enable_pin_asserted = true
            }),
            ("channel 0 armed", |s| s.channels[0].motor_armed = true),
            ("channel 1 armed", |s| s.channels[1].motor_armed = true),
            ("channel 0 commission", |s| {
                s.channels[0].commission_in_flight = true
            }),
            ("channel 1 commission", |s| {
                s.channels[1].commission_in_flight = true
            }),
            ("channel 0 config", |s| {
                s.channels[0].config_apply_pending = 1
            }),
            ("channel 1 config", |s| {
                s.channels[1].config_apply_pending = 1
            }),
            ("channel 0 feedforward", |s| {
                s.channels[0].feedforward_reset = true
            }),
            ("channel 1 feedforward", |s| {
                s.channels[1].feedforward_reset = true
            }),
            ("channel 0 PID", |s| {
                s.channels[0].pid_error_sums_clear = true
            }),
            ("channel 1 PID", |s| {
                s.channels[1].pid_error_sums_clear = true
            }),
            ("channel 0 clamp", |s| {
                s.channels[0].homing_clamp_active = true
            }),
            ("channel 1 clamp", |s| {
                s.channels[1].homing_clamp_active = true
            }),
            ("channel 0 stall", |s| s.channels[0].stall_latched = true),
            ("channel 1 stall", |s| s.channels[1].stall_latched = true),
            ("channel 0 timer", |s| {
                s.channels[0].step_timer_active = true
            }),
            ("channel 1 timer", |s| {
                s.channels[1].step_timer_active = true
            }),
            ("channel 0 queue", |s| {
                s.channels[0].step_queue_empty = false
            }),
            ("channel 1 queue", |s| {
                s.channels[1].step_queue_empty = false
            }),
            ("trace", |s| s.trace_enabled = true),
        ];

        assert_eq!(
            maintenance_arm(quiescent_snapshot()),
            MaintenanceArm::Disarmed
        );
        for (name, invalidate) in invalidations {
            let mut snapshot = quiescent_snapshot();
            invalidate(&mut snapshot);
            assert_eq!(maintenance_arm(snapshot), MaintenanceArm::Armed, "{name}");
        }
    }

    #[test]
    fn maintenance_quiescence_fails_active_for_unsupported_modes_and_multiple_work_items() {
        for mode in [
            ChannelMode::Bootstrap,
            ChannelMode::Commissioning,
            ChannelMode::Restore,
            ChannelMode::Parked,
        ] {
            let mut snapshot = quiescent_snapshot();
            snapshot.channels[0].mode = mode;
            assert_eq!(maintenance_arm(snapshot), MaintenanceArm::Armed);
        }

        let mut snapshot = quiescent_snapshot();
        snapshot.channels[0].motor_armed = true;
        snapshot.channels[1].step_queue_empty = false;
        snapshot.fault_active = true;
        assert_eq!(maintenance_arm(snapshot), MaintenanceArm::Armed);
    }

    #[test]
    fn maintenance_snapshot_is_owned_copy_state() {
        fn assert_owned_copy_static<T: Copy + 'static>() {}

        assert_owned_copy_static::<MaintenanceSnapshot>();
        assert_owned_copy_static::<ChannelMaintenanceSnapshot>();
    }
}

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
