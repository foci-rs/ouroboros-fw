#![no_std]
#![allow(dead_code)]

#[cfg(test)]
extern crate std;

mod board_config {
    pub(crate) const TMC_CHANNEL_COUNT: u8 = 2;
}

#[cfg(test)]
mod stage_b_board_contract_tests {
    const MAIN: &str = include_str!("../../src/main.rs");
    const REQUEST: &str = include_str!("../../src/tmc_control/request.rs");
    const REPLIES: &str = include_str!("../../src/ankyra/replies.rs");
    const CONSTANTS: &str = include_str!("../../src/ankyra/constants.rs");

    const REPLY_ID_WORST_CASE_BYTES: usize = 3;
    const KLIPPER_PAYLOAD_WITH_ID_MAX: usize = 59;

    fn assert_reply_contract(method: &str, reply: &str, fields: &[&str]) {
        let start = REPLIES
            .find(method)
            .unwrap_or_else(|| panic!("missing board reply method {method}"));
        let body = &REPLIES[start..];
        assert!(body.contains(reply), "{method} does not forward {reply}");

        let mut cursor = 0;
        for field in fields {
            let offset = body[cursor..]
                .find(field)
                .unwrap_or_else(|| panic!("{method} missing field {field}"));
            cursor += offset + field.len();
        }
    }

    fn assert_wire_budget(name: &str, field_widths: &[usize]) {
        let encoded = field_widths.iter().sum::<usize>() + REPLY_ID_WORST_CASE_BYTES;
        assert!(
            encoded <= KLIPPER_PAYLOAD_WITH_ID_MAX,
            "{name} worst-case payload with id is {encoded} bytes"
        );
    }

    #[test]
    fn board_owns_one_stage_b_candidate_and_threads_it_through_dispatch() {
        assert_eq!(MAIN.matches("StageBCandidateSlot::new()").count(), 1);
        assert!(MAIN.contains("&mut stage_b_candidate"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stage_b"));
    }

    #[test]
    fn board_forwards_the_complete_stage_b_reply_family() {
        for method in [
            "foci_velocity_structured_boundary",
            "foci_velocity_directional_region_core",
            "foci_velocity_directional_region_model",
            "foci_velocity_directional_region_rates",
            "foci_velocity_directional_region_boundary",
            "foci_velocity_joint_region",
            "foci_velocity_stage_b_handoff_core",
            "foci_velocity_stage_b_nomination",
            "foci_velocity_stage_b_directional_handoff",
            "foci_velocity_stage_b_reproduction_core",
            "foci_velocity_stage_b_reproduction_membership",
            "foci_velocity_stage_b_reproduction_interval",
            "foci_velocity_stage_b_reproduction_digest",
            "foci_velocity_stage_b_terminal_core",
            "foci_velocity_stage_b_terminal_identity",
            "foci_velocity_stage_b_terminal_interval",
        ] {
            assert!(REPLIES.contains(method), "missing board reply {method}");
        }
    }

    #[test]
    fn board_publishes_the_stage_b_schema_revision() {
        assert!(CONSTANTS.contains("pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u32 = 2;"));
    }

    #[test]
    fn board_forwards_exact_consensus_reply_contracts() {
        assert_reply_contract(
            "foci_velocity_rung_consensus_core",
            "FociVelocityRungConsensusCore",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "fragment: u8",
                "rung_index: u8",
                "velocity_p: u16",
                "forward_class: u8",
                "reverse_class: u8",
                "forward_collected_mask: u8",
                "reverse_collected_mask: u8",
                "forward_eligible_mask: u8",
                "reverse_eligible_mask: u8",
                "forward_included_mask: u8",
                "reverse_included_mask: u8",
                "forward_component_count: u8",
                "reverse_component_count: u8",
                "forward_operable_count: u8",
                "reverse_operable_count: u8",
                "flags: u8",
            ],
        );
        assert_reply_contract(
            "foci_velocity_rung_consensus_component",
            "FociVelocityRungConsensusComponent",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "fragment: u8",
                "rung_index: u8",
                "direction: u8",
                "component_index: u8",
                "low_q16: i32",
                "high_q16: i32",
            ],
        );
        assert_reply_contract(
            "foci_velocity_rung_consensus_pool",
            "FociVelocityRungConsensusPool",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "fragment: u8",
                "rung_index: u8",
                "direction: u8",
                "pooled_q16: i32",
                "pooled_low_q16: i32",
                "pooled_high_q16: i32",
                "variance_floor_observations: u8",
                "mean_min_mantissa: i32",
                "mean_max_mantissa: i32",
                "envelope_low_mantissa: i32",
                "envelope_high_mantissa: i32",
                "rate_shift: u8",
            ],
        );
        assert!(!REPLIES.contains("FociVelocityRungVerdictBand"));
        assert!(!REPLIES.contains("FociVelocityRungVerdictQuality"));
    }

    #[test]
    fn consensus_replies_fit_the_board_wire_budget() {
        assert_wire_budget(
            "consensus core",
            &[2, 5, 3, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
        );
        assert_wire_budget("consensus component", &[2, 5, 3, 2, 2, 2, 2, 5, 5]);
        assert_wire_budget(
            "consensus pool",
            &[2, 5, 3, 2, 2, 2, 5, 5, 5, 2, 5, 5, 5, 5, 2],
        );
    }
}

#[path = "../../src/state/stepper.rs"]
mod stepper;

#[path = "../../src/pulse_train.rs"]
mod pulse_train;

#[path = "../../src/step_dispatch.rs"]
mod step_dispatch;

#[path = "../../src/tmc_control/containment.rs"]
mod containment;

#[cfg(test)]
mod containment_tests {
    use crate::containment::{
        CommissionWaitExit, CommissionWaitPrecheck, classify_commission_wait_precheck,
    };

    #[test]
    fn observable_containment_prevents_status_fallback() {
        let cases = [
            (true, false, false, false, CommissionWaitExit::Shutdown),
            (false, true, false, false, CommissionWaitExit::QueueFlush),
            (
                false,
                false,
                true,
                false,
                CommissionWaitExit::EmergencyDisable,
            ),
            (
                false,
                false,
                false,
                true,
                CommissionWaitExit::EmergencyDisable,
            ),
        ];

        for (shutdown, flush, disable, fault, expected) in cases {
            assert_eq!(
                classify_commission_wait_precheck(shutdown, flush, disable, fault),
                CommissionWaitPrecheck::Exit(expected),
            );
        }
    }

    #[test]
    fn clear_containment_snapshot_permits_status_fallback() {
        assert_eq!(
            classify_commission_wait_precheck(false, false, false, false),
            CommissionWaitPrecheck::PollCriticalStatus,
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
