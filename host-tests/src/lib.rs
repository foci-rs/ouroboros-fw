#![no_std]
#![allow(dead_code)]

#[cfg(test)]
extern crate std;

mod board_config {
    pub(crate) const TMC_CHANNEL_COUNT: u8 = 2;
}

#[cfg(test)]
mod commissioning_board_contract_tests {
    const MAIN: &str = include_str!("../../src/main.rs");
    const TMC_DISPATCH: &str = include_str!("../../src/tmc_dispatch.rs");
    const REQUEST: &str = include_str!("../../src/tmc_control/request.rs");
    const REPLIES: &str = include_str!("../../src/ankyra/replies.rs");
    const CONSTANTS: &str = include_str!("../../src/ankyra/constants.rs");
    const PROVIDER: &str = include_str!("../../src/ankyra/provider.rs");
    const SHARED_EVENTS: &str =
        include_str!("../../../../shared/foci-firmware/src/trace/events.rs");
    const SHARED_TYPES: &str =
        include_str!("../../../../shared/foci-firmware/src/commissioning/types.rs");
    const SHARED_INTEGRAL_REPORTING: &str = include_str!(
        "../../../../shared/foci-firmware/src/commissioning/outer/velocity/integral_reporting.rs"
    );
    const SHARED_INTEGRAL_PLAN: &str = include_str!(
        "../../../../shared/foci-firmware/src/commissioning/outer/velocity/integral_plan.rs"
    );
    const SHARED_COMMISSION_REPLY_MANIFEST: &str =
        include_str!("../../../../shared/foci-firmware/src/commission_dispatch/replies.rs");

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
    fn startup_establishes_pi_representation_per_channel_before_adc_vm_capture() {
        let chip_identity = MAIN
            .find("CHIPINFO_SI_TYPE")
            .expect("startup must verify each TMC4671 identity");
        let representation = MAIN
            .find("bootstrap_pi_representation")
            .expect("startup must establish the fixed PI representation");
        let adc_vm = MAIN
            .find("capture_adc_vm_offset")
            .expect("startup must retain per-channel ADC-VM capture");

        assert!(chip_identity < representation);
        assert!(representation < adc_vm);
        assert_eq!(MAIN.matches("bootstrap_pi_representation").count(), 1);
        assert!(MAIN.contains("TMC4671 PI representation failed at boot"));
        assert!(!MAIN.contains("0x15"));
    }

    #[test]
    fn board_does_not_forward_removed_stage_b_reply_families() {
        assert!(!REPLIES.contains("foci_velocity_stage_b_reproduction_v3_"));
        for removed in [
            "structured_boundary",
            "directional_region_core",
            "joint_region",
            "stage_b_handoff_core",
            "stage_b_nomination",
            "stage_b_directional_handoff",
            "stage_b_reproduction_v4_",
        ] {
            assert!(!REPLIES.contains(removed), "retained board reply {removed}");
        }
    }

    #[test]
    fn board_publishes_the_stage_b_schema_revision() {
        assert!(CONSTANTS.contains("pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
    }

    #[test]
    fn board_owns_one_channel_checked_stage_c_candidate() {
        assert_eq!(MAIN.matches("StageCCandidateSlot::new()").count(), 1);
        assert_eq!(MAIN.matches("stage_c_candidate.observe_fault()").count(), 2);
        assert!(MAIN.contains("CommissionTerminalContext::new"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
        assert!(REQUEST.contains("stage_c_candidate"));
        assert!(!MAIN.contains("[StageCCandidateSlot"));
    }

    #[test]
    fn board_forwards_the_complete_stage_c_reply_family() {
        for method in [
            "foci_commissioning_workflow_plan",
            "foci_velocity_integral_plan_core",
            "foci_velocity_integral_plan_geometry",
            "foci_velocity_integral_plan_authority",
            "foci_velocity_integral_plan_timing",
            "foci_velocity_integral_plan_travel",
            "foci_velocity_integral_plan_rung",
            "foci_velocity_integral_observation_core",
            "foci_velocity_integral_observation_rate",
            "foci_velocity_integral_observation_quality",
            "foci_velocity_integral_rung_core",
            "foci_velocity_integral_rung_component",
            "foci_velocity_integral_run_summary",
            "foci_velocity_integral_curve_interval",
            "foci_velocity_integral_drift",
            "foci_velocity_integral_stage_b_comparison",
            "foci_velocity_integral_reproduction_core",
            "foci_velocity_integral_reproduction_mask",
            "foci_velocity_integral_reproduction_interval",
            "foci_velocity_integral_reproduction_digest",
            "foci_velocity_integral_terminal_core",
            "foci_velocity_integral_terminal_identity",
            "foci_velocity_integral_terminal_timing",
            "foci_robustness_reversal_terminal",
        ] {
            assert!(REPLIES.contains(method), "missing board reply {method}");
        }
    }

    #[test]
    fn board_forwards_robustness_cycle_evidence_via_the_manifest() {
        assert!(
            SHARED_COMMISSION_REPLY_MANIFEST.contains("name: foci_robustness_cycle_evidence"),
            "missing manifest entry foci_robustness_cycle_evidence"
        );
    }

    #[test]
    fn board_invokes_the_shared_commission_reply_manifest() {
        assert!(
            REPLIES.contains("commission_reply_forwarding_impls"),
            "board no longer wires the shared CommissionReplySink manifest \
             (foci_firmware::__commission_reply_manifest!/\
             foci_firmware_macros::commission_reply_forwarding_impls) -- \
             a board that stops calling it silently drops every \
             manifest-driven reply"
        );
    }

    #[test]
    fn board_forwards_integral_authority_without_narrowing() {
        assert_reply_contract(
            "foci_velocity_integral_plan_authority",
            "FociVelocityIntegralPlanAuthority",
            &[
                "directional_membership: u32",
                "joint_membership: u32",
                "directional_validity: u8",
                "reduced_margin: u8",
            ],
        );
        assert_reply_contract(
            "foci_velocity_integral_run_summary",
            "FociVelocityIntegralRunSummary",
            &["opening_available_mask: u8", "bookend_available_mask: u8"],
        );
        assert_reply_contract(
            "foci_velocity_integral_stage_b_comparison",
            "FociVelocityIntegralStageBComparison",
            &["direction: u8", "available: u8"],
        );
    }

    #[test]
    fn board_publishes_the_stage_c_schema_and_geometry_constants() {
        for declaration in [
            "pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u32 = 13;",
            "pub const STAGE_C_MAX_POSITIVE_RUNGS: u32 = 30;",
            "pub const STAGE_C_OBSERVATIONS_PER_DIRECTION: u32 = 4;",
            "pub const STAGE_C_OBSERVATIONS_PER_RUNG: u32 = 8;",
        ] {
            assert!(CONSTANTS.contains(declaration), "missing {declaration}");
        }
        for constant in [
            "STAGE_B_EVIDENCE_SCHEMA_REVISION",
            "STAGE_C_EVIDENCE_SCHEMA_REVISION",
            "STAGE_C_MAX_POSITIVE_RUNGS",
            "STAGE_C_OBSERVATIONS_PER_DIRECTION",
            "STAGE_C_OBSERVATIONS_PER_RUNG",
        ] {
            assert!(PROVIDER.contains(constant), "provider omits {constant}");
        }
        assert!(SHARED_TYPES.contains("Failed = 5"));
        assert!(
            SHARED_INTEGRAL_REPORTING.contains("PLAN_PROBE_CONSTRAINED_TEST_POINT: u8 = 1 << 1")
        );
        assert!(
            SHARED_INTEGRAL_REPORTING
                .contains("TERMINAL_PROBE_CONSTRAINED_TEST_POINT: u8 = 1 << 3")
        );
        assert_reply_contract(
            "foci_commissioning_workflow_plan",
            "FociCommissioningWorkflowPlan",
            &["shape: u8"],
        );
        assert!(!REPLIES.contains("foci_combined_"));
        assert!(!REPLIES.contains("foci_velocity_combined_"));
    }

    #[test]
    fn board_forwards_rung_origin_recovery() {
        for method in [
            "foci_rung_origin_recovery_summary",
            "foci_velocity_integral_plan_recovery",
        ] {
            assert!(REPLIES.contains(method), "missing board reply {method}");
        }
        for retired in [
            "foci_rung_origin_recovery_core",
            "foci_rung_origin_recovery_position",
            "foci_rung_origin_recovery_timing",
            "foci_rung_origin_recovery_limits",
        ] {
            assert!(!REPLIES.contains(retired), "retained board reply {retired}");
        }
        assert_reply_contract(
            "foci_rung_origin_recovery_summary",
            "FociRungOriginRecoverySummary",
            &[
                "oid: u8",
                "run_sequence: u32",
                "evidence_sequence: u16",
                "stage: u8",
                "rung_index: u8",
                "p_raw: u16",
                "binding_source: u8",
                "outcome: u8",
            ],
        );
        assert!(CONSTANTS.contains("pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
        assert!(CONSTANTS.contains("pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u32 = 13;"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_REST_SELECTION_DIAGNOSTIC: u16 = 151"));
        assert!(!REPLIES.contains("rest_selection"));
    }

    #[test]
    fn trace_event_ids_match_shared_recovery_and_timing_markers() {
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_OUTER_SAFETY_TIMING: u16 = 144"));
    }

    #[test]
    fn board_forwards_the_tune_result_provenance_fields() {
        assert_reply_contract(
            "foci_tune_result",
            "FociTuneResult",
            &[
                "oid: u8",
                "status: u8",
                "velocity_p: u16",
                "velocity_i: u16",
                "velocity_limit: u32",
                "position_p: u16",
                "position_i: u16",
                "velocity_filter_hz: u16",
                "position_filter_hz: u16",
                "flux_filter_hz: u16",
                "torque_filter_hz: u16",
                "warning_code: u8",
                "probed_velocity_mrev_s: u32",
                "d_eq_q: i32",
                "confidence_q: u16",
                "band_lower_percent: u8",
                "band_upper_percent: u8",
                "band_position_q: u16",
            ],
        );
    }

    #[test]
    fn dump_registers_emits_velocity_clamp_values_after_sub_registers() {
        let sub_registers = TMC_DISPATCH
            .find("DUMP_SUB_REGISTERS")
            .expect("dump dispatch must read the sub-register table");
        let clamp_values = TMC_DISPATCH
            .find("clamp_dump_values")
            .expect("dump dispatch must emit the velocity-clamp values");
        let done = TMC_DISPATCH
            .find("FociDumpDone")
            .expect("dump dispatch must send the done reply");

        assert!(
            sub_registers < clamp_values,
            "clamp values must be emitted after the sub-register loop"
        );
        assert!(
            clamp_values < done,
            "clamp values must be emitted before the done reply"
        );
        assert!(TMC_DISPATCH.contains("velocity_clamp_latched()"));
        assert!(TMC_DISPATCH.contains("velocity_clamp_count()"));
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
