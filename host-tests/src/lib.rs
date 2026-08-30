#![no_std]
#![allow(dead_code)]

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
    const BOARD_CARGO_TOML: &str = include_str!("../../Cargo.toml");
    const TRIGGER_DOMAIN: &str = include_str!("../../src/trigger_domain.rs");
    const BOARD_INIT: &str = include_str!("../../src/board_init.rs");

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

    /// Same contract `assert_reply_contract` checks, against the shared
    /// manifest's `entry { ... }` text instead of a board's compiled
    /// REPLIES -- valid because `entry { name: <method>, ..., reply:
    /// <Type>, ..., fields: { <field>: <ty>, ... } }` contains the method
    /// name, then (searching forward) the reply type, then each field in
    /// order, the same shape `assert_reply_contract` already expects.
    fn assert_manifest_entry(method: &str, reply: &str, fields: &[&str]) {
        let start = SHARED_COMMISSION_REPLY_MANIFEST
            .find(&std::format!("name: {method}"))
            .unwrap_or_else(|| panic!("missing manifest entry {method}"));
        let after = &SHARED_COMMISSION_REPLY_MANIFEST[start..];
        // Bound the search to this one entry: many entries share common
        // field names (`oid: u8` alone appears in most of them), so without
        // an upper bound a missing/misordered field in this entry could be
        // silently satisfied by a *later* entry's matching text instead --
        // the check would pass while proving nothing about this method.
        // `after` starts at "name: <method>", i.e. strictly after this
        // entry's own opening "entry {", so the next occurrence of
        // "entry {" (if any) is the following entry's, never this one's.
        let entry_end = after.find("entry {").unwrap_or(after.len());
        let body = &after[..entry_end];
        assert!(
            body.contains(reply),
            "{method} manifest entry does not declare {reply}"
        );

        let mut cursor = 0;
        for field in fields {
            let offset = body[cursor..]
                .find(field)
                .unwrap_or_else(|| panic!("{method} manifest entry missing field {field}"));
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
    fn spi_dma_interrupt_priority_is_explicit_and_below_hard_real_time() {
        assert!(
            BOARD_INIT.contains(
                "config.dma_interrupt_priority = embassy_stm32::interrupt::Priority::P15;"
            ),
            "rcc_config() must set dma_interrupt_priority explicitly to P15, not inherit Embassy's P0 default"
        );
        assert!(
            MAIN.contains("board_init::verify_dma_interrupt_priority();"),
            "init() must call verify_dma_interrupt_priority() to check the programmed NVIC priority at boot"
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
        assert!(MAIN.contains("CommissionRetention::new"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
        assert!(REQUEST.contains("stage_c_candidate"));
        assert!(!MAIN.contains("[StageCCandidateSlot"));
    }

    #[test]
    fn board_polls_the_terminal_restore_and_cancels_it_on_every_exit() {
        // The per-channel terminal restore is a poll-driven machine on each
        // runtime, not an awaited chain inside this task's future. Every
        // commissioning-loop exit must latch a cancel, and the loop must drain
        // that latch before touching the channel again.
        assert!(
            !MAIN.contains("drive_commissioning_poll_with_stages"),
            "the board must not await the whole terminal chain in one future"
        );
        assert!(MAIN.contains("begin_commissioning_poll_with_stages"));
        assert!(MAIN.contains("poll_commissioning_terminal"));
        assert!(MAIN.contains("is_restore_pending"));
        assert!(
            MAIN.contains("restore_cancel_pending") && MAIN.contains("drive_restore_cancel"),
            "the loop must drain a latched restore cancel"
        );
        assert_eq!(
            MAIN.matches("request_restore_cancel").count(),
            10,
            "every commissioning-loop exit must latch a restore cancel"
        );
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
            "foci_velocity_integral_terminal",
            "foci_robustness_reversal_terminal",
        ] {
            assert!(
                SHARED_COMMISSION_REPLY_MANIFEST.contains(&std::format!("name: {method}")),
                "missing manifest entry {method}"
            );
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
    fn board_cargo_toml_depends_on_foci_firmware_macros() {
        assert!(
            BOARD_CARGO_TOML.contains("foci-firmware-macros"),
            "board's Cargo.toml no longer depends on foci-firmware-macros, \
             so the shared manifest invocation above cannot resolve"
        );
    }

    #[test]
    fn board_forwards_integral_authority_without_narrowing() {
        assert_manifest_entry(
            "foci_velocity_integral_plan_authority",
            "FociVelocityIntegralPlanAuthority",
            &[
                "directional_membership: u32",
                "joint_membership: u32",
                "directional_validity: u8",
                "reduced_margin: u8",
            ],
        );
        assert_manifest_entry(
            "foci_velocity_integral_terminal",
            "FociVelocityIntegralTerminal",
            &["oid: u8", "payload: &[u8]"],
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
        assert_manifest_entry(
            "foci_commissioning_workflow_plan",
            "FociCommissioningWorkflowPlan",
            &["shape: u8"],
        );
        assert!(!REPLIES.contains("foci_combined_"));
        assert!(!REPLIES.contains("foci_velocity_combined_"));
    }

    #[test]
    fn board_forwards_recovery_plan_geometry_but_not_the_retired_recovery_reply() {
        assert!(
            SHARED_COMMISSION_REPLY_MANIFEST.contains("name: foci_velocity_integral_plan_recovery"),
            "missing manifest entry foci_velocity_integral_plan_recovery"
        );
        for retired in [
            "foci_rung_origin_recovery_core",
            "foci_rung_origin_recovery_position",
            "foci_rung_origin_recovery_timing",
            "foci_rung_origin_recovery_limits",
            "foci_rung_origin_recovery_summary",
        ] {
            assert!(!REPLIES.contains(retired), "retained board reply {retired}");
        }
        assert!(CONSTANTS.contains("pub const STAGE_B_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
        assert!(CONSTANTS.contains("pub const STAGE_C_EVIDENCE_SCHEMA_REVISION: u32 = 13;"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
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

    #[test]
    fn p1_unmasked_drain_never_reacquires_a_critical_section() {
        let unmasked_start = TRIGGER_DOMAIN
            .find("QueueAccess::Unmasked => {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess::Unmasked");
        let locked_start = TRIGGER_DOMAIN
            .find("QueueAccess::LockedInPlace => {")
            .expect("the Unmasked arm must be followed by the LockedInPlace arm");
        assert!(unmasked_start < locked_start);
        let unmasked_arm = &TRIGGER_DOMAIN[unmasked_start..locked_start];

        let detach_pos = unmasked_arm
            .find("detach_consumer(")
            .expect("the unmasked arm must detach the consumer before draining");
        let dequeue_pos = unmasked_arm.find("consumer.dequeue()").expect(
            "the unmasked arm must actually drain the detached consumer, \
                     not just call detach/attach with nothing in between",
        );
        let attach_pos = unmasked_arm
            .find("attach_consumer(")
            .expect("the unmasked arm must reattach the consumer after draining");

        assert!(
            detach_pos < dequeue_pos,
            "the drain loop must run after detaching the consumer, not before"
        );
        assert!(
            dequeue_pos < attach_pos,
            "the drain loop must run before reattaching the consumer, not after"
        );
        assert!(
            !unmasked_arm.contains("interrupt::free"),
            "the unmasked arm must never reacquire a critical section directly -- \
             masking belongs only inside detach_consumer/attach_consumer, not the drain loop"
        );
    }

    /// Return the index of the `}` matching the `{` at `open_brace_pos`
    /// (which must itself be a `{` character), by counting brace depth.
    /// Comment- and string-literal-blind, like every other source-contract
    /// check in this module -- adequate for this crate's existing use of
    /// `include_str!` guards, not a real parser.
    fn find_matching_close_brace(text: &str, open_brace_pos: usize) -> usize {
        let mut depth = 0i32;
        for (offset, ch) in text[open_brace_pos..].char_indices() {
            match ch {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return open_brace_pos + offset;
                    }
                }
                _ => {}
            }
        }
        panic!("no matching closing brace found for the brace at byte {open_brace_pos}");
    }

    #[test]
    fn stop_physical_stepper_side_effects_are_not_duplicated_per_queue_access_branch() {
        assert_eq!(
            TRIGGER_DOMAIN.matches("record_physical_stop(").count(),
            1,
            "diagnostic recording must be shared after the QueueAccess branch, not \
             duplicated per arm -- duplicated tail logic can silently diverge between arms"
        );
        let locked_open_marker = TRIGGER_DOMAIN
            .find("QueueAccess::LockedInPlace => {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess::LockedInPlace");
        let locked_open_brace = locked_open_marker + "QueueAccess::LockedInPlace => ".len();
        assert_eq!(
            &TRIGGER_DOMAIN[locked_open_brace..locked_open_brace + 1],
            "{"
        );
        let locked_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, locked_open_brace);

        let record_pos = TRIGGER_DOMAIN
            .find("record_physical_stop(")
            .expect("stop_physical_stepper_inner must record the stop");
        let reset_idle_pos = TRIGGER_DOMAIN
            .find("stepper_pins::reset_idle(")
            .expect("stop_physical_stepper_inner must reset STEP/DIR GPIOs to idle");

        assert!(
            locked_close_brace < record_pos,
            "diagnostic recording must run after the LockedInPlace arm's actual \
             closing brace, not nested inside it"
        );
        assert!(
            locked_close_brace < reset_idle_pos,
            "the GPIO idle reset must run after the LockedInPlace arm's actual \
             closing brace, not nested inside it"
        );
        assert!(
            record_pos < reset_idle_pos,
            "diagnostic recording must run before the GPIO idle reset, both \
             unconditionally outside the branch -- so both happen regardless of \
             whether the locked path observed None"
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
                classify_commission_wait_precheck(shutdown, flush, disable, fault, false),
                CommissionWaitPrecheck::Exit(expected),
            );
        }
    }

    #[test]
    fn clear_containment_snapshot_permits_status_fallback() {
        assert_eq!(
            classify_commission_wait_precheck(false, false, false, false, false),
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
