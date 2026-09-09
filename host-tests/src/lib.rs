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
    const INTERRUPTS: &str = include_str!("../../src/interrupts.rs");
    const SHARED_AUTOTUNE_TYPES: &str =
        include_str!("../../../../shared/foci-firmware/src/autotune/types.rs");
    const SHARED_CALIBRATION: &str =
        include_str!("../../../../shared/foci-firmware/src/calibration.rs");
    const SHARED_FOCI_HANDLERS: &str =
        include_str!("../../../../shared/foci-firmware/src/ankyra/handlers/foci.rs");
    const USB_TRACE: &str = include_str!("../../src/usb_trace.rs");
    const DRIVER: &str = include_str!("../../../../shared/foci-firmware/src/tmc_control/driver.rs");
    const BOARD: &str = include_str!("../../src/tmc_control/board.rs");
    const STATE_USB_TRACE: &str = include_str!("../../src/state/usb_trace.rs");

    const REPLY_ID_WORST_CASE_BYTES: usize = 3;
    const KLIPPER_PAYLOAD_WITH_ID_MAX: usize = 59;

    fn extract_fn<'a>(text: &'a str, name: &str) -> &'a str {
        let marker = std::format!("fn {name}");
        let start = text
            .find(&marker)
            .unwrap_or_else(|| panic!("missing fn {name}"));
        let brace_start = text[start..]
            .find('{')
            .map(|offset| start + offset)
            .expect("unterminated function signature");
        let mut depth = 0usize;
        for (offset, ch) in text[brace_start..].char_indices() {
            match ch {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        return &text[start..brace_start + offset + 1];
                    }
                }
                _ => {}
            }
        }
        panic!("unterminated fn {name}");
    }

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
    fn internal_usb_phy_remains_usable_during_cpu_sleep() {
        let usb_clock = extract_fn(BOARD_INIT, "enable_usb_clock");
        assert!(
            usb_clock.contains(".ahb1lpenr()")
                && usb_clock.contains("set_usb_otg_hs_ulpilpen(false)"),
            "the internal USB PHY requires the external ULPI clock disabled during CPU sleep"
        );
        assert!(usb_clock.contains("set_usb_otg_hsen(true)"));
        assert!(extract_fn(MAIN, "init").contains("board_init::enable_usb_clock();"));
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
        let chip_identity = DRIVER
            .find("CHIPINFO_SI_TYPE")
            .expect("startup must verify each TMC4671 identity");
        let representation = DRIVER
            .find("bootstrap_pi_representation")
            .expect("startup must establish the fixed PI representation");
        let adc_vm = DRIVER
            .find("capture_adc_vm_offset")
            .expect("startup must retain per-channel ADC-VM capture");

        assert!(chip_identity < representation);
        assert!(representation < adc_vm);
        assert_eq!(DRIVER.matches("bootstrap_pi_representation").count(), 1);
        assert!(MAIN.contains("TMC4671 PI representation failed at boot"));
        assert!(!DRIVER.contains("0x15"));
        assert!(!MAIN.contains("bootstrap_pi_representation"));
        assert!(
            !MAIN.contains("for channel in 0..crate::board_config::TMC_CHANNEL_COUNT"),
            "no channel loops in the task"
        );
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
    fn board_publishes_the_proportional_schema_revision() {
        assert!(CONSTANTS.contains("pub const PROPORTIONAL_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
    }

    #[test]
    fn board_owns_one_channel_checked_integral_candidate() {
        assert_eq!(MAIN.matches("IntegralRetentionSlot::new()").count(), 1);
        assert_eq!(
            MAIN.matches("integral_candidate.observe_fault()").count(),
            2
        );
        assert!(BOARD.contains("CommissionRetention::new"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
        assert!(REQUEST.contains("integral_candidate"));
        assert!(!MAIN.contains("[IntegralRetentionSlot"));
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
        assert!(DRIVER.contains(".classify_commission_poll(poll)"));
        assert!(DRIVER.contains("poll_commissioning_terminal"));
        assert!(DRIVER.contains("is_restore_pending"));
        assert!(
            MAIN.contains("restore_cancel_pending") && MAIN.contains("drive_restore_cancel"),
            "the loop must drain a latched restore cancel"
        );
        assert_eq!(
            MAIN.matches("request_restore_cancel").count(),
            3,
            "every commissioning-loop exit must latch a restore cancel"
        );
    }

    #[test]
    fn board_forwards_the_complete_integral_reply_family() {
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
    fn board_mixes_the_stm32_uid_into_the_trace_boot_id() {
        let compute_boot_id = extract_fn(USB_TRACE, "compute_boot_id");
        assert!(
            compute_boot_id.contains("0x1FF1_E800"),
            "must read the STM32H7 UID base address"
        );
        assert!(compute_boot_id.contains("read_volatile"));
        assert!(compute_boot_id.contains("tim5"));
        assert!(compute_boot_id.contains("dwt"));
    }

    #[test]
    fn board_reports_real_trace_metadata_instead_of_placeholders() {
        for field in [
            "boot_tick_low",
            "boot_tick_high",
            "configured_tick_low",
            "configured_tick_high",
            "pre_config_elapsed_ms",
            "pre_config_skipped_critical",
            "pre_config_skipped_control_detail",
            "pre_config_skipped_status",
        ] {
            assert!(
                STATE_USB_TRACE.contains(field),
                "TraceCounters is missing {field}"
            );
        }
        assert!(
            USB_TRACE.contains("fn set_trace_serial"),
            "missing set_trace_serial"
        );
        assert!(
            USB_TRACE.contains("fn trace_serial"),
            "missing trace_serial"
        );
        assert!(MAIN.contains("usb_trace::set_trace_serial("));

        for declaration in [
            "const CRITICAL_HZ: u32 = 1000;",
            "const CONTROL_DETAIL_HZ: u32 = 500;",
            "const STATUS_HZ: u32 = 10;",
        ] {
            assert!(USB_TRACE.contains(declaration), "missing {declaration}");
        }

        let emit_metadata = extract_fn(USB_TRACE, "emit_metadata");
        assert!(emit_metadata.contains("skipped_sample_counter_count: 3"));
        assert!(emit_metadata.contains("critical_hz: CRITICAL_HZ"));
        assert!(emit_metadata.contains("control_detail_hz: CONTROL_DETAIL_HZ"));
        assert!(emit_metadata.contains("status_hz: STATUS_HZ"));
        for field in [
            "pre_config_elapsed_ms",
            "pre_config_skipped_critical",
            "pre_config_skipped_control_detail",
            "pre_config_skipped_status",
        ] {
            assert!(
                emit_metadata.contains(&std::format!("TRACE_COUNTERS\n            .{field}"))
                    || emit_metadata.contains(&std::format!("TRACE_COUNTERS.{field}")),
                "emit_metadata does not read TRACE_COUNTERS.{field}"
            );
        }
        assert!(!emit_metadata.contains(r#"serial: "","#));
        assert!(emit_metadata.contains("serial: trace_serial()"));
    }

    #[test]
    fn board_emits_the_shared_lifecycle_event_stream() {
        for declaration in [
            "pub(crate) const EVENT_FIRMWARE_BOOT: u16 = 1;",
            "pub(crate) const EVENT_USB_CONFIGURED: u16 = 2;",
            "pub(crate) const EVENT_MOTOR_ARM: u16 = 3;",
            "pub(crate) const EVENT_MOTOR_DISARM: u16 = 4;",
            "pub(crate) const EVENT_STATUS_INTERRUPT: u16 = 9;",
        ] {
            assert!(USB_TRACE.contains(declaration), "missing {declaration}");
        }
        let note_usb_configured = extract_fn(USB_TRACE, "note_usb_configured");
        assert!(note_usb_configured.contains("EVENT_FIRMWARE_BOOT"));
        assert!(note_usb_configured.contains("EVENT_USB_CONFIGURED"));

        let arm_motor = extract_fn(REQUEST, "arm_motor");
        assert!(arm_motor.contains("usb_trace::EVENT_MOTOR_ARM"));
        let disarm_motor = extract_fn(REQUEST, "disarm_motor");
        assert!(disarm_motor.contains("usb_trace::EVENT_MOTOR_DISARM"));

        assert!(MAIN.contains("crate::usb_trace::EVENT_STATUS_INTERRUPT"));
    }

    #[test]
    fn board_wires_the_velocity_limit_latch_test_command_behind_its_feature() {
        assert!(
            PROVIDER.contains(r#"#[cfg(feature = "trace-velocity-limit-latch-test")]"#),
            "provider has no trace-velocity-limit-latch-test-gated block"
        );
        assert!(
            PROVIDER.contains(r#"#[cfg(not(feature = "trace-velocity-limit-latch-test"))]"#),
            "provider has no fallback block for when the feature is disabled"
        );
        assert!(
            PROVIDER.contains("crate::ankyra::handlers::foci::tmc_velocity_limit_latch_test"),
            "provider never lists the tmc_velocity_limit_latch_test command"
        );
    }

    #[test]
    fn board_publishes_the_integral_schema_and_geometry_constants() {
        for declaration in [
            "pub const INTEGRAL_EVIDENCE_SCHEMA_REVISION: u32 = 13;",
            "pub const INTEGRAL_MAX_POSITIVE_RUNGS: u32 = 30;",
            "pub const INTEGRAL_OBSERVATIONS_PER_DIRECTION: u32 = 4;",
            "pub const INTEGRAL_OBSERVATIONS_PER_RUNG: u32 = 8;",
        ] {
            assert!(CONSTANTS.contains(declaration), "missing {declaration}");
        }
        for constant in [
            "PROPORTIONAL_EVIDENCE_SCHEMA_REVISION",
            "INTEGRAL_EVIDENCE_SCHEMA_REVISION",
            "INTEGRAL_MAX_POSITIVE_RUNGS",
            "INTEGRAL_OBSERVATIONS_PER_DIRECTION",
            "INTEGRAL_OBSERVATIONS_PER_RUNG",
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
        assert!(CONSTANTS.contains("pub const PROPORTIONAL_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
        assert!(CONSTANTS.contains("pub const INTEGRAL_EVIDENCE_SCHEMA_REVISION: u32 = 13;"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(!REPLIES.contains("rest_selection"));
    }

    #[test]
    fn trace_event_ids_match_shared_recovery_and_timing_markers() {
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_RUNG_ORIGIN_RECOVERY: u16 = 143"));
        assert!(SHARED_EVENTS.contains("EVENT_COMMISSION_OUTER_SAFETY_TIMING: u16 = 144"));
    }

    #[test]
    fn board_recognizes_commission_cancel_status_code_74() {
        assert!(
            SHARED_AUTOTUNE_TYPES.contains("Cancelled"),
            "CommissionError::Cancelled must exist"
        );
        assert!(
            SHARED_AUTOTUNE_TYPES.contains("Self::Cancelled => 74"),
            "CommissionError::Cancelled must have status code 74"
        );
    }

    #[test]
    fn board_recognizes_calibration_cancel_status_code_10() {
        assert!(
            SHARED_CALIBRATION.contains("Cancelled"),
            "CalibrationError::Cancelled must exist"
        );
        assert!(
            SHARED_CALIBRATION.contains("Self::Cancelled => 10"),
            "CalibrationError::Cancelled must have status code 10"
        );
    }

    #[test]
    fn board_references_the_foci_commission_cancel_command() {
        assert!(
            SHARED_FOCI_HANDLERS.contains("pub fn foci_commission_cancel"),
            "foci_commission_cancel command handler must be defined"
        );
        assert!(
            SHARED_FOCI_HANDLERS.contains("#[klipper_command]\npub fn foci_commission_cancel"),
            "foci_commission_cancel must be marked with #[klipper_command]"
        );
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

    #[test]
    fn step_dispatch_holds_while_homing_preparation_is_pending() {
        assert!(
            MAIN.matches("HOMING_STATE.load(Relaxed) == 1").count() >= 3,
            "every step-dispatch entry must defer while HOMING_STATE == 1"
        );
    }

    #[test]
    fn sampler_reads_position_error_right_after_position_actual() {
        let body = USB_TRACE
            .split("pub(crate) async fn sample_tick")
            .nth(1)
            .expect("sample_tick present");
        let at = |needle: &str| {
            body.find(needle)
                .unwrap_or_else(|| panic!("missing {needle}"))
        };
        let step = at("refs.stepper.position.load(Relaxed)");
        let actual = at("read_register_trace(tmc, PID_POSITION_ACTUAL");
        let error = at("read_sub_register_trace(tmc, PID_ERROR_PID_POSITION_ERROR");
        let velocity = at("read_register_trace(tmc, PID_VELOCITY_ACTUAL");
        assert!(step < actual && actual < error && error < velocity);
        assert!(!body.contains("step_position.wrapping_sub(position_actual)"));
    }

    #[test]
    fn sampler_budget_is_recorded_once_per_channel_maintenance_unit() {
        let body = USB_TRACE
            .split("pub(crate) async fn sample_tick")
            .nth(1)
            .expect("sample_tick present")
            .split("pub(crate) async fn sample_commission_closed_loop_activation")
            .next()
            .expect("sample_tick body");
        assert!(
            !body.contains("record_spi_sample_cycles("),
            "sample_tick must return cycles, not record them"
        );
        assert!(
            USB_TRACE.contains("record_spi_sample_cycles(sample_tick(tmc, context, inputs).await)")
        );

        // Exactly one recording site: the driven unit's own Maintenance
        // outcome, one channel at a time -- no channel loop accumulates
        // cycles across both channels any more.
        assert_eq!(MAIN.matches("record_spi_sample_cycles(").count(), 1);
        assert!(!MAIN.contains("trace_spi_cycles.saturating_add("));
        assert!(!MAIN.contains("for channel in 0..crate::board_config::TMC_CHANNEL_COUNT"));
    }

    #[test]
    fn control_loop_drives_one_unit_per_iteration_and_never_loops_over_channels() {
        let task_start = MAIN
            .find("async fn tmc_control(")
            .expect("tmc_control task");
        let task_end = MAIN[task_start..]
            .find("#[task(")
            .map_or(MAIN.len(), |o| task_start + o);
        let body = &MAIN[task_start..task_end];
        assert!(
            !body.contains("for channel in"),
            "no channel loops inside the future"
        );
        assert!(
            !body.contains("'commission_poll"),
            "the commission poll loop is gone"
        );
        assert_eq!(
            body.matches(".drive(").count(),
            1,
            "exactly one drive await site"
        );
        assert_eq!(body.matches("scheduler.pick(").count(), 1);
        assert_eq!(
            body.matches("scheduler.record(").count(),
            1,
            "finalize and drive share one record site"
        );
        assert!(body.contains("select(rx.recv(), control_sleep)"));
        assert!(body.contains("status_pin_asserted"));
        assert!(!body.contains("CHANNEL1_STATUS_FALLBACK"));
        assert!(!body.contains("poll_channel1_critical_status"));
    }

    #[test]
    fn fault_active_is_cleared_only_after_the_interlock_recheck() {
        let task_start = MAIN
            .find("async fn tmc_control(")
            .expect("tmc_control task");
        let body = &MAIN[task_start..];
        assert_eq!(
            body.matches("FAULT_ACTIVE.store(false").count(),
            1,
            "one emergency-disable clear shared by the fault and deadline-miss paths"
        );
        let clear = body
            .find("FAULT_ACTIVE.store(false")
            .expect("the emergency-disable clear");
        // Wide enough to span the whole containment block from lowering
        // every enable pin down to the clear, with margin.
        let window = &body[clear.saturating_sub(3_200)..clear];
        assert!(
            window.contains("status_levels("),
            "the clear follows a fresh pin poll"
        );
        assert!(
            window.contains("TMC_DISABLE_SIGNAL.signaled()"),
            "the clear follows a pending-signal check"
        );
        assert!(
            window.contains("lower_all_enable_pins(") || window.contains("quiesce_all_channels("),
            "the clear follows lowering every enable pin"
        );
        assert!(
            window.contains("record_stepper_event("),
            "the clear follows the STATUS diagnostic"
        );
        assert!(!MAIN.contains("status_poll_counters"));
    }

    #[test]
    fn board_owns_both_tmc_status_inputs_and_binds_the_tim15_capture() {
        assert!(
            BOARD_INIT.contains("-> Input<'static>"),
            "PE2 STATUS input is returned"
        );
        assert!(!BOARD_INIT.contains("core::mem::forget(status0_pin)"));
        assert!(BOARD_INIT.contains("pub fn status1_level() -> bool"));
        assert!(BOARD_INIT.contains("pub fn init_tim15_status_capture()"));
        assert!(
            BOARD_INIT.contains("set_as_af_unchecked(4"),
            "PA2 is TIM15_CH1 on AF4"
        );
        assert!(MAIN.contains("binds = TIM15"));
        assert!(MAIN.contains("crate::interrupts::tmc_status1_irq("));
        assert!(INTERRUPTS.contains("pub fn tmc_status_irq_body("));
        assert!(!INTERRUPTS.contains("defmt::"), "ISR bodies never log");
        assert!(MAIN.contains("tmc_status0: Input<'static>"));
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
                classify_commission_wait_precheck(shutdown, flush, disable, fault, false, false),
                CommissionWaitPrecheck::Exit(expected),
            );
        }
    }

    #[test]
    fn clear_containment_snapshot_permits_status_fallback() {
        assert_eq!(
            classify_commission_wait_precheck(false, false, false, false, false, false),
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
