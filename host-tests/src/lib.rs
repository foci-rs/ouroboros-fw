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
    const SHARED_COMMISSION_REPLY_MANIFEST: &str =
        include_str!("../../../../shared/foci-firmware/src/commission_dispatch/replies.rs");
    const BOARD_CARGO_TOML: &str = include_str!("../../Cargo.toml");
    const TRIGGER_DOMAIN: &str = include_str!("../../src/trigger_domain.rs");
    const BOARD_INIT: &str = include_str!("../../src/board_init.rs");
    const INTERRUPTS: &str = include_str!("../../src/interrupts.rs");
    const STEPPER_HELPERS: &str = include_str!("../../src/helpers/stepper.rs");
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
    const TMC_DIAGNOSTICS: &str = include_str!("../../src/tmc_diagnostics.rs");
    const TMC_STATE: &str = include_str!("../../src/state/tmc.rs");
    const TMC_HELPERS: &str = include_str!("../../src/helpers/tmc.rs");
    const FIRMWARE_CTX: &str = include_str!("../../src/firmware_ctx.rs");
    const HOMING_WAKE: &str = include_str!("../../src/tmc_control/wake.rs");
    const MAINTENANCE: &str = include_str!("../../src/tmc_control/maintenance.rs");

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
    fn debug_sleep_clock_is_disabled_before_idle() {
        assert!(
            !BOARD_INIT.contains("set_dbgsleep_d1(true)"),
            "production must not keep the D1 debug clock running during Sleep"
        );

        let helper_name = "disable_debug_clock_during_sleep";
        let helper = extract_fn(BOARD_INIT, helper_name);
        assert!(
            helper.contains(".modify(|w| w.set_dbgsleep_d1(false));"),
            "the board helper must explicitly clear DBGSLEEP_D1 on every boot"
        );

        let helper_start = BOARD_INIT
            .find(&std::format!("pub fn {helper_name}"))
            .expect("missing debug-sleep helper declaration");
        let declaration_start = BOARD_INIT[..helper_start]
            .rfind("\n}\n")
            .map_or(0, |offset| offset + 3);
        assert!(
            !BOARD_INIT[declaration_start..helper_start].contains("#[cfg"),
            "the debug-sleep helper must not be feature-gated"
        );

        let init = extract_fn(MAIN, "init");
        let hal_init = init
            .find("embassy_stm32::init(crate::board_init::rcc_config())")
            .expect("application init must initialize the HAL");
        let call = init
            .find("crate::board_init::disable_debug_clock_during_sleep();")
            .expect("application init must disable the D1 debug clock");
        let scheduler_start = init
            .find("stepper_reconcile::spawn()")
            .expect("application init must start the scheduler tasks");
        assert!(hal_init < call);
        assert!(call < scheduler_start);
        assert!(
            !init[hal_init..call].contains("#[cfg") && !init[hal_init..call].contains("cfg!("),
            "the boot-path debug-sleep call must be unconditional"
        );
    }

    #[test]
    fn maintenance_arm_integration_applies_a_fresh_complete_snapshot() {
        let builder = extract_fn(BOARD, "apply_live_maintenance_arm");
        let channel_builder = extract_fn(BOARD, "maintenance_channel_snapshot");

        let decision = builder
            .find("let arm = match maintenance_arm(MaintenanceSnapshot {")
            .expect("fresh arm decision");
        let apply = builder
            .find("scheduler.set_maintenance_arm(arm, now_us);")
            .expect("immediate scheduler application");
        assert!(decision < apply);
        assert!(!builder[decision..apply].contains(".await"));
        assert!(builder.contains("MaintenanceArm::Armed => SchedulerMaintenanceArm::Armed"));
        assert!(builder.contains("MaintenanceArm::Disarmed => SchedulerMaintenanceArm::Disarmed"));
        assert!(builder.contains("maintenance_channel_snapshot(0, &drivers[0])"));
        assert!(builder.contains("maintenance_channel_snapshot(1, &drivers[1])"));
        assert!(channel_builder.contains("driver.maintenance_obligations_clear()"));
        assert!(!channel_builder.contains("driver.runtime"));
        assert!(channel_builder.contains("STEP_QUEUE_CHANNELS[channel as usize].is_empty()"));
        assert!(builder.contains("trace_enabled: cfg!(feature = \"trace\")"));
        assert!(!MAINTENANCE.contains("embassy_"));
        assert!(!MAINTENANCE.contains("rtic"));
        assert!(!MAINTENANCE.contains("crate::state"));
    }

    #[test]
    fn maintenance_arm_integration_orders_handoff_before_request_latches() {
        let builder = extract_fn(BOARD, "maintenance_channel_snapshot");
        let timer = builder
            .find("timer_active_mirror.load(Relaxed)")
            .expect("step-timer source read");
        let queue = builder
            .find("STEP_QUEUE_CHANNELS[channel as usize].is_empty()")
            .expect("step-queue source read");
        let fence = builder
            .find("compiler_fence(Acquire)")
            .expect("compiler handoff fence");
        let feedforward = builder
            .find("feedforward_reset.load(Relaxed)")
            .expect("feed-forward request-latch read");
        let pid = builder
            .find("pid_error_sums_clear.load(Relaxed)")
            .expect("PID-clear request-latch read");

        for source in [timer, queue] {
            assert!(source < fence);
        }
        for latch in [feedforward, pid] {
            assert!(fence < latch);
        }
    }

    #[test]
    fn maintenance_arm_integration_recomputes_at_every_control_boundary() {
        let body = extract_fn(MAIN, "tmc_control");
        let calls: std::vec::Vec<_> = body
            .match_indices("apply_live_maintenance_arm(")
            .map(|(offset, _)| offset)
            .collect();
        let wait = body
            .find("let wait = homing_wake.next_wait(")
            .expect("wait decision");
        let receive = body
            .find("let received_event = match wait")
            .expect("receive path");
        let request = body
            .find("if let Some(req) = request")
            .expect("request path");
        let request_await = body[request..]
            .find(".await")
            .map(|offset| request + offset)
            .expect("request handler await");
        let pick = body
            .find("scheduler.pick(now, homing_state)")
            .expect("scheduler pick");
        let final_receive_await = body[..request].rfind(".await").expect("receive wait");

        assert_eq!(calls.len(), 3);
        assert!(calls[0] < wait);
        assert!(!body[calls[0]..wait].contains(".await"));
        assert!(receive < final_receive_await);
        assert!(final_receive_await < calls[1]);
        assert!(calls[1] < request);
        assert!(request_await < calls[2]);
        assert!(calls[2] < pick);
        assert!(!body[calls[2]..pick].contains(".await"));
    }

    #[test]
    fn maintenance_arm_integration_removes_the_empty_group_seam() {
        let body = extract_fn(MAIN, "tmc_control");
        let helper = extract_fn(BOARD, "apply_live_maintenance_arm");
        let first_apply = body
            .find("apply_live_maintenance_arm(")
            .expect("level-driven arm application");
        let wait = body
            .find("let wait = homing_wake.next_wait(")
            .expect("wait decision");
        let finalize = body
            .find("UnitClass::MaintenanceFinalize")
            .expect("maintenance finalize path");
        let record = body[finalize..]
            .find("scheduler.record(")
            .map(|offset| finalize + offset)
            .expect("finalize record");
        let next_iteration = body[record..]
            .find("let Some(outcome) = outcome else {\n                continue;")
            .expect("finalize continues to the next level-driven decision");

        assert!(!body.contains("advance_quiescent_maintenance_grid"));
        assert!(!BOARD.contains("fn live_maintenance_arm"));
        assert!(!helper.contains("maintenance_group"));
        assert!(first_apply < wait);
        assert!(next_iteration > 0);
    }

    #[test]
    fn maintenance_arm_integration_arms_urgent_homing_before_reading_its_deadline() {
        let body = extract_fn(MAIN, "tmc_control");
        let urgent_start = body
            .find("HomingWait::MaintenanceOnly =>")
            .expect("urgent homing wait arm");
        let urgent_end = body[urgent_start..]
            .find("HomingWait::ReceiverOrDeadline =>")
            .map(|offset| urgent_start + offset)
            .expect("receiver wait arm");
        let urgent = &body[urgent_start..urgent_end];
        let arm = urgent
            .find("scheduler.set_maintenance_arm(MaintenanceArm::Armed, now_us());")
            .expect("explicit urgent rearm");
        let deadline = urgent
            .find("scheduler.maintenance_due_us()")
            .expect("urgent maintenance deadline read");
        let await_point = urgent.find(".await").expect("urgent deadline await");

        assert!(arm < deadline);
        assert!(deadline < await_point);
    }

    #[test]
    fn maintenance_arm_integration_reports_state_and_transition_counts() {
        let body = extract_fn(MAIN, "tmc_control");

        assert!(body.contains("let arm = scheduler.maintenance_arm_stats();"));
        assert!(body.contains("maint arm={:?} a2d={} d2a={}"));
        assert!(body.contains("arm.state"));
        assert!(body.contains("arm.armed_to_disarmed"));
        assert!(body.contains("arm.disarmed_to_armed"));
        assert_eq!(body.matches("maintenance_arm_stats()").count(), 1);
    }

    #[test]
    fn tim5_compare_flags_use_race_safe_write_zero_to_clear_writes() {
        let clear_compare = extract_fn(STEPPER_HELPERS, "clear_tim5_compare_flag");
        assert!(clear_compare.contains("tim5.sr().write("));
        assert!(!clear_compare.contains(".modify("));

        for source in [MAIN, STEPPER_HELPERS, TRIGGER_DOMAIN] {
            assert!(
                !source.contains("tim5.sr().modify("),
                "TIM5 compare flags must not use read-modify-write on a \
                 write-zero-to-clear register"
            );
        }
    }

    #[test]
    fn queue_empty_marks_stepper_idle_before_disarming_channel() {
        let load_next = extract_fn(MAIN, "load_next_step_move");
        let mark_idle = load_next
            .find("stepper.mark_idle();")
            .expect("queue-empty path must mark the stepper idle");
        let record_empty = load_next
            .find("record_queue_empty(channel);")
            .expect("queue-empty path must record the transition");
        let clear_channel = load_next[record_empty..]
            .find("clear_step_channel(channel, next_event_clocks);")
            .map(|offset| record_empty + offset)
            .expect("queue-empty path must disarm the channel");

        assert!(mark_idle < record_empty);
        assert!(record_empty < clear_channel);
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
    fn board_publishes_the_proportional_schema_revision() {
        assert!(CONSTANTS.contains("pub const PROPORTIONAL_EVIDENCE_SCHEMA_REVISION: u32 = 14;"));
    }

    #[test]
    fn board_owns_and_indexes_integral_candidates_by_channel() {
        assert!(MAIN.contains("IntegralRetentionSlots<"));
        assert!(MAIN.contains("{ TMC_CHANNEL_COUNT as usize }"));
        assert_eq!(
            MAIN.matches("integral_candidates.invalidate_all()").count(),
            2
        );
        assert!(MAIN.contains(".slot_mut(ch)"));
        assert!(BOARD.contains("CommissionRetention::new"));
        assert!(REQUEST.contains("handle_tmc_control_request_with_stages"));
        assert!(REQUEST.contains(".slot_mut(channel)"));
    }

    #[test]
    fn board_polls_the_terminal_restore_and_cancels_it_on_every_exit() {
        // The per-channel terminal restore is a poll-driven machine on each
        // runtime, not an awaited chain inside this task's future. Every
        // commissioning-loop exit must latch a cancel, and the loop must drain
        // that latch before touching the channel again.
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
    fn board_declares_the_status_led_reserve_pin() {
        // Unlike RESERVE_PINS_DRV, Ouroboros has a genuine MCU-driven status
        // LED on PA15 (confirmed against the schematic's MCU and LEDs
        // blocks, distinct from the PE2/PA2 pins repurposed as TMC4671
        // STATUS inputs on production v1.0). It must be reserved so
        // Klipper's pin allocator cannot hand PA15 to user config.
        assert!(
            CONSTANTS.contains(r#"pub const RESERVE_PINS_LED: &str = "PA15";"#),
            "RESERVE_PINS_LED must declare the board status LED pin (PA15)"
        );
        assert_eq!(
            PROVIDER
                .matches("crate::ankyra::constants::RESERVE_PINS_LED,")
                .count(),
            1,
            "provider must wire RESERVE_PINS_LED into the constants list"
        );
    }

    #[test]
    fn board_reserves_the_layer2_brake_gpio() {
        // PE14 (BRK3) drives the STM32-controlled Layer 2 brake circuit,
        // independent of the TMC4671's own autonomous ADC_VM_LIMITS brake
        // path. Not yet driven by firmware, but it must still be reserved
        // so Klipper's pin allocator cannot hand it to user config.
        assert!(
            CONSTANTS.contains(r#"pub const RESERVE_PINS_TMC: &str = "PD1,PD6,PE2,PA2,PE14";"#),
            "RESERVE_PINS_TMC must include the Layer 2 brake GPIO (PE14)"
        );
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
                "report_seq: u8",
                "status: u8",
                "velocity_p: u16",
                "velocity_i: u16",
                "position_p: u16",
                "probed_velocity_mrev_s: u32",
                "d_eq_q: i32",
                "confidence_q: u16",
                "band_lower_percent: u8",
                "band_upper_percent: u8",
                "band_position_q: u16",
                "predicted_start_p: u16",
                "rungs_measured: u8",
                "stimulus_feedforward_paths: u8",
                "position_tune_outcome_code: u8",
            ],
        );
    }

    #[test]
    fn board_forwards_the_tune_position_evidence_fields() {
        assert_reply_contract(
            "foci_tune_position_evidence",
            "FociTunePositionEvidence",
            &[
                "oid: u8",
                "report_seq: u8",
                "last_rung_p: u16",
                "min_overshoot_p: u16",
                "min_overshoot_units: u32",
                "bound_units: u32",
                "homing_speed_mrev_s: u32",
                "motion_speed_mrev_s: u32",
                "homing_peak_abs_units: u32",
                "motion_cruise_mean_abs_units: u32",
                "motion_overshoot_units: u32",
                "motion_cruise_rms_units: u32",
                "dither_margin_milli: i32",
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

    fn extract_match_arm<'a>(text: &'a str, variant: &str) -> &'a str {
        let start = text
            .find(variant)
            .unwrap_or_else(|| panic!("missing match arm for {variant}"));
        let open = text[start..]
            .find('{')
            .map(|offset| start + offset)
            .unwrap_or_else(|| panic!("missing match arm block for {variant}"));
        let close = find_matching_close_brace(text, open);
        &text[start..=close]
    }

    #[test]
    fn locked_arm_hands_off_instead_of_draining_inline() {
        let locked_open_marker = TRIGGER_DOMAIN
            .find("QueueAccess::LockedInPlace => {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess::LockedInPlace");
        let locked_open_brace = locked_open_marker + "QueueAccess::LockedInPlace => ".len();
        assert_eq!(
            &TRIGGER_DOMAIN[locked_open_brace..locked_open_brace + 1],
            "{"
        );
        let locked_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, locked_open_brace);
        let locked_arm = &TRIGGER_DOMAIN[locked_open_brace..=locked_close_brace];

        assert!(
            locked_arm.contains("detach_consumer("),
            "the LockedInPlace arm must detach the consumer, not borrow it in place"
        );
        assert!(
            !locked_arm.contains(".dequeue()"),
            "the LockedInPlace arm must never drain the queue itself -- that is exactly \
             the critical-section-duration problem this ticket fixes. Draining belongs \
             only in drain_all_populated_slots, run by trigger_drain_task with interrupts \
             enabled."
        );
        assert!(
            locked_arm.contains("store_pending_drain("),
            "the LockedInPlace arm must hand the detached consumer to trigger_drain_task"
        );
        assert!(
            !locked_arm.contains("record_physical_stop("),
            "diagnostic recording for a masked stop that was successfully handed off must \
             happen in drain_all_populated_slots (after the drain, with real segment/step \
             counts), not inside stop_physical_stepper_inner's LockedInPlace arm itself"
        );
    }

    #[test]
    fn locked_stop_still_resets_gpio_state_unconditionally() {
        // Regression guard for a defect caught in this plan's own cross-agent
        // review: an early `return` *inside* the LockedInPlace arm would skip
        // direction/STEP GPIO reset, corrupting the next segment's pulse
        // parity. This must check from the start of the match (so it covers
        // the match arms themselves, where the original defect actually
        // lived) through the end of the function -- an earlier draft of this
        // guard checked only after the match's closing brace, which does not
        // cover a `return` inside an arm and would not have caught the
        // defect it was written to guard against. A legitimate early
        // `return` exists before the match (the
        // `!PHYSICAL_STEPPER_INDICES.contains(...)` bounds check at the top
        // of the function) and must stay outside this checked region.
        let fn_open_marker = TRIGGER_DOMAIN
            .find("fn stop_physical_stepper_inner(")
            .expect("trigger_domain.rs must define stop_physical_stepper_inner");
        let fn_open_brace = TRIGGER_DOMAIN[fn_open_marker..]
            .find('{')
            .map(|offset| fn_open_marker + offset)
            .expect("stop_physical_stepper_inner must have a body");
        let fn_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, fn_open_brace);
        let fn_body = &TRIGGER_DOMAIN[fn_open_brace..=fn_close_brace];

        let match_pos = fn_body
            .find("match access {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess");
        let from_match_onward = &fn_body[match_pos..];

        // Word-boundary match, not a plain substring search: the region
        // legitimately contains the comment word "returned" (describing
        // detach_consumer's Option), which a bare `contains("return")`
        // would misfire on even though it is not the `return` keyword.
        let has_return_keyword = from_match_onward.match_indices("return").any(|(pos, _)| {
            let before_ok = from_match_onward[..pos]
                .chars()
                .next_back()
                .is_none_or(|c| !c.is_alphanumeric() && c != '_');
            let after_ok = from_match_onward["return".len() + pos..]
                .chars()
                .next()
                .is_none_or(|c| !c.is_alphanumeric() && c != '_');
            before_ok && after_ok
        });
        assert!(
            !has_return_keyword,
            "no `return` is allowed anywhere from the start of the QueueAccess match \
             through the end of stop_physical_stepper_inner -- this includes inside \
             either match arm, not just after the match closes -- because \
             direction-GPIO and STEP-GPIO reset must run unconditionally for every \
             stop, and an early return from an arm would skip them"
        );

        let reset_idle_pos = TRIGGER_DOMAIN
            .find("stepper_pins::reset_idle(")
            .expect("stop_physical_stepper_inner must reset STEP/DIR GPIOs to idle");
        let locked_open_marker = TRIGGER_DOMAIN
            .find("QueueAccess::LockedInPlace => {")
            .expect("stop_physical_stepper_inner must branch on QueueAccess::LockedInPlace");
        let locked_open_brace = locked_open_marker + "QueueAccess::LockedInPlace => ".len();
        let locked_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, locked_open_brace);
        assert!(
            locked_close_brace < reset_idle_pos,
            "the GPIO idle reset must run after the LockedInPlace arm's actual \
             closing brace, not nested inside it"
        );
    }

    #[test]
    fn drain_all_populated_slots_scans_every_channel_and_records_after_draining() {
        let fn_open_marker = TRIGGER_DOMAIN
            .find("pub(crate) fn drain_all_populated_slots()")
            .expect("trigger_domain.rs must define drain_all_populated_slots");
        let fn_open_brace = TRIGGER_DOMAIN[fn_open_marker..]
            .find('{')
            .map(|offset| fn_open_marker + offset)
            .expect("drain_all_populated_slots must have a body");
        let fn_close_brace = find_matching_close_brace(TRIGGER_DOMAIN, fn_open_brace);
        let fn_body = &TRIGGER_DOMAIN[fn_open_brace..=fn_close_brace];

        assert!(
            fn_body.contains("STEPPER_CHANNEL_COUNT"),
            "drain_all_populated_slots must scan every physical stepper channel, not a \
             hardcoded single index -- Ouroboros has two independently detachable channels"
        );
        // A loop that iterates `STEPPER_CHANNEL_COUNT` times but calls
        // take_pending_drain/attach_consumer with a hardcoded index (e.g.
        // always 0) would still pass the assertion above. Requiring the
        // loop's own index variable to appear in both calls closes that
        // gap -- this pins the exact identifier the reference
        // implementation uses (`idx`); if a future implementation
        // legitimately renames it, update this string alongside the code,
        // not the other way around.
        assert!(
            fn_body.contains("take_pending_drain(idx)"),
            "each channel's take must use the loop's own index, not a hardcoded channel -- \
             otherwise every iteration would drain (at most) the same single slot"
        );
        assert!(
            fn_body.contains("attach_consumer(idx as u8"),
            "each channel's reattach must use the loop's own index, not a hardcoded channel -- \
             otherwise a drained channel other than the hardcoded one would never be reattached"
        );
        let dequeue_pos = fn_body
            .find(".dequeue()")
            .expect("drain_all_populated_slots must actually drain the detached consumer");
        let record_pos = fn_body
            .find("record_physical_stop(")
            .expect("drain_all_populated_slots must record the stop after draining it");
        let attach_pos = fn_body
            .find("attach_consumer(")
            .expect("drain_all_populated_slots must reattach the consumer after draining");

        assert!(
            dequeue_pos < record_pos,
            "must record the drained segment/step counts after actually draining, \
             not before (the counts would be wrong -- always zero)"
        );
        assert!(
            record_pos < attach_pos,
            "must record before reattaching, matching the existing Unmasked-arm ordering \
             (detach -> drain -> record -> reattach)"
        );
        assert!(
            !fn_body.contains("interrupt::free"),
            "drain_all_populated_slots must never hold a critical section across the \
             drain loop -- masking belongs only inside take_pending_drain/attach_consumer, \
             per-channel, never across the dequeue loop"
        );
    }

    #[test]
    fn step_dispatch_hold_is_gated_on_the_homing_start_clock() {
        assert_eq!(
            MAIN.matches("step_held_for_homing(stepper.next_step_clock())")
                .count(),
            1,
            "the only homing hold sits where a due step is emitted and keys off \
             that step's scheduled clock"
        );
        assert!(
            !MAIN.contains("HOMING_STATE.load(Relaxed) == 1"),
            "a hold keyed on HOMING_STATE alone stalls the previous move's \
             in-flight steps past their deadline: Klipper sends \
             stepper_stop_on_trigger as soon as it plans the home, often \
             seconds before the homing move starts"
        );
    }

    #[test]
    fn deferred_homing_preparation_keeps_state_without_containment() {
        let maintenance = MAIN
            .split("UnitOutcome::Maintenance { homing } => {")
            .nth(1)
            .expect("maintenance outcome arm");
        let deferred_arm = extract_match_arm(maintenance, "HomingMaintenanceResult::Deferred");
        assert!(deferred_arm.contains("group_all_ready = false"));
        assert!(!deferred_arm.contains("TMC_DISABLE_SIGNAL.signal(())"));
        assert!(!deferred_arm.contains("clear_homing_runtime()"));
        assert!(!deferred_arm.contains("homing_clamp_active.store(false"));
        assert!(
            MAIN.contains("HomingPreparationProgress::Deferred"),
            "a deferred group must release the homing-start latch so P1 returns \
             to its normal wait instead of spinning on urgent maintenance"
        );
        assert!(
            MAIN.contains(
                "if snapshot == 1 && HomingWakePolicy::<TmcRequest>::should_clear_latch(progress)"
            ) && !MAIN.contains("&& HOMING_STATE.load(Relaxed) != 1"),
            "the finalize latch release must not require HOMING_STATE to have left 1: \
             a deferred group keeps state 1 on purpose, and gating on the state \
             would keep the latch set and starve requests until the window opens"
        );
    }

    #[test]
    fn specific_tmc_fault_classification_is_reported_before_stepper_too_far_in_past() {
        let adc_pos = MAIN
            .find("EmergencyDisableFault::AdcCurrentClipped =>")
            .expect("ADC-clip fault arm");
        let status_read_pos = MAIN
            .find("EmergencyDisableFault::StatusReadFailed =>")
            .expect("status-read-failed fault arm");
        let deadline_pos = MAIN
            .find("ShutdownReason::StepperTooFarInPast")
            .expect("StepperTooFarInPast shutdown call");
        assert!(
            adc_pos < deadline_pos && status_read_pos < deadline_pos,
            "a specific TMC fault classification (ADC current clipped, STATUS_FLAGS read \
             failed) must be checked and reported before falling back to \
             StepperTooFarInPast -- a stepper naturally misses its own deadline the \
             instant its motor is emergency-disabled, so an unconditional preference for \
             the deadline-miss label masks the real electrical fault that caused it"
        );

        let generic_pos = MAIN
            .find("EmergencyDisableFault::Generic =>")
            .expect("Generic fault arm");
        assert!(
            generic_pos < deadline_pos,
            "StepperTooFarInPast must be reported only from within the Generic-fault \
             arm (no specific TMC fault was observed), not as an unconditional \
             override ahead of fault classification"
        );
    }

    #[test]
    fn homing_maintenance_outcomes_preserve_retry_and_fail_closed_containment() {
        let maintenance = MAIN
            .split("UnitOutcome::Maintenance { homing } => {")
            .nth(1)
            .expect("maintenance outcome arm")
            .split("UnitOutcome::Audit(result) => {")
            .next()
            .expect("maintenance outcome body");

        assert!(maintenance.contains("match homing {"));
        assert_eq!(maintenance.matches("HomingMaintenanceResult::").count(), 4);

        let ready_arm = extract_match_arm(maintenance, "HomingMaintenanceResult::ReadyToAdvance");
        assert!(!ready_arm.contains("group_all_ready = false"));

        let retry_arm =
            extract_match_arm(maintenance, "HomingMaintenanceResult::RetryCurrentState");
        assert!(retry_arm.contains("group_all_ready = false"));
        assert!(!retry_arm.contains("TMC_DISABLE_SIGNAL.signal(())"));
        assert!(!retry_arm.contains("clear_homing_runtime()"));

        let fail_closed_arm = extract_match_arm(maintenance, "HomingMaintenanceResult::FailClosed");
        assert!(fail_closed_arm.contains("group_all_ready = false"));
        assert!(
            fail_closed_arm.contains("tmc_state(ch).homing_clamp_active.store(false, Relaxed)")
        );
        assert!(fail_closed_arm.contains("tmc_state(ch).stall_latched.store(false, Relaxed)"));
        assert!(fail_closed_arm.contains("drivers[idx].runtime.clear_homing_runtime()"));
        assert!(fail_closed_arm.contains("TMC_DISABLE_SIGNAL.signal(())"));

        assert_eq!(maintenance.matches("group_all_ready = false").count(), 3);
        assert_eq!(
            maintenance
                .matches("homing_clamp_active.store(false")
                .count(),
            1
        );
        assert_eq!(maintenance.matches("stall_latched.store(false").count(), 1);
        assert_eq!(maintenance.matches("clear_homing_runtime()").count(), 1);
        assert_eq!(
            maintenance.matches("TMC_DISABLE_SIGNAL.signal(())").count(),
            1
        );
    }

    #[test]
    fn tmc_control_event_wraps_requests_without_changing_queue_capacity() {
        assert!(TMC_STATE.contains("pub(crate) const TMC_CHANNEL_SIZE: usize = 8;"));
        assert!(TMC_STATE.contains("enum TmcControlEvent"));
        assert!(TMC_STATE.contains("Request(TmcRequest)"));
        assert!(TMC_STATE.contains("HomingStart"));
        assert!(TMC_STATE.contains("Sender<'static, TmcControlEvent, TMC_CHANNEL_SIZE>"));

        let enqueue = extract_fn(TMC_HELPERS, "try_send_tmc_request");
        assert!(enqueue.contains("sender.try_send(TmcControlEvent::Request(req)).is_ok()"));
        assert_eq!(enqueue.matches("try_send(").count(), 1);
    }

    #[test]
    fn tmc_control_event_homing_notifications_latch_before_coalescing() {
        let notify = extract_fn(FIRMWARE_CTX, "notify_homing_start");
        let latch = notify
            .find("HOMING_START_PENDING.swap(true")
            .expect("homing-start notification must set the authoritative latch");
        let sender = notify
            .find("TMC_CONTROL_SENDER")
            .expect("homing-start notification must inspect the board event sender");

        assert!(latch < sender, "the latch must be set before sender access");
        assert!(notify.contains("if HOMING_START_PENDING.swap(true, AcqRel)"));
        assert!(
            notify.contains("return Ok(())"),
            "repeated notifications must coalesce"
        );
        assert!(notify.contains("let Some(sender)"));
        assert!(notify.contains("else {\n                return Err(())"));
        assert!(notify.contains("TmcControlEvent::HomingStart"));

        let result_match = notify
            .find("match sender.try_send(TmcControlEvent::HomingStart)")
            .expect("homing-start enqueue result match");
        let result_open = notify[result_match..]
            .find('{')
            .map(|offset| result_match + offset)
            .expect("homing-start enqueue result body");
        let result_close = find_matching_close_brace(notify, result_open);
        let normalized = notify[result_open + 1..result_close]
            .split_whitespace()
            .collect::<std::string::String>();
        assert_eq!(
            normalized,
            "Ok(())|Err(TrySendError::Full(_))|Err(TrySendError::NoReceiver(_))=>Ok(()),",
            "every initialized-sender result must acknowledge the authoritative latch"
        );
    }

    #[test]
    fn tmc_control_event_receiver_dispatches_every_event_variant() {
        assert!(MAIN.contains("Receiver<'static, TmcControlEvent, TMC_CHANNEL_SIZE>"));
        assert!(MAIN.contains("Channel<TmcControlEvent, TMC_CHANNEL_SIZE>"));

        let control = extract_fn(MAIN, "tmc_control");
        assert!(control.contains("match event {"));
        assert_eq!(control.matches("TmcControlEvent::").count(), 2);
        assert!(control.contains("TmcControlEvent::Request(req)"));
        assert!(control.contains("TmcControlEvent::HomingStart"));
        assert_eq!(
            control
                .matches("HOMING_START_PENDING.store(false, Release)")
                .count(),
            3,
            "only state-1 advance, fail-closed containment, and queue flush release the latch"
        );
    }

    #[test]
    fn homing_wake_keeps_the_control_wait_to_two_futures() {
        let control = extract_fn(MAIN, "tmc_control");

        assert_eq!(control.matches("select(").count(), 1);
        assert!(control.contains("select(rx.recv(), control_sleep)"));
        assert!(!control.contains("select3"));
        assert!(!control.contains("select(rx.recv(), control_sleep,"));
        assert!(control.contains("HomingWait::MaintenanceOnly"));
        assert!(control.contains("scheduler.maintenance_due_us()"));
        assert!(control.contains("HomingWait::ProcessRetained"));
        assert!(HOMING_WAKE.contains("retained_request: Option<T>"));

        let pre_wait_latch = control
            .find("HOMING_START_PENDING.load(Acquire)")
            .expect("authoritative latch check before the wait");
        let select = control.find("select(rx.recv()").expect("two-future select");
        let post_receive_latch = control[select..]
            .find("HOMING_START_PENDING.load(Acquire)")
            .expect("authoritative latch check after receive");
        assert!(pre_wait_latch < select);
        assert!(post_receive_latch > 0);

        let maintenance_only = control
            .find("HomingWait::MaintenanceOnly =>")
            .expect("maintenance-only wait arm");
        let receiver_wait = control[maintenance_only..]
            .find("HomingWait::ReceiverOrDeadline =>")
            .map(|offset| maintenance_only + offset)
            .expect("receiver wait arm");
        assert!(!control[maintenance_only..receiver_wait].contains("rx.recv()"));
    }

    #[test]
    fn homing_wake_shutdown_and_queue_flush_release_owned_state() {
        let shutdown = extract_fn(FIRMWARE_CTX, "enter_shutdown");
        let homing_reset = shutdown
            .find("HOMING_STATE.store(0, Relaxed)")
            .expect("shutdown homing-state reset");
        let latch_release = shutdown
            .find("HOMING_START_PENDING.store(false, Release)")
            .expect("shutdown homing-start latch release");
        assert!(homing_reset < latch_release);
        assert!(
            shutdown.contains("HOMING_REENTER_PENDING.store(false, Relaxed)"),
            "shutdown must drop a queued homing re-entry or the next restore re-arms homing"
        );

        let control = extract_fn(MAIN, "tmc_control");
        let flush_epoch = control
            .find("let queue_flush = critical_section::with")
            .expect("atomic queue-flush epoch boundary");
        let flush_claim = control
            .find("TMC_QUEUE_FLUSH.swap(false, AcqRel)")
            .expect("atomic queue-flush epoch claim");
        let flush_latch_release = control[flush_claim..]
            .find("HOMING_START_PENDING.store(false, Release)")
            .map(|offset| flush_claim + offset)
            .expect("queue-flush latch release");
        let retained_reset = control[flush_latch_release..]
            .find("homing_wake.reset_for_queue_flush()")
            .map(|offset| flush_latch_release + offset)
            .expect("queue-flush retained-slot reset");
        let flush_body = control
            .find("if queue_flush {")
            .expect("claimed queue-flush body");
        assert!(
            flush_epoch < flush_claim
                && flush_claim < flush_latch_release
                && flush_latch_release < flush_body
                && flush_body < retained_reset
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

    #[test]
    fn diagnostic_dispatch_uses_raw_lease_expired_not_the_classifier() {
        assert!(
            TMC_DIAGNOSTICS.contains("tmc_control::containment::lease_expired("),
            "diagnostic dispatch must wire the host-liveness check to the raw \
             lease_expired function"
        );
        assert!(
            !TMC_DIAGNOSTICS.contains("classify_lease_check")
                && !TMC_DIAGNOSTICS.contains("LeaseCheckDisposition"),
            "diagnostic dispatch must not use classify_lease_check -- its \
             shutdown-active suppression does not apply to a diagnostic \
             runner's one-shot poll"
        );
    }

    #[test]
    fn safety_bootstrap_runs_before_the_first_status_pin_read_and_before_drive() {
        let tmc_control_fn = extract_fn(MAIN, "tmc_control");
        let bootstrap_pos = tmc_control_fn
            .find("bootstrap_tmc_safety(")
            .expect("eager safety-bootstrap call must exist in tmc_control()");
        let status_read_pos = tmc_control_fn
            .find("status_levels(status_pin0)")
            .expect("the control loop's STATUS-pin read must exist in tmc_control()");
        let drive_call_pos = tmc_control_fn
            .find(".drive(unit, &mut backend, ctx_unit)")
            .or_else(|| tmc_control_fn.find("drivers[idx].drive("))
            .expect("a per-channel drive() call must exist in tmc_control()");
        let window_end = (bootstrap_pos + 200).min(tmc_control_fn.len());

        assert!(
            bootstrap_pos < status_read_pos,
            "safety bootstrap must run before the control loop's first STATUS-pin \
             read -- a stale STATUS_MASK from a prior session can otherwise trip \
             the emergency-disable branch before bootstrap ever corrects it, \
             permanently skipping the correction on every subsequent boot"
        );
        assert!(
            bootstrap_pos < drive_call_pos,
            "safety bootstrap must run before any per-channel drive() call"
        );
        assert!(
            tmc_control_fn[bootstrap_pos..window_end].contains(", ch,"),
            "the eager bootstrap call must pass the per_channel! loop variable \
             `ch`, not a hardcoded channel number -- otherwise only one of the \
             two channels is protected"
        );
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
