//! Klipper reply emitters for commission/calibrate/selftest/tune outcomes.
//!
//! The semantic `ReplyPayload` mapping lives in `foci_firmware`; this module
//! keeps only the board-generated ankyra sender calls.

struct BoardCommissionReplySink;

impl foci_firmware::commission_dispatch::CommissionReplySink for BoardCommissionReplySink {
    foci_firmware::__commission_reply_manifest!(
        foci_firmware_macros::commission_reply_forwarding_impls
    );

    fn foci_commission_detail(
        &mut self,
        oid: u8,
        event: foci_firmware::selftest::CommissionDiagnostic,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCommissionDetail,
            oid: u8 = oid,
            phase: u8 = event.phase,
            code: u8 = event.code,
            status: u8 = event.status,
            value0: u32 = event.value0,
            value1: u32 = event.value1,
            value2: u32 = event.value2,
        );
    }

    fn foci_commission_phase(&mut self, oid: u8, phase: u8, status: u8) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCommissionPhase,
            oid: u8 = oid,
            phase: u8 = phase,
            status: u8 = status,
        );
    }

    fn foci_commission_result(
        &mut self,
        oid: u8,
        status: u8,
        result: &foci_firmware::commissioning::types::CommissionResult,
    ) {
        let inner = &result.inner;
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCommissionResult,
            oid: u8 = oid,
            status: u8 = status,
            r_count_milli: u32 = inner.r_count_milli,
            l_count_micro: u32 = inner.l_count_micro,
            flux_p: u16 = inner.flux_p,
            flux_i: u16 = inner.flux_i,
            torque_p: u16 = inner.torque_p,
            torque_i: u16 = inner.torque_i,
            lambda_us: u32 = inner.lambda_us,
            theta_e_us: u32 = inner.theta_e_us,
            theta_source: u8 = inner.theta_source,
            ringing_count: u8 = inner.ringing_count,
            bandwidth_hz: u32 = inner.bandwidth_hz,
            fallback_velocity_p: u16 = result.fallback_velocity_p,
            fallback_velocity_i: u16 = result.fallback_velocity_i,
            fallback_position_p: u16 = result.fallback_position_p,
            fallback_position_i: u16 = result.fallback_position_i,
            fallback_velocity_limit: u32 = result.fallback_velocity_limit,
            tau_e_us: u32 = inner.tau_e_us,
            inner_warning_flags: u8 = inner.inner_warning_flags,
            timing_summary: u32 = inner.timing_summary,
        );
    }

    fn foci_resistance_run(&mut self, report: foci_firmware::tmc_diagnostics::ResistanceRunReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociResistanceRun,
            oid: u8 = report.oid,
            status: u8 = report.status,
            selected_r_count_slope_milli: i32 = report.selected_r_count_slope_milli,
            gain_path_count_slope_milli: i32 = report.gain_path_count_slope_milli,
            warning_flags: u32 = report.warning_flags,
            status_flags_or: u32 = report.status_flags_or,
            peak_abs_current_count: u16 = report.peak_abs_current_count,
            max_abs_steady_mean_current_count: u16 = report.max_abs_steady_mean_current_count,
            current_ceiling_count: u16 = report.current_ceiling_count,
            power_stage_tripped: u8 = u8::from(report.power_stage_tripped),
            pwm_maxcnt_readback: u16 = report.pwm_maxcnt_readback,
            bbm_readback: u16 = report.bbm_readback,
            dsadc_mdec_readback: u32 = report.dsadc_mdec_readback,
            pwm_sv_chop_readback: u32 = report.pwm_sv_chop_readback,
        );
    }

    fn foci_resistance_axis(
        &mut self,
        report: foci_firmware::tmc_diagnostics::ResistanceAxisReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociResistanceAxis,
            oid: u8 = report.oid,
            electrical_axis: u8 = report.electrical_axis,
            phi_e_ext: u16 = report.phi_e_ext,
            r_count_slope_milli: i32 = report.r_count_slope_milli,
            intercept_count: i32 = report.intercept_count,
            rmse_permille: u16 = report.rmse_permille,
            selected_mask: u16 = report.selected_mask,
            excluded_point_mask: u16 = report.excluded_point_mask,
            selected_count: u8 = report.selected_count,
            signed_count_slope_milli: i32 = report.signed_count_slope_milli,
            signed_asymmetry_permille: u16 = report.signed_asymmetry_permille,
            drift_permille: u16 = report.drift_permille,
            warning_flags: u32 = report.warning_flags,
        );
    }

    fn foci_inductance_run(&mut self, report: foci_firmware::tmc_diagnostics::InductanceRunReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociInductanceRun,
            oid: u8 = report.oid,
            source: u8 = report.source,
            status: u8 = report.status,
            warning_flags: u16 = report.warning_flags,
            ud_count: i16 = report.ud_count,
            realized_frequency_millihz: u32 = report.realized_frequency_millihz,
            elapsed_us: u32 = report.elapsed_us,
            openloop_phi_delta_counts: u32 = report.openloop_phi_delta_counts,
            sample_count: u16 = report.sample_count,
            encoder_delta_counts: i32 = report.encoder_delta_counts,
            status_flags_or: u32 = report.status_flags_or,
        );
    }

    fn foci_inductance_frame(
        &mut self,
        report: foci_firmware::tmc_diagnostics::InductanceFrameReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociInductanceFrame,
            oid: u8 = report.oid,
            id_mean_milli_count: i32 = report.id_mean_milli_count,
            iq_mean_milli_count: i32 = report.iq_mean_milli_count,
            id_rms_milli_count: u32 = report.id_rms_milli_count,
            iq_rms_milli_count: u32 = report.iq_rms_milli_count,
            drift_permille: u16 = report.drift_permille,
            zero_id_mean_milli_count: i32 = report.zero_id_mean_milli_count,
            zero_iq_mean_milli_count: i32 = report.zero_iq_mean_milli_count,
        );
    }

    fn foci_inductance_estimate(
        &mut self,
        report: foci_firmware::tmc_diagnostics::InductanceEstimateReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociInductanceEstimate,
            oid: u8 = report.oid,
            x_average_count_ratio_milli: u32 = report.x_average_count_ratio_milli,
            x_d_count_ratio_milli: u32 = report.x_d_count_ratio_milli,
            x_q_count_ratio_milli: u32 = report.x_q_count_ratio_milli,
            saliency_status: u8 = report.saliency_status,
            saliency_permille: u16 = report.saliency_permille,
            x_mag_nominal_count_ratio_milli: u32 = report.x_mag_nominal_count_ratio_milli,
            x_mag_shift_minus_permille: u16 = report.x_mag_shift_minus_permille,
            x_mag_shift_plus_permille: u16 = report.x_mag_shift_plus_permille,
            x_mag_vs_quad_permille: u16 = report.x_mag_vs_quad_permille,
        );
    }

    fn foci_current_loop_hold(
        &mut self,
        report: foci_firmware::tmc_diagnostics::CurrentLoopHoldReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentLoopHold,
            oid: u8 = report.oid,
            hold_status: u8 = report.hold_status,
            warning_flags: u16 = report.warning_flags,
            elapsed_us: u32 = report.elapsed_us,
            requested_sample_period_us: u16 = report.requested_sample_period_us,
            sample_count: u8 = report.sample_count,
            position_span_count: u16 = report.position_span_count,
            position_drift_count: u16 = report.position_drift_count,
            torque_mean_count: i16 = report.torque_mean_count,
            torque_rms_count: u16 = report.torque_rms_count,
            torque_peak_to_peak_count: u16 = report.torque_peak_to_peak_count,
            torque_crossing_count: u8 = report.torque_crossing_count,
            flux_mean_count: i16 = report.flux_mean_count,
            flux_rms_count: u16 = report.flux_rms_count,
            flux_peak_to_peak_count: u16 = report.flux_peak_to_peak_count,
            flux_crossing_count: u8 = report.flux_crossing_count,
            status_flags_or: u32 = report.status_flags_or,
            actionable_status_count: u8 = report.actionable_status_count,
        );
    }

    fn foci_closed_loop_activation(
        &mut self,
        report: foci_firmware::tmc_diagnostics::ClosedLoopActivationReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociClosedLoopActivation,
            oid: u8 = report.oid,
            entry_status: u8 = report.entry_status,
            position_1: i32 = report.position_1,
            position_2: i32 = report.position_2,
            drift_count: u16 = report.drift_count,
            threshold_count: u16 = report.threshold_count,
            runaway: u8 = report.runaway,
        );
    }

    fn foci_current_loop_filters(
        &mut self,
        report: foci_firmware::tmc_diagnostics::CurrentLoopFiltersReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentLoopFilters,
            oid: u8 = report.oid,
            velocity_filter_hz: u16 = report.velocity_filter_hz,
            torque_filter_hz: u16 = report.torque_filter_hz,
            position_filter_hz: u16 = report.position_filter_hz,
            flux_filter_hz: u16 = report.flux_filter_hz,
        );
    }

    fn foci_current_loop_run(
        &mut self,
        report: foci_firmware::tmc_diagnostics::CurrentLoopRunReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentLoopRun,
            oid: u8 = report.oid,
            status: u8 = report.status,
            gains_source: u8 = report.gains_source,
            candidate_gains_source: u8 = report.candidate_gains_source,
            axis_split_source: u8 = report.axis_split_source,
            candidate_axis_split_source: u8 = report.candidate_axis_split_source,
            gains_tier: u8 = report.gains_tier,
            candidate_gains_tier: u8 = report.candidate_gains_tier,
            measured_axis_split_permille: u16 = report.measured_axis_split_permille,
            candidate_measured_axis_split_permille: u16 =
                report.candidate_measured_axis_split_permille,
            applied_axis_split_permille: u16 = report.applied_axis_split_permille,
            candidate_applied_axis_split_permille: u16 =
                report.candidate_applied_axis_split_permille,
            axis_split_clamped: u8 = report.axis_split_clamped,
            candidate_axis_split_clamped: u8 = report.candidate_axis_split_clamped,
            candidate_flux_p: u16 = report.candidate_flux_p,
            candidate_flux_i: u16 = report.candidate_flux_i,
            candidate_torque_p: u16 = report.candidate_torque_p,
            candidate_torque_i: u16 = report.candidate_torque_i,
            candidate_attempt: u8 = report.candidate_attempt,
            current_validation_axes: u8 = report.current_validation_axes,
            flux_validation_sample_count: u8 = report.flux_validation_sample_count,
            torque_validation_sample_count: u8 = report.torque_validation_sample_count,
            retry_budget_exhausted: u8 = report.retry_budget_exhausted,
            failure_reason: u8 = report.failure_reason,
        );
    }

    fn foci_current_validation_axis(
        &mut self,
        report: foci_firmware::tmc_diagnostics::CurrentValidationAxisReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentValidationAxis,
            oid: u8 = report.oid,
            axis: u8 = report.axis,
            sample_index: u8 = report.sample_index,
            status: u8 = report.status,
            attempt: u8 = report.attempt,
            target: i16 = report.target,
            sample_delay_ms: u16 = report.sample_delay_ms,
            positive_response_permille: u16 = report.positive_response_permille,
            negative_response_permille: u16 = report.negative_response_permille,
            cross_axis_permille: u16 = report.cross_axis_permille,
            cross_axis_peak_permille: u16 = report.cross_axis_peak_permille,
            voltage_output_permille: u16 = report.voltage_output_permille,
            encoder_delta_counts: u16 = report.encoder_delta_counts,
            positive_encoder_delta_counts: i32 = report.positive_encoder_delta_counts,
            negative_encoder_delta_counts: i32 = report.negative_encoder_delta_counts,
            status_flags_or: u32 = report.status_flags_or,
        );
    }

    fn foci_tune_result(
        &mut self,
        oid: u8,
        status: u8,
        result: &foci_firmware::commissioning::types::OuterResult,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociTuneResult,
            oid: u8 = oid,
            status: u8 = status,
            velocity_p: u16 = result.velocity_p,
            velocity_i: u16 = result.velocity_i,
            velocity_limit: u32 = result.velocity_limit,
            position_p: u16 = result.position_p,
            position_i: u16 = result.position_i,
            velocity_filter_hz: u16 = result.velocity_filter_hz,
            position_filter_hz: u16 = result.position_filter_hz,
            flux_filter_hz: u16 = result.flux_filter_hz,
            torque_filter_hz: u16 = result.torque_filter_hz,
            warning_code: u8 = result.warning_code,
            probed_velocity_mrev_s: u32 = result.provenance.probed_velocity_mrev_s,
            d_eq_q: i32 = result.provenance.d_eq_q,
            confidence_q: u16 = result.provenance.confidence_q,
            band_lower_percent: u8 = result.provenance.band_lower_percent,
            band_upper_percent: u8 = result.provenance.band_upper_percent,
            band_position_q: u16 = result.provenance.band_position_q,
            nominal_bandwidth_hz: u32 = result.position_tune.nominal_bandwidth_hz,
            inner_bandwidth_rad_s: u32 = result.position_tune.inner_bandwidth_rad_s,
            dither_margin_milli: i32 = result.position_tune.dither_margin_milli,
            settling_time_us: u32 = result.position_tune.settling_time_us,
            overshoot_counts: u32 = result.position_tune.overshoot_counts,
            final_position_error_counts: i32 = result.position_tune.final_position_error_counts,
            oscillation_detected: bool = result.position_tune.oscillation_detected,
            position_tune_outcome_code: u8 = result.position_tune_outcome_code,
        );
    }

    fn foci_outer_safety_fault(
        &mut self,
        oid: u8,
        fault: foci_firmware::commissioning::types::OuterSafetyFault,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociOuterSafetyFault,
            oid: u8 = oid,
            reason: u8 = fault.reason,
            max_travel_mrev: u16 = fault.max_travel_mrev,
            max_velocity_mrev_s: u16 = fault.max_velocity_mrev_s,
            max_duration_ms: u16 = fault.max_duration_ms,
            direction_mask: u8 = fault.direction_mask,
            delta_counts: i32 = fault.delta_counts,
            dt_us: u32 = fault.dt_us,
            velocity_counts_per_ms: i32 = fault.velocity_counts_per_ms,
            velocity_cap_counts_per_ms: i32 = fault.velocity_cap_counts_per_ms,
            position_counts: i32 = fault.position_counts,
            position_window_counts: i32 = fault.position_window_counts,
            elapsed_us: u32 = fault.elapsed_us,
            duration_cap_us: u32 = fault.duration_cap_us,
        );
    }

    fn foci_calibrate_result(
        &mut self,
        oid: u8,
        status: u8,
        result: &foci_firmware::commissioning::types::AdcCalResult,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCalibrateResult,
            oid: u8 = oid,
            status: u8 = status,
            adc_i0: u16 = result.adc_i0,
            adc_i1: u16 = result.adc_i1,
            encoder_count: u32 = result.encoder_count,
        );
    }

    fn foci_selftest_result(
        &mut self,
        oid: u8,
        stage: u8,
        value: foci_firmware::selftest::SelftestStageValue,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociSelftestResult,
            oid: u8 = oid,
            stage: u8 = stage,
            status: u8 = value.status,
            value: u32 = value.value,
        );
    }

    fn foci_selftest_done(&mut self, oid: u8, status: u8) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociSelftestDone,
            oid: u8 = oid,
            status: u8 = status,
        );
    }
}

/// Emit terminal commissioning-family replies.
pub(crate) fn emit_terminal_reply(payload: foci_firmware::commission_dispatch::ReplyPayload) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::emit_terminal_reply(&mut sink, payload);
}

/// Emit one nonterminal velocity-integral evidence group.
pub(crate) fn emit_velocity_integral_event(
    oid: u8,
    event: foci_firmware::commissioning::types::VelocityIntegralEvent,
) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::replies::emit_velocity_integral_event(
        &mut sink, oid, event,
    );
}

/// Emit one breakaway-campaign evidence group.
pub(crate) fn emit_breakaway_campaign_event(
    oid: u8,
    event: foci_firmware::commissioning::types::BreakawayCampaignEvent,
) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::replies::emit_breakaway_campaign_event(
        &mut sink, oid, event,
    );
}

/// Emit the exact velocity-integral plan an accepted breakaway campaign handed off to.
///
/// The host assembler refuses every velocity-integral observation until it has seen a
/// complete plan, and a breakaway hand-off reaches the integral phase on the
/// scheduled-tick path, which carries the plan as a `CommissionEvent` rather
/// than through a dispatch reply.
pub(crate) fn emit_velocity_integral_plan(
    oid: u8,
    run_sequence: u32,
    plan: foci_firmware::commissioning::outer::velocity::integral_plan::VelocityIntegralPlan,
) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::replies::emit_velocity_integral_plan(
        &mut sink,
        oid,
        run_sequence,
        plan,
    );
}

/// Emit a `FociCommissionPhase` reply for a phase transition.
pub(crate) fn emit_phase_reply(oid: u8, phase_id: foci_firmware::autotune::CommissionPhase) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::emit_phase_reply(&mut sink, oid, phase_id);
}
