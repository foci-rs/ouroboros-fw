//! Klipper reply emitters for commission/calibrate/selftest/tune outcomes.
//!
//! The semantic `ReplyPayload` mapping lives in `foci_firmware`; this module
//! keeps only the board-generated ankyra sender calls.

use ankyra::prelude::klipper_reply;

/// `foci_impedance_profile` reply - impedance profile constants for display.
#[klipper_reply]
pub struct FociImpedanceProfile {
    pub oid: u8,
    pub point_count: u8,
    pub sample_count: u16,
    pub sample_interval_us: u16,
    pub carrier_frequency_millihz: u32,
    pub samples_per_cycle: u16,
    pub cycles_per_angle: u16,
    pub max_ud: u16,
    pub hold_ud: u16,
    pub carrier_ud: u16,
    pub status_flags_fail_mask: u32,
    pub status_flags_warn_mask: u32,
    pub scale_metadata_validated: u8,
}

/// `foci_impedance_run` reply - impedance run status and aggregate fit result.
#[klipper_reply]
pub struct FociImpedanceRun {
    pub oid: u8,
    pub status: u8,
    pub selected_axis_max_count_per_ud_milli: u32,
    pub selected_axis_min_count_per_ud_milli: u32,
    pub repeatability_permille: u16,
    pub warning_flags: u32,
    pub status_flags_or: u32,
    pub readback_mismatch_addr: u8,
    pub readback_expected: u32,
    pub readback_actual: u32,
    pub filter_torque_enable_before: u8,
    pub filter_flux_enable_before: u8,
    pub filter_torque_enable_during: u8,
    pub filter_flux_enable_during: u8,
    pub duplicate_zero_encoder_drift_counts: u32,
}

/// `foci_impedance_fit` reply - per-injection impedance fit result.
#[klipper_reply]
pub struct FociImpedanceFit {
    pub oid: u8,
    pub point_index: u8,
    pub frequency_millihz: u32,
    pub ud_abs: u16,
    pub axis_max_count_per_ud_milli: u32,
    pub axis_min_count_per_ud_milli: u32,
    pub axis_angle_electrical_counts: u16,
    pub residual_permille: u16,
    pub coverage_permille: u16,
    pub scalar_2f_mismatch_permille: u16,
    pub magnitude_mean_count_per_ud_milli: u32,
    pub magnitude_2f_count_per_ud_milli: u32,
    pub magnitude_2f_residual_permille: u16,
    pub magnitude_2f_coverage_permille: u16,
    pub warning_flags: u32,
}

/// `foci_impedance_fit_v2` reply - supplemental fixed-angle impedance fit evidence.
#[klipper_reply]
pub struct FociImpedanceFitV2 {
    pub oid: u8,
    pub point_index: u8,
    pub mean_response_count_per_ud_milli: u32,
    pub saliency_permille: u16,
    pub saliency_determinable: u8,
    pub angle_coverage_permille: u16,
    pub axis_deviation_counts: u16,
    pub inductive_response_count_per_ud_milli: u32,
    pub in_phase_response_count_per_ud_milli: u32,
    pub secondary_adc_response_count_per_ud_milli: u32,
}

/// `foci_impedance_observation` reply - per-injection live chip observation.
#[klipper_reply]
pub struct FociImpedanceObservation {
    pub oid: u8,
    pub point_index: u8,
    pub frequency_millihz: u32,
    pub ud_abs: u16,
    pub status_flags_or: u32,
    pub openloop_phi_sample: u16,
    pub openloop_velocity_actual: i32,
    pub adc_iux_sample: i16,
    pub adc_iwy_sample: i16,
    pub foc_iux_sample: i16,
    pub foc_iwy_sample: i16,
    pub foc_uq_sample: i16,
    pub foc_ud_sample: i16,
    pub foc_uq_limited_sample: i16,
    pub foc_ud_limited_sample: i16,
    pub foc_iq_sample: i16,
    pub foc_id_sample: i16,
}

/// `foci_impedance_observation_v2` reply - supplemental fixed-angle observation evidence.
#[klipper_reply]
pub struct FociImpedanceObservationV2 {
    pub oid: u8,
    pub point_index: u8,
    pub angle_electrical_counts: u16,
    pub encoder_before: u32,
    pub encoder_after: u32,
    pub encoder_min: u32,
    pub encoder_max: u32,
    pub encoder_peak_counts: u16,
    pub angle_valid: u8,
    pub inductive_response_count_per_ud_milli: u32,
    pub in_phase_response_count_per_ud_milli: u32,
}

#[klipper_reply]
pub struct FociImpedanceBaseline {
    pub oid: u8,
    pub point_index: u8,
    pub frequency_millihz: u32,
    pub ud_abs: u16,
    pub dc_adc_iux_sample: i16,
    pub dc_adc_iwy_sample: i16,
    pub dc_foc_iux_sample: i16,
    pub dc_foc_iwy_sample: i16,
    pub dc_foc_iq_sample: i16,
    pub dc_foc_id_sample: i16,
}

struct BoardCommissionReplySink;

impl foci_firmware::commission_dispatch::CommissionReplySink for BoardCommissionReplySink {
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
            r_int: u32 = inner.r_int,
            l_int: u32 = inner.l_int,
            r_mohm: u32 = inner.r_mohm,
            l_uh: u32 = inner.l_uh,
            flux_p: u16 = inner.flux_p,
            flux_i: u16 = inner.flux_i,
            torque_p: u16 = inner.torque_p,
            torque_i: u16 = inner.torque_i,
            lambda_us: u32 = inner.lambda_us,
            theta_e_us: u32 = inner.theta_e_us,
            ringing_count: u8 = inner.ringing_count,
            bandwidth_hz: u32 = inner.bandwidth_hz,
            fallback_velocity_p: u16 = result.fallback_velocity_p,
            fallback_velocity_i: u16 = result.fallback_velocity_i,
            fallback_position_p: u16 = result.fallback_position_p,
            fallback_position_i: u16 = result.fallback_position_i,
            fallback_velocity_limit: u32 = result.fallback_velocity_limit,
            tau_e_us: u32 = inner.tau_e_us,
            tau_e_crosscheck_us: u32 = inner.tau_e_crosscheck_us,
            tau_residual_permille: u16 = inner.tau_residual_permille,
            inner_warning_flags: u8 = inner.inner_warning_flags,
        );
    }

    fn foci_resistance_run(&mut self, report: foci_firmware::tmc_diagnostics::ResistanceRunReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociResistanceRun,
            oid: u8 = report.oid,
            status: u8 = report.status,
            profile_version: u16 = report.profile_version,
            selected_r_count_slope_milli: i32 = report.selected_r_count_slope_milli,
            gain_path_count_slope_milli: i32 = report.gain_path_count_slope_milli,
            warning_flags: u32 = report.warning_flags,
            status_flags_or: u32 = report.status_flags_or,
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

    fn foci_encoder_alignment(
        &mut self,
        report: foci_firmware::tmc_diagnostics::EncoderAlignmentReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociEncoderAlignment,
            oid: u8 = report.oid,
            encoder_count: u32 = report.encoder_count,
            electrical_residual_counts: u16 = report.electrical_residual_counts,
            stability_counts: u16 = report.stability_counts,
            movement_counts: u16 = report.movement_counts,
            min_movement_counts: u16 = report.min_movement_counts,
            counts_per_electrical_rev: u16 = report.counts_per_electrical_rev,
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
            j_eff: u32 = result.j_eff,
            b_eff: u32 = result.b_eff,
            warning_code: u8 = result.warning_code,
            mech_torque_step: u16 = result.mech_torque_step,
            mech_fwd_accel: u32 = result.mech_fwd_accel,
            mech_rev_accel: u32 = result.mech_rev_accel,
            mech_fwd_peak_velocity: u32 = result.mech_fwd_peak_velocity,
            mech_rev_peak_velocity: u32 = result.mech_rev_peak_velocity,
            mech_fwd_travel: u32 = result.mech_fwd_travel,
            mech_rev_travel: u32 = result.mech_rev_travel,
            synth_velocity_p: u16 = result.synth_velocity_p,
            synth_velocity_i: u16 = result.synth_velocity_i,
            synth_position_p: u16 = result.synth_position_p,
            synth_position_i: u16 = result.synth_position_i,
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

/// Emit a `FociCommissionPhase` reply for a phase transition.
pub(crate) fn emit_phase_reply(oid: u8, phase_id: foci_firmware::autotune::PhaseId) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::emit_phase_reply(&mut sink, oid, phase_id);
}
