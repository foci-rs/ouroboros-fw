//! Klipper reply emitters for commission/calibrate/selftest/tune outcomes.
//!
//! The semantic `ReplyPayload` mapping lives in `foci_firmware`; this module
//! keeps only the board-generated ankyra sender calls.

struct BoardCommissionReplySink;

macro_rules! forward_commission_reply {
    ($method:ident, $reply:ident, $( $field:ident : $field_ty:ty ),+ $(,)?) => {
        fn $method(&mut self, report: foci_firmware::ankyra::replies::$reply) {
            ::ankyra::klipper_reply_from!(
                &mut crate::_ankyra_config::Sender,
                foci_firmware::ankyra::replies::$reply,
                $( $field: $field_ty = report.$field, )+
            );
        }
    };
}

impl foci_firmware::commission_dispatch::CommissionReplySink for BoardCommissionReplySink {
    forward_commission_reply!(foci_velocity_structured_boundary, FociVelocityStructuredBoundary,
        oid: u8, run_sequence: u32, evidence_sequence: u16, rung_index: u8, slot: u8,
        passing_suffix_mask: u8, rate_low_mantissa: i32, rate_high_mantissa: i32,
        rate_shift: u8, slope_margin_mantissa: i32, slope_margin_shift: u8,
        lag_one_margin_q: i32);
    forward_commission_reply!(foci_velocity_directional_region_core, FociVelocityDirectionalRegionCore,
        oid: u8, run_sequence: u32, evidence_sequence: u16, direction_kind_closure: u8,
        member_mask: u32, rung_bounds: u16, p_low: u16, p_high: u16, member_count: u8,
        boundary_rung_plus_one: u8);
    forward_commission_reply!(foci_velocity_directional_region_model, FociVelocityDirectionalRegionModel,
        oid: u8, run_sequence: u32, evidence_sequence: u16, member_mask: u32,
        common_low_q16: i32, common_high_q16: i32, pooled_q16: i32, pooled_low_q16: i32,
        pooled_high_q16: i32, variance_floor_observations: u8);
    forward_commission_reply!(foci_velocity_directional_region_rates, FociVelocityDirectionalRegionRates,
        oid: u8, run_sequence: u32, evidence_sequence: u16, member_mask: u32,
        mean_min_mantissa: i32, mean_max_mantissa: i32, envelope_low_mantissa: i32,
        envelope_high_mantissa: i32, rate_shift: u8);
    forward_commission_reply!(foci_velocity_directional_region_boundary, FociVelocityDirectionalRegionBoundary,
        oid: u8, run_sequence: u32, evidence_sequence: u16, member_mask: u32,
        tested_low_q16: i32, tested_high_q16: i32);
    forward_commission_reply!(foci_velocity_joint_region, FociVelocityJointRegion,
        oid: u8, run_sequence: u32, evidence_sequence: u16, member_mask: u32,
        rung_bounds: u16, member_count: u8, p_low: u16, p_high: u16, closure: u8);
    forward_commission_reply!(foci_velocity_stage_b_handoff_core, FociVelocityStageBHandoffCore,
        oid: u8, run_sequence: u32, evidence_sequence: u16, nominated_p: u16,
        joint_member_mask: u32, forward_member_mask: u32, reverse_member_mask: u32, flags: u8);
    forward_commission_reply!(foci_velocity_stage_b_nomination, FociVelocityStageBNomination,
        oid: u8, run_sequence: u32, evidence_sequence: u16, nominated_rung: u8,
        nominated_p: u16, distance_to_start_q16: u32, distance_to_top_q16: u32, flags: u8);
    forward_commission_reply!(foci_velocity_stage_b_directional_handoff, FociVelocityStageBDirectionalHandoff,
        oid: u8, run_sequence: u32, evidence_sequence: u16, direction: u8, member_mask: u32,
        coverage: u8, signed_rung_distance: i32, gain_ratio_num: u16, gain_ratio_den: u16,
        settled_rate_difference_mantissa: i32, settled_rate_difference_shift: u8);
    forward_commission_reply!(foci_velocity_stage_b_reproduction_core, FociVelocityStageBReproductionCore,
        oid: u8, run_sequence: u32, evidence_sequence: u16, outcome: u8,
        previous_nominated_p: u16, current_nominated_p: u16);
    forward_commission_reply!(foci_velocity_stage_b_reproduction_membership, FociVelocityStageBReproductionMembership,
        oid: u8, run_sequence: u32, evidence_sequence: u16, previous_forward_mask: u32,
        previous_reverse_mask: u32, previous_joint_mask: u32, current_forward_mask: u32,
        current_reverse_mask: u32, current_joint_mask: u32);
    forward_commission_reply!(foci_velocity_stage_b_reproduction_interval, FociVelocityStageBReproductionInterval,
        oid: u8, run_sequence: u32, evidence_sequence: u16, direction: u8,
        previous_low_q16: i32, previous_high_q16: i32, current_low_q16: i32,
        current_high_q16: i32);
    forward_commission_reply!(foci_velocity_stage_b_reproduction_digest, FociVelocityStageBReproductionDigest,
        oid: u8, run_sequence: u32, evidence_sequence: u16, previous_digest_low: u32,
        previous_digest_high: u32, current_digest_low: u32, current_digest_high: u32);
    forward_commission_reply!(foci_velocity_stage_b_terminal_core, FociVelocityStageBTerminalCore,
        oid: u8, run_sequence: u32, evidence_sequence: u16, fragment: u8, outcome: u8,
        cause: u8, model_direction_mask: u8, coverage_mask: u8, forward_region_count: u8,
        reverse_region_count: u8, forward_fragment_count: u8, reverse_fragment_count: u8,
        expected_observations: u16, emitted_observations: u16, expected_rungs: u16,
        emitted_rungs: u16);
    forward_commission_reply!(foci_velocity_stage_b_terminal_identity, FociVelocityStageBTerminalIdentity,
        oid: u8, run_sequence: u32, evidence_sequence: u16, fragment: u8,
        forward_member_mask: u32, reverse_member_mask: u32, joint_member_mask: u32,
        nominated_p: u16, plan_digest_low: u32, plan_digest_high: u32,
        digest_low: u32, digest_high: u32);
    forward_commission_reply!(foci_velocity_stage_b_terminal_interval, FociVelocityStageBTerminalInterval,
        oid: u8, run_sequence: u32, evidence_sequence: u16, fragment: u8, direction: u8,
        pooled_low_q16: i32, pooled_high_q16: i32, started_low: u32, started_high: u32,
        completed_low: u32, completed_high: u32);
    fn foci_velocity_sweep_plan_limits(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocitySweepPlanLimits,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocitySweepPlanLimits,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            requested_velocity_mrev_s: u32 = report.requested_velocity_mrev_s,
            requested_velocity_source: u8 = report.requested_velocity_source,
            planned_velocity_mrev_s: u32 = report.planned_velocity_mrev_s,
            effective_ceiling_mrev_s: u32 = report.effective_ceiling_mrev_s,
            clamp_flags: u16 = report.clamp_flags,
            binding_source: u8 = report.binding_source,
        );
    }

    fn foci_velocity_sweep_plan_geometry(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocitySweepPlanGeometry,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocitySweepPlanGeometry,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            target_velocity_rpm: i32 = report.target_velocity_rpm,
            p_start: u16 = report.p_start,
            p_top: u16 = report.p_top,
            rung_count: u8 = report.rung_count,
            observations_per_direction: u16 = report.observations_per_direction,
            max_stroke_travel_mrev: u32 = report.max_stroke_travel_mrev,
            settle_travel_reserve_mrev: u32 = report.settle_travel_reserve_mrev,
            negative_position_headroom_mrev: u32 = report.negative_position_headroom_mrev,
            positive_position_headroom_mrev: u32 = report.positive_position_headroom_mrev,
            hard_torque_limit: u16 = report.hard_torque_limit,
            usable_torque_limit: u16 = report.usable_torque_limit,
        );
    }

    fn foci_velocity_sweep_plan_timing(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocitySweepPlanTiming,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocitySweepPlanTiming,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            moving_stroke_us: u32 = report.moving_stroke_us,
            zero_settle_us: u32 = report.zero_settle_us,
            nominal_workflow_ms: u32 = report.nominal_workflow_ms,
            maximum_workflow_ms: u32 = report.maximum_workflow_ms,
        );
    }

    fn foci_velocity_observation_core(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocityObservationCore,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityObservationCore,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            rung_index: u8 = report.rung_index,
            slot: u8 = report.slot,
            classification: u8 = report.classification,
            flags: u8 = report.flags,
            delta_sign: u8 = report.delta_sign,
            delta_mantissa: u32 = report.delta_mantissa,
            delta_shift: u8 = report.delta_shift,
            elapsed_mantissa: u32 = report.elapsed_mantissa,
            elapsed_shift: u8 = report.elapsed_shift,
        );
    }

    fn foci_velocity_observation_rate(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocityObservationRate,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityObservationRate,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            rate_low: i32 = report.rate_low,
            rate_mean: i32 = report.rate_mean,
            rate_high: i32 = report.rate_high,
            deficit_low: i32 = report.deficit_low,
            deficit_high: i32 = report.deficit_high,
            rate_shift: u8 = report.rate_shift,
            suffix_len: u16 = report.suffix_len,
            selected_level: u8 = report.selected_level,
            selected_blocks: u32 = report.selected_blocks,
            variance_mantissa: u32 = report.variance_mantissa,
            variance_shift: u8 = report.variance_shift,
        );
    }

    fn foci_velocity_observation_stationarity(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocityObservationStationarity,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityObservationStationarity,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            slope_mantissa: i32 = report.slope_mantissa,
            slope_shift: u8 = report.slope_shift,
            slope_half_width_mantissa: u32 = report.slope_half_width_mantissa,
            slope_half_width_shift: u8 = report.slope_half_width_shift,
            lag_one_q: i32 = report.lag_one_q,
            lag_one_half_width_q: u32 = report.lag_one_half_width_q,
            residual_mantissa: u32 = report.residual_mantissa,
            residual_shift: u8 = report.residual_shift,
            tested_suffixes: u8 = report.tested_suffixes,
        );
    }

    fn foci_velocity_observation_disturbance(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocityObservationDisturbance,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityObservationDisturbance,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            velocity_p: u16 = report.velocity_p,
            target_velocity_rpm: i32 = report.target_velocity_rpm,
            disturbance_q16: i32 = report.disturbance_q16,
            disturbance_low_q16: i32 = report.disturbance_low_q16,
            disturbance_high_q16: i32 = report.disturbance_high_q16,
            variance_mantissa: u32 = report.variance_mantissa,
            variance_shift: u8 = report.variance_shift,
            predicted_torque_target_abs: u16 = report.predicted_torque_target_abs,
        );
    }

    fn foci_velocity_rung_verdict_band(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocityRungVerdictBand,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityRungVerdictBand,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            rung_index: u8 = report.rung_index,
            velocity_p: u16 = report.velocity_p,
            forward_low_q16: i32 = report.forward_low_q16,
            forward_high_q16: i32 = report.forward_high_q16,
            reverse_low_q16: i32 = report.reverse_low_q16,
            reverse_high_q16: i32 = report.reverse_high_q16,
            forward_p_low: u16 = report.forward_p_low,
            forward_p_high: u16 = report.forward_p_high,
            reverse_p_low: u16 = report.reverse_p_low,
            reverse_p_high: u16 = report.reverse_p_high,
        );
    }

    fn foci_velocity_rung_verdict_quality(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocityRungVerdictQuality,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityRungVerdictQuality,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            rung_index: u8 = report.rung_index,
            forward_class: u8 = report.forward_class,
            reverse_class: u8 = report.reverse_class,
            forward_closure: u8 = report.forward_closure,
            reverse_closure: u8 = report.reverse_closure,
            flags: u8 = report.flags,
            forward_eligible_rungs: u8 = report.forward_eligible_rungs,
            reverse_eligible_rungs: u8 = report.reverse_eligible_rungs,
            forward_eligible_observations: u16 = report.forward_eligible_observations,
            reverse_eligible_observations: u16 = report.reverse_eligible_observations,
        );
    }

    fn foci_velocity_sweep_terminal_direction(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocitySweepTerminalDirection,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocitySweepTerminalDirection,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            direction_closure: u8 = report.direction_closure,
            first_moving: u16 = report.first_moving,
            p_low: u16 = report.p_low,
            p_high: u16 = report.p_high,
            common_low_q16: i32 = report.common_low_q16,
            common_high_q16: i32 = report.common_high_q16,
            pooled_q16: i32 = report.pooled_q16,
            pooled_low_q16: i32 = report.pooled_low_q16,
            pooled_high_q16: i32 = report.pooled_high_q16,
            eligible_rungs: u8 = report.eligible_rungs,
            eligible_observations: u16 = report.eligible_observations,
            flags: u8 = report.flags,
        );
    }

    fn foci_velocity_sweep_terminal_integrity(
        &mut self,
        report: foci_firmware::ankyra::replies::FociVelocitySweepTerminalIntegrity,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocitySweepTerminalIntegrity,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            expected_observations: u16 = report.expected_observations,
            emitted_observations: u16 = report.emitted_observations,
            expected_rungs: u16 = report.expected_rungs,
            emitted_rungs: u16 = report.emitted_rungs,
            digest_low: u32 = report.digest_low,
            digest_high: u32 = report.digest_high,
            outcome: u8 = report.outcome,
            cause: u8 = report.cause,
            sufficient_direction_mask: u8 = report.sufficient_direction_mask,
        );
    }

    fn foci_outer_inconclusive(
        &mut self,
        report: foci_firmware::ankyra::replies::FociOuterInconclusive,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociOuterInconclusive,
            oid: u8 = report.oid,
            run_sequence: u32 = report.run_sequence,
            evidence_sequence: u16 = report.evidence_sequence,
            fragment: u8 = report.fragment,
            phase: u8 = report.phase,
            sufficient_direction_mask: u8 = report.sufficient_direction_mask,
            cause: u8 = report.cause,
            digest_low: u32 = report.digest_low,
            digest_high: u32 = report.digest_high,
        );
    }

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

    fn foci_commission_timing(
        &mut self,
        report: foci_firmware::tmc_diagnostics::CommissionTimingReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCommissionTiming,
            oid: u8 = report.oid,
            method: u8 = report.method,
            status: u8 = report.status,
            requested_period_us: u16 = report.requested_period_us,
            valid_samples: u16 = report.valid_samples,
            missed_samples: u16 = report.missed_samples,
            max_consecutive_misses: u8 = report.max_consecutive_misses,
            max_lateness_us: u32 = report.max_lateness_us,
            max_interval_us: u32 = report.max_interval_us,
            max_poll_wall_us: u32 = report.max_poll_wall_us,
            max_spi_wall_us: u32 = report.max_spi_wall_us,
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

    fn foci_adc_residual(&mut self, report: foci_firmware::tmc_diagnostics::AdcResidualReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociAdcResidual,
            oid: u8 = report.oid,
            stage: u8 = report.stage,
            sample_count: u8 = report.sample_count,
            pwm_sv_chop: u8 = report.pwm_sv_chop,
            pwm_bbm: u32 = report.pwm_bbm,
            adc_i0_scale_offset: u32 = report.adc_i0_scale_offset,
            adc_i1_scale_offset: u32 = report.adc_i1_scale_offset,
            adc_iux_mean_count: i16 = report.adc_iux_mean_count,
            adc_iwy_mean_count: i16 = report.adc_iwy_mean_count,
            pid_flux_mean_count: i16 = report.pid_flux_mean_count,
            pid_torque_mean_count: i16 = report.pid_torque_mean_count,
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

    fn foci_closed_loop_entry(
        &mut self,
        report: foci_firmware::tmc_diagnostics::ClosedLoopEntryReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociClosedLoopEntry,
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

    fn foci_current_validation_settled_sample(
        &mut self,
        report: foci_firmware::tmc_diagnostics::CurrentValidationSettledSampleReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentValidationSettledSample,
            oid: u8 = report.oid,
            axis: u8 = report.axis,
            sample_index: u8 = report.sample_index,
            direction: u8 = report.direction,
            raw_index: u8 = report.raw_index,
            attempt: u8 = report.attempt,
            target: i16 = report.target,
            sample_delay_ms: u16 = report.sample_delay_ms,
            same_axis_count: i16 = report.same_axis_count,
            cross_axis_count: i16 = report.cross_axis_count,
            cross_axis_permille: u16 = report.cross_axis_permille,
            voltage_output_permille: u16 = report.voltage_output_permille,
            encoder_delta_counts: i32 = report.encoder_delta_counts,
            status_flags: u32 = report.status_flags,
        );
    }

    fn foci_current_validation_envelope(
        &mut self,
        report: foci_firmware::tmc_diagnostics::CurrentValidationEnvelopeReport,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentValidationEnvelope,
            oid: u8 = report.oid,
            step_amplitude: u16 = report.step_amplitude,
            flux_step_amplitude: u16 = report.flux_step_amplitude,
            torque_step_amplitude: u16 = report.torque_step_amplitude,
            pidout_limit: u16 = report.pidout_limit,
            current_limited: u16 = report.current_limited,
            voltage_limited: u16 = report.voltage_limited,
            max_p: u16 = report.max_p,
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
            outer_evidence_flags: u8 = result.outer_evidence_flags,
            stiffness_timebase_ms: u16 = result.stiffness_timebase_ms,
            velocity_search_stop_reason: u8 = result.velocity_search_stop_reason,
            motion_budget_mrev: u16 = result.motion_budget_mrev,
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

/// Emit one nonterminal velocity-sweep evidence group.
pub(crate) fn emit_velocity_sweep_event(
    oid: u8,
    event: foci_firmware::commissioning::types::VelocitySweepEvent,
) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::emit_velocity_sweep_event(&mut sink, oid, event);
}

/// Emit a `FociCommissionPhase` reply for a phase transition.
pub(crate) fn emit_phase_reply(oid: u8, phase_id: foci_firmware::autotune::PhaseId) {
    let mut sink = BoardCommissionReplySink;
    foci_firmware::commission_dispatch::emit_phase_reply(&mut sink, oid, phase_id);
}
