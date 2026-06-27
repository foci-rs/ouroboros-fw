//! Ouroboros active diagnostic reply glue.
//!
//! The diagnostic state machines live in `foci-firmware`; this module keeps
//! RTIC delay wiring, channel readiness, and Ouroboros reply emission local.

use core::sync::atomic::Ordering::Relaxed;

use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_control::TmcControlRuntime;
use foci_firmware::tmc_diagnostics::{
    CurrentStepReport, CurrentTorqueSampleReport, DiagnosticDelay, DiagnosticOutcome,
    DiagnosticReadiness, ImpedanceBaselineReport, ImpedanceFitReport, ImpedanceObservationReport,
    ImpedanceProfileReport, ImpedanceRunReport, ResistanceAxisReport, ResistanceProfileReport,
    ResistanceRunReport, VoltageStepReport,
};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::{Monotonic, fugit::ExtU64};

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::tmc_control::board::{stepper_state, tmc_state};

struct MonoDiagnosticDelay;

impl DiagnosticDelay for MonoDiagnosticDelay {
    async fn delay_ms(&mut self, ms: u64) {
        Mono::delay(ms.millis()).await;
    }

    async fn delay_us(&mut self, us: u32) {
        Mono::delay(u64::from(us).micros()).await;
    }

    fn now_us(&self) -> u64 {
        Mono::now().duration_since_epoch().to_micros()
    }
}

struct BoardDiagnosticReplySink;

impl foci_firmware::tmc_diagnostics::DiagnosticReplySink for BoardDiagnosticReplySink {
    fn foci_current_step_result(&mut self, report: CurrentStepReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentStepResult,
            oid: u8 = report.oid,
            status: u8 = report.status,
            target: i16 = report.torque_target,
            duration_ms: u16 = report.duration_ms,
            voltage_limit: u16 = report.voltage_limit,
            torque_before: i16 = report.torque_before,
            torque_during: i16 = report.torque_during,
            torque_after: i16 = report.torque_after,
            flux_during: i16 = report.flux_during,
            iq_during: i16 = report.iq_during,
            id_during: i16 = report.id_during,
            uq_limited: i16 = report.uq_limited,
            ud_limited: i16 = report.ud_limited,
            encoder_before: u32 = report.encoder_before,
            encoder_after: u32 = report.encoder_after,
            encoder_delta: i32 = report.encoder_delta,
            adc_vm_raw: u16 = report.adc_vm_raw,
        );
    }

    fn foci_current_vector_step_result(&mut self, report: CurrentStepReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentVectorStepResult,
            oid: u8 = report.oid,
            status: u8 = report.status,
            torque_target: i16 = report.torque_target,
            flux_target: i16 = report.flux_target,
            duration_ms: u16 = report.duration_ms,
            voltage_limit: u16 = report.voltage_limit,
            torque_before: i16 = report.torque_before,
            torque_during: i16 = report.torque_during,
            torque_after: i16 = report.torque_after,
            flux_during: i16 = report.flux_during,
            iq_during: i16 = report.iq_during,
            id_during: i16 = report.id_during,
            uq_limited: i16 = report.uq_limited,
            ud_limited: i16 = report.ud_limited,
            encoder_before: u32 = report.encoder_before,
            encoder_after: u32 = report.encoder_after,
            encoder_delta: i32 = report.encoder_delta,
            adc_vm_raw: u16 = report.adc_vm_raw,
        );
    }

    fn foci_current_torque_sample_result(&mut self, report: CurrentTorqueSampleReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentTorqueSampleResult,
            oid: u8 = report.oid,
            status: u8 = report.status,
            target: i16 = report.target,
            flux_target: i16 = report.flux_target,
            sample_delay_ms: u16 = report.sample_delay_ms,
            voltage_limit: u16 = report.voltage_limit,
            torque_before: i16 = report.torque_before,
            torque_sample: i16 = report.torque_sample,
            torque_after: i16 = report.torque_after,
            flux_sample: i16 = report.flux_sample,
            iq_sample: i16 = report.iq_sample,
            id_sample: i16 = report.id_sample,
            uq_limited: i16 = report.uq_limited,
            ud_limited: i16 = report.ud_limited,
            encoder_before: u32 = report.encoder_before,
            encoder_sample: u32 = report.encoder_sample,
            encoder_after: u32 = report.encoder_after,
            encoder_delta_sample: i32 = report.encoder_delta_sample,
            encoder_delta_after: i32 = report.encoder_delta_after,
            adc_vm_raw: u16 = report.adc_vm_raw,
            pidin_target_torque: i32 = report.pidin_target_torque,
            pidin_target_flux: i32 = report.pidin_target_flux,
            pidout_target_torque: i32 = report.pidout_target_torque,
            pidout_target_flux: i32 = report.pidout_target_flux,
            pid_torque_target_monitor: i16 = report.pid_torque_target_monitor,
            status_flags: u32 = report.status_flags,
        );
    }

    fn foci_current_torque_sample_detail_result(&mut self, report: &CurrentTorqueSampleReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociCurrentTorqueSampleDetailResult,
            oid: u8 = report.oid,
            target: i16 = report.target,
            flux_target: i16 = report.flux_target,
            sample_delay_ms: u16 = report.sample_delay_ms,
            voltage_limit: u16 = report.voltage_limit,
            torque_error: i32 = report.torque_error,
            flux_error: i32 = report.flux_error,
            torque_error_sum: i32 = report.torque_error_sum,
            flux_error_sum: i32 = report.flux_error_sum,
            uq_prelimit: i16 = report.uq_prelimit,
            ud_prelimit: i16 = report.ud_prelimit,
            ff_velocity: i32 = report.ff_velocity,
            ff_torque: i16 = report.ff_torque,
        );
    }

    fn foci_voltage_step_result(&mut self, report: VoltageStepReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVoltageStepResult,
            oid: u8 = report.oid,
            status: u8 = report.status,
            uq_ext: i16 = report.uq_ext,
            ud_ext: i16 = report.ud_ext,
            sample_delay_ms: u16 = report.sample_delay_ms,
            torque_before: i16 = report.torque_before,
            torque_sample: i16 = report.torque_sample,
            torque_after: i16 = report.torque_after,
            flux_sample: i16 = report.flux_sample,
            iq_sample: i16 = report.iq_sample,
            id_sample: i16 = report.id_sample,
            uq_limited: i16 = report.uq_limited,
            ud_limited: i16 = report.ud_limited,
            uux_sample: i16 = report.uux_sample,
            uwy_sample: i16 = report.uwy_sample,
            pwm_ux_sample: i16 = report.pwm_ux_sample,
            pwm_wy_sample: i16 = report.pwm_wy_sample,
            pwm_sv_chop: u32 = report.pwm_sv_chop,
            pwm_bbm: u32 = report.pwm_bbm,
            pwm_maxcnt: u32 = report.pwm_maxcnt,
            phi_e_sample: i16 = report.phi_e_sample,
            phi_m_sample: i16 = report.phi_m_sample,
            encoder_before: u32 = report.encoder_before,
            encoder_sample: u32 = report.encoder_sample,
            encoder_after: u32 = report.encoder_after,
            encoder_delta_sample: i32 = report.encoder_delta_sample,
            encoder_delta_after: i32 = report.encoder_delta_after,
            adc_vm_raw: u16 = report.adc_vm_raw,
            status_flags: u32 = report.status_flags,
        );
    }

    fn foci_resistance_profile(&mut self, report: ResistanceProfileReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociResistanceProfile,
            oid: u8 = report.oid,
            profile_version: u16 = report.profile_version,
            pwm_maxcnt: u16 = report.pwm_maxcnt,
            bbm_h: u8 = report.bbm_h,
            bbm_l: u8 = report.bbm_l,
            dsadc_mdec_a: u16 = report.dsadc_mdec_a,
            dsadc_mdec_b: u16 = report.dsadc_mdec_b,
            linear_current_threshold_count: u16 = report.linear_current_threshold_count,
            encoder_move_warn_counts: u16 = report.encoder_move_warn_counts,
            status_flags_warn_mask: u32 = report.status_flags_warn_mask,
            scale_metadata_validated: u8 = u8::from(report.scale_metadata_validated),
        );
    }

    fn foci_impedance_profile(&mut self, report: ImpedanceProfileReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            crate::ankyra::replies::FociImpedanceProfile,
            oid: u8 = report.oid,
            point_count: u8 = report.point_count,
            sample_count: u16 = report.sample_count,
            sample_interval_us: u16 = report.sample_interval_us,
            carrier_frequency_millihz: u32 = report.carrier_frequency_millihz,
            samples_per_cycle: u16 = report.samples_per_cycle,
            cycles_per_angle: u16 = report.cycles_per_angle,
            max_ud: u16 = report.max_ud,
            hold_ud: u16 = report.hold_ud,
            carrier_ud: u16 = report.carrier_ud,
            status_flags_fail_mask: u32 = report.status_flags_fail_mask,
            status_flags_warn_mask: u32 = report.status_flags_warn_mask,
            scale_metadata_validated: u8 = u8::from(report.scale_metadata_validated),
        );
    }

    fn foci_impedance_fit(&mut self, report: ImpedanceFitReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            crate::ankyra::replies::FociImpedanceFit,
            oid: u8 = report.oid,
            point_index: u8 = report.point_index,
            frequency_millihz: u32 = report.frequency_millihz,
            ud_abs: u16 = report.ud_abs,
            axis_max_count_per_ud_milli: u32 = report.axis_max_count_per_ud_milli,
            axis_min_count_per_ud_milli: u32 = report.axis_min_count_per_ud_milli,
            axis_angle_electrical_counts: u16 = report.axis_angle_electrical_counts,
            residual_permille: u16 = report.residual_permille,
            coverage_permille: u16 = report.coverage_permille,
            scalar_2f_mismatch_permille: u16 = report.scalar_2f_mismatch_permille,
            magnitude_mean_count_per_ud_milli: u32 = report.magnitude_mean_count_per_ud_milli,
            magnitude_2f_count_per_ud_milli: u32 = report.magnitude_2f_count_per_ud_milli,
            magnitude_2f_residual_permille: u16 = report.magnitude_2f_residual_permille,
            magnitude_2f_coverage_permille: u16 = report.magnitude_2f_coverage_permille,
            warning_flags: u32 = report.warning_flags,
        );
    }

    fn foci_impedance_fit_v2(&mut self, report: ImpedanceFitReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            crate::ankyra::replies::FociImpedanceFitV2,
            oid: u8 = report.oid,
            point_index: u8 = report.point_index,
            mean_response_count_per_ud_milli: u32 = report.mean_response_count_per_ud_milli,
            saliency_permille: u16 = report.saliency_permille,
            saliency_determinable: u8 = report.saliency_determinable,
            angle_coverage_permille: u16 = report.angle_coverage_permille,
            axis_deviation_counts: u16 = report.axis_deviation_counts,
            inductive_response_count_per_ud_milli: u32 = report
                .inductive_response_count_per_ud_milli,
            in_phase_response_count_per_ud_milli: u32 = report
                .in_phase_response_count_per_ud_milli,
            secondary_adc_response_count_per_ud_milli: u32 = report
                .secondary_adc_response_count_per_ud_milli,
        );
    }

    fn foci_impedance_observation(&mut self, report: ImpedanceObservationReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            crate::ankyra::replies::FociImpedanceObservation,
            oid: u8 = report.oid,
            point_index: u8 = report.point_index,
            frequency_millihz: u32 = report.frequency_millihz,
            ud_abs: u16 = report.ud_abs,
            status_flags_or: u32 = report.status_flags_or,
            openloop_phi_sample: u16 = report.openloop_phi_sample,
            openloop_velocity_actual: i32 = report.openloop_velocity_actual,
            adc_iux_sample: i16 = report.adc_iux_sample,
            adc_iwy_sample: i16 = report.adc_iwy_sample,
            foc_iux_sample: i16 = report.foc_iux_sample,
            foc_iwy_sample: i16 = report.foc_iwy_sample,
            foc_uq_sample: i16 = report.foc_uq_sample,
            foc_ud_sample: i16 = report.foc_ud_sample,
            foc_uq_limited_sample: i16 = report.foc_uq_limited_sample,
            foc_ud_limited_sample: i16 = report.foc_ud_limited_sample,
            foc_iq_sample: i16 = report.foc_iq_sample,
            foc_id_sample: i16 = report.foc_id_sample,
        );
    }

    fn foci_impedance_observation_v2(&mut self, report: ImpedanceObservationReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            crate::ankyra::replies::FociImpedanceObservationV2,
            oid: u8 = report.oid,
            point_index: u8 = report.point_index,
            angle_electrical_counts: u16 = report.angle_electrical_counts,
            encoder_before: u32 = report.encoder_before,
            encoder_after: u32 = report.encoder_after,
            encoder_min: u32 = report.encoder_min,
            encoder_max: u32 = report.encoder_max,
            encoder_peak_counts: u16 = report.encoder_peak_counts,
            angle_valid: u8 = report.angle_valid,
            inductive_response_count_per_ud_milli: u32 = report
                .inductive_response_count_per_ud_milli,
            in_phase_response_count_per_ud_milli: u32 = report
                .in_phase_response_count_per_ud_milli,
        );
    }

    fn foci_impedance_baseline(&mut self, report: ImpedanceBaselineReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            crate::ankyra::replies::FociImpedanceBaseline,
            oid: u8 = report.oid,
            point_index: u8 = report.point_index,
            frequency_millihz: u32 = report.frequency_millihz,
            ud_abs: u16 = report.ud_abs,
            dc_adc_iux_sample: i16 = report.dc_adc_iux_sample,
            dc_adc_iwy_sample: i16 = report.dc_adc_iwy_sample,
            dc_foc_iux_sample: i16 = report.dc_foc_iux_sample,
            dc_foc_iwy_sample: i16 = report.dc_foc_iwy_sample,
            dc_foc_iq_sample: i16 = report.dc_foc_iq_sample,
            dc_foc_id_sample: i16 = report.dc_foc_id_sample,
        );
    }

    fn foci_resistance_axis(&mut self, report: ResistanceAxisReport) {
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

    fn foci_impedance_run(&mut self, report: ImpedanceRunReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            crate::ankyra::replies::FociImpedanceRun,
            oid: u8 = report.oid,
            status: u8 = report.status,
            selected_axis_max_count_per_ud_milli: u32 = report
                .selected_axis_max_count_per_ud_milli,
            selected_axis_min_count_per_ud_milli: u32 = report
                .selected_axis_min_count_per_ud_milli,
            repeatability_permille: u16 = report.repeatability_permille,
            warning_flags: u32 = report.warning_flags,
            status_flags_or: u32 = report.status_flags_or,
            readback_mismatch_addr: u8 = report.readback_mismatch_addr,
            readback_expected: u32 = report.readback_expected,
            readback_actual: u32 = report.readback_actual,
            filter_torque_enable_before: u8 = report.filter_torque_enable_before,
            filter_flux_enable_before: u8 = report.filter_flux_enable_before,
            filter_torque_enable_during: u8 = report.filter_torque_enable_during,
            filter_flux_enable_during: u8 = report.filter_flux_enable_during,
            duplicate_zero_encoder_drift_counts: u32 = report
                .duplicate_zero_encoder_drift_counts,
        );
    }

    fn foci_resistance_run(&mut self, report: ResistanceRunReport) {
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
}

pub(crate) async fn run_diagnostic_request<'spi>(
    diagnostic_req: TmcRequest,
    channel: u8,
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
) {
    let homing_active = HOMING_STATE.load(Relaxed) != 0;
    let stepper_active = stepper_state(channel).timer_active_mirror.load(Relaxed);
    let saved_voltage_limit = tmc_state(channel).voltage_limit_cfg.load(Relaxed);
    let mut delay = MonoDiagnosticDelay;

    let outcome = foci_firmware::tmc_diagnostics::handle_diagnostic_request(
        diagnostic_req,
        foci_firmware::tmc_diagnostics::DiagnosticContext {
            tmc,
            runtime,
            delay: &mut delay,
            readiness: DiagnosticReadiness {
                homing_active,
                stepper_active,
                saved_voltage_limit,
            },
            tmc_config: &crate::board_config::TMC_CONFIG,
        },
    )
    .await;

    match outcome {
        DiagnosticOutcome::Handled(report) => {
            let mut sink = BoardDiagnosticReplySink;
            foci_firmware::tmc_diagnostics::emit_diagnostic_report(&mut sink, report);
        }
        DiagnosticOutcome::NotDiagnostic(_req) => {
            defmt::error!("diagnostic dispatcher returned non-diagnostic request");
        }
    }
}
