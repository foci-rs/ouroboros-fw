//! Ouroboros active diagnostic reply glue.
//!
//! The diagnostic state machines live in `foci-firmware`; this module keeps
//! RTIC delay wiring, channel readiness, and Ouroboros reply emission local.

use core::sync::atomic::Ordering::Relaxed;

use embassy_stm32::gpio::Output;
use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_control::TmcControlRuntime;
#[cfg(feature = "trace-velocity-limit-latch-test")]
use foci_firmware::tmc_diagnostics::VelocityLimitLatchReport;
use foci_firmware::tmc_diagnostics::{
    CurrentStepReport, CurrentTorqueSampleReport, DiagnosticDelay, DiagnosticOutcome,
    DiagnosticPowerStage, DiagnosticReadiness, ResistanceAxisReport, ResistanceProfileReport,
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

struct OuroborosDiagnosticPowerStage<'a> {
    channel: u8,
    enable_pin: &'a mut Output<'static>,
}

impl DiagnosticPowerStage for OuroborosDiagnosticPowerStage<'_> {
    fn trip_power_stage(&mut self) {
        self.enable_pin.set_low();
        crate::helpers::stepper::stepper_stop_from_p1(self.channel);
    }
}

/// Emit a deterministic not-ready reply without touching diagnostic hardware.
pub(crate) fn emit_commission_busy_diagnostic(request: TmcRequest) {
    match foci_firmware::tmc_diagnostics::busy_diagnostic_report(
        request,
        &crate::board_config::TMC_CONFIG,
    ) {
        DiagnosticOutcome::Handled(report) => {
            foci_firmware::tmc_diagnostics::emit_diagnostic_report(
                &mut BoardDiagnosticReplySink,
                report,
            );
        }
        DiagnosticOutcome::NotDiagnostic(_) => {
            defmt::error!("commission busy hook received non-diagnostic request");
        }
    }
}

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

    fn foci_resistance_run(&mut self, report: ResistanceRunReport) {
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

    #[cfg(feature = "trace-velocity-limit-latch-test")]
    fn foci_velocity_limit_latch_flags(&mut self, report: VelocityLimitLatchReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityLimitLatchFlags,
            oid: u8 = report.oid,
            active_status_flags: u32 = report.active_status_flags,
            post_pulse_status_flags: u32 = report.post_pulse_status_flags,
            delayed_status_flags: u32 = report.delayed_status_flags,
            post_clear_status_flags: u32 = report.post_clear_status_flags,
        );
    }

    #[cfg(feature = "trace-velocity-limit-latch-test")]
    fn foci_velocity_limit_latch_motion(&mut self, report: VelocityLimitLatchReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityLimitLatchMotion,
            oid: u8 = report.oid,
            pulse_elapsed_us: u32 = report.pulse_elapsed_us,
            delayed_read_elapsed_us: u32 = report.delayed_read_elapsed_us,
            encoder_before: u32 = report.encoder_before,
            encoder_after: u32 = report.encoder_after,
            encoder_delta: i32 = report.encoder_delta,
        );
    }

    #[cfg(feature = "trace-velocity-limit-latch-test")]
    fn foci_velocity_limit_latch_restore(&mut self, report: VelocityLimitLatchReport) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityLimitLatchRestore,
            oid: u8 = report.oid,
            saved_limit: u32 = report.saved_limit,
            saved_gains: u32 = report.saved_gains,
            saved_target: i32 = report.saved_target,
            saved_mode: u32 = report.saved_mode,
            restored_limit_readback: u32 = report.restored_limit_readback,
            restored_gains_readback: u32 = report.restored_gains_readback,
            restored_target_readback: i32 = report.restored_target_readback,
            restored_mode_readback: u32 = report.restored_mode_readback,
            restore_verified_mask: u8 = report.restore_verified_mask,
        );
    }

    #[cfg(feature = "trace-velocity-limit-latch-test")]
    fn foci_velocity_limit_latch_result(&mut self, report: VelocityLimitLatchReport) {
        crate::usb_trace::request_event(
            foci_firmware::trace::events::EVENT_LEVEL_INFO,
            foci_firmware::trace::events::SUBSYSTEM_COMMISSIONING,
            foci_firmware::trace::events::EVENT_COMMISSION_VELOCITY_LIMIT_LATCH_TEST,
            [
                i32::from(report.oid),
                i32::from(report.status),
                report.active_status_flags as i32,
                report.delayed_status_flags as i32,
            ],
            4,
        );
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociVelocityLimitLatchCore,
            oid: u8 = report.oid,
            status: u8 = report.status,
            test_limit: u16 = report.test_limit,
            p_raw: u16 = report.p_raw,
            i_raw: u16 = report.i_raw,
            target_velocity_rpm: i32 = report.target_velocity_rpm,
            limit_readback: u32 = report.limit_readback,
            gains_readback: u32 = report.gains_readback,
            target_readback: i32 = report.target_readback,
            power_stage_tripped: u8 = u8::from(report.power_stage_tripped),
        );
    }
}

pub(crate) async fn run_diagnostic_request<'spi>(
    diagnostic_req: TmcRequest,
    channel: u8,
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    enable_pin: &mut Output<'static>,
) {
    let homing_active = HOMING_STATE.load(Relaxed) != 0;
    let stepper_active = stepper_state(channel).timer_active_mirror.load(Relaxed);
    let saved_voltage_limit = tmc_state(channel).voltage_limit_cfg.load(Relaxed);
    let mut delay = MonoDiagnosticDelay;
    let mut power_stage = OuroborosDiagnosticPowerStage {
        channel,
        enable_pin,
    };

    let outcome = foci_firmware::tmc_diagnostics::handle_diagnostic_request(
        diagnostic_req,
        foci_firmware::tmc_diagnostics::DiagnosticContext {
            tmc,
            runtime,
            delay: &mut delay,
            power_stage: &mut power_stage,
            readiness: DiagnosticReadiness {
                homing_active,
                stepper_active,
                saved_voltage_limit,
            },
            tmc_config: &crate::board_config::TMC_CONFIG,
            feedback_config: crate::board_config::FEEDBACK_CONFIGS[channel as usize],
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
