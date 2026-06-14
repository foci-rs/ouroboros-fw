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
    DiagnosticReadiness, VoltageStepReport,
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
