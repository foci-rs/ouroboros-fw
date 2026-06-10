//! Ouroboros TMC control request handling.

use core::sync::atomic::Ordering::Relaxed;

use foci_firmware::pid::{InnerLoopGains, OuterLoopGains};
use foci_firmware::state::trsync::HOMING_STATE;
use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
use foci_firmware::tmc_control::helpers::{apply_live_inner_gains, apply_live_outer_gains};
use foci_firmware::tmc_control::{
    BoardAction, ShutdownReason, StepperAction, TmcControlRequestOutcome, TmcControlRequestTail,
    TmcControlRuntime,
};
use foci_firmware::tmc_diagnostics::{
    CurrentStepReport, CurrentTorqueSampleReport, DiagnosticReport, VoltageStepReport,
};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CHANNEL_COUNT;
#[cfg(feature = "dev")]
use crate::helpers::dev::{do_test_spin, tmc_enter_openloop};
use crate::helpers::stepper::apply_stepper_post_action;
use crate::tmc_control::board::{
    TmcEnablePins, TmcRuntimes, enable_pin, next_commission_trace_seq, ouroboros_backend,
    stepper_state, tmc_state,
};
use crate::tmc_control_glue::ActionDisposition;
use crate::tmc_dispatch::handle_tmc_request;

struct MonoDiagnosticDelay;

impl foci_firmware::tmc_diagnostics::DiagnosticDelay for MonoDiagnosticDelay {
    async fn delay_ms(&mut self, ms: u64) {
        Mono::delay(ms.millis()).await;
    }
}

fn emit_current_step_reply(vector_reply: bool, report: CurrentStepReport) {
    if vector_reply {
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
    } else {
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
}

fn emit_current_torque_sample_reply(report: CurrentTorqueSampleReport) {
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

fn emit_current_torque_sample_detail_reply(report: &CurrentTorqueSampleReport) {
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

fn emit_current_torque_sample_report(report: CurrentTorqueSampleReport, emit_detail: bool) {
    if emit_detail {
        emit_current_torque_sample_detail_reply(&report);
    }
    emit_current_torque_sample_reply(report);
}

fn emit_voltage_step_reply(report: VoltageStepReport) {
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

fn emit_diagnostic_report(report: DiagnosticReport) {
    match report {
        DiagnosticReport::CurrentStep {
            vector_reply,
            report,
        } => emit_current_step_reply(vector_reply, report),
        DiagnosticReport::CurrentTorqueSample {
            report,
            emit_detail,
        } => emit_current_torque_sample_report(report, emit_detail),
        DiagnosticReport::VoltageStep(report) => emit_voltage_step_reply(report),
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
            readiness: foci_firmware::tmc_diagnostics::DiagnosticReadiness {
                homing_active,
                stepper_active,
                saved_voltage_limit,
            },
        },
    )
    .await;

    match outcome {
        foci_firmware::tmc_diagnostics::DiagnosticOutcome::Handled(report) => {
            emit_diagnostic_report(report);
        }
        foci_firmware::tmc_diagnostics::DiagnosticOutcome::NotDiagnostic(_req) => {
            defmt::error!("diagnostic dispatcher returned non-diagnostic request");
        }
    }
}

pub(crate) struct TmcControlRequestContext<'a, 'spi> {
    pub(crate) channel: u8,
    pub(crate) commissioning_blocked: bool,
    pub(crate) runtimes: &'a mut TmcRuntimes,
    pub(crate) tmc: &'a mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    pub(crate) enable_pins: &'a mut TmcEnablePins,
    pub(crate) commission_trace_engine_seq: &'a mut [u32; TMC_CHANNEL_COUNT as usize],
}

pub(crate) async fn handle_tmc_control_request<'a, 'spi>(
    req: TmcRequest,
    disposition: ActionDisposition,
    context: TmcControlRequestContext<'a, 'spi>,
) -> TmcControlRequestOutcome {
    let TmcControlRequestContext {
        channel,
        commissioning_blocked,
        runtimes,
        tmc,
        enable_pins,
        commission_trace_engine_seq,
    } = context;
    let runtime = &mut runtimes[channel as usize];

    match disposition {
        ActionDisposition::RequestHandled => TmcControlRequestOutcome::fall_through(),
        ActionDisposition::RunDiagnostic(diagnostic_req) => {
            run_diagnostic_request(diagnostic_req, channel, runtime, tmc).await;
            TmcControlRequestOutcome::fall_through()
        }
        ActionDisposition::ContinueRequest => {
            match req {
                TmcRequest {
                    kind: TmcRequestKind::SetEnable(enabled),
                    ..
                } => {
                    if enabled {
                        if !runtime.is_calibrated() {
                            // Auto-calibrate: Klipper may send enable before
                            // the Python module's foci_calibrate arrives due
                            // to queue_digital_out timing. Launch the
                            // commissioning engine to run ADC calibration
                            // asynchronously; the CalibrateDone handler will
                            // complete encoder alignment and closed-loop entry.
                            if commissioning_blocked {
                                defmt::warn!(
                                    "Auto-calibrate rejected: commissioning already active"
                                );
                                return TmcControlRequestOutcome::skip_tick_tail();
                            }
                            if !foci_firmware::tmc::auto_calibrate_on_enable_allowed(channel) {
                                defmt::warn!("Auto-calibrate rejected: host gate closed");
                                return TmcControlRequestOutcome::skip_tick_tail();
                            }
                            if tmc_state(channel)
                                .commission_in_flight
                                .swap(true, core::sync::atomic::Ordering::Relaxed)
                            {
                                defmt::warn!("Auto-calibrate skipped: commission in flight");
                                return TmcControlRequestOutcome::skip_tick_tail();
                            }
                            defmt::info!("Auto-calibrating on enable");
                            runtime.set_commission_engine(
                                foci_firmware::commission::CommissioningEngine::new_calibrate_for_tmc_with_trace_seq(
                                    0,
                                    tmc_state(channel),
                                    next_commission_trace_seq(commission_trace_engine_seq, channel),
                                ),
                            );
                            // Don't set motor_enabled/tmc_state().motor_armed yet --
                            // the CalibrateDone handler does that after
                            // encoder alignment + closed-loop entry.
                            return TmcControlRequestOutcome::skip_tick_tail();
                        }
                        // Existing precondition: this arm runs only when
                        // calibrated == true (the !calibrated branch above
                        // continues to the auto-calibrate path).
                        if runtime.is_motor_enabled() {
                            defmt::info!("Motor enable sync (already armed)");
                            return TmcControlRequestOutcome::skip_tick_tail();
                        }

                        let actions = runtime.prepare_enable_request(tmc_state(channel));
                        for action in actions {
                            if let BoardAction::Stepper(StepperAction::ArmCheck { post_action }) =
                                action
                            {
                                if !runtime.restore_phase_advance_now(tmc).await {
                                    defmt::warn!(
                                        "phase advance: failed to restore ABN offset before arm"
                                    );
                                    return TmcControlRequestOutcome::skip_tick_tail();
                                }
                                {
                                    let mut backend = ouroboros_backend(
                                        channel,
                                        tmc,
                                        enable_pin(enable_pins, channel),
                                    );
                                    let mut state = runtime
                                        .dispatch_state(tmc_state(channel), stepper_state(channel));
                                    if let Err(error) =
                                        foci_firmware::tmc_control::lifecycle::arm_motor_electrical(
                                            &mut backend,
                                            &mut state,
                                        )
                                        .await
                                    {
                                        defmt::error!(
                                            "Motor enable rejected by sentinel verify: {}",
                                            error
                                        );
                                        foci_firmware::tmc_control::lifecycle::disarm_motor(
                                            &mut backend,
                                            &mut state,
                                        )
                                        .await;
                                        return TmcControlRequestOutcome::skip_tick_tail();
                                    }
                                    apply_stepper_post_action(post_action, &mut state, channel);
                                }

                                defmt::info!("Motor enabled (calibrated, step/dir armed)");
                            }
                        }
                    } else {
                        // Disable: stop stepping first, then kill motor electrically.
                        let stop_disposition = crate::tmc_control_glue::apply_pre_request_actions(
                            channel,
                            runtime.prepare_disable_request(),
                            |_| {},
                        );
                        debug_assert_eq!(stop_disposition, ActionDisposition::ContinueRequest);

                        let phase_advance_restored = runtime.restore_phase_advance_now(tmc).await;
                        if !phase_advance_restored {
                            defmt::warn!(
                                "phase advance: failed to restore ABN offset before disarm"
                            );
                        }
                        {
                            let mut backend =
                                ouroboros_backend(channel, tmc, enable_pin(enable_pins, channel));
                            let mut state =
                                runtime.dispatch_state(tmc_state(channel), stepper_state(channel));
                            foci_firmware::tmc_control::lifecycle::disarm_motor(
                                &mut backend,
                                &mut state,
                            )
                            .await;
                        }

                        // Zero position counter
                        stepper_state(channel)
                            .position
                            .store(0, core::sync::atomic::Ordering::Relaxed);

                        if !phase_advance_restored {
                            return TmcControlRequestOutcome::skip_tick_tail();
                        }

                        defmt::info!("TMC4671 motor disabled");
                    }
                }
                TmcRequest {
                    kind: TmcRequestKind::Calibrate { oid },
                    ..
                } => {
                    if commissioning_blocked {
                        defmt::warn!("foci_calibrate rejected: commission in flight");
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCalibrateResult,
                            oid: u8 = oid,
                            status: u8 = foci_firmware::calibration::CalibrationError::InternalError.status_code(),
                            adc_i0: u16 = 0,
                            adc_i1: u16 = 0,
                            encoder_count: u32 = 0,
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    if runtime.is_motor_enabled() {
                        defmt::warn!("foci_calibrate rejected: motor already enabled");
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCalibrateResult,
                            oid: u8 = oid,
                            status: u8 = foci_firmware::calibration::CalibrationError::AlreadyEnabled.status_code(),
                            adc_i0: u16 = 0,
                            adc_i1: u16 = 0,
                            encoder_count: u32 = 0,
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    // Engine tick will drive ADC cal; CalibrateDone handler
                    // performs encoder alignment and closed-loop entry.
                    runtime.clear_calibrated();
                    runtime.set_commission_engine(
                        foci_firmware::commission::CommissioningEngine::new_calibrate_for_tmc_with_trace_seq(
                            oid,
                            tmc_state(channel),
                            next_commission_trace_seq(commission_trace_engine_seq, channel),
                        ),
                    );
                }
                #[cfg(feature = "dev")]
                TmcRequest {
                    kind: TmcRequestKind::Init,
                    ..
                } => {
                    defmt::info!("TMC4671 init: starting open-loop stepper config");
                    tmc_enter_openloop(tmc, enable_pin(enable_pins, channel)).await;
                }
                #[cfg(feature = "dev")]
                TmcRequest {
                    kind: TmcRequestKind::TestSpin { revolutions },
                    ..
                } => {
                    do_test_spin(tmc, revolutions).await;
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::SetPidGains {
                            flux_p,
                            flux_i,
                            torque_p,
                            torque_i,
                        },
                    ..
                } => {
                    let gains = InnerLoopGains {
                        flux_p,
                        flux_i,
                        torque_p,
                        torque_i,
                    };
                    let write_status = apply_live_inner_gains(runtime, tmc, gains).await;
                    if !write_status.all_ok() {
                        defmt::warn!("PID gains live write failed");
                    }
                    defmt::info!(
                        "PID gains received: flux P={} I={}, torque P={} I={}",
                        flux_p,
                        flux_i,
                        torque_p,
                        torque_i,
                    );
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::SetPositionGains {
                            position_p,
                            position_i,
                            velocity_p,
                            velocity_i,
                        },
                    ..
                } => {
                    let gains = OuterLoopGains {
                        position_p,
                        position_i,
                        velocity_p,
                        velocity_i,
                    };
                    let write_status = apply_live_outer_gains(tmc_state(channel), tmc, gains).await;
                    if !write_status.all_ok() {
                        defmt::warn!("Position gains live write failed");
                    }
                    defmt::info!(
                        "Position gains received: pos P={} I={}, vel P={} I={}",
                        position_p,
                        position_i,
                        velocity_p,
                        velocity_i,
                    );
                }
                TmcRequest {
                    kind: TmcRequestKind::SetVelocityFeedforward { enable, multiplier },
                    ..
                } => {
                    defmt::info!(
                        "Velocity feedforward: {} multiplier={} (via queue)",
                        enable,
                        multiplier
                    );
                    // Feedforward flag is already set by the atomic in the
                    // command handler; this arm exists for exhaustiveness.
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::SetVelocityTransientFeedforward {
                            enable,
                            lead_time_us,
                            gain_permille,
                            max_offset,
                            rate_hz,
                        },
                    ..
                } => {
                    defmt::info!(
                        "Velocity transient feedforward: {} lead_time_us={} \
                gain_permille={} max_offset={} rate_hz={} (via queue)",
                        enable,
                        lead_time_us,
                        gain_permille,
                        max_offset,
                        rate_hz
                    );
                    // Transient feedforward atomics are set by the command handler;
                    // this arm exists for exhaustiveness and operator-visible logging.
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::SetAccelFeedforward {
                            enable,
                            accel_gain_permille,
                            decel_gain_permille,
                        },
                    ..
                } => {
                    defmt::info!(
                        "Acceleration feedforward: {} accel_gain_permille={} \
                decel_gain_permille={} (via queue)",
                        enable,
                        accel_gain_permille,
                        decel_gain_permille
                    );
                    // Acceleration feedforward atomics are set by the command handler;
                    // this arm exists for exhaustiveness and operator-visible logging.
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::SetDecouplingFeedforward {
                            enable,
                            r_int,
                            l_int,
                            pole_pairs,
                            position_units_per_rev,
                            f_pwm_hz,
                            max_offset,
                        },
                    ..
                } => {
                    defmt::info!(
                        "Decoupling feedforward: {} r_int={} l_int={} pole_pairs={} \
                position_units_per_rev={} f_pwm_hz={} max_offset={} (via queue)",
                        enable,
                        r_int,
                        l_int,
                        pole_pairs,
                        position_units_per_rev,
                        f_pwm_hz,
                        max_offset
                    );
                    // Feedforward atomics are set by the command handler; this arm
                    // exists for request-shell parity and exhaustiveness.
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::SetPositionLead {
                            enable,
                            gain_permille,
                            max_counts,
                        },
                    ..
                } => {
                    defmt::info!(
                        "Position lead: {} gain_permille={} max_counts={} (via queue)",
                        enable,
                        gain_permille,
                        max_counts
                    );
                    // Position-lead atomics are set by the command handler; this arm
                    // exists for exhaustiveness and operator-visible logging.
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::SetPhaseAdvance {
                            enable,
                            gain_ppm,
                            max_counts,
                            deadband,
                        },
                    ..
                } => {
                    defmt::info!(
                        "Phase advance: {} gain_ppm={} max_counts={} deadband={} \
                (via queue)",
                        enable,
                        gain_ppm,
                        max_counts,
                        deadband
                    );
                    // Phase-advance atomics are set by the command handler; this arm
                    // exists for exhaustiveness and operator-visible logging.
                }
                TmcRequest {
                    kind: TmcRequestKind::Commission { oid, profile },
                    ..
                } => {
                    if commissioning_blocked {
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::AlreadyRunning
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    if runtime.is_motor_enabled() {
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::MotorEnabled
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    let current_limit = tmc_state(channel)
                        .current_limit_reg
                        .load(core::sync::atomic::Ordering::Relaxed);
                    if current_limit == 0 {
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::NoCurrent
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    runtime.clear_calibrated();
                    runtime.set_commission_engine(
                        foci_firmware::commission::CommissioningEngine::new_commission_for_tmc_with_trace_seq(
                            oid,
                            tmc_state(channel),
                            foci_firmware::autotune::CommissionProfile::from_u8(profile),
                            next_commission_trace_seq(commission_trace_engine_seq, channel),
                        ),
                    );
                }
                TmcRequest {
                    kind:
                        TmcRequestKind::Tune {
                            oid,
                            profile,
                            mode,
                            inner_lambda_us,
                            theta_e_us,
                            current_ringing_count,
                            current_bandwidth_hz,
                            tau_e_us,
                            tau_e_crosscheck_us,
                            tau_residual_permille,
                            inner_warning_flags,
                        },
                    ..
                } => {
                    // Stage 2 requires the motor to be enabled and
                    // holding in closed-loop position mode (from Stage 1
                    // or a prior calibrate). Reject if not ready.
                    if commissioning_blocked {
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::AlreadyRunning
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    if !runtime.is_motor_enabled() || !runtime.is_calibrated() {
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::EncoderNotAligned
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    let current_limit = tmc_state(channel)
                        .current_limit_reg
                        .load(core::sync::atomic::Ordering::Relaxed);
                    if current_limit == 0 {
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociCommissionPhase,
                            oid: u8 = oid,
                            phase: u8 = 0,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::NoCurrent
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    let entry_snapshot = {
                        use core::sync::atomic::Ordering::Relaxed;
                        foci_firmware::commission::EntrySnapshot {
                            velocity_p: tmc_state(channel).velocity_p_value.load(Relaxed),
                            velocity_i: tmc_state(channel).velocity_i_value.load(Relaxed),
                            position_p: tmc_state(channel).position_p_value.load(Relaxed),
                            position_i: tmc_state(channel).position_i_value.load(Relaxed),
                            velocity_limit: tmc_state(channel).velocity_limit_cfg.load(Relaxed),
                            voltage_limit: tmc_state(channel).voltage_limit_cfg.load(Relaxed),
                            velocity_filter_hz: tmc_state(channel).velocity_filter_hz.load(Relaxed),
                            torque_filter_hz: tmc_state(channel).torque_filter_hz.load(Relaxed),
                            position_filter_hz: tmc_state(channel).position_filter_hz.load(Relaxed),
                            flux_filter_hz: tmc_state(channel).flux_filter_hz.load(Relaxed),
                        }
                    };
                    let outer_params = foci_firmware::commission::OuterTuneParams {
                        profile: foci_firmware::autotune::CommissionProfile::from_u8(profile),
                        mode: foci_firmware::autotune::OperatingMode::from_u8(mode),
                        inner_lambda_us,
                        theta_e_us,
                        current_ringing: current_ringing_count > 1,
                        current_bandwidth_hz,
                        tau_e_us,
                        tau_e_crosscheck_us,
                        tau_residual_permille,
                        inner_warning_flags,
                    };
                    runtime.set_commission_engine(
                        foci_firmware::commission::CommissioningEngine::new_outer_for_tmc_with_trace_seq(
                            oid,
                            tmc_state(channel),
                            outer_params,
                            entry_snapshot,
                            next_commission_trace_seq(commission_trace_engine_seq, channel),
                        ),
                    );
                }
                TmcRequest {
                    kind: TmcRequestKind::SelfTest { oid },
                    ..
                } => {
                    if commissioning_blocked {
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociSelftestDone,
                            oid: u8 = oid,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::AlreadyRunning
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    if runtime.is_motor_enabled() {
                        defmt::warn!("foci_selftest rejected: motor enabled");
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        ::ankyra::klipper_reply_from!(
                            &mut crate::_ankyra_config::Sender,
                            foci_firmware::ankyra::replies::FociSelftestDone,
                            oid: u8 = oid,
                            status: u8 =
                                foci_firmware::autotune::CommissionError::MotorEnabled
                                    .status_code(),
                        );
                        return TmcControlRequestOutcome::skip_tick_tail();
                    }
                    // Engine tick drives selftest phases; SelftestDone
                    // handler sends the completion signal.
                    runtime.set_commission_engine(
                        foci_firmware::commission::CommissioningEngine::new_selftest_for_tmc_with_trace_seq(
                            oid,
                            tmc_state(channel),
                            next_commission_trace_seq(commission_trace_engine_seq, channel),
                        ),
                    );
                }
                req @ TmcRequest {
                    kind: TmcRequestKind::SetEncoder { .. },
                    ..
                } => {
                    if handle_tmc_request(req, tmc).await.is_err() {
                        return TmcControlRequestOutcome::shutdown_then_refresh_velocity_scale(
                            ShutdownReason::RequestValidationFailed,
                            TmcControlRequestTail::FallThrough,
                        );
                    }
                    return TmcControlRequestOutcome::refresh_velocity_scale(
                        TmcControlRequestTail::FallThrough,
                    );
                }
                other => {
                    if handle_tmc_request(other, tmc).await.is_err() {
                        return TmcControlRequestOutcome::shutdown(
                            ShutdownReason::RequestValidationFailed,
                            TmcControlRequestTail::FallThrough,
                        );
                    }
                }
            }
            TmcControlRequestOutcome::fall_through()
        }
    }
}
