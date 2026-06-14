//! Klipper reply emitters for commission/calibrate/selftest/tune outcomes.
//!
//! The semantic `ReplyPayload` mapping lives in `foci_firmware`; this module
//! keeps only the board-generated ankyra sender calls.

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
