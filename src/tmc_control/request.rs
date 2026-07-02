//! Ouroboros TMC control request handling.

use foci_firmware::autotune::CommissionError;
use foci_firmware::commission_dispatch::{ReplyPayload, StepperPostAction};
use foci_firmware::commissioning::backend::CommissionRegisterIo;
use foci_firmware::state::stepper::StepperChannelState;
use foci_firmware::state::tmc::TmcChannelState;
use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
use foci_firmware::tmc_control::request::{
    TmcPreRequestDisposition, TmcRequestBoard, handle_tmc_control_request as handle_shared_request,
};
use foci_firmware::tmc_control::{
    BoardActions, TmcControlRequestOutcome, TmcControlRequestTail, TmcControlRuntime,
};
use foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CHANNEL_COUNT;
#[cfg(feature = "dev")]
use crate::helpers::dev::{do_test_spin, tmc_enter_openloop};
use crate::helpers::stepper::apply_stepper_post_action;
use crate::tmc_control::board::{
    TmcEnablePins, TmcRuntimes, enable_pin, next_commission_trace_seq, ouroboros_backend,
    stepper_state, tmc_state,
};
use crate::tmc_dispatch::handle_tmc_request;

struct OuroborosRequestBoard<'a> {
    channel: u8,
    commissioning_blocked: bool,
    enable_pins: &'a mut TmcEnablePins,
    commission_trace_engine_seq: &'a mut [u32; TMC_CHANNEL_COUNT as usize],
}

impl<'a> OuroborosRequestBoard<'a> {
    fn new(
        channel: u8,
        commissioning_blocked: bool,
        enable_pins: &'a mut TmcEnablePins,
        commission_trace_engine_seq: &'a mut [u32; TMC_CHANNEL_COUNT as usize],
    ) -> Self {
        Self {
            channel,
            commissioning_blocked,
            enable_pins,
            commission_trace_engine_seq,
        }
    }
}

impl<'spi> TmcRequestBoard<TmcSpiDevice<'spi>> for OuroborosRequestBoard<'_> {
    fn channel(&self) -> u8 {
        self.channel
    }

    fn tmc_state(&self) -> &'static TmcChannelState {
        tmc_state(self.channel)
    }

    fn stepper_state(&self) -> &'static StepperChannelState {
        stepper_state(self.channel)
    }

    fn commissioning_blocked(&self, _runtime: &TmcControlRuntime) -> bool {
        self.commissioning_blocked
    }

    fn next_commission_trace_seq(&mut self) -> u32 {
        next_commission_trace_seq(self.commission_trace_engine_seq, self.channel)
    }

    fn normal_tail(&self) -> TmcControlRequestTail {
        TmcControlRequestTail::SkipTickTail
    }

    fn emit_reply(&mut self, payload: ReplyPayload) {
        crate::ankyra::replies::emit_terminal_reply(payload);
    }

    fn apply_stepper_actions(&mut self, actions: BoardActions) {
        let disposition =
            crate::tmc_control::effects::apply_pre_request_actions(self.channel, actions, |_| {});
        debug_assert_eq!(disposition, TmcPreRequestDisposition::ContinueRequest);
    }

    async fn arm_motor(
        &mut self,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
        post_action: StepperPostAction,
    ) -> Result<(), CommissionError> {
        let mut backend = ouroboros_backend(
            self.channel,
            tmc,
            enable_pin(self.enable_pins, self.channel),
        );
        let mut state =
            runtime.dispatch_state(tmc_state(self.channel), stepper_state(self.channel));
        foci_firmware::tmc_control::lifecycle::arm_motor_electrical(&mut backend, &mut state)
            .await?;
        apply_stepper_post_action(post_action, &mut state, self.channel);
        Ok(())
    }

    async fn arm_motor_for_voltage_vectors(
        &mut self,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
        voltage_limit: u16,
    ) -> Result<(), CommissionError> {
        let mut backend = ouroboros_backend(
            self.channel,
            tmc,
            enable_pin(self.enable_pins, self.channel),
        );
        backend
            .write_register(PIDOUT_UQ_UD_LIMITS, u32::from(voltage_limit))
            .await
            .map_err(|_| CommissionError::Spi)?;
        let mut state =
            runtime.dispatch_state(tmc_state(self.channel), stepper_state(self.channel));
        foci_firmware::tmc_control::lifecycle::arm_motor_for_voltage_vectors(
            &mut backend,
            &mut state,
        )
        .await
    }

    async fn disarm_motor(
        &mut self,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    ) {
        let mut backend = ouroboros_backend(
            self.channel,
            tmc,
            enable_pin(self.enable_pins, self.channel),
        );
        let mut state =
            runtime.dispatch_state(tmc_state(self.channel), stepper_state(self.channel));
        foci_firmware::tmc_control::lifecycle::disarm_motor(&mut backend, &mut state).await;
        stepper_state(self.channel)
            .position
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    async fn dispatch_register_request(
        &mut self,
        req: TmcRequest,
        tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    ) -> Result<(), ()> {
        handle_tmc_request(req, tmc).await
    }

    async fn run_diagnostic_request(
        &mut self,
        req: TmcRequest,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    ) {
        if matches!(&req.kind, TmcRequestKind::HighRateCaptureTest { .. }) {
            let readiness = crate::tmc_diagnostics::diagnostic_readiness(self.channel);
            let mut sink = crate::tmc_diagnostics::BoardDiagnosticReplySink;
            let Ok(req) = foci_firmware::tmc_diagnostics::preflight_high_rate_capture_request(
                req, readiness, &mut sink,
            ) else {
                return;
            };

            let voltage_limit = foci_firmware::tmc_diagnostics::high_rate_capture_voltage_limit();
            let phase_advance_restored = runtime.restore_phase_advance_now(tmc).await;
            let arm_result = if phase_advance_restored {
                self.arm_motor_for_voltage_vectors(runtime, tmc, voltage_limit)
                    .await
            } else {
                Err(CommissionError::Spi)
            };

            if arm_result.is_ok() {
                let mut delay = crate::tmc_diagnostics::MonoDiagnosticDelay;
                let context = foci_firmware::tmc_diagnostics::DiagnosticContext {
                    tmc,
                    runtime,
                    delay: &mut delay,
                    readiness,
                    tmc_config: &crate::board_config::TMC_CONFIG,
                };
                foci_firmware::tmc_diagnostics::run_high_rate_capture_request(
                    req, context, &mut sink,
                )
                .await;
                self.disarm_motor(runtime, tmc).await;
            } else {
                self.disarm_motor(runtime, tmc).await;
                foci_firmware::tmc_diagnostics::emit_high_rate_capture_not_ready(req, &mut sink);
            }
            return;
        }

        crate::tmc_diagnostics::run_diagnostic_request(req, self.channel, runtime, tmc).await;
    }

    #[cfg(feature = "dev")]
    async fn run_dev_init(&mut self, tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>) {
        tmc_enter_openloop(tmc, enable_pin(self.enable_pins, self.channel)).await;
    }

    #[cfg(feature = "dev")]
    async fn run_dev_test_spin(
        &mut self,
        tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
        revolutions: u8,
    ) {
        do_test_spin(tmc, revolutions).await;
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
    disposition: TmcPreRequestDisposition,
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

    let mut board = OuroborosRequestBoard::new(
        channel,
        commissioning_blocked,
        enable_pins,
        commission_trace_engine_seq,
    );
    handle_shared_request(req, disposition, runtime, tmc, &mut board).await
}
