//! Ouroboros TMC control request handling.

use core::sync::atomic::Ordering;

use foci_firmware::autotune::CommissionError;
use foci_firmware::commission_dispatch::{ReplyPayload, StepperPostAction};
use foci_firmware::state::stepper::StepperChannelState;
use foci_firmware::state::tmc::TmcChannelState;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_control::dispatch::RegisterRequestError;
#[cfg(not(feature = "trace-velocity-sweep"))]
use foci_firmware::tmc_control::request::handle_tmc_control_request as handle_shared_request;
#[cfg(feature = "trace-velocity-sweep")]
use foci_firmware::tmc_control::request::handle_tmc_control_request_with_stages;
use foci_firmware::tmc_control::request::{
    MotionScaleMapperInstallError, TmcPreRequestDisposition, TmcRequestBoard,
};
use foci_firmware::tmc_control::{
    BoardActions, TmcControlRequestOutcome, TmcControlRequestTail, TmcControlRuntime,
};
use foci_tmc4671::spi::Tmc4671Spi;

use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CHANNEL_COUNT;
#[cfg(feature = "dev")]
use crate::helpers::dev::{do_test_spin, tmc_enter_openloop};
use crate::helpers::stepper::apply_stepper_post_action;
use crate::state::stepper::{
    isr_stepper, motion_scale_install_admitted, replace_motion_scale_mapper,
    reset_mapper_observables, step_queue,
};
use crate::tmc_control::board::{
    TmcEnablePins, TmcRuntimes, enable_pin, next_commission_trace_seq, ouroboros_backend,
    stepper_state, tmc_state,
};
use crate::tmc_dispatch::handle_tmc_request;
use crate::trigger_domain::with_trigger_domain;

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

    fn default_current_filter_hz(&self) -> u16 {
        crate::board_config::TMC_CONFIG.default_current_filter_hz()
    }

    fn pwm_freq_hz(&self) -> u32 {
        crate::board_config::TMC_CONFIG.pwm_freq_hz()
    }

    fn velocity_limit_evidence_capability(&self) -> u32 {
        crate::ankyra::constants::VELOCITY_LIMIT_EVIDENCE_CAPABILITY
    }

    fn motion_scale_queue_empty(&self) -> bool {
        // SAFETY: The TMC request runs at P1 and only observes queue state.
        unsafe { step_queue(self.channel).is_some_and(|queue| queue.is_empty()) }
    }

    fn install_motion_scale_mapper(
        &mut self,
        mapper: foci_klipper::pulse_mapper::PulseMapperConfig,
    ) -> Result<(), MotionScaleMapperInstallError> {
        with_trigger_domain(|td| {
            let motor_armed = tmc_state(self.channel).motor_armed.load(Ordering::Relaxed);
            let queue_empty =
                unsafe { step_queue(self.channel).is_some_and(|queue| queue.is_empty()) };
            let timer_active = td.timer_active(self.channel as usize);
            let admitted = motion_scale_install_admitted(motor_armed, queue_empty, timer_active);

            // SAFETY: The trigger-domain lock excludes P2/P3, and the checked
            // disarmed/empty/inactive state keeps this channel quiesced afterward.
            let stepper =
                unsafe { isr_stepper(self.channel).ok_or(MotionScaleMapperInstallError)? };
            reset_mapper_observables(self.channel);
            replace_motion_scale_mapper(stepper, mapper, admitted)
                .map_err(|_| MotionScaleMapperInstallError)
        })
    }

    fn report_motion_scale_rejection(
        &mut self,
        oid: u8,
        rejection: foci_firmware::motion_scale::MotionScaleRejection,
    ) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociMotionScaleRejected,
            oid: u8 = oid,
            stage: u8 = foci_firmware::motion_scale::MOTION_SCALE_REJECTION_STAGE_P1,
            reason: u8 = rejection.code(),
        );
    }

    fn emit_reply(&mut self, payload: ReplyPayload) {
        crate::ankyra::replies::emit_terminal_reply(payload);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        &mut self,
        record: foci_firmware::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        crate::usb_trace::request_velocity_sweep_evidence(self.channel, record);
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
    ) -> Result<(), RegisterRequestError> {
        handle_tmc_request(req, tmc).await
    }

    async fn run_diagnostic_request(
        &mut self,
        req: TmcRequest,
        runtime: &mut TmcControlRuntime,
        tmc: &mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    ) {
        crate::tmc_diagnostics::run_diagnostic_request(
            req,
            self.channel,
            runtime,
            tmc,
            enable_pin(self.enable_pins, self.channel),
        )
        .await;
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
    #[cfg(feature = "trace-velocity-sweep")]
    pub(crate) stage_c_candidate: &'a mut foci_firmware::commissioning::outer::velocity::integral_reproduction::StageCCandidateSlot,
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
        #[cfg(feature = "trace-velocity-sweep")]
        stage_c_candidate,
    } = context;
    let runtime = &mut runtimes[channel as usize];

    let mut board = OuroborosRequestBoard::new(
        channel,
        commissioning_blocked,
        enable_pins,
        commission_trace_engine_seq,
    );
    #[cfg(feature = "trace-velocity-sweep")]
    return handle_tmc_control_request_with_stages(
        req,
        disposition,
        runtime,
        stage_c_candidate,
        tmc,
        &mut board,
    )
    .await;
    #[cfg(not(feature = "trace-velocity-sweep"))]
    handle_shared_request(req, disposition, runtime, tmc, &mut board).await
}
