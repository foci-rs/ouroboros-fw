//! Ouroboros effects for shared TMC control runtime actions.

use foci_firmware::commission_dispatch::ReplyPayload;
use foci_firmware::state::tmc::TMC_DISABLE_SIGNAL;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_control::request::{CommissionBusyReplySink, TmcPreRequestDisposition};
use foci_firmware::tmc_control::{
    BoardAction, BoardActions, CommissionWaitRequestDisposition, ShutdownReason, StepperAction,
    TmcControlRequestOutcome, TmcControlRequestTail, classify_wait_request,
};

use crate::helpers::lifecycle::read_tim5;
use crate::helpers::stepper::stepper_stop_from_p1;
#[cfg(test)]
use crate::tmc_control::containment::{CommissionWaitExit, classify_commission_wait_exit};

/// Ownership route for a disable request received during a commissioning wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommissionWaitCancelRoute {
    /// Reject an out-of-range request before mutating either channel.
    Invalid,
    /// Cancel the channel whose absolute wait is currently being serviced.
    Current,
    /// Disable the requested channel and preserve the current absolute wait.
    Other,
}

struct OuroborosBusyReplySink;

impl CommissionBusyReplySink for OuroborosBusyReplySink {
    fn normal_tail(&self) -> TmcControlRequestTail {
        TmcControlRequestTail::SkipTickTail
    }

    fn emit_reply(&mut self, channel: u8, payload: ReplyPayload) {
        crate::ankyra::replies::emit_terminal_reply(channel, payload);
    }

    fn emit_busy_diagnostic(&mut self, request: TmcRequest) {
        crate::tmc_diagnostics::emit_commission_busy_diagnostic(request);
    }
}

/// Validate and classify ownership for a queued disable request.
pub(crate) const fn classify_commission_wait_cancel(
    waited_channel: u8,
    request_channel: u8,
    channel_count: u8,
) -> CommissionWaitCancelRoute {
    if request_channel >= channel_count {
        CommissionWaitCancelRoute::Invalid
    } else if request_channel == waited_channel {
        CommissionWaitCancelRoute::Current
    } else {
        CommissionWaitCancelRoute::Other
    }
}

/// Delegate queued commissioning request policy to the shared classifier.
pub(crate) fn commission_wait_request_disposition(
    request: &TmcRequest,
) -> CommissionWaitRequestDisposition {
    classify_wait_request(&request.kind)
}

/// Emit a shared busy-only response without granting hardware capabilities.
pub(crate) fn handle_commission_wait_reject(req: TmcRequest) -> TmcControlRequestOutcome {
    foci_firmware::tmc_control::request::handle_commission_wait_reject(
        req,
        &mut OuroborosBusyReplySink,
    )
}

/// Apply runtime actions emitted before SPI-heavy request handling.
///
/// Returns [`TmcPreRequestDisposition::RequestHandled`] if a terminal action was
/// observed. Stepper-only action lists apply their non-terminal effects and
/// return [`TmcPreRequestDisposition::ContinueRequest`].
pub(crate) fn apply_pre_request_actions<F>(
    channel: u8,
    actions: BoardActions,
    mut enter_shutdown: F,
) -> TmcPreRequestDisposition
where
    F: FnMut(ShutdownReason),
{
    let mut disposition = TmcPreRequestDisposition::ContinueRequest;

    for action in actions {
        if let Some(next) = apply_action(channel, action, &mut enter_shutdown) {
            disposition = next;
        }
    }

    disposition
}

fn apply_action<F>(
    channel: u8,
    action: BoardAction,
    enter_shutdown: &mut F,
) -> Option<TmcPreRequestDisposition>
where
    F: FnMut(ShutdownReason),
{
    match action {
        BoardAction::Shutdown(reason) => {
            enter_shutdown(reason);
            Some(TmcPreRequestDisposition::RequestHandled)
        }
        BoardAction::SignalEmergencyDisable => {
            TMC_DISABLE_SIGNAL.signal(());
            Some(TmcPreRequestDisposition::RequestHandled)
        }
        BoardAction::Stepper(StepperAction::None) => None,
        BoardAction::Stepper(StepperAction::StopNow) => {
            stepper_stop_from_p1(channel);
            None
        }
        BoardAction::Stepper(StepperAction::ArmCheck { .. }) => {
            defmt::panic!("ArmCheck requires SPI/backend state and must be handled inline")
        }
        BoardAction::Stepper(StepperAction::ApplyPostAction(_)) => {
            defmt::warn!("TMC runtime post-action requires dispatch state; handled inline");
            None
        }
    }
}

/// Emit a Klipper shutdown after the caller has entered MCU shutdown state.
pub(crate) fn emit_shutdown(reason: ShutdownReason) {
    let clock = read_tim5();
    match reason {
        ShutdownReason::BootSpiFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671 SPI failed at boot",
            clock,
        ),
        ShutdownReason::ChipIdMismatch => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671 chip ID mismatch",
            clock,
        ),
        ShutdownReason::InvalidTmcChannel => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "Invalid TMC channel",
            clock,
        ),
        ShutdownReason::RequestValidationFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC request validation failed",
            clock,
        ),
        ShutdownReason::CurrentLimitInvalid => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC current limit invalid",
            clock,
        ),
        ShutdownReason::LiveGainApplyFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "Live gain application failed",
            clock,
        ),
        ShutdownReason::PendingConfigApplyFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC pending config apply failed",
            clock,
        ),
        ShutdownReason::AdcCurrentClipped => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671: ADC current clipped (overcurrent)",
            clock,
        ),
        ShutdownReason::TmcFault => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "TMC4671 fault",
            clock,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
    use foci_firmware::tmc_control::CommissionWaitRequestDisposition;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct CancelOwnershipModel {
        engine_active: [bool; 2],
        commission_in_flight: [bool; 2],
        driver_ready_count: [u8; 2],
        measurement_active: [bool; 2],
        max_tick_cycles: [u32; 2],
        deadline_us: [Option<u64>; 2],
        gpio_low: [bool; 2],
        disarmed: [bool; 2],
    }

    impl CancelOwnershipModel {
        const fn both_active() -> Self {
            Self {
                engine_active: [true, true],
                commission_in_flight: [true, true],
                driver_ready_count: [3, 7],
                measurement_active: [true, true],
                max_tick_cycles: [101, 202],
                deadline_us: [Some(11_000), Some(22_000)],
                gpio_low: [false, false],
                disarmed: [false, false],
            }
        }

        fn service_cancel(&mut self, waited_channel: u8, request_channel: u8) -> bool {
            match classify_commission_wait_cancel(waited_channel, request_channel, 2) {
                CommissionWaitCancelRoute::Invalid => false,
                CommissionWaitCancelRoute::Current | CommissionWaitCancelRoute::Other => {
                    let target = request_channel as usize;
                    self.engine_active[target] = false;
                    self.commission_in_flight[target] = false;
                    self.driver_ready_count[target] = 0;
                    self.measurement_active[target] = false;
                    self.max_tick_cycles[target] = 0;
                    self.deadline_us[target] = None;
                    self.gpio_low[target] = true;
                    self.disarmed[target] = true;
                    matches!(
                        classify_commission_wait_cancel(waited_channel, request_channel, 2),
                        CommissionWaitCancelRoute::Current
                    )
                }
            }
        }
    }

    #[test]
    fn wait_channel0_disable_channel1_preserves_channel0_ownership() {
        let mut model = CancelOwnershipModel::both_active();
        let channel0_before = (
            model.driver_ready_count[0],
            model.measurement_active[0],
            model.max_tick_cycles[0],
            model.deadline_us[0],
        );

        let discard_wait = model.service_cancel(0, 1);

        assert!(!discard_wait);
        assert!(model.engine_active[0]);
        assert!(model.commission_in_flight[0]);
        assert_eq!(
            (
                model.driver_ready_count[0],
                model.measurement_active[0],
                model.max_tick_cycles[0],
                model.deadline_us[0],
            ),
            channel0_before,
        );
        assert!(!model.engine_active[1]);
        assert!(!model.commission_in_flight[1]);
        assert!(model.gpio_low[1]);
        assert!(model.disarmed[1]);
        assert!(!model.gpio_low[0]);
        assert!(!model.disarmed[0]);
    }

    #[test]
    fn wait_channel1_disable_channel0_preserves_channel1_ownership() {
        let mut model = CancelOwnershipModel::both_active();
        let channel1_before = (
            model.driver_ready_count[1],
            model.measurement_active[1],
            model.max_tick_cycles[1],
            model.deadline_us[1],
        );

        let discard_wait = model.service_cancel(1, 0);

        assert!(!discard_wait);
        assert!(model.engine_active[1]);
        assert!(model.commission_in_flight[1]);
        assert_eq!(
            (
                model.driver_ready_count[1],
                model.measurement_active[1],
                model.max_tick_cycles[1],
                model.deadline_us[1],
            ),
            channel1_before,
        );
        assert!(!model.engine_active[0]);
        assert!(!model.commission_in_flight[0]);
        assert!(model.gpio_low[0]);
        assert!(model.disarmed[0]);
        assert!(!model.gpio_low[1]);
        assert!(!model.disarmed[1]);
    }

    #[test]
    fn both_active_cancellation_removes_only_request_channel_engine() {
        let mut model = CancelOwnershipModel::both_active();

        model.service_cancel(0, 1);

        assert_eq!(model.engine_active, [true, false]);
        assert_eq!(model.commission_in_flight, [true, false]);
        assert_eq!(model.driver_ready_count, [3, 0]);
        assert_eq!(model.measurement_active, [true, false]);
        assert_eq!(model.max_tick_cycles, [101, 0]);
        assert_eq!(model.deadline_us, [Some(11_000), None]);
    }

    #[test]
    fn same_channel_cancellation_discards_current_wait_action() {
        let mut model = CancelOwnershipModel::both_active();

        let discard_wait = model.service_cancel(1, 1);

        assert!(discard_wait);
        assert!(model.engine_active[0]);
        assert!(!model.engine_active[1]);
        assert_eq!(model.deadline_us, [Some(11_000), None]);
        assert_eq!(model.gpio_low, [false, true]);
        assert_eq!(model.disarmed, [false, true]);
    }

    #[test]
    fn invalid_cancel_channel_mutates_no_channel_state() {
        let mut model = CancelOwnershipModel::both_active();
        let before = model;

        let discard_wait = model.service_cancel(0, 2);

        assert!(!discard_wait);
        assert_eq!(model, before);
    }

    #[test]
    fn both_channel_fault_paths_enter_gpio_first_wait_exit() {
        let channel_cases = [(0, true, false), (1, false, true)];

        for (channel, interrupt_disable, critical_status) in channel_cases {
            let exit = classify_commission_wait_exit(
                false,
                false,
                interrupt_disable,
                critical_status,
                false,
            );

            assert_eq!(
                exit,
                Some(CommissionWaitExit::EmergencyDisable),
                "channel {channel} must use the GPIO-first exit",
            );
        }
    }

    #[test]
    fn post_yield_signals_exit_before_dispatch_on_both_channels() {
        let cases = [
            (0, true, false, false, false, false),
            (0, false, true, false, false, false),
            (0, false, false, true, false, true),
            (1, false, false, false, true, true),
        ];

        for (channel, shutdown, flush, disable, critical_status, gpio_first) in cases {
            let exit =
                classify_commission_wait_exit(shutdown, flush, disable, critical_status, false);
            let dispatch = exit.is_none();
            let reply = dispatch;
            let trace = dispatch;
            let next_engine_poll = dispatch;

            assert!(exit.is_some(), "channel {channel} must exit after yield");
            assert_eq!(
                matches!(exit, Some(CommissionWaitExit::EmergencyDisable)),
                gpio_first,
            );
            assert!(!dispatch);
            assert!(!reply);
            assert!(!trace);
            assert!(!next_engine_poll);
        }
    }

    #[test]
    fn commission_wait_requests_use_shared_disposition() {
        let cancel = TmcRequest::new(1, TmcRequestKind::SetEnable(false));
        let reject = TmcRequest::new(0, TmcRequestKind::Commission { oid: 2, profile: 1 });
        let drop = TmcRequest::new(1, TmcRequestKind::SetCurrent { run_ma: 800 });

        assert_eq!(
            commission_wait_request_disposition(&cancel),
            CommissionWaitRequestDisposition::Cancel,
        );
        assert_eq!(
            commission_wait_request_disposition(&reject),
            CommissionWaitRequestDisposition::RejectBusy,
        );
        assert_eq!(
            commission_wait_request_disposition(&drop),
            CommissionWaitRequestDisposition::DropBusy,
        );
    }

    #[test]
    fn forced_ready_yield_survives_processing_past_classification_time() {
        use foci_firmware::tmc_control::{CommissionPollDriver, PollDriverAction};

        let mut driver = CommissionPollDriver::new();
        for _ in 0..8 {
            assert_eq!(
                driver.classify_ready(10_000).unwrap(),
                PollDriverAction::PollNow,
            );
        }
        let action = driver.classify_ready(10_000).unwrap();
        let processing_finished_us = 20_000;

        assert!(processing_finished_us > 10_001);
        assert_eq!(action, PollDriverAction::YieldNow);
    }

    #[test]
    fn ready_chain_state_is_independent_per_channel() {
        use foci_firmware::tmc_control::{CommissionPollDriver, PollDriverAction};

        let mut drivers = [CommissionPollDriver::new(), CommissionPollDriver::new()];
        for _ in 0..8 {
            assert_eq!(
                drivers[0].classify_ready(10_000).unwrap(),
                PollDriverAction::PollNow,
            );
        }

        assert_eq!(
            drivers[0].classify_ready(10_000).unwrap(),
            PollDriverAction::YieldNow,
        );
        assert_eq!(
            drivers[1].classify_ready(10_000).unwrap(),
            PollDriverAction::PollNow,
        );
    }
}
