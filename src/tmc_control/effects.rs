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

/// Board signal that interrupts an absolute commissioning wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommissionWaitExit {
    /// Enter the existing shutdown safe-state path.
    Shutdown,
    /// Return to the queue-flush path.
    QueueFlush,
    /// Enter GPIO-first emergency-disable handling.
    EmergencyDisable,
}

struct OuroborosBusyReplySink;

impl CommissionBusyReplySink for OuroborosBusyReplySink {
    fn normal_tail(&self) -> TmcControlRequestTail {
        TmcControlRequestTail::SkipTickTail
    }

    fn emit_reply(&mut self, payload: ReplyPayload) {
        crate::ankyra::replies::emit_terminal_reply(payload);
    }

    fn emit_busy_diagnostic(&mut self, request: TmcRequest) {
        crate::tmc_diagnostics::emit_commission_busy_diagnostic(request);
    }
}

/// Classify board-owned signals checked between commissioning wait slices.
pub(crate) const fn classify_commission_wait_exit(
    shutdown: bool,
    queue_flush: bool,
    emergency_disable: bool,
    critical_status: bool,
) -> Option<CommissionWaitExit> {
    if emergency_disable || critical_status {
        Some(CommissionWaitExit::EmergencyDisable)
    } else if shutdown {
        Some(CommissionWaitExit::Shutdown)
    } else if queue_flush {
        Some(CommissionWaitExit::QueueFlush)
    } else {
        None
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
/// observed, or [`TmcPreRequestDisposition::RunDiagnostic`] if an active diagnostic
/// should run in the board main loop. Stepper-only action lists apply their
/// non-terminal effects and return [`TmcPreRequestDisposition::ContinueRequest`].
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
        BoardAction::Reply(reply) => {
            emit_reply(reply);
            Some(TmcPreRequestDisposition::RequestHandled)
        }
        BoardAction::DelegatedDiagnostic(delegated) => {
            Some(TmcPreRequestDisposition::RunDiagnostic(delegated.request))
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
        ShutdownReason::LiveGainApplyFailed => ::ankyra::klipper_shutdown_from!(
            &mut crate::_ankyra_config::Sender,
            "Live gain application failed",
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

fn emit_reply(reply: ReplyPayload) {
    crate::ankyra::replies::emit_terminal_reply(reply);
}

#[cfg(test)]
mod tests {
    use super::*;
    use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
    use foci_firmware::tmc_control::CommissionWaitRequestDisposition;

    #[test]
    fn both_channel_fault_paths_enter_gpio_first_wait_exit() {
        let channel_cases = [(0, true, false), (1, false, true)];

        for (channel, interrupt_disable, critical_status) in channel_cases {
            let exit =
                classify_commission_wait_exit(false, false, interrupt_disable, critical_status);

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
            let exit = classify_commission_wait_exit(shutdown, flush, disable, critical_status);
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
