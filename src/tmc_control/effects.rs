//! Ouroboros effects for shared TMC control runtime actions.

use foci_firmware::commission_dispatch::ReplyPayload;
use foci_firmware::state::tmc::TMC_DISABLE_SIGNAL;
use foci_firmware::tmc_control::request::TmcPreRequestDisposition;
use foci_firmware::tmc_control::{BoardAction, BoardActions, ShutdownReason, StepperAction};

use crate::helpers::lifecycle::read_tim5;
use crate::helpers::stepper::stepper_stop_from_p1;

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
