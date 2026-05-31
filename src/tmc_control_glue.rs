//! Ouroboros glue for applying shared TMC control runtime actions.

use foci_firmware::commission_dispatch::ReplyPayload;
use foci_firmware::state::tmc::TMC_DISABLE_SIGNAL;
use foci_firmware::tmc_control::{BoardAction, BoardActions, ShutdownReason, StepperAction};

use crate::helpers::lifecycle::read_tim5;
use crate::helpers::stepper::stepper_stop_from_p1;

/// Result of applying pre-request runtime actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ActionDisposition {
    /// No terminal action was applied; continue the existing request match.
    ContinueRequest,
    /// A terminal action consumed the request.
    RequestHandled,
}

/// Apply runtime actions emitted before SPI-heavy request handling.
///
/// Returns [`ActionDisposition::RequestHandled`] if a terminal action was
/// observed. Stepper-only action lists apply their non-terminal effects and
/// return [`ActionDisposition::ContinueRequest`].
pub(crate) fn apply_pre_request_actions<F>(
    channel: u8,
    actions: BoardActions,
    mut enter_shutdown: F,
) -> ActionDisposition
where
    F: FnMut(ShutdownReason),
{
    let mut disposition = ActionDisposition::ContinueRequest;

    for action in actions {
        if apply_action(channel, action, &mut enter_shutdown) {
            disposition = ActionDisposition::RequestHandled;
        }
    }

    disposition
}

fn apply_action<F>(channel: u8, action: BoardAction, enter_shutdown: &mut F) -> bool
where
    F: FnMut(ShutdownReason),
{
    match action {
        BoardAction::Shutdown(reason) => {
            enter_shutdown(reason);
            true
        }
        BoardAction::SignalEmergencyDisable => {
            TMC_DISABLE_SIGNAL.signal(());
            true
        }
        BoardAction::Stepper(StepperAction::None) => false,
        BoardAction::Stepper(StepperAction::StopNow) => {
            stepper_stop_from_p1(channel);
            false
        }
        BoardAction::Stepper(StepperAction::ArmCheck { .. }) => {
            defmt::panic!("ArmCheck requires SPI/backend state and must be handled inline")
        }
        BoardAction::Stepper(StepperAction::ApplyPostAction(_)) => {
            defmt::warn!("TMC runtime post-action requires dispatch state; handled inline");
            false
        }
        BoardAction::Reply(reply) => {
            emit_reply(reply);
            true
        }
        BoardAction::DelegatedDiagnostic(_delegated) => true,
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
