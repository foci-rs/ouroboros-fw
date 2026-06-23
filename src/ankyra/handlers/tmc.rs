//! Board-local shim wrapping the shared impedance-test handler.

use ankyra::prelude::klipper_command;

use crate::firmware_ctx::FirmwareCtx;

/// Queue the shared impedance-identification diagnostic for an already-armed motor.
#[klipper_command]
pub fn tmc_impedance_test(ctx: &mut FirmwareCtx<'_>, oid: u8, detail: u8) {
    foci_firmware::ankyra::handlers::tmc::tmc_impedance_test(
        ctx,
        &mut crate::_ankyra_config::Sender,
        oid,
        detail,
    );
}
