//! Handle `TmcRequest` messages that don't need main-loop motor state.
//!
//! Returned from the P2 protocol handler via the tmc-control channel.
//! `SetEnable`, `Init`, `TestSpin`, `SetPidGains`, `SetPositionGains`,
//! `Commission`, `Calibrate`, and `Tune` remain in the tmc_control main
//! loop because they mutate `motor_enabled` / `enable_pin` / engine
//! state. Everything else — SetCurrent / SetEncoder / SetEncoderDir /
//! DumpRegisters, and the `#[cfg(feature = "dev")]` Read / Write pair —
//! dispatches here.

use foci_firmware::tmc::{TmcRequest, TmcRequestKind};
use foci_firmware::tmc_control::dispatch::{
    RegisterRequestError, SharedTmcDispatch, apply_shared_tmc_request,
};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;

fn report_invalid_channel(channel: u8) {
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        foci_firmware::ankyra::replies::FociDiagnostic,
        fault: u8 = foci_firmware::diagnostic::FaultClass::InvalidChannel as u8,
        operation: u8 = foci_firmware::diagnostic::operation::TMC_CONTROL,
        oid: u8 = 255,
        channel: u8 = channel,
        data0: u32 = 0,
        data1: u32 = 0,
    );
}

/// Handle a TMC request that does not require main-loop state (motor_enabled, enable_pin).
///
/// SetEnable, Init, and TestSpin are handled inline by the tmc_control main loop.
/// Returns `Ok(())` on success or the reason the caller should enter shutdown.
pub(crate) async fn handle_tmc_request(
    req: TmcRequest,
    tmc: &mut Tmc4671Spi<TmcSpiDevice<'_>>,
) -> Result<(), RegisterRequestError> {
    let channel = req.channel;
    let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
        defmt::error!("Invalid TMC channel {}", channel);
        report_invalid_channel(channel);
        return Err(RegisterRequestError::Other);
    };

    let kind = req.kind;
    match apply_shared_tmc_request(
        &kind,
        tmc,
        tmc_state,
        TMC_CONFIG.current_scale_ma_per_lsb,
        TMC_CONFIG.pwm_maxcnt,
    )
    .await?
    {
        SharedTmcDispatch::Handled => return Ok(()),
        SharedTmcDispatch::Unhandled => {}
    }

    match kind {
        #[cfg(feature = "dev")]
        TmcRequestKind::Write { addr, value } => {
            let reg = foci_tmc4671::registers::Register::new(addr);
            if tmc.write_register(reg, value).await.is_err() {
                defmt::error!("TMC4671 SPI write failed: addr={:#04x}", addr);
            }
        }
        #[cfg(feature = "dev")]
        TmcRequestKind::Read { oid, addr } => {
            let reg = foci_tmc4671::registers::Register::new(addr);
            match tmc.read_register(reg).await {
                Ok(value) => {
                    ::ankyra::klipper_reply_from!(
                        &mut crate::_ankyra_config::Sender,
                        foci_firmware::ankyra::replies::TmcRegisterValue,
                        oid: u8 = oid,
                        addr: u8 = addr,
                        value: u32 = value,
                    );
                }
                Err(_) => {
                    defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                }
            }
        }
        TmcRequestKind::CurrentStepTest { .. } => {
            defmt::warn!("handle_tmc_request: CurrentStepTest handled by main loop");
        }
        TmcRequestKind::CurrentVectorStepTest { .. } => {
            defmt::warn!("handle_tmc_request: CurrentVectorStepTest handled by main loop");
        }
        TmcRequestKind::CurrentTorqueSampleTest { .. } => {
            defmt::warn!("handle_tmc_request: CurrentTorqueSampleTest handled by main loop");
        }
        TmcRequestKind::PositionTorqueOffsetSampleTest { .. } => {
            defmt::warn!("handle_tmc_request: PositionTorqueOffsetSampleTest handled by main loop");
        }
        TmcRequestKind::VoltageStepTest { .. } => {
            defmt::warn!("handle_tmc_request: VoltageStepTest handled by main loop");
        }
        TmcRequestKind::SelfTest { .. } => {
            defmt::warn!("handle_tmc_request: SelfTest handled by main loop");
        }
        TmcRequestKind::DumpRegisters { oid } => {
            for &addr in foci_firmware::dump::DUMP_REGISTERS {
                Mono::delay(1.millis()).await;

                let reg = foci_tmc4671::registers::Register::new(addr);
                let value = match tmc.read_register_blocking(reg) {
                    Ok(v) => v,
                    Err(_) => {
                        defmt::error!("TMC4671 SPI read failed: addr={:#04x}", addr);
                        0
                    }
                };
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociDumpValue,
                    oid: u8 = oid,
                    addr: u8 = addr,
                    value: u32 = value,
                );
            }
            for &(synthetic_addr, sub_reg) in foci_firmware::dump::DUMP_SUB_REGISTERS {
                Mono::delay(1.millis()).await;

                let value = match tmc.read_sub_register_blocking(sub_reg) {
                    Ok(v) => v,
                    Err(_) => {
                        defmt::error!(
                            "TMC4671 sub-register read failed: addr={:#04x}",
                            synthetic_addr
                        );
                        0
                    }
                };
                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociDumpValue,
                    oid: u8 = oid,
                    addr: u8 = synthetic_addr,
                    value: u32 = value,
                );
            }
            let clamp_values = foci_firmware::dump::clamp_dump_values(
                tmc_state.velocity_clamp_latched(),
                tmc_state.velocity_clamp_count(),
            );
            for &(addr, value) in &clamp_values {
                Mono::delay(1.millis()).await;

                ::ankyra::klipper_reply_from!(
                    &mut crate::_ankyra_config::Sender,
                    foci_firmware::ankyra::replies::FociDumpValue,
                    oid: u8 = oid,
                    addr: u8 = addr,
                    value: u32 = value,
                );
            }
            Mono::delay(1.millis()).await;
            ::ankyra::klipper_reply_from!(
                &mut crate::_ankyra_config::Sender,
                foci_firmware::ankyra::replies::FociDumpDone,
                oid: u8 = oid,
            );
        }
        TmcRequestKind::SetPidGains { .. } => {
            defmt::warn!("handle_tmc_request: SetPidGains handled by main loop");
        }
        TmcRequestKind::SetEnable(_) => {
            defmt::warn!("handle_tmc_request: SetEnable handled by main loop");
        }
        TmcRequestKind::Calibrate { .. } => {
            defmt::warn!("handle_tmc_request: Calibrate handled by main loop");
        }
        TmcRequestKind::SetPositionGains { .. } => {
            defmt::warn!("handle_tmc_request: SetPositionGains handled by main loop");
        }
        TmcRequestKind::Commission { .. } => {
            defmt::warn!("handle_tmc_request: Commission handled by main loop");
        }
        TmcRequestKind::Tune { .. } => {
            defmt::warn!("handle_tmc_request: Tune handled by main loop");
        }
        #[cfg(feature = "dev")]
        TmcRequestKind::Init => {
            defmt::warn!("handle_tmc_request: Init handled by main loop");
        }
        #[cfg(feature = "dev")]
        TmcRequestKind::TestSpin { .. } => {
            defmt::warn!("handle_tmc_request: TestSpin handled by main loop");
        }
        _ => {
            defmt::error!("handle_tmc_request: unhandled request kind after shared dispatch");
            unreachable!("shared TMC dispatch coverage mismatch");
        }
    }
    Ok(())
}
