//! Ouroboros wiring for the shared TMC request handler: reply sender, delay and channel policy.

use embedded_hal_async::delay::DelayNs;
use foci_firmware::tmc::TmcRequest;
use foci_firmware::tmc_control::dispatch::RegisterRequestError;
use foci_firmware::tmc_control::register_dispatch::{
    AnyTmcChannel, TmcDispatchEnv, handle_tmc_request as handle_shared_tmc_request,
};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;

/// Cooperative delay backed by the RTIC monotonic.
struct MonoDelay;

impl DelayNs for MonoDelay {
    async fn delay_ns(&mut self, ns: u32) {
        Mono::delay(u64::from(ns).div_ceil(1_000).micros()).await;
    }

    async fn delay_us(&mut self, us: u32) {
        Mono::delay(u64::from(us).micros()).await;
    }

    async fn delay_ms(&mut self, ms: u32) {
        Mono::delay(u64::from(ms).millis()).await;
    }
}

/// Handle a TMC request that does not require main-loop state (`motor_enabled`, `enable_pin`).
///
/// `SetEnable`, Init, and `TestSpin` are handled inline by the `tmc_control` main loop.
/// Returns `Ok(())` on success or the reason the caller should enter shutdown.
pub(crate) async fn handle_tmc_request(
    req: TmcRequest,
    tmc: &mut Tmc4671Spi<TmcSpiDevice<'_>>,
) -> Result<(), RegisterRequestError> {
    let mut env = TmcDispatchEnv {
        config: &TMC_CONFIG,
        sender: &mut crate::_ankyra_config::Sender,
        delay: &mut MonoDelay,
        policy: &AnyTmcChannel,
    };
    handle_shared_tmc_request(req, tmc, &mut env).await
}
