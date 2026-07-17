//! Ouroboros commissioning backend hooks.
//!
//! Board-specific hooks provide `Mono`, shutdown signal, configuration, and
//! trace integration to the shared firmware commissioning backend core.

use embassy_stm32::gpio::Output;
use foci_firmware::commissioning::backend::{CommissionBackendHooks, SharedCommissioningBackend};
use foci_firmware::config::TmcConfig;
use foci_firmware::state::tmc::TMC_DISABLE_SIGNAL;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;

pub(crate) struct OuroborosHooks;

impl CommissionBackendHooks for OuroborosHooks {
    fn commission_tmc_config() -> &'static TmcConfig {
        &TMC_CONFIG
    }

    fn commission_elapsed_us() -> u64 {
        // Microseconds since Mono start. Returned as u64 so deadline
        // comparisons in commission.rs never wrap across the 51-second
        // TIM5 boundary that previously broke the engine.
        //
        // Source: TIM2 monotonic at 1 MHz (1 us resolution).
        Mono::now().duration_since_epoch().to_micros()
    }

    fn commission_is_shutdown_requested() -> bool {
        if TMC_DISABLE_SIGNAL.try_take().is_some() {
            TMC_DISABLE_SIGNAL.signal(());
            true
        } else {
            false
        }
    }

    async fn commission_delay_us(us: u32) {
        Mono::delay((us as u64).micros()).await;
    }

    #[cfg(feature = "trace")]
    fn record_commission_trace_event(
        channel: u8,
        event: foci_firmware::trace::events::CommissionTraceEvent,
    ) {
        crate::usb_trace::request_commission_event(channel, event);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        channel: u8,
        observation: foci_firmware::trace::stream::VelocityTraceObservation,
    ) {
        crate::usb_trace::request_velocity_observation(channel, observation);
    }

    #[cfg(feature = "trace-velocity-sweep")]
    fn record_velocity_sweep_trace_record(
        channel: u8,
        record: foci_firmware::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        crate::usb_trace::request_velocity_sweep_evidence(channel, record);
    }
}

pub(crate) type OuroborosBackend<'a, 'spi> =
    SharedCommissioningBackend<'a, TmcSpiDevice<'spi>, Output<'static>, OuroborosHooks>;
