//! `CommissioningBackend` impl for Ouroboros.
//!
//! Wraps the TMC4671 SPI driver, enable pin, and `Mono` timer so the
//! board-agnostic [`foci_firmware::commission::CommissioningEngine`] can
//! perform register I/O, pin control, and timing.

use embassy_stm32::gpio::Output;
use foci_firmware::commissioning::backend::{
    CommissionConfigAccess, CommissionEnablePinAccess, CommissionTimingAccess,
    CommissionTmcSpiAccess, CommissionTmcStateAccess, CommissionTraceAccess,
};
use foci_firmware::config::TmcConfig;
use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TmcChannelState};
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;

pub(crate) struct OuroborosBackend<'a, 'spi> {
    #[cfg_attr(not(feature = "trace"), allow(dead_code))]
    pub(crate) channel: u8,
    pub(crate) tmc: &'a mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    /// Channel-selected TMC4671 ENABLE output.
    pub(crate) enable_pin: &'a mut Output<'static>,
    pub(crate) tmc_state: &'static TmcChannelState,
}

impl<'a, 'spi> CommissionTmcSpiAccess for OuroborosBackend<'a, 'spi> {
    type Spi = TmcSpiDevice<'spi>;

    fn tmc_spi(&mut self) -> &mut Tmc4671Spi<Self::Spi> {
        self.tmc
    }
}

impl<'a, 'spi> CommissionTmcStateAccess for OuroborosBackend<'a, 'spi> {
    fn tmc_state(&self) -> &'static TmcChannelState {
        self.tmc_state
    }
}

impl<'a, 'spi> CommissionEnablePinAccess for OuroborosBackend<'a, 'spi> {
    fn set_commission_enable_pin(&mut self, state: bool) {
        if state {
            self.enable_pin.set_high();
        } else {
            self.enable_pin.set_low();
        }
    }
}

impl<'a, 'spi> CommissionConfigAccess for OuroborosBackend<'a, 'spi> {
    fn commission_tmc_config(&self) -> &'static TmcConfig {
        &TMC_CONFIG
    }
}

impl<'a, 'spi> CommissionTimingAccess for OuroborosBackend<'a, 'spi> {
    fn commission_elapsed_us(&self) -> u64 {
        // Microseconds since Mono start. Returned as u64 so deadline
        // comparisons in commission.rs never wrap across the 51-second
        // TIM5 boundary that previously broke the engine.
        //
        // Source: TIM2 monotonic at 1 MHz (1 us resolution).
        Mono::now().duration_since_epoch().to_micros()
    }

    fn commission_is_shutdown_requested(&self) -> bool {
        if TMC_DISABLE_SIGNAL.try_take().is_some() {
            TMC_DISABLE_SIGNAL.signal(());
            true
        } else {
            false
        }
    }

    async fn commission_delay_us(&mut self, us: u32) {
        Mono::delay((us as u64).micros()).await;
    }
}

impl<'a, 'spi> CommissionTraceAccess for OuroborosBackend<'a, 'spi> {
    #[cfg(feature = "trace")]
    fn record_commission_trace_event_impl(
        &mut self,
        event: foci_firmware::trace::events::CommissionTraceEvent,
    ) {
        crate::usb_trace::request_commission_event(self.channel, event);
    }
}
