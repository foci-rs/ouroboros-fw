//! `CommissioningBackend` impl for Ouroboros.
//!
//! Wraps the TMC4671 SPI driver, enable pin, and `Mono` timer so the
//! board-agnostic [`foci_firmware::commission::CommissioningEngine`] can
//! perform register I/O, pin control, and timing.

use core::sync::atomic::Ordering::Relaxed;

use embassy_stm32::gpio::Output;
use foci_tmc4671::{registers::ABN_DECODER_COUNT, spi::Tmc4671Spi};
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;
use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TmcChannelState};

pub(crate) struct OuroborosBackend<'a, 'spi> {
    pub(crate) tmc: &'a mut Tmc4671Spi<TmcSpiDevice<'spi>>,
    /// Channel-selected TMC4671 ENABLE output.
    pub(crate) enable_pin: &'a mut Output<'static>,
    pub(crate) tmc_state: &'static TmcChannelState,
}

impl<'a, 'spi> foci_firmware::commission::CommissioningBackend for OuroborosBackend<'a, 'spi> {
    type Error = foci_tmc4671::spi::SpiError<
        embedded_hal_bus::spi::DeviceError<embassy_stm32::spi::Error, core::convert::Infallible>,
    >;

    async fn read_register(
        &mut self,
        reg: foci_tmc4671::registers::Register,
    ) -> Result<u32, Self::Error> {
        self.tmc.read_register(reg).await
    }

    async fn write_register(
        &mut self,
        reg: foci_tmc4671::registers::Register,
        val: u32,
    ) -> Result<(), Self::Error> {
        self.tmc.write_register(reg, val).await
    }

    fn set_enable_pin(&mut self, state: bool) {
        if state {
            self.enable_pin.set_high();
        } else {
            self.enable_pin.set_low();
        }
    }

    async fn read_encoder_count(&mut self) -> u32 {
        self.tmc.read_register(ABN_DECODER_COUNT).await.unwrap_or(0)
    }

    async fn try_read_encoder_count(&mut self) -> Result<u32, Self::Error> {
        self.tmc.read_register(ABN_DECODER_COUNT).await
    }

    fn elapsed_us(&self) -> u64 {
        // Microseconds since Mono start. Returned as u64 so deadline
        // comparisons in commission.rs never wrap across the 51-second
        // TIM5 boundary that previously broke the engine.
        //
        // Source: TIM2 monotonic at 1 MHz (1 us resolution).
        Mono::now().duration_since_epoch().to_micros()
    }

    fn is_shutdown_requested(&self) -> bool {
        if TMC_DISABLE_SIGNAL.try_take().is_some() {
            TMC_DISABLE_SIGNAL.signal(());
            true
        } else {
            false
        }
    }

    fn current_limit(&self) -> u32 {
        self.tmc_state.current_limit_reg.load(Relaxed)
    }

    fn pwm_maxcnt(&self) -> u16 {
        TMC_CONFIG.pwm_maxcnt
    }

    fn bbm_register(&self) -> u32 {
        TMC_CONFIG.bbm_register()
    }

    fn dsadc_mdec_register(&self) -> u32 {
        TMC_CONFIG.dsadc_mdec_register()
    }

    fn excitation_voltage(&self) -> u16 {
        TMC_CONFIG.excitation_voltage
    }

    fn tmc_config(&self) -> &foci_firmware::config::TmcConfig {
        &TMC_CONFIG
    }

    fn step_width(&self) -> u32 {
        self.tmc_state.step_width_value.load(Relaxed).max(0) as u32
    }

    fn operational_velocity_limit(&self) -> u32 {
        self.tmc_state.velocity_limit_cfg.load(Relaxed)
    }

    fn operational_voltage_limit(&self) -> u32 {
        self.tmc_state.voltage_limit_cfg.load(Relaxed)
    }

    async fn delay_us(&mut self, us: u32) {
        Mono::delay((us as u64).micros()).await;
    }
}
