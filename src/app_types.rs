//! Board-level type aliases and shared helper types.
//!
//! Items that are naturally `mod app`-scoped but do not need to live
//! next to the RTIC task graph are collected here so `main.rs` stays
//! focused on the RTIC wiring. The `Pins` wire enumeration moved to
//! `crate::ankyra::enumerations` during the ankyra migration (Task 8a).

use embassy_stm32::mode::Async as AsyncMode;
use embassy_stm32::spi::Spi;

/// SPI bus type for TMC4671 communication.
pub type TmcSpiBus = Spi<'static, AsyncMode, embassy_stm32::spi::mode::Master>;

/// Sub-microsecond busy-wait delay for SPI read pauses.
///
/// The TMC4671-LA requires a 500 ns pause after the address byte for
/// SPI reads above 2 MHz (datasheet Table 2, tSCKpause). At 520 MHz,
/// 500 ns = 260 CPU cycles - too short for async yield.
pub struct CycleDelay;

impl embedded_hal_async::delay::DelayNs for CycleDelay {
    async fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * 520 / 1000) as u32;
        cortex_m::asm::delay(cycles);
    }
}

/// Selected TMC4671 SPI device borrowed from the Ouroboros mux.
pub type TmcSpiDevice<'a> = crate::tmc_mux::SelectedTmcDevice<'a>;
