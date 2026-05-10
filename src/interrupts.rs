//! Hardware interrupt service routine bodies.
//!
//! The RTIC `#[task(binds = ...)]` declarations live in `main.rs`; the
//! bodies of the simpler ISRs live here. `step_timer_irq` remains in
//! `main.rs` until its hot-path statics migrate to `foci-firmware` in a
//! later task — moving it now would require either duplicating ~40
//! statics across modules or routing every access through
//! `crate::app::*` paths.

use crate::transport::USB_WAKE;

/// USB OTG HS interrupt: run the USB driver's ISR hook and wake
/// `usb_task` so it can drain RX / flush TX.
pub fn otg_fs_irq() {
    foci_usb_stm32::irq_handler();
    USB_WAKE.signal(());
}

/// TMC4671 STATUS pin (PE2 / EXTI line 2) interrupt.
///
/// Fires when any `STATUS_MASK`-enabled flag is asserted
/// (`not_PLL_locked`, `adc_i_clipped`, …). Quiesces step generation
/// immediately and signals `tmc_disable_signal` so `tmc_control` can
/// read `STATUS_FLAGS` on its next tick and log the cause.
pub fn tmc_status_irq(
    fault_active: &core::sync::atomic::AtomicBool,
    tmc_disable_signal: &embassy_sync::signal::Signal<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        (),
    >,
) {
    let exti = embassy_stm32::pac::EXTI;
    exti.pr(0).write(|w| w.set_line(2, true));

    // Immediately quiesce step generation (same P3, atomic).
    fault_active.store(true, core::sync::atomic::Ordering::Relaxed);
    let tim5 = embassy_stm32::pac::TIM5;
    tim5.dier().modify(|w| w.set_ccie(0, false));

    tmc_disable_signal.signal(());
    defmt::warn!("TMC4671 STATUS interrupt: fault detected, step timer disabled");
}
