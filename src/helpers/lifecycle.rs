//! Lifecycle helpers: Klipper clock reads and Katapult bootloader
//! reset, shared by handlers and the RTIC task graph.
//!
//! Shutdown entry is split in two: the board-agnostic Klipper-level transition lives in
//! `foci_firmware::helpers::lifecycle::enter_shutdown`; the
//! trigger-domain quiesce and post-quiesce atomics live in
//! `FirmwareCtx`'s `ShutdownRecovery::enter_shutdown` impl in
//! `firmware_ctx.rs`.

/// Katapult bootloader vector table base (from Klipper `armcm_reset.c`).
pub(crate) const KATAPULT_BOOT_ADDRESS: u32 = 0x0800_0000;
/// Signature at `bl_vectors[reset_handler - 9]` when a Katapult
/// bootloader is resident.
pub(crate) const KATAPULT_SIGNATURE: u64 = 0x2174_6f6f_426e_6143; // "CanBoot!"
/// Request signature that skips the bootloader on next boot
/// (used for `FIRMWARE_RESTART`).
pub(crate) const KATAPULT_BYPASS: u64 = 0x7b06_ec45_a9a8_243d;

/// Read the TIM5 counter (Klipper clock at 260 MHz).
pub(crate) fn read_tim5() -> u32 {
    embassy_stm32::pac::TIM5.cnt().read()
}

/// Write a Katapult request signature and perform a system reset.
///
/// `KATAPULT_REQUEST`: stay in bootloader (for reflashing via 1200-baud DTR).
/// `KATAPULT_BYPASS`: skip bootloader, boot application (for FIRMWARE_RESTART).
///
/// Matches Klipper's `canboot_reset()` in armcm_reset.c. Verifies the
/// bootloader signature before writing, no-ops if no bootloader is present.
pub(crate) fn katapult_reset(req_signature: u64) {
    // SAFETY: The bootloader vector table at KATAPULT_BOOT_ADDRESS is
    // always readable. We verify the Katapult signature before writing.
    unsafe {
        let bl_vectors = KATAPULT_BOOT_ADDRESS as *const u32;
        let reset_handler = core::ptr::read_volatile(bl_vectors.add(1));
        let boot_sig_addr = (reset_handler - 9) as *const u64;
        let req_sig_addr = core::ptr::read_volatile(bl_vectors) as *mut u64;

        if !(boot_sig_addr as usize).is_multiple_of(8)
            || core::ptr::read_volatile(boot_sig_addr) != KATAPULT_SIGNATURE
            || !(req_sig_addr as usize).is_multiple_of(8)
        {
            cortex_m::peripheral::SCB::sys_reset();
        }

        cortex_m::interrupt::disable();
        core::ptr::write_volatile(req_sig_addr, req_signature);
    }

    cortex_m::peripheral::SCB::sys_reset();
}
