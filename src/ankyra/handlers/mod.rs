//! Board-local ankyra `#[klipper_command]` handlers. After Task 24,
//! only `foci` remains here — it contains `foci_tune`, which emits
//! replies through the board-local transport sender and cannot be
//! lifted to foci-firmware. All other handlers were lifted to
//! foci-firmware and are re-exported through `FOCI_FIRMWARE_PROVIDER`
//! in the `foci_ankyra_config!` macro.

pub mod foci;
