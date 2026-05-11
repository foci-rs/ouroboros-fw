//! Trace-buffer state (ring buffer, generation counter, capture mask).
//! Task 8b populates this from `mod app` in main.rs.

use crate::board_config::TMC_CHANNEL_COUNT;

/// Trace capture buffers in CCM RAM. Initialized via `reset()` in init.
///
/// SAFETY: Only accessed from tmc_control (P1) after init. MaybeUninit
/// because CCM RAM has no default initialization; init() writes a valid
/// TraceBuffer for each board-local channel via ptr::write before any read.
#[unsafe(link_section = ".ccmram")]
pub(crate) static mut TRACE_BUFS: [core::mem::MaybeUninit<foci_firmware::trace::TraceBuffer>;
    TMC_CHANNEL_COUNT as usize] =
    [const { core::mem::MaybeUninit::uninit() }; TMC_CHANNEL_COUNT as usize];

/// Get a mutable reference to a channel's trace buffer.
///
/// # Safety
///
/// Must only be called from tmc_control (P1) after init has completed.
#[inline]
pub(crate) unsafe fn trace_buf(
    channel: u8,
) -> Option<&'static mut foci_firmware::trace::TraceBuffer> {
    if channel >= TMC_CHANNEL_COUNT {
        return None;
    }
    // Use addr_of_mut! to avoid creating a reference to the mutable static
    // (Rust 2024 edition compatibility). MaybeUninit<T> has the same layout as T.
    // SAFETY: caller upholds the P1-only + post-init contract.
    let ptr = unsafe {
        core::ptr::addr_of_mut!(TRACE_BUFS)
            .cast::<foci_firmware::trace::TraceBuffer>()
            .add(channel as usize)
    };
    Some(unsafe { &mut *ptr })
}
