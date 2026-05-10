//! USB task body.
//!
//! [`pump`] is invoked from the RTIC `usb_task` shell inside `mod app`;
//! it waits on the USB wake signal, drains any queued RX frames into
//! the Klipper protocol via the caller-supplied state lock closure, and
//! flushes TX after each wake.

use ankyra::{InputBuffer, SliceInputBuffer};
use core::sync::atomic::{AtomicBool, Ordering::Relaxed};
use foci_klipper::state::State;

use crate::firmware_ctx::FirmwareCtx;
use crate::transport::USB_WAKE;

/// One wake cycle of the USB task: wait for a wake signal, drain RX
/// into the protocol via `lock_app_state`, then flush TX.
///
/// `lock_app_state` wraps the RTIC shared-resource lock; it hands out
/// the `State` for a single protocol-dispatch call and no longer.
/// `report_pending` is the hot-path AtomicBool that the trigger-report
/// path sets to preempt further RX processing in favor of
/// `report_task`.
pub async fn pump(
    mut lock_app_state: impl FnMut(&mut dyn FnMut(&mut State)),
    report_pending: &AtomicBool,
) {
    USB_WAKE.wait().await;

    // Process pending USB events. `poll` reads new bulk OUT data and
    // drains bulk IN. Loop while making progress — break if the
    // protocol cannot consume any bytes (incomplete frame).
    while foci_usb_stm32::poll() {
        let data = foci_usb_stm32::rx_data();
        if data.is_empty() {
            break;
        }
        let len = data.len().min(128);
        let mut local_buf = [0u8; 128];
        local_buf[..len].copy_from_slice(&data[..len]);

        // Feed to protocol — only consume what transport processed.
        // Partial frames at the end must stay in the RX buffer.
        let mut input = SliceInputBuffer::new(&local_buf[..len]);
        lock_app_state(&mut |state| {
            let mut ctx = FirmwareCtx::new(state);
            crate::KLIPPER_TRANSPORT.receive(&mut input, &mut ctx);
        });
        let consumed = len - input.available();
        foci_usb_stm32::rx_consume(consumed);
        if consumed == 0 {
            // No progress — wait for more USB data.
            break;
        }
        // Yield to let `report_task` run when a trigger report is
        // pending. Both tasks are P2; yielding gives `report_task` a
        // chance to send the `trsync_state` reply before we process
        // more commands.
        if report_pending.load(Relaxed) {
            break;
        }
    }
    // Final poll to flush any remaining TX.
    foci_usb_stm32::poll();
}
