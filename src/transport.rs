//! Ankyra transport glue: shepherds wire-format frames into the USB TX
//! buffer and wakes the USB task when new data is pending.

use ankyra::{ScratchOutput, TransportOutput};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// USB wake signal. The OTG_HS ISR and any frame-producing task
/// signals this to wake `usb_task`, which drains the TX buffer.
pub static USB_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Ankyra `TransportOutput` that writes each encoded frame to the USB
/// TX buffer and signals [`USB_WAKE`] so `usb_task` can flush it.
pub struct BufferTransportOutput;

/// Static instance consumed by `ankyra_config!`.
pub const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

impl TransportOutput for BufferTransportOutput {
    type Output = ScratchOutput;
    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        let mut scratch = ScratchOutput::new();
        f(&mut scratch);
        let output = scratch.result();
        foci_usb_stm32::tx_write(output);
        // Wake usb_task to drain TX buffer. Critical for data
        // generated outside usb_task (stats output from P1).
        USB_WAKE.signal(());
    }
}
