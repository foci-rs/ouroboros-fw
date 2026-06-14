//! Ankyra transport glue: shepherds wire-format frames into the USB TX buffer
//! and wakes the USB task when new data is pending.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use foci_firmware::transport::{AnkyraUsbTransport, UsbTx, UsbWake};

/// USB wake signal. The USB ISR and any frame-producing task signals this to
/// wake `usb_task`, which drains the TX buffer.
pub static USB_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Board-local USB transmit hook for the shared Ankyra transport.
pub struct BoardUsbTx;

impl UsbTx for BoardUsbTx {
    fn write(data: &[u8]) {
        foci_usb_stm32::tx_write(data);
    }
}

/// Board-local USB wake hook for the shared Ankyra transport.
pub struct BoardUsbWake;

impl UsbWake for BoardUsbWake {
    fn wake() {
        USB_WAKE.signal(());
    }
}

/// Ankyra transport output backed by the board USB CDC transport.
pub type BufferTransportOutput = AnkyraUsbTransport<BoardUsbTx, BoardUsbWake>;

/// Static instance consumed by `ankyra_config!`.
pub const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput::new();
