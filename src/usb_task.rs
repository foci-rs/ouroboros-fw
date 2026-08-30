//! USB task body.
//!
//! [`OuroborosUsbBackend`] is driven by the RTIC `usb_task` shell inside
//! `mod app` via `foci_firmware::usb_pump::run_once`: it waits on the USB
//! wake signal, drains queued RX frames into the Klipper protocol via the
//! caller-supplied state lock closure, and flushes TX after each drain
//! quantum.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

use foci_firmware::usb_pump::{USB_YIELD_DELAY_US, UsbPumpBackend, UsbPumpCounters};
use foci_klipper::state::State;
use rtic_monotonics::{Monotonic, fugit::ExtU64};

use crate::app::Mono;
use crate::firmware_ctx::FirmwareCtx;
use crate::transport::USB_WAKE;

/// Board hooks for the shared `foci_firmware::usb_pump::run_once` drain loop.
///
/// `lock_app_state` wraps the RTIC shared-resource lock; it hands out the
/// `State` for a single protocol-dispatch call and no longer.
/// `report_pending` is the hot-path `AtomicBool` that the trigger-report path
/// sets to preempt further RX processing in favor of `report_task`.
pub struct OuroborosUsbBackend<'a, L: FnMut(&mut dyn FnMut(&mut State))> {
    lock_app_state: L,
    report_pending: &'a AtomicBool,
    emitter: &'a Emitter,
    counters: &'a UsbPumpCounters,
}

impl<'a, L: FnMut(&mut dyn FnMut(&mut State))> OuroborosUsbBackend<'a, L> {
    /// Construct the backend from the board's shared-resource lock closure and the statics it borrows.
    pub fn new(
        lock_app_state: L,
        report_pending: &'a AtomicBool,
        emitter: &'a Emitter,
        counters: &'a UsbPumpCounters,
    ) -> Self {
        Self {
            lock_app_state,
            report_pending,
            emitter,
            counters,
        }
    }
}

impl<'a, L: FnMut(&mut dyn FnMut(&mut State))> UsbPumpBackend for OuroborosUsbBackend<'a, L> {
    async fn wait_wake(&mut self) {
        USB_WAKE.wait().await;
    }

    fn poll(&mut self) -> bool {
        foci_usb_stm32::poll()
    }

    fn rx_data(&self) -> &[u8] {
        foci_usb_stm32::rx_data()
    }

    fn rx_consume(&mut self, n: usize) {
        foci_usb_stm32::rx_consume(n);
    }

    fn dispatch(&mut self, chunk: &[u8]) -> usize {
        use ankyra::{InputBuffer, SliceInputBuffer};
        let mut input = SliceInputBuffer::new(chunk);
        (self.lock_app_state)(&mut |state| {
            let mut ctx = FirmwareCtx::new(state);
            crate::KLIPPER_TRANSPORT.receive(&mut input, &mut ctx);
        });
        chunk.len() - input.available()
    }

    fn report_pending(&self) -> bool {
        self.report_pending.load(Relaxed)
    }

    fn wake(&mut self) {
        USB_WAKE.signal(());
    }

    async fn yield_now(&mut self) {
        Mono::delay(u64::from(USB_YIELD_DELAY_US).micros()).await;
    }

    fn final_poll(&mut self) {
        foci_usb_stm32::poll();
    }

    fn on_wake(&mut self) {
        #[cfg(feature = "trace")]
        if foci_usb_stm32::is_configured() && !crate::usb_trace::is_configured() {
            crate::usb_trace::note_usb_configured();
        }
    }

    fn on_drain_complete(&mut self) {
        #[cfg(feature = "trace")]
        {
            crate::usb_trace::drain_pending_events();
            crate::usb_trace::drain_once();
        }
        self.emitter.maybe_emit(self.counters);
    }
}

/// Throttled defmt summary of [`UsbPumpCounters`], emitted at most once per
/// `foci_firmware::usb_pump::USB_EMIT_MIN_INTERVAL_US` or when
/// `max_activation_bytes` grows, so a sustained stream cannot spam RTT.
pub struct Emitter {
    last_emit_ticks: AtomicU32,
    last_max: AtomicU32,
}

impl Emitter {
    /// Construct a fresh emitter with no prior emission recorded.
    pub const fn new() -> Self {
        Self {
            last_emit_ticks: AtomicU32::new(0),
            last_max: AtomicU32::new(0),
        }
    }

    fn maybe_emit(&self, counters: &UsbPumpCounters) {
        let s = counters.snapshot();
        let now = Mono::now().ticks() as u32;
        if foci_firmware::usb_pump::should_emit(
            now,
            self.last_emit_ticks.load(Relaxed),
            &s,
            self.last_max.load(Relaxed),
        ) {
            self.last_emit_ticks.store(now, Relaxed);
            self.last_max.store(s.max_activation_bytes, Relaxed);
            defmt::info!(
                "usb_pump acts={} exh={} last={} max={}",
                s.activation_count,
                s.exhaustion_count,
                s.last_activation_bytes,
                s.max_activation_bytes
            );
        }
    }
}

impl Default for Emitter {
    fn default() -> Self {
        Self::new()
    }
}
