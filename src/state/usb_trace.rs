//! USB trace stream state for trace builds.

#![cfg(feature = "trace")]

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use foci_firmware::trace::stream::{FrameBuf, StatusSample};

/// Motion/counter queue depth in frames.
///
/// `FrameBuf` is 256 bytes, so depth 32 reserves 8 KiB. This matches the
/// OpenFFBoard trace transport and fits the STM32H723 SRAM budget.
pub(crate) const MOTION_QUEUE_DEPTH: usize = 32;

/// Event queue depth in frames.
pub(crate) const EVENT_QUEUE_DEPTH: usize = 8;

/// Pending structured-event queue depth.
pub(crate) const PENDING_EVENT_DEPTH: usize = 16;

/// Structured event request safe to create from any RTIC context.
#[derive(Clone, Copy)]
pub(crate) struct PendingEvent {
    pub(crate) timestamp_ticks: u64,
    pub(crate) level: u8,
    pub(crate) subsystem_id: u8,
    pub(crate) event_id: u16,
    pub(crate) args: [i32; 4],
    pub(crate) arg_count: u8,
}

/// A queued frame currently being packetized over 64-byte full-speed USB packets.
#[derive(Clone, Copy)]
pub(crate) struct QueuedFrame {
    pub(crate) frame: FrameBuf,
    pub(crate) offset: u16,
}

impl QueuedFrame {
    /// Create a queued frame with no bytes sent yet.
    pub(crate) const fn new(frame: FrameBuf) -> Self {
        Self { frame, offset: 0 }
    }
}

/// Trace queue counters.
pub(crate) struct TraceCounters {
    pub(crate) configured: AtomicBool,
    pub(crate) boot_id: AtomicU32,
    pub(crate) sequence: AtomicU32,
    pub(crate) motion_drops: AtomicU32,
    pub(crate) motion_drop_gap: AtomicBool,
    pub(crate) event_drops: AtomicU32,
    pub(crate) event_drop_gap: AtomicBool,
    pub(crate) high_water: AtomicU16,
    pub(crate) endpoint_not_ready: AtomicU32,
}

impl TraceCounters {
    /// Create zeroed trace counters.
    pub(crate) const fn new() -> Self {
        Self {
            configured: AtomicBool::new(false),
            boot_id: AtomicU32::new(0),
            sequence: AtomicU32::new(0),
            motion_drops: AtomicU32::new(0),
            motion_drop_gap: AtomicBool::new(false),
            event_drops: AtomicU32::new(0),
            event_drop_gap: AtomicBool::new(false),
            high_water: AtomicU16::new(0),
            endpoint_not_ready: AtomicU32::new(0),
        }
    }

    /// Allocate the next global per-device frame sequence.
    pub(crate) fn next_sequence(&self) -> u32 {
        self.sequence.fetch_add(1, Ordering::Relaxed)
    }
}

#[derive(Clone, Copy)]
pub(crate) struct StatusSnapshot {
    pub(crate) source_kind: u8,
    pub(crate) source_index: u8,
    pub(crate) sample: StatusSample,
}

impl StatusSnapshot {
    pub(crate) const fn new() -> Self {
        Self {
            source_kind: 0,
            source_index: 0,
            sample: StatusSample {
                status_flags: 0,
                status_interrupt_count: 0,
                clipping_flags: 0,
                spi_error_count: 0,
                watchdog_or_step_overrun_count: 0,
                klipper_rx_queue_depth: 0,
                usb_trace_queue_high_water: 0,
                dropped_motion_sample_count: 0,
                dropped_event_log_count: 0,
                endpoint_not_ready_count: 0,
                suppressed_status_change_count: 0,
            },
        }
    }
}

pub(crate) static TRACE_COUNTERS: TraceCounters = TraceCounters::new();

pub(crate) static mut MOTION_QUEUE: heapless::spsc::Queue<QueuedFrame, MOTION_QUEUE_DEPTH> =
    heapless::spsc::Queue::new();
pub(crate) static mut EVENT_QUEUE: heapless::spsc::Queue<QueuedFrame, EVENT_QUEUE_DEPTH> =
    heapless::spsc::Queue::new();
pub(crate) static mut PENDING_EVENTS: heapless::Deque<PendingEvent, PENDING_EVENT_DEPTH> =
    heapless::Deque::new();
pub(crate) static mut LAST_STATUS: [StatusSnapshot; 4] = [StatusSnapshot::new(); 4];

/// Partially sent frame owned by the single USB trace consumer.
pub(crate) static mut CURRENT_TX: Option<QueuedFrame> = None;

/// Return the trace motion/counter queue.
///
/// # Safety
///
/// Callers must uphold the single-producer/single-consumer access discipline:
/// producer contexts enqueue only, and the USB task is the sole consumer.
#[inline]
pub(crate) unsafe fn motion_queue()
-> &'static mut heapless::spsc::Queue<QueuedFrame, MOTION_QUEUE_DEPTH> {
    let ptr = core::ptr::addr_of_mut!(MOTION_QUEUE);
    // SAFETY: The caller guarantees exclusive access for this RTIC context.
    unsafe { &mut *ptr }
}

/// Return the trace event queue.
///
/// # Safety
///
/// Callers must uphold the single-producer/single-consumer access discipline:
/// producer contexts enqueue only, and the USB task is the sole consumer.
#[inline]
pub(crate) unsafe fn event_queue()
-> &'static mut heapless::spsc::Queue<QueuedFrame, EVENT_QUEUE_DEPTH> {
    let ptr = core::ptr::addr_of_mut!(EVENT_QUEUE);
    // SAFETY: The caller guarantees exclusive access for this RTIC context.
    unsafe { &mut *ptr }
}

/// Return the partial trace TX frame slot.
///
/// # Safety
///
/// Only the USB task may call this function.
#[inline]
pub(crate) unsafe fn current_tx() -> &'static mut Option<QueuedFrame> {
    let ptr = core::ptr::addr_of_mut!(CURRENT_TX);
    // SAFETY: The caller is the only consumer of this state.
    unsafe { &mut *ptr }
}
