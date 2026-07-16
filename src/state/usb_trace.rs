//! USB trace stream state for trace builds.

#![cfg(feature = "trace")]

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use foci_firmware::trace::{
    stream::FrameBuf,
    transport::{self, StatusSnapshot},
};

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

const STATUS_SLOT_COUNT: usize =
    transport::status_slot_count(crate::board_config::TMC_CHANNEL_COUNT as usize);

pub(crate) static TRACE_COUNTERS: TraceCounters = TraceCounters::new();

struct MotionQueueEndpoints {
    producer: heapless::spsc::Producer<'static, QueuedFrame>,
    consumer: heapless::spsc::Consumer<'static, QueuedFrame>,
}

static mut MOTION_QUEUE_ENDPOINTS: MotionQueueEndpoints = {
    static mut QUEUE: heapless::spsc::Queue<QueuedFrame, MOTION_QUEUE_DEPTH> =
        heapless::spsc::Queue::new();
    // SAFETY: `QUEUE` is private to this initializer. The returned endpoints
    // are stored separately and accessed only by their owning RTIC contexts.
    #[allow(static_mut_refs)]
    let (producer, consumer) = unsafe { QUEUE.split_const() };
    MotionQueueEndpoints { producer, consumer }
};
pub(crate) static mut EVENT_QUEUE: heapless::spsc::Queue<QueuedFrame, EVENT_QUEUE_DEPTH> =
    heapless::spsc::Queue::new();
pub(crate) static mut PENDING_EVENTS: heapless::Deque<PendingEvent, PENDING_EVENT_DEPTH> =
    heapless::Deque::new();
pub(crate) static mut LAST_STATUS: [StatusSnapshot; STATUS_SLOT_COUNT] =
    [StatusSnapshot::new(); STATUS_SLOT_COUNT];

/// Partially sent frame owned by the single USB trace consumer.
pub(crate) static mut CURRENT_TX: Option<QueuedFrame> = None;

/// Return the P1-owned trace motion/counter producer.
///
/// # Safety
///
/// Only the priority-1 TMC control task may call this function.
#[inline]
pub(crate) unsafe fn motion_producer() -> &'static mut heapless::spsc::Producer<'static, QueuedFrame>
{
    let ptr = unsafe { core::ptr::addr_of_mut!(MOTION_QUEUE_ENDPOINTS.producer) };
    // SAFETY: The caller guarantees exclusive P1 producer access.
    unsafe { &mut *ptr }
}

/// Return the P2-owned trace motion/counter consumer.
///
/// # Safety
///
/// Only the priority-2 USB task may call this function.
#[inline]
pub(crate) unsafe fn motion_consumer() -> &'static mut heapless::spsc::Consumer<'static, QueuedFrame>
{
    let ptr = unsafe { core::ptr::addr_of_mut!(MOTION_QUEUE_ENDPOINTS.consumer) };
    // SAFETY: The caller guarantees exclusive P2 consumer access.
    unsafe { &mut *ptr }
}

/// Return the trace event queue.
///
/// # Safety
///
/// Only the priority-2 USB task may call this function. Pending event requests
/// from other priorities are converted to frames by that task before enqueue.
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
