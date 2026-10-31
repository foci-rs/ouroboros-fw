//! Pure trace-mode and pre-configuration counter policy, shared with the host tests.

use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};

use foci_firmware::context::QueueFull;
use foci_firmware::trace::sampling::TraceMode;

/// Nominal control-detail rate advertised in metadata and used for pre-configuration skips.
pub(crate) const CONTROL_DETAIL_HZ: u32 = 100;

/// Nominal status rate advertised in metadata and used for pre-configuration skips.
pub(crate) const STATUS_HZ: u32 = 10;

const TRACE_MODE_REQUEST_DEPTH: u8 = 4;

fn skipped_at_rate(elapsed_ms: u32, rate_hz: u32) -> u32 {
    ((u64::from(elapsed_ms) * u64::from(rate_hz)) / 1000).min(u64::from(u32::MAX)) as u32
}

/// Critical frames skipped before USB configuration.
///
/// `critical_mode_ms` is the time spent in a critical-sampling mode before configuration;
/// it is capped at `elapsed_ms`.
pub(crate) fn skipped_critical_for(elapsed_ms: u32, critical_mode_ms: u32) -> u32 {
    skipped_at_rate(
        critical_mode_ms.min(elapsed_ms),
        TraceMode::Critical.critical_hz(),
    )
}

/// Control-detail frames skipped during `elapsed_ms` before USB configuration.
pub(crate) fn skipped_control_detail_for(elapsed_ms: u32) -> u32 {
    skipped_at_rate(elapsed_ms, CONTROL_DETAIL_HZ)
}

/// Status frames skipped during `elapsed_ms` before USB configuration.
pub(crate) fn skipped_status_for(elapsed_ms: u32) -> u32 {
    skipped_at_rate(elapsed_ms, STATUS_HZ)
}

/// Four-entry single-producer single-consumer ring of `(mode, oid)` trace-mode requests.
///
/// The producer is the priority-2 USB task and the consumer is the priority-1 control task.
/// `head` and `tail` are free-running counters, so all four slots are usable.
pub(crate) struct TraceModeRequests {
    slots: [AtomicU16; TRACE_MODE_REQUEST_DEPTH as usize],
    head: AtomicU8,
    tail: AtomicU8,
}

impl TraceModeRequests {
    /// Create an empty ring.
    pub(crate) const fn new() -> Self {
        Self {
            slots: [
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
                AtomicU16::new(0),
            ],
            head: AtomicU8::new(0),
            tail: AtomicU8::new(0),
        }
    }

    fn slot(&self, counter: u8) -> &AtomicU16 {
        &self.slots[usize::from(counter % TRACE_MODE_REQUEST_DEPTH)]
    }

    /// Producer side: queue one request. `Err(QueueFull)` leaves the ring unchanged.
    pub(crate) fn push(&self, mode: TraceMode, oid: u8) -> Result<(), QueueFull> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        if tail.wrapping_sub(head) >= TRACE_MODE_REQUEST_DEPTH {
            return Err(QueueFull);
        }
        self.slot(tail).store(
            (u16::from(mode as u8) << 8) | u16::from(oid),
            Ordering::Relaxed,
        );
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Ok(())
    }

    /// Consumer side: take the oldest request.
    pub(crate) fn pop(&self) -> Option<(TraceMode, u8)> {
        let head = self.head.load(Ordering::Relaxed);
        if self.tail.load(Ordering::Acquire) == head {
            return None;
        }
        let packed = self.slot(head).load(Ordering::Relaxed);
        self.head.store(head.wrapping_add(1), Ordering::Release);
        TraceMode::from_wire((packed >> 8) as u8).map(|mode| (mode, packed as u8))
    }

    /// Return whether no request is queued.
    pub(crate) fn is_empty(&self) -> bool {
        self.tail.load(Ordering::Acquire) == self.head.load(Ordering::Relaxed)
    }
}

/// Take the next request the control task may service this tick.
///
/// While the trace stream is configured, a request waits until no channel still owes the
/// snapshot frame of the previous metadata emission. While unconfigured nothing can be
/// queued, so requests are taken without that gate.
pub(crate) fn next_mode_request(
    ring: &TraceModeRequests,
    configured: bool,
    snapshots_pending: bool,
) -> Option<(TraceMode, u8)> {
    if configured && snapshots_pending {
        return None;
    }
    ring.pop()
}
