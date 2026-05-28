//! Ouroboros USB trace stream integration.

#![cfg(feature = "trace")]

use core::sync::atomic::Ordering;

use foci_firmware::trace_events::TraceClockExt;
use foci_firmware::trace_stream::{FrameBuf, METADATA_PAYLOAD_MAX};

use crate::state::usb_trace::{
    PENDING_EVENTS, PendingEvent, QueuedFrame, TRACE_COUNTERS, current_tx, event_queue,
};

/// Product string used by trace builds.
pub(crate) const TRACE_PRODUCT: &str = "FOCI Ouroboros (trace)";

static mut TRACE_CLOCK_EXT: TraceClockExt = TraceClockExt::new();

fn compute_boot_id() -> u32 {
    let tim5 = crate::helpers::lifecycle::read_tim5();
    let dwt = cortex_m::peripheral::DWT::cycle_count();
    tim5 ^ dwt
}

/// Extend the 32-bit TIM5 Klipper clock into the u64 trace timestamp domain.
pub(crate) fn trace_now_ticks() -> u64 {
    let low = crate::helpers::lifecycle::read_tim5();
    extend_tim5_low(low)
}

/// Extend a previously sampled TIM5 low word into the trace timestamp domain.
pub(crate) fn extend_tim5_low(low: u32) -> u64 {
    cortex_m::interrupt::free(|_| unsafe {
        let ext = &mut *core::ptr::addr_of_mut!(TRACE_CLOCK_EXT);
        ext.extend_low(low)
    })
}

/// Record the trace boot timestamp after TIM5 starts.
pub(crate) fn note_trace_boot() {
    let _ = trace_now_ticks();
    TRACE_COUNTERS
        .boot_id
        .store(compute_boot_id(), Ordering::Relaxed);
}

/// Return true when trace producers should emit frames.
pub(crate) fn is_configured() -> bool {
    TRACE_COUNTERS.configured.load(Ordering::Relaxed) && foci_usb_stm32::is_configured()
}

/// Mark the trace stream configured after USB configuration.
pub(crate) fn note_usb_configured() {
    TRACE_COUNTERS.configured.store(true, Ordering::Relaxed);
    emit_metadata();
}

fn header_at_timestamp(
    frame_type: u8,
    payload_len: usize,
    source_kind: u8,
    source_index: u8,
    flags: u8,
    timestamp_ticks: u64,
) -> foci_firmware::trace_stream::FrameHeader {
    foci_firmware::trace_stream::FrameHeader {
        frame_type,
        flags,
        payload_len: payload_len as u16,
        source_kind,
        source_index,
        sequence: TRACE_COUNTERS.next_sequence(),
        timestamp_ticks,
        boot_id: TRACE_COUNTERS.boot_id.load(Ordering::Relaxed),
    }
}

fn enqueue_event(frame: FrameBuf) {
    // SAFETY: Trace event producers enqueue complete frames; the USB task is
    // the only consumer in this initial wiring slice.
    unsafe {
        if event_queue().enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
        } else {
            crate::transport::USB_WAKE.signal(());
        }
    }
}

/// Request a structured event from any RTIC context.
pub(crate) fn request_event(
    level: u8,
    subsystem_id: u8,
    event_id: u16,
    args: [i32; 4],
    arg_count: u8,
) {
    if !is_configured() {
        return;
    }
    let timestamp_ticks = trace_now_ticks();
    let event = PendingEvent {
        timestamp_ticks,
        level,
        subsystem_id,
        event_id,
        args,
        arg_count,
    };
    // O(1) critical section: push one fixed-size event or count one drop.
    cortex_m::interrupt::free(|_| unsafe {
        let queue = &mut *core::ptr::addr_of_mut!(PENDING_EVENTS);
        if queue.push_back(event).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
        }
    });
}

/// Translate a shared commissioning event into a USB trace event frame.
pub(crate) fn request_commission_event(
    source_index: u8,
    event: foci_firmware::trace_events::CommissionTraceEvent,
) {
    request_event(
        event.level as u8,
        foci_firmware::trace_events::SUBSYSTEM_COMMISSIONING,
        event.event_id,
        [
            i32::from(source_index),
            i32::from(foci_firmware::trace_events::engine_kind_wire_code(
                event.engine_kind,
            )),
            event.payload[0],
            event.payload[1],
        ],
        2 + event.payload_count.min(2),
    );
}

fn enqueue_event_frame(event: PendingEvent) {
    use foci_firmware::trace_stream::*;

    let sample = EventSample {
        level: event.level,
        subsystem_id: event.subsystem_id,
        event_id: event.event_id,
        arg_count: event.arg_count,
        args: event.args,
    };
    let mut payload = [0u8; EVENT_PAYLOAD_LEN];
    encode_event_payload(&sample, &mut payload);
    let flags = if TRACE_COUNTERS.event_drop_gap.swap(false, Ordering::Relaxed) {
        FLAG_DROP_GAP_BEFORE_THIS_FRAME
    } else {
        0
    };
    let mut frame = FrameBuf::new();
    frame.write_header(header_at_timestamp(
        FRAME_TYPE_EVENT,
        EVENT_PAYLOAD_LEN,
        SOURCE_KIND_BOARD,
        SOURCE_INDEX_NA,
        flags,
        event.timestamp_ticks,
    ));
    frame
        .payload_mut(EVENT_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_event(frame);
}

/// Convert pending multi-producer event requests into event frames.
pub(crate) fn drain_pending_events() {
    loop {
        let event = cortex_m::interrupt::free(|_| unsafe {
            let queue = &mut *core::ptr::addr_of_mut!(PENDING_EVENTS);
            queue.pop_front()
        });
        let Some(event) = event else {
            break;
        };
        enqueue_event_frame(event);
    }
}

fn push_str(out: &mut heapless::Vec<u8, METADATA_PAYLOAD_MAX>, value: &str) {
    let bytes = value.as_bytes();
    let len = bytes.len().min(31);
    let _ = out.push(len as u8);
    let _ = out.extend_from_slice(&bytes[..len]);
}

/// Emit stream metadata.
pub(crate) fn emit_metadata() {
    use foci_firmware::trace_stream::*;

    if !is_configured() {
        return;
    }

    let source_count = crate::board_config::TMC_CHANNEL_COUNT;
    let mut payload: heapless::Vec<u8, METADATA_PAYLOAD_MAX> = heapless::Vec::new();
    let _ = payload.extend_from_slice(&[
        1, // metadata schema
        1, // trace feature revision
        source_count,
        0, // skipped sample counter count
    ]);
    let _ = payload.extend_from_slice(&crate::ankyra::constants::CLOCK_FREQ.to_le_bytes());
    let _ = payload.extend_from_slice(&0u32.to_le_bytes());
    let _ = payload.extend_from_slice(&0u32.to_le_bytes());
    let _ = payload.extend_from_slice(&0u32.to_le_bytes());
    push_str(&mut payload, "ouroboros-fw");
    push_str(&mut payload, TRACE_PRODUCT);
    push_str(&mut payload, "");
    for channel in 0..source_count {
        let _ = payload.extend_from_slice(&[SOURCE_KIND_FOCI, channel, 7]);
        let _ = payload.extend_from_slice(b"channel");
    }

    let mut frame = FrameBuf::new();
    frame.write_header(header_at_timestamp(
        FRAME_TYPE_METADATA,
        payload.len(),
        SOURCE_KIND_BOARD,
        SOURCE_INDEX_NA,
        0,
        trace_now_ticks(),
    ));
    frame.payload_mut(payload.len()).copy_from_slice(&payload);
    enqueue_event(frame);
}

/// Drain at most one 64-byte trace packet to USB.
pub(crate) fn drain_once() {
    // SAFETY: This is the only trace consumer. Producers only enqueue complete
    // `QueuedFrame` values. `current_tx` is therefore single-consumer state and
    // preserves partially sent frames across calls.
    unsafe {
        let current_tx = current_tx();
        if current_tx.is_none() {
            *current_tx = event_queue().dequeue();
        }
        let Some(current) = current_tx.as_mut() else {
            return;
        };
        let written = {
            let bytes = current.frame.bytes();
            let offset = current.offset as usize;
            foci_usb_stm32::trace_write(&bytes[offset..])
        };
        if written > 0 {
            current.offset = current.offset.saturating_add(written as u16);
            if current.offset as usize >= current.frame.bytes().len() {
                *current_tx = None;
            }
        } else {
            TRACE_COUNTERS
                .endpoint_not_ready
                .fetch_add(1, Ordering::Relaxed);
        }
    }
}
