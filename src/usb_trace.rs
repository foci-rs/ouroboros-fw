//! Ouroboros USB trace stream integration.

use core::sync::atomic::{AtomicI32, AtomicU8, AtomicU32, Ordering};

use foci_firmware::commissioning::inner::closed_loop_activation;
use foci_firmware::context::QueueFull;
use foci_firmware::trace::events::TraceClockExt;
use foci_firmware::trace::pressure::{TracePressureRuntime, TracePressureStatusInputs};
use foci_firmware::trace::sampling::{
    TraceMode, TraceSampleInputs, TraceSamplePlan, commissioning_trace_sample_plan,
};
use foci_firmware::trace::stream::{FrameBuf, frame_sequence};
use foci_firmware::trace::transport::{self, TraceQueueSelection};

use crate::state::usb_trace::{
    MOTION_QUEUE_DEPTH, PENDING_EVENTS, PendingEvent, QueuedFrame, TRACE_CONTROL_WAKE,
    TRACE_COUNTERS, TRACE_MODE_REQUESTS, current_tx, event_queue, motion_consumer, motion_producer,
};
use crate::usb_trace_policy::{
    CONTROL_DETAIL_HZ, EmitOutcome, STATUS_HZ, emit_pending, skipped_control_detail_for,
    skipped_critical_for, skipped_status_for,
};

static TRACE_PRESSURE: TracePressureRuntime = TracePressureRuntime::new();

pub(crate) const EVENT_FIRMWARE_BOOT: u16 = 1;
pub(crate) const EVENT_USB_CONFIGURED: u16 = 2;
pub(crate) const EVENT_MOTOR_ARM: u16 = 3;
pub(crate) const EVENT_MOTOR_DISARM: u16 = 4;
pub(crate) const EVENT_STATUS_INTERRUPT: u16 = 9;

static TRACE_SERIAL_LEN: AtomicU8 = AtomicU8::new(0);
static mut TRACE_SERIAL: [u8; 31] = [0; 31];

/// Cache the USB serial string for metadata frames.
///
/// Call once during init before trace producers run. The metadata path reads
/// the cached bytes without an ownership handoff.
pub(crate) fn set_trace_serial(serial: &str) {
    let bytes = serial.as_bytes();
    let len = bytes.len().min(31);
    // SAFETY: the critical section excludes trace_serial's concurrent read;
    // len is capped to TRACE_SERIAL's 31-byte size, so the copy stays in
    // bounds.
    cortex_m::interrupt::free(|_| unsafe {
        let dst = core::ptr::addr_of_mut!(TRACE_SERIAL).cast::<u8>();
        core::ptr::copy_nonoverlapping(bytes.as_ptr(), dst, len);
        TRACE_SERIAL_LEN.store(len as u8, Ordering::Relaxed);
    });
}

fn trace_serial() -> &'static str {
    let len = TRACE_SERIAL_LEN.load(Ordering::Relaxed) as usize;
    // SAFETY: the critical section excludes set_trace_serial's concurrent
    // write; len never exceeds what set_trace_serial wrote, and that write
    // is always valid UTF-8 (the source is a &str).
    cortex_m::interrupt::free(|_| unsafe {
        let src = core::ptr::addr_of!(TRACE_SERIAL).cast::<u8>();
        let bytes = core::slice::from_raw_parts(src, len);
        core::str::from_utf8_unchecked(bytes)
    })
}

fn store_split_u64(low: &AtomicU32, high: &AtomicU32, value: u64) {
    cortex_m::interrupt::free(|_| {
        low.store(value as u32, Ordering::Relaxed);
        high.store((value >> 32) as u32, Ordering::Relaxed);
    });
}

fn load_split_u64(low: &AtomicU32, high: &AtomicU32) -> u64 {
    cortex_m::interrupt::free(|_| {
        u64::from(low.load(Ordering::Relaxed)) | (u64::from(high.load(Ordering::Relaxed)) << 32)
    })
}

fn ms_since_boot(now: u64) -> u32 {
    let boot = load_split_u64(
        &TRACE_COUNTERS.boot_tick_low,
        &TRACE_COUNTERS.boot_tick_high,
    );
    let ticks_per_millisecond = u64::from(crate::ankyra::constants::CLOCK_FREQ / 1000);
    (now.saturating_sub(boot) / ticks_per_millisecond).min(u64::from(u32::MAX)) as u32
}

/// Close the pre-configuration segment that ends at `end_ms` for the live mode.
fn accumulate_pre_config_mode_time(end_ms: u32) {
    let since = TRACE_COUNTERS
        .pre_config_mode_since_ms
        .swap(end_ms, Ordering::Relaxed);
    if trace_mode().critical_sampling() {
        TRACE_COUNTERS
            .pre_config_critical_mode_ms
            .fetch_add(end_ms.saturating_sub(since), Ordering::Relaxed);
    }
}

fn note_trace_usb_configured_time() {
    let configured = trace_now_ticks();
    let elapsed_ms = ms_since_boot(configured);
    store_split_u64(
        &TRACE_COUNTERS.configured_tick_low,
        &TRACE_COUNTERS.configured_tick_high,
        configured,
    );
    if !TRACE_COUNTERS.configured.load(Ordering::Acquire) {
        accumulate_pre_config_mode_time(elapsed_ms);
    }
    TRACE_COUNTERS
        .pre_config_elapsed_ms
        .store(elapsed_ms, Ordering::Relaxed);
    TRACE_COUNTERS.pre_config_skipped_critical.store(
        skipped_critical_for(
            elapsed_ms,
            TRACE_COUNTERS
                .pre_config_critical_mode_ms
                .load(Ordering::Relaxed),
        ),
        Ordering::Relaxed,
    );
    TRACE_COUNTERS
        .pre_config_skipped_control_detail
        .store(skipped_control_detail_for(elapsed_ms), Ordering::Relaxed);
    TRACE_COUNTERS
        .pre_config_skipped_status
        .store(skipped_status_for(elapsed_ms), Ordering::Relaxed);
}

/// Return the live trace mode. Written only by the control task.
pub(crate) fn trace_mode() -> TraceMode {
    TraceMode::from_wire(TRACE_COUNTERS.trace_mode.load(Ordering::Relaxed))
        .unwrap_or(TraceMode::Dormant)
}

/// USB-task context: queue a trace-mode request for the control task.
///
/// Touches no frame queue and no live trace state; wakes the control task, which may be asleep
/// with maintenance disarmed. `Err(QueueFull)` when four requests are already waiting.
pub(crate) fn set_trace_mode(mode: TraceMode, oid: u8) -> Result<(), QueueFull> {
    TRACE_MODE_REQUESTS.push(mode, oid)?;
    TRACE_CONTROL_WAKE.signal(());
    Ok(())
}

/// Control-task context, at the start of every control tick before the driver runs.
///
/// Emits the configuration-time baseline once `note_usb_configured` asked for it, then
/// services at most one queued trace-mode request: stores it as the live mode, re-emits
/// metadata while configured, and replies with the live mode. Requests still queued after that
/// wake the control task for another pass.
///
/// A request never waits for owed snapshot frames: with no recorder draining the trace
/// endpoint they would stay owed indefinitely. The snapshot flag coalesces, so the frame that
/// finally goes out reports the state after the newest metadata.
pub(crate) fn service_trace_mode_changes() {
    if TRACE_COUNTERS
        .baseline_pending
        .swap(false, Ordering::AcqRel)
    {
        emit_metadata();
        request_event(
            foci_firmware::trace::events::EVENT_LEVEL_INFO,
            foci_firmware::trace::events::SUBSYSTEM_TMC_CONTROL,
            EVENT_FIRMWARE_BOOT,
            [0; 4],
            0,
        );
        request_event(
            foci_firmware::trace::events::EVENT_LEVEL_INFO,
            foci_firmware::trace::events::SUBSYSTEM_USB_TRACE,
            EVENT_USB_CONFIGURED,
            [0; 4],
            0,
        );
    }

    let Some((mode, oid)) = TRACE_MODE_REQUESTS.pop() else {
        return;
    };
    if !TRACE_MODE_REQUESTS.is_empty() {
        TRACE_CONTROL_WAKE.signal(());
    }
    if !TRACE_COUNTERS.configured.load(Ordering::Acquire) {
        accumulate_pre_config_mode_time(ms_since_boot(trace_now_ticks()));
    }
    TRACE_COUNTERS
        .trace_mode
        .store(mode as u8, Ordering::Relaxed);
    if is_configured() {
        emit_metadata();
    }
    crate::ankyra::replies::emit_trace_mode_state(oid, trace_mode());
}

const TRACE_DETAIL_SKIP_THRESHOLD_CYCLES: u32 = 372_000;

static mut TRACE_CLOCK_EXT: TraceClockExt = TraceClockExt::new();

/// Trace timing diagnostics for the board-global TIM5 CC1 step service.
pub(crate) struct TraceTiming {
    max_step_jitter_ticks: AtomicU32,
    min_deadline_slack_ticks: AtomicI32,
    last_served_step_tick_low: AtomicU32,
    cached_last_served_step_tick_low: AtomicU32,
    cached_last_served_step_tick_high: AtomicU32,
    missed_or_deferred_steps: AtomicU32,
}

impl TraceTiming {
    /// Create zeroed trace timing counters.
    const fn new() -> Self {
        Self {
            max_step_jitter_ticks: AtomicU32::new(0),
            min_deadline_slack_ticks: AtomicI32::new(i32::MAX),
            last_served_step_tick_low: AtomicU32::new(0),
            cached_last_served_step_tick_low: AtomicU32::new(0),
            cached_last_served_step_tick_high: AtomicU32::new(0),
            missed_or_deferred_steps: AtomicU32::new(0),
        }
    }
}

/// Board-global step timing values copied into critical trace samples.
#[derive(Clone, Copy)]
pub(crate) struct StepTimingSnapshot {
    pub(crate) step_timer_jitter: u32,
    pub(crate) min_deadline_slack: i32,
    pub(crate) last_served_step_tick: u64,
    pub(crate) missed_or_deferred_step_counter: u32,
}

pub(crate) static TRACE_TIMING: TraceTiming = TraceTiming::new();

fn compute_boot_id() -> u32 {
    const UID_BASE: *const u32 = 0x1FF1_E800 as *const u32;
    // SAFETY: UID registers are always readable at this address on STM32H7.
    let uid0 = unsafe { core::ptr::read_volatile(UID_BASE) };
    // SAFETY: UID registers are always readable at this address on STM32H7.
    let uid1 = unsafe { core::ptr::read_volatile(UID_BASE.add(1)) };
    // SAFETY: UID registers are always readable at this address on STM32H7.
    let uid2 = unsafe { core::ptr::read_volatile(UID_BASE.add(2)) };
    let tim5 = crate::helpers::lifecycle::read_tim5();
    let dwt = cortex_m::peripheral::DWT::cycle_count();
    uid0 ^ uid1 ^ uid2 ^ tim5 ^ dwt
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

fn store_last_served_step_tick_low(ticks_low: u32) {
    TRACE_TIMING
        .last_served_step_tick_low
        .store(ticks_low, Ordering::Relaxed);
}

fn store_cached_last_served_step_tick(ticks: u64) {
    TRACE_TIMING
        .cached_last_served_step_tick_low
        .store(ticks as u32, Ordering::Relaxed);
    TRACE_TIMING
        .cached_last_served_step_tick_high
        .store((ticks >> 32) as u32, Ordering::Relaxed);
}

/// Load the cached extended last-served step tick.
pub(crate) fn load_last_served_step_tick() -> u64 {
    u64::from(
        TRACE_TIMING
            .cached_last_served_step_tick_low
            .load(Ordering::Relaxed),
    ) | (u64::from(
        TRACE_TIMING
            .cached_last_served_step_tick_high
            .load(Ordering::Relaxed),
    ) << 32)
}

/// Store the last Klipper command-service tick.
pub(crate) fn store_last_klipper_command_service_tick(ticks: u64) {
    TRACE_PRESSURE.store_last_command_service_tick(ticks);
}

fn load_last_klipper_command_service_tick() -> u64 {
    TRACE_PRESSURE.last_command_service_tick()
}

/// Record the latest successful step queue depth for trace samples.
pub(crate) fn record_step_queue_depth(queue_len: usize) {
    TRACE_PRESSURE.record_queue_depth(queue_len);
}

fn take_jitter_and_slack() -> (u32, i32, bool) {
    let jitter = TRACE_TIMING
        .max_step_jitter_ticks
        .swap(0, Ordering::Relaxed);
    let slack = TRACE_TIMING
        .min_deadline_slack_ticks
        .swap(i32::MAX, Ordering::Relaxed);
    let event_observed = slack != i32::MAX;
    let normalized_slack = if event_observed { slack } else { 0 };
    (jitter, normalized_slack, event_observed)
}

/// Capture one board-global step timing snapshot for critical trace samples.
pub(crate) fn take_step_timing_snapshot() -> StepTimingSnapshot {
    let (step_timer_jitter, min_deadline_slack, event_observed) = take_jitter_and_slack();
    let last_served_step_tick = if event_observed {
        let low = TRACE_TIMING
            .last_served_step_tick_low
            .load(Ordering::Relaxed);
        let now_low = crate::helpers::lifecycle::read_tim5();
        let now_extended = extend_tim5_low(now_low);
        let elapsed = now_low.wrapping_sub(low);
        let extended = now_extended.wrapping_sub(u64::from(elapsed));
        store_cached_last_served_step_tick(extended);
        extended
    } else {
        load_last_served_step_tick()
    };

    StepTimingSnapshot {
        step_timer_jitter,
        min_deadline_slack,
        last_served_step_tick,
        missed_or_deferred_step_counter: TRACE_TIMING
            .missed_or_deferred_steps
            .load(Ordering::Relaxed),
    }
}

/// Record timing for a served step-timer deadline.
pub(crate) fn record_step_timing(scheduled: u32, actual: u32) {
    let jitter = actual.wrapping_sub(scheduled) as i32;
    let abs = jitter.unsigned_abs();
    let mut current = TRACE_TIMING.max_step_jitter_ticks.load(Ordering::Relaxed);
    while abs > current {
        match TRACE_TIMING.max_step_jitter_ticks.compare_exchange_weak(
            current,
            abs,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(next) => current = next,
        }
    }

    let slack = scheduled.wrapping_sub(actual) as i32;
    let mut min = TRACE_TIMING
        .min_deadline_slack_ticks
        .load(Ordering::Relaxed);
    while slack < min {
        match TRACE_TIMING.min_deadline_slack_ticks.compare_exchange_weak(
            min,
            slack,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(next) => min = next,
        }
    }

    store_last_served_step_tick_low(actual);
}

/// Count a missed deadline or IRQ-budget deferral in the step-timer path.
pub(crate) fn note_missed_or_deferred_step() {
    TRACE_TIMING
        .missed_or_deferred_steps
        .fetch_add(1, Ordering::Relaxed);
}

/// Record a TMC SPI error observed by trace-only reads.
pub(crate) fn record_spi_error() {
    TRACE_PRESSURE.record_spi_error();
}

/// Record a TMC STATUS pin interrupt for trace status frames.
pub(crate) fn record_status_interrupt() {
    TRACE_PRESSURE.record_status_interrupt();
}

/// Record measured trace sample await-span duration.
pub(crate) fn record_spi_sample_cycles(cycles: u32) {
    TRACE_PRESSURE.record_spi_sample_cycles(cycles, TRACE_DETAIL_SKIP_THRESHOLD_CYCLES);
}

/// Record the trace boot timestamp after TIM5 starts.
pub(crate) fn note_trace_boot() {
    let boot = trace_now_ticks();
    TRACE_COUNTERS
        .boot_id
        .store(compute_boot_id(), Ordering::Relaxed);
    store_split_u64(
        &TRACE_COUNTERS.boot_tick_low,
        &TRACE_COUNTERS.boot_tick_high,
        boot,
    );
}

/// Return true when trace producers should emit frames.
pub(crate) fn is_configured() -> bool {
    TRACE_COUNTERS.configured.load(Ordering::Relaxed) && foci_usb_stm32::is_configured()
}

/// Mark the trace stream configured after USB configuration.
///
/// Runs in the USB task, which must not touch the control task's motion-queue producer, so
/// the metadata baseline and its events are left to `service_trace_mode_changes`.
pub(crate) fn note_usb_configured() {
    note_trace_usb_configured_time();
    TRACE_COUNTERS.configured.store(true, Ordering::Release);
    TRACE_COUNTERS
        .baseline_pending
        .store(true, Ordering::Release);
    TRACE_CONTROL_WAKE.signal(());
}

fn header_at_timestamp(
    frame_type: u8,
    payload_len: usize,
    source_kind: u8,
    source_index: u8,
    flags: u8,
    timestamp_ticks: u64,
) -> foci_firmware::trace::stream::FrameHeader {
    foci_firmware::trace::stream::FrameHeader {
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

fn header(
    frame_type: u8,
    payload_len: usize,
    source_kind: u8,
    source_index: u8,
    flags: u8,
) -> foci_firmware::trace::stream::FrameHeader {
    header_at_timestamp(
        frame_type,
        payload_len,
        source_kind,
        source_index,
        flags,
        trace_now_ticks(),
    )
}

fn motion_flags() -> u8 {
    if TRACE_COUNTERS
        .motion_drop_gap
        .swap(false, Ordering::Relaxed)
    {
        foci_firmware::trace::stream::FLAG_DROP_GAP_BEFORE_THIS_FRAME
    } else {
        0
    }
}

fn enqueue_frame(frame: FrameBuf) {
    enqueue_motion(frame);
}

fn enqueue_motion(frame: FrameBuf) {
    // SAFETY: TMC control is the sole motion-frame producer. The USB task owns
    // the separately split consumer endpoint.
    unsafe {
        let producer = motion_producer();
        if producer.enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.motion_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS
                .motion_drop_gap
                .store(true, Ordering::Relaxed);
        } else {
            crate::transport::USB_WAKE.signal(());
        }

        let len = producer.len() as u16;
        transport::update_high_water(&TRACE_COUNTERS.high_water, len);
    }
}

fn enqueue_event(frame: FrameBuf) {
    // SAFETY: Pending events are converted, enqueued, and consumed only by the
    // priority-2 USB task.
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
    let queued = cortex_m::interrupt::free(|_| unsafe {
        let queue = &mut *core::ptr::addr_of_mut!(PENDING_EVENTS);
        if queue.push_back(event).is_err() {
            TRACE_COUNTERS.event_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS.event_drop_gap.store(true, Ordering::Relaxed);
            false
        } else {
            true
        }
    });
    if queued {
        crate::transport::USB_WAKE.signal(());
    }
}

/// Translate a shared commissioning event into a USB trace event frame.
pub(crate) fn request_commission_event(
    source_index: u8,
    event: foci_firmware::trace::events::CommissionTraceEvent,
) {
    request_event(
        event.level as u8,
        foci_firmware::trace::events::SUBSYSTEM_COMMISSIONING,
        event.event_id,
        [
            i32::from(source_index),
            i32::from(foci_firmware::trace::events::engine_kind_wire_code(
                event.engine_kind,
            )),
            event.payload[0],
            event.payload[1],
        ],
        2 + event.payload_count.min(2),
    );
}

fn enqueue_event_frame(event: PendingEvent) {
    use foci_firmware::trace::stream::{
        EVENT_PAYLOAD_LEN, EventSample, FLAG_DROP_GAP_BEFORE_THIS_FRAME, FRAME_TYPE_EVENT,
        SOURCE_INDEX_NA, SOURCE_KIND_BOARD,
    };

    let sample = EventSample {
        level: event.level,
        subsystem_id: event.subsystem_id,
        event_id: event.event_id,
        arg_count: event.arg_count,
        args: event.args,
    };
    let flags = if TRACE_COUNTERS.event_drop_gap.swap(false, Ordering::Relaxed) {
        FLAG_DROP_GAP_BEFORE_THIS_FRAME
    } else {
        0
    };
    let frame = transport::build_event_frame(
        header_at_timestamp(
            FRAME_TYPE_EVENT,
            EVENT_PAYLOAD_LEN,
            SOURCE_KIND_BOARD,
            SOURCE_INDEX_NA,
            flags,
            event.timestamp_ticks,
        ),
        sample,
    );
    enqueue_event(frame);
}

fn motion_queue_len() -> usize {
    // SAFETY: This runs in the P1 producer context.
    unsafe { motion_producer().len() }
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

/// Emit a one-shot event with the measured trace SPI sample duration.
pub(crate) fn request_spi_duration_event_if_ready() {
    if !is_configured() {
        return;
    }
    if let Some(event) = TRACE_PRESSURE.take_spi_duration_event() {
        request_event(
            event.level,
            event.subsystem_id,
            event.event_id,
            event.args,
            event.arg_count,
        );
    }
}

/// Emit stream metadata. Control-task context only: it uses the motion-queue producer.
pub(crate) fn emit_metadata() {
    use foci_firmware::trace::stream::{
        FRAME_TYPE_METADATA, FrameBuf, SOURCE_INDEX_NA, SOURCE_KIND_BOARD,
    };

    if !is_configured() {
        return;
    }

    let Ok(sources) = transport::canonical_sources(crate::board_config::TMC_CHANNEL_COUNT) else {
        return;
    };
    let Ok(payload) = transport::build_metadata_payload(transport::TraceMetadataInputs {
        skipped_sample_counter_count: 3,
        clock_freq: crate::ankyra::constants::CLOCK_FREQ,
        critical_hz: trace_mode().critical_hz(),
        control_detail_hz: CONTROL_DETAIL_HZ,
        status_hz: STATUS_HZ,
        pre_configuration_elapsed_ms: TRACE_COUNTERS.pre_config_elapsed_ms.load(Ordering::Relaxed),
        pre_configuration_skipped_critical: TRACE_COUNTERS
            .pre_config_skipped_critical
            .load(Ordering::Relaxed),
        pre_configuration_skipped_control_detail: TRACE_COUNTERS
            .pre_config_skipped_control_detail
            .load(Ordering::Relaxed),
        pre_configuration_skipped_status: TRACE_COUNTERS
            .pre_config_skipped_status
            .load(Ordering::Relaxed),
        firmware_id: "ouroboros-fw",
        product: crate::board_config::USB_PRODUCT,
        serial: trace_serial(),
        sources: sources.as_slice(),
    }) else {
        return;
    };

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
    enqueue_motion(frame);
    for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
        crate::tmc_control::board::tmc_state(channel)
            .control_state
            .mark_snapshot();
    }
}

const CLOSED_LOOP_VELOCITY_SELECTION: u8 = {
    let selection = closed_loop_activation::CLOSED_LOOP_VELOCITY_SELECTION;
    assert!(selection <= u8::MAX as u32);
    selection as u8
};

fn control_state_sample(
    header: foci_firmware::trace::control_state::ControlStateHeader,
    state: &foci_firmware::state::tmc::TmcChannelState,
    inner_gains: Option<foci_firmware::pid::InnerLoopGains>,
) -> foci_firmware::trace::control_state::ControlStateSample {
    use core::sync::atomic::Ordering::Relaxed;

    let outer = state.load_outer_gains(Relaxed).ok();
    let outer_raw = |gain: fn(&foci_firmware::pid::OuterLoopGains) -> u16| {
        outer.as_ref().map_or(0, |g| i32::from(gain(g)))
    };
    let inner_raw = |gain: fn(&foci_firmware::pid::InnerLoopGains) -> u16| {
        inner_gains.as_ref().map_or(0, |g| i32::from(gain(g)))
    };
    foci_firmware::trace::control_state::ControlStateSample {
        header,
        encoder_counts_per_rev: state.encoder_counts_per_rev().unwrap_or(0),
        pwm_hz: crate::board_config::TMC_CONFIG.pwm_freq_hz(),
        velocity_meter_selection: CLOSED_LOOP_VELOCITY_SELECTION,
        velocity_filter_hz: state.velocity_filter_hz.load_or_zero(Relaxed),
        torque_filter_hz: state.torque_filter_hz.load_or_zero(Relaxed),
        position_filter_hz: state.position_filter_hz.load_or_zero(Relaxed),
        flux_filter_hz: state.flux_filter_hz.load_or_zero(Relaxed),
        velocity_p: outer_raw(|g| g.velocity_p.raw()),
        velocity_i: outer_raw(|g| g.velocity_i.raw()),
        position_p: outer_raw(|g| g.position_p.raw()),
        position_i: outer_raw(|g| g.position_i.raw()),
        torque_p: inner_raw(|g| g.torque_p.raw()),
        torque_i: inner_raw(|g| g.torque_i.raw()),
        flux_p: inner_raw(|g| g.flux_p.raw()),
        flux_i: inner_raw(|g| g.flux_i.raw()),
        velocity_ff_enable: u8::from(state.velocity_feedforward.load(Relaxed)),
        velocity_ff_permille: state.velocity_feedforward_gain_permille.load(Relaxed),
        accel_ff_enable: u8::from(state.accel_feedforward.load(Relaxed)),
        accel_ff_gain: state.accel_feedforward_accel_gain_permille.load(Relaxed),
        decel_ff_gain: state.accel_feedforward_decel_gain_permille.load(Relaxed),
        run_current: state.run_current_ma_cfg.load(Relaxed),
        voltage_limit: state
            .voltage_limit_cfg
            .load(Relaxed)
            .min(u32::from(u16::MAX)) as u16,
    }
}

/// Queue one control-state frame, or refuse without consuming a sequence number or
/// counting a drop when the motion queue has no room.
fn try_enqueue_control_state(
    channel: u8,
    sample: &foci_firmware::trace::control_state::ControlStateSample,
) -> Result<(), QueueFull> {
    use foci_firmware::trace::stream::{
        CONTROL_STATE_PAYLOAD_LEN, FRAME_TYPE_CONTROL_STATE, FrameBuf, SOURCE_KIND_FOCI,
    };

    // SAFETY: TMC control is the sole motion-frame producer, so room seen by `ready` cannot
    // be taken before the enqueue below.
    let producer = unsafe { motion_producer() };
    if !producer.ready() {
        return Err(QueueFull);
    }
    let mut payload = [0u8; CONTROL_STATE_PAYLOAD_LEN];
    foci_firmware::trace::control_state::encode_control_state_payload(sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_CONTROL_STATE,
        CONTROL_STATE_PAYLOAD_LEN,
        SOURCE_KIND_FOCI,
        channel,
        motion_flags(),
    ));
    frame
        .payload_mut(CONTROL_STATE_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_motion(frame);
    Ok(())
}

/// Control-task context, once per channel at the top of each control-loop pass, before the
/// driver runs.
///
/// Stamps every mutator recorded since the last pass with the current trace tick, then queues
/// the snapshot and mutator frames the channel owes; anything the motion queue cannot take
/// stays owed.
pub(crate) fn emit_control_state_frame(
    channel: u8,
    state: &foci_firmware::state::tmc::TmcChannelState,
    inner_gains: Option<foci_firmware::pid::InnerLoopGains>,
) {
    state.control_state.stamp(trace_now_ticks());
    if !is_configured() {
        return;
    }
    let outcome = emit_pending(&state.control_state, |header| {
        try_enqueue_control_state(channel, &control_state_sample(header, state, inner_gains))
    });
    if outcome == EmitOutcome::Capped {
        TRACE_CONTROL_WAKE.signal(());
    }
}

async fn read_register_trace<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    register: foci_tmc4671::registers::Register,
    sample_flags: &mut u8,
) -> u32
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    match tmc.read_register(register).await {
        Ok(value) => value,
        Err(_) => {
            record_spi_error();
            *sample_flags |= foci_firmware::trace::stream::SAMPLE_FLAG_SPI_ERROR;
            0
        }
    }
}

async fn read_sub_register_trace<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    register: foci_tmc4671::registers::SubRegister,
    sample_flags: &mut u8,
) -> u32
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    match tmc.read_sub_register(register).await {
        Ok(value) => value,
        Err(_) => {
            record_spi_error();
            *sample_flags |= foci_firmware::trace::stream::SAMPLE_FLAG_SPI_ERROR;
            0
        }
    }
}

/// Board-local inputs for one Ouroboros trace sample.
#[derive(Clone, Copy)]
pub(crate) struct TraceSampleContext<'a> {
    /// TMC channel and FOCI trace source index.
    pub(crate) channel: u8,
    /// Borrowed TMC and stepper state for the selected channel.
    pub(crate) refs: foci_firmware::tmc_control::TmcControlStateRefs<'a>,
    /// Whether the selected channel's runtime currently considers the motor enabled.
    pub(crate) motor_enabled: bool,
    /// Optional board-global step timing snapshot for the critical frame.
    pub(crate) step_timing: Option<StepTimingSnapshot>,
}

async fn emit_control_detail<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    channel: u8,
    refs: foci_firmware::tmc_control::TmcControlStateRefs<'_>,
    control_tick: u32,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace::stream::{
        CONTROL_DETAIL_PAYLOAD_LEN, ControlDetailSample, FRAME_TYPE_CONTROL_DETAIL, FrameBuf,
        SOURCE_KIND_FOCI, control_detail_due, encode_control_detail_payload,
    };
    use foci_tmc4671::registers::{
        INTERIM_PIDIN_TARGET_VELOCITY, INTERIM_PIDOUT_TARGET_FLUX, INTERIM_PIDOUT_TARGET_TORQUE,
        INTERIM_PIDOUT_TARGET_VELOCITY,
    };

    if !control_detail_due(
        control_tick,
        motion_queue_len(),
        MOTION_QUEUE_DEPTH,
        TRACE_PRESSURE.take_control_detail_skip(),
    ) {
        return;
    }
    let mut detail_flags = 0u8;
    let detail = ControlDetailSample {
        interim_pidout_target_torque: read_sub_register_trace(
            tmc,
            INTERIM_PIDOUT_TARGET_TORQUE,
            &mut detail_flags,
        )
        .await,
        interim_pidout_target_flux: read_sub_register_trace(
            tmc,
            INTERIM_PIDOUT_TARGET_FLUX,
            &mut detail_flags,
        )
        .await,
        interim_pidin_target_velocity: read_sub_register_trace(
            tmc,
            INTERIM_PIDIN_TARGET_VELOCITY,
            &mut detail_flags,
        )
        .await,
        interim_pidout_target_velocity: read_sub_register_trace(
            tmc,
            INTERIM_PIDOUT_TARGET_VELOCITY,
            &mut detail_flags,
        )
        .await,
        pidout_uq_ud_limits: refs.tmc.voltage_limit_cfg.load(Relaxed),
        pid_torque_flux_limits: refs.tmc.load_current_limit(Relaxed).register_value(),
        pid_velocity_limit: refs.tmc.velocity_limit_cfg.load(Relaxed),
    };
    let mut payload = [0u8; CONTROL_DETAIL_PAYLOAD_LEN];
    encode_control_detail_payload(&detail, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_CONTROL_DETAIL,
        CONTROL_DETAIL_PAYLOAD_LEN,
        SOURCE_KIND_FOCI,
        channel,
        motion_flags(),
    ));
    frame
        .payload_mut(CONTROL_DETAIL_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_frame(frame);
}

/// Sample one channel's TMC state for the trace stream.
///
/// Returns the DWT cycles this sample spent; the caller sums both channels and records the
/// iteration total against the threshold.
pub(crate) async fn sample_tick<B>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    context: TraceSampleContext<'_>,
    inputs: TraceSampleInputs,
) -> u32
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace::stream::{
        CRITICAL_PAYLOAD_LEN, CURRENT_PATH_PAYLOAD_LEN, CriticalSample, CurrentPathSample,
        FRAME_TYPE_CRITICAL, FRAME_TYPE_CURRENT_PATH, FrameBuf, SOURCE_KIND_FOCI, current_path_due,
        encode_critical_payload, encode_current_path_payload,
    };
    use foci_tmc4671::registers::{
        INTERIM_ADC_I1_I0, INTERIM_FOC_IQ_ID, INTERIM_FOC_IWY_IUX, INTERIM_FOC_UQ_UD,
        INTERIM_FOC_UQ_UD_LIMITED, INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL,
        PID_ERROR_PID_POSITION_ERROR, PID_ERROR_PID_VELOCITY_ERROR, PID_POSITION_ACTUAL,
        PID_VELOCITY_ACTUAL,
    };

    if !is_configured() {
        return 0;
    }

    let channel = context.channel;
    let refs = context.refs;
    let spi_sample_start = cortex_m::peripheral::DWT::cycle_count();
    let mode = trace_mode();
    if !mode.critical_sampling() {
        emit_control_detail(tmc, channel, refs, inputs.control_tick).await;
        return cortex_m::peripheral::DWT::cycle_count().wrapping_sub(spi_sample_start);
    }
    let mut flags = 0u8;
    let step_position = refs.stepper.position.load(Relaxed);
    let position_actual = read_register_trace(tmc, PID_POSITION_ACTUAL, &mut flags).await as i32;
    let position_error =
        read_sub_register_trace(tmc, PID_ERROR_PID_POSITION_ERROR, &mut flags).await as i32;
    let pid_velocity_actual =
        read_register_trace(tmc, PID_VELOCITY_ACTUAL, &mut flags).await as i32;
    let velocity_error =
        read_sub_register_trace(tmc, PID_ERROR_PID_VELOCITY_ERROR, &mut flags).await as i32;
    let interim_pid_torque_target_actual =
        read_sub_register_trace(tmc, INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL, &mut flags).await;
    let foc_uq_ud = read_sub_register_trace(tmc, INTERIM_FOC_UQ_UD, &mut flags).await;
    let foc_uq_ud_limited =
        read_sub_register_trace(tmc, INTERIM_FOC_UQ_UD_LIMITED, &mut flags).await;
    let (
        step_timer_jitter,
        min_deadline_slack,
        last_served_step_tick,
        missed_or_deferred_step_counter,
    ) = if let Some(snapshot) = context.step_timing {
        (
            snapshot.step_timer_jitter,
            snapshot.min_deadline_slack,
            snapshot.last_served_step_tick,
            snapshot.missed_or_deferred_step_counter,
        )
    } else {
        (0, 0, 0, 0)
    };

    let critical = CriticalSample {
        step_position,
        tmc_position_actual: position_actual,
        position_error,
        target_velocity: inputs.command_velocity_reference,
        pid_velocity_actual,
        cached_pid_velocity_offset: inputs.cached_velocity_offset,
        velocity_error,
        enable_and_direction: (if context.motor_enabled { 0x80 } else { 0 })
            | (u8::from(refs.stepper.direction.load(Relaxed)) & 0x01),
        sample_flags: flags,
        timer_queue_depth: TRACE_PRESSURE.latest_queue_depth(),
        step_timer_jitter,
        min_deadline_slack,
        last_served_step_tick,
        last_klipper_command_service_tick: load_last_klipper_command_service_tick(),
        missed_or_deferred_step_counter,
        interim_pidin_target_torque: 0,
        interim_pid_torque_target_actual,
        interim_foc_uq_ud: foc_uq_ud,
        interim_foc_uq_ud_limited: foc_uq_ud_limited,
        cached_accel_torque_offset: inputs.cached_accel_torque_offset,
        accel_feedforward_gain_permille: inputs.accel_feedforward_gain_permille,
        accel_feedforward_flags: inputs.accel_feedforward_flags,
        cached_velocity_clamped: inputs.cached_velocity_clamped,
    };
    let mut payload = [0u8; CRITICAL_PAYLOAD_LEN];
    encode_critical_payload(&critical, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_CRITICAL,
        CRITICAL_PAYLOAD_LEN,
        SOURCE_KIND_FOCI,
        channel,
        motion_flags(),
    ));
    frame
        .payload_mut(CRITICAL_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_frame(frame);

    if current_path_due(
        mode.current_path(),
        inputs.control_tick,
        motion_queue_len(),
        MOTION_QUEUE_DEPTH,
    ) {
        let mut current_path_flags = 0u8;
        let interim_foc_iq_id =
            read_sub_register_trace(tmc, INTERIM_FOC_IQ_ID, &mut current_path_flags).await;
        let interim_foc_iwy_iux =
            read_sub_register_trace(tmc, INTERIM_FOC_IWY_IUX, &mut current_path_flags).await;
        let interim_adc_i1_i0 =
            read_sub_register_trace(tmc, INTERIM_ADC_I1_I0, &mut current_path_flags).await;
        let current_path = CurrentPathSample {
            sample_flags: current_path_flags,
            interim_foc_iq_id,
            interim_foc_iwy_iux,
            interim_adc_i1_i0,
        };
        let mut payload = [0u8; CURRENT_PATH_PAYLOAD_LEN];
        encode_current_path_payload(&current_path, &mut payload);
        let mut frame = FrameBuf::new();
        frame.write_header(header(
            FRAME_TYPE_CURRENT_PATH,
            CURRENT_PATH_PAYLOAD_LEN,
            SOURCE_KIND_FOCI,
            channel,
            motion_flags(),
        ));
        frame
            .payload_mut(CURRENT_PATH_PAYLOAD_LEN)
            .copy_from_slice(&payload);
        enqueue_frame(frame);
    }

    emit_control_detail(tmc, channel, refs, inputs.control_tick).await;

    cortex_m::peripheral::DWT::cycle_count().wrapping_sub(spi_sample_start)
}

/// Sample the selected channel during commissioning closed-loop entry.
pub(crate) async fn sample_commission_closed_loop_activation<B, R, D>(
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    context: TraceSampleContext<'_>,
    control_tick: u32,
    remaining_budget_us: R,
    past_deadline: D,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    R: Fn() -> u64,
    D: Fn() -> bool,
{
    let trace_configured = is_configured();
    let past_deadline = trace_configured && past_deadline();
    let remaining_budget_us = if trace_configured && !past_deadline {
        remaining_budget_us()
    } else {
        0
    };

    match commissioning_trace_sample_plan(
        trace_configured,
        trace_mode(),
        past_deadline,
        remaining_budget_us,
        control_tick,
    ) {
        TraceSamplePlan::SkipUnconfigured => {}
        TraceSamplePlan::DropForBudget => {
            TRACE_COUNTERS.motion_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS
                .motion_drop_gap
                .store(true, Ordering::Relaxed);
        }
        TraceSamplePlan::Sample(inputs) => {
            record_spi_sample_cycles(sample_tick(tmc, context, inputs).await);
        }
    }
}

fn emit_one_status(
    slot: usize,
    source_kind: u8,
    source_index: u8,
    sample: foci_firmware::trace::stream::StatusSample,
    force: bool,
) {
    use foci_firmware::trace::stream::{FRAME_TYPE_STATUS, STATUS_PAYLOAD_LEN};

    let previous = unsafe {
        let ptr = core::ptr::addr_of!(crate::state::usb_trace::LAST_STATUS)
            .cast::<transport::StatusSnapshot>();
        core::ptr::read(ptr.add(slot))
    };
    if !transport::should_emit_status(&previous, source_kind, source_index, &sample, force) {
        return;
    }

    let frame = transport::build_status_frame(
        header(
            FRAME_TYPE_STATUS,
            STATUS_PAYLOAD_LEN,
            source_kind,
            source_index,
            motion_flags(),
        ),
        sample,
    );
    enqueue_frame(frame);
    unsafe {
        let ptr = core::ptr::addr_of_mut!(crate::state::usb_trace::LAST_STATUS)
            .cast::<transport::StatusSnapshot>();
        core::ptr::write(
            ptr.add(slot),
            transport::StatusSnapshot {
                source_kind,
                source_index,
                sample,
            },
        );
    }
}

pub(crate) async fn emit_status_frames<B>(
    channel: u8,
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    control_tick: u32,
    regular_tick: bool,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace::stream::{
        SOURCE_INDEX_NA, SOURCE_KIND_BOARD, SOURCE_KIND_FOCI, SOURCE_KIND_KLIPPER, SOURCE_KIND_USB,
        StatusSample,
    };
    use foci_tmc4671::registers::STATUS_FLAGS;

    if !is_configured() {
        return;
    }

    if !TRACE_PRESSURE.should_emit_status(control_tick, regular_tick, channel == 0) {
        return;
    }

    let mut flags = 0u8;
    let status_flags = read_register_trace(tmc, STATUS_FLAGS, &mut flags).await;
    const STATUS_PID_LIMIT_MASK: u32 = 0x0000_FFFF;
    const STATUS_PWM_LIMIT_MASK: u32 = (1 << 24) | (1 << 25);
    const STATUS_ADC_CLIP_MASK: u32 = (1 << 26) | (1 << 27);
    let clipping_flags =
        status_flags & (STATUS_PID_LIMIT_MASK | STATUS_PWM_LIMIT_MASK | STATUS_ADC_CLIP_MASK);
    let counters = TRACE_PRESSURE.status_counters(TracePressureStatusInputs {
        endpoint_not_ready_count: TRACE_COUNTERS.endpoint_not_ready.load(Relaxed),
        usb_trace_queue_high_water: TRACE_COUNTERS.high_water.load(Relaxed),
        dropped_motion_sample_count: TRACE_COUNTERS.motion_drops.load(Relaxed),
        dropped_event_log_count: TRACE_COUNTERS.event_drops.load(Relaxed),
        missed_or_deferred_step_count: TRACE_TIMING.missed_or_deferred_steps.load(Relaxed),
    });
    let foci = StatusSample {
        status_flags,
        status_interrupt_count: counters.status_interrupt_count,
        clipping_flags,
        spi_error_count: counters.spi_error_count,
        watchdog_or_step_overrun_count: counters.watchdog_or_step_overrun_count,
        ..StatusSample::default()
    };
    let klipper = StatusSample {
        klipper_rx_queue_depth: counters.latest_queue_depth,
        ..StatusSample::default()
    };

    if channel == 0 {
        let board = StatusSample {
            status_flags,
            status_interrupt_count: counters.status_interrupt_count,
            clipping_flags,
            spi_error_count: counters.spi_error_count,
            watchdog_or_step_overrun_count: counters.watchdog_or_step_overrun_count,
            klipper_rx_queue_depth: counters.latest_queue_depth,
            usb_trace_queue_high_water: counters.usb_trace_queue_high_water,
            dropped_motion_sample_count: counters.dropped_motion_sample_count,
            dropped_event_log_count: counters.dropped_event_log_count,
            endpoint_not_ready_count: counters.endpoint_not_ready_count,
            suppressed_status_change_count: counters.suppressed_status_change_count,
            trace_deadline_pressure_count: counters.trace_deadline_pressure_count,
            max_spi_sample_cycles: counters.max_spi_sample_cycles,
        };
        let usb = StatusSample {
            usb_trace_queue_high_water: counters.usb_trace_queue_high_water,
            dropped_motion_sample_count: counters.dropped_motion_sample_count,
            dropped_event_log_count: counters.dropped_event_log_count,
            endpoint_not_ready_count: counters.endpoint_not_ready_count,
            suppressed_status_change_count: counters.suppressed_status_change_count,
            ..StatusSample::default()
        };
        emit_one_status(
            transport::board_status_slot(),
            SOURCE_KIND_BOARD,
            SOURCE_INDEX_NA,
            board,
            regular_tick,
        );
        emit_one_status(
            transport::usb_status_slot(),
            SOURCE_KIND_USB,
            SOURCE_INDEX_NA,
            usb,
            false,
        );
        emit_one_status(
            transport::klipper_status_slot(),
            SOURCE_KIND_KLIPPER,
            SOURCE_INDEX_NA,
            klipper,
            false,
        );
    }

    emit_one_status(
        transport::foci_status_slot(channel),
        SOURCE_KIND_FOCI,
        channel,
        foci,
        false,
    );
}

fn dequeue_next_frame() -> Option<QueuedFrame> {
    // SAFETY: The USB task is the only trace consumer and the only event-queue
    // owner. The motion producer is isolated behind its split SPSC endpoint.
    unsafe {
        let motion = motion_consumer();
        let event = event_queue();
        let motion_seq = motion.peek().map(|queued| frame_sequence(&queued.frame));
        let event_seq = event.peek().map(|queued| frame_sequence(&queued.frame));

        match transport::select_next_queue(motion_seq, event_seq) {
            TraceQueueSelection::Motion => motion.dequeue(),
            TraceQueueSelection::Event => event.dequeue(),
            TraceQueueSelection::None => None,
        }
    }
}

/// Drain at most one 64-byte trace packet to USB.
pub(crate) fn drain_once() {
    // SAFETY: This is the only trace consumer. Producers only enqueue complete
    // `QueuedFrame` values. `current_tx` is therefore single-consumer state and
    // preserves partially sent frames across calls.
    unsafe {
        let current_tx = current_tx();
        if current_tx.is_none() {
            *current_tx = dequeue_next_frame();
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
