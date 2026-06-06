//! Ouroboros USB trace stream integration.

#![cfg(feature = "trace")]

use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};

use foci_firmware::trace::events::TraceClockExt;
use foci_firmware::trace::pressure::{TracePressureRuntime, TracePressureStatusInputs};
use foci_firmware::trace::stream::{FrameBuf, METADATA_PAYLOAD_MAX, frame_sequence};
use foci_firmware::trace::transport::{self, TraceQueueSelection};

use crate::state::usb_trace::{
    MOTION_QUEUE_DEPTH, PENDING_EVENTS, PendingEvent, QueuedFrame, TRACE_COUNTERS, current_tx,
    event_queue, motion_queue,
};

/// Product string used by trace builds.
pub(crate) const TRACE_PRODUCT: &str = "FOCI Ouroboros (trace)";

static TRACE_PRESSURE: TracePressureRuntime = TracePressureRuntime::new();

/// Enable the lower-rate raw current-path diagnostic frame.
const CURRENT_PATH_DIAGNOSTIC_ENABLED: bool = cfg!(feature = "trace-current-path");

// OpenFFBoard uses 120_000 cycles at 168 MHz, about 714 us. Scale that
// wall-time budget to the 520 MHz H723 DWT clock: 120_000 * 520 / 168 =
// 371_429, rounded up. Do not derive this from CLOCK_FREQ; CLOCK_FREQ is TIM5.
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
    // SAFETY: Trace producers enqueue complete frames; the USB task is the
    // only consumer. Producers never dequeue from the motion queue.
    unsafe {
        let queue = motion_queue();
        if queue.enqueue(QueuedFrame::new(frame)).is_err() {
            TRACE_COUNTERS.motion_drops.fetch_add(1, Ordering::Relaxed);
            TRACE_COUNTERS
                .motion_drop_gap
                .store(true, Ordering::Relaxed);
        } else {
            crate::transport::USB_WAKE.signal(());
        }

        let len = queue.len() as u16;
        transport::update_high_water(&TRACE_COUNTERS.high_water, len);
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
    use foci_firmware::trace::stream::*;

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
    // SAFETY: Trace producers may inspect the queue length for decimation. The
    // USB task remains the sole consumer, and enqueue paths retain ownership of
    // actual queue mutation.
    unsafe { motion_queue().len() }
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

fn push_str(out: &mut heapless::Vec<u8, METADATA_PAYLOAD_MAX>, value: &str) {
    let bytes = value.as_bytes();
    let len = bytes.len().min(31);
    let _ = out.push(len as u8);
    let _ = out.extend_from_slice(&bytes[..len]);
}

/// Emit stream metadata.
pub(crate) fn emit_metadata() {
    use foci_firmware::trace::stream::*;

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
    enqueue_motion(frame);
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

pub(crate) fn emit_phase_advance(
    channel: u8,
    phase_advance_counts: i16,
    phase_advance_flags: u8,
    gain_ppm: i32,
    max_counts: u16,
    deadband: u16,
) {
    use foci_firmware::trace::stream::*;

    if !is_configured() {
        return;
    }

    let sample = PhaseAdvanceSample {
        phase_advance_counts,
        phase_advance_flags,
        reserved: 0,
        gain_ppm,
        max_counts,
        deadband,
    };
    let mut payload = [0u8; PHASE_ADVANCE_PAYLOAD_LEN];
    encode_phase_advance_payload(&sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header(
        FRAME_TYPE_PHASE_ADVANCE,
        PHASE_ADVANCE_PAYLOAD_LEN,
        SOURCE_KIND_FOCI,
        channel,
        motion_flags(),
    ));
    frame
        .payload_mut(PHASE_ADVANCE_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    enqueue_frame(frame);
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn sample_tick<B>(
    channel: u8,
    refs: foci_firmware::tmc_control::TmcControlStateRefs<'_>,
    tmc: &mut foci_tmc4671::spi::Tmc4671Spi<B>,
    motor_enabled: bool,
    command_velocity_reference: i32,
    cached_velocity_offset: i32,
    cached_velocity_transient_offset: i32,
    cached_accel_torque_offset: i32,
    accel_feedforward_gain_permille: u16,
    accel_feedforward_flags: u8,
    step_timing: Option<StepTimingSnapshot>,
    control_tick: u32,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::trace::stream::*;
    use foci_tmc4671::registers::*;

    if !is_configured() {
        return;
    }

    let spi_sample_start = cortex_m::peripheral::DWT::cycle_count();
    let mut flags = 0u8;
    let step_position = refs.stepper.position.load(Relaxed);
    let position_actual = read_register_trace(tmc, PID_POSITION_ACTUAL, &mut flags).await as i32;
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
    ) = if let Some(snapshot) = step_timing {
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
        position_error: step_position.wrapping_sub(position_actual),
        target_velocity: command_velocity_reference,
        pid_velocity_actual: 0,
        cached_pid_velocity_offset: cached_velocity_offset,
        velocity_error: 0,
        enable_and_direction: (if motor_enabled { 0x80 } else { 0 })
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
        cached_accel_torque_offset,
        accel_feedforward_gain_permille,
        accel_feedforward_flags,
        cached_velocity_transient_offset,
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
        CURRENT_PATH_DIAGNOSTIC_ENABLED,
        control_tick,
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

    let emit_control_detail = control_detail_due(
        control_tick,
        motion_queue_len(),
        MOTION_QUEUE_DEPTH,
        TRACE_PRESSURE.take_control_detail_skip(),
    );
    if emit_control_detail {
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
            interim_pidin_target_velocity: 0,
            interim_pidout_target_velocity: 0,
            pidout_uq_ud_limits: refs.tmc.voltage_limit_cfg.load(Relaxed),
            pid_torque_flux_limits: refs.tmc.current_limit_reg.load(Relaxed),
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

    let spi_sample_cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(spi_sample_start);
    record_spi_sample_cycles(spi_sample_cycles);
}

fn emit_one_status(
    slot: usize,
    source_kind: u8,
    source_index: u8,
    sample: foci_firmware::trace::stream::StatusSample,
    force: bool,
) {
    use foci_firmware::trace::stream::*;

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
    use foci_firmware::trace::stream::*;
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

    if channel == 0 {
        // Trace deadline pressure is telemetry degradation, so it is reported
        // with endpoint-not-ready pressure instead of motion watchdog/step overruns.
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
        };
        emit_one_status(0, SOURCE_KIND_BOARD, SOURCE_INDEX_NA, board, regular_tick);
    }

    emit_one_status(1 + channel as usize, SOURCE_KIND_FOCI, channel, foci, false);

    if channel == 0 {
        let usb = StatusSample {
            usb_trace_queue_high_water: counters.usb_trace_queue_high_water,
            dropped_motion_sample_count: counters.dropped_motion_sample_count,
            dropped_event_log_count: counters.dropped_event_log_count,
            endpoint_not_ready_count: counters.endpoint_not_ready_count,
            suppressed_status_change_count: counters.suppressed_status_change_count,
            ..StatusSample::default()
        };
        emit_one_status(3, SOURCE_KIND_USB, SOURCE_INDEX_NA, usb, false);
    }
}

fn dequeue_next_frame() -> Option<QueuedFrame> {
    // SAFETY: The USB task is the only trace consumer. Peeking copies sequence
    // numbers from queue heads before dequeuing from exactly one queue.
    unsafe {
        let motion = motion_queue();
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
