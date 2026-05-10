//! Periodic board heartbeat tasks: LED blink, stats output, and
//! trsync-report drain. Each function is the body of an RTIC task
//! declared in `main.rs`; the task shells handle only RTIC plumbing
//! (Mono, local-resource binding) and delegate all work here.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};
use embassy_stm32::peripherals;
use embassy_stm32::wdg::IndependentWatchdog;
use foci_firmware::ankyra::replies::{Stats, TrsyncState};

/// SumSq base used by `stats`. Must match `STATS_SUMSQ_BASE` advertised
/// to Klipper via `klipper_constant`.
const STATS_SUMSQ_BASE: u32 = 256;

/// Accumulator state for the stats task between ticks.
pub struct StatsAccumulator {
    count: u32,
    sum: u32,
    sumsq: u32,
    last_clock: u32,
}

impl StatsAccumulator {
    pub fn new(initial_clock: u32) -> Self {
        Self {
            count: 0,
            sum: 0,
            sumsq: 0,
            last_clock: initial_clock,
        }
    }
}

/// Perform a single stats-tick cycle: compute elapsed ticks, update
/// the stats accumulator, advance the 32-bit CYCCNT wrap counter, emit
/// the Klipper `stats` message, and kick the watchdog.
///
/// `stats_send_time` and `stats_send_time_high` are the atomics shared
/// with `get_uptime`. Write-order matters: the `swap` on
/// `stats_send_time` must happen before the `fetch_add` on
/// `stats_send_time_high` so a preempting `get_uptime` on P2 cannot
/// observe an incremented high with a stale low.
pub fn stats_tick(
    accumulator: &mut StatsAccumulator,
    now: u32,
    stats_send_time: &AtomicU32,
    stats_send_time_high: &AtomicU32,
    wdg: &mut IndependentWatchdog<'static, peripherals::IWDG1>,
) {
    let elapsed = now.wrapping_sub(accumulator.last_clock);
    accumulator.last_clock = now;

    // One "iteration" per stats period with elapsed ticks.
    accumulator.count += 1;
    accumulator.sum += elapsed;
    let diff_scaled = elapsed / STATS_SUMSQ_BASE;
    accumulator.sumsq = accumulator
        .sumsq
        .saturating_add(diff_scaled.saturating_mul(elapsed));

    // Track 32-bit clock wraps for `get_uptime`'s high word.
    let prev = stats_send_time.swap(now, Relaxed);
    if now < prev {
        stats_send_time_high.fetch_add(1, Relaxed);
    }

    defmt::info!("stats tick");
    ::ankyra::klipper_reply_from!(
        &mut crate::_ankyra_config::Sender,
        Stats,
        count: u32 = accumulator.count,
        sum: u32 = accumulator.sum,
        sumsq: u32 = accumulator.sumsq,
    );
    wdg.pet();

    accumulator.count = 0;
    accumulator.sum = 0;
    accumulator.sumsq = 0;
}

/// Drain pending trsync reports and emit `trsync_state` replies.
///
/// Spawned by the TIM5 CC2 ISR when any trsync has a pending report.
/// The caller-supplied `drain_snapshots` closure collects snapshots in
/// one brief critical section (typically inside the trigger-domain
/// lock); the replies are then sent at P2 without holding that lock.
pub fn emit_trigger_reports<F>(drain_snapshots: F, report_pending: &AtomicBool)
where
    F: FnOnce(&mut heapless::Vec<foci_klipper::trigger::ReportSnapshot, 4>),
{
    let mut snapshots: heapless::Vec<foci_klipper::trigger::ReportSnapshot, 4> =
        heapless::Vec::new();
    drain_snapshots(&mut snapshots);
    report_pending.store(false, Relaxed);
    for snap in &snapshots {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            TrsyncState,
            oid: u8 = snap.oid,
            can_trigger: u8 = snap.can_trigger,
            trigger_reason: u8 = snap.trigger_reason,
            clock: u32 = snap.clock,
        );
    }
}
