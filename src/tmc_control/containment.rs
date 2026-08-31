//! Pure containment decisions for commissioning wait boundaries.

const CLOCK_SYNC_TIMEOUT_TICKS: u32 = 5 * crate::ankyra::constants::CLOCK_FREQ;

/// Whether the host-liveness lease has expired: any channel is
/// physically enabled, and either no valid `get_clock` has ever been
/// received, or more than `CLOCK_SYNC_TIMEOUT_TICKS` have elapsed since
/// the last one. `now` must be a freshly sampled `read_tim5()` at the
/// call site — never a value captured once and reused across iterations.
/// The lease is shared across both channels: it is armed whenever *any*
/// channel is enabled, and does not disarm just because one of two
/// channels disables while the other remains enabled.
pub(crate) fn lease_expired(now: u32) -> bool {
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::state::lifecycle::{HAS_RECEIVED_VALID_CLOCK, LAST_HOST_LIVENESS_TICKS};

    let any_enabled = (0..crate::board_config::TMC_CHANNEL_COUNT).any(|channel| {
        crate::tmc_control::board::tmc_state(channel)
            .enable_pin_asserted
            .load(Relaxed)
    });
    if !any_enabled {
        return false;
    }
    if !HAS_RECEIVED_VALID_CLOCK.load(Relaxed) {
        return true;
    }
    let elapsed =
        foci_firmware::helpers::clock::elapsed_ticks(now, LAST_HOST_LIVENESS_TICKS.load(Relaxed));
    elapsed > CLOCK_SYNC_TIMEOUT_TICKS
}

/// What the P1 loop should do with the clock-sync lease this iteration.
/// Checking the lease while shutdown is already active would re-enter
/// shutdown every tick and starve the safe-state application above this
/// check from ever completing, so this always resolves to `Proceed` in
/// that case regardless of `lease_expired`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LeaseCheckDisposition {
    EnterClockSyncShutdown,
    Proceed,
}

pub(crate) const fn classify_lease_check(
    shutdown_active: bool,
    lease_expired: bool,
) -> LeaseCheckDisposition {
    if !shutdown_active && lease_expired {
        LeaseCheckDisposition::EnterClockSyncShutdown
    } else {
        LeaseCheckDisposition::Proceed
    }
}

/// Board signal that interrupts an absolute commissioning wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommissionWaitExit {
    /// Enter the existing shutdown safe-state path.
    Shutdown,
    /// Return to the queue-flush path.
    QueueFlush,
    /// Enter GPIO-first emergency-disable handling.
    EmergencyDisable,
    /// The host-liveness lease has expired; enter the shutdown safe-state
    /// path exactly like `Shutdown` above.
    ClockSyncExpired,
}

/// Decision made before the optional channel-1 critical-STATUS fallback.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommissionWaitPrecheck {
    /// Return to the outer containment path without performing fallback SPI.
    Exit(CommissionWaitExit),
    /// No containment state is observable, so fallback SPI is permitted.
    PollCriticalStatus,
}

/// Classify board-owned signals checked between commissioning wait slices.
pub(crate) const fn classify_commission_wait_exit(
    shutdown: bool,
    queue_flush: bool,
    emergency_disable: bool,
    critical_status: bool,
    clock_sync_expired: bool,
) -> Option<CommissionWaitExit> {
    if emergency_disable || critical_status {
        Some(CommissionWaitExit::EmergencyDisable)
    } else if shutdown {
        Some(CommissionWaitExit::Shutdown)
    } else if queue_flush {
        Some(CommissionWaitExit::QueueFlush)
    } else if clock_sync_expired {
        Some(CommissionWaitExit::ClockSyncExpired)
    } else {
        None
    }
}

/// Decide whether a containment snapshot permits the fallback STATUS read.
/// A clock-sync expiry is decided here too, before any SPI -- same
/// motivation as the existing fault/shutdown/queue-flush prechecks: skip
/// the channel-1 poll when a cheap flag already says to exit.
pub(crate) const fn classify_commission_wait_precheck(
    shutdown: bool,
    queue_flush: bool,
    emergency_disable: bool,
    fault_active: bool,
    clock_sync_expired: bool,
) -> CommissionWaitPrecheck {
    match classify_commission_wait_exit(
        shutdown,
        queue_flush,
        emergency_disable,
        fault_active,
        clock_sync_expired,
    ) {
        Some(exit) => CommissionWaitPrecheck::Exit(exit),
        None => CommissionWaitPrecheck::PollCriticalStatus,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::Ordering::Relaxed;
    use foci_firmware::state::lifecycle::{HAS_RECEIVED_VALID_CLOCK, LAST_HOST_LIVENESS_TICKS};

    /// Serializes every test in this module that touches the global
    /// liveness atomics or either channel's `enable_pin_asserted` --
    /// neither is otherwise synchronized, and `cargo test` runs this
    /// module's tests in parallel by default.
    static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn reset_lease_state() -> std::sync::MutexGuard<'static, ()> {
        let guard = TEST_LOCK.lock().expect("lease test lock is available");
        HAS_RECEIVED_VALID_CLOCK.store(false, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(0, Relaxed);
        for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
            crate::tmc_control::board::tmc_state(channel)
                .enable_pin_asserted
                .store(false, Relaxed);
        }
        guard
    }

    #[test]
    fn lease_expired_false_when_no_channel_enabled() {
        let _guard = reset_lease_state();
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        assert!(!lease_expired(CLOCK_SYNC_TIMEOUT_TICKS + 1));
    }

    #[test]
    fn lease_expired_true_when_only_channel_1_enabled_and_stale() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state(1)
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(100, Relaxed);
        assert!(lease_expired(100 + CLOCK_SYNC_TIMEOUT_TICKS + 1));
    }

    #[test]
    fn lease_expired_false_when_only_channel_0_enabled_and_recent() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state(0)
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(100, Relaxed);
        assert!(!lease_expired(100 + CLOCK_SYNC_TIMEOUT_TICKS - 1));
    }

    #[test]
    fn lease_expired_true_when_enabled_and_no_clock_ever_received() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state(0)
            .enable_pin_asserted
            .store(true, Relaxed);
        assert!(lease_expired(0));
    }

    #[test]
    fn lease_expired_false_when_last_liveness_races_ahead_of_now() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state(0)
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(1_000, Relaxed);
        assert!(!lease_expired(999));
    }

    #[test]
    fn lease_expired_stays_armed_when_one_of_two_channels_disarms() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state(0)
            .enable_pin_asserted
            .store(true, Relaxed);
        crate::tmc_control::board::tmc_state(1)
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(100, Relaxed);
        crate::tmc_control::board::tmc_state(1)
            .enable_pin_asserted
            .store(false, Relaxed);
        assert!(lease_expired(100 + CLOCK_SYNC_TIMEOUT_TICKS + 1));
    }

    #[test]
    fn lease_expired_transitions_true_across_an_advancing_sequence() {
        let _guard = reset_lease_state();
        crate::tmc_control::board::tmc_state(0)
            .enable_pin_asserted
            .store(true, Relaxed);
        HAS_RECEIVED_VALID_CLOCK.store(true, Relaxed);
        LAST_HOST_LIVENESS_TICKS.store(0, Relaxed);

        let slice_ticks = 1_000 * (crate::ankyra::constants::CLOCK_FREQ / 1_000_000);
        let mut now = 0u32;
        let mut saw_expired = false;
        for _ in 0..(CLOCK_SYNC_TIMEOUT_TICKS / slice_ticks + 2) {
            if lease_expired(now) {
                saw_expired = true;
                break;
            }
            now = now.wrapping_add(slice_ticks);
        }
        assert!(
            saw_expired,
            "lease_expired never fired across an advancing sequence"
        );
    }

    #[test]
    fn classify_lease_check_enters_shutdown_when_not_active_and_expired() {
        assert_eq!(
            classify_lease_check(false, true),
            LeaseCheckDisposition::EnterClockSyncShutdown
        );
    }

    #[test]
    fn classify_lease_check_proceeds_when_not_active_and_not_expired() {
        assert_eq!(
            classify_lease_check(false, false),
            LeaseCheckDisposition::Proceed
        );
    }

    #[test]
    fn classify_lease_check_proceeds_when_active_and_expired() {
        assert_eq!(
            classify_lease_check(true, true),
            LeaseCheckDisposition::Proceed
        );
    }

    #[test]
    fn classify_lease_check_proceeds_when_active_and_not_expired() {
        assert_eq!(
            classify_lease_check(true, false),
            LeaseCheckDisposition::Proceed
        );
    }

    #[test]
    fn classify_commission_wait_exit_reports_clock_sync_expired() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, false, true),
            Some(CommissionWaitExit::ClockSyncExpired)
        );
    }

    #[test]
    fn classify_commission_wait_exit_emergency_disable_takes_priority_over_clock_sync() {
        assert_eq!(
            classify_commission_wait_exit(false, false, true, false, true),
            Some(CommissionWaitExit::EmergencyDisable)
        );
    }

    #[test]
    fn classify_commission_wait_exit_critical_status_takes_priority_over_clock_sync() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, true, true),
            Some(CommissionWaitExit::EmergencyDisable)
        );
    }

    #[test]
    fn classify_commission_wait_exit_shutdown_takes_priority_over_clock_sync() {
        assert_eq!(
            classify_commission_wait_exit(true, false, false, false, true),
            Some(CommissionWaitExit::Shutdown)
        );
    }

    #[test]
    fn classify_commission_wait_exit_none_when_nothing_set() {
        assert_eq!(
            classify_commission_wait_exit(false, false, false, false, false),
            None
        );
    }

    #[test]
    fn classify_commission_wait_precheck_exits_on_clock_sync_without_polling() {
        assert_eq!(
            classify_commission_wait_precheck(false, false, false, false, true),
            CommissionWaitPrecheck::Exit(CommissionWaitExit::ClockSyncExpired)
        );
    }

    #[test]
    fn classify_commission_wait_precheck_polls_when_clock_sync_not_expired() {
        assert_eq!(
            classify_commission_wait_precheck(false, false, false, false, false),
            CommissionWaitPrecheck::PollCriticalStatus
        );
    }
}
