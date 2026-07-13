//! Pure containment decisions for commissioning wait boundaries.

/// Board signal that interrupts an absolute commissioning wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommissionWaitExit {
    /// Enter the existing shutdown safe-state path.
    Shutdown,
    /// Return to the queue-flush path.
    QueueFlush,
    /// Enter GPIO-first emergency-disable handling.
    EmergencyDisable,
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
) -> Option<CommissionWaitExit> {
    if emergency_disable || critical_status {
        Some(CommissionWaitExit::EmergencyDisable)
    } else if shutdown {
        Some(CommissionWaitExit::Shutdown)
    } else if queue_flush {
        Some(CommissionWaitExit::QueueFlush)
    } else {
        None
    }
}

/// Decide whether a containment snapshot permits the fallback STATUS read.
pub(crate) const fn classify_commission_wait_precheck(
    shutdown: bool,
    queue_flush: bool,
    emergency_disable: bool,
    fault_active: bool,
) -> CommissionWaitPrecheck {
    match classify_commission_wait_exit(shutdown, queue_flush, emergency_disable, fault_active) {
        Some(exit) => CommissionWaitPrecheck::Exit(exit),
        None => CommissionWaitPrecheck::PollCriticalStatus,
    }
}
