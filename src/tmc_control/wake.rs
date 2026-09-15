// Pure wait policy for urgent homing preparation.

/// Source selected by the control task for its next wait.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum HomingWait {
    /// Poll the event receiver while awaiting the normal scheduler deadline.
    ReceiverOrDeadline,
    /// Await only the maintenance deadline without polling the receiver.
    MaintenanceOnly,
    /// Run scheduler work immediately because at least one source is due.
    RunScheduler,
    /// Process the retained request before polling the receiver again.
    ProcessRetained,
}

/// Event presented to the pure homing-wake policy.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum HomingWakeEvent<T> {
    /// An ordinary TMC control request.
    Request(T),
    /// A coalescing homing-start doorbell.
    HomingStart,
}

/// Result of presenting an event to the policy.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum HomingReceive<T> {
    /// Process the request through the normal request path.
    Process(T),
    /// The request was retained while urgent preparation owns the loop.
    Retained,
    /// The doorbell carries no data; the authoritative latch owns urgency.
    Doorbell,
    /// The existing request was preserved and the new request is returned.
    RetainedSlotOccupied(T),
}

/// Result of an urgent homing-preparation attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum HomingPreparationProgress {
    /// State 1 remains active and requires another maintenance group.
    Retry,
    /// State 1 advanced successfully.
    Advanced,
    /// The fail-closed path took ownership of containment.
    FailClosed,
}

/// Pure policy state for the single request that can race a homing start.
pub(crate) struct HomingWakePolicy<T> {
    retained_request: Option<T>,
}

impl<T> HomingWakePolicy<T> {
    /// Create an empty policy state.
    pub(crate) const fn new() -> Self {
        Self {
            retained_request: None,
        }
    }

    /// Choose whether the task may poll the receiver on its next wait.
    pub(crate) const fn next_wait(
        &self,
        homing_start_pending: bool,
        scheduler_work_due: bool,
    ) -> HomingWait {
        if homing_start_pending {
            if scheduler_work_due {
                HomingWait::RunScheduler
            } else {
                HomingWait::MaintenanceOnly
            }
        } else if self.retained_request.is_some() {
            HomingWait::ProcessRetained
        } else {
            HomingWait::ReceiverOrDeadline
        }
    }

    /// Classify a received event and retain a racing request when required.
    pub(crate) fn on_event(
        &mut self,
        event: HomingWakeEvent<T>,
        homing_start_pending: bool,
    ) -> HomingReceive<T> {
        match event {
            HomingWakeEvent::HomingStart => HomingReceive::Doorbell,
            HomingWakeEvent::Request(request) => {
                if homing_start_pending || self.retained_request.is_some() {
                    if self.retained_request.is_some() {
                        HomingReceive::RetainedSlotOccupied(request)
                    } else {
                        self.retained_request = Some(request);
                        HomingReceive::Retained
                    }
                } else {
                    HomingReceive::Process(request)
                }
            }
        }
    }

    /// Take the retained request, leaving the slot empty.
    pub(crate) fn take_retained_request(&mut self) -> Option<T> {
        self.retained_request.take()
    }

    /// Discard work owned by the request epoch that a queue flush ends.
    pub(crate) fn reset_for_queue_flush(&mut self) {
        self.retained_request = None;
    }

    /// Return whether an urgent preparation result releases the latch.
    pub(crate) const fn should_clear_latch(progress: HomingPreparationProgress) -> bool {
        match progress {
            HomingPreparationProgress::Retry => false,
            HomingPreparationProgress::Advanced | HomingPreparationProgress::FailClosed => true,
        }
    }
}
