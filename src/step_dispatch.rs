//! Board-local control flow for one TIM5 CC1 step dispatch.

/// Result of one step-dispatch iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StepDispatchOutcome {
    /// Continue processing due channels in the current interrupt.
    Continue,
    /// No more work should be processed in the current interrupt.
    Complete,
    /// A fatal condition requires the current interrupt to abort immediately.
    Fatal,
}

/// Run step-dispatch iterations until the dispatch completes or becomes fatal.
pub(crate) fn run_step_dispatch(
    mut dispatch_one: impl FnMut() -> StepDispatchOutcome,
) -> StepDispatchOutcome {
    loop {
        match dispatch_one() {
            StepDispatchOutcome::Continue => {}
            outcome => return outcome,
        }
    }
}

/// What the dispatch refresh does with one channel's pending next-event clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ChannelRefresh {
    /// The channel is not ready; drop its pending clock.
    Clear,
    /// Drop any pending clock and load the channel's next segment.
    Load,
    /// Keep the pending clock.
    Keep,
}

/// Decide how the refresh pass treats one channel.
///
/// A stop outside the step ISR resets the stepper engine but cannot clear the ISR-local
/// schedule, so a ready channel without an active segment can still carry a stale clock.
/// Waiting for that clock to come round on the 32-bit timer delays the first segment past
/// its deadline, so such a channel always reloads.
pub(crate) fn channel_refresh(
    ready: bool,
    has_active_segment: bool,
    next_event_clock: Option<u32>,
) -> ChannelRefresh {
    if !ready {
        ChannelRefresh::Clear
    } else if !has_active_segment || next_event_clock.is_none() {
        ChannelRefresh::Load
    } else {
        ChannelRefresh::Keep
    }
}
