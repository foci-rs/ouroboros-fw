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
