//! Board-pure ordering for one complete logical STEP/DIR pulse train.

use foci_klipper::stepper::LogicalStepEvent;

/// GPIO and timer operations required to emit a pulse train.
pub(crate) trait PulseTrainIo {
    /// Error returned by GPIO writes.
    type Error;

    /// Drive STEP to an explicit level.
    fn set_step(&mut self, high: bool) -> Result<(), Self::Error>;

    /// Drive DIR to an explicit level.
    fn set_dir(&mut self, forward: bool) -> Result<(), Self::Error>;

    /// Wait for at least `ticks` timer ticks.
    fn wait_ticks(&mut self, ticks: u32);
}

/// Emit one logical event and leave STEP low before returning.
#[inline(always)]
pub(crate) fn emit_logical_step_event<I: PulseTrainIo>(
    io: &mut I,
    event: LogicalStepEvent,
    dir_setup_ticks: u32,
    half_period_ticks: u32,
) -> Result<(), I::Error> {
    io.set_step(false)?;
    if event.dir_setup_required {
        io.set_dir(event.direction)?;
        io.wait_ticks(dir_setup_ticks);
    }

    assert!(
        event.physical_pulses <= 2,
        "logical event exceeds two physical pulses"
    );
    for _ in 0..event.physical_pulses {
        io.set_step(true)?;
        io.wait_ticks(half_period_ticks);
        io.set_step(false)?;
        io.wait_ticks(half_period_ticks);
    }
    Ok(())
}
