use foci_firmware::tmc_control::ChannelMode;

/// Maintenance-relevant state copied from one TMC channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ChannelMaintenanceSnapshot {
    pub(crate) mode: ChannelMode,
    pub(crate) driver_obligations_clear: bool,
    pub(crate) enable_pin_asserted: bool,
    pub(crate) motor_armed: bool,
    pub(crate) commission_in_flight: bool,
    pub(crate) config_apply_pending: u8,
    pub(crate) feedforward_reset: bool,
    pub(crate) pid_error_sums_clear: bool,
    pub(crate) homing_clamp_active: bool,
    pub(crate) stall_latched: bool,
    pub(crate) step_timer_active: bool,
    pub(crate) step_queue_empty: bool,
}

impl ChannelMaintenanceSnapshot {
    fn proves_quiescence(self) -> bool {
        matches!(self.mode, ChannelMode::Ready)
            && self.driver_obligations_clear
            && !self.enable_pin_asserted
            && !self.motor_armed
            && !self.commission_in_flight
            && self.config_apply_pending == 0
            && !self.feedforward_reset
            && !self.pid_error_sums_clear
            && !self.homing_clamp_active
            && !self.stall_latched
            && !self.step_timer_active
            && self.step_queue_empty
    }
}

/// Owned state used for one dual-channel maintenance-arm decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct MaintenanceSnapshot {
    pub(crate) shutdown_active: bool,
    pub(crate) fault_active: bool,
    pub(crate) queue_flush_pending: bool,
    pub(crate) disable_signal_pending: bool,
    pub(crate) homing_state: u8,
    pub(crate) homing_triggered: u8,
    pub(crate) homing_start_pending: bool,
    pub(crate) channels: [ChannelMaintenanceSnapshot; 2],
    pub(crate) trace_enabled: bool,
}

impl MaintenanceSnapshot {
    fn proves_quiescence(self) -> bool {
        !self.shutdown_active
            && !self.fault_active
            && !self.queue_flush_pending
            && !self.disable_signal_pending
            && self.homing_state == 0
            && self.homing_triggered == 0xFF
            && !self.homing_start_pending
            && self
                .channels
                .into_iter()
                .all(ChannelMaintenanceSnapshot::proves_quiescence)
            && !self.trace_enabled
    }
}

/// Whether the shared maintenance deadline must remain active.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MaintenanceArm {
    /// Required work exists or quiescence cannot be proven.
    Armed,
    /// Both channels and all global state prove quiescence.
    Disarmed,
}

/// Consume one snapshot and derive its conservative maintenance-arm state.
pub(crate) fn maintenance_arm(snapshot: MaintenanceSnapshot) -> MaintenanceArm {
    if snapshot.proves_quiescence() {
        MaintenanceArm::Disarmed
    } else {
        MaintenanceArm::Armed
    }
}
