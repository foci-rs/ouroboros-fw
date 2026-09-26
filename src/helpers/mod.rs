//! Subsystem-level helper functions shared between handlers and the
//! RTIC task graph.
//!
//! Each submodule collects small free functions operating on the
//! corresponding `state::*` statics.

#[cfg(feature = "dev")]
pub mod dev;
pub mod lifecycle;
pub mod stepper;
pub mod stepper_diag;
pub mod tmc;
pub mod trsync;
