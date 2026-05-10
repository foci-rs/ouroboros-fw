//! Subsystem-level helper functions shared between handlers and the
//! RTIC task graph.
//!
//! Each submodule collects small free functions operating on the
//! corresponding `state::*` statics. Task 8c–8d populate these from
//! `mod app` in main.rs.

#[cfg(feature = "dev")]
pub mod dev;
pub mod lifecycle;
pub mod stepper;
pub mod tmc;
pub mod trsync;
