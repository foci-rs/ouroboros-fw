//! Ouroboros commissioning backend hooks.
//!
//! Board-specific hooks provide `Mono`, shutdown signal, configuration, and
//! trace integration to the shared firmware commissioning backend core.

use core::sync::atomic::Ordering::Relaxed;

use embassy_stm32::gpio::Output;
use foci_firmware::commissioning::backend::{
    CommissionBackendHooks, SharedCommissioningBackend, StimulusError, shutdown_requested_from,
};
use foci_firmware::config::TmcConfig;
use foci_firmware::context::stepper::{self, StepperEnqueueError};
use foci_firmware::diagnostic::StepperEventReason;
use foci_firmware::state::lifecycle::{FAULT_ACTIVE, SHUTDOWN_ACTIVE};
use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, tmc_channel};
use foci_klipper::commands::CommandError;
use foci_klipper::scheduler::SchedulerEventKind;
use foci_klipper::stepper::StepMove;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;
use crate::firmware_ctx::OuroborosStepperBoard;
use crate::helpers::lifecycle::read_tim5;
use crate::tmc_control::containment::lease_expired;
use crate::trigger_domain::with_trigger_domain;

pub(crate) struct OuroborosHooks;

impl CommissionBackendHooks for OuroborosHooks {
    fn commission_tmc_config() -> &'static TmcConfig {
        &TMC_CONFIG
    }

    fn commission_feedback_configs() -> &'static [foci_firmware::config::FeedbackConfig] {
        &crate::board_config::FEEDBACK_CONFIGS
    }

    fn commission_elapsed_us() -> u64 {
        // Microseconds since Mono start. Returned as u64 so deadline
        // comparisons in commission.rs never wrap across the 51-second
        // TIM5 boundary that previously broke the engine.
        //
        // Source: TIM2 monotonic at 1 MHz (1 us resolution).
        Mono::now().duration_since_epoch().to_micros()
    }

    fn commission_is_shutdown_requested() -> bool {
        let disable_signal = if TMC_DISABLE_SIGNAL.try_take().is_some() {
            TMC_DISABLE_SIGNAL.signal(());
            true
        } else {
            false
        };
        shutdown_requested_from(
            disable_signal,
            lease_expired(read_tim5()),
            SHUTDOWN_ACTIVE.load(Relaxed),
            FAULT_ACTIVE.load(Relaxed),
        )
    }

    async fn commission_delay_us(us: u32) {
        Mono::delay((us as u64).micros()).await;
    }

    #[cfg(feature = "trace")]
    fn record_commission_trace_event(
        channel: u8,
        event: foci_firmware::trace::events::CommissionTraceEvent,
    ) {
        crate::usb_trace::request_commission_event(channel, event);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        channel: u8,
        observation: foci_firmware::trace::stream::VelocityTraceObservation,
    ) {
        crate::usb_trace::request_velocity_observation(channel, observation);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        channel: u8,
        record: foci_firmware::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        crate::usb_trace::request_velocity_sweep_evidence(channel, record);
    }

    fn commission_step_timer_hz() -> u32 {
        crate::state::stepper::STEP_TIMER_HZ
    }

    fn commission_step_clock_now() -> u32 {
        read_tim5()
    }

    fn commission_stimulus_queue_free(channel: u8) -> usize {
        crate::state::stepper::STEP_QUEUE_CHANNELS
            .get(channel as usize)
            .map_or(0, |queue| {
                foci_klipper::step_queue::STEP_QUEUE_CAPACITY.saturating_sub(queue.len())
            })
    }

    fn commission_stimulus_trigger_blocked(channel: u8) -> bool {
        with_trigger_domain(|td| {
            td.state()
                .stepper_stop
                .get(channel as usize)
                .is_none_or(|stop| stop.blocked_by_trigger)
        })
    }

    fn commission_stimulus_set_direction(channel: u8, forward: bool) {
        with_trigger_domain(|_| {
            // SAFETY: the trigger-domain lock serialises this against the P2
            // command handlers that otherwise own the P2 stepper; the host
            // sends no stepper commands while commissioning owns the channel.
            if let Some(stepper) = unsafe { crate::state::stepper::p2_stepper(channel) } {
                stepper.set_next_step_dir(forward);
            }
        });
    }

    fn commission_stimulus_reset_step_clock(channel: u8, clock: u32) -> Result<(), StimulusError> {
        let tmc_state = tmc_channel(channel).ok_or(StimulusError::InvalidChannel)?;
        with_trigger_domain(|td| {
            let mut board = OuroborosStepperBoard {
                tmc_state: Some(tmc_state),
            };
            stepper::reset_step_clock(td.state_mut(), &mut board, channel, clock).map_err(
                |error| match error {
                    CommandError::StepperActive => StimulusError::TimerActive,
                    _ => StimulusError::InvalidChannel,
                },
            )?;
            if td.state().stepper_stop[channel as usize].need_reset {
                return Err(StimulusError::TriggerBlocked);
            }
            Ok(())
        })
    }

    fn commission_stimulus_queue_segment(
        channel: u8,
        segment: StepMove,
    ) -> Result<(), StimulusError> {
        let tmc_state = tmc_channel(channel).ok_or(StimulusError::InvalidChannel)?;
        with_trigger_domain(|td| {
            let mut board = OuroborosStepperBoard {
                tmc_state: Some(tmc_state),
            };
            let was_len = crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].len();
            match stepper::queue_step_move(td.state_mut(), &mut board, channel, segment) {
                Ok(()) => {
                    if td.state().stepper_stop[channel as usize].need_reset {
                        Err(StimulusError::NeedsReset)
                    } else {
                        Ok(())
                    }
                }
                Err(StepperEnqueueError::QueueFull { queue_len, .. }) => {
                    Err(StimulusError::QueueRefused {
                        queue_len: queue_len as u32,
                    })
                }
                Err(_) => Err(StimulusError::QueueRefused {
                    queue_len: was_len as u32,
                }),
            }
        })
    }

    fn commission_stimulus_arm(channel: u8) -> Result<(), StimulusError> {
        let tmc_state = tmc_channel(channel).ok_or(StimulusError::InvalidChannel)?;
        if !tmc_state.enable_pin_asserted.load(Relaxed) {
            return Err(StimulusError::NotEnergized);
        }
        with_trigger_domain(|td| {
            // Captures whatever `motor_armed` actually holds -- true or
            // false -- once per stimulus session; a channel a prior
            // commissioning run left armed (e.g. an ArmPreservePosition
            // post-action) is not distinguishable here from a channel this
            // hook itself just armed, so the session-scoped capture guard
            // lives on `TmcChannelState`, not a bare `motor_armed` read.
            tmc_state.stimulus_capture_motor_armed_once();
            tmc_state.motor_armed.store(true, Relaxed);
            let mut board = OuroborosStepperBoard {
                tmc_state: Some(tmc_state),
            };
            stepper::arm_check(td.state_mut(), &mut board, channel);
        });
        Ok(())
    }

    fn commission_stimulus_stop(channel: u8) {
        let Some(tmc_state) = tmc_channel(channel) else {
            return;
        };
        let idx = channel as usize;
        with_trigger_domain(|td| {
            td.state_mut()
                .scheduler
                .cancel(|kind| matches!(kind, SchedulerEventKind::Stepper(i) if *i == idx));
            crate::trigger_domain::stop_physical_stepper_masked(idx, StepperEventReason::P1Stop);
            crate::trigger_domain::set_timer_active(td.state_mut(), idx, false);
            td.state_mut().stepper_stop[idx].need_reset = true;
            tmc_state.stimulus_restore_motor_armed();
        });
    }
}

pub(crate) type OuroborosBackend<'a, 'spi> =
    SharedCommissioningBackend<'a, TmcSpiDevice<'spi>, Output<'static>, OuroborosHooks>;
