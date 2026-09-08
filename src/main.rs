#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

// PAC shim for RTIC + embassy-stm32
pub mod pac {
    pub use embassy_stm32::Peripherals;
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

mod ankyra;
mod app_types;
mod board_config;
mod board_init;
mod commissioning_backend;
mod firmware_ctx;
mod heartbeat;
mod helpers;
mod interrupts;
mod pulse_train;
mod state;
mod step_dispatch;
mod stepper_pins;
mod tmc_control;
mod tmc_diagnostics;
mod tmc_dispatch;
mod tmc_mux;
mod transport;
mod trigger_domain;
mod usb_task;
#[cfg(feature = "trace")]
mod usb_trace;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [UART4, UART5, USART3])]
mod app {
    use core::sync::atomic::Ordering::Relaxed;

    use crate::ankyra::constants::CLOCK_FREQ;
    use embassy_stm32::gpio::{Input, Level, Output, Speed};
    use embassy_stm32::spi::Spi;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals};
    #[cfg(feature = "trace")]
    use foci_firmware::autotune::CommissionPhase;
    #[cfg(feature = "trace")]
    use foci_firmware::commissioning::types::EngineKind;
    use foci_klipper::state::State;
    use rtic_monotonics::fugit::ExtU64;
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;

    use crate::helpers::lifecycle::*;
    use crate::helpers::stepper::*;
    use crate::helpers::trsync::*;
    use crate::state::tmc::*;
    use crate::step_dispatch::{StepDispatchOutcome, run_step_dispatch};
    use crate::tmc_control::board::{
        TmcEnablePins, TmcRuntimes, channel_may_be_energized, channel_needs_safe_state,
        commissioning_request_blocked, enable_pin, new_tmc_runtimes, ouroboros_backend,
        refresh_velocity_scale, stepper_state, tmc_driver, tmc_state,
    };
    use crate::trigger_domain::{
        TRIGGER_DOMAIN, TriggerDomain, with_trigger_domain, with_trigger_domain_site,
    };
    use foci_firmware::context::stepper::{
        StepperArmCheckBoard, StepperStopCheckBoard, StepperTimerStateBoard,
    };
    use foci_firmware::context::stepper_reconcile::drain_stepper_reconciliation;
    use foci_firmware::state::lifecycle::*;
    use foci_firmware::state::stepper::{NO_STEPPER_CHANNEL, STEPPER_DEADLINE_MISS_CHANNEL};
    use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TMC_QUEUE_FLUSH};
    use foci_firmware::state::trsync::*;
    use foci_firmware::tmc_control::request::TmcPreRequestDisposition;
    use foci_firmware::tmc_control::{
        BoardAction, BoardActions, HomingMaintenanceResult, StepperAction, TmcControlHomingRefs,
        TmcControlRuntime, TmcControlStateRefs,
    };

    bind_interrupts!(struct Irqs {
        DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
        DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
    });

    // TIM2-backed monotonic at 1 MHz (1 us resolution).
    //
    // Previously used SysTick at 1 kHz, but the commissioning engine's
    // tight sample loops in Phase 5 (R/L measurement) rely on sub-ms
    // deadlines. Using a TIM2-backed monotonic gives true u64 us
    // precision without the 51-second wraparound bug that affected
    // the old read_tim5()/84 path.
    //
    // The rtic-monotonics crate handles the hardware timer overflow
    // automatically via an extended u64 counter in a TimerQueueBackend.
    stm32_tim2_monotonic!(Mono, 1_000_000);

    static USB_PUMP_COUNTERS: foci_firmware::usb_pump::UsbPumpCounters =
        foci_firmware::usb_pump::UsbPumpCounters::new();
    static USB_PUMP_EMITTER: crate::usb_task::Emitter = crate::usb_task::Emitter::new();

    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_ITERS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_MAX_GAP_US: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(0);
    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_LAST_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

    use foci_firmware::diagnostic::StepperEventReason;
    use foci_firmware::tmc::{TmcRequest, TmcRequestKind};

    use crate::board_config::{CURRENT_SCALE_MA_PER_LSB, TMC_CONFIG};
    use crate::helpers::stepper_diag::{UNKNOWN_QUEUE_LEN, record_stepper_event};

    const STATUS_ADC_I_CLIPPED: u32 = 1 << 26;
    // TMC4671-LA Rev 1, 02/2024 Table 30 marks STATUS_FLAGS[19] reserved.
    // Only ADC current clipping is routed as a STATUS critical fault.
    const STATUS_CRITICAL_FAULT_MASK: u32 = STATUS_ADC_I_CLIPPED;
    const CHANNEL1_STATUS_FALLBACK: u8 = 1;

    fn lower_all_enable_pins(enable_pins: &mut TmcEnablePins) {
        for pin in enable_pins {
            pin.set_low();
        }
    }

    fn apply_runtime_stepper_actions(channel: u8, actions: BoardActions) {
        for action in actions {
            match action {
                BoardAction::Stepper(StepperAction::StopNow) => {
                    stepper_stop_from_p1(channel);
                }
                BoardAction::Stepper(StepperAction::None)
                | BoardAction::Stepper(StepperAction::ArmCheck { .. })
                | BoardAction::Stepper(StepperAction::ApplyPostAction(_))
                | BoardAction::Shutdown(_)
                | BoardAction::SignalEmergencyDisable
                | BoardAction::DelegatedDiagnostic(_) => {}
            }
        }
    }

    fn quiesce_all_channels(runtimes: &mut TmcRuntimes, enable_pins: &mut TmcEnablePins) {
        for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
            let actions = runtimes[channel as usize].prepare_shutdown_quiesce();
            apply_runtime_stepper_actions(channel, actions);
            tmc_state(channel)
                .commission_in_flight
                .store(false, core::sync::atomic::Ordering::Relaxed);
        }
        lower_all_enable_pins(enable_pins);
    }

    fn latch_tmc_fault() {
        FAULT_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);
        let tim5 = embassy_stm32::pac::TIM5;
        tim5.dier().modify(|w| w.set_ccie(0, false));
    }

    fn emit_invalid_channel_diagnostic(channel: u8) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociDiagnostic,
            fault: u8 = foci_firmware::diagnostic::FaultClass::InvalidChannel as u8,
            operation: u8 = foci_firmware::diagnostic::operation::TMC_CONTROL,
            oid: u8 = 255,
            channel: u8 = channel,
            data0: u32 = 0,
            data1: u32 = 0,
        );
    }

    // TriggerDomain, PHYSICAL_STEPPER_INDICES, TRIGGER_TIMER_MIN_LEAD_TICKS,
    // TRIGGER_DOMAIN, with_trigger_domain(), stop_physical_stepper_unmasked(),
    // and stop_physical_stepper_masked() live in crate::trigger_domain and
    // are re-imported below.

    // ---- Klipper constants ----
    // Declared in `crate::ankyra::constants` and registered into the data
    // dictionary via the `constants: [...]` list in `crate::ankyra::provider`.

    // ---- TMC4671 register access commands ----
    // Ported to `crate::ankyra::handlers::tmc` by Task 8k.
    // FOCI commissioning/tune/trace commands ported to
    // `crate::ankyra::handlers::foci` by Task 8l.

    // ---- RTIC resources ----

    #[shared]
    struct Shared {
        app_state: State,
    }

    /// Persistent Stage-C candidate slot carried across `tmc_control` task calls.
    ///
    /// Retains the Stage-C authority a `breakaway_seeded` campaign hands off to
    /// `integral_resume`, so the production build runs the full autotune
    /// hand-off exactly as the trace build does.
    type IntegralCandidateTaskLocal =
        foci_firmware::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;

    #[local]
    struct Local {
        tmc_mux: crate::tmc_mux::TmcSpiMux,
        tmc_enable: TmcEnablePins,
        tmc_rx: Receiver<'static, TmcRequest, TMC_CHANNEL_SIZE>,
        tmc_status0: Input<'static>,
        wdg: IndependentWatchdog<'static, peripherals::IWDG1>,
        integral_candidate: IntegralCandidateTaskLocal,
    }

    #[init(local = [
        tmc_channel: rtic_sync::channel::Channel<TmcRequest, TMC_CHANNEL_SIZE> =
            rtic_sync::channel::Channel::new(),
        uid_str: [u8; 24] = [0; 24],
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        foci_firmware::diagnostic::recover_last_boot_breakaway_breadcrumb();
        defmt::info!("FOCI Ouroboros starting");

        // SAFETY: init runs exactly once, before any other task starts.
        unsafe {
            crate::state::stepper::init_step_queues();
        }

        let p = embassy_stm32::init(crate::board_init::rcc_config());
        defmt::info!("SYSCLK: 520 MHz, USB: 48 MHz");
        crate::board_init::verify_dma_interrupt_priority();
        crate::board_init::enable_debug_clock_during_sleep();

        crate::board_init::init_tim5_clock();
        #[cfg(feature = "trace")]
        crate::usb_trace::note_trace_boot();
        crate::board_init::init_stepdir_gpio();
        defmt::info!("TIM5 enabled for Klipper clock @ 260 MHz, STEP/DIR GPIO configured");

        // TIM2 clock: 260 MHz (APB1 timer clock, same bus as TIM5). At the
        // 1 MHz target monotonic rate, the prescaler is 260 (integer divisor).
        // rtic-monotonics enables and configures TIM2 internally; no PAC
        // setup is required beyond passing the timer clock frequency.
        Mono::start(260_000_000);

        // TMC4671 ENABLE pins are active high and start disabled:
        // channel 0 = PD6, channel 1 = PD1.
        let tmc_enable = [
            Output::new(p.PD6, Level::Low, Speed::Low),
            Output::new(p.PD1, Level::Low, Speed::Low),
        ];

        let tmc_status0 = crate::board_init::init_tmc_status_pins(p.PE2, p.PA2);
        crate::board_init::init_tim15_status_capture();

        // IWDG: 10-second timeout, kicked by the stats task every 5 seconds.
        let mut wdg = IndependentWatchdog::new(p.IWDG1, 10_000_000);
        wdg.unleash();
        defmt::info!("IWDG started: 10s timeout");

        // SPI2: TMC4671 communication at 8 MHz, Mode 3 (CPOL=1, CPHA=1)
        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.mode = embassy_stm32::spi::Mode {
            polarity: embassy_stm32::spi::Polarity::IdleHigh,
            phase: embassy_stm32::spi::Phase::CaptureOnSecondTransition,
        };
        spi_config.frequency = embassy_stm32::time::Hertz(8_000_000);

        let spi2 = Spi::new(
            p.SPI2, p.PB13,     // SCK
            p.PB15,     // MOSI
            p.PB14,     // MISO
            p.DMA1_CH0, // TX DMA
            p.DMA1_CH1, // RX DMA
            Irqs, spi_config,
        );

        let spi_cs0 = Output::new(p.PD0, Level::High, Speed::VeryHigh);
        let spi_cs1 = Output::new(p.PD2, Level::High, Speed::VeryHigh);
        let tmc_mux = crate::tmc_mux::TmcSpiMux::new(spi2, spi_cs0, spi_cs1);

        // TMC register request channel
        let (tmc_tx, tmc_rx) = ctx.local.tmc_channel.split();
        critical_section::with(|cs| {
            TMC_REQUEST_SENDER.borrow(cs).replace(Some(tmc_tx));
        });

        defmt::info!("SPI2 configured: 8 MHz, Mode 3, DMA");

        crate::board_init::init_usb_pins(p.PA11, p.PA12);
        crate::board_init::enable_usb_clock();

        let uid = crate::board_init::read_stm32_uid(ctx.local.uid_str);

        // SAFETY: Called exactly once, before any other klipper-usb function.
        // base_addr is the USB_OTG_HS peripheral base on STM32H723.
        unsafe {
            foci_usb_stm32::usb_init(&foci_usb_stm32::UsbConfig {
                base_addr: 0x4004_0000,
                irq_num: 77,
                trdt: 6,
                vbus_detection: false,
                double_buffer_tx: true,
                vid: 0x1D50,
                pid: 0x614E,
                manufacturer: "foci",
                product: "FOCI Ouroboros",
                serial: uid,
            });
        }

        crate::board_init::enable_dwt_cyccnt();

        // Initialize TriggerDomain
        // SAFETY: init runs once before any task starts. No concurrent access.
        unsafe {
            core::ptr::addr_of_mut!(TRIGGER_DOMAIN)
                .cast::<TriggerDomain>()
                .write(TriggerDomain::new());
        }

        // Spawn tasks
        stepper_reconcile::spawn().expect("stepper reconciliation worker must spawn exactly once");
        tmc_control::spawn().expect("tmc_control task must spawn exactly once");
        usb_task::spawn().expect("usb_task must spawn exactly once");
        stats::spawn().expect("stats task must spawn exactly once");
        watchdog::spawn().expect("watchdog task must spawn exactly once");

        defmt::info!("FOCI Ouroboros initialized");

        (
            Shared {
                app_state: State::new(),
            },
            Local {
                tmc_mux,
                tmc_enable,
                tmc_rx,
                tmc_status0,
                wdg,
                integral_candidate:
                    foci_firmware::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(),
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    use crate::ankyra::replies::{emit_phase_reply, emit_terminal_reply};

    /// TMC4671 control task. Runs at priority 1 (preempted by protocol tasks).
    ///
    /// On startup, reads the TMC4671 chip identification register to verify
    /// SPI communication. Then processes register read/write requests from
    /// the protocol handler via the TMC request channel.
    ///
    /// The 1ms loop also advances the emulated stepper, converting queued
    /// step commands into position target updates. This must run on a timer
    /// rather than only on USB receive — Klipper sends queue_step commands
    /// ahead of time and expects the MCU to process them at the scheduled
    /// clock values, even when no USB traffic is arriving.
    #[task(
        priority = 1,
        shared = [app_state],
        local = [
            tmc_mux,
            tmc_enable,
            tmc_status0,
            tmc_rx,
            integral_candidate,
            drivers: [foci_firmware::tmc_control::ChannelDriver; 2] = [
                foci_firmware::tmc_control::ChannelDriver::new(0),
                foci_firmware::tmc_control::ChannelDriver::new(1),
            ]
        ]
    )]
    async fn tmc_control(mut ctx: tmc_control::Context) {
        let enable_pins = ctx.local.tmc_enable;
        let rx = ctx.local.tmc_rx;
        let integral_candidate = ctx.local.integral_candidate;
        let _tmc_status0 = ctx.local.tmc_status0;
        let _drivers = ctx.local.drivers;

        // NOTE: auto-init disabled - motor current regulation not yet safe
        // tmc_init_open_loop(tmc, enable_pin(enable_pins, channel)).await;

        let mut bootstrap_results = [Ok(()); crate::board_config::TMC_CHANNEL_COUNT as usize];
        for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
            let Some(device) = ctx.local.tmc_mux.device(channel) else {
                defmt::error!("Ouroboros TMC channel {} is not available", channel);
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        ::ankyra::klipper_shutdown_from!(
                            &mut crate::_ankyra_config::Sender,
                            "Invalid TMC channel",
                            read_tim5(),
                        );
                    }
                });
                return;
            };
            let mut tmc = tmc_driver(device);

            // Verify TMC4671 chip identity — shutdown if SPI fails or wrong chip.
            match tmc
                .read_sub_register(foci_tmc4671::registers::CHIPINFO_SI_TYPE)
                .await
            {
                Ok(chip_id) => {
                    if chip_id == 0x3436_3731 {
                        defmt::info!(
                            "TMC4671 channel {} chip ID verified: {:#010x}",
                            channel,
                            chip_id
                        );
                    } else {
                        defmt::error!(
                            "TMC4671 channel {} chip ID mismatch: got {:#010x}",
                            channel,
                            chip_id
                        );
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "TMC4671 chip ID mismatch",
                                    read_tim5(),
                                );
                            }
                        });
                        return;
                    }
                }
                Err(_) => {
                    defmt::error!("TMC4671 channel {} SPI communication failed", channel);
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            ::ankyra::klipper_shutdown_from!(
                                &mut crate::_ankyra_config::Sender,
                                "TMC4671 SPI failed at boot",
                                read_tim5(),
                            );
                        }
                    });
                    return;
                }
            }

            let Some(channel_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
                lower_all_enable_pins(enable_pins);
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        ::ankyra::klipper_shutdown_from!(
                            &mut crate::_ankyra_config::Sender,
                            "Invalid TMC channel",
                            read_tim5(),
                        );
                    }
                });
                return;
            };
            let pi_representation = {
                let mut backend =
                    ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                foci_firmware::tmc_control::pi_representation::bootstrap_pi_representation(
                    &mut backend,
                    channel_state,
                )
                .await
            };
            if let Err(error) = pi_representation {
                lower_all_enable_pins(enable_pins);
                defmt::error!(
                    "TMC4671 channel {} PI representation failed at boot: {:?}",
                    channel,
                    error
                );
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        ::ankyra::klipper_shutdown_from!(
                            &mut crate::_ankyra_config::Sender,
                            "TMC4671 PI representation failed at boot",
                            read_tim5(),
                        );
                    }
                });
                return;
            }

            match foci_firmware::adc_vm::capture_adc_vm_offset(&mut tmc).await {
                Ok(capture) => {
                    if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) {
                        tmc_state.store_adc_vm_offset(capture.offset_raw, capture.sample_count);
                    }
                    defmt::info!(
                        "TMC4671 channel {} ADC_VM offset cached: raw={} samples={}",
                        channel,
                        capture.offset_raw,
                        capture.sample_count
                    );
                }
                Err(_) => {
                    defmt::warn!(
                        "TMC4671 ch {} ADC_VM offset failed; host rejects VM decode",
                        channel
                    );
                }
            }

            // Disable TMC4671 brake chopper unconditionally.
            //
            // Write both ADC_VM_LIMIT_HIGH and ADC_VM_LIMIT_LOW to u16::MAX
            // (0xFFFFFFFF). Since ADC_VM_RAW is a 16-bit value that can
            // never exceed 0xFFFF, the comparison `ADC_VM > LIMIT_HIGH` is
            // never true and the BRAKE pin never activates.
            //
            // WARNING: Do NOT write 0/0 to disable the brake, despite the
            // TMC4671 rev 2.08 datasheet section 4.7.12 claim that "BRAKE can
            // be deactivated by setting both registers to Zero". Bench testing
            // on 2026-04-09 proved that writing 0/0 causes the brake to fire
            // continuously — consistent with the comparison `ADC_VM > 0` being
            // always true for any positive bus voltage. The "deactivation"
            // language in the datasheet is either wrong or refers to stopping
            // an already-firing brake (not the default state). The safe
            // "disabled" pattern is 0xFFFFFFFF, not 0.
            //
            // The prior auto-configuration (sample VM at boot, set HIGH at +15%)
            // had two fundamental issues:
            //
            //   1. Threshold too tight: 15% above nominal bus voltage leaves
            //      insufficient margin for regen transients and PSU ripple. At
            //      24V nominal, HIGH was ~26.65V - easily crossed during any
            //      brief motor spin, causing continuous brake firing.
            //   2. ADC_VM scaling is board-specific; production-v1 values are
            //      not confirmed in this compile-only scaffold.
            //
            // Proper board-level overvoltage protection is intentionally not
            // configured in this compile-only scaffold until the production-v1
            // power-stage pins are verified.
            // Configure STATUS_MASK: route ADC current clipping to the channel-local
            // STATUS pin. PE2/channel 0 is routed to EXTI2 below. PA2/channel 1
            // shares EXTI line 2 with PE2, and SYSCFG can select only one port for
            // a given EXTI line, so channel 1 clipping is polled every tick
            // while the channel may be energized.
            let result = {
                let mut backend =
                    ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                foci_firmware::tmc_control::bootstrap_tmc_safety(
                    &mut backend,
                    STATUS_CRITICAL_FAULT_MASK,
                )
                .await
            };
            if let Err(failure) = result {
                defmt::error!(
                    "TMC4671 channel {} safety bootstrap failed: {:?}",
                    channel,
                    failure.stage
                );
            } else {
                defmt::info!(
                    "TMC4671 channel {} STATUS_MASK: {:#010x}",
                    channel,
                    STATUS_CRITICAL_FAULT_MASK
                );
            }
            bootstrap_results[channel as usize] = result;
        }

        if let Some((channel, failure)) =
            foci_firmware::tmc_control::first_tmc_safety_bootstrap_failure(&bootstrap_results)
        {
            lower_all_enable_pins(enable_pins);
            let reason = failure.shutdown_reason(channel);
            loop {
                if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                    Mono::delay(1.millis()).await;
                    continue;
                }
                let _request = rx.recv().await;
                if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                    continue;
                }
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        foci_firmware::dispatch_shutdown!(
                            reason,
                            read_tim5(),
                            &mut crate::_ankyra_config::Sender
                        );
                    }
                });
            }
        }

        let syscfg = embassy_stm32::pac::SYSCFG;
        // EXTICR1[11:8] = 4 selects port E for EXTI line 2. This routes
        // PE2/channel 0 to the interrupt. PA2/channel 1 cannot also be routed
        // to EXTI2, so channel 1 uses the critical-fault polling fallback above.
        syscfg.exticr(0).modify(|w| w.set_exti(2, 4));
        let exti = embassy_stm32::pac::EXTI;
        exti.rtsr(0).modify(|w| w.set_line(2, true));
        exti.ftsr(0).modify(|w| w.set_line(2, false));
        exti.pr(0).write(|w| w.set_line(2, true)); // Clear pending
        exti.imr(0).modify(|w| w.set_line(2, true)); // Unmask
        defmt::info!("TMC4671 STATUS EXTI2 enabled");

        let mut runtimes = new_tmc_runtimes();
        // Board constants bundled once and passed to runtime commissioning tick.
        let tmc_control_config = foci_firmware::tmc_control::TmcControlConfig::new(
            CLOCK_FREQ,
            TMC_CONFIG.pwm_freq_hz(),
            CURRENT_SCALE_MA_PER_LSB,
            TMC_CONFIG,
        );
        let mut commission_trace_engine_seq =
            [0u32; crate::board_config::TMC_CHANNEL_COUNT as usize];
        let mut commission_poll_drivers = [
            foci_firmware::tmc_control::CommissionPollDriver::new(),
            foci_firmware::tmc_control::CommissionPollDriver::new(),
        ];
        let mut commission_tick_max_cycles =
            [0u32; crate::board_config::TMC_CHANNEL_COUNT as usize];
        let mut commission_measurement_active =
            [false; crate::board_config::TMC_CHANNEL_COUNT as usize];
        #[cfg(feature = "trace")]
        let mut commission_trace_phase: [Option<(EngineKind, CommissionPhase)>;
            crate::board_config::TMC_CHANNEL_COUNT as usize] =
            [None; crate::board_config::TMC_CHANNEL_COUNT as usize];

        // Counter for 1s STATUS_FLAGS poll (1000 * slow-control ticks).
        let mut status_poll_counters = [0u16; crate::board_config::TMC_CHANNEL_COUNT as usize];
        let mut slow_control_accum_us = 0u16;

        // Whether the shutdown safe-state has already been applied this
        // shutdown cycle. Reset when shutdown clears.
        let mut shutdown_handled = false;

        // Main loop: wait for channel message or 1ms timeout.
        // On message: handle request. On timeout: write position target.
        // Position is read from stepper_state().position_target atomic (updated
        // by usb_receive at P2) — no shared resource lock needed.
        'control_loop: loop {
            #[cfg(feature = "usb-p1-probe")]
            {
                use core::sync::atomic::Ordering::Relaxed;
                let now = Mono::now().ticks() as u32;
                let last = P1_PROBE_LAST_US.swap(now, Relaxed);
                if last != 0 {
                    P1_PROBE_MAX_GAP_US.fetch_max(now.wrapping_sub(last), Relaxed);
                }
                P1_PROBE_ITERS.fetch_add(1, Relaxed);
            }

            use embassy_futures::select::{Either, select};

            // A terminal restore cancelled by a commissioning-loop exit still
            // owns its channel. Drive it to safe state before anything else
            // touches that channel; only this drain retires its slots.
            for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                let channel_idx = channel as usize;
                if !runtimes[channel_idx].restore_cancel_pending() {
                    continue;
                }
                let Some(device) = ctx.local.tmc_mux.device(channel) else {
                    continue;
                };
                let mut tmc = tmc_driver(device);
                let mut backend =
                    ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                let _ = runtimes[channel_idx]
                    .drive_restore_cancel(
                        &mut backend,
                        tmc_state(channel),
                        stepper_state(channel),
                        foci_firmware::autotune::CommissionError::Shutdown,
                    )
                    .await;
            }

            for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                let channel_idx = channel as usize;
                if !tmc_state(channel)
                    .cancel_requested
                    .load(core::sync::atomic::Ordering::Relaxed)
                {
                    continue;
                }
                let Some(device) = ctx.local.tmc_mux.device(channel) else {
                    continue;
                };
                let mut tmc = tmc_driver(device);
                let mut backend =
                    ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                if let Some(foci_firmware::commission_dispatch::CommissionOutcome::Terminal {
                    reply,
                    actions,
                }) = runtimes[channel_idx]
                    .drive_operator_cancel(
                        &mut backend,
                        tmc_state(channel),
                        stepper_state(channel),
                        TMC_CONFIG.pwm_maxcnt,
                    )
                    .await
                {
                    #[cfg(feature = "trace")]
                    {
                        commission_trace_phase[channel_idx] = None;
                    }
                    {
                        let runtime = &mut runtimes[channel_idx];
                        let mut state =
                            runtime.dispatch_state(tmc_state(channel), stepper_state(channel));
                        apply_stepper_post_action(actions.stepper, &mut state, channel);
                    }
                    emit_terminal_reply(reply);
                    commission_measurement_active[channel_idx] = false;
                }
            }

            macro_rules! quiesce_and_report_tmc_fault {
                () => {{
                    quiesce_all_channels(&mut runtimes, enable_pins);
                    #[cfg(feature = "trace")]
                    commission_trace_phase.fill(None);
                    let mut fault = foci_firmware::tmc_control::EmergencyDisableFault::Generic;
                    let mut shutdown_reported = false;
                    for report_channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                        let Some(device) = ctx.local.tmc_mux.device(report_channel) else {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "Invalid TMC channel",
                                        read_tim5(),
                                    );
                                }
                            });
                            continue;
                        };
                        let mut report_tmc = tmc_driver(device);
                        let report_status = report_tmc
                            .read_register_retried(foci_tmc4671::registers::STATUS_FLAGS)
                            .await
                            .map_err(|_| ());
                        let report_flags = report_status.unwrap_or(0);
                        let channel_fault =
                            foci_firmware::tmc_control::classify_emergency_disable_fault(
                                report_status,
                            );
                        fault =
                            foci_firmware::tmc_control::combine_emergency_disable_fault(
                                fault,
                                channel_fault,
                            );
                        match report_status {
                            Ok(report_flags) => defmt::error!(
                                "TMC4671 channel {} emergency disable: STATUS={:#010x}",
                                report_channel,
                                report_flags
                            ),
                            Err(_) => defmt::error!(
                                "TMC4671 channel {} emergency disable: STATUS read failed",
                                report_channel
                            ),
                        }
                        record_stepper_event(
                            StepperEventReason::TmcDisableSignal,
                            report_channel,
                            read_tim5(),
                            UNKNOWN_QUEUE_LEN,
                            report_flags,
                            0,
                        );
                        if channel_fault
                            == foci_firmware::tmc_control::EmergencyDisableFault::AdcCurrentClipped
                            && !shutdown_reported
                        {
                            shutdown_reported = true;
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "TMC4671: ADC current clipped (overcurrent)",
                                        read_tim5(),
                                    );
                                }
                            });
                        }
                    }
                    if !shutdown_reported {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                match fault {
                                    foci_firmware::tmc_control::EmergencyDisableFault::AdcCurrentClipped => {
                                        unreachable!(
                                            "shutdown_reported is set in the same loop iteration that classifies AdcCurrentClipped"
                                        )
                                    }
                                    foci_firmware::tmc_control::EmergencyDisableFault::StatusReadFailed => {
                                        ::ankyra::klipper_shutdown_from!(
                                            &mut crate::_ankyra_config::Sender,
                                            "TMC4671: STATUS_FLAGS read failed",
                                            read_tim5(),
                                        );
                                    }
                                    foci_firmware::tmc_control::EmergencyDisableFault::Generic => {
                                        ::ankyra::klipper_shutdown_from!(
                                            &mut crate::_ankyra_config::Sender,
                                            "TMC4671 fault",
                                            read_tim5(),
                                        );
                                    }
                                }
                            }
                        });
                    }
                    FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
                }};
            }

            macro_rules! handle_tmc_disable_signal {
                () => {
                    handle_tmc_disable_signal!({});
                };
                ($before_classification:block) => {{
                    if TMC_DISABLE_SIGNAL.try_take().is_some() {
                        $before_classification
                        let deadline_channel =
                            STEPPER_DEADLINE_MISS_CHANNEL.swap(NO_STEPPER_CHANNEL, Relaxed);
                        if deadline_channel != NO_STEPPER_CHANNEL {
                            quiesce_all_channels(&mut runtimes, enable_pins);
                            #[cfg(feature = "trace")]
                            commission_trace_phase.fill(None);
                            let clock = read_tim5();
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    foci_firmware::dispatch_shutdown!(
                                        foci_firmware::shutdown::ShutdownReason::StepperTooFarInPast,
                                        clock,
                                        &mut crate::_ankyra_config::Sender
                                    );
                                }
                            });
                            FAULT_ACTIVE.store(false, Relaxed);
                        } else {
                            quiesce_and_report_tmc_fault!();
                        }
                        continue 'control_loop;
                    }
                }};
            }

            macro_rules! poll_channel1_critical_status {
                () => {{
                    let channel = CHANNEL1_STATUS_FALLBACK;
                    let mut critical_fault = false;
                    if channel < crate::board_config::TMC_CHANNEL_COUNT
                        && channel_may_be_energized(channel, &runtimes[channel as usize])
                    {
                        let Some(device) = ctx.local.tmc_mux.device(channel) else {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "Invalid TMC channel",
                                        read_tim5(),
                                    );
                                }
                            });
                            continue 'control_loop;
                        };
                        let mut tmc = tmc_driver(device);
                        match tmc
                            .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                            .await
                        {
                            Ok(flags) => {
                                critical_fault = flags & STATUS_CRITICAL_FAULT_MASK != 0;
                                if critical_fault {
                                    defmt::error!(
                                        "TMC4671 channel {} critical STATUS fault: {:#010x}",
                                        channel,
                                        flags
                                    );
                                    latch_tmc_fault();
                                }
                            }
                            Err(_) => {
                                critical_fault = true;
                                defmt::error!(
                                    "TMC4671 channel {} critical STATUS fallback read failed",
                                    channel
                                );
                                latch_tmc_fault();
                            }
                        }
                    }
                    critical_fault
                }};
            }

            macro_rules! commission_wait_containment_snapshot {
                () => {{
                    (
                        SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
                        TMC_QUEUE_FLUSH.load(core::sync::atomic::Ordering::Relaxed),
                        TMC_DISABLE_SIGNAL.try_take().is_some(),
                        FAULT_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
                    )
                }};
            }

            macro_rules! handle_commission_wait_exit {
                ($channel:expr) => {{
                    let clock_sync_expired =
                        crate::tmc_control::containment::lease_expired(read_tim5());
                    let (shutdown, queue_flush, emergency_disable, fault_active) =
                        commission_wait_containment_snapshot!();
                    // Cancel fast-path stays off: a genuine fault must still win the
                    // channel-1 critical-STATUS poll below, so this is `false` until
                    // that priority is revisited.
                    let precheck = crate::tmc_control::effects::classify_commission_wait_precheck(
                        shutdown,
                        queue_flush,
                        emergency_disable,
                        fault_active,
                        clock_sync_expired,
                        false,
                    );
                    if let crate::tmc_control::effects::CommissionWaitPrecheck::Exit(exit) =
                        precheck
                    {
                        if !matches!(
                            exit,
                            crate::tmc_control::effects::CommissionWaitExit::Cancelled
                        ) {
                            for runtime in runtimes.iter_mut() {
                                runtime.request_restore_cancel();
                            }
                        }
                        if matches!(
                            exit,
                            crate::tmc_control::effects::CommissionWaitExit::EmergencyDisable
                        ) {
                            lower_all_enable_pins(enable_pins);
                            TMC_DISABLE_SIGNAL.signal(());
                        }
                        continue 'control_loop;
                    }

                    let critical_status = poll_channel1_critical_status!();
                    let (shutdown, queue_flush, emergency_disable, fault_active) =
                        commission_wait_containment_snapshot!();
                    let exit = crate::tmc_control::effects::classify_commission_wait_exit(
                        shutdown,
                        queue_flush,
                        emergency_disable,
                        critical_status || fault_active,
                        clock_sync_expired,
                        tmc_state($channel)
                            .cancel_requested
                            .load(core::sync::atomic::Ordering::Relaxed),
                    );
                    if let Some(exit) = exit {
                        if !matches!(
                            exit,
                            crate::tmc_control::effects::CommissionWaitExit::Cancelled
                        ) {
                            for runtime in runtimes.iter_mut() {
                                runtime.request_restore_cancel();
                            }
                        }
                        if matches!(
                            exit,
                            crate::tmc_control::effects::CommissionWaitExit::EmergencyDisable
                        ) {
                            lower_all_enable_pins(enable_pins);
                            TMC_DISABLE_SIGNAL.signal(());
                        }
                        continue 'control_loop;
                    }
                }};
            }

            handle_tmc_disable_signal!();

            // On shutdown, perform one-shot motor safe-state then idle until
            // shutdown clears. This catches USB disconnect / host crash where
            // no further commands arrive but the TMC4671 remains energized.
            if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                commission_measurement_active.fill(false);
                if !shutdown_handled {
                    quiesce_all_channels(&mut runtimes, enable_pins);
                    #[cfg(feature = "trace")]
                    commission_trace_phase.fill(None);
                    let mut all_shutdown_safe_state_complete = true;
                    for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                        let Some(device) = ctx.local.tmc_mux.device(channel) else {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "Invalid TMC channel",
                                        read_tim5(),
                                    );
                                }
                            });
                            continue;
                        };
                        let mut tmc = tmc_driver(device);
                        let runtime = &mut runtimes[channel as usize];
                        let mut backend =
                            ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                        all_shutdown_safe_state_complete &=
                            foci_firmware::tmc_control::lifecycle::apply_shutdown_safe_state(
                                runtime,
                                &mut backend,
                                tmc_state(channel),
                                stepper_state(channel),
                            )
                            .await;
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                    }
                    shutdown_handled = all_shutdown_safe_state_complete;
                    integral_candidate.observe_fault();
                }
                Mono::delay(1.millis()).await;
                continue;
            }
            shutdown_handled = false;

            // classify_lease_check(shutdown_active, ...) always returns
            // Proceed while shutdown is already active -- see its doc
            // comment for why.
            match crate::tmc_control::containment::classify_lease_check(
                SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed),
                crate::tmc_control::containment::lease_expired(read_tim5()),
            ) {
                crate::tmc_control::containment::LeaseCheckDisposition::EnterClockSyncShutdown => {
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            foci_firmware::dispatch_shutdown!(
                                foci_firmware::shutdown::ShutdownReason::HostClockSyncExpired,
                                read_tim5(),
                                &mut crate::_ankyra_config::Sender
                            );
                        }
                    });
                    continue;
                }
                crate::tmc_control::containment::LeaseCheckDisposition::Proceed => {}
            }

            // Drain stale requests after config_reset
            if TMC_QUEUE_FLUSH.swap(false, core::sync::atomic::Ordering::Relaxed) {
                commission_measurement_active.fill(false);
                integral_candidate.observe_fault();
                while rx.try_recv().is_ok() {}
                let mut needs_disarm = [false; crate::board_config::TMC_CHANNEL_COUNT as usize];
                for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                    needs_disarm[channel as usize] =
                        channel_needs_safe_state(channel, &runtimes[channel as usize]);
                }
                // Stop steppers, clear local armed/enabled state, and lower both
                // enable pins before any SPI awaits in the restore-safe-state pass.
                quiesce_all_channels(&mut runtimes, enable_pins);
                #[cfg(feature = "trace")]
                commission_trace_phase.fill(None);

                for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                    if needs_disarm[channel as usize] {
                        let Some(device) = ctx.local.tmc_mux.device(channel) else {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "Invalid TMC channel",
                                        read_tim5(),
                                    );
                                }
                            });
                            continue;
                        };
                        let mut tmc = tmc_driver(device);
                        let mut backend =
                            ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                        let runtime = &mut runtimes[channel as usize];
                        let mut state =
                            runtime.dispatch_state(tmc_state(channel), stepper_state(channel));
                        foci_firmware::tmc_control::lifecycle::disarm_motor(
                            &mut backend,
                            &mut state,
                        )
                        .await;
                    }
                    runtimes[channel as usize].reset_runtime_state();
                    foci_firmware::tmc_control::TmcControlRuntime::reset_channel_state(
                        tmc_state(channel),
                        stepper_state(channel),
                    );
                    tmc_state(channel)
                        .commission_in_flight
                        .store(false, core::sync::atomic::Ordering::Relaxed);
                }
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
                status_poll_counters.fill(0);
                defmt::info!("TMC request queue flushed after config_reset");
            }

            // Tick an active commissioning engine through the shared absolute scheduler.
            'commission_channel: for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                let channel_idx = channel as usize;
                if !runtimes[channel_idx].is_commission_active() {
                    commission_measurement_active[channel_idx] = false;
                    continue;
                }
                if !commission_measurement_active[channel_idx] {
                    commission_poll_drivers[channel_idx] =
                        foci_firmware::tmc_control::CommissionPollDriver::new();
                    commission_tick_max_cycles[channel_idx] = 0;
                    commission_measurement_active[channel_idx] = true;
                }

                'commission_poll: loop {
                    handle_commission_wait_exit!(channel);

                    // A terminal restore owns this channel until it reports its
                    // terminal: advance it one backend operation per pass
                    // instead of stepping the engine again.
                    if runtimes[channel_idx].is_restore_pending() {
                        let action = {
                            let Some(device) = ctx.local.tmc_mux.device(channel) else {
                                ctx.shared.app_state.lock(|state| {
                                    if crate::firmware_ctx::enter_shutdown(state) {
                                        ::ankyra::klipper_shutdown_from!(
                                            &mut crate::_ankyra_config::Sender,
                                            "Invalid TMC channel",
                                            read_tim5(),
                                        );
                                    }
                                });
                                runtimes[channel_idx].request_restore_cancel();
                                continue 'commission_channel;
                            };
                            let mut tmc = tmc_driver(device);
                            let mut backend = ouroboros_backend(
                                channel,
                                &mut tmc,
                                enable_pin(enable_pins, channel),
                            );
                            let mut retention =
                                foci_firmware::commission_dispatch::CommissionRetention::new(
                                    &mut *integral_candidate,
                                );
                            runtimes[channel_idx]
                                .poll_commissioning_terminal(
                                    &mut backend,
                                    &mut retention,
                                    tmc_state(channel),
                                    stepper_state(channel),
                                )
                                .await
                        };
                        match action {
                            None => continue 'commission_poll,
                            Some(
                                foci_firmware::tmc_control::ScheduledCommissionTickAction::Terminal {
                                    reply,
                                    actions,
                                },
                            ) => {
                                #[cfg(feature = "trace")]
                                {
                                    commission_trace_phase[channel_idx] = None;
                                }
                                {
                                    let runtime = &mut runtimes[channel_idx];
                                    let mut state = runtime.dispatch_state(
                                        tmc_state(channel),
                                        stepper_state(channel),
                                    );
                                    apply_stepper_post_action(actions.stepper, &mut state, channel);
                                }
                                emit_terminal_reply(reply);
                                defmt::info!(
                                    "commissioning channel {} terminal max tick cycles: {}",
                                    channel,
                                    commission_tick_max_cycles[channel_idx],
                                );
                                commission_measurement_active[channel_idx] = false;
                                break 'commission_poll;
                            }
                            Some(_) => {
                                unreachable!("terminal restore produced a nonterminal action")
                            }
                        }
                    }

                    #[cfg(feature = "trace")]
                    let engine_tick_deadline =
                        Mono::now() + fugit::Duration::<u64, 1, 1_000_000>::millis(1);
                    let engine_kind = runtimes[channel_idx]
                        .pending_commission_kind()
                        .expect("active commissioning engine has a kind");
                    let tick_start = cortex_m::peripheral::DWT::cycle_count();

                    let mut poll = {
                        let Some(device) = ctx.local.tmc_mux.device(channel) else {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "Invalid TMC channel",
                                        read_tim5(),
                                    );
                                }
                            });
                            runtimes[channel_idx].request_restore_cancel();
                            continue 'commission_channel;
                        };
                        let mut tmc = tmc_driver(device);
                        let runtime = &mut runtimes[channel_idx];
                        let mut backend =
                            ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                        runtime
                            .step_commissioning_scheduled(&mut backend)
                            .await
                            .expect("active commissioning engine produces a poll")
                    };
                    let now_us = Mono::now().duration_since_epoch().to_micros();
                    let driver_action = match commission_poll_drivers[channel_idx]
                        .classify(&poll, now_us)
                    {
                        Ok(action) => action,
                        Err(error) => {
                            poll =
                                foci_firmware::commissioning::schedule::CommissionPoll::Terminal {
                                    result: if matches!(
                                        engine_kind,
                                        foci_firmware::commissioning::types::EngineKind::OuterTune
                                    ) {
                                        foci_firmware::commissioning::schedule::CommissionTerminal::OuterFailed(
                                    error,
                                )
                                    } else {
                                        foci_firmware::commissioning::schedule::CommissionTerminal::InnerFailed(
                                    error,
                                )
                                    },
                                };
                            foci_firmware::tmc_control::PollDriverAction::Terminal
                        }
                    };
                    if matches!(
                        driver_action,
                        foci_firmware::tmc_control::PollDriverAction::YieldNow
                    ) {
                        Mono::delay(foci_firmware::tmc_control::FORCED_READY_YIELD_US.micros())
                            .await;
                        handle_commission_wait_exit!(channel);
                    }

                    let action = {
                        let mut retention =
                            foci_firmware::commission_dispatch::CommissionRetention::new(
                                &mut *integral_candidate,
                            );
                        runtimes[channel_idx].begin_commissioning_poll_with_stages(
                            poll,
                            &mut retention,
                            &tmc_control_config,
                        )
                    };
                    let Some(action) = action else {
                        // A terminal poll starts the restore machine instead of
                        // producing an action; the drain block above advances it.
                        assert!(
                            runtimes[channel_idx].is_restore_pending(),
                            "active commissioning poll produces an action"
                        );
                        continue 'commission_poll;
                    };
                    let tick_cycles =
                        cortex_m::peripheral::DWT::cycle_count().wrapping_sub(tick_start);
                    commission_tick_max_cycles[channel_idx] =
                        commission_tick_max_cycles[channel_idx].max(tick_cycles);
                    handle_commission_wait_exit!(channel);

                    use foci_firmware::tmc_control::ScheduledCommissionTickAction;
                    match action {
                        ScheduledCommissionTickAction::ReadyNow { event }
                        | ScheduledCommissionTickAction::At { event, .. } => {
                            let phase_changed = matches!(
                                &event,
                                Some(
                                    foci_firmware::commissioning::schedule::CommissionEvent::PhaseChanged {
                                        ..
                                    }
                                )
                            );
                            if let Some(event) = event {
                                match event {
                                    foci_firmware::commissioning::schedule::CommissionEvent::PhaseChanged {
                                        oid,
                                        phase_id,
                                        engine_kind,
                                    } => {
                                        #[cfg(not(feature = "trace"))]
                                        let _ = engine_kind;
                                        #[cfg(feature = "trace")]
                                        {
                                            commission_trace_phase[channel_idx] =
                                                Some((engine_kind, phase_id));
                                        }
                                        emit_phase_reply(oid, phase_id);
                                    }
                                    foci_firmware::commissioning::schedule::CommissionEvent::VelocityIntegral {
                                        oid,
                                        event,
                                    } => {
                                        crate::ankyra::replies::emit_velocity_integral_event(
                                            oid, event,
                                        );
                                    }
                                    foci_firmware::commissioning::schedule::CommissionEvent::BreakawayCampaign {
                                        oid,
                                        event,
                                    } => {
                                        crate::ankyra::replies::emit_breakaway_campaign_event(
                                            oid, event,
                                        );
                                    }
                                    foci_firmware::commissioning::schedule::CommissionEvent::VelocityIntegralPlan {
                                        oid,
                                        run_sequence,
                                        plan,
                                    } => {
                                        crate::ankyra::replies::emit_velocity_integral_plan(
                                            oid, run_sequence, plan,
                                        );
                                    }
                                }
                            }
                            #[cfg(feature = "trace")]
                        if let Some((engine_kind, phase_id)) =
                            commission_trace_phase[channel_idx]
                            && foci_firmware::trace::commissioning::commission_trace_phase_samples_closed_loop_activation(
                                engine_kind,
                                phase_id,
                            ) {
                            let trace_tick = runtimes[channel_idx].advance_trace_sample_tick();
                            let refs =
                                TmcControlStateRefs::new(tmc_state(channel), stepper_state(channel));
                            let motor_enabled = runtimes[channel_idx].is_motor_enabled();
                            let step_timing = crate::usb_trace::take_step_timing_snapshot();
                            let step_timing = if crate::usb_trace::is_configured() {
                                Some(step_timing)
                            } else {
                                None
                            };
                            let Some(device) = ctx.local.tmc_mux.device(channel) else {
                                ctx.shared.app_state.lock(|state| {
                                    if crate::firmware_ctx::enter_shutdown(state) {
                                        ::ankyra::klipper_shutdown_from!(
                                            &mut crate::_ankyra_config::Sender,
                                            "Invalid TMC channel",
                                            read_tim5(),
                                        );
                                    }
                                });
                                runtimes[channel_idx].request_restore_cancel();
                                continue 'control_loop;
                            };
                            let mut tmc = tmc_driver(device);
                            crate::usb_trace::sample_commission_closed_loop_activation(
                                &mut tmc,
                                crate::usb_trace::TraceSampleContext {
                                    channel,
                                    refs,
                                    motor_enabled,
                                    step_timing,
                                },
                                trace_tick,
                                || {
                                    engine_tick_deadline
                                        .checked_duration_since(Mono::now())
                                        .map(|d| d.to_micros())
                                        .unwrap_or(0)
                                },
                                || Mono::now() > engine_tick_deadline,
                            )
                            .await;
                        }

                            match foci_firmware::tmc_control::nonterminal_continuation(
                                driver_action,
                                phase_changed,
                            ) {
                                foci_firmware::tmc_control::PollDriverAction::PollNow
                                | foci_firmware::tmc_control::PollDriverAction::YieldNow => {
                                    runtimes[channel_idx].request_restore_cancel();
                                    continue 'control_loop;
                                }
                                foci_firmware::tmc_control::PollDriverAction::WaitUntil(
                                    deadline_us,
                                ) => {
                                    loop {
                                        handle_commission_wait_exit!(channel);

                                        if let Ok(req) = rx.try_recv() {
                                            use foci_firmware::tmc_control::CommissionWaitRequestDisposition;
                                            match crate::tmc_control::effects::commission_wait_request_disposition(&req) {
                                            CommissionWaitRequestDisposition::Cancel => {
                                                let request_channel = req.channel;
                                                let cancel_route = crate::tmc_control::effects::classify_commission_wait_cancel(
                                                    channel,
                                                    request_channel,
                                                    crate::board_config::TMC_CHANNEL_COUNT,
                                                );
                                                if matches!(
                                                    cancel_route,
                                                    crate::tmc_control::effects::CommissionWaitCancelRoute::Invalid
                                                ) {
                                                    emit_invalid_channel_diagnostic(request_channel);
                                                    ctx.shared.app_state.lock(|state| {
                                                        if crate::firmware_ctx::enter_shutdown(state) {
                                                            ::ankyra::klipper_shutdown_from!(
                                                                &mut crate::_ankyra_config::Sender,
                                                                "Invalid TMC channel",
                                                                read_tim5(),
                                                            );
                                                        }
                                                    });
                                                    runtimes[channel_idx].request_restore_cancel();
                                                    continue 'control_loop;
                                                }

                                                let request_idx = request_channel as usize;
                                                enable_pin(enable_pins, request_channel).set_low();
                                                tmc_state(request_channel).commission_in_flight.store(
                                                    false,
                                                    core::sync::atomic::Ordering::Relaxed,
                                                );
                                                commission_poll_drivers[request_idx] =
                                                    foci_firmware::tmc_control::CommissionPollDriver::new();
                                                commission_tick_max_cycles[request_idx] = 0;
                                                commission_measurement_active[request_idx] = false;
                                                #[cfg(feature = "trace")]
                                                {
                                                    commission_trace_phase[request_idx] = None;
                                                }
                                                let Some(device) = ctx.local.tmc_mux.device(request_channel) else {
                                                    ctx.shared.app_state.lock(|state| {
                                                        if crate::firmware_ctx::enter_shutdown(state) {
                                                            ::ankyra::klipper_shutdown_from!(
                                                                &mut crate::_ankyra_config::Sender,
                                                                "Invalid TMC channel",
                                                                read_tim5(),
                                                            );
                                                        }
                                                    });
                                                    runtimes[channel_idx].request_restore_cancel();
                                                    continue 'control_loop;
                                                };
                                                let mut request_tmc = tmc_driver(device);
                                                let outcome = crate::tmc_control::request::handle_tmc_control_request(
                                                    req,
                                                    TmcPreRequestDisposition::ContinueRequest,
                                                    crate::tmc_control::request::TmcControlRequestContext {
                                                        channel: request_channel,
                                                        commissioning_blocked: false,
                                                        runtimes: &mut runtimes,
                                                        tmc: &mut request_tmc,
                                                        enable_pins,
                                                        commission_trace_engine_seq: &mut commission_trace_engine_seq,
                                                        integral_candidate: &mut *integral_candidate,
                                                    },
                                                )
                                                .await;
                                                if let Some(reason) = outcome.shutdown {
                                                    ctx.shared.app_state.lock(|state| {
                                                        if crate::firmware_ctx::enter_shutdown(state) {
                                                            crate::tmc_control::effects::emit_shutdown(reason);
                                                        }
                                                    });
                                                }
                                                if outcome.should_refresh_velocity_scale {
                                                    refresh_velocity_scale(
                                                        request_channel,
                                                        &mut runtimes[request_idx],
                                                    );
                                                }
                                                if matches!(
                                                    cancel_route,
                                                    crate::tmc_control::effects::CommissionWaitCancelRoute::Current
                                                ) {
                                                    runtimes[channel_idx].request_restore_cancel();
                                                    continue 'control_loop;
                                                }
                                            }
                                            CommissionWaitRequestDisposition::RejectBusy => {
                                                let outcome = crate::tmc_control::effects::handle_commission_wait_reject(req);
                                                if let Some(reason) = outcome.shutdown {
                                                    ctx.shared.app_state.lock(|state| {
                                                        if crate::firmware_ctx::enter_shutdown(state) {
                                                            crate::tmc_control::effects::emit_shutdown(reason);
                                                        }
                                                    });
                                                }
                                            }
                                            CommissionWaitRequestDisposition::DropBusy => {}
                                        }
                                        }

                                        handle_commission_wait_exit!(channel);
                                        let now_us = Mono::now().duration_since_epoch().to_micros();
                                        match foci_firmware::tmc_control::classify_absolute_wait(
                                            now_us,
                                            deadline_us,
                                        ) {
                                            Ok(
                                                foci_firmware::tmc_control::AbsoluteWaitAction::PollNow,
                                            ) => break,
                                            Ok(
                                                foci_firmware::tmc_control::AbsoluteWaitAction::SleepUntil(
                                                    wake_us,
                                                ),
                                            ) => {
                                                let wake = fugit::Instant::<u64, 1, 1_000_000>::from_ticks(
                                                    wake_us,
                                                );
                                                Mono::delay_until(wake).await;
                                            }
                                            Err(_) => {
                                                lower_all_enable_pins(enable_pins);
                                                TMC_DISABLE_SIGNAL.signal(());
                                                runtimes[channel_idx].request_restore_cancel();
                                                continue 'control_loop;
                                            }
                                        }
                                    }
                                    continue 'commission_poll;
                                }
                                foci_firmware::tmc_control::PollDriverAction::Terminal => {
                                    unreachable!("nonterminal poll classified as terminal")
                                }
                            }
                        }
                        ScheduledCommissionTickAction::WorkflowContinuation => {
                            continue 'commission_poll;
                        }
                        ScheduledCommissionTickAction::Terminal { reply, actions } => {
                            #[cfg(feature = "trace")]
                            {
                                commission_trace_phase[channel_idx] = None;
                            }
                            {
                                let runtime = &mut runtimes[channel_idx];
                                let mut state = runtime
                                    .dispatch_state(tmc_state(channel), stepper_state(channel));
                                apply_stepper_post_action(actions.stepper, &mut state, channel);
                            }
                            emit_terminal_reply(reply);
                            defmt::info!(
                                "commissioning channel {} terminal max tick cycles: {}",
                                channel,
                                commission_tick_max_cycles[channel_idx],
                            );
                            commission_measurement_active[channel_idx] = false;
                            break 'commission_poll;
                        }
                    }
                }
                // Terminal cases fall through to normal channel-receive processing.
                break;
            }

            let control_refs = [
                TmcControlStateRefs::new(tmc_state(0), stepper_state(0)),
                TmcControlStateRefs::new(tmc_state(1), stepper_state(1)),
            ];
            let control_period_us =
                TmcControlRuntime::minimum_control_period_us(&runtimes, &control_refs);

            match select(
                rx.recv(),
                Mono::delay(u64::from(control_period_us).micros()),
            )
            .await
            {
                // Channel message received
                Either::First(Ok(req)) => {
                    if req.channel >= crate::board_config::TMC_CHANNEL_COUNT {
                        emit_invalid_channel_diagnostic(req.channel);
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "Invalid TMC channel",
                                    read_tim5(),
                                );
                            }
                        });
                        continue;
                    }
                    let channel = req.channel;
                    let Some(device) = ctx.local.tmc_mux.device(channel) else {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "Invalid TMC channel",
                                    read_tim5(),
                                );
                            }
                        });
                        continue;
                    };
                    let mut tmc = tmc_driver(device);

                    let commissioning_blocked = match &req.kind {
                        TmcRequestKind::SetEnable(true) => {
                            !runtimes[channel as usize].is_calibrated()
                                && commissioning_request_blocked(&runtimes, channel)
                        }
                        TmcRequestKind::Calibrate { .. }
                        | TmcRequestKind::Commission { .. }
                        | TmcRequestKind::Tune { .. }
                        | TmcRequestKind::SelfTest { .. } => {
                            commissioning_request_blocked(&runtimes, channel)
                        }
                        _ => false,
                    };
                    let outcome = crate::tmc_control::request::handle_tmc_control_request(
                        req,
                        TmcPreRequestDisposition::ContinueRequest,
                        crate::tmc_control::request::TmcControlRequestContext {
                            channel,
                            commissioning_blocked,
                            runtimes: &mut runtimes,
                            tmc: &mut tmc,
                            enable_pins,
                            commission_trace_engine_seq: &mut commission_trace_engine_seq,
                            integral_candidate: &mut *integral_candidate,
                        },
                    )
                    .await;

                    if let Some(reason) = outcome.shutdown {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                crate::tmc_control::effects::emit_shutdown(reason);
                            }
                        });
                    }

                    if outcome.should_refresh_velocity_scale {
                        refresh_velocity_scale(channel, &mut runtimes[channel as usize]);
                    }

                    match outcome.tail {
                        foci_firmware::tmc_control::TmcControlRequestTail::FallThrough => {}
                        foci_firmware::tmc_control::TmcControlRequestTail::SkipTickTail => {
                            continue 'control_loop;
                        }
                    }
                }
                Either::First(Err(_)) => {}
                // 1ms timeout — write position target via SPI, and push any
                // configuration recorded by config-time setters. Those setters
                // record state instead of enqueueing, so a connect-time burst
                // cannot overflow the bounded TMC request channel.
                Either::Second(()) => {
                    for pending_channel in 0..foci_firmware::state::tmc::MAX_TMC_CHANNELS as u8 {
                        let Some(device) = ctx.local.tmc_mux.device(pending_channel) else {
                            continue;
                        };
                        let mut pending_tmc = tmc_driver(device);
                        if foci_firmware::tmc_control::filters::apply_pending_config(
                            &mut pending_tmc,
                            tmc_state(pending_channel),
                            TMC_CONFIG.pwm_maxcnt,
                        )
                        .await
                        .is_err()
                        {
                            defmt::error!(
                                "pending TMC config apply failed: channel {}",
                                pending_channel
                            );
                        }
                    }
                }
            }

            // Check emergency disable (non-blocking). Triggered by EXTI
            // STATUS interrupt or enter_shutdown. Disables motor and reports
            // the fault to Klipper so the host knows the MCU is down.
            handle_tmc_disable_signal!();

            // Poll STATUS_FLAGS every ~1 second on the shared slow-control
            // cadence. Channel 1 still gets a per-control-tick critical poll
            // while it may be energized because PA2 cannot share EXTI2 with PE2.
            let slow_control_tick = TmcControlRuntime::update_slow_control_accumulator(
                &mut slow_control_accum_us,
                control_period_us,
            );

            let homing = HOMING_STATE.load(core::sync::atomic::Ordering::Relaxed);
            let mut all_channels_ready_for_homing_advance = true;
            #[cfg(feature = "trace")]
            let mut trace_metadata_due = false;
            #[cfg(feature = "trace")]
            let step_timing = if slow_control_tick {
                let snapshot = crate::usb_trace::take_step_timing_snapshot();
                if crate::usb_trace::is_configured() {
                    Some(snapshot)
                } else {
                    None
                }
            } else {
                None
            };

            #[cfg(feature = "trace")]
            let mut trace_spi_cycles: u32 = 0;

            for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                let Some(device) = ctx.local.tmc_mux.device(channel) else {
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            ::ankyra::klipper_shutdown_from!(
                                &mut crate::_ankyra_config::Sender,
                                "Invalid TMC channel",
                                read_tim5(),
                            );
                        }
                    });
                    continue;
                };
                let mut tmc = tmc_driver(device);
                let poll_channel1_critical = channel == CHANNEL1_STATUS_FALLBACK
                    && channel_may_be_energized(channel, &runtimes[channel as usize]);
                if poll_channel1_critical {
                    match tmc
                        .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                        .await
                    {
                        Ok(flags) if flags & STATUS_CRITICAL_FAULT_MASK != 0 => {
                            defmt::error!(
                                "TMC4671 channel {} critical STATUS fault: {:#010x}",
                                channel,
                                flags
                            );
                            latch_tmc_fault();
                            let _ = tmc;
                            quiesce_and_report_tmc_fault!();
                            continue 'control_loop;
                        }
                        Ok(_) => {}
                        Err(_) => {
                            defmt::error!(
                                "TMC4671 channel {} critical STATUS fallback read failed",
                                channel
                            );
                            latch_tmc_fault();
                            let _ = tmc;
                            quiesce_and_report_tmc_fault!();
                            continue 'control_loop;
                        }
                    }
                }
                let runtime = &mut runtimes[channel as usize];

                let refs = TmcControlStateRefs::new(tmc_state(channel), stepper_state(channel));
                let homing_active = homing != 0;
                let reset = refs
                    .tmc
                    .feedforward_reset
                    .swap(false, core::sync::atomic::Ordering::Relaxed);
                let command_velocity_reference =
                    runtime.command_velocity_reference(refs, homing_active, reset);

                if runtime
                    .apply_feedforward_tick(
                        &mut tmc,
                        refs,
                        &tmc_control_config,
                        homing_active,
                        reset,
                        slow_control_tick,
                    )
                    .await
                    .is_err()
                {
                    defmt::warn!("TMC feedforward tick failed");
                }

                if !slow_control_tick {
                    if homing != 0 && runtime.is_motor_enabled() {
                        all_channels_ready_for_homing_advance = false;
                    }
                    continue;
                }

                runtime
                    .apply_phase_and_position_tick(
                        &mut tmc,
                        refs,
                        command_velocity_reference,
                        homing_active,
                        reset,
                    )
                    .await;

                #[cfg(feature = "trace")]
                {
                    let trace_tick = runtime.advance_trace_tick();
                    let inputs =
                        foci_firmware::trace::sampling::TraceSampleInputs::from_runtime_trace(
                            runtime.trace_state(),
                            trace_tick,
                        );
                    trace_spi_cycles = trace_spi_cycles.saturating_add(
                        crate::usb_trace::sample_tick(
                            &mut tmc,
                            crate::usb_trace::TraceSampleContext {
                                channel,
                                refs,
                                motor_enabled: runtime.is_motor_enabled(),
                                step_timing,
                            },
                            inputs,
                        )
                        .await,
                    );
                    if trace_tick.phase_advance_due {
                        crate::usb_trace::emit_phase_advance(
                            channel,
                            trace_tick.phase_advance_counts,
                            trace_tick.phase_advance_flags,
                            refs.tmc
                                .phase_advance_gain_ppm
                                .load(core::sync::atomic::Ordering::Relaxed),
                            refs.tmc
                                .phase_advance_max_counts
                                .load(core::sync::atomic::Ordering::Relaxed),
                            refs.tmc
                                .phase_advance_deadband
                                .load(core::sync::atomic::Ordering::Relaxed),
                        );
                    }
                    crate::usb_trace::emit_status_frames(
                        channel,
                        &mut tmc,
                        trace_tick.control_tick,
                        trace_tick.status_detail_due,
                    )
                    .await;
                    if channel == 0 {
                        trace_metadata_due = trace_tick.metadata_due;
                    }
                }

                let homing_result = runtime
                    .apply_slow_channel_maintenance_tick(
                        &mut tmc,
                        refs,
                        &tmc_control_config,
                        homing,
                    )
                    .await;
                match homing_result {
                    HomingMaintenanceResult::ReadyToAdvance => {}
                    HomingMaintenanceResult::RetryCurrentState => {
                        all_channels_ready_for_homing_advance = false;
                        continue;
                    }
                    HomingMaintenanceResult::FailClosed => {
                        defmt::error!(
                            "TMC4671 channel {} homing failed closed, disabling motors",
                            channel
                        );
                        refs.tmc
                            .homing_clamp_active
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                        latch_tmc_fault();
                        let _ = tmc;
                        quiesce_and_report_tmc_fault!();
                        continue 'control_loop;
                    }
                }

                // TrSync and endstop polling have been replaced by the TIM5 CC2
                // trigger scheduler. See step_timer_irq CC2 dispatch above.

                let status_idx = channel as usize;
                status_poll_counters[status_idx] =
                    status_poll_counters[status_idx].saturating_add(1);
                let poll_status = if status_poll_counters[status_idx] >= 1000 {
                    status_poll_counters[status_idx] = 0;
                    true
                } else {
                    false
                };

                if poll_status
                    && let Ok(flags) = tmc
                        .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                        .await
                {
                    // Warning flags: PID output limits (position, velocity, current)
                    const PID_LIMIT_MASK: u32 = 0x0000_FFFF; // bits 0-15
                    const PWM_LIMIT_MASK: u32 = (1 << 24) | (1 << 25); // pwm_min, pwm_max
                    const FAULT_MASK: u32 = STATUS_ADC_I_CLIPPED;

                    if flags & FAULT_MASK != 0 {
                        defmt::error!("TMC4671 channel {} STATUS fault: {:#010x}", channel, flags);
                        latch_tmc_fault();
                        let _ = tmc;
                        quiesce_and_report_tmc_fault!();
                        continue 'control_loop;
                    } else if runtime.is_motor_enabled()
                        && flags & (PID_LIMIT_MASK | PWM_LIMIT_MASK) != 0
                    {
                        defmt::warn!("TMC4671 channel {} STATUS: {:#010x}", channel, flags);
                    }
                }
            }

            #[cfg(feature = "trace")]
            if trace_spi_cycles != 0 {
                crate::usb_trace::record_spi_sample_cycles(trace_spi_cycles);
            }

            #[cfg(feature = "trace")]
            if slow_control_tick {
                crate::usb_trace::request_spi_duration_event_if_ready();
                if trace_metadata_due {
                    crate::usb_trace::emit_metadata();
                }
            }

            // Suppressing: advance to RestorePending once a trigger has fired
            // via the HOMING_TRIGGERED sentinel (set by TriggerDomain on first
            // trigger, 0xFF = no trigger pending). P1 never enters
            // TriggerDomain per spec. If host's trsync_trigger runs before P1
            // polls, trsync_trigger advances HOMING_STATE 2->3 directly.
            TmcControlRuntime::advance_homing_state_after_channels(
                TmcControlHomingRefs::new(&HOMING_STATE, &HOMING_TRIGGERED),
                homing,
                all_channels_ready_for_homing_advance,
            );
        }
    }

    #[inline]
    fn step_clock_is_due(clock: u32, now: u32) -> bool {
        (clock as i32).wrapping_sub(now as i32) <= 0
    }

    fn program_next_step_compare(
        next_event_clocks: &[Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        let tim5 = embassy_stm32::pac::TIM5;
        if let Some(due) = foci_firmware::stepper_runtime::earliest_due(next_event_clocks) {
            arm_step_timer(due.clock);
        } else {
            tim5.dier().modify(|w| w.set_ccie(0, false));
        }
    }

    fn clear_step_channel(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        use core::sync::atomic::Ordering::Relaxed;

        let idx = channel as usize;
        if idx < next_event_clocks.len() {
            next_event_clocks[idx] = None;
        }
        if let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) {
            stepper_state.interval.store(0, Relaxed);
            stepper_state.timer_active_mirror.store(false, Relaxed);
        }
        if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) {
            tmc_state.pid_error_sums_clear.store(true, Relaxed);
        }
        with_trigger_domain(|td| {
            td.set_timer_active(idx, false);
        });
    }

    fn fail_step_channel_deadline(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) {
            tmc_state
                .motor_armed
                .store(false, core::sync::atomic::Ordering::Relaxed);
        }
        clear_step_channel(channel, next_event_clocks);
        // SAFETY: ISR stepper state is only accessed from the P3 TIM5 ISR
        // for this channel. P3 tasks cannot preempt each other on Cortex-M.
        unsafe {
            if let Some(stepper) = crate::state::stepper::isr_stepper(channel) {
                stepper.reset();
            }
        }
        let _ = crate::stepper_pins::reset_idle(channel);
        TMC_DISABLE_SIGNAL.signal(());
    }

    fn abort_step_dispatch(
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        use core::sync::atomic::Ordering::Relaxed;

        embassy_stm32::pac::TIM5
            .dier()
            .modify(|w| w.set_ccie(0, false));
        for idx in 0..crate::state::stepper::STEPPER_CHANNEL_COUNT {
            let channel = idx as u8;
            if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) {
                tmc_state.motor_armed.store(false, Relaxed);
            }
            clear_step_channel(channel, next_event_clocks);
            // SAFETY: ISR stepper state is only accessed from the P3 TIM5 ISR.
            unsafe {
                if let Some(stepper) = crate::state::stepper::isr_stepper(channel) {
                    stepper.reset();
                }
            }
            let _ = crate::stepper_pins::reset_idle(channel);
        }
    }

    fn channel_stepper_ready(channel: u8) -> bool {
        use core::sync::atomic::Ordering::Relaxed;

        let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
            return false;
        };
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            return false;
        };
        tmc_state.motor_armed.load(Relaxed) && stepper_state.timer_active_mirror.load(Relaxed)
    }

    #[inline(always)]
    fn add_scheduler_cycles(accumulator: &core::cell::Cell<u32>, start: u32) {
        accumulator.set(
            accumulator
                .get()
                .wrapping_add(cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start)),
        );
    }

    fn record_schedule_result(result: foci_klipper::scheduler::ScheduleResult) {
        if matches!(result, foci_klipper::scheduler::ScheduleResult::Full) {
            crate::state::stepper_diag::record_scheduler_full();
        }
    }

    #[inline(always)]
    fn spin_tim5_ticks(start: u32, ticks: u32) {
        while read_tim5().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }

    struct OuroborosPulseTrainIo {
        channel: u8,
    }

    impl crate::pulse_train::PulseTrainIo for OuroborosPulseTrainIo {
        type Error = ();

        #[inline(always)]
        fn set_step(&mut self, high: bool) -> Result<(), Self::Error> {
            crate::stepper_pins::set_step(self.channel, high)
        }

        #[inline(always)]
        fn set_dir(&mut self, forward: bool) -> Result<(), Self::Error> {
            crate::stepper_pins::set_dir(self.channel, forward)
        }

        #[inline(always)]
        fn wait_ticks(&mut self, ticks: u32) {
            let start = read_tim5();
            spin_tim5_ticks(start, ticks);
        }
    }

    /// Emit one bounded logical event as a complete, non-preemptible pulse train.
    ///
    /// This P3 hot path performs no SPI, logging, allocation, scheduler work,
    /// or yield. The mapper guarantees zero, one, or two physical pulses.
    #[inline(always)]
    fn emit_logical_step_event(
        channel: u8,
        event: foci_klipper::stepper::LogicalStepEvent,
    ) -> Result<(), ()> {
        crate::pulse_train::emit_logical_step_event(
            &mut OuroborosPulseTrainIo { channel },
            event,
            crate::state::stepper::STEP_DIR_SETUP_TICKS,
            crate::state::stepper::STEP_HALF_PERIOD_TICKS,
        )
    }

    fn load_next_step_move(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) -> StepDispatchOutcome {
        use core::sync::atomic::Ordering::Relaxed;

        let idx = channel as usize;
        if !channel_stepper_ready(channel) {
            next_event_clocks[idx] = None;
            return StepDispatchOutcome::Continue;
        }
        if HOMING_STATE.load(Relaxed) == 1 {
            // P1 has not finished homing preparation; re-check in 1 ms.
            next_event_clocks[idx] = Some(read_tim5().wrapping_add(CLOCK_FREQ / 1000));
            return StepDispatchOutcome::Continue;
        }
        let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };

        // SAFETY: ISR stepper state is only accessed from this P3 ISR for
        // this channel. P3 tasks cannot preempt each other on Cortex-M.
        let Some(stepper) = (unsafe { crate::state::stepper::isr_stepper(channel) }) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        if stepper.has_active_segment() {
            if next_event_clocks[idx].is_none() {
                next_event_clocks[idx] = Some(stepper.next_step_clock());
            }
            return StepDispatchOutcome::Continue;
        }

        // SAFETY: P3 is the sole consumer of this channel's queue. P3 ISRs
        // cannot preempt each other on Cortex-M.
        let next_move =
            unsafe { crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].isr_dequeue() };
        let Some(move_) = next_move else {
            crate::state::stepper_diag::record_queue_empty(channel);
            if HOMING_STATE.load(Relaxed) != 0 {
                record_stepper_event(
                    StepperEventReason::QueueEmpty,
                    channel,
                    read_tim5(),
                    0,
                    0,
                    0,
                );
            }
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        let loaded_steps = u32::from(move_.count);

        let old_direction = stepper.dir_state();
        stepper.load_move(move_);
        let dir_changed = old_direction != stepper.dir_state();
        stepper_state
            .interval
            .store(stepper.current_interval(), Relaxed);
        if dir_changed {
            tmc_state.feedforward_reset.store(true, Relaxed);
        }

        let tim5 = embassy_stm32::pac::TIM5;
        let now = tim5.cnt().read();
        let scheduled = stepper.next_step_clock();
        match stepper.check_deadline(scheduled, now, CLOCK_FREQ) {
            Ok(compare) => {
                crate::state::stepper_diag::record_loaded_segment_timing(
                    channel,
                    loaded_steps,
                    now,
                    scheduled,
                    compare,
                );
                let immediate = compare != scheduled;
                if immediate {
                    next_event_clocks[idx] = Some(now);
                } else {
                    next_event_clocks[idx] = Some(compare);
                }
            }
            Err(foci_klipper::stepper::MissedDeadline) => {
                crate::state::stepper_diag::record_loaded_segment(channel, loaded_steps);
                crate::state::stepper_diag::record_missed_deadline(channel);
                record_stepper_event(
                    StepperEventReason::MissedDeadlineLoad,
                    channel,
                    now,
                    UNKNOWN_QUEUE_LEN,
                    scheduled,
                    now,
                );
                #[cfg(feature = "trace")]
                crate::usb_trace::note_missed_or_deferred_step();
                STEPPER_DEADLINE_MISS_CHANNEL.store(channel, Relaxed);
                fail_step_channel_deadline(channel, next_event_clocks);
                return StepDispatchOutcome::Fatal;
            }
        }
        StepDispatchOutcome::Continue
    }

    fn process_due_step_event(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) -> StepDispatchOutcome {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::stepper::StepEventResult;

        let idx = channel as usize;
        if !channel_stepper_ready(channel) {
            next_event_clocks[idx] = None;
            return StepDispatchOutcome::Continue;
        }
        if HOMING_STATE.load(Relaxed) == 1 {
            // P1 has not finished homing preparation; re-check in 1 ms.
            next_event_clocks[idx] = Some(read_tim5().wrapping_add(CLOCK_FREQ / 1000));
            return StepDispatchOutcome::Continue;
        }
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };

        // SAFETY: ISR stepper state is only accessed from this P3 ISR for
        // this channel. P3 tasks cannot preempt each other on Cortex-M.
        let Some(stepper) = (unsafe { crate::state::stepper::isr_stepper(channel) }) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        if !stepper.has_active_segment() {
            next_event_clocks[idx] = None;
            return load_next_step_move(channel, next_event_clocks);
        }

        let result = stepper.step_event();
        let event = result.event();
        if emit_logical_step_event(channel, event).is_err() {
            fail_step_channel_deadline(channel, next_event_clocks);
            return StepDispatchOutcome::Fatal;
        }

        // STEP is low before mapper state, pulse counts, or the logical
        // execution become observable outside this P3 train.
        let now = embassy_stm32::pac::TIM5.cnt().read();
        crate::state::stepper::publish_logical_event(channel, stepper, event);
        let logical_delta = if event.direction { 1 } else { -1 };
        stepper_state.position.fetch_add(logical_delta, Relaxed);
        stepper_state.direction.store(event.direction, Relaxed);
        stepper_state
            .interval
            .store(stepper.current_interval(), Relaxed);
        crate::state::stepper_diag::record_executed_step(
            channel,
            event.direction,
            event.physical_pulses,
            now,
        );

        match result {
            StepEventResult::ScheduleNext { clock, .. } => {
                match stepper.check_deadline(clock, now, CLOCK_FREQ) {
                    Ok(compare) => {
                        let immediate = compare != clock;
                        next_event_clocks[idx] = Some(if immediate { now } else { compare });
                    }
                    Err(foci_klipper::stepper::MissedDeadline) => {
                        crate::state::stepper_diag::record_missed_deadline(channel);
                        record_stepper_event(
                            StepperEventReason::MissedDeadlineStep,
                            channel,
                            now,
                            UNKNOWN_QUEUE_LEN,
                            clock,
                            now,
                        );
                        #[cfg(feature = "trace")]
                        crate::usb_trace::note_missed_or_deferred_step();
                        STEPPER_DEADLINE_MISS_CHANNEL.store(channel, Relaxed);
                        fail_step_channel_deadline(channel, next_event_clocks);
                        return StepDispatchOutcome::Fatal;
                    }
                }
            }
            StepEventResult::NeedMove { .. } => {
                next_event_clocks[idx] = None;
                return load_next_step_move(channel, next_event_clocks);
            }
        }
        StepDispatchOutcome::Continue
    }

    fn refresh_step_channels(
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) -> StepDispatchOutcome {
        use core::sync::atomic::Ordering::Relaxed;

        for idx in 0..crate::state::stepper::STEPPER_CHANNEL_COUNT {
            let channel = idx as u8;
            if !channel_stepper_ready(channel) {
                next_event_clocks[idx] = None;
            } else if HOMING_STATE.load(Relaxed) == 1 {
                // P1 has not finished homing preparation; re-check in 1 ms.
                next_event_clocks[idx] = Some(read_tim5().wrapping_add(CLOCK_FREQ / 1000));
            } else if next_event_clocks[idx].is_none()
                && load_next_step_move(channel, next_event_clocks) == StepDispatchOutcome::Fatal
            {
                return StepDispatchOutcome::Fatal;
            }
        }
        StepDispatchOutcome::Continue
    }

    /// P2 software task: reconcile pending stepper stop and arm state.
    #[task(priority = 2)]
    async fn stepper_reconcile(_ctx: stepper_reconcile::Context) {
        loop {
            STEPPER_RECONCILE_WAKE.wait().await;
            loop {
                let pending = STEPPER_RECONCILE_PENDING.take_pending();
                if pending.is_empty() {
                    break;
                }
                with_trigger_domain(|td| {
                    let mut board = crate::firmware_ctx::OuroborosStepperBoard { tmc_state: None };
                    drain_stepper_reconciliation(td.state_mut(), &mut board, pending);
                });
            }
        }
    }

    impl StepperStopCheckBoard for crate::firmware_ctx::OuroborosStepperBoard {
        fn stop_check_index(&self, channel: u8) -> Option<usize> {
            Some(channel as usize)
        }

        fn clear_step_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            _channel: u8,
        ) {
            self.set_timer_active(state, stepper_idx, false);
        }
    }

    impl StepperArmCheckBoard for crate::firmware_ctx::OuroborosStepperBoard {
        fn arm_check_index(&self, channel: u8) -> Option<usize> {
            foci_firmware::state::tmc::tmc_channel(channel).map(|_| channel as usize)
        }

        fn motor_armed(&self, channel: u8) -> bool {
            foci_firmware::state::tmc::tmc_channel(channel)
                .map(|state| {
                    state
                        .motor_armed
                        .load(core::sync::atomic::Ordering::Relaxed)
                })
                .unwrap_or(false)
        }

        fn queue_has_moves(&self, channel: u8) -> bool {
            !crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].is_empty()
        }

        fn arm_check_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            let now = read_tim5();
            crate::state::stepper_diag::record_timer_activation(channel, now);
            arm_step_timer_if_earlier(now.wrapping_add(1));
            self.set_timer_active(state, stepper_idx, true);
        }
    }

    /// TIM5 CC1 step timer interrupt.
    ///
    /// Dequeues moves from SPSC consumer, toggles STEP GPIO, schedules
    /// next compare. Same priority as USB OTG and fault EXTI (P3).
    #[task(
        binds = TIM5,
        priority = 3,
        local = [
            next_event_clocks: [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT] =
                [None; crate::state::stepper::STEPPER_CHANNEL_COUNT],
        ],
    )]
    fn step_timer_irq(ctx: step_timer_irq::Context) {
        use core::{cell::Cell, sync::atomic::Ordering::Relaxed};

        struct Tim5PerfSample<'a> {
            irq_start: u32,
            dispatch_start: u32,
            dispatch_cycles: &'a Cell<u32>,
            event_count: &'a Cell<u32>,
            scheduler_cycles: &'a Cell<u32>,
            scheduler_event_count: &'a Cell<u32>,
            armed: bool,
        }

        impl Tim5PerfSample<'_> {
            fn finish_dispatch(&self) {
                if self.dispatch_cycles.get() == 0 {
                    // A real dispatch cannot complete in zero cycles; zero means unfinished.
                    self.dispatch_cycles.set(
                        cortex_m::peripheral::DWT::cycle_count().wrapping_sub(self.dispatch_start),
                    );
                }
            }
        }

        impl Drop for Tim5PerfSample<'_> {
            fn drop(&mut self) {
                if !self.armed {
                    return;
                }
                self.finish_dispatch();
                let irq_cycles =
                    cortex_m::peripheral::DWT::cycle_count().wrapping_sub(self.irq_start);
                let tim5_event_count = self
                    .event_count
                    .get()
                    .saturating_add(self.scheduler_event_count.get());
                crate::state::stepper_diag::record_tim5_irq(
                    irq_cycles,
                    self.dispatch_cycles.get(),
                    tim5_event_count,
                );
                crate::state::stepper_diag::record_scheduler_cycles(
                    self.scheduler_cycles.get(),
                    self.scheduler_event_count.get(),
                );
            }
        }

        let tim5 = embassy_stm32::pac::TIM5;
        let sr = tim5.sr().read();
        let dier = tim5.dier().read();
        let cc1_pending = sr.ccif(0) && dier.ccie(0);
        let cc2_pending = sr.ccif(1) && dier.ccie(1);
        if !cc1_pending && !cc2_pending {
            return;
        }

        let irq_start = cortex_m::peripheral::DWT::cycle_count();
        let entry_now = tim5.cnt().read();
        let mut entry_lateness = 0i32;
        if cc1_pending {
            let ccr = tim5.ccr(0).read();
            let delta = entry_now.wrapping_sub(ccr) as i32;
            if delta > entry_lateness {
                entry_lateness = delta;
            }
        }
        if cc2_pending {
            let ccr = tim5.ccr(1).read();
            let delta = entry_now.wrapping_sub(ccr) as i32;
            if delta > entry_lateness {
                entry_lateness = delta;
            }
        }
        if entry_lateness > 0 {
            crate::state::stepper_diag::record_tim5_entry_latency(entry_lateness as u32);
        }

        let dispatch_cycles = Cell::new(0);
        let event_count = Cell::new(0);
        let scheduler_cycles = Cell::new(0);
        let scheduler_event_count = Cell::new(0);
        let dispatch_start = cortex_m::peripheral::DWT::cycle_count();
        let perf_sample = Tim5PerfSample {
            irq_start,
            dispatch_start,
            dispatch_cycles: &dispatch_cycles,
            event_count: &event_count,
            scheduler_cycles: &scheduler_cycles,
            scheduler_event_count: &scheduler_event_count,
            armed: true,
        };

        // --- CC2: trigger scheduler (process before CC1) ---
        if cc2_pending {
            tim5.sr().modify(|w| w.set_ccif(1, false));
            let mut need_report = false;
            with_trigger_domain_site(crate::state::stepper_diag::PERF_SITE_TIM5, |td| {
                use foci_klipper::scheduler::SchedulerEventKind;
                let now = read_tim5();
                // Collect (kind, wake_clock) pairs before mutating scheduler
                let mut due_events: heapless::Vec<(SchedulerEventKind, u32), 13> =
                    heapless::Vec::new();
                loop {
                    let start = cortex_m::peripheral::DWT::cycle_count();
                    let slot = td.scheduler.pop_due_slot(now);
                    add_scheduler_cycles(&scheduler_cycles, start);
                    let Some(slot) = slot else {
                        break;
                    };
                    let event = slot.event();
                    let lateness = now.wrapping_sub(event.wake_clock) as i32;
                    if lateness > 0 {
                        crate::state::stepper_diag::record_scheduler_pop_lateness(lateness as u32);
                    }
                    scheduler_event_count.set(scheduler_event_count.get().saturating_add(1));
                    let _ = due_events.push((event.kind, event.wake_clock));
                }

                for &(kind, wake_clock) in &due_events {
                    match kind {
                        SchedulerEventKind::Stepper(_) => {}
                        SchedulerEventKind::EndstopSample(endstop_idx) => {
                            use foci_klipper::endstop::{EndstopPhase, EndstopSampleResult};
                            let pin_id = ENDSTOP_PIN_IDS[endstop_idx].load(Relaxed);
                            let read_pin = || -> bool {
                                use crate::ankyra::enumerations::Pins;
                                let stall_channel = match Pins::try_from(pin_id) {
                                    Ok(Pins::STALL0) => Some(0),
                                    Ok(Pins::STALL1) => Some(1),
                                    _ => None,
                                };
                                match stall_channel {
                                    Some(channel) => tmc_state(channel).stall_latched.load(Relaxed),
                                    None if pin_id != 0xFF => gpio_read_pin(pin_id),
                                    None => false,
                                }
                            };

                            let phase = td.endstops[endstop_idx].phase();
                            let result = match phase {
                                EndstopPhase::Scanning => {
                                    td.endstops[endstop_idx].scan(read_pin(), wake_clock)
                                }
                                EndstopPhase::Oversampling => {
                                    td.endstops[endstop_idx].oversample(read_pin(), wake_clock)
                                }
                            };

                            match result {
                                EndstopSampleResult::EnterOversamplingNow => {
                                    // Second GPIO read (C double-read)
                                    match td.endstops[endstop_idx]
                                        .oversample(read_pin(), wake_clock)
                                    {
                                        EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                            {
                                                let trigger_start =
                                                    cortex_m::peripheral::DWT::cycle_count();
                                                td.trigger(trsync_oid, reason, now);
                                                let trigger_cycles =
                                                    cortex_m::peripheral::DWT::cycle_count()
                                                        .wrapping_sub(trigger_start);
                                                crate::state::stepper_diag::record_critical_section(
                                                    crate::state::stepper_diag::PERF_SITE_TRIGGER_STOP,
                                                    trigger_cycles,
                                                );
                                            }
                                            need_report = true;
                                        }
                                        EndstopSampleResult::Reschedule { next_clock } => {
                                            let start = cortex_m::peripheral::DWT::cycle_count();
                                            let result = td.scheduler.try_schedule(
                                                foci_klipper::scheduler::SchedulerEvent {
                                                    wake_clock: next_clock,
                                                    kind: SchedulerEventKind::EndstopSample(
                                                        endstop_idx,
                                                    ),
                                                },
                                            );
                                            add_scheduler_cycles(&scheduler_cycles, start);
                                            record_schedule_result(result);
                                        }
                                        _ => {}
                                    }
                                }
                                EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                    {
                                        let trigger_start =
                                            cortex_m::peripheral::DWT::cycle_count();
                                        td.trigger(trsync_oid, reason, now);
                                        let trigger_cycles =
                                            cortex_m::peripheral::DWT::cycle_count()
                                                .wrapping_sub(trigger_start);
                                        crate::state::stepper_diag::record_critical_section(
                                            crate::state::stepper_diag::PERF_SITE_TRIGGER_STOP,
                                            trigger_cycles,
                                        );
                                    }
                                    need_report = true;
                                }
                                EndstopSampleResult::Reschedule { next_clock } => {
                                    let start = cortex_m::peripheral::DWT::cycle_count();
                                    let result = td.scheduler.try_schedule(
                                        foci_klipper::scheduler::SchedulerEvent {
                                            wake_clock: next_clock,
                                            kind: SchedulerEventKind::EndstopSample(endstop_idx),
                                        },
                                    );
                                    add_scheduler_cycles(&scheduler_cycles, start);
                                    record_schedule_result(result);
                                }
                                EndstopSampleResult::Inactive => {}
                            }
                        }
                        SchedulerEventKind::TrSyncExpire(trsync_idx) => {
                            // The scheduler event already confirms the deadline
                            // has passed. Call td.trigger() directly — do NOT call
                            // check_timeout() which internally triggers the
                            // TrSyncRuntime, causing td.trigger() to see
                            // AlreadyTriggered and skip stop signal dispatch.
                            let oid = td
                                .trsync_by_oid
                                .iter()
                                .find(|&(_, &idx)| idx == trsync_idx)
                                .map(|(&oid, _)| oid);
                            if let Some(oid) = oid {
                                let expire_reason = td.trsync[trsync_idx].expire_reason();
                                {
                                    let trigger_start = cortex_m::peripheral::DWT::cycle_count();
                                    td.trigger(oid, expire_reason, now);
                                    let trigger_cycles = cortex_m::peripheral::DWT::cycle_count()
                                        .wrapping_sub(trigger_start);
                                    crate::state::stepper_diag::record_critical_section(
                                        crate::state::stepper_diag::PERF_SITE_TRIGGER_STOP,
                                        trigger_cycles,
                                    );
                                }
                                need_report = true;
                            }
                        }
                        SchedulerEventKind::TrSyncReport(trsync_idx) => {
                            let oid = td
                                .trsync_by_oid
                                .iter()
                                .find(|&(_, &idx)| idx == trsync_idx)
                                .map(|(&oid, _)| oid);
                            if let Some(oid) = oid
                                && td.trsync[trsync_idx].can_trigger()
                            {
                                td.trsync[trsync_idx].set_liveness_report(oid, now);
                                need_report = true;
                                // Advance report_clock by report_ticks (stable cadence)
                                let report_ticks = td.trsync[trsync_idx].report_ticks();
                                if report_ticks > 0 {
                                    let next_report = td.trsync[trsync_idx]
                                        .report_clock()
                                        .wrapping_add(report_ticks);
                                    td.trsync[trsync_idx].set_report_clock(next_report);
                                    let start = cortex_m::peripheral::DWT::cycle_count();
                                    let result = td.scheduler.try_schedule(
                                        foci_klipper::scheduler::SchedulerEvent {
                                            wake_clock: next_report,
                                            kind: SchedulerEventKind::TrSyncReport(trsync_idx),
                                        },
                                    );
                                    add_scheduler_cycles(&scheduler_cycles, start);
                                    record_schedule_result(result);
                                }
                                // After trigger, no more liveness reports — don't reschedule
                            }
                        }
                    }
                }
                crate::trigger_domain::reprogram_cc2_measured(td.state_mut(), &scheduler_cycles);
            });
            if need_report {
                REPORT_PENDING.store(true, Relaxed);
                report_task::spawn().ok();
            }
        }

        // --- CC1: step timer ---
        // Skip CC1 if the flag is not set, or if CC2 trigger processing
        // disabled CC1 because no channel remains active.
        if !tim5.sr().read().ccif(0) || !tim5.dier().read().ccie(0) {
            perf_sample.finish_dispatch();
            return;
        }
        tim5.sr().modify(|w| w.set_ccif(0, false));

        let next_event_clocks = ctx.local.next_event_clocks;
        if FAULT_ACTIVE.load(Relaxed) {
            perf_sample.finish_dispatch();
            *next_event_clocks = [None; crate::state::stepper::STEPPER_CHANNEL_COUNT];
            tim5.dier().modify(|w| w.set_ccie(0, false));
            return;
        }

        let mut budget = foci_firmware::stepper_runtime::StepIrqBudget::new();
        let outcome = run_step_dispatch(|| {
            if refresh_step_channels(next_event_clocks) == StepDispatchOutcome::Fatal {
                return StepDispatchOutcome::Fatal;
            }
            let Some(due) = foci_firmware::stepper_runtime::earliest_due(next_event_clocks) else {
                return StepDispatchOutcome::Complete;
            };
            let now = tim5.cnt().read();
            if !step_clock_is_due(due.clock, now) {
                return StepDispatchOutcome::Complete;
            }
            if !budget.take() {
                #[cfg(feature = "trace")]
                crate::usb_trace::note_missed_or_deferred_step();
                crate::state::stepper_diag::record_tim5_defer();
                return StepDispatchOutcome::Complete;
            }
            #[cfg(feature = "trace")]
            crate::usb_trace::record_step_timing(due.clock, now);
            event_count.set(event_count.get().saturating_add(1));
            process_due_step_event(due.channel, next_event_clocks)
        });
        perf_sample.finish_dispatch();
        if outcome == StepDispatchOutcome::Fatal {
            abort_step_dispatch(next_event_clocks);
            return;
        }
        program_next_step_compare(next_event_clocks);
    }

    /// USB OTG HS interrupt handler. Masks interrupt sources and wakes usb_task.
    ///
    /// Priority 3: must be above P2 protocol tasks so the ISR always runs
    /// immediately. The handler only masks and signals — no FIFO access.
    #[task(binds = OTG_HS, priority = 3)]
    fn otg_hs_irq(_ctx: otg_hs_irq::Context) {
        crate::interrupts::otg_fs_irq();
    }

    /// TMC4671 STATUS pin interrupt (PE2, EXTI line 2).
    ///
    /// Fires when any STATUS_MASK-enabled flag is asserted (`adc_i_clipped`).
    /// Signals immediate motor disable; tmc_control reads
    /// STATUS_FLAGS on next tick to log the cause.
    #[task(binds = EXTI2, priority = 3)]
    fn tmc_status_irq(_ctx: tmc_status_irq::Context) {
        crate::interrupts::tmc_status_irq(&FAULT_ACTIVE, &TMC_DISABLE_SIGNAL);
    }

    /// TMC4671 channel-1 STATUS pin interrupt (PA2, TIM15 CH1 input capture).
    ///
    /// PA2 cannot share EXTI line 2 with PE2, so channel 1's STATUS line is
    /// routed to a TIM15 input-capture interrupt instead. Same disable path
    /// as `tmc_status_irq`.
    #[task(binds = TIM15, priority = 3)]
    fn tmc_status1_irq(_ctx: tmc_status1_irq::Context) {
        crate::interrupts::tmc_status1_irq(&FAULT_ACTIVE, &TMC_DISABLE_SIGNAL);
    }

    /// Single USB task replacing usb_pump, usb_receive, usb_send, usb_control.
    ///
    /// Handles EP0 setup, bulk RX, protocol dispatch, and bulk TX in one pass.
    /// No scheduling gap between receiving commands and sending acks.
    /// Katapult 1200-baud DTR detection is handled by klipper-usb internally.
    #[task(priority = 2, shared = [app_state])]
    async fn usb_task(mut ctx: usb_task::Context) {
        let mut backend = crate::usb_task::OuroborosUsbBackend::new(
            |f| ctx.shared.app_state.lock(f),
            &REPORT_PENDING,
            &USB_PUMP_EMITTER,
            &USB_PUMP_COUNTERS,
        );
        loop {
            foci_firmware::usb_pump::run_once(&mut backend, &USB_PUMP_COUNTERS).await;
        }
    }

    /// Send pending trsync_state reports outside the TriggerDomain lock.
    ///
    /// Spawned by the TIM5 CC2 ISR when any trsync has a pending report.
    /// Drains all pending snapshots in one brief critical section, then
    /// sends replies at P2 without holding the lock.
    #[task(priority = 2)]
    async fn report_task(_ctx: report_task::Context) {
        crate::heartbeat::emit_trigger_reports(
            |snaps| {
                with_trigger_domain(|td| {
                    for ts in td.trsync.iter_mut() {
                        if let Some(snap) = ts.take_report() {
                            let _ = snaps.push(snap);
                        }
                    }
                });
            },
            &REPORT_PENDING,
        );
    }

    /// Send pending stepper diagnostic events outside ISR/P1 contexts.
    #[task(priority = 2)]
    async fn stepper_diag_task(_ctx: stepper_diag_task::Context) {
        crate::heartbeat::emit_stepper_diagnostics(&STEPPER_DIAG_PENDING);
    }

    /// Periodic stats heartbeat sent to Klipper.
    ///
    /// Klipper requires regular stats messages to confirm the MCU is alive.
    /// Sends `stats count=%u sum=%u sumsq=%u` every 5 seconds with basic
    /// CPU usage metrics.
    #[task(priority = 1)]
    async fn stats(_ctx: stats::Context) {
        let mut accumulator = crate::heartbeat::StatsAccumulator::new(read_tim5());
        loop {
            Mono::delay(5000.millis()).await;
            crate::heartbeat::stats_tick(
                &mut accumulator,
                read_tim5(),
                &STATS_SEND_TIME,
                &STATS_SEND_TIME_HIGH,
            );
        }
    }

    /// Service the independent watchdog above sustained priority-1 work.
    #[task(priority = 2, local = [wdg])]
    async fn watchdog(ctx: watchdog::Context) {
        loop {
            Mono::delay(crate::heartbeat::WATCHDOG_INTERVAL_MS.millis()).await;
            crate::heartbeat::watchdog_tick(ctx.local.wdg);
        }
    }
}

foci_firmware::foci_ankyra_config! {
    transport = crate::transport::TRANSPORT_OUTPUT: crate::transport::BufferTransportOutput,
    context = &'ctx mut crate::firmware_ctx::FirmwareCtx<'ctx>,
    providers = [
        foci_firmware::ankyra::provider::FOCI_FIRMWARE_PROVIDER,
        crate::ankyra::provider::BOARD_PROVIDER,
    ],
    board_static_strings = [
        "TMC4671 PI representation failed at boot",
        "TMC4671: STATUS_FLAGS read failed",
    ],
    app = "foci",
    version = env!("CARGO_PKG_VERSION"),
    build_versions = "",
    license = "MIT OR Apache-2.0",
}
