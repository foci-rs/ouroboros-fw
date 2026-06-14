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
mod platform_impl;
mod state;
mod stepper_pins;
mod tmc_control;
mod tmc_control_glue;
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
    use crate::ankyra::constants::CLOCK_FREQ;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::spi::Spi;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals};
    #[cfg(feature = "trace")]
    use foci_firmware::autotune::PhaseId;
    #[cfg(feature = "trace")]
    use foci_firmware::commissioning::types::EngineKind;
    use foci_klipper::state::State;
    use foci_tmc4671::spi::Tmc4671Spi;
    use rtic_monotonics::fugit::ExtU64;
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;

    use crate::helpers::lifecycle::*;
    use crate::helpers::stepper::*;
    use crate::helpers::trsync::*;
    use crate::state::tmc::*;
    use crate::tmc_control::board::{
        TmcEnablePins, TmcRuntimes, channel_may_be_energized, channel_needs_safe_state,
        commissioning_request_blocked, enable_pin, new_tmc_runtimes, ouroboros_backend,
        refresh_velocity_scale, stepper_state, tmc_state,
    };
    use crate::trigger_domain::{
        TRIGGER_DOMAIN, TriggerDomain, with_trigger_domain, with_trigger_domain_site,
    };
    use foci_firmware::context::stepper::{
        StepperArmCheckBoard, StepperStopCheckBoard, StepperTimerStateBoard,
    };
    use foci_firmware::state::lifecycle::*;
    use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TMC_QUEUE_FLUSH};
    use foci_firmware::state::trsync::*;
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
                | BoardAction::Reply(_)
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
    // TRIGGER_DOMAIN, with_trigger_domain(), and stop_physical_stepper_no_lock()
    // live in crate::trigger_domain and are re-imported below.

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

    #[local]
    struct Local {
        tmc_mux: crate::tmc_mux::TmcSpiMux,
        tmc_enable: TmcEnablePins,
        tmc_rx: Receiver<'static, TmcRequest, TMC_CHANNEL_SIZE>,
        wdg: IndependentWatchdog<'static, peripherals::IWDG1>,
    }

    #[init(local = [
        tmc_channel: rtic_sync::channel::Channel<TmcRequest, TMC_CHANNEL_SIZE> =
            rtic_sync::channel::Channel::new(),
        uid_str: [u8; 24] = [0; 24],
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("FOCI Ouroboros starting");

        let p = embassy_stm32::init(crate::board_init::rcc_config());
        defmt::info!("SYSCLK: 520 MHz, USB: 48 MHz");

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

        crate::board_init::init_tmc_status_pins(p.PE2, p.PA2);

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
        tmc_control::spawn().ok();
        usb_task::spawn().ok();
        stats::spawn().ok();

        defmt::info!("FOCI Ouroboros initialized");

        (
            Shared {
                app_state: State::new(),
            },
            Local {
                tmc_mux,
                tmc_enable,
                tmc_rx,
                wdg,
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
    #[task(priority = 1, shared = [app_state], local = [tmc_mux, tmc_enable, tmc_rx])]
    async fn tmc_control(mut ctx: tmc_control::Context) {
        let enable_pins = ctx.local.tmc_enable;
        let rx = ctx.local.tmc_rx;

        // NOTE: auto-init disabled - motor current regulation not yet safe
        // tmc_init_open_loop(tmc, enable_pin(enable_pins, channel)).await;

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
            let mut tmc = Tmc4671Spi::new(device)
                .with_retry_policy(foci_tmc4671::spi::RetryPolicy::production());

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
            {
                use foci_tmc4671::registers::*;
                let _ = tmc.write_register(ADC_VM_LIMITS, 0xFFFF_FFFF).await;
            }

            // Configure STATUS_MASK: route ADC current clipping to the channel-local
            // STATUS pin. PE2/channel 0 is routed to EXTI2 below. PA2/channel 1
            // shares EXTI line 2 with PE2, and SYSCFG can select only one port for
            // a given EXTI line, so channel 1 clipping is polled every tick
            // while the channel may be energized.
            {
                use foci_tmc4671::registers::*;
                let mask = STATUS_CRITICAL_FAULT_MASK;
                let _ = tmc.write_register(STATUS_MASK, mask).await;
                // Clear pending STATUS flags from power-on before enabling EXTI/polling.
                let _ = tmc.read_register(STATUS_FLAGS).await;
                let _ = tmc.write_register(STATUS_FLAGS, 0).await;
                defmt::info!("TMC4671 channel {} STATUS_MASK: {:#010x}", channel, mask);
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
        #[cfg(feature = "trace")]
        let mut commission_trace_phase: [Option<(EngineKind, PhaseId)>;
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
            use embassy_futures::select::{Either, select};

            macro_rules! quiesce_and_report_tmc_fault {
                () => {{
                    quiesce_all_channels(&mut runtimes, enable_pins);
                    #[cfg(feature = "trace")]
                    commission_trace_phase.fill(None);
                    let mut reported_shutdown = false;
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
                        let mut report_tmc = Tmc4671Spi::new(device);
                        let report_flags = report_tmc
                            .read_register(foci_tmc4671::registers::STATUS_FLAGS)
                            .await
                            .unwrap_or(0);
                        defmt::error!(
                            "TMC4671 channel {} emergency disable: STATUS={:#010x}",
                            report_channel,
                            report_flags
                        );
                        record_stepper_event(
                            StepperEventReason::TmcDisableSignal,
                            report_channel,
                            read_tim5(),
                            UNKNOWN_QUEUE_LEN,
                            report_flags,
                            0,
                        );
                        if report_flags & STATUS_ADC_I_CLIPPED != 0 {
                            reported_shutdown = true;
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
                    if !reported_shutdown {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "TMC4671 fault",
                                    read_tim5(),
                                );
                            }
                        });
                    }
                    FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
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
                        let mut tmc = Tmc4671Spi::new(device);
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

            if TMC_DISABLE_SIGNAL.try_take().is_some() {
                quiesce_and_report_tmc_fault!();
                continue;
            }

            // On shutdown, perform one-shot motor safe-state then idle until
            // shutdown clears. This catches USB disconnect / host crash where
            // no further commands arrive but the TMC4671 remains energized.
            if SHUTDOWN_ACTIVE.load(core::sync::atomic::Ordering::Relaxed) {
                if !shutdown_handled {
                    quiesce_all_channels(&mut runtimes, enable_pins);
                    #[cfg(feature = "trace")]
                    commission_trace_phase.fill(None);
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
                        let mut tmc = Tmc4671Spi::new(device);
                        let runtime = &mut runtimes[channel as usize];
                        let mut backend =
                            ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                        let mut state =
                            runtime.dispatch_state(tmc_state(channel), stepper_state(channel));
                        // disarm_motor clears tmc_state().motor_armed, motor_enabled, calibrated,
                        // and runs restore_safe_state_via_backend (best-effort SPI writes).
                        foci_firmware::tmc_control::lifecycle::disarm_motor(
                            &mut backend,
                            &mut state,
                        )
                        .await;
                        tmc_state(channel)
                            .commission_in_flight
                            .store(false, core::sync::atomic::Ordering::Relaxed);
                    }
                    shutdown_handled = true;
                }
                Mono::delay(1.millis()).await;
                continue;
            }
            shutdown_handled = false;

            // Drain stale requests after config_reset
            if TMC_QUEUE_FLUSH.swap(false, core::sync::atomic::Ordering::Relaxed) {
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
                        let mut tmc = Tmc4671Spi::new(device);
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
                    tmc_state(channel)
                        .commission_in_flight
                        .store(false, core::sync::atomic::Ordering::Relaxed);
                    // Clear commissioned outer configuration so the next closed-
                    // loop entry uses defaults, not stale commissioned values.
                    use core::sync::atomic::Ordering::Relaxed as R;
                    tmc_state(channel).velocity_limit_cfg.store(500_000, R);
                    tmc_state(channel).position_p_value.store(640, R);
                    tmc_state(channel).position_i_value.store(0, R);
                    tmc_state(channel).velocity_p_value.store(1152, R);
                    tmc_state(channel).velocity_i_value.store(0, R);
                    tmc_state(channel).velocity_filter_hz.store(0, R);
                    tmc_state(channel).position_filter_hz.store(0, R);
                    tmc_state(channel).flux_filter_hz.store(0, R);
                    tmc_state(channel).torque_filter_hz.store(0, R);
                    stepper_state(channel)
                        .position
                        .store(0, core::sync::atomic::Ordering::Relaxed);
                }
                FAULT_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
                defmt::info!("TMC request queue flushed after config_reset");
            }

            // Tick the commissioning engine if one is active.
            // Each step() does at most 1ms of work, then returns.
            for channel in 0..crate::board_config::TMC_CHANNEL_COUNT {
                if !runtimes[channel as usize].is_commission_active() {
                    continue;
                }

                if poll_channel1_critical_status!() {
                    quiesce_and_report_tmc_fault!();
                    continue 'control_loop;
                }
                if TMC_DISABLE_SIGNAL.try_take().is_some() {
                    quiesce_and_report_tmc_fault!();
                    continue 'control_loop;
                }

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
                let mut tmc = Tmc4671Spi::new(device);
                #[cfg(feature = "trace")]
                let engine_tick_deadline =
                    Mono::now() + fugit::Duration::<u64, 1, 1_000_000>::millis(1);

                // Sequential borrows: engine.step() consumes the backend
                // first, then dispatch re-borrows it for post-processing.
                // Two scoped constructions keep the borrows non-overlapping
                // and preserve the emergency-disable check between them.
                let result = {
                    let runtime = &mut runtimes[channel as usize];
                    let mut backend =
                        ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                    runtime.step_commissioning(&mut backend).await
                };
                let Some(result) = result else {
                    #[cfg(feature = "trace")]
                    {
                        commission_trace_phase[channel as usize] = None;
                    }
                    let _ = tmc;
                    continue;
                };
                if TMC_DISABLE_SIGNAL.try_take().is_some() {
                    let _ = tmc;
                    quiesce_and_report_tmc_fault!();
                    continue 'control_loop;
                }

                let action = {
                    let runtime = &mut runtimes[channel as usize];
                    let mut backend =
                        ouroboros_backend(channel, &mut tmc, enable_pin(enable_pins, channel));
                    runtime
                        .drive_commissioning_result(
                            result,
                            &mut backend,
                            tmc_state(channel),
                            stepper_state(channel),
                            &tmc_control_config,
                        )
                        .await
                };
                let _ = tmc;
                if TMC_DISABLE_SIGNAL.try_take().is_some() {
                    quiesce_and_report_tmc_fault!();
                    continue 'control_loop;
                }
                if poll_channel1_critical_status!() {
                    quiesce_and_report_tmc_fault!();
                    continue 'control_loop;
                }
                if TMC_DISABLE_SIGNAL.try_take().is_some() {
                    quiesce_and_report_tmc_fault!();
                    continue 'control_loop;
                }

                use foci_firmware::tmc_control::CommissionTickAction;
                let Some(action) = action else {
                    #[cfg(feature = "trace")]
                    {
                        commission_trace_phase[channel as usize] = None;
                    }
                    continue;
                };
                match action {
                    CommissionTickAction::Continue => {
                        if TMC_DISABLE_SIGNAL.try_take().is_some() {
                            quiesce_and_report_tmc_fault!();
                            continue 'control_loop;
                        }
                        if poll_channel1_critical_status!() {
                            quiesce_and_report_tmc_fault!();
                            continue 'control_loop;
                        }
                        if TMC_DISABLE_SIGNAL.try_take().is_some() {
                            quiesce_and_report_tmc_fault!();
                            continue 'control_loop;
                        }
                        #[cfg(feature = "trace")]
                        if let Some((engine_kind, phase_id)) =
                            commission_trace_phase[channel as usize]
                            && foci_firmware::trace::commissioning::commission_trace_phase_samples_closed_loop_entry(
                                engine_kind,
                                phase_id,
                            ) {
                            let trace_tick =
                                runtimes[channel as usize].advance_trace_sample_tick();
                            let refs =
                                TmcControlStateRefs::new(tmc_state(channel), stepper_state(channel));
                            let motor_enabled = runtimes[channel as usize].is_motor_enabled();
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
                                continue 'control_loop;
                            };
                            let mut tmc = Tmc4671Spi::new(device);
                            crate::usb_trace::sample_commission_closed_loop_entry(
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
                        Mono::delay(1.millis()).await;
                        continue 'control_loop;
                    }
                    CommissionTickAction::PhaseChanged {
                        oid,
                        phase_id,
                        engine_kind,
                    } => {
                        #[cfg(feature = "trace")]
                        {
                            commission_trace_phase[channel as usize] =
                                Some((engine_kind, phase_id));
                        }
                        #[cfg(not(feature = "trace"))]
                        let _ = engine_kind;
                        emit_phase_reply(oid, phase_id);
                        if TMC_DISABLE_SIGNAL.try_take().is_some() {
                            quiesce_and_report_tmc_fault!();
                            continue 'control_loop;
                        }
                        Mono::delay(1.millis()).await;
                        continue 'control_loop;
                    }
                    CommissionTickAction::Terminal { reply, actions } => {
                        #[cfg(feature = "trace")]
                        {
                            commission_trace_phase[channel as usize] = None;
                        }
                        {
                            let runtime = &mut runtimes[channel as usize];
                            let mut state =
                                runtime.dispatch_state(tmc_state(channel), stepper_state(channel));
                            apply_stepper_post_action(actions.stepper, &mut state, channel);
                        }
                        emit_terminal_reply(reply);
                    }
                }
                // For Terminal cases, fall through to normal channel-receive processing.
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
                    let mut tmc = Tmc4671Spi::new(device);
                    let actions = runtimes[channel as usize].classify_request(req.clone());
                    let disposition = crate::tmc_control_glue::apply_pre_request_actions(
                        channel,
                        actions,
                        |reason| {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    crate::tmc_control_glue::emit_shutdown(reason);
                                }
                            });
                        },
                    );

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
                        disposition,
                        crate::tmc_control::request::TmcControlRequestContext {
                            channel,
                            commissioning_blocked,
                            runtimes: &mut runtimes,
                            tmc: &mut tmc,
                            enable_pins,
                            commission_trace_engine_seq: &mut commission_trace_engine_seq,
                        },
                    )
                    .await;

                    if let Some(reason) = outcome.shutdown {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                crate::tmc_control_glue::emit_shutdown(reason);
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
                // 1ms timeout — write position target via SPI
                Either::Second(()) => {}
            }

            // Check emergency disable (non-blocking). Triggered by EXTI
            // STATUS interrupt or enter_shutdown. Disables motor and reports
            // the fault to Klipper so the host knows the MCU is down.
            if TMC_DISABLE_SIGNAL.try_take().is_some() {
                quiesce_and_report_tmc_fault!();
                continue;
            }

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
                let mut tmc = Tmc4671Spi::new(device);
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
                    .await;
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

                runtime.apply_pid_error_settle_tick(&mut tmc, refs).await;

                if runtime.is_motor_enabled() {
                    let homing_result = runtime
                        .apply_homing_channel_tick(&mut tmc, refs, homing)
                        .await;
                    all_channels_ready_for_homing_advance &=
                        matches!(homing_result, HomingMaintenanceResult::ReadyToAdvance);
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
            if slow_control_tick {
                crate::usb_trace::request_spi_duration_event_if_ready();
                crate::usb_trace::drain_pending_events();
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

    fn load_next_step_move(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        use core::sync::atomic::Ordering::Relaxed;

        let idx = channel as usize;
        if !channel_stepper_ready(channel) {
            next_event_clocks[idx] = None;
            return;
        }
        let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return;
        };
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return;
        };

        // SAFETY: ISR stepper state is only accessed from this P3 ISR for
        // this channel. P3 tasks cannot preempt each other on Cortex-M.
        let Some(stepper) = (unsafe { crate::state::stepper::isr_stepper(channel) }) else {
            clear_step_channel(channel, next_event_clocks);
            return;
        };
        if stepper.has_active_segment() {
            if next_event_clocks[idx].is_none() {
                next_event_clocks[idx] = Some(stepper.next_step_clock());
            }
            return;
        }

        // SAFETY: P3 is the sole consumer of this channel's queue. P3 ISRs
        // cannot preempt each other on Cortex-M.
        let next_move =
            unsafe { crate::state::stepper::step_queue(channel).and_then(|queue| queue.dequeue()) };
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
            return;
        };
        let loaded_steps = u32::from(move_.count);

        let dir_changed = stepper.load_move(move_);
        stepper_state
            .interval
            .store(stepper.current_interval(), Relaxed);
        if crate::stepper_pins::set_dir(channel, stepper.dir_state()).is_err() {
            fail_step_channel_deadline(channel, next_event_clocks);
            return;
        }
        stepper_state.direction.store(stepper.dir_state(), Relaxed);
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
            Err(_) => {
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
                fail_step_channel_deadline(channel, next_event_clocks);
            }
        }
    }

    fn process_due_step_event(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::stepper::{StepEventResult, StepPhase};

        let idx = channel as usize;
        if !channel_stepper_ready(channel) {
            next_event_clocks[idx] = None;
            return;
        }
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return;
        };

        // SAFETY: ISR stepper state is only accessed from this P3 ISR for
        // this channel. P3 tasks cannot preempt each other on Cortex-M.
        let Some(stepper) = (unsafe { crate::state::stepper::isr_stepper(channel) }) else {
            clear_step_channel(channel, next_event_clocks);
            return;
        };
        if !stepper.has_active_segment() {
            next_event_clocks[idx] = None;
            load_next_step_move(channel, next_event_clocks);
            return;
        }

        if crate::stepper_pins::toggle_step(channel).is_err() {
            fail_step_channel_deadline(channel, next_event_clocks);
            return;
        }

        match stepper.step_event() {
            StepEventResult::ScheduleNext { clock, phase } => {
                let now = embassy_stm32::pac::TIM5.cnt().read();
                let is_step = matches!(phase, StepPhase::Step);
                if is_step {
                    let dir = if stepper.dir_state() { 1i32 } else { -1i32 };
                    stepper_state.position.fetch_add(dir, Relaxed);
                    crate::state::stepper_diag::record_executed_step(
                        channel,
                        stepper.dir_state(),
                        now,
                    );
                }
                stepper_state
                    .interval
                    .store(stepper.current_interval(), Relaxed);
                match stepper.check_deadline(clock, now, CLOCK_FREQ) {
                    Ok(compare) => {
                        let immediate = compare != clock;
                        next_event_clocks[idx] = Some(if immediate { now } else { compare });
                    }
                    Err(_) => {
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
                        fail_step_channel_deadline(channel, next_event_clocks);
                    }
                }
            }
            StepEventResult::NeedMove => {
                next_event_clocks[idx] = None;
                load_next_step_move(channel, next_event_clocks);
            }
        }
    }

    fn refresh_step_channels(
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        for idx in 0..crate::state::stepper::STEPPER_CHANNEL_COUNT {
            let channel = idx as u8;
            if !channel_stepper_ready(channel) {
                next_event_clocks[idx] = None;
            } else if next_event_clocks[idx].is_none() {
                load_next_step_move(channel, next_event_clocks);
            }
        }
    }

    /// P2 software task: clear TriggerDomain timer_active after P1 stop.
    ///
    /// Spawned by `stepper_stop_from_p1()`. P1 performs the physical
    /// quiesce (CC1 disable, queue drain) immediately, then this task
    /// updates the authoritative flag so `reset_step_clock` and
    /// `arm_check` see the correct state.
    #[task(priority = 2)]
    async fn stop_check(_ctx: stop_check::Context, channel: u8) {
        with_trigger_domain(|td| {
            let mut board = crate::firmware_ctx::OuroborosStepperBoard { tmc_state: None };
            foci_firmware::context::stepper::stop_check(td.state_mut(), &mut board, channel);
        });
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

    /// P2 software task: arm TIM5 CC1 if motor is ready and queue has moves.
    ///
    /// Spawned by P1 after calibration/enable completion. P1 must never
    /// enter TriggerDomain — this task crosses the boundary at P2.
    #[task(priority = 2)]
    async fn arm_check(_ctx: arm_check::Context, channel: u8) {
        with_trigger_domain(|td| {
            let mut board = crate::firmware_ctx::OuroborosStepperBoard { tmc_state: None };
            foci_firmware::context::stepper::arm_check(td.state_mut(), &mut board, channel);
        });
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
            // SAFETY: P2 is the sole producer while the timer is inactive.
            unsafe {
                crate::state::stepper::step_queue(channel)
                    .map(|queue| !queue.is_empty())
                    .unwrap_or(false)
            }
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
                                if pin_id != 0xFF {
                                    gpio_read_pin(pin_id)
                                } else {
                                    false
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
                                            td.trigger(trsync_oid, reason, now);
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
                                    td.trigger(trsync_oid, reason, now);
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
                                td.trigger(oid, expire_reason, now);
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
        loop {
            refresh_step_channels(next_event_clocks);
            let Some(due) = foci_firmware::stepper_runtime::earliest_due(next_event_clocks) else {
                perf_sample.finish_dispatch();
                program_next_step_compare(next_event_clocks);
                return;
            };
            let now = tim5.cnt().read();
            if !step_clock_is_due(due.clock, now) {
                perf_sample.finish_dispatch();
                program_next_step_compare(next_event_clocks);
                return;
            }
            if !budget.take() {
                #[cfg(feature = "trace")]
                crate::usb_trace::note_missed_or_deferred_step();
                crate::state::stepper_diag::record_tim5_defer();
                perf_sample.finish_dispatch();
                program_next_step_compare(next_event_clocks);
                return;
            }
            #[cfg(feature = "trace")]
            crate::usb_trace::record_step_timing(due.clock, now);
            event_count.set(event_count.get().saturating_add(1));
            process_due_step_event(due.channel, next_event_clocks);
        }
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

    /// Single USB task replacing usb_pump, usb_receive, usb_send, usb_control.
    ///
    /// Handles EP0 setup, bulk RX, protocol dispatch, and bulk TX in one pass.
    /// No scheduling gap between receiving commands and sending acks.
    /// Katapult 1200-baud DTR detection is handled by klipper-usb internally.
    #[task(priority = 2, shared = [app_state])]
    async fn usb_task(mut ctx: usb_task::Context) {
        loop {
            crate::usb_task::pump(|f| ctx.shared.app_state.lock(f), &REPORT_PENDING).await;
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
        crate::heartbeat::emit_stepper_diagnostics();
    }

    /// Periodic stats heartbeat sent to Klipper.
    ///
    /// Klipper requires regular stats messages to confirm the MCU is alive.
    /// Sends `stats count=%u sum=%u sumsq=%u` every 5 seconds with basic
    /// CPU usage metrics. Also kicks the IWDG watchdog.
    #[task(priority = 1, local = [wdg])]
    async fn stats(ctx: stats::Context) {
        let mut accumulator = crate::heartbeat::StatsAccumulator::new(read_tim5());
        loop {
            Mono::delay(5000.millis()).await;
            crate::heartbeat::stats_tick(
                &mut accumulator,
                read_tim5(),
                &STATS_SEND_TIME,
                &STATS_SEND_TIME_HIGH,
                ctx.local.wdg,
            );
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
    board_static_strings = [],
    app = "foci",
    version = env!("CARGO_PKG_VERSION"),
    build_versions = "",
    license = "MIT OR Apache-2.0",
}
