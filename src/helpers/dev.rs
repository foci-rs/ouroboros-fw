//! Dev-only helpers: open-loop init for raw register prodding and a
//! manual `do_test_spin` for sweeping `PHI_E_EXT`. Gated behind the
//! `dev` feature so production builds never link them.

#![cfg(feature = "dev")]

use embassy_stm32::gpio::Output;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::registers::*;
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;

/// Sweep PHI_E_EXT through N full mechanical revolutions.
///
/// With 50 pole pairs, one mechanical revolution = 50 electrical
/// cycles. Each electrical cycle sweeps PHI_E_EXT from 0 to 65535.
/// Uses whatever voltage/mode settings were configured by tmc_init.
pub(crate) async fn do_test_spin(tmc: &mut Tmc4671Spi<TmcSpiDevice<'_>>, revolutions: u8) {
    let total_steps = revolutions as u32 * 50 * 256; // 256 microsteps per e-cycle
    defmt::info!(
        "Spinning {} revolutions ({} steps)",
        revolutions,
        total_steps
    );

    for step in 0..total_steps {
        let angle = (step * 256) % 65536;
        let _ = tmc.write_register(PHI_E_EXT, angle).await;
        Mono::delay(1.millis()).await;
    }

    // Return to angle 0
    let _ = tmc.write_register(PHI_E_EXT, 0).await;
    defmt::info!("Spin complete");
}

/// Configure TMC4671 for open-loop stepper operation with current regulation.
///
/// Performs hardware init, ADC offset calibration, and configures torque
/// mode with PHI_E_EXT for open-loop angle control. Self-contained dev
/// helper; production ADC calibration uses the commissioning engine's
/// cooperative AdcCalibrate phase.
///
/// Register values verified against TMC4671-LA datasheet rev 2.08.
pub(crate) async fn tmc_enter_openloop(
    tmc: &mut Tmc4671Spi<TmcSpiDevice<'_>>,
    enable_pin: &mut Output<'static>,
) {
    // --- ADC calibration (inlined from former tmc_adc_calibrate) ---

    // 1. SAFETY FIRST: stop motor, disable outputs, set voltage limit
    enable_pin.set_low();
    let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0).await;
    let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await;
    let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = tmc.write_register(UQ_UD_EXT, 0).await;

    // 2. PWM configuration (chopping disabled for ADC calibration)
    let _ = tmc.write_register(PWM_POLARITIES, 0).await;
    let _ = tmc
        .write_register(PWM_MAXCNT, TMC_CONFIG.pwm_maxcnt as u32)
        .await;
    let _ = tmc
        .write_register(PWM_BBM_H_BBM_L, TMC_CONFIG.bbm_register())
        .await;
    if tmc.write_register(PWM_SV_CHOP, 0).await.is_err() {
        defmt::error!("open-loop init: PWM_SV_CHOP write failed");
        return;
    }

    // 3. ADC configuration
    let _ = tmc
        .write_register(DSADC_MDEC_B_MDEC_A, TMC_CONFIG.dsadc_mdec_register())
        .await;
    let _ = tmc
        .write_register(ADC_I_SELECT, TMC_CONFIG.adc_i_select)
        .await;

    // 4. Motor type: 2-phase stepper (type 2), 50 pole pairs
    let _ = tmc
        .write_register(MOTOR_TYPE_N_POLE_PAIRS, (2 << 16) | 50)
        .await;

    // 4b. Velocity meter: PWM-synchronized for advanced PI stability
    let _ = tmc.write_register(VELOCITY_SELECTION, (1 << 8) | 0).await;

    // 5. ADC offset calibration (PWM_CHOP=0, no current flows)
    let _ = tmc.write_register(ADC_I0_SCALE_OFFSET, 256 << 16).await;
    let _ = tmc.write_register(ADC_I1_SCALE_OFFSET, 256 << 16).await;
    enable_pin.set_high();
    Mono::delay(250_u64.millis()).await;

    let mut sum_i0: i32 = 0;
    let mut sum_i1: i32 = 0;
    const N_SAMPLES: i32 = 128;
    for _ in 0..N_SAMPLES {
        if let Ok(raw) = tmc.read_register(ADC_IWY_IUX).await {
            sum_i0 += (raw & 0xFFFF) as i16 as i32;
            sum_i1 += ((raw >> 16) & 0xFFFF) as i16 as i32;
        }
        Mono::delay(1_u64.millis()).await;
    }
    let offset_i0 = (sum_i0 / N_SAMPLES) as i16 as u16;
    let offset_i1 = (sum_i1 / N_SAMPLES) as i16 as u16;

    if !(30000..=36000).contains(&offset_i0) || !(30000..=36000).contains(&offset_i1) {
        defmt::error!(
            "ADC offset out of range: I0={}, I1={} (expected 30000..=36000)",
            offset_i0,
            offset_i1
        );
        return;
    }

    let _ = tmc
        .write_register(ADC_I0_SCALE_OFFSET, (256 << 16) | offset_i0 as u32)
        .await;
    let _ = tmc
        .write_register(ADC_I1_SCALE_OFFSET, (256 << 16) | offset_i1 as u32)
        .await;
    defmt::info!("ADC calibrated: I0={}, I1={}", offset_i0, offset_i1);

    // 6. Re-enable PWM chopping
    let _ = tmc.write_register(PWM_SV_CHOP, 7).await;

    // --- Open-loop configuration ---

    // 7. PID gains for torque (current) control
    // Conservative Q8.8 PID gains: P = 1.577 * 256 = 404, I = 0.1 * 256 = 26.
    let pid_pi: u32 = (404 << 16) | 26;
    let _ = tmc.write_register(PID_FLUX_P_FLUX_I, pid_pi).await;
    let _ = tmc.write_register(PID_TORQUE_P_TORQUE_I, pid_pi).await;

    // 9. Current limit via PID_TORQUE_FLUX_LIMITS
    // The shared torque/flux limit is 433 counts (~0.5A with 1.155 mA/LSB).
    // Flux limit must be nonzero so the flux PID can actively regulate
    // the D-axis current to zero (reject disturbances).
    let current_limit = PidTorqueFluxLimit::new(433)
        .expect("dev current limit must fit the TMC4671 register domain");
    let _ = tmc
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit.register_value())
        .await;

    // 10. PHI_E source: external angle register (open-loop)
    let _ = tmc.write_register(PHI_E_SELECTION, 1).await;
    let _ = tmc.write_register(PHI_E_EXT, 0).await;

    // 11. Set flux (D-axis) target for alignment/holding at PHI_E=0
    // CRITICAL: Flux target is bits[15:0], torque target is bits[31:16].
    // Setting torque (Q-axis) instead of flux (D-axis) causes the rotor to
    // align 90 electrical degrees off, producing reversed commutation.
    let flux_target: i16 = 200; // ~0.23A, conservative
    let target_reg: u32 = PidTorqueFluxTarget::new()
        .with_pid_flux_target(flux_target)
        .with_pid_torque_target(0)
        .into();
    let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg).await;

    // 12. Motion mode: TORQUE mode with current regulation
    // MODE_PID_TYPE=1 (bit 31), MODE_MOTION=1 (torque)
    let _ = tmc.write_register(MODE_RAMP_MODE_MOTION, 0x8000_0001).await;

    // Verify critical registers after enabling torque mode
    let v_limit_final = tmc.read_register(PIDOUT_UQ_UD_LIMITS).await.unwrap_or(0);
    let mode_final = tmc.read_register(MODE_RAMP_MODE_MOTION).await.unwrap_or(0);
    let target_final = tmc.read_register(PID_TORQUE_FLUX_TARGET).await.unwrap_or(0);
    defmt::info!(
        "TMC4671 init done: V_limit={}, mode=0x{:08X}, target=0x{:08X}",
        v_limit_final,
        mode_final,
        target_final
    );

    // Diagnostic: log PID state for 5 seconds after enabling
    for i in 0..10u32 {
        Mono::delay(500.millis()).await;
        let actual = tmc.read_register(PID_TORQUE_FLUX_ACTUAL).await.unwrap_or(0);
        let adc = tmc.read_register(ADC_IWY_IUX).await.unwrap_or(0);
        let torque_actual = ((actual >> 16) & 0xFFFF) as i16;
        let flux_actual = (actual & 0xFFFF) as i16;
        let i_ux = (adc & 0xFFFF) as i16;
        let i_wy = ((adc >> 16) & 0xFFFF) as i16;
        defmt::info!(
            "[{}] PID: torque={} flux={} | ADC: ux={} wy={}",
            i,
            torque_actual,
            flux_actual,
            i_ux,
            i_wy
        );
    }
}
