//! Dev-only helpers: open-loop init for raw register prodding and a
//! manual `do_test_spin` for sweeping `PHI_E_EXT`. Gated behind the
//! `dev` feature so production builds never link them.

#![cfg(feature = "dev")]

use embassy_stm32::gpio::Output;
use foci_firmware::tmc_register_values::{
    MOTION_STOPPED, MOTION_TORQUE_ADVANCED_PI, PWM_CHOP_CENTERED_FOC, PWM_CHOP_OFF,
    STEPPER_MOTOR_TYPE_N_POLE_PAIRS, adc_i0_scale_offset, adc_i1_scale_offset,
};
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::registers::*;
use foci_tmc4671::spi::Tmc4671Spi;
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::app::Mono;
use crate::app_types::TmcSpiDevice;
use crate::board_config::TMC_CONFIG;

const POLE_PAIRS: u32 = 50;
const PHI_E_STEPS_PER_ELECTRICAL_CYCLE: u32 = 256;
const PHI_E_STEP_SIZE: u32 = 65536 / PHI_E_STEPS_PER_ELECTRICAL_CYCLE;
const OPENLOOP_PID_P: u32 = 404;
const OPENLOOP_PID_I: u32 = 26;
const OPENLOOP_CURRENT_LIMIT: u32 = 433;
const OPENLOOP_FLUX_TARGET: i16 = 200;
const VELOCITY_METER_ADVANCED: u8 = 1;
const PHI_E_SELECTION_EXTERNAL: u32 = 1;

/// Sweep PHI_E_EXT through N full mechanical revolutions.
///
/// With 50 pole pairs, one mechanical revolution = 50 electrical
/// cycles. Each electrical cycle sweeps PHI_E_EXT from 0 to 65535.
/// Uses whatever voltage/mode settings were configured by tmc_init.
pub(crate) async fn do_test_spin(tmc: &mut Tmc4671Spi<TmcSpiDevice<'_>>, revolutions: u8) {
    let total_steps = revolutions as u32 * POLE_PAIRS * PHI_E_STEPS_PER_ELECTRICAL_CYCLE;
    defmt::info!(
        "Spinning {} revolutions ({} steps)",
        revolutions,
        total_steps
    );

    for step in 0..total_steps {
        let angle = (step * PHI_E_STEP_SIZE) % 65536;
        let _ = tmc.write_register(PHI_E_EXT, angle).await;
        Mono::delay(1.millis()).await;
    }

    let _ = tmc.write_register(PHI_E_EXT, 0).await;
    defmt::info!("Spin complete");
}

/// Configure TMC4671 for open-loop stepper operation with current regulation.
///
/// Performs hardware init, ADC offset calibration, and configures torque
/// mode with PHI_E_EXT for open-loop angle control.
pub(crate) async fn tmc_enter_openloop(
    tmc: &mut Tmc4671Spi<TmcSpiDevice<'_>>,
    enable_pin: &mut Output<'static>,
) {
    enable_pin.set_low();
    let _ = tmc
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
        .await;
    let _ = tmc.write_register(PIDOUT_UQ_UD_LIMITS, 1024).await;
    let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let _ = tmc.write_register(UQ_UD_EXT, 0).await;

    let _ = tmc.write_register(PWM_POLARITIES, 0).await;
    let _ = tmc
        .write_register(PWM_MAXCNT, TMC_CONFIG.pwm_maxcnt as u32)
        .await;
    let _ = tmc
        .write_register(PWM_BBM_H_BBM_L, TMC_CONFIG.bbm_register())
        .await;
    if tmc.write_register(PWM_SV_CHOP, PWM_CHOP_OFF).await.is_err() {
        defmt::error!("open-loop init: PWM_SV_CHOP write failed");
        return;
    }

    let _ = tmc
        .write_register(DSADC_MDEC_B_MDEC_A, TMC_CONFIG.dsadc_mdec_register())
        .await;
    let _ = tmc
        .write_register(ADC_I_SELECT, TMC_CONFIG.adc_i_select)
        .await;

    let _ = tmc
        .write_register(MOTOR_TYPE_N_POLE_PAIRS, STEPPER_MOTOR_TYPE_N_POLE_PAIRS)
        .await;

    let velocity_selection =
        VelocitySelection::new().with_velocity_meter_selection(VELOCITY_METER_ADVANCED);
    let _ = tmc
        .write_register(VELOCITY_SELECTION, velocity_selection.into_bits())
        .await;

    let _ = tmc
        .write_register(ADC_I0_SCALE_OFFSET, adc_i0_scale_offset(0))
        .await;
    let _ = tmc
        .write_register(ADC_I1_SCALE_OFFSET, adc_i1_scale_offset(0))
        .await;
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
        .write_register(ADC_I0_SCALE_OFFSET, adc_i0_scale_offset(offset_i0))
        .await;
    let _ = tmc
        .write_register(ADC_I1_SCALE_OFFSET, adc_i1_scale_offset(offset_i1))
        .await;
    defmt::info!("ADC calibrated: I0={}, I1={}", offset_i0, offset_i1);

    let _ = tmc.write_register(PWM_SV_CHOP, PWM_CHOP_CENTERED_FOC).await;

    let pid_pi: u32 = (OPENLOOP_PID_P << 16) | OPENLOOP_PID_I;
    let _ = tmc.write_register(PID_FLUX_P_FLUX_I, pid_pi).await;
    let _ = tmc.write_register(PID_TORQUE_P_TORQUE_I, pid_pi).await;

    let current_limit = PidTorqueFluxLimit::new(OPENLOOP_CURRENT_LIMIT)
        .expect("dev current limit must fit the TMC4671 register domain");
    let _ = tmc
        .write_register(PID_TORQUE_FLUX_LIMITS, current_limit.register_value())
        .await;

    let _ = tmc
        .write_register(PHI_E_SELECTION, PHI_E_SELECTION_EXTERNAL)
        .await;
    let _ = tmc.write_register(PHI_E_EXT, 0).await;

    let target_reg: u32 = PidTorqueFluxTarget::new()
        .with_pid_flux_target(OPENLOOP_FLUX_TARGET)
        .with_pid_torque_target(0)
        .into();
    let _ = tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg).await;

    let _ = tmc
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_TORQUE_ADVANCED_PI)
        .await;

    let v_limit_final = tmc.read_register(PIDOUT_UQ_UD_LIMITS).await.unwrap_or(0);
    let mode_final = tmc.read_register(MODE_RAMP_MODE_MOTION).await.unwrap_or(0);
    let target_final = tmc.read_register(PID_TORQUE_FLUX_TARGET).await.unwrap_or(0);
    defmt::info!(
        "TMC4671 init done: V_limit={}, mode=0x{:08X}, target=0x{:08X}",
        v_limit_final,
        mode_final,
        target_final
    );

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
