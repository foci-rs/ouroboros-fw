//! Ouroboros production-v1 STEP/DIR GPIO operations.

use core::sync::atomic::{AtomicBool, Ordering::Relaxed};

// Match Klipper's STM32H7 GPIO strategy: keep STEP parity in RAM and write
// BSRR from the mirror instead of deriving toggle state from GPIO readback.
static STEP_LEVELS: [AtomicBool; crate::board_config::TMC_CHANNEL_COUNT as usize] =
    [const { AtomicBool::new(false) }; crate::board_config::TMC_CHANNEL_COUNT as usize];

/// Set the DIR output for a motor channel.
pub(crate) fn set_dir(channel: u8, forward: bool) -> Result<(), ()> {
    let level = if forward {
        embassy_stm32::pac::gpio::vals::Odr::HIGH
    } else {
        embassy_stm32::pac::gpio::vals::Odr::LOW
    };
    match channel {
        0 => {
            embassy_stm32::pac::GPIOD
                .odr()
                .modify(|w| w.set_odr(3, level));
            Ok(())
        }
        1 => {
            embassy_stm32::pac::GPIOC
                .odr()
                .modify(|w| w.set_odr(11, level));
            Ok(())
        }
        _ => Err(()),
    }
}

/// Toggle STEP for a motor channel and return true when the new level is high.
pub(crate) fn toggle_step(channel: u8) -> Result<bool, ()> {
    let idx = channel as usize;
    let Some(level) = STEP_LEVELS.get(idx) else {
        return Err(());
    };
    let new_high = !level.load(Relaxed);
    match channel {
        0 => {
            let gpio = embassy_stm32::pac::GPIOD;
            gpio.bsrr().write(|w| {
                if new_high {
                    w.set_bs(4, true);
                } else {
                    w.set_br(4, true);
                }
            });
        }
        1 => {
            let gpio = embassy_stm32::pac::GPIOC;
            gpio.bsrr().write(|w| {
                if new_high {
                    w.set_bs(12, true);
                } else {
                    w.set_br(12, true);
                }
            });
        }
        _ => return Err(()),
    }
    level.store(new_high, Relaxed);
    Ok(new_high)
}

/// Return STEP/DIR outputs to safe idle state for a motor channel.
pub(crate) fn reset_idle(channel: u8) -> Result<(), ()> {
    let idx = channel as usize;
    let Some(level) = STEP_LEVELS.get(idx) else {
        return Err(());
    };
    match channel {
        0 => {
            let gpio = embassy_stm32::pac::GPIOD;
            gpio.bsrr().write(|w| {
                w.set_br(3, true);
                w.set_br(4, true);
            });
        }
        1 => {
            let gpio = embassy_stm32::pac::GPIOC;
            gpio.bsrr().write(|w| {
                w.set_br(11, true);
                w.set_br(12, true);
            });
        }
        _ => return Err(()),
    }
    level.store(false, Relaxed);
    Ok(())
}
