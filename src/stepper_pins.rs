//! Ouroboros production-v1 STEP/DIR GPIO operations.

use core::sync::atomic::{AtomicBool, Ordering::Relaxed};

// Match Klipper's STM32H7 GPIO strategy: keep STEP parity in RAM and write
// BSRR from the mirror instead of deriving toggle state from GPIO readback.
static STEP_LEVELS: [AtomicBool; crate::board_config::TMC_CHANNEL_COUNT as usize] =
    [const { AtomicBool::new(false) }; crate::board_config::TMC_CHANNEL_COUNT as usize];

/// Set the DIR output for a motor channel.
pub(crate) fn set_dir(channel: u8, forward: bool) -> Result<(), ()> {
    match channel {
        0 => {
            embassy_stm32::pac::GPIOD.bsrr().write(|w| {
                if forward {
                    w.set_bs(3, true)
                } else {
                    w.set_br(3, true)
                }
            });
            Ok(())
        }
        1 => {
            embassy_stm32::pac::GPIOC.bsrr().write(|w| {
                if forward {
                    w.set_bs(11, true)
                } else {
                    w.set_br(11, true)
                }
            });
            Ok(())
        }
        _ => Err(()),
    }
}

/// Drive STEP to an explicit level for a motor channel.
pub(crate) fn set_step(channel: u8, high: bool) -> Result<(), ()> {
    let idx = channel as usize;
    let Some(level) = STEP_LEVELS.get(idx) else {
        return Err(());
    };
    match channel {
        0 => {
            let gpio = embassy_stm32::pac::GPIOD;
            gpio.bsrr().write(|w| {
                if high {
                    w.set_bs(4, true);
                } else {
                    w.set_br(4, true);
                }
            });
        }
        1 => {
            let gpio = embassy_stm32::pac::GPIOC;
            gpio.bsrr().write(|w| {
                if high {
                    w.set_bs(12, true);
                } else {
                    w.set_br(12, true);
                }
            });
        }
        _ => return Err(()),
    }
    level.store(high, Relaxed);
    Ok(())
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
