//! Ouroboros production-v1 STEP/DIR GPIO operations.

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
    Ok(())
}

/// Return STEP/DIR outputs to safe idle state for a motor channel.
pub(crate) fn reset_idle(channel: u8) -> Result<(), ()> {
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
    Ok(())
}
