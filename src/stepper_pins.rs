//! Ouroboros production-v1 STEP/DIR GPIO operations.

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
    use embassy_stm32::pac::gpio::vals::Idr;
    match channel {
        0 => {
            let gpio = embassy_stm32::pac::GPIOD;
            let is_high = gpio.idr().read().idr(4) == Idr::HIGH;
            gpio.bsrr().write(|w| {
                if is_high {
                    w.set_br(4, true);
                } else {
                    w.set_bs(4, true);
                }
            });
            Ok(!is_high)
        }
        1 => {
            let gpio = embassy_stm32::pac::GPIOC;
            let is_high = gpio.idr().read().idr(12) == Idr::HIGH;
            gpio.bsrr().write(|w| {
                if is_high {
                    w.set_br(12, true);
                } else {
                    w.set_bs(12, true);
                }
            });
            Ok(!is_high)
        }
        _ => Err(()),
    }
}

/// Return STEP/DIR outputs to safe idle state for a motor channel.
pub(crate) fn reset_idle(channel: u8) -> Result<(), ()> {
    match channel {
        0 => {
            let gpio = embassy_stm32::pac::GPIOD;
            gpio.odr()
                .modify(|w| w.set_odr(3, embassy_stm32::pac::gpio::vals::Odr::LOW));
            gpio.odr()
                .modify(|w| w.set_odr(4, embassy_stm32::pac::gpio::vals::Odr::LOW));
            Ok(())
        }
        1 => {
            let gpio = embassy_stm32::pac::GPIOC;
            gpio.odr()
                .modify(|w| w.set_odr(11, embassy_stm32::pac::gpio::vals::Odr::LOW));
            gpio.odr()
                .modify(|w| w.set_odr(12, embassy_stm32::pac::gpio::vals::Odr::LOW));
            Ok(())
        }
        _ => Err(()),
    }
}
