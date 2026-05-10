//! Board-side implementations of the [`foci_firmware::platform`]
//! traits. The impls shim foci-firmware's hardware-agnostic interface
//! down to the Ouroboros concrete PAC register accesses, RTIC-owned
//! peripherals, and USB transport helpers.
//!
//! These types exist so later tasks in Phase A-7 can migrate callers
//! off the direct-hardware path and onto the trait surface one group
//! at a time. For now, the board still calls the original helpers
//! directly — these impls are additive only. `dead_code` is allowed
//! here because callers land in later commits; removing the allow is
//! a Task 30 cleanup item.

#![allow(dead_code)]

use core::sync::atomic::{AtomicBool, Ordering::Relaxed};

use embassy_stm32::gpio::Output;
use foci_firmware::platform::{
    Clock, EndstopInput, FaultInput, MotorEnable, StepPins, StepTimer, TransportTx,
};

use crate::transport::USB_WAKE;

/// Total number of GPIO ports exposed in the pin enumeration
/// (`PA`..`PE`).
const GPIO_NUM_PORTS: u8 = 5;

/// Pin enum value representing the first non-GPIO virtual pin.
/// Pin enum: `PA0=0..PA15=15, PB0=16..PB15=31, ..., PE0=64..PE15=79`.
const GPIO_MAX_PIN_VALUE: u8 = GPIO_NUM_PORTS * 16;

/// Minimum lead time between TIM5.cnt() and the programmed CCR value.
/// Arming closer than this risks the ISR firing immediately on the
/// next tick, which `step_timer_irq` would then process before
/// `queue_step` can register the following move.
const STEP_TIMER_ARM_MIN_LEAD_TICKS: u32 = 128;

/// TIM5-backed Klipper clock (260 MHz, 32-bit free-running counter).
pub struct Tim5Clock;

impl Clock for Tim5Clock {
    fn now_ticks(&self) -> u32 {
        embassy_stm32::pac::TIM5.cnt().read()
    }

    fn frequency_hz(&self) -> u32 {
        260_000_000
    }
}

/// Step-pulse timer backed by TIM5 CC1.
pub struct Tim5StepTimer;

impl StepTimer for Tim5StepTimer {
    fn arm(&mut self, clock: u32) {
        let tim5 = embassy_stm32::pac::TIM5;
        // SAFETY: NVIC::unmask is idempotent; TIM5 is the only user.
        unsafe { cortex_m::peripheral::NVIC::unmask(embassy_stm32::pac::Interrupt::TIM5) };
        let now = tim5.cnt().read();
        let min_clock = now.wrapping_add(STEP_TIMER_ARM_MIN_LEAD_TICKS);
        let clock = if (clock as i32).wrapping_sub(min_clock as i32) < 0 {
            min_clock
        } else {
            clock
        };
        tim5.ccr(0).write_value(clock);
        tim5.sr().modify(|w| w.set_ccif(0, false)); // clear pending CC1
        tim5.dier().modify(|w| w.set_ccie(0, true)); // enable CC1 interrupt
    }

    fn cancel(&mut self) {
        let tim5 = embassy_stm32::pac::TIM5;
        tim5.dier().modify(|w| w.set_ccie(0, false));
    }
}

/// Channel-0 STEP (PD4) and DIR (PD3) pin driver.
pub struct StepDirPins;

impl StepPins for StepDirPins {
    fn set_step(&mut self, value: bool) {
        let gpio_d = embassy_stm32::pac::GPIOD;
        gpio_d.bsrr().write(|w| {
            if value {
                w.set_bs(4, true);
            } else {
                w.set_br(4, true);
            }
        });
    }

    fn set_dir(&mut self, value: bool) {
        use embassy_stm32::pac::gpio::vals;
        let gpio_d = embassy_stm32::pac::GPIOD;
        let odr_val = if value {
            vals::Odr::HIGH
        } else {
            vals::Odr::LOW
        };
        gpio_d.odr().modify(|w| w.set_odr(3, odr_val));
    }
}

/// PAC-backed GPIO read/configure for endstop / trsync inputs.
pub struct GpioEndstopInput;

fn gpio_port(port: u8) -> Option<embassy_stm32::pac::gpio::Gpio> {
    match port {
        0 => Some(embassy_stm32::pac::GPIOA),
        1 => Some(embassy_stm32::pac::GPIOB),
        2 => Some(embassy_stm32::pac::GPIOC),
        3 => Some(embassy_stm32::pac::GPIOD),
        4 => Some(embassy_stm32::pac::GPIOE),
        _ => None,
    }
}

impl EndstopInput for GpioEndstopInput {
    fn read(&self, pin_id: u8) -> bool {
        if pin_id >= GPIO_MAX_PIN_VALUE {
            return false;
        }
        let port_idx = pin_id / 16;
        let pin_idx = (pin_id % 16) as usize;
        gpio_port(port_idx).is_some_and(|gpio| {
            use embassy_stm32::pac::gpio::vals;
            gpio.idr().read().idr(pin_idx) == vals::Idr::HIGH
        })
    }

    fn configure(&mut self, pin_id: u8, pull_up: bool) {
        let port_idx = pin_id / 16;
        let pin_idx = (pin_id % 16) as usize;

        let rcc = embassy_stm32::pac::RCC;
        rcc.ahb4enr().modify(|w| match port_idx {
            0 => w.set_gpioaen(true),
            1 => w.set_gpioben(true),
            2 => w.set_gpiocen(true),
            3 => w.set_gpioden(true),
            4 => w.set_gpioeen(true),
            _ => {}
        });
        // Brief delay for clock to stabilize.
        cortex_m::asm::delay(2);

        if let Some(gpio) = gpio_port(port_idx) {
            use embassy_stm32::pac::gpio::vals;
            gpio.moder()
                .modify(|w| w.set_moder(pin_idx, vals::Moder::INPUT));
            let pull = if pull_up {
                vals::Pupdr::PULL_UP
            } else {
                vals::Pupdr::FLOATING
            };
            gpio.pupdr().modify(|w| w.set_pupdr(pin_idx, pull));
        }
    }
}

/// Wraps the TMC4671 enable pin held as an RTIC local resource.
pub struct TmcMotorEnable<'pin>(pub &'pin mut Output<'static>);

impl MotorEnable for TmcMotorEnable<'_> {
    fn set_enabled(&mut self, value: bool) {
        if value {
            self.0.set_high();
        } else {
            self.0.set_low();
        }
    }
}

/// Reads-and-clears a latched fault flag (`FAULT_ACTIVE`).
pub struct AtomicFaultInput<'flag>(pub &'flag AtomicBool);

impl FaultInput for AtomicFaultInput<'_> {
    fn take_latched(&mut self) -> bool {
        self.0.swap(false, Relaxed)
    }
}

/// Host transport: writes bytes to the USB CDC-ACM TX buffer and wakes
/// the USB task via [`USB_WAKE`].
pub struct UsbTransportTx;

impl TransportTx for UsbTransportTx {
    fn write(&mut self, bytes: &[u8]) -> usize {
        foci_usb_stm32::tx_write(bytes);
        bytes.len()
    }

    fn flush(&mut self) {
        USB_WAKE.signal(());
    }
}
