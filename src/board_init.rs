//! Board-bring-up helpers for Ouroboros production-v1.
//!
//! These functions perform the hardware-specific pieces of the RTIC
//! `init` task: clock tree configuration, timer bring-up, GPIO setup,
//! USB peripheral preparation, and DWT instrumentation. Each helper is a
//! pure step in the init sequence; the RTIC `init` task orchestrates the
//! ordering and ownership of peripherals.

use embassy_stm32::Peri;
use embassy_stm32::gpio::{AfType, Flex, Input, Pull};
use embassy_stm32::peripherals;

/// Read the STM32 96-bit unique device ID and format as a hex string.
///
/// The UID is at 0x1FF1_E800 on STM32H7. Returns a 24-character hex
/// string. Reads the 12 raw bytes sequentially (matching
/// Klipper/Katapult convention) and outputs high nibble first for each
/// byte.
pub fn read_stm32_uid(buf: &mut [u8; 24]) -> &str {
    const UID_BASE: *const u8 = 0x1FF1_E800 as *const u8;
    let hex = b"0123456789ABCDEF";
    for i in 0..12 {
        // SAFETY: UID registers are always readable at this address on STM32H7.
        let b = unsafe { core::ptr::read_volatile(UID_BASE.add(i)) };
        buf[i * 2] = hex[(b >> 4) as usize];
        buf[i * 2 + 1] = hex[(b & 0x0F) as usize];
    }
    // SAFETY: buf contains only ASCII hex characters
    unsafe { core::str::from_utf8_unchecked(buf) }
}

/// Build the embassy-stm32 clock configuration for 520 MHz SYSCLK from HSI64.
pub fn rcc_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = Some(HSIPrescaler::DIV1);
    config.rcc.hse = None;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV16,
        mul: PllMul::MUL130,
        divp: Some(PllDiv::DIV1),
        divq: None,
        divr: None,
    });
    config.rcc.d1c_pre = AHBPrescaler::DIV1;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    config
}

/// Configure TIM5 as the free-running 32-bit Klipper clock at 260 MHz.
///
/// TIM5 is the sole Klipper clock source and the step-timer base. CC1
/// interrupt is enabled later (first `queue_step` arms the timer);
/// unmasking TIM5 in the NVIC is similarly deferred so the UIF flag
/// set by the initial EGR.UG does not fire a spurious ISR.
pub fn init_tim5_clock() {
    let rcc = embassy_stm32::pac::RCC;
    rcc.apb1lenr().modify(|w| w.set_tim5en(true));
    // Two cycles for clock to stabilize.
    cortex_m::asm::delay(2);
    let tim5 = embassy_stm32::pac::TIM5;
    tim5.psc().write_value(0); // No prescaler: 260 MHz timer clock
    tim5.arr().write_value(0xFFFF_FFFF); // Full 32-bit range
    tim5.egr().write(|w| w.set_ug(true)); // Force update to load PSC/ARR
    tim5.cr1().modify(|w| w.set_cen(true)); // Start counting

    // Disable all TIM5 interrupts and clear pending flags.
    tim5.dier().write_value(Default::default());
    tim5.sr().write_value(Default::default());
}

/// Configure verified Ouroboros STEP/DIR outputs at very high speed.
pub fn init_stepdir_gpio() {
    use embassy_stm32::pac::gpio::vals;
    let rcc = embassy_stm32::pac::RCC;
    rcc.ahb4enr().modify(|w| {
        w.set_gpiocen(true);
        w.set_gpioden(true);
    });

    let gpio_d = embassy_stm32::pac::GPIOD;
    configure_output(&gpio_d, 4); // STEP0
    configure_output(&gpio_d, 3); // DIR0

    let gpio_c = embassy_stm32::pac::GPIOC;
    configure_output(&gpio_c, 12); // STEP1
    configure_output(&gpio_c, 11); // DIR1

    fn configure_output(gpio: &embassy_stm32::pac::gpio::Gpio, pin: usize) {
        gpio.moder()
            .modify(|w| w.set_moder(pin, vals::Moder::OUTPUT));
        gpio.otyper().modify(|w| w.set_ot(pin, vals::Ot::PUSH_PULL));
        gpio.ospeedr()
            .modify(|w| w.set_ospeedr(pin, vals::Ospeedr::VERY_HIGH_SPEED));
        gpio.odr().modify(|w| w.set_odr(pin, vals::Odr::LOW));
    }
}

/// Configure the TMC4671 STATUS lines as pulled-down inputs.
/// EXTI interrupt is enabled later in `tmc_control` after `STATUS_MASK`
/// is written to the TMC4671.
pub fn init_tmc_status_pins(
    pe2: Peri<'static, peripherals::PE2>,
    pa2: Peri<'static, peripherals::PA2>,
) {
    let status0_pin = Input::new(pe2, Pull::Down);
    let status1_pin = Input::new(pa2, Pull::Down);
    core::mem::forget(status0_pin);
    core::mem::forget(status1_pin);
}

/// Configure PA11 (USB D-) and PA12 (USB D+) for USB OTG HS in FS mode.
/// AF 10 is the USB OTG alternate function on STM32H7.
pub fn init_usb_pins(
    pa11: Peri<'static, peripherals::PA11>,
    pa12: Peri<'static, peripherals::PA12>,
) {
    let mut dm = Flex::new(pa11);
    dm.set_as_af_unchecked(10, AfType::input(Pull::None));
    let mut dp = Flex::new(pa12);
    dp.set_as_af_unchecked(10, AfType::input(Pull::None));
    // Pins must stay configured — don't drop the Flex handles.
    core::mem::forget(dm);
    core::mem::forget(dp);
}

/// Enable the USB OTG HS peripheral clock and briefly delay so the
/// clock can stabilize before further register accesses.
pub fn enable_usb_clock() {
    embassy_stm32::pac::RCC
        .ahb1enr()
        .modify(|w| w.set_usb_otg_hsen(true));
    cortex_m::asm::delay(100);
}

/// Enable the DWT cycle counter.
///
/// CYCCNT is disabled on reset; it requires DEMCR.TRCENA and
/// DWT.CYCCNTENA to be set. Used for first-sample SPI timing
/// measurements.
pub fn enable_dwt_cyccnt() {
    // SAFETY: accessed exactly once during init. DCB and DWT are
    // Cortex-M system registers that are always safe to read/write here.
    unsafe {
        let dcb = &*cortex_m::peripheral::DCB::PTR;
        dcb.demcr.modify(|v| v | (1 << 24)); // TRCENA
        let dwt = &*cortex_m::peripheral::DWT::PTR;
        dwt.cyccnt.write(0);
        dwt.ctrl.modify(|v| v | 1); // CYCCNTENA
    }
}
