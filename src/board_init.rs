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

/// Build the embassy-stm32 clock configuration for 520 MHz SYSCLK from the
/// 25 MHz HSE crystal.
pub fn rcc_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;

    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = Some(HSIPrescaler::DIV1);
    config.rcc.hse = Some(Hse {
        freq: embassy_stm32::time::Hertz(25_000_000),
        mode: HseMode::Oscillator,
    });
    // CRS trims HSI48 against host SOF timing; USB full-speed's tolerance is
    // tighter than HSI48's factory trim alone.
    config.rcc.hsi48 = Some(Hsi48Config {
        sync_from_usb: true,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV5,
        mul: PllMul::MUL104,
        divp: Some(PllDiv::DIV1),
        divq: None,
        divr: None,
    });
    // SPI123 and the ADC have no usable kernel clock from PLL1: sysclk 520 MHz
    // pins PLL1's VCO to 520 (DIVP must be 1, since DIVP=2 needs a 1040 MHz
    // VCO, above the 836 MHz ceiling), and 520 = 8 * 65 with 65 odd, so no
    // DIVQ yields a multiple-of-8 MHz kernel. PLL2 at 32 MHz divides to
    // exactly 8 MHz for the TMC4671 link.
    config.rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV5,
        mul: PllMul::MUL96,
        divp: Some(PllDiv::DIV15),
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
    config.rcc.mux.spi123sel = mux::Saisel::PLL2_P;
    config.rcc.mux.usbsel = mux::Usbsel::HSI48;

    // SPI2 RX/TX DMA completion (DMA1_STREAM0/DMA1_STREAM1) defaults to
    // Embassy's Priority::P0, the highest hardware priority on this part --
    // above every RTIC task, including the P3 TIM5/USB/STATUS handlers RTIC
    // treats as the hard-real-time ceiling. H723 implements 4 NVIC priority
    // bits, the same as F407; RTIC's cortex_logical2hw maps its logical
    // priorities to raw NVIC bytes as P3 -> 0xD0, P2 -> 0xE0, P1 -> 0xF0.
    // Embassy's `Priority` enum uses the same byte encoding (`Priority::P13`
    // = 0xD0, `P14` = 0xE0, `P15` = 0xF0).
    //
    // Use P15 (RTIC-P1 level), not P14 (RTIC-P2 level): P14 can preempt P1
    // tasks, including tmc_control itself, mid-execution; P15 cannot, since
    // equal-priority interrupts never preempt each other on Cortex-M, so a
    // P15 DMA completion can only run in a gap between tmc_control's own
    // execution slices, never inside one. P14 does not eliminate contention
    // with the P2 tasks either way -- P14 sits at the exact same priority as
    // usb_task and the other P2 tasks (so it cannot preempt them), and P15
    // sits strictly below P2 (so it cannot preempt them either); neither
    // choice bounds usb_task's unyielding per-wake loop. Live homing testing
    // on the sibling OpenFFBoard firmware correlated P14 with an intermittent
    // TMC4671 current-loop/encoder-read disruption that did not reproduce at
    // P15 across repeated trials; the full causal mechanism is not yet
    // instrumented and this should be revisited with per-source
    // DMA/USB/TIM5 activation-and-duration counters if further tuning is
    // needed.
    // `board_init::verify_dma_interrupt_priority` checks the programmed
    // priority stays below P3 at boot.
    config.dma_interrupt_priority = embassy_stm32::interrupt::Priority::P15;

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

/// Keep the debug/trace clock domain alive while the core sleeps.
///
/// The `#[idle]` task's `wfi()` puts the Cortex-M7 core into D1-domain
/// Sleep mode whenever no interrupt is pending -- most of the time, given
/// the 10 kHz control tick. Without `DBGSLEEP_D1` set, an attached
/// debugger can lose the core or stall RTT reads across every sleep
/// entry. `DBGSTOP_D1`/`DBGSTBY_D1` are left unset: this firmware never
/// sets `SLEEPDEEP`, so it never reaches D1 Stop or Standby.
pub fn enable_debug_clock_during_sleep() {
    embassy_stm32::pac::DBGMCU
        .cr()
        .modify(|w| w.set_dbgsleep_d1(true));
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

/// Configure the two TMC4671 STATUS lines and return the channel-0 input.
///
/// PE2 (channel 0) is a plain pulled-down input; EXTI line 2 is armed later
/// by `tmc_control` once `STATUS_MASK` is written. PA2 (channel 1) cannot
/// share EXTI line 2, so it is routed to TIM15 channel 1 (AF4) for an
/// input-capture interrupt; its level stays readable through GPIOA IDR.
pub fn init_tmc_status_pins(
    pe2: Peri<'static, peripherals::PE2>,
    pa2: Peri<'static, peripherals::PA2>,
) -> Input<'static> {
    let mut status1 = Flex::new(pa2);
    status1.set_as_af_unchecked(4, AfType::input(Pull::Down));
    core::mem::forget(status1);
    Input::new(pe2, Pull::Down)
}

/// Current level of the channel-1 STATUS line (PA2) via the input data register.
#[allow(dead_code)]
pub fn status1_level() -> bool {
    embassy_stm32::pac::GPIOA.idr().read().idr(2) == embassy_stm32::pac::gpio::vals::Idr::HIGH
}

/// Clock TIM15 and arm channel 1 as a rising-edge input capture on PA2.
///
/// The capture interrupt (DIER.CC1IE) stays masked until channel 1's safety
/// bootstrap has written `STATUS_MASK`.
pub fn init_tim15_status_capture() {
    embassy_stm32::rcc::enable_and_reset::<peripherals::TIM15>();
    let tim = embassy_stm32::pac::TIM15;
    tim.ccmr_input()
        .modify(|w| w.set_ccs(0, embassy_stm32::pac::timer::vals::CcmrInputCcs::TI4));
    tim.ccer().modify(|w| {
        w.set_ccp(0, false);
        w.set_ccnp(0, false);
        w.set_cce(0, true);
    });
    tim.cr1().modify(|w| w.set_cen(true));
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
    // The H7 USB transceiver sits in its own 3.3 V supply domain. Without
    // USB33DEN the PHY signals well enough to be seen on the bus but stalls
    // during descriptor reads. Klipper's OTG init enables it the same way; the
    // F407 has no equivalent bit, which is why the shared USB crate does not.
    let pwr = embassy_stm32::pac::PWR;
    if !pwr.cr3().read().usb33rdy() {
        pwr.cr3().modify(|w| w.set_usb33den(true));
        while !pwr.cr3().read().usb33rdy() {}
    }
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

/// Confirm the SPI2 RX/TX DMA completion interrupts (DMA1_STREAM0,
/// DMA1_STREAM1) are configured below the RTIC P3 hard-real-time ceiling
/// (TIM5 CC1/CC2, OTG HS, EXTI2 STATUS -- raw NVIC priority `0xD0` on this
/// 4-priority-bit part, `Priority::P13` in Embassy's encoding).
///
/// Panics if either DMA interrupt is at or above that level: that would let
/// SPI DMA completion preempt STEP/DIR dispatch on either channel, the exact
/// regression this check exists to catch (a future embassy-stm32 upgrade
/// could reintroduce it by changing the default, or a later edit could
/// remove the explicit override in `rcc_config`).
pub fn verify_dma_interrupt_priority() {
    use embassy_stm32::interrupt::{Interrupt, InterruptExt, Priority};

    let rx = Interrupt::DMA1_STREAM0.get_priority();
    let tx = Interrupt::DMA1_STREAM1.get_priority();
    defmt::info!(
        "SPI DMA NVIC priority: DMA1_STREAM0={} DMA1_STREAM1={}",
        rx,
        tx
    );
    assert!(
        rx > Priority::P13 && tx > Priority::P13,
        "SPI DMA interrupt priority is at or above RTIC P3 (TIM5/USB/STATUS)"
    );
}
