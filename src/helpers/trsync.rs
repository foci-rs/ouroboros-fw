//! Trsync / endstop helpers: raw-PAC GPIO configuration and sampling
//! for the Klipper pin enumeration, shared by the trsync/endstop
//! handlers and the TIM5 CC1 ISR so board GPIO access happens in one
//! place.

/// Total number of GPIO ports exposed in the pin enumeration
/// (`PA`..`PE`).
const GPIO_NUM_PORTS: u8 = 5;
/// Pin enum value representing the first non-GPIO virtual pin.
///
/// Pin enum: `PA0=0..PA15=15, PB0=16..PB15=31, ..., PE0=64..PE15=79`.
pub(crate) const GPIO_MAX_PIN_VALUE: u8 = GPIO_NUM_PORTS * 16;

/// Get the PAC GPIO port for the given port index (0=A, 1=B, ..., 4=E).
///
/// Returns `None` for out-of-range port indices.
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

/// Configure a GPIO pin as input with optional pull-up.
///
/// Enables the GPIO port clock, sets the pin to input mode, and
/// configures the pull-up/pull-down resistor. Uses raw PAC register
/// access to avoid the type-erasure problem with embassy-stm32
/// `Input` pins.
///
/// `pin_value` is the pin enum value (PA0=0, PB0=16, etc.).
pub(crate) fn gpio_configure_input(pin_value: u8, pull_up: bool) {
    let port_idx = pin_value / 16;
    let pin_idx = (pin_value % 16) as usize;

    // Enable GPIO port clock on AHB4.
    let rcc = embassy_stm32::pac::RCC;
    rcc.ahb4enr().modify(|w| match port_idx {
        0 => w.set_gpioaen(true),
        1 => w.set_gpioben(true),
        2 => w.set_gpiocen(true),
        3 => w.set_gpioden(true),
        4 => w.set_gpioeen(true),
        _ => {}
    });
    // Brief delay for clock to stabilize
    cortex_m::asm::delay(2);

    if let Some(gpio) = gpio_port(port_idx) {
        use embassy_stm32::pac::gpio::vals;
        // Set pin as input
        gpio.moder()
            .modify(|w| w.set_moder(pin_idx, vals::Moder::INPUT));
        // Configure pull-up or floating
        let pull = if pull_up {
            vals::Pupdr::PULL_UP
        } else {
            vals::Pupdr::FLOATING
        };
        gpio.pupdr().modify(|w| w.set_pupdr(pin_idx, pull));
    }
}

/// Read the current state of a GPIO pin.
///
/// `pin_value` is the pin enum value (PA0=0, PB0=16, etc.).
/// Returns `false` if the pin value is out of range.
pub(crate) fn gpio_read_pin(pin_value: u8) -> bool {
    if pin_value >= GPIO_MAX_PIN_VALUE {
        return false;
    }
    let port_idx = pin_value / 16;
    let pin_idx = (pin_value % 16) as usize;

    gpio_port(port_idx).is_some_and(|gpio| {
        use embassy_stm32::pac::gpio::vals;
        gpio.idr().read().idr(pin_idx) == vals::Idr::HIGH
    })
}
