# ouroboros-fw

FOCI firmware for the Ouroboros production-v1 board (STM32H723ZG, RTIC).

Part of the [FOCI](https://github.com/foci-rs/foci) project — a
Klipper/Kalico-compatible MCU firmware family for STM32 boards with
TMC4671 FOC servo controller ICs.

## Status

Pre-1.0 compile-only scaffold. Hardware bring-up is blocked on
production-v1 pin verification.

## Use

Add as a git dependency in your `Cargo.toml`:

```toml
ouroboros-fw = { git = "https://github.com/foci-rs/ouroboros-fw" }
```

## License

Dual-licensed under MIT or Apache-2.0 at your option. See `LICENSE-MIT` and `LICENSE-APACHE`.
