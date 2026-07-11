# Ouroboros Host Contract Tests

This durable test crate compiles the board's production
`src/state/stepper.rs` module directly. It does not copy production logic.

The firmware binary cannot use the host test harness because its RTIC and
Cortex-M dependencies contain ARM-only inline assembly, while the thumb target
does not provide Rust's `libtest`. Run the board-pure contracts explicitly on
the development host:

```sh
cargo test -p ouroboros-host-tests
```

Run that command from the aggregate FOCI checkout. The host-test manifest uses
the same git dependency URLs as the board manifest, while the aggregate
workspace supplies local `[patch]` overrides during development.
