//! Wire-format enumerations for the ankyra Klipper protocol.
//!
//! Ankyra's `klipper_enumeration!` uses block-form syntax with an
//! optional `(name = "...", rename_all = "...")` header on the enum
//! ident. When `name` is set explicitly, it is emitted verbatim as the
//! wire name — no PascalCase → snake_case conversion is applied.
//! Variant idents are likewise untouched unless `rename_all` is set
//! (see `ankyra-macros/src/enumeration.rs`).

// Migrated from app_types.rs: the `Pins` enum previously declared with
// anchor's `klipper_enumeration!`. The explicit `name = "pin"` header
// preserves the wire name byte-for-byte against the baseline data
// dictionary. TMC4671 handles all pins internally — these exist only
// for Klipper config pin name resolution.
::ankyra::klipper_enumeration! {
    #[allow(non_camel_case_types)]
    pub enum Pins(name = "pin") {
        Range(PA, 0, 16),
        Range(PB, 0, 16),
        Range(PC, 0, 16),
        Range(PD, 0, 16),
        Range(PE, 0, 16),
        // Virtual pins for FOCI stepper control.
        // STEP/DIR are handled internally (no GPIO toggle).
        // ENA maps to the channel-local TMC4671 enable pin.
        STEP0,
        DIR0,
        ENA0,
        STEP1,
        DIR1,
        ENA1,
        STALL0,
        STALL1,
        TH0,
        TH1,
        MCU_TEMP,
    }
}
