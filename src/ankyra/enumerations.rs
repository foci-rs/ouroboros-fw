//! Wire-format enumerations for the ankyra Klipper protocol.

::ankyra::klipper_enumeration! {
    #[allow(non_camel_case_types)]
    pub enum Pins(name = "pin") {
        Range(PA, 0, 16),
        Range(PB, 0, 16),
        Range(PC, 0, 16),
        Range(PD, 0, 16),
        Range(PE, 0, 16),
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
