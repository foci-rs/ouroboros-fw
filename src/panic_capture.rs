//! Probe-free panic and HardFault capture.
//!
//! The board's default panic handler was `panic-probe`, whose message reaches
//! only defmt-RTT; on a board with no debugger attached a panic escalates to a
//! HardFault and the independent watchdog silently resets it, losing every clue.
//! This module instead persists the fault into a no-init CCMRAM record that
//! survives the warm reset, then hands it to [`foci_firmware::diagnostic`] at
//! startup so the connect-time register dump reports it to the host over the
//! ordinary command transport.
//!
//! `CCMRAM` here is Ouroboros' `memory.x` alias for DTCM (0x2000_0000):
//! startup code never clears it, so it retains its contents across an
//! `SCB::sys_reset` or watchdog reset while power is held. The 32-bit magic
//! guards against reading uninitialised RAM as a record on the first cold
//! boot.
//!
//! The record carries no message string: a Rust panic is located by its source
//! `line` plus the FNV-1a `file_hash` of the panic path (match against hashed
//! candidate paths), and a HardFault by its `pc` (resolve with `addr2line`
//! against the ELF). That keeps the wire reply a fixed scalar layout.

use core::ptr::addr_of_mut;

use foci_firmware::diagnostic::{LastPanic, last_panic_kind};

/// Marks a slot that holds a valid captured record ("FpAN").
const PANIC_MAGIC: u32 = 0x4670_414E;

const FNV1A_OFFSET: u32 = 0x811c_9dc5;
const FNV1A_PRIME: u32 = 0x0100_0193;

/// FNV-1a hash of a byte string, matching the offline helper used to name the
/// panic file from its hash.
const fn fnv1a(bytes: &[u8]) -> u32 {
    let mut hash = FNV1A_OFFSET;
    let mut index = 0;
    while index < bytes.len() {
        hash ^= bytes[index] as u32;
        hash = hash.wrapping_mul(FNV1A_PRIME);
        index += 1;
    }
    hash
}

/// No-init CCMRAM record. `u32` fields keep it 4-byte aligned.
#[repr(C)]
struct PanicPersist {
    magic: u32,
    kind: u32,
    line: u32,
    pc: u32,
    file_hash: u32,
}

/// Lives in the `.ccmram.diag.panic` NOLOAD section so it is neither loaded nor
/// zeroed at startup and survives a reset. The initializer is never written to
/// the image.
#[unsafe(link_section = ".ccmram.diag.panic")]
static mut PANIC_PERSIST: PanicPersist = PanicPersist {
    magic: 0,
    kind: 0,
    line: 0,
    pc: 0,
    file_hash: 0,
};

/// Persist a record and reset. `SAFETY`: only ever called from the panic or
/// HardFault handler, i.e. single-threaded with no concurrent access to the
/// no-init slot.
unsafe fn persist_and_reset(kind: u8, line: u32, pc: u32, file_hash: u32) -> ! {
    let slot = addr_of_mut!(PANIC_PERSIST);
    // SAFETY: exclusive access in a fault handler; the slot is valid CCMRAM.
    unsafe {
        (*slot).kind = u32::from(kind);
        (*slot).line = line;
        (*slot).pc = pc;
        (*slot).file_hash = file_hash;
        // Publish the magic last so a reset mid-write never yields a torn record.
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        (*slot).magic = PANIC_MAGIC;
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    }
    cortex_m::peripheral::SCB::sys_reset()
}

/// Recover any record persisted by the previous boot, without clearing it. Call
/// once, early in startup, before the command transport comes up.
///
/// The magic is deliberately left set: a crash triggers an autonomous watchdog
/// reboot that recovers the record before the host reconnects, and the host's
/// own `FIRMWARE_RESTART` then resets the board a second time. Clearing here
/// would drop the record on that first reboot, before the connect-time dump
/// could ever report it. Leaving it set lets every reboot re-recover the same
/// record until a newer fault overwrites it or power is cycled; the host-facing
/// dedup (report once per boot) lives in the diagnostic layer's take.
pub fn recover() -> Option<LastPanic> {
    let slot = addr_of_mut!(PANIC_PERSIST);
    // SAFETY: called once at startup before any handler can run concurrently.
    unsafe {
        if (*slot).magic != PANIC_MAGIC {
            return None;
        }
        Some(LastPanic {
            kind: (*slot).kind as u8,
            line: (*slot).line,
            pc: (*slot).pc,
            file_hash: (*slot).file_hash,
        })
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let (line, file_hash) = info.location().map_or((0, 0), |location| {
        (location.line(), fnv1a(location.file().as_bytes()))
    });
    // SAFETY: panic context is single-threaded and terminal.
    unsafe { persist_and_reset(last_panic_kind::RUST_PANIC, line, 0, file_hash) }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    // SAFETY: fault context is single-threaded and terminal.
    unsafe { persist_and_reset(last_panic_kind::HARD_FAULT, 0, frame.pc(), 0) }
}

/// Route defmt's own panic path (`defmt::unwrap!`, `defmt::assert!`) through the
/// core panic handler above so those faults are persisted too. Previously
/// supplied by `panic-probe`.
#[defmt::panic_handler]
fn defmt_panic() -> ! {
    core::panic!("defmt")
}
