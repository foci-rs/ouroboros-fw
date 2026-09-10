use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Copy the appropriate memory layout to OUT_DIR.
    // Default: Katapult bootloader offset (0x08008000).
    // With --features dev-flash: direct flash at 0x08000000 for probe-rs.
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let memory_x = if env::var("CARGO_FEATURE_DEV_FLASH").is_ok() {
        "memory-dev.x"
    } else {
        "memory.x"
    };
    fs::copy(memory_x, out_dir.join("memory.x")).unwrap();

    // The allowlist admits only the two named no-init diagnostic records; any
    // other `.ccmram*` input section becomes an orphan and is rejected by
    // --orphan-handling=error below rather than silently absorbed into this
    // section.
    let ccmram_x = out_dir.join("ccmram.x");
    fs::write(
        &ccmram_x,
        "SECTIONS {\n\
         \x20 .ccmram (NOLOAD) : ALIGN(4) {\n\
         \x20   *(.ccmram.diag.panic)\n\
         \x20   *(.ccmram.diag.breakaway)\n\
         \x20 } > CCMRAM\n\
         }\n\
         /* PanicPersist (5 x u32) + the breakaway AtomicU32 breadcrumb. */\n\
         ASSERT(SIZEOF(.ccmram) == 24, \"unexpected CCMRAM contents\");\n\
         \n\
         /* --orphan-handling=error hard-errors on any input section the script\n\
          * does not place explicitly, including debug/symbol metadata GNU ld\n\
          * otherwise places implicitly. Route those here so the flag only ever\n\
          * fires on an unexpected `.ccmram*` input. */\n\
         SECTIONS {\n\
         \x20 .debug_loc 0 : { *(.debug_loc) }\n\
         \x20 .debug_abbrev 0 : { *(.debug_abbrev) }\n\
         \x20 .debug_info 0 : { *(.debug_info) }\n\
         \x20 .debug_aranges 0 : { *(.debug_aranges) }\n\
         \x20 .debug_ranges 0 : { *(.debug_ranges) }\n\
         \x20 .debug_str 0 : { *(.debug_str) }\n\
         \x20 .debug_frame 0 : { *(.debug_frame) }\n\
         \x20 .debug_line 0 : { *(.debug_line) }\n\
         \x20 .comment 0 : { *(.comment) }\n\
         \x20 .ARM.attributes 0 : { *(.ARM.attributes) }\n\
         \x20 .symtab 0 : { *(.symtab) }\n\
         \x20 .strtab 0 : { *(.strtab) }\n\
         \x20 .shstrtab 0 : { *(.shstrtab) }\n\
         }\n",
    )
    .expect("failed to write ccmram.x");

    println!("cargo:rustc-link-search={}", out_dir.display());

    // Linker configuration
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=--orphan-handling=error");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-arg-bins=-Tccmram.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-dev.x");
    println!("cargo:rerun-if-changed=build.rs");
}
