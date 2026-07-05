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

    // Generate CCMRAM linker section supplement
    let ccmram_x = out_dir.join("ccmram.x");
    fs::write(
        &ccmram_x,
        "SECTIONS {\n\
         \x20 .ccmram (NOLOAD) : ALIGN(4) {\n\
         \x20   *(.ccmram .ccmram.*)\n\
         \x20 } > CCMRAM\n\
         }\n",
    )
    .expect("failed to write ccmram.x");

    println!("cargo:rustc-link-search={}", out_dir.display());

    // Linker configuration
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-arg-bins=-Tccmram.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-dev.x");
    println!("cargo:rerun-if-changed=build.rs");
}
