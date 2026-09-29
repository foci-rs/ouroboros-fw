use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let memory_x = if env::var("CARGO_FEATURE_DEV_FLASH").is_ok() {
        "memory-dev.x"
    } else {
        "memory.x"
    };
    fs::copy(memory_x, out_dir.join("memory.x")).unwrap();

    let ccmram_x = out_dir.join("ccmram.x");
    fs::write(
        &ccmram_x,
        "SECTIONS {\n\
         \x20 .ccmram (NOLOAD) : ALIGN(4) {\n\
         \x20   *(.ccmram.diag.panic)\n\
         \x20   *(.ccmram.diag.breakaway)\n\
         \x20 } > CCMRAM\n\
         }\n\
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

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=--orphan-handling=error");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-arg-bins=-Tccmram.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-dev.x");
    println!("cargo:rerun-if-changed=build.rs");
}
