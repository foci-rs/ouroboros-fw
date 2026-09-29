use std::env;
use std::fs;
use std::path::PathBuf;
use std::process::Command;

fn git_output(repo: &str, args: &[&str]) -> Option<String> {
    let out = Command::new("git")
        .arg("-C")
        .arg(repo)
        .args(args)
        .output()
        .ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
        .filter(|text| !text.is_empty())
}

fn describe(repo: &str) -> String {
    git_output(
        repo,
        &["describe", "--always", "--abbrev=8", "--dirty=-dirty"],
    )
    .unwrap_or_else(|| "unknown".to_string())
}

fn emit_build_info() {
    let board = describe(".");
    let pkg = env::var("CARGO_PKG_VERSION").unwrap();
    let commits_since_bump = git_output(
        ".",
        &[
            "log",
            "-1",
            "--format=%H",
            "-G^version = ",
            "--",
            "Cargo.toml",
        ],
    )
    .and_then(|bump| git_output(".", &["rev-list", "--count", &format!("{bump}..HEAD")]))
    .unwrap_or_else(|| "0".to_string());
    let version = format!("v{pkg}-{commits_since_bump}-g{board}");

    let rustc = env::var("RUSTC").unwrap_or_else(|_| "rustc".to_string());
    let rust = Command::new(rustc)
        .arg("--version")
        .output()
        .ok()
        .and_then(|out| {
            String::from_utf8_lossy(&out.stdout)
                .split_whitespace()
                .nth(1)
                .map(str::to_string)
        })
        .unwrap_or_else(|| "unknown".to_string());

    let mut features: Vec<String> = env::vars()
        .filter_map(|(key, _)| key.strip_prefix("CARGO_FEATURE_").map(str::to_string))
        .filter(|name| name != "DEFAULT")
        .map(|name| name.to_lowercase().replace('_', "-"))
        .collect();
    features.sort();
    let features = if features.is_empty() {
        "none".to_string()
    } else {
        features.join(",")
    };

    let deps = [
        ("fw", "../../shared/foci-firmware"),
        ("tmc", "../../shared/foci-tmc4671"),
        ("ankyra", "../../ankyra"),
    ];
    let mut build_versions = format!("rust={rust} features={features} board={board}");
    for (label, repo) in deps {
        build_versions.push_str(&format!(" {label}={}", describe(repo)));
    }

    for repo in std::iter::once(".").chain(deps.iter().map(|(_, repo)| *repo)) {
        println!("cargo:rerun-if-changed={repo}");
        if let Some(git_dir) = git_output(repo, &["rev-parse", "--absolute-git-dir"]) {
            println!("cargo:rerun-if-changed={git_dir}");
        }
    }
    println!("cargo:rustc-env=FOCI_VERSION={version}");
    println!("cargo:rustc-env=FOCI_BUILD_VERSIONS={build_versions}");
}

fn main() {
    emit_build_info();
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
