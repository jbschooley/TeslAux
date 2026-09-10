//! Put the selected board's memory layout on the linker search path.
use std::{env, fs, path::PathBuf};

fn main() {
    // One board feature is active; `board.rs` enforces that. The flash sizes
    // differ (2 MB against 16 MB) and linking against the wrong one either
    // wastes most of the part or overflows it.
    let (src, name): (&[u8], &str) = if cfg!(feature = "board-pizero") {
        (include_bytes!("memory-pizero.x"), "memory-pizero.x")
    } else {
        (include_bytes!("memory.x"), "memory.x")
    };

    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), src).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:warning=linking for {name}");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-pizero.x");
    println!("cargo:rerun-if-changed=build.rs");
}
