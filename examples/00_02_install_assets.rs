//! Copies this checkout's Panda model assets, `assets/`, into the shared
//! asset directory, `roplat_world::assets_dir()`, where simulators and viewers
//! resolve the `RobotDescription` paths of `FrankaEmika` and `FrankaFR3`.
//! Driving a real robot needs none of them. Run it from a source checkout:
//! `cargo run --example 00_02_install_assets`.
use std::{fs, io, path::Path};

fn copy_dir(from: &Path, to: &Path) -> io::Result<usize> {
    fs::create_dir_all(to)?;
    let mut copied = 0;
    for entry in fs::read_dir(from)? {
        let entry = entry?;
        let target = to.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copied += copy_dir(&entry.path(), &target)?;
        } else {
            fs::copy(entry.path(), target)?;
            copied += 1;
        }
    }
    Ok(copied)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let source = Path::new(env!("CARGO_MANIFEST_DIR")).join("assets");
    let target = roplat_world::assets_dir().ok_or("no user data directory to install into")?;
    let copied = copy_dir(&source, &target)?;
    println!("copied {copied} files to {}", target.display());
    Ok(())
}
