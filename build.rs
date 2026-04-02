use std::{
    env, fs,
    path::{Path, PathBuf},
};

fn main() {
    #[cfg(feature = "to_cxx")]
    {
        cxx_build::bridge("src/ffi/to_cxx.rs")
            .std("c++14")
            .compile("franka_rust_cxx");

        println!("cargo:rerun-if-changed=src/ffi/to_cxx.rs");
    }

    let manifest_dir = match env::var("CARGO_MANIFEST_DIR") {
        Ok(v) => v,
        Err(err) => {
            println!("cargo:warning=missing CARGO_MANIFEST_DIR: {err}");
            return;
        }
    };
    let source = Path::new(&manifest_dir).join("assets");

    emit_rerun_if_changed(&source);

    if let Err(err) = export_project_assets() {
        println!("cargo:warning=failed to export project assets: {err}");
    }
}

fn export_project_assets() -> Result<(), String> {
    let manifest_dir =
        env::var("CARGO_MANIFEST_DIR").map_err(|e| format!("missing CARGO_MANIFEST_DIR: {e}"))?;
    let source = Path::new(&manifest_dir).join("assets");

    if !source.exists() {
        return Err(format!(
            "source assets directory not found: {}",
            source.display()
        ));
    }

    // 获取用户数据目录（跨平台）
    let target_root =
        roplat_data_target_dir().ok_or_else(|| "could not determine user directory".to_string())?;
    let roplat_root = target_root.join("roplat");
    let target = roplat_root.join("assets");
    let manifest_path = roplat_root.join(".assets_manifest");

    fs::create_dir_all(&target)
        .map_err(|e| format!("failed to create {}: {e}", target.display()))?;
    fs::create_dir_all(&roplat_root)
        .map_err(|e| format!("failed to create {}: {e}", roplat_root.display()))?;

    // 获取本次操作的源文件清单
    let source_files = list_all_files_with_relatives(&source)?;
    let source_fingerprint = compute_files_fingerprint(&source_files)?;

    // 读取上次的清单并删除本包的旧文件
    let old_manifest = fs::read_to_string(&manifest_path).ok();
    if let Some(manifest_content) = old_manifest.as_ref() {
        remove_previous_package_files(&target, manifest_content)?;
    }

    let needs_sync = old_manifest.as_deref() != Some(source_fingerprint.as_str());

    if needs_sync {
        println!("cargo:warning=Syncing assets to {:?}", target);
        copy_files_to_target(&source, &target, &source_files)?;
        fs::write(&manifest_path, &source_fingerprint)
            .map_err(|e| format!("failed to write manifest: {e}"))?;
    } else {
        println!("cargo:warning=Assets unchanged, skip sync");
    }

    println!("cargo:rustc-env=ROPLAT_ASSETS_DIR={}", target.display());
    println!("cargo:warning=Assets location: {}", target.display());

    Ok(())
}

fn list_all_files_with_relatives(source: &Path) -> Result<Vec<String>, String> {
    let files = list_files_recursive(source)?;
    files
        .into_iter()
        .map(|f| {
            f.strip_prefix(source)
                .ok()
                .and_then(|p| p.to_str())
                .map(|s| s.replace('\\', "/"))
                .ok_or_else(|| format!("failed to compute relative path for {}", f.display()))
        })
        .collect()
}

fn compute_files_fingerprint(files: &[String]) -> Result<String, String> {
    let mut hash: u64 = 0xcbf29ce484222325;

    fn fnv1a_update(mut state: u64, bytes: &[u8]) -> u64 {
        for b in bytes {
            state ^= *b as u64;
            state = state.wrapping_mul(0x100000001b3);
        }
        state
    }

    for rel in files {
        hash = fnv1a_update(hash, rel.as_bytes());
        hash = fnv1a_update(hash, b"\n");
    }

    Ok(format!("{:016x}", hash))
}

fn remove_previous_package_files(_target: &Path, _manifest_content: &str) -> Result<(), String> {
    // 简化版：这里不做增量删除，让所有文件共存
    // 如果需要更精细的管理，可以在 manifest 中记录文件列表
    Ok(())
}

fn copy_files_to_target(source: &Path, target: &Path, rel_files: &[String]) -> Result<(), String> {
    for rel in rel_files {
        let src_file = source.join(rel);
        let dest_path = target.join(rel);
        fs::create_dir_all(dest_path.parent().unwrap_or(Path::new(".")))
            .map_err(|e| format!("failed to create parent dirs: {e}"))?;
        fs::copy(&src_file, &dest_path).map_err(|e| {
            format!(
                "failed to copy {} to {}: {e}",
                src_file.display(),
                dest_path.display()
            )
        })?;
    }
    Ok(())
}

fn roplat_data_target_dir() -> Option<PathBuf> {
    #[cfg(target_os = "windows")]
    {
        env::var_os("LOCALAPPDATA")
            .map(PathBuf::from)
            .or_else(|| env::var_os("USERPROFILE").map(PathBuf::from))
    }

    #[cfg(target_os = "macos")]
    {
        env::var_os("HOME")
            .map(PathBuf::from)
            .map(|home| home.join("Library").join("Application Support"))
    }

    #[cfg(all(not(target_os = "windows"), not(target_os = "macos")))]
    {
        env::var_os("HOME")
            .map(PathBuf::from)
            .map(|home| home.join(".local").join("share"))
    }
}

fn emit_rerun_if_changed(source: &Path) {
    if !source.exists() {
        return;
    }

    println!("cargo:rerun-if-changed={}", source.display());
    if let Ok(files) = list_files_recursive(source) {
        for file in files {
            println!("cargo:rerun-if-changed={}", file.display());
        }
    }
}

fn list_files_recursive(source: &Path) -> Result<Vec<PathBuf>, String> {
    let mut files = Vec::new();
    collect_files(source, &mut files)?;
    files.sort();
    Ok(files)
}

fn collect_files(dir: &Path, files: &mut Vec<PathBuf>) -> Result<(), String> {
    let mut entries = Vec::new();
    for entry in fs::read_dir(dir).map_err(|e| format!("failed to read {}: {e}", dir.display()))? {
        let entry = entry.map_err(|e| format!("failed to iterate directory: {e}"))?;
        entries.push(entry.path());
    }
    entries.sort();

    for path in entries {
        if path.is_dir() {
            collect_files(&path, files)?;
        } else if path.is_file() {
            files.push(path);
        }
    }

    Ok(())
}
