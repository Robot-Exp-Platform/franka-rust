fn main() {
    // The build writes nothing outside its output directory: model assets are
    // installed at run time into roplat_world::assets_dir().
    println!("cargo:rerun-if-changed=build.rs");
    #[cfg(feature = "to_cxx")]
    {
        cxx_build::bridge("src/ffi/to_cxx.rs")
            .std("c++14")
            .compile("franka_rust_cxx");

        println!("cargo:rerun-if-changed=src/ffi/to_cxx.rs");
    }
}
