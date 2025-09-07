fn main() {
    #[cfg(feature = "to_cxx")]
    {
        cxx_build::bridge("src/ffi/to_cxx.rs")
            .std("c++14")
            .compile("franka_rust_cxx");

        println!("cargo:rerun-if-changed=src/ffi/to_cxx.rs");
    }
}
